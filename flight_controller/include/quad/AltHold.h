// ============================================================
//  AltHold.h  -  測距センサによる高度ホールド
// ============================================================
//  Stage 5 の s5c (高度ホールド) の本体。drone_s5.cpp に散らばっていた
//  7 個のグローバル (g_alt_active / g_alt_hold_m / g_alt_thr_base /
//  g_alt_vz_tar / g_alt_thr_corr / g_alt_thr_out / alt_rate_pid) と
//  状態遷移をまとめたもの。PosHold.h / OpticalFlow.h と同じ
//  「1クラス1責務」の形にそろえてある。
//
//  カスケード:
//    高度誤差[m] ──[ALT_POS_KP]──► 目標上昇速度[m/s] ──[RATE PID]──►
//      スロットル補正[割合] → ALT_HOVER_THR + 補正 → ミキサーへ
//
//  ★ 基準スロットルは Q::ALT_HOVER_THR だけ。プロポのスロットル位置は
//    「ALT_ENABLE_THR を超えているか」の enable ゲートにしか使わない。
//    (旧実装は未設定時に engage 時のスティック値を掴んでいたため、
//     スティックが 100% だと base=1.00 になって飽和していた)
//
//  ★ 状態を enum 1 個で持つ。旧実装の lost_range_only は「want が false に
//    なった理由」を見ずに POSHOLD+armed だけで成立していたため、スロットルを
//    絞ってもホバースロットルを出し続け、モーターが止まらなかった。
//    ここでは各条件を順に判定し、どの理由で外れたかを state() で公開する。
//
//  設定は QuadConfig.h の § 7-3 (ALT_*)。
// ============================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include "quad/QuadConfig.h"
#include "quad/QuadPID.h"

namespace Quad {

//  高度ホールドの状態。表示 / ログ / PosHold のゲートに使う。
enum class AltState : uint8_t {
    Off        = 0,  // シリアルのトグルで無効にされている
    Standby    = 1,  // 未アーム / POSHOLD でない / スロットルを絞っている
    NoHoverThr = 2,  // ALT_HOVER_THR が未設定 (0) なので engage しない
    NoRange    = 3,  // 測距が無い / まだ一度も掴めていない
    Holding    = 4,  // 通常動作。スロットルを握っている
    RangeLost  = 5,  // engage 後に測距「だけ」を失い、base で保持している
};

class AltitudeHold {
public:
    // 起動時に1回。
    void begin() {
        _rate_pid.set_gains(ALT_RATE_KP, ALT_RATE_KI, ALT_RATE_KD);
        _rate_pid.set_d_alpha(ALT_RATE_D_ALPHA);
        _rate_pid.set_i_limit(ALT_RATE_I_LIMIT);
        _pos_kp = ALT_POS_KP;
        reset(0.0f);
    }

    // アーム / モード切替でクリアする。
    void reset(float range_h_m) {
        _state    = AltState::Standby;
        _hold_m   = (range_h_m > 0.0f) ? range_h_m : FLOW_ASSUMED_HEIGHT_M;
        _engage_h = 0.0f;
        _vz_tar   = 0.0f;
        _thr_corr = 0.0f;
        _thr_out  = 0.0f;
        _airborne = false;
        _thr_applied    = 0.0f;
        _dt_since_fresh = 0.0f;
        _rate_pid.reset();
    }

    // ------------------------------------------------------------
    //  update()  — RANGE_LOOP_HZ で毎回呼ぶ (fresh でなくても呼ぶこと)
    //    dt_s        : 前回 update() からの経過 [s]
    //    enabled     : シリアルのトグル
    //    armed       : アーム済みか
    //    poshold     : いま POSHOLD モードか
    //    range_valid : 測距が信用できるか
    //    fresh       : このループで「新しい測距サンプル」が入ったか
    //    h_m         : 鉛直対地高度 [m]
    //    climb_mps   : 上昇速度 [m/s] (上 +)
    //    thr_stick   : プロポのスロットル [0..1] (enable ゲート専用)
    //    thr_applied : 前回この update() 以降にミキサーが実際に使った
    //                  スロットルの平均 (MixInfo::thr_used の平均)。
    //                  負を渡すと「情報なし」= 従来動作。
    //
    //  ★ fresh でない回は PID を進めない。測距 (ソナー ~20Hz) より速い
    //    レートで呼ぶと、同じ climb 値に対して D項が 0 → スパイクを繰り返す。
    //    状態遷移だけは毎回評価する (スロットルを絞ったら即座に手放すため)。
    //
    //  ★ 2026-09-07: dt_s は「この呼び出しの周期」であって「前回 fresh から
    //    の経過」ではない。旧コードは fresh の回に dt_s をそのまま PID へ
    //    渡していたが、呼び出しは RANGE_LOOP_HZ (20ms)、実際の測距は 65ms
    //    間隔だったため、D項が 3.05倍過大 / I項が 0.33倍過小になっていた
    //    (log_037 で実測: fresh 116点の平均間隔 65.0ms)。
    //    ここで fresh 間の経過を自前で積算し、それを PID の dt にする。
    // ------------------------------------------------------------
    void update(float dt_s, bool enabled, bool armed, bool poshold,
                bool range_valid, bool fresh,
                float h_m, float climb_mps, float thr_stick,
                float thr_applied = -1.0f) {

        // fresh 間の経過時間。release() されるまで積み続ける。
        if (dt_s > 0.0f) _dt_since_fresh += dt_s;

        // --- 1) engage を外す条件を「理由つき」で順に見る -------------
        //   release() = スロットルをプロポの値に戻す。
        //   RangeLost だけは例外で、base を保持したまま生き延びる。
        if (!enabled)                    { release(AltState::Off,        thr_stick); return; }
        if (!armed || !poshold)          { release(AltState::Standby,    thr_stick); return; }
        if (thr_stick <= ALT_ENABLE_THR) { release(AltState::Standby,    thr_stick); return; }
        if (ALT_HOVER_THR <= 0.01f)      { release(AltState::NoHoverThr, thr_stick); return; }

        if (!range_valid) {
            // ★ engage 済みなら「測距だけ」を失った状態。プロペラ後流で
            //   ソナーが一時的に飛ぶことがあるので、直前の基準スロットルで
            //   保持して落下を防ぐ。まだ engage していないなら普通に手放す。
            if (isEngaged()) {
                _state    = AltState::RangeLost;
                _vz_tar   = 0.0f;
                _thr_corr = 0.0f;
                _thr_out  = ALT_HOVER_THR;
                _dt_since_fresh = 0.0f;
                _rate_pid.reset();
                return;
            }
            release(AltState::NoRange, thr_stick);
            return;
        }

        // --- 2) inactive → active の初期化 ---------------------------
        if (!isEngaged()) {
            _hold_m   = (ALT_TARGET_M > 0.0f) ? ALT_TARGET_M : h_m;
            _engage_h = h_m;              // 離陸判定の地面基準
            _dt_since_fresh = 0.0f;       // engage 前の積算は捨てる
            // engage した時点で既に十分高ければ、地上ではなく空中で
            // 引き継いだとみなす (ANGLE ホバリングからの移行)。
            _airborne = (h_m > ALT_AIRBORNE_GROUND_MAX_M);
            _rate_pid.reset();
        }
        _state = AltState::Holding;

        // --- 3) 離陸検知 (一度立ったら release まで下ろさない) --------
        //   PosHold はこれを見て、地上にいる間の積分ワインドアップを避ける。
        if (!_airborne && h_m > _engage_h + ALT_AIRBORNE_RISE_M) _airborne = true;

        // --- 4) 目標上昇速度 -----------------------------------------
        if (ALT_STICK_VZ_ENABLE) {
            const float dev = thr_stick - ALT_HOVER_THR;
            if (fabsf(dev) > ALT_STICK_DEAD) {
                _vz_tar = dev * ALT_STICK_VZ;
                _hold_m = h_m;                  // 目標高度を今へ張り付け
            } else {
                _vz_tar = posLoop(h_m);
            }
        } else {
            _vz_tar = posLoop(h_m);             // 完全自動: 常に位置ループ
        }

        // --- 5) 内側: 上昇速度PID → スロットル補正 --------------------
        //   ★ 新しい測距が来た回だけ進める。
        //   ★ アンチワインドアップ: 補正が権限 (ALT_THR_AUTH) に張り付いて
        //     いる間は、同じ向きの積分を止める。地上で目標高度に届かない
        //     まま I項が溜まり、離陸した瞬間に飛び出すのを防ぐ。
        //  ★ 2026-09-07: 飽和判定に「ミキサーがスロットルを奪ったか」を足した。
        //    ALT_THR_AUTH に張り付いていたのは log_037 で全体の 7% だけで、
        //    実際に効いていた飽和はこちら (63%) だった。ミキサーが姿勢優先で
        //    スロットルを押し上げている間は、こちらがいくら絞っても機体は
        //    降りない = 下向きの積分は windup にしかならない。
        if (fresh && _dt_since_fresh > 0.0f) {
            const float dt_fresh = _dt_since_fresh;
            _dt_since_fresh = 0.0f;

            //  自分の権限 (±ALT_THR_AUTH) での飽和
            bool sat_hi = (_thr_corr >=  ALT_THR_AUTH - 1e-4f);
            bool sat_lo = (_thr_corr <= -ALT_THR_AUTH + 1e-4f);

            //  ミキサー側の飽和。押し上げられた = これ以上下げられない (sat_lo)。
            //  押し下げられた = これ以上上げられない (sat_hi)。
            if (thr_applied >= 0.0f) {
                _thr_applied = thr_applied;
                const float steal = thr_applied - _thr_out;
                if (steal >  ALT_MIX_STEAL_DEAD) sat_lo = true;
                if (steal < -ALT_MIX_STEAL_DEAD) sat_hi = true;
            }

            const float err = _vz_tar - climb_mps;
            const bool  integrate = !((sat_hi && err > 0.0f) || (sat_lo && err < 0.0f));

            _thr_corr = constrain(_rate_pid.update(_vz_tar, climb_mps, dt_fresh, integrate),
                                  -ALT_THR_AUTH, ALT_THR_AUTH);
        }

        // --- 6) 出力 --------------------------------------------------
        //  ★ 2026-09-06: log_034 の転倒原因。下限を 0.05 のままにしていたら
        //    ALT_THR_AUTH を広げた (0.55) ことで alt_thr が 0.052 まで落ちた。
        //    これは THR_MIN_MIX(0.05)+THR_RAMP_RANGE(0.05)=0.10 より下 =
        //    Mixer::mix() の姿勢補正権限ランプ (auth=(thr-THR_MIN_MIX)/
        //    THR_RAMP_RANGE) にかかり、auth=0.04 まで絞られた。つまり
        //    「素早く降ろす」つもりの出力が「空中で姿勢補正をほぼ全部捨てる」
        //    になってしまい、ATTITUDE_PRIORITY=true でも救えず、778ms
        //    (t=0.84〜1.62s) 無補正のまま角速度がジャイロのフルスケール
        //    (±250dps) に張り付くまで回転して裏返った。
        //    高度ループの下限を「地上判定(THR_MIN_MIX)+ランプ幅」より
        //    確実に上、余裕を持って ALT_MIN_THR_OUT に固定する。ALT_THR_AUTH
        //    をどれだけ広げても、この下限より下の出力は二度と出ない
        //    (= 姿勢補正権限を絶対に失わない)。0.60 ホバーからでも 0.15 まで
        //    絞れれば十分な降下力がある。
        _thr_out = constrain(ALT_HOVER_THR + _thr_corr, ALT_MIN_THR_OUT, 1.0f);
    }

    // --- 出力 -----------------------------------------------------
    //  active() が true の間だけ、ミキサーへ throttle() を渡すこと。
    bool  active()   const { return isEngaged(); }
    float throttle() const { return _thr_out; }

    //  離陸したか。PosHold の active ゲートに使う (地上での積分を止める)。
    bool  airborne() const { return _airborne; }

    // --- 表示 / ログ用 --------------------------------------------
    AltState state() const { return _state; }
    float holdM()    const { return _hold_m; }
    float vzTar()    const { return _vz_tar; }
    float thrBase()  const { return isEngaged() ? ALT_HOVER_THR : 0.0f; }
    float thrCorr()  const { return _thr_corr; }
    float thrOut()   const { return _thr_out; }
    float engageH()  const { return _engage_h; }

    //  ミキサーが実際に使ったスロットル。thrOut() との差が
    //  「姿勢優先に奪われた量」= 高度が言うことを聞かない量。
    float thrApplied() const { return _thr_applied; }
    float thrSteal()   const { return _thr_applied - _thr_out; }

    const char* stateName() const {
        switch (_state) {
            case AltState::Off:        return "OFF(手動)";
            case AltState::Standby:    return "standby";
            case AltState::NoHoverThr: return "NO_HOVER_THR";
            case AltState::NoRange:    return "NO_RANGE";
            case AltState::Holding:    return "ACTIVE";
            case AltState::RangeLost:  return "RANGE_LOST(base保持)";
        }
        return "?";
    }

    // --- ゲイン調整 (シリアルメニューから) ------------------------
    Pid&  ratePid()          { return _rate_pid; }
    float posKp() const      { return _pos_kp; }
    void  setPosKp(float kp) { _pos_kp = kp; }

private:
    // engage 中 = スロットルを握っている状態
    bool isEngaged() const {
        return _state == AltState::Holding || _state == AltState::RangeLost;
    }

    float posLoop(float h_m) const {
        return constrain(_pos_kp * (_hold_m - h_m), -ALT_POS_VZ_LIM, ALT_POS_VZ_LIM);
    }

    // スロットルを手放す。ミキサーはプロポのスロットルをそのまま使う。
    void release(AltState why, float thr_stick) {
        _state    = why;
        _vz_tar   = 0.0f;
        _thr_corr = 0.0f;
        _thr_out  = thr_stick;
        _airborne = false;
        _dt_since_fresh = 0.0f;
        _rate_pid.reset();
    }

    Pid      _rate_pid;
    float    _pos_kp   = ALT_POS_KP;
    AltState _state    = AltState::Standby;
    float    _hold_m   = 0.0f;
    float    _engage_h = 0.0f;
    float    _vz_tar   = 0.0f;
    float    _thr_corr = 0.0f;
    float    _thr_out  = 0.0f;
    bool     _airborne = false;
    float    _thr_applied    = 0.0f;  // ミキサーが実際に使ったスロットル (ログ/診断)
    float    _dt_since_fresh = 0.0f;  // 前回 fresh からの経過 [s] (PID の真の dt)
};

} // namespace Quad
