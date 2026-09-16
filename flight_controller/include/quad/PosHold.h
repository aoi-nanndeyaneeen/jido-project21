// ============================================================
//  PosHold.h  -  オプティカルフローによる水平 速度/位置ホールド
// ============================================================
//  Stage 5 の s5b (速度・位置ホールド) と s5d (地面固定フレーム化) の本体。
//  drone_s5.cpp に散らばっていた 10 個以上のグローバルとロジックを、
//  OpticalFlow.h / Rangefinder.h と同じ「1クラス1責務」の形にまとめたもの。
//
//  カスケード:
//    位置誤差[m] ──[FLOW_POS_KP]──► 目標速度[m/s] ──[VEL PID]──► 目標リーン角[deg]
//                                                             └─► 角度ループへ
//
//  ★ s5d: 位置積分を「地面固定フレーム (N/E)」で行う。
//     s5b までは機体座標のまま積分していたので、機体がヨーすると保持基準が
//     一緒に回ってしまい、位置が流れていた。ここでヘディング(g_yaw_est)で
//     回してから積分し、位置ループの出力を機体座標へ戻して速度PIDに渡す。
//       body(x=前, y=右) → earth(n, e):  n =  x cosψ - y sinψ
//                                        e =  x sinψ + y cosψ
//       earth → body:                    x =  n cosψ + e sinψ
//                                        y = -n sinψ + e cosψ
//     ψ は「アーム時を0とした相対方位」でよい (絶対方位は不要)。
//
//  ★ スティックは機体座標のまま扱う (パイロットの前後左右 = 機首基準)。
//     触っている間は速度指令、離すとその瞬間の地面位置を保持する。
//
//  設定は QuadConfig.h の § 7-2 (FLOW_VEL_* / FLOW_POS_* / FLOW_STICK_* …)。
//
//  ※ このヘッダを include するのは drone_s5.cpp だけ。他機体に影響しません。
// ============================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include "quad/QuadConfig.h"
#include "quad/QuadPID.h"

namespace Quad {

class PositionHold {
public:
    PositionHold() = default;

    // 起動時に1回。速度PIDのゲインとフィルタを config から入れる。
    void begin() {
        setVelGains(FLOW_VEL_KP, FLOW_VEL_KI, FLOW_VEL_KD);
        for (Pid* p : { &_vx_pid, &_vy_pid }) {
            p->set_d_alpha(FLOW_VEL_D_ALPHA);
            p->set_i_limit(FLOW_VEL_I_LIMIT);
        }
        _pos_kp = FLOW_POS_KP;
        reset();
    }

    // ------------------------------------------------------------
    //  外部からの目標速度指令 (地上局ガイド飛行)
    // ------------------------------------------------------------
    //  ★ 地上局は「スティック」ではなく「機体座標の目標速度 [m/s]」を送る。
    //    スティック経路 (sx * FLOW_STICK_VEL) を通すと、地上局が意図した
    //    速度が FLOW_STICK_VEL のチューニングに巻き込まれて変わってしまう。
    //    m/s をそのまま渡せる口を分けておく。
    //
    //  ★ 指令が不感帯以下 (= 止まれ) のときは、スティックを離したのと
    //    まったく同じ「その場の地面位置を保持」に落ちる。地上局からの
    //    指令が途切れたら 0 を入れるだけでホールドになるのはこの性質。
    //
    //    vx : 前 + [m/s]   vy : 右 + [m/s]
    void setVelCommand(float vx, float vy) {
        _ext_vx = vx;
        _ext_vy = vy;
        _ext_on = true;
    }
    void clearVelCommand() {
        _ext_on = false;
        _ext_vx = _ext_vy = 0.0f;
    }
    bool velCommanded() const { return _ext_on; }

    // ------------------------------------------------------------
    //  地上局からの絶対位置補正 (地上局ガイド飛行、数秒に1回)
    // ------------------------------------------------------------
    //  ★ _hold_n/_hold_e には触らない。_pos_n/_pos_e だけを書き換えて、
    //    次の update() で (hold - pos) の見かけの誤差を作らせる。
    //    その誤差への追従速度は update() 側で FLOW_POS_VEL_LIM に
    //    クランプされるので、ここでどれだけ飛んだ値が来ても暴れない。
    //
    //  ★ GUIDED 巡航中 (stick_active) は毎ループ hold_n=pos_n に
    //    張り付け直されるため、この補正は事実上無効になる。
    //    効くのは静止保持中 (スティック/地上局速度指令が不感帯以下) だけ。
    //    これは意図した挙動: 巡航中に位置を書き換えても行き先が
    //    変わるわけではなく、無駄にレイテンシを持ち込むだけだから。
    void correctPosition(float n_m, float e_m) {
        _pos_n = n_m;
        _pos_e = e_m;
    }

    // ------------------------------------------------------------
    //  フレームの原点合わせ (地上局から1回。フェンスの基準)
    // ------------------------------------------------------------
    //  correctPosition() と違い _hold_n/_hold_e も同じ量だけ動かす。
    //  「今いる場所の座標名を付け替える」だけで、hold-pos の誤差は
    //  変わらないので機体は動かない。これ以降 _pos_n/_pos_e は地上局が
    //  決めた座標系 (mission.py ならフィールド座標) になり、FENCE_* の
    //  矩形がその座標系で効き始める。
    //  ★ reset()/reset_outputs() で無効に戻る (離陸前・モード切替)。
    //    地上局は telemetry の F_FRAME_OK を見て、落ちていたら送り直す。
    void shiftFrame(float n_m, float e_m) {
        const float dn = n_m - _pos_n;
        const float de = e_m - _pos_e;
        _pos_n = n_m;       _pos_e = e_m;
        _hold_n += dn;      _hold_e += de;
        _frame_ok = true;
    }
    bool frameOk()   const { return _frame_ok; }
    bool fenceOn()   const { return FENCE_ENABLE && _frame_ok; }
    bool fencePush() const { return _fence_push; }   // いま境界で押し返している

    // アーム/モード切替でクリアする (drone_s5 の resetControllers から)
    void reset() {
        _vx_ctl = _vy_ctl = 0.0f;
        _vx_tar = _vy_tar = 0.0f;
        _pos_n  = _pos_e  = 0.0f;
        _hold_n = _hold_e = 0.0f;
        _lean_roll = _lean_pitch = 0.0f;
        _holding   = false;
        _bad_count = 0;
        _frame_ok  = false;
        _fence_push = false;
        _vx_pid.reset();
        _vy_pid.reset();
        clearVelCommand();
    }

    // ------------------------------------------------------------
    //  update()  — FLOW_LOOP_HZ で呼ぶ
    //    dt_s     : 前回からの経過 [s]
    //    vx, vy   : フローの対地速度 [m/s] (機体座標: 前 +, 右 +)
    //    yaw_deg  : ヘディング [deg] (アーム時を0とした相対方位でよい)
    //    sx, sy   : スティック (符号適用済み, -1..+1)。sx=前後, sy=左右
    //    active   : ホールドを効かせるか (アーム && POSHOLD && スロットル十分)
    //
    //  active でない間は出力を 0 に固定し、積分・PID もクリアし続ける。
    //  こうしておくと、次に active になった瞬間が常にゼロから始まる。
    // ------------------------------------------------------------
    void update(float dt_s, float vx, float vy, float yaw_deg,
                float sx, float sy, bool active) {
        if (dt_s <= 0.0f) return;

        // --- 実測速度: 異常値クランプ + 制御用 LPF (active でなくても回す) ---
        const float vx_raw = constrain(vx, -FLOW_VEL_SANE, FLOW_VEL_SANE);
        const float vy_raw = constrain(vy, -FLOW_VEL_SANE, FLOW_VEL_SANE);
        _vx_ctl += FLOW_VEL_MEAS_ALPHA * (vx_raw - _vx_ctl);
        _vy_ctl += FLOW_VEL_MEAS_ALPHA * (vy_raw - _vy_ctl);

        if (!active) { reset_outputs(); return; }

        // --- 失探検出: |速度| が上限に張り付き続けたら水平指令に固める ---
        if (fabsf(vx) >= FLOW_VEL_SANE || fabsf(vy) >= FLOW_VEL_SANE) _bad_count++;
        else                                                          _bad_count = 0;
        if (_bad_count > (FLOW_CTRL_HZ / 2)) {          // 0.5 秒 (update は FLOW_CTRL_HZ で呼ばれる)
            _lean_roll = _lean_pitch = 0.0f;
            _vx_pid.reset();
            _vy_pid.reset();
            return;
        }

        // --- s5d: 機体座標の速度を地面固定フレームへ回してから積分 ---
        const float psi = yaw_deg * DEG2RAD;
        const float c = cosf(psi), s = sinf(psi);
        const float vn = _vx_ctl * c - _vy_ctl * s;
        const float ve = _vx_ctl * s + _vy_ctl * c;
        _pos_n += vn * dt_s;
        _pos_e += ve * dt_s;

        // --- スティック(または地上局指令) → 目標速度 ---
        //   触っている間は速度指令、離す (= 指令 0) と位置ホールド。
        //   ★ 地上局指令が入っているときはスティックより優先する。GUIDED 中は
        //     drone_s5 側がスティックを 0 にして渡すので競合はしないが、
        //     ここでも優先順位を明示しておく。
        bool  stick_active;
        float tar_x, tar_y;
        if (_ext_on) {
            stick_active = (fabsf(_ext_vx) > FLOW_VEL_CMD_DEAD) ||
                           (fabsf(_ext_vy) > FLOW_VEL_CMD_DEAD);
            tar_x = constrain(_ext_vx, -FLOW_STICK_VEL, FLOW_STICK_VEL);
            tar_y = constrain(_ext_vy, -FLOW_STICK_VEL, FLOW_STICK_VEL);
        } else {
            stick_active = (fabsf(sx) > FLOW_STICK_DEAD) ||
                           (fabsf(sy) > FLOW_STICK_DEAD);
            // スティックは機体座標のまま (パイロットの前後左右 = 機首基準)
            tar_x = FLOW_STICK_SIGN_X * sx * FLOW_STICK_VEL;
            tar_y = FLOW_STICK_SIGN_Y * sy * FLOW_STICK_VEL;
        }
        // --- フェンス (地面固定フレームの矩形。shiftFrame() 後だけ有効) ---
        //   境界の外へ向かう速度成分を 0 にし、はみ出た量に比例して内側へ
        //   押し戻す。スティック / 地上局指令 / 位置ループのどれが出した
        //   目標速度でも同じ場所で必ず通るので、手動・GUIDED・自動旋回の
        //   区別なく効く。リンクが切れていても機体単独で効く。
        const bool fence = fenceOn();
        _fence_push = false;

        if (stick_active) {
            if (fence) {
                float vn =  tar_x * c - tar_y * s;
                float ve =  tar_x * s + tar_y * c;
                vn = fenceAxis(vn, _pos_n, FENCE_N_LIM);
                ve = fenceAxis(ve, _pos_e, FENCE_E_LIM);
                tar_x =  vn * c + ve * s;
                tar_y = -vn * s + ve * c;
            }
            _vx_tar = tar_x;
            _vy_tar = tar_y;
            _hold_n = _pos_n;          // 保持基準を今の地面位置へ張り付け
            _hold_e = _pos_e;
            if (fence) clampHoldToFence();
            _holding = false;
        } else {
            // 保持基準を「今の推定位置」へゆっくり緩和する (リーク)。
            //  これが無いと、フローのノイズが積分されて位置推定がズレたとき、
            //  そのズレが消えず、静止していても目標リーン角が0に戻らなくなる。
            if (FLOW_POS_HOLD_TAU_S > 0.0f) {
                const float k = constrain(dt_s / FLOW_POS_HOLD_TAU_S, 0.0f, 1.0f);
                _hold_n += (_pos_n - _hold_n) * k;
                _hold_e += (_pos_e - _hold_e) * k;
            }
            // 保持基準そのものは常にフェンスの内側に置く (外で止まったまま
            // リークで基準が外へ寄っていくのを防ぐ)。
            if (fence) clampHoldToFence();
            // 位置ループは地面固定フレームで解き、出力を機体座標へ戻す
            float vn_tar = constrain(_pos_kp * (_hold_n - _pos_n),
                                     -FLOW_POS_VEL_LIM, FLOW_POS_VEL_LIM);
            float ve_tar = constrain(_pos_kp * (_hold_e - _pos_e),
                                     -FLOW_POS_VEL_LIM, FLOW_POS_VEL_LIM);
            if (fence) {
                vn_tar = fenceAxis(vn_tar, _pos_n, FENCE_N_LIM);
                ve_tar = fenceAxis(ve_tar, _pos_e, FENCE_E_LIM);
            }
            _vx_tar =  vn_tar * c + ve_tar * s;
            _vy_tar = -vn_tar * s + ve_tar * c;
            _holding = true;
        }

        // --- 内側: 速度PID → 目標リーン角 [deg] ---
        const float px = _vx_pid.update(_vx_tar, _vx_ctl, dt_s, true);
        const float py = _vy_pid.update(_vy_tar, _vy_ctl, dt_s, true);
        _lean_pitch = constrain(FLOW_LEAN_SIGN_PITCH * px, -FLOW_MAX_LEAN, FLOW_MAX_LEAN);
        _lean_roll  = constrain(FLOW_LEAN_SIGN_ROLL  * py, -FLOW_MAX_LEAN, FLOW_MAX_LEAN);
    }

    // --- 出力 (角度ループへ渡す目標リーン角 [deg]) ---
    float leanRoll()  const { return _lean_roll;  }
    float leanPitch() const { return _lean_pitch; }

    // --- 表示 / ログ用 ---
    float vxCtl()  const { return _vx_ctl; }   // 制御に使っている速度 [m/s] (機体)
    float vyCtl()  const { return _vy_ctl; }
    float vxTar()  const { return _vx_tar; }   // 速度ループの目標 [m/s] (機体)
    float vyTar()  const { return _vy_tar; }
    float posN()   const { return _pos_n;  }   // 地面固定フレームの推定位置 [m]
    float posE()   const { return _pos_e;  }
    float holdN()  const { return _hold_n; }   // 保持したい地面位置 [m]
    float holdE()  const { return _hold_e; }
    bool  holding() const { return _holding; }
    int   badCount() const { return _bad_count; }

    // --- ゲイン調整 (シリアルメニューから) ---
    void  setVelGains(float kp, float ki, float kd) {
        _vx_pid.set_gains(kp, ki, kd);
        _vy_pid.set_gains(kp, ki, kd);
    }
    void  setPosKp(float kp) { _pos_kp = kp; }
    float posKp() const      { return _pos_kp; }
    // テレメトリで「今どのゲインで飛んでいるか」を送るための読み出し。
    // x/y は setVelGains() で必ず同じ値にしているので x 側だけ見ればよい。
    const Pid& velPid() const { return _vx_pid; }

private:
    static constexpr float DEG2RAD = 0.01745329252f;

    // active を外れている間の出力クリア。積分もゼロに戻す。
    void reset_outputs() {
        _pos_n = _pos_e = 0.0f;
        _hold_n = _hold_e = 0.0f;
        _holding = false;
        _vx_tar = _vy_tar = 0.0f;
        _lean_roll = _lean_pitch = 0.0f;
        _vx_pid.reset();
        _vy_pid.reset();
        _bad_count = 0;
        _frame_ok = false;     // 原点が消えたのでフェンスも無効 (地上局が送り直す)
        _fence_push = false;
    }

    // 1軸ぶんのフェンス。p が ±lim の外なら外向き成分を殺し、はみ出し量に
    // 比例した内向き速度 (FENCE_VEL_MAX でクランプ) を最低限確保する。
    float fenceAxis(float v, float p, float lim) {
        if (p > lim) {
            const float back = -fminf(FENCE_KP * (p - lim), FENCE_VEL_MAX);
            _fence_push = true;
            return fminf(v, back);
        }
        if (p < -lim) {
            const float back = fminf(FENCE_KP * (-lim - p), FENCE_VEL_MAX);
            _fence_push = true;
            return fmaxf(v, back);
        }
        return v;
    }
    void clampHoldToFence() {
        _hold_n = constrain(_hold_n, -FENCE_N_LIM, FENCE_N_LIM);
        _hold_e = constrain(_hold_e, -FENCE_E_LIM, FENCE_E_LIM);
    }

    // 地上局からの目標速度指令 (setVelCommand)。_ext_on=false なら従来動作。
    bool  _ext_on = false;
    float _ext_vx = 0.0f, _ext_vy = 0.0f;

    Pid   _vx_pid, _vy_pid;      // 速度ループ (機体座標 x=前 / y=右)
    float _pos_kp     = FLOW_POS_KP;

    float _vx_ctl = 0.0f, _vy_ctl = 0.0f;   // LPF 済み実測速度 [m/s] (機体)
    float _vx_tar = 0.0f, _vy_tar = 0.0f;   // 速度ループ目標 [m/s] (機体)
    float _pos_n  = 0.0f, _pos_e  = 0.0f;   // 積分位置 [m] (地面固定)
    float _hold_n = 0.0f, _hold_e = 0.0f;   // 保持基準 [m] (地面固定)
    float _lean_roll = 0.0f, _lean_pitch = 0.0f;   // 目標リーン角 [deg]
    bool  _holding   = false;
    int   _bad_count = 0;
    bool  _frame_ok   = false;   // shiftFrame() 済み (座標系が地上局のもの)
    bool  _fence_push = false;   // 直近の update() で境界に当たった
};

} // namespace Quad
