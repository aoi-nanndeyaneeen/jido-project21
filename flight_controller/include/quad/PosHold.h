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

    // アーム/モード切替でクリアする (drone_s5 の resetControllers から)
    void reset() {
        _vx_ctl = _vy_ctl = 0.0f;
        _vx_tar = _vy_tar = 0.0f;
        _pos_n  = _pos_e  = 0.0f;
        _hold_n = _hold_e = 0.0f;
        _lean_roll = _lean_pitch = 0.0f;
        _holding   = false;
        _bad_count = 0;
        _vx_pid.reset();
        _vy_pid.reset();
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
        if (_bad_count > (FLOW_LOOP_HZ / 2)) {          // 0.5 秒
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

        // --- スティック → 目標速度 (触っている間は速度指令、離すと位置ホールド) ---
        const bool stick_active = (fabsf(sx) > FLOW_STICK_DEAD) ||
                                  (fabsf(sy) > FLOW_STICK_DEAD);
        if (stick_active) {
            // スティックは機体座標のまま (パイロットの前後左右 = 機首基準)
            _vx_tar = FLOW_STICK_SIGN_X * sx * FLOW_STICK_VEL;
            _vy_tar = FLOW_STICK_SIGN_Y * sy * FLOW_STICK_VEL;
            _hold_n = _pos_n;          // 保持基準を今の地面位置へ張り付け
            _hold_e = _pos_e;
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
            // 位置ループは地面固定フレームで解き、出力を機体座標へ戻す
            const float vn_tar = constrain(_pos_kp * (_hold_n - _pos_n),
                                           -FLOW_POS_VEL_LIM, FLOW_POS_VEL_LIM);
            const float ve_tar = constrain(_pos_kp * (_hold_e - _pos_e),
                                           -FLOW_POS_VEL_LIM, FLOW_POS_VEL_LIM);
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
    }

    Pid   _vx_pid, _vy_pid;      // 速度ループ (機体座標 x=前 / y=右)
    float _pos_kp     = FLOW_POS_KP;

    float _vx_ctl = 0.0f, _vy_ctl = 0.0f;   // LPF 済み実測速度 [m/s] (機体)
    float _vx_tar = 0.0f, _vy_tar = 0.0f;   // 速度ループ目標 [m/s] (機体)
    float _pos_n  = 0.0f, _pos_e  = 0.0f;   // 積分位置 [m] (地面固定)
    float _hold_n = 0.0f, _hold_e = 0.0f;   // 保持基準 [m] (地面固定)
    float _lean_roll = 0.0f, _lean_pitch = 0.0f;   // 目標リーン角 [deg]
    bool  _holding   = false;
    int   _bad_count = 0;
};

} // namespace Quad
