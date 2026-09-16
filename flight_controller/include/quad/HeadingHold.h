// ============================================================
//  HeadingHold.h  -  ヨー: ジャイロ積分の相対方位と、そのホールド
// ============================================================
//  drone_s5.cpp § 7-2 にあった g_yaw_est / g_yaw_hold / g_yaw_holding /
//  g_yaw_hold_kp を 1 クラスにまとめたもの。
//
//    ・スティックを触っている間 → 素直にレート指令
//    ・中立に戻した瞬間          → そのときの方位を目標として保持
//    ・定型機動中                → Maneuver の一定レートをそのまま流し、
//                                  目標方位は現在値に追従させておく
//
//  レートPID の I 項は「回転速度を 0 にする」までしか保証しない。突風で
//  30deg 振られたら、その 30deg は戻ってこない。振られた分まで戻すために
//  相対方位の外側ループを足している。
//
//  ★ 積分に使うのは「制御に使っているジャイロ値」。レートループと位相が揃う。
//  ★ 絶対方位は持たない (6 軸 IMU)。アーム時を 0 とした相対値。地上局が
//    カメラで絶対ヨーを測ったときだけ rebase() で真値へ寄せる (目標 hold は
//    触らない。戻す動きはヘディングホールド P が YAW_HOLD_RATE_LIM で
//    クランプしながらやる。無線遅延を姿勢ループに持ち込まないため)。
//
//  ゲインは S5Gains.h の Gain::YAW_HOLD_*。
// ============================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include "quad/S5Gains.h"

namespace Quad {

class HeadingHold {
public:
    // アーム/モード切替で「今の向き」を基準に取り直す。相対値なので 0 でよい。
    void reset() {
        _est = _hold = 0.0f;
        _holding = false;
    }

    // 毎メインループ (1000Hz)。制御に使うジャイロ値で相対方位を積分する。
    void integrate(float yaw_rate_dps, float dt_s) { _est += yaw_rate_dps * dt_s; }

    // カメラ絶対ヨーで推定値だけを再基準する (Guided が新しいパケットの回に呼ぶ)。
    void rebase(float abs_deg) { _est = abs_deg; }

    // ------------------------------------------------------------
    //  目標ヨーレート [deg/s] を返す。
    //    stick         : ラダー (符号適用済み, -1..+1)
    //    can_hold      : 保持してよいか (= I 項が有効な高スロットル。地上では false)
    //    maneuver_rate : 定型機動中はその一定レート。それ以外は NAN
    // ------------------------------------------------------------
    float update(float stick, bool can_hold, float maneuver_rate_dps, float max_rate_dps) {
        if (!isnan(maneuver_rate_dps)) {
            // 定型機動中: 一定レート指令。終わった瞬間に「そのままの機首」で
            // ホールドへ引き継げるよう、目標は現在値に追従させる。
            _hold    = _est;
            _holding = false;
            return maneuver_rate_dps;
        }
        const bool stick_active = fabsf(stick) > Gain::YAW_STICK_DEAD;
        if (stick_active || !can_hold) {
            // 操作中、または低スロットル (地上)。目標を現在値に追従させておく
            // ことで、スティックを離した瞬間から「今の向き」の保持が始まる。
            _hold    = _est;
            _holding = false;
            return stick * max_rate_dps;
        }
        // 中立: 保持方位との差を消しにいく
        const float err = constrain(_hold - _est, -Gain::YAW_HOLD_ERR_LIM, +Gain::YAW_HOLD_ERR_LIM);
        _holding = true;
        return constrain(_kp * err, -Gain::YAW_HOLD_RATE_LIM, +Gain::YAW_HOLD_RATE_LIM);
    }

    float est()     const { return _est; }      // 相対方位 [deg] (アーム時 0)
    float hold()    const { return _hold; }     // 保持したい方位 [deg]
    float error()   const { return _hold - _est; }
    bool  holding() const { return _holding; }
    float kp()      const { return _kp; }
    void  setKp(float kp) { _kp = kp; }

private:
    float _est  = 0.0f;
    float _hold = 0.0f;
    bool  _holding = false;
    float _kp = Gain::YAW_HOLD_KP;   // シリアル 'p' メニューから変えられる
};

} // namespace Quad
