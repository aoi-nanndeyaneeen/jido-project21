// ============================================================
//  BodyDv.h  -  ヨー推定用の機体Δv (重力除去・FRD前+右+) を積算する
// ============================================================
//  position_estimator/YAW_HANDOFF.md の「機体側の融合」に必要な入力を
//  作る。カメラは「フィールド座標系でのΔv」を、機体は「機体座標系での
//  Δv」を持っていて、その方位差がヨーそのもの (S5Cmd.h 冒頭参照)。
//
//  quad::Attitude::acc_x/y/z (FRD, [g]) は重力を含んだ比力 (静止・水平で
//  acc_z ≈ -1g)。AltEstimator::predict() が高度方向で行っている重力除去
//  と全く同じ回転行列の係数を、前・右方向にも使う:
//      重力の機体座標成分 [g] = (-sin p, cos p sin r, cos p cos r)
//                              = (前, 右, 下)
//  なので
//      真の水平加速度 = 測定値 - (-重力成分) = 測定値 + 重力の機体座標成分
//  逆ではない (符号を間違えると位置ループと同じく正帰還になる)。
//
//  ★ 積分するのは「直前に drain() を呼んでからの経過時間」ぶん。
//    S5::TelemetryTx::tick() の送信間隔がモードで変わるため、固定窓にすると
//    実際の積分区間とズレる (S5Telem.h の DvFrame コメント参照)。
// ============================================================
#pragma once
#include <Arduino.h>
#include "quad/BodyFrame.h"

namespace Quad {

class BodyDvAccumulator {
public:
    // 毎制御ループ (実測 dt) 呼ぶ。att は BodyFrame.h の姿勢 (傾きの sin/cos 計算済み)。
    void update(float dt_s, const Attitude& att) {
        if (dt_s <= 0.0f || dt_s > 0.1f) return;   // 異常な dt は捨てる (AltEstimator と同じ)

        // 重力の機体座標成分 [g] (前, 右)。AltEstimator::predict() の
        // f_down 式 (-sin(p)*ax + cos(p)sin(r)*ay + cos(p)cos(r)*az) と
        // 同じ回転行列から出てくる、下方向以外の2成分。
        const float g_front_g = -att.sp;
        const float g_right_g =  att.cp * att.sr;

        const float true_ax_g = att.acc_x + g_front_g;
        const float true_ay_g = att.acc_y + g_right_g;

        _dvx += true_ax_g * GRAVITY_MPS2 * dt_s;
        _dvy += true_ay_g * GRAVITY_MPS2 * dt_s;
    }

    // 送信直前に呼ぶ。積んだΔvを取り出してゼロへ戻す。
    void drain(float& dvx, float& dvy) {
        dvx = _dvx; dvy = _dvy;
        _dvx = _dvy = 0.0f;
    }

    void reset() { _dvx = _dvy = 0.0f; }

private:
    static constexpr float GRAVITY_MPS2 = 9.80665f;
    float _dvx = 0.0f, _dvy = 0.0f;
};

} // namespace Quad
