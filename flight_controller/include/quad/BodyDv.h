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
//    S5Tel::tick() の送信間隔がモードで変わるため、固定窓にすると
//    実際の積分区間とズレる (S5Telem.h の DvFrame コメント参照)。
// ============================================================
#pragma once
#include <Arduino.h>

namespace Quad {

class BodyDvAccumulator {
public:
    // 毎制御ループ (1000Hz) 呼ぶ。roll_deg/pitch_deg は機体姿勢、
    // ax/ay/az は g_att.acc_x/y/z (FRD, [g])。
    void update(float dt_s, float roll_deg, float pitch_deg,
                float ax, float ay, float az) {
        if (dt_s <= 0.0f || dt_s > 0.1f) return;   // 異常な dt は捨てる (AltEstimator と同じ)

        const float r = roll_deg  * DEG2RAD;
        const float p = pitch_deg * DEG2RAD;
        const float sr = sinf(r);
        const float sp = sinf(p), cp = cosf(p);

        // 重力の機体座標成分 [g] (前, 右)。AltEstimator::predict() の
        // f_down 式 (-sin(p)*ax + cos(p)sin(r)*ay + cos(p)cos(r)*az) と
        // 同じ回転行列から出てくる、下方向以外の2成分。
        const float g_front_g = -sp;
        const float g_right_g =  cp * sr;

        const float true_ax_g = ax + g_front_g;
        const float true_ay_g = ay + g_right_g;

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
    static constexpr float DEG2RAD = 3.14159265358979323846f / 180.0f;
    float _dvx = 0.0f, _dvy = 0.0f;
};

} // namespace Quad
