// ============================================================
//  CircleTrack.h  -  機体単独の周回: 円軌道の目標点を時間で進める
// ============================================================
//  2026-09-17: SW_HOVER を cen(POSHOLD) → up(GUIDED) に上げた瞬間、
//  「今いる点を円の左端 (右旋回なら) とし、機首方向を接線にして半径 R を 1 周」
//  を地上局なしで飛ぶために作った。位置も方位も機体のセンサ (フロー積分 +
//  ジャイロ積分) だけで決まる。
//
//  旧 Maneuver.h (前進速度一定 + ヨーレート一定) との違い:
//    Maneuver は「速度ループに前進速度を与えるだけ」で位置のフィードバックが
//    無く、風や速度の立ち上がりで円がずれても戻す手段が無かった。ここでは
//    **円周上の目標点** を作り、PosHold が目標点との誤差まで消しにいく
//    (位置 P + 接線速度 FF + 向心加速度 FF)。ヨーも目標点の接線方向へ寄せる。
//
//  幾何 (地面固定フレーム N/E。ψ は機首方位、右回り +):
//      d  = +1 右旋回 / -1 左旋回
//      中心      C  = P0 + R·d·(-sinψ0, cosψ0)         機首の右 (d=+1) に R
//      方位角    β  = 中心から見た目標点の向き (N から右回り)
//      目標点    P  = C + R·(cosβ, sinβ)
//      開始      β0 = ψ0 - d·90°                         ← P(β0) = P0
//      接線速度  V  = v·(-sinβ, cosβ)·d
//      向心加速度 A = -(v²/R)·(cosβ, sinβ)
//      目標機首  ψ  = β + d·90°
//
//  進め方: 弧長の進行 s [rad] を 0 → 2π×laps。速さは加速度上限つきで立ち上げ、
//  終点 (= 開始点) の手前で同じ加速度で減速して止まる。止まった点で PosHold
//  に引き継ぐので、終わった後に行き過ぎない。
//
//  機体が遅れた場合: 目標点が機体の実際の進行 (中心から見た方位角) より
//  CIRCLE_LEAD_MAX_DEG 以上先へ行かないように止める。目標だけ先に 1 周して
//  「機体は半周なのに終わった」を防ぐ。
//
//  完了: 目標が終点に着き、かつ **機体の実測進行** が 360°×laps - CIRCLE_CLOSE_TOL_DEG
//  に届いたとき。laps 周は途中で止まらずに続けて回る (上昇旋回)。届かなくても想定時間 × CIRCLE_TIME_CAP で打ち切る (timedOut)。
//
//  ★ このクラスは制御を持たない。出力 (目標点 / FF / ヨーレート) を PosHold と
//    HeadingHold に渡すのは Guided.h と drone_s5.cpp。
// ============================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include "quad/QuadConfig.h"

namespace Quad {

class CircleTrack {
public:
    // 開始。今の推定位置 (N,E) [m] と機首方位 [deg] を円の起点にする。
    void begin(float pos_n, float pos_e, float yaw_deg,
               float radius_m, float speed_mps, float accel_mps2, int dir, int laps = 1) {
        _laps = (laps < 1) ? 1 : laps;
        _R   = fmaxf(radius_m, 0.2f);
        _V   = fmaxf(speed_mps, 0.05f);
        _A   = fmaxf(accel_mps2, 0.02f);
        _d   = (dir < 0) ? -1.0f : 1.0f;

        const float psi = yaw_deg * DEG_TO_RAD;
        _c_n = pos_n + _R * _d * (-sinf(psi));
        _c_e = pos_e + _R * _d * ( cosf(psi));
        _yaw0_deg = yaw_deg;
        _beta0    = psi - _d * HALF_PI;

        _s = 0.0f;  _v = 0.0f;
        _s_meas = 0.0f;
        _beta_meas_prev = _beta0;
        _elapsed_s = 0.0f;
        _t_expect_s = TWO_PI * _laps * _R / _V + _V / _A;
        _active = true;
        _done = false;
        _timed_out = false;
        computeRef();
        _rad_meas = _R;
    }

    // GUIDED_HZ で毎回。pos は PosHold の推定位置 (地面固定)。
    void update(float dt_s, float pos_n, float pos_e) {
        if (!_active || dt_s <= 0.0f) return;
        _elapsed_s += dt_s;

        // --- 機体の実測進行 (中心から見た方位角の変化を巻き戻して積む) ---
        const float rn = pos_n - _c_n, re = pos_e - _c_e;
        _rad_meas = sqrtf(rn * rn + re * re);
        // 中心付近では方位角が暴れるので、半径の 1/3 より内側にいる間は積まない
        if (_rad_meas > _R * 0.33f) {
            const float b = atan2f(re, rn);
            float db = b - _beta_meas_prev;
            while (db >  PI) db -= TWO_PI;
            while (db < -PI) db += TWO_PI;
            _s_meas += _d * db;
            _beta_meas_prev = b;
        }

        // --- 目標の進行: 加速度上限 + 終点での減速 ---
        const float s_end   = TWO_PI * _laps;
        const float remain  = fmaxf((s_end - _s) * _R, 0.0f);          // 残り弧長 [m]
        float v_new = fminf(_V, _v + _A * dt_s);
        v_new = fminf(v_new, sqrtf(2.0f * _A * remain));
        float s_new = fminf(_s + v_new / _R * dt_s, s_end);

        // --- 機体が遅れていたら目標を先へ行かせない (後退はさせない) ---
        const float lead_max = CIRCLE_LEAD_MAX_DEG * DEG_TO_RAD;
        s_new = fminf(s_new, fmaxf(_s, _s_meas + lead_max));

        // FF に使う速さは「目標が実際に進んだ速さ」(止めた回は 0 に近づく)
        _v = (s_new - _s) * _R / dt_s;
        _s = s_new;
        computeRef();

        // --- 完了 ---
        const bool ref_end = (_s >= s_end - 1e-4f);
        const bool closed  = _s_meas >= s_end - CIRCLE_CLOSE_TOL_DEG * DEG_TO_RAD;
        if (ref_end && closed) {
            finish(false);
        } else if (_elapsed_s > _t_expect_s * CIRCLE_TIME_CAP) {
            finish(true);
        }
    }

    // ヨーレート指令 [deg/s] (右 +)。接線方向の目標機首へ P + 角速度 FF。
    float yawRateCmd(float yaw_est_deg) const {
        if (!_active) return 0.0f;
        const float ff  = _d * (_v / _R) * RAD_TO_DEG;
        const float err = constrain(yawRefDeg() - yaw_est_deg,
                                    -CIRCLE_YAW_ERR_LIM_DEG, CIRCLE_YAW_ERR_LIM_DEG);
        return constrain(ff + CIRCLE_YAW_KP * err,
                         -MANEUVER_MAX_YAW_RATE_DPS, MANEUVER_MAX_YAW_RATE_DPS);
    }

    void abort() { _active = false; _v = 0.0f; }

    bool  active()   const { return _active; }
    bool  done()     const { return _done; }
    bool  timedOut() const { return _timed_out; }

    // ---- 出力 (地面固定フレーム N/E) ----
    float refN()  const { return _ref_n; }
    float refE()  const { return _ref_e; }
    float velN()  const { return _vel_n; }
    float velE()  const { return _vel_e; }
    float accN()  const { return _acc_n; }
    float accE()  const { return _acc_e; }
    // 目標機首 [deg]。HeadingHold::est() と同じく巻き戻さない連続値。
    float yawRefDeg() const { return _yaw0_deg + _d * _s * RAD_TO_DEG; }

    // ---- 表示 / テレメトリ ----
    float progressRefDeg()  const { return _s * RAD_TO_DEG; }
    float progressMeasDeg() const { return _s_meas * RAD_TO_DEG; }
    float speed()      const { return _v; }
    float radius()     const { return _R; }
    int   laps()       const { return _laps; }
    // 機体が実際に回った周回数 (0.0〜laps)。上昇旋回の高度目標に使う
    float lapsMeas()   const { return constrain(_s_meas / TWO_PI, 0.0f, (float)_laps); }
    float radiusMeas() const { return _rad_meas; }   // 中心から機体までの距離 [m]
    float centerN()    const { return _c_n; }
    float centerE()    const { return _c_e; }
    float elapsedS()   const { return _elapsed_s; }
    float expectS()    const { return _t_expect_s; }

private:
    void computeRef() {
        const float beta = _beta0 + _d * _s;
        const float cb = cosf(beta), sb = sinf(beta);
        _ref_n = _c_n + _R * cb;
        _ref_e = _c_e + _R * sb;
        _vel_n = _d * _v * (-sb);
        _vel_e = _d * _v * ( cb);
        const float ac = _v * _v / _R;
        _acc_n = -ac * cb;
        _acc_e = -ac * sb;
    }
    void finish(bool timed_out) {
        _active = false;
        _done = true;
        _timed_out = timed_out;
        _v = 0.0f;
        computeRef();
    }

    float _R = 1.0f, _V = 0.3f, _A = 0.2f, _d = 1.0f;
    int   _laps = 1;
    float _c_n = 0.0f, _c_e = 0.0f;
    float _yaw0_deg = 0.0f, _beta0 = 0.0f;
    float _s = 0.0f, _v = 0.0f;             // 目標の進行 [rad] と速さ [m/s]
    float _s_meas = 0.0f;                   // 機体の実測進行 [rad]
    float _beta_meas_prev = 0.0f;
    float _rad_meas = 0.0f;
    float _elapsed_s = 0.0f, _t_expect_s = 0.0f;
    float _ref_n = 0.0f, _ref_e = 0.0f;
    float _vel_n = 0.0f, _vel_e = 0.0f;
    float _acc_n = 0.0f, _acc_e = 0.0f;
    bool  _active = false, _done = false, _timed_out = false;
};

} // namespace Quad
