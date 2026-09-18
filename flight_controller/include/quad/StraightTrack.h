// ============================================================
//  StraightTrack.h  -  機体単独の直進: 直線上の目標点を時間で進める
// ============================================================
//  2026-09-18: 本番の着陸点ボーナスは「カメラの外で離陸 → 22m まっすぐ進んで
//  幅 6m (±3m) のフィールドに入る」ことが条件。機体のセンサ (フロー積分 +
//  ジャイロ積分) だけでどれだけ真っ直ぐ行けるかを測るために作った。
//
//  幾何 (地面固定フレーム N/E。PosHold と同じ。ψ は HeadingHold::est() の系):
//      方向      u  = (cosψ, sinψ)
//      目標点    P  = P0 + s·u           s: 目標の進行 [m] (0 → L)
//      速度 FF   V  = v·u
//      実測      along = (pos - P0)·u    cross = (pos - P0)×u (右 +)
//
//  進め方は CircleTrack と同じ: 加速度上限で立ち上げ、終点手前で同じ加速度で
//  減速して止まる。機体が遅れたら目標を STRAIGHT_LEAD_MAX_M より先へ行かせない。
//  横ずれ (cross) は PosHold の軌道追従 (CIRCLE_POS_KP) がフロー座標で消しにいく。
//
//  ★ ここで 0 にできるのは「フローが見ている横ずれ」だけ。次の 3 つはログの cross
//    に出ないまま実際の軌跡を曲げる (22m 先の横ずれ = 22m × tan(角度))。
//      1. 置いた向きの誤差 (人)                  → 配置試験で測る
//      2. ジャイロのバイアス (方位推定が回る)    → drift 試験 (analyze_straight.py drift)
//      3. フローセンサの取り付けヨー角           → 往復飛行で機体固定の偏りとして出る
//    実際の着地点を巻尺で測って比べるしかない。手順は scripts/analyze_straight.py 冒頭。
// ============================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include "quad/QuadConfig.h"

namespace Quad {

class StraightTrack {
public:
    // 開始。起点 (N,E) [m]、進む方位 ψ [deg] (HeadingHold::est() の系)、距離 [m]。
    void begin(float pos_n, float pos_e, float heading_deg,
               float dist_m, float speed_mps, float accel_mps2) {
        _p0_n = pos_n;
        _p0_e = pos_e;
        _psi_deg = heading_deg;
        const float psi = heading_deg * DEG_TO_RAD;
        _u_n = cosf(psi);
        _u_e = sinf(psi);
        _dist = fmaxf(dist_m, 0.1f);
        _vmax = fmaxf(speed_mps, 0.05f);
        _amax = fmaxf(accel_mps2, 0.02f);
        _s = 0.0f;  _v = 0.0f;  _a = 0.0f;
        _along = 0.0f;  _cross = 0.0f;  _cross_max = 0.0f;
        _elapsed_s = 0.0f;
        _t_expect_s = _dist / _vmax + _vmax / _amax;
        _active = true;
        _done = false;
        _timed_out = false;
        computeRef();
    }

    // GUIDED_HZ で毎回。pos は PosHold の推定位置 (地面固定)。
    void update(float dt_s, float pos_n, float pos_e) {
        if (!_active || dt_s <= 0.0f) return;
        _elapsed_s += dt_s;

        const float dn = pos_n - _p0_n, de = pos_e - _p0_e;
        _along = dn * _u_n + de * _u_e;
        _cross = -dn * _u_e + de * _u_n;          // 進行方向の右が +
        if (fabsf(_cross) > fabsf(_cross_max)) _cross_max = _cross;

        // --- 目標の進行: 加速度上限 + 終点での減速 ---
        const float remain = fmaxf(_dist - _s, 0.0f);
        float v_new = fminf(_vmax, _v + _amax * dt_s);
        v_new = fminf(v_new, sqrtf(2.0f * _amax * remain));
        float s_new = fminf(_s + v_new * dt_s, _dist);

        // --- 機体が遅れていたら目標を先へ行かせない (後退はさせない) ---
        s_new = fminf(s_new, fmaxf(_s, _along + STRAIGHT_LEAD_MAX_M));

        const float v_act = (s_new - _s) / dt_s;
        _a = constrain((v_act - _v) / dt_s, -_amax, _amax);
        _v = v_act;
        _s = s_new;
        computeRef();

        const bool ref_end = (_s >= _dist - 1e-3f);
        if (ref_end && _along >= _dist - STRAIGHT_END_TOL_M) {
            finish(false);
        } else if (_elapsed_s > _t_expect_s * STRAIGHT_TIME_CAP) {
            finish(true);
        }
    }

    // ヨーレート指令 [deg/s] (右 +)。進む方位へ P で寄せるだけ (FF は無い)。
    float yawRateCmd(float yaw_est_deg) const {
        if (!_active) return 0.0f;
        const float err = constrain(_psi_deg - yaw_est_deg,
                                    -CIRCLE_YAW_ERR_LIM_DEG, CIRCLE_YAW_ERR_LIM_DEG);
        return constrain(CIRCLE_YAW_KP * err, -STRAIGHT_YAW_RATE_LIM_DPS, STRAIGHT_YAW_RATE_LIM_DPS);
    }

    void abort() { _active = false; _v = 0.0f; _a = 0.0f; }

    bool  active()   const { return _active; }
    bool  done()     const { return _done; }
    bool  timedOut() const { return _timed_out; }

    // ---- 出力 (地面固定フレーム N/E) ----
    float refN() const { return _ref_n; }
    float refE() const { return _ref_e; }
    float velN() const { return _v * _u_n; }
    float velE() const { return _v * _u_e; }
    float accN() const { return _a * _u_n; }
    float accE() const { return _a * _u_e; }
    float headingDeg() const { return _psi_deg; }

    // ---- 表示 / テレメトリ ----
    float distance()  const { return _dist; }
    float progressRef() const { return _s; }
    float along()     const { return _along; }      // 実測の進んだ距離 [m] (フロー)
    float cross()     const { return _cross; }      // 実測の横ずれ [m] (フロー, 右 +)
    float crossMax()  const { return _cross_max; }  // |横ずれ| 最大 (符号つき)
    float speed()     const { return _v; }
    float elapsedS()  const { return _elapsed_s; }
    float expectS()   const { return _t_expect_s; }

private:
    void computeRef() {
        _ref_n = _p0_n + _s * _u_n;
        _ref_e = _p0_e + _s * _u_e;
    }
    void finish(bool timed_out) {
        _active = false;
        _done = true;
        _timed_out = timed_out;
        _v = 0.0f;
        _a = 0.0f;
    }

    float _p0_n = 0.0f, _p0_e = 0.0f;
    float _psi_deg = 0.0f, _u_n = 1.0f, _u_e = 0.0f;
    float _dist = 1.0f, _vmax = 0.3f, _amax = 0.2f;
    float _s = 0.0f, _v = 0.0f, _a = 0.0f;
    float _along = 0.0f, _cross = 0.0f, _cross_max = 0.0f;
    float _ref_n = 0.0f, _ref_e = 0.0f;
    float _elapsed_s = 0.0f, _t_expect_s = 0.0f;
    bool  _active = false, _done = false, _timed_out = false;
};

} // namespace Quad
