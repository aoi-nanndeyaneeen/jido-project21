// ============================================================
//  QuadPID.h  -  dt を引数で受け取る PID
// ============================================================
//  既存の Control.h の PID は、呼ばれる周期に関係なく常に
//  Config::Timing::Main_dt (= 1ms) を dt として使っていました。
//  そのため 200Hz に間引いて呼んでいる角度ループの I項・D項が
//  実際の 1/5 の dt で計算されていました。
//
//  ここでは dt を呼び出し側から渡します。レートループと角度ループで
//  同じクラスを使い回しても、それぞれ正しい dt になります。
//
//  ※ Control.h には手を付けていないので、他の .cpp は今まで通り動きます。
// ============================================================
#pragma once
#include <Arduino.h>

namespace Quad {

class Pid {
private:
    float _kp = 0.0f, _ki = 0.0f, _kd = 0.0f;
    float _i_term   = 0.0f;   // 積分項 (ゲインを掛けた後の値で持つ)
    float _prev_meas = 0.0f;  // 前回の測定値
    float _d_lpf    = 0.0f;   // ローパス後の微分値
    float _d_alpha  = 0.7f;   // 0 = フィルタなし, 1 に近いほど強く効く
    float _i_limit  = 0.3f;   // 積分項の上限 (出力と同じ単位)
    bool  _first    = true;   // 初回は微分をスキップする

public:
    Pid() = default;
    Pid(float kp, float ki, float kd, float d_alpha = 0.7f, float i_limit = 0.3f)
        : _kp(kp), _ki(ki), _kd(kd), _d_alpha(d_alpha), _i_limit(i_limit) {}

    void set_gains(float kp, float ki, float kd) { _kp = kp; _ki = ki; _kd = kd; }
    void set_d_alpha(float a)   { _d_alpha = a; }
    void set_i_limit(float lim) { _i_limit = lim; }

    float kp() const { return _kp; }
    float ki() const { return _ki; }
    float kd() const { return _kd; }
    float i_term() const { return _i_term; }

    void reset() {
        _i_term    = 0.0f;
        _prev_meas = 0.0f;
        _d_lpf     = 0.0f;
        _first     = true;
    }

    // ------------------------------------------------------------
    //  update()
    //    target      : 目標値
    //    measurement : 測定値
    //    dt_s        : 前回呼び出しからの経過時間 [秒]
    //    integrate   : false の間は積分を止める (アンチワインドアップ)
    //
    //  微分は「誤差の微分」ではなく「測定値の微分」を使います。
    //  こうするとスティックを急に動かしたときに D項が跳ねません
    //  (derivative kick の回避)。旧 PID は誤差の微分だったので、
    //  スティック操作のたびに D項がスパイクしていました。
    // ------------------------------------------------------------
    float update(float target, float measurement, float dt_s, bool integrate = true) {
        if (dt_s <= 0.0f) return 0.0f;

        const float error = target - measurement;

        // --- P項 ---
        const float p = _kp * error;

        // --- I項 ---
        if (integrate) {
            _i_term += _ki * error * dt_s;
            _i_term = constrain(_i_term, -_i_limit, _i_limit);
        }

        // --- D項 (測定値の微分。符号を反転して使う) ---
        float d = 0.0f;
        if (!_first) {
            const float d_raw = -(measurement - _prev_meas) / dt_s;
            _d_lpf = _d_alpha * _d_lpf + (1.0f - _d_alpha) * d_raw;
            d = _kd * _d_lpf;
        }
        _prev_meas = measurement;
        _first = false;

        return p + _i_term + d;
    }
};

// ------------------------------------------------------------
//  1軸ぶんの状態をまとめて持つ
// ------------------------------------------------------------
struct Axis {
    Pid   rate;      // 内側ループ: 角速度 [deg/s] → トルク指令
    Pid   angle;     // 外側ループ: 角度   [deg]   → 角速度目標 [deg/s]

    float stick     = 0.0f;  // スティック入力 [-1, 1]
    float ang_meas  = 0.0f;  // 実測角度   [deg]
    float rate_meas = 0.0f;  // 実測角速度 [deg/s]
    float ang_tar   = 0.0f;  // 目標角度   [deg]
    float rate_tar  = 0.0f;  // 目標角速度 [deg/s]
    float cmd       = 0.0f;  // ミキサーへ渡すトルク指令 [-1, 1]

    void reset() {
        rate.reset();
        angle.reset();
        ang_tar  = 0.0f;
        rate_tar = 0.0f;
        cmd      = 0.0f;
    }
};

} // namespace Quad
