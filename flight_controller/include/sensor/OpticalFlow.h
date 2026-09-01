// ============================================================
//  OpticalFlow.h  -  PMW3901 オプティカルフローの機体側ラッパ
// ============================================================
//  Stage 5 (drone_s5.cpp) で使用。
//
//  やっていること:
//    1. PMW3901 から「前回読み出しからの移動カウント(ピクセル)」を取得
//    2. QuadConfig.h の符号 / スワップで機体座標 (FRD: 前=x, 右=y) に合わせる
//    3. de-rotation: 機体の回転が作る「見かけの流れ」をジャイロで差し引く
//       (これを省くと、傾いた瞬間に偽の並進速度が出て発振する)
//    4. 高度を与えて対地速度 [m/s] に換算する
//
//  ★ 高度は外から与える。距離センサが無い間 (s5a / s5b) は
//    QuadConfig.h の FLOW_ASSUMED_HEIGHT_M が初期値として入る。
//    距離センサが載ったら (s5c) 測距値を毎ループ setHeight() するだけ。
//
//  依存: bitcraze/Bitcraze PMW3901  (platformio.ini の lib_deps に既に有り)
//        グローバル SPI              (Teensy 4.0: SCK=13 MOSI=11 MISO=12)
//
//  ※ 既存の他機体コードには影響しません (このヘッダを include するのは
//    drone_s5.cpp だけ)。
// ============================================================
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "Bitcraze_PMW3901.h"
#include "quad/QuadConfig.h"

class OpticalFlow {
public:
    explicit OpticalFlow(uint8_t cs_pin = (uint8_t)Quad::FLOW_CS_PIN)
        : _sensor(cs_pin) {}

    // 戻り値 false = PMW3901 が応答しない (配線 / SPI / 電源を確認)
    bool begin() {
        _ok_init  = _sensor.begin();
        _height_m = Quad::FLOW_ASSUMED_HEIGHT_M;
        return _ok_init;
    }
    bool initialized() const { return _ok_init; }

    // 換算スケールに使う高度 [m] をセットする。
    // 極端な値は無視して直前の値を保持する (測距センサの外れ値対策)。
    void setHeight(float h_m) {
        if (h_m > 0.05f && h_m < 40.0f) _height_m = h_m;
    }
    float height() const { return _height_m; }

    // ------------------------------------------------------------
    //  update()
    //    dt_s                : 前回 update() からの経過 [s]
    //    gyro_roll_rate_dps  : 機体 X軸まわり角速度 [deg/s]  (FRD: 右バンク +)
    //    gyro_pitch_rate_dps : 機体 Y軸まわり角速度 [deg/s]  (FRD: 機首上げ +)
    //
    //  ジャイロは BodyFrame.h の Attitude.roll_rate / pitch_rate をそのまま渡す。
    //  レートループが使っているのと同じ値なので位相が揃う。
    // ------------------------------------------------------------
    void update(float dt_s, float gyro_roll_rate_dps, float gyro_pitch_rate_dps) {
        if (!_ok_init || dt_s <= 0.0f) return;

        int16_t dx = 0, dy = 0;
        _sensor.readMotionCount(&dx, &dy);

        // --- 1) 生カウント → 機体座標 (FRD) ---
        float fx = (float)dx;
        float fy = (float)dy;
        if (Quad::FLOW_SWAP_XY) { const float t = fx; fx = fy; fy = t; }
        fx *= Quad::FLOW_SIGN_X;
        fy *= Quad::FLOW_SIGN_Y;

        raw_x = fx;
        raw_y = fy;

        // --- 2) de-rotation ---
        //  pitch レート → flow_x に乗る / roll レート → flow_y に乗る
        //  見かけ流量 [px] = PX_PER_RAD * 角速度[rad/s] * dt
        float dfx = fx, dfy = fy;
        if (Quad::FLOW_DEROTATE) {
            const float gr = gyro_roll_rate_dps  * DEG2RAD;
            const float gp = gyro_pitch_rate_dps * DEG2RAD;
            dfx += Quad::FLOW_DEROT_SIGN_X * Quad::FLOW_PX_PER_RAD * gp * dt_s;
            dfy += Quad::FLOW_DEROT_SIGN_Y * Quad::FLOW_PX_PER_RAD * gr * dt_s;
        }
        derot_x = dfx;
        derot_y = dfy;

        // --- 3) 対地速度 [m/s] ---
        //  omega_apparent = (flow_px / dt) / PX_PER_RAD     [rad/s]
        //  v              = omega_apparent * height         [m/s]
        const float k = _height_m / (Quad::FLOW_PX_PER_RAD * dt_s);
        vx = dfx * k;
        vy = dfy * k;

        _last_dt = dt_s;
        _fresh   = true;
    }

    // 直近 update() の結果 -----------------------------------------
    float raw_x   = 0.0f, raw_y   = 0.0f;   // 機体座標に直しただけの生カウント [px]
    float derot_x = 0.0f, derot_y = 0.0f;   // de-rotation 後 [px]
    float vx      = 0.0f, vy      = 0.0f;   // 対地速度 [m/s] (前 +, 右 +)

    float lastDt() const { return _last_dt; }
    // 表示 / ログで「今ループで更新されたか」を知りたいとき用
    bool  consumeFresh() { const bool f = _fresh; _fresh = false; return f; }

private:
    static constexpr float DEG2RAD = 0.01745329252f;

    Bitcraze_PMW3901 _sensor;
    bool  _ok_init  = false;
    float _height_m = 1.0f;
    float _last_dt  = 0.0f;
    bool  _fresh    = false;
};
