// 地上局(PC側) Config.h  ※ 機体側と構造体を完全一致させること
#pragma once
#include <Arduino.h>

// ============================================================
//  § 通信用の構造体
// ============================================================
// IM920SL実効ペイロード上限: 64 - 4(モジュール内部) = 60 bytes
// PlaneData: 4 + 8*4 = 36 bytes  ← OK
// GroundData: 6*4 + 2 + 4*4 = 42 bytes ← OK
struct __attribute__((__packed__)) PlaneData {
    float ax, ay, az;       // 加速度 [g]
    float roll, pitch, yaw; // 姿勢角 [deg]
    float altitude;         // 高度 [m]
    // 合計 28 bytes
};

struct __attribute__((__packed__)) GroundData {
    float p_adj, i_adj, d_adj; // PIDゲイン絶対値
    float roll, pitch, yaw;    // BANK_ANGLE / TURN_MS 調整用
    uint8_t reset_cmd;         // 1: リセット
    uint8_t param_sel;         // 0=なし 1=RollRate 2=PitchRate 3=YawRate 4=RollAngle 5=PitchAngle
    // 地上局(position_estimator)のオートパイロットが計算したRC相当コマンド。
    // SW_AUTOスイッチONかつ受信が新鮮なときだけ MODE_AUTONOMOUS で使用される。
    float ap_roll, ap_pitch, ap_yaw; // -1.0〜1.0 (スティック相当)
    float ap_throttle;                // 0.0〜1.0 ※現状は未使用(drone.cpp参照)
    // 合計 42 bytes (IM920SL実効ペイロード上限60byte以内)
    // ★機体側 flight_controller/include/Config.h と完全一致させること

    void print() const {
        Serial.println("=== Ground Data ===");
        Serial.printf("PID: P=%.4f I=%.4f D=%.4f  sel=%d\n", p_adj, i_adj, d_adj, param_sel);
        Serial.printf("Att: Roll=%.1f Pitch=%.1f Yaw=%.1f\n", roll, pitch, yaw);
        Serial.printf("AP : Roll=%.3f Pitch=%.3f Yaw=%.3f Thr=%.3f\n",
                      ap_roll, ap_pitch, ap_yaw, ap_throttle);
    }
};
constexpr int GROUND_DATA_NUM = 6;

extern unsigned long dt;
extern int counter;

constexpr float FREQUENCY = 1000.0f;
constexpr unsigned long PERIOD = 1 * 1e6f / FREQUENCY;

inline bool frec() {
    static u_int32_t t_prev = micros();
    u_int32_t t_now = micros();
    dt = t_now - t_prev;
    if (dt < PERIOD) return false;
    t_prev = t_now;
    if (counter == 1000) counter = 1; else counter++;
    return true;
}