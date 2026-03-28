// 地上局(PC側) Config.h
#pragma once
#include <Arduino.h>

// ============================================================
//  § 通信用の構造体 (機体側と完全に一致させること！)
// ============================================================
// 構造体のパディング/アライメントによるマイコン間のクラッシュ・通信不良を防ぐため、
// 4バイト(int32_t)とfloat配列で構成します。(合計 36 bytes)
struct __attribute__((__packed__)) PlaneData {
    int32_t packet_type; // 0: 姿勢データ, 1: ゲインデータ
    float data[6];      // 最大32バイト (姿勢は7個、ゲインは8個使用)
};

struct __attribute__((__packed__)) GroundData {
    float p_adj, i_adj, d_adj; 
    float roll, pitch, yaw;    
    uint8_t reset_cmd;         
    uint8_t param_sel;         
    // 0=なし 1=RollRate 2=PitchRate 3=YawRate 4=RollAngle 5=PitchAngle
    // 10 = テレメトリ一時停止 ＆ 現在のゲイン送信要求
    // 11 = テレメトリ再開
    
    void print() const {
        Serial.println("=== Ground Data ===");
        Serial.printf("PID Adjust: P=%.4f, I=%.4f, D=%.4f  sel=%d\n", p_adj, i_adj, d_adj, param_sel);
        Serial.printf("Attitude  : Roll=%.1f, Pitch=%.1f, Yaw=%.1f\n", roll, pitch, yaw);
    }
};
constexpr int GROUND_DATA_NUM = 6;

// ============================================================
//  § グローバル変数とタイミング制御
// ============================================================
extern unsigned long dt;
extern int counter;

constexpr float FREQUENCY = 1000.0f;  // 制御周期(Hz)
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