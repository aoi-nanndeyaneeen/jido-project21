//設定、共通データ
#pragma once
#include <Arduino.h>

// ==== 通信用の構造体 ====
struct __attribute__((__packed__)) PlaneData {
    float ax, ay, az;
    float gx, gy, gz;
    float altitude; 

    void print() const {
        Serial.printf("Accel: [%.2f, %.2f, %.2f] g\n", ax, ay, az);
        Serial.printf("Gyro : [%.2f, %.2f, %.2f] deg/s\n", gx, gy, gz);
        Serial.printf("Alt  : %.2f m\n", altitude);
    }
};

struct __attribute__((__packed__)) GroundData {
    float p_adj, i_adj, d_adj;
    float roll, pitch, yaw;
    uint8_t reset_cmd; // 1: Reset trigger
    
    void print() const {
        Serial.println("=== Ground Data ===");
        Serial.printf("PID Adjust: P=%.3f, I=%.3f, D=%.3f\n", p_adj, i_adj, d_adj);
        Serial.printf("Attitude  : Roll=%.1f, Pitch=%.1f, Yaw=%.1f\n", roll, pitch, yaw);
    }
};
constexpr int GROUND_DATA_NUM = 6;
// ==== グローバル変数の宣言 (実体はmain.cppに置く) ====
extern unsigned long dt;
extern int counter;

// ==== スイッチやチャンネルの定義 ====
enum Sw {
    up,
    cen,
    down
};

enum Ch {
  ROLL,PITCH,THR,YAW,Aux1,THR_CUT,Aux2,Aux3,Aux4,Aux5
};

// ==== タイミング制御 ====
constexpr float FREQUENCY = 1000.0f;  //(Hz)とっても大事！！！制御周期！！！
constexpr unsigned long PERIOD = 1*1e6f/FREQUENCY;//microに合わせた一周期当たりの時間
// ★ここに inline を追加するだけ！
inline bool frec() {
    static u_int32_t t_prev = micros();
    u_int32_t t_now = micros();
    dt = t_now - t_prev;
    if (dt < PERIOD) return false;

    t_prev = t_now;
    if (counter == 1000) counter = 1; else counter++;
    //if (dt > PERIOD*1.01) Serial.println(dt);

    return true;
}