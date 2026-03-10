//設定
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
    
    void print() const {
        Serial.println("=== Ground Data ===");
        Serial.printf("PID Adjust: P=%.3f, I=%.3f, D=%.3f\n", p_adj, i_adj, d_adj);
        Serial.printf("Attitude  : Roll=%.1f, Pitch=%.1f, Yaw=%.1f\n", roll, pitch, yaw);
    }
};
//ゲインの構造体
struct PID_Gains {
    float kp_rate, ki_rate, kd_rate;
    float kp_angle, ki_angle, kd_angle;
    float sensitivity;
    float rate_d_alpha=0.7, rate_i_limit=200.0f;
    float angle_d_alpha=0.7, angle_i_limit=200.0f;
};

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
inline unsigned long dt;
inline int counter;

bool frec() {
    static u_int32_t t_prev = micros();
    u_int32_t t_now = micros();
    dt = t_now - t_prev;
    if (dt < PERIOD) return false;

    t_prev = t_now;
    if (counter == 1000) counter = 1; else counter++;
    if (dt > PERIOD*1.01) Serial.println(dt);

    return true;
}

//初期値

    // ---- PIDゲイン設定 ----
    // setgains(kp_rate, ki_rate, kd_rate, kp_angle, ki_angle, kd_angle, sensitivity)
    //
    // [重要] 全て0からスタートし、必ず地上で手持ちしながら少しずつ上げること
    //  手順:
    //  1. kp_angleを上げて機体が目標角に向かうか確認
    //  2. kp_rateを上げてサーボの追従速度を上げる
    //  3. kd_rateを足して振動を抑える
    //  4. ki_rateは最後に少しだけ足す

PID_Gains   ROLL_gain  = {1.2f, 0.5f, 0.01f, 4.5f, 0.0f, 0.0f, 1.0f, 0.7f, 200.0f, 0.7f, 200.0f},
            PITCH_gain = {1.2f, 0.5f, 0.01f, 4.5f, 0.0f, 0.0f, 1.0f},//デフォルト値は書かなくてもいいよ
            YAW_gain   = {1.2f, 0.5f, 0.01f, 4.5f, 0.0f, 0.0f, 1.0f};
            //kp_rate, ki_rate, kd_rate, kp_angle, ki_angle, kd_angle, sensitivity, rate_d_alpha, rate_i_limit, angle_d_alpha, angle_i_limit

namespace Config {

    namespace serial{
        inline HardwareSerial* const im920 = &Serial1; // IM920SL用シリアルポート
        inline HardwareSerial* const sbus  = &Serial2; // S.BUS用シリアルポート
    }

    namespace wire {
        inline TwoWire *const mpu = &Wire;   // MPU用I2Cポート
        inline TwoWire *const bmp = &Wire1;  // 気圧センサ用I2Cポート
    }
}