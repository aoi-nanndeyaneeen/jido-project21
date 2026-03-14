//設定
#pragma once
#include <Arduino.h>
#include <Wire.h>

// ==== 通信用の構造体 ====
struct __attribute__((__packed__)) PlaneData {
    float ax, ay, az;
    float gx, gy, gz;
    float altitude; 

    void update(float ax_in, float ay_in, float az_in, float gx_in, float gy_in, float gz_in, float alt_in) {
        ax = ax_in; ay = ay_in; az = az_in;
        gx = gx_in; gy = gy_in; gz = gz_in;
        altitude = alt_in;
    }   
    
    void print() const {
        Serial.printf("Accel: [%.2f, %.2f, %.2f] g\n", ax, ay, az);
        Serial.printf("Gyro : [%.2f, %.2f, %.2f] deg/s\n", gx, gy, gz);
        Serial.printf("Alt  : %.2f m\n", altitude);
    }
};

struct __attribute__((__packed__)) GroundData {
    float p_adj, i_adj, d_adj;
    float roll, pitch, yaw;
    
    void update(float p, float i, float d, float r, float pt, float y) {
        p_adj = p; i_adj = i; d_adj = d;
        roll = r; pitch = pt; yaw = y;
    }

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
  ROLL,PITCH,THR,YAW,Aux1,Aux2,THR_CUT,Aux3,Aux4,Aux5
};

enum FlightMode : uint8_t {
    MODE_MANUAL     = 0,  // プロポ直接操作
    MODE_LEVEL_TURN = 1,  // 自動水平旋回
    MODE_FIGURE_8   = 2,  // 8の字飛行
    MODE_SEMI_MANUAL= 4,
};


// ============================================================
//  自律飛行パラメータ
// ============================================================
inline float         BANK_ANGLE = 40.0f;    // バンク角 [deg]
inline unsigned long TURN_MS    = 4000UL;   // 8の字の片道時間 [ms]



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

    namespace Timing {
        // ==== タイミング制御 ====
        constexpr int MAIN_Hz = 1000;  //(Hz)とっても大事！！！制御周期！！！
        constexpr int DEBUG_Hz = 10;   //(Hz)デバッグ周期
        constexpr unsigned long MAIN_PERIOD = 1000000UL / MAIN_Hz; // 周期 [us]
        inline unsigned long Main_dt = 1/MAIN_Hz; // センサ更新周期 (us)

        template <int Hz>
        bool freq(unsigned long &dt) {
            static uint32_t t_prev = 0;
            constexpr uint32_t period = 1000000UL / Hz;
            
            uint32_t t_now = micros();
            if (t_now - t_prev < period) return false;

            t_prev = t_now;
            dt = t_now - t_prev;
            return true;
        }

        template <int Hz>
        bool freq() {
            static uint32_t t_prev = 0;
            constexpr uint32_t period = 1000000UL / Hz;
            
            uint32_t t_now = micros();
            if (t_now - t_prev < period) return false;

            t_prev = t_now;
            return true;
        }
    }
}