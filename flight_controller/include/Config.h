// ============================================================
//  Config.h  -  全設定の一元管理
// ============================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>


// ============================================================
//  § 共通型定義  (構造体・列挙型)
// ============================================================

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
    uint8_t reset_cmd; // 1: Reset trigger

    void update(float p, float i, float d, float r, float pt, float y, uint8_t reset = 0) {
        p_adj = p; i_adj = i; d_adj = d;
        roll = r; pitch = pt; yaw = y;
    }

    void print() const {
        Serial.println("=== Ground Data ===");
        Serial.printf("PID Adjust: P=%.3f, I=%.3f, D=%.3f\n", p_adj, i_adj, d_adj);
        Serial.printf("Attitude  : Roll=%.1f, Pitch=%.1f, Yaw=%.1f\n", roll, pitch, yaw);
    }
};

// ゲインの構造体
struct PID_Gains {
    float kp_rate, ki_rate, kd_rate;
    float kp_angle, ki_angle, kd_angle;
    float sensitivity;
    float rate_d_alpha  = 0.7f,  rate_i_limit  = 200.0f;
    float angle_d_alpha = 0.7f,  angle_i_limit = 200.0f;
};

// ==== スイッチやチャンネルの定義 ====
enum Sw { up, cen, down };

enum Ch {
    ROLL, PITCH, THR, YAW, Aux1, Aux2, THR_CUT, Aux3, Aux4, Aux5
};

enum FlightMode : uint8_t {
    MODE_MANUAL      = 0,  // プロポ直接操作
    MODE_LEVEL_TURN  = 1,  // 自動水平旋回
    MODE_FIGURE_8    = 2,  // 8の字飛行
    MODE_SEMI_MANUAL = 4,
};


// ============================================================
//  § 1  PIDゲイン
// ============================================================
// setgains(kp_rate, ki_rate, kd_rate, kp_angle, ki_angle, kd_angle, sensitivity)
//
// [重要] 全て0からスタートし、必ず地上で手持ちしながら少しずつ上げること
//  手順:
//  1. kp_angle を上げて機体が目標角に向かうか確認
//  2. kp_rate  を上げてサーボの追従速度を上げる
//  3. kd_rate  を足して振動を抑える
//  4. ki_rate  は最後に少しだけ足す

//                      kp_rate  ki_rate  kd_rate  kp_angle  ki_angle  kd_angle  sensitivity
PID_Gains ROLL_gain  = { -0.05f,    0.0f,    0.0f,   -1.5f,     0.0f,     0.0f,     0.0f,
                          0.0f, 0.0f,   // rate_d_alpha, rate_i_limit
                          0.0f, 0.0f }; // angle_d_alpha, angle_i_limit
PID_Gains PITCH_gain = { 0.05f,    0.0f,    0.00f,   -1.5f,     0.0f,     0.0f,     0.0f };
PID_Gains YAW_gain   = { 0.0f,    0.0f,    0.00f,   0.0f,     0.0f,     0.0f,     0.0f };


// ============================================================
//  § 2  自律飛行パラメータ
// ============================================================
inline float         BANK_ANGLE = 40.0f;   // バンク角 [deg]
inline unsigned long TURN_MS    = 4000UL;  // 8の字の片道時間 [ms]


// ============================================================
//  § 3–5  ハードウェア設定 (namespace Config)
// ============================================================
namespace Config {

    // ============================================================
    //  § 3  センサ設定
    // ============================================================
    namespace sensor {

        // MPU6050のスケール (±2g, ±250dps)
        constexpr float ACCEL_SCALE = 16384.0f;
        constexpr float GYRO_SCALE  = 131.0f;

        // ---- ソフトウェア・キャリブレーション補正値 ----
        // monitor.py と同じ仕組み:
        //   1. 各軸のバイアスを引く
        //   2. ay は符号を反転する
        //   3. az は 1.0g 分を残すようにオフセット計算する
        // ※ monitor.py の Rキーキャリブレーションで得られた値を入力する
        // ※ デフォルトは0。機体ごとに調整すること
        // ※ ハードウェアオフセット(レジスタ書き込み)は不具合の原因になるため使用しない
        inline float s_ax_bias = 0.0f;
        inline float s_ay_bias = 0.0f;
        inline float s_az_bias = 0.0f;
        inline float s_gx_bias = 0.0f;
        inline float s_gy_bias = 0.0f;
        inline float s_gz_bias = 0.0f;

        constexpr int16_t ACCEL_X_OFFSET = 0;
        constexpr int16_t ACCEL_Y_OFFSET = 0;
        constexpr int16_t ACCEL_Z_OFFSET = 0;
        constexpr int16_t GYRO_X_OFFSET  = 0;
        constexpr int16_t GYRO_Y_OFFSET  = 0;
        constexpr int16_t GYRO_Z_OFFSET  = 0;

        // BMP280 気圧センサー設定 (main_test の実績値に合わせる)
        constexpr float BARO_SEA_LEVEL_HPA = 1013.25f;
        constexpr float BARO_ALPHA         = 0.1f;
        constexpr auto  BMP_STANDBY = Adafruit_BMP280::STANDBY_MS_500;
        constexpr auto  BMP_SAMP_P  = Adafruit_BMP280::SAMPLING_X16;
        constexpr auto  BMP_SAMP_T  = Adafruit_BMP280::SAMPLING_X2;
        constexpr auto  BMP_FILTER  = Adafruit_BMP280::FILTER_X4;

        // EZ2 Ultrasonic Sensor
        constexpr uint8_t EZ2_PW_PIN = 4;
        constexpr float   EZ2_ALPHA  = 0.1f;

        // Altitude Fusion / Switching logic
        // 高度3m以下 かつ バンク/ピッチ角が一定値以下のとき超音波センサ優先
        inline float ALT_SWITCH_THRESHOLD_M = 3.0f;   // 切替高度 [m]
        inline float ALT_BANK_LIMIT_DEG     = 20.0f;  // 切替許容バンク角 [deg]
    }

    // ============================================================
    //  § 4  タイミング設定
    // ============================================================
    namespace Timing {
        constexpr int MAIN_Hz  = 1000;  // 制御ループ周期 [Hz] ← とっても大事！！！
        constexpr int DEBUG_Hz = 10;    // デバッグ出力周期 [Hz]
        constexpr unsigned long MAIN_PERIOD = 1000000UL / MAIN_Hz; // 周期 [us]
        inline   unsigned long  Main_dt     = 1000000UL / MAIN_Hz; // センサ更新周期 [us]

        template <int Hz>
        bool freq(unsigned long &dt) {
            static uint32_t t_prev = 0;
            constexpr uint32_t period = 1000000UL / Hz;
            uint32_t t_now = micros();
            if (t_now - t_prev < period) return false;
            dt = t_now - t_prev;
            t_prev = t_now;
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

        inline void resetTiming() {
            // 注意: テンプレート内の静的変数はリセットできないため、
            // 各テンプレートインスタンスを手動で「一度空回し」させるか、
            // 大局的な基準時間を更新する設計にする必要があります。
            // ここではシンプルに Main_dt を標準値に戻し、次の freq 呼び出しを待つ設計にします。
            Main_dt = 1000000UL / MAIN_Hz;
        }
    }

    // ============================================================
    //  § 5  通信ピン設定
    // ============================================================
    namespace serial {
        inline HardwareSerial* const im920 = &Serial3; // IM920SL用シリアルポート
        inline HardwareSerial* const sbus  = &Serial2; // S.BUS用シリアルポート
    }

    namespace wire {
        inline TwoWire* const mpu = &Wire;   // MPU6050 用 I2C ポート
        inline TwoWire* const bmp = &Wire1;  // 気圧センサ用 I2C ポート
    }

} // namespace Config