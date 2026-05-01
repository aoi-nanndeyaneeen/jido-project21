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
    // 合計 26 bytes

    void print() const {
        Serial.println("=== Ground Data ===");
        Serial.printf("PID: P=%.4f I=%.4f D=%.4f  sel=%d\n", p_adj, i_adj, d_adj, param_sel);
        Serial.printf("Att: Roll=%.1f Pitch=%.1f Yaw=%.1f\n", roll, pitch, yaw);
    }
};
constexpr int GROUND_DATA_NUM = 6;


// ==== スイッチやチャンネルの定義 ====
enum Sw { up, cen, down };

enum Ch {
    ROLL, PITCH, THR, YAW, Aux1, Aux2, THR_CUT, Aux3, Aux4, Aux5
};

enum FlightMode : uint8_t {
    MODE_MANUAL      = 0,  // プロポ直接操作
    MODE_LEVEL_TURN  = 1,  // 自動水平旋回
    MODE_LEVEL_FLIGHT= 2,  // 水平飛行
    MODE_SEMI_MANUAL = 3,  //4にするな範囲外アクセスになる
};



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
} // namespace Config