#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>
#include "Config.h"

class IMU {
private:
    // recalibrate() の妥当性チェックのしきい値。
    //  傾き: 水平な床に置いたつもりでも脚の高さで数度は出るので 5度まで許す。
    //        それを超えたら「水平ではない」と見なして採用しない。
    //  ジャイロ振れ幅: 手で持っているとすぐ数 deg/s 動く。静止なら 1 未満。
    static constexpr float MAX_CAL_TILT_DEG  = 5.0f;
    static constexpr float MAX_CAL_GYRO_SPAN = 3.0f;

    MPU6050 mpu;
    Madgwick filter;
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    TwoWire *wire;

public:
    IMU(TwoWire *wire_i = &Wire) : mpu(0x68), wire(wire_i) {};

    void begin() {
        wire->begin();
        wire->setClock(400000);
        mpu.initialize();

        // --- スケール強制設定 MPU6050 (±2g / ±250dps) ---
        wire->beginTransmission(0x68); wire->write(0x1C); wire->write(0x00); wire->endTransmission(); // Accel ±2g
        wire->beginTransmission(0x68); wire->write(0x1B); wire->write(0x00); wire->endTransmission(); // Gyro ±250dps
        
        // ★ここを追加：DLPF (Digital Low Pass Filter) を42Hzに設定
        wire->beginTransmission(0x68); wire->write(0x1A); wire->write(0x03); wire->endTransmission();
        
        filter.begin(Config::Timing::MAIN_Hz);
    }

    void update() {
        // センサから生データを読み出す
        mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

        // --- ソフトウェア・キャリブレーション補正 ---
        float c_ax = getAccX();
        float c_ay = getAccY();
        float c_az = getAccZ();
        float c_gx = getGyroX();
        float c_gy = getGyroY();
        float c_gz = getGyroZ();

        // Madgwickフィルタの更新
        filter.updateIMU(c_gx, c_gy, c_gz, c_ax, c_ay, c_az);
    }

    // 生データスケール変換のみ (Raw Scaled)
    float getAccX_Raw() { return (float)ax_raw / Config::sensor::ACCEL_SCALE; }
    float getAccY_Raw() { return (float)ay_raw / Config::sensor::ACCEL_SCALE; }
    float getAccZ_Raw() { return (float)az_raw / Config::sensor::ACCEL_SCALE; }
    float getGyroX_Raw() { return (float)gx_raw / Config::sensor::GYRO_SCALE; }
    float getGyroY_Raw() { return (float)gy_raw / Config::sensor::GYRO_SCALE; }
    float getGyroZ_Raw() { return (float)gz_raw / Config::sensor::GYRO_SCALE; }
    
    // ソフトウェア補正適用済みの値 (機体座標系: Forward, Left, Up)
    float getAccX()  { return (getAccX_Raw() - Config::sensor::s_ax_bias); } // 前 = -X
    float getAccY()  { return (getAccY_Raw() - Config::sensor::s_ay_bias); } // 左 = -Y
    float getAccZ()  { return (getAccZ_Raw() - Config::sensor::s_az_bias); } // 上 = Z

    float getGyroX() { return -(getGyroX_Raw() - Config::sensor::s_gx_bias); }
    float getGyroY() { return -(getGyroY_Raw() - Config::sensor::s_gy_bias); }
    float getGyroZ() { return  (getGyroZ_Raw() - Config::sensor::s_gz_bias); }

    float getRoll()  { return filter.getRoll(); }
    float getPitch() { return filter.getPitch(); }
    float getYaw()   { return filter.getYaw(); }

    void recalibrate() {
        Serial.println("INFO: MPU6050 Recalibration (ax=0, ay=0, az=1 mode)...");
        
        // 却下したときに戻せるよう、今の値を退避しておく
        const float old_ax = Config::sensor::s_ax_bias;
        const float old_ay = Config::sensor::s_ay_bias;
        const float old_az = Config::sensor::s_az_bias;
        const float old_gx = Config::sensor::s_gx_bias;
        const float old_gy = Config::sensor::s_gy_bias;
        const float old_gz = Config::sensor::s_gz_bias;

        Config::sensor::s_ax_bias = 0.0f;
        Config::sensor::s_ay_bias = 0.0f;
        Config::sensor::s_az_bias = 0.0f;
        Config::sensor::s_gx_bias = 0.0f;
        Config::sensor::s_gy_bias = 0.0f;
        Config::sensor::s_gz_bias = 0.0f;

        double sum_ax=0, sum_ay=0, sum_az=0;
        double sum_gx=0, sum_gy=0, sum_gz=0;
        // 「動いていないか」を見るためにジャイロの振れ幅も取る
        float min_gx=1e9f, max_gx=-1e9f, min_gy=1e9f, max_gy=-1e9f,
              min_gz=1e9f, max_gz=-1e9f;
        const int samples = 400;

        for(int i=0; i<samples; i++) {
            int16_t r_ax, r_ay, r_az, r_gx, r_gy, r_gz;
            mpu.getMotion6(&r_ax, &r_ay, &r_az, &r_gx, &r_gy, &r_gz);
            sum_ax += (float)r_ax / Config::sensor::ACCEL_SCALE;
            sum_ay += (float)r_ay / Config::sensor::ACCEL_SCALE;
            sum_az += (float)r_az / Config::sensor::ACCEL_SCALE;
            const float gxv = (float)r_gx / Config::sensor::GYRO_SCALE;
            const float gyv = (float)r_gy / Config::sensor::GYRO_SCALE;
            const float gzv = (float)r_gz / Config::sensor::GYRO_SCALE;
            sum_gx += gxv;  sum_gy += gyv;  sum_gz += gzv;
            min_gx = fminf(min_gx, gxv);  max_gx = fmaxf(max_gx, gxv);
            min_gy = fminf(min_gy, gyv);  max_gy = fmaxf(max_gy, gyv);
            min_gz = fminf(min_gz, gzv);  max_gz = fmaxf(max_gz, gzv);
            if (i % 100 == 0) Serial.print(".");
            delay(2);
        }
        Serial.println(" Done.");

        const float m_ax = (float)(sum_ax / samples);
        const float m_ay = (float)(sum_ay / samples);
        const float m_az = (float)(sum_az / samples);
        const float m_gx = (float)(sum_gx / samples);
        const float m_gy = (float)(sum_gy / samples);
        const float m_gz = (float)(sum_gz / samples);

        // ------------------------------------------------------------
        //  ★ 妥当性チェック (2026-09-04 追加)
        //
        //  以前はここで無条件に採用していた。そのため、傾いた床・手に持った
        //  状態・裏返しで 'k' を押すと、その姿勢が黙って「水平」として
        //  登録され、機体はその方向へ飛んでいく。エラーも警告も出ないので
        //  外からは絶対に気づけない。実機で az = -1.01g (裏返し) のまま
        //  キャリブレーションが通ってしまうのを確認した。
        //
        //  水平に静止していれば必ず ax≈0, ay≈0, az≈+1, ジャイロの振れ幅も
        //  小さい。そうでなければ採用せず、元の値を残す。
        // ------------------------------------------------------------
        const float tilt_deg = atan2f(sqrtf(m_ax * m_ax + m_ay * m_ay),
                                      fabsf(m_az)) * 57.2957795f;
        const float g_norm   = sqrtf(m_ax * m_ax + m_ay * m_ay + m_az * m_az);
        const float g_span   = fmaxf(fmaxf(max_gx - min_gx, max_gy - min_gy),
                                     max_gz - min_gz);

        const char* reason = nullptr;
        if (m_az < 0.0f)                       reason = "機体が裏返っています (az が負)";
        else if (g_norm < 0.85f || g_norm > 1.15f)
                                               reason = "加速度の大きさが 1g から外れています (動いている / センサ異常)";
        else if (tilt_deg > MAX_CAL_TILT_DEG)  reason = "機体が傾いています";
        else if (g_span > MAX_CAL_GYRO_SPAN)   reason = "機体が動いています";

        if (reason) {
            // 採用しない。退避しておいた元の値を戻す。
            Config::sensor::s_ax_bias = old_ax;
            Config::sensor::s_ay_bias = old_ay;
            Config::sensor::s_az_bias = old_az;
            Config::sensor::s_gx_bias = old_gx;
            Config::sensor::s_gy_bias = old_gy;
            Config::sensor::s_gz_bias = old_gz;
            Serial.println();
            Serial.printf("!! CALIBRATION REJECTED: %s\n", reason);
            Serial.printf("   実測 ax=%+.4f ay=%+.4f az=%+.4f |a|=%.4f g   "
                          "傾き %.1f deg   ジャイロ振れ幅 %.2f deg/s\n",
                          m_ax, m_ay, m_az, g_norm, tilt_deg, g_span);
            Serial.printf("   許容: 傾き < %.1f deg / ジャイロ振れ幅 < %.1f deg/s / az > 0\n",
                          MAX_CAL_TILT_DEG, MAX_CAL_GYRO_SPAN);
            Serial.println("   → 水平な床に置き、手を離して静止させてから 'k' を押し直してください。");
            Serial.println("   キャリブレーション値は変更していません (前の値のまま)。");
            return;
        }

        Config::sensor::s_ax_bias = m_ax;
        Config::sensor::s_ay_bias = m_ay;
        Config::sensor::s_az_bias = m_az - 1.0f;

        Config::sensor::s_gx_bias = m_gx;
        Config::sensor::s_gy_bias = m_gy;
        Config::sensor::s_gz_bias = m_gz;

        filter.reset();
        filter.begin(Config::Timing::MAIN_Hz);
        Serial.println("INFO: MPU6050 Recalibration Finished.");
        Serial.printf("INFO: New Biases: ax=%.4f ay=%.4f az=%.4f gx=%.4f gy=%.4f gz=%.4f\n",
                      Config::sensor::s_ax_bias, Config::sensor::s_ay_bias, Config::sensor::s_az_bias,
                      Config::sensor::s_gx_bias, Config::sensor::s_gy_bias, Config::sensor::s_gz_bias);
    }
};
