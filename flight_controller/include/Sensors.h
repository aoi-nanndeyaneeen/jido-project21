//センサー系
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <MadgwickAHRS.h>
#include "Config.h"

class IMU {
private:
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
    // ユーザーの報告に基づき: 前倒しでX=-1 -> 前は-X, 右倒しでY=-1 (左は上がって+1のはず) -> 左は-Y
    float getAccX()  { return -(getAccX_Raw() - Config::sensor::s_ax_bias); } // 前 = -X
    float getAccY()  { return -(getAccY_Raw() - Config::sensor::s_ay_bias); } // 左 = -Y
    float getAccZ()  { return  (getAccZ_Raw() - Config::sensor::s_az_bias); } // 上 = +Z

    float getGyroX() { return -(getGyroX_Raw() - Config::sensor::s_gx_bias); }
    float getGyroY() { return -(getGyroY_Raw() - Config::sensor::s_gy_bias); }
    float getGyroZ() { return  (getGyroZ_Raw() - Config::sensor::s_gz_bias); }

    float getRoll()  { return filter.getRoll(); }
    float getPitch() { return filter.getPitch(); }
    float getYaw()   { return filter.getYaw(); }

    void recalibrate() {
        Serial.println("INFO: MPU6050 Recalibration (ax=0, ay=0, az=1 mode)...");
        
        // バイアスを一旦リセット
        Config::sensor::s_ax_bias = 0.0f;
        Config::sensor::s_ay_bias = 0.0f;
        Config::sensor::s_az_bias = 0.0f;
        Config::sensor::s_gx_bias = 0.0f;
        Config::sensor::s_gy_bias = 0.0f;
        Config::sensor::s_gz_bias = 0.0f;

        double sum_ax=0, sum_ay=0, sum_az=0;
        double sum_gx=0, sum_gy=0, sum_gz=0;
        const int samples = 400;
        
        for(int i=0; i<samples; i++) {
            int16_t r_ax, r_ay, r_az, r_gx, r_gy, r_gz;
            mpu.getMotion6(&r_ax, &r_ay, &r_az, &r_gx, &r_gy, &r_gz);
            sum_ax += (float)r_ax / Config::sensor::ACCEL_SCALE;
            sum_ay += (float)r_ay / Config::sensor::ACCEL_SCALE;
            sum_az += (float)r_az / Config::sensor::ACCEL_SCALE;
            sum_gx += (float)r_gx / Config::sensor::GYRO_SCALE;
            sum_gy += (float)r_gy / Config::sensor::GYRO_SCALE;
            sum_gz += (float)r_gz / Config::sensor::GYRO_SCALE;
            if (i % 100 == 0) Serial.print(".");
            delay(2);
        }
        Serial.println(" Done.");

        // 平均をバイアスに設定
        Config::sensor::s_ax_bias = (float)(sum_ax / samples);
        Config::sensor::s_ay_bias = (float)(sum_ay / samples);
        // az を 1.0 にするように設定 (s_az_bias = mean_az - 1.0)
        Config::sensor::s_az_bias = (float)(sum_az / samples - 1.0f);
        
        Config::sensor::s_gx_bias = (float)(sum_gx / samples);
        Config::sensor::s_gy_bias = (float)(sum_gy / samples);
        Config::sensor::s_gz_bias = (float)(sum_gz / samples);

        filter.reset();
        filter.begin(Config::Timing::MAIN_Hz);
        Serial.println("INFO: MPU6050 Recalibration Finished.");
    }
};

// --- 気圧センサ クラス ---
class BarometerSensor {
private:
    Adafruit_BMP280 bmp; 
    float temperature;
    float pressure;
    float raw_altitude;
    float smoothed_altitude;

    float sea_level_pressure; 
    float alpha;              
    float baseline_altitude; // 相対高度のためのベースライン

public:
    BarometerSensor(float sea_level = 1013.25, float filter_alpha = 0.1, TwoWire *wire_i = &Wire1) : bmp(wire_i) {
        sea_level_pressure = sea_level;
        alpha = filter_alpha;
        temperature = 0.0;
        pressure = 0.0;
        raw_altitude = 0.0;
        smoothed_altitude = 0.0;
        baseline_altitude = 0.0;
    }

    bool begin() {
        bool status = bmp.begin(0x76);
        if (!status) status = bmp.begin(0x77);
        
        if (!status) {
            Serial.println("【エラー】BMP280未検出！");
            return false; 
        }
        
        Serial.println("【成功】BMP280接続完了！");
        
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                        Config::sensor::BMP_SAMP_T,     
                        Config::sensor::BMP_SAMP_P,    
                        Config::sensor::BMP_FILTER,      
                        Config::sensor::BMP_STANDBY); 
        return true;
    }

    void update() {
        temperature = bmp.readTemperature();
        pressure = bmp.readPressure() / 100.0F; 
        float alt = bmp.readAltitude(sea_level_pressure);

        // NaN or Inf guard
        if (isnan(alt) || isinf(alt)) {
            // エラー時は前回の値を保持するか、0にする
            if (raw_altitude == 0.0) raw_altitude = 0.0; 
        } else {
            raw_altitude = alt;
        }

        if (smoothed_altitude == 0.0 && !isnan(raw_altitude) && !isinf(raw_altitude)) {
            smoothed_altitude = raw_altitude; 
        } else if (!isnan(raw_altitude) && !isinf(raw_altitude)) {
            smoothed_altitude = (alpha * raw_altitude) + ((1.0 - alpha) * smoothed_altitude);
        }
    }

    float get_temperature() { return temperature; }
    float get_pressure() { return pressure; }
    float get_raw_altitude() { return raw_altitude - baseline_altitude; }
    float get_smoothed_altitude() { return smoothed_altitude - baseline_altitude; }

    void reset() {
        baseline_altitude = raw_altitude;
        smoothed_altitude = raw_altitude;
        Serial.println("INFO: Barometer Baseline Reset.");
    }
};
