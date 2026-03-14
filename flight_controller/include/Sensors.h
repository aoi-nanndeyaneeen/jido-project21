//センサー系
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <MadgwickAHRS.h>
#include "Config.h" // Timing::FREQUENCYなどを使うため

class IMU {
private:
    MPU6050 mpu;
    Madgwick filter;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    TwoWire *wire;

public:
    IMU(TwoWire *wire_i = &Wire) : mpu(0x68), wire(wire_i) {};

    void begin() {
        wire->begin();
        wire->setClock(400000);
        mpu.initialize();

        // --- スケール強制設定 MPU6050 (±2g / ±250dps) ---
        // main_test.cpp と同等の精度と感度にするためアンプゲインを固定
        wire->beginTransmission(0x68); wire->write(0x1C); wire->write(0x00); wire->endTransmission(); // Accel ±2g
        wire->beginTransmission(0x68); wire->write(0x1B); wire->write(0x00); wire->endTransmission(); // Gyro ±250dps

        filter.begin(Config::Timing::MAIN_Hz);
        
        // Config.h で定義されたオフセットを適用
        mpu.setXAccelOffset(Config::sensor::ACCEL_X_OFFSET);
        mpu.setYAccelOffset(Config::sensor::ACCEL_Y_OFFSET);
        mpu.setZAccelOffset(Config::sensor::ACCEL_Z_OFFSET);
        mpu.setXGyroOffset(Config::sensor::GYRO_X_OFFSET);
        mpu.setYGyroOffset(Config::sensor::GYRO_Y_OFFSET);
        mpu.setZGyroOffset(Config::sensor::GYRO_Z_OFFSET);
    }

    void update() {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        filter.updateIMU(getGyroX(), getGyroY(), getGyroZ(), getAccX(), getAccY(), getAccZ());
    }

    float getAccX() { return (float)ax / Config::sensor::ACCEL_SCALE; }
    float getAccY() { return (float)ay / Config::sensor::ACCEL_SCALE; }
    float getAccZ() { return (float)az / Config::sensor::ACCEL_SCALE; }
    float getGyroX() { return (float)gx / Config::sensor::GYRO_SCALE; }
    float getGyroY() { return (float)gy / Config::sensor::GYRO_SCALE; }
    float getGyroZ() { return (float)gz / Config::sensor::GYRO_SCALE; }
    float getRoll()  { return filter.getRoll(); }
    float getPitch() { return filter.getPitch(); }
    float getYaw()   { return filter.getYaw(); }

    void recalibrate() {
        Serial.println("INFO: MPU6050 Recalibrating (KEEP STILL)...");
        // 一度オフセットをクリアして生データを確認する
        mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
        mpu.setXGyroOffset(0);  mpu.setYGyroOffset(0);  mpu.setZGyroOffset(0);
        delay(100);

        long s_ax=0, s_ay=0, s_az=0, s_gx=0, s_gy=0, s_gz=0;
        const int samples = 200;
        for(int i=0; i<samples; i++) {
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            s_ax+=ax; s_ay+=ay; s_az+=az;
            s_gx+=gx; s_gy+=gy; s_gz+=gz;
            delay(5);
        }
        // MPU6050のハードウェアオフセットレジスタへの書き込み
        // accelは/8、gyroは/4のスケール調整が必要（チップの仕様）
        // また、符号は「現在の値を打ち消す（マイナス）」方向
        mpu.setXAccelOffset((int16_t)(-s_ax/samples/8));
        mpu.setYAccelOffset((int16_t)(-s_ay/samples/8));
        mpu.setZAccelOffset((int16_t)-(s_az/samples - 16384)/8);
        mpu.setXGyroOffset ((int16_t)(-s_gx/samples/4));
        mpu.setYGyroOffset ((int16_t)(-s_gy/samples/4));
        mpu.setZGyroOffset ((int16_t)(-s_gz/samples/4));

        // フィルタ内部状態も完全にリセット
        filter.reset();
        filter.begin(Config::Timing::MAIN_Hz);
        Serial.println("INFO: MPU6050 Ready.");
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
        // ★ 0x76 と 0x77 の両方を自動で試す最強の初期化
        bool status = bmp.begin(0x76);
        if (!status) {
            status = bmp.begin(0x77);
        }
        
        if (!status) {
            Serial.println("【エラー】BMP280が見つかりません！配線を確認してください！");
            return false; 
        }
        
        Serial.println("【成功】BMP280の接続を確認しました！");
        
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                        Config::sensor::BMP_SAMP_T,     
                        Config::sensor::BMP_SAMP_P,    
                        Config::sensor::BMP_FILTER,      
                        Config::sensor::BMP_STANDBY); 
        return true;
    }

// ... 以下略 (update関数などはそのまま) ...
    void update() {
        temperature = bmp.readTemperature();
        pressure = bmp.readPressure() / 100.0F; 
        raw_altitude = bmp.readAltitude(sea_level_pressure);

        if (smoothed_altitude == 0.0) {
            smoothed_altitude = raw_altitude; 
        } else {
            smoothed_altitude = (alpha * raw_altitude) + ((1.0 - alpha) * smoothed_altitude);
        }
    }

    float get_temperature() { return temperature; }
    float get_pressure() { return pressure; }
    float get_raw_altitude() { return raw_altitude - baseline_altitude; }
    float get_smoothed_altitude() { return smoothed_altitude - baseline_altitude; }

    void reset() {
        // 現在の気圧高度をベースラインに設定
        baseline_altitude = raw_altitude;
        smoothed_altitude = raw_altitude; // フィルタも同期させる
        Serial.println("INFO: Barometer Baseline Reset to 0.0m.");
    }
};