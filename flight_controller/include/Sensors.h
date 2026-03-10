//センサー系
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
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
    const float ACCEL_SCALE = 16384.0;
    const float GYRO_SCALE = 131.0;

public:
    IMU(TwoWire *wire_i = &Wire) : mpu(0x68), wire(wire_i) {};

    void begin() {
        wire->begin();
        wire->setClock(400000);
        mpu.initialize();
        filter.begin(FREQUENCY);
        mpu.setXAccelOffset(0);
        mpu.setYAccelOffset(0);
        mpu.setZAccelOffset(0);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
    }

    void update() {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        filter.updateIMU(getGyroX(), getGyroY(), getGyroZ(), getAccX(), getAccY(), getAccZ());
    }

    float getAccX() { return (float)ax / ACCEL_SCALE; }
    float getAccY() { return (float)ay / ACCEL_SCALE; }
    float getAccZ() { return (float)az / ACCEL_SCALE; }
    float getGyroX() { return (float)gx / GYRO_SCALE; }
    float getGyroY() { return (float)gy / GYRO_SCALE; }
    float getGyroZ() { return (float)gz / GYRO_SCALE; }
    float getRoll()  { return filter.getRoll(); }
    float getPitch() { return filter.getPitch(); }
    float getYaw()   { return filter.getYaw(); }
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

public:
    BarometerSensor(float sea_level = 1013.25, float filter_alpha = 0.1, TwoWire *wire_i = &Wire1) : bmp(wire_i) {
        sea_level_pressure = sea_level;
        alpha = filter_alpha;
        temperature = 0.0;
        pressure = 0.0;
        raw_altitude = 0.0;
        smoothed_altitude = 0.0;
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
                        Adafruit_BMP280::SAMPLING_X2,     
                        Adafruit_BMP280::SAMPLING_X16,    
                        Adafruit_BMP280::FILTER_X16,      
                        Adafruit_BMP280::STANDBY_MS_500); 
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
    float get_raw_altitude() { return raw_altitude; }
    float get_smoothed_altitude() { return smoothed_altitude; }
};