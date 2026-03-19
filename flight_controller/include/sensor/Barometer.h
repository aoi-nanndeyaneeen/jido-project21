#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "Config.h"

class BarometerSensor {
private:
    Adafruit_BMP280 bmp; 
    float temperature;
    float pressure;
    float raw_altitude;
    float smoothed_altitude;

    float sea_level_pressure; 
    float alpha;              
    float baseline_altitude; 

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

        if (isnan(alt) || isinf(alt)) {
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
