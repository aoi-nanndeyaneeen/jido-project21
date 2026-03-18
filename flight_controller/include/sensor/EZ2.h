#pragma once
#include <Arduino.h>
#include "Config.h"

// MaxBotix EZ2 Ultrasonic Sensor (Pulse Width Output)
class EZ2Sensor {
private:
    uint8_t pin;
    volatile uint32_t pulse_start_us;
    volatile uint32_t last_pulse_duration_us;
    float alpha;
    float smoothed_distance_cm;
    
    // 割り込みハンドラ用の静的ポインタ
    static EZ2Sensor* instance;

    static void handleInterrupt() {
        if (instance == nullptr) return;
        if (digitalRead(instance->pin) == HIGH) {
            instance->pulse_start_us = micros();
        } else {
            uint32_t now = micros();
            if (now > instance->pulse_start_us) {
                instance->last_pulse_duration_us = now - instance->pulse_start_us;
            }
        }
    }

public:
    EZ2Sensor(uint8_t pw_pin, float filter_alpha = 0.1) : pin(pw_pin), alpha(filter_alpha) {
        pulse_start_us = 0;
        last_pulse_duration_us = 0;
        smoothed_distance_cm = 0.0;
        instance = this;
    }

    void begin() {
        pinMode(pin, INPUT);
        attachInterrupt(digitalPinToInterrupt(pin), handleInterrupt, CHANGE);
    }

    void update() {
        uint32_t duration;
        // 割り込みとの競合を避けるために一時的に止めるか、単純に読み込む(32bitなら基本アトミック)
        duration = last_pulse_duration_us;

        if (duration > 0) {
            // 仕様: 1インチあたり147us
            float distance_inches = (float)duration / 147.0f;
            float current_cm = distance_inches * 2.54f;

            // 異常値フィルタ (30cm~500cm程度を有効とする)
            if (current_cm < 20.0f || current_cm > 700.0f) return;

            if (smoothed_distance_cm == 0.0f) {
                smoothed_distance_cm = current_cm;
            } else {
                smoothed_distance_cm = (alpha * current_cm) + ((1.0f - alpha) * smoothed_distance_cm);
            }
        }
    }

    float get_distance_cm() { return smoothed_distance_cm; }
    float get_distance_m() { return smoothed_distance_cm / 100.0f; }
};

// 静的メンバの初期化
inline EZ2Sensor* EZ2Sensor::instance = nullptr;
