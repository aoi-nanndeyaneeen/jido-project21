#pragma once
#include "Config.h"
#include <Arduino.h>

class Flight_mode {
private:
    FlightMode currentMode = MODE_MANUAL;
    FlightMode prevMode = MODE_MANUAL;
public:
    unsigned long modeStartMs = 0;

    // 優先度: 水平飛行 > 水平旋回 > ホバリング(セミマニュアル) > マニュアル
    void update(Sw sw_turn, Sw sw_level, Sw sw_hover) {
        if (sw_level == up) {
            currentMode = MODE_LEVEL_FLIGHT;
        } else if (sw_turn == up) {
            currentMode = MODE_LEVEL_TURN;
        } else if (sw_hover == up) {
            currentMode = MODE_SEMI_MANUAL;
        } else {
            currentMode = MODE_MANUAL;
        }
    }

    bool change() {
        if (currentMode == prevMode) return false;
        prevMode = currentMode;
        return true;
    }

    void set_mode(FlightMode mode) { currentMode = mode; }
    FlightMode get_mode() { return currentMode; }
};