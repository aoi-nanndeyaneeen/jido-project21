// 制御、演算系
#pragma once
#include "Config.h" // dt などを使うために必要
#include <Arduino.h>

class Flight_mode {
private:
    FlightMode currentMode = MODE_MANUAL;
    FlightMode prevMode = MODE_MANUAL; // モード変化検出用
public:

    unsigned long modeStartMs = 0;
    void update(Sw _Aux1,Sw _Aux2) {
        // ============================================================
        //  モード判定 (スイッチ読み取り)
        // ============================================================
        // 優先度: 8の字 > 水平旋回 > マニュアル
        if (_Aux1 == up)
            currentMode = MODE_FIGURE_8;
        else if (_Aux2 == down)
            currentMode = MODE_LEVEL_TURN;
        else    currentMode = MODE_MANUAL;
    }

    bool change(){
        if (currentMode == prevMode) return false;
        else{
            prevMode = currentMode;
            return true;
        }
    }

    FlightMode get_mode() {return currentMode;}
};