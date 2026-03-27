// 制御、演算系
#pragma once
#include "Config.h" // dt などを使うために必要
#include <Arduino.h>

class Flight_mode {
private:
    FlightMode currentMode = MODE_MANUAL; // 初期モードをMANUALに変更
    FlightMode prevMode = MODE_MANUAL;    // モード変化検出用
public:

    unsigned long modeStartMs = 0;
    
    // main.cpp から Mode.update(Aux2, Aux3) として呼ばれる
    void update(Sw switch_Turn, Sw switch_Fig8) {
        // ============================================================
        //  モード判定 (独立スイッチ版)
        // ============================================================
        // 優先度: 8の字 > 水平旋回 > マニュアル(基本モード)
        if (switch_Fig8 == up) {
            currentMode = MODE_FIGURE_8;   // Aux3 が上なら8の字
        }
        else if (switch_Turn == up) {
            currentMode = MODE_LEVEL_TURN; // Aux2 が上なら水平旋回
        }
        else {
            currentMode = MODE_MANUAL;     // どちらもオフならマニュアル
        }
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