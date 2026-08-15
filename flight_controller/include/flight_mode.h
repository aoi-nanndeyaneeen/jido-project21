#pragma once
#include "Config.h"
#include <Arduino.h>

class Flight_mode {
private:
    FlightMode currentMode = MODE_MANUAL;
    FlightMode prevMode = MODE_MANUAL;
public:
    unsigned long modeStartMs = 0;

    // 優先度: 地上局自律制御(SW_AUTO+受信新鮮) > 水平飛行 > 水平旋回 > ホバリング(セミマニュアル) > マニュアル
    // ★ SW_AUTOが最優先なのは、パイロットが物理スイッチで明示的に権限委譲した場合のみ
    //   有効になるため。スイッチOFF、またはIM920リンクが途絶えた瞬間に下の優先順位へ
    //   自動的にフォールバックする（安全側）。
    void update(Sw sw_turn, Sw sw_level, Sw sw_hover, Sw sw_auto, bool ground_fresh) {
        if (sw_auto == up && ground_fresh) {
            currentMode = MODE_AUTONOMOUS;
        } else if (sw_level == up) {
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