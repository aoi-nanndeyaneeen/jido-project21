// ============================================================
//  StatusLed.h  -  モード表示LED (pin 5/6/9) と機体検出LED (pin 21/22/23)
// ============================================================
//  配線は「コモンアノード」前提。
//    3.3V ──┬── R アノード
//           ├── G アノード
//           └── B アノード
//    StatusLed: pin5/6/9
//    BlinkLed : pin21/22/23
//  ピンを HIGH(3.3V) にすると LED 側と電位差ゼロ = 消灯、
//  LOW(GND) にすると 3.3V→LED→pin(GND) に電流が流れて点灯する。
//  なので各ピンは「点けたい色だけ LOW、他は HIGH」で使う (負論理)。
//
//  ★ 抵抗は各色ごとに必ず1本ずつ挟むこと (直結すると過電流)。
//    各色個別抵抗の配線なので混色 (白/黄/シアン) も問題なく出せる。
//
//  【色の意味】 (set() の呼び出し側 = drone_s5.cpp で決めている)
//    黄(点灯) : スロットルカット (DISARM)
//    赤(点灯) : ARMED かつ MODE_ANGLE (手動 bail-out)
//    青(点滅) : ARMED かつ POSHOLD/ALTHOLD (自動系)
//    緑(点滅) : ARMED かつ GUIDED (地上局ガイド飛行)
//  色を変えたいときはこのファイルではなく、呼び出し側の分岐を直す。
// ============================================================
#pragma once
#include <Arduino.h>

namespace StatusLed {

constexpr uint8_t PIN_R = 5;
constexpr uint8_t PIN_G = 6;
constexpr uint8_t PIN_B = 9;

inline void begin() {
    pinMode(PIN_R, OUTPUT);
    pinMode(PIN_G, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    digitalWrite(PIN_R, HIGH);   // 起動直後は全消灯 (負論理なので HIGH=OFF)
    digitalWrite(PIN_G, HIGH);
    digitalWrite(PIN_B, HIGH);
}

// r/g/b = true でその色を点ける (中で負論理に変換する)。
inline void set(bool r, bool g, bool b) {
    digitalWrite(PIN_R, r ? LOW : HIGH);
    digitalWrite(PIN_G, g ? LOW : HIGH);
    digitalWrite(PIN_B, b ? LOW : HIGH);
}

inline void off()   { set(false, false, false); }
inline void white() { set(true,  true,  true);  }
inline void red()   { set(true,  false, false); }
inline void green() { set(false, true,  false); }
inline void blue()  { set(false, false, true);  }
inline void yellow(){ set(true,  true,  false); }
inline void cyan()  { set(false, true,  true);  }
inline void magenta(){set(true,  false, true);  }

} // namespace StatusLed

namespace BlinkLed {

constexpr uint8_t PIN_R = 21;
constexpr uint8_t PIN_G = 22;
constexpr uint8_t PIN_B = 23;

inline void begin() {
    pinMode(PIN_R, OUTPUT);
    pinMode(PIN_G, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    digitalWrite(PIN_R, HIGH);
    digitalWrite(PIN_G, HIGH);
    digitalWrite(PIN_B, HIGH);
}

inline void white(bool on) {
    const uint8_t level = on ? LOW : HIGH;
    digitalWrite(PIN_R, level);
    digitalWrite(PIN_G, level);
    digitalWrite(PIN_B, level);
}

} // namespace BlinkLed
