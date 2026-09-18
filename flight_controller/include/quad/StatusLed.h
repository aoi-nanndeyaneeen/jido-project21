// ============================================================
//  StatusLed.h  -  モード表示LED (StatusLed) と機体検出LED (BlinkLed)。ピンは quad/BoardPins.h
// ============================================================
//  配線は「コモンアノード」前提。
//    3.3V ──┬── R アノード
//           ├── G アノード
//           └── B アノード
//    StatusLed: pin5/6/9
//    BlinkLed : pin20/22/23
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
#include "quad/BoardPins.h"

namespace StatusLed {

// 板ごとのピンは quad/BoardPins.h (Teensy 外付け 5/6/9 / XIAO RP2040 内蔵 17/16/25)。どちらも負論理。
constexpr uint8_t PIN_R = BOARD_LED_R;
constexpr uint8_t PIN_G = BOARD_LED_G;
constexpr uint8_t PIN_B = BOARD_LED_B;

#ifdef ARDUINO_ARCH_RP2040
// XIAO RP2040: この板にはピンが無いので色を覚えるだけ。drone_s5.cpp の loop が
// rgbBits() を LogLink::serviceLed() に渡し、ロガー (ESP32C3) 側の LED が光る。
static uint8_t s_rgb = 0;
inline void begin() {}
inline void set(bool r, bool g, bool b) {
    // 実機の LED 配線では赤/青が逆に繋がっているので、論理色の R/B を物理ピンへ再マッピングする。
    s_rgb = (uint8_t)((b ? 1u : 0u) | (g ? 2u : 0u) | (r ? 4u : 0u));
}
inline uint8_t rgbBits() { return s_rgb; }
#else
inline void begin() {
    pinMode(PIN_R, OUTPUT);
    pinMode(PIN_G, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    digitalWrite(PIN_R, HIGH);   // 起動直後は全消灯 (負論理なので HIGH=OFF)
    digitalWrite(PIN_G, HIGH);
    digitalWrite(PIN_B, HIGH);
}

// r/g/b = true でその色を点ける (中で負論理に変換する)。
// 実機の配線では R と B を入れ替える必要がある。
inline void set(bool r, bool g, bool b) {
    digitalWrite(PIN_R, b ? LOW : HIGH);
    digitalWrite(PIN_G, g ? LOW : HIGH);
    digitalWrite(PIN_B, r ? LOW : HIGH);
}
inline uint8_t rgbBits() { return 0; }   // Teensy では未使用
#endif

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

// 板ごとのピンは quad/BoardPins.h。XIAO RP2040 では 3 色とも同じピン (ドライバ経由 1 本)。
constexpr uint8_t PIN_R = BOARD_BLINK_R;
constexpr uint8_t PIN_G = BOARD_BLINK_G;
constexpr uint8_t PIN_B = BOARD_BLINK_B;

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
