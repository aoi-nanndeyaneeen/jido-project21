// ============================================================
//  StatusLed.h  -  モードをひと目で分かるようにする RGB LED (pin 5/6/9)
// ============================================================
//  配線は「コモンアノード」前提。
//    3.3V ──┬── R アノード
//           ├── G アノード
//           └── B アノード
//    pin5 ──[抵抗]── R カソード
//    pin6 ──[抵抗]── G カソード
//    pin9 ──[抵抗]── B カソード
//  ピンを HIGH(3.3V) にすると LED 側と電位差ゼロ = 消灯、
//  LOW(GND) にすると 3.3V→LED→pin(GND) に電流が流れて点灯する。
//  なので各ピンは「点けたい色だけ LOW、他は HIGH」で使う (負論理)。
//
//  ★ 抵抗は各色ごとに必ず1本ずつ挟むこと (直結すると過電流)。
//    アノード側に1本だけの「共通抵抗」にすると、Vf が一番低い赤ダイが
//    電流を独り占めして混色 (白/黄/シアン) が出せない。現状の配線が
//    これなので、呼び出し側は単色 (赤/緑/青) だけで状態を区別している。
//
//  【色の意味】 (set() の呼び出し側 = drone_s5.cpp で決めている)
//    赤  : DISARM
//    青  : ARMED かつ MODE_ANGLE (手動 bail-out)
//    緑  : ARMED かつそれ以外 (ALTHOLD / POSHOLD など自動系)
//  色を変えたいときはこのファイルではなく、呼び出し側の分岐を直す。
// ============================================================
#pragma once
#include <Arduino.h>

namespace StatusLed {

// ★ 2026-09-10: B を 7 -> 9 に移動。ピン7/8 を Serial2 (LOGLINK) に明け渡すため。
//   ピン9 は SD HW-125 の CS だったが、USE_LOGLINK=true では SD を積まないので空く。
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
