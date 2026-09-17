// ============================================================
//  BoardPins.h  -  drone_s5 の「板ごとに違うもの」を 1 か所に集める
// ============================================================
//  Teensy 4.0 (現行) と XIAO RP2040 (2026-09-17 Teensy 故障で移行) の
//  シリアルポート・モーターピン・LED ピンをここで切り替える。
//  経緯と RP2040 側のピン収支は quad/LogLink.h の冒頭を参照。
//
//  分岐マクロ:
//    ARDUINO_ARCH_RP2040 … arduino-pico (earlephilhower) core が自動定義
//    それ以外            … Teensy (platformio.ini の -D USE_TEENSY)
//
//  ★ RP2040 の制約 (arduino-pico SerialUART.cpp setRX() より):
//      UART0 RX 可能 GPIO = {1, 13, 17, 29}  → XIAO では GP1 (D7) と GP29 (D3)
//      UART1 RX 可能 GPIO = {5, 9, 21, 25}   → XIAO のパッドには 1 本も無い
//    つまりハード UART で RX を取れるのは実質 UART0 だけ。2Mbaud 双方向の
//    LogLink を UART0 (Serial1, D6/D7) に置き、SBUS (100kbaud 反転 RX のみ) は
//    SerialPIO で受ける (src/trainer.cpp と同じ手法。実績あり)。
// ============================================================
#pragma once
#include <Arduino.h>

#ifdef ARDUINO_ARCH_RP2040

// ---- XIAO RP2040 ----------------------------------------------------------
//   D6  GP0   LogLink TX  → C3 GPIO20 (RX)      UART0 = Serial1 の既定ピン
//   D7  GP1   LogLink RX  ← C3 GPIO21 (TX)
//   D4  GP6   I2C SDA                            Wire の既定ピン
//   D5  GP7   I2C SCL
//   D0  GP26  モーター 1 PWM  (slice 5A)
//   D1  GP27  モーター 2 PWM  (slice 5B)
//   D2  GP28  モーター 3 PWM  (slice 6A)
//   D3  GP29  モーター 4 PWM  (slice 6B)
//   D10 GP3   SBUS RX (SerialPIO, 反転)
//   D8  GP2   機体検出 LED (BlinkLed) → ドライバ (ロジックシフタ) の入力 1 本
//   D9  GP4   空き (UART1 TX 可。IM920 を復活させるならここ + SerialPIO RX)
//   内蔵 GP17/16/25  (未使用)
//
//  ★ BlinkLed (地上カメラ用の白ストロボ) は Teensy では 3 ピン (17/22/23) だったが、
//    実機はドライバ経由で 3 色を同時に叩いているので信号は 1 本でよい。
//    BlinkLed::white() は 3 色を常に同じ値にするので、R/G/B を全部 D8 に束ねる。
//  ★ StatusLed (モード表示 RGB) は FC にピンが無いのでロガー (ESP32C3) 側に付ける。
//    FC は色を LogLink の T_LED で送るだけ (S5::STATUS_LED_VIA_LINK)。
//    ロガー側のピンは log_recorder/src/main.cpp の Led:: を参照 (D8/D9/D10)。
//
//  SBUS: RX のみ。TX は NOPIN (Arduino.h のマクロ。SerialPIO:: を付けると展開が壊れる)。
//  ★ setInvertRX(true) は begin() より前に呼ぶこと (drone_s5.cpp の setup)。
inline SerialPIO g_sbus_serial(NOPIN, D10);
#define BOARD_SBUS_SERIAL     g_sbus_serial
#define BOARD_LOGLINK_SERIAL  Serial1
// IM920 は RP2040 ビルドでは未接続 (GROUND_LINK=BLE のため USE_IM920=false)。
// S5T::Tx / S5C::Rx がポインタを要求するので、どこにも繋がらないダミーを渡す。
inline SerialPIO g_im920_serial(NOPIN, NOPIN);
#define BOARD_IM920_SERIAL    g_im920_serial

constexpr int     BOARD_MOTOR_PIN[4] = { D0, D1, D2, D3 };
constexpr uint8_t BOARD_LED_R = 17, BOARD_LED_G = 16, BOARD_LED_B = 25;   // 未使用 (StatusLed はロガー側)
constexpr uint8_t BOARD_BLINK_R = D8, BOARD_BLINK_G = D8, BOARD_BLINK_B = D8;   // ドライバ経由、1 本

#define BOARD_LABEL           "XIAO RP2040"
#define BOARD_LOGLINK_DESC    "Serial1: TX=D6/GP0 RX=D7/GP1"
#define BOARD_SBUS_DESC       "PIO RX=D10/GP3"
#define BOARD_IM920_DESC      "未接続"
#define BOARD_I2C_DESC        "SDA=D4/GP6 SCL=D5/GP7"

#else

// ---- Teensy 4.0 (現行。変更なし) --------------------------------------------
#define BOARD_SBUS_SERIAL     Serial5     // pins 20/21
#define BOARD_LOGLINK_SERIAL  Serial2     // RX7 / TX8
#define BOARD_IM920_SERIAL    Serial3     // pins 14/15

constexpr int     BOARD_MOTOR_PIN[4] = { 1, 2, 3, 4 };
constexpr uint8_t BOARD_LED_R = 5, BOARD_LED_G = 6, BOARD_LED_B = 9;
constexpr uint8_t BOARD_BLINK_R = 22, BOARD_BLINK_G = 23, BOARD_BLINK_B = 17;

#define BOARD_LABEL           "Teensy 4.0"
#define BOARD_LOGLINK_DESC    "Serial2: TX=8 RX=7"
#define BOARD_SBUS_DESC       "Serial5"
#define BOARD_IM920_DESC      "Serial3"
#define BOARD_I2C_DESC        "SDA=18 SCL=19"

#endif
