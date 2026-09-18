// ============================================================
//  UsbOwner.h  -  USB シリアルをどちらのコアが使ってよいかの取り決め (RP2040)
// ============================================================
//  ★ 2026-09-17: デバッグ画面とキー入力をコア1 へ移した。理由:
//    arduino-pico の USB シリアルは 2 コア共通のロック (USB.mutex) で守られていて、
//    片方のコアが書き込みで待っている間 (PC が読まないと最大 1 秒)、もう片方のコアが
//    Serial.available() を呼んだだけでも同じだけ止まる (cores/rp2040/CoreMutex.cpp は
//    相手コアが持っていると mutex_enter_blocking)。画面をコア1 へ移すだけでは足りず、
//    「コア0 は普段 USB に触らない」ことが要る。
//
//  取り決め:
//    ・普段はコア1 が USB の持ち主。画面表示 / キー読み取り / SafeLog の吐き出し。
//    ・コア0 が USB を使うのは次の場合だけで、そのときコア1 は USB から手を引く (parked):
//        1) キー処理: コア1 がキーを読んだら自分で parked にしてから key_pending を立てる。
//           コア0 は handleKey() (メニュー等のブロッキング入出力を含む) を実行し、
//           終わったら key_pending を下ろす。
//        2) BLE の地上メンテ指令の出力: コア0 が acquire() でコア1 を止めてから書く
//           (ディスアームのときだけ。アーム中の指令は SafeLog だけで出す)。
//        3) USB 直結ログ ('l'): FlightLog::Usb::active の間は、コア1 は画面と
//           SafeLog を止め、キーだけ読む (ログはコア0 が空きを見ながら書く)。
//    ・setup() の間はコア0 が持ち主 (起動メッセージ)。終わったら setupDone() で渡す。
//
//  フラグは 1 バイトの volatile。書き手が決まっているので排他は要らない。
//  Teensy (1 コア) と -D S5_SINGLE_CORE では何もしない。
// ============================================================
#pragma once
#include <Arduino.h>

namespace S5 {
namespace UsbOwner {

#if defined(ARDUINO_ARCH_RP2040) && !defined(S5_SINGLE_CORE)
constexpr bool DUAL = true;

static volatile bool s_setup_done  = false;   // 書き手: コア0
static volatile bool s_park_req    = false;   // 書き手: コア0 (acquire/release)
static volatile bool s_parked      = false;   // 書き手: コア1
static volatile bool s_key_pending = false;   // 立てる: コア1 / 下ろす: コア0
static volatile char s_key_char    = 0;       // 書き手: コア1

inline void setupDone() { s_setup_done = true; }
inline bool setupIsDone() { return s_setup_done; }

// コア0: コア1 を USB から退かせる。timeout_ms 待っても退かなければ false (使わないこと)。
inline bool acquire(uint32_t timeout_ms) {
    s_park_req = true;
    const uint32_t t0 = millis();
    while (!s_parked) {
        if (millis() - t0 >= timeout_ms) return false;
    }
    return true;
}
inline void release() { s_park_req = false; }

// コア0: 処理待ちのキーがあれば取り出す (コア1 は既に parked)。
inline bool keyPending(char& c) {
    if (!s_key_pending) return false;
    c = s_key_char;
    return true;
}
inline void keyDone() { s_key_pending = false; }

// コア1: いま USB に触ってよいか。触れないときは parked を立てて false。
inline bool core1MayUse() {
    if (!s_setup_done || s_park_req || s_key_pending) { s_parked = true; return false; }
    s_parked = false;
    return true;
}
// コア1: キーを 1 つコア0 へ渡す。渡した時点で USB から手を引く。
inline void core1PostKey(char c) {
    s_parked = true;
    s_key_char = c;
    s_key_pending = true;
}
#else
constexpr bool DUAL = false;
inline void setupDone() {}
inline bool setupIsDone() { return true; }
inline bool acquire(uint32_t) { return true; }
inline void release() {}
inline bool keyPending(char&) { return false; }
inline void keyDone() {}
#endif

} // namespace UsbOwner
} // namespace S5
