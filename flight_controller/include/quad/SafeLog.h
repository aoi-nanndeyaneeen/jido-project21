// ============================================================
//  SafeLog.h  -  飛行中に呼んでも「待たない」USB シリアル出力
// ============================================================
//  ★ 2026-09-17: arduino-pico の SerialUSB::write() は、PC がポートを開いている
//    (DTR が立っている) のに読んでいないと、送信バッファの空きを最大 1 秒待つ
//    (framework-arduinopico cores/rp2040/SerialUSB.cpp)。PC 側のモニタが遅い/固まって
//    いると、printf 1 回ごとに制御ループが止まる。Teensy の USB Serial も同様に待つ。
//
//  logf() は先に文字列を作ってから出す。出し方は板で違う:
//    RP2040 (2 コア): USB シリアルはコア1 の持ち物 (quad/UsbOwner.h)。コア0 は
//                     リングバッファに積むだけで、コア1 が drainTo() で USB へ流す。
//                     コア0 は USB に一切触らない = USB がどれだけ詰まっても止まらない。
//                     リングに入りきらなければ捨てて dropped() を数える。
//    Teensy (1 コア): 送信バッファの空きに丸ごと収まるときだけ直接書く。
//  アーム中に通りうる出力 (モード遷移 / GUIDED の開始・完了など) はこれを使う。
//  ★ logf() を呼んでよいのはコア0 だけ (リングの書き手は 1 人)。
//
//  ★ 1 回の出力は 240 バイト以内 (日本語は 1 文字 3 バイト)。超える分は切り詰める。
// ============================================================
#pragma once
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

namespace Quad {
namespace SafeLog {

static uint32_t s_dropped = 0;

#if defined(ARDUINO_ARCH_RP2040) && !defined(S5_SINGLE_CORE)
// 1 書き手 (コア0) / 1 読み手 (コア1) のリング。head は書き手だけ、tail は読み手だけが進める。
constexpr uint16_t RING = 4096;
static char s_ring[RING];
static volatile uint16_t s_head = 0;
static volatile uint16_t s_tail = 0;

inline void push(const char* buf, int n) {
    const uint16_t h = s_head, t = s_tail;
    const uint16_t used = (uint16_t)(h - t) & (RING - 1);
    if (n > (int)(RING - 1 - used)) { s_dropped++; return; }
    for (int i = 0; i < n; ++i) s_ring[(h + i) & (RING - 1)] = buf[i];
    s_head = (uint16_t)((h + n) & (RING - 1));
}

// コア1 から。溜まっているぶんを USB へ (ここは待ってよい)。
inline void drainTo(Print& out) {
    char chunk[128];
    for (;;) {
        const uint16_t h = s_head, t = s_tail;
        if (h == t) return;
        uint16_t n = 0;
        while (n < sizeof(chunk) && ((t + n) & (RING - 1)) != h) {
            chunk[n] = s_ring[(t + n) & (RING - 1)];
            n++;
        }
        out.write((const uint8_t*)chunk, n);
        s_tail = (uint16_t)((t + n) & (RING - 1));
    }
}
#else
inline void push(const char* buf, int n) {
    if (Serial.availableForWrite() < n) { s_dropped++; return; }
    Serial.write((const uint8_t*)buf, (size_t)n);
}
inline void drainTo(Print&) {}
#endif

inline void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
inline void logf(const char* fmt, ...) {
    char buf[240];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    push(buf, n);
}

inline uint32_t dropped() { return s_dropped; }

} // namespace SafeLog
} // namespace Quad
