// ============================================================
//  Scheduler.h  -  周期実行のヘルパ (Ticker / blink)
// ============================================================
//  drone_s5 の loop() は 1000Hz を目標にしたメインループを起点に、複数の周期を
//  重ねて回す。周期はすべて時間基準の Ticker で作る。
//
//    Ticker  : 時間基準。micros() で周期を測り、実測 dt も返す。
//
//  ★ 2026-09-18: 回数基準の Divider (親ループ N 回に 1 回) は廃止した。
//    「1000Hz の 1/5 = 200Hz」は Teensy でしか成り立たず、RP2040 の実効
//    630〜700Hz では角度ループが 130Hz、GUIDED 翻訳が 65Hz になっていた。
//    メインループの周期が板で変わっても各サブ周期が変わらないよう、
//    周期が要るものは Ticker、要らないもの (角度 PID) は毎ループ回す。
// ============================================================
#pragma once
#include <Arduino.h>

namespace Quad {

struct Ticker {
    uint32_t period_us;
    uint32_t prev_us = 0;
    uint32_t dt_us   = 0;

    explicit Ticker(uint32_t hz) : period_us(1000000UL / hz) {}

    // setup() の最後に呼ぶ。これが無いと prev_us=0 のまま起動するので、
    // 初回 ready() の dt_us が「起動からの経過時間」(数秒) になり、
    // その dt で PID や積分が一気に進んでしまう。
    void prime() { prev_us = micros(); }

    // 周期が来ていれば true を返し、dt_us を更新する。
    bool ready() {
        const uint32_t now = micros();
        if (now - prev_us < period_us) return false;
        dt_us   = now - prev_us;
        prev_us = now;
        return true;
    }

    float dt_s() const { return (float)dt_us * 1e-6f; }
};

// LED 点滅の位相。half_period_ms ごとに ON/OFF が反転する矩形波。
inline bool blinkOn(uint32_t now_ms, uint32_t half_period_ms) {
    return (now_ms / half_period_ms) % 2 == 0;
}

} // namespace Quad
