// ============================================================
//  Scheduler.h  -  周期実行のヘルパ (Ticker / Divider / blink)
// ============================================================
//  drone_s5 の loop() は 1000Hz のメインループを起点に、複数の周期を
//  重ねて回す。以前は Ticker・手書きの分周カウンタ・millis()/125 の 3 種類が
//  混在していたので、ここに 2 種類だけ用意して全部そろえる。
//
//    Ticker  : 時間基準。micros() で周期を測り、実測 dt も返す。
//              センサ読み出しやテレメトリのように「壁時計で何 Hz」が要るもの。
//    Divider : 回数基準。親ループ N 回に 1 回。角度ループ・ログ・GUIDED 翻訳の
//              ように「メインループに同期して間引く」もの。ジッタが出ない。
//
//  ★ 使い分けの目安: dt が要る/親と同期しなくてよい → Ticker、
//                    親ループのサブ周期 → Divider。
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

// 親ループ n 回に 1 回だけ true を返す。
struct Divider {
    int n;
    int count = 0;

    explicit Divider(int div) : n(div) {}

    bool tick() {
        if (++count < n) return false;
        count = 0;
        return true;
    }
    void reset() { count = 0; }
};

// LED 点滅の位相。half_period_ms ごとに ON/OFF が反転する矩形波。
inline bool blinkOn(uint32_t now_ms, uint32_t half_period_ms) {
    return (now_ms / half_period_ms) % 2 == 0;
}

} // namespace Quad
