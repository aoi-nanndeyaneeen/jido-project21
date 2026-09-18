// ============================================================
//  LoopProfile.h  -  メインループの区間ごとの所要時間を数える (平均 / 最大)
// ============================================================
//  ★ 2026-09-17: RP2040 に移ってから実効ループが 630〜680Hz (Teensy は 991Hz) に
//    落ちている (LOG0042〜0045 の dt_us)。StallLog は 5ms を超えた回しか残さない
//    ので、「普段どの区間が 1ms の枠を食っているか」が分からなかった。
//    ここでは全ループを区間ごとに積算する。
//
//    win    : 直近 1 秒ぶん。1 秒ごとに last() へ写して画面 (S5Status) に出す
//    flight : アームからディスアームまで。ディスアームの瞬間に SafeLog で要約を出し、
//             lastFlight() に残す ('w' でも再表示)
//
//  書くのはコア0 (loop) だけ。コア1 (画面) は写しを読むだけ (途中の値が混ざっても
//  表示が一瞬ずれるだけなので排他はしない)。
// ============================================================
#pragma once
#include <Arduino.h>
#include "quad/SafeLog.h"

namespace S5 {
namespace LoopProfile {

// loop() の区間 (FcWatchdog の Section から NONE を除いたものと同じ並び)
enum Sec : uint8_t { IMU = 0, SBUS, FLOW, RANGE, GROUND, CONTROL, CONSOLE, SERVICE, TELEM, STATUS, N };

inline const char* secName(int s) {
    static const char* const K[N] = { "imu", "sbus", "flow", "range", "gnd", "ctrl",
                                      "key", "svc", "telem", "stat" };
    return (s >= 0 && s < N) ? K[s] : "?";
}

struct Acc {
    uint32_t sum_us[N] = {};    // 区間ごとの合計 [us] (70 分で桁あふれ。飛行 1 本なら十分)
    uint32_t max_us[N] = {};
    uint32_t work_sum_us = 0;   // 1 ループの処理時間の合計
    uint32_t work_max_us = 0;
    uint32_t period_max_us = 0; // ループ周期 (前回開始からの間隔) の最大
    uint32_t n = 0;             // ループ回数
    uint32_t over1 = 0, over2 = 0, over5 = 0;   // 処理時間が 1/2/5 ms を超えた回数
    uint32_t t0_ms = 0, t1_ms = 0;

    void add(const uint32_t sec[N], uint32_t work_us, uint32_t period_us, uint32_t now_ms) {
        if (n == 0) t0_ms = now_ms;
        t1_ms = now_ms;
        for (int i = 0; i < N; ++i) {
            sum_us[i] += sec[i];
            if (sec[i] > max_us[i]) max_us[i] = sec[i];
        }
        work_sum_us += work_us;
        if (work_us > work_max_us) work_max_us = work_us;
        if (period_us > period_max_us) period_max_us = period_us;
        if (work_us > 1000) over1++;
        if (work_us > 2000) over2++;
        if (work_us > 5000) over5++;
        n++;
    }
    float seconds() const { return (t1_ms - t0_ms) * 1e-3f; }
    float hz() const { return (seconds() > 0.0f) ? (float)n / seconds() : 0.0f; }
};

static Acc s_win, s_last, s_flight, s_last_flight;
static bool s_have_flight = false;

// 1 ループぶん。sec[] は区間ごとの所要時間 [us]。
inline void add(const uint32_t sec[N], uint32_t work_us, uint32_t period_us, bool armed) {
    const uint32_t now = millis();
    s_win.add(sec, work_us, period_us, now);
    if (armed) s_flight.add(sec, work_us, period_us, now);
    if (now - s_win.t0_ms >= 1000) {
        s_last = s_win;
        s_win = Acc{};
    }
}

// 表示 (Print は USB の Serial。コア1 の画面か、コア0 の 'w' キー処理から)
inline void print(Print& out, const Acc& a, const char* title) {
    if (a.n == 0) { out.printf("%s: (データなし)\n", title); return; }
    out.printf("%s: %.1fs  実効 %.0f Hz  処理 平均 %.2f / 最大 %.1f ms  周期最大 %.1f ms  "
               ">1ms %.1f%% >2ms %.1f%% >5ms %.2f%%\n",
               title, a.seconds(), a.hz(), a.work_sum_us / 1000.0f / a.n, a.work_max_us / 1000.0f,
               a.period_max_us / 1000.0f,
               100.0f * a.over1 / a.n, 100.0f * a.over2 / a.n, 100.0f * a.over5 / a.n);
    out.print("   平均/最大[us]");
    for (int i = 0; i < N; ++i)
        out.printf(" %s %lu/%lu", secName(i), (unsigned long)(a.sum_us[i] / a.n),
                   (unsigned long)a.max_us[i]);
    out.println();
}
inline const Acc& last()       { return s_last; }
inline const Acc& lastFlight() { return s_last_flight; }
inline bool haveFlight()       { return s_have_flight; }

// アーム/ディスアームのエッジ (コア0)。ディスアームで飛行 1 本ぶんの要約を出す。
inline void armEdge(bool armed) {
    if (armed) { s_flight = Acc{}; return; }
    s_last_flight = s_flight;
    s_have_flight = true;
    const Acc& a = s_last_flight;
    if (a.n == 0) return;
    Quad::SafeLog::logf("\n[ループ計測 この飛行] %.1fs 実効 %.0f Hz 処理 平均 %.2f / 最大 %.1f ms "
                        ">1ms %.1f%% >5ms %.2f%%\n",
                        a.seconds(), a.hz(), a.work_sum_us / 1000.0f / a.n,
                        a.work_max_us / 1000.0f, 100.0f * a.over1 / a.n, 100.0f * a.over5 / a.n);
    char line[200];
    int k = snprintf(line, sizeof(line), "   平均[us]");
    for (int i = 0; i < N && k < (int)sizeof(line); ++i)
        k += snprintf(line + k, sizeof(line) - k, " %s %lu", secName(i),
                      (unsigned long)(a.sum_us[i] / a.n));
    Quad::SafeLog::logf("%s\n", line);
    k = snprintf(line, sizeof(line), "   最大[us]");
    for (int i = 0; i < N && k < (int)sizeof(line); ++i)
        k += snprintf(line + k, sizeof(line) - k, " %s %lu", secName(i),
                      (unsigned long)a.max_us[i]);
    Quad::SafeLog::logf("%s\n", line);
}

} // namespace LoopProfile
} // namespace S5
