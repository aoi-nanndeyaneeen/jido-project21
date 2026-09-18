// ============================================================
//  FcWatchdog.h  -  メインループが止まったらチップごとリセットする安全装置
//                   + 「どの処理の最中に止まったか」をリセット越しに残す
// ============================================================
//  ★ 2026-09-17: XIAO RP2040 で、ループが 0.2Hz 程度まで落ちる (= ほぼ停止) 事象が
//    時々起きる。モーターは RP2040 のハードウェア PWM (analogWrite 400Hz) なので、
//    CPU が止まっても最後のデューティを出し続け、プロポを切っても THR_CUT を入れても
//    スロットルが入ったままになる (止める処理自体がループの中にあるため)。
//
//    ハードウェアのウォッチドッグは CPU と独立したタイマーなので、CPU が止まって
//    いても働く。アーム中に WDT_TIMEOUT_MS ループが回らなければチップをリセット
//    → GPIO が入力 (Hi-Z) に戻って PWM が消える → ESC は信号喪失でモーターを止める。
//    空中なら落ちるが、スロットル固定のまま飛び続けるより良い。
//
//    ★ ESC の信号線に 10kΩ 程度のプルダウンを付けると、リセット中の Hi-Z でノイズを
//      パルスと誤認しない (推奨)。
//
//  有効なのはアーム中だけ。ディスアーム中は 'k' の再キャリブレーションや 'p' メニュー
//  のように数秒ブロックする地上操作があるので止めておく。
//
//  ★ どこで止まったか: loop() が区間の境目で mark(区間) を呼ぶ。値はウォッチドッグの
//    scratch[0..1] レジスタに書く (リセットでは消えない。scratch[4..7] は SDK が使う)。
//    起動時に watchdog が原因のリセットなら、その区間を表示する。
//
//  Teensy ビルドでは全部何もしない (Teensy 側の WDOG は未対応)。
// ============================================================
#pragma once
#include <Arduino.h>
#include "quad/QuadConfig.h"
#ifdef ARDUINO_ARCH_RP2040
#include <hardware/watchdog.h>
#endif

namespace S5 {
namespace FcWatchdog {

// loop() の区間。mark() に渡す。表示名は sectionName()。
enum Section : uint32_t {
    SEC_NONE = 0, SEC_IMU, SEC_SBUS, SEC_FLOW, SEC_RANGE, SEC_GROUND, SEC_CONTROL,
    SEC_CONSOLE, SEC_SERVICE, SEC_TELEM, SEC_STATUS,
    // 2026-09-18: 「自動ミッションが終わるとループが止まる」の切り分け用に細分化
    SEC_FAILSAFE, SEC_FLOWHOLD, SEC_ALTHOLD, SEC_GUIDED, SEC_LEDS, SEC_LOGFILES,
    SEC_FLIGHTLOG, SEC_LOGLINK, SEC_BLEACT, SEC_ARMCHECK,
    SEC__COUNT
};

inline const char* sectionName(uint32_t s) {
    static const char* const N[] = {
        "-", "IMU 読み (I2C)", "SBUS", "フロー (LogLink)", "測距 (I2C)",
        "地上局受信/GUIDED", "姿勢制御/ESC", "USB キー入力", "ログ/LED/LogLink 送信",
        "テレメトリ", "デバッグ画面 (USB)",
        "フェイルセーフ判定", "フロー位置ホールド (PosHold)", "高度ホールド (AltHold)",
        "GUIDED 更新 (周回/直進)", "LED", "ログファイル開閉", "ログ 1 行作成",
        "LogLink UART 送受信", "BLE メンテ指令", "アーム判定"
    };
    return (s < SEC__COUNT) ? N[s] : "?";
}

#ifdef ARDUINO_ARCH_RP2040
constexpr uint32_t MAGIC = 0x5A5A0000u;   // scratch[0] の上位で「自分が書いた値」を識別

static bool s_enabled = false;
static bool s_rebooted = false;          // 今回の起動は watchdog によるリセットか
static uint32_t s_last_section = 0;      // そのとき止まっていた区間
static uint32_t s_last_ms = 0;           // そのときの millis()
static uint32_t s_last_state = 0;        // そのときの状態 (markState。0 = 不明)
static uint8_t  s_reset_reason = 0;      // rp2040.getResetReason() (RP2040Support.h resetReason_t)
static uint32_t s_now_ms = 0;            // feed() で 1 回だけ取る millis() (mark() が 20 回/ループ使う)

// setup() の最初の方で 1 回。前回の記録を読んで、区間表示をクリアする。
inline void begin() {
    s_reset_reason = (uint8_t)rp2040.getResetReason();
    s_rebooted = watchdog_enable_caused_reboot();
    const uint32_t v = watchdog_hw->scratch[0];
    if (s_rebooted && (v & 0xFFFF0000u) == MAGIC) {
        s_last_section = v & 0xFFFFu;
        s_last_ms      = watchdog_hw->scratch[1];
        const uint32_t st = watchdog_hw->scratch[2];
        if ((st & 0xFFFF0000u) == MAGIC) s_last_state = st & 0xFFFFu;
    }
    watchdog_hw->scratch[0] = MAGIC | SEC_NONE;
    watchdog_hw->scratch[2] = 0;
}

// ループ内の区間ごとに呼ぶ (1 ループに約 20 回)。millis() は feed() で取った値を使う
// (64bit 割り算を毎回しない。区間の時刻は ms 単位でよい)。
inline void mark(Section s) {
    watchdog_hw->scratch[0] = MAGIC | (uint32_t)s;
    watchdog_hw->scratch[1] = s_now_ms;
}
// 止まったときの機体の状態 (モード / GUIDED の段階 / フェイルセーフ / アーム)。1 ループに 1 回。
inline void markState(uint8_t mode, uint8_t guided_phase, uint8_t fs, bool armed) {
    watchdog_hw->scratch[2] = MAGIC | ((uint32_t)armed << 12) | ((uint32_t)fs << 8)
                            | ((uint32_t)mode << 4) | (uint32_t)(guided_phase & 0x0F);
}

// アーム/ディスアームのエッジで呼ぶ
inline void arm() {
    if (Quad::WDT_TIMEOUT_MS == 0) return;
    watchdog_enable(Quad::WDT_TIMEOUT_MS, /*pause_on_debug=*/true);
    s_enabled = true;
}
inline void disarm() {
    if (!s_enabled) return;
    watchdog_disable();
    s_enabled = false;
}
// loop() の先頭で毎回
inline void feed() { s_now_ms = millis(); if (s_enabled) watchdog_update(); }

inline bool rebooted()      { return s_rebooted; }
inline uint32_t lastSection() { return s_last_section; }
inline uint32_t lastMs()      { return s_last_ms; }
// 今回の起動のリセット原因 (0 UNKNOWN / 1 PWRON / 2 RUN_PIN / 3 SOFT / 4 WDT / 5 DEBUG / 6 GLITCH / 7 BROWNOUT)
inline uint8_t resetReason() { return s_reset_reason; }

// 電源投入以外で起動した = 飛行中の異常の疑い。起動後に BLE でヘッダを届ける (drone_s5.cpp)
inline bool bootReportWanted() {
    return s_rebooted || s_reset_reason == 4 || s_reset_reason == 6 || s_reset_reason == 7;
}

// ログヘッダ後半 16B (LogLinkProto BIN_HDR_EXTRA_*)。scripts/boot_report.py が読む。
//   [0..1] 'W','D'  [2] 版=1  [3] リセット原因  [4] 1=ウォッチドッグの記録あり
//   [5] 止まった区間  [6..7] 状態 (u16: armed<<12 | fs<<8 | mode<<4 | guided_phase)
//   [8..11] 止まった時の起動からの ms (u32)
inline void fillHeaderExtra(uint8_t* out) {
    memset(out, 0, 16);
    out[0] = 'W'; out[1] = 'D'; out[2] = 1;
    out[3] = s_reset_reason;
    out[4] = s_rebooted ? 1 : 0;
    out[5] = (uint8_t)s_last_section;
    const uint16_t st = (uint16_t)s_last_state;
    memcpy(out + 6, &st, 2);
    memcpy(out + 8, &s_last_ms, 4);
}

// 止まったときの状態。armed / fs / mode / guided_phase (GuidedPhase の値)
inline bool lastState(bool& armed, uint8_t& fs, uint8_t& mode, uint8_t& gp) {
    if (s_last_state == 0) return false;
    armed = (s_last_state >> 12) & 1;  fs = (s_last_state >> 8) & 0x0F;
    mode  = (s_last_state >> 4) & 0x0F; gp = s_last_state & 0x0F;
    return true;
}
#else
inline void begin() {}
inline void mark(Section) {}
inline void markState(uint8_t, uint8_t, uint8_t, bool) {}
inline uint8_t resetReason() { return 0; }
inline bool bootReportWanted() { return false; }
inline void fillHeaderExtra(uint8_t* out) { memset(out, 0, 16); }
inline bool lastState(bool&, uint8_t&, uint8_t&, uint8_t&) { return false; }
inline void arm() {}
inline void disarm() {}
inline void feed() {}
inline bool rebooted()        { return false; }
inline uint32_t lastSection() { return 0; }
inline uint32_t lastMs()      { return 0; }
#endif

} // namespace FcWatchdog
} // namespace S5
