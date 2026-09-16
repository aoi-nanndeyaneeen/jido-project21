// ============================================================
//  GroundConfig.h  -  地上局 (XIAO) の配線と周期
// ============================================================
//  【地上局の周期一覧】 ★ loop() の骨組みそのもの
//
//      全速      IM920 (Serial1) から 1 文字ずつ → 行が出来たら TelemetryStore::handleLine
//      全速      USB (Serial) から 1 文字ずつ  → 1 文字キー / "CMD,..." 行
//      毎ループ  Uplink::service   … mailbox の指令を無線へ (蓋 CMD_MIN_GAP_MS)
//      1Hz       STAT 行 (PC の診断画面用)
//      1Hz       人間向け状態表示 (CSV 出力 OFF のときだけ)
//      2Hz       「テレメトリ待機中」の心拍 (CSV 出力 ON で受信が 2 秒無いとき)
//
//  上り (PC → 機体) のレートを決めるのはここの CMD_MIN_GAP_MS だけ。
//  下り (機体 → PC) は flight_controller の S5::TELEM_TX_HZ。IM920 は半二重なので
//  2 つセットで UART 占有率 55% 程度を上限に見ること (protocol/S5Cmd.h 冒頭)。
// ============================================================
#pragma once
#include <Arduino.h>

namespace Ground {

// ---- 配線 ------------------------------------------------------------
//  XIAO RP2040 : シルク TX = D6 = GP0、RX = D7 = GP1
//  XIAO ESP32C3: シルク TX = D6 = GPIO21、RX = D7 = GPIO20  (番号が違うだけで向きは同じ)
//  ★ IM920 の電源は 3V3 から。5V ピンから取ると USB 列挙に失敗して書き込めなくなる。
//  ★ マルチホップ設定 (ENHP/DSHP) は両機で一致していないと一切通信できない。
#if defined(ARDUINO_ARCH_ESP32)
constexpr int PIN_XIAO_TX = 21;  // D6 -> IM920 RXD
constexpr int PIN_XIAO_RX = 20;  // D7 <- IM920 TXD
#elif defined(ARDUINO_ARCH_RP2040)
constexpr int PIN_XIAO_TX = 0;   // D6 -> IM920 RXD
constexpr int PIN_XIAO_RX = 1;   // D7 <- IM920 TXD
#else
constexpr int PIN_XIAO_TX = -1;
constexpr int PIN_XIAO_RX = -1;
#endif

constexpr unsigned long IM_BAUD  = 19200;
constexpr unsigned long USB_BAUD = 115200;

// ---- 上りコマンド (PC -> 無線) ------------------------------------------
//  以前は CMD_TX_INTERVAL_MS=200ms の「格子」で送っていた。PC 側も 5Hz だったので
//  同じ周期の門が直列に 2 枚あることになり、平均 100ms / 最悪 200ms の純粋な待ちと、
//  位相がずれて「1 指令まるごと上書きされて電波に出ないまま消える」回が出ていた。
//  格子をやめ、帯域の蓋とキープアライブに分けた (2026-09-16)。
//
//  上りを無線へ出してよい最短間隔 [ms] = 上りの帯域上限 (8Hz)。
//    CmdFrame 22+4=26 byte = "TXDA "+52桁+CRLF = 59 文字 = 19200bps で 30.7ms。8Hz で占有 25%。
constexpr uint32_t CMD_MIN_GAP_MS   = 125;
//  PC が黙っているあいだ、最後の指令を送り直す間隔 [ms]。機体の cmd_fresh
//  (GUIDED_STALE_HOLD_MS=1000) を維持するための保険。PC が一瞬詰まっただけで
//  水平指令が 0 に落ちるのを防ぐ。
constexpr uint32_t CMD_KEEPALIVE_MS = 200;
//  PC からこの時間なにも来なければ送信を止める (機体はフェイルセーフ: 1s ホールド → 4s 着陸)。
//  ★ 地上局が落ちたのに最後の指令を送り続けるほうが危ない。
constexpr uint32_t CMD_PC_TIMEOUT_MS = 1500;

// ---- PC 向け出力 -------------------------------------------------------
constexpr uint32_t STAT_PERIOD_MS      = 1000;   // STAT 行
constexpr uint32_t STATUS_PERIOD_MS    = 1000;   // 人間向け表示 (CSV OFF 時)
constexpr uint32_t HEARTBEAT_PERIOD_MS = 2000;   // 「テレメトリ待機中」(CSV ON で受信断)
constexpr uint32_t LINK_LOST_MS        = 1000;   // 表示上「LOST」にする無受信時間

} // namespace Ground
