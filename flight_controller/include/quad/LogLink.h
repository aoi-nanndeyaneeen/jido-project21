// ============================================================
//  LogLink.h  -  ログを UART で RP2040 ロガーへ流す (SdLog.h の差し替え)
// ============================================================
//  quad/SdLog.h と「同じ形の API」にしてある。呼び出し側 (drone_s5.cpp) は
//    begin / startFile / push / service / stopFile / brief / status
//  を SdLog:: から LogLink:: に読み替えるだけでよい。
//
//   push(rec, n)  … フレーム化してリングへ memcpy するだけ。UART も SD も
//                   触らない。リングが一杯ならそのレコードを捨てて dropped++。
//   service()     … 毎ループ。availableForWrite() のぶんだけ UART へ流す。
//                   1 回あたり MAX_TX で頭打ち。ブロックしない。
//                   ついでにロガーからの T_STAT を受けて状態を持つ。
//
//  【SdLog との決定的な違い】
//    SdLog は「SD が GC で数十ms 黙る」ことに耐える必要があったが、こちらの
//    相手は UART なので停止しない。定速で捌けるぶん、リングは小さくてよい
//    (SdLog は 32KB、こちらは 8KB = 約 130ms ぶん)。
//    ★ DMAMEM (RAM2) は RamLog が ~480KB + SdLog が 32KB でほぼ満杯なので、
//      このリングは通常 RAM (RAM1) に置く。8KB なら余裕がある。
//
//  【帯域】
//    Rec 116B + フレーム 6B = 122B/レコード。500Hz で 61 kB/s = 610 kbps。
//    8N1 なので BAUD は最低 700k 必要。既定 2Mbaud で 3 倍の余裕を取る。
//
//  【配線】 (既定は Teensy4.0 Serial4: TX=17 / RX=16)
//     Teensy TX17 ---- RP2040(XIAO) D7/GP1 (RX)
//     Teensy RX16 ---- RP2040(XIAO) D6/GP0 (TX)   ※状態表示 + BLE経由の
//                                                    デバッグ指令(T_ACT) +
//                                                    操縦指令(T_CMD)用。
//                                                    ★ drone_s5.cpp の GROUND_LINK
//                                                    が BLE のときは必須
//     GND         ---- GND                        ※必須
//    電源は分ける (SD 書き込みの突入で FC を巻き込まないため)。GND だけ共通。
//
// ============================================================
//  【2026-09-17 Teensy 故障時の移行方針 (実装済み。env: drone_s5_rp2040)】
// ============================================================
//  Teensy 4.0 が故障した。手持ちは XIAO RP2040 x1 / XIAO ESP32C3 x2。
//  買い足さずに 2 枚で組む場合の結論を残す。
//
//  ★ 結論: FC = XIAO RP2040 / C3 = BLE 中継 + フロー読み の 2 枚。
//    鍵は「PMW3901 を FC から C3 側へ移す」こと。C3 とは元々この UART が
//    張ってあるので、線も UART も増えない。
//
//  --- ピン収支 (XIAO は両方とも 11 パッド) -------------------------------
//                          RP2040(FC)   ESP32C3(BLE+フロー)
//      モーター PWM x4          4            -
//      I2C (MPU/BMP/VL53L1X)    2            -
//      SBUS RX                  1            -
//      SPI (PMW3901)            -            4
//      相互 UART TX/RX          2            2
//      BLE                      -            内蔵 (0)
//      StatusLed                オンボード NeoPixel (0)
//                             ----         ----
//                              9/11         6/11
//
//    フローの SPI 4 本が FC から消えるのが効く (13 本 -> 9 本で 11 に収まる)。
//    ★ 訂正: RP2040 の UART1 RX が使える GPIO {5,9,21,25} は XIAO のパッドに
//      1 本も出ていない。ハード UART で RX を取れるのは UART0 (D6/D7) だけ
//      なので、LogLink (2Mbaud 双方向) を UART0、SBUS (100k 反転 RX のみ) を
//      SerialPIO に置く (src/trainer.cpp と同じ。実績あり)。ピンは quad/BoardPins.h。
//
//  --- プロトコル拡張 -----------------------------------------------------
//    T_FLOW = 0x84 を 1 つ足すだけ (0x8X = ロガー->FC の既存規約に乗る)。
//    payload は PMW3901 の生デルタ + タイムスタンプ。
//    ★ de-rotation はジャイロとの時刻同期が要るので必ず FC 側に残すこと。
//      C3 は「生デルタを投げるだけ」。そうすれば FlowObs / PosHold は無改造で、
//      OpticalFlow の読み取り部だけが「LogLink から最新値を取る」に変わる。
//
//  --- なぜモーター出力のほうを別基板に割らないか -------------------------
//    モーターを割ると 1000Hz のレートループの内側に遅延とジッタが入る
//    (1 サンプル = 1ms、交差 ~30Hz で約 11 度の位相損失)。さらにリンク断時に
//    「最後の PWM を保持したまま」になるため、ウォッチドッグとフェイルセーフを
//    新規に書く必要がある (飛行実績ゼロのコードが安全系に入る)。
//    フロー側なら 100Hz 入力 + 0.3Hz 帯の位置ループなので位相損失は約 1 度で済み、
//    かつリンク断は既存の flowAlive() -> MODE_ALTHOLD 縮退がそのまま効く
//    (drone_s5.cpp の mode 決定部。新しい安全コードが要らない)。
//
//  --- 移植時の作業 -------------------------------------------------------
//    1. ★ 1000Hz が回るかの実測検証。ここが唯一の本当の山。
//       RP2040 は 133MHz Cortex-M0+ で FPU 無し (ESP32C3 も FPU 無し)。
//       1000Hz で走るのは IMU 読み + レート PID + ミキサーのみ (角度 200Hz /
//       フロー 100Hz は間引き済み) で、うち IMU の I2C 約 360us はバス律速。
//       概算では 600〜700us/1000us に収まるが未検証。dt_us と StallLog で測る。
//       ダメなら 500Hz へ落とすが、それは PID 再調整を意味する。
//       ★ 結果 (2026-09-17〜18 の実飛行 LOG0056〜0064): 回らない。実効 630〜700Hz
//         (dt 平均 1.3〜1.6ms, p99 3.4ms)。PID は実測 dt だったので無事だったが、
//         Madgwick が固定 1ms 積分で角度が 0.6〜0.7 倍に縮んでいた (sensor/IMU.h)。
//         対処は「周期を上げる」ではなく「全部を実測 dt にする」(2026-09-18。
//         QuadPID / Scheduler / drone_s5.cpp)。周期を上げたければ platformio.ini の
//         f_cpu / -O2 のメモ。
//    2. RamLog を 8 秒 -> 2 秒へ (FlightLog.h の SECONDS)。
//       RP2040 の SRAM は 264KB。実測で RamLog と SdLog リングを除いた
//       下限は約 55KB。2 秒 (116KB) で実ビルド RAM 140KB (53%)。
//       ※ 0.23Hz の発振調査には 125Hz の BLE ストリームで足りるので、
//         RamLog が縮むこと自体は当面の解析に影響しない。
//    3. begin() の addMemoryForWrite() -> arduino-pico の setFIFOSize()
//       (下の begin() がテンプレートなのは Teensy の HardwareSerialIMXRT
//        都合。RP2040 に移すならここも書き換える)
//    4. Actuators.cpp の analogWriteResolution/Frequency(400Hz) を
//       RP2040 の PWM スライス設定へ
//    5. SBUS の反転は arduino-pico の setInvertRX(true) (begin より前に呼ぶ)。
//       外付けインバータ不要。
//    6. C3 側に SPI 読み + T_FLOW 送出を追加 (100 行程度)
//
//  --- 比較 ---------------------------------------------------------------
//    Teensy 4.0 買い直し (~5000 円) なら移植ゼロで即復帰できる。
//    上の移植は数日かかるので、発振調査を止めてまでやるかは要判断。
//    なお XIAO 系は ESP32S3 も 11 パッドなので、3 枚目を買うなら
//    RP2040-Zero (~500 円, GPIO 20 本) のほうがピン問題ごと消える。
// ============================================================
#pragma once
#include <Arduino.h>
#include "quad/LogLinkProto.h"

namespace LogLink {

namespace P = LogLinkProto;

// ---- チューニング定数 --------------------------------------------------
constexpr uint32_t BAUD        = 2000000;      // 610kbps 必要に対して 3 倍の余裕
constexpr size_t   RING_BYTES  = 8u * 1024u;   // 約 130ms ぶん
constexpr size_t   MAX_TX      = 512u;         // service() 1 回で流す上限 [byte]
constexpr size_t   TXBUF_BYTES = 2048u;        // HardwareSerial に足す送信バッファ
constexpr uint32_t HDR_RESEND_MS   = 1000;     // START ヘッダの再送間隔
constexpr uint32_t STAT_TIMEOUT_MS = 2000;     // これだけ T_STAT が来なければ STALE

// ---- 内部状態 (include するのは drone_s5.cpp だけ。SdLog.h と同じ流儀) ----
static uint8_t  s_ring[RING_BYTES];
#ifndef ARDUINO_ARCH_RP2040
static uint8_t  s_txbuf[TXBUF_BYTES];   // Teensy: addMemoryForWrite 用
#endif
static HardwareSerial* s_port = nullptr;

static bool     s_ok        = false;
static bool     s_recording = false;
static uint16_t s_rec_size  = 0;
static uint8_t  s_rec_ver   = 0;
static uint16_t s_rate_hz   = 0;

static size_t   s_head = 0, s_tail = 0, s_count = 0;
static uint8_t  s_seq  = 0;
static uint8_t  s_bin_hdr[P::BIN_HDR_LEN];     // 再送用に保持
static uint8_t  s_hdr_extra[P::BIN_HDR_EXTRA_LEN] = {};   // ヘッダ後半 (setHeaderExtra)
static uint32_t s_hdr_sent_ms = 0;

static uint32_t s_sent_rec  = 0;   // 送ったレコード数
static uint32_t s_dropped   = 0;   // リング溢れで捨てたレコード数
static size_t   s_peak_ring = 0;   // リング使用量のピーク

// ロガーからの状態 (RX を配線していないときは s_stat_ms が 0 のまま)
static P::Stat  s_stat = {};
static uint32_t s_stat_ms = 0;
static uint32_t s_stat_n  = 0;

// ロガー(BLE Write 中継)からの単発メンテナンス指令。pollAction() で
// 「前回取り出した後に新しく届いたか」だけを見る (取りこぼし前提の
// 単純なエッジ検出。重複排除そのものは drone_s5.cpp 側の action_seq 比較で行う)。
static P::ActReq s_act_req   = {};
static uint32_t  s_act_rx_n  = 0;   // 受信した ActReq の総数
static uint32_t  s_act_seen_n = 0;  // pollAction() 済みの数

// 地上局リンク (S5::GROUND_LINK == BLE) の上りコマンド。最新 1 個だけ持つ
// mailbox。機体側 (S5C::Rx) が使うのも常に最新値だけなので、1 回の
// pollStat() の中で複数届いたら古いほうは上書きでよい (seq の飛びとして
// S5C::Rx が数える)。
static uint8_t  s_cmd_buf[P::CMD_PAYLOAD];
static uint32_t s_cmd_rx_n   = 0;   // 受信した T_CMD の総数
static uint32_t s_cmd_seen_n = 0;   // pollCmd() 済みの数
static uint32_t s_cmd_badlen = 0;   // 長さが CmdFrame と合わなかった T_CMD
static uint32_t s_telem_drop = 0;   // T_TELEM をリングに積めなかった回数

// ロガー側に載せた PMW3901 の生カウント (T_FLOW)。累積和で来るので、
// pollFlow() は「前回返した時点の sum との差」を返す (取りこぼしても次で吸収)。
constexpr uint32_t FLOW_TIMEOUT_MS = 200;   // これだけ T_FLOW が来なければ STALE
static P::FlowFrame s_flow      = {};
static uint32_t     s_flow_ms   = 0;
static uint32_t     s_flow_rx_n = 0;
static int32_t      s_flow_last_sum_dx = 0, s_flow_last_sum_dy = 0;
static bool         s_flow_primed = false;   // 最初の 1 フレームは差分を取らず基準にする

// ---- RX 診断カウンタ (配線の切り分け用) ----
//  rx_bytes==0      : 線に何も来ていない → GND 未共通 / D6→RX16 断線 / RX16 不良
//  rx_bytes>0 crc>0 : 線は生きているが化けている → BAUD が速すぎ (1000000 へ) / 配線を短く
//  rx_bytes>0 crc==0 stat_n==0 : 枠は取れるが中身違い → proto ずれ (両方ビルドし直す)
static uint32_t s_rx_bytes   = 0;
static uint32_t s_rx_crcfail = 0;

// ---- 小物 ----------------------------------------------------------------
inline void resetRing() { s_head = s_tail = s_count = 0; }

inline bool statFresh() {
    return s_stat_ms != 0 && (millis() - s_stat_ms) < STAT_TIMEOUT_MS;
}
// ロガー側の SD が初期化できているか。statFresh() が false のときは
// 「分からない」ので false を返す (SelfTest はそれを NOSIG として扱う)。
inline bool loggerSdOk() {
    return statFresh() && (s_stat.flags & P::STAT_SD_OK);
}
inline bool flowFresh() {
    return s_flow_ms != 0 && (millis() - s_flow_ms) < FLOW_TIMEOUT_MS;
}
// ロガー側の PMW3901 が begin() に応答したか (T_STAT の flags)。
inline bool loggerFlowOk() {
    return statFresh() && (s_stat.flags & P::STAT_FLOW_OK);
}
inline uint32_t flowRxCount() { return s_flow_rx_n; }
inline uint8_t  flowSqual()   { return s_flow.squal; }

// リングへ n バイト積む。入らなければ「1 バイトも積まず」false。
//  ★ 途中まで積むとフレームが千切れてロガー側が同期を失うので、
//    必ず全部入るときだけ書く。
inline bool ringPut(const uint8_t* p, size_t n) {
    if (s_count + n > RING_BYTES) return false;
    size_t first = RING_BYTES - s_head;
    if (first > n) first = n;
    memcpy(s_ring + s_head, p, first);
    if (n > first) memcpy(s_ring, p + first, n - first);
    s_head  = (s_head + n) % RING_BYTES;
    s_count += n;
    if (s_count > s_peak_ring) s_peak_ring = s_count;
    return true;
}

// フレームを組み立ててリングへ。制御ループから呼ばれる想定なのでブロックしない。
inline bool sendFrame(uint8_t type, const void* payload, size_t len) {
    uint8_t f[P::MAX_FRAME];
    const size_t n = P::buildFrame(f, type, s_seq, payload, len);
    if (n == 0) return false;
    if (!ringPut(f, n)) return false;
    s_seq++;               // 積めたときだけ進める (seq 飛び = 本当の UART 落ち)
    return true;
}

// ---- ロガーからの受信 (T_STAT / T_ACT。小さな受信ステートマシン) -----------
inline void pollStat() {
    static uint8_t st = 0, type = 0, len = 0, seq = 0, idx = 0;
    static uint8_t buf[P::MAX_PAYLOAD];
    if (!s_port) return;

    int guard = 512;    // 1 回の service() で読む上限。制御ループを守る
    while (s_port->available() > 0 && guard-- > 0) {
        const uint8_t c = (uint8_t)s_port->read();
        s_rx_bytes++;
        switch (st) {
            case 0: if (c == P::SOF1) st = 1; break;
            case 1: st = (c == P::SOF2) ? 2 : ((c == P::SOF1) ? 1 : 0); break;
            case 2: type = c; st = 3; break;
            case 3: len  = c; st = 4; break;
            case 4: seq  = c; idx = 0; st = (len == 0) ? 6 : 5; break;
            case 5: buf[idx++] = c; if (idx >= len) st = 6; break;
            case 6: {
                uint8_t crc = P::crc8(&type, 1);
                crc = P::crc8(&len, 1, crc);
                crc = P::crc8(&seq, 1, crc);
                crc = P::crc8(buf, len, crc);
                if (crc == c && type == P::T_STAT && len == sizeof(P::Stat)) {
                    memcpy(&s_stat, buf, sizeof(s_stat));
                    s_stat_ms = millis();
                    s_stat_n++;
                } else if (crc == c && type == P::T_ACT && len == sizeof(P::ActReq)) {
                    memcpy(&s_act_req, buf, sizeof(s_act_req));
                    s_act_rx_n++;
                } else if (crc == c && type == P::T_FLOW && len == sizeof(P::FlowFrame)) {
                    memcpy(&s_flow, buf, sizeof(s_flow));
                    s_flow_ms = millis();
                    s_flow_rx_n++;
                } else if (crc == c && type == P::T_CMD) {
                    if (len == P::CMD_PAYLOAD) {
                        memcpy(s_cmd_buf, buf, P::CMD_PAYLOAD);
                        s_cmd_rx_n++;
                    } else {
                        s_cmd_badlen++;   // S5Cmd.h が機体とロガーでずれている
                    }
                } else if (crc != c) {
                    s_rx_crcfail++;
                }
                st = 0;
                break;
            }
        }
    }
}

// BLE 経由の単発メンテナンス指令を取り出す。前回 pollAction() 済みの
// ものより新しいものが届いていれば true (取り出したら「消費済み」)。
// ★ drone_s5.cpp 側でさらに action_seq の変化を見て重複排除すること
//   (これ自体は「新着があったか」しか見ていない)。
inline bool pollAction(P::ActReq& out) {
    if (s_act_seen_n >= s_act_rx_n) return false;
    s_act_seen_n = s_act_rx_n;
    out = s_act_req;
    return true;
}

// 地上局リンク (BLE) の上りコマンドを取り出す。前回から新着があれば true。
//  out は P::CMD_PAYLOAD バイト以上。中身の検証 (magic/ver/seq) は
//  S5C::Rx::acceptRaw() がやる (IM920 経由と同じカウンタに載せるため)。
inline bool pollCmd(uint8_t* out) {
    if (s_cmd_seen_n >= s_cmd_rx_n) return false;
    s_cmd_seen_n = s_cmd_rx_n;
    memcpy(out, s_cmd_buf, P::CMD_PAYLOAD);
    return true;
}
// ロガー経由のフロー生カウント。前回 pollFlow() からの累積差分を dx/dy に返す。
//  新着が無ければ dx=dy=0 で false (呼び出し側は 0,0 を updateFrom に渡す)。
//  最初の 1 フレームは基準にするだけで差分を返さない (起動からの累積を
//  「今の移動」と誤認しないため)。
inline bool pollFlow(int16_t& dx, int16_t& dy, uint8_t& squal) {
    static uint32_t seen_n = 0;
    squal = s_flow.squal;
    if (seen_n == s_flow_rx_n) { dx = dy = 0; return false; }
    seen_n = s_flow_rx_n;
    if (!s_flow_primed) {
        s_flow_last_sum_dx = s_flow.sum_dx;
        s_flow_last_sum_dy = s_flow.sum_dy;
        s_flow_primed = true;
        dx = dy = 0;
        return false;
    }
    const int32_t ddx = s_flow.sum_dx - s_flow_last_sum_dx;
    const int32_t ddy = s_flow.sum_dy - s_flow_last_sum_dy;
    s_flow_last_sum_dx = s_flow.sum_dx;
    s_flow_last_sum_dy = s_flow.sum_dy;
    // 100Hz 同士なので通常 1 サンプルぶん (数十カウント)。int16 に収まる。
    dx = (int16_t)constrain(ddx, -32768, 32767);
    dy = (int16_t)constrain(ddy, -32768, 32767);
    return true;
}

inline uint32_t cmdRxCount()  { return s_cmd_rx_n; }
inline uint32_t cmdBadLen()   { return s_cmd_badlen; }
inline uint32_t telemDrops()  { return s_telem_drop; }

// 地上局リンク (BLE) の下りテレメトリ。S5T の各フレームを連結したバイト列を
// 1 フレームで送る。記録中かどうかに関係なく流す (非アーム中も地上局は
// 機体の状態を見たい)。積めなければ false (呼び出し側が F_TX_DROP を立てる)。
inline bool sendTelem(const uint8_t* p, size_t n) {
    if (!s_ok || n == 0 || n > P::TELEM_MAX_PAYLOAD) return false;
    if (sendFrame(P::T_TELEM, p, n)) return true;
    s_telem_drop++;
    return false;
}

// モード表示 LED の色をロガーへ送る (S5::STATUS_LED_VIA_LINK)。毎ループ呼んでよい。
//  色が変わった瞬間と、変わらなくても 500ms ごと (キープアライブ) に 1 フレーム。
//  ロガーは 1 秒来なければ消灯する。
inline void serviceLed(uint8_t rgb) {
    static uint8_t  last_rgb = 0xFF;
    static uint32_t last_ms  = 0;
    if (!s_ok) return;
    const uint32_t now = millis();
    if (rgb == last_rgb && (now - last_ms) < 500) return;
    P::LedFrame f{rgb};
    if (sendFrame(P::T_LED, &f, sizeof(f))) { last_rgb = rgb; last_ms = now; }
}

// 上記の実行結果を FC -> ロガー -> BLE へ送る。既存の送信リング
// (T_START/T_REC/T_STOP と同じ) に積むだけなので service() が捌く。
inline bool sendAck(uint8_t action, uint8_t action_seq, uint8_t result,
                    uint8_t imu_ok = 0, uint8_t i2c_found = 0) {
    P::ActAck a{action, action_seq, result, imu_ok, i2c_found};
    return sendFrame(P::T_ACT_ACK, &a, sizeof(a));
}

// ============================================================
//  begin  -  setup() で 1 回。ポートとレコード形式を渡す。
//    ★ 戻り値は「UART を開けたか」であって「ロガーが生きているか」ではない。
//      生死は statFresh() (= T_STAT が来ているか) を見ること。
// ============================================================
//    ★ port をテンプレートで受けるのは addMemoryForWrite() が抽象基底の
//      HardwareSerial ではなく HardwareSerialIMXRT 側にあるため。
//      保持は基底ポインタでよい (write/available/flush は基底で virtual)。
template <class PortT>
inline bool begin(PortT& port, uint16_t rec_size, uint8_t rec_ver,
                  uint16_t rate_hz, uint32_t baud = BAUD) {
    s_port     = &port;
    s_rec_size = rec_size;
    s_rec_ver  = rec_ver;
    s_rate_hz  = rate_hz;
    // 既定の送信バッファは数十バイトしかない。500Hz × 122B を service() で
    // 捌ききるために 2KB 足す (足さないと availableForWrite() が常に小さく、
    // リング側に溜まって drop する)。
#ifdef ARDUINO_ARCH_RP2040
    // arduino-pico: begin() より前に setFIFOSize()。バッファは core が確保する。
    port.setFIFOSize(TXBUF_BYTES);
    port.begin(baud);
#else
    port.begin(baud);
    port.addMemoryForWrite(s_txbuf, sizeof(s_txbuf));
#endif
    resetRing();
    s_ok = true;
    return s_ok;
}

inline bool ok()        { return s_ok; }
inline bool recording() { return s_recording; }

// ============================================================
//  startFile  -  アームした瞬間に。ロガーへ「新しい BIN を開け」と伝える。
//    ★ SdLog::startFile() と違い open も preAllocate もしないので
//      ブロックしない (アーム直後の数十ms スパイクが消える)。
// ============================================================
// ヘッダ後半 16B に載せる内容 (次の startFile() から。リセット原因など)
inline void setHeaderExtra(const uint8_t* b, size_t n) {
    memset(s_hdr_extra, 0, sizeof(s_hdr_extra));
    memcpy(s_hdr_extra, b, (n < sizeof(s_hdr_extra)) ? n : sizeof(s_hdr_extra));
}

inline void startFile() {
    if (!s_ok) return;

    // SdLog と同一の 32B ヘッダ。scripts/bin2csv.py がこれを見る。
    memset(s_bin_hdr, 0, sizeof(s_bin_hdr));
    memcpy(s_bin_hdr, "S5LOG", 5);              // [5] = 0
    s_bin_hdr[6]  = 1;                          // FMT_VER (SdLog::FMT_VER と同じ)
    s_bin_hdr[7]  = s_rec_ver;
    s_bin_hdr[8]  = (uint8_t)(s_rec_size & 0xFF);
    s_bin_hdr[9]  = (uint8_t)(s_rec_size >> 8);
    s_bin_hdr[10] = (uint8_t)(s_rate_hz & 0xFF);
    s_bin_hdr[11] = (uint8_t)(s_rate_hz >> 8);
    const uint32_t t0 = millis();               // ロガー側のファイル識別子も兼ねる
    memcpy(s_bin_hdr + P::BIN_HDR_T0_OFS, &t0, 4);
    memcpy(s_bin_hdr + P::BIN_HDR_EXTRA_OFS, s_hdr_extra, sizeof(s_hdr_extra));

    resetRing();
    s_dropped   = 0;
    s_sent_rec  = 0;
    s_peak_ring = 0;
    s_recording = true;
    sendFrame(P::T_START, s_bin_hdr, sizeof(s_bin_hdr));
    s_hdr_sent_ms = millis();
}

// ============================================================
//  push  -  1 レコード。フレーム化してリングへ積むだけ。
// ============================================================
inline void push(const void* rec, size_t n) {
    if (!s_recording || n == 0) return;
    if (sendFrame(P::T_REC, rec, n)) s_sent_rec++;
    else                             s_dropped++;
}

// ============================================================
//  service  -  毎ループ。UART の空きぶんだけ流す。
// ============================================================
inline void service() {
    if (!s_ok || !s_port) return;

    // ロガーが後から起動した場合に備えてヘッダを 1Hz で再送する。
    // 同じ t0_ms なので、既に記録中のロガーはこれを無視する。
    if (s_recording && (millis() - s_hdr_sent_ms) >= HDR_RESEND_MS) {
        s_hdr_sent_ms = millis();
        sendFrame(P::T_START, s_bin_hdr, sizeof(s_bin_hdr));
    }

    size_t budget = MAX_TX;
    while (s_count > 0 && budget > 0) {
        const int avail = s_port->availableForWrite();
        if (avail <= 0) break;                    // 送信バッファ満杯。次のループで
        size_t chunk = s_count;
        if (chunk > budget)        chunk = budget;
        if (chunk > (size_t)avail) chunk = (size_t)avail;
        size_t first = RING_BYTES - s_tail;
        if (first > chunk) first = chunk;
        s_port->write(s_ring + s_tail, first);    // 空きを確認済み = 詰まらない
        s_tail  = (s_tail + first) % RING_BYTES;
        s_count -= first;
        budget  -= first;
    }

    pollStat();
}

// ============================================================
//  stopFile  -  ディスアームで。リングを吐き切って STOP を送る。
//    ここは地上なので、SdLog::stopFile() と同じくブロックしてよい。
// ============================================================
inline void stopFile() {
    if (!s_recording) return;
    sendFrame(P::T_STOP, nullptr, 0);
    s_recording = false;

    // 残りを吐き切る。2Mbaud なら 8KB でも ~40ms。上限を切って無限待ちを防ぐ。
    const uint32_t t_limit = millis() + 200;
    while (s_count > 0 && (int32_t)(millis() - t_limit) < 0) {
        const int avail = s_port->availableForWrite();
        if (avail <= 0) continue;
        size_t chunk = s_count;
        if (chunk > (size_t)avail) chunk = (size_t)avail;
        size_t first = RING_BYTES - s_tail;
        if (first > chunk) first = chunk;
        s_port->write(s_ring + s_tail, first);
        s_tail  = (s_tail + first) % RING_BYTES;
        s_count -= first;
    }
    s_port->flush();
}

// printStatus() のライブ画面用の 1 行。SdLog::brief() と同じ位置に出る。
inline void brief(Print& out) {
    if (!s_ok) { out.println("LOGLINK: NG"); return; }
    if (!statFresh()) {
        out.printf("LOGLINK: %s  tx=%lu drop=%lu  [ロガー応答なし]\n",
                   s_recording ? "REC" : "idle",
                   (unsigned long)s_sent_rec, (unsigned long)s_dropped);
        return;
    }
    out.printf("LOGLINK: %s  LOG%04u.BIN  %lu KB  txdrop=%lu  logdrop=%lu  "
               "crc=%lu gap=%lu  worstW=%lu us\n",
               (s_stat.flags & P::STAT_REC) ? "REC" : "idle",
               (unsigned)s_stat.file_idx, (unsigned long)s_stat.kbytes,
               (unsigned long)s_dropped, (unsigned long)s_stat.drop_rec,
               (unsigned long)s_stat.crc_err, (unsigned long)s_stat.seq_gap,
               (unsigned long)s_stat.worst_w_us);
}

// ============================================================
//  status  -  シリアル 's' で状態表示
// ============================================================
inline void status() {
    Serial.printf("LOGLINK: UART %lu baud  記録中=%d  送信rec=%lu  "
                  "送信drop=%lu  リングpeak=%lu/%lu B\n",
                  (unsigned long)BAUD, s_recording ? 1 : 0,
                  (unsigned long)s_sent_rec, (unsigned long)s_dropped,
                  (unsigned long)s_peak_ring, (unsigned long)RING_BYTES);
    if (!statFresh()) {
        Serial.printf("    ロガー: 応答なし (有効STAT %lu 件 / RX生バイト %lu / CRC不一致 %lu)\n",
                      (unsigned long)s_stat_n, (unsigned long)s_rx_bytes,
                      (unsigned long)s_rx_crcfail);
        if (s_rx_bytes == 0)
            Serial.println("    → 線に何も来ていない。GND 未共通 / ロガーTX→FC RX 断線 / "
                           "FC 側 RX ピン不良 を疑う");
        else if (s_rx_crcfail > 0)
            Serial.printf("    → 線は生きているが化けている。両側 BAUD を 1000000 へ "
                          "(今 %lu) / 配線を短く / GND を太く\n", (unsigned long)BAUD);
        else
            Serial.println("    → バイトは来るが STAT にならない。proto ずれ。"
                           "FC と log_recorder を両方ビルドし直す");
        Serial.println("    ※ RX を配線していない片方向運用なら「RX生バイト 0」で正常");
        return;
    }
    Serial.printf("    ロガー: SD=%s  記録中=%d  現/次ファイル=LOG%04u.BIN  書込=%lu KB\n",
                  (s_stat.flags & P::STAT_SD_OK) ? "OK" : "NG",
                  (s_stat.flags & P::STAT_REC) ? 1 : 0,
                  (unsigned)s_stat.file_idx, (unsigned long)s_stat.kbytes);
    Serial.printf("    フロー(T_FLOW): %s  受信 %lu  squal=%u  sum=(%ld,%ld)  PMW3901@ロガー=%s\n",
                  flowFresh() ? "fresh" : "stale",
                  (unsigned long)s_flow_rx_n, (unsigned)s_flow.squal,
                  (long)s_flow.sum_dx, (long)s_flow.sum_dy,
                  loggerFlowOk() ? "OK" : "NG");
    Serial.printf("    ロガー: drop(rec)=%lu  crc_err=%lu  seq_gap=%lu  "
                  "最悪write=%lu us  リング=%u%%\n",
                  (unsigned long)s_stat.drop_rec, (unsigned long)s_stat.crc_err,
                  (unsigned long)s_stat.seq_gap, (unsigned long)s_stat.worst_w_us,
                  (unsigned)s_stat.ring_pct);
    if (s_stat.crc_err || s_stat.seq_gap)
        Serial.println("    ★ crc_err/seq_gap が増える = UART が化けている。"
                       "配線長を詰める / GND を太くする / BAUD を 1000000 に落とす");
}

} // namespace LogLink
