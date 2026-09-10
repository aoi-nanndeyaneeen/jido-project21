// ============================================================
//  log_recorder / main.cpp  -  RP2040 を「SD ログ専用機」にする
// ============================================================
//  Teensy(FC) --UART 2Mbaud--> RP2040 --SPI0--> HW-125 --> microSD
//
//  【なぜ分けたか】
//    FC (Teensy4.0) では PMW3901 と HW-125 が SPI0 を共有していて共存できず、
//    「フローを使う飛行では SD ログを諦めて RAM ログ 8 秒」になっていた
//    (flight_controller/src/drone_s5.cpp の USE_FLOW / USE_SD)。
//    SD を丸ごとこちらへ引っ越すと、FC の SPI0 は PMW3901 専用になり、
//    フロー使用中でも飛行まるごとのログが録れる。
//
//  【2 コアの分担】 ここがこのスケッチの肝
//    core0 : UART を受けてフレームを検証し、リングへ積むだけ。SD に触らない。
//    core1 : リングから取り出して SD へ書くだけ。UART の受信に触らない。
//    SD カードは GC で 100ms 以上黙ることがある。同じコアでやると、その間の
//    UART が確実に溢れる。分けておくと core0 は止まらず、停止はリングが
//    吸収する (128KB = 500Hz なら約 2 秒ぶん)。
//
//  【出来上がる BIN】
//    SdLog.h が作っていたものと完全に同じ = 32B ヘッダ + Rec の連続。
//    変換はこれまでどおり PC で flight_controller/scripts/bin2csv.py。
//
//  【配線 (XIAO RP2040)】
//     D7 / GP1  RX  <---- Teensy TX17 (Serial4)
//     D6 / GP0  TX  ----> Teensy RX16          ※状態返信用。省略可
//     GND       <--------> Teensy GND          ※必須
//     D8 / GP2  SCK ----> HW-125 SCK
//     D10/ GP3  MOSI----> HW-125 MOSI
//     D9 / GP4  MISO<---- HW-125 MISO
//     D2 / GP28 CS  ----> HW-125 CS
//     5V        ----> HW-125 VCC (モジュール上でレギュレータ + レベル変換)
//     GND       ----> HW-125 GND
//    ★ RP2040 の電源は FC と分ける。SD 書き込みの突入電流で FC を
//      巻き込まないため。GND だけ共通にすること。
//
//  【LED (XIAO RP2040 のオンボード、いずれも負論理)】
//     緑 = 記録中     青 = 待機 (SD OK)     赤 = SD NG / ロスト発生
// ============================================================
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

#include "quad/LogLinkProto.h"   // ★ flight_controller 側の実体を include

namespace P = LogLinkProto;

// ---- ピン ----------------------------------------------------------------
constexpr uint8_t PIN_UART_TX = 0;    // D6
constexpr uint8_t PIN_UART_RX = 1;    // D7
constexpr uint8_t PIN_SPI_SCK = 2;    // D8
constexpr uint8_t PIN_SPI_TX  = 3;    // D10 (MOSI)
constexpr uint8_t PIN_SPI_RX  = 4;    // D9  (MISO)
constexpr uint8_t PIN_SD_CS   = 28;   // D2

constexpr uint8_t PIN_LED_RED   = 17;  // 負論理
constexpr uint8_t PIN_LED_GREEN = 16;
constexpr uint8_t PIN_LED_BLUE  = 25;

// ---- チューニング定数 ----------------------------------------------------
constexpr uint32_t LINK_BAUD  = 2000000;   // LogLink::BAUD と一致させること
constexpr size_t   UART_FIFO  = 4096;      // Philhower core の受信バッファ
// リングは 2 の冪。128KB = 122B/rec × 500Hz なら約 2.1 秒ぶんのカード停止を吸収。
// RP2040 の SRAM は 264KB なので、書き込みバッファと合わせても余裕がある。
constexpr uint32_t RING_BYTES = 128u * 1024u;
constexpr uint32_t RING_MASK  = RING_BYTES - 1;
static_assert((RING_BYTES & RING_MASK) == 0, "RING_BYTES は 2 の冪にすること");

constexpr size_t   WBUF_BYTES = 8192;              // SD へ渡す前の整列バッファ
constexpr size_t   WRITE_CHUNK = 4096;             // 1 回の write サイズ (512 の倍数)
constexpr uint32_t SD_SCK_MHZ_VAL = 20;            // 手配線の HW-125。安定したら 25
constexpr uint64_t PREALLOC   = 16ull * 1024 * 1024;   // ≒300 秒ぶん
constexpr uint32_t SYNC_MS    = 2000;              // FAT 保存間隔 (電源断保険)
constexpr uint32_t STAT_MS    = 500;               // FC への状態返信 (2Hz)

// ============================================================
//  コア間リング (SPSC)
//    core0 だけが g_head を進め、core1 だけが g_tail を進める。
//    インデックスは 32bit のフリーランニング。使用量は head - tail の
//    引き算で出る (符号なしのラップがそのまま正しく効く)。こうすると
//    「満杯と空が同じ値」問題が起きない。
//    アクセスは __atomic_* で acquire/release を明示する。RP2040 は
//    2 コアが同じ SRAM を見るので、これが無いと最適化で壊れる。
// ============================================================
static uint8_t  g_ring[RING_BYTES];
static volatile uint32_t g_head = 0;
static volatile uint32_t g_tail = 0;

static inline uint32_t ringUsed() {
    const uint32_t h = __atomic_load_n(&g_head, __ATOMIC_ACQUIRE);
    const uint32_t t = __atomic_load_n(&g_tail, __ATOMIC_ACQUIRE);
    return h - t;
}

// ---- 統計 (core1 が書き、core0 が読んで FC へ返す) ------------------------
//  ★ 複数フィールドをまたぐ一貫性までは保証しない (診断用なので許容)。
struct Stats {
    volatile bool     sd_ok     = false;
    volatile bool     recording = false;
    volatile uint16_t file_idx  = 0;
    volatile uint32_t bytes     = 0;
    // ★ ロス系カウンタは「誰が書くか」でコアを分けてある。同じ変数を両コアから
    //   ++ すると read-modify-write が競合して数が狂う (診断値そのものが
    //   信用できなくなる)。合算は読む側 (core0) でやる。
    volatile uint32_t drop_ring = 0;   // core0: リング溢れで捨てたレコード
    volatile uint32_t drop_wbuf = 0;   // core1: 整列バッファに入らず捨てたレコード
    volatile uint32_t crc_err   = 0;   // CRC 不一致フレーム
    volatile uint32_t seq_gap   = 0;   // seq 飛び = UART 取りこぼし
    volatile uint32_t orphan    = 0;   // START 前に来た REC
    volatile uint32_t worst_w_us = 0;
    volatile uint32_t peak_ring = 0;
};
static Stats g_st;

// ============================================================
//  §1  core0 : UART 受信 + フレーム検証 + リングへ push
// ============================================================
namespace Rx {

// 完成したフレームを「丸ごと」リングへ積む。途中まで積むことはしない
// (core1 は「リングの中身は必ず完全なフレーム」を前提にパースするため)。
static bool push(const uint8_t* f, uint32_t n) {
    const uint32_t h = __atomic_load_n(&g_head, __ATOMIC_ACQUIRE);
    const uint32_t t = __atomic_load_n(&g_tail, __ATOMIC_ACQUIRE);
    if ((h - t) + n > RING_BYTES) return false;      // 溢れ

    const uint32_t idx   = h & RING_MASK;
    const uint32_t first = min(n, RING_BYTES - idx);
    memcpy(g_ring + idx, f, first);
    if (n > first) memcpy(g_ring, f + first, n - first);

    __atomic_store_n(&g_head, h + n, __ATOMIC_RELEASE);
    const uint32_t used = (h + n) - t;
    if (used > g_st.peak_ring) g_st.peak_ring = used;
    return true;
}

// バイト単位のステートマシン。SOF2 + CRC8 で再同期する。
//  UART は 1 バイト落ちると後ろが全部ズレるので、CRC が合わないフレームは
//  捨てて次の SOF を探しにいく。seq を見ていれば「何レコード失ったか」も分かる。
static uint8_t s_st = 0, s_type = 0, s_len = 0, s_seq = 0, s_idx = 0;
static uint8_t s_pay[P::MAX_PAYLOAD];
static bool    s_seq_init = false;
static uint8_t s_seq_prev = 0;

static void onFrame() {
    // seq の連続性チェック (欠落レコード数を数える)
    if (s_seq_init) {
        const uint8_t gap = (uint8_t)(s_seq - s_seq_prev - 1);
        if (gap) g_st.seq_gap += gap;
    }
    s_seq_prev = s_seq;
    s_seq_init = true;

    uint8_t f[P::MAX_FRAME];
    const size_t n = P::buildFrame(f, s_type, s_seq, s_pay, s_len);
    if (n && !push(f, (uint32_t)n) && s_type == P::T_REC) g_st.drop_ring++;
}

static void feed(const uint8_t* p, size_t n) {
    while (n--) {
        const uint8_t c = *p++;
        switch (s_st) {
            case 0: if (c == P::SOF1) s_st = 1; break;
            case 1: s_st = (c == P::SOF2) ? 2 : ((c == P::SOF1) ? 1 : 0); break;
            case 2: s_type = c; s_st = 3; break;
            case 3: s_len  = c; s_st = 4; break;
            case 4: s_seq  = c; s_idx = 0; s_st = (s_len == 0) ? 6 : 5; break;
            case 5: s_pay[s_idx++] = c; if (s_idx >= s_len) s_st = 6; break;
            case 6: {
                uint8_t crc = P::crc8(&s_type, 1);
                crc = P::crc8(&s_len, 1, crc);
                crc = P::crc8(&s_seq, 1, crc);
                crc = P::crc8(s_pay, s_len, crc);
                if (crc == c) onFrame();
                else          g_st.crc_err++;
                s_st = 0;
                break;
            }
        }
    }
}

// FC へ状態を返す (2Hz)。FC 側は LogLink::brief()/status() に出す。
static void sendStat() {
    P::Stat s = {};
    s.flags     = (uint8_t)((g_st.sd_ok ? P::STAT_SD_OK : 0) |
                            (g_st.recording ? P::STAT_REC : 0));
    s.ring_pct  = (uint8_t)((uint64_t)ringUsed() * 100 / RING_BYTES);
    s.file_idx  = g_st.file_idx;
    s.kbytes    = g_st.bytes / 1024;
    s.drop_rec  = g_st.drop_ring + g_st.drop_wbuf;
    s.crc_err   = g_st.crc_err;
    s.seq_gap   = g_st.seq_gap;
    s.worst_w_us = g_st.worst_w_us;

    uint8_t f[P::MAX_FRAME];
    static uint8_t seq = 0;
    const size_t n = P::buildFrame(f, P::T_STAT, seq++, &s, sizeof(s));
    // 送信は 34B/回 × 2Hz。availableForWrite() を見て、詰まっていたら諦める
    // (受信を止めてまで返す価値のあるデータではない)。
    if ((size_t)Serial1.availableForWrite() >= n) Serial1.write(f, n);
}

} // namespace Rx

// ============================================================
//  §2  core1 : リング -> SD
// ============================================================
namespace Sd {

static SdFs   s_sd;
static FsFile s_file;
static bool   s_ok        = false;
static bool   s_recording = false;
static uint32_t s_next_idx = 0;
static uint32_t s_t0       = 0;      // 今開いているファイルの識別子 (BIN ヘッダの t0_ms)
static uint32_t s_bytes    = 0;
static uint32_t s_last_rec_ms  = 0;
static uint32_t s_last_sync_ms = 0;

static uint8_t s_wbuf[WBUF_BYTES];
static size_t  s_wlen = 0;

static void scanNextIndex() {
    char name[16];
    for (uint32_t i = 0; i < 10000; ++i) {
        snprintf(name, sizeof(name), "LOG%04lu.BIN", (unsigned long)i);
        if (!s_sd.exists(name)) { s_next_idx = i; return; }
    }
    s_next_idx = 0;   // 全部埋まっていたら 0 から上書き
}

static bool init() {
    SPI.setSCK(PIN_SPI_SCK);
    SPI.setTX(PIN_SPI_TX);
    SPI.setRX(PIN_SPI_RX);
    // このバスには SD しか繋がっていないので DEDICATED_SPI にできる。
    // SdFat がマルチブロック書き込みを使えるようになり、FC 側 (SHARED_SPI)
    // より格段に速い。ここが「SD を別 MCU に出した」ことの副次的な利得。
    s_ok = s_sd.begin(SdSpiConfig(PIN_SD_CS, DEDICATED_SPI,
                                  SD_SCK_MHZ(SD_SCK_MHZ_VAL), &SPI));
    if (s_ok) scanNextIndex();
    g_st.sd_ok    = s_ok;
    g_st.file_idx = (uint16_t)s_next_idx;
    return s_ok;
}

// 実際に SD へ流す。512 の倍数だけ書き、端数は次に持ち越す
// (セクタ境界で書くとカード内部の read-modify-write が起きない)。
static void flushAligned(bool force) {
    if (s_wlen == 0) return;
    size_t n = force ? s_wlen : (s_wlen / 512) * 512;
    if (!force && n > WRITE_CHUNK) n = WRITE_CHUNK;
    if (n == 0) return;

    const uint32_t t0 = micros();
    const size_t w = s_file.write(s_wbuf, n);
    const uint32_t dt = micros() - t0;
    if (dt > g_st.worst_w_us) g_st.worst_w_us = dt;

    s_bytes += w;
    g_st.bytes = s_bytes;
    if (w < s_wlen) memmove(s_wbuf, s_wbuf + w, s_wlen - w);
    s_wlen -= w;
}

static void closeFile() {
    if (!s_recording) return;
    flushAligned(true);              // 端数も含めて全部出す
    s_file.truncate(s_bytes);        // preAllocate の未使用ぶんを解放
    s_file.sync();
    s_file.close();
    s_recording   = false;
    g_st.recording = false;
    Serial.printf("[SD] close LOG%04lu.BIN  %lu bytes\n",
                  (unsigned long)(s_next_idx - 1), (unsigned long)s_bytes);
}

static void openFile(const uint8_t* bin_hdr) {
    if (!s_ok) return;
    if (s_recording) closeFile();

    char name[16];
    snprintf(name, sizeof(name), "LOG%04lu.BIN", (unsigned long)s_next_idx);
    if (!s_file.open(name, O_WRONLY | O_CREAT | O_TRUNC)) {
        Serial.printf("[SD] open 失敗: %s\n", name);
        return;
    }
    s_file.preAllocate(PREALLOC);    // 連続領域 → 書き込みレイテンシが安定

    s_wlen  = 0;
    s_bytes = 0;
    s_file.write(bin_hdr, P::BIN_HDR_LEN);   // SdLog と同一の 32B ヘッダ
    s_bytes = P::BIN_HDR_LEN;

    memcpy(&s_t0, bin_hdr + P::BIN_HDR_T0_OFS, 4);
    s_recording    = true;
    g_st.recording = true;
    g_st.bytes     = s_bytes;
    g_st.file_idx  = (uint16_t)s_next_idx;
    s_last_rec_ms  = millis();
    s_last_sync_ms = millis();
    s_next_idx++;
    Serial.printf("[SD] open %s (t0=%lu)\n", name, (unsigned long)s_t0);
}

// リングから 1 フレーム取り出して処理する。core0 が完全なフレームしか
// 積まないので、ここでは CRC を見直す必要がない。
static inline uint8_t peek(uint32_t tail, uint32_t i) {
    return g_ring[(tail + i) & RING_MASK];
}

//  ★ 1 回の呼び出しで処理するフレーム数に上限を設ける。
//    リングが空になるまで回す作りだと、FC が 500Hz で流し続けている間は
//    ここから戻れず、service() の sync() とタイムアウト close が
//    永久に実行されない (= 電源断保険が効かない)。
//    500Hz に対して 64 は十分な追い上げ余力がある (loop1 は空回りが速い)。
constexpr int DRAIN_MAX_FRAMES = 64;

static void drainRing() {
    for (int i = 0; i < DRAIN_MAX_FRAMES; ++i) {
        const uint32_t used = ringUsed();
        if (used < P::OVERHEAD) return;

        const uint32_t tail = __atomic_load_n(&g_tail, __ATOMIC_ACQUIRE);
        const uint8_t  type = peek(tail, 2);
        const uint8_t  len  = peek(tail, 3);
        const uint32_t flen = P::OVERHEAD + len;
        if (used < flen) return;                 // まだ全部来ていない

        // payload を取り出す
        static uint8_t pay[P::MAX_PAYLOAD];
        const uint32_t idx   = (tail + P::HDR_LEN) & RING_MASK;
        const uint32_t first = min((uint32_t)len, RING_BYTES - idx);
        memcpy(pay, g_ring + idx, first);
        if (len > first) memcpy(pay + first, g_ring, len - first);

        __atomic_store_n(&g_tail, tail + flen, __ATOMIC_RELEASE);

        switch (type) {
            case P::T_START: {
                if (len != P::BIN_HDR_LEN) break;
                uint32_t t0; memcpy(&t0, pay + P::BIN_HDR_T0_OFS, 4);
                // FC は同じヘッダを 1Hz で再送してくる (ロガーが後から
                // 起動しても拾えるように)。同じ t0 なら既に開いている
                // ファイルの続き = 無視する。違えば別フライト。
                if (s_recording && t0 == s_t0) break;
                openFile(pay);
                break;
            }
            case P::T_REC: {
                if (!s_recording) { g_st.orphan++; break; }
                s_last_rec_ms = millis();
                if (s_wlen + len > WBUF_BYTES) flushAligned(false);
                if (s_wlen + len > WBUF_BYTES) { g_st.drop_wbuf++; break; } // まだ入らない
                memcpy(s_wbuf + s_wlen, pay, len);
                s_wlen += len;
                break;
            }
            case P::T_STOP:
                closeFile();
                break;
            default:
                break;
        }

        if (s_wlen >= WRITE_CHUNK) flushAligned(false);
    }
}

static void service() {
    if (!s_ok) {
        // カードが後から挿されることもあるので 2 秒ごとに再挑戦する。
        static uint32_t last = 0;
        if (millis() - last > 2000) { last = millis(); init(); }
        return;
    }
    drainRing();
    if (!s_recording) return;

    // 電源断保険。FAT/dir を定期保存する。
    if (millis() - s_last_sync_ms >= SYNC_MS) {
        s_last_sync_ms = millis();
        flushAligned(false);
        s_file.sync();
    }
    // STOP が化けて届かなかったときの保険。REC が途切れたら勝手に閉じる。
    // これが無いと preAllocate した 16MB が open のまま残り、次回起動時に
    // ゴミファイルになる。
    if (millis() - s_last_rec_ms >= P::IDLE_CLOSE_MS) {
        Serial.println("[SD] REC 途切れ -> タイムアウトで close");
        closeFile();
    }
}

} // namespace Sd

// ============================================================
//  §3  LED / USB コンソール
// ============================================================
static void led(bool r, bool g, bool b) {   // 負論理
    digitalWrite(PIN_LED_RED,   r ? LOW : HIGH);
    digitalWrite(PIN_LED_GREEN, g ? LOW : HIGH);
    digitalWrite(PIN_LED_BLUE,  b ? LOW : HIGH);
}

static void printStatus() {
    Serial.println("---- log_recorder ----");
    Serial.printf("  SD        : %s   次/現ファイル = LOG%04u.BIN\n",
                  g_st.sd_ok ? "OK" : "NG (CS=GP28 / VCC=5V / FAT32 / 配線)",
                  (unsigned)g_st.file_idx);
    Serial.printf("  記録      : %s   %lu bytes\n",
                  g_st.recording ? "REC" : "idle", (unsigned long)g_st.bytes);
    Serial.printf("  リング    : %lu / %lu B (peak %lu = %lu%%)\n",
                  (unsigned long)ringUsed(), (unsigned long)RING_BYTES,
                  (unsigned long)g_st.peak_ring,
                  (unsigned long)((uint64_t)g_st.peak_ring * 100 / RING_BYTES));
    Serial.printf("  ロス      : drop=%lu (ring %lu / wbuf %lu)  crc_err=%lu  "
                  "seq_gap=%lu  orphan=%lu\n",
                  (unsigned long)(g_st.drop_ring + g_st.drop_wbuf),
                  (unsigned long)g_st.drop_ring, (unsigned long)g_st.drop_wbuf,
                  (unsigned long)g_st.crc_err,
                  (unsigned long)g_st.seq_gap, (unsigned long)g_st.orphan);
    Serial.printf("  最悪write : %lu us\n", (unsigned long)g_st.worst_w_us);
    if (g_st.crc_err || g_st.seq_gap)
        Serial.println("  ★ crc_err/seq_gap が増える = UART が化けている。"
                       "配線を短く / GND を確実に / 両側の BAUD を 1000000 へ");
    if (g_st.peak_ring > RING_BYTES / 2)
        Serial.println("  ★ リングが半分を超えた = SD が遅い。"
                       "カードを速いものに変える / SD_SCK_MHZ_VAL を上げる");
}

// ============================================================
//  §4  core0  (UART 受信専用)
// ============================================================
void setup() {
    pinMode(PIN_LED_RED, OUTPUT); pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_BLUE, OUTPUT);
    led(true, true, true);        // 起動中は白

    Serial.begin(115200);         // USB (デバッグ用)

    Serial1.setRX(PIN_UART_RX);
    Serial1.setTX(PIN_UART_TX);
    Serial1.setFIFOSize(UART_FIFO);   // ★ begin() より前に呼ぶこと
    Serial1.begin(LINK_BAUD);

    led(false, false, true);      // 青 = 待機
}

void loop() {
    // まとめて読む。1 バイトずつ read() すると 2Mbaud (200kB/s) では
    // 呼び出しオーバヘッドが効いてくる。
    uint8_t buf[256];
    int n;
    while ((n = Serial1.available()) > 0) {
        if (n > (int)sizeof(buf)) n = sizeof(buf);
        n = Serial1.readBytes(buf, n);
        if (n > 0) Rx::feed(buf, (size_t)n);
    }

    static uint32_t last_stat = 0;
    if (millis() - last_stat >= STAT_MS) {
        last_stat = millis();
        Rx::sendStat();
    }

    // USB コンソール
    if (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == 's') printStatus();
        if (c == 'r') { g_st.drop_ring = g_st.drop_wbuf = 0;
                        g_st.crc_err = g_st.seq_gap = 0;
                        g_st.orphan = 0; g_st.peak_ring = 0; g_st.worst_w_us = 0;
                        Serial.println("統計をリセットしました"); }
    }
}

// ============================================================
//  §5  core1  (SD 書き込み専用)
// ============================================================
void setup1() {
    // core0 が SPI ピンを触らないよう、SD の初期化はここに閉じる。
    delay(50);                    // core0 の Serial 初期化と competing しないように
    Sd::init();
}

void loop1() {
    Sd::service();

    // LED: 記録中=緑 / SD NG=赤 / ロスト有り=赤点滅 / 待機=青
    static uint32_t last_led = 0;
    if (millis() - last_led >= 100) {
        last_led = millis();
        const bool lost = (g_st.drop_ring || g_st.drop_wbuf ||
                           g_st.crc_err   || g_st.seq_gap);
        if (!g_st.sd_ok)          led(true, false, false);
        else if (g_st.recording)  led(lost && (millis() / 200) % 2, true, false);
        else                      led(lost, false, true);
    }
}
