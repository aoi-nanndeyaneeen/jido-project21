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
//     Teensy RX16 ---- RP2040(XIAO) D6/GP0 (TX)   ※状態表示用。省いても動く
//     GND         ---- GND                        ※必須
//    電源は分ける (SD 書き込みの突入で FC を巻き込まないため)。GND だけ共通。
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
static uint8_t  s_txbuf[TXBUF_BYTES];
static HardwareSerial* s_port = nullptr;

static bool     s_ok        = false;
static bool     s_recording = false;
static uint16_t s_rec_size  = 0;
static uint8_t  s_rec_ver   = 0;
static uint16_t s_rate_hz   = 0;

static size_t   s_head = 0, s_tail = 0, s_count = 0;
static uint8_t  s_seq  = 0;
static uint8_t  s_bin_hdr[P::BIN_HDR_LEN];     // 再送用に保持
static uint32_t s_hdr_sent_ms = 0;

static uint32_t s_sent_rec  = 0;   // 送ったレコード数
static uint32_t s_dropped   = 0;   // リング溢れで捨てたレコード数
static size_t   s_peak_ring = 0;   // リング使用量のピーク

// ロガーからの状態 (RX を配線していないときは s_stat_ms が 0 のまま)
static P::Stat  s_stat = {};
static uint32_t s_stat_ms = 0;
static uint32_t s_stat_n  = 0;

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

// ---- ロガーからの T_STAT 受信 (小さな受信ステートマシン) -------------------
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
                } else if (crc != c) {
                    s_rx_crcfail++;
                }
                st = 0;
                break;
            }
        }
    }
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
    port.begin(baud);
    // 既定の送信バッファは数十バイトしかない。500Hz × 122B を service() で
    // 捌ききるために 2KB 足す (足さないと availableForWrite() が常に小さく、
    // リング側に溜まって drop する)。
    port.addMemoryForWrite(s_txbuf, sizeof(s_txbuf));
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
