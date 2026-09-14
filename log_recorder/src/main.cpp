// ============================================================
//  log_recorder / main.cpp  -  XIAO ESP32C3 を「BLE ログ中継機」にする
// ============================================================
//  Teensy(FC) --UART 2Mbaud--> XIAO ESP32C3 --BLE Notify--> 受信側(スマホ/PC)
//
//  【経緯】
//    元々は SD (HW-125) へ書く専用機だったが、SD カードが壊れたため、
//    SD/SPI 周りを全部やめて BLE で常時送信する方式に置き換えた。
//    ボードも RP2040/ESP32C3 両対応をやめ、内蔵 BLE を持つ ESP32C3 専用にした。
//
//  【役割分担】
//    rx役 : UART を受けてフレームを検証し、リングへ積むだけ。BLE に触らない。
//    ble役: リングから取り出して BLE Notify で送るだけ。UART の受信に触らない。
//    ESP32C3 はシングルコアなので FreeRTOS の 2 タスクに役割分担させている
//    (rx = loop()、ble = 専用タスク)。BLE の送信待ち・切断でも rx 側は
//    止まらず、そのぶんはリングが吸収する (128KB ≒ 全レート換算で約 2 秒ぶん)。
//
//  【BLE の帯域制約 ★ここが要】
//    実測できる Notify のスループットはコネクション間隔次第で
//    だいたい 20〜80KB/s が上限 (理論値はもっと出るが安定運用の目安として)。
//    一方 FC からの T_REC は 500Hz × 122B/rec ≒ 61KB/s あり、フルレートを
//    そのまま Notify すると確実に破綻する。そのため BLE_REC_DECIM で
//    REC を間引いて送る (既定は 1/4 = 実質 125Hz ≒ 15KB/s。値は要調整)。
//    T_START/T_STOP は間引かずそのまま転送する (頻度が低いので問題ない)。
//    間引き後も溜まる分はリングが吸収する。BLE が未接続/輻輳中はフレームを
//    ring に残したまま送信を待つので、それでも溢れて初めて drop_ring で数える
//    (ring から取り出した後で送信に失敗して黙って捨てる、ということはしない)。
//
//  【送るフレーム形式】
//    UART と全く同じ LogLinkProto のバイト列 (SOF1 SOF2 type len seq payload crc8)
//    をそのまま Notify に乗せる。受信側は既存の LogLinkProto::crc8()/フレーム
//    パーサをそのまま流用できる。
//
//  【配線】
//     D7  RX  <---- Teensy TX17 (Serial4)
//     D6  TX  ----> Teensy RX16          ※状態返信用。省略可
//     GND     <--------> Teensy GND      ※必須
//    ★ XIAO の電源は FC と分ける。GND だけ共通にすること。
//
//  【状態確認】
//    USB シリアル (115200) の 's' コマンドで状態表示、'r' で統計リセット。
//    ESP32C3 にはユーザー制御可能なオンボードLEDが無いため LED 表示は無い。
// ============================================================
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "quad/LogLinkProto.h"   // ★ flight_controller 側の実体を include

namespace P = LogLinkProto;

// ---- ピン (XIAO ESP32C3) ----------------------------------------------
constexpr uint8_t PIN_UART_TX = 21;   // D6
constexpr uint8_t PIN_UART_RX = 20;   // D7

// ---- チューニング定数 ----------------------------------------------------
constexpr uint32_t LINK_BAUD  = 2000000;   // LogLink::BAUD と一致させること
constexpr size_t   UART_FIFO  = 4096;      // 受信バッファ
// リングは 2 の冪。128KB は全レート (61kB/s) 換算で約 2.1 秒ぶんの
// BLE 側停止・輻輳・切断を吸収する猶予。
constexpr uint32_t RING_BYTES = 128u * 1024u;
constexpr uint32_t RING_MASK  = RING_BYTES - 1;
static_assert((RING_BYTES & RING_MASK) == 0, "RING_BYTES は 2 の冪にすること");

constexpr uint32_t STAT_MS = 500;   // FC への状態返信 (2Hz)

// T_REC をこの分の1だけ BLE へ転送する (帯域対策。3で割り切れなくてよい)。
// 1 なら間引きなし (非推奨: 500Hz フルレートは BLE の帯域を確実に超える)。
constexpr uint8_t BLE_REC_DECIM = 4;   // 500Hz -> 実質 125Hz ≒ 15kB/s

// BLE 輻輳対策の下限送信間隔。これより速く notify() を連打すると
// コントローラ側の送信キューが溢れて notify が無言で落ちることがある。
constexpr uint32_t BLE_MIN_TX_INTERVAL_MS = 5;   // 上限 200 notify/s 程度に抑える

static const char* BLE_DEVICE_NAME    = "S5-LogBLE";
static const char* BLE_SERVICE_UUID   = "d5913036-2d8a-41ee-85b9-4e361aa5c8a7";
static const char* BLE_CHAR_UUID      = "d5913037-2d8a-41ee-85b9-4e361aa5c8a7";

// ============================================================
//  コア間 (タスク間) リング (SPSC)
//    rx タスク (loop()) だけが g_head を進め、ble タスクだけが g_tail を進める。
//    インデックスは 32bit のフリーランニング。使用量は head - tail の
//    引き算で出る (符号なしのラップがそのまま正しく効く)。
// ============================================================
static uint8_t  g_ring[RING_BYTES];
static volatile uint32_t g_head = 0;
static volatile uint32_t g_tail = 0;

static inline uint32_t ringUsed() {
    const uint32_t h = __atomic_load_n(&g_head, __ATOMIC_ACQUIRE);
    const uint32_t t = __atomic_load_n(&g_tail, __ATOMIC_ACQUIRE);
    return h - t;
}

// ---- 統計 (ble タスクが書き、rx タスクが読んで FC へ返す) ------------------
//  ★ 複数フィールドをまたぐ一貫性までは保証しない (診断用なので許容)。
struct Stats {
    volatile bool     ble_ok    = false;   // BLE クライアントが接続 & Notify 購読中
    volatile bool     recording = false;
    volatile uint32_t bytes     = 0;       // BLE へ実際に送ったバイト数 (Stat.kbytes に載せる)
    volatile uint32_t drop_ring = 0;   // rx: リング溢れで捨てたレコード (BLE未接続/輻輳が続くと増える)
    volatile uint32_t crc_err   = 0;   // CRC 不一致フレーム
    volatile uint32_t seq_gap   = 0;   // seq 飛び = UART 取りこぼし
    volatile uint32_t orphan    = 0;   // START 前に来た REC
    volatile uint32_t worst_tx_us = 0;
    volatile uint32_t peak_ring = 0;
};
static Stats g_st;

// ============================================================
//  §1  rx役 : UART 受信 + フレーム検証 + リングへ push
// ============================================================
namespace Rx {

// 完成したフレームを「丸ごと」リングへ積む。途中まで積むことはしない
// (ble 役は「リングの中身は必ず完全なフレーム」を前提にパースするため)。
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
//  ★ Stat.flags の STAT_SD_OK ビットは「BLE クライアント接続中」に読み替えている。
//    Stat.file_idx は未使用 (常に 0)。Stat.kbytes は「BLE へ送った累計KB」。
//    LogLinkProto.h / flight_controller 側の表示文言 ("SD") は元の意味のまま
//    残っているので、実際の値の意味とラベルがずれる点に注意 (別途要修正)。
static void sendStat() {
    P::Stat s = {};
    s.flags     = (uint8_t)((g_st.ble_ok ? P::STAT_SD_OK : 0) |
                            (g_st.recording ? P::STAT_REC : 0));
    s.ring_pct  = (uint8_t)((uint64_t)ringUsed() * 100 / RING_BYTES);
    s.file_idx  = 0;
    s.kbytes    = g_st.bytes / 1024;
    s.drop_rec  = g_st.drop_ring;
    s.crc_err   = g_st.crc_err;
    s.seq_gap   = g_st.seq_gap;
    s.worst_w_us = g_st.worst_tx_us;

    uint8_t f[P::MAX_FRAME];
    static uint8_t seq = 0;
    const size_t n = P::buildFrame(f, P::T_STAT, seq++, &s, sizeof(s));
    // 送信は 34B/回 × 2Hz。availableForWrite() を見て、詰まっていたら諦める
    // (受信を止めてまで返す価値のあるデータではない)。
    if ((size_t)Serial1.availableForWrite() >= n) Serial1.write(f, n);
}

} // namespace Rx

// ============================================================
//  §1.5  デバッグ用: FC 無しで BLE 経路を試すためのダミーデータ注入
//    USB シリアルの 't' で、UART を経由せず直接リングへ
//    T_START(ダミーヘッダ) -> T_REC x N(パターンデータ) -> T_STOP を積む。
//    Rx::feed() のフレーム検証は素通りする (自分で組み立てるので当然正しい)。
//    本番の UART 受信とは無関係。ブリングアップ/BLE 単体試験専用。
// ============================================================
namespace Dbg {

static void injectDummy() {
    static uint8_t seq = 0;
    uint8_t f[P::MAX_FRAME];

    // T_START: SdLog と同じ 32B ヘッダ形式 (FlightLog::REC_VER=2 を模す)
    uint8_t hdr[P::BIN_HDR_LEN] = {0};
    memcpy(hdr, "S5LOG", 5);
    hdr[6] = 1;                          // fmt_ver
    hdr[7] = 2;                          // rec_ver (FlightLog::REC_VER)
    hdr[8] = 116; hdr[9] = 0;            // rec_size = sizeof(FlightLog::Rec)
    hdr[10] = 500 & 0xFF; hdr[11] = (500 >> 8) & 0xFF;   // rate_hz
    const uint32_t t0 = millis();
    memcpy(hdr + P::BIN_HDR_T0_OFS, &t0, 4);

    size_t n = P::buildFrame(f, P::T_START, seq++, hdr, sizeof(hdr));
    Rx::push(f, (uint32_t)n);
    Serial.printf("[DBG] dummy T_START 送信 (t0=%lu)\n", (unsigned long)t0);

    // T_REC を N 個。中身は検証用のパターン (116B = FlightLog::Rec と同サイズ)。
    constexpr int N = 200;
    uint8_t pay[116];
    int sent = 0;
    for (int i = 0; i < N; ++i) {
        for (size_t k = 0; k < sizeof(pay); ++k) pay[k] = (uint8_t)(i + k);
        n = P::buildFrame(f, P::T_REC, seq++, pay, sizeof(pay));
        if (!Rx::push(f, (uint32_t)n)) { Serial.println("[DBG] ring 溢れで中断"); break; }
        sent++;
    }
    Serial.printf("[DBG] dummy T_REC x%d 送信\n", sent);

    n = P::buildFrame(f, P::T_STOP, seq++, nullptr, 0);
    Rx::push(f, (uint32_t)n);
    Serial.println("[DBG] dummy T_STOP 送信");
}

} // namespace Dbg

// ============================================================
//  §2  ble役 : リング -> BLE Notify
// ============================================================
namespace Ble {

static BLEServer*         s_server = nullptr;
static BLECharacteristic* s_char   = nullptr;
static volatile bool      s_connected = false;

static bool     s_recording = false;
static uint32_t s_t0        = 0;      // 現在の記録セッション識別子 (BIN ヘッダの t0_ms)
static uint32_t s_bytes     = 0;
static uint32_t s_last_rec_ms = 0;
static uint32_t s_last_tx_ms  = 0;
static uint8_t  s_rec_decim_ctr = 0;
// 「tail の REC は間引き判定の結果すでに送信対象と確定していて、あとは
// BLE の送信間隔 (gate) が空くのを待っているだけ」を覚えておくフラグ。
// ★ これが無いと、gate 待ちで drainRing() が return して次回また同じ
//   (pop していない) REC を見たときに間引きカウンタをもう一度進めてしまい、
//   本来送るはずだった REC まで間引き判定で捨ててしまう
//   (実機テストで 200 REC 中 21 件しか送れなかったのはこれが原因)。
static bool     s_rec_pending = false;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        s_connected  = true;
        g_st.ble_ok  = true;
        Serial.println("[BLE] connected");
    }
    void onDisconnect(BLEServer*) override {
        s_connected  = false;
        g_st.ble_ok  = false;
        Serial.println("[BLE] disconnected -> advertising 再開");
        BLEDevice::startAdvertising();
    }
};

static void init() {
    BLEDevice::init(BLE_DEVICE_NAME);
    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    s_server = server;

    BLEService* svc = server->createService(BLE_SERVICE_UUID);
    s_char = svc->createCharacteristic(BLE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    s_char->addDescriptor(new BLE2902());
    svc->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SERVICE_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("[BLE] advertising開始");
}

static inline uint8_t peek(uint32_t tail, uint32_t i) {
    return g_ring[(tail + i) & RING_MASK];
}

// notify() を連打しすぎるとコントローラの送信キューが溢れて黙って落ちる
// ことがあるので、最低送信間隔を空ける (BLE_MIN_TX_INTERVAL_MS)。
//  ★ これは「送っていいタイミングか」を見るだけの関数にしてある。
//    ここが false のときにフレームを ring から pop 済みで捨ててしまうと、
//    間引き後のデータまで黙って失う (実機テストで実際に踏んだバグ)。
//    なので drainRing() 側は「pop する前」にこれを確認する。
static inline bool readyToSend() {
    if (!s_connected) return false;
    return millis() - s_last_tx_ms >= BLE_MIN_TX_INTERVAL_MS;
}

static void sendFrame(const uint8_t* f, size_t n) {
    s_last_tx_ms = millis();
    const uint32_t t0 = micros();
    s_char->setValue((uint8_t*)f, n);
    s_char->notify();
    const uint32_t dt = micros() - t0;
    if (dt > g_st.worst_tx_us) g_st.worst_tx_us = dt;

    s_bytes += n;
    g_st.bytes = s_bytes;
}

//  ★ 1 回の呼び出しで処理するフレーム数に上限を設ける。
//    リングが空になるまで回す作りだと、FC が 500Hz で流し続けている間は
//    ここから戻れず、タイムアウト close が永久に実行されない。
constexpr int DRAIN_MAX_FRAMES = 32;

static void drainRing() {
    for (int i = 0; i < DRAIN_MAX_FRAMES; ++i) {
        const uint32_t used = ringUsed();
        if (used < P::OVERHEAD) return;

        const uint32_t tail = __atomic_load_n(&g_tail, __ATOMIC_ACQUIRE);
        const uint8_t  type = peek(tail, 2);
        const uint8_t  len  = peek(tail, 3);
        const uint32_t flen = P::OVERHEAD + len;
        if (used < flen) return;                 // まだ全部来ていない

        static uint8_t f[P::MAX_FRAME];
        const uint32_t idx   = tail & RING_MASK;
        const uint32_t first = min(flen, RING_BYTES - idx);
        memcpy(f, g_ring + idx, first);
        if (flen > first) memcpy(f + first, g_ring, flen - first);
        const uint8_t* pay = f + P::HDR_LEN;

        // pop (tail 前進) するのは「送信する」か「意図的に捨てる」と決まってから。
        // 送信を試みる予定なのに BLE が今送れる状態でなければ、pop せずに
        //今回の drainRing を切り上げる (フレームは ring に残る = 次回に再試行)。
        const auto discard = [&]() {
            __atomic_store_n(&g_tail, tail + flen, __ATOMIC_RELEASE);
        };

        switch (type) {
            case P::T_START: {
                if (len != P::BIN_HDR_LEN) { discard(); break; }
                uint32_t t0; memcpy(&t0, pay + P::BIN_HDR_T0_OFS, 4);
                // FC は同じヘッダを 1Hz で再送してくる。同じ t0 なら継続中の
                // セッションの続き = 送らずに捨てる。違えば別フライト。
                if (s_recording && t0 == s_t0) { discard(); break; }
                if (!readyToSend()) return;   // pop せず次回に持ち越す
                discard();
                s_t0        = t0;
                s_recording = true;
                g_st.recording = true;
                Serial.printf("[BLE] START (t0=%lu)\n", (unsigned long)s_t0);
                sendFrame(f, flen);
                break;
            }
            case P::T_REC: {
                if (!s_recording) { discard(); g_st.orphan++; break; }
                s_last_rec_ms = millis();
                // 帯域対策の間引き。捨てた分は drop に数えない (意図的な間引きのため)。
                // ★ s_rec_pending が立っていなければ「まだこの REC の間引き判定を
                //   していない」ので一度だけ判定する。gate 待ちで戻ってきた再訪
                //   (pending==true) では判定をやり直さない (カウンタを二重に
                //   進めてしまうため)。
                if (!s_rec_pending) {
                    if (BLE_REC_DECIM > 1) {
                        s_rec_decim_ctr++;
                        if (s_rec_decim_ctr < BLE_REC_DECIM) { discard(); break; }
                        s_rec_decim_ctr = 0;
                    }
                    s_rec_pending = true;
                }
                if (!readyToSend()) return;   // pop せず次回に持ち越す (判定はやり直さない)
                discard();
                s_rec_pending = false;
                sendFrame(f, flen);
                break;
            }
            case P::T_STOP: {
                if (!s_recording) { discard(); break; }
                if (!readyToSend()) return;   // pop せず次回に持ち越す
                discard();
                sendFrame(f, flen);
                Serial.printf("[BLE] STOP  %lu bytes 送信\n", (unsigned long)s_bytes);
                s_recording    = false;
                g_st.recording = false;
                s_bytes        = 0;
                break;
            }
            default:
                discard();
                break;
        }
    }
}

static void service() {
    drainRing();
    if (!s_recording) return;
    // STOP が化けて届かなかったときの保険。REC が途切れたら勝手に閉じる。
    if (millis() - s_last_rec_ms >= P::IDLE_CLOSE_MS) {
        Serial.println("[BLE] REC 途切れ -> タイムアウトで終了扱い");
        s_recording    = false;
        g_st.recording = false;
        s_bytes        = 0;
    }
}

// シングルコアなので BLE 送信役を専用 FreeRTOS タスクとして起動する。
// rx 側 (loop()) より低い優先度にしてあるので、UART 受信の処理が滞らない。
static void task(void*) {
    delay(50);   // core0/loop() 側の Serial 初期化と競合しないように
    init();
    for (;;) {
        service();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

} // namespace Ble

// ============================================================
//  §3  USB コンソール
// ============================================================
static void printStatus() {
    Serial.println("---- log_recorder (BLE) ----");
    Serial.printf("  BLE       : %s   (decim=1/%u)\n",
                  g_st.ble_ok ? "接続中" : "未接続 (advertising中)",
                  (unsigned)BLE_REC_DECIM);
    Serial.printf("  記録      : %s   送信済み %lu bytes\n",
                  g_st.recording ? "REC" : "idle", (unsigned long)g_st.bytes);
    Serial.printf("  リング    : %lu / %lu B (peak %lu = %lu%%)\n",
                  (unsigned long)ringUsed(), (unsigned long)RING_BYTES,
                  (unsigned long)g_st.peak_ring,
                  (unsigned long)((uint64_t)g_st.peak_ring * 100 / RING_BYTES));
    Serial.printf("  ロス      : drop_ring=%lu  crc_err=%lu  seq_gap=%lu  orphan=%lu\n",
                  (unsigned long)g_st.drop_ring,
                  (unsigned long)g_st.crc_err,
                  (unsigned long)g_st.seq_gap, (unsigned long)g_st.orphan);
    Serial.printf("  最悪tx    : %lu us\n", (unsigned long)g_st.worst_tx_us);
    if (g_st.crc_err || g_st.seq_gap)
        Serial.println("  ★ crc_err/seq_gap が増える = UART が化けている。"
                       "配線を短く / GND を確実に / 両側の BAUD を 1000000 へ");
    if (!g_st.ble_ok && g_st.recording)
        Serial.println("  ★ BLE 未接続のまま記録中 = drop_ring が伸び続ける。"
                       "受信側アプリを接続すること");
    if (g_st.peak_ring > RING_BYTES / 2)
        Serial.println("  ★ リングが半分を超えた = BLE が遅い/輻輳している。"
                       "BLE_REC_DECIM を上げる / 受信側との距離を詰める");
}

// ============================================================
//  §4  rx役 (UART 受信専用。既定の loop タスク)
// ============================================================
void setup() {
    Serial.begin(115200);         // USB (デバッグ用)

    Serial1.setRxBufferSize(UART_FIFO);   // ★ begin() より前に呼ぶこと
    Serial1.begin(LINK_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

    // BLE 送信役 (ESP32C3 はシングルコアなので専用タスクに分離)。
    xTaskCreate(Ble::task, "ble_task", 4096, nullptr, 1, nullptr);
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
        if (c == 'r') { g_st.drop_ring = 0;
                        g_st.crc_err = g_st.seq_gap = 0;
                        g_st.orphan = 0; g_st.peak_ring = 0; g_st.worst_tx_us = 0;
                        Serial.println("統計をリセットしました"); }
        if (c == 't') Dbg::injectDummy();
    }

    // シングルコアなので ble タスク (と裏の idle/WDT) に実行機会を渡す。
    // UART 受信バッファは割込みで積まれ続けるので、この程度の delay では
    // 取りこぼさない (UART_FIFO のサイズで数十 ms ぶん吸収できる)。
    vTaskDelay(pdMS_TO_TICKS(1));
}
