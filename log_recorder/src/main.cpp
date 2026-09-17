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
//  【PC -> 機体 のデバッグ指令 (2026-09-14 追加)】
//    BLE_CHAR_CMD_UUID (Write) に ActReq (action + action_seq の2byte) を
//    書くと、そのまま UART (D6 TX) 経由で機体へ中継する (Cmd:: 参照)。
//    運べるのは PID reset / IMU校正 / デバイス確認の3種類だけで、速度・
//    高度・離着陸要求のような操縦系のフィールドはプロトコル上存在しない。
//
//  【地上局リンク (IM920 の代わり。2026-09-17 追加)】
//    機体の drone_s5.cpp で S5::GROUND_LINK == BLE のとき、IM920 が担って
//    いた地上局との通信をここが中継する (詳細は LogLinkProto.h)。
//      上り: PC ─BLE Write(BLE_CHAR_CTRL_UUID, CmdFrame 22B)→ ここ
//            ─UART T_CMD→ 機体。seq はここで振る。
//      下り: 機体 ─UART T_TELEM→ ここ ─BLE Notify (ログと同じ char)→ PC
//    ★ 上りの扱いは ground_receiver/src/tools/s5_log.cpp (IM920 地上局) と
//      同じ: 最新値だけの mailbox / PC が黙っている間は 200ms ごとに再送 /
//      PC から 1.5 秒来なければ送信停止 (機体はホールド -> 自動着陸)。
//      さらに BLE が切れた瞬間にも送信を止める。
//    ★ デバッグ指令 (上の ActReq) とは characteristic もフレーム型も別。
//      ble_monitor.py などデバッグ用ツールから操縦が出ることはない。
//    ★ 機体を IM920 に戻した場合、ここに操縦指令が来ても機体は読まない
//      (T_CMD を見るのは GROUND_LINK == BLE のときだけ)。ファームはそのままでよい。
//
//  【配線】
//     D7  RX  <---- Teensy TX17 (Serial4)
//     D6  TX  ----> Teensy RX16          ※状態返信 + デバッグ指令 + 操縦指令用。
//                                           ★ 地上局リンクが BLE (既定) の
//                                           ときは必須。無いと機体は PC の
//                                           指令を 1 つも受け取れない
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
#include <SPI.h>
#include <Bitcraze_PMW3901.h>
#include <string>

#include "quad/LogLinkProto.h"   // ★ flight_controller 側の実体を include
#include "S5Cmd.h"               // 同上 (CmdFrame のサイズとオフセットだけ使う)

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
static const char* BLE_CHAR_UUID      = "d5913037-2d8a-41ee-85b9-4e361aa5c8a7";   // Notify (ログ)
// PC -> 機体 のデバッグ指令 (Write)。2026-09-14 追加。
//  ★ ここに書けるのは ActReq (action + action_seq の2byte) だけ。
//    速度・高度・離着陸要求のような操縦系は一切運べない設計にしてある
//    (機体を操縦する経路は今まで通り IM920 だけ。詳細は LogLinkProto.h)。
static const char* BLE_CHAR_CMD_UUID  = "d5913038-2d8a-41ee-85b9-4e361aa5c8a7";
// PC -> 機体 の操縦指令 (Write / Write Without Response)。2026-09-17 追加。
//  中身は S5Cmd.h の CmdFrame (22B) そのもの。IM920 で運んでいたのと同じ構造体。
static const char* BLE_CHAR_CTRL_UUID = "d5913039-2d8a-41ee-85b9-4e361aa5c8a7";

// ---- 地上局リンク (操縦指令の中継) -----------------------------------------
//  値の意味は ground_receiver/src/tools/s5_log.cpp と同じにしてある。
constexpr uint32_t CTRL_KEEPALIVE_MS  = 200;    // PC が黙っている間の再送間隔
constexpr uint32_t CTRL_PC_TIMEOUT_MS = 1500;   // これだけ来なければ送信停止
static_assert(sizeof(S5C::CmdFrame) == LogLinkProto::CMD_PAYLOAD,
              "LogLinkProto::CMD_PAYLOAD を S5Cmd.h の CmdFrame に合わせること");

// リングの滞留がこれを超えたら、送らずに古いフレームから捨てる。
//  ★ 2026-09-17: 地上局リンクのテレメトリ (T_TELEM) も同じ FIFO を通る
//    ようになった。BLE が詰まって REC の山が溜まると、その後ろのテレメトリ
//    (= PC が操縦判断に使う機体状態) が数秒遅れで届く。それは「届かない」
//    より危ない (PC は古い状態を今の状態だと思って指令を出す) ので、
//    遅延の上限をここで切る。16KB = 機体側 500Hz の REC 約 0.26 秒ぶん。
//    代わりに BLE が短時間切れたときの REC の穴埋め (旧: 128KB = 2 秒) は
//    効かなくなる。ログの完全性より操縦の鮮度を優先した結果。
constexpr uint32_t SHED_BYTES = 16u * 1024u;

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
    volatile uint32_t shed      = 0;   // 滞留超過で送らずに捨てたフレーム (SHED_BYTES)
    volatile uint32_t telem_tx  = 0;   // BLE へ送った T_TELEM
    volatile uint32_t telem_drop = 0;  // BLE 未接続/滞留超過で捨てた T_TELEM
};
static Stats g_st;

// ============================================================
//  §1.6  PC -> 機体 のデバッグ指令 (BLE Write -> UART 中継)
// ============================================================
//  onWrite() は BLE スタックのタスクから呼ばれる (rx/ble タスクとは別)。
//  Serial1 への書き込みは rx役 (loop()) だけがやる、という既存の取り決め
//  (Rx::sendStat() 参照) を崩さないよう、ここではフラグを立てるだけにして
//  実際の送信は loop() 側に任せる。1回分しか覚えない (デバッグ用の単発
//  ボタン押下が対象で、連打を全部拾う必要はない)。
namespace Cmd {

static volatile bool    s_pending = false;
static volatile uint8_t s_action  = 0;
static volatile uint8_t s_seq     = 0;
static uint32_t         s_n_rx    = 0;   // 診断用 (printStatus)
static uint32_t         s_n_bad   = 0;   // 長さ不正などで捨てた数

// ★ action の値そのものは中身を見ない (0..255 何でも素通し)。
//   意味の検証 (Action として妥当か / action_seq の重複排除) は
//   全部 FC 側 (drone_s5.cpp handleBleAction()) の役目にしてある。
//   ここで判定を持つと、Action が増えたときに両方直す羽目になる。
static void onWrite(const uint8_t* data, size_t len) {
    if (len != 2) { s_n_bad++; return; }
    s_action  = data[0];
    s_seq     = data[1];
    s_pending = true;
    s_n_rx++;
}

// loop() から毎回呼ぶ。Serial1 への書き込みはここでだけ行う。
static void service() {
    if (!s_pending) return;
    s_pending = false;
    uint8_t f[P::MAX_FRAME];
    P::ActReq req{ s_action, s_seq };
    static uint8_t seq = 0;
    const size_t n = P::buildFrame(f, P::T_ACT, seq++, &req, sizeof(req));
    if ((size_t)Serial1.availableForWrite() >= n) {
        Serial1.write(f, n);
        Serial.printf("[CMD] BLE -> 機体: action=%u seq=%u\n",
                      (unsigned)req.action, (unsigned)req.action_seq);
    } else {
        Serial.println("[CMD] UART送信バッファが詰まっていて送れませんでした");
    }
}

class WriteCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        const std::string v = c->getValue();
        Cmd::onWrite((const uint8_t*)v.data(), v.size());
    }
};

} // namespace Cmd

// ============================================================
//  §1.7  地上局リンク: PC -> 機体 の操縦指令 (BLE Write -> UART T_CMD)
// ============================================================
//  Cmd:: (デバッグ指令) と同じく、onWrite() では mailbox に置くだけで、
//  Serial1 に書くのは loop() の service() だけ。
//  ★ デバッグ指令と違い「最新値を送り続ける」性質のものなので、s5_log.cpp
//    (IM920 地上局) の serviceCmdTx() と同じ振る舞いにしてある:
//      ・新しい指令は届いた次のループで即送る
//      ・PC が黙っている間は CTRL_KEEPALIVE_MS ごとに最後の指令を再送
//      ・PC から CTRL_PC_TIMEOUT_MS 来なければ送信停止
//      ・BLE が切れたら即送信停止 (IM920 版には無い。切断を確実に知れるので)
//    止めたあと機体は「コマンドが来ない」としてホールド -> 自動着陸へ落ちる。
namespace Ctrl {

static portMUX_TYPE      s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t           s_box[LogLinkProto::CMD_PAYLOAD];   // 最新の指令
static uint32_t          s_box_gen    = 0;   // 指令が届くたびに進む (送信中の上書き検出)
static uint32_t          s_last_pc_ms = 0;   // PC から最後に来た時刻 (0 = 未受信)
// 以下は loop() 側だけが触る
static bool     s_have       = false;        // 中継中
static uint32_t s_sent_gen   = 0;            // 最後に UART へ出した指令の世代
static uint32_t s_last_tx_ms = 0;
static uint8_t  s_seq        = 0;
static uint32_t s_n_rx = 0, s_n_bad = 0, s_n_tx = 0, s_n_busy = 0, s_n_stop = 0;

static void onWrite(const uint8_t* data, size_t len) {
    if (len != sizeof(s_box)) { s_n_bad++; return; }
    portENTER_CRITICAL(&s_mux);
    memcpy(s_box, data, sizeof(s_box));
    s_box_gen++;
    s_last_pc_ms = millis();
    portEXIT_CRITICAL(&s_mux);
    s_n_rx++;
}

static void stop(const char* why) {
    if (!s_have) return;
    s_have = false;
    s_n_stop++;
    Serial.printf("[CTRL] 操縦指令の中継を停止 (%s)。機体はホールド -> 自動着陸へ\n", why);
}

// loop() から毎回呼ぶ。Serial1 への書き込みはここでだけ行う。
static void service() {
    const uint32_t now = millis();

    uint8_t  box[LogLinkProto::CMD_PAYLOAD];
    uint32_t gen, last_pc;
    portENTER_CRITICAL(&s_mux);
    memcpy(box, s_box, sizeof(box));
    gen     = s_box_gen;
    last_pc = s_last_pc_ms;
    portEXIT_CRITICAL(&s_mux);

    const bool fresh_cmd = (gen != s_sent_gen);   // まだ送っていない指令がある

    if (!g_st.ble_ok) {
        stop("BLE 切断");
        s_sent_gen = gen;          // 切断前の指令を再接続後に蒸し返さない
        return;
    }
    if (last_pc == 0) return;
    if (now - last_pc > CTRL_PC_TIMEOUT_MS) {
        stop("PC から途切れた");
        s_sent_gen = gen;
        return;
    }
    if (!s_have) {
        if (!fresh_cmd) return;    // 停止後は新しい指令が来るまで再開しない
        s_have = true;
        Serial.println("[CTRL] 操縦指令の中継を開始");
    }
    if (!fresh_cmd && now - s_last_tx_ms < CTRL_KEEPALIVE_MS) return;

    // seq はここで振る (IM920 版では地上局 XIAO が振っていたのと同じ)。
    //  機体の S5C::Rx は seq が同じなら重複として捨てるので、再送でも進める。
    box[offsetof(S5C::CmdFrame, seq)] = s_seq;
    uint8_t f[LogLinkProto::MAX_FRAME];
    static uint8_t link_seq = 0;
    const size_t n = LogLinkProto::buildFrame(f, LogLinkProto::T_CMD, link_seq,
                                              box, sizeof(box));
    if ((size_t)Serial1.availableForWrite() < n) { s_n_busy++; return; }   // 次のループで
    Serial1.write(f, n);
    link_seq++;
    s_seq++;
    s_n_tx++;
    s_last_tx_ms = now;
    s_sent_gen   = gen;
}

class WriteCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        const std::string v = c->getValue();
        Ctrl::onWrite((const uint8_t*)v.data(), v.size());
    }
};

} // namespace Ctrl

// ============================================================
//  §0c  オプティカルフロー (PMW3901) 読み → T_FLOW で FC へ (2026-09-17)
// ============================================================
//  Teensy 故障で FC を XIAO RP2040 にしたところ PMW3901 の SPI 4 本が
//  収まらなくなったので、センサをこの板に載せて生カウントだけ FC へ送る。
//  経緯と全体像は flight_controller/include/quad/LogLink.h の冒頭。
//
//  やること: 100Hz でバースト読み → 累積和を更新 → T_FLOW を Serial1 へ。
//  de-rotation (ジャイロ補正) は FC 側。ここは生値しか触らない。
//
//  ★ 送信は loop() からだけ (Serial1 に書くのは loop() だけ、の規約)。
//  ★ 累積和で送る理由は LogLinkProto.h の FlowFrame を参照。
//  ★ SPI はストラップピン (GPIO2/8/9) を避けて D1〜D4 (GPIO3/4/5/6) に置く。
//    XIAO の既定 SPI パッド (D8/D9/D10 = GPIO8/9/10) を使うと、リセット中に
//    PMW3901 の MISO が GPIO9 を引いてブートモードが化けることがある。
//    ESP32 は GPIO マトリクスでどのピンにも SPI を振れる (4MHz なので性能差なし)。
//    これで CS のプルアップ抵抗は不要。
//  ★ PMW3901 が居なくても起動は続ける (BLE 中継が本業)。FC には
//    STAT_FLOW_OK を立てないことで「フロー無し」を伝える。
namespace Flow {

constexpr uint8_t  PIN_SCK  = 3;    // D1  (ストラップピン GPIO2/8/9 を避ける)
constexpr uint8_t  PIN_MISO = 4;    // D2
constexpr uint8_t  PIN_MOSI = 5;    // D3
constexpr uint8_t  PIN_CS   = 6;    // D4
constexpr uint32_t PERIOD_US = 10000;   // 100Hz (FC の FLOW_LOOP_HZ と同じ)

static Bitcraze_PMW3901 s_sensor(PIN_CS);
static bool     s_ok     = false;
static int32_t  s_sum_dx = 0, s_sum_dy = 0;
static uint16_t s_n      = 0;
static uint8_t  s_squal  = 0;
static uint32_t s_last_us = 0;
static uint32_t s_n_tx   = 0;    // 送った T_FLOW
static uint32_t s_n_busy = 0;    // availableForWrite 不足で見送った回数

inline bool ok() { return s_ok; }

// flight_controller/include/sensor/OpticalFlow.h の readMotionBurst() と同一。
//   CS LOW → 0x16 → tSRAD → 12B 連続読み → CS HIGH。SPI は 4MHz/MSB/MODE3。
static void readMotionBurst(int16_t* dx, int16_t* dy, uint8_t* squal) {
    uint8_t buf[12];
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(50);
    SPI.transfer(0x16);
    delayMicroseconds(50);
    for (int i = 0; i < 12; ++i) buf[i] = SPI.transfer(0);
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
    *dx = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    *dy = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    *squal = buf[6];
}

static void begin() {
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    s_ok = s_sensor.begin();
    Serial.printf("FLOW: PMW3901 %s (SCK=%u MISO=%u MOSI=%u CS=%u)\n",
                  s_ok ? "OK" : "応答なし (配線/電源を確認)",
                  PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    s_last_us = micros();
}

// loop() から毎回呼ぶ。100Hz を超えない範囲で読んで送る。
static void service() {
    if (!s_ok) return;
    const uint32_t now = micros();
    if (now - s_last_us < PERIOD_US) return;
    s_last_us = now;

    int16_t dx = 0, dy = 0;
    readMotionBurst(&dx, &dy, &s_squal);
    s_sum_dx += dx;
    s_sum_dy += dy;
    s_n++;

    P::FlowFrame fr;
    fr.t_us   = now;
    fr.sum_dx = s_sum_dx;
    fr.sum_dy = s_sum_dy;
    fr.n      = s_n;
    fr.squal  = s_squal;

    uint8_t f[P::MAX_FRAME];
    static uint8_t seq = 0;
    const size_t n = P::buildFrame(f, P::T_FLOW, seq, &fr, sizeof(fr));
    // 21B × 100Hz = 2.1kB/s。詰まっていたら見送る (累積和なので次で吸収される)。
    if ((size_t)Serial1.availableForWrite() >= n) {
        Serial1.write(f, n);
        seq++;
        s_n_tx++;
    } else {
        s_n_busy++;
    }
}

} // namespace Flow

// ============================================================
//  §0d  モード表示 LED (StatusLed) をこの板で光らせる (2026-09-17)
// ============================================================
//  FC (XIAO RP2040) はピンを使い切ったので、モード表示の RGB LED をこちらに付ける。
//  FC が T_LED (bit0=R bit1=G bit2=B) を色変化時 + 0.5 秒ごとに送ってくる。
//  1 秒来なければ消灯 = 「FC と繋がっていない」の表示。点滅は FC が作る。
//
//  配線はコモンアノード (Teensy 時代と同じ): 3V3 → LED → 抵抗 → ピン。
//  LOW で点灯、HIGH で消灯 (負論理)。各色に抵抗 1 本ずつ必須。
//  ★ D8/D9 (GPIO8/9) は ESP32C3 のストラップピンだが、コモンアノードの LED は
//    ピンを「3V3 側へ引く」ことしかできないので、リセット中に Low へ落とす
//    ことは無く、ブートモードに影響しない (SPI の MISO とは事情が違う)。
namespace Led {

constexpr uint8_t PIN_R = 8;    // D8
constexpr uint8_t PIN_G = 9;    // D9
constexpr uint8_t PIN_B = 10;   // D10
constexpr uint32_t TIMEOUT_MS = 1000;

static uint32_t s_last_ms = 0;
static uint32_t s_n_rx    = 0;
static uint8_t  s_rgb     = 0;

static void apply(uint8_t rgb) {
    digitalWrite(PIN_R, (rgb & P::LED_R) ? LOW : HIGH);
    digitalWrite(PIN_G, (rgb & P::LED_G) ? LOW : HIGH);
    digitalWrite(PIN_B, (rgb & P::LED_B) ? LOW : HIGH);
}

static void begin() {
    pinMode(PIN_R, OUTPUT);
    pinMode(PIN_G, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    apply(0);
}

// onFrame() (rx役) から T_LED のたびに呼ばれる
static void onLed(const uint8_t* pay, uint8_t len) {
    if (len != sizeof(P::LedFrame)) return;
    s_rgb = pay[0];
    s_last_ms = millis();
    s_n_rx++;
    apply(s_rgb);
}

// loop() から毎回。FC が黙ったら消灯
static void service() {
    if (s_last_ms != 0 && millis() - s_last_ms > TIMEOUT_MS) {
        s_last_ms = 0;
        s_rgb = 0;
        apply(0);
    }
}

} // namespace Led

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

    // T_LED はこの板で消費する (BLE へは流さない)
    if (s_type == P::T_LED) { Led::onLed(s_pay, s_len); return; }

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
                            (g_st.recording ? P::STAT_REC : 0) |
                            (Flow::ok() ? P::STAT_FLOW_OK : 0));
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

    // PC -> 機体 のデバッグ指令 (Write)。ActReq (2byte) のみ受け付ける。
    BLECharacteristic* cmd_char = svc->createCharacteristic(
        BLE_CHAR_CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
    cmd_char->setCallbacks(new Cmd::WriteCallbacks());

    // PC -> 機体 の操縦指令 (CmdFrame 22B)。応答なし Write を許すのは遅延を
    // 減らすため (最新値を送り続ける性質なので、1 発落ちても次で上書きされる)。
    BLECharacteristic* ctrl_char = svc->createCharacteristic(
        BLE_CHAR_CTRL_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    ctrl_char->setCallbacks(new Ctrl::WriteCallbacks());

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
        // 滞留が SHED_BYTES を超えていたら、送らずに古いほうから捨てる
        // (理由は SHED_BYTES のコメント)。BLE 未接続でもここは進むので、
        // リングは常に「直近 SHED_BYTES ぶん」程度しか持たない。
        const bool shed = used > SHED_BYTES;

        switch (type) {
            case P::T_START: {
                if (len != P::BIN_HDR_LEN) { discard(); break; }
                uint32_t t0; memcpy(&t0, pay + P::BIN_HDR_T0_OFS, 4);
                // FC は同じヘッダを 1Hz で再送してくる。同じ t0 なら継続中の
                // セッションの続き = 送らずに捨てる。違えば別フライト。
                if (s_recording && t0 == s_t0) { discard(); break; }
                if (shed) { discard(); g_st.shed++; break; }   // 1Hz で再送されてくる
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
                if (shed) { discard(); g_st.shed++; s_rec_pending = false; break; }
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
                if (shed) {
                    // 送れないが、記録終了の状態遷移だけはやる
                    discard(); g_st.shed++;
                    s_recording = false; g_st.recording = false; s_bytes = 0;
                    break;
                }
                if (!readyToSend()) return;   // pop せず次回に持ち越す
                discard();
                sendFrame(f, flen);
                Serial.printf("[BLE] STOP  %lu bytes 送信\n", (unsigned long)s_bytes);
                s_recording    = false;
                g_st.recording = false;
                s_bytes        = 0;
                break;
            }
            case P::T_ACT_ACK: {
                // デバッグ指令(T_ACT)の実行結果。記録中かどうかに関係なく、
                // 常に転送する (recording state gate の対象外)。
                if (shed) { discard(); g_st.shed++; break; }
                if (!readyToSend()) return;   // pop せず次回に持ち越す
                discard();
                sendFrame(f, flen);
                Serial.println("[BLE] ACT_ACK 送信");
                break;
            }
            case P::T_TELEM: {
                // 地上局リンクの下りテレメトリ。記録中かどうかに関係なく送る。
                //  ★ BLE 未接続なら待たずに捨てる。つながった後に古い状態を
                //    まとめて送っても PC を惑わせるだけ (機体は 20Hz で
                //    新しいものを送り続けている)。
                if (shed || !s_connected) { discard(); g_st.telem_drop++; break; }
                if (!readyToSend()) return;   // pop せず次回に持ち越す
                discard();
                sendFrame(f, flen);
                g_st.telem_tx++;
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
    Serial.printf("  フロー    : %s  送信 %lu  詰まり %lu  sum=(%ld,%ld) n=%u squal=%u\n",
                  Flow::ok() ? "PMW3901 OK" : "PMW3901 無し",
                  (unsigned long)Flow::s_n_tx, (unsigned long)Flow::s_n_busy,
                  (long)Flow::s_sum_dx, (long)Flow::s_sum_dy,
                  (unsigned)Flow::s_n, (unsigned)Flow::s_squal);
    Serial.printf("  LED       : %s  rgb=%u  受信 %lu\n",
                  Led::s_last_ms ? "FC から受信中" : "消灯 (FC 未受信)",
                  (unsigned)Led::s_rgb, (unsigned long)Led::s_n_rx);
    Serial.printf("  デバッグ指令(BLE->機体): 受信 %lu 件  不正 %lu 件\n",
                  (unsigned long)Cmd::s_n_rx, (unsigned long)Cmd::s_n_bad);
    Serial.printf("  操縦指令(BLE->機体)    : %s  受信 %lu  UART送信 %lu  不正 %lu  "
                  "詰まり %lu  停止 %lu 回\n",
                  Ctrl::s_have ? "中継中" : "停止",
                  (unsigned long)Ctrl::s_n_rx, (unsigned long)Ctrl::s_n_tx,
                  (unsigned long)Ctrl::s_n_bad, (unsigned long)Ctrl::s_n_busy,
                  (unsigned long)Ctrl::s_n_stop);
    Serial.printf("  テレメトリ(機体->BLE)  : 送信 %lu  破棄 %lu   滞留超過で破棄 %lu フレーム\n",
                  (unsigned long)g_st.telem_tx, (unsigned long)g_st.telem_drop,
                  (unsigned long)g_st.shed);
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

    Flow::begin();    // PMW3901。居なくても続行 (STAT_FLOW_OK が立たないだけ)
    Led::begin();     // モード表示 LED (FC からの T_LED で光る)

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
    Cmd::service();   // BLE から指令が来ていれば、ここで初めて Serial1 へ書く
    Ctrl::service();  // 操縦指令の中継 (地上局リンク)。同上
    Flow::service();  // PMW3901 を 100Hz で読んで T_FLOW を送る。同上
    Led::service();   // FC が黙ったら LED を消す

    // USB コンソール
    if (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == 's') printStatus();
        if (c == 'r') { g_st.drop_ring = 0;
                        g_st.crc_err = g_st.seq_gap = 0;
                        g_st.orphan = 0; g_st.peak_ring = 0; g_st.worst_tx_us = 0;
                        g_st.shed = 0; g_st.telem_tx = 0; g_st.telem_drop = 0;
                        Serial.println("統計をリセットしました"); }
        if (c == 't') Dbg::injectDummy();
    }

    // シングルコアなので ble タスク (と裏の idle/WDT) に実行機会を渡す。
    // UART 受信バッファは割込みで積まれ続けるので、この程度の delay では
    // 取りこぼさない (UART_FIFO のサイズで数十 ms ぶん吸収できる)。
    vTaskDelay(pdMS_TO_TICKS(1));
}
