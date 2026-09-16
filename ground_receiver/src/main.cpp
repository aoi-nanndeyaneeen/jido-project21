// ============================================================
//  main.cpp  -  地上局 (XIAO + IM920sL): テレメトリ受信 → USB CSV / 上りコマンド中継
// ============================================================
//  機体 (flight_controller drone_s5) と PC (position_estimator) の間に立つ。
//
//      機体 ──IM920 8Hz──> [ここ] ──USB 115200──> PC   S5Link (console.py / main.py)
//      機体 <──IM920 8Hz── [ここ] <──USB────────  PC   "CMD,..." 行
//
//  やることは 3 つだけ (制御は一切しない):
//    1. IM920 の受信行をデコードして CSV (DATA/PARAM) と STAT を USB へ  … TelemetryStore.h
//    2. PC の "CMD,..." 行を mailbox に持ち、蓋 (125ms) が開くたび無線へ   … Uplink.h
//    3. 1 文字キー (1/0/l/s/d/z/h) と人間向け表示                        … StatusView.h
//  配線と周期は GroundConfig.h。パケット定義は ../protocol/ (機体と共有)。
//
//  ビルド / 書き込み:
//      pio run -e xiao_s5_log -t upload
//      pio device monitor -e xiao_s5_log      (手で見るだけならこれでも可)
//  ★ 普段の運用では PC 側の console.py / main.py が USB を開く (同じ COM は 1 プロセス)。
// ============================================================
#include <Arduino.h>
#include "GroundConfig.h"
#include "TelemetryStore.h"
#include "Uplink.h"
#include "StatusView.h"

using namespace Ground;

static HardwareSerial* IM = &Serial1;
static TelemetryStore  telem;
static Uplink          uplink(&Serial1);
static IM920::LineAssembler<256> im_line;   // IM920 からの行
static bool raw_dump      = false;          // 'd' で IM920 の生の行をそのまま出す
static bool im920_boot_ok = false;

// USB から読みかけの行 ("CMD,..." だけ行として溜める。先頭が 'C' 以外は即キー扱い)
static char   usb_line[96];
static size_t usb_n = 0;

// ------------------------------------------------------------
//  キー入力。★ どのキーにも必ず 1 行返す (無反応だと「キーが届いていない」のか
//  「届いたが機体からパケットが来ていない」のか区別できない)。
//  ★ '1'/'0' は冪等な ON/OFF。PC 側はこちらを使う ('l' のトグルだと、受信機が既に
//    出力中のときに PC を起動した場合に「開始のつもりが停止」になる)。
//  ★ 'C' を使うキーは今も将来も作らないこと ("CMD," 行の先頭と衝突する)。
// ------------------------------------------------------------
static void handleKey(char c) {
    if (c == '\r' || c == '\n' || c == ' ') return;   // 改行は無視 (返事もしない)
    switch (c) {
        case '1': telem.csvOn();  break;
        case '0': telem.csvOff(); break;
        case 'l': case 'L': telem.csv_on ? telem.csvOff() : telem.csvOn(); break;
        case 's': case 'S': printStatus(telem, uplink); break;
        case 'd': case 'D':
            raw_dump = !raw_dump;
            Serial.printf("# 生データ表示 = %s\n", raw_dump ? "ON" : "OFF");
            break;
        case 'z': case 'Z':
            telem.clearStats();
            uplink.clearStats();
            Serial.println("# 統計をクリアしました");
            break;
        case 'h': case 'H': case '?': printHelp(); break;
        default:
            Serial.printf("# 未知のキー '%c' (0x%02X)。使えるのは 1 / 0 / l / s / d / z / h\n",
                          (c >= 0x20 && c < 0x7f) ? c : '?', (uint8_t)c);
            break;
    }
}

// USB 入力: 1 文字キーと複数文字の "CMD,..." 行を同じストリームで受ける
static void pollUsb() {
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (usb_n == 0 && c != 'C') { handleKey(c); continue; }
        if (c == '\r') continue;
        if (c == '\n') {
            usb_line[usb_n] = '\0';
            if (usb_n >= 4 && strncmp(usb_line, "CMD,", 4) == 0) uplink.handleCmdLine(usb_line);
            else if (usb_n) { uplink.n_bad++; Serial.printf("# 未知の行: %s\n", usb_line); }
            usb_n = 0;
            continue;
        }
        if (usb_n < sizeof(usb_line) - 1) usb_line[usb_n++] = c;
        else { usb_n = 0; uplink.n_bad++; }
    }
}

// IM920 入力: 1 文字ずつ行に組み立て、完成したら TelemetryStore へ
static void pollIm920() {
    while (IM->available()) {
        const char c = (char)IM->read();
        telem.n_rx_bytes++;
        if (im_line.feed(c)) telem.handleLine(im_line.line(), raw_dump);
    }
}

// ------------------------------------------------------------
//  起動時の IM920 疎通チェック。RDCH を投げて応答が返るかだけ見る。
//  ★ ここで NG なら配線/電源/ボーレートの問題。テレメトリが来ないのを
//    何十秒も待つ前に切り分けられる。
// ------------------------------------------------------------
static void imBootCheck() {
    while (IM->available()) IM->read();   // 起動直後のゴミを捨てる
    IM->print("RDCH\r\n");

    const unsigned long t0 = millis();
    String got;
    while (millis() - t0 < 300) {
        while (IM->available()) got += (char)IM->read();
    }
    got.trim();

    if (got.length() == 0) {
        im920_boot_ok = false;
        Serial.println("# [BOOT CHECK] IM920 応答なし。配線/電源(3V3か)/ボーレート(19200)を確認してください。");
    } else {
        im920_boot_ok = true;
        Serial.printf("# [BOOT CHECK] IM920 応答あり: %s\n", got.c_str());
    }
}

// ------------------------------------------------------------
void setup() {
    Serial.begin(USB_BAUD);
#if defined(ARDUINO_ARCH_ESP32)
    Serial1.begin(IM_BAUD, SERIAL_8N1, PIN_XIAO_RX, PIN_XIAO_TX);
#elif defined(ARDUINO_ARCH_RP2040)
    Serial1.setTX(PIN_XIAO_TX);
    Serial1.setRX(PIN_XIAO_RX);
    IM->begin(IM_BAUD);
#else
    IM->begin(IM_BAUD);
#endif

    while (!Serial && millis() < 3000) {}
    Serial.println();
    Serial.println("# === s5 telemetry receiver / ground station ===");
    imBootCheck();
    printHelp();
}

// ------------------------------------------------------------
//  loop  — 周期の骨組み (一覧は GroundConfig.h)
// ------------------------------------------------------------
void loop() {
    pollIm920();            // 全速: 下りテレメトリ → CSV/STAT
    pollUsb();              // 全速: キー / CMD 行
    uplink.service();       // 毎ループ: 上りコマンドの送信 (蓋 CMD_MIN_GAP_MS)

    const uint32_t now = millis();

    // PC の常時診断用 (1Hz)。S5Link が受け取り、画面にだけ表示する。
    static uint32_t last_stat = 0;
    if (now - last_stat >= STAT_PERIOD_MS) {
        last_stat = now;
        telem.emitStat(im920_boot_ok, uplink.n_lines, uplink.n_tx, uplink.n_bad, uplink.have());
    }

    if (!telem.csv_on) {
        // CSV を出していないときは、人間向けの状態を 1Hz で流す
        static uint32_t last_draw = 0;
        if (now - last_draw >= STATUS_PERIOD_MS) { last_draw = now; printStatus(telem, uplink); }
        return;
    }

    // CSV 中にテレメトリが途切れたら、黙り込まずに知らせる ('#' 行なので PC は CSV に書かない)
    static uint32_t last_beat = 0;
    if (telem.ageMs() > HEARTBEAT_PERIOD_MS && now - last_beat >= HEARTBEAT_PERIOD_MS) {
        last_beat = now;
        Serial.printf("# テレメトリ待機中: IM920 %lu bytes / %lu 行, "
                      "A=%lu B=%lu C=%lu D=%lu badcs=%lu badlen=%lu\n",
                      (unsigned long)telem.n_rx_bytes, (unsigned long)telem.n_rx_lines,
                      (unsigned long)telem.n_alt, (unsigned long)telem.n_pos,
                      (unsigned long)telem.n_att, (unsigned long)telem.n_dv,
                      (unsigned long)telem.n_bad_cs, (unsigned long)telem.n_bad_len);
    }
}
