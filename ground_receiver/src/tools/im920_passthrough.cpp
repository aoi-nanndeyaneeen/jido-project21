// IM920 疎通確認用の使い捨てスケッチ。
// USBシリアル(115200) と IM920 を素通しする。
// 本番ビルドには入らない: platformio.ini の build_src_filter で tools/ を除外し、
// env:xiao_im920_test のときだけこのファイルをビルドする。
//
// USBシリアルから使えるコマンド:
//   SCAN        TX/RX の向き × ボーレートを総当たりして応答がある組合せを探す
//   SWAP        TX/RX を入れ替える (配線を触らずに試せる)
//   BAUD <n>    ボーレート変更
//   LISTEN      10秒間、IM920から勝手に流れてくるバイトを待つ
//   PINS        GP0/GP1 が外部から駆動されているか(=線が繋がっているか)を判定
//   PINSALL     D0〜D10 を全部調べ、外部から駆動されているピンを探す(挿し間違い探し)
//   それ以外    そのまま IM920 へ CR+LF 付きで送信
#include <Arduino.h>

// XIAO RP2040: シルクの TX = D6 = GP0、RX = D7 = GP1
constexpr int PIN_XIAO_TX = 0; // -> IM920 RXD
constexpr int PIN_XIAO_RX = 1; // <- IM920 TXD

constexpr unsigned long BAUD_LIST[] = {19200, 38400, 115200, 9600, 57600, 4800};
constexpr size_t BAUD_N = sizeof(BAUD_LIST) / sizeof(BAUD_LIST[0]);

// 入れ替え側は GP1 を TX にする必要があり、ハードUART0では割り当てられないので
// PIO ベースのソフトUARTを使う。
SerialPIO pio_swapped(PIN_XIAO_RX, PIN_XIAO_TX, 256); // (tx, rx) = (GP1, GP0)

bool     g_swapped = false;
unsigned long g_baud = 19200;
Stream*  g_im = &Serial1;

void applyConfig(bool swapped, unsigned long baud) {
    Serial1.end();
    pio_swapped.end();
    delay(20);
    if (swapped) {
        pio_swapped.begin(baud);
        g_im = &pio_swapped;
    } else {
        Serial1.setTX(PIN_XIAO_TX);
        Serial1.setRX(PIN_XIAO_RX);
        Serial1.begin(baud);
        g_im = &Serial1;
    }
    g_swapped = swapped;
    g_baud = baud;
    delay(50);
    while (g_im->available()) g_im->read();
}

void printConfig() {
    Serial.printf("[cfg] %s  baud=%lu   (XIAO TX=GP%d, RX=GP%d)\n",
                  g_swapped ? "SWAPPED" : "NORMAL ", g_baud,
                  g_swapped ? PIN_XIAO_RX : PIN_XIAO_TX,
                  g_swapped ? PIN_XIAO_TX : PIN_XIAO_RX);
}

// 1組合せぶん試す。何かバイトが返ってきたら true。
bool trial(bool swapped, unsigned long baud) {
    applyConfig(swapped, baud);
    g_im->print("RDID\r\n");
    unsigned long t0 = millis();
    String got;
    while (millis() - t0 < 600) {
        while (g_im->available()) got += (char)g_im->read();
    }
    Serial.printf("  %-7s %6lu : ", swapped ? "SWAPPED" : "NORMAL", baud);
    if (got.length() == 0) { Serial.println("(no response)"); return false; }
    Serial.printf("%d bytes <- ", got.length());
    for (size_t i = 0; i < got.length(); i++) {
        char c = got[i];
        if (c >= 0x20 && c < 0x7f) Serial.print(c);
        else Serial.printf("<%02X>", (uint8_t)c);
    }
    Serial.println();
    return true;
}

void doScan() {
    Serial.println("=== SCAN start (RDID を各設定で送信) ===");
    bool hit = false;
    for (int s = 0; s < 2; s++)
        for (size_t b = 0; b < BAUD_N; b++)
            if (trial(s == 1, BAUD_LIST[b])) hit = true;
    Serial.println(hit ? "=== SCAN done: 応答あり(上の行を確認) ==="
                       : "=== SCAN done: 全滅。配線か電源を疑う ===");
    applyConfig(false, 19200);
    printConfig();
}

void doListen() {
    Serial.println("=== LISTEN 10s (何もせず受信だけ待つ) ===");
    unsigned long t0 = millis();
    int n = 0;
    while (millis() - t0 < 10000) {
        while (g_im->available()) { Serial.write(g_im->read()); n++; }
    }
    Serial.printf("\n=== LISTEN done: %d bytes ===\n", n);
}

// 内部プルアップ/プルダウンを切り替えて読み比べ、ピンが外部から駆動されているか判定する。
// 相手のUART TXDはアイドル時にHighを出しているので、繋がっていれば必ず "driven HIGH" になる。
void probePin(int pin, const char* label) {
    pinMode(pin, INPUT_PULLUP);   delay(20); int up   = digitalRead(pin);
    pinMode(pin, INPUT_PULLDOWN); delay(20); int down = digitalRead(pin);
    pinMode(pin, INPUT);
    Serial.printf("  GP%-2d %-22s pullup=%d pulldown=%d  -> ", pin, label, up, down);
    if (up == down) Serial.println(up ? "driven HIGH (外部が駆動中 = 繋がっている)"
                                     : "driven LOW  (外部がGNDに引いている / 短絡?)");
    else            Serial.println("FLOATING (何も繋がっていない or 相手が出力していない)");
}

// XIAO RP2040 のシルク D0〜D10 と GPIO 番号の対応
struct XiaoPin { const char* label; int gp; };
const XiaoPin XIAO_PINS[] = {
    {"D0/A0", 26}, {"D1/A1", 27}, {"D2/A2", 28}, {"D3/A3", 29},
    {"D4/SDA", 6}, {"D5/SCL", 7},
    {"D6/TX",  0}, {"D7/RX",  1},
    {"D8/SCK", 2}, {"D9/MISO", 4}, {"D10/MOSI", 3},
};

void doPinsAll() {
    Serial1.end(); pio_swapped.end(); delay(50);
    Serial.println("=== PINSALL: D0-D10 を全部調べる ===");
    int driven = 0;
    for (const XiaoPin& p : XIAO_PINS) {
        pinMode(p.gp, INPUT_PULLUP);   delay(15); int up   = digitalRead(p.gp);
        pinMode(p.gp, INPUT_PULLDOWN); delay(15); int down = digitalRead(p.gp);
        pinMode(p.gp, INPUT);
        Serial.printf("  %-9s (GP%-2d) up=%d dn=%d  %s\n", p.label, p.gp, up, down,
                      (up == down) ? (up ? "<<< driven HIGH" : "<<< driven LOW") : "floating");
        if (up == down) driven++;
    }
    Serial.printf("=== PINSALL done: driven pins = %d ===\n", driven);
    Serial.println("IM920 TXD の線が刺さっているピンが driven HIGH で出るはずです。");
    applyConfig(g_swapped, g_baud);
}

void doPins() {
    Serial1.end(); pio_swapped.end(); delay(50);
    Serial.println("=== PINS ===");
    probePin(PIN_XIAO_RX, "D7 <- IM920 TXD");
    probePin(PIN_XIAO_TX, "D6 -> IM920 RXD");
    Serial.println("期待値: D7(GP1) が driven HIGH。FLOATING なら IM920 TXD が未接続か無給電。");
    applyConfig(g_swapped, g_baud);
}

void setup() {
    Serial.begin(115200);
    applyConfig(false, 19200);
    while (!Serial && millis() < 3000) {}
    Serial.println("=== IM920 passthrough / SCAN, SWAP, BAUD <n>, LISTEN ===");
    printConfig();
}

void loop() {
    static String line;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c != '\n') { if (line.length() < 128) line += c; continue; }

        String cmd = line; line = "";
        cmd.trim();
        String upper = cmd; upper.toUpperCase();

        if (upper == "SCAN")        { doScan(); }
        else if (upper == "LISTEN") { doListen(); }
        else if (upper == "PINS")   { doPins(); }
        else if (upper == "PINSALL"){ doPinsAll(); }
        else if (upper == "SWAP")   { applyConfig(!g_swapped, g_baud); printConfig(); }
        else if (upper.startsWith("BAUD ")) {
            applyConfig(g_swapped, cmd.substring(5).toInt()); printConfig();
        } else {
            g_im->print(cmd); g_im->print("\r\n");
            Serial.print("[TX] "); Serial.println(cmd);
        }
    }
    while (g_im->available()) Serial.write(g_im->read());
}
