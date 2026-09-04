// IM920 疎通確認用の使い捨てスケッチ (機体側 / Teensy 4.0)。
// USBシリアル(115200) と IM920 を素通しする。
// 地上局側 (XIAO RP2040) の ground_receiver/src/tools/im920_passthrough.cpp と対になる。
//
//   pio run -e im920_test -t upload
//   pio device monitor -e im920_test
//
// USBシリアルから使えるコマンド:
//   ID          RDID を送って自分のモジュールIDを読む (認識確認)
//   INFO        RDID/RDVR/RDNN/RDGN/RDCH をまとめて読む
//   SCAN        Serial1..7 × ボーレートを総当たりして応答がある組合せを探す
//   SER <n>     使う HardwareSerial を切り替える (1..7)
//   BAUD <n>    ボーレート変更
//   LISTEN      10秒間、IM920から勝手に流れてくるバイトを待つ
//   PINS        全UARTのRXピンを調べ、外部から駆動されているピンを探す(挿し間違い探し)
//   PING        相手機へ "TXDA 50494E47" を送る (地上局側に PING と出れば往路OK)
//   それ以外    そのまま IM920 へ CR+LF 付きで送信 (例: TXDA 3031)
#include <Arduino.h>

// Teensy 4.0 の HardwareSerial とピン割り当て
struct UartDef { HardwareSerial* ser; int tx; int rx; };
const UartDef UARTS[] = {
    {&Serial1, 1, 0}, {&Serial2, 8, 7},  {&Serial3, 14, 15}, {&Serial4, 17, 16},
    {&Serial5, 20, 21}, {&Serial6, 24, 25}, {&Serial7, 29, 28},
};
constexpr int UART_N = sizeof(UARTS) / sizeof(UARTS[0]);

constexpr unsigned long BAUD_LIST[] = {19200, 38400, 115200, 9600, 57600, 4800};
constexpr size_t BAUD_N = sizeof(BAUD_LIST) / sizeof(BAUD_LIST[0]);

// ★ QuadConfig.h / drone_s5.cpp の配線に合わせた既定値: Serial3 (TX=14, RX=15) @ 19200
int           g_uart = 3;
unsigned long g_baud = 19200;
HardwareSerial* g_im = &Serial3;

void applyConfig(int uart, unsigned long baud) {
    if (uart < 1 || uart > UART_N) { Serial.printf("[err] SER は 1..%d\n", UART_N); return; }
    g_im->end();
    delay(20);
    g_uart = uart; g_baud = baud;
    g_im = UARTS[uart - 1].ser;
    g_im->begin(baud);
    delay(50);
    while (g_im->available()) g_im->read();
}

void printConfig() {
    const UartDef& u = UARTS[g_uart - 1];
    Serial.printf("[cfg] Serial%d  baud=%lu   (Teensy TX=pin%d -> IM920 RXD / RX=pin%d <- IM920 TXD)\n",
                  g_uart, g_baud, u.tx, u.rx);
}

// 受信を待って表示する。何かバイトが返ってきたら true。
bool waitReply(unsigned long ms) {
    unsigned long t0 = millis();
    String got;
    while (millis() - t0 < ms) {
        while (g_im->available()) got += (char)g_im->read();
    }
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

void sendCmd(const char* cmd, unsigned long ms = 600) {
    Serial.printf("[TX] %-6s : ", cmd);
    g_im->print(cmd); g_im->print("\r\n");
    waitReply(ms);
}

void doInfo() {
    Serial.println("=== INFO (モジュール設定の読み出し) ===");
    sendCmd("RDID");  // 自分のモジュールID
    sendCmd("RDVR");  // ファームウェアバージョン
    sendCmd("RDNN");  // 自ノード番号 (STNN)
    sendCmd("RDGN");  // グループ番号 (STGN) ← 相手機と揃っている必要あり
    sendCmd("RDCH");  // チャンネル (STCH) ← 相手機と揃っている必要あり
    Serial.println("=== INFO done ===");
    Serial.println("実測値(2026-09-04): 機体 ID=0001C7A3 / NN=0001 / GN=0001C7A3 / CH=20");
    Serial.println("                   地上 ID=0001C7A2 / NN=0002 / GN=0001C7A3 / CH=20");
    Serial.println("★GN(グループ番号)と CH は両機で同じ値。違っていたら通信できない。");
}

bool trial(int uart, unsigned long baud) {
    applyConfig(uart, baud);
    g_im->print("RDID\r\n");
    Serial.printf("  Serial%d %6lu : ", uart, baud);
    return waitReply(400);
}

void doScan() {
    Serial.println("=== SCAN start (RDID を各設定で送信) ===");
    int save_uart = g_uart; unsigned long save_baud = g_baud;
    bool hit = false;
    for (int u = 1; u <= UART_N; u++)
        for (size_t b = 0; b < BAUD_N; b++)
            if (trial(u, BAUD_LIST[b])) hit = true;
    Serial.println(hit ? "=== SCAN done: 応答あり(上の行を確認) ==="
                       : "=== SCAN done: 全滅。配線か電源を疑う ===");
    applyConfig(save_uart, save_baud);
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
// 相手のUART TXDはアイドル時にHighを出しているので、繋がっていれば "driven HIGH" になる。
void doPins() {
    g_im->end(); delay(50);
    Serial.println("=== PINS: 各UARTのRXピンが外部から駆動されているか ===");
    int driven = 0;
    for (int i = 0; i < UART_N; i++) {
        int pin = UARTS[i].rx;
        pinMode(pin, INPUT_PULLUP);   delay(15); int up   = digitalRead(pin);
        pinMode(pin, INPUT_PULLDOWN); delay(15); int down = digitalRead(pin);
        pinMode(pin, INPUT);
        Serial.printf("  Serial%d RX = pin%-2d  up=%d dn=%d  %s\n", i + 1, pin, up, down,
                      (up == down) ? (up ? "<<< driven HIGH" : "<<< driven LOW") : "floating");
        if (up == down) driven++;
    }
    Serial.printf("=== PINS done: driven pins = %d ===\n", driven);
    Serial.println("IM920 TXD が刺さっているピンが driven HIGH で出るはずです(期待: pin15 = Serial3 RX)。");
    Serial.println("floating なら未接続か IM920 が無給電。");
    applyConfig(g_uart, g_baud);
}

void doPing() {
    Serial.println("=== PING: 相手機へ 'PING' を無線送信 ===");
    sendCmd("TXDA 50494E47", 1000);   // "PING" の ASCII
    Serial.println("OK が返れば送信成功。地上局側のモニタに 00,C7A2,xx:50,49,4E,47 のような行が出れば往復OK。");
}

void setup() {
    Serial.begin(115200);
    applyConfig(g_uart, g_baud);
    while (!Serial && millis() < 3000) {}
    Serial.println("=== IM920 passthrough (Teensy) / ID, INFO, SCAN, SER <n>, BAUD <n>, LISTEN, PINS, PING ===");
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

        if (upper == "ID")          { sendCmd("RDID"); }
        else if (upper == "INFO")   { doInfo(); }
        else if (upper == "SCAN")   { doScan(); }
        else if (upper == "LISTEN") { doListen(); }
        else if (upper == "PINS")   { doPins(); }
        else if (upper == "PING")   { doPing(); }
        else if (upper.startsWith("SER "))  { applyConfig(cmd.substring(4).toInt(), g_baud); printConfig(); }
        else if (upper.startsWith("BAUD ")) { applyConfig(g_uart, cmd.substring(5).toInt()); printConfig(); }
        else {
            g_im->print(cmd); g_im->print("\r\n");
            Serial.print("[TX] "); Serial.println(cmd);
        }
    }
    while (g_im->available()) Serial.write(g_im->read());
}
