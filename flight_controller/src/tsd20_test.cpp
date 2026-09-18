// TSD20 疎通・性能確認用の使い捨てスケッチ (XIAO RP2040 / ベンチ専用)。
//
//   pio run -e tsd20_test -t upload
//   pio device monitor -e tsd20_test
//
// 目的は COMPETITION_OPEN_ISSUES.md の A1「3m超の高度をどう測るか」に対して
// **飛ばさずに判断できること** を全部ベンチで潰すこと:
//   1. そもそも通信できるか            → USCAN (UART) / SCAN (I2C)
//   2. 3.6m / 4.0m が読めるか          → USTAT / STAT (巻尺と突き合わせ)
//   3. 実効レートと欠測率はいくつか    → URUN / RUN
//
// ============================================================================
//  ★ 2026-09-18 メーカー公式マニュアル (akizukidenshi.com "TSD20 user manual.pdf")
//    を読んで判明したこと。ここが今までの詰まりの原因。
//
//  ・TSD20 は **UART と IIC の両対応だが、モードは排他**。Quick Test の章が
//    UART 前提で書かれており、レジスタ 0x04/0x05 に「IIC to UART Switch」、
//    UART 側に逆向きの「Change IIC」コマンドがある。
//    → **出荷時は UART モード。I2C で話すには先に UART でモード切替が要る。**
//      I2C バスをいくら探してもセンサはそこにいない。
//
//  ・used/tsd20_backend.patch の I2C 側の記述 (addr 0x52 7bit / 0x00-0x01 距離 /
//    0x02 レーザ / 0x03 ID=0x4A) は **マニュアルと完全に一致していて正しい**。
//    間違っていたのは「最初から I2C で喋れる」という前提だけ。
//
//  ・配線は 6 ピン。1 と 5 が NC なので「端から 4 本」と数えるとずれる:
//      1 NC / 2 3.3V / 3 TX(SCL) / 4 RX(SDA) / 5 NC / 6 GND
//    ★ I2C は SCL↔SCL / SDA↔SDA の **ストレート**。
//      UART は TX↔RX の **クロス**。IM920sL の癖でクロスにすると SDA/SCL が
//      入れ替わり、UART で喋り続ける TX を SDA と誤認して I2C スキャンに
//      毎回違う幽霊アドレスが出る (実際 0x44 → 0x43+0x5A が観測された)。
// ============================================================================
//
// ---- 配線 (XIAO RP2040) -------------------------------------------------
//  UART モード (出荷時。まずこちら):
//    TSD20 pin3 TX ──→ D7 / GP1  (XIAO RX)      ★クロス
//    TSD20 pin4 RX ←── D6 / GP0  (XIAO TX)
//  I2C モード (TOI2C で切り替えた後):
//    TSD20 pin3 SCL ── D5 / GP7                 ★ストレート
//    TSD20 pin4 SDA ── D4 / GP6
//  共通:
//    TSD20 pin2 3.3V ── 3V3   (★3.3V 専用。5V を入れない。逆接・過電圧保護は無い)
//    TSD20 pin6 GND  ── GND
//
// ---- USB シリアルから使えるコマンド -------------------------------------
//  [共通]
//    HELP            このヘルプ
//    MODE            今どちらのモードでスケッチが動いているか表示
//    UART / I2C      スケッチ側の通信方法を切り替える (モジュール設定は変えない)
//  [UART モード]
//    USCAN           全ボーレートを試して測距フレームが取れる所を探す ★最初にこれ
//    UBAUD <n>       スケッチ側のボーレートを変更 (モジュールは変えない)
//    START / STOP    測距の開始 / 停止
//    VER / SN        ソフトバージョン / シリアル番号の読み出し (疎通確認)
//    FREQ <hz>       出力周波数 200/100/50/20/10/1
//    SETBAUD <n>     ★モジュール側のボーレートを永続変更
//    TOI2C           ★モジュールを I2C モードへ切り替える (永続)
//    URUN            連続表示トグル (1秒ごとに 距離/実効Hz/不正フレーム)
//    USTAT [n]       n フレームの統計 (既定 200)
//  [I2C モード]
//    PINS            SDA/SCL の電圧を実測 (バスが成立しているか)
//    SCAN [n]        n 回総当たりし、毎回応答するアドレスだけを「実在」と判定
//    ID / DUMP / RAW レジスタ読み出し
//    ON / OFF        レーザ (reg 0x02)
//    RUN / STAT [n]  連続表示 / 統計
//    TOUART          ★モジュールを UART モードへ戻す (永続)
//    ADDR/REG/LE/CLK アドレス・レジスタ・エンディアン・クロックの変更
// =========================================================================
#include <Arduino.h>
#include <Wire.h>

// ---- モジュール定数 (すべてマニュアル由来) --------------------------------
constexpr uint8_t TSD20_I2C_ADDR_DEFAULT = 0x52;   // 7bit
constexpr uint8_t REG_DIST_H   = 0x00;   // 距離 上位バイト (I2C は上位→下位)
constexpr uint8_t REG_LASER    = 0x02;   // 0:off 1:on
constexpr uint8_t REG_ID       = 0x03;   // 既定 0x4A
constexpr uint8_t REG_TO_UART0 = 0x04;   // 0xA5 と
constexpr uint8_t REG_TO_UART1 = 0x05;   //   0x5A を同時に書くと UART へ
constexpr uint8_t REG_VER_H    = 0x06;
constexpr uint8_t REG_SN_H     = 0x08;
constexpr uint8_t REG_I2C_ADDR = 0x0C;
constexpr uint8_t REG_FACTORY  = 0x0D;   // 0x01 で工場出荷状態
constexpr uint8_t TSD20_ID_EXPECT = 0x4A;

constexpr uint8_t UART_HDR = 0x5C;       // 測距フレームのヘッダ
// UART フレーム: 5C | 距離L | 距離H | cksum   (距離はリトルエンディアン)
//   範囲外のときは 50000 が返る
constexpr uint16_t UART_OUT_OF_RANGE = 50000;

// マニュアルが認めるボーレートだけ。符号は baud/100 のリトルエンディアン
// (0x0012 -> 4608 -> 460800。全 7 種を検算済み)
constexpr unsigned long BAUDS[] = {460800, 256000, 230400, 115200, 38400, 19200, 9600};
constexpr int BAUD_N = sizeof(BAUDS) / sizeof(BAUDS[0]);

// ---- 可変にしてある「前提」------------------------------------------------
bool          g_uart_mode = true;                      // 出荷時が UART なので既定は UART
unsigned long g_baud      = 460800;                    // 既定値 (マニュアル)
uint8_t       g_addr      = TSD20_I2C_ADDR_DEFAULT;
uint8_t       g_reg_dist  = REG_DIST_H;
bool          g_big_endian = true;                     // I2C は上位→下位
uint32_t      g_clk       = 100000;
bool          g_run       = false;

#define TSD Serial1   // UART0 = D6/GP0 (TX), D7/GP1 (RX)

// ===========================================================================
//  UART 側
// ===========================================================================

// マニュアルのチェックサム: 2 バイト目から最後から 2 番目までの総和の反転。
uint8_t checkSum(const uint8_t* p, uint16_t n) {
    uint8_t s = 0;
    for (uint16_t i = 0; i < n; ++i) s += p[i];
    return (uint8_t)(~s);
}

// 5A | cmd | len | payload... | cksum を組んで送る。
void sendCmd(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    uint8_t f[16];
    f[0] = 0x5A;
    f[1] = cmd;
    f[2] = len;
    for (uint8_t i = 0; i < len; ++i) f[3 + i] = payload[i];
    const uint8_t n = 3 + len;
    f[n] = checkSum(&f[1], n - 1);          // 2 バイト目から cksum の直前まで
    while (TSD.available()) TSD.read();     // 溜まった測距フレームを捨てる
    TSD.write(f, n + 1);
    TSD.flush();
}

// 応答を待って 16 進で表示する。
bool waitReply(uint32_t ms, uint8_t* out = nullptr, uint8_t* out_n = nullptr) {
    uint8_t buf[32];
    uint8_t n = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < ms && n < sizeof(buf)) {
        while (TSD.available() && n < sizeof(buf)) buf[n++] = (uint8_t)TSD.read();
    }
    if (n == 0) { Serial.println("  (応答なし)"); return false; }
    Serial.print("  返答:");
    for (uint8_t i = 0; i < n; ++i) Serial.printf(" %02X", buf[i]);
    Serial.println();
    if (out && out_n) { *out_n = n > *out_n ? *out_n : n; for (uint8_t i = 0; i < *out_n; ++i) out[i] = buf[i]; }
    return true;
}

// 測距フレームを 1 つ取り出す。ヘッダを探してチェックサムまで検証する。
//   戻り値: 1=正常フレーム / 0=まだ来ていない / -1=チェックサム不一致
int readFrame(uint16_t& mm) {
    static uint8_t buf[4];
    static uint8_t n = 0;
    while (TSD.available()) {
        const uint8_t c = (uint8_t)TSD.read();
        if (n == 0 && c != UART_HDR) continue;      // ヘッダ待ち
        buf[n++] = c;
        if (n < 4) continue;
        n = 0;
        if (checkSum(&buf[1], 2) != buf[3]) return -1;
        mm = (uint16_t)((uint16_t)buf[2] << 8 | buf[1]);   // リトルエンディアン
        return 1;
    }
    return 0;
}

void uartBegin(unsigned long baud) {
    TSD.end();
    delay(10);
    TSD.setTX(D6);
    TSD.setRX(D7);
    TSD.begin(baud);
    delay(20);
    while (TSD.available()) TSD.read();
    g_baud = baud;
}

void cmdStart() { const uint8_t p[] = {0x02, 0x00}; sendCmd(0x0A, p, 2); Serial.println("[uart] 測距開始"); waitReply(150); }
void cmdStop()  { const uint8_t p[] = {0x00, 0x00}; sendCmd(0x0A, p, 2); Serial.println("[uart] 測距停止"); waitReply(150); }
void cmdVer()   { const uint8_t p[] = {0x16, 0x16}; sendCmd(0x16, p, 2); Serial.println("[uart] バージョン読み出し (期待 5A 96 02 ...)"); waitReply(200); }
void cmdSn()    { const uint8_t p[] = {0x0D, 0x0D, 0x0D, 0x0D}; sendCmd(0x0D, p, 4); Serial.println("[uart] シリアル番号読み出し"); waitReply(200); }

void cmdFreq(int hz) {
    // 分周比 = 10000/hz - 1 をリトルエンディアンで送る (10Hz なら 999 = E7 03)
    const int allowed[] = {200, 100, 50, 20, 10, 1};
    bool ok = false;
    for (int a : allowed) if (a == hz) ok = true;
    if (!ok) { Serial.println("[uart] 対応周波数は 200/100/50/20/10/1 のみ"); return; }
    const uint16_t div = (uint16_t)(10000 / hz - 1);
    const uint8_t p[] = {(uint8_t)(div & 0xFF), (uint8_t)(div >> 8)};
    sendCmd(0x0B, p, 2);
    Serial.printf("[uart] 出力周波数 %d Hz (分周比 %u)\n", hz, div);
    waitReply(200);
}

void cmdSetBaud(long baud) {
    int idx = -1;
    for (int i = 0; i < BAUD_N; ++i) if ((long)BAUDS[i] == baud) idx = i;
    if (idx < 0) {
        Serial.print("[uart] 対応ボーレートは:");
        for (int i = 0; i < BAUD_N; ++i) Serial.printf(" %lu", BAUDS[i]);
        Serial.println("  のみ");
        return;
    }
    const uint16_t code = (uint16_t)(baud / 100);
    const uint8_t p[] = {(uint8_t)(code & 0xFF), (uint8_t)(code >> 8)};
    sendCmd(0x06, p, 2);
    Serial.printf("[uart] モジュールのボーレートを %ld に変更 (符号 %02X %02X)\n",
                  baud, p[0], p[1]);
    waitReply(200);
    Serial.println("  → スケッチ側も合わせる。UBAUD で追従するか USCAN で探し直すこと。");
}

// 全ボーレートを試して、正常な測距フレームが取れるものを探す。
// ★ モジュールのボーレートは永続設定なので、前に誰かが変えていると既定の
//   460800 では一生読めない。まずこれを打つこと。
void cmdUscan() {
    Serial.println("[uscan] 対応ボーレートを総当たりして測距フレームを探す");
    int best = -1, best_ok = 0;
    for (int i = 0; i < BAUD_N; ++i) {
        uartBegin(BAUDS[i]);
        const uint8_t p[] = {0x02, 0x00};
        sendCmd(0x0A, p, 2);                  // 測距開始 (止まっていた場合の保険)
        delay(50);
        while (TSD.available()) TSD.read();

        int ok = 0, bad = 0;
        uint16_t mm = 0, last = 0;
        const uint32_t t0 = millis();
        while (millis() - t0 < 250) {
            const int r = readFrame(mm);
            if (r == 1) { ++ok; last = mm; }
            else if (r == -1) ++bad;
        }
        Serial.printf("  %7lu baud : 正常 %3d / 不正 %3d", BAUDS[i], ok, bad);
        if (ok) Serial.printf("   最新 %u mm (%.3f m)", last, last * 0.001f);
        Serial.println();
        if (ok > best_ok) { best_ok = ok; best = i; }
    }
    if (best < 0) {
        Serial.println("  どのボーレートでも取れない。");
        Serial.println("  → 配線を疑う。UART は **クロス**:");
        Serial.println("     TSD20 pin3 TX -> XIAO D7/GP1 (RX) / pin4 RX <- XIAO D6/GP0 (TX)");
        Serial.println("     pin1 と pin5 は NC。pin2=3.3V / pin6=GND (端から 4 本と数えない)");
        Serial.println("  → それでも駄目なら既に I2C モードの可能性。I2C で SCAN を試すこと。");
        uartBegin(460800);
    } else {
        uartBegin(BAUDS[best]);
        Serial.printf("  ★ %lu baud を採用。以後はこれで動く。\n", BAUDS[best]);
    }
}

void cmdToI2c() {
    const uint8_t p[] = {0x1F, 0x1F};
    sendCmd(0x1F, p, 2);
    Serial.println("[uart] モジュールを I2C モードへ切り替え (期待 5A 9F 02 1F 1F 20)");
    waitReply(300);
    Serial.println("  ★ この設定は永続。電源を入れ直してから I2C コマンドで確認すること:");
    Serial.println("     1. 配線を I2C の **ストレート** に組み替える");
    Serial.println("        pin3 SCL -> D5/GP7 / pin4 SDA -> D4/GP6");
    Serial.println("     2. I2C  → スケッチを I2C モードへ");
    Serial.println("     3. SCAN → 0x52 が 5/5 で出れば成功");
    Serial.println("     4. ID   → 0x4A が返れば確定");
    Serial.println("  戻したくなったら I2C モードで TOUART。");
}

// ===========================================================================
//  I2C 側
// ===========================================================================
bool regWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(g_addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool regRead(uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(g_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;   // repeated start
    if (Wire.requestFrom((int)g_addr, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = (uint8_t)Wire.read();
    return true;
}

bool rawRead(uint8_t* buf, uint8_t n) {
    if (Wire.requestFrom((int)g_addr, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = (uint8_t)Wire.read();
    return true;
}

// 距離を 1 回読む。false = I2C そのものが失敗。mm=0 は「測れていない」応答
bool readMmI2c(uint16_t& mm) {
    uint8_t b[2];
    if (!regRead(g_reg_dist, b, 2)) return false;
    mm = g_big_endian ? (uint16_t)((uint16_t)b[0] << 8 | b[1])
                      : (uint16_t)((uint16_t)b[1] << 8 | b[0]);
    return true;
}

// SDA/SCL の電圧を直接読む。
// ★ SCAN が 0x08..0x77 で埋まるのは「たくさん繋がっている」ではなく
//   「SDA が LOW に張り付いている」。マスターは SDA が引かれたかでしか ACK を
//   判定できないので、SDA が GND に落ちていると全アドレスが応答して見える。
void cmdPins() {
#if !defined(PIN_WIRE_SDA) || !defined(PIN_WIRE_SCL)
    Serial.println("[pins] この板では SDA/SCL のピン番号が分からない");
#else
    const uint8_t sda = PIN_WIRE_SDA, scl = PIN_WIRE_SCL;
    Wire.end();

    pinMode(sda, INPUT);  pinMode(scl, INPUT);   delay(5);
    const bool sda_float = digitalRead(sda), scl_float = digitalRead(scl);
    pinMode(sda, INPUT_PULLUP); pinMode(scl, INPUT_PULLUP); delay(5);
    const bool sda_pu = digitalRead(sda), scl_pu = digitalRead(scl);

    Serial.printf("[pins] SDA=GP%u SCL=GP%u (アイドル時はどちらも HIGH が正常)\n", sda, scl);
    Serial.printf("  外付けPUのみ : SDA=%s SCL=%s\n", sda_float ? "HIGH" : "LOW", scl_float ? "HIGH" : "LOW");
    Serial.printf("  内蔵PUあり   : SDA=%s SCL=%s\n", sda_pu    ? "HIGH" : "LOW", scl_pu    ? "HIGH" : "LOW");

    if (!sda_pu || !scl_pu) {
        Serial.println("  ★ 内蔵PUでも LOW = そのラインが GND に落ちている。");
        Serial.println("    pin1/pin5 は NC。端から数えて GND を掴んでいないか確認すること。");
    } else if (sda_float && scl_float) {
        Serial.println("  外付けプルアップ有り。バスとしては健全。");
    } else {
        Serial.println("  外付けPU無し (内蔵の弱いPUで動作中)。400kHz は厳しい → 4.7k を 3V3 へ。");
    }
    Wire.begin();
    Wire.setClock(g_clk);
#endif
}

void cmdScan(int passes) {
    if (passes < 1)  passes = 1;
    if (passes > 20) passes = 20;

    uint8_t hit[0x78] = {0};
    for (int p = 0; p < passes; ++p) {
        for (uint8_t a = 0x08; a <= 0x77; ++a) {
            Wire.beginTransmission(a);
            if (Wire.endTransmission() == 0) ++hit[a];
        }
        delay(20);
    }

    Serial.printf("[scan] %lu kHz / %d 回\n", (unsigned long)(g_clk / 1000), passes);
    int stable = 0, flaky = 0, all = 0;
    for (uint8_t a = 0x08; a <= 0x77; ++a) {
        if (hit[a] == 0) continue;
        ++all;
        if (hit[a] == passes) ++stable; else ++flaky;
    }
    if (all > 64) {
        // 全アドレスが ACK。個別に並べても意味がないので要約する。
        Serial.printf("  0x08..0x77 のうち %d 個が応答 = **SDA が LOW に張り付いている**\n", all);
        Serial.println("  これは「たくさん繋がっている」ではなく配線不良の波形。PINS で確定させること。");
        return;
    }
    for (uint8_t a = 0x08; a <= 0x77; ++a) {
        if (hit[a] == 0) continue;
        Serial.printf("  0x%02X  %d/%d  %s", a, hit[a], passes,
                      hit[a] == passes ? "実在" : "★不安定 = 幽霊アドレスの疑い");
        if (a == TSD20_I2C_ADDR_DEFAULT) Serial.print("   <- TSD20 既定");
        if (a == 0x29) Serial.print("   <- VL53L1X");
        if (a == 0x6A || a == 0x6B) Serial.print("   <- LSM6DSV16X");
        Serial.println();
    }
    if (stable == 0 && flaky == 0) {
        Serial.println("  応答なし。");
        Serial.println("  → まだ UART モードの可能性が高い。UART に戻して USCAN を試すこと。");
        Serial.println("    (TSD20 の出荷時は UART。I2C を使うには TOI2C での切り替えが要る)");
    } else if (stable == 0) {
        Serial.println("  ★ 毎回出るアドレスが 1 つも無い = バスが電気的に成立していない。");
        Serial.println("    UART モードのモジュールの TX を SDA に繋いでいると、この症状になる。");
    }
}

void cmdId() {
    uint8_t id = 0;
    if (!regRead(REG_ID, &id, 1)) {
        Serial.printf("[id] 読み出し失敗 (アドレス 0x%02X が応答しない)\n", g_addr);
        return;
    }
    Serial.printf("[id] reg0x%02X = 0x%02X (期待 0x%02X) %s\n", REG_ID, id, TSD20_ID_EXPECT,
                  id == TSD20_ID_EXPECT ? "OK" : "<- 不一致");
}

void cmdDump() {
    // マニュアルのレジスタ定義に沿って名前を付ける
    static const char* NAME[] = {
        "距離H", "距離L", "レーザ制御(W)", "LiDAR ID", "IIC->UART(W)", "IIC->UART(W)",
        "バージョンH", "バージョンL", "シリアルH", "シリアル(sub-h)",
        "シリアル(sub-l)", "シリアルL", "IICアドレス", "工場出荷リセット(W)",
    };
    Serial.println("[dump] reg 0x00..0x0D");
    for (uint8_t r = 0x00; r <= 0x0D; ++r) {
        uint8_t v = 0;
        if (!regRead(r, &v, 1)) { Serial.printf("  0x%02X %-16s read失敗\n", r, NAME[r]); continue; }
        Serial.printf("  0x%02X %-16s 0x%02X (%3u)\n", r, NAME[r], v, v);
    }
}

void cmdRaw() {
    uint8_t b[4] = {0, 0, 0, 0};
    if (!rawRead(b, 4)) { Serial.println("[raw] 読み出し失敗"); return; }
    Serial.printf("[raw] %02X %02X %02X %02X   BE先頭2=%u mm  LE先頭2=%u mm\n",
                  b[0], b[1], b[2], b[3],
                  (unsigned)((uint16_t)b[0] << 8 | b[1]),
                  (unsigned)((uint16_t)b[1] << 8 | b[0]));
}

void cmdToUart() {
    // マニュアル: 0x04 に 0xA5、0x05 に 0x5A を「同時に」書く → 連続書き込み
    Wire.beginTransmission(g_addr);
    Wire.write(REG_TO_UART0);
    Wire.write(0xA5);
    Wire.write(0x5A);
    const bool ok = (Wire.endTransmission() == 0);
    Serial.printf("[i2c] UART モードへ切り替え %s\n", ok ? "送信済み" : "失敗");
    Serial.println("  ★ 永続設定。配線をクロスに戻し、UART → USCAN で確認すること。");
}

// ===========================================================================
//  統計 / 連続表示 (モード共通)
// ===========================================================================
// レートは 2 つ出す。意味が違うので混ぜないこと:
//   受信レート : 実際に届いたサンプル数/秒。これが本当の測距レート (公称 200Hz)
//   値更新レート: 前回と値が変わった回数/秒。静止した的に向けると当然下がる
//
// ★ 2026-09-18 実測で分かった落とし穴:
//   used/tsd20_backend.patch の readSlant() は「前回と同じ値 = 新サンプルではない」
//   で新旧を判定し、新しい値が RANGE_TSD20_STALE_MS(200ms) 来なければ失探にする。
//   だが TSD20 は安定した的では同じ mm を返し続ける (実測で値更新が 21回/秒 まで低下)。
//   **高度が安定しているときほど失探しやすい** という逆立ちした挙動になるので、
//   機体に入れるときは「読めたら毎回新しいサンプル」として扱うこと
//   (FC の測距ループは RANGE_LOOP_HZ でセンサより十分遅い)。
void cmdStat(int n) {
    if (n < 10)   n = 10;
    if (n > 5000) n = 5000;
    Serial.printf("[stat] %d サンプル収集中... (センサを動かさないこと)\n", n);

    int      bad = 0, zero = 0, oor = 0, used = 0, changed = 0;
    uint16_t mn = 0xFFFF, mx = 0, prev = 0xFFFF;
    double   sum = 0.0, sum2 = 0.0;
    const uint32_t t0 = millis();
    int got = 0;

    while (got < n && millis() - t0 < 15000) {
        uint16_t mm;
        if (g_uart_mode) {
            const int r = readFrame(mm);
            if (r == 0) continue;
            if (r < 0)  { ++bad; ++got; continue; }
        } else {
            if (!readMmI2c(mm)) { ++bad; ++got; delay(2); continue; }
            delay(2);
        }
        ++got;
        if (mm != prev) { ++changed; prev = mm; }
        if (mm == UART_OUT_OF_RANGE) { ++oor; continue; }
        if (mm == 0) { ++zero; continue; }
        ++used;
        sum  += mm;
        sum2 += (double)mm * mm;
        if (mm < mn) mn = mm;
        if (mm > mx) mx = mm;
    }
    const uint32_t dt = millis() - t0;

    if (used < 2) {
        Serial.printf("[stat] 有効サンプルなし (不正 %d / 0mm %d / 範囲外 %d)\n", bad, zero, oor);
        return;
    }
    const double mean = sum / used;
    double var = sum2 / used - mean * mean;
    if (var < 0) var = 0;
    const double sd = sqrt(var);

    Serial.printf("[stat] n=%d  %lu ms  (%s)\n", got, (unsigned long)dt, g_uart_mode ? "UART" : "I2C");
    Serial.printf("  平均      %.1f mm  (%.3f m)\n", mean, mean * 0.001);
    Serial.printf("  標準偏差  %.1f mm   ★繰り返し精度の公称は ±10mm\n", sd);
    Serial.printf("  最小/最大 %u / %u mm  (幅 %u mm)\n",
                  (unsigned)mn, (unsigned)mx, (unsigned)(mx - mn));
    Serial.printf("  欠測      0mm %d / 範囲外(50000) %d / 不正 %d  (%.1f%%)\n",
                  zero, oor, bad, got ? 100.0 * (zero + oor + bad) / got : 0.0);
    Serial.printf("  実効レート %.1f Hz\n", dt ? 1000.0 * changed / dt : 0.0);
    Serial.println("  ★ 巻尺の実測値と並べて COMPETITION_OPEN_ISSUES.md の A1 に記録すること。");
    Serial.printf("     公称精度は ±5cm(<5m)。3.6m で平均がこれを超えてずれるなら要注意。\n");
}

// ===========================================================================
//  コマンド処理
// ===========================================================================
void printMode() {
    if (g_uart_mode)
        Serial.printf("[mode] UART %lu baud  (TSD20 pin3 TX -> D7 / pin4 RX <- D6 ★クロス)\n", g_baud);
    else
        Serial.printf("[mode] I2C addr=0x%02X %lukHz reg=0x%02X %s  (pin3 SCL -> D5 / pin4 SDA -> D4 ★ストレート)\n",
                      g_addr, (unsigned long)(g_clk / 1000), g_reg_dist, g_big_endian ? "BE" : "LE");
}

void printHelp() {
    Serial.println("---- TSD20 ベンチ試験 ----");
    Serial.println(" [共通]   HELP / MODE / UART / I2C");
    Serial.println(" [UART]   USCAN ★最初にこれ / UBAUD <n> / START / STOP / VER / SN");
    Serial.println("          FREQ <hz> / SETBAUD <n> / TOI2C / URUN / USTAT [n]");
    Serial.println(" [I2C ]   PINS / SCAN [n] / ID / DUMP / RAW / ON / OFF / TOUART");
    Serial.println("          RUN / STAT [n] / ADDR <hex> / REG <hex> / LE / CLK 100|400");
    printMode();
}

void applyClock(uint32_t hz) { g_clk = hz; Wire.setClock(hz); Serial.printf("[cfg] I2C %lu kHz\n", (unsigned long)(hz / 1000)); }

void handleLine(String s) {
    s.trim();
    if (s.length() == 0) return;
    String u = s; u.toUpperCase();

    // --- 共通 ---
    if      (u == "HELP" || u == "?") printHelp();
    else if (u == "MODE") printMode();
    else if (u == "UART") { g_uart_mode = true;  uartBegin(g_baud); printMode(); }
    else if (u == "I2C")  { g_uart_mode = false; Wire.begin(); Wire.setClock(g_clk); printMode(); }
    // --- UART ---
    else if (u == "USCAN")  cmdUscan();
    else if (u == "START")  cmdStart();
    else if (u == "STOP")   cmdStop();
    else if (u == "VER")    cmdVer();
    else if (u == "SN")     cmdSn();
    else if (u == "TOI2C")  cmdToI2c();
    else if (u == "URUN")   { g_run = !g_run; Serial.printf("[run] %s\n", g_run ? "開始" : "停止"); }
    else if (u.startsWith("USTAT"))   cmdStat(u.length() > 5 ? u.substring(5).toInt() : 200);
    else if (u.startsWith("UBAUD"))   { uartBegin(u.substring(5).toInt()); printMode(); }
    else if (u.startsWith("SETBAUD")) cmdSetBaud(u.substring(7).toInt());
    else if (u.startsWith("FREQ"))    cmdFreq(u.substring(4).toInt());
    // --- I2C ---
    else if (u == "PINS")   cmdPins();
    else if (u == "ID")     cmdId();
    else if (u == "DUMP")   cmdDump();
    else if (u == "RAW")    cmdRaw();
    else if (u == "TOUART") cmdToUart();
    else if (u == "ON")     Serial.println(regWrite(REG_LASER, 1) ? "[laser] ON"  : "[laser] 書き込み失敗");
    else if (u == "OFF")    Serial.println(regWrite(REG_LASER, 0) ? "[laser] OFF" : "[laser] 書き込み失敗");
    else if (u == "LE")     { g_big_endian = !g_big_endian; printMode(); }
    else if (u == "RUN")    { g_run = !g_run; Serial.printf("[run] %s\n", g_run ? "開始" : "停止"); }
    else if (u.startsWith("SCAN")) cmdScan(u.length() > 4 ? u.substring(4).toInt() : 5);
    else if (u.startsWith("STAT")) cmdStat(u.length() > 4 ? u.substring(4).toInt() : 200);
    else if (u.startsWith("CLK"))  applyClock(u.substring(3).toInt() >= 400 ? 400000 : 100000);
    else if (u.startsWith("ADDR")) { g_addr = (uint8_t)strtol(u.substring(4).c_str(), nullptr, 16); printMode(); }
    else if (u.startsWith("REG"))  { g_reg_dist = (uint8_t)strtol(u.substring(3).c_str(), nullptr, 16); printMode(); }
    else Serial.printf("[?] 不明なコマンド: %s   (HELP)\n", s.c_str());
}

void setup() {
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { }

    Wire.begin();
    Wire.setClock(g_clk);
    uartBegin(g_baud);

    Serial.println();
    Serial.println("=== TSD20 ベンチ試験 (機体コードには影響しません) ===");
    Serial.println("★ TSD20 の出荷時は UART モード。I2C で話すには TOI2C での切り替えが要る。");
    Serial.println("  配線 6pin: 1=NC 2=3.3V 3=TX(SCL) 4=RX(SDA) 5=NC 6=GND");
    Serial.println("  UART はクロス (pin3->D7 / pin4<-D6) / I2C はストレート (pin3->D5 / pin4->D4)");
    printHelp();
    Serial.println();
    cmdUscan();
}

void loop() {
    static String line;
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { handleLine(line); line = ""; }
        else if (line.length() < 64) line += c;
    }

    static uint32_t t_print = 0;
    static uint16_t prev = 0xFFFF;
    static int      n = 0, changed = 0, zero = 0, oor = 0, bad = 0;
    static uint32_t sum = 0;

    if (g_run) {
        uint16_t mm;
        bool got = false;
        if (g_uart_mode) {
            const int r = readFrame(mm);
            if (r == 1) got = true;
            else if (r == -1) ++bad;
        } else {
            if (readMmI2c(mm)) got = true; else ++bad;
            delay(2);
        }
        if (got) {
            ++n;
            if (mm != prev) { ++changed; prev = mm; }
            if (mm == UART_OUT_OF_RANGE) ++oor;
            else if (mm == 0) ++zero;
            else sum += mm;
        }

        if (millis() - t_print >= 1000) {
            const int valid = n - zero - oor;
            Serial.printf("[run] %.3f m  実効 %d Hz  欠測 0mm %d / 範囲外 %d / 不正 %d  (受信 %d)\n",
                          valid > 0 ? (sum / (double)valid) * 0.001 : 0.0,
                          changed, zero, oor, bad, n);
            t_print = millis();
            n = changed = zero = oor = bad = 0;
            sum = 0;
        }
    } else {
        t_print = millis();
        n = changed = zero = oor = bad = 0;
        sum = 0;
    }
}
