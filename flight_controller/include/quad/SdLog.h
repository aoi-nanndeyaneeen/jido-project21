// ============================================================
//  SdLog.h  -  飛行中に「生バイト列」を SD へストリーミングする
//              (drone_s5.cpp 専用)。CSV 化は PC で scripts/bin2csv.py。
// ============================================================
//  この機体は 1000Hz の制御ループを数十ms止めると落ちる。だから SD への
//  書き込みは飛行中の制御ループから絶対に同期で呼ばない。
//
//   push(rec, n)  … DMAMEM リングバッファへ memcpy するだけ。SD には触れない。
//                   リングが一杯なら「そのレコードを捨てて dropped を +1」。
//                   ブロックも失敗もしない。毎サンプル呼ぶ。
//   service()     … 毎ループ呼ぶ。溜まったぶんをセクタ境界で、1回あたり
//                   最大 MAX_WRITE バイトだけ書く。カードが GC 等で詰まっても
//                   1回の待ち時間が有界。溢れたぶんは push() 側で落ちる
//                   (= リング容量ぶんの一時的なカード停止までは耐える。
//                      それを超えたら「制御を止める」より「ログを捨てる」)。
//   startFile()   … アームした瞬間に呼ぶ。open + preAllocate で数十ms
//                   ブロックする。離陸前なので許容。
//   stopFile()    … ディスアームで呼ぶ。リングを吐き切って truncate/close。
//
//  SPI0 は PMW3901 と共有する。SdFat を SHARED_SPI 指定で使うと、各アクセスを
//  SPI.beginTransaction / endTransaction で括る。Bitcraze の PMW3901
//  ライブラリも同じく毎アクセスで括り、CS を分けている
//  (PMW3901 CS=10 / SD CS=呼び出し時指定・drone_s5 では 9) ので共存できる。
//
//  バイナリ形式  LOGnnnn.BIN:
//    [32B ヘッダ]  "S5LOG\0" | u8 fmt_ver | u8 rec_ver |
//                  u16 rec_size | u16 rate_hz | u32 t0_ms | 残り 0 埋め
//    [rec_size バイトのレコード] × N        (中身は呼び出し側の構造体そのまま)
//
//  ★ レコード構造体 (FlightLog::Rec) の列を足したら FlightLog::REC_VER を +1 する。
//    scripts/bin2csv.py は rec_size / rec_ver 不一致を弾く。
// ============================================================
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

#ifndef DMAMEM
#define DMAMEM
#endif

namespace SdLog {

// ---- チューニング定数 --------------------------------------------------
constexpr uint8_t  FMT_VER    = 1;
// DMAMEM(RAM2, 512KB)に置く。2026-09-12 に本体RAMへの飛行中バッファリング
// (旧 FlightLog::Ram) を廃止したので、RAM2 はほぼこのリングバッファ専用。
// 32KB で約 0.6 秒ぶん (Rec≈125B × 500Hz) のカード停止を吸収できる。
constexpr size_t   RING_BYTES = 32u * 1024u;
constexpr size_t   MAX_WRITE  = 2u * 1024u;    // service() 1回で書く上限 [byte]
constexpr size_t   SECTOR     = 512u;
constexpr uint64_t PREALLOC   = 16ull * 1024 * 1024;  // ≒300秒。超えたら普通の追記に落ちる
constexpr uint32_t SCK_MHZ    = 16;            // HW-125 + 手配線。安定したら 25 まで
constexpr uint32_t SYNC_US    = 2000000u;      // 記録中に FAT を保存する間隔 (電源断保険)

// ---- 内部状態 (このヘッダを include するのは drone_s5.cpp だけなので static) --
DMAMEM static uint8_t s_ring[RING_BYTES];
static SdFs   s_sd;
static FsFile s_file;

static bool     s_ok        = false;   // begin() が成功したか
static bool     s_recording = false;   // startFile()〜stopFile() の間
static uint16_t s_rec_size  = 0;
static uint8_t  s_rec_ver   = 0;
static uint16_t s_rate_hz   = 0;

static size_t   s_head = 0, s_tail = 0, s_count = 0;   // リング (バイト単位)
static uint32_t s_next_idx  = 0;       // 次に作る LOGnnnn の番号
static char     s_cur_name[16] = {0};  // 今 (or 直近) 開いていた BIN 名

static uint64_t s_bytes     = 0;       // 現ファイルへ書いた総バイト
static uint32_t s_dropped   = 0;       // リング溢れで捨てたレコード数
static uint32_t s_worst_us  = 0;       // service() の write 1回の最悪所要
static uint32_t s_werr      = 0;       // write が途中で切れた回数
static uint32_t s_operr     = 0;       // open 失敗回数
static uint32_t s_last_sync_us = 0;

// ---- 小物 ----------------------------------------------------------------
inline void resetRing() { s_head = s_tail = s_count = 0; }

inline void scanNextIndex() {
    char name[16];
    for (uint32_t i = 0; i < 10000; ++i) {
        snprintf(name, sizeof(name), "LOG%04lu.BIN", (unsigned long)i);
        if (!s_sd.exists(name)) { s_next_idx = i; return; }
    }
    s_next_idx = 0;   // 全部埋まっていたら 0 から上書き
}

// ============================================================
//  begin  -  setup() で1回。CS ピンとレコード形式を渡す。
// ============================================================
inline bool begin(uint8_t cs_pin, uint16_t rec_size, uint8_t rec_ver, uint16_t rate_hz) {
    s_rec_size = rec_size;
    s_rec_ver  = rec_ver;
    s_rate_hz  = rate_hz;
    s_ok = s_sd.begin(SdSpiConfig(cs_pin, SHARED_SPI, SD_SCK_MHZ(SCK_MHZ)));
    if (s_ok) scanNextIndex();
    return s_ok;
}

inline bool ok()        { return s_ok; }
inline bool recording() { return s_recording; }

// ============================================================
//  startFile  -  アームした瞬間に。新しい LOGnnnn.BIN を開く。
// ============================================================
inline void startFile() {
    if (!s_ok) return;
    if (s_recording) {   // 前回を閉じ忘れていたら閉じる
        s_file.truncate(s_bytes); s_file.sync(); s_file.close(); s_recording = false;
    }
    resetRing();

    char name[16];
    snprintf(name, sizeof(name), "LOG%04lu.BIN", (unsigned long)s_next_idx);
    if (!s_file.open(name, O_WRONLY | O_CREAT | O_TRUNC)) { s_operr++; return; }
    s_file.preAllocate(PREALLOC);   // 連続領域を確保 → 書き込みレイテンシが安定

    uint8_t hdr[32];
    memset(hdr, 0, sizeof(hdr));
    memcpy(hdr, "S5LOG", 5);                 // hdr[5] = '\0'
    hdr[6]  = FMT_VER;
    hdr[7]  = s_rec_ver;
    hdr[8]  = (uint8_t)(s_rec_size & 0xFF);
    hdr[9]  = (uint8_t)(s_rec_size >> 8);
    hdr[10] = (uint8_t)(s_rate_hz & 0xFF);
    hdr[11] = (uint8_t)(s_rate_hz >> 8);
    const uint32_t t0 = millis();
    memcpy(hdr + 12, &t0, 4);
    s_file.write(hdr, sizeof(hdr));

    s_bytes        = sizeof(hdr);
    s_dropped      = 0;
    s_worst_us     = 0;
    s_werr         = 0;
    s_recording    = true;
    s_last_sync_us = micros();
    strcpy(s_cur_name, name);
    s_next_idx++;
}

// ============================================================
//  push  -  1レコードをリングへ積むだけ。ブロックしない。
// ============================================================
inline void push(const void* rec, size_t n) {
    if (!s_recording) return;
    if (n == 0) return;
    if (s_count + n > RING_BYTES) { s_dropped++; return; }   // 溢れ → 捨てる

    const uint8_t* p = (const uint8_t*)rec;
    size_t first = RING_BYTES - s_head;
    if (first > n) first = n;
    memcpy(s_ring + s_head, p, first);
    if (n > first) memcpy(s_ring, p + first, n - first);
    s_head  = (s_head + n) % RING_BYTES;
    s_count += n;
}

// ============================================================
//  service  -  毎ループ。溜まったぶんをセクタ境界で少しだけ書く。
// ============================================================
inline void service() {
    if (!s_recording) return;

    if (s_count >= SECTOR) {
        size_t chunk = (s_count / SECTOR) * SECTOR;   // 完全セクタのみ
        if (chunk > MAX_WRITE) chunk = MAX_WRITE;

        size_t first = RING_BYTES - s_tail;
        if (first > chunk) first = chunk;

        const uint32_t t0 = micros();
        size_t w = s_file.write(s_ring + s_tail, first);
        if (w == first && chunk > first) w += s_file.write(s_ring, chunk - first);
        const uint32_t dt = micros() - t0;
        if (dt > s_worst_us) s_worst_us = dt;

        s_tail  = (s_tail + w) % RING_BYTES;
        s_count -= w;
        s_bytes += w;
        if (w != chunk) s_werr++;
    }

    // 電源断保険。FAT/dir を定期保存する (数ms〜十数ms かかるが飛行中でも
    // 有界。落ちても直前 SYNC_US ぶんだけの損失で済む)。
    const uint32_t now = micros();
    if (now - s_last_sync_us >= SYNC_US) {
        s_last_sync_us = now;
        s_file.sync();
    }
}

// ============================================================
//  stopFile  -  ディスアームで。リングを吐き切って閉じる。
// ============================================================
inline void stopFile() {
    if (!s_recording) return;

    // 残りを全部書く (セクタ端数も含む)。ここは地上なのでブロック可。
    uint32_t guard = 0;
    while (s_count > 0 && guard++ < 4096) {
        size_t first = RING_BYTES - s_tail;
        if (first > s_count) first = s_count;
        size_t w = s_file.write(s_ring + s_tail, first);
        s_tail  = (s_tail + w) % RING_BYTES;
        s_count -= w;
        s_bytes += w;
        if (w != first) { s_werr++; break; }
    }
    s_file.truncate(s_bytes);   // preAllocate の未使用ぶんを解放
    s_file.sync();
    s_file.close();
    s_recording = false;
}

// printStatus() のライブ画面用の 1 行。
inline void brief(Print& out) {
    if (!s_ok) { out.println("SD: NG"); return; }
    out.printf("SD: %s  %s  %lu KB  drop=%lu  worstW=%lu us\n",
               s_recording ? "REC" : "idle",
               s_cur_name[0] ? s_cur_name : "-",
               (unsigned long)(s_bytes / 1024), (unsigned long)s_dropped,
               (unsigned long)s_worst_us);
}

// ============================================================
//  selftest  -  SD 単体の詳細チェック (地上・非記録時のみ)。
//    再 begin して カード種別/容量/FAT を出し、小さいファイルを
//    書いて読み戻して照合する。失敗時は SdFat のエラーコードを出す。
//    シリアル 's' から呼ぶ (記録中は status() だけ)。
// ============================================================
inline void selftest(Print& out) {
    if (s_recording) { out.println("SD: 記録中。テストはディスアーム後に"); return; }

    // ★ re-init + ファイル書込/読戻で ~100ms ブロックする。's' を押しっぱなし
    //   にすると毎ループ走って制御ループが 7Hz まで落ちる。3秒に1回だけにする。
    static uint32_t last_ms = 0;
    if (last_ms != 0 && (millis() - last_ms) < 3000) {
        out.println("SD: selftest クールダウン中 (3秒に1回)");
        return;
    }
    last_ms = millis();

    out.println("--- SD selftest ---");
    s_sd.end();
    const uint8_t cs = 9;   // drone_s5 の SD CS
    if (!s_sd.begin(SdSpiConfig(cs, SHARED_SPI, SD_SCK_MHZ(SCK_MHZ)))) {
        out.printf("begin 失敗 @CS=%u  SCK=%luMHz\n", cs, (unsigned long)SCK_MHZ);
        s_sd.initErrorPrint(&out);      // "No card, wrong chip select, or wiring?" 等
        out.printf("  errorCode=0x%02X  errorData=0x%02X\n",
                   s_sd.sdErrorCode(), s_sd.sdErrorData());
        out.println("  0x01=CMD0(無応答/CS/配線) 0x0B前後=ACMD41(電圧/相性) "
                    "fatType=0 は未フォーマット/非FAT32");
        s_ok = false;
        return;
    }
    s_ok = true;

    out.print("  fatType="); s_sd.printFatType(&out);
    uint32_t sectors = 0;
    if (s_sd.card()) sectors = s_sd.card()->sectorCount();
    out.printf("  容量=%lu MB\n", (unsigned long)(sectors / 2048));

    // 書き込み→読み戻し照合
    char wbuf[48];
    snprintf(wbuf, sizeof(wbuf), "SDTEST t=%lu ok", (unsigned long)millis());
    FsFile f;
    if (!f.open("SDTEST.TXT", O_WRONLY | O_CREAT | O_TRUNC)) {
        out.println("  書き込み open 失敗 (書込禁止 or FS 破損)");
        return;
    }
    f.print(wbuf);
    f.sync();
    f.close();

    char rbuf[48] = {0};
    if (!f.open("SDTEST.TXT", O_RDONLY)) { out.println("  読み戻し open 失敗"); return; }
    int n = f.read(rbuf, sizeof(rbuf) - 1);
    f.close();
    if (n > 0) rbuf[n] = 0;

    if (n == (int)strlen(wbuf) && strcmp(wbuf, rbuf) == 0) {
        out.printf("  書込/読戻 PASS: \"%s\"\n", rbuf);
        out.println("  → SD 単体は正常。共有バスでダメなら HW-125 の MISO バッファが犯人");
    } else {
        out.printf("  書込/読戻 FAIL: 書=\"%s\" 読=\"%s\" (n=%d)\n", wbuf, rbuf, n);
    }
    scanNextIndex();   // s_sd.end()/begin() で状態が変わったので採番し直す
}

// ============================================================
//  status  -  シリアル 's' で状態表示
// ============================================================
inline void status() {
    Serial.printf("SD: %s  次ファイル=LOG%04lu.BIN  記録中=%d\n",
                  s_ok ? "OK" : "NG (CS=9/VCC=5V/配線/FAT32 を確認)",
                  (unsigned long)s_next_idx, s_recording ? 1 : 0);
    Serial.printf("    直近 file=%s  書込=%lu B  drop(rec)=%lu  最悪write=%lu us  "
                  "write_err=%lu  open_err=%lu  ring=%lu/%lu B\n",
                  s_cur_name[0] ? s_cur_name : "(none)",
                  (unsigned long)s_bytes, (unsigned long)s_dropped,
                  (unsigned long)s_worst_us, (unsigned long)s_werr, (unsigned long)s_operr,
                  (unsigned long)s_count, (unsigned long)RING_BYTES);
}

} // namespace SdLog
