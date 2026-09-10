// ============================================================
//  LogLinkProto.h  -  Teensy(FC) <-> RP2040(ロガー) の UART フレーム定義
// ============================================================
//  ★ このファイルは「両方のプロジェクトが同じものを include する唯一の定義」。
//    片方だけ直すと必ず壊れる。log_recorder/platformio.ini は
//      build_flags = -I ../flight_controller/include
//    で、この include/quad/ を直接見にいっている (コピーを作らないこと)。
//
//  【背景】
//    SD (HW-125) と PMW3901 が Teensy の SPI0 を食い合うため、
//    drone_s5 では USE_FLOW と USE_SD が排他になっていた
//    (src/drone_s5.cpp の static_assert)。SD 書き込みを RP2040 に
//    丸ごと引っ越すと、この排他が消えて「フロー使用中でもフルログ」が録れる。
//
//  【フレーム】
//     0xA5 0x5A | type(1) | len(1) | seq(1) | payload(len) | crc8(1)
//     crc8 は type,len,seq,payload を対象 (SOF は含まない)。多項式 0x07。
//     len は 0..255。REC の payload は FlightLog::Rec (116B) そのもの。
//
//  【なぜ seq があるか】
//    UART は 1 バイト落ちるとその後ろが全部ズレる。SOF+CRC で再同期はできるが
//    「何レコード失ったか」が分からない。seq の飛びを数えて、地上で
//    「このログは信用してよいか」を判断できるようにしている。
//
//  【流れ】
//    アーム      : Teensy -> T_START (32B の BIN ヘッダ入り)
//    飛行中      : Teensy -> T_REC を 500Hz。加えて T_START を 1Hz で再送
//                  (ロガーが後から電源投入されても拾えるように。ヘッダ内の
//                   t0_ms が同じなら RP2040 は無視する)
//    ディスアーム: Teensy -> T_STOP
//    保険        : T_REC が IDLE_CLOSE_MS 途切れたら RP2040 が勝手に閉じる
//                  (STOP が化けても BIN が壊れたまま残らない)
//    逆方向      : RP2040 -> T_STAT を 2Hz。Teensy の 's' 表示に出る
// ============================================================
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace LogLinkProto {

constexpr uint8_t SOF1 = 0xA5;
constexpr uint8_t SOF2 = 0x5A;

// FC -> ロガー
constexpr uint8_t T_START = 0x01;   // payload = 32B の BIN ヘッダ ("S5LOG"...)
constexpr uint8_t T_REC   = 0x02;   // payload = FlightLog::Rec
constexpr uint8_t T_STOP  = 0x03;   // payload なし
// ロガー -> FC
constexpr uint8_t T_STAT  = 0x81;   // payload = Stat

constexpr size_t  HDR_LEN     = 5;    // SOF1 SOF2 type len seq
constexpr size_t  OVERHEAD    = HDR_LEN + 1;          // + crc
constexpr size_t  MAX_PAYLOAD = 255;
constexpr size_t  MAX_FRAME   = OVERHEAD + MAX_PAYLOAD;

constexpr size_t  BIN_HDR_LEN = 32;   // SdLog.h と同じ 32B ヘッダ
constexpr size_t  BIN_HDR_T0_OFS = 12;  // ヘッダ内の t0_ms の位置 (ファイル識別に使う)

// ロガーが 2Hz で返す状態。Teensy 側は 's' と printStatus() で出す。
struct __attribute__((packed)) Stat {
    uint8_t  flags;        // bit0: SD 初期化 OK  bit1: 記録中
    uint8_t  ring_pct;     // ロガー内リングの使用率 [%]
    uint16_t file_idx;     // 今 (or 次) の LOGnnnn の n
    uint32_t kbytes;       // 現ファイルへ書いた [KB]
    uint32_t drop_rec;     // リング溢れで捨てたレコード数
    uint32_t crc_err;      // CRC 不一致フレーム数
    uint32_t seq_gap;      // seq 飛び (UART 取りこぼし) の合計レコード数
    uint32_t worst_w_us;   // SD write 1 回の最悪所要 [us]
};
constexpr uint8_t STAT_SD_OK  = 1u << 0;
constexpr uint8_t STAT_REC    = 1u << 1;

// ロガーが T_REC の途切れを検出して勝手にファイルを閉じるまでの時間。
// STOP フレームが化けても BIN が open のまま残らないための保険。
constexpr uint32_t IDLE_CLOSE_MS = 2000;

// ---- CRC8 (多項式 0x07, 初期値 0x00) --------------------------------------
//  1 フレーム最大 261 バイト、500Hz。ビット演算のままで両者とも余裕がある
//  (Teensy 600MHz / RP2040 125MHz)。テーブルは持たない。
inline uint8_t crc8(const uint8_t* p, size_t n, uint8_t crc = 0) {
    while (n--) {
        crc ^= *p++;
        for (uint8_t i = 0; i < 8; ++i)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

// ---- フレーム組み立て -----------------------------------------------------
//  out は MAX_FRAME バイト以上。戻り値 = 実際の長さ。
inline size_t buildFrame(uint8_t* out, uint8_t type, uint8_t seq,
                         const void* payload, size_t len) {
    if (len > MAX_PAYLOAD) return 0;
    out[0] = SOF1;
    out[1] = SOF2;
    out[2] = type;
    out[3] = (uint8_t)len;
    out[4] = seq;
    if (len) __builtin_memcpy(out + HDR_LEN, payload, len);
    out[HDR_LEN + len] = crc8(out + 2, len + 3);   // type,len,seq,payload
    return OVERHEAD + len;
}

} // namespace LogLinkProto
