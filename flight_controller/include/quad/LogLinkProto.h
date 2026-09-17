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
//    デバッグ    : RP2040 -> T_ACT (BLE Write を中継。PID reset/IMU校正/
//                  デバイス確認のみ)。Teensy -> T_ACT_ACK (実行結果)
//    地上局リンク: (2026-09-17 追加。drone_s5.cpp の S5::GROUND_LINK == BLE のとき)
//                  Teensy -> T_TELEM (S5Telem.h のフレームを束ねたもの)
//                  ロガー -> T_CMD   (S5Cmd.h の CmdFrame。BLE Write を中継)
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
constexpr uint8_t T_ACT_ACK = 0x04; // payload = ActAck (下記)。単発。定期送信ではない
constexpr uint8_t T_TELEM = 0x05;   // payload = S5T の各フレームを連結したもの (下記)
// ロガー -> FC
constexpr uint8_t T_STAT  = 0x81;   // payload = Stat
constexpr uint8_t T_ACT   = 0x82;   // payload = ActReq (下記)。単発。BLE から来る
constexpr uint8_t T_CMD   = 0x83;   // payload = S5C::CmdFrame (22B) そのもの。BLE から来る

// ------------------------------------------------------------
//  BLE 経由の単発メンテナンス指令 (2026-09-14 追加)
// ------------------------------------------------------------
//  PC ─BLE Write→ ロガー(XIAO) ─UART(この T_ACT)→ FC ─UART(T_ACT_ACK)→
//  ロガー ─BLE Notify→ PC、という往復。デバッグ用 (IMU再校正/デバイス確認/
//  PIDリセット) 専用の経路で、S5Cmd.h の Action をそのまま運ぶ。
//
//  ★ このフレームには action と action_seq しか無い。操縦系 (速度・高度・
//    離着陸要求) はここには絶対に混ぜない。
//    2026-09-17 に地上局リンク (操縦 + テレメトリ) を IM920 から BLE へ
//    移したが、それは下の T_CMD / T_TELEM という「別のフレーム型・別の
//    BLE characteristic」で運ぶ。デバッグ用の単発指令と操縦指令を同じ
//    口に入れないことで、ble_monitor.py のようなデバッグツールからは
//    今までどおり構造的に操縦できないままにしてある。
//
//  action_seq の重複排除は FC 側 (drone_s5.cpp handleBleAction()) が
//  S5Cmd.h の CmdFrame.action_seq と同じ流儀 (値が変わった最初の1回だけ
//  実行) でやる。IM920 経由の action_seq とは別カウンタで独立に見る。
struct __attribute__((packed)) ActReq {
    uint8_t action;       // S5Cmd.h の Action (PID_RESET/IMU_CAL/SELFTEST)
    uint8_t action_seq;   // 1..255。0 は「無視される」
};

// 上記の実行結果。S5Telem.h の AckResult をそのまま使う。
struct __attribute__((packed)) ActAck {
    uint8_t action;
    uint8_t action_seq;
    uint8_t result;        // S5Telem.h の AckResult
    uint8_t imu_ok;        // SELFTEST のときだけ意味を持つ
    uint8_t i2c_found;     // 同上
};

// ------------------------------------------------------------
//  地上局リンク (IM920 の代わり。2026-09-17 追加)
// ------------------------------------------------------------
//  IM920 で運んでいたものを「中身はそのまま・運び方だけ」差し替える。
//    下り T_TELEM : S5Telem.h の AltFrame/PosFrame/AttFrame/DvFrame/ParamFrame
//                   を 1 個以上そのまま連結したもの。各フレームの先頭 1B が
//                   type なので、S5T::payloadBytesFor(type) で切り分けられる。
//                   IM920 の 32B 制限が無いので 1 フレームに複数枚束ねて、
//                   BLE Notify の回数 (= 帯域) を節約する。
//                   IM920 版に付いていた 4B チェックサムは付けない
//                   (LogLink の CRC8 が同じ役をする)。
//    上り T_CMD   : S5Cmd.h の CmdFrame (22B) そのもの。seq はロガーが振り直す
//                   (IM920 版で地上局 XIAO が振っていたのと同じ役割分担)。
//  ★ 中身の構造体は IM920 と完全に共通。S5::GROUND_LINK を IM920 に戻せば
//    旧経路 (ground_receiver の xiao_s5_log) がそのまま動く。
constexpr size_t  TELEM_MAX_PAYLOAD = 255;
constexpr size_t  CMD_PAYLOAD       = 22;    // sizeof(S5C::CmdFrame)。S5Cmd.h 側で static_assert

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
