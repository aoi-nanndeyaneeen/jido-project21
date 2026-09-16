// ============================================================
//  S5Telem.h  -  s5 (POSHOLD) 解析用 IM920SL テレメトリ
// ============================================================
//  目的:
//    実飛行中は USB を挿せないので、drone_s5.cpp の 500Hz ログ
//    (Log:: 名前空間) が取れない。その代わりに「POSHOLD の解析に
//    必要な信号だけ」を無線で落とし、地上局側で CSV にする。
//    姿勢(数十Hz)の解析はできないが、高度・位置ホールドのループは
//    1〜2Hz の現象なので、これで足りる。
//
//  ★ このファイルはリポジトリ直下の protocol/ にあり、flight_controller と
//    ground_receiver の両方が platformio.ini の -I ../protocol で同じ実体を
//    読む (以前は両プロジェクトに手コピーしていた)。Python 側の定数は
//      python protocol/gen_py_protocol.py
//    で position_estimator/src/core/s5_protocol.py に生成する。
//    構造体を変えたら VERSION を上げ、生成スクリプトを再実行すること。
//    ずれの検出用に VERSION と static_assert(sizeof) を入れてある。
//
// ------------------------------------------------------------
//  【実測した制約】 2026-09-04 実機で測定
//
//  1. IM920sL の実効ペイロードは 32 バイト。
//     33 バイト以上を TXDA すると、モジュールは OK を返すのに
//     受信側には先頭 32 バイトしか届かない (黙って切り詰められる)。
//     ※ Config.h に「64-4=60バイト」とあるのは誤り。あれは IM920 /
//       IM920s の値で、IM920sL は 32 バイト。旧 PlaneData(28+4=32) が
//       たまたま上限ちょうどだったので動いていただけ。
//       GroundData(42+4=46) は最初から切り詰められている。
//     → データ 28 バイト + チェックサム 4 バイト = 32 バイト厳守。
//
//  2. 持続できるパケットレートは 15Hz まで。
//       10Hz 欠落 0% / 12Hz 0% / 15Hz 0% / 20Hz 5% / 25Hz 22.5%
//     (32バイト = "TXDA " + 64桁 + CRLF = 71文字 = 19200bps で 37ms。
//      15Hz で UART 占有率 55%)
//     → drone_s5.cpp の TELEM_TX_HZ = 15。
//
//  この 2 つから、1 パケットに全部は載らない。そこでフレームを分けて
//  順番に送る。0.3〜2Hz のループを見るにはこれで十分。
//    A = 高度ループ + 姿勢角
//    B = 水平位置ループ
//    C = 姿勢ループの内部 (モーター出力 / 角速度 実測・目標 / ミキサー飽和)
//    D = ヨー推定用の機体Δv (地上局ガイド飛行のヨー保持に使う)
//    P = ゲイン一覧 (数秒に1回、枠を1つ borrow する)
//
//  ★ 送る順番はモードで変える (quad/S5Telemetry.h の TelemetryTx::tick):
//      ANGLE / RATE  … A,C の交互          (各 7.5Hz)
//        B(水平位置)/D(Δv) は POSHOLD 以外では意味が薄いので送らない。
//        代わりに C を厚くして、離陸時の転倒やモーター飽和を追えるようにする。
//      POSHOLD/GUIDED … A,B,A,B,C,A,B,D の8枠巡回 (8Hz で A 3Hz / B 3Hz / C 1Hz / D 1Hz)
//        B は「PC が自分の指令の効きを見る唯一の枠」なので C/D より厚くしてある
//        (2026-09-16。旧: A,B,D,A,C の5枠)。
//
//  ★ 送信は必ず非同期 (S5T::Tx::service)。HardwareSerial::print を直接
//    呼ぶと TXバッファ(40B)が溢れた時点でブロックし、1000Hz の制御
//    ループが数十ms止まる = それ自体が振動源になる。
//
//  【スケーリング】
//    float をそのまま並べると入らないので int16 に量子化する。
//    分解能は下の SC_* (位置 1cm / 速度 1mm/s / 角度 0.01deg)。
// ============================================================
#pragma once
#include <Arduino.h>
#include "Im920Frame.h"

namespace S5T {

// 構造体を変えたら必ずインクリメントすること (地上局が不一致を検出する)
// VERSION 8 (2026-09-15): TYPE_DV / DvFrame を追加 (ヨー推定用の機体Δv。
//   position_estimator/YAW_HANDOFF.md §8-3, S5Cmd.h の yaw_abs_cdeg と対)。
//   これに合わせて長さチェックを type ごとの可変長にした
//   (payloadBytesFor())。固定 28 byte 前提の受信側コードを触るときは
//   ここも見ること。
// VERSION 9 (2026-09-16): DvFrame に上りリンクの統計 (cmd_age_cs /
//   cmd_good / cmd_lost / cmd_bad / cmd_seq) を足した。地上局から
//   「上りコマンドが機体にどれだけ遅れて・どれだけ落ちて届いているか」
//   を見る手段が cmd_fresh (1秒以内か) しか無く、遅延の切り分けができ
//   なかったため。DvFrame は 12/28 byte しか使っていなかったので、
//   無線の帯域は 1 バイトも増えない。
constexpr uint8_t VERSION = 9;

// IM920sL の実効ペイロード上限 [byte]。これを超えると黙って切られる。
constexpr size_t IM920SL_MAX_PAYLOAD = IM920::MAX_PAYLOAD;      // 32
constexpr size_t CHECKSUM_BYTES      = IM920::CHECKSUM_BYTES;   // 4
constexpr size_t PACKET_BYTES        = IM920SL_MAX_PAYLOAD - CHECKSUM_BYTES;  // 28

constexpr uint8_t TYPE_ALT   = 0x41;  // 'A'  高度ループ + 姿勢
constexpr uint8_t TYPE_POS   = 0x42;  // 'B'  水平位置ループ
constexpr uint8_t TYPE_ATT   = 0x43;  // 'C'  姿勢ループ内部 (モーター出力/レート)
constexpr uint8_t TYPE_DV    = 0x44;  // 'D'  ヨー推定用の機体Δv
constexpr uint8_t TYPE_PARAM = 0x50;  // 'P'  ゲイン一覧

// ---- 量子化スケール (物理値 = 整数値 / SC_xxx) ----
constexpr float SC_CDEG = 100.0f;    // 0.01 deg
constexpr float SC_DDEG = 10.0f;     // 0.1  deg  (ヨー積分値は ±3276deg まで)
constexpr float SC_MM   = 1000.0f;   // 1 mm / 1 mm/s
constexpr float SC_CM   = 100.0f;    // 1 cm      (位置は ±327m まで)
constexpr float SC_1E4  = 10000.0f;  // スロットル割合 0.0001
constexpr float SC_GAIN = 1000.0f;   // ゲイン 0.001 刻み (±32.7)
constexpr float SC_STICK = 100.0f;   // スティック 0.01 刻み (int8, ±1.27)

// flags のビット割り当て (A/B 共通)
enum Flag : uint16_t {
    F_ARMED       = 1u << 0,
    F_FLOW_OK     = 1u << 1,   // PMW3901 が初期化できている
    F_RANGE_OK    = 1u << 2,   // 測距センサが初期化できている
    F_RANGE_VALID = 1u << 3,   // 今この瞬間の測距が信用できる
    F_ALT_EN      = 1u << 4,   // 高度ホールドが有効化されている (シリアル 'g')
    F_ALT_ACT     = 1u << 5,   // 高度ホールドが実際にスロットルを握っている
    F_POS_HOLD    = 1u << 6,   // 位置ホールドが基準を保持中 (スティック中立)
    F_AIRBORNE    = 1u << 7,   // 離陸判定が立っている
    F_DRY_RUN     = 1u << 8,   // ESC へ 0 しか出していない
    F_SAT         = 1u << 9,   // ミキサーがどこかで飽和した
    F_TX_DROP     = 1u << 10,  // 送信バッファが空かず直前に1回捨てた
    // ★ VERSION 5 で追加 (地上局ガイド飛行 / S5Cmd.h)
    F_GUIDED      = 1u << 11,  // 地上局コマンドで飛んでいる (MODE_GUIDED)
    F_CMD_FRESH   = 1u << 12,  // 上りコマンドが規定時間内に届いている
    F_LANDED      = 1u << 13,  // 自動着陸が完了して出力を切った
    // ★ VERSION 6 で追加 (S5Cmd.h の REQ_CIRCLE)
    F_MANEUVER    = 1u << 14,  // 自動水平旋回(等)を実行中。完了すると自然に落ちる
    F_FRAME_OK    = 1u << 15,  // CF_POS_SHIFT で原点合わせ済み = 機体側フェンスが有効
                               //   (離陸前・モード切替で落ちる。地上局は落ちたら送り直す)
};

// ------------------------------------------------------------
//  飛行モード / 高度ホールド状態  (packModes で 1 バイトに載る値)
// ------------------------------------------------------------
//  ★ 値は無線に乗るので、機体側 (drone_s5.cpp selectMode / AltHold.h) も
//    地上側 (ground_receiver / console.py / mission.py) もここだけを見る。
//    以前は各所に手書きの対応表があり、console.py の alt_state 名が
//    機体の実体 (Standby/NoHoverThr/…) とずれたまま表示されていた。
enum Mode : uint8_t {
    MODE_RATE    = 0,   // 封印 (アクロ)。selectMode は返さない
    MODE_ANGLE   = 1,   // 完全手動 (自己水平のみ)。SW_HOVER=down。bail-out
    MODE_GUIDED  = 2,   // 地上局ガイド飛行 (旧名 MODE_AUTO)
    MODE_POSHOLD = 3,   // フロー水平ホールド + 高度ホールド。SW_HOVER=cen/up
    MODE_ALTHOLD = 4,   // 高度ホールドのみ (フロー喪失時のフォールバック)
};

inline const char* modeName(uint8_t m) {
    switch (m) {
        case MODE_RATE:    return "RATE";
        case MODE_ANGLE:   return "ANGLE";
        case MODE_GUIDED:  return "GUIDED";
        case MODE_POSHOLD: return "POSHOLD";
        case MODE_ALTHOLD: return "ALTHOLD";
    }
    return "?";
}

//  高度ホールドの状態 (quad/AltHold.h の実体。表示 / ログ / PosHold のゲート)
enum class AltState : uint8_t {
    Off        = 0,  // シリアルのトグルで無効にされている
    Standby    = 1,  // 未アーム / POSHOLD でない / スロットルを絞っている
    NoHoverThr = 2,  // ALT_HOVER_THR が未設定 (0) なので engage しない
    NoRange    = 3,  // 測距が無い / まだ一度も掴めていない
    Holding    = 4,  // 通常動作。スロットルを握っている
    RangeLost  = 5,  // engage 後に測距「だけ」を失い、base で保持している
};

inline const char* altStateName(uint8_t s) {
    switch ((AltState)s) {
        case AltState::Off:        return "OFF";
        case AltState::Standby:    return "STANDBY";
        case AltState::NoHoverThr: return "NO_HOVER_THR";
        case AltState::NoRange:    return "NO_RANGE";
        case AltState::Holding:    return "HOLDING";
        case AltState::RangeLost:  return "RANGE_LOST";
    }
    return "?";
}

// mode と alt_state を 1 バイトに詰める (上位=alt_state 下位=mode)
inline uint8_t packModes(uint8_t mode, uint8_t alt_state) {
    return (uint8_t)(((alt_state & 0x0F) << 4) | (mode & 0x0F));
}
inline uint8_t unpackMode(uint8_t m)     { return (uint8_t)(m & 0x0F); }
inline uint8_t unpackAltState(uint8_t m) { return (uint8_t)((m >> 4) & 0x0F); }

// ------------------------------------------------------------
//  共通ヘッダ (6 byte)
//    t_cs は millis()/10 [10ms単位]。655.35 秒で一周するので、地上側で
//    自前の受信時刻を使って巻き戻しを展開する (ground_receiver TelemetryStore の unwrapTime)。
//    uint32 の生 millis を載せる余裕は 28 バイトには無い。
// ------------------------------------------------------------
struct __attribute__((__packed__)) Header {
    uint8_t  type;    // TYPE_ALT / TYPE_POS / TYPE_PARAM
    uint8_t  seq;     // A/B 通しの連番。地上で欠落を数える
    uint16_t flags;   // Flag のビット論理和
    uint16_t t_cs;    // 機体側 millis()/10
};
static_assert(sizeof(Header) == 6, "Header は 6 byte");

// ------------------------------------------------------------
//  A: 高度ループ + 姿勢  (28 byte)
// ------------------------------------------------------------
struct __attribute__((__packed__)) AltFrame {
    Header  h;                 //  6
    uint8_t modes;             //  7  packModes(mode, alt_state)
    uint8_t thr;               //  8  スロットルスティック 0..250 (=0.000..1.000)
    int16_t roll_cd;           // 10  [0.01 deg]
    int16_t pitch_cd;          // 12
    int16_t yaw_dd;            // 14  ヘディング積分値 [0.1 deg]
    int16_t range_h_mm;        // 16  傾き補正後の対地高度
    int16_t range_raw_mm;      // 18  補正前の斜め距離 (センサ生値の健全性確認用)
    int16_t alt_hold_mm;       // 20  目標高度
    int16_t climb_mmps;        // 22  上昇速度 実測
    int16_t alt_vz_tar_mmps;   // 24  上昇速度 目標 (位置ループ出力)
    int16_t alt_thr_corr;      // 26  スロットル補正 (速度PID出力) [1e-4]
    uint16_t alt_thr_out;      // 28  実際に出したスロットル [1e-4]
};
static_assert(sizeof(AltFrame) == PACKET_BYTES, "AltFrame が 28 byte ではありません");

// ------------------------------------------------------------
//  B: 水平位置ループ  (28 byte)
// ------------------------------------------------------------
struct __attribute__((__packed__)) PosFrame {
    Header  h;                 //  6
    uint8_t modes;             //  7
    uint8_t bad;               //  8  poshold.badCount() (フロー失探の連続回数)
    int16_t vx_mmps;           // 10  制御に使う対地速度 (機体座標 前+) [mm/s]
    int16_t vy_mmps;           // 12  (右+)
    int16_t vx_tar_mmps;       // 14  速度ループの目標
    int16_t vy_tar_mmps;       // 16
    int16_t pos_n_cm;          // 18  地面固定フレームの推定位置 [cm]
    int16_t pos_e_cm;          // 20
    int16_t hold_n_cm;         // 22  保持したい地面位置 [cm]
    int16_t hold_e_cm;         // 24
    int16_t lean_roll_cd;      // 26  位置ループが出した目標リーン角 [0.01deg]
    int16_t lean_pitch_cd;     // 28
};
static_assert(sizeof(PosFrame) == PACKET_BYTES, "PosFrame が 28 byte ではありません");

// ------------------------------------------------------------
//  C: 姿勢ループの内部  (28 byte)
// ------------------------------------------------------------
//  離陸時の転倒や発振の原因を追うためのフレーム。A/B だけでは
//  「どのモーターが飽和したか」「レートループが指令に追従しているか」が
//  分からず、接地したまま転がったのか空中で発振したのかを切り分けられない。
//
//  ★ 角速度は 0.1 deg/s 刻み (±3276 deg/s)。トルク指令 cmd は [-1,1] を
//    1e-4 刻みで。モーター出力は 0..250 = 0.000..1.000 (分解能 0.004)。
//  ★ yaw_cmd は入っていない (28バイトに入らなかった)。ヨーの問題は
//    yaw_rate_dd で見る。枠はスティック2本に使っている。
struct __attribute__((__packed__)) AttFrame {
    Header  h;                 //  6
    uint8_t modes;             //  7
    uint8_t sat;               //  8  Q::MixInfo::sat  bit0..3 = M1..M4 が張り付いた
    uint8_t m1, m2, m3, m4;    // 12  各モーター出力 0..250 (= 0.000..1.000)
    int16_t roll_rate_dd;      // 14  実測角速度 [0.1 deg/s]
    int16_t pitch_rate_dd;     // 16
    int16_t yaw_rate_dd;       // 18
    int16_t roll_rate_tar_dd;  // 20  角度ループが出した目標角速度 [0.1 deg/s]
    int16_t pitch_rate_tar_dd; // 22
    int16_t roll_cmd;          // 24  ミキサーへ渡したトルク指令 [1e-4]
    int16_t pitch_cmd;         // 26
    // ★ 受信機から読んだ生のスティック値 (STICK_SIGN を掛ける前)。
    //   トリムずれの検出用。2026-09-04、ロールスティックが中立のつもりで
    //   +0.318 ずれており、角度ループが常時「+9.5度傾け」と言われていた。
    //   これが「離陸しようとすると必ず一方向へ流れる」の正体だった。
    //   逆算でしか分からず時間を溶かしたので、以後は直接載せる。
    //   ※ 非アーム中でも入る (sbus から直接読む)。飛ばす前に地上で確認できる。
    int8_t  roll_stick;        // 27  [0.01] -1.00..+1.00
    int8_t  pitch_stick;       // 28
};
static_assert(sizeof(AttFrame) == PACKET_BYTES, "AttFrame が 28 byte ではありません");

// ------------------------------------------------------------
//  D: ヨー推定用の機体Δv
// ------------------------------------------------------------
//  position_estimator の yaw_estimator が、カメラのΔv (フィールド座標) と
//  この機体Δv (FRD の前+右+) を突き合わせて機首の絶対方位を求める
//  (YAW_HANDOFF.md 参照)。中身は「重力を除去した水平加速度」を、
//  直前にこのフレームを送ってからの経過時間ぶん積分した値 = 速度変化。
//
//  ★ 積分窓は固定 200ms ではなく可変長 (前回の D 送信から今回まで)。
//    送信頻度はモードで変わる (TelemetryTx::tick) ため、固定長にすると実際の
//    積分区間とズレる。地上側は h.t_cs (このフレームの送信時刻 = 窓の
//    終端) をそのまま使えばよい (窓の開始は特に要らない。PC側
//    yaw_estimator は複数フレームを跨いで合成する)。
//  ★ yaw_dd は AltFrame と同じ量 (機体のヨー推定 HeadingHold::est) だが、Δv とちょうど同じ
//    瞬間の値をペアで送る (旋回ゲート判定の精度のため。AltFrame は
//    別の巡回スロットで届くので厳密に同時刻にならない)。
//  ★ A/B/C と違って 28 byte ちょうどにしていない (中身が少ないので
//    無駄に埋めない)。payloadBytesFor() が type ごとに正しい長さを返す。
//
//  ★ VERSION 9: 空いていた後半に「上りリンクの言い分」を同乗させた。
//    ここ以外に置き場所が無い (A/B/C/P は 28 byte ちょうどで満杯)。
//    D は巡回の中で一番遅い枠 (1Hz 前後) だが、この統計は STAT と同じ
//    粒度で見られれば足りるので問題ない。
//    ★ 「Δv のフレーム」ではなく「機体→地上の状態報告のうち、位置・姿勢
//      以外を運ぶ枠」だと思うこと。ここに足すぶんには帯域は増えない。
struct __attribute__((__packed__)) DvFrame {
    Header   h;                //  6
    int16_t  dvx_mmps;         //  8  重力除去後のΔv 前+ [mm/s]
    int16_t  dvy_mmps;         // 10  同 右+ [mm/s]
    int16_t  yaw_dd;           // 12  この瞬間の機体ヨー推定 [0.1 deg]
    // ---- 上りリンク (S5C::Rx) の統計。すべて「機体が実際に見た値」 ----
    uint16_t cmd_age_cs;       // 14  最後に受けた上りコマンドからの経過 [10ms]
                               //     0xFFFF = 一度も受けていない / 655秒以上
    uint16_t cmd_good;         // 16  受信成功した上りコマンド数 (下位16bit)
    uint16_t cmd_lost;         // 18  seq の飛びから数えた欠落 (下位16bit)
    uint16_t cmd_bad;          // 20  チェックサム+長さ+バージョン不一致 (下位16bit)
    uint8_t  cmd_seq;          // 21  最後に受けたコマンドの seq (地上の送信seqと対)
    int8_t   cmd_rssi;         // 22  そのコマンドの RSSI (未受信なら -1)
};
static_assert(sizeof(DvFrame) <= PACKET_BYTES, "DvFrame が 28 byte を超えています");

// ------------------------------------------------------------
//  P: ゲイン一覧  (28 byte)
//    シリアル 'p' メニューで飛行中に変えられる値があるので、CSV だけ
//    見て「どのゲインの結果か」が分かるようにするため数秒に1回混ぜる。
//    float は入らないので 0.001 刻みの int16 にする (±32.7 まで)。
// ------------------------------------------------------------
enum ParamFlag : uint8_t {
    PF_ALT_HOLD_EN = 1u << 0,
    PF_DRY_RUN     = 1u << 1,
    PF_SONAR       = 1u << 2,   // 1=MaxBotix EZ (PW) / 0=VL53L1X (I2C)
    PF_STICK_VZ    = 1u << 3,
};

struct __attribute__((__packed__)) ParamFrame {
    uint8_t type;          //  1  TYPE_PARAM
    uint8_t seq;           //  2
    uint8_t ver;           //  3  VERSION
    uint8_t cfg_flags;     //  4  ParamFlag
    int16_t flow_vel_kp;   //  6  すべて [x1000]
    int16_t flow_vel_ki;   //  8
    int16_t flow_vel_kd;   // 10
    int16_t flow_pos_kp;   // 12
    int16_t alt_pos_kp;    // 14
    int16_t alt_rate_kp;   // 16
    int16_t alt_rate_ki;   // 18
    int16_t alt_rate_kd;   // 20
    int16_t alt_hover_thr; // 22
    int16_t alt_target_m;  // 24
    int16_t flow_max_lean; // 26
    int16_t alt_thr_auth;  // 28
};
static_assert(sizeof(ParamFrame) == PACKET_BYTES, "ParamFrame が 28 byte ではありません");

// ------------------------------------------------------------
//  単発メンテナンス指令 (S5Cmd.h の Action) の実行結果コード
// ------------------------------------------------------------
//  ★ 2026-09-14: 元は IM920 の AckFrame (このファイル) で返していたが、
//    単発メンテナンス指令そのものを BLE 専用にしたため、AckFrame と
//    TYPE_ACK は削除した。この列挙値だけは BLE 側 (LogLinkProto.h の
//    ActAck.result / drone_s5.cpp の runMaintenanceAction()) が
//    そのまま使い続けている。
enum AckResult : uint8_t {
    ACK_OK             = 0,  // 実行した
    ACK_REFUSED_ARMED  = 1,  // アーム中だったので拒否した (IMU_CAL/SELFTEST)
    ACK_CAL_REJECTED   = 2,  // IMU_CAL は実行したが、妥当性チェックで却下された
                             // (機体が水平/静止していなかった。値は変えていない)
};

// ------------------------------------------------------------
//  type ごとの想定ペイロード長
// ------------------------------------------------------------
//  ★ 以前は「全フレーム固定 28 byte」を前提に、受信側が type を見る前に
//    長さだけで弾いていた。DvFrame は 28 byte 未満なので、type を見て
//    から期待長を引くようにした。未知の type には 0 を返すので、
//    呼び出し側は 0 を「弾く」扱いにすること。
inline size_t payloadBytesFor(uint8_t type) {
    switch (type) {
        case TYPE_ALT:   return sizeof(AltFrame);
        case TYPE_POS:   return sizeof(PosFrame);
        case TYPE_ATT:   return sizeof(AttFrame);
        case TYPE_DV:    return sizeof(DvFrame);
        case TYPE_PARAM: return sizeof(ParamFrame);
    }
    return 0;
}

// ------------------------------------------------------------
//  量子化ヘルパ (飽和付き)
// ------------------------------------------------------------
inline int16_t q16(float v, float scale) {
    const float x = v * scale;
    if (x >  32767.0f) return  32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)lroundf(x);
}
inline uint16_t qu16(float v, float scale) {
    const float x = v * scale;
    if (x > 65535.0f) return 65535;
    if (x < 0.0f)     return 0;
    return (uint16_t)lroundf(x);
}
inline uint8_t qu8(float v, float scale) {
    const float x = v * scale;
    if (x > 255.0f) return 255;
    if (x < 0.0f)   return 0;
    return (uint8_t)lroundf(x);
}

// ------------------------------------------------------------
// ------------------------------------------------------------
//  行の組み立て / チェックサム / 送信は IM920 層 (Im920Frame.h) に一本化
// ------------------------------------------------------------
//  以前はここと S5Cmd.h に同じ checksum() と Tx クラスが 2 つずつあった。
using IM920::checksum;
using Tx = IM920::Tx;

} // namespace S5T
