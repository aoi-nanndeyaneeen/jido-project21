// ============================================================
//  FlightLog.h  -  飛行ログの「1行の定義」と 3 つの吐き出し先
//                  (drone_s5.cpp 専用)
// ============================================================
//  【なぜ切り出したか】
//  以前は drone_s5.cpp の中に namespace Log (USB 直結ストリーム) と
//  namespace RamLog (本体RAMのリングバッファ) が別々に住んでいて、
//  同じ 60 列を「2 本の printf 書式」で二重に持っていた。列を 1 つ足すと
//    (1) Log::HEADER        列名
//    (2) Log::sample()      USB 用の printf
//    (3) RamLog::Rec        量子化した構造体
//    (4) RamLog::fillRec()  詰める処理
//    (5) RamLog::formatRow() SD/ダンプ用の printf
//    (6) scripts/bin2csv.py PC 側デコーダ
//  の 6 箇所を手で揃える必要があり、実際に取りこぼしが起きていた
//  (2026-09-09: accx/accy/accz の量子化スケールを変えたとき、(2) と (5) の
//   両方を直す必要があった)。
//
//  【いまの構造】
//    Rec        … 1 行を量子化して詰めた構造体。これが唯一の「行の定義」
//    HEADER     … 列名。唯一の一覧
//    formatRow  … Rec -> CSV 1 行。唯一のフォーマッタ
//    Usb::      … 'l' で USB へ 500Hz ストリーム (scripts/logger.py が受ける)
//    (SD への書き出しは quad/SdLog.h。Rec をそのまま byte 列として流す)
//
//  ★ 2026-09-12: 本体RAMへの飛行中バッファリング (Ram::、'n'/'v'/'y' キー) は
//    廃止した。GUIDED 診断列を足すたびに RAM2 の空きと綱引きになるのが
//    煩わしく、SD ログ (quad/SdLog.h) で時間無制限に代替できるため。
//
//  呼び出し側 (drone_s5.cpp) は 500Hz で Rec を 1 個だけ作り、
//  それを Usb / SdLog / LogLink へ配る。
//
//  ★ 列を足すときに触るのは Rec / HEADER / formatRow / fillRec(呼び出し側) と
//    scripts/bin2csv.py の 5 箇所。printf は 1 本になったので 6 -> 5。
//  ★ Rec の並びか型を変えたら REC_VER を +1 し、bin2csv.py も合わせること。
// ============================================================
#pragma once
#include <Arduino.h>

#include "quad/QuadConfig.h"
#include "S5Telem.h"     // 量子化ヘルパ q16/qu8 とスケール定数 SC_*

#ifndef DMAMEM
#define DMAMEM
#endif

//  ★ 状態変数は static (= 内部リンケージ)。このヘッダを include するのは
//    drone_s5.cpp だけという前提。quad/SdLog.h と同じ流儀。
namespace FlightLog {

// ---- レート ---------------------------------------------------------------
constexpr int LOG_HZ = 500;                        // 記録レート [Hz]
constexpr int DIV    = Quad::RATE_LOOP_HZ / LOG_HZ; // メインループ何回に1回
static_assert(Quad::RATE_LOOP_HZ % LOG_HZ == 0,
              "LOG_HZ は RATE_LOOP_HZ の約数にしてください");

// ---- flags のビット (S5Telem.h の Flag と揃えてある。無線とは無関係、内部専用) --
enum RFlag : uint16_t {
    RF_ARMED    = 1u << 0,
    RF_FLOW_OK  = 1u << 1,
    RF_RANGE_OK = 1u << 2,
    RF_ALT_EN   = 1u << 3,
    RF_ALT_ACT  = 1u << 4,
    RF_HOLDING  = 1u << 5,   // poshold.holding()
    // 2026-09-12: GUIDED 診断用 (地上局からの上り指令 / スイッチ生値)。
    RF_GUIDED_ENGAGED = 1u << 6,  // g_guided_engaged
    RF_SW_AUTO_UP     = 1u << 7,  // Ch::SW_AUTO == up
};

// ★ Rec の列を足す/型を変えたら +1 する。SdLog の BIN ヘッダに書き込まれ、
//   scripts/bin2csv.py が古い BIN を弾くのに使う。
// v2 (2026-09-09): accx/accy/accz の量子化スケールを SC_1E4 -> 1000 に変更
//   (±8g 化で値域が広がり ±3.27g で頭打ちしていた)。列の並び・サイズは不変。
// v3 (2026-09-12): GUIDED 診断用に cmd_req/cmd_age_ms/cmd_vx_mmps/cmd_vy_mmps/
//   cmd_alt_cm を末尾に追加。flags に RF_GUIDED_ENGAGED/RF_SW_AUTO_UP を追加。
constexpr uint8_t REC_VER = 3;

struct __attribute__((packed)) Rec {
    uint32_t t_ms;
    uint16_t dt_us;
    uint16_t flags;
    uint8_t  mode;
    uint8_t  mixsat;
    uint8_t  thr;                                  // /250
    int8_t   roll_stick, pitch_stick, yaw_stick;    // x100
    int16_t  roll_ang, pitch_ang, yaw_est;          // cdeg
    int16_t  roll_rate, pitch_rate, yaw_rate;       // ddeg/s
    int16_t  roll_cmd, pitch_cmd, yaw_cmd;          // 1e4
    uint8_t  m1, m2, m3, m4;                        // /250
    int16_t  span_limit;                            // 1e3
    int16_t  roll_ratetar, pitch_ratetar;           // ddeg/s
    int16_t  roll_angtar, pitch_angtar;             // cdeg
    int16_t  flow_raw_x, flow_raw_y, flow_dx, flow_dy;  // dpx (x10)
    int16_t  flow_vx, flow_vy;                      // mm/s
    int16_t  flow_h;                                // mm
    float    flow_accx, flow_accy;                  // m (蓄積値なので float のまま)
    int16_t  fh_vxc, fh_vyc, fh_vxt, fh_vyt;        // mm/s
    int16_t  fh_leanr, fh_leanp;                    // cdeg
    int16_t  fh_posn, fh_pose, fh_holdn, fh_holde;  // mm
    int16_t  range_raw, range_h;                    // mm
    int16_t  climb;                                 // mm/s
    int16_t  alt_holdm;                             // mm
    int16_t  alt_vzt;                               // mm/s
    int16_t  alt_corr;                              // 1e4
    uint8_t  alt_thr_out;                           // /250
    // 2026-09-07: ミキサーが実際に使ったスロットル (MixInfo::thr_used)。
    //   alt_thr_out との差 = 姿勢優先に奪われた量。
    uint8_t  alt_used;                              // /250
    // 2026-09-06: 加速度Z相補フィルタ検討用。制御には未使用、記録のみ。
    int16_t  accx, accy, accz;                      // g x1e3 (FRD, g_att.acc_*)
    // 2026-09-07: AltEstimator (加速度Z×測距の相補フィルタ) の出力。
    int16_t  acc_up;                                // m/s^2 x1e3
    int16_t  est_h;                                 // mm
    int16_t  est_vz;                                // mm/s
    int16_t  est_bias;                              // m/s^2 x1e3
    // 2026-09-12: GUIDED 診断用。地上局からの最後の上り指令とその鮮度。
    //   ★ SW_HOVER は mode 列 (POSHOLD=up+フロー正常, ALTHOLD=up+フロー死 or cen,
    //     ANGLE=down) から逆算できるが、SW_AUTO の生値は他に出ていないので
    //     flags の RF_SW_AUTO_UP を見ること。
    uint8_t  cmd_req;                               // S5C::Req (last() の値)
    uint16_t cmd_age_ms;                             // s5rx.ageMs() (65535 で頭打ち)
    int16_t  cmd_vx_mmps, cmd_vy_mmps;               // 最後に受けた指令の目標速度
    int16_t  cmd_alt_cm;                             // 同 目標高度
};
// ★ この値が変わる = SD の BIN 形式が変わった。REC_VER を +1 し、
//   scripts/bin2csv.py の _FIELDS / REC_VER も合わせること。
static_assert(sizeof(Rec) == 125, "FlightLog::Rec のサイズが変わった。上のコメント参照");

// ---- 列名 (唯一の一覧) ----------------------------------------------------
//  scripts/logger.py が "HEADER," の後ろをそのまま CSV の1行目に使い、
//  scripts/analyze_log.py / analyze_alt_pid.py は列名で参照する。
constexpr char HEADER[] =
    "t_ms,dt_us,mode,armed,thr,"
    "roll_sbus,pitch_sbus,yaw_sbus,"
    "roll_ang,pitch_ang,yaw_ang,"
    "roll_gyr,pitch_gyr,yaw_gyr,"
    "roll_cmd,pitch_cmd,yaw_cmd,"
    "m1,m2,m3,m4,corr_limit,sat,"
    "roll_ratetar,pitch_ratetar,roll_angtar,pitch_angtar,"
    // --- s5a: オプティカルフロー (末尾に追記。analyze_log.py は列名参照なので
    //     既存の解析はそのまま動く。新列を見たいときだけ列名を足す) ---
    "flow_ok,flow_raw_x,flow_raw_y,flow_dx,flow_dy,flow_vx,flow_vy,flow_h,"
    "flow_accx,flow_accy,"
    // --- s5b/s5d: 速度・位置ホールド (位置は地面固定フレーム N/E) ---
    "fh_vxc,fh_vyc,fh_vxt,fh_vyt,fh_leanr,fh_leanp,fh_posn,fh_pose,"
    "fh_holdn,fh_holde,fh_hold,"
    // --- s5c: 距離センサ + 高度ホールド ---
    "range_ok,range_raw,range_h,climb,alt_en,alt_act,alt_hold,alt_vzt,alt_base,alt_corr,alt_thr,"
    // --- 2026-09-07: alt_used = ミキサーが実際に使ったスロットル。
    //     alt_thr との差が「姿勢優先に奪われた量」。ここが常時 0 でなければ
    //     高度制御は自分の指令どおりに飛べていない (log_037 では平均 +0.127)。
    "alt_used,"
    // --- 2026-09-06: 加速度Z相補フィルタの検討用。制御には未使用、記録のみ。
    //     g_att.acc_* (FRD系, g単位) をそのまま出す。回転・積分・フィルタは
    //     全部 Python 側でオフライン検証する (analyze_alt_pid.py 系のツール)。
    "accx,accy,accz,"
    // --- 2026-09-07: 加速度Z×測距の相補フィルタ (quad/AltEstimator.h)。
    //     ALT_USE_ACC_FUSION=false の間は制御に未使用、記録のみ。
    //     est_vz が climb より何ms 速いかを analyze_alt_pid.py が判定する。
    "acc_up,est_h,est_vz,est_bias,"
    // --- 2026-09-12: GUIDED 診断用。地上局からの上り指令とスイッチ生値 ---
    //     sw_hover は mode 列から逆算できるので出さない。sw_auto だけ追加。
    "guided_engaged,sw_auto,cmd_req,cmd_age_ms,cmd_vx,cmd_vy,cmd_alt_cm";

// ============================================================
//  formatRow  -  Rec を CSV 1 行にする「唯一のフォーマッタ」
//    with_prefix=true  : "DATA,..." (USB 用。scripts/logger.py が剥がす)
//    with_prefix=false : 先頭 "DATA," 無し (そのまま CSV に使える)
//  out は Serial でも FsFile でもよい。
//  ★ 列順・書式は HEADER と厳密に一致させること (解析は列名参照なので順序が命)。
// ============================================================
inline void formatRow(Print& out, const Rec& r, bool with_prefix = true) {
    // alt_base は Rec に持たせていない (高度ホールド中は常に ALT_HOVER_THR 固定)。
    const float thrBase = (r.flags & RF_ALT_ACT) ? Quad::ALT_HOVER_THR : 0.0f;
    if (with_prefix) out.print("DATA,");
    out.printf(
        "%lu,%lu,%d,%d,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,"
        "%.4f,%.4f,%.4f,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%u,"
        "%.1f,%.1f,%.1f,%.1f,"
        "%d,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.2f,"
        "%.4f,%.4f,"
        "%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,"
        "%.3f,%.3f,%d,"
        "%d,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,%.3f,%.4f,%.3f,"
        "%.3f,"
        "%.4f,%.4f,%.4f,"
        "%.3f,%.3f,%.3f,%.3f,"
        "%d,%d,%u,%u,%.3f,%.3f,%.1f\n",
        (unsigned long)r.t_ms, (unsigned long)r.dt_us, (int)r.mode,
        (r.flags & RF_ARMED) ? 1 : 0, r.thr / 250.0f,
        r.roll_stick / 100.0f, r.pitch_stick / 100.0f, r.yaw_stick / 100.0f,
        r.roll_ang / S5T::SC_CDEG, r.pitch_ang / S5T::SC_CDEG, r.yaw_est / S5T::SC_CDEG,
        r.roll_rate / S5T::SC_DDEG, r.pitch_rate / S5T::SC_DDEG, r.yaw_rate / S5T::SC_DDEG,
        r.roll_cmd / S5T::SC_1E4, r.pitch_cmd / S5T::SC_1E4, r.yaw_cmd / S5T::SC_1E4,
        r.m1 / 250.0f, r.m2 / 250.0f, r.m3 / 250.0f, r.m4 / 250.0f,
        r.span_limit / 1000.0f, (unsigned)r.mixsat,
        r.roll_ratetar / S5T::SC_DDEG, r.pitch_ratetar / S5T::SC_DDEG,
        r.roll_angtar / S5T::SC_CDEG, r.pitch_angtar / S5T::SC_CDEG,
        (r.flags & RF_FLOW_OK) ? 1 : 0,
        r.flow_raw_x / 10.0f, r.flow_raw_y / 10.0f, r.flow_dx / 10.0f, r.flow_dy / 10.0f,
        r.flow_vx / S5T::SC_MM, r.flow_vy / S5T::SC_MM, r.flow_h / S5T::SC_MM,
        (double)r.flow_accx, (double)r.flow_accy,
        r.fh_vxc / S5T::SC_MM, r.fh_vyc / S5T::SC_MM, r.fh_vxt / S5T::SC_MM, r.fh_vyt / S5T::SC_MM,
        r.fh_leanr / S5T::SC_CDEG, r.fh_leanp / S5T::SC_CDEG,
        r.fh_posn / S5T::SC_MM, r.fh_pose / S5T::SC_MM,
        r.fh_holdn / S5T::SC_MM, r.fh_holde / S5T::SC_MM,
        (r.flags & RF_HOLDING) ? 1 : 0,
        (r.flags & RF_RANGE_OK) ? 1 : 0, r.range_raw / S5T::SC_MM, r.range_h / S5T::SC_MM,
        r.climb / S5T::SC_MM,
        (r.flags & RF_ALT_EN) ? 1 : 0, (r.flags & RF_ALT_ACT) ? 1 : 0,
        r.alt_holdm / S5T::SC_MM, r.alt_vzt / S5T::SC_MM, thrBase,
        r.alt_corr / S5T::SC_1E4, r.alt_thr_out / 250.0f,
        r.alt_used / 250.0f,
        r.accx / 1000.0f, r.accy / 1000.0f, r.accz / 1000.0f,
        r.acc_up / 1000.0f, r.est_h / S5T::SC_MM,
        r.est_vz / S5T::SC_MM, r.est_bias / 1000.0f,
        (r.flags & RF_GUIDED_ENGAGED) ? 1 : 0, (r.flags & RF_SW_AUTO_UP) ? 1 : 0,
        (unsigned)r.cmd_req, (unsigned)r.cmd_age_ms,
        r.cmd_vx_mmps / S5T::SC_MM, r.cmd_vy_mmps / S5T::SC_MM, (float)r.cmd_alt_cm);
}

// ============================================================
//  Usb  -  USB シリアルへ 500Hz でそのまま流す (シリアル 'l' でトグル)
// ============================================================
//  scripts/logger.py が期待するプロトコル:
//    HEADER,<列名>   ... LOG_START の直前に1回
//    LOG_START       ... 記録開始
//    DATA,<値,...>   ... 1サンプル
//    LOG_STOP        ... 記録終了
//  USB を挿しっぱなしのベンチ試験専用。実飛行は Ram:: か SdLog を使う。
namespace Usb {

static bool     active  = false;
static uint32_t dropped = 0;   // USBが詰まって捨てたサンプル数
static uint32_t written = 0;

inline void start() {
    Serial.println();
    Serial.print("HEADER,"); Serial.println(HEADER);
    Serial.println("LOG_START");
    active  = true;
    dropped = 0;
    written = 0;
}

inline void stop() {
    active = false;
    Serial.println("LOG_STOP");
    Serial.printf("INFO: %lu行記録 / %lu行ドロップ(USB詰まり)\n",
                  (unsigned long)written, (unsigned long)dropped);
}

inline void toggle() { active ? stop() : start(); }

// 1サンプル出力。★USBが詰まっていたら書かずに捨てる。
//   ここでブロックすると制御ループが止まって、それ自体が振動源になる。
inline void sample(const Rec& r) {
    if (!active) return;
    // 1行ぶん(約160バイト)の空きが無ければ捨てる
    if (Serial.availableForWrite() < 200) { dropped++; return; }
    formatRow(Serial, r, /*with_prefix=*/true);
    written++;
}

} // namespace Usb


} // namespace FlightLog
