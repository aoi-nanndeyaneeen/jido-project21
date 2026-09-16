// ============================================================
//  S5Features.h  -  drone_s5 のビルド時スイッチと周期の一覧
// ============================================================
//  「この機体に何が載っていて、どのループを何 Hz で回すか」だけをここに置く。
//  制御のゲイン (QuadConfig.h / S5Gains.h) とは分けてある。
//
//  ------------------------------------------------------------
//  【drone_s5 の周期一覧】 ★ loop() の骨組みそのもの。変えるならここと loop() を一緒に見る
//
//      1000Hz  MAIN_HZ (= RATE_LOOP_HZ)   IMU 読み → 姿勢推定 → レートPID → ミキサー → ESC
//       200Hz  ANGLE_LOOP_HZ (÷5)         角度PID (目標角 → 目標角速度)
//       100Hz  FLOW_LOOP_HZ               PMW3901 読み + de-rotation サンプル
//        25Hz  FLOW_CTRL_HZ (÷4)          ↳ 窓を締めて PosHold (速度/位置ループ)
//       100Hz  RANGE_LOOP_HZ              測距ポーリング → AltHold (新サンプルの回だけ PID)
//       200Hz  TELEM_RX_HZ                IM920 上りコマンドの受信ポーリング
//       100Hz  GUIDED_HZ (÷10)            地上局要求 → 目標速度/高度への翻訳 (制御はしない)
//       500Hz  FlightLog::LOG_HZ (÷2)     125Hz ログ 1 行を作って USB/RAM/SD/LogLink へ
//         8Hz  TELEM_TX_HZ                下りテレメトリ 1 パケット (A,B,A,B,C,A,B,D 枠)
//        10Hz  DEBUG_HZ                   シリアル画面
//      毎ループ                           s5tx.service() / LogLink::service() / LED
//
//  ★ 上り (地上局→機体) の実効レートは ground_receiver 側の CMD_MIN_GAP_MS
//    (125ms = 8Hz) が決める。IM920 は半二重なので下り TELEM_TX_HZ とセットで
//    UART 占有率 55% 程度を上限に見ること (protocol/S5Cmd.h 冒頭)。
// ============================================================
#pragma once
#include "quad/QuadConfig.h"
#include "S5Telem.h"          // S5T::Mode (値は無線に乗る。protocol/ が唯一の定義)

namespace S5 {

// ---- 搭載デバイス ----------------------------------------------------
constexpr bool USE_MPU   = true;
constexpr bool USE_SBUS  = true;
constexpr bool USE_MOTOR = true;
constexpr bool USE_IM920 = true;   // 地上局 (position_estimator) との無線
constexpr bool USE_FLOW  = true;   // PMW3901 オプティカルフロー
constexpr bool USE_RANGE = true;   // 測距 (QuadConfig の RANGE_BACKEND で ToF/SONAR)

// ★ SD (HW-125) と PMW3901 は SPI0 を共有していて共存できない (HW-125 クローンの
//   74LVC125 が MISO を Hi-Z にせずフローを潰す)。SD 書き込みは RP2040 ロガー
//   (log_recorder/) へ UART で出す USE_LOGLINK が現行運用。配線は quad/LogLink.h。
constexpr bool USE_LOGLINK = true;
constexpr bool USE_SD = !USE_FLOW && !USE_LOGLINK;
static_assert(!(USE_FLOW && USE_SD),
              "USE_FLOW と USE_SD は同時に true にできない (SPI0 共有)。"
              "別バスに分けたなら、この static_assert を消して自己責任で。");
static_assert(!(USE_SD && USE_LOGLINK),
              "USE_SD と USE_LOGLINK は同時に true にできない "
              "(同じ Rec を2箇所に流すと、どちらが正のログか分からなくなる)。");

// ---- 機能スイッチ ----------------------------------------------------
// 高度ホールド (スロットルPID)。false にすると POSHOLD でも高度は手動のまま。
// シリアル 'g' でも実行時に切り替えられる (ベンチ用)。
constexpr bool USE_ALT_HOLD = true;

// ドライラン。true / シリアル 'm' で ON にすると制御パイプラインは全部回るが
// ESC へは 0 (アイドル) しか送らない。g_out は計算・記録される。
// ※ アーム状態でも一切回らないので、飛ばす前に必ず OFF に戻すこと。
constexpr bool DRY_RUN = false;

// レートPID の I 項を積分してよいスロットル下限 (地上での windup 防止)
constexpr float I_ENABLE_THR = 0.15f;

// 角度ループが出せる角速度の上限 [deg/s] (レートループが追えない目標を出さない蓋)
constexpr float ANGLE_OUT_LIMIT = 300.0f;

// POSHOLD/GUIDED で高度ホールドが engage する前のスロットル上限。
//  地面に置いた状態では測距が RANGE_MIN_M を割って AltHold が NoRange のまま
//  engage しないので、その間プロポのスロットルが素通しだった (姿勢をスティックで
//  当てられないのに全開で上がれた)。engage 後の権限と同程度で頭打ちにする。
//  ★ ALT_HOVER_THR を触ったらここも見直す (余裕 0.10 以上を static_assert で強制)。
//    ホバー 0.55 に対し 0.70 = (0.70/0.55)^2 = 1.62 倍 = 上向き 0.62g。経緯は TUNING_HISTORY §6。
constexpr float POSHOLD_THR_CAP = 0.70f;
static_assert(POSHOLD_THR_CAP >= Quad::ALT_HOVER_THR + 0.10f,
              "POSHOLD_THR_CAP はホバースロットル +0.10 以上にすること "
              "(離陸と電池消耗ぶんの余裕が要る)");

// ---- 周期 ------------------------------------------------------------
constexpr int MAIN_HZ  = Quad::RATE_LOOP_HZ;   // 1000
constexpr int DEBUG_HZ = Quad::DEBUG_HZ;       // 10

// 地上局要求の翻訳 (Guided::update) を回すレート。1000Hz で回す必要は無く
// (中身は millis 比較と代入)、上りコマンドは 200Hz でポーリングしているので
// 100Hz で足りる。メインループの分周で回す。
constexpr int GUIDED_HZ  = 100;
constexpr int GUIDED_DIV = MAIN_HZ / GUIDED_HZ;
static_assert(MAIN_HZ % GUIDED_HZ == 0, "GUIDED_HZ は MAIN_HZ の約数にすること");

// 下りテレメトリ [Hz]。IM920sL が持続できるのは 15Hz まで (2026-09-04 実測)。
//  GUIDED では上り 8Hz と帯域を分け合うので 8Hz (UART 占有 下り30%+上り23%)。
//  枠割りは S5Telemetry.h の tick()。経緯は TUNING_HISTORY §5。
constexpr int TELEM_TX_HZ = Quad::GUIDED_ENABLE ? 8 : 15;

// ゲイン一覧 (S5T::Param) を送る周期 [ms]。この回だけ State を1つ落とす。
constexpr uint32_t TELEM_PARAM_MS = 5000;

// 上りコマンド受信のポーリングレート [Hz]。19200 baud なら 200Hz で 1 回
// 最大 12 バイト程度。Teensy の受信バッファ (64B) を溢れさせない。
constexpr int TELEM_RX_HZ = 200;

// ---- 飛行モード (値は S5Telem.h の S5T::Mode。地上局・PC と共有) ----------
//  MODE_ANGLE   : 完全手動 (自己水平のみ)。SW_HOVER=down。これが bail-out
//  MODE_ALTHOLD : 姿勢手動 + 高度だけ自動。POSHOLD でフローが死んだときの受け皿
//  MODE_POSHOLD : 完全自動 (フロー水平 + 測距高度)。SW_HOVER=cen、または up で
//                 GUIDED の資格が無いとき
//  MODE_GUIDED  : 地上局ガイド飛行。SW_HOVER=up で Guided が engage したとき
//  MODE_RATE    : 封印 (selectMode は返さない。enum 値の互換のため残置)
using Mode = S5T::Mode;
using S5T::MODE_RATE;
using S5T::MODE_ANGLE;
using S5T::MODE_GUIDED;
using S5T::MODE_POSHOLD;
using S5T::MODE_ALTHOLD;

inline const char* modeLabel(Mode m) {
    switch (m) {
        case MODE_GUIDED:  return "GUIDED (地上局ガイド)";
        case MODE_POSHOLD: return "POSHOLD(フロー位置保持)";
        case MODE_ALTHOLD: return "ALTHOLD(高度保持のみ)";
        case MODE_ANGLE:   return "ANGLE  (水平維持)";
        default:           return "RATE   (アクロ)";
    }
}

} // namespace S5
