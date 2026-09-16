// ============================================================
//  S5Gains.h  -  drone_s5 の姿勢ループゲインと姿勢基準トリム
// ============================================================
//  ここにあるのは「今飛んでいる値」と、その値である理由の1行だけ。
//  値を動かした経緯 (どのログで何を見てどう変えたか) は
//      flight_controller/TUNING_HISTORY.md
//  に日付順で全部残してある。**値を変えたら必ずそちらへ追記すること。**
//  ここに長い履歴を書き戻さない (以前は §1 だけで 300 行を超えて
//  コードが埋もれていた)。
//
//  水平位置 (FLOW_*) / 高度 (ALT_*) のゲインは QuadConfig.h § 7-2 / § 7-4。
// ============================================================
#pragma once

namespace Gain {

// ---- レート (内側ループ, 1000Hz) ----          kp       ki       kd
//  I 項は 0。2026-09-09 に ki/ANG ki/D_ALPHA を同時に変えて roll が崩れた
//  (LOG0057) ため、飛んでいた LOG0054 の値に戻してある。積分の置き場所は
//  M1/M4 の非対称を直した機体で 1 つずつ検証すること (TUNING_HISTORY §1)。
constexpr float RATE_ROLL [3] = { 0.0015f, 0.0000f, 0.00004f };
constexpr float RATE_PITCH[3] = { 0.0015f, 0.0000f, 0.00004f };
//  ヨーだけ I 項あり。モーター取付角/ペラ差の一定ヨートルクは P では消えない。
constexpr float RATE_YAW  [3] = { 0.0015f, 0.0020f, 0.00004f };

//  D 項 LPF。0.95 (遮断 ~8Hz) にするとクロスオーバー帯で D が加振に回った。
constexpr float RATE_D_ALPHA = 0.80f;
constexpr float RATE_I_LIMIT = 0.15f;      // 出力の 15% で頭打ち (windup 防止)

// ---- ヘディングホールド (ヨー) ----
//  ジャイロZ積分の「相対」方位を保持する。絶対方位 (磁気/GPS) は使わない。
//  Madgwick の getYaw() は 6 軸では純積分と同じで、制御ジャイロと位相がずれるので使わない。
constexpr float YAW_HOLD_KP       = 3.0f;   // 方位誤差 1deg あたりの戻し角速度 [(deg/s)/deg]
constexpr float YAW_HOLD_RATE_LIM = 60.0f;  // 保持が出してよい角速度の上限 [deg/s]
constexpr float YAW_HOLD_ERR_LIM  = 20.0f;  // これ以上の方位誤差は追わない [deg]
constexpr float YAW_STICK_DEAD    = 0.03f;  // ラダー不感帯。超えたら「操作中」

// ---- 角度 (外側ループ, 200Hz) ----              kp     ki    kd
//  出力は [deg/s]。kp=30 なら「10deg 傾いていたら 300deg/s で戻す」。
//  ki は本来 0 が筋 (レート側 I と干渉する) だが、飛んでいた値 (LOG0054) を保つ。
constexpr float ANG_ROLL [3] = { 30.0f, 0.04f, 0.0f };
constexpr float ANG_PITCH[3] = { 30.0f, 0.04f, 0.0f };
constexpr float ANG_D_ALPHA  = 0.70f;
constexpr float ANG_I_LIMIT  = 30.0f;      // 角度ループ積分項の上限 [deg/s]

// ---- 姿勢基準トリム [deg] ----
//  IMU 取付面と「推力が鉛直になる面」の間の固定回転を、飛行中の fh_lean* の
//  収束値で吸収する。'k' の水平キャリブでは原理的に取れない (推力線は飛ば
//  ないと現れない)。updateControl() が ang_meas を作る 1 箇所でだけ引く。
//  追い込み手順と 6 巡ぶんの経緯は TUNING_HISTORY.md §4。
//  ★ 2026-09-15(3): PITCH はトリムをどちらへ動かしても fh_leanp が増えた
//    (= leanp はトリムでは決まっていない) ので、3便で最小だった -1.45 で打ち止め。
constexpr float ROLL_TRIM_DEG  = +0.33f;
constexpr float PITCH_TRIM_DEG = -1.45f;

} // namespace Gain
