// ============================================================
//  drone_s5.cpp  -  Stage 5 : オプティカルフロー / 距離センサ / 自動ホバリング
// ============================================================
//  Stage 4 (drone_s4.cpp) の角度PID + モード切替 + ヘディングホールドを
//  そのまま土台にして、位置・高度の保持を段階的に足していく。
//
//  【このファイルの現在地: s5d — 1スイッチ完全自動ホバリング】
//    ・s5a (PMW3901 の符号/スケール/de-rotation のベンチ確定) 完了。
//    ・s5b (フロー速度・位置ホールド) 完了。調整値は QuadConfig.h § 7 / § 7-2。
//    ・s5c (対地距離センサ + 高度ホールド) 完了。§ 7-3 / § 7-4。
//        測距は RANGE_BACKEND で ToF(VL53L1X, I2C) / SONAR(MaxBotix EZ, PW) を選択。
//        傾き cos 補正した鉛直高度を毎ループ flow.setHeight() へも渡す
//        (失探したら FLOW_ASSUMED_HEIGHT_M へ自動フォールバック)。
//    ・s5d で足したもの:
//        1. 運用を 2モードに固定。SW_HOVER 1本だけで決める:
//             down → ANGLE   完全手動 (スロットルも手動)。これが bail-out。
//             up   → POSHOLD 完全自動 (水平位置 + 高度 + ヘディングを同時)。
//           SW_AUTO / 地上局AUTO / RATE は封印。実飛行では PC を繋げないので
//           シリアル 'g' 等に依存しない運用にしてある。
//        2. 位置積分を「地面固定フレーム (N/E)」化。s5c までは機体座標のまま
//           積分していたので、機体がヨーすると保持基準ごと回って位置が流れた。
//           ヘディング g_yaw_est で回してから積分し、位置ループの出力を
//           機体座標へ戻して速度PIDへ渡す。
//        3. フロー水平ホールドを quad/PosHold.h (Q::PositionHold) に切り出し。
//           速度LPF / 失探検出 / 位置積分 / 位置ループ / 速度PID / スティック
//           処理を1クラスに集約し、drone_s5.cpp 側のグローバルを一掃した。
//    ★ 高度ホールドの基準スロットルは Q::ALT_HOVER_THR (実測値)。0 のときは
//      POSHOLD 突入時のスティック値を掴むので、ANGLE で安定ホバリングしてから
//      SW_HOVER を上げること。POSHOLD 中に測距だけ失った場合はスティック値へ
//      飛ばさず直前のホバースロットルを保持する (落下防止)。
//    ・調整は QuadConfig.h § 7-2 / § 7-4 と シリアル 'p' メニュー
//      ([i][j][o] = フロー水平, [s][t][u][v] = 高度)。
//    ★ 初回は広い床で、指をモードスイッチに。符号ミス = 即壁/天井行き。
//    ★ 地上確認は 'm' のドライラン (ESCへ0のみ / g_out は計算・表示) を使う。
//      フロー制御は「アーム && POSHOLD && スロットル > FLOW_ENABLE_THR」でしか
//      出力しないので、ドライラン中もスティックを 35% 程度まで上げること。
//
//  ------------------------------------------------------------
//  以下、Stage 4 から引き継いだ説明:
//
//  【前提】Stage 3 のレートPIDが飛べる状態になっていること。
//    角度ループは、レートループが安定していない限り絶対に安定しません。
//    Stage 3 で機体がふらつくなら、ここへ来ても悪化するだけです。
//
//  【制御の構造】カスケード
//
//    ANGLE モード:
//      スティック → 目標角度[deg] → 角度PID(200Hz) → 目標角速度[deg/s]
//                                 → レートPID(1000Hz) → トルク指令 → ミキサー
//    RATE モード:
//      スティック → 目標角速度[deg/s] → レートPID(1000Hz) → ...
//
//    ヨーはスティックを触っている間だけレート指令です。
//    中立に戻すと「ヘディングホールド」に切り替わり、そのときの機首方位を
//    保持します (§ 7-2 を参照)。
//
//  【モード】
//    SW_HOVER が UP  → ANGLE (手を離すと水平に戻る)
//    それ以外        → RATE  (アクロ。Stage 3 と同じ挙動)
//
//    旧コードの MODE_LEVEL_TURN / MODE_LEVEL_FLIGHT は固定翼用なので
//    削除しました (クアッドで 25度バンクを保持し続けるのは危険です)。
//    MODE_MANUAL (PIDを通さない素通し) も、クアッドでは意味がないので
//    削除しています。
//
//  【旧コードから直したところ】
//    ・角度PIDの dt を、実際の呼び出し周期 (200Hz) で渡すようにした
//      (旧: 200Hz で呼びながら dt は 1ms 固定で、I項とD項が5倍ずれていた)
//    ・SEMI_MANUAL の単位不整合を解消
//      (旧: 目標「角度」30deg を、そのままレートPIDの目標「角速度」に渡していた)
//    ・モード遷移時に目標値もクリアするようにした
//      (旧: pid_reset() は呼ぶが tar は残るので、
//       LEVEL_FLIGHT の前傾10度が次のモードに持ち越されていた)
//    ・ヨーのレートPIDに I項を入れ、ヘディングホールドを追加した (§ 7-2)
//      (旧: ヨーは kp のみ。P制御は定常偏差を消せないので、モーター取付角の
//       わずかな傾きやプロペラの個体差が作る一定のヨートルクに負けて、
//       機首がじわじわ回り続けていた)
// ============================================================

#include <Arduino.h>
#include <math.h>

#include "Config.h"
#include "Actuators.h"
#include "Receiver.h"
#include "sensor/IMU.h"   // 気圧・超音波は使わないので Sensors.h ごとは読まない
#include "sensor/OpticalFlow.h"  // PMW3901 (SPI)
#include "sensor/Rangefinder.h"  // 測距 (QuadConfig の RANGE_BACKEND で ToF/SONAR 切替)
#include "Telemetry.h"    // IM920SL (地上局との送受信)

#include "quad/QuadConfig.h"
#include "quad/QuadPID.h"
#include "quad/Mixer.h"
#include "quad/BodyFrame.h"
#include "quad/PosHold.h"   // s5b/s5d: フロー水平ホールド (地面固定フレーム)

namespace Q = Quad;

// ============================================================
//  § 1  ゲイン
// ============================================================
namespace Gain {

// ---- レート (内側ループ) ----  Stage 3 で決めた値をここに写す
//                        kp      ki      kd
constexpr float RATE_ROLL [3] = { 0.0010f, 0.0f, 0.00004f };
constexpr float RATE_PITCH[3] = { 0.0010f, 0.0f, 0.00004f };

// ★ ヨーの I項。P制御だけでは定常偏差が残る。
//   機体には必ず一定のヨートルクが残っている:
//     ・モーター取付角のわずかな傾き (推力ベクトルが真上を向いていない)
//     ・CW/CCW プロペラの特性差、モーターのKV差、ESCの個体差
//   これらは P では釣り合った角速度で回り続けるだけで、消えない。
//   ki = kp は積分時定数 1秒に相当する。まずこの値で試し、
//   戻りが遅ければ 0.005 まで上げてよい (上げすぎると 1Hz 前後で揺れる)。
constexpr float RATE_YAW  [3] = { 0.0010f, 0.0000f, 0.00004f     };

constexpr float RATE_D_ALPHA = 0.80f;
constexpr float RATE_I_LIMIT = 0.15f;

// ---- ヘディングホールド (ヨー) ----
//  ジャイロZを積分した「相対」方位を保持する。絶対方位 (磁気センサ/GPS) は
//  使わない。市販のフライトコントローラのヨーもこれと同じ仕組み。
//
//  ・ドリフトは無関係: 積分値がどれだけずれても「今向いている方向」を
//    保ち続けることに変わりはない
//  ・±180度の折り返し処理も不要: 相対値なので連続量のまま扱える
//
//  Madgwick の getYaw() は使わない。あれは updateIMU() (6軸) なので
//  ヨーに補正項が一切入らず、結局ジャイロの純積分でしかない上に、
//  制御に使うジャイロ値と位相がずれる。

// 方位誤差 1度あたり何 deg/s で戻すか [(deg/s)/deg]
constexpr float YAW_HOLD_KP       = 3.0f;
// ヘディングホールドが出してよい角速度の上限 [deg/s]
constexpr float YAW_HOLD_RATE_LIM = 60.0f;
// これ以上の方位誤差は追わない [deg]
//  大きく振られたときに全力で振り戻すと危ないので、ここで頭打ちにする。
constexpr float YAW_HOLD_ERR_LIM  = 20.0f;
// ラダースティックの不感帯。これを超えたら「操作中」と判定する
constexpr float YAW_STICK_DEAD    = 0.03f;

// ---- 角度 (外側ループ) ----
//  出力の単位は [deg/s] です。kp = 4.0 なら「10度傾いていたら 40deg/s で戻す」。
//  まず kp だけで調整し、ki / kd は基本 0 のままで構いません。
//                         kp     ki    kd
constexpr float ANG_ROLL [3] = { 4.0f, 0.0f, 0.0f };
constexpr float ANG_PITCH[3] = { 4.0f, 0.0f, 0.0f };

constexpr float ANG_D_ALPHA = 0.70f;
// 角度ループの積分項の上限 [deg/s]
constexpr float ANG_I_LIMIT = 30.0f;

} // namespace Gain

// ============================================================
//  § 2  このステージの設定
// ============================================================
namespace S5 {

constexpr bool USE_MPU   = true;
constexpr bool USE_SBUS  = true;
constexpr bool USE_MOTOR = true;
constexpr bool USE_IM920 = true;   // 地上局(position_estimator)との無線
constexpr bool USE_FLOW  = true;   // PMW3901 オプティカルフロー
constexpr bool USE_RANGE = true;   // s5c: VL53L1X 測距 (高度を flow へ供給)

// s5c: 高度ホールド (スロットルPID)。
//  ★ 運用は2モードのみ:
//      SW_HOVER=down → ANGLE   … 完全手動 (スロットルも手動)。bail-out はこれ。
//      SW_HOVER=up   → POSHOLD … 完全自動 (水平位置ホールド + 高度ホールド)。
//    POSHOLD に入れば高度ホールドは自動で effective。専用スイッチは不要。
//    基準ホバースロットルは Q::ALT_HOVER_THR (実測値)。0 のときは POSHOLD
//    突入時のスロットルスティック値を掴む → ANGLE で安定ホバリングしてから
//    SW_HOVER を上げること。
//  ・USE_ALT_HOLD = false にすると、POSHOLD でも高度は手動のまま
//    (フロー水平ホールドだけ効かせたいとき)。シリアル 'g' でも切替可 (ベンチ用)。
constexpr bool USE_ALT_HOLD = true;

// ★ ドライラン (地上検証用)。true / シリアル 'm' で ON にすると:
//     ・制御パイプラインは通常どおり全部回る (mix() まで計算する)
//     ・ESC へは 0 (アイドル) しか送らない = プロペラが付いていても回らない
//     ・g_out[] (各モーターの「出すはずだった値") はログと printStatus に出る
//   プロペラを外さずに、傾けたときの各モーター配分の符号を確認できる。
//   ※ アーム状態でも一切回らないので、飛ばす前に必ず OFF に戻すこと。
constexpr bool DRY_RUN = false;

// s5b: SW_AUTO UP のとき、地上局AUTO ではなく「フロー位置ホールド」に入る。
//  地上局(position_estimator)を使う構成に戻すときは true にする。
constexpr bool USE_GROUND_AUTO = false;

// 運用は2モード固定。SW_HOVER の1本だけで決める (selectMode 参照):
//     SW_HOVER UP   → POSHOLD (完全自動。フローが死んでいたら ANGLE に自動フォールバック)
//     SW_HOVER 以外 → ANGLE   (完全手動。bail-out)
//   RATE(アクロ) と SW_AUTO/地上局AUTO は封印。どちらの位置も自己水平は効く。
constexpr bool POSHOLD_ON_HOVER_SW = true;  // (歴史的フラグ。今は常に true 運用)

constexpr float I_ENABLE_THR = 0.15f;

// 姿勢テレメトリの送信レート [Hz] (IM920SL の帯域を食い過ぎない範囲で)
constexpr int TELEM_TX_HZ = 10;

// 受信のポーリングレート [Hz]
//  IM920SL は 19200 baud なので 200Hz でも 1回あたり最大12バイト程度。
//  Teensy の受信バッファ(64B)を溢れさせずに、1000Hz の制御ループから
//  String 操作を追い出せる。地上局からの更新は10Hz程度なので十分。
constexpr int TELEM_RX_HZ = 200;

// 角度ループが出せる角速度の上限 [deg/s]
// (大きく傾いたときに、レートループが追えない目標を出さないための蓋)
constexpr float ANGLE_OUT_LIMIT = 300.0f;

constexpr int MAIN_HZ  = Q::RATE_LOOP_HZ;
constexpr int DEBUG_HZ = Q::DEBUG_HZ;

//  MODE_ANGLE    : 完全手動 (自己水平のみ)。SW_HOVER=down。bail-out。
//  MODE_POSHOLD  : 完全自動。フローで水平位置保持 + 測距で高度保持。SW_HOVER=up。
//    センサ喪失の瞬間に ANGLE へ自動フォールバック。
//  MODE_RATE / MODE_AUTO : 封印 (enum は互換のため残置。selectMode は返さない)。
enum Mode : uint8_t { MODE_RATE = 0, MODE_ANGLE = 1, MODE_AUTO = 2, MODE_POSHOLD = 3 };

} // namespace S5

// ============================================================
//  § 3  インスタンス
// ============================================================
IMU   mpu(&Wire);
Sbus  sbus(&Serial5);
motor motors[Q::MOTOR_COUNT];

// IM920SL。drone.cpp と同じ Serial3 に合わせてある。
// 配線が違う場合はここを変えること。
FlightTelemetry telemetry(&Serial3);

// PMW3901 オプティカルフロー。CS ピンは QuadConfig.h の FLOW_CS_PIN。
// グローバル SPI (Teensy 4.0: SCK=13 MOSI=11 MISO=12) を使う。
OpticalFlow flow;

// s5c: 対地距離センサ。QuadConfig の RANGE_BACKEND で ToF(VL53L1X, I2C) と
//  SONAR(MaxBotix LV-MaxSonar-EZ, PW=pin RANGE_SONAR_PW_PIN) を切り替える。
Rangefinder rangefinder;

// s5b/s5d: フロー水平ホールド。速度ループ・位置ループ・地面固定フレーム積分を
//  すべて内包する (quad/PosHold.h)。出力は leanRoll()/leanPitch() [deg]。
Q::PositionHold poshold;

// s5c: 高度ホールドの内側ループ (目標上昇速度[m/s] → スロットル補正[割合])。
Q::Pid alt_rate_pid;

Q::Axis roll_axis, pitch_axis, yaw_axis;

// ============================================================
//  § 4  状態
// ============================================================
namespace {

Q::Attitude g_att;
float       g_out[Q::MOTOR_COUNT] = {0};

S5::Mode g_mode      = S5::MODE_ANGLE;
S5::Mode g_prev_mode = S5::MODE_ANGLE;
bool     g_prev_armed = false;

// 角度ループの間引きカウンタと、実際の経過時間
int      g_angle_div_count = 0;
uint32_t g_angle_prev_us   = 0;

// ---- ヘディングホールド ----
float g_yaw_est   = 0.0f;   // ジャイロZを積分した相対方位 [deg]
float g_yaw_hold  = 0.0f;   // 保持したい相対方位 [deg]
bool  g_yaw_holding = false; // 表示用: いま保持中か

// シリアルから調整できるようにゲインだけ変数で持つ
float g_yaw_hold_kp = Gain::YAW_HOLD_KP;

// ---- ミキサーの報告 (ログ用) ----
Q::MixInfo g_mix;

// ---- オプティカルフロー観測 ----
bool  g_flow_ok    = false;   // PMW3901 が初期化できたか
float g_flow_raw_x = 0.0f, g_flow_raw_y = 0.0f;  // 機体座標の生カウント [px]
float g_flow_dx    = 0.0f, g_flow_dy    = 0.0f;  // de-rotation 後 [px]
float g_flow_vx    = 0.0f, g_flow_vy    = 0.0f;  // 対地速度 [m/s] (前 +, 右 +)
float g_flow_vx_f  = 0.0f, g_flow_vy_f  = 0.0f;  // 表示用にLPFした速度 (ログは生値)

// ---- s5b/s5d: フロー水平ホールドの状態は Q::PositionHold (poshold) が保持 ----
//  以前ここにあった g_flow_vx_ctl / g_pos_* / g_flow_lean_* などのグローバルは
//  すべて quad/PosHold.h の中に移した。参照は poshold.xxx() を使う。

// シリアルから調整するゲイン (QuadConfig.h の初期値から起動時にコピー)
float g_flow_vel_kp = Q::FLOW_VEL_KP;
float g_flow_vel_ki = Q::FLOW_VEL_KI;
float g_flow_pos_kp = Q::FLOW_POS_KP;

// ---- キャリブレーション用の積算 ----
//  既知距離をゆっくり動かし、acc_px と実測距離を突き合わせて
//  QuadConfig.h の FLOW_PX_PER_RAD を決める ([z] でゼロ)。
double g_flow_acc_raw_x = 0.0, g_flow_acc_raw_y = 0.0; // 生ピクセルの積算 (de-rotation 前)
double g_flow_acc_px_x  = 0.0, g_flow_acc_px_y  = 0.0; // de-rotation 後ピクセルの積算
double g_flow_acc_m_x   = 0.0, g_flow_acc_m_y   = 0.0; // 推定変位の積算 [m]

// ---- s5c: 距離センサ (VL53L1X) 観測 ----
bool  g_range_ok    = false;   // VL53L1X が初期化できたか
bool  g_range_valid = false;   // 高度が信用できるか (失探すると false)
float g_range_raw_m = 0.0f;    // 傾き補正前の斜め距離 [m]
float g_range_h_m   = 0.0f;    // 鉛直対地高度 [m] (flow.setHeight() に渡す値)
float g_climb_mps   = 0.0f;    // 上昇速度 [m/s] (上 +)

// ---- s5c: 高度ホールド ----
bool  g_alt_hold_enable = S5::USE_ALT_HOLD;  // シリアル 'g' でトグル
bool  g_alt_active   = false;   // いま実際にスロットルを握っているか
float g_alt_hold_m   = 0.0f;    // 保持したい高度 [m]
float g_alt_thr_base = 0.0f;    // POSHOLD 突入時に掴んだホバースロットル [割合]
float g_alt_vz_tar   = 0.0f;    // 目標上昇速度 [m/s]
float g_alt_thr_corr = 0.0f;    // PID が出したスロットル補正 [割合]
float g_alt_thr_out  = 0.0f;    // 最終スロットル指令 [割合] (active 時のみ使用)

// シリアルから調整するゲイン (QuadConfig.h の初期値から起動時にコピー)
float g_alt_pos_kp = Q::ALT_POS_KP;   // 高度誤差[m] → 目標上昇速度[m/s]

// ドライラン: true の間は ESC へ 0 しか送らない (g_out は計算・記録する)
bool  g_dry_run = S5::DRY_RUN;

} // anonymous namespace

// ============================================================
//  § 4-2  ログ出力 (USBシリアル → scripts/logger.py)
// ============================================================
//  drone.cpp から移植。logger.py / analyze_log.py が期待するプロトコル:
//    HEADER,<列名>   ... LOG_START の直前に1回
//    LOG_START       ... 記録開始
//    DATA,<値,...>   ... 1サンプル
//    LOG_STOP        ... 記録終了
//  シリアルで 'l' を送るたびに 開始/停止 が切り替わる。
//
//  ★列名は drone.cpp と完全に同じにしてある。
//    scripts/analyze_log.py を一切変更せずにそのまま使える。
//
//  10Hzの画面表示では数十Hzの振動は絶対に見えないので、
//  制御ループと同じ土俵(500Hz)で全信号を落とす。
namespace Log {

constexpr int  LOG_HZ = 500;                    // 記録レート [Hz]
constexpr int  DIV    = S5::MAIN_HZ / LOG_HZ;   // メインループ何回に1回
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
    "range_ok,range_raw,range_h,climb,alt_en,alt_act,alt_hold,alt_vzt,alt_base,alt_corr,alt_thr";

bool     active  = false;
uint32_t dropped = 0;   // USBが詰まって捨てたサンプル数
uint32_t written = 0;
int      div_cnt = 0;

inline void start() {
    Serial.println();
    Serial.print("HEADER,"); Serial.println(HEADER);
    Serial.println("LOG_START");
    active  = true;
    dropped = 0;
    written = 0;
    div_cnt = 0;
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
inline void sample(uint32_t dt_us, int mode, bool armed, float thr) {
    if (!active) return;
    if (++div_cnt < DIV) return;
    div_cnt = 0;

    // 1行ぶん(約160バイト)の空きが無ければ捨てる
    if (Serial.availableForWrite() < 200) { dropped++; return; }

    // yaw_ang にはヘディングホールドの積分値を入れる。
    // (Madgwick の6軸ヨーは意味を持たないので記録しても仕方がない)
    Serial.printf(
        "DATA,%lu,%lu,%d,%d,%.3f,"
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
        "%d,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,%.3f,%.4f,%.3f\n",
        (unsigned long)millis(), (unsigned long)dt_us, mode, armed ? 1 : 0, thr,
        roll_axis.stick,     pitch_axis.stick,     yaw_axis.stick,
        roll_axis.ang_meas,  pitch_axis.ang_meas,  g_yaw_est,
        roll_axis.rate_meas, pitch_axis.rate_meas, yaw_axis.rate_meas,
        roll_axis.cmd,       pitch_axis.cmd,       yaw_axis.cmd,
        g_out[0], g_out[1], g_out[2], g_out[3],
        g_mix.span_limit, (unsigned)g_mix.sat,
        roll_axis.rate_tar,  pitch_axis.rate_tar,
        roll_axis.ang_tar,   pitch_axis.ang_tar,
        g_flow_ok ? 1 : 0,
        g_flow_raw_x, g_flow_raw_y, g_flow_dx, g_flow_dy,
        g_flow_vx, g_flow_vy, flow.height(),
        (double)g_flow_acc_m_x, (double)g_flow_acc_m_y,
        poshold.vxCtl(), poshold.vyCtl(), poshold.vxTar(), poshold.vyTar(),
        poshold.leanRoll(), poshold.leanPitch(), poshold.posN(), poshold.posE(),
        poshold.holdN(), poshold.holdE(), poshold.holding() ? 1 : 0,
        g_range_ok ? 1 : 0, g_range_raw_m, g_range_h_m, g_climb_mps,
        g_alt_hold_enable ? 1 : 0, g_alt_active ? 1 : 0,
        g_alt_hold_m, g_alt_vz_tar, g_alt_thr_base, g_alt_thr_corr, g_alt_thr_out);
    written++;
}

} // namespace Log

// ============================================================
//  § 5  周期実行のヘルパ
// ============================================================
struct Ticker {
    uint32_t period_us;
    uint32_t prev_us = 0;
    uint32_t dt_us   = 0;
    explicit Ticker(uint32_t hz) : period_us(1000000UL / hz) {}
    bool ready() {
        const uint32_t now = micros();
        if (now - prev_us < period_us) return false;
        dt_us   = now - prev_us;
        prev_us = now;
        return true;
    }
};

static Ticker main_tick (S5::MAIN_HZ);
static Ticker debug_tick(S5::DEBUG_HZ);
static Ticker telem_tick(S5::TELEM_TX_HZ);
static Ticker telem_rx_tick(S5::TELEM_RX_HZ);
static Ticker flow_tick (Q::FLOW_LOOP_HZ);   // オプティカルフロー読み出し (FLOW_LOOP_HZ)
static Ticker range_tick(Q::RANGE_LOOP_HZ);  // s5c: VL53L1X ポーリング (RANGE_LOOP_HZ)

// ============================================================
//  § 6  出力
// ============================================================
static void writeMotors() {
    if (!S5::USE_MOTOR) return;
    // ドライラン中は g_out をそのまま残して (ログ/表示用)、ESC へは 0 だけ送る。
    if (g_dry_run) {
        for (int i = 0; i < Q::MOTOR_COUNT; ++i) motors[i].write(0.0f);
        return;
    }
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) motors[i].write(g_out[i]);
}

static void stopAllMotors() {
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) g_out[i] = 0.0f;
    g_mix = Q::MixInfo{};
    writeMotors();
}

static bool isArmed() {
    if (!S5::USE_SBUS) return false;
    if (!sbus.isSafe()) return false;
    return sbus.Ch_state(Ch::THR_CUT) == Q::ARM_SWITCH_STATE;
}

// PIDの内部状態と目標値の両方をクリアする。
// 旧コードは目標値 (tar) を残していたため、モードをまたいで
// 前のモードの目標角が生き残っていました。
static void resetControllers() {
    roll_axis.reset();
    pitch_axis.reset();
    yaw_axis.reset();
    g_angle_div_count = 0;
    g_angle_prev_us   = micros();

    // ヘディングホールドも「今の向き」を基準に取り直す。
    // 相対値なので 0 に戻すだけでよい。
    g_yaw_est     = 0.0f;
    g_yaw_hold    = 0.0f;
    g_yaw_holding = false;

    // s5b/s5d: フロー水平ホールドも「今ここ」を基準に取り直す。
    poshold.reset();

    // s5c: 高度ホールドも「今ここ」を基準に取り直す。
    alt_rate_pid.reset();
    g_alt_active   = false;
    g_alt_hold_m   = g_range_valid ? g_range_h_m : Q::FLOW_ASSUMED_HEIGHT_M;
    g_alt_thr_base = 0.0f;
    g_alt_vz_tar   = 0.0f;
    g_alt_thr_corr = 0.0f;
    g_alt_thr_out  = 0.0f;
}

static const char* modeName(S5::Mode m) {
    switch (m) {
        case S5::MODE_AUTO:    return "AUTO   (地上局)";
        case S5::MODE_POSHOLD: return "POSHOLD(フロー位置保持)";
        case S5::MODE_ANGLE:   return "ANGLE  (水平維持)";
        default:               return "RATE   (アクロ)";
    }
}

// ------------------------------------------------------------
//  モード判定  — 2モードのみ。SW_HOVER の1本だけで決める。
//
//    SW_HOVER=up かつ フロー生存 → POSHOLD (完全自動: 水平位置 + 高度)
//    それ以外                    → ANGLE   (完全手動。これが bail-out)
//
//  ・SW_AUTO / 地上局AUTO / RATE(アクロ) は使わない (封印)。
//  ・フローが死んでいるときは SW_HOVER=up でも ANGLE に落とす (安全側)。
//  ・スロットルカット (THR_CUT) は selectMode より上位で、常にモーターを止める。
// ------------------------------------------------------------
static S5::Mode selectMode() {
    if (!S5::USE_SBUS) return S5::MODE_ANGLE;

    if (sbus.Ch_state(Ch::SW_HOVER) == up && S5::USE_FLOW && g_flow_ok)
        return S5::MODE_POSHOLD;
    return S5::MODE_ANGLE;
}

// ------------------------------------------------------------
//  § 6-2  s5b/s5d : フロー水平ホールド  (FLOW_LOOP_HZ で呼ぶ)
//
//   中身は quad/PosHold.h (Q::PositionHold) に移した。ここは
//   「効かせる条件」と「スティックの取り込み」だけを組み立てて渡す薄い層。
//
//   出力は poshold.leanRoll() / leanPitch() [deg]。updateControl() が POSHOLD の
//   ときに角度ループの目標角として使う。スロットルは触らない (高度は § 6-3)。
//
//   ★ active の条件に注意: アーム済 && POSHOLD && スロットル > FLOW_ENABLE_THR。
//     地上でスロットルを下げたままだとフロー制御は一切出力しない (安全)。
//     ドライラン('m')で符号を確認するときも、スティックを上げる必要がある。
// ------------------------------------------------------------
static void updateFlowHold(float dt_s) {
    const float thr = S5::USE_SBUS ? constrain(sbus.des[Ch::THR], 0.0f, 1.0f) : 0.0f;
    const bool  active = isArmed() && (g_mode == S5::MODE_POSHOLD)
                      && (thr > Q::FLOW_ENABLE_THR)
                      && S5::USE_FLOW && g_flow_ok;

    // スティックは機体座標のまま渡す (パイロットの前後左右 = 機首基準)
    const float sx = Q::STICK_SIGN_PITCH * sbus.des[Ch::PITCH];
    const float sy = Q::STICK_SIGN_ROLL  * sbus.des[Ch::ROLL];

    // s5d: 位置積分を地面固定フレームで行うため、ヘディングを渡す。
    //  g_yaw_est はアーム時を 0 とした相対方位 (§ 7-2 が積分している)。
    poshold.update(dt_s, flow.vx, flow.vy, g_yaw_est, sx, sy, active);
}

// ------------------------------------------------------------
//  § 6-3  s5c : 高度ホールド  (RANGE_LOOP_HZ で呼ぶ)
//
//   高度誤差[m] ─[ALT_POS_KP]→ 目標上昇速度[m/s] ─[ALT_RATE PID]→
//     スロットル補正[割合] → (突入時に掴んだホバースロットル) + 補正
//
//   出力は g_alt_thr_out。updateControl() が「active かつ POSHOLD」の
//   ときだけ、これを手動スロットルの代わりにミキサーへ渡す。
//   それ以外は g_alt_thr_out は手動スロットルに追従させて bumpless にする。
// ------------------------------------------------------------
static void updateAltHold(float dt_s) {
    if (dt_s <= 0.0f) return;

    const bool  armed = isArmed();
    const float thr   = S5::USE_SBUS ? constrain(sbus.des[Ch::THR], 0.0f, 1.0f) : 0.0f;

    // POSHOLD は完全自動。モードに入っていて浮いていて測距が生きていれば effective。
    //  抜けるのは SW_HOVER=down (→ANGLE) か THR_CUT のみ。
    const bool  want  = g_alt_hold_enable && armed
                     && (g_mode == S5::MODE_POSHOLD)
                     && (thr > Q::ALT_ENABLE_THR)
                     && S5::USE_RANGE && g_range_valid;

    if (!want) {
        alt_rate_pid.reset();
        g_alt_vz_tar   = 0.0f;
        g_alt_thr_corr = 0.0f;

        // ★ POSHOLD 継続中に「測距だけ」を失った場合 (超音波はプロペラ後流で
        //   一時的に飛ぶことがある): スロットルをスティック値へいきなり戻すと
        //   落下する。直前のホバースロットルで保持し続け、手動復帰は
        //   SW_HOVER を下げてもらう。
        const bool lost_range_only = g_alt_active
                                  && (g_mode == S5::MODE_POSHOLD)
                                  && armed
                                  && (g_alt_thr_base > 0.05f);
        if (lost_range_only) {
            g_alt_thr_out = g_alt_thr_base;   // g_alt_active は true のまま
            return;
        }

        // 通常の非active (ANGLE など): 手動スロットルへ bumpless 追従
        g_alt_active   = false;
        g_alt_thr_base = thr;
        g_alt_hold_m   = g_range_valid ? g_range_h_m : g_alt_hold_m;
        g_alt_thr_out  = thr;
        return;
    }

    if (!g_alt_active) {
        // inactive → active:
        //  基準スロットル = 実測ホバースロットル (ALT_HOVER_THR>0)。
        //                   未設定なら POSHOLD 突入時のスティック値を掴む
        //                   (→ ANGLE で安定ホバリングしてから SW_HOVER を上げること)。
        //  保持高度       = ALT_TARGET_M (>0) か、0 なら突入時の実測高度。
        g_alt_active   = true;
        g_alt_thr_base = (Q::ALT_HOVER_THR > 0.01f) ? Q::ALT_HOVER_THR : thr;
        g_alt_hold_m   = (Q::ALT_TARGET_M   > 0.0f)  ? Q::ALT_TARGET_M  : g_range_h_m;
        alt_rate_pid.reset();
    }

    // スロットルスティックをホバー基準からずらしている間は上昇/下降速度指令。
    // 戻すとその高度を保持。
    const float thr_dev = thr - g_alt_thr_base;
    if (fabsf(thr_dev) > Q::ALT_STICK_DEAD) {
        g_alt_vz_tar = thr_dev * Q::ALT_STICK_VZ;
        g_alt_hold_m = g_range_h_m;               // 目標高度を今へ張り付け
    } else {
        g_alt_vz_tar = constrain(g_alt_pos_kp * (g_alt_hold_m - g_range_h_m),
                                 -Q::ALT_POS_VZ_LIM, Q::ALT_POS_VZ_LIM);
    }

    // 内側: 上昇速度PID → スロットル補正
    g_alt_thr_corr = constrain(
        alt_rate_pid.update(g_alt_vz_tar, g_climb_mps, dt_s, true),
        -Q::ALT_THR_AUTH, Q::ALT_THR_AUTH);

    // ホバースロットル ± 補正。完全停止しないよう下限を残す。
    g_alt_thr_out = constrain(g_alt_thr_base + g_alt_thr_corr, 0.05f, 1.0f);
}

// ============================================================
//  § 7  制御
// ============================================================
static void updateControl(float dt_s) {
    // --- モード判定 ---
    g_mode = selectMode();

    const bool armed = isArmed();

    // --- アーム状態やモードが変わったらクリア ---
    if (armed != g_prev_armed) {
        resetControllers();
        g_prev_armed = armed;
        Serial.println(armed ? "\n>>> ARMED" : "\n>>> DISARMED");
    }
    if (g_mode != g_prev_mode) {
        resetControllers();
        g_prev_mode = g_mode;
        Serial.printf("\n>>> MODE = %s\n", modeName(g_mode));
    }

    if (!armed) { stopAllMotors(); return; }

    // s5c: 高度ホールドが active なら、ミキサーへ渡すスロットルを
    //  ホバースロットル±PID補正 (g_alt_thr_out) に差し替える。
    //  それ以外は従来どおり物理プロポのスロットルをそのまま使う。
    const float thr_stick = constrain(sbus.des[Ch::THR], 0.0f, 1.0f);
    const float thr = g_alt_active ? g_alt_thr_out : thr_stick;
    const bool  integrate = (thr > S5::I_ENABLE_THR);

    // --- 測定値 ---
    roll_axis.rate_meas  = g_att.roll_rate;
    pitch_axis.rate_meas = g_att.pitch_rate;
    yaw_axis.rate_meas   = g_att.yaw_rate;

    roll_axis.ang_meas  = g_att.roll;
    pitch_axis.ang_meas = g_att.pitch;
    // yaw_axis.ang_meas は § 7-2 でヘディングホールドの積分値を入れる。
    // g_att.yaw (Madgwick の6軸ヨー) は絶対方位として意味を持たないので使わない。

    // ------------------------------------------------------------
    //  スティック入力
    //
    //  AUTO モードでは、地上局(position_estimator)が計算した RC相当コマンドを
    //  スティック値の「代わりに」使う。以降の制御経路は ANGLE モードと完全に
    //  同じなので、自律制御専用のPID経路は存在しない。
    //
    //  ★ スロットルだけは全モード共通で常に物理プロポから取る (下の thr)。
    //    このコード全体の安全上の不変条件として崩さない。
    //    g.ap_throttle はあえて使わない。
    // ------------------------------------------------------------
    if (g_mode == S5::MODE_AUTO) {
        const GroundData& g = telemetry.lastGroundData();
        roll_axis.stick  = constrain(g.ap_roll,  -1.0f, 1.0f);
        pitch_axis.stick = constrain(g.ap_pitch, -1.0f, 1.0f);
        yaw_axis.stick   = constrain(g.ap_yaw,   -1.0f, 1.0f);
    } else {
        roll_axis.stick  = Q::STICK_SIGN_ROLL  * sbus.des[Ch::ROLL];
        pitch_axis.stick = Q::STICK_SIGN_PITCH * sbus.des[Ch::PITCH];
        yaw_axis.stick   = Q::STICK_SIGN_YAW   * sbus.des[Ch::YAW];
    }

    // ------------------------------------------------------------
    //  外側ループ: 角度 → 目標角速度   (ANGLE / AUTO / POSHOLD, 200Hz)
    // ------------------------------------------------------------
    if (g_mode == S5::MODE_ANGLE || g_mode == S5::MODE_AUTO ||
        g_mode == S5::MODE_POSHOLD) {

        if (g_mode == S5::MODE_POSHOLD) {
            // 目標角は updateFlowHold() が 50Hz で計算済み (すでにクランプ済み)。
            // スティックはそこで「目標速度」として使っている。
            roll_axis.ang_tar  = poshold.leanRoll();
            pitch_axis.ang_tar = poshold.leanPitch();
        } else {
            roll_axis.ang_tar  = roll_axis.stick  * Q::MAX_ANGLE_ROLL;
            pitch_axis.ang_tar = pitch_axis.stick * Q::MAX_ANGLE_PITCH;
        }

        if (++g_angle_div_count >= Q::ANGLE_LOOP_DIV) {
            g_angle_div_count = 0;

            // ★ 実測の経過時間を渡す。
            //   旧コードはここで常に 1ms (メインループのdt) を使っていたため、
            //   角度ループの I項と D項が 5倍ずれていました。
            const uint32_t now = micros();
            const float ang_dt_s = (float)(now - g_angle_prev_us) * 1e-6f;
            g_angle_prev_us = now;

            roll_axis.rate_tar = constrain(
                roll_axis.angle.update(roll_axis.ang_tar, roll_axis.ang_meas,
                                       ang_dt_s, integrate),
                -S5::ANGLE_OUT_LIMIT, S5::ANGLE_OUT_LIMIT);

            pitch_axis.rate_tar = constrain(
                pitch_axis.angle.update(pitch_axis.ang_tar, pitch_axis.ang_meas,
                                        ang_dt_s, integrate),
                -S5::ANGLE_OUT_LIMIT, S5::ANGLE_OUT_LIMIT);
        }
        // 間引かれたループでは、前回の rate_tar をそのまま使う
    } else {
        // RATE モード: スティックが直接、目標角速度になる
        roll_axis.ang_tar  = 0.0f;
        pitch_axis.ang_tar = 0.0f;
        roll_axis.rate_tar  = roll_axis.stick  * Q::MAX_RATE_ROLL;
        pitch_axis.rate_tar = pitch_axis.stick * Q::MAX_RATE_PITCH;
    }

    // ------------------------------------------------------------
    //  § 7-2  ヨー: ヘディングホールド
    //
    //  ・スティックを触っている間 → 素直にレート指令 (従来どおり)
    //  ・中立に戻した瞬間          → そのときの方位を目標として保持
    //
    //  レートPIDの I項 (§1) は「回転速度を0にする」ところまでしか
    //  保証しない。突風で30度振られたら、その30度は戻ってこない。
    //  振られた分まで戻したいので、相対方位の外側ループを足す。
    // ------------------------------------------------------------
    //  積分は「制御に使っているジャイロ値」をそのまま使う。
    //  こうするとレートループと位相が完全に揃う。
    g_yaw_est += yaw_axis.rate_meas * dt_s;

    const bool yaw_stick_active = (fabsf(yaw_axis.stick) > Gain::YAW_STICK_DEAD);

    if (yaw_stick_active || !integrate) {
        // 操作中、または低スロットル(地上)。
        // 目標方位を現在値に追従させておくことで、スティックを離した
        // 瞬間から「今の向き」の保持が始まる。
        yaw_axis.rate_tar = yaw_axis.stick * Q::MAX_RATE_YAW;
        g_yaw_hold        = g_yaw_est;
        g_yaw_holding     = false;
    } else {
        // 中立: 保持方位との差を消しにいく
        const float err = constrain(g_yaw_hold - g_yaw_est,
                                    -Gain::YAW_HOLD_ERR_LIM,
                                    +Gain::YAW_HOLD_ERR_LIM);
        yaw_axis.rate_tar = constrain(g_yaw_hold_kp * err,
                                      -Gain::YAW_HOLD_RATE_LIM,
                                      +Gain::YAW_HOLD_RATE_LIM);
        g_yaw_holding     = true;
    }

    // 表示用 (ヨーには角度PIDを通していないが、保持誤差をここに入れておく)
    yaw_axis.ang_tar  = g_yaw_hold;
    yaw_axis.ang_meas = g_yaw_est;

    // ------------------------------------------------------------
    //  内側ループ: 角速度 → トルク指令   (1000Hz)
    // ------------------------------------------------------------
    roll_axis.cmd  = roll_axis.rate .update(roll_axis.rate_tar,  roll_axis.rate_meas,  dt_s, integrate);
    pitch_axis.cmd = pitch_axis.rate.update(pitch_axis.rate_tar, pitch_axis.rate_meas, dt_s, integrate);
    yaw_axis.cmd   = yaw_axis.rate  .update(yaw_axis.rate_tar,   yaw_axis.rate_meas,   dt_s, integrate);

    // --- ミキサー ---
    Q::mix(thr, roll_axis.cmd, pitch_axis.cmd, yaw_axis.cmd, g_out, &g_mix);
    writeMotors();
}

// ============================================================
//  § 8  シリアル (ゲイン調整)
// ============================================================
static void tuningMenu() {
    stopAllMotors();
    resetControllers();

    const unsigned long old_timeout = Serial.getTimeout();
    Serial.setTimeout(30000);
    while (Serial.available()) Serial.read();

    Serial.println("\n\n!! モーターを停止しました。制御ループも止まっています !!");
    Serial.println("========== PID Tuning ==========");
    Serial.println("-- Rate (内側) --");
    Serial.printf(" [1] Roll  P %9.5f  [2] Roll  I %9.5f  [3] Roll  D %9.5f\n",
                  roll_axis.rate.kp(), roll_axis.rate.ki(), roll_axis.rate.kd());
    Serial.printf(" [4] Pitch P %9.5f  [5] Pitch I %9.5f  [6] Pitch D %9.5f\n",
                  pitch_axis.rate.kp(), pitch_axis.rate.ki(), pitch_axis.rate.kd());
    Serial.printf(" [7] Yaw   P %9.5f  [8] Yaw   I %9.5f  [9] Yaw   D %9.5f\n",
                  yaw_axis.rate.kp(), yaw_axis.rate.ki(), yaw_axis.rate.kd());
    Serial.println("-- Angle (外側) --");
    Serial.printf(" [a] Roll  P %9.4f  [b] Roll  I %9.4f  [c] Roll  D %9.4f\n",
                  roll_axis.angle.kp(), roll_axis.angle.ki(), roll_axis.angle.kd());
    Serial.printf(" [d] Pitch P %9.4f  [e] Pitch I %9.4f  [f] Pitch D %9.4f\n",
                  pitch_axis.angle.kp(), pitch_axis.angle.ki(), pitch_axis.angle.kd());
    Serial.println("-- ヘディングホールド (ヨー) --");
    Serial.printf(" [h] Hold  P %9.4f   (0 にすると保持を切って従来のレートのみになる)\n",
                  g_yaw_hold_kp);
    Serial.println("-- s5b フロー位置ホールド --");
    Serial.printf(" [i] Vel P %9.4f  [j] Vel I %9.4f  [o] Pos P %9.4f\n",
                  g_flow_vel_kp, g_flow_vel_ki, g_flow_pos_kp);
    Serial.println("-- s5c 高度ホールド --");
    Serial.printf(" [s] Pos P %9.4f  [t] Rate P %9.4f  [u] Rate I %9.4f  [v] Rate D %9.5f\n",
                  g_alt_pos_kp, alt_rate_pid.kp(), alt_rate_pid.ki(), alt_rate_pid.kd());
    Serial.println(" [q] 抜ける");
    Serial.print("選択 > ");

    while (!Serial.available()) { /* 入力待ち */ }
    String sel = Serial.readStringUntil('\n');
    sel.trim();
    sel.toLowerCase();

    if (sel.length() == 0 || sel[0] == 'q') {
        Serial.setTimeout(old_timeout);
        Serial.println("\n再開します。");
        return;
    }

    Serial.print("新しい値 > ");
    const float v = Serial.parseFloat();
    Serial.println(v, 5);

    // ヘディングホールドは Pid クラスではないので先に処理する
    if (sel[0] == 'h') {
        g_yaw_hold_kp = constrain(v, 0.0f, 20.0f);
        Serial.printf("更新: ヘディングホールド kp=%.4f\n", g_yaw_hold_kp);
        resetControllers();
        Serial.setTimeout(old_timeout);
        Serial.println("再開します。");
        return;
    }

    // s5b フロー位置ホールドのゲイン (vx/vy 両方に同じ値を入れる)
    if (sel[0] == 'i' || sel[0] == 'j' || sel[0] == 'o') {
        if (sel[0] == 'i') g_flow_vel_kp = constrain(v, 0.0f, 40.0f);
        if (sel[0] == 'j') g_flow_vel_ki = constrain(v, 0.0f, 20.0f);
        if (sel[0] == 'o') g_flow_pos_kp = constrain(v, 0.0f, 5.0f);
        poshold.setVelGains(g_flow_vel_kp, g_flow_vel_ki, Q::FLOW_VEL_KD);
        poshold.setPosKp(g_flow_pos_kp);
        Serial.printf("更新: Flow vel P=%.3f I=%.3f  pos P=%.3f\n",
                      g_flow_vel_kp, g_flow_vel_ki, g_flow_pos_kp);
        resetControllers();
        Serial.setTimeout(old_timeout);
        Serial.println("再開します。");
        return;
    }

    // s5c 高度ホールドのゲイン
    if (sel[0] == 's' || sel[0] == 't' || sel[0] == 'u' || sel[0] == 'v') {
        if (sel[0] == 's') g_alt_pos_kp = constrain(v, 0.0f, 5.0f);
        if (sel[0] == 't') alt_rate_pid.set_gains(constrain(v, 0.0f, 2.0f),
                                                  alt_rate_pid.ki(), alt_rate_pid.kd());
        if (sel[0] == 'u') alt_rate_pid.set_gains(alt_rate_pid.kp(),
                                                  constrain(v, 0.0f, 2.0f), alt_rate_pid.kd());
        if (sel[0] == 'v') alt_rate_pid.set_gains(alt_rate_pid.kp(), alt_rate_pid.ki(),
                                                  constrain(v, 0.0f, 1.0f));
        Serial.printf("更新: Alt pos P=%.3f  rate P=%.3f I=%.3f D=%.5f\n",
                      g_alt_pos_kp, alt_rate_pid.kp(), alt_rate_pid.ki(), alt_rate_pid.kd());
        resetControllers();
        Serial.setTimeout(old_timeout);
        Serial.println("再開します。");
        return;
    }

    Q::Pid* pid = nullptr;
    int which = -1;   // 0=P 1=I 2=D
    switch (sel[0]) {
        case '1': pid = &roll_axis.rate;   which = 0; break;
        case '2': pid = &roll_axis.rate;   which = 1; break;
        case '3': pid = &roll_axis.rate;   which = 2; break;
        case '4': pid = &pitch_axis.rate;  which = 0; break;
        case '5': pid = &pitch_axis.rate;  which = 1; break;
        case '6': pid = &pitch_axis.rate;  which = 2; break;
        case '7': pid = &yaw_axis.rate;    which = 0; break;
        case '8': pid = &yaw_axis.rate;    which = 1; break;
        case '9': pid = &yaw_axis.rate;    which = 2; break;
        case 'a': pid = &roll_axis.angle;  which = 0; break;
        case 'b': pid = &roll_axis.angle;  which = 1; break;
        case 'c': pid = &roll_axis.angle;  which = 2; break;
        case 'd': pid = &pitch_axis.angle; which = 0; break;
        case 'e': pid = &pitch_axis.angle; which = 1; break;
        case 'f': pid = &pitch_axis.angle; which = 2; break;
        default: break;
    }

    if (pid) {
        const float p = (which == 0) ? v : pid->kp();
        const float i = (which == 1) ? v : pid->ki();
        const float d = (which == 2) ? v : pid->kd();
        pid->set_gains(p, i, d);
        Serial.printf("更新: P=%.5f I=%.5f D=%.5f\n", p, i, d);
    } else {
        Serial.println("不明な選択です");
    }

    resetControllers();
    Serial.setTimeout(old_timeout);
    Serial.println("再開します。");
}

// I2C バススキャン。0x08..0x77 を叩いて ACK を返したアドレスを列挙する。
//  期待値:  0x29 = VL53L1X (距離)   0x68 = MPU6050 (IMU)
//  0x68 だけ見えて 0x29 が見えない → VL53L1X が配線に乗れていない
//    (SDA/SCL 断線・VIN 断線・XSHUT が Low・センサ故障)。
//  両方見えない → バスが Low に張り付いている (モジュール故障 or 電源ショート)。
static void i2cScan() {
    Serial.println("\n--- I2C scan (Wire: SDA=18 / SCL=19) ---");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        Wire.beginTransmission(addr);
        const uint8_t err = Wire.endTransmission();
        if (err == 0) {
            const char *tag = (addr == 0x29) ? "  <- VL53L1X (距離)"
                            : (addr == 0x68) ? "  <- MPU6050 (IMU)"
                                             : "";
            Serial.printf("  0x%02X  ACK%s\n", addr, tag);
            ++found;
        }
    }
    if (found == 0)
        Serial.println("  応答なし。SDA/SCL が Low 固着 or 全モジュール断線。");
    Serial.printf("--- %d device(s) ---\n", found);
}

static void handleSerial() {
    if (!Serial.available()) return;
    const char c = (char)tolower(Serial.read());

    switch (c) {
        case 'p':
            tuningMenu();
            break;
        case 'i':
            i2cScan();
            break;
        case 'm':
            g_dry_run = !g_dry_run;
            stopAllMotors();
            Serial.printf("\n>>> ドライラン = %s\n",
                          g_dry_run
                            ? "ON  (ESCへは0のみ。g_out は計算/記録。飛行前に必ずOFF)"
                            : "OFF (通常。モーターが回る)");
            break;
        case 'k':
            stopAllMotors();
            Serial.println("CALIBRATE: 機体を水平に置いて動かさないでください");
            if (S5::USE_MPU) mpu.recalibrate();
            resetControllers();
            break;
        case 'r':
            resetControllers();
            Serial.println("PID reset");
            break;
        case 'l':
            // 500Hz ログの開始/停止。scripts/logger.py と対で使う。
            Log::toggle();
            break;
        case 'g': {
            // s5c: 高度ホールド (スロットルPID) の ON/OFF トグル。
            //  ★ OFF にした瞬間、スロットルは物理プロポの値へ戻る。
            //    ホバー中に切るときは、スロットルスティックを今の
            //    ホバー位置に戻してから 'g' を押すこと (段差防止)。
            g_alt_hold_enable = !g_alt_hold_enable;
            if (!g_alt_hold_enable) { g_alt_active = false; alt_rate_pid.reset(); }
            Serial.printf("\n>>> 高度ホールド = %s%s\n",
                          g_alt_hold_enable ? "ON" : "OFF (スロットル手動)",
                          (g_alt_hold_enable && !g_range_valid)
                            ? "  ※ただし今は測距が無効なので効きません" : "");
            break;
        }
        case 'z':
            // フロー積算のゼロ。既知距離キャリブレーションの開始点。
            g_flow_acc_raw_x = g_flow_acc_raw_y = 0.0;
            g_flow_acc_px_x  = g_flow_acc_px_y  = 0.0;
            g_flow_acc_m_x   = g_flow_acc_m_y   = 0.0;
            Serial.println("FLOW: 積算をゼロにしました");
            break;
        case 'h': {
            // フローのスケール高度 [m] を手で入れる。
            // s5c 以降は VL53L1X が有効なら毎ループ上書きするので、これは
            // 測距を切っている / 失探しているときの手動フォールバック用。
            const unsigned long old_to = Serial.getTimeout();
            Serial.setTimeout(15000);
            Serial.print("\nフロー高度 h [m] > ");
            const float v = Serial.parseFloat();
            Serial.println(v, 3);
            flow.setHeight(v);
            Serial.printf("FLOW: h = %.3f m にしました", flow.height());
            if (S5::USE_RANGE && g_range_valid)
                Serial.print(" (※測距が有効なので次のループで上書きされます)");
            Serial.println();
            Serial.setTimeout(old_to);
            break;
        }
        default:
            break;
    }
}

// ============================================================
//  § 9  デバッグ表示
// ============================================================
static void printStatus(uint32_t dt_us) {
    Serial.print("\033[2J\033[H");
    Serial.println("=== Stage 5d : 1スイッチ完全自動ホバリング ===");
    if (g_dry_run)
        Serial.println(">>> DRY-RUN: ESCへは0のみ (モーターは回らない)  ['m']で解除 <<<");
    Serial.printf("loop dt = %6lu us (%6.1f Hz)   %s   link=%s\n",
                  (unsigned long)dt_us, 1000000.0f / (float)dt_us,
                  isArmed() ? "ARMED" : "DISARMED",
                  sbus.isSafe() ? "OK" : "LOST");
    Serial.printf("MODE = %s   (SW_HOVER UP=POSHOLD[完全自動] / それ以外=ANGLE[手動])\n",
                  modeName(g_mode));

    if (S5::USE_IM920) {
        const GroundData& g = telemetry.lastGroundData();
        Serial.printf("IM920 link=%s   AP: roll=%+.3f pitch=%+.3f yaw=%+.3f (thr=%+.3f 未使用)\n",
                      telemetry.groundLinkFresh() ? "FRESH" : "STALE",
                      g.ap_roll, g.ap_pitch, g.ap_yaw, g.ap_throttle);
    }

    if (g_mode == S5::MODE_ANGLE || g_mode == S5::MODE_AUTO ||
        g_mode == S5::MODE_POSHOLD) {
        Serial.println("\n[角度ループ]  目標[deg]  実測[deg]  → 角速度目標[deg/s]");
        Serial.printf("  roll  %10.1f %10.1f %18.1f\n",
                      roll_axis.ang_tar, roll_axis.ang_meas, roll_axis.rate_tar);
        Serial.printf("  pitch %10.1f %10.1f %18.1f\n",
                      pitch_axis.ang_tar, pitch_axis.ang_meas, pitch_axis.rate_tar);
    }

    if (g_mode == S5::MODE_POSHOLD) {
        Serial.printf("\n[フロー位置ホールド] %s  vel P=%.2f I=%.2f  pos P=%.2f  失探=%d\n",
                      poshold.holding() ? "HOLD " : "STICK",
                      g_flow_vel_kp, g_flow_vel_ki, g_flow_pos_kp, poshold.badCount());
        Serial.printf("  速度[m/s]  実測(前,右)=%+6.2f %+6.2f   目標(前,右)=%+6.2f %+6.2f\n",
                      poshold.vxCtl(), poshold.vyCtl(), poshold.vxTar(), poshold.vyTar());
        Serial.printf("  位置[m]    現在(N,E)=%+6.2f %+6.2f   保持(N,E)=%+6.2f %+6.2f"
                      "   (地面固定, ψ=%+.1fdeg)\n",
                      poshold.posN(), poshold.posE(),
                      poshold.holdN(), poshold.holdE(), g_yaw_est);
        Serial.printf("  → 目標リーン角[deg]  roll=%+5.1f  pitch=%+5.1f\n",
                      poshold.leanRoll(), poshold.leanPitch());
    }

    // --- s5c: 距離センサ + 高度ホールド ---
    if (S5::USE_RANGE) {
        Serial.printf("\n[距離:%s] %s  斜め=%.2fm  → 鉛直h=%.2fm  上昇=%+.2fm/s\n",
                      (Q::RANGE_BACKEND == Q::RangeBackend::Sonar_EZ) ? "SONAR" : "ToF",
                      !g_range_ok ? "FAIL "
                                  : (g_range_valid ? "OK   " : "失探 "),
                      g_range_raw_m, g_range_h_m, g_climb_mps);
        Serial.printf("[高度ホールド] %s  %s  hold=%.2fm  vz_tar=%+.2fm/s  "
                      "base=%.2f corr=%+.3f → thr=%.2f\n",
                      g_alt_hold_enable ? "ENABLED" : "OFF(手動)",
                      g_alt_active ? "ACTIVE" : "standby",
                      g_alt_hold_m, g_alt_vz_tar,
                      g_alt_thr_base, g_alt_thr_corr, g_alt_thr_out);
        Serial.printf("  gains: pos P=%.2f  rate P=%.2f I=%.2f D=%.3f   ['g']切替  ['p']-[s..v]調整\n",
                      g_alt_pos_kp, alt_rate_pid.kp(), alt_rate_pid.ki(), alt_rate_pid.kd());
    }

    Serial.println("\n[レートループ] 目標[deg/s] 実測[deg/s]      cmd      I項");
    Serial.printf("  roll  %11.1f %11.1f %9.4f %8.4f\n",
                  roll_axis.rate_tar, roll_axis.rate_meas,
                  roll_axis.cmd, roll_axis.rate.i_term());
    Serial.printf("  pitch %11.1f %11.1f %9.4f %8.4f\n",
                  pitch_axis.rate_tar, pitch_axis.rate_meas,
                  pitch_axis.cmd, pitch_axis.rate.i_term());
    Serial.printf("  yaw   %11.1f %11.1f %9.4f %8.4f\n",
                  yaw_axis.rate_tar, yaw_axis.rate_meas,
                  yaw_axis.cmd, yaw_axis.rate.i_term());

    // --- ヘディングホールド ---
    //  err がじわじわ片側に増え続けるなら、機体が回っているのではなく
    //  ジャイロZのバイアスが残っている (= [k] で再キャリブレーションが必要)。
    Serial.printf("\n[ヨー保持] %s  kp=%.2f  目標%+8.2f  推定%+8.2f  誤差%+7.2f [deg]\n",
                  g_yaw_holding ? "HOLD  " : "STICK ",
                  g_yaw_hold_kp, g_yaw_hold, g_yaw_est,
                  g_yaw_hold - g_yaw_est);

    Serial.printf("\n  attitude roll=%+7.2f pitch=%+7.2f [deg]  thr=%.2f\n",
                  g_att.roll, g_att.pitch, sbus.des[Ch::THR]);

    Serial.println("\n[モーター]  M1=左前 M2=右前 M3=右後 M4=左後");
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) {
        Serial.printf("  M%d %5.3f  ", i + 1, g_out[i]);
        const int bar = (int)(g_out[i] * 40.0f);
        for (int j = 0; j < bar; ++j) Serial.print('#');
        Serial.println();
    }

    Serial.printf("\n[ミキサー] span_limit=%.3f  scale=%.3f  sat=0b%c%c%c%c (M4..M1)\n",
                  g_mix.span_limit, g_mix.scale,
                  (g_mix.sat & 8) ? '1' : '0', (g_mix.sat & 4) ? '1' : '0',
                  (g_mix.sat & 2) ? '1' : '0', (g_mix.sat & 1) ? '1' : '0');

    // --- オプティカルフロー (s5a: 観測のみ) ---
    //  ① 符号/軸: 机の上などで機体を「ゆっくり大きく」平行移動させて acc_px を見る
    //     ・前へ動かす → acc_px の x が一方向に大きく伸びる (逆なら FLOW_SIGN_X=-1)
    //     ・右へ動かす → acc_px の y                        (逆なら FLOW_SIGN_Y=-1)
    //     ・前後に動かして y が伸びる / 左右で x が伸びる     → FLOW_SWAP_XY=true
    //  ③ スケール ([z]でゼロ → 定規で測った高さ h で、定規で測った距離 D を
    //     一定高さでスライド → 止めて acc_px を読む):
    //        counts_per_m    = acc_px ÷ D
    //        FLOW_PX_PER_RAD = counts_per_m × h        ← これを QuadConfig.h に
    //     x(前後) と y(左右) を別々に測る。ほぼ同じ値になるはず。
    //     ※ dt にも仮定高度にも依存しない (counts/m がセンサの素の性質)。
    //  ② de-rotation: ③のあとで。機体の「位置は固定・向きだけ」前後/左右に傾ける。
    //     raw は振れるが derot(x,y) が ~0 のままなら OK。
    //     raw と逆向きに振れる → FLOW_DEROT_SIGN_* を反転。
    if (S5::USE_FLOW) {
        Serial.printf("\n[フロー] %s  h=%.2fm  raw(x,y)=%+6.1f %+6.1f  "
                      "derot(x,y)=%+6.1f %+6.1f\n",
                      g_flow_ok ? "OK  " : "FAIL", flow.height(),
                      g_flow_raw_x, g_flow_raw_y, g_flow_dx, g_flow_dy);
        Serial.printf("         v=%+6.2f %+6.2f m/s (LPF %+6.2f %+6.2f)   [z]ゼロ\n",
                      g_flow_vx, g_flow_vy, g_flow_vx_f, g_flow_vy_f);
        Serial.printf("         積算 raw=(%+9.0f,%+9.0f)  derot=(%+9.0f,%+9.0f)  "
                      "m=(%+7.3f,%+7.3f)\n",
                      g_flow_acc_raw_x, g_flow_acc_raw_y,
                      g_flow_acc_px_x,  g_flow_acc_px_y,
                      g_flow_acc_m_x,   g_flow_acc_m_y);
    }

    Serial.println("\n[p]ゲイン [k]IMUキャリブ [r]PIDリセット [l]ログ [z]フロー積算ゼロ "
                   "[h]フロー高度(手動) [g]高度ホールド切替 [i]I2Cスキャン [m]ドライラン切替");
}

// ============================================================
//  § 10  setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    const uint32_t start_ms = millis();
    while (!Serial && (millis() - start_ms < 2000)) { }

    Serial.println("\n\n=== Stage 5d : 1スイッチ完全自動ホバリング ===");
    Serial.println("!! 2モード: SW_HOVER UP = POSHOLD(完全自動) / それ以外 = ANGLE(手動) !!");
    Serial.println("!! bail-out = SW_HOVER を下げる or THR_CUT。初回は広い床で指をスイッチに !!");

    roll_axis.rate .set_gains(Gain::RATE_ROLL [0], Gain::RATE_ROLL [1], Gain::RATE_ROLL [2]);
    pitch_axis.rate.set_gains(Gain::RATE_PITCH[0], Gain::RATE_PITCH[1], Gain::RATE_PITCH[2]);
    yaw_axis.rate  .set_gains(Gain::RATE_YAW  [0], Gain::RATE_YAW  [1], Gain::RATE_YAW  [2]);

    roll_axis.angle .set_gains(Gain::ANG_ROLL [0], Gain::ANG_ROLL [1], Gain::ANG_ROLL [2]);
    pitch_axis.angle.set_gains(Gain::ANG_PITCH[0], Gain::ANG_PITCH[1], Gain::ANG_PITCH[2]);

    for (Q::Axis* ax : { &roll_axis, &pitch_axis, &yaw_axis }) {
        ax->rate.set_d_alpha(Gain::RATE_D_ALPHA);
        ax->rate.set_i_limit(Gain::RATE_I_LIMIT);
        ax->angle.set_d_alpha(Gain::ANG_D_ALPHA);
        ax->angle.set_i_limit(Gain::ANG_I_LIMIT);
    }

    // s5b/s5d: フロー水平ホールド (速度ループ + 地面固定フレーム位置ループ)
    poshold.begin();
    poshold.setVelGains(g_flow_vel_kp, g_flow_vel_ki, Q::FLOW_VEL_KD);
    poshold.setPosKp(g_flow_pos_kp);

    // s5c: 高度ホールドの内側ループ (上昇速度[m/s] → スロットル補正[割合])
    alt_rate_pid.set_gains(Q::ALT_RATE_KP, Q::ALT_RATE_KI, Q::ALT_RATE_KD);
    alt_rate_pid.set_d_alpha(Q::ALT_RATE_D_ALPHA);
    alt_rate_pid.set_i_limit(Q::ALT_RATE_I_LIMIT);

    if (S5::USE_MOTOR) {
        Serial.println("Init motors...");
        for (int i = 0; i < Q::MOTOR_COUNT; ++i) {
            motors[i].set_pin(Q::MOTOR_PIN[i]).begin();
        }
        delay(500);
        stopAllMotors();
    }
    if (g_dry_run)
        Serial.println("!! DRY-RUN 有効: モーターは回りません ('m' で解除) !!");

    if (S5::USE_SBUS)  { Serial.println("Init SBUS...");  sbus.begin(); }
    if (S5::USE_MPU)   { Serial.println("Init IMU...");   mpu.begin();  }
    if (S5::USE_IM920) { Serial.println("Init IM920..."); telemetry.begin(); }
    if (S5::USE_FLOW) {
        Serial.println("Init OpticalFlow (PMW3901)...");
        g_flow_ok = flow.begin();
        Serial.println(g_flow_ok
            ? "  PMW3901 OK"
            : "  !! PMW3901 応答なし (CSピン/SPI配線/電源を確認) !!");
    }
    if (S5::USE_RANGE) {
        const bool sonar = (Q::RANGE_BACKEND == Q::RangeBackend::Sonar_EZ);
        Serial.printf("Init Rangefinder (%s)...\n",
                      sonar ? "SONAR MaxBotix LV-MaxSonar-EZ" : "ToF VL53L1X");
        g_range_ok = rangefinder.begin();
        if (sonar) {
            Serial.printf("  SONAR PW=pin %d / 3V3給電。数百ms後に [距離] に値が出れば配線OK\n",
                          Q::RANGE_SONAR_PW_PIN);
            Serial.println("  (I2Cは使わないので VL53L1X のバス問題とは無関係)");
        } else {
            Serial.println(g_range_ok
                ? "  VL53L1X OK"
                : "  !! VL53L1X 応答なし (SDA=18/SCL=19/3V3/GND 配線を確認) !!");
        }
        Serial.printf("  高度ホールド: %s (POSHOLD で自動)。ホバースロットル=%s  目標高度=%s\n",
                      g_alt_hold_enable ? "有効" : "無効(POSHOLDでも手動)",
                      (Q::ALT_HOVER_THR > 0.01f) ? "実測値" : "POSHOLD突入時のスティック",
                      (Q::ALT_TARGET_M  > 0.0f)  ? "固定"   : "突入時の高度");
    }

    resetControllers();
    Serial.println("--- Setup complete ---");
}

void loop() {
    if (!main_tick.ready()) return;

    const float dt_s = (float)main_tick.dt_us * 1e-6f;

    if (S5::USE_MPU) {
        mpu.update();
        g_att = Q::readAttitude(mpu);
    }
    if (S5::USE_SBUS) sbus.update();

    // --- オプティカルフロー (FLOW_LOOP_HZ) ---
    //  読んで機体座標化 + de-rotation + 対地速度換算 → updateFlowHold() で
    //  s5b の速度・位置ホールドを計算 (POSHOLD 以外では出力せず基準を保持)。
    //  de-rotation にはレートループと同じジャイロ値 (g_att) を渡して位相を揃える。
    if (S5::USE_FLOW && g_flow_ok && flow_tick.ready()) {
        const float flow_dt_s = (float)flow_tick.dt_us * 1e-6f;
        flow.update(flow_dt_s, g_att.roll_rate, g_att.pitch_rate);
        g_flow_raw_x = flow.raw_x;   g_flow_raw_y = flow.raw_y;
        g_flow_dx    = flow.derot_x; g_flow_dy    = flow.derot_y;
        g_flow_vx    = flow.vx;      g_flow_vy    = flow.vy;

        // キャリブレーション用の積算 (生px / de-rot後px / 推定変位[m])
        g_flow_acc_raw_x += flow.raw_x;          g_flow_acc_raw_y += flow.raw_y;
        g_flow_acc_px_x  += flow.derot_x;        g_flow_acc_px_y  += flow.derot_y;
        g_flow_acc_m_x   += flow.vx * flow_dt_s; g_flow_acc_m_y   += flow.vy * flow_dt_s;

        // 表示用の軽い LPF (生値は g_flow_vx/vy とログに残す)
        constexpr float A = 0.2f;
        g_flow_vx_f += A * (flow.vx - g_flow_vx_f);
        g_flow_vy_f += A * (flow.vy - g_flow_vy_f);

        // s5b: 速度・位置ホールド
        updateFlowHold(flow_dt_s);
    }

    // --- s5c: 距離センサ (RANGE_LOOP_HZ) ---
    //  VL53L1X を非ブロッキングで読み、傾き補正した鉛直高度を毎回 flow へ渡す。
    //  失探したら valid() が false になり、flow は FLOW_ASSUMED_HEIGHT_M へ戻る。
    if (S5::USE_RANGE && g_range_ok && range_tick.ready()) {
        const float range_dt_s = (float)range_tick.dt_us * 1e-6f;
        rangefinder.update(g_att.roll, g_att.pitch);
        g_range_valid = rangefinder.valid();
        g_range_raw_m = rangefinder.rawM();
        g_range_h_m   = rangefinder.heightM();
        g_climb_mps   = rangefinder.climbMps();

        if (S5::USE_FLOW && g_range_valid) flow.setHeight(g_range_h_m);

        // s5c: 高度ホールド (スロットルPID)
        updateAltHold(range_dt_s);
    }

    // 地上局からの受信。groundLinkFresh() の判定に使うので制御より前に読む。
    // (PIDゲインのリモート調整やリモートリセットは行わない。receive() は
    //  GroundData を取り込んで鮮度を更新するだけ)
    if (S5::USE_IM920 && telem_rx_tick.ready()) telemetry.receive();

    updateControl(dt_s);
    handleSerial();

    // 500Hz ログ (制御の直後。この周期の指令と出力が揃った状態で落とす)
    Log::sample(main_tick.dt_us, (int)g_mode, isArmed(),
                S5::USE_SBUS ? sbus.des[Ch::THR] : 0.0f);

    // 姿勢テレメトリの送信 (10Hz)
    //  yaw はヘディングホールドの積分値を送る。Madgwick の6軸ヨーは
    //  絶対方位として意味を持たないため。
    if (S5::USE_IM920 && telem_tick.ready()) {
        telemetry.sendAttitudeOnly(g_att.roll, g_att.pitch, g_yaw_est);
    }

    // ログ中は画面表示を止める。同じUSBシリアルを奪い合うと
    // ログが落ちるうえ、logger.py 側のパースも乱れる。
    if (!Log::active && debug_tick.ready()) printStatus(main_tick.dt_us);
}
