// ============================================================
//  drone_s5.cpp  -  Stage 5 : オプティカルフロー / 距離センサ / 自動ホバリング
// ============================================================
//  Stage 4 (drone_s4.cpp) の角度PID + モード切替 + ヘディングホールドを
//  そのまま土台にして、位置・高度の保持を段階的に足していく。
//
//  【このファイルの現在地: s5a — PMW3901 搭載 + ログのみ】
//    ・制御には一切入れない。s4 と飛びは完全に同じ。
//    ・PMW3901 を 100Hz で読み、機体座標化 + de-rotation + 対地速度換算まで
//      やってシリアル表示と 500Hz ログに出すだけ。
//    ・目的: 符号 (前進で flow_dx が +)、軸の入れ替わり、de-rotation の符号、
//      ピクセル⇔角度の換算係数 FLOW_PX_PER_RAD をベンチで確定する。
//      調整先はすべて include/quad/QuadConfig.h の § 7。
//
//  【この先の段取り】(platformio.ini のコメントと対応)
//    s5b : フロー速度・位置ホールド。flow の vx/vy → 速度PID → 目標リーン角を
//          roll_axis.ang_tar / pitch_axis.ang_tar に入れる (MODE_AUTO で ap_roll が
//          入るのと同じ経路)。高度は QuadConfig.h の FLOW_ASSUMED_HEIGHT_M 固定、
//          スロットルは手動のまま。
//    s5c : 距離センサ搭載後。高度ホールド (スロットルPID) を足し、
//          FLOW_ASSUMED_HEIGHT_M を測距値 (flow.setHeight()) に差し替える。
//    s5d : 高度 + 位置 + ヘディングを1スイッチで同時起動 = 完全自動ホバリング。
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
#include "Telemetry.h"    // IM920SL (地上局との送受信)

#include "quad/QuadConfig.h"
#include "quad/QuadPID.h"
#include "quad/Mixer.h"
#include "quad/BodyFrame.h"

namespace Q = Quad;

// ============================================================
//  § 1  ゲイン
// ============================================================
namespace Gain {

// ---- レート (内側ループ) ----  Stage 3 で決めた値をここに写す
//                        kp      ki      kd
constexpr float RATE_ROLL [3] = { 0.0020f, 0.0f, 0.00004f };
constexpr float RATE_PITCH[3] = { 0.0020f, 0.0f, 0.00004f };

// ★ ヨーの I項。P制御だけでは定常偏差が残る。
//   機体には必ず一定のヨートルクが残っている:
//     ・モーター取付角のわずかな傾き (推力ベクトルが真上を向いていない)
//     ・CW/CCW プロペラの特性差、モーターのKV差、ESCの個体差
//   これらは P では釣り合った角速度で回り続けるだけで、消えない。
//   ki = kp は積分時定数 1秒に相当する。まずこの値で試し、
//   戻りが遅ければ 0.005 まで上げてよい (上げすぎると 1Hz 前後で揺れる)。
constexpr float RATE_YAW  [3] = { 0.0020f, 0.0000f, 0.00004f     };

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
constexpr bool USE_FLOW  = true;   // PMW3901 オプティカルフロー (s5a: 観測のみ)

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

//  ★ MODE_AUTO を追加 (drone.cpp の MODE_AUTONOMOUS 相当)
//    SW_AUTO が UP かつ IM920 の受信が新鮮なときだけ入る。
//    スイッチOFF、またはリンク途絶の瞬間に ANGLE へ自動フォールバックする。
enum Mode : uint8_t { MODE_RATE = 0, MODE_ANGLE = 1, MODE_AUTO = 2 };

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

Q::Axis roll_axis, pitch_axis, yaw_axis;

// ============================================================
//  § 4  状態
// ============================================================
namespace {

Q::Attitude g_att;
float       g_out[Q::MOTOR_COUNT] = {0};

S5::Mode g_mode      = S5::MODE_RATE;
S5::Mode g_prev_mode = S5::MODE_RATE;
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

// ---- オプティカルフロー (s5a: 観測のみ。制御には未使用) ----
bool  g_flow_ok    = false;   // PMW3901 が初期化できたか
float g_flow_raw_x = 0.0f, g_flow_raw_y = 0.0f;  // 機体座標の生カウント [px]
float g_flow_dx    = 0.0f, g_flow_dy    = 0.0f;  // de-rotation 後 [px]
float g_flow_vx    = 0.0f, g_flow_vy    = 0.0f;  // 対地速度 [m/s] (前 +, 右 +)

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
    "flow_ok,flow_raw_x,flow_raw_y,flow_dx,flow_dy,flow_vx,flow_vy,flow_h";

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
        "%d,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.2f\n",
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
        g_flow_vx, g_flow_vy, flow.height());
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
static Ticker flow_tick (Q::FLOW_LOOP_HZ);   // オプティカルフロー読み出し (100Hz)

// ============================================================
//  § 6  出力
// ============================================================
static void writeMotors() {
    if (!S5::USE_MOTOR) return;
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
}

static const char* modeName(S5::Mode m) {
    switch (m) {
        case S5::MODE_AUTO:  return "AUTO  (地上局)";
        case S5::MODE_ANGLE: return "ANGLE (水平維持)";
        default:             return "RATE  (アクロ)";
    }
}

// ------------------------------------------------------------
//  モード判定
//    優先度: AUTO (SW_AUTO UP かつ受信新鮮) > ANGLE (SW_HOVER UP) > RATE
//
//  ★ SW_AUTO が最優先なのは、パイロットが物理スイッチで明示的に権限委譲した
//    場合のみ有効になるため。スイッチOFF、または IM920 リンクが途絶えた瞬間に
//    下の優先順位へ自動的にフォールバックする (安全側)。
//    drone.cpp の Flight_mode::update() と同じ考え方。
// ------------------------------------------------------------
static S5::Mode selectMode() {
    if (!S5::USE_SBUS) return S5::MODE_RATE;

    if (S5::USE_IM920 &&
        sbus.Ch_state(Ch::SW_AUTO) == up &&
        telemetry.groundLinkFresh()) {
        return S5::MODE_AUTO;
    }
    if (sbus.Ch_state(Ch::SW_HOVER) == up) return S5::MODE_ANGLE;
    return S5::MODE_RATE;
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

    const float thr = constrain(sbus.des[Ch::THR], 0.0f, 1.0f);
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
    //  外側ループ: 角度 → 目標角速度   (ANGLE / AUTO モード, 200Hz)
    // ------------------------------------------------------------
    if (g_mode == S5::MODE_ANGLE || g_mode == S5::MODE_AUTO) {
        roll_axis.ang_tar  = roll_axis.stick  * Q::MAX_ANGLE_ROLL;
        pitch_axis.ang_tar = pitch_axis.stick * Q::MAX_ANGLE_PITCH;

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

static void handleSerial() {
    if (!Serial.available()) return;
    const char c = (char)tolower(Serial.read());

    switch (c) {
        case 'p':
            tuningMenu();
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
        default:
            break;
    }
}

// ============================================================
//  § 9  デバッグ表示
// ============================================================
static void printStatus(uint32_t dt_us) {
    Serial.print("\033[2J\033[H");
    Serial.println("=== Stage 5a : オプティカルフロー観測 (制御は s4 と同じ) ===");
    Serial.printf("loop dt = %6lu us (%6.1f Hz)   %s   link=%s\n",
                  (unsigned long)dt_us, 1000000.0f / (float)dt_us,
                  isArmed() ? "ARMED" : "DISARMED",
                  sbus.isSafe() ? "OK" : "LOST");
    Serial.printf("MODE = %s   (SW_AUTO UP+受信新鮮 で AUTO / SW_HOVER UP で ANGLE)\n",
                  modeName(g_mode));

    if (S5::USE_IM920) {
        const GroundData& g = telemetry.lastGroundData();
        Serial.printf("IM920 link=%s   AP: roll=%+.3f pitch=%+.3f yaw=%+.3f (thr=%+.3f 未使用)\n",
                      telemetry.groundLinkFresh() ? "FRESH" : "STALE",
                      g.ap_roll, g.ap_pitch, g.ap_yaw, g.ap_throttle);
    }

    if (g_mode == S5::MODE_ANGLE || g_mode == S5::MODE_AUTO) {
        Serial.println("\n[角度ループ]  目標[deg]  実測[deg]  → 角速度目標[deg/s]");
        Serial.printf("  roll  %10.1f %10.1f %18.1f\n",
                      roll_axis.ang_tar, roll_axis.ang_meas, roll_axis.rate_tar);
        Serial.printf("  pitch %10.1f %10.1f %18.1f\n",
                      pitch_axis.ang_tar, pitch_axis.ang_meas, pitch_axis.rate_tar);
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
    //  確認手順:
    //   ・機体を前へ平行移動   → dx が + に大きく振れる (符号は FLOW_SIGN_X)
    //   ・機体を右へ平行移動   → dy が +               (FLOW_SIGN_Y)
    //   ・前後に動かして dy が動く / 左右で dx が動く → FLOW_SWAP_XY = true
    //   ・その場で傾けるだけ (並進させない) → dx,dy が ~0 のまま
    //       逆に振れる → FLOW_DEROT_SIGN_* を反転 / 量が合わない → FLOW_PX_PER_RAD
    if (S5::USE_FLOW) {
        Serial.printf("\n[フロー] %s  h=%.2fm  raw(x,y)=%+7.1f %+7.1f  "
                      "derot(x,y)=%+7.1f %+7.1f  v(x,y)=%+6.2f %+6.2f m/s\n",
                      g_flow_ok ? "OK  " : "FAIL",
                      flow.height(),
                      g_flow_raw_x, g_flow_raw_y,
                      g_flow_dx, g_flow_dy, g_flow_vx, g_flow_vy);
    }

    Serial.println("\n[p] ゲイン調整  [k] IMUキャリブレーション  [r] PIDリセット  [l] ログ開始/停止");
}

// ============================================================
//  § 10  setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    const uint32_t start_ms = millis();
    while (!Serial && (millis() - start_ms < 2000)) { }

    Serial.println("\n\n=== Stage 5a : オプティカルフロー観測 + モード切替 ===");
    Serial.println("!! 制御は Stage 4 と同一。フローはログ/表示のみで制御に入れていません !!");

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

    if (S5::USE_MOTOR) {
        Serial.println("Init motors...");
        for (int i = 0; i < Q::MOTOR_COUNT; ++i) {
            motors[i].set_pin(Q::MOTOR_PIN[i]).begin();
        }
        delay(500);
        stopAllMotors();
    }

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

    // --- オプティカルフロー (100Hz) ---
    //  s5a: 読んで機体座標化 + de-rotation + 対地速度換算までやって、
    //       グローバルに置くだけ。制御 (updateControl) では一切参照しない。
    //  de-rotation にはレートループと同じジャイロ値 (g_att) を渡して位相を揃える。
    if (S5::USE_FLOW && g_flow_ok && flow_tick.ready()) {
        const float flow_dt_s = (float)flow_tick.dt_us * 1e-6f;
        flow.update(flow_dt_s, g_att.roll_rate, g_att.pitch_rate);
        g_flow_raw_x = flow.raw_x;   g_flow_raw_y = flow.raw_y;
        g_flow_dx    = flow.derot_x; g_flow_dy    = flow.derot_y;
        g_flow_vx    = flow.vx;      g_flow_vy    = flow.vy;
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
