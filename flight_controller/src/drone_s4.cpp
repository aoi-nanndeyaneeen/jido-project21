// ============================================================
//  drone_s4.cpp  -  Stage 4 : 角度PID (水平維持) + モード切替
// ============================================================
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
//    ヨーは常にレートのみです (角度で押さえると機首が固定されて扱いにくい)。
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
// ============================================================

#include <Arduino.h>
#include <math.h>

#include "Config.h"
#include "Actuators.h"
#include "Receiver.h"
#include "sensor/IMU.h"   // 気圧・超音波は使わないので Sensors.h ごとは読まない

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
constexpr float RATE_ROLL [3] = { 0.0010f, 0.0f, 0.00002f };
constexpr float RATE_PITCH[3] = { 0.0010f, 0.0f, 0.00002f };
constexpr float RATE_YAW  [3] = { 0.0020f, 0.0f, 0.0f     };

constexpr float RATE_D_ALPHA = 0.80f;
constexpr float RATE_I_LIMIT = 0.15f;

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
namespace S4 {

constexpr bool USE_MPU   = true;
constexpr bool USE_SBUS  = true;
constexpr bool USE_MOTOR = true;

constexpr float I_ENABLE_THR = 0.15f;

// 角度ループが出せる角速度の上限 [deg/s]
// (大きく傾いたときに、レートループが追えない目標を出さないための蓋)
constexpr float ANGLE_OUT_LIMIT = 250.0f;

constexpr int MAIN_HZ  = Q::RATE_LOOP_HZ;
constexpr int DEBUG_HZ = Q::DEBUG_HZ;

enum Mode : uint8_t { MODE_RATE = 0, MODE_ANGLE = 1 };

} // namespace S4

// ============================================================
//  § 3  インスタンス
// ============================================================
IMU   mpu(&Wire);
Sbus  sbus(&Serial5);
motor motors[Q::MOTOR_COUNT];

Q::Axis roll_axis, pitch_axis, yaw_axis;

// ============================================================
//  § 4  状態
// ============================================================
namespace {

Q::Attitude g_att;
float       g_out[Q::MOTOR_COUNT] = {0};

S4::Mode g_mode      = S4::MODE_RATE;
S4::Mode g_prev_mode = S4::MODE_RATE;
bool     g_prev_armed = false;

// 角度ループの間引きカウンタと、実際の経過時間
int      g_angle_div_count = 0;
uint32_t g_angle_prev_us   = 0;

} // anonymous namespace

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

static Ticker main_tick (S4::MAIN_HZ);
static Ticker debug_tick(S4::DEBUG_HZ);

// ============================================================
//  § 6  出力
// ============================================================
static void writeMotors() {
    if (!S4::USE_MOTOR) return;
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) motors[i].write(g_out[i]);
}

static void stopAllMotors() {
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) g_out[i] = 0.0f;
    writeMotors();
}

static bool isArmed() {
    if (!S4::USE_SBUS) return false;
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
}

static const char* modeName(S4::Mode m) {
    return (m == S4::MODE_ANGLE) ? "ANGLE (水平維持)" : "RATE  (アクロ)";
}

// ============================================================
//  § 7  制御
// ============================================================
static void updateControl(float dt_s) {
    // --- モード判定 ---
    g_mode = (S4::USE_SBUS && sbus.Ch_state(Ch::SW_HOVER) == up)
             ? S4::MODE_ANGLE : S4::MODE_RATE;

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
    const bool  integrate = (thr > S4::I_ENABLE_THR);

    // --- 測定値 ---
    roll_axis.rate_meas  = g_att.roll_rate;
    pitch_axis.rate_meas = g_att.pitch_rate;
    yaw_axis.rate_meas   = g_att.yaw_rate;

    roll_axis.ang_meas  = g_att.roll;
    pitch_axis.ang_meas = g_att.pitch;
    yaw_axis.ang_meas   = g_att.yaw;

    // --- スティック ---
    roll_axis.stick  = Q::STICK_SIGN_ROLL  * sbus.des[Ch::ROLL];
    pitch_axis.stick = Q::STICK_SIGN_PITCH * sbus.des[Ch::PITCH];
    yaw_axis.stick   = Q::STICK_SIGN_YAW   * sbus.des[Ch::YAW];

    // ------------------------------------------------------------
    //  外側ループ: 角度 → 目標角速度   (ANGLE モードのみ, 200Hz)
    // ------------------------------------------------------------
    if (g_mode == S4::MODE_ANGLE) {
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
                -S4::ANGLE_OUT_LIMIT, S4::ANGLE_OUT_LIMIT);

            pitch_axis.rate_tar = constrain(
                pitch_axis.angle.update(pitch_axis.ang_tar, pitch_axis.ang_meas,
                                        ang_dt_s, integrate),
                -S4::ANGLE_OUT_LIMIT, S4::ANGLE_OUT_LIMIT);
        }
        // 間引かれたループでは、前回の rate_tar をそのまま使う
    } else {
        // RATE モード: スティックが直接、目標角速度になる
        roll_axis.ang_tar  = 0.0f;
        pitch_axis.ang_tar = 0.0f;
        roll_axis.rate_tar  = roll_axis.stick  * Q::MAX_RATE_ROLL;
        pitch_axis.rate_tar = pitch_axis.stick * Q::MAX_RATE_PITCH;
    }

    // ヨーは常にレート指令
    yaw_axis.rate_tar = yaw_axis.stick * Q::MAX_RATE_YAW;

    // ------------------------------------------------------------
    //  内側ループ: 角速度 → トルク指令   (1000Hz)
    // ------------------------------------------------------------
    roll_axis.cmd  = roll_axis.rate .update(roll_axis.rate_tar,  roll_axis.rate_meas,  dt_s, integrate);
    pitch_axis.cmd = pitch_axis.rate.update(pitch_axis.rate_tar, pitch_axis.rate_meas, dt_s, integrate);
    yaw_axis.cmd   = yaw_axis.rate  .update(yaw_axis.rate_tar,   yaw_axis.rate_meas,   dt_s, integrate);

    // --- ミキサー ---
    Q::mix(thr, roll_axis.cmd, pitch_axis.cmd, yaw_axis.cmd, g_out);
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
            if (S4::USE_MPU) mpu.recalibrate();
            resetControllers();
            break;
        case 'r':
            resetControllers();
            Serial.println("PID reset");
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
    Serial.println("=== Stage 4 : 角度PID + モード切替 ===");
    Serial.printf("loop dt = %6lu us (%6.1f Hz)   %s   link=%s\n",
                  (unsigned long)dt_us, 1000000.0f / (float)dt_us,
                  isArmed() ? "ARMED" : "DISARMED",
                  sbus.isSafe() ? "OK" : "LOST");
    Serial.printf("MODE = %s        (SW_HOVER が UP で ANGLE)\n", modeName(g_mode));

    if (g_mode == S4::MODE_ANGLE) {
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

    Serial.printf("\n  attitude roll=%+7.2f pitch=%+7.2f yaw=%+7.2f [deg]  thr=%.2f\n",
                  g_att.roll, g_att.pitch, g_att.yaw, sbus.des[Ch::THR]);

    Serial.println("\n[モーター]  M1=左前 M2=右前 M3=右後 M4=左後");
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) {
        Serial.printf("  M%d %5.3f  ", i + 1, g_out[i]);
        const int bar = (int)(g_out[i] * 40.0f);
        for (int j = 0; j < bar; ++j) Serial.print('#');
        Serial.println();
    }

    Serial.println("\n[p] ゲイン調整  [k] IMUキャリブレーション  [r] PIDリセット");
}

// ============================================================
//  § 10  setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    const uint32_t start_ms = millis();
    while (!Serial && (millis() - start_ms < 2000)) { }

    Serial.println("\n\n=== Stage 4 : 角度PID + モード切替 ===");
    Serial.println("!! Stage 3 のレートPIDが飛べる状態であることが前提です !!");

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

    if (S4::USE_MOTOR) {
        Serial.println("Init motors...");
        for (int i = 0; i < Q::MOTOR_COUNT; ++i) {
            motors[i].set_pin(Q::MOTOR_PIN[i]).begin();
        }
        delay(500);
        stopAllMotors();
    }

    if (S4::USE_SBUS) { Serial.println("Init SBUS..."); sbus.begin(); }
    if (S4::USE_MPU)  { Serial.println("Init IMU...");  mpu.begin();  }

    resetControllers();
    Serial.println("--- Setup complete ---");
}

void loop() {
    if (!main_tick.ready()) return;

    const float dt_s = (float)main_tick.dt_us * 1e-6f;

    if (S4::USE_MPU) {
        mpu.update();
        g_att = Q::readAttitude(mpu);
    }
    if (S4::USE_SBUS) sbus.update();

    updateControl(dt_s);
    handleSerial();

    if (debug_tick.ready()) printStatus(main_tick.dt_us);
}
