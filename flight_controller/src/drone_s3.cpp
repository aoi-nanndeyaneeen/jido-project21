// ============================================================
//  drone_s3.cpp  -  Stage 3 : レートPID (アクロモード)
// ============================================================
//  ここで初めて姿勢制御が入ります。初飛行はこのステージです。
//
//  【前提】
//    Stage 1 / Stage 2 が通っていること。
//    ・MOTOR_PIN の並びが実機と合っている
//    ・ミキサーの方向が合っている (roll+ で正しいモーターが上がる)
//    ・センサの符号が合っている (右に傾けて roll rate が +)
//    どれか1つでも合っていないと、機体は必ず裏返ります。
//
//  【制御の中身】
//    スティック → 目標角速度 [deg/s] → レートPID → トルク指令 → ミキサー
//    角度は使いません。手を離しても水平には戻りません (アクロ)。
//    水平維持は Stage 4 です。
//
//  【旧コードから直したところ】
//    ・PID の dt を実測値で渡すようにした
//      (旧: 呼ばれる周期に関係なく常に 1ms 固定だった)
//    ・スティックを deg/s にスケールするようにした
//      (旧: -1.0〜1.0 のスティック値をそのまま角速度目標にしていたので、
//       全開でも 1 deg/s しか要求していなかった)
//    ・D項を「誤差の微分」から「測定値の微分」に変更
//      (スティックを急に動かしたときの D項スパイクを防ぐ)
//    ・スロットルが低い間は積分を止める (離陸前に I項が溜まるのを防ぐ)
//    ・PIDチューニングメニューに入る前にモーターを停止するようにした
//      (旧: 最大30秒ブロックする間、直前のPWMを出し続けていた)
//
//  【ゲイン調整の手順】
//    全部 0 から始めます。P → D → I の順。必ず機体を固定するか手で持って。
//      1. kp を、機体が振動しない範囲で少しずつ上げる
//      2. 振動が出たら kd を足して抑える
//      3. 最後に ki を少しだけ足す (ドリフト補正用)
//    シリアルの 'p' でメニューが出ます (メニュー中はモーターが止まります)。
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
//  ★ 必ず 0 から始めてください。下の値は「小さすぎて何も起きない」側に
//     倒してあります。安全ですが、このままではまともに飛びません。
namespace Gain {

//                        kp      ki      kd
constexpr float ROLL [3] = { 0.0010f, 0.0f, 0.00002f };
constexpr float PITCH[3] = { 0.0010f, 0.0f, 0.00002f };
constexpr float YAW  [3] = { 0.0020f, 0.0f, 0.0f     };

// D項のローパス (0 = フィルタなし、1 に近いほど強い)
constexpr float D_ALPHA = 0.80f;
// 積分項の上限 (トルク指令と同じ単位。0.3 なら最大出力の30%まで)
constexpr float I_LIMIT = 0.15f;

} // namespace Gain

// ============================================================
//  § 2  このステージの設定
// ============================================================
namespace S3 {

constexpr bool USE_MPU   = true;
constexpr bool USE_SBUS  = true;
constexpr bool USE_MOTOR = true;

// このスロットル以下では積分を止める (地上で I項が溜まるのを防ぐ)
constexpr float I_ENABLE_THR = 0.15f;

constexpr int MAIN_HZ  = Q::RATE_LOOP_HZ;
constexpr int DEBUG_HZ = Q::DEBUG_HZ;

} // namespace S3

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
bool        g_prev_armed = false;

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

static Ticker main_tick (S3::MAIN_HZ);
static Ticker debug_tick(S3::DEBUG_HZ);

// ============================================================
//  § 6  出力
// ============================================================
static void writeMotors() {
    if (!S3::USE_MOTOR) return;
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) motors[i].write(g_out[i]);
}

static void stopAllMotors() {
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) g_out[i] = 0.0f;
    writeMotors();
}

static bool isArmed() {
    if (!S3::USE_SBUS) return false;
    if (!sbus.isSafe()) return false;
    return sbus.Ch_state(Ch::THR_CUT) == Q::ARM_SWITCH_STATE;
}

static void resetControllers() {
    roll_axis.reset();
    pitch_axis.reset();
    yaw_axis.reset();
}

// ============================================================
//  § 7  制御
// ============================================================
static void updateControl(float dt_s) {
    const bool armed = isArmed();

    // --- アーム状態が変わった瞬間に積分などをクリア ---
    if (armed != g_prev_armed) {
        resetControllers();
        g_prev_armed = armed;
        Serial.println(armed ? "\n>>> ARMED" : "\n>>> DISARMED");
    }

    if (!armed) { stopAllMotors(); return; }

    const float thr = constrain(sbus.des[Ch::THR], 0.0f, 1.0f);

    // --- 測定値 (Stage 2 で符号を確認済みのもの) ---
    roll_axis.rate_meas  = g_att.roll_rate;
    pitch_axis.rate_meas = g_att.pitch_rate;
    yaw_axis.rate_meas   = g_att.yaw_rate;

    roll_axis.ang_meas  = g_att.roll;
    pitch_axis.ang_meas = g_att.pitch;
    yaw_axis.ang_meas   = g_att.yaw;

    // --- スティック → 目標角速度 [deg/s] ---
    roll_axis.stick  = Q::STICK_SIGN_ROLL  * sbus.des[Ch::ROLL];
    pitch_axis.stick = Q::STICK_SIGN_PITCH * sbus.des[Ch::PITCH];
    yaw_axis.stick   = Q::STICK_SIGN_YAW   * sbus.des[Ch::YAW];

    roll_axis.rate_tar  = roll_axis.stick  * Q::MAX_RATE_ROLL;
    pitch_axis.rate_tar = pitch_axis.stick * Q::MAX_RATE_PITCH;
    yaw_axis.rate_tar   = yaw_axis.stick   * Q::MAX_RATE_YAW;

    // --- レートPID ---
    //  スロットルが低い間は積分しない (地上で I項が溜まって、
    //  離陸した瞬間に機体が跳ねるのを防ぐ)
    const bool integrate = (thr > S3::I_ENABLE_THR);

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
//  serial_com.h の handlePIDTuning() は Control.h の Axis_value 用なので
//  ここでは使えません。Quad::Axis 用に書き直しています。
//  ★ メニューに入る前に必ずモーターを止めます。
static void tuningMenu() {
    stopAllMotors();
    resetControllers();

    const unsigned long old_timeout = Serial.getTimeout();
    Serial.setTimeout(30000);
    while (Serial.available()) Serial.read();

    Serial.println("\n\n!! モーターを停止しました。制御ループも止まっています !!");
    Serial.println("========== Rate PID Tuning ==========");
    Serial.printf(" [1] Roll  P : %9.5f    [2] Roll  I : %9.5f    [3] Roll  D : %9.5f\n",
                  roll_axis.rate.kp(), roll_axis.rate.ki(), roll_axis.rate.kd());
    Serial.printf(" [4] Pitch P : %9.5f    [5] Pitch I : %9.5f    [6] Pitch D : %9.5f\n",
                  pitch_axis.rate.kp(), pitch_axis.rate.ki(), pitch_axis.rate.kd());
    Serial.printf(" [7] Yaw   P : %9.5f    [8] Yaw   I : %9.5f    [9] Yaw   D : %9.5f\n",
                  yaw_axis.rate.kp(), yaw_axis.rate.ki(), yaw_axis.rate.kd());
    Serial.println(" [q] 抜ける");
    Serial.print("選択 > ");

    while (!Serial.available()) { /* 入力待ち */ }
    String sel = Serial.readStringUntil('\n');
    sel.trim();

    if (sel.length() == 0 || sel[0] == 'q' || sel[0] == 'Q') {
        Serial.setTimeout(old_timeout);
        Serial.println("\n再開します。スロットルが下がっていることを確認してください。");
        return;
    }

    Serial.print("新しい値 > ");
    const float v = Serial.parseFloat();
    Serial.println(v, 5);

    Q::Axis* ax = nullptr;
    int which = -1;   // 0=P 1=I 2=D
    switch (sel[0]) {
        case '1': ax = &roll_axis;  which = 0; break;
        case '2': ax = &roll_axis;  which = 1; break;
        case '3': ax = &roll_axis;  which = 2; break;
        case '4': ax = &pitch_axis; which = 0; break;
        case '5': ax = &pitch_axis; which = 1; break;
        case '6': ax = &pitch_axis; which = 2; break;
        case '7': ax = &yaw_axis;   which = 0; break;
        case '8': ax = &yaw_axis;   which = 1; break;
        case '9': ax = &yaw_axis;   which = 2; break;
        default: break;
    }

    if (ax) {
        const float p = (which == 0) ? v : ax->rate.kp();
        const float i = (which == 1) ? v : ax->rate.ki();
        const float d = (which == 2) ? v : ax->rate.kd();
        ax->rate.set_gains(p, i, d);
        Serial.printf("更新: P=%.5f I=%.5f D=%.5f\n", p, i, d);
    } else {
        Serial.println("不明な選択です");
    }

    resetControllers();
    Serial.setTimeout(old_timeout);
    Serial.println("再開します。スロットルが下がっていることを確認してください。");
}

static void handleSerial() {
    if (!Serial.available()) return;
    const char c = (char)tolower(Serial.peek());

    switch (c) {
        case 'p':
            Serial.read();
            tuningMenu();
            break;
        case 'c':
            Serial.read();
            stopAllMotors();
            Serial.println("CALIBRATE: 機体を水平に置いて動かさないでください");
            if (S3::USE_MPU) mpu.recalibrate();
            resetControllers();
            break;
        case 'r':
            Serial.read();
            resetControllers();
            Serial.println("PID reset");
            break;
        default:
            Serial.read();   // その他は捨てる
            break;
    }
}

// ============================================================
//  § 9  デバッグ表示
// ============================================================
static void printStatus(uint32_t dt_us) {
    Serial.print("\033[2J\033[H");
    Serial.println("=== Stage 3 : レートPID (アクロ) ===");
    Serial.printf("loop dt = %6lu us (%6.1f Hz)   %s   link=%s\n",
                  (unsigned long)dt_us, 1000000.0f / (float)dt_us,
                  isArmed() ? "ARMED" : "DISARMED",
                  sbus.isSafe() ? "OK" : "LOST");

    Serial.println("\n            目標[deg/s]   実測[deg/s]      cmd      I項");
    Serial.printf("  roll   %11.1f %13.1f %9.4f %8.4f\n",
                  roll_axis.rate_tar, roll_axis.rate_meas,
                  roll_axis.cmd, roll_axis.rate.i_term());
    Serial.printf("  pitch  %11.1f %13.1f %9.4f %8.4f\n",
                  pitch_axis.rate_tar, pitch_axis.rate_meas,
                  pitch_axis.cmd, pitch_axis.rate.i_term());
    Serial.printf("  yaw    %11.1f %13.1f %9.4f %8.4f\n",
                  yaw_axis.rate_tar, yaw_axis.rate_meas,
                  yaw_axis.cmd, yaw_axis.rate.i_term());

    Serial.printf("\n  gain roll  P=%.5f I=%.5f D=%.5f\n",
                  roll_axis.rate.kp(), roll_axis.rate.ki(), roll_axis.rate.kd());
    Serial.printf("  gain pitch P=%.5f I=%.5f D=%.5f\n",
                  pitch_axis.rate.kp(), pitch_axis.rate.ki(), pitch_axis.rate.kd());
    Serial.printf("  gain yaw   P=%.5f I=%.5f D=%.5f\n",
                  yaw_axis.rate.kp(), yaw_axis.rate.ki(), yaw_axis.rate.kd());

    Serial.printf("\n  angle  roll=%+7.2f pitch=%+7.2f yaw=%+7.2f [deg]   thr=%.2f\n",
                  g_att.roll, g_att.pitch, g_att.yaw, sbus.des[Ch::THR]);

    Serial.println("\n[モーター]  M1=左前 M2=右前 M3=右後 M4=左後");
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) {
        Serial.printf("  M%d %5.3f  ", i + 1, g_out[i]);
        const int bar = (int)(g_out[i] * 40.0f);
        for (int j = 0; j < bar; ++j) Serial.print('#');
        Serial.println();
    }

    Serial.println("\n[p] ゲイン調整  [c] IMUキャリブレーション  [r] PIDリセット");
}

// ============================================================
//  § 10  setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    const uint32_t start_ms = millis();
    while (!Serial && (millis() - start_ms < 2000)) { }

    Serial.println("\n\n=== Stage 3 : レートPID (アクロ) ===");
    Serial.println("!! Stage 1 / Stage 2 の確認が済んでいることを前提にしています !!");

    roll_axis.rate .set_gains(Gain::ROLL [0], Gain::ROLL [1], Gain::ROLL [2]);
    pitch_axis.rate.set_gains(Gain::PITCH[0], Gain::PITCH[1], Gain::PITCH[2]);
    yaw_axis.rate  .set_gains(Gain::YAW  [0], Gain::YAW  [1], Gain::YAW  [2]);

    for (Q::Axis* ax : { &roll_axis, &pitch_axis, &yaw_axis }) {
        ax->rate.set_d_alpha(Gain::D_ALPHA);
        ax->rate.set_i_limit(Gain::I_LIMIT);
    }

    if (S3::USE_MOTOR) {
        Serial.println("Init motors...");
        for (int i = 0; i < Q::MOTOR_COUNT; ++i) {
            motors[i].set_pin(Q::MOTOR_PIN[i]).begin();
        }
        delay(500);
        stopAllMotors();
    }

    if (S3::USE_SBUS) { Serial.println("Init SBUS..."); sbus.begin(); }
    if (S3::USE_MPU)  { Serial.println("Init IMU...");  mpu.begin();  }

    resetControllers();
    Serial.println("--- Setup complete ---");
}

void loop() {
    if (!main_tick.ready()) return;

    const float dt_s = (float)main_tick.dt_us * 1e-6f;

    // --- 1) 入力 ---
    if (S3::USE_MPU) {
        mpu.update();
        g_att = Q::readAttitude(mpu);
    }
    if (S3::USE_SBUS) sbus.update();

    // --- 2) 制御 ---
    updateControl(dt_s);

    // --- 3) シリアル (メニューに入るとモーターを止めてからブロックする) ---
    handleSerial();

    // --- 4) 表示 ---
    if (debug_tick.ready()) printStatus(main_tick.dt_us);
}
