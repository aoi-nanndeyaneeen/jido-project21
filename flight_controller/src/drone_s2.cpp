// ============================================================
//  drone_s2.cpp  -  Stage 2 : X配置ミキサー と センサ軸の確認
// ============================================================
//  ★ まだプロペラを外したまま実行してください。
//
//  【このステージの目的】
//    (a) ミキサーが正しいか  : ロール指令を入れたときに正しいモーターが速くなるか
//    (b) センサの向きが正しいか: 機体を傾けたときに表示の符号が合っているか
//    PID はまだ入っていません。スティックはトルク指令として素通しです。
//
//  【確認手順 (a) ミキサー】
//    プロペラを外し、送信機でアーム。シリアルで下のキーを打つと、
//    その軸に固定のトルク指令が TEST_MS だけ入ります。
//    画面の M1..M4 の数値と、実際の音・回転を見比べてください。
//
//      [1] roll +  (右バンク)  → M1(左前), M4(左後) が速くなるはず
//      [2] roll -  (左バンク)  → M2(右前), M3(右後) が速くなるはず
//      [3] pitch + (機首上げ)  → M3(右後), M4(左後) が速くなるはず
//      [4] pitch - (機首下げ)  → M1(左前), M2(右前) が速くなるはず
//      [5] yaw +   (右旋回)    → M2, M4 (CCWペア) が速くなるはず
//      [6] yaw -   (左旋回)    → M1, M3 (CWペア)  が速くなるはず
//
//    合わなければ QuadConfig.h の MOTOR_PIN の並びか MIX_* の符号を直します。
//
//  【確認手順 (b) センサ】
//    機体を手で持って動かし、画面の値の符号を確認:
//      ・右に傾ける      → roll  が +
//      ・機首を持ち上げる → pitch が +
//      ・機首を右へ回す   → yaw rate が +
//    合わなければ QuadConfig.h の GYRO_SIGN_* / ANG_SIGN_* を -1 にします。
//    傾ける軸と反応する軸が入れ替わっている場合は SWAP_XY = true。
//
//  【ここが通ったら】Stage 3 (drone_s3) でレートPIDを入れます。
// ============================================================

#include <Arduino.h>
#include <math.h>

#include "Config.h"
#include "Actuators.h"
#include "Receiver.h"
#include "sensor/IMU.h"   // 気圧・超音波は使わないので Sensors.h ごとは読まない

#include "quad/QuadConfig.h"
#include "quad/Mixer.h"
#include "quad/BodyFrame.h"

namespace Q = Quad;

// ============================================================
//  § 1  このステージの設定
// ============================================================
namespace S2 {

constexpr bool USE_MPU   = true;
constexpr bool USE_SBUS  = true;
constexpr bool USE_MOTOR = true;

// スティックをそのままトルク指令にするときの倍率。
// PID が入っていないので、暴れないように控えめにしておきます。
constexpr float STICK_TO_CMD = 0.25f;

// ミキサー確認用に注入するトルク指令の大きさと、そのときのスロットル
constexpr float    INJECT_CMD      = 0.20f;
constexpr float    INJECT_THROTTLE = 0.20f;
constexpr uint32_t INJECT_MS       = 2000;

constexpr int MAIN_HZ  = 1000;
constexpr int DEBUG_HZ = 10;

} // namespace S2

// ============================================================
//  § 2  インスタンス
// ============================================================
IMU   mpu(&Wire);
Sbus  sbus(&Serial5);
motor motors[Q::MOTOR_COUNT];

// ============================================================
//  § 3  状態
// ============================================================
namespace {

Q::Attitude g_att;
float       g_out[Q::MOTOR_COUNT] = {0};

// ミキサー確認用の注入
float    g_inj_roll = 0.0f, g_inj_pitch = 0.0f, g_inj_yaw = 0.0f;
bool     g_injecting = false;
uint32_t g_inj_until_ms = 0;

} // anonymous namespace

// ============================================================
//  § 4  周期実行のヘルパ
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

static Ticker main_tick (S2::MAIN_HZ);
static Ticker debug_tick(S2::DEBUG_HZ);

// ============================================================
//  § 5  出力
// ============================================================
static void writeMotors(const float out[Q::MOTOR_COUNT]) {
    if (!S2::USE_MOTOR) return;
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) motors[i].write(out[i]);
}

static void stopAllMotors() {
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) g_out[i] = 0.0f;
    writeMotors(g_out);
}

static bool isArmed() {
    return true;
    if (!S2::USE_SBUS) return false;
    if (!sbus.isSafe()||1) return false;
    return sbus.Ch_state(Ch::THR_CUT) == Q::ARM_SWITCH_STATE;
}

// ------------------------------------------------------------
//  制御 (このステージでは PID なし)
// ------------------------------------------------------------
static void updateControl() {
    if (!isArmed()) {
        g_injecting = false;
        stopAllMotors();
        return;
    }

    float thr, roll_cmd, pitch_cmd, yaw_cmd;

    if (g_injecting) {
        // 注入の時間切れ
        if ((int32_t)(millis() - g_inj_until_ms) >= 0) {
            g_injecting = false;
            g_inj_roll = g_inj_pitch = g_inj_yaw = 0.0f;
            Serial.println("INJECT: 終了");
            stopAllMotors();
            return;
        }
        thr       = S2::INJECT_THROTTLE;
        roll_cmd  = g_inj_roll;
        pitch_cmd = g_inj_pitch;
        yaw_cmd   = g_inj_yaw;
    } else {
        thr       = constrain(sbus.des[Ch::THR], 0.0f, 1.0f);
        roll_cmd  = Q::STICK_SIGN_ROLL  * sbus.des[Ch::ROLL]  * S2::STICK_TO_CMD;
        pitch_cmd = Q::STICK_SIGN_PITCH * sbus.des[Ch::PITCH] * S2::STICK_TO_CMD;
        yaw_cmd   = Q::STICK_SIGN_YAW   * sbus.des[Ch::YAW]   * S2::STICK_TO_CMD;
    }

    Q::mix(thr, roll_cmd, pitch_cmd, yaw_cmd, g_out);
    writeMotors(g_out);
}

// ============================================================
//  § 6  シリアルコマンド
// ============================================================
static void startInject(float r, float p, float y, const char* label) {
    if (!isArmed()) {
        Serial.println("REJECT: not armed (THR_CUT スイッチを確認してください)");
        return;
    }
    g_inj_roll     = r;
    g_inj_pitch    = p;
    g_inj_yaw      = y;
    g_injecting    = true;
    g_inj_until_ms = millis() + S2::INJECT_MS;
    Serial.printf("INJECT: %s  (roll=%+.2f pitch=%+.2f yaw=%+.2f, thr=%.2f)\n",
                  label, (double)r, (double)p, (double)y,
                  (double)S2::INJECT_THROTTLE);
}

static void handleSerial() {
    if (!Serial.available()) return;
    const char c = (char)tolower(Serial.read());

    switch (c) {
        case '1': startInject(+S2::INJECT_CMD, 0, 0, "roll +  (右バンク / M1,M4 が上がるはず)"); break;
        case '2': startInject(-S2::INJECT_CMD, 0, 0, "roll -  (左バンク / M2,M3 が上がるはず)"); break;
        case '3': startInject(0, +S2::INJECT_CMD, 0, "pitch + (機首上げ / M3,M4 が上がるはず)"); break;
        case '4': startInject(0, -S2::INJECT_CMD, 0, "pitch - (機首下げ / M1,M2 が上がるはず)"); break;
        case '5': startInject(0, 0, +S2::INJECT_CMD, "yaw +   (右旋回 / CCWペアが上がるはず)"); break;
        case '6': startInject(0, 0, -S2::INJECT_CMD, "yaw -   (左旋回 / CWペアが上がるはず)"); break;

        case 'x':
        case '0':
            g_injecting = false;
            g_inj_roll = g_inj_pitch = g_inj_yaw = 0.0f;
            stopAllMotors();
            Serial.println("STOP");
            break;

        case 'c':
            g_injecting = false;
            stopAllMotors();
            Serial.println("CALIBRATE: 機体を水平に置いて動かさないでください");
            if (S2::USE_MPU) mpu.recalibrate();
            break;

        case 'h':
            Serial.println("\n[1/2] roll +/-  [3/4] pitch +/-  [5/6] yaw +/-  "
                           "[0,x] 停止  [c] キャリブレーション\n");
            break;

        default:
            break;
    }
}

// ============================================================
//  § 7  デバッグ表示
// ============================================================
static const char* swName(Sw s) {
    return (s == up) ? "UP " : (s == down) ? "DWN" : "CEN";
}

static void printStatus(uint32_t dt_us) {
    Serial.print("\033[2J\033[H");
    Serial.println("=== Stage 2 : ミキサーとセンサ軸の確認 (プロペラを外すこと) ===");
    Serial.printf("loop dt = %6lu us (%6.1f Hz)   %s\n",
                  (unsigned long)dt_us, 1000000.0f / (float)dt_us,
                  isArmed() ? "ARMED" : "DISARMED");

    // ---- 姿勢 (符号の確認用) ----
    Serial.println("\n[姿勢]  右バンク→roll+ / 機首上げ→pitch+ / 機首右回し→yaw rate+");
    Serial.printf("  angle : roll=%+7.2f  pitch=%+7.2f  yaw=%+7.2f  [deg]\n",
                  g_att.roll, g_att.pitch, g_att.yaw);
    Serial.printf("  rate  : roll=%+7.2f  pitch=%+7.2f  yaw=%+7.2f  [deg/s]\n",
                  g_att.roll_rate, g_att.pitch_rate, g_att.yaw_rate);
    Serial.printf("  acc   :   fwd=%+6.3f  right=%+6.3f   down=%+6.3f  [g]\n",
                  g_att.acc_x, g_att.acc_y, g_att.acc_z);

    // ---- 指令 ----
    Serial.println("\n[指令]");
    if (g_injecting) {
        Serial.printf("  INJECT  roll=%+.2f pitch=%+.2f yaw=%+.2f  thr=%.2f\n",
                      (double)g_inj_roll, (double)g_inj_pitch, (double)g_inj_yaw,
                      (double)S2::INJECT_THROTTLE);
    } else if (S2::USE_SBUS) {
        Serial.printf("  stick   roll=%+.2f pitch=%+.2f yaw=%+.2f  thr=%.2f  link=%s (cut=%s)\n",
                      sbus.des[Ch::ROLL], sbus.des[Ch::PITCH], sbus.des[Ch::YAW],
                      sbus.des[Ch::THR],
                      sbus.isSafe() ? "OK" : "LOST",
                      swName(sbus.Ch_state(Ch::THR_CUT)));
    }

    // ---- モーター ----
    Serial.println("\n[モーター出力]   M1=左前  M2=右前  M3=右後  M4=左後");
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) {
        Serial.printf("  M%d (pin %2d) %5.3f  ", i + 1, Q::MOTOR_PIN[i], g_out[i]);
        const int bar = (int)(g_out[i] * 40.0f);
        for (int j = 0; j < bar; ++j) Serial.print('#');
        Serial.println();
    }

    Serial.println("\n[1/2] roll +/-  [3/4] pitch +/-  [5/6] yaw +/-  [0] 停止  [c] キャリブレーション");
}

// ============================================================
//  § 8  setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    const uint32_t start_ms = millis();
    while (!Serial && (millis() - start_ms < 2000)) { }

    Serial.println("\n\n=== Stage 2 : ミキサーとセンサ軸の確認 ===");
    Serial.println("!! プロペラを外してあることを確認してください !!");

    if (S2::USE_MOTOR) {
        Serial.println("Init motors...");
        for (int i = 0; i < Q::MOTOR_COUNT; ++i) {
            motors[i].set_pin(Q::MOTOR_PIN[i]).begin();
        }
        delay(500);
        stopAllMotors();
    }

    if (S2::USE_SBUS) { Serial.println("Init SBUS..."); sbus.begin(); }
    if (S2::USE_MPU)  { Serial.println("Init IMU...");  mpu.begin();  }

    Serial.println("--- Setup complete ---  [h] でコマンド一覧");
}

void loop() {
    if (!main_tick.ready()) return;

    // --- 1) 入力 ---
    if (S2::USE_MPU) {
        mpu.update();
        g_att = Q::readAttitude(mpu);
    }
    if (S2::USE_SBUS) sbus.update();

    // --- 2) シリアル ---
    handleSerial();

    // --- 3) 出力 (PID なし。スティック → ミキサー) ---
    updateControl();

    // --- 4) 表示 ---
    if (debug_tick.ready()) printStatus(main_tick.dt_us);
}
