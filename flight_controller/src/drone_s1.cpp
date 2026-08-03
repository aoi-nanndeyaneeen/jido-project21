// ============================================================
//  drone_s1.cpp  -  Stage 1 : 出力段とセンサの生存確認
// ============================================================
//  ★ 必ずプロペラを外して実行してください。
//
//  【このステージの目的】
//    分割ESC + 新シャシーの配線を確定させる。
//    姿勢制御 (PID) は一切通っていません。落ちようがない代わりに、
//    飛ばすこともできません。ここは「配線を決めるための道具」です。
//
//  【確認すること】
//    1. どのピンがどの物理位置のモーターか   → シリアルで 1〜4 を打つ
//    2. 各モーターの回転方向 (CW / CCW)      → 目視
//    3. 4つのESCが同じスロットルで同じように回るか
//    4. THR_CUT スイッチの向きが想定通りか
//    5. SBUS の各チャンネルが期待通りに動くか
//    6. IMU が生きていて、傾けると値が動くか
//
//  【シリアルコマンド】
//    1 / 2 / 3 / 4 : そのモーターだけを TEST_THROTTLE で TEST_MS ミリ秒回す
//    s             : 全モーター即停止
//    a             : 4つ同時に TEST_THROTTLE で回す (回転差の確認用)
//    i             : IMU の生の値を1回だけ詳しく表示
//
//  【確定したら】
//    結果を include/quad/QuadConfig.h の MOTOR_PIN と MIX_YAW に反映して
//    Stage 2 (drone_s2) へ進んでください。
// ============================================================

#include <Arduino.h>
#include <math.h>

#include "Config.h"
#include "Actuators.h"
#include "Receiver.h"
#include "sensor/IMU.h"   // 気圧・超音波は使わないので Sensors.h ごとは読まない

// ============================================================
//  § 1  このステージの設定
// ============================================================
namespace S1 {

// ---- サブシステムの有効/無効 -------------------------------
//  constexpr にしてあるので、false にした側のコードはビルドから丸ごと消えます。
//  (旧 drone.cpp は非 const のグローバル bool だったため、OFF にしても
//   コードは残り、しかも実行時に毎回分岐していました)
constexpr bool USE_MPU   = true;
constexpr bool USE_SBUS  = true;
constexpr bool USE_MOTOR = true;

// ---- モーターのピン ----------------------------------------
//  M1(左前), M2(右前), M3(右後), M4(左後) のつもりで並べていますが、
//  実際にどうなっているかを確かめるのがこのステージです。
constexpr int MOTOR_COUNT = 4;
constexpr int MOTOR_PIN[MOTOR_COUNT] = { 17, 16, 15, 14 };

// ---- モーターテスト ----------------------------------------
constexpr float    TEST_THROTTLE = 0.12f;   // テスト時のスロットル [0.0-1.0]
constexpr uint32_t TEST_MS       = 2000;    // 自動停止までの時間 [ms]

// 受信機なしでモーターテストを許可するか。
// true にすると送信機の安全スイッチを無視します。ベンチ専用。
constexpr bool ALLOW_TEST_WITHOUT_RX = false;

// ---- スロットルカット --------------------------------------
//  旧コードは drone.cpp 側が「down で出力OK」、Receiver.h の th_cut() が
//  「up でカット」という食い違った定義を持っていました。ここで1本化します。
//  中央 (cen) では出力しません。
constexpr Sw ARM_SWITCH_STATE = down;

// ---- ループ周期 --------------------------------------------
constexpr int MAIN_HZ  = 1000;
constexpr int DEBUG_HZ = 10;

} // namespace S1

// ============================================================
//  § 2  インスタンス
// ============================================================
IMU   mpu(&Wire);
Sbus  sbus(&Serial5);
motor motors[S1::MOTOR_COUNT];

// ============================================================
//  § 3  状態
// ============================================================
namespace {

int      g_test_motor    = -1;   // テスト中のモーター番号 (0-3)。-1 = なし
bool     g_test_all      = false;
uint32_t g_test_until_ms = 0;

} // anonymous namespace

// ============================================================
//  § 4  周期実行のヘルパ
// ============================================================
//  Config::Timing::freq<Hz>() はテンプレート内の static を使うため、
//  同じ Hz で2箇所から呼ぶと状態を共有してしまいます。
//  ここでは呼び出し場所ごとに独立した変数を持つ形にしました。
struct Ticker {
    uint32_t period_us;
    uint32_t prev_us = 0;
    uint32_t dt_us   = 0;

    explicit Ticker(uint32_t hz) : period_us(1000000UL / hz) {}

    bool ready() {
        const uint32_t now = micros();
        if (now - prev_us < period_us) return false;
        dt_us  = now - prev_us;
        prev_us = now;
        return true;
    }
};

static Ticker main_tick (S1::MAIN_HZ);
static Ticker debug_tick(S1::DEBUG_HZ);

// ============================================================
//  § 5  モーター出力
// ============================================================
static void stopAllMotors() {
    if (!S1::USE_MOTOR) return;
    for (int i = 0; i < S1::MOTOR_COUNT; ++i) motors[i].write(0.0f);
}

// 送信機側でアーム状態か
static bool isArmed() {
    if (!S1::USE_SBUS) return false;
    if (!sbus.isSafe()) return false;
    return sbus.Ch_state(Ch::THR_CUT) == S1::ARM_SWITCH_STATE;
}

// モーターテストを開始してよい状態か
static bool testAllowed() {
    if (S1::ALLOW_TEST_WITHOUT_RX) return true;
    if (!isArmed()) {
        Serial.println("REJECT: not armed (THR_CUT スイッチを確認してください)");
        return false;
    }
    if (sbus.des[Ch::THR] > 0.02f) {
        Serial.println("REJECT: スロットルスティックを一番下まで下げてください");
        return false;
    }
    return true;
}

static void startMotorTest(int index) {
    if (!testAllowed()) return;
    g_test_motor    = index;
    g_test_all      = false;
    g_test_until_ms = millis() + S1::TEST_MS;
    Serial.printf("TEST: M%d (pin %d) を %.2f で %lu ms 回します\n",
                  index + 1, S1::MOTOR_PIN[index],
                  (double)S1::TEST_THROTTLE, (unsigned long)S1::TEST_MS);
}

static void startAllTest() {
    if (!testAllowed()) return;
    g_test_motor    = -1;
    g_test_all      = true;
    g_test_until_ms = millis() + S1::TEST_MS;
    Serial.printf("TEST: 全モーターを %.2f で %lu ms 回します\n",
                  (double)S1::TEST_THROTTLE, (unsigned long)S1::TEST_MS);
}

static void stopTest() {
    g_test_motor = -1;
    g_test_all   = false;
    stopAllMotors();
}

// ------------------------------------------------------------
//  毎ループのモーター出力
//    ・テスト中     : 指定したモーターだけ回す
//    ・アーム中     : スロットルスティックを4つに素通し (姿勢制御なし)
//    ・それ以外     : 全停止
// ------------------------------------------------------------
static void updateMotors() {
    if (!S1::USE_MOTOR) return;

    // テストの時間切れ
    if ((g_test_motor >= 0 || g_test_all) && (int32_t)(millis() - g_test_until_ms) >= 0) {
        Serial.println("TEST: 終了");
        stopTest();
        return;
    }

    if (g_test_all) {
        for (int i = 0; i < S1::MOTOR_COUNT; ++i) motors[i].write(S1::TEST_THROTTLE);
        return;
    }
    if (g_test_motor >= 0) {
        for (int i = 0; i < S1::MOTOR_COUNT; ++i) {
            motors[i].write(i == g_test_motor ? S1::TEST_THROTTLE : 0.0f);
        }
        return;
    }

    if (!isArmed()) { stopAllMotors(); return; }

    // 姿勢制御なし。4つに同じ値を出すだけ。
    const float thr = constrain(sbus.des[Ch::THR], 0.0f, 1.0f);
    for (int i = 0; i < S1::MOTOR_COUNT; ++i) motors[i].write(thr);
}

// ============================================================
//  § 6  シリアルコマンド
// ============================================================
static void printImuDetail() {
    Serial.println("\n--- IMU raw (scaled) ---");
    Serial.printf("  acc  raw : x=%+8.4f y=%+8.4f z=%+8.4f [g]\n",
                  mpu.getAccX_Raw(), mpu.getAccY_Raw(), mpu.getAccZ_Raw());
    Serial.printf("  acc  cal : x=%+8.4f y=%+8.4f z=%+8.4f [g]  |a|=%.4f\n",
                  mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(),
                  sqrtf(mpu.getAccX() * mpu.getAccX() +
                        mpu.getAccY() * mpu.getAccY() +
                        mpu.getAccZ() * mpu.getAccZ()));
    Serial.printf("  gyro cal : x=%+8.3f y=%+8.3f z=%+8.3f [deg/s]\n",
                  mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ());
    Serial.printf("  angle    : roll=%+7.2f pitch=%+7.2f yaw=%+7.2f [deg]\n",
                  mpu.getRoll(), mpu.getPitch(), mpu.getYaw());
    Serial.println("  ※ 静止させて |a| が 1.00 に近ければ加速度計はOK");
    Serial.println("  ※ 静止させて gyro が 0 付近でなければ 'c' でキャリブレーション");
    Serial.println("------------------------\n");
}

static void handleSerial() {
    if (!Serial.available()) return;
    const char c = (char)tolower(Serial.read());

    switch (c) {
        case '1': case '2': case '3': case '4':
            startMotorTest(c - '1');
            break;
        case 'a':
            startAllTest();
            break;
        case 's':
            Serial.println("STOP");
            stopTest();
            break;
        case 'i':
            printImuDetail();
            break;
        case 'c':
            Serial.println("CALIBRATE: 機体を水平に置いて動かさないでください");
            stopTest();
            if (S1::USE_MPU) mpu.recalibrate();
            break;
        case 'h':
            Serial.println("\n[1-4] 個別モーター  [a] 全部  [s] 停止  "
                           "[i] IMU詳細  [c] IMUキャリブレーション  [h] ヘルプ\n");
            break;
        default:
            break;  // 改行などは捨てる
    }
}

// ============================================================
//  § 7  デバッグ表示
// ============================================================
static const char* swName(Sw s) {
    return (s == up) ? "UP " : (s == down) ? "DWN" : "CEN";
}

static void printStatus(uint32_t dt_us) {
    Serial.print("\033[2J\033[H");   // 画面クリア
    Serial.println("=== Stage 1 : 出力段の検証 (プロペラを外すこと) ===");
    Serial.printf("loop dt = %6lu us (%6.1f Hz)\n",
                  (unsigned long)dt_us, 1000000.0f / (float)dt_us);

    // ---- 受信機 ----
    if (S1::USE_SBUS) {
        Serial.printf("\n[SBUS] link=%s  arm=%s  (THR_CUT=%s)\n",
                      sbus.isSafe() ? "OK  " : "LOST",
                      isArmed()     ? "ARMED   " : "DISARMED",
                      swName(sbus.Ch_state(Ch::THR_CUT)));
        Serial.printf("  roll=%+6.2f pitch=%+6.2f thr=%5.2f yaw=%+6.2f\n",
                      sbus.des[Ch::ROLL], sbus.des[Ch::PITCH],
                      sbus.des[Ch::THR],  sbus.des[Ch::YAW]);
        Serial.printf("  sw5=%s sw6=%s sw7(cut)=%s sw8=%s\n",
                      swName(sbus.Ch_state(Ch::SW_TURN)),
                      swName(sbus.Ch_state(Ch::SW_LEVEL)),
                      swName(sbus.Ch_state(Ch::THR_CUT)),
                      swName(sbus.Ch_state(Ch::SW_HOVER)));
    } else {
        Serial.println("\n[SBUS] disabled");
    }

    // ---- IMU ----
    if (S1::USE_MPU) {
        Serial.printf("\n[IMU] angle  roll=%+7.2f pitch=%+7.2f yaw=%+7.2f [deg]\n",
                      mpu.getRoll(), mpu.getPitch(), mpu.getYaw());
        Serial.printf("      gyro   x=%+7.2f y=%+7.2f z=%+7.2f [deg/s]\n",
                      mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ());
        Serial.printf("      acc    x=%+7.3f y=%+7.3f z=%+7.3f [g]\n",
                      mpu.getAccX(), mpu.getAccY(), mpu.getAccZ());
    } else {
        Serial.println("\n[IMU] disabled");
    }

    // ---- モーター ----
    Serial.print("\n[MOTOR] ");
    if (g_test_all)            Serial.println("TEST: ALL");
    else if (g_test_motor >= 0) Serial.printf("TEST: M%d only\n", g_test_motor + 1);
    else if (isArmed())         Serial.printf("throttle passthrough = %.2f\n",
                                              sbus.des[Ch::THR]);
    else                        Serial.println("stopped (disarmed)");

    Serial.print("  pins: ");
    for (int i = 0; i < S1::MOTOR_COUNT; ++i) {
        Serial.printf("M%d=%d ", i + 1, S1::MOTOR_PIN[i]);
    }
    Serial.println();

    Serial.println("\n[1-4] 個別  [a] 全部  [s] 停止  [i] IMU詳細  [c] キャリブレーション");
}

// ============================================================
//  § 8  setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    const uint32_t start_ms = millis();
    while (!Serial && (millis() - start_ms < 2000)) { /* USBシリアルを少し待つ */ }

    Serial.println("\n\n=== Stage 1 : 出力段の検証 ===");
    Serial.println("!! プロペラを外してあることを確認してください !!");

    if (S1::USE_MOTOR) {
        Serial.println("Init motors...");
        // ※ 注意: motor::write() (src/sub_lib/Actuators.cpp) は 0.0-1.0 を
        //    1000-2000us 相当に固定でマッピングしており、set_minPWM/set_maxPWM は
        //    参照していません。設定しても効かないので、あえて呼んでいません。
        //    (auto_flight.cpp が set_minPWM(600) で呼んでいるため、Actuators.cpp 側を
        //     直すと別機体の出力が変わってしまいます。触っていません)
        for (int i = 0; i < S1::MOTOR_COUNT; ++i) {
            motors[i].set_pin(S1::MOTOR_PIN[i]).begin();
        }
        // ESC が初期化パルスを認識するまで少し待つ
        delay(500);
        stopAllMotors();
    }

    if (S1::USE_SBUS) {
        Serial.println("Init SBUS...");
        sbus.begin();
    }

    if (S1::USE_MPU) {
        Serial.println("Init IMU...");
        mpu.begin();
    }

    Serial.println("--- Setup complete ---");
    Serial.println("[h] でコマンド一覧");
}

void loop() {
    if (!main_tick.ready()) return;

    // --- 1) 入力の更新 ---
    if (S1::USE_MPU)  mpu.update();
    if (S1::USE_SBUS) sbus.update();

    // --- 2) シリアルコマンド ---
    handleSerial();

    // --- 3) モーター出力 (姿勢制御なし) ---
    updateMotors();

    // --- 4) 表示 ---
    if (debug_tick.ready()) printStatus(main_tick.dt_us);
}
