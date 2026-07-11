// ============================================================
// main.cpp  -  自律飛行対応版 (軽量化・カルマンフィルタ削除版)
// ============================================================

#include <Arduino.h>
#include <math.h>

#include <cctype>
#include <cmath>

#include "Config.h"

// 大量のヘッダファイル
#include "Actuators.h"
#include "Control.h"
#include "Receiver.h"
#include "Sensors.h"
#include "Serial_com.h"
#include "Telemetry.h"
#include "flight_mode.h"

namespace T = Config::Timing;

// ============================================================
//  インスタンス生成
// ============================================================

// ============================================================
//  § 1  PIDゲイン
// ============================================================
// setgains(kp_rate, ki_rate, kd_rate, kp_angle, ki_angle, kd_angle,
// sensitivity)
//
// [重要] 全て0からスタートし、必ず地上で手持ちしながら少しずつ上げること
//  手順:
//  1. kp_angle を上げて機体が目標角に向かうか確認
//  2. kp_rate  を上げてサーボの追従速度pを上げる
//  3. kd_rate  を足して振動を抑える
//  4. ki_rate  は最後に少しだけ足す

// kp_rate  ki_rate  kd_rate  kp_angle  ki_angle  kd_angle
//  sensitivity　rate_d_alpha, rate_i_limit　angle_d_alpha, angle_i_limit

Axis_value Roll(0.003f, 0.00f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.7f, 0.0f, 0.5f,
                0.0f),//angleをなくすときは291行目からの場所の、RateAnglePIDをRatePIDに変える
                // 0.3f, 0.0f, 0.01f, 0.00f, 0.0f, 0.0f, 1.0f, 0.7f, 0.0f, 0.0f, 0.0f),(pを感じた)
                //0.6f, 0.00f, 0.003f, 10.0f, 0.0f, 0.2f, 1.0f, 0.7f, 0.0f, 0.5f,0.0f,(水平に戻ろうとする、うれしい)
    Pitch(0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 1.0f, 0.8f, 0.0f, 0.0f, 0.0f),
    Yaw(0.001f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 1.0f, 0.8f, 0.0f, 0.0f, 0.0f);

float BANK_ANGLE = 25.0f;  // バンク角 [deg]  ← 0だとラダーも動かないので要注意
unsigned long TURN_MS = 4000UL;  // 8 of 8 or a single trip time [ms]
float RUDDER_COORD = 0.66;       // 協調ラダー量 [0.0~1.0]  1.0=全開, 0.0=なし
float LEVEL_PITCH_ANGLE = 10.0f; // LEVEL_FLIGHTで前傾する固定角度 [deg] ← 要調整、控えめから
float MAX_ANGLE_CMD = 30.0f;     // SEMI_MANUAL(ホバリング)でスティック全開時の目標角度 [deg]

IMU mpu(&Wire);
BarometerSensor barometer(1013.25, 0.1, &Wire);  // 海面気圧1013.25hPa, α=0.1
// EZ2Sensor       ez2(Config::sensor::EZ2_PW_PIN, Config::sensor::EZ2_ALPHA);
// // 停止中 (搭載無し)

Sbus sbus(&Serial5);
FlightTelemetry telemetry(&Serial2);

Flight_mode Mode;

motor motor1, motor2, motor3, motor4;  // Teensy4.X Pin 20
// ============================================================
//  プロトタイプ宣言
// ============================================================
void updateSensorsAndComms();
void autonomousControl();
void writeServos();
void reset_all();
// ============================================================
//  setup
// ============================================================
// ============================================================
//  § 組み合わせテスト用フラグ (ここを true/false で切り替えてください)
// ============================================================
bool USE_MPU = true;    // 加速度センサー (MPU6050)
bool USE_BARO = false;  // 気圧センサー (BMP280)
bool USE_SBUS = true;   // 受信機 (SBUS)
bool USE_IM920 = true;  // 無線モジュール (IM920)
bool USE_SERVO = true;  // サーボ・アンプ出力 (Servo/ESC)

void setup() {
  // サーボの宣言

  Serial.begin(115200);
  uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms < 2000))  // 多分いったん待機？
    ;
  Serial.println("\n\n--- TEENSY SYSTEM BOOT (High-Power Triage) ---");

  if (USE_MPU) {
    Serial.println("Init MPU...");
    mpu.begin();
  }

  if (USE_SBUS) {
    Serial.println("Init SBUS...");
    sbus.begin();
  }

  if (USE_SERVO) {
    Serial.println("Init Actuators...");

    motor1.set_pin(17).set_minPWM(1000).set_maxPWM(2000).begin();//1175,125
    motor2.set_pin(16).set_minPWM(1000).set_maxPWM(2000).begin();
    motor3.set_pin(15).set_minPWM(1000).set_maxPWM(2000).begin();
    motor4.set_pin(14).set_minPWM(1000).set_maxPWM(2000).begin();
  }

  if (USE_IM920) {
    Serial.println("Init IM920...");
    telemetry.begin();
  }

  if (USE_BARO) {
    Serial.println("Init Barometer...");
    if (!barometer.begin()) Serial.println("Barometer init failed!");
  }

  /*
  Serial.println("Init EZ2...");
  ez2.begin();
  */

  Serial.println("--- Setup Complete. Loop start ---");
}

// ============================================================
//  loop
// ============================================================
void loop() {
  // --- 究極の最小ループ (ボード本体生存確認) ---
  static uint32_t last_alive_ms = 0;
  if (millis() - last_alive_ms > 1000) {
    last_alive_ms = millis();
    Serial.printf("### Teensy Alive - MPU:%s BARO:%s SBUS:%s ###\n",
                  USE_MPU ? "ON" : "OFF", USE_BARO ? "ON" : "OFF",
                  USE_SBUS ? "ON" : "OFF");
  }

  if (T::freq<T::MAIN_Hz>(T::Main_dt)) {  // 周期制御 (1000Hz)
    // 1) センサ・受信機・通信の更新
    if (USE_MPU || USE_SBUS) {
      updateSensorsAndComms();
    }

    // モード切替時にPIリセット
    if (Mode.change()) {
      Mode.modeStartMs = millis();
      Roll.pid_reset();
      Pitch.pid_reset();
      Yaw.pid_reset();
    }

    // 3) モードに応じた制御演算
    if (USE_MPU && USE_SBUS) {
      autonomousControl();
    }

    // 3-2) サーボ出力 (ONの場合のみ)
    if (USE_SERVO) {
      writeServos();
    }

    // 4) スロットル出力 (ONの場合のみ)
    if (USE_SERVO) {
      if (sbus.isSafe()&&sbus.Ch_state(THR_CUT)==down) {  // スロットルカットがOFFの時のみ出力
        motor1.write(sbus.des[Ch::THR] + (sbus.des[Ch::THR]>0.1f ? constrain(-Pitch.cmd + Yaw.cmd, -0.1f, 0.1f) : 0));
        motor2.write(sbus.des[Ch::THR] - (sbus.des[Ch::THR]>0.1f ? constrain(-Roll.cmd + Yaw.cmd, -0.1f, 0.1f) : 0));
        motor3.write(sbus.des[Ch::THR] + (sbus.des[Ch::THR]>0.1f ? constrain(-Roll.cmd + Yaw.cmd, -0.1f, 0.1f) : 0));
        motor4.write(sbus.des[Ch::THR] - (sbus.des[Ch::THR]>0.1f ? constrain(-Pitch.cmd + Yaw.cmd, -0.1f, 0.1f) : 0));
      } else {
        motor1.write(0);
        motor2.write(0);
        motor3.write(0);
        motor4.write(0);
      }
    }

    // 6) テレメトリ・デバッグ (10Hz)
    static int dbg_cnt = 0;
    if (++dbg_cnt >= 100) {
      dbg_cnt = 0;
      if (USE_BARO) barometer.update();
      float fused_alt = (USE_BARO) ? barometer.get_smoothed_altitude() : 0.0f;

      // 姿勢データ送信 (28 bytes = float 7個、10Hz)
      if (USE_IM920) {
        telemetry.sendAttitude(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(),
                               Roll.ang, Pitch.ang, Yaw.ang, fused_alt);
      }

      Serial.print("\033[2J\033[H");
      Serial.printf("### MPU=%s BARO=%s SBUS=%s IM920=%s SERVO=%s ###\n",
                    USE_MPU ? "ON" : "OFF", USE_BARO ? "ON" : "OFF",
                    USE_SBUS ? "ON" : "OFF", USE_IM920 ? "ON" : "OFF",
                    USE_SERVO ? "ON" : "OFF");
      print_flightmode(Mode.get_mode(), BANK_ANGLE, TURN_MS);
      if (USE_MPU)
        print_MPU(Roll.ang, Pitch.ang, Yaw.ang, Roll.gyr, Pitch.gyr, Yaw.gyr);
      if (USE_SBUS)
        print_sbus(sbus.des[Ch::ROLL], sbus.des[Ch::PITCH], sbus.des[Ch::THR],
             sbus.des[Ch::YAW], sbus.des[Ch::SW_TURN], sbus.des[Ch::SW_LEVEL],
             sbus.des[Ch::SW_HOVER]);
      if (USE_BARO) Serial.printf("Alt: %+7.2f m\n", fused_alt);
      print_timing(T::Main_dt);

      // デバッグ用に一時的に追加
      Serial.printf("\n cmd=%+.4f\n", Roll.cmd);
    }
  }
}

// ============================================================
//  センサ・受信機・無線通信の更新
// ============================================================
void updateSensorsAndComms() {
  mpu.update();
  sbus.update();
  Mode.update(sbus.Ch_state(SW_TURN), sbus.Ch_state(SW_LEVEL), sbus.Ch_state(SW_HOVER));

  Roll.update_value(sbus.des[Ch::ROLL], mpu.getPitch(), mpu.getAccY(),
                    mpu.getGyroY());
  Pitch.update_value(sbus.des[Ch::PITCH], mpu.getRoll(), mpu.getAccX(),
                     mpu.getGyroX());
  Yaw.update_value(sbus.des[Ch::YAW], mpu.getYaw(), mpu.getAccZ(),
                   mpu.getGyroZ());

  switch (Update_SerialCommand()) {
    case 'R':
      reset_all();
      break;
    case 'P':
      handlePIDTuning(Roll, Pitch, Yaw);
      motor1.write(0);
      motor2.write(0);
      motor3.write(0);
      motor4.write(0);
      break;
    default:
      break;
  }
  // 地上局からパラメータ受信
  if (USE_IM920) {
    telemetry.receiveAndProcess(Roll, Pitch, Yaw, mpu, barometer, BANK_ANGLE,
                                TURN_MS);
  }
}

// ============================================================
//  自律制御
// ============================================================
void autonomousControl() {
  // --- 🔴 電波がない場合の「地上PIDセッティングモード」
  // ---(リポが断線していてプロポの電源の5Vをサーボ用に使っていた時用)
  if (!sbus.isSafe()) {
    Roll.tar = 0.0f;   // 目標ロール角 0度（常に水平を維持）
    Pitch.tar = 0.0f;  // 目標ピッチ角 0度（常に水平を維持）
    Yaw.tar = 0.0f;    // ラダー目標 0

    // 角度＆レートPIDを計算してコマンドを出力
    Roll.update_RateAnglePID();
    Pitch.update_RateAnglePID();
    Yaw.cmd =
        Yaw.sbus;  // 0固定ではなく、プロポのトリム位置（または受信機のフェイルセーフ位置）を維持

    return;  // 通常のフライトモード判定をスキップしてここで終了
  }

  // --- 🟢 電波がある場合（ここから下は元のコードそのまま） ---
  switch (Mode.get_mode()) {
    case MODE_LEVEL_TURN: {
      // 左旋回: 固定バンク角。ラダー協調は下の共通ブロックで自動適用される
      Roll.tar = -BANK_ANGLE;  // マイナスで左旋回
      break;
    }

    case MODE_LEVEL_FLIGHT: {
      // 固定角度で前傾しながら直進。スティックは無視する
      Roll.tar = 0.0f;
      Pitch.tar = LEVEL_PITCH_ANGLE;
      break;
    }

    case MODE_SEMI_MANUAL: {
      // ホバリング: スティック入力を目標角度としてPID制御
      // Yawはここではスティック直接(角度PIDを介さない)なので早期return
      Roll.tar  = Roll.sbus  * MAX_ANGLE_CMD;
      Pitch.tar = Pitch.sbus * MAX_ANGLE_CMD;
      Roll.update_RatePID();
      Pitch.update_RatePID();
      Yaw.cmd = Yaw.sbus;   // ヨーはスティックで直接レート操作
      return;
    }

    case MODE_MANUAL: {
      // PIDなし。スティックの倒し量をそのまま出力し続ける(通常ラジコン挙動)
      if (sbus.isSafe()) {
        Roll.cmd  = Roll.sbus;
        Pitch.cmd = Pitch.sbus;
        Yaw.cmd   = Yaw.sbus;
      } else {
        Roll.cmd = 0.0f;
        Pitch.cmd = 0.0f;
        Yaw.cmd = 0.0f;
      }
      return;
    }

    default:
      break;
  }

  // --- LEVEL_TURN / LEVEL_FLIGHT 共通: 角度外ループ+レート内ループ ---
  Roll.update_RateAnglePID();
  Pitch.update_RateAnglePID();

  if (Roll.tar > 0.1f)
    Yaw.cmd = Yaw.sbus + RUDDER_COORD;
  else if (Roll.tar < -0.1f)
    Yaw.cmd = Yaw.sbus - RUDDER_COORD;
  else
    Yaw.cmd = Yaw.sbus;
}
void reset_all() {
  mpu.recalibrate();
  barometer.reset();
  Config::Timing::resetTiming();
  Roll.pid_reset();
  Pitch.pid_reset();
  Yaw.pid_reset();
  Serial.println("INFO: Remote Reset Complete.");
}

void writeServos() {}
