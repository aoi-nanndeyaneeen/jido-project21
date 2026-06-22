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

Axis_value Roll(-0.005f, 0.0f, 0.0f, 15.0f, 0.0f, 0.0f, 1.0f, 0.8f, 0.0f, 0.0f,
                0.0f),
    Pitch(0.003f, 0.0f, 0.0f, 8.0f, 0.0f, 0.0f, 1.0f, 0.8f, 0.0f, 0.0f, 0.0f),
    Yaw(-0.03f, 0.0f, 0.0f, -1.5f, 0.0f, 0.0f, 1.0f, 0.8f, 0.0f, 0.0f, 0.0f);

float BANK_ANGLE = 25.0f;  // バンク角 [deg]  ← 0だとラダーも動かないので要注意
unsigned long TURN_MS = 4000UL;  // 8 of 8 or a single trip time [ms]
float RUDDER_COORD = 0.66;       // 協調ラダー量 [0.0~1.0]  1.0=全開, 0.0=なし


// EZ2Sensor       ez2(Config::sensor::EZ2_PW_PIN, Config::sensor::EZ2_ALPHA);
// // 停止中 (搭載無し)
Sbus Main_sbus(&Serial4);
Sbus sbus(&Serial5);


Flight_mode Mode;

Norm_Servo Ail1, Ail2, Ele, Rud;
Flap_Servo Flp1, Flp2;
motor Thr;

// ============================================================
//  プロトタイプ宣言
// ============================================================
void updateSensorsAndComms();

void writeServos();
void reset_all();
// ============================================================
//  setup
// ============================================================
// ============================================================
//  § 組み合わせテスト用フラグ (ここを true/false で切り替えてください)
// ============================================================
bool USE_MPU = false;    // 加速度センサー (MPU6050)
bool USE_BARO = false;   // 気圧センサー (BMP280)
bool USE_SBUS = true;    // 受信機 (SBUS)
bool USE_IM920 = false;  // 無線モジュール (IM920)
bool USE_SERVO = true;   // サーボ・アンプ出力 (Servo/ESC)

void setup() {
  // サーボの宣言
  Ail1.set_pin(1).set_endpoints(-1.0, 1.0).set_offset(-0.15).begin();

  Ail2.set_pin(6)
      .set_endpoints(-1.0, 1.0)// リバース
      .set_offset(-0.3)  
      .begin();

  Ele.set_pin(2).set_offset(0.5).set_endpoints(-1.0, 1.0).begin();

  Rud.set_pin(10)
      .set_endpoints(-1.0, 1.0)  // リバース
      .begin();

  Flp1.set_pin(11).set_endpoints(-1.0, 1.0).begin();

  Flp2.set_pin(9).set_endpoints(-1.0, 1.0).begin();

  Thr.set_pin(3).set_minPWM(600).set_maxPWM(2000).begin();

  Serial.begin(115200);
  uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms < 2000));
  Serial.println("\n\n--- TEENSY SYSTEM BOOT (High-Power Triage) ---");


  if (USE_SBUS) {
    Serial.println("Init SBUS...");
    sbus.begin();
    Main_sbus.begin();
  }

  if (USE_SERVO) {
    Serial.println("Init Actuators...");
    Ail1.begin();
    Ail2.begin();
    Ele.begin();
    Rud.begin();
    Flp1.begin();
    Flp2.begin();
    Thr.begin();
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



    // 3) モードに応じた制御演算


    // 3-2) サーボ出力 (ONの場合のみ)
    if (USE_SERVO) {
      writeServos();
    }

    // 4) スロットル出力 (ONの場合のみ)
    if (USE_SERVO) {
      if (sbus.isSafe() || Main_sbus.isSafe()) {
        if (Main_sbus.Ch_state(Aux2) ==up||Main_sbus.Ch_state(THR_CUT) == up) {  // 教官側のスロットルカットがoffなら操作が教官側優先(sbus2)
          Thr.write(Main_sbus.des[Ch::THR]);
        } else if (Main_sbus.Ch_state(Aux2) == cen) {
          Thr.write(sbus.des[Ch::THR] * 0.7f + Main_sbus.des[Ch::THR] * 0.3f);
        } else {
          Thr.write(sbus.des[Ch::THR]);
        }
      } else {
        Thr.write(0);
      }
    }

    // 5) フラップ (ONの場合のみ)
      if (sbus.Ch_state(THR_CUT) == up) {
        Flp1.write(sbus.Ch_state(Aux1));
        Flp2.write(sbus.Ch_state(Aux1));

      } else {
        Flp1.write(Main_sbus.Ch_state(Aux1));
        Flp2.write(Main_sbus.Ch_state(Aux1));
      }

    // 6) テレメトリ・デバッグ (10Hz)
    static int dbg_cnt = 0;
    if (++dbg_cnt >= 100) {
      dbg_cnt = 0;


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
                   sbus.des[Ch::YAW], sbus.des[Ch::Aux1], sbus.des[Ch::Aux2],
                   sbus.des[Ch::Aux3]);
      print_sbus(Main_sbus.des[Ch::ROLL], Main_sbus.des[Ch::PITCH],
                 Main_sbus.des[Ch::THR], Main_sbus.des[Ch::YAW],
                 Main_sbus.des[Ch::Aux1], Main_sbus.des[Ch::Aux2],
                 Main_sbus.des[Ch::Aux3]);

      print_timing(T::Main_dt);
    }
  }
}

// ============================================================
//  センサ・受信機・無線通信の更新
// ============================================================
void updateSensorsAndComms() {

  sbus.update();
  Main_sbus.update();
  switch (Main_sbus.Ch_state(Aux2)) {
    case down:
      Roll.update_value(sbus.des[Ch::ROLL], 0,0,
                        0);
      Pitch.update_value(sbus.des[Ch::PITCH], 0, 0,
                         0);
      Yaw.update_value(sbus.des[Ch::YAW],0, 0,
                      0);
      break;
    case cen:
      Roll.update_value(
          constrain(sbus.des[Ch::ROLL] + Main_sbus.des[Ch::ROLL] * 1.3, -1, 1),
          0, 0, 0);
      Pitch.update_value(
          constrain(sbus.des[Ch::PITCH] + Main_sbus.des[Ch::PITCH] * 1.3, -1, 1),
          0, 0, 0);
      Yaw.update_value(
          constrain(sbus.des[Ch::YAW] + Main_sbus.des[Ch::YAW] * 1.3, -1, 1),
          0, 0, 0);
      break;

    case up:
      Roll.update_value(Main_sbus.des[Ch::ROLL], 0, 0,
                        0);
      Pitch.update_value(Main_sbus.des[Ch::PITCH], 0, 0, 0);
      Yaw.update_value(Main_sbus.des[Ch::YAW], 0, 0,
                       0);
      break;
    default:
      break;
  }

  switch (Update_SerialCommand()) {
    case 'R':
      reset_all();
      break;
    case 'P':
      handlePIDTuning(Roll, Pitch, Yaw);
      break;
    default:
      break;
  }
  // 地上局からパラメータ受信

}

void reset_all() {

  Config::Timing::resetTiming();
  Roll.pid_reset();
  Pitch.pid_reset();
  Yaw.pid_reset();
  Serial.println("INFO: Remote Reset Complete.");
}

void writeServos() {
  Roll.cmd = Roll.sbus;
  Pitch.cmd = Pitch.sbus;
  Yaw.cmd = Yaw.sbus;
 if(Main_sbus.Ch_state(THR_CUT) == up){
    Ail1.write(Main_sbus.des[Ch::ROLL]);
    Ail2.write(Main_sbus.des[Ch::ROLL]);
    Ele.write(Main_sbus.des[Ch::PITCH]);
    Rud.write(Main_sbus.des[Ch::YAW]);
 }else{
  Ail1.write(Roll.cmd);
  Ail2.write(Roll.cmd);
  Ele.write(Pitch.cmd);
  Rud.write(Yaw.cmd);
}
}