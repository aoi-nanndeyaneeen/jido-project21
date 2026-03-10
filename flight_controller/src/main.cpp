// ============================================================
// main.cpp  -  自律飛行対応版
//
// 【モード切替チャンネル割り当て】(プロポ側で要確認)
//   Aux2 (ch7/index=6) : 上 → 自動水平旋回
//   Aux3 (ch8/index=7) : 上 → 8の字
//   優先度: 8の字 > 水平旋回 > マニュアル
//   ※両スイッチがOFFでマニュアルに戻る
//
// 【GroundDataによる地上局からのパラメータ変更】
//   Ground_Data.roll  : バンク角 [deg] (0なら変更なし)
//   Ground_Data.pitch : 8の字の片道時間 [s] (0なら変更なし)
// ============================================================

#include <Arduino.h>
#include "Config.h"
#include "Telemetry.h"
#include "Control.h"
#include "Sensors.h"
#include "Receiver.h"
#include "Actuators.h"

// ============================================================
//  グローバル変数の実体定義
// ============================================================
unsigned long dt;
int counter;

// ============================================================
//  飛行モード定義
// ============================================================
enum FlightMode : uint8_t {
    MODE_MANUAL     = 0,  // プロポ直接操作
    MODE_LEVEL_TURN = 1,  // 自動水平旋回
    MODE_FIGURE_8   = 2,  // 8の字飛行
};
FlightMode currentMode = MODE_MANUAL;
FlightMode prevMode    = MODE_MANUAL;  // モード変化検出用

// ============================================================
//  自律飛行パラメータ
// ============================================================
float         BANK_ANGLE = 40.0f;    // バンク角 [deg]
unsigned long TURN_MS    = 4000UL;   // 8の字の片道時間 [ms]
unsigned long modeStartMs = 0;

// ============================================================
//  インスタンス生成
// ============================================================
Axis_value Roll, Pitch, Yaw;
mpu_value MPU;
Sbus sbus(&Serial2);
BarometerSensor barometer(1013.25, 0.1);

PlaneData  Plane_Data;
GroundData Ground_Data;
IM920SL_Generic<PlaneData, GroundData> im920(&Serial3);

RC_servo Ail1(1, 0.0, -1.0, 1.0),
         Ail2(6, 0.0, -1.0, 1.0),
         Ele (2, 0.0, -1.0, 1.0),
         Rud (4, 0.0, -1.0, 1.0),
         Flp1(8, 0.0, -1.0, 1.0),
         Flp2(9, 0.0, -1.0, 1.0);
RC_motor Thr_r(3, 1.0), Thr_l(5, 1.0);

// ============================================================
//  プロトタイプ宣言
// ============================================================
FlightMode getFlightMode();
void       updateSensorsAndComms();
void       manualControl();
void       autonomousControl();
void       writeServos(bool autonomous);
void       print_PID(float r, float p, float y);
void       print_MPU();
void       print_sbus();

// ============================================================
//  setup
// ============================================================
void setup() {
    MPU.begin();
    sbus.begin();

    // ---- PIDゲイン設定 ----
    // setgains(kp_rate, ki_rate, kd_rate, kp_angle, ki_angle, kd_angle, sensitivity)
    //
    // [重要] 全て0からスタートし、必ず地上で手持ちしながら少しずつ上げること
    //  手順:
    //  1. kp_angleを上げて機体が目標角に向かうか確認
    //  2. kp_rateを上げてサーボの追従速度を上げる
    //  3. kd_rateを足して振動を抑える
    //  4. ki_rateは最後に少しだけ足す

    Roll.setgains(
        0.01f, 0.00f, 0.00f,   // レートPID: kp, ki, kd
        1.0f, 0.0f, 0.01f,   // 角度PID:  kp, ki, kd
        1.0f                  // 感度(Sensitivity)
    );
    Pitch.setgains(
        0.4f, 0.01f, 0.04f,
        1.5f, 0.00f, 0.00f,
        1.0f
    );
    Yaw.setgains(
        0.02f, 0.00f, 0.00f,   // ラダーはレートのみ
        0.0f, 0.00f, 0.00f,
        1.0f
    );

    Ail1.begin(); Ail2.begin();
    Ele.begin();  Rud.begin();
    Flp1.begin(); Flp2.begin();
    Thr_r.begin(); Thr_l.begin();

    im920.begin();
    Serial.begin(115200);
    if (!barometer.begin()) Serial.println("Barometer init failed!");
}

// ============================================================
//  loop
// ============================================================
void loop() {
    if (!frec()) return; // 周期制御 (1000Hz)

    // 1) センサ・受信機・通信の更新
    updateSensorsAndComms();

    // 2) モード判定
    currentMode = getFlightMode();

    // モード切替時にPIリセットとタイマ更新
    if (currentMode != prevMode) {
        modeStartMs = millis();
        prevMode    = currentMode;
        // 積分リセットはAxis_value経由でPIDがリセットされる
        // (現状のPIDクラスにreset関数がないため遷移時は自然減衰に任せる)
    }

    // 3) モードに応じた制御演算
    if (currentMode == MODE_MANUAL) {
        manualControl();
        writeServos(false);
    } else {
        autonomousControl();
        writeServos(true);
    }

    // 4) スロットル出力 (モード問わずスティック入力)
    if (sbus.isSafe()) {
        Thr_r.write(sbus.des[Ch::THR]);
        Thr_l.write(sbus.des[Ch::THR]);
    } else {
        Thr_r.write(0);
        Thr_l.write(0);
    }

    // 5) フラップ
    Flp1.flap(sbus.Ch_state(Aux1));
    Flp2.flap(sbus.Ch_state(Aux1));

    // 6) テレメトリ・デバッグ (1000Hz ÷ 100 = 10Hz)
    if (counter % 100 == 0) {
        barometer.update();
        Plane_Data.ax       = MPU.getRoll();
        Plane_Data.ay       = MPU.getPitch();
        Plane_Data.az       = MPU.getYaw();
        Plane_Data.gx       = MPU.getGyroX();
        Plane_Data.gy       = MPU.getGyroY();
        Plane_Data.gz       = MPU.getGyroZ();
        Plane_Data.altitude = barometer.get_smoothed_altitude();

        Serial.print("\033[2J\033[H"); // ターミナルクリア
        im920.write(Plane_Data);

        // モード表示
        const char* modeStr[] = {"MANUAL", "LEVEL_TURN", "FIGURE_8"};
        Serial.print("Mode: "); Serial.print(modeStr[(int)currentMode]);
        Serial.print("  BankAngle: "); Serial.print(BANK_ANGLE, 1);
        Serial.print(" deg  TurnMs: "); Serial.print(TURN_MS);
        Serial.println(" ms");

        print_PID(Roll.pid, Pitch.pid, Yaw.pid);
        print_MPU();
        print_sbus();
        Ground_Data.print();

        Serial.print("Altitude: "); Serial.print(Plane_Data.altitude, 2);
        Serial.println(" m");
    }
}

// ============================================================
//  センサ・受信機・無線通信の更新
// ============================================================
void updateSensorsAndComms() {
    MPU.update();
    sbus.update();

    // Axis_valueへセンサ値を流し込む (des は後で上書きされるので0でOK)
    Roll.update_value(sbus.des[Ch::ROLL],  MPU.getRoll(),  MPU.getAccX(), MPU.getGyroX());
    Pitch.update_value(sbus.des[Ch::PITCH], MPU.getPitch(), MPU.getAccY(), MPU.getGyroY());
    Yaw.update_value(sbus.des[Ch::YAW],   MPU.getYaw(),   MPU.getAccZ(), MPU.getGyroZ());

    // 地上局からパラメータ受信
    if (im920.read(Ground_Data)) {
        if (Ground_Data.roll  != 0.0f) BANK_ANGLE = fabsf(Ground_Data.roll);
        if (Ground_Data.pitch != 0.0f) TURN_MS    = (unsigned long)(Ground_Data.pitch * 1000.0f);
        // PIDゲイン調整 (Rollのレートゲインに適用)
        if (Ground_Data.p_adj != 0.0f || Ground_Data.i_adj != 0.0f || Ground_Data.d_adj != 0.0f) {
            Roll.Rate_PID_adj(Ground_Data.p_adj, Ground_Data.i_adj, Ground_Data.d_adj);
            Pitch.Rate_PID_adj(Ground_Data.p_adj, Ground_Data.i_adj, Ground_Data.d_adj);
        }
    }
}

// ============================================================
//  モード判定 (スイッチ読み取り)
// ============================================================
FlightMode getFlightMode() {
    // 優先度: 8の字 > 水平旋回 > マニュアル
    if (sbus.Ch_state(Ch::Aux3) == up) return MODE_FIGURE_8;
    if (sbus.Ch_state(Ch::Aux2) == up) return MODE_LEVEL_TURN;
    return MODE_MANUAL;
}

// ============================================================
//  マニュアル制御: スティック → レートPID → サーボ
// ============================================================
void manualControl() {
    // Roll.desはupdate_value()でスティック値が入っている
    Roll.update_RatePID(Roll.des);
    Pitch.update_RatePID(Pitch.des);
    Yaw.update_RatePID(Yaw.des);
}

// ============================================================
//  自律制御: 目標バンク角 → 角度PID → レートPID → サーボ
// ============================================================
void autonomousControl() {
    float target_roll  = 0.0f;
    float target_pitch = 0.0f; // ピッチは常に水平維持

    // --- 目標バンク角の決定 ---
    switch (currentMode) {
        case MODE_LEVEL_TURN:
            // ★ 左旋回にしたい場合は -BANK_ANGLE にする
            target_roll = +BANK_ANGLE;
            break;

        case MODE_FIGURE_8: {
            // 時間で左右を切り替える (TURN_MS ごとに反転)
            unsigned long elapsed = millis() - modeStartMs;
            int phase = (int)(elapsed / TURN_MS) % 2;
            target_roll = (phase == 0) ? +BANK_ANGLE : -BANK_ANGLE;
            break;
        }
        default: break;
    }

    // --- Axis_valueの目標値を上書き ---
    Roll.des  = target_roll;
    Pitch.des = target_pitch;

    // --- 角度外ループ + レート内ループ (Control.hのupdate_RateAnglePID使用) ---
    //   角度PID: 100Hzで目標レートを算出 (counter%10)
    //   レートPID: 1000Hzでサーボ指令を算出
    Roll.update_RateAnglePID();
    Pitch.update_RateAnglePID();

    // --- 協調ラダー: バンク角に比例してラダーを打つ ---
    //   係数(0.05f)は実機でのスリップ量を見ながら調整
    Yaw.pid = constrain(target_roll * 0.05f, -1.0f, 1.0f);
}

// ============================================================
//  サーボ書き込み
//  autonomous=true  → PID出力を使用
//  autonomous=false → スティック直値を使用
// ============================================================
void writeServos(bool autonomous) {
    if (autonomous) {
        Ail1.write(Roll.pid);
        Ail2.write(Roll.pid);
        Ele.write(Pitch.pid);
        Rud.write(Yaw.pid);
    } else {
        Ail1.write(Roll.des);
        Ail2.write(Roll.des);
        Ele.write(Pitch.des);
        Rud.write(Yaw.des);
    }
}

// ============================================================
//  デバッグ表示
// ============================================================
void print_PID(float r, float p, float y) {
    Serial.print("|PIDRoll= ");  Serial.print(r, 3);
    Serial.print("|PIDPitch= "); Serial.print(p, 3);
    Serial.print("|PIDYaw= ");   Serial.print(y, 3);
    Serial.print("\n");
}

void print_MPU() {
    Serial.print(Roll.ang, 2);  Serial.print(",");
    Serial.print(Pitch.ang, 2); Serial.print(",");
    Serial.print(Yaw.ang, 2);   Serial.print(",");
    Serial.print(Roll.gyr, 2);  Serial.print(",");
    Serial.print(Pitch.gyr, 2); Serial.print(",");
    Serial.print(Yaw.gyr, 2);   Serial.print(",");
    Serial.print("\n");
}

void print_sbus() {
    Serial.print("|ch1(ail) = ");  Serial.print(Roll.des, 2);
    Serial.print(" |ch2(ele) = "); Serial.print(Pitch.des, 2);
    Serial.print(" |ch3(thr) = "); Serial.print(sbus.des[Ch::THR], 2);
    Serial.print(" |ch4(rud) = "); Serial.print(Yaw.des, 2);
    Serial.print(" |Aux1(flp)= "); Serial.print(sbus.des[Ch::Aux1], 2);
    Serial.print(" |Aux2(trn)= "); Serial.print(sbus.des[Ch::Aux2], 2);
    Serial.print(" |Aux3(fig)= "); Serial.print(sbus.des[Ch::Aux3], 2);
    Serial.print("\n");
}