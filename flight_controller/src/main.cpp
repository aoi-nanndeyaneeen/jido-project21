// ============================================================
// main.cpp  -  自律飛行対応版 (軽量化・カルマンフィルタ削除版)
// ============================================================

#include <Arduino.h>
#include <cctype>
#include <cmath>
#include "Config.h"
#include "Telemetry.h"
#include "Control.h"
#include "Sensors.h"
#include "Receiver.h"
#include "Actuators.h"
#include "Serial_com.h"
#include "flight_mode.h"

namespace T = Config::Timing;

// ============================================================
//  インスタンス生成
// ============================================================

Axis_value  Roll(ROLL_gain.kp_rate, ROLL_gain.ki_rate, ROLL_gain.kd_rate, ROLL_gain.kp_angle, ROLL_gain.ki_angle, ROLL_gain.kd_angle, ROLL_gain.sensitivity, ROLL_gain.rate_d_alpha, ROLL_gain.rate_i_limit, ROLL_gain.angle_d_alpha, ROLL_gain.angle_i_limit),
            Pitch(PITCH_gain.kp_rate, PITCH_gain.ki_rate, PITCH_gain.kd_rate, PITCH_gain.kp_angle, PITCH_gain.ki_angle, PITCH_gain.kd_angle, PITCH_gain.sensitivity, PITCH_gain.rate_d_alpha, PITCH_gain.rate_i_limit, PITCH_gain.angle_d_alpha, PITCH_gain.angle_i_limit),
            Yaw(YAW_gain.kp_rate, YAW_gain.ki_rate, YAW_gain.kd_rate, YAW_gain.kp_angle, YAW_gain.ki_angle, YAW_gain.kd_angle, YAW_gain.sensitivity, YAW_gain.rate_d_alpha, YAW_gain.rate_i_limit, YAW_gain.angle_d_alpha, YAW_gain.angle_i_limit);

IMU mpu(Config::wire::mpu);
BarometerSensor barometer(1013.25, 0.1, Config::wire::bmp);

Sbus sbus(Config::serial::sbus);
IM920SL_Generic<PlaneData, GroundData> im920(Config::serial::im920);

PlaneData  Plane_Data;
GroundData Ground_Data;

Flight_mode Mode;

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
void updateSensorsAndComms();
void autonomousControl();
void writeServos();

// ============================================================
//  setup
// ============================================================
void setup() {
    mpu.begin();
    sbus.begin();

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
    if (T::freq<T::MAIN_Hz>(T::Main_dt)) { // 周期制御 (1000Hz)
        // 1) センサ・受信機・通信の更新
        updateSensorsAndComms();

        // モード切替時にPIリセット
        if (Mode.change()) {
            Mode.modeStartMs = millis();
            Roll.pid_reset();   Pitch.pid_reset();  Yaw.pid_reset();
        }

        // 3) モードに応じた制御演算
        autonomousControl();
        writeServos();

        // 4) スロットル出力
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

        // 6) テレメトリ・デバッグ (10Hz)
        static int dbg_cnt = 0;
        if (++dbg_cnt >= 100) {
            dbg_cnt = 0;
            barometer.update();
            Plane_Data.update(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(),
                             mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ(),
                             barometer.get_smoothed_altitude());

            Serial.print("\033[2J\033[H"); // 常にシリアルモニタの上に表示する

            print_flightmode(int(Mode.get_mode()), BANK_ANGLE, TURN_MS);
            print_PID(Roll.pid, Pitch.pid, Yaw.pid);
            print_MPU(Roll.ang, Pitch.ang, Yaw.ang, Roll.gyr, Pitch.gyr, Yaw.gyr);
            print_ACC(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ());
            print_sbus(sbus.des[Ch::ROLL], sbus.des[Ch::PITCH], sbus.des[Ch::THR], sbus.des[Ch::YAW], sbus.des[Ch::Aux1], sbus.des[Ch::Aux2], sbus.des[Ch::Aux3]);
            Ground_Data.print();
            
            Serial.printf("Alt: %+7.2f m\n", Plane_Data.altitude);
            print_timing(T::Main_dt);
        }
    }
}

// ============================================================
//  センサ・受信機・無線通信の更新
// ============================================================
void updateSensorsAndComms() {
    mpu.update();
    sbus.update();
    Mode.update(sbus.Ch_state(Aux1), sbus.Ch_state(Aux2));

    Roll.update_value(sbus.des[Ch::ROLL],   mpu.getRoll(),  mpu.getAccX(), mpu.getGyroX());
    Pitch.update_value(sbus.des[Ch::PITCH], mpu.getPitch(), mpu.getAccY(), mpu.getGyroY());
    Yaw.update_value(sbus.des[Ch::YAW],     mpu.getYaw(),   mpu.getAccZ(), mpu.getGyroZ());

    // シリアルコマンド (R:リセット)
    if (Serial.available()) {
        char c = toupper(Serial.read());
        if (c == 'R') {
            mpu.recalibrate();
            barometer.reset();
            T::resetTiming();
            Roll.pid_reset(); Pitch.pid_reset(); Yaw.pid_reset();
            Serial.println("INFO: Reset Complete (Acc to 0,0,1).");
        }
    }

    // 地上局からパラメータ受信
    if (im920.read(Ground_Data)) {
        if (Ground_Data.roll  != 0.0f) BANK_ANGLE = fabsf(Ground_Data.roll);
        if (Ground_Data.pitch != 0.0f) TURN_MS    = (unsigned long)(Ground_Data.pitch * 1000.0f);
        
        if (Ground_Data.p_adj != 0.0f || Ground_Data.i_adj != 0.0f || Ground_Data.d_adj != 0.0f) {
            Roll.Rate_PID_adj(Ground_Data.p_adj, Ground_Data.i_adj, Ground_Data.d_adj);
            Pitch.Rate_PID_adj(Ground_Data.p_adj, Ground_Data.i_adj, Ground_Data.d_adj);
        }
    }
}

// ============================================================
//  自律制御
// ============================================================
void autonomousControl() {
    float target_roll  = 0.0f;
    float target_pitch = 0.0f;

    switch (Mode.get_mode()) {
        case MODE_LEVEL_TURN:
            target_roll = +BANK_ANGLE;
            break;

        case MODE_FIGURE_8: {
            unsigned long elapsed = millis() - Mode.modeStartMs;
            int phase = (int)(elapsed / TURN_MS) % 2;
            target_roll = (phase == 0) ? +BANK_ANGLE : -BANK_ANGLE;
            break;
        }

        case MODE_MANUAL:
            Roll.tar = Roll.sbus;
            Pitch.tar= Pitch.sbus;
            Yaw.tar  = Yaw.sbus;
            Roll.update_RatePID();
            Pitch.update_RatePID();
            Yaw.update_RatePID();
            return;
        
        default: break;
    }

    Roll.tar  = target_roll;
    Pitch.tar = target_pitch;

    Roll.update_RateAnglePID();
    Pitch.update_RateAnglePID();

    Yaw.cmd = constrain(target_roll * 0.05f, -1.0f, 1.0f);
}

void writeServos() {
    Ail1.write(Roll.cmd);
    Ail2.write(Roll.cmd);
    Ele.write(Pitch.cmd);
    Rud.write(Yaw.cmd);
}
