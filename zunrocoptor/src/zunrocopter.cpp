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
#include "Serial_com.h"
#include "flight_mode.h"

namespace T = Config::Timing; // T に凝縮


// ============================================================
//  インスタンス生成
// ============================================================

Axis_value  Roll(ROLL_gain.kp_rate, ROLL_gain.ki_rate, ROLL_gain.kd_rate, ROLL_gain.kp_angle, ROLL_gain.ki_angle, ROLL_gain.kd_angle, ROLL_gain.sensitivity, ROLL_gain.rate_d_alpha, ROLL_gain.rate_i_limit, ROLL_gain.angle_d_alpha, ROLL_gain.angle_i_limit),
            Pitch(PITCH_gain.kp_rate, PITCH_gain.ki_rate, PITCH_gain.kd_rate, PITCH_gain.kp_angle, PITCH_gain.ki_angle, PITCH_gain.kd_angle, PITCH_gain.sensitivity, PITCH_gain.rate_d_alpha, PITCH_gain.rate_i_limit, PITCH_gain.angle_d_alpha, PITCH_gain.angle_i_limit),
            Yaw(YAW_gain.kp_rate, YAW_gain.ki_rate, YAW_gain.kd_rate, YAW_gain.kp_angle, YAW_gain.ki_angle, YAW_gain.kd_angle, YAW_gain.sensitivity, YAW_gain.rate_d_alpha, YAW_gain.rate_i_limit, YAW_gain.angle_d_alpha, YAW_gain.angle_i_limit);
            //kp_rate, ki_rate, kd_rate, kp_angle, ki_angle, kd_angle, sensitivity, rate_d_alpha, rate_i_limit, angle_d_alpha, angle_i_limit
            //とってもながい

IMU mpu(Config::wire::mpu);
//BarometerSensor barometer(1013.25, 0.1, Config::wire::bmp);

Sbus sbus(Config::serial::sbus);
//IM920SL_Generic<PlaneData, GroundData> im920(Config::serial::im920);


PlaneData  Plane_Data;
GroundData Ground_Data;

Flight_mode Mode;
//RC_servo(int pin,float offset, float end1, float end2,bool reverse = false, int minPWM = 1000, int maxPWM = 2000) 
RC_servo Ele_von1(5, 0.0, -1.0, 1.0),
         Ele_von2(6, 0.0, -1.0, 1.0,true),
         Ele (24, 0.0, -1.0, 1.0),
         Rud (25, 0.0, -1.0, 1.0);
RC_motor Thr(9, 1.0);

// ============================================================
//  プロトタイプ宣言
// ============================================================
void       updateSensorsAndComms();
void       autonomousControl();
void       writeServos();

// ============================================================
//  setup
// ============================================================
void setup() {
    mpu.begin();
    sbus.begin();


    Ele_von1.begin(); Ele_von2.begin();
    Ele.begin();
    Rud.begin();
    Thr.begin();

    //im920.begin();
    Serial.begin(115200);
    //if (!barometer.begin()) Serial.println("Barometer init failed!");
}

// ============================================================
//  loop
// ============================================================
void loop() {
    if (T::freq<T::MAIN_Hz>(T::Main_dt)){// 周期制御 (1000Hz)
        // 1) センサ・受信機・通信の更新
        updateSensorsAndComms();


        // モード切替時にPIリセットとタイマ更新
        if (Mode.change()) {
            Mode.modeStartMs = millis();
            Roll.pid_reset();   Pitch.pid_reset();  Yaw.pid_reset();
        }

        // 3) モードに応じた制御演算
            autonomousControl();
            writeServos();

        // 4) スロットル出力 (モード問わずスティック入力)
        if (sbus.isSafe()) {
            Thr.write(sbus.des[Ch::THR]);
        } else {
            Thr.write(0);
        }

        // 6) テレメトリ・デバッグ (1000Hz ÷ 100 = 10Hz)
        //    ★ 1000Hzブロック内に配置し、Serial出力がループをブロックしないようにする
        static int dbg_cnt = 0;
        if (++dbg_cnt >= 100) {
            dbg_cnt = 0;
            //barometer.update();
            Plane_Data.update(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(),
                             mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ(),
                             //barometer.get_smoothed_altitude()
                             0.0f);

            // モード表示
            print_flightmode(int(Mode.get_mode()), BANK_ANGLE, TURN_MS);

            Serial.print("\033[2J\033[H"); // ターミナルクリア
            //im920.write(Plane_Data);

            print_PID(Roll.pid, Pitch.pid, Yaw.pid);
            print_MPU(Roll.ang, Pitch.ang, Yaw.ang, Roll.gyr, Pitch.gyr, Yaw.gyr);
            print_ACC(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ());
            print_sbus(sbus.des[Ch::ROLL], sbus.des[Ch::PITCH], sbus.des[Ch::THR], sbus.des[Ch::YAW], sbus.des[Ch::Aux1], sbus.des[Ch::Aux2], sbus.des[Ch::Aux3]);
            Ground_Data.print();

            Serial.print("Altitude: "); Serial.print(Plane_Data.altitude, 2);
            Serial.println(" m");
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
    Mode.update(sbus.Ch_state(Aux1),sbus.Ch_state(Aux2));

    // Axis_valueへセンサ値を流し込む (des は後で上書きされるので0でOK)
    Roll.update_value(sbus.des[Ch::ROLL],  mpu.getRoll(),  mpu.getAccX(), mpu.getGyroX());
    Pitch.update_value(sbus.des[Ch::PITCH], mpu.getPitch(), mpu.getAccY(), mpu.getGyroY());
    Yaw.update_value(sbus.des[Ch::YAW],   mpu.getYaw(),   mpu.getAccZ(), mpu.getGyroZ());

    // --- シリアルコマンド (PCモニタからのRキー等) ---
    if (reset()) {
        mpu.recalibrate();
        //barometer.reset();
        Config::Timing::resetTiming();
        Roll.pid_reset(); Pitch.pid_reset(); Yaw.pid_reset();
        Serial.println("INFO: System-wide Reset Complete.");
    }

    // 地上局からパラメータ受信
    // if (im920.read(Ground_Data)) {
    //     if (Ground_Data.roll  != 0.0f) BANK_ANGLE = fabsf(Ground_Data.roll);
    //     if (Ground_Data.pitch != 0.0f) TURN_MS    = (unsigned long)(Ground_Data.pitch * 1000.0f);//これやってることすごい
    //     // PIDゲイン調整 (Rollのレートゲインに適用)
    //     if (Ground_Data.p_adj != 0.0f || Ground_Data.i_adj != 0.0f || Ground_Data.d_adj != 0.0f) {
    //         Roll.Rate_PID_adj(Ground_Data.p_adj, Ground_Data.i_adj, Ground_Data.d_adj);
    //         //Pitch.Rate_PID_adj(Ground_Data.p_adj, Ground_Data.i_adj, Ground_Data.d_adj);
    //     }
    // }
}

// ============================================================
//  自律制御: 目標バンク角 → 角度PID → レートPID → サーボ
// ============================================================
void autonomousControl() {


    // --- 目標バンク角の決定 ---
    switch (Mode.get_mode()) {
        case MODE_LEVEL_TURN:
            // ★ 左旋回にしたい場合は -BANK_ANGLE にする
            Roll.tar = +BANK_ANGLE;
            break;

        case MODE_FIGURE_8: {
            // 時間で左右を切り替える (TURN_MS ごとに反転)
            unsigned long elapsed = millis() - Mode.modeStartMs;
            int phase = (int)(elapsed / TURN_MS) % 2;
            Roll.tar = (phase == 0) ? +BANK_ANGLE : -BANK_ANGLE;
            break;
        }

        case MODE_SEMI_MANUAL:{
            // ============================================================
            //  マニュアル制御: スティック → レートPID → サーボ
            // ============================================================
            Roll.tar = Roll.sbus;
            Pitch.tar= Pitch.sbus;
            Yaw.tar  = Yaw.sbus;
            // 何もないなら目標値が参照される、何か書き込んであればそっちの値が優先される
            Roll.update_RatePID();
            Pitch.update_RatePID();
            Yaw.update_RatePID();
            return; //ここで戻る   
        }

        case MODE_MANUAL:{
            Roll.cmd = Roll.sbus;
            Pitch.cmd= Pitch.sbus;
            Yaw.cmd =  Yaw.sbus;
            return;//これもここで戻す
        }
        default: break;
    }

    // --- 角度外ループ + レート内ループ (Control.hのupdate_RateAnglePID使用) ---
    //   角度PID: 100Hzで目標レートを算出 (counter%10)
    //   レートPID: 1000Hzでサーボ指令を算出
    //   内部.cmdの数値を勝手に、目標値からいじる
    Roll.update_RateAnglePID();
    Pitch.update_RateAnglePID();

    // --- 協調ラダー: バンク角に比例してラダーを打つ ---
    //   係数(0.05f)は実機でのスリップ量を見ながら調整
    Yaw.cmd = constrain(Roll.tar * 0.05f, -1.0f, 1.0f);
}

void writeServos() {
    Ele_von1.elevon(Roll.cmd, Pitch.cmd,
                    -1.0,1.0,-0.9,0.85);
    Ele_von2.elevon(Roll.cmd, Pitch.cmd,
                    -1.0,0.5,-0.9,1.0,true);
    //Ele.write(Pitch.cmd);
    Rud.write(Yaw.cmd);
}

