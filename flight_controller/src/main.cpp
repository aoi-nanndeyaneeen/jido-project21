// ============================================================
// main.cpp  -  自律飛行対応版 (軽量化・カルマンフィルタ削除版)
// ============================================================

#include <Arduino.h>
#include <cctype>
#include <cmath>
#include <math.h>
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
// EZ2Sensor       ez2(Config::sensor::EZ2_PW_PIN, Config::sensor::EZ2_ALPHA); // 停止中 (搭載無し)

Sbus sbus(Config::serial::sbus);
IM920SL_Generic<PlaneData, GroundData> im920(Config::serial::im920);

PlaneData  Plane_Data;
GroundData Ground_Data;

Flight_mode Mode;

        //ピン・サブトリム(-1~1)・エンドポイント・エンドポイント・リバース(trueが反転する)
RC_servo Ail1(1, 0.0, -1.0, 1.0, false), // 1番ピンに戻しました
         Ail2(6, 0.0, -1.0, 1.0, false),
         Ele (2, 0.5, -1.0, 1.0, false),
         Rud (10, 0.0, -1.0, 1.0, false), // 4番(EZ2)と被るので10番へ
         Flp1(11, 0.0, -1.0, 1.0, false), // 8番(SBUS TX)と被るので11番へ
         Flp2(9, 0.0, -1.0, 1.0, false);
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
// ============================================================
//  § 組み合わせテスト用フラグ (ここを true/false で切り替えてください)
// ============================================================
bool USE_MPU   = true;  // 加速度センサー (MPU6050)
bool USE_BARO  = true;  // 気圧センサー (BMP280)
bool USE_SBUS  = true;  // 受信機 (SBUS)
bool USE_IM920 = true;  // 無線モジュール (IM920)
bool USE_SERVO = true;  // サーボ・アンプ出力 (Servo/ESC)

static bool is_telemetry_active = true; // テレメトリ送信フラグ

void setup() {
    Serial.begin(115200);
    uint32_t start_ms = millis();
    while(!Serial && (millis() - start_ms < 2000));
    Serial.println("\n\n--- TEENSY SYSTEM BOOT (High-Power Triage) ---");
    Serial.printf("sizeof(PlaneData)=%d  sizeof(GroundData)=%d\n",
        sizeof(PlaneData), sizeof(GroundData));

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
        Ail1.begin(); Ail2.begin();
        Ele.begin();  Rud.begin();
        Flp1.begin(); Flp2.begin();
        Thr_r.begin(); Thr_l.begin();
    }

    if (USE_IM920) {
        Serial.println("Init IM920...");
        im920.begin();
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
            USE_MPU?"ON":"OFF", USE_BARO?"ON":"OFF", USE_SBUS?"ON":"OFF");
    }

    if (T::freq<T::MAIN_Hz>(T::Main_dt)) { // 周期制御 (1000Hz)
        // 1) センサ・受信機・通信の更新
        if (USE_MPU || USE_SBUS) {
            updateSensorsAndComms();
        }

        // モード切替時にPIリセット
        if (Mode.change()) {
            Mode.modeStartMs = millis();
            Roll.pid_reset();   Pitch.pid_reset();  Yaw.pid_reset();
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
            if (sbus.isSafe()) {
                Thr_r.write(sbus.des[Ch::THR]);
                Thr_l.write(sbus.des[Ch::THR]);
            } else {
                Thr_r.write(0);
                Thr_l.write(0);
            }
        }

        // 5) フラップ (ONの場合のみ)
        if (USE_SERVO) {
            Flp1.flap(sbus.Ch_state(Aux1));
            Flp2.flap(sbus.Ch_state(Aux1));
        }

        // 6) テレメトリ・デバッグ (10Hz)
        static int dbg_cnt = 0;
        if (++dbg_cnt >= 100) {
            dbg_cnt = 0;
            if (USE_BARO) barometer.update();

            float fused_alt = (USE_BARO) ? barometer.get_smoothed_altitude() : 0.0f;
            
            // 🌟 姿勢データを送る (is_telemetry_active が true の時だけ)
            if (USE_IM920 && is_telemetry_active) {
                PlaneData att_packet;
                att_packet.packet_type = 0; // 姿勢データとして送信
                att_packet.data[0]       = mpu.getAccX(); 
                att_packet.data[1]       = mpu.getAccY(); 
                att_packet.data[2]       = mpu.getAccZ();
                att_packet.data[3]       = Roll.ang; 
                att_packet.data[4]       = Pitch.ang; 
                att_packet.data[5]       = Yaw.ang;
                att_packet.data[6]       = fused_alt;
                att_packet.data[7]       = 0.0f; // unused
                im920.write(att_packet);
            }

             Serial.print("\033[2J\033[H"); // ターミナルクリア
             Serial.printf("### Triage: MPU=%s BARO=%s SBUS=%s IM920=%s SERVO=%s TEL=%s ###\n", 
                USE_MPU?"ON":"OFF", USE_BARO?"ON":"OFF", USE_SBUS?"ON":"OFF", USE_IM920?"ON":"OFF", USE_SERVO?"ON":"OFF", is_telemetry_active?"ON":"OFF");
             Serial.printf("[TIME: %lu ms] PlaneData=%d GroundData=%d\n", millis(), (int)sizeof(PlaneData), (int)sizeof(GroundData));
            print_flightmode(Mode.get_mode(), BANK_ANGLE, TURN_MS);
             if (USE_MPU) print_MPU(Roll.ang, Pitch.ang, Yaw.ang, Roll.gyr, Pitch.gyr, Yaw.gyr);
             if (USE_SBUS) print_sbus(sbus.des[Ch::ROLL], sbus.des[Ch::PITCH], sbus.des[Ch::THR], sbus.des[Ch::YAW], sbus.des[Ch::Aux1], sbus.des[Ch::Aux2], sbus.des[Ch::Aux3]);
             
             if (USE_BARO) Serial.printf("Alt: %+7.2f m\n", fused_alt);
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
    Mode.update(sbus.Ch_state(Ch::Aux2), sbus.Ch_state(Ch::Aux3));

    Roll.update_value(sbus.des[Ch::ROLL],    -mpu.getRoll(),  mpu.getAccX(), mpu.getGyroX());
    Pitch.update_value(sbus.des[Ch::PITCH], -mpu.getPitch(),  mpu.getAccY(), mpu.getGyroY());
    Yaw.update_value(sbus.des[Ch::YAW],        mpu.getYaw(),  mpu.getAccZ(), mpu.getGyroZ());

    // --- シリアルコマンド (PCモニタからのRキー等) ---
    if (Serial.available()) {
        char c = toupper(Serial.peek());
        if (c == 'R') {
            Serial.read(); // 'R' を消費
            mpu.recalibrate();
            barometer.reset();
            Config::Timing::resetTiming();
            Roll.pid_reset(); Pitch.pid_reset(); Yaw.pid_reset();
            Serial.println("INFO: System-wide Reset Complete.");
        } 
        else if (c == 'P') {
            Serial.read(); // 'P' を消費
            handlePIDTuning(Roll, Pitch, Yaw);
        }
        else {
            Serial.read(); // その他の文字は捨てる
        }
    }

    // 地上局からパラメータ受信
    if (im920.read(Ground_Data)) {

        // リモートリセット
        if (Ground_Data.reset_cmd == 1) {
            mpu.recalibrate();
            barometer.reset();
            Config::Timing::resetTiming();
            Roll.pid_reset(); Pitch.pid_reset(); Yaw.pid_reset();
            Serial.println("INFO: Remote Reset Complete.");
            Ground_Data.reset_cmd = 0;
        }

        // BANK_ANGLE / TURN_MS 更新
        if (Ground_Data.roll  != 0.0f) BANK_ANGLE = fabsf(Ground_Data.roll);
        if (Ground_Data.pitch != 0.0f) TURN_MS    = (unsigned long)(Ground_Data.pitch * 1000.0f);

        if (Ground_Data.param_sel == 10) {
            // テレメトリ停止 ＆ 現在のゲインを1回だけ送り返す
            is_telemetry_active = false;
            PlaneData gain_packet;
            gain_packet.packet_type = 1; // ゲインデータとして送信
            gain_packet.data[0]       = Roll.c_rate.get_kp();
            gain_packet.data[1]       = Roll.c_rate.get_ki();
            gain_packet.data[2]       = Roll.c_rate.get_kd();
            gain_packet.data[3]       = Pitch.c_rate.get_kp();
            gain_packet.data[4]       = Pitch.c_rate.get_ki();
            gain_packet.data[5]       = Pitch.c_rate.get_kd();
            gain_packet.data[6]       = Roll.c_ang.get_kp();
            gain_packet.data[7]       = Pitch.c_ang.get_kp();
            im920.write(gain_packet);
            Serial.println("INFO: Sent current gains to Ground Station.");
            Ground_Data.param_sel = 0;
        } 
        else if (Ground_Data.param_sel == 11) {
            // テレメトリ再開
            is_telemetry_active = true;
            Serial.println("INFO: Telemetry Resumed.");
            Ground_Data.param_sel = 0;
        }
        else if (Ground_Data.param_sel != 0) {
            float p = Ground_Data.p_adj;
            float i = Ground_Data.i_adj;
            float d = Ground_Data.d_adj;
            switch (Ground_Data.param_sel) {
                case 1: Roll.set_rate_gains(p, i, d);
                        Serial.printf("INFO: Roll  Rate  P=%.4f I=%.4f D=%.4f\n", p, i, d); break;
                case 2: Pitch.set_rate_gains(p, i, d);
                        Serial.printf("INFO: Pitch Rate  P=%.4f I=%.4f D=%.4f\n", p, i, d); break;
                case 3: Yaw.set_rate_gains(p, i, d);
                        Serial.printf("INFO: Yaw   Rate  P=%.4f I=%.4f D=%.4f\n", p, i, d); break;
                case 4: Roll.set_angle_gains(p, i, d);
                        Serial.printf("INFO: Roll  Angle P=%.4f I=%.4f D=%.4f\n", p, i, d); break;
                case 5: Pitch.set_angle_gains(p, i, d);
                        Serial.printf("INFO: Pitch Angle P=%.4f I=%.4f D=%.4f\n", p, i, d); break;
                default: break;
            }
            Ground_Data.param_sel = 0; // 処理済みクリア
        }
    }
}

// ============================================================
//  自律制御
// ============================================================
void autonomousControl() {
    // --- 🔴 電波がない場合の「地上PIDセッティングモード」 ---(リポが断線していてプロポの電源の5Vをサーボ用に使っていた時用)
    if (!sbus.isSafe()) {
        Roll.tar = 0.0f;   // 目標ロール角 0度（常に水平を維持）
        Pitch.tar = 0.0f;  // 目標ピッチ角 0度（常に水平を維持）
        Yaw.tar = 0.0f;    // ラダー目標 0

        // 角度＆レートPIDを計算してコマンドを出力
        Roll.update_RateAnglePID();
        Pitch.update_RateAnglePID();
        Yaw.cmd = 0.0f;    // 手持ちテスト中、ラダーは暴れないよう0固定

        return; // 通常のフライトモード判定をスキップしてここで終了
    }

    // --- 🟢 電波がある場合（ここから下は元のコードそのまま） ---
    switch (Mode.get_mode()) {
        case MODE_LEVEL_TURN:
            // ★ 左旋回にしたい場合は -BANK_ANGLE にする
            Roll.tar = +BANK_ANGLE;
            break;

        case MODE_FIGURE_8: {
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
            Roll.update_RatePID();
            Pitch.update_RatePID();
            Yaw.update_RatePID();
            return; //ここで戻る   
        }

        case MODE_MANUAL:{
            // SBUSが安全(通信中)ならプロポの値を、途絶しているなら0.0(ニュートラル)にする
            if (sbus.isSafe()) {
                Roll.cmd = Roll.sbus;
                Pitch.cmd= Pitch.sbus;
                Yaw.cmd =  Yaw.sbus;
            } else {
                Roll.cmd = 0.0f;
                Pitch.cmd= 0.0f;
                Yaw.cmd =  0.0f;
            }
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

    // --- 協調ラダー: バンク方向にRUDDER_COORD量のラダーを打つ ---
    // Config.h の RUDDER_COORD で量を調整 (1.0=全開, 0.5=半分, 0=なし)
    if      (Roll.tar >  0.1f) Yaw.cmd =  RUDDER_COORD;
    else if (Roll.tar < -0.1f) Yaw.cmd = -RUDDER_COORD;
    else                       Yaw.cmd =  0.0f;
}

void writeServos() {
    Ail1.write(Roll.cmd);
    Ail2.write(Roll.cmd);
    Ele.write(Pitch.cmd);
    Rud.write(Yaw.cmd);
}
