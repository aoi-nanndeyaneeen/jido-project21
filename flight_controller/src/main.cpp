#include <Arduino.h>

// 作成したモジュール（ヘッダ）群を読み込む
#include "Config.h"
#include "Telemetry.h"
#include "Control.h"
#include "Sensors.h"
#include "Receiver.h"
#include "Actuators.h"

// ==== グローバル変数の実体定義 ====
// Config.h で extern 宣言された変数の実体をここで作ります
unsigned long dt;
int counter;

// ==== 各種インスタンスの生成 ====
Axis_value Roll, Pitch, Yaw;
mpu_value MPU;
Sbus sbus(&Serial2);
BarometerSensor barometer(1013.25, 0.1);

PlaneData Plane_Data;     
GroundData Ground_Data;   
IM920SL_Generic<PlaneData, GroundData> im920(&Serial3);

RC_servo  Ail1(1,0.0,-1.0,1.0), Ail2(6,0.0,-1.0,1.0), Ele (2,0.0,-1.0,1.0),//ail:エルロン、ele:エレベーター、rud:ラダー、flp:フラップ
          Rud (4,0.0,-1.0,1.0), Flp1(8,0.0,-1,1), Flp2(9,0.0,-1,1);

RC_motor Thr_r(3,1.0),  Thr_l(5,1.0);

// ==== プロトタイプ宣言 ==== 
void update();
void print_PID(float r, float p, float y);
void print_MPU();
void print_sbus();

void setup() {
    MPU.begin();
    sbus.begin();

    Roll.setgains(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f);
    Pitch.setgains(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f);
    Yaw.setgains(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f);

    Ail1.begin(); Ail2.begin();
    Ele.begin();  Rud.begin();
    Flp1.begin(); Flp2.begin();
    
    Thr_r.begin(); Thr_l.begin();

    im920.begin();
    Serial.begin(115200);

    if (!barometer.begin()) {
        Serial.println("Barometer init failed!");
    }
}

void loop() {
    if (!frec()) return; // 周期制御

    update(); // データ更新

    // ミキシングとサーボ出力
    Ail1.write(Roll.des); 
    Ail2.write(Roll.des);
    Ele.write(Pitch.des); 
    Rud.write(Yaw.des);
    Flp1.flap(sbus.Ch_state(Aux1));
    Flp2.flap(sbus.Ch_state(Aux1));

    // モータのミキシングと出力
    if (1) {
        Thr_r.write(sbus.des[Ch::THR]);
        Thr_l.write(sbus.des[Ch::THR]);
    }
    else {
        Thr_r.write(0);
        Thr_l.write(0);
    }



// 通信とデバッグ表示
    if (counter % 100 == 0) {
            // 気圧センサ更新と通信パケット準備
        barometer.update();
        Plane_Data.ax = MPU.getRoll();//ax:ｘ軸の角度
        Plane_Data.ay = MPU.getPitch();//ay:yの
        Plane_Data.az = MPU.getYaw();//az:zの
        Plane_Data.gx = MPU.getGyroX();
        Plane_Data.gy = MPU.getGyroY();
        Plane_Data.gz = MPU.getGyroZ();
        Plane_Data.altitude = barometer.get_smoothed_altitude();

        Serial.print("\033[2J\033[H");
        im920.write(Plane_Data);
        print_PID(Roll.pid, Pitch.pid, Yaw.pid);
        print_MPU();
        print_sbus();
        Ground_Data.print();
        
        // ★ここを追加：Teensyの画面にも高度を出す
        Serial.print("★現在高度(Teensy側): ");
        Serial.print(Plane_Data.altitude);
        Serial.println(" m");
    }
}

// ==== 以下、メイン処理を支える関数 ====
void update() {
    MPU.update();
    sbus.update();

    Roll.update_value(sbus.des[Ch::ROLL], MPU.getRoll(), MPU.getAccX(), MPU.getGyroX());
    Pitch.update_value(sbus.des[Ch::PITCH], MPU.getPitch(), MPU.getAccY(), MPU.getGyroY());
    Yaw.update_value(sbus.des[Ch::YAW], MPU.getYaw(), MPU.getAccZ(), MPU.getGyroZ());

    Roll.update_RatePID(Roll.des);
    Pitch.update_RatePID(Pitch.des);
    Yaw.update_RatePID(Yaw.des);

    im920.read(Ground_Data);
}


void print_PID(float r, float p, float y) {
    Serial.print("|PIDRoll= "); Serial.print(r);
    Serial.print("|PIDPitch= "); Serial.print(p);
    Serial.print("|PIDYaw= "); Serial.print(y);
    Serial.print("\n");
}

void print_MPU() {
    Serial.print(Roll.ang, 2); Serial.print(",");
    Serial.print(Pitch.ang, 2); Serial.print(",");
    Serial.print(Yaw.ang, 2); Serial.print(",");
    Serial.print(Roll.gyr, 2); Serial.print(",");
    Serial.print(Pitch.gyr, 2); Serial.print(",");
    Serial.print(Yaw.gyr, 2); Serial.print(",");
    Serial.print("\n");
}

void print_sbus() {
    Serial.print("|ch1(ail1) = "); Serial.print(Roll.des, 2);
    Serial.print(" |ch2(ele) = "); Serial.print(Pitch.des, 2);
    Serial.print(" |ch3(thr) = "); Serial.print(sbus.des[Ch::THR], 2);
    Serial.print(" |ch4(rud) = "); Serial.print(Yaw.des, 2);
    Serial.print(" |ch5(swD) = "); Serial.print(sbus.des[4], 2);
    Serial.print(" |ch6(swB) = "); Serial.print(sbus.des[5], 2);
    Serial.print(" |ch7(swF) = "); Serial.print(sbus.des[Ch::THR_CUT], 2);
    Serial.print(" |ch8(swE) = "); Serial.print(sbus.des[7], 2);
    Serial.print(" |ch9(swF) = "); Serial.print(sbus.des[8], 2);
    Serial.print("\n");
}