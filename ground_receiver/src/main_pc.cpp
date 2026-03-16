#include <Arduino.h>
#include "Config.h"
#include "Telemetry.h"
#include "Serial_monitor.h"
#include "Serial_com.h"

unsigned long dt;
int counter;

PlaneData Plane_Data;
GroundData Ground_Data;
IM920SL_Generic<GroundData, PlaneData> im920(&Serial1);
Serial_monitor serial(GROUND_DATA_NUM);

void setup() {
  im920.begin();
  Serial.begin(115200);
  while(!Serial && millis() < 3000); // Wait for Serial to initialize
  Serial.println("\n\n[NEW FIRMWARE V2] STARTING...");
  serial.begin(115200);
}

void loop() {
  if (!frec())  return; 

  serial.read();
  serial.update(&Ground_Data);
  im920.read(Plane_Data);

  // 100回に1回 (10Hz) で画面を更新して出力 (制御周期1000Hz想定)
  if (counter % 100 == 0) {
    im920.write(Ground_Data);
    
    // 画面クリア（シリアルモニタの最上部に固定）
    Serial.print("\033[2J\033[H");
    Serial.println("=== GROUND STATION RECEIVER ===");
    
    // 1. 基本タイミング情報
    print_timing(dt);

    // 2. 機体から受信した姿勢データ
    // ※Teensy側で Plane_Data.gx, gy に Roll, Pitch を入れている仕様に合わせる
    print_MPU(Plane_Data.gx, Plane_Data.gy, Plane_Data.gz, 0.0, 0.0, 0.0);
    print_ACC(Plane_Data.ax, Plane_Data.ay, Plane_Data.az);
    Serial.printf("Alt: %+7.2f m\n", Plane_Data.altitude);

    Serial.println("-------------------------------------------");

    // 3. 地上局側の設定・送信データ
    Ground_Data.print();

    // 4. Teleplot用（必要なら残す）
    // Serial.print(">Roll:"); Serial.println(Plane_Data.gx, 2);
    // Serial.print(">Pitch:"); Serial.println(Plane_Data.gy, 2);
  }
}