#include <Arduino.h>
#include "Config.h"
#include "Telemetry.h"
#include "Serial_monitor.h"

unsigned long dt;
int counter;

PlaneData Plane_Data;
GroundData Ground_Data;
IM920SL_Generic<GroundData, PlaneData> im920(&Serial1);
Serial_monitor serial(GROUND_DATA_NUM);

void setup() {
  im920.begin();
  serial.begin(115200);
}

void loop() {
  if (!frec())  return; 

  serial.read();
  serial.update(&Ground_Data);
  im920.read(Plane_Data);

  // 500回に1回（2Hz）で出力
  if (counter % 500 == 0) {
    im920.write(Ground_Data);
    
    // ==========================================
    // ★ Teleplot用フォーマット (>変数名:値)
    // ==========================================
    
    // 1. 加速度の生データ
    Serial.print(">ax:"); Serial.println(Plane_Data.ax, 2);
    Serial.print(">ay:"); Serial.println(Plane_Data.ay, 2);
    Serial.print(">az:"); Serial.println(Plane_Data.az, 2);
    
    // 2. 機体の姿勢（Teensy側で gx, gy に Roll, Pitch を入れています）
    Serial.print(">Roll:"); Serial.println(Plane_Data.gx, 2);
    Serial.print(">Pitch:"); Serial.println(Plane_Data.gy, 2);
    
    // ※気圧センサ未接続のため、Plane_Data.altitude は出力しません
  }
}