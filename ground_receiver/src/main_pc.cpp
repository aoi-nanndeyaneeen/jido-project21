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
    
    // ★ここが超重要！Python向けの形式
    Serial.print("DATA,");
    Serial.print(Plane_Data.altitude, 2); Serial.print(",");
    Serial.print(Plane_Data.ax, 2); Serial.print(",");
    Serial.print(Plane_Data.ay, 2); Serial.print(",");
    Serial.print(Plane_Data.az, 2);
    Serial.println(); // 改行
  }
}