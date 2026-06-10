#include <Arduino.h>
#include "Bitcraze_PMW3901.h"
#include <SPI.h>

// XIAO RP2040のD7ピン（デジタルピン番号 7）をCSに指定
Bitcraze_PMW3901 flow(7);

int16_t deltaX, deltaY;

void setup() {
  Serial.begin(115200); // 高速通信用に115200に推奨

  // RP2040のSPIハードウェアを初期化
  SPI.begin();

  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(1) { delay(1); }
  }
}

void loop() {
  // 前回呼び出し時からの移動カウント数を取得
  flow.readMotionCount(&deltaX, &deltaY);

  // シリアルモニターに出力
  Serial.print("X: ");
  Serial.print(deltaX);
  Serial.print(", Y: ");
  Serial.println(deltaY);

  // 100msごとにサンプリング (10Hz)
  // ※フライト制御に組み込む際はdelayを無くしてループを高速で回します
  delay(100); 
}