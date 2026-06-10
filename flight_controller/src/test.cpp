#include <ComponentObject.h>
#include <PMW3901.h>
#include <SPI.h>

// CS（チップセレクト）ピンをD7に指定
// (XIAO RP2040のD7は、Arduinoのピン番号指定では 7 になります)
PMW3901 flow(7);

void setup() {
  Serial.begin(115200); // RP2040なのでシリアルは高速でOK

  // SPI通信の開始
  SPI.begin();

  // センサーの初期化
  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed!");
    while (1); 
  }
  Serial.println("Sensor initialized successfully.");
}

void loop() {
  int16_t deltaX, deltaY;

  // センサーデータの更新
  flow.readMotionCount(&deltaX, &deltaY);

  // 移動量（X, Y）を出力
  Serial.print("X: ");
  Serial.print(deltaX);
  Serial.print("\tY: ");
  Serial.println(deltaY);

  delay(50); // 応答性を上げるために50ms（20Hz周期）に変更
}
