#include "Bitcraze_PMW3901.h"

Bitcraze_PMW3901 flow(10);

char frame[35 * 35];

void setup() {
  Serial.begin(115200);

  while (!Serial);

  if (!flow.begin()) {
    Serial.println("Init Failed");
    while (1);
  }

  flow.enableFrameBuffer();

  Serial.println("READY");
}

void loop() {

  flow.readFrameBuffer(frame);

  // フレーム開始
  Serial.println("FRAME");

  for (int i = 0; i < 35 * 35; i++) {

    Serial.println((uint8_t)frame[i]);

  }

  // フレーム終了
  Serial.println("END");

  delay(50);      // 約20FPS
}