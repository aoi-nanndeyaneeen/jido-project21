#include <Arduino.h>
#include <DShot.h>

DShot motor0(&Serial1, DShotType::DShot600);
DShot motor1(&Serial2, DShotType::DShot600);
DShot motor2(&Serial6, DShotType::DShot600);
DShot motor3(&Serial7, DShotType::DShot600);

int currentThrottle = 0;
unsigned long loopCount = 0;

void sendAll(int throttle);

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== DShot検証開始 ===");
  Serial.println("Teensy起動確認OK");

  Serial.println("アーム開始: throttle=0 を4000回送信");
  for (size_t i = 0; i < 10000; i++) {
    motor0.sendThrottle(0, false);
    motor1.sendThrottle(0, false);
    motor2.sendThrottle(0, false);
    motor3.sendThrottle(0, false);
    if (i % 500 == 0) {
      Serial.print("アーム中... ");
      Serial.print(i);
      Serial.println("/4000");
    }
    delayMicroseconds(1000);
  }
  Serial.println("アーム完了");
  Serial.println("1:motor0のみ / 2:全300 / 3:全600 / 4:全1000 / s:停止");
}

void loop() {
  loopCount++;

  // 5秒ごとに生存確認
  if (loopCount % 5000 == 0) {
    Serial.print("loop alive: ");
    Serial.print(loopCount);
    Serial.print(" / currentThrottle=");
    Serial.println(currentThrottle);
  }

  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.print("コマンド受信: ");
    Serial.println(cmd);

    if (cmd == '1') {
      Serial.println("motor0のみ: throttle=300 x200回送信開始");
      for (int i = 0; i < 200; i++) {
        motor0.sendThrottle(300, false);
        motor1.sendThrottle(0, false);
        motor2.sendThrottle(0, false);
        motor3.sendThrottle(0, false);
        delayMicroseconds(1000);
      }
      Serial.println("motor0のみ: 送信完了");
    }
    else if (cmd == '2') {
      Serial.println("全モータ: throttle=300 送信開始");
      sendAll(300);
      Serial.println("全モータ: 送信完了");
    }
    else if (cmd == '3') {
      Serial.println("全モータ: throttle=600 送信開始");
      sendAll(600);
      Serial.println("全モータ: 送信完了");
    }
    else if (cmd == '4') {
      Serial.println("全モータ: throttle=1000 送信開始");
      sendAll(1000);
      Serial.println("全モータ: 送信完了");
    }
    else if (cmd == 's') {
      Serial.println("停止: throttle=0");
      sendAll(0);
      Serial.println("停止完了");
    }
    else {
      Serial.print("未知のコマンド: ");
      Serial.println((int)cmd);
    }
  }

  motor0.sendThrottle(currentThrottle, false);
  motor1.sendThrottle(currentThrottle, false);
  motor2.sendThrottle(currentThrottle, false);
  motor3.sendThrottle(currentThrottle, false);
  delayMicroseconds(1000);
}

void sendAll(int throttle) {
  currentThrottle = throttle;
  Serial.print("sendAll: throttle=");
  Serial.println(throttle);
  for (int i = 0; i < 200; i++) {
    motor0.sendThrottle(currentThrottle, false);
    motor1.sendThrottle(currentThrottle, false);
    motor2.sendThrottle(currentThrottle, false);
    motor3.sendThrottle(currentThrottle, false);
    if (i % 50 == 0) {
      Serial.print("  送信中: ");
      Serial.print(i);
      Serial.println("/200");
    }
    delayMicroseconds(1000);
  }
}