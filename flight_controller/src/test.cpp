#include <Arduino.h>
 
// === PWM出力ピン定義 ===
const int MOTOR_PINS[4] = {8, 9, 10, 11};
 
// === サーボPWMパラメータ ===
const int PWM_MIN_US  = 1000;
const int PWM_MAX_US  = 2000;
const int PWM_FREQ_HZ = 400;  // BLHeli_S/32 は250〜500Hz推奨
 
// DShot スロットル値(0〜2047) → PWMパルス幅(μs) に変換
int throttleToPulseUs(int throttle) {
  throttle = constrain(throttle, 0, 2047);
  return map(throttle, 0, 2047, PWM_MIN_US, PWM_MAX_US);
}
 
void writePulseUs(int motorIndex, int pulseUs) {
  // Teensy 4.x: 16bit分解能でduty計算
  // duty = pulseUs / (1000000 / PWM_FREQ_HZ) * 65535
  float period_us = 1000000.0f / PWM_FREQ_HZ;
  int duty = (int)((float)pulseUs / period_us * 65535.0f);
  duty = constrain(duty, 0, 65535);
  analogWrite(MOTOR_PINS[motorIndex], duty);
}
 
void sendAll(int throttle);
 
int currentThrottle = 0;
unsigned long loopCount = 0;
 
void setup() {
  Serial.begin(115200);
 
  // PWMピン初期化 (Servo.hは使わない)
  analogWriteResolution(16);
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    analogWriteFrequency(MOTOR_PINS[i], PWM_FREQ_HZ);
    analogWrite(MOTOR_PINS[i], 0);  // 念のため0出力
  }
 
  delay(3000);
  Serial.println("=== PWM(analogWrite 400Hz)検証開始 ===");
  Serial.println("Teensy起動確認OK");
  Serial.print("PWM周波数: "); Serial.print(PWM_FREQ_HZ); Serial.println(" Hz");
  Serial.print("PWM範囲: "); Serial.print(PWM_MIN_US); Serial.print(" 〜 "); Serial.print(PWM_MAX_US); Serial.println(" μs");
 
  // アーム: 1000μs を5秒間送り続ける
  Serial.println("アーム開始: 1000μs を5秒送信中...");
  unsigned long armStart = millis();
  while (millis() - armStart < 5000) {
    for (int i = 0; i < 4; i++) {
      writePulseUs(i, PWM_MIN_US);
    }
    unsigned long elapsed = millis() - armStart;
    if (elapsed % 500 < 10) {
      Serial.print("アーム中... "); Serial.print(elapsed / 1000); Serial.println(" / 5 秒");
      delay(10);
    }
  }
  Serial.println("アーム完了");
  Serial.println("1:motor0のみ(300) / 2:全300 / 3:全600 / 4:全1000 / s:停止");
}
 
void loop() {
  loopCount++;
 
  if (loopCount % 5000 == 0) {
    Serial.print("loop alive: "); Serial.print(loopCount);
    Serial.print(" / currentThrottle="); Serial.print(currentThrottle);
    Serial.print(" ("); Serial.print(throttleToPulseUs(currentThrottle)); Serial.println(" μs)");
  }
 
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.print("コマンド受信: "); Serial.println(cmd);
 
    if (cmd == '1') {
      Serial.println("motor0のみ: throttle=300 x200回送信開始");
      for (int i = 0; i < 200; i++) {
        writePulseUs(0, throttleToPulseUs(300));
        writePulseUs(1, PWM_MIN_US);
        writePulseUs(2, PWM_MIN_US);
        writePulseUs(3, PWM_MIN_US);
        delayMicroseconds(500);
      }
      currentThrottle = 0;
      Serial.println("motor0のみ: 送信完了");
    }
    else if (cmd == '2') { sendAll(300); }
    else if (cmd == '3') { sendAll(600); }
    else if (cmd == '4') { sendAll(1000); }
    else if (cmd == 's') {
      Serial.println("停止: 1000μs");
      sendAll(0);
    }
    else {
      Serial.print("未知のコマンド: "); Serial.println((int)cmd);
    }
  }
 
  // メインループで常時送信
  int pulseUs = throttleToPulseUs(currentThrottle);
  for (int i = 0; i < 4; i++) {
    writePulseUs(i, pulseUs);
  }
  delayMicroseconds(500);
}
 
void sendAll(int throttle) {
  currentThrottle = throttle;
  int pulseUs = throttleToPulseUs(throttle);
  Serial.print("sendAll: throttle="); Serial.print(throttle);
  Serial.print(" → "); Serial.print(pulseUs); Serial.println(" μs");
  for (int i = 0; i < 200; i++) {
    for (int m = 0; m < 4; m++) {
      writePulseUs(m, pulseUs);
    }
    if (i % 50 == 0) {
      Serial.print("  送信中: "); Serial.print(i); Serial.println("/200");
    }
    delayMicroseconds(500);
  }
}