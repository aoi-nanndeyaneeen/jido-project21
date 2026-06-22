#include <Arduino.h>
#include <Servo.h>

// === PWM出力ピン定義 ===
const int MOTOR_PINS[4] = {9, 24, 10, 11};

// Servoオブジェクトの配列を作成
Servo motors[4];

// === サーボPWMパラメータ ===
const int PWM_MIN_US = 1000;
const int PWM_MAX_US = 2000;

// DShot スロットル値(0〜2047) → PWMパルス幅(μs) に変換
int throttleToPulseUs(int throttle) {
  throttle = constrain(throttle, 0, 2047);
  return map(throttle, 0, 2047, PWM_MIN_US, PWM_MAX_US);
}

void sendAll(int throttle);

int currentThrottle = 0;
unsigned long loopCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);  // シリアル通信の安定待ち

  Serial.println("=== Servo.h (PWM) 検証開始 ===");
  Serial.println("Teensy起動確認OK");
  Serial.print("PWM範囲: ");
  Serial.print(PWM_MIN_US);
  Serial.print(" 〜 ");
  Serial.print(PWM_MAX_US);
  Serial.println(" μs");

  // 【重要】アタッチする（通信を開始する）前に、出力初期値(1000μs)をセットしておく
  Serial.println("初期信号(1000μs)を設定中...");
  for (int i = 0; i < 4; i++) {
    motors[i].writeMicroseconds(1000);
    // ピンをサーボライブラリに紐付け（同時にパルス出力がスタートする）
    motors[i].attach(MOTOR_PINS[i], PWM_MIN_US, PWM_MAX_US);
  }

  // アーム: 1000μs
  // を5秒間送り続ける（Servo.hが裏で自動送信してくれるのでdelayで待つだけでOK）
  Serial.println("アーム開始: 1000μs を5秒キープ中...");
  unsigned long armStart = millis();
  while (millis() - armStart < 5000) {
    unsigned long elapsed = millis() - armStart;
    if (elapsed % 500 < 10) {

      Serial.print("アーム中... ");
      Serial.print(elapsed / 1000);
      Serial.println(" / 5 秒");
      delay(10);
    }
  }
  Serial.println("アーム完了");
  Serial.println("1:motor0のみ(300) / 2:全300 / 3:全600 / 4:全1000 / s:停止");
}

void loop() {
  loopCount++;

  if (loopCount % 50000 == 0) {  // loop内のdelayが減ったのでカウント頻度を調整
    Serial.print("loop alive: ");
    Serial.print(loopCount);
    Serial.print(" / currentThrottle=");
    Serial.print(currentThrottle);
    Serial.print(" (");
    Serial.print(throttleToPulseUs(currentThrottle));
    Serial.println(" μs)");

  }

  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.print("コマンド受信: ");
    Serial.println(cmd);

    if (cmd == '1') {
      Serial.println("motor0のみ: throttle=300 に変更");
      motors[0].writeMicroseconds(throttleToPulseUs(300));
      motors[1].writeMicroseconds(PWM_MIN_US);
      motors[2].writeMicroseconds(PWM_MIN_US);
      motors[3].writeMicroseconds(PWM_MIN_US);
      currentThrottle = 0;
    } else if (cmd == '2') {
      sendAll(50);
    } else if (cmd == '3') {
      sendAll(600);
    } else if (cmd == '4') {
      sendAll(1000);
    } else if (cmd == 's') {
      Serial.println("停止: 1000μs");
      sendAll(0);
    } else {
      Serial.print("未知のコマンド: ");
      Serial.println((int)cmd);
    }
  }

  // メインループで常時値を更新
  // (Servo.hがバックグラウンドでパルスを維持するため、毎回のループ処理は不要ですが追従性を出すため残しています)
  int pulseUs = throttleToPulseUs(currentThrottle);
  for (int i = 0; i < 4; i++) {
    motors[i].writeMicroseconds(pulseUs);
  }

  delay(
      2);  // ループが速すぎてシリアルバッファが詰まるのを防ぐためのわずかなウェイト
}

void sendAll(int throttle) {
  currentThrottle = throttle;
  int pulseUs = throttleToPulseUs(throttle);
  Serial.print("sendAll: throttle=");
  Serial.print(throttle);
  Serial.print(" → ");
  Serial.print(pulseUs);
  Serial.println(" μs");

  for (int m = 0; m < 4; m++) {
    motors[m].writeMicroseconds(pulseUs);
  }
}