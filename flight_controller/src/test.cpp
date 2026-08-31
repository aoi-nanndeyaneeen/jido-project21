#include <Arduino.h>
#include <Servo.h>

// ============================================================
// モータ回転テスト (Servo.h / 標準PWM)
//
// 【重要】プロペラは必ず外すこと
// 【重要】ESCファームウェアが Bluejay の場合は動作しない
//         (Bluejay は bidirectional DShot 必須)
//         → BLHeli_S / AM32 (Oneshot125 or PWM) が必要
// ============================================================

// ---------------- 調整パラメータ ----------------
const int PIN_ESC   = 0;      // ESC信号線を接続したピン

const int PWM_STOP  = 1000;   // 停止 (マルチロータESCのアーム値)
const int PWM_TEST  = 1150;   // テスト回転数 (まずは低めから)
const int PWM_MAX   = 2000;   // 最大 (このテストでは未使用)

const int ARM_TIME_MS   = 3000;  // アーミング待ち時間
const int SPIN_TIME_MS  = 2000;  // 回転させる時間
const int PAUSE_TIME_MS = 3000;  // 停止して待つ時間
// ------------------------------------------------

Servo esc;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== Motor Spin Test ===");
  Serial.print("ESC pin: ");
  Serial.println(PIN_ESC);

  esc.attach(PIN_ESC, PWM_STOP, PWM_MAX);

  // --- アーミング ---
  // 最低スロットルを一定時間送り続けてESCに認識させる
  Serial.println("Arming... (sending 1000us)");
  esc.writeMicroseconds(PWM_STOP);
  delay(ARM_TIME_MS);
  Serial.println("Armed. Starting test loop.");
}

void loop() {
  // --- 回す ---
  Serial.print("SPIN  -> ");
  Serial.print(PWM_TEST);
  Serial.println(" us");
  esc.writeMicroseconds(PWM_TEST);
  delay(SPIN_TIME_MS);

  // --- 止める ---
  Serial.print("STOP  -> ");
  Serial.print(PWM_STOP);
  Serial.println(" us");
  esc.writeMicroseconds(PWM_STOP);
  delay(PAUSE_TIME_MS);
}