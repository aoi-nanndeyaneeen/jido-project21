// Teensyは生値を送るだけ。処理は全部Python側。
// 出力フォーマット: ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw\n
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

constexpr float        LOOP_HZ = 200.0f;
constexpr unsigned long PERIOD = (unsigned long)(1e6f / LOOP_HZ);

MPU6050 mpu(0x68);
int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
unsigned long t_prev = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    delay(300);
    mpu.initialize();
    // スケール設定だけここで行う（レジスタ直書き）
    Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); // ±2g
    Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(); // ±250dps
    delay(100);
    Serial.println("READY"); // Python側が起動完了を検知するために送る
}

void loop() {
    unsigned long t_now = micros();
    if (t_now - t_prev < PERIOD) return;
    t_prev = t_now;

    mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);
    // Python側でシリアルを読むのが十分速ければ全部送れるが、
    // 200Hzは少し余裕がないため50Hzで送る（グラフ表示には十分）
    static int cnt = 0;
    if (++cnt >= 4) {
        cnt = 0;
        Serial.printf("%d,%d,%d,%d,%d,%d\n",
                      ax_r, ay_r, az_r, gx_r, gy_r, gz_r);
    }
}