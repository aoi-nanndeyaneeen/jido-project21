// 出力フォーマット: ax_raw,ay_raw,az_raw,baro_alt\n
// baro_alt は float [m]
// Pythonが全ての処理を担当する

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>

constexpr float        LOOP_HZ  = 200.0f;
constexpr unsigned long PERIOD  = (unsigned long)(1e6f / LOOP_HZ);

MPU6050         mpu(0x68);
Adafruit_BMP280 bmp(&Wire1);   // SDA=17, SCL=16

int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
unsigned long t_prev = 0;
float baro_alt = 0.0f;
bool bmp_ok = false;

void setup() {
    Serial.begin(115200);

    // MPU6050 (Wire: SDA=18, SCL=19)
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    // スケール強制設定 ±2g / ±250dps
    Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission();

    // BMP280 (Wire1: SDA=17, SCL=16)
    Wire1.begin();
    Wire1.setClock(400000);
    bmp_ok = bmp.begin(0x76);
    if (!bmp_ok) bmp_ok = bmp.begin(0x77);
    if (bmp_ok) {
        // 高速設定: 約26Hz更新
        // FILTER_X16で内部ノイズを最大限除去 → σ≈0.1m が期待値
        // 応答の遅さ(約1.5秒)はカルマンの加速度予測で補う
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,   // 温度: 少し上げる
                        Adafruit_BMP280::SAMPLING_X16,  // 気圧: 最大オーバーサンプリング
                        Adafruit_BMP280::FILTER_X16,    // IIRフィルタ最大
                        Adafruit_BMP280::STANDBY_MS_500); // 500ms待機 → 約2Hz更新
    }
    delay(300);
    Serial.println("READY");
    t_prev = micros();
}

void loop() {
    unsigned long t_now = micros();
    if (t_now - t_prev < PERIOD) return;
    t_prev = t_now;

    mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

    // BMP280は約26Hz更新なので毎ループ読んでも問題なし
    if (bmp_ok) baro_alt = bmp.readAltitude(1013.25); // 海面気圧はPython側で補正

    // 50Hzで送信 (200Hz ÷ 4)
    static int cnt = 0;
    if (++cnt >= 4) {
        cnt = 0;
        Serial.printf("%d,%d,%d,%.3f\n", ax_r, ay_r, az_r, baro_alt);
    }
}