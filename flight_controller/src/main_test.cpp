// 出力フォーマット: ax_raw,ay_raw,az_raw,baro_alt\n
// baro_alt は float [m]
// Pythonが全ての処理を担当する

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>

// BMP280 チューニング
// STANDBY_MS_63 + SAMPLING_X16 で約10Hz更新 (安定性重視)
// STANDBY_MS_1  + SAMPLING_X16 で約23Hz更新 (即応性重視)
constexpr auto BMP_STANDBY = Adafruit_BMP280::STANDBY_MS_63;
constexpr auto BMP_FILTER  = Adafruit_BMP280::FILTER_X4;
constexpr auto BMP_SAMP_P  = Adafruit_BMP280::SAMPLING_X16;
constexpr auto BMP_SAMP_T  = Adafruit_BMP280::SAMPLING_X2;

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
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        BMP_SAMP_T,
                        BMP_SAMP_P,
                        BMP_FILTER,
                        BMP_STANDBY);
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