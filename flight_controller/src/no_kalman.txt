// 出力フォーマット: ax_raw,ay_raw,az_raw,baro_alt\n
// baro_alt は float [m]

#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>

// ============================================================
//  ★ここだけ変える: 2 / 10 / 23 / 50 / 75
// ============================================================
#define BMP_HZ 50

// ============================================================
//  BMP_HZ に応じて自動設定
// ============================================================
#if BMP_HZ == 2
  // 安定性最重視。FILTER_X16で約22サンプル分平滑化
  #define BMP_STANDBY_VAL  Adafruit_BMP280::STANDBY_MS_500
  #define BMP_FILTER_VAL   Adafruit_BMP280::FILTER_X16
  #define BMP_SAMP_P_VAL   Adafruit_BMP280::SAMPLING_X16
  #define BMP_SAMP_T_VAL   Adafruit_BMP280::SAMPLING_X2
  #define BMP_READ_DIV_VAL 100   // 200Hz / 100 = 2Hz

#elif BMP_HZ == 10
  // バランス型。ただしFILTER_X16が動きをほぼ消してしまう
  #define BMP_STANDBY_VAL  Adafruit_BMP280::STANDBY_MS_63
  #define BMP_FILTER_VAL   Adafruit_BMP280::FILTER_X16
  #define BMP_SAMP_P_VAL   Adafruit_BMP280::SAMPLING_X16
  #define BMP_SAMP_T_VAL   Adafruit_BMP280::SAMPLING_X2
  #define BMP_READ_DIV_VAL 20    // 200Hz / 20 = 10Hz

#elif BMP_HZ == 23
  // 即応性重視。FILTER_X4で5サンプル分平滑化。計測時間43ms
  #define BMP_STANDBY_VAL  Adafruit_BMP280::STANDBY_MS_1
  #define BMP_FILTER_VAL   Adafruit_BMP280::FILTER_X4
  #define BMP_SAMP_P_VAL   Adafruit_BMP280::SAMPLING_X16
  #define BMP_SAMP_T_VAL   Adafruit_BMP280::SAMPLING_X2
  #define BMP_READ_DIV_VAL 9     // 200Hz / 9 ≈ 22Hz

#elif BMP_HZ == 50
  // 高速。SAMPLING_X4で計測時間15ms→最大65Hz。FILTER_X2で3サンプル平滑化
  #define BMP_STANDBY_VAL  Adafruit_BMP280::STANDBY_MS_1
  #define BMP_FILTER_VAL   Adafruit_BMP280::FILTER_X2
  #define BMP_SAMP_P_VAL   Adafruit_BMP280::SAMPLING_X4
  #define BMP_SAMP_T_VAL   Adafruit_BMP280::SAMPLING_X1
  #define BMP_READ_DIV_VAL 4     // 200Hz / 4 = 50Hz

#elif BMP_HZ == 75
  // 最高速。SAMPLING_X2で計測時間8ms→最大115Hz。FILTER_OFFでフィルタなし
  #define BMP_STANDBY_VAL  Adafruit_BMP280::STANDBY_MS_1
  #define BMP_FILTER_VAL   Adafruit_BMP280::FILTER_OFF
  #define BMP_SAMP_P_VAL   Adafruit_BMP280::SAMPLING_X2
  #define BMP_SAMP_T_VAL   Adafruit_BMP280::SAMPLING_X1
  #define BMP_READ_DIV_VAL 3     // 200Hz / 3 ≈ 67Hz

#else
  #error "BMP_HZ must be 2, 10, 23, 50, or 75"
#endif

constexpr auto BMP_STANDBY  = BMP_STANDBY_VAL;
constexpr auto BMP_FILTER   = BMP_FILTER_VAL;
constexpr auto BMP_SAMP_P   = BMP_SAMP_P_VAL;
constexpr auto BMP_SAMP_T   = BMP_SAMP_T_VAL;
constexpr int  BMP_READ_DIV = BMP_READ_DIV_VAL;

// ============================================================
//  その他設定（変更不要）
// ============================================================
MPU6050 mpu(0x68);
Adafruit_BMP280 bmp(&Wire1); // SDA=17, SCL=16

int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
unsigned long t_prev = 0;
float baro_alt = 0.0f;
bool bmp_ok = false;
constexpr int PERIOD = 5000; // 200Hz ループ（5000μs）

void setup() {
  Serial.begin(115200);

  // MPU6050 (Wire: SDA=18, SCL=19)
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission();

  // BMP280 (Wire1: SDA=17, SCL=16)
  Wire1.begin();
  Wire1.setClock(400000);
  bmp_ok = bmp.begin(0x76);
  if (!bmp_ok) bmp_ok = bmp.begin(0x77);
  if (bmp_ok) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    BMP_SAMP_T, BMP_SAMP_P, BMP_FILTER, BMP_STANDBY);
  }

  delay(300);
  Serial.printf("READY BMP_HZ=%d\n", BMP_HZ);
  t_prev = micros();
}

void loop() {
  unsigned long t_now = micros();
  if (t_now - t_prev < PERIOD) return;
  t_prev = t_now;

  static int loop_cnt = 0;
  loop_cnt++;

  mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

  if (bmp_ok && (loop_cnt % BMP_READ_DIV == 0))
    baro_alt = bmp.readAltitude(1013.25);

  // 50Hz で送信 (200Hz / 4)
  static int tx_cnt = 0;
  if (++tx_cnt >= 4) {
    tx_cnt = 0;
    Serial.printf("%d,%d,%d,%.3f\n", ax_r, ay_r, az_r, baro_alt);
  }
}