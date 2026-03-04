#include "MPU6050.h"
MPU6050 accelgyro;

int16_t ax, ay, az;//加速度　int16_tは2バイトの符号付き整数 
int16_t gx, gy, gz;//角速度　同上

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  accelgyro.initialize();//I2Cデバイスの初期化
  delay(300);
}

void loop()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az); Serial.print(" : ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.print(gz); Serial.println("");
  delay(100);//0.1秒停止。停止時間はお好みで調整。
}