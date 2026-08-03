// ============================================================
//  BodyFrame.h  -  IMU の出力を機体座標系 (FRD) に揃える
// ============================================================
//  IMU.h の getGyroX/Y/Z は FLU 系 (前・左・上) を返します。
//  クアッドの制御は FRD 系 (前・右・下) で書いたほうが素直なので、
//  ここで一度だけ変換します。符号は QuadConfig.h で調整できます。
//
//  旧 drone.cpp では
//      Roll.update_value(..., mpu.getPitch(), mpu.getAccY(), mpu.getGyroY());
//      Pitch.update_value(..., mpu.getRoll(), mpu.getAccX(), mpu.getGyroX());
//  のように Roll と Pitch で軸が入れ替わっていました。
//  意図的なのか事故なのか読み取れないのが一番の問題だったので、
//  「変換は1箇所だけ」という形にしています。
// ============================================================
#pragma once
#include <Arduino.h>
#include "quad/QuadConfig.h"
#include "sensor/IMU.h"

namespace Quad {

struct Attitude {
    // 角度 [deg]  (FRD: 右バンク +, 機首上げ +, 右旋回 +)
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;
    // 角速度 [deg/s]
    float roll_rate  = 0.0f;
    float pitch_rate = 0.0f;
    float yaw_rate   = 0.0f;
    // 加速度 [g]
    float acc_x = 0.0f;   // 前
    float acc_y = 0.0f;   // 右
    float acc_z = 0.0f;   // 下
};

inline Attitude readAttitude(IMU& imu) {
    // --- IMU の生の値 (FLU 系) ---
    float gx = imu.getGyroX();   // 前まわり
    float gy = imu.getGyroY();   // 左まわり
    float gz = imu.getGyroZ();   // 上まわり
    float ax = imu.getAccX();    // 前
    float ay = imu.getAccY();    // 左
    float az = imu.getAccZ();    // 上
    float ar = imu.getRoll();
    float ap = imu.getPitch();
    float ay_ang = imu.getYaw();

    // --- IMU が機体に対して90度回って付いている場合 ---
    if (SWAP_XY) {
        float t;
        t = gx; gx = gy; gy = t;
        t = ax; ax = ay; ay = t;
        t = ar; ar = ap; ap = t;
    }

    Attitude a;
    a.roll_rate  = GYRO_SIGN_ROLL  * gx;
    a.pitch_rate = GYRO_SIGN_PITCH * gy;
    a.yaw_rate   = GYRO_SIGN_YAW   * gz;

    a.roll  = ANG_SIGN_ROLL  * ar;
    a.pitch = ANG_SIGN_PITCH * ap;
    a.yaw   = ANG_SIGN_YAW   * ay_ang;

    a.acc_x =  ax;
    a.acc_y = -ay;   // FLU の左 → FRD の右
    a.acc_z = -az;   // FLU の上 → FRD の下

    return a;
}

} // namespace Quad
