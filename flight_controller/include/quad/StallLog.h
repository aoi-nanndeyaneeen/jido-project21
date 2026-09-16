// ============================================================
//  StallLog.h  -  ループが異常に長くかかった回だけ、どの処理が原因かを
//                 区間ごとの所要時間で記録する (原因調査用)
// ============================================================
//  I2C バスの瞬断とみられる長時間停止 (実測 818ms〜4.3秒) が発生していた。
//  I2Cdev::readTimeout 短縮 (IMU.h) だけでは直らず、VL53L1X は I2Cdev を
//  経由せず生の Wire を使うため対象外だった可能性が高い。確証が無いので
//  実際にどの区間で止まっているかを記録する。シリアル 'w' で USB へダンプ。
//
//  loop() 側は各区間の境界で micros() を取り、maybeLog() に渡すだけ。
// ============================================================
#pragma once
#include <Arduino.h>

namespace S5 {
namespace StallLog {

constexpr uint32_t THRESHOLD_US = 5000;  // これを超えた回だけ記録
constexpr int       CAPACITY    = 32;    // 最初の32件だけ残す (以後は無視)

struct Rec {
    uint32_t t_ms;
    uint32_t total_us;
    uint32_t imu_us, sbus_us, flow_us, range_us, rx_us, ctrl_us;
};

DMAMEM static Rec buf[CAPACITY];
static int count = 0;

inline void maybeLog(uint32_t total_us, uint32_t imu_us, uint32_t sbus_us,
                     uint32_t flow_us, uint32_t range_us, uint32_t rx_us,
                     uint32_t ctrl_us) {
    if (total_us < THRESHOLD_US) return;
    if (count >= CAPACITY) return;   // 最初の数件が分かれば十分
    Rec& r = buf[count++];
    r.t_ms = millis();
    r.total_us = total_us;
    r.imu_us = imu_us; r.sbus_us = sbus_us; r.flow_us = flow_us;
    r.range_us = range_us; r.rx_us = rx_us; r.ctrl_us = ctrl_us;
}

inline void dump() {
    Serial.printf("StallLog: %d 件 (閾値 %lu us)\n", count,
                  (unsigned long)THRESHOLD_US);
    for (int i = 0; i < count; ++i) {
        Rec& r = buf[i];
        Serial.printf(" [%2d] t=%lums total=%luus  imu=%lu sbus=%lu flow=%lu"
                      " range=%lu rx=%lu ctrl=%lu (us)\n",
                      i, (unsigned long)r.t_ms, (unsigned long)r.total_us,
                      (unsigned long)r.imu_us, (unsigned long)r.sbus_us,
                      (unsigned long)r.flow_us, (unsigned long)r.range_us,
                      (unsigned long)r.rx_us, (unsigned long)r.ctrl_us);
    }
    if (count == 0) Serial.println("  (異常な停止は記録されていません)");
}

} // namespace StallLog
} // namespace S5
