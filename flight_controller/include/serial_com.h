#pragma once
#include <Arduino.h>
#include <cmath>
#include "Config.h"

void print_PID(float r, float p, float y) {
    Serial.printf("|PIDRoll=%+7.3f |PIDPitch=%+7.3f |PIDYaw=%+7.3f\n", r, p, y);
}

void print_MPU(float r, float p, float y, float gr, float gp, float gy) {
    Serial.printf("Roll_Ang:%+7.2f,Pitch_Ang:%+7.2f,Yaw_Ang:%+7.2f,Roll_Gyr:%+7.2f,Pitch_Gyr:%+7.2f,Yaw_Gyr:%+7.2f\n",
                  r, p, y, gr, gp, gy);
}

void print_sbus(float Roll_des, float Pitch_des, float Thr_des, float Yaw_des, float Aux1_des, float Aux2_des, float Aux3_des) {
    Serial.printf("|ch1(ail)=%+6.2f |ch2(ele)=%+6.2f |ch3(thr)=%+6.2f |ch4(rud)=%+6.2f |Aux1(flp)=%+6.2f |Aux2(trn)=%+6.2f |Aux3(fig)=%+6.2f\n",
                  Roll_des, Pitch_des, Thr_des, Yaw_des, Aux1_des, Aux2_des, Aux3_des);
}

void print_flightmode(int currentMode, float bankAngle, unsigned long turnMs) {
    const char* modeStr[] = {"MANUAL    ", "LEVEL_TURN ", "FIGURE_8  "};
    Serial.printf("Mode: %s  BankAngle: %+6.1f deg  TurnMs: %5lu ms\n", modeStr[currentMode], bankAngle, turnMs);
}

void print_ACC(float ax, float ay, float az) {
    Serial.printf("Acc: ax=%+7.4f ay=%+7.4f az=%+7.4f [g]  |a|=%+7.4f\n",
                  ax, ay, az, sqrtf(ax*ax + ay*ay + az*az));
}

void print_timing(unsigned long dt_us) {
    Serial.printf("Timing: dt=%6lu us (%6.1f Hz)\n", dt_us, 1000000.0f / dt_us);
}
