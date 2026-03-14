#pragma once
#include <Arduino.h>
#include "Config.h"

void print_PID(float r, float p, float y) {
    Serial.print("|PIDRoll= ");  Serial.print(r, 3);
    Serial.print("|PIDPitch= "); Serial.print(p, 3);
    Serial.print("|PIDYaw= ");   Serial.print(y, 3);
    Serial.print("\n");
}

void print_MPU(float ax, float ay, float az, float gx, float gy, float gz) {
    Serial.print("Roll_Ang");Serial.print(ax, 2);  Serial.print(",");
    Serial.print("Pitch_Ang");Serial.print(ay, 2); Serial.print(",");
    Serial.print("Yaw_Ang");Serial.print(az, 2);   Serial.print(",");
    Serial.print("Roll_Gyr");Serial.print(gx, 2);  Serial.print(",");
    Serial.print("Pitch_Gyr");Serial.print(gy, 2); Serial.print(",");
    Serial.print("Yaw_Gyr");Serial.print(gz, 2);   Serial.print(",");
    Serial.print("\n");
}

void print_sbus(float Roll_des, float Pitch_des, float Thr_des, float Yaw_des, float Aux1_des, float Aux2_des, float Aux3_des) {
    Serial.print("|ch1(ail) = ");  Serial.print(Roll_des, 2);
    Serial.print(" |ch2(ele) = "); Serial.print(Pitch_des, 2);
    Serial.print(" |ch3(thr) = "); Serial.print(Thr_des, 2);
    Serial.print(" |ch4(rud) = "); Serial.print(Yaw_des, 2);
    Serial.print(" |Aux1(flp)= "); Serial.print(Aux1_des, 2);
    Serial.print(" |Aux2(trn)= "); Serial.print(Aux2_des, 2);
    Serial.print(" |Aux3(fig)= "); Serial.print(Aux3_des, 2);
    Serial.print("\n");
}

void print_flightmode(int currentMode, float bankAngle, unsigned long turnMs) {
    const char* modeStr[] = {"MANUAL", "LEVEL_TURN", "FIGURE_8"};
    Serial.print("Mode: "); Serial.print(modeStr[currentMode]);
    Serial.print("  BankAngle: "); Serial.print(bankAngle, 1);
    Serial.print(" deg  TurnMs: "); Serial.print(turnMs);
    Serial.println(" ms");
}

void print_ACC(float ax, float ay, float az) {
    Serial.printf("Acc: ax=%+.4f ay=%+.4f az=%+.4f [g]  |a|=%.4f\n",
                  ax, ay, az, sqrtf(ax*ax + ay*ay + az*az));
}

void print_timing(unsigned long dt_us) {
    Serial.printf("Timing: dt=%lu us (%.0f Hz)\n", dt_us, 1000000.0f / dt_us);
}
