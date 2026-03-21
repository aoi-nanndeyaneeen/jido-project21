#pragma once
#include <Arduino.h>
#include <cmath>
#include "Config.h"

bool reset(){
    if (Serial.available()){
        char c = toupper(Serial.read());
        if( c == 'R') return true;
    }
    return false;
}

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
    const char* modeStr[] = {"MANUAL    ", "LEVEL_TURN ", "FIGURE_8  ", "SEMI_MANUAL"};
    Serial.printf("Mode: %s  BankAngle: %+6.1f deg  TurnMs: %5lu ms\n", modeStr[currentMode], bankAngle, turnMs);
}

void print_ACC(float ax, float ay, float az) {
    Serial.printf("Acc: ax=%+7.4f ay=%+7.4f az=%+7.4f [g]  |a|=%+7.4f\n",
                  ax, ay, az, sqrtf(ax*ax + ay*ay + az*az));
}

void print_timing(unsigned long dt_us) {
    Serial.printf("Timing: dt=%6lu us (%6.1f Hz)\n", dt_us, 1000000.0f / dt_us);
}

// ============================================================
//  Live PID Tuning Menu
// ============================================================
#include "Control.h" // Axis_value の定義に必要

void handlePIDTuning(Axis_value &roll, Axis_value &pitch, Axis_value &yaw) {
    long old_timeout = Serial.getTimeout();
    Serial.setTimeout(30000); // 30秒待機
    
    // バッファに残っている改行などを掃除
    while(Serial.available()) Serial.read();

    Serial.println("\n\n!! WARNING: CONTROL LOOP STOPPED DURING TUNING !!");
    Serial.println("========== PID Tuning Menu ==========");
    Serial.println("Select Parameter (ex: '1' for Roll Rate P):");
    Serial.printf(" [1] Roll  Rate P  : % 7.4f\n", roll.c_rate.get_kp());
    Serial.printf(" [2] Roll  Rate I  : % 7.4f\n", roll.c_rate.get_ki());
    Serial.printf(" [3] Roll  Rate D  : % 7.4f\n", roll.c_rate.get_kd());
    Serial.printf(" [4] Roll  Angle P : % 7.4f\n", roll.c_ang.get_kp());
    Serial.println("------------------------------------");
    Serial.printf(" [5] Pitch Rate P  : % 7.4f\n", pitch.c_rate.get_kp());
    Serial.printf(" [6] Pitch Rate I  : % 7.4f\n", pitch.c_rate.get_ki());
    Serial.printf(" [7] Pitch Rate D  : % 7.4f\n", pitch.c_rate.get_kd());
    Serial.printf(" [8] Pitch Angle P : % 7.4f\n", pitch.c_ang.get_kp());
    Serial.println("------------------------------------");
    Serial.printf(" [9] Yaw   Rate P  : % 7.4f\n", yaw.c_rate.get_kp());
    Serial.printf(" [A] Yaw   Rate I  : % 7.4f\n", yaw.c_rate.get_ki());
    Serial.printf(" [B] Yaw   Rate D  : % 7.4f\n", yaw.c_rate.get_kd());
    Serial.println(" [Q] Quit & Resume Control");
    Serial.print("Selection > ");

    while(!Serial.available());
    String sel = Serial.readStringUntil('\n');
    sel.trim();
    sel.toUpperCase();
    if (sel == "Q" || sel == "") {
        Serial.setTimeout(old_timeout);
        return;
    }

    Serial.printf("\nTarget [%s] chosen.\nEnter new value (e.g. 0.05) and press Enter > ", sel.c_str());
    
    // 次の数値を待つ前にバッファをもう一度クリア（改行対策）
    // while(Serial.available()) Serial.read(); // parseFloatは前のバッファを読んでしまう可能性あり

    float val = Serial.parseFloat();
    Serial.println(val, 4);

    if      (sel == "1") roll.set_rate_gains(val, roll.c_rate.get_ki(), roll.c_rate.get_kd());
    else if (sel == "2") roll.set_rate_gains(roll.c_rate.get_kp(), val, roll.c_rate.get_kd());
    else if (sel == "3") roll.set_rate_gains(roll.c_rate.get_kp(), roll.c_rate.get_ki(), val);
    else if (sel == "4") roll.set_angle_gains(val, roll.c_ang.get_ki(), roll.c_ang.get_kd());
    
    else if (sel == "5") pitch.set_rate_gains(val, pitch.c_rate.get_ki(), pitch.c_rate.get_kd());
    else if (sel == "6") pitch.set_rate_gains(pitch.c_rate.get_kp(), val, pitch.c_rate.get_kd());
    else if (sel == "7") pitch.set_rate_gains(pitch.c_rate.get_kp(), pitch.c_rate.get_ki(), val);
    else if (sel == "8") pitch.set_angle_gains(val, pitch.c_ang.get_ki(), pitch.c_ang.get_kd());

    else if (sel == "9") yaw.set_rate_gains(val, yaw.c_rate.get_ki(), yaw.c_rate.get_kd());
    else if (sel == "A") yaw.set_rate_gains(yaw.c_rate.get_kp(), val, yaw.c_rate.get_kd());
    else if (sel == "B") yaw.set_rate_gains(yaw.c_rate.get_kp(), yaw.c_rate.get_ki(), val);

    Serial.println("INFO: Parameter Updated. Resuming Control...");
    Serial.setTimeout(old_timeout);
}
