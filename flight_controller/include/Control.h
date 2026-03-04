//制御、演算系
#pragma once
#include <Arduino.h>
#include "Config.h" // dt などを使うために必要

class PID {
    float kp_ori, ki_ori, kd_ori;
    float kp, ki, kd;
    float i, prev, d_lpf;
    float d_alpha, i_limit;

public:
    void initPID(float kp_i, float ki_i, float kd_i) {
        kp = kp_i; ki = ki_i; kd = kd_i;
        kp_ori = kp; ki_ori = ki; kd_ori = kd;
        i = 0; prev = 0; d_lpf = 0;
        i_limit = 200.0f;
        d_alpha = 0.7f;
    }

    void addjest(float kp_adj, float ki_adj, float kd_adj) {
        kp = kp_ori + kp_adj; ki = ki_ori + ki_adj; kd = kd_ori + kd_adj;
    }

    float pidStep(float input, float des, float kando) {
        float dt_sec = dt / 1e6f; // Config.h で extern 宣言された dt を使用
        float PID_value = 0;
        float e = des * kando - input;
        float d_raw = (e - prev) / dt_sec;
        i += e * dt_sec;
        d_lpf = d_alpha * d_lpf + (1.0f - d_alpha) * d_raw;
        prev = e;
        i = constrain(i, -i_limit, i_limit);
        PID_value = e * kp + i * ki + d_lpf * kd;
        return PID_value;
    }
};

class Axis_value {
public:
    float des, ang, acc, gyr;
    float pid_rate, pid_ang, pid;
    float Sen;
    PID c_rate, c_ang;

    void setgains(float kp_r, float ki_r, float kd_r, float kp_a, float ki_a, float kd_a, float Sen_in) {
        c_rate.initPID(kp_r, ki_r, kd_r);
        c_ang.initPID(kp_a, ki_a, kd_a);
        Sen = Sen_in;
    }

    void update_value(float des_in, float ang_in, float acc_in, float gyr_in) {
        des = des_in; ang = ang_in; acc = acc_in; gyr = gyr_in;
    }

    void update_RateAnglePID(){
        if(counter%10 == 0)  pid_ang  = c_ang.pidStep (ang,des,Sen);
        pid = c_rate.pidStep(gyr,pid_ang ,1);
    }

    void update_RatePID(float input) {
        pid = c_rate.pidStep(gyr, input, Sen);
    }

    void update_AnglePID(float input) {
        pid = c_ang.pidStep(ang, input, Sen);
    }

    void Rate_PID_adj(float kp_adj, float ki_adj, float kd_adj) {
        c_rate.addjest(kp_adj, ki_adj, kd_adj);
    }

    void Angle_PID_adj(float kp_adj, float ki_adj, float kd_adj) {
        c_ang.addjest(kp_adj, ki_adj, kd_adj);
    }
};