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
    PID(float kp_i = 0, float ki_i = 0, float kd_i = 0,float d_alpha_i = 0.7f, float i_limit_i = 200.0f) : 
        kp(kp_i), ki(ki_i), kd(kd_i),
        kp_ori(kp_i), ki_ori(ki_i), kd_ori(kd_i),
        i(0), prev(0), d_lpf(0), d_alpha(d_alpha_i), i_limit(i_limit_i) {}

    void setgains(float kp_i, float ki_i, float kd_i, float d_alpha_i = 0.7f, float i_limit_i = 200.0f) {
        kp = kp_i; ki = ki_i; kd = kd_i;
        kp_ori = kp_i; ki_ori = ki_i; kd_ori = kd_i;
        d_alpha = d_alpha_i; i_limit = i_limit_i;
    }

    void addjest(float kp_adj, float ki_adj, float kd_adj) {
        kp = kp_ori + kp_adj; ki = ki_ori + ki_adj; kd = kd_ori + kd_adj;
    }

    void reset() {
    i = 0.0f;       // 積分項をゼロに
    prev = 0.0f;    // 前回誤差をゼロに
    d_lpf = 0.0f;   // フィルタ後の微分値もクリア
    }

    float pidStep(float input, float des, float kando) {
        float dt_sec = Config::Timing::Main_dt / 1e6f; // Config.h 宣言された メイン周期のdt を使用
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

    Axis_value (float kp_r, float ki_r, float kd_r, float kp_a, float ki_a, float kd_a, float Sen_in = 1.0f, 
                float d_alpha_r = 0.7f, float i_limit_r = 200.0f, float d_alpha_a = 0.7f, float i_limit_a = 200.0f)
    {
        c_rate.setgains(kp_r, ki_r, kd_r, d_alpha_r, i_limit_r);
        c_ang.setgains(kp_a, ki_a, kd_a, d_alpha_a, i_limit_a);
        Sen = Sen_in;
    }

    void update_value(float des_in, float ang_in, float acc_in, float gyr_in) {
        des = des_in; ang = ang_in; acc = acc_in; gyr = gyr_in;
    }
    
    void pid_reset() {
        c_rate.reset();
        c_ang.reset();
    }

    void update_RateAnglePID(float input){
        static int counter = 0;
        if(++counter >= 10){// 1000Hz / 10 = 100Hz
            pid_ang  = c_ang.pidStep (ang,input,Sen);
            counter = 0;
        }  
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