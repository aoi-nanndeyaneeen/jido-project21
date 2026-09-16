// ============================================================
//  S5LogFill.h  -  ログ 1 行 (FlightLog::Rec) に機体の今の全状態を詰める
// ============================================================
//  この 1 個の Rec を loop() が USB / RAM / SD / LogLink の 4 つのシンクへ配る。
//  Rec の定義・ヘッダ・整形は quad/FlightLog.h。
//
//  ★ 列を足すときに触るのは、ここと FlightLog.h の Rec / HEADER /
//    formatRow、それに scripts/bin2csv.py の合計 5 箇所。
//    型か並びを変えたら FlightLog::REC_VER を +1 すること。
// ============================================================
#pragma once
#include <Arduino.h>
#include "quad/FlightLog.h"
#include "quad/S5Vehicle.h"

namespace S5 {

inline void fillRec(FlightLog::Rec& r, Vehicle& v, uint32_t dt_us, bool armed, float thr) {
    r.t_ms  = millis();
    r.dt_us = (uint16_t)constrain(dt_us, 0u, 65535u);

    uint16_t f = 0;
    if (armed)                  f |= FlightLog::RF_ARMED;
    if (v.flowobs.ok)           f |= FlightLog::RF_FLOW_OK;
    if (v.range.ok)             f |= FlightLog::RF_RANGE_OK;
    if (v.alt_hold_enable)      f |= FlightLog::RF_ALT_EN;
    if (v.althold.active())     f |= FlightLog::RF_ALT_ACT;
    if (v.poshold.holding())    f |= FlightLog::RF_HOLDING;
    if (USE_SBUS) {
        if (v.sbus.failsafeFlag())    f |= FlightLog::RF_SBUS_FS;
        if (v.sbus.lostFrameFlag())   f |= FlightLog::RF_SBUS_LOST;
        if (v.sbus.failCount() > 0)   f |= FlightLog::RF_SBUS_STALE;
    }
    r.flags = f;

    r.mode   = (uint8_t)v.mode;
    r.mixsat = v.mix.sat;
    r.thr    = S5T::qu8(thr, 250.0f);

    r.roll_stick  = (int8_t)constrain(lroundf(v.roll_axis.stick  * 100.0f), -127L, 127L);
    r.pitch_stick = (int8_t)constrain(lroundf(v.pitch_axis.stick * 100.0f), -127L, 127L);
    r.yaw_stick   = (int8_t)constrain(lroundf(v.yaw_axis.stick   * 100.0f), -127L, 127L);

    r.roll_ang  = S5T::q16(v.roll_axis.ang_meas,  S5T::SC_CDEG);
    r.pitch_ang = S5T::q16(v.pitch_axis.ang_meas, S5T::SC_CDEG);
    r.yaw_est   = S5T::q16(v.heading.est(),       S5T::SC_CDEG);

    r.roll_rate  = S5T::q16(v.roll_axis.rate_meas,  S5T::SC_DDEG);
    r.pitch_rate = S5T::q16(v.pitch_axis.rate_meas, S5T::SC_DDEG);
    r.yaw_rate   = S5T::q16(v.yaw_axis.rate_meas,   S5T::SC_DDEG);

    r.roll_cmd  = S5T::q16(v.roll_axis.cmd,  S5T::SC_1E4);
    r.pitch_cmd = S5T::q16(v.pitch_axis.cmd, S5T::SC_1E4);
    r.yaw_cmd   = S5T::q16(v.yaw_axis.cmd,   S5T::SC_1E4);

    r.m1 = S5T::qu8(v.out[0], 250.0f);
    r.m2 = S5T::qu8(v.out[1], 250.0f);
    r.m3 = S5T::qu8(v.out[2], 250.0f);
    r.m4 = S5T::qu8(v.out[3], 250.0f);

    r.span_limit = S5T::q16(v.mix.span_limit, 1000.0f);

    r.roll_ratetar  = S5T::q16(v.roll_axis.rate_tar,  S5T::SC_DDEG);
    r.pitch_ratetar = S5T::q16(v.pitch_axis.rate_tar, S5T::SC_DDEG);
    r.roll_angtar   = S5T::q16(v.roll_axis.ang_tar,   S5T::SC_CDEG);
    r.pitch_angtar  = S5T::q16(v.pitch_axis.ang_tar,  S5T::SC_CDEG);

    r.flow_raw_x = S5T::q16(v.flowobs.raw_x, 10.0f);
    r.flow_raw_y = S5T::q16(v.flowobs.raw_y, 10.0f);
    r.flow_dx    = S5T::q16(v.flowobs.dx,    10.0f);
    r.flow_dy    = S5T::q16(v.flowobs.dy,    10.0f);
    r.flow_vx    = S5T::q16(v.flowobs.vx, S5T::SC_MM);
    r.flow_vy    = S5T::q16(v.flowobs.vy, S5T::SC_MM);
    r.flow_h     = S5T::q16(v.flow.height(), S5T::SC_MM);
    r.flow_accx  = (float)v.flowobs.acc_m_x;
    r.flow_accy  = (float)v.flowobs.acc_m_y;

    r.fh_vxc = S5T::q16(v.poshold.vxCtl(), S5T::SC_MM);
    r.fh_vyc = S5T::q16(v.poshold.vyCtl(), S5T::SC_MM);
    r.fh_vxt = S5T::q16(v.poshold.vxTar(), S5T::SC_MM);
    r.fh_vyt = S5T::q16(v.poshold.vyTar(), S5T::SC_MM);
    r.fh_leanr = S5T::q16(v.poshold.leanRoll(),  S5T::SC_CDEG);
    r.fh_leanp = S5T::q16(v.poshold.leanPitch(), S5T::SC_CDEG);
    r.fh_posn  = S5T::q16(v.poshold.posN(),  S5T::SC_MM);
    r.fh_pose  = S5T::q16(v.poshold.posE(),  S5T::SC_MM);
    r.fh_holdn = S5T::q16(v.poshold.holdN(), S5T::SC_MM);
    r.fh_holde = S5T::q16(v.poshold.holdE(), S5T::SC_MM);

    r.range_raw = S5T::q16(v.range.raw_m,     S5T::SC_MM);
    r.range_h   = S5T::q16(v.range.h_m,       S5T::SC_MM);
    r.climb     = S5T::q16(v.range.climb_mps, S5T::SC_MM);

    r.alt_holdm   = S5T::q16(v.althold.holdM(),   S5T::SC_MM);
    r.alt_vzt     = S5T::q16(v.althold.vzTar(),   S5T::SC_MM);
    r.alt_corr    = S5T::q16(v.althold.thrCorr(), S5T::SC_1E4);
    r.alt_thr_out = S5T::qu8(v.althold.thrOut(),  250.0f);
    r.alt_used    = S5T::qu8(v.mix.thr_used,      250.0f);

    // 加速度Z相補フィルタ検討用 (制御には未使用)。
    // scale 1000 (±32.7g)。±8g 化 + バイアスで acc_z は [-10,+6] を取りうる (REC_VER 2)。
    r.accx = S5T::q16(v.att.acc_x, 1000.0f);
    r.accy = S5T::q16(v.att.acc_y, 1000.0f);
    r.accz = S5T::q16(v.att.acc_z, 1000.0f);
    //  acc_up / est_bias は m/s^2。1e3 倍で ±32 m/s^2 まで入る。
    r.acc_up   = S5T::q16(v.altest.accUp(),    1000.0f);
    r.est_h    = S5T::q16(v.altest.heightM(),  S5T::SC_MM);
    r.est_vz   = S5T::q16(v.altest.climbMps(), S5T::SC_MM);
    r.est_bias = S5T::q16(v.altest.biasMps2(), 1000.0f);
}

} // namespace S5
