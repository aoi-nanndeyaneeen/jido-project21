// ============================================================
//  S5Telemetry.h  -  下りテレメトリ (機体 -> IM920 -> 地上局) の枠割りと詰め込み
// ============================================================
//  FlightLog の 500Hz USB ログは「PC を繋いだ地上テスト」でしか取れない。
//  実飛行では USB が無いので、解析に要る信号だけを TELEM_TX_HZ (8Hz) で
//  無線に落とし、地上局 (ground_receiver) が CSV に書く。
//  列の意味と分解能は protocol/S5Telem.h。
//
//  ★ 1 回の tick() で送れるのは 1 パケット (IM920sL は 32B 上限)。
//    何を送るかはモードで変える:
//      ANGLE / ALTHOLD … A,C の交互 (各 4Hz)。B(水平位置)/D(Δv) は意味が無い。
//      POSHOLD / GUIDED … A,B,A,B,C,A,B,D の 8 枠巡回
//                         (A 3Hz / B 3Hz / C 1Hz / D 1Hz)。
//        B は「PC が自分の指令の効きを見る唯一の枠」なので C/D より優先。
//        D (Δv + 上りリンク統計) は STAT と同じ 1Hz 粒度で足りる。
//        Δv を使う YawEstimator を本気で使う日が来たら D の枠を戻すこと。
//      P (ゲイン一覧) は TELEM_PARAM_MS ごとにその回の枠を borrow する。
//
//  ★ send() は組み立てるだけ。実際の UART 書き込みは s5tx.service() が
//    毎ループ空きぶんだけ進める。前のパケットが残っていれば捨てて
//    F_TX_DROP で地上局に知らせる (制御ループを止めるより良い)。
//
//  ★ 2026-09-17: GROUND_LINK == BLE (S5Features.h) では tick() ではなく tickBle()
//    を BLE_TELEM_HZ で呼ぶ。フレームの中身 (send*) は共通で、出口 (emit) だけが
//    「IM920 の送信バッファ」か「束ねて LogLink の T_TELEM」かで変わる。
// ============================================================
#pragma once
#include <Arduino.h>
#include "S5Telem.h"
#include "quad/S5Vehicle.h"
#include "quad/LogLink.h"
#include "quad/LogLinkProto.h"

namespace S5 {

class TelemetryTx {
public:
    // telem_tick (TELEM_TX_HZ) から呼ぶ。
    void tick(Vehicle& v) {
        const uint32_t now = millis();
        if (now - _last_param_ms >= TELEM_PARAM_MS) {
            _last_param_ms = now;
            sendParam(v);
            return;
        }

        if (!v.holdMode()) {
            // ANGLE / ALTHOLD: A と C の交互
            _slot = (uint8_t)((_slot + 1) & 0x03);   // 4刻みのままでよい (0/2→A, 1/3→C)
            if (_slot & 1) sendAtt(v);
            else           sendAlt(v);
            return;
        }

        // POSHOLD / GUIDED: A,B,A,B,C,A,B,D の8枠巡回
        _slot = (uint8_t)((_slot + 1) & 0x07);
        switch (_slot) {
            case 0: case 2: case 5: sendAlt(v); break;
            case 1: case 3: case 6: sendPos(v); break;
            case 4:                 sendAtt(v); break;
            default:                sendDv(v);  break;   // case 7
        }
    }

    // BLE 経路 (GROUND_LINK == BLE) の送信。BLE_TELEM_HZ で呼ぶ。
    //  IM920 のように「1 回 1 枚」に縛られないので、毎回 A と B を送り、
    //  C と D を交互に足して 1 フレームに束ねる。
    //    POSHOLD/GUIDED : A,B + (C または D)   → A,B 20Hz / C,D 10Hz
    //    ANGLE/ALTHOLD  : A,B,C                → D は送らない (tick() と同じ理由)
    //  P は数秒に 1 回、枠を奪わずに同じ束へ足す。
    //  ★ D は束の最後に置く。PC 側 (main_loop.py) は「最新行が D か」を
    //    ポーリングしているので、後ろに別フレームが続くと D 行を見逃しやすい。
    void tickBle(Vehicle& v) {
        sendAlt(v);
        sendPos(v);

        _slot = (uint8_t)(_slot + 1);
        const bool send_dv = v.holdMode() && (_slot & 1);

        const uint32_t now = millis();
        if (now - _last_param_ms >= TELEM_PARAM_MS) {
            _last_param_ms = now;
            sendParam(v);
        }
        if (send_dv) sendDv(v);
        else         sendAtt(v);
        flushBle();
    }

private:
    // 1 パケットを地上局リンクへ出す。経路は GROUND_LINK で決まる。
    //  IM920 : IM920::Tx の送信バッファへ (従来どおり 1 回 1 パケット)
    //  BLE   : 束ねる箱に足すだけ。実際に積むのは tickBle() 末尾の flushBle()
    template <typename T>
    void emit(Vehicle& v, const T& pkt) {
        if (USE_BLE_LINK) {
            if (_ble_n + sizeof(T) > sizeof(_ble_bundle)) flushBle();
            memcpy(_ble_bundle + _ble_n, &pkt, sizeof(T));
            _ble_n += sizeof(T);
            return;
        }
        if (!v.s5tx.send(pkt)) _tx_drop_flag = true;
    }

    // BLE: 束ねたぶんを 1 フレームで LogLink へ積む (UART へ流すのは LogLink::service)。
    void flushBle() {
        if (_ble_n == 0) return;
        if (!LogLink::sendTelem(_ble_bundle, _ble_n)) _tx_drop_flag = true;
        _ble_n = 0;
    }

    // A/B/C/D 共通のヘッダを埋める。
    void fillHeader(Vehicle& v, S5T::Header& h, uint8_t type) {
        h.type = type;
        h.seq  = _seq++;

        uint16_t f = 0;
        if (isArmed(v))                 f |= S5T::F_ARMED;
        if (v.flowobs.ok)               f |= S5T::F_FLOW_OK;
        if (v.range.ok)                 f |= S5T::F_RANGE_OK;
        if (v.range.valid)              f |= S5T::F_RANGE_VALID;
        if (v.alt_hold_enable)          f |= S5T::F_ALT_EN;
        if (v.althold.active())         f |= S5T::F_ALT_ACT;
        if (v.poshold.holding())        f |= S5T::F_POS_HOLD;
        if (v.althold.airborne())       f |= S5T::F_AIRBORNE;
        if (v.dry_run)                  f |= S5T::F_DRY_RUN;
        if (v.mix.sat)                  f |= S5T::F_SAT;
        if (_tx_drop_flag)            { f |= S5T::F_TX_DROP; _tx_drop_flag = false; }
        // 地上局ガイド飛行の状態。地上局はこれを見て「離陸が終わったか」
        // 「まだ自分の指令で飛んでいるか」を判断する。
        if (v.guided.engaged())         f |= S5T::F_GUIDED;
        if (Quad::GUIDED_ENABLE && v.s5rx.fresh(Quad::GUIDED_STALE_HOLD_MS))
                                        f |= S5T::F_CMD_FRESH;
        if (v.guided.landed())          f |= S5T::F_LANDED;
        if (v.guided.inManeuver())      f |= S5T::F_MANEUVER;
        if (v.poshold.frameOk())        f |= S5T::F_FRAME_OK;
        h.flags = f;

        // 28バイトに uint32 の millis は載らないので 10ms 単位。
        // 655.35秒で一周する。地上側が展開する。
        h.t_cs = (uint16_t)(millis() / 10u);
    }

    uint8_t modes(const Vehicle& v) const {
        return S5T::packModes((uint8_t)v.mode, (uint8_t)v.althold.state());
    }

    // A: 高度ループ + 姿勢
    void sendAlt(Vehicle& v) {
        S5T::AltFrame a{};
        fillHeader(v, a.h, S5T::TYPE_ALT);
        a.modes = modes(v);
        a.thr   = S5T::qu8(USE_SBUS ? v.sbus.des[Ch::THR] : 0.0f, 250.0f);

        a.roll_cd  = S5T::q16(v.att.roll,      S5T::SC_CDEG);
        a.pitch_cd = S5T::q16(v.att.pitch,     S5T::SC_CDEG);
        a.yaw_dd   = S5T::q16(v.heading.est(), S5T::SC_DDEG);

        a.range_h_mm      = S5T::q16(v.range.h_m,         S5T::SC_MM);
        a.range_raw_mm    = S5T::q16(v.range.raw_m,       S5T::SC_MM);
        a.alt_hold_mm     = S5T::q16(v.althold.holdM(),   S5T::SC_MM);
        a.climb_mmps      = S5T::q16(v.range.climb_mps,   S5T::SC_MM);
        a.alt_vz_tar_mmps = S5T::q16(v.althold.vzTar(),   S5T::SC_MM);
        a.alt_thr_corr    = S5T::q16(v.althold.thrCorr(), S5T::SC_1E4);
        a.alt_thr_out     = S5T::qu16(v.althold.active() ? v.althold.thrOut() : 0.0f, S5T::SC_1E4);

        emit(v, a);
    }

    // B: 水平位置ループ
    void sendPos(Vehicle& v) {
        S5T::PosFrame b{};
        fillHeader(v, b.h, S5T::TYPE_POS);
        b.modes = modes(v);
        b.bad   = (uint8_t)constrain(v.poshold.badCount(), 0, 255);

        b.vx_mmps       = S5T::q16(v.poshold.vxCtl(),     S5T::SC_MM);
        b.vy_mmps       = S5T::q16(v.poshold.vyCtl(),     S5T::SC_MM);
        b.vx_tar_mmps   = S5T::q16(v.poshold.vxTar(),     S5T::SC_MM);
        b.vy_tar_mmps   = S5T::q16(v.poshold.vyTar(),     S5T::SC_MM);
        b.pos_n_cm      = S5T::q16(v.poshold.posN(),      S5T::SC_CM);
        b.pos_e_cm      = S5T::q16(v.poshold.posE(),      S5T::SC_CM);
        b.hold_n_cm     = S5T::q16(v.poshold.holdN(),     S5T::SC_CM);
        b.hold_e_cm     = S5T::q16(v.poshold.holdE(),     S5T::SC_CM);
        b.lean_roll_cd  = S5T::q16(v.poshold.leanRoll(),  S5T::SC_CDEG);
        b.lean_pitch_cd = S5T::q16(v.poshold.leanPitch(), S5T::SC_CDEG);

        emit(v, b);
    }

    // C: 姿勢ループの内部 (モーター出力 / 角速度 / ミキサー飽和)
    //  離陸時の転倒や発振の切り分け用。「どのモーターが飽和したか」
    //  「レートループが指令に追従しているか」は A/B では分からない。
    void sendAtt(Vehicle& v) {
        S5T::AttFrame c{};
        fillHeader(v, c.h, S5T::TYPE_ATT);
        c.modes = modes(v);
        c.sat   = v.mix.sat;

        c.m1 = S5T::qu8(v.out[0], 250.0f);
        c.m2 = S5T::qu8(v.out[1], 250.0f);
        c.m3 = S5T::qu8(v.out[2], 250.0f);
        c.m4 = S5T::qu8(v.out[3], 250.0f);

        c.roll_rate_dd      = S5T::q16(v.roll_axis.rate_meas,  S5T::SC_DDEG);
        c.pitch_rate_dd     = S5T::q16(v.pitch_axis.rate_meas, S5T::SC_DDEG);
        c.yaw_rate_dd       = S5T::q16(v.yaw_axis.rate_meas,   S5T::SC_DDEG);
        c.roll_rate_tar_dd  = S5T::q16(v.roll_axis.rate_tar,   S5T::SC_DDEG);
        c.pitch_rate_tar_dd = S5T::q16(v.pitch_axis.rate_tar,  S5T::SC_DDEG);

        c.roll_cmd  = S5T::q16(v.roll_axis.cmd,  S5T::SC_1E4);
        c.pitch_cmd = S5T::q16(v.pitch_axis.cmd, S5T::SC_1E4);

        // ★ roll_axis.stick ではなく sbus から直接読む。roll_axis.stick は
        //   非アーム中は入らないので 0 のままになる。トリム確認は飛ばす前にやりたい。
        if (USE_SBUS) {
            const float rs = constrain(v.sbus.des[Ch::ROLL],  -1.0f, 1.0f);
            const float ps = constrain(v.sbus.des[Ch::PITCH], -1.0f, 1.0f);
            c.roll_stick  = (int8_t)constrain(lroundf(rs * S5T::SC_STICK), -127L, 127L);
            c.pitch_stick = (int8_t)constrain(lroundf(ps * S5T::SC_STICK), -127L, 127L);
        }
        emit(v, c);
    }

    // D: ヨー推定用の機体Δv + 上りリンクの統計。POSHOLD/GUIDED でだけ送る。
    //  上りリンクの統計をここに同乗させる理由は S5Telem.h の DvFrame 参照
    //  (A/B/C は 28 byte 満杯で、空いているのがここしか無い)。
    void sendDv(Vehicle& v) {
        S5T::DvFrame d{};
        fillHeader(v, d.h, S5T::TYPE_DV);

        float dvx, dvy;
        v.body_dv.drain(dvx, dvy);
        d.dvx_mmps = S5T::q16(dvx, S5T::SC_MM);
        d.dvy_mmps = S5T::q16(dvy, S5T::SC_MM);
        d.yaw_dd   = S5T::q16(v.heading.est(), S5T::SC_DDEG);

        // ageMs() は未受信のとき 0xFFFFFFFF。10ms 単位に落とす前に飽和させる。
        const uint32_t age_ms = v.s5rx.ageMs();
        d.cmd_age_cs = (age_ms > 655340u) ? 0xFFFFu : (uint16_t)(age_ms / 10u);
        d.cmd_good   = (uint16_t)v.s5rx.nGood();
        d.cmd_lost   = (uint16_t)v.s5rx.nLost();
        d.cmd_bad    = (uint16_t)(v.s5rx.nBadCs() + v.s5rx.nBadLen() + v.s5rx.nBadVer());
        d.cmd_seq    = v.s5rx.last().seq;
        d.cmd_rssi   = (int8_t)constrain(v.s5rx.rssi(), -128, 127);

        emit(v, d);
    }

    // P: 今どのゲインで飛んでいるか。数秒に1回。
    void sendParam(Vehicle& v) {
        S5T::ParamFrame p{};
        p.type = S5T::TYPE_PARAM;
        p.seq  = _seq++;
        p.ver  = S5T::VERSION;
        p.cfg_flags = (uint8_t)((v.alt_hold_enable ? S5T::PF_ALT_HOLD_EN : 0) |
                                (v.dry_run         ? S5T::PF_DRY_RUN     : 0) |
                                ((Quad::RANGE_BACKEND == Quad::RangeBackend::Sonar_EZ)
                                                   ? S5T::PF_SONAR       : 0) |
                                (Quad::ALT_STICK_VZ_ENABLE ? S5T::PF_STICK_VZ : 0));

        // 実際に効いている値を読む (シリアル 'p' で飛行中に変えられるため)
        const Quad::Pid& vp = v.poshold.velPid();
        p.flow_vel_kp = S5T::q16(vp.kp(), S5T::SC_GAIN);
        p.flow_vel_ki = S5T::q16(vp.ki(), S5T::SC_GAIN);
        p.flow_vel_kd = S5T::q16(vp.kd(), S5T::SC_GAIN);
        p.flow_pos_kp = S5T::q16(v.poshold.posKp(), S5T::SC_GAIN);

        const Quad::Pid& ap = v.althold.ratePid();
        p.alt_pos_kp  = S5T::q16(v.althold.posKp(), S5T::SC_GAIN);
        p.alt_rate_kp = S5T::q16(ap.kp(), S5T::SC_GAIN);
        p.alt_rate_ki = S5T::q16(ap.ki(), S5T::SC_GAIN);
        p.alt_rate_kd = S5T::q16(ap.kd(), S5T::SC_GAIN);

        p.alt_hover_thr = S5T::q16(Quad::ALT_HOVER_THR, S5T::SC_GAIN);
        p.alt_target_m  = S5T::q16(Quad::ALT_TARGET_M,  S5T::SC_GAIN);
        p.flow_max_lean = S5T::q16(Quad::FLOW_MAX_LEAN, S5T::SC_GAIN);
        p.alt_thr_auth  = S5T::q16(Quad::ALT_THR_AUTH,  S5T::SC_GAIN);

        emit(v, p);
    }

    uint8_t  _seq           = 0;
    uint32_t _last_param_ms = 0;
    bool     _tx_drop_flag  = false;  // 直前の送信が捨てられたか (次パケットで通知)
    uint8_t  _slot          = 0;      // 枠カウンタ

    // BLE 経路で 1 回ぶんのフレームを束ねる箱。A+B+C+P でも 28x4 = 112B で 255B に収まる。
    uint8_t  _ble_bundle[LogLinkProto::TELEM_MAX_PAYLOAD];
    size_t   _ble_n = 0;
};

} // namespace S5
