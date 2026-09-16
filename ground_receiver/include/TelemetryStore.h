// ============================================================
//  TelemetryStore.h  -  下りテレメトリの受信・保持・CSV/STAT 出力
// ============================================================
//  機体 (drone_s5) が 8Hz で投げてくる S5T::AltFrame / PosFrame / AttFrame /
//  DvFrame / ParamFrame を受け取り、直近値を持ち、USB へ CSV として吐く。
//
//  ★ IM920sL の実効ペイロードは 32 byte なので 1 パケットに全部は載らず、
//    A(高度+姿勢角) / B(水平位置) / C(姿勢ループ内部) / D(Δv+上りリンク統計) を
//    順番に送っている。ここでは受け取るたびに「他のフレームは前回値」で 1 行を
//    組み立てる (forward-fill)。frame 列が「今回どれが新しいか」 = 0:A 1:B 2:C 3:D。
//    列の意味は protocol/S5Telem.h。
//
//  出力プロトコル (PC 側 position_estimator/src/core/s5_link.py と対):
//      HEADER,<列名...>     ... LOG_START の直前に 1 回
//      LOG_START / LOG_STOP
//      DATA,<値...>         ... 1 パケット = 1 行
//      PARAM,<ゲイン...>    ... 数秒に 1 回。どのゲインで飛んでいたかの記録
//      STAT,k=v,...         ... 毎秒。PC の通信診断画面用 (CSV には含めない)
//      # ...                ... 人間向けのメッセージ (PC 側は読み飛ばす)
// ============================================================
#pragma once
#include <Arduino.h>
#include "S5Telem.h"
#include "Im920Frame.h"
#include "GroundConfig.h"

namespace Ground {

class TelemetryStore {
public:
    // ---- CSV の列。A/B/C/D を forward-fill して 1 行にまとめる ----
    //  ★ 列を足したら必ず emitData() 側も同じ順で足すこと。
    static constexpr const char* CSV_HEADER =
        "rx_ms,t_ms,seq,lost,rssi,frame,"
        "mode,alt_state,armed,flow_ok,range_ok,range_valid,"
        "alt_en,alt_act,pos_hold,airborne,dry_run,sat,tx_drop,"
        "guided,cmd_fresh,landed,maneuver,frame_ok,"
        "thr,bad,"
        "roll,pitch,yaw,"
        "range_h,range_raw,alt_hold,climb,alt_vzt,alt_corr,alt_thr,"
        "fh_vxc,fh_vyc,fh_vxt,fh_vyt,"
        "fh_posn,fh_pose,fh_holdn,fh_holde,"
        "fh_leanr,fh_leanp,"
        "m1,m2,m3,m4,mixsat,"
        "roll_gyr,pitch_gyr,yaw_gyr,roll_ratetar,pitch_ratetar,"
        "roll_cmd,pitch_cmd,roll_stick,pitch_stick,"
        "dvx,dvy,dv_yaw,"
        // 上りリンクを機体側から見た値 (D フレーム同乗。S5Telem.h の DvFrame)。
        //  cmd_age  : 機体が最後に上りコマンドを受けてからの経過 [s] (-1 = 未受信)
        //  cmd_good / cmd_lost / cmd_bad : 機体が数えた累積 (下位 16bit で一周)
        //  cmd_seq  : 機体が最後に受けたコマンドの seq  cmd_rssi : その RSSI
        //  ★ STAT の cmd_tx (こちらが送った回数) と並べると「PC が遅いのか / 上りが
        //    落ちているのか / 下りが遅いのか」が分かれる。
        "cmd_age,cmd_good,cmd_lost,cmd_bad,cmd_seq,cmd_rssi";

    // ------------------------------------------------------------
    //  IM920 から 1 行入ったら呼ぶ。テレメトリなら取り込み、CSV ON なら 1 行出す。
    //  ★ "OK"/"NG" (TXDA への応答) は数えるだけ。NG = その上りコマンドは電波に
    //    出ていない。半二重なので下りを受信している最中に出やすい。
    // ------------------------------------------------------------
    void handleLine(const char* line, bool raw_dump) {
        n_rx_lines++;
        if (raw_dump) { Serial.print("# RAW "); Serial.println(line); }

        uint8_t buf[S5T::PACKET_BYTES];
        const IM920::Decoded d = IM920::decodeLine(line, buf, sizeof(buf));
        if (d.st == IM920::Decode::NOT_DATA) {
            if (strcmp(line, "OK") == 0)      { n_im_ok++; return; }
            if (strcmp(line, "NG") == 0)      { n_im_ng++; return; }
            if (line[0]) { Serial.print("# "); Serial.println(line); }   // 起動メッセージ等
            return;
        }
        if (d.st == IM920::Decode::BAD_CS)  { n_bad_cs++;  return; }
        if (d.st != IM920::Decode::OK)      { n_bad_len++; return; }

        last_rx_ms = millis();
        last_rssi  = d.rssi;

        // type を見てから期待長を引く (DvFrame は 28 byte ちょうどではない)
        const size_t expected = S5T::payloadBytesFor(buf[0]);
        if (expected == 0 || d.n != expected) {
            n_bad_len++;
            if (n_bad_len <= 3)
                Serial.printf("# 長さ不一致 type=0x%02X len=%u (期待 %u)  "
                              "S5Telem.h が機体側とずれていないか\n",
                              buf[0], (unsigned)d.n, (unsigned)expected);
            return;
        }

        // seq の飛びで欠落を数える (255 -> 0 の折り返しも自然に扱える)
        const uint8_t seq = buf[1];
        if (seq_init) {
            const uint8_t gap = (uint8_t)(seq - prev_seq);
            if (gap > 1) n_lost += (uint32_t)(gap - 1);
        }
        prev_seq = seq;
        seq_init = true;

        switch (buf[0]) {
            case S5T::TYPE_ALT:
                memcpy(&alt, buf, sizeof(alt)); have_alt = true; n_alt++;
                live_h = alt.h; live_modes = alt.modes;
                emitIfOn(seq, 0, alt.h.t_cs);
                break;
            case S5T::TYPE_POS:
                memcpy(&pos, buf, sizeof(pos)); have_pos = true; n_pos++;
                live_h = pos.h; live_modes = pos.modes;
                emitIfOn(seq, 1, pos.h.t_cs);
                break;
            case S5T::TYPE_ATT:
                memcpy(&att, buf, sizeof(att)); have_att = true; n_att++;
                live_h = att.h; live_modes = att.modes;
                emitIfOn(seq, 2, att.h.t_cs);
                break;
            case S5T::TYPE_DV:
                memcpy(&dv, buf, sizeof(dv)); have_dv = true; n_dv++;
                live_h = dv.h;      // DvFrame は modes を持たない (直近の A/B/C の値のまま)
                emitIfOn(seq, 3, dv.h.t_cs);
                break;
            case S5T::TYPE_PARAM:
                memcpy(&param, buf, sizeof(param)); have_param = true; n_param++;
                if (param.ver != S5T::VERSION)
                    Serial.printf("# !! パケットバージョン不一致: 機体=%u 地上=%u  "
                                  "protocol/S5Telem.h を両側そろえて焼き直すこと\n",
                                  (unsigned)param.ver, (unsigned)S5T::VERSION);
                if (csv_on) emitParam();
                break;
            default:
                n_bad_len++;
                if (n_bad_len <= 3)
                    Serial.printf("# 未知の type=0x%02X (期待 A=0x%02X B=0x%02X C=0x%02X D=0x%02X P=0x%02X)\n",
                                  buf[0], S5T::TYPE_ALT, S5T::TYPE_POS,
                                  S5T::TYPE_ATT, S5T::TYPE_DV, S5T::TYPE_PARAM);
                break;
        }
    }

    // ---- CSV 出力の ON / OFF (冪等。PC 側 s5_link.py は '1'/'0' を使う) ----
    void csvOn() {
        Serial.println(csv_on ? "# CSV 出力は既に ON。HEADER を再送します"
                              : "# CSV 出力 ON。※ファイルを作るのは PC 側 (S5Link / s5_logger.py) です");
        Serial.println();
        Serial.print("HEADER,"); Serial.println(CSV_HEADER);
        Serial.println("LOG_START");
        csv_on = true;
        if (have_param) emitParam();
        if (!live())
            Serial.println("# まだテレメトリが来ていないので DATA 行は出ません "
                           "(2秒ごとに待機中の表示を出します)");
    }
    void csvOff() {
        if (!csv_on) { Serial.println("# CSV 出力は既に OFF"); return; }
        csv_on = false;
        Serial.println("LOG_STOP");
        Serial.printf("# A=%lu B=%lu C=%lu D=%lu param=%lu lost=%lu badcs=%lu badlen=%lu\n",
                      (unsigned long)n_alt, (unsigned long)n_pos, (unsigned long)n_att,
                      (unsigned long)n_dv, (unsigned long)n_param, (unsigned long)n_lost,
                      (unsigned long)n_bad_cs, (unsigned long)n_bad_len);
    }

    void clearStats() {
        n_rx_bytes = n_rx_lines = 0;
        n_alt = n_pos = n_att = n_dv = n_param = n_lost = n_bad_cs = n_bad_len = 0;
        n_im_ok = n_im_ng = 0;
        seq_init = false;
    }

    bool     live()  const { return have_alt || have_pos || have_att || have_dv; }
    uint32_t ageMs() const { return millis() - last_rx_ms; }
    bool     linkOk() const { return live() && ageMs() < LINK_LOST_MS; }

    // ---- 直近フレーム (表示用) ----
    S5T::AltFrame   alt{};
    S5T::PosFrame   pos{};
    S5T::AttFrame   att{};
    S5T::DvFrame    dv{};
    S5T::ParamFrame param{};
    bool have_alt = false, have_pos = false, have_att = false, have_dv = false, have_param = false;
    S5T::Header live_h{};       // 直近に届いたフレームのヘッダ (flags)。seq の大小比較は
    uint8_t     live_modes = 0; //  255->0 で古い方を選ぶので「最後に届いたもの」を使う
    int      last_rssi  = -1;
    uint32_t last_rx_ms = 0;
    bool     csv_on = false;

    // ---- 受信統計 ----
    //  n_rx_bytes / n_rx_lines は「IM920 から素のバイトが来ているか」の切り分け用。
    //  0 のまま = 配線か電源かボーレート。増えているのに n_alt+n_pos が 0 =
    //  相手が違う / パケット定義がずれている。
    uint32_t n_rx_bytes = 0, n_rx_lines = 0;
    uint32_t n_alt = 0, n_pos = 0, n_att = 0, n_dv = 0, n_param = 0;
    uint32_t n_lost = 0;      // seq の飛びから数えた累積欠落
    uint32_t n_bad_cs = 0;    // チェックサム不一致
    uint32_t n_bad_len = 0;   // 長さ / type が合わない (構造体バージョン違い?)
    uint32_t n_im_ok = 0, n_im_ng = 0;   // IM920 が TXDA に返した OK / NG

    // ------------------------------------------------------------
    //  STAT 行 (PC の常時診断画面用。CSV とは別なので ON/OFF に関係なく毎秒出す)
    //    cmd_lines/cmd_tx/cmd_bad/cmd_have は Uplink の値を引数で受ける。
    // ------------------------------------------------------------
    void emitStat(bool im920_boot_ok, uint32_t cmd_lines, uint32_t cmd_tx,
                  uint32_t cmd_bad, bool cmd_have) const {
        Serial.printf("STAT,im920_ok=%d,rx_bytes=%lu,rx_lines=%lu,alt=%lu,pos=%lu,att=%lu,dv=%lu,"
                      "param=%lu,lost=%lu,badcs=%lu,badlen=%lu,cmd_lines=%lu,cmd_tx=%lu,"
                      "cmd_bad=%lu,cmd_have=%d,im_ok=%lu,im_ng=%lu,"
                      "fc_cmd_good=%u,fc_cmd_lost=%u,fc_cmd_bad=%u,fc_cmd_age_cs=%u\n",
                      im920_boot_ok ? 1 : 0,
                      (unsigned long)n_rx_bytes, (unsigned long)n_rx_lines,
                      (unsigned long)n_alt, (unsigned long)n_pos, (unsigned long)n_att,
                      (unsigned long)n_dv,
                      (unsigned long)n_param, (unsigned long)n_lost,
                      (unsigned long)n_bad_cs, (unsigned long)n_bad_len,
                      (unsigned long)cmd_lines, (unsigned long)cmd_tx,
                      (unsigned long)cmd_bad, cmd_have ? 1 : 0,
                      (unsigned long)n_im_ok, (unsigned long)n_im_ng,
                      // 機体が「受け取った」と言っている数 (D フレーム同乗)
                      have_dv ? (unsigned)dv.cmd_good : 0u,
                      have_dv ? (unsigned)dv.cmd_lost : 0u,
                      have_dv ? (unsigned)dv.cmd_bad  : 0u,
                      have_dv ? (unsigned)dv.cmd_age_cs : 0xFFFFu);
    }

private:
    // 機体時刻の展開。t_cs は 10ms 単位の uint16 なので 655.36 秒で一周する。
    uint32_t unwrapTime(uint16_t t_cs) {
        if (!t_init) { t_init = true; t_cs_prev = t_cs; t_ms_unwrapped = (uint32_t)t_cs * 10u; }
        else {
            const uint16_t d = (uint16_t)(t_cs - t_cs_prev);   // 巻き戻しは自然に扱える
            t_cs_prev = t_cs;
            t_ms_unwrapped += (uint32_t)d * 10u;
        }
        return t_ms_unwrapped;
    }
    uint32_t t_ms_unwrapped = 0;
    uint16_t t_cs_prev = 0;
    bool     t_init = false;
    bool     seq_init = false;
    uint8_t  prev_seq = 0;

    static inline int flg(uint16_t f, uint16_t m) { return (f & m) ? 1 : 0; }

    void emitIfOn(uint8_t seq, int fresh, uint16_t t_cs) {
        const uint32_t t = unwrapTime(t_cs);   // 巻き戻し展開は CSV OFF でも進める
        if (csv_on) emitData(last_rx_ms, t, seq, fresh);
    }

    void emitParam() const {
        const S5T::ParamFrame& p = param;
        Serial.printf("PARAM,ver=%u,alt_en=%d,dry=%d,sonar=%d,stick_vz=%d,"
                      "flow_vel_kp=%.3f,flow_vel_ki=%.3f,flow_vel_kd=%.3f,flow_pos_kp=%.3f,"
                      "alt_pos_kp=%.3f,alt_rate_kp=%.3f,alt_rate_ki=%.3f,alt_rate_kd=%.3f,"
                      "alt_hover_thr=%.3f,alt_target_m=%.3f,flow_max_lean=%.3f,alt_thr_auth=%.3f\n",
                      (unsigned)p.ver,
                      flg(p.cfg_flags, S5T::PF_ALT_HOLD_EN),
                      flg(p.cfg_flags, S5T::PF_DRY_RUN),
                      flg(p.cfg_flags, S5T::PF_SONAR),
                      flg(p.cfg_flags, S5T::PF_STICK_VZ),
                      p.flow_vel_kp / S5T::SC_GAIN, p.flow_vel_ki / S5T::SC_GAIN,
                      p.flow_vel_kd / S5T::SC_GAIN, p.flow_pos_kp / S5T::SC_GAIN,
                      p.alt_pos_kp  / S5T::SC_GAIN, p.alt_rate_kp / S5T::SC_GAIN,
                      p.alt_rate_ki / S5T::SC_GAIN, p.alt_rate_kd / S5T::SC_GAIN,
                      p.alt_hover_thr / S5T::SC_GAIN, p.alt_target_m / S5T::SC_GAIN,
                      p.flow_max_lean / S5T::SC_GAIN, p.alt_thr_auth / S5T::SC_GAIN);
    }

    // 量子化を物理値へ戻して 1 行書く。CSV_HEADER と同じ順であること。
    //  fresh: 今回どのフレームが届いたか (0:A 1:B 2:C 3:D)。他は前回値。
    void emitData(uint32_t rx_ms, uint32_t t_ms, uint8_t seq, int fresh) const {
        const S5T::AltFrame& a = alt;
        const S5T::PosFrame& b = pos;
        const S5T::AttFrame& c = att;
        const S5T::DvFrame&  d = dv;
        const uint16_t f = live_h.flags;
        const uint8_t  m = live_modes;

        Serial.printf(
            "DATA,%lu,%lu,%u,%lu,%d,%d,"
            "%u,%u,%d,%d,%d,%d,"
            "%d,%d,%d,%d,%d,%d,%d,"
            "%d,%d,%d,%d,%d,"
            "%.3f,%u,"
            "%.2f,%.2f,%.1f,"
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,"
            "%.3f,%.3f,%.3f,%.3f,"
            "%.2f,%.2f,%.2f,%.2f,"
            "%.2f,%.2f,"
            "%.3f,%.3f,%.3f,%.3f,%u,"
            "%.1f,%.1f,%.1f,%.1f,%.1f,"
            "%.4f,%.4f,%.2f,%.2f,"
            "%.3f,%.3f,%.1f,"
            "%.2f,%u,%u,%u,%u,%d\n",
            (unsigned long)rx_ms, (unsigned long)t_ms, (unsigned)seq,
            (unsigned long)n_lost, last_rssi, fresh,
            (unsigned)S5T::unpackMode(m), (unsigned)S5T::unpackAltState(m),
            flg(f, S5T::F_ARMED), flg(f, S5T::F_FLOW_OK),
            flg(f, S5T::F_RANGE_OK), flg(f, S5T::F_RANGE_VALID),
            flg(f, S5T::F_ALT_EN), flg(f, S5T::F_ALT_ACT),
            flg(f, S5T::F_POS_HOLD), flg(f, S5T::F_AIRBORNE),
            flg(f, S5T::F_DRY_RUN), flg(f, S5T::F_SAT), flg(f, S5T::F_TX_DROP),
            flg(f, S5T::F_GUIDED), flg(f, S5T::F_CMD_FRESH), flg(f, S5T::F_LANDED),
            flg(f, S5T::F_MANEUVER), flg(f, S5T::F_FRAME_OK),
            a.thr / 250.0f, (unsigned)b.bad,
            a.roll_cd / S5T::SC_CDEG, a.pitch_cd / S5T::SC_CDEG,
            a.yaw_dd / S5T::SC_DDEG,
            a.range_h_mm / S5T::SC_MM, a.range_raw_mm / S5T::SC_MM,
            a.alt_hold_mm / S5T::SC_MM, a.climb_mmps / S5T::SC_MM,
            a.alt_vz_tar_mmps / S5T::SC_MM,
            a.alt_thr_corr / S5T::SC_1E4, a.alt_thr_out / S5T::SC_1E4,
            b.vx_mmps / S5T::SC_MM, b.vy_mmps / S5T::SC_MM,
            b.vx_tar_mmps / S5T::SC_MM, b.vy_tar_mmps / S5T::SC_MM,
            b.pos_n_cm / S5T::SC_CM, b.pos_e_cm / S5T::SC_CM,
            b.hold_n_cm / S5T::SC_CM, b.hold_e_cm / S5T::SC_CM,
            b.lean_roll_cd / S5T::SC_CDEG, b.lean_pitch_cd / S5T::SC_CDEG,
            c.m1 / 250.0f, c.m2 / 250.0f, c.m3 / 250.0f, c.m4 / 250.0f,
            (unsigned)c.sat,
            c.roll_rate_dd / S5T::SC_DDEG, c.pitch_rate_dd / S5T::SC_DDEG,
            c.yaw_rate_dd / S5T::SC_DDEG,
            c.roll_rate_tar_dd / S5T::SC_DDEG, c.pitch_rate_tar_dd / S5T::SC_DDEG,
            c.roll_cmd / S5T::SC_1E4, c.pitch_cmd / S5T::SC_1E4,
            c.roll_stick / S5T::SC_STICK, c.pitch_stick / S5T::SC_STICK,
            d.dvx_mmps / S5T::SC_MM, d.dvy_mmps / S5T::SC_MM,
            d.yaw_dd / S5T::SC_DDEG,
            // 未受信 (0xFFFF) は -1。0.00 と並ぶと「たった今届いた」と読み違える
            (d.cmd_age_cs == 0xFFFFu) ? -1.0f : (d.cmd_age_cs / 100.0f),
            (unsigned)d.cmd_good, (unsigned)d.cmd_lost, (unsigned)d.cmd_bad,
            (unsigned)d.cmd_seq, (int)d.cmd_rssi);
    }
};

} // namespace Ground
