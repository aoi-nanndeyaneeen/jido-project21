// ============================================================
//  StatusView.h  -  人間向けの表示 (CSV を止めているときだけ 1Hz) とヘルプ
// ============================================================
//  ★ ANSI のクリア (\033[2J) は使わない。PlatformIO / VSCode のシリアルモニタでは
//    処理されずに画面が崩れるうえ、崩れた画面と「キーが効いていない」の区別が
//    付かなくなるため。ただ下へ流していく。
// ============================================================
#pragma once
#include <Arduino.h>
#include "S5Telem.h"
#include "S5Cmd.h"
#include "TelemetryStore.h"
#include "Uplink.h"

namespace Ground {

inline int flg(uint16_t f, uint16_t m) { return (f & m) ? 1 : 0; }

inline void printHelp() {
    Serial.println("# --- s5 telemetry receiver (ground station) ---");
    Serial.println("#   1 : CSV 出力 ON    0 : CSV 出力 OFF  (冪等。PC の S5Link / s5_logger.py はこれを使う)");
    Serial.println("#   l : CSV 出力のトグル (人間用)");
    Serial.println("#       ★これは「画面に CSV を吐く」だけ。ファイルを作るのは PC 側");
    Serial.println("#   s : 状態を1回表示");
    Serial.println("#   d : IM920 の生の行をそのまま表示 (リンクの切り分け用)");
    Serial.println("#   z : 統計クリア");
    Serial.println("#   h : このヘルプ");
    Serial.println("#   CMD,req,vx_mmps,vy_mmps,alt_cm,yawrate[,flags[,corrN,corrE[,yawabs[,laps]]]] : 上りコマンド");
    Serial.println("#       req 0=IDLE 1=HOLD 2=TAKEOFF 3=GUIDED 4=LAND 5=ABORT 6=CIRCLE 7=FIGURE8 8=CLIMB");
    Serial.printf ("#       届いた順に無線へ流す (最短 %lu ms 間隔)。%lu ms 来なければ送信停止\n",
                   (unsigned long)CMD_MIN_GAP_MS, (unsigned long)CMD_PC_TIMEOUT_MS);
    Serial.printf ("#   ver=%u  A=%u B=%u D=%u P=%u byte (+checksum4 = %u, IM920sL上限 %u)\n",
                   (unsigned)S5T::VERSION,
                   (unsigned)sizeof(S5T::AltFrame), (unsigned)sizeof(S5T::PosFrame),
                   (unsigned)sizeof(S5T::DvFrame), (unsigned)sizeof(S5T::ParamFrame),
                   (unsigned)(sizeof(S5T::AltFrame) + S5T::CHECKSUM_BYTES),
                   (unsigned)S5T::IM920SL_MAX_PAYLOAD);
}

inline void printStatus(const TelemetryStore& t, const Uplink& up) {
    Serial.println();
    Serial.println("---- s5 TELEMETRY RECEIVER  [l]=CSV出力 [d]=生データ [z]=統計クリア [h]=help ----");
    Serial.printf("link : %s  (最終受信 %lu ms前)  RSSI=%d\n",
                  t.linkOk() ? "OK" : "LOST",
                  (unsigned long)(t.live() ? t.ageMs() : 0), t.last_rssi);
    Serial.printf("IM920 生受信: %lu bytes / %lu 行\n",
                  (unsigned long)t.n_rx_bytes, (unsigned long)t.n_rx_lines);
    Serial.printf("上りCMD: PC行=%lu 送信=%lu 不正=%lu  IM920応答 OK=%lu NG=%lu  %s\n",
                  (unsigned long)up.n_lines, (unsigned long)up.n_tx, (unsigned long)up.n_bad,
                  (unsigned long)t.n_im_ok, (unsigned long)t.n_im_ng,
                  up.have() ? "(送信中)" : "(PC からの指令なし)");
    // 機体が「実際に受け取った」と言っている数。上の 送信= と並べて読む。
    //  送信 >> 機体good なら電波で落ちている (半二重の取り合いを疑う)。
    //  送信 == 機体good なのに遅いなら、遅れは下り側か PC 側にある。
    if (t.have_dv) {
        char age_s[16];
        if (t.dv.cmd_age_cs == 0xFFFFu) strcpy(age_s, "未受信");
        else snprintf(age_s, sizeof(age_s), "%.2fs", t.dv.cmd_age_cs / 100.0f);
        Serial.printf("機体側の上り: good=%u lost=%u bad=%u seq=%u age=%s RSSI=%d\n",
                      (unsigned)t.dv.cmd_good, (unsigned)t.dv.cmd_lost,
                      (unsigned)t.dv.cmd_bad, (unsigned)t.dv.cmd_seq,
                      age_s, (int)t.dv.cmd_rssi);
    }
    Serial.printf("stats: A=%lu B=%lu C=%lu D=%lu P=%lu  lost=%lu  badcs=%lu badlen=%lu",
                  (unsigned long)t.n_alt, (unsigned long)t.n_pos, (unsigned long)t.n_att,
                  (unsigned long)t.n_dv, (unsigned long)t.n_param, (unsigned long)t.n_lost,
                  (unsigned long)t.n_bad_cs, (unsigned long)t.n_bad_len);
    const uint32_t tot = t.n_alt + t.n_pos + t.n_att + t.n_dv + t.n_param + t.n_lost;
    if (tot) Serial.printf("   欠落率 %.1f%%", 100.0f * (float)t.n_lost / (float)tot);
    Serial.println();

    if (!t.live()) {
        Serial.println("まだ1パケットも受信していません。");
        if (t.n_rx_bytes == 0) {
            Serial.println("  IM920 から1バイトも来ていません。無線ではなく配線側の問題です:");
            Serial.println("   ・IM920 の TXD が D7 に来ているか (GroundConfig.h の PIN_XIAO_RX)");
            Serial.println("   ・IM920 の電源は 3V3 か (5Vピンだと書き込みも不安定になる)");
            Serial.println("   ・ボーレート 19200 か");
        } else {
            Serial.println("  バイトは来ているのでリンク自体は生きています。");
            Serial.println("   ・GN(グループ番号)/CH/ホップ(ENHP/DSHP) が両機で同じか (RPRM)");
            Serial.println("   ・[d] で生の行を見て、中身が想定どおりか確認する");
            Serial.println("   ・badlen が増える = protocol/S5Telem.h が機体側とずれている");
        }
        return;
    }

    const uint16_t f = t.live_h.flags;
    const uint8_t  m = t.live_modes;
    Serial.printf("\nMODE=%-8s  ALT=%-12s  %s%s%s\n",
                  S5T::modeName(S5T::unpackMode(m)),
                  S5T::altStateName(S5T::unpackAltState(m)),
                  flg(f, S5T::F_ARMED) ? "ARMED " : "DISARMED ",
                  flg(f, S5T::F_DRY_RUN) ? "[DRY-RUN] " : "",
                  flg(f, S5T::F_SAT) ? "[MIX-SAT] " : "");

    if (t.have_alt) {
        const S5T::AltFrame& a = t.alt;
        Serial.printf("att  : roll=%+7.2f pitch=%+7.2f yaw=%+7.1f [deg]   thr_stick=%.3f\n",
                      a.roll_cd / S5T::SC_CDEG, a.pitch_cd / S5T::SC_CDEG,
                      a.yaw_dd / S5T::SC_DDEG, a.thr / 250.0f);
        Serial.println("[高度ホールド]");
        Serial.printf("  h=%.3f m (生 %.3f)  target=%.3f m  err=%+.3f m   range %s\n",
                      a.range_h_mm / S5T::SC_MM, a.range_raw_mm / S5T::SC_MM,
                      a.alt_hold_mm / S5T::SC_MM,
                      (a.alt_hold_mm - a.range_h_mm) / S5T::SC_MM,
                      flg(f, S5T::F_RANGE_VALID) ? "valid" : "LOST");
        Serial.printf("  vz  : 実測 %+.3f / 目標 %+.3f [m/s]\n",
                      a.climb_mmps / S5T::SC_MM, a.alt_vz_tar_mmps / S5T::SC_MM);
        Serial.printf("  thr : out=%.3f  corr=%+.4f   %s\n",
                      a.alt_thr_out / S5T::SC_1E4, a.alt_thr_corr / S5T::SC_1E4,
                      flg(f, S5T::F_ALT_ACT) ? "ACTIVE" : "(手動)");
    }

    if (t.have_pos) {
        const S5T::PosFrame& b = t.pos;
        Serial.println("[水平位置ホールド]");
        Serial.printf("  v   : 実測 vx=%+.3f vy=%+.3f / 目標 vx=%+.3f vy=%+.3f [m/s]\n",
                      b.vx_mmps / S5T::SC_MM, b.vy_mmps / S5T::SC_MM,
                      b.vx_tar_mmps / S5T::SC_MM, b.vy_tar_mmps / S5T::SC_MM);
        const float dn = (b.pos_n_cm - b.hold_n_cm) / S5T::SC_CM;
        const float de = (b.pos_e_cm - b.hold_e_cm) / S5T::SC_CM;
        Serial.printf("  pos : N=%+.2f E=%+.2f / hold N=%+.2f E=%+.2f [m]  ずれ %.2f m\n",
                      b.pos_n_cm / S5T::SC_CM, b.pos_e_cm / S5T::SC_CM,
                      b.hold_n_cm / S5T::SC_CM, b.hold_e_cm / S5T::SC_CM,
                      sqrtf(dn * dn + de * de));
        Serial.printf("  lean: roll=%+.2f pitch=%+.2f [deg]  bad=%u  %s\n",
                      b.lean_roll_cd / S5T::SC_CDEG, b.lean_pitch_cd / S5T::SC_CDEG,
                      (unsigned)b.bad,
                      flg(f, S5T::F_POS_HOLD) ? "HOLDING" : "(スティック操作中)");
    }

    if (t.have_att) {
        const S5T::AttFrame& c = t.att;
        Serial.println("[姿勢ループ]");
        Serial.printf("  motor: M1=%.3f M2=%.3f M3=%.3f M4=%.3f   飽和=%c%c%c%c\n",
                      c.m1 / 250.0f, c.m2 / 250.0f, c.m3 / 250.0f, c.m4 / 250.0f,
                      (c.sat & 1) ? '1' : '-', (c.sat & 2) ? '2' : '-',
                      (c.sat & 4) ? '3' : '-', (c.sat & 8) ? '4' : '-');
        Serial.printf("  rate : 実測 r=%+7.1f p=%+7.1f y=%+7.1f / 目標 r=%+7.1f p=%+7.1f [deg/s]\n",
                      c.roll_rate_dd / S5T::SC_DDEG, c.pitch_rate_dd / S5T::SC_DDEG,
                      c.yaw_rate_dd / S5T::SC_DDEG,
                      c.roll_rate_tar_dd / S5T::SC_DDEG,
                      c.pitch_rate_tar_dd / S5T::SC_DDEG);
        Serial.printf("  cmd  : roll=%+.4f pitch=%+.4f\n",
                      c.roll_cmd / S5T::SC_1E4, c.pitch_cmd / S5T::SC_1E4);
        // 中立のはずなのに 0 でなければプロポのトリムずれ (角度ループは ×MAX_ANGLE を目標にする)
        const float rs = c.roll_stick / S5T::SC_STICK;
        const float ps = c.pitch_stick / S5T::SC_STICK;
        Serial.printf("  stick: roll=%+.2f pitch=%+.2f  (目標角 %+.1f / %+.1f deg)%s\n",
                      rs, ps, rs * 30.0f, ps * 30.0f,
                      (fabsf(rs) > 0.03f || fabsf(ps) > 0.03f)
                        ? "  ★中立でないならトリムずれ" : "");
    }

    if (t.have_dv) {
        Serial.printf("[ヨー推定用Δv] dvx=%+.3f dvy=%+.3f [m/s]  yaw=%+.1f [deg]\n",
                      t.dv.dvx_mmps / S5T::SC_MM, t.dv.dvy_mmps / S5T::SC_MM,
                      t.dv.yaw_dd / S5T::SC_DDEG);
    }

    if (t.have_param) {
        const S5T::ParamFrame& p = t.param;
        Serial.println("[ゲイン (機体から受信)]");
        Serial.printf("  flow vel P=%.3f I=%.3f D=%.3f   flow pos P=%.3f\n",
                      p.flow_vel_kp / S5T::SC_GAIN, p.flow_vel_ki / S5T::SC_GAIN,
                      p.flow_vel_kd / S5T::SC_GAIN, p.flow_pos_kp / S5T::SC_GAIN);
        Serial.printf("  alt  pos P=%.3f   alt rate P=%.3f I=%.3f D=%.3f  hover=%.3f\n",
                      p.alt_pos_kp  / S5T::SC_GAIN, p.alt_rate_kp / S5T::SC_GAIN,
                      p.alt_rate_ki / S5T::SC_GAIN, p.alt_rate_kd / S5T::SC_GAIN,
                      p.alt_hover_thr / S5T::SC_GAIN);
    }
}

} // namespace Ground
