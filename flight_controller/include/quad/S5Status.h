// ============================================================
//  S5Status.h  -  USB シリアルのデバッグ画面 (DEBUG_HZ = 10Hz)
// ============================================================
//  drone_s5.cpp § 9 printStatus() をそのまま移したもの。読むだけで
//  状態は変えない。FlightLog::Usb が動いている間は呼ばれない
//  (同じ USB を奪い合うとログが落ちる)。
//  ★ RP2040 ではコア1 から呼ぶ。Vehicle は読むだけにし、状態を書き換える関数
//    (isArmed() など) は呼ばないこと。値はコア0 が書いている最中のものが混ざりうる。
// ============================================================
#pragma once
#include <Arduino.h>
#include "quad/S5Vehicle.h"
#include "quad/SelfTest.h"
#include "quad/SdLog.h"
#include "quad/LogLink.h"
#include "quad/LoopProfile.h"
#include "quad/SafeLog.h"
#include "quad/FcWatchdog.h"

namespace S5 {

inline void printStatus(Vehicle& v, uint32_t dt_us) {
    Serial.print("\033[2J\033[H");
    Serial.println("=== Stage 5d : 1スイッチ完全自動ホバリング ===");
    if (v.dry_run)
        Serial.println(">>> DRY-RUN: ESCへは0のみ (モーターは回らない)  ['m']で解除 <<<");
    Serial.printf("loop dt = %6lu us (%6.1f Hz)   %s   link=%s\n",
                  (unsigned long)dt_us, 1000000.0f / (float)dt_us,
                  v.armed_now ? "ARMED" : "DISARMED",
                  v.sbus.isSafe() ? "OK" : "LOST");
    Serial.printf("MODE = %s   (SW_HOVER: down=ANGLE / cen=POSHOLD / up=GUIDED%s)\n",
                  modeLabel(v.mode),
                  Quad::GUIDED_ENABLE ? " (資格切れ中はPOSHOLD)" : "");
    SelfTest::printCompact(Serial);   // 起動時に何がつながっていたか (画面に残す)
    LoopProfile::print(Serial, LoopProfile::last(), "[ループ 直近1秒]");
    if (LoopProfile::haveFlight())
        LoopProfile::print(Serial, LoopProfile::lastFlight(), "[ループ 前回の飛行]");
    if (v.fs != FS_NONE)
        Serial.printf("!!!! フェイルセーフ: %s (%s) !!!!\n",
                      v.fs == FS_LAND ? "自動着陸中" : "モーター停止", v.fs_why);
    Serial.printf("SBUS 最終フレーム %lu ms 前 (間隔の最大: アーム中 %lu / 前回の飛行 %lu ms)  "
                  "IMU 固まり %lu ms\n",
                  (unsigned long)v.sbus.msSinceFrame(), (unsigned long)v.sbus_gap_max_ms,
                  (unsigned long)v.sbus_gap_last_ms, (unsigned long)v.mpu.frozenMs());
    Serial.printf("I2C: 読み失敗 %lu 回  バス復旧 %u 回  (失敗が増え続ける = IMU の電源/配線)\n",
                  (unsigned long)v.mpu.ioFailTotal(), (unsigned)v.mpu.recoverCount());
    if (v.wdt_rebooted) {
        bool armed; uint8_t fs, mode, gp;
        Serial.printf("!!!! 前回はループ停止でウォッチドッグがリセット: 区間「%s」 起動 %lu ms 後",
                      FcWatchdog::sectionName(FcWatchdog::lastSection()),
                      (unsigned long)FcWatchdog::lastMs());
        if (FcWatchdog::lastState(armed, fs, mode, gp))
            Serial.printf("  (mode=%s guided=%s fs=%u armed=%d)", modeLabel((Mode)mode),
                          Quad::guidedPhaseName((Quad::GuidedPhase)gp), (unsigned)fs, armed ? 1 : 0);
        Serial.println(" !!!!");
    }
    if (Quad::SafeLog::dropped() > 0)
        Serial.printf("(SafeLog 取りこぼし %lu 件)\n", (unsigned long)Quad::SafeLog::dropped());

    if (USE_BLE_LINK) {
        // BLE 経路。テレメトリは LogLink のリングへ積むだけなので、drop が増えるなら
        // リング (8KB) が詰まっている = UART かロガーが止まっている。
        Serial.printf("GROUND LINK: BLE (log_recorder 経由)  TELEM drop=%lu  "
                      "CMD 中継受信=%lu 長さ不正=%lu\n",
                      (unsigned long)LogLink::telemDrops(),
                      (unsigned long)LogLink::cmdRxCount(),
                      (unsigned long)LogLink::cmdBadLen());
    }
    if (USE_IM920) {
        // 下りテレメトリの送信状況。drop が増え続けるなら TELEM_TX_HZ が速すぎる。
        Serial.printf("TELEM tx=%lu drop=%lu %s\n",
                      (unsigned long)v.s5tx.sent(), (unsigned long)v.s5tx.dropped(),
                      v.s5tx.busy() ? "(sending)" : "");
    }
    if (USE_IM920 || USE_BLE_LINK) {

        // ---- 上りコマンド (GUIDED) --------------------------------
        //  ★ ベンチで「地上局のコマンドが届いているか」を確認する唯一の場所。
        //    good が増えないなら無線かパケット定義。badver なら S5Cmd.h が
        //    機体側と地上側でずれている。badcs が増えるなら電波が弱い。
        if (Quad::GUIDED_ENABLE) {
            const S5C::Rx& rx = v.s5rx;
            Serial.printf("CMD   rx: good=%lu lost=%lu badcs=%lu badlen=%lu badver=%lu  "
                          "RSSI=%d %s\n",
                          (unsigned long)rx.nGood(),  (unsigned long)rx.nLost(),
                          (unsigned long)rx.nBadCs(), (unsigned long)rx.nBadLen(),
                          (unsigned long)rx.nBadVer(), rx.rssi(),
                          rx.everReceived() ? "" : "(まだ1つも受信していません)");
            if (rx.everReceived()) {
                const S5C::CmdFrame& c = rx.last();
                Serial.printf("         %lu ms前  req=%-7s vx=%+.3f vy=%+.3f alt=%.2f m "
                              "flags=0x%04X\n",
                              (unsigned long)rx.ageMs(), S5C::reqName(c.req),
                              (float)c.vx_mmps / S5C::SC_MMPS,
                              (float)c.vy_mmps / S5C::SC_MMPS,
                              (float)c.alt_cm  / S5C::SC_CM,
                              (unsigned)c.flags);
            }
            const Quad::Guided& g = v.guided;
            Serial.printf("GUIDED %s  phase=%-7s  目標 vx=%+.3f vy=%+.3f alt=%.2f m "
                          "(slew %.2f m/s)%s%s\n",
                          g.engaged() ? "ENGAGED" : "----   ",
                          Quad::guidedPhaseName(g.phase()), g.vx(), g.vy(), g.altM(), g.slew(),
                          (!g.engaged() && g.why()[0]) ? "  直前の解除理由: " : "",
                          (!g.engaged() && g.why()[0]) ? g.why() : "");
            Serial.printf("  機首 (置いた向きから) %+.1f deg\n", v.heading.sinceArm());
            if (g.phase() == Quad::GP_ALIGN) {
                Serial.printf("  向き合わせ中: 方位 目標 %.1f 実測 %.1f -> %+.1f deg/s\n",
                              g.lineDeg(), v.heading.est(), g.yawRate());
            } else if (g.phase() == Quad::GP_STRAIGHT) {
                const Quad::StraightTrack& st = g.straight();
                Serial.printf("  直進: 目標 %.2f / %.1f m  実測 前 %.2f 横 %+.2f m (最大 %+.2f)  "
                              "%.1f/%.1f s  速さ %.2f m/s  ヨー 目標 %.1f 実測 %.1f -> %+.1f deg/s\n",
                              st.progressRef(), st.distance(), st.along(), st.cross(), st.crossMax(),
                              st.elapsedS(), st.expectS(), st.speed(),
                              st.headingDeg(), v.heading.est(), g.yawRate());
            } else if (g.tracking()) {
                const Quad::CircleTrack& ct = g.circle();
                Serial.printf("  円 %d/%d (%d周, 実測 %.2f周, 高度目標 %.2f m) 進行 目標 %5.1f / 実測 %5.1f deg  "
                              "%.1f/%.1f s  速さ %.2f m/s  "
                              "半径 %.2f m  ヨー 目標 %.1f 実測 %.1f -> %+.1f deg/s\n",
                              g.legIndex() + 1, g.legCount(), ct.laps(), ct.lapsMeas(), g.altM(),
                              ct.progressRefDeg(), ct.progressMeasDeg(), ct.elapsedS(), ct.expectS(),
                              ct.speed(), ct.radiusMeas(), ct.yawRefDeg(), v.heading.est(), g.yawRate());
            }
            Serial.printf("FENCE  %s  (N±%.1f E±%.1f m)%s\n",
                          v.poshold.fenceOn() ? "有効"
                              : (Quad::FENCE_ENABLE ? "待機 (原点未設定: CF_POS_SHIFT 待ち)" : "無効"),
                          Quad::FENCE_N_LIM, Quad::FENCE_E_LIM,
                          v.poshold.fencePush() ? "  ★境界で押し返し中" : "");
        }
    }

    if (v.sd_ok)   SdLog::brief(Serial);
    if (v.link_ok) LogLink::brief(Serial);

    if (v.mode != MODE_RATE) {
        Serial.println("\n[角度ループ]  目標[deg]  実測[deg]  → 角速度目標[deg/s]");
        Serial.printf("  roll  %10.1f %10.1f %18.1f\n",
                      v.roll_axis.ang_tar, v.roll_axis.ang_meas, v.roll_axis.rate_tar);
        Serial.printf("  pitch %10.1f %10.1f %18.1f\n",
                      v.pitch_axis.ang_tar, v.pitch_axis.ang_meas, v.pitch_axis.rate_tar);
    }

    if (v.mode == MODE_POSHOLD) {
        const Quad::PositionHold& ph = v.poshold;
        Serial.printf("\n[フロー位置ホールド] %s  vel P=%.2f I=%.2f  pos P=%.2f  失探=%d\n",
                      ph.holding() ? "HOLD " : "STICK",
                      v.flow_vel_kp, v.flow_vel_ki, v.flow_pos_kp, ph.badCount());
        Serial.printf("  速度[m/s]  実測(前,右)=%+6.2f %+6.2f   目標(前,右)=%+6.2f %+6.2f\n",
                      ph.vxCtl(), ph.vyCtl(), ph.vxTar(), ph.vyTar());
        Serial.printf("  位置[m]    現在(N,E)=%+6.2f %+6.2f   保持(N,E)=%+6.2f %+6.2f"
                      "   (地面固定, ψ=%+.1fdeg)\n",
                      ph.posN(), ph.posE(), ph.holdN(), ph.holdE(), v.heading.est());
        Serial.printf("  → 目標リーン角[deg]  roll=%+5.1f  pitch=%+5.1f\n",
                      ph.leanRoll(), ph.leanPitch());
    }

    // --- フローセンサの生死 (常時表示) ---
    //  以前は POSHOLD のときしか出ず、「フローが死んでいるので POSHOLD に入れない」
    //  状態だと画面のどこにも出なかった。常に出す。
    if (USE_FLOW) {
        Serial.printf("\n[フロー] %s  窓カウント(%.0f,%.0f)  SQUAL=%u(床%u)  "
                      "0連続=%.1fs 低品質=%.1fs  h=%.2fm  (読%dHz/制御%dHz)\n",
                      !v.flowobs.ok         ? "FAIL(起動時に応答なし)"
                    : v.flow.suspectDead()  ? "凍結? (SQUAL床下 or 窓カウント0が継続)"
                                            : "OK  ",
                      v.flowobs.raw_x, v.flowobs.raw_y, v.flow.squal(), (unsigned)Quad::FLOW_SQUAL_MIN,
                      v.flow.zeroRunS(), v.flow.lowQualS(), v.flow.height(),
                      Quad::FLOW_LOOP_HZ, Quad::FLOW_CTRL_HZ);
    }

    // --- 距離センサ + 高度ホールド ---
    if (USE_RANGE) {
        const Quad::AltitudeHold& ah = v.althold;
        Serial.printf("\n[距離:%s] %s  斜め=%.2fm  → 鉛直h=%.2fm  上昇=%+.2fm/s  "
                      "飛びで同期し直し %u 回\n",
                      Quad::RANGE_INFO.name,
                      !v.range.ok ? "FAIL " : (v.rangefinder.stepRejecting() ? "飛び?"
                                              : (v.range.valid ? "OK   " : "失探 ")),
                      v.range.raw_m, v.range.h_m, v.range.climb_mps,
                      (unsigned)v.rangefinder.stepCount());
        // ★ 失探の理由。io が増えるなら I2C が通っていない (配線/接触不良/電源) で、
        //   センサが 0mm や範囲外を返しているのとは原因が別物。
        Serial.printf("  捨てた内訳: I2C失敗=%lu  0mm=%lu  範囲外=%lu  レンジ外=%lu  傾き=%lu\n",
                      (unsigned long)v.rangefinder.ioFailCount(),
                      (unsigned long)v.rangefinder.zeroCount(),
                      (unsigned long)v.rangefinder.oorCount(),
                      (unsigned long)v.rangefinder.rangeCount(),
                      (unsigned long)v.rangefinder.tiltCount());
        Serial.printf("[高度ホールド] %s  %s  hold=%.2fm  vz_tar=%+.2fm/s  "
                      "base=%.2f corr=%+.3f → thr=%.2f\n",
                      v.alt_hold_enable ? "ENABLED" : "OFF(手動)",
                      ah.stateName(), ah.holdM(), ah.vzTar(),
                      ah.thrBase(), ah.thrCorr(), ah.thrOut());
        Serial.printf("  base=ALT_HOVER_THR(%.2f)固定  スティックvz=%s  "
                      "離陸=%s (engage時h=%.2fm)  → スロットルは高度のみで決まる\n",
                      Quad::ALT_HOVER_THR,
                      Quad::ALT_STICK_VZ_ENABLE ? "有効" : "無効",
                      ah.airborne() ? "検知済" : "未検知(水平ホールド待機)",
                      ah.engageH());
        Serial.printf("  gains: pos P=%.2f  rate P=%.2f I=%.2f D=%.3f   ['g']切替  ['p']-[s..v]調整\n",
                      ah.posKp(), ah.ratePid().kp(), ah.ratePid().ki(), ah.ratePid().kd());
    }

    Serial.println("\n[レートループ] 目標[deg/s] 実測[deg/s]      cmd      I項");
    Serial.printf("  roll  %11.1f %11.1f %9.4f %8.4f\n",
                  v.roll_axis.rate_tar, v.roll_axis.rate_meas,
                  v.roll_axis.cmd, v.roll_axis.rate.i_term());
    Serial.printf("  pitch %11.1f %11.1f %9.4f %8.4f\n",
                  v.pitch_axis.rate_tar, v.pitch_axis.rate_meas,
                  v.pitch_axis.cmd, v.pitch_axis.rate.i_term());
    Serial.printf("  yaw   %11.1f %11.1f %9.4f %8.4f\n",
                  v.yaw_axis.rate_tar, v.yaw_axis.rate_meas,
                  v.yaw_axis.cmd, v.yaw_axis.rate.i_term());

    // --- ヘディングホールド ---
    //  err がじわじわ片側に増え続けるなら、機体が回っているのではなく
    //  ジャイロZのバイアスが残っている (= [k] で再キャリブレーションが必要)。
    Serial.printf("\n[ヨー保持] %s  kp=%.2f  目標%+8.2f  推定%+8.2f  誤差%+7.2f [deg]\n",
                  v.heading.holding() ? "HOLD  " : "STICK ",
                  v.heading.kp(), v.heading.hold(), v.heading.est(), v.heading.error());

    Serial.printf("\n  attitude roll=%+7.2f pitch=%+7.2f [deg]  thr=%.2f\n",
                  v.att.roll, v.att.pitch, v.sbus.des[Ch::THR]);

    Serial.println("\n[モーター]  M1=左前 M2=右前 M3=右後 M4=左後");
    for (int i = 0; i < Quad::MOTOR_COUNT; ++i) {
        Serial.printf("  M%d %5.3f  ", i + 1, v.out[i]);
        const int bar = (int)(v.out[i] * 40.0f);
        for (int j = 0; j < bar; ++j) Serial.print('#');
        Serial.println();
    }

    Serial.printf("\n[ミキサー] span_limit=%.3f  scale=%.3f  sat=0b%c%c%c%c (M4..M1)\n",
                  v.mix.span_limit, v.mix.scale,
                  (v.mix.sat & 8) ? '1' : '0', (v.mix.sat & 4) ? '1' : '0',
                  (v.mix.sat & 2) ? '1' : '0', (v.mix.sat & 1) ? '1' : '0');

    // --- オプティカルフロー (観測値とキャリブ積算) ---
    //  ① 符号/軸: 機体を「ゆっくり大きく」平行移動させて acc_px を見る
    //     ・前へ動かす → x が一方向に伸びる (逆なら FLOW_SIGN_X=-1)
    //     ・右へ動かす → y                  (逆なら FLOW_SIGN_Y=-1)
    //     ・前後で y / 左右で x が伸びる     → FLOW_SWAP_XY=true
    //  ③ スケール ([z]でゼロ → 高さ h で距離 D をスライド → acc_px を読む):
    //        FLOW_PX_PER_RAD = (acc_px ÷ D) × h     ← QuadConfig.h へ
    //  ② de-rotation: 位置固定で向きだけ傾ける。derot が ~0 なら OK。
    //     raw と逆向きに振れる → FLOW_DEROT_SIGN_* を反転。
    if (USE_FLOW) {
        const FlowObs& fo = v.flowobs;
        Serial.printf("\n[フロー] %s  h=%.2fm  窓raw(x,y)=%+6.1f %+6.1f  "
                      "derot(x,y)=%+6.1f %+6.1f  SQUAL=%u\n",
                      fo.ok ? "OK  " : "FAIL", v.flow.height(),
                      fo.raw_x, fo.raw_y, fo.dx, fo.dy, v.flow.squal());
        Serial.printf("         v=%+6.2f %+6.2f m/s (LPF %+6.2f %+6.2f)   [z]ゼロ\n",
                      fo.vx, fo.vy, fo.vx_f, fo.vy_f);
        Serial.printf("         積算 raw=(%+9.0f,%+9.0f)  derot=(%+9.0f,%+9.0f)  "
                      "m=(%+7.3f,%+7.3f)\n",
                      fo.acc_raw_x, fo.acc_raw_y, fo.acc_px_x, fo.acc_px_y,
                      fo.acc_m_x, fo.acc_m_y);
    }

    Serial.println("\n[p]ゲイン [k]IMUキャリブ(EEPROM保存) [x]キャリブ消去 [r]PIDリセット "
                   "[l]ログ(USB直結時) [n]RAMログ手動トリガ [v]RAMログdump [y]RAMログ状態 "
                   "[w]停止調査ログdump "
                   "[z]フロー積算ゼロ "
                   "[h]フロー高度(手動) [g]高度ホールド切替 [i]I2Cスキャン [m]ドライラン切替 "
                   "[s]SD状態/単体テスト [d]起動時デバイスチェック再表示");
}

} // namespace S5
