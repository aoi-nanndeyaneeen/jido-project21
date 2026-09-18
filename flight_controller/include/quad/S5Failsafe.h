// ============================================================
//  S5Failsafe.h  -  「ループは回っているのに入力が死んでいる」ときの安全装置
// ============================================================
//  ★ 2026-09-17 LOG0057: 接地の衝撃 (accz -4.65g) の直後から推力が固定され、プロポも
//    THR_CUT も効かなくなった。ウォッチドッグ (FcWatchdog.h) はループ自体が止まらないと
//    働かないので、次の 2 つの穴を塞ぐ:
//
//    1) SBUS の途絶判定がループ回数 (connection_fail > 3000) だった。1000Hz 前提の 3 秒が
//       RP2040 (~550Hz) では 5〜6 秒。その間、最後に受けたスティックとスイッチ (アーム)
//       のまま飛び続ける。受信機そのものが止まる (電源瞬断・SBUS 線の接触) と
//       フェイルセーフのフラグも来ないので、THR_CUT もプロポ OFF も届かない。
//       → 最後のフレームからの「時間」で判定する (SBUS_LOST_MS)。
//    2) IMU の読み出し失敗 (I2C) を見ていなかった。getMotion6 が失敗すると前回の生データが
//       残り、姿勢推定は同じ角速度を積分し続ける。
//       → 6 軸の生データが完全に同じまま IMU_FROZEN_MS 続いたら固まったとみなす。
//
//  動作 (アーム中だけ):
//    IMU 固まり                      → FS_CUT  (即モーター停止。姿勢が分からないので着陸もできない)
//    SBUS 途絶 + 高度ホールド/フロー生存 → FS_LAND (その場で自動着陸。スティックは中立扱い)
//    SBUS 途絶 + それ以外 (ANGLE など)  → FS_CUT
//    FS_LAND 中に SBUS が SBUS_RECOVER_MS 続けて戻る → FS_NONE (操縦に戻す)
//    FS_LAND で接地 / 時間上限 / 高度ホールドが外れた → FS_CUT
//    FS_CUT の解除: SBUS が来ていて、IMU が動いていて、THR_CUT がカット側
//                   (= パイロットが一度切った)。解除後のアームは通常どおり。
//
//  ★ 受信機のフェイルセーフフラグ (プロポ OFF / 電波切れを受信機が知らせる) は従来どおり
//    isArmed() で即ディスアーム (パイロットのキル操作として使われているため変えない)。
//  ★ 状態は Vehicle::fs。isArmed() は FS_CUT なら false を返す。呼ぶのはコア0 だけ。
// ============================================================
#pragma once
#include <Arduino.h>
#include "quad/S5Vehicle.h"
#include "quad/SafeLog.h"

namespace S5 {
namespace Failsafe {

inline const char* stateName(uint8_t s) {
    switch (s) {
        case FS_LAND: return "自動着陸";
        case FS_CUT:  return "モーター停止";
        default:      return "-";
    }
}

inline void cut(Vehicle& v, const char* why) {
    if (v.fs != FS_CUT)
        Quad::SafeLog::logf("\n!!!! フェイルセーフ: %s -> モーター停止 (THR_CUT を一度切るまで回しません)\n", why);
    v.fs = FS_CUT;
    v.fs_why = why;
}

// loop() で sbus.update() と IMU 読みの直後に毎回。v.prev_armed は「前のループでアームしていたか」。
inline void update(Vehicle& v) {
    const uint32_t now = millis();
    const bool was_armed   = v.prev_armed;
    const bool frames_lost = USE_SBUS && v.sbus.msSinceFrame() >= Quad::SBUS_LOST_MS;
    const bool imu_frozen  = USE_MPU && v.mpu.frozenMs() >= Quad::IMU_FROZEN_MS;
    if (was_armed && USE_SBUS) {
        const uint32_t gap = v.sbus.msSinceFrame();
        if (gap != 0xFFFFFFFFu && gap > v.sbus_gap_max_ms) v.sbus_gap_max_ms = gap;
    }

    if (imu_frozen && was_armed && v.fs != FS_CUT) cut(v, "IMU の値が固まった (I2C 読み出し失敗)");

    switch (v.fs) {
        case FS_NONE:
            if (frames_lost && was_armed) {
                const bool can_land = v.holdMode() && v.althold.active() && v.althold.airborne()
                                   && v.flowAlive() && v.range.valid;
                if (can_land) {
                    v.fs = FS_LAND;
                    v.fs_why = "SBUS 途絶";
                    v.fs_since_ms = now;
                    v.fs_touch_ms = 0;
                    v.fs_fresh_ms = 0;
                    Quad::SafeLog::logf("\n!!!! フェイルセーフ: SBUS 途絶 %lu ms -> その場で自動着陸\n",
                                        (unsigned long)v.sbus.msSinceFrame());
                } else {
                    cut(v, "SBUS 途絶 (高度ホールド/フローが使えないので着陸できない)");
                }
            }
            break;

        case FS_LAND: {
            // プロポが戻ったら操縦に返す (一瞬の復帰で行ったり来たりしないよう連続で見る)
            if (!frames_lost) {
                if (v.fs_fresh_ms == 0) v.fs_fresh_ms = now;
                if (now - v.fs_fresh_ms >= Quad::SBUS_RECOVER_MS) {
                    v.fs = FS_NONE;
                    Quad::SafeLog::logf("\n>>> フェイルセーフ解除: SBUS 復帰。操縦に戻します\n");
                    break;
                }
            } else {
                v.fs_fresh_ms = 0;
            }
            if (!v.althold.active()) { cut(v, "自動着陸中に高度ホールドが外れた"); break; }
            // 接地判定は GUIDED の自動着陸と同じ
            if (v.range.valid && v.range.h_m > 0.0f && v.range.h_m < Quad::GUIDED_LAND_TOUCH_M) {
                if (v.fs_touch_ms == 0) v.fs_touch_ms = now;
            } else {
                v.fs_touch_ms = 0;
            }
            if (v.fs_touch_ms != 0 && now - v.fs_touch_ms >= Quad::GUIDED_LAND_TOUCH_MS) {
                cut(v, "SBUS 途絶の自動着陸が完了");
            } else if (now - v.fs_since_ms >= Quad::GUIDED_LAND_TIMEOUT_MS) {
                cut(v, "SBUS 途絶の自動着陸が時間上限");
            }
            break;
        }

        case FS_CUT:
            if (!frames_lost && !imu_frozen &&
                v.sbus.Ch_state(Ch::THR_CUT) != Quad::ARM_SWITCH_STATE) {
                v.fs = FS_NONE;
                Quad::SafeLog::logf("\n>>> フェイルセーフ解除 (THR_CUT カット側を確認)\n");
            }
            break;
    }

    // 自動着陸中は最後に届いたスティックを使わない (ロール/ピッチ/ヨーは中立)。
    // スロットルとスイッチは最後の値のまま (高度ホールドの有効条件とモードを保つ)。
    if (v.fs == FS_LAND) {
        v.sbus.des[Ch::ROLL]  = 0.0f;
        v.sbus.des[Ch::PITCH] = 0.0f;
        v.sbus.des[Ch::YAW]   = 0.0f;
    }
}

} // namespace Failsafe
} // namespace S5
