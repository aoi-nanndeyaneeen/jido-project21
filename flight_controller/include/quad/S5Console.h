// ============================================================
//  S5Console.h  -  USB シリアルのキー操作 / ゲイン調整メニュー /
//                  単発メンテナンス指令 (BLE 経由) の実処理
// ============================================================
//  drone_s5.cpp § 8 にあった handleSerial() / tuningMenu() / i2cScan() /
//  runMaintenanceAction() / handleBleAction() をまとめたもの。
//  制御ループには一切関与しない (呼ぶのは loop() の末尾、毎ループ 1 回)。
//
//  ★ tuningMenu() はブロッキング (キー入力待ち)。アーム中は入れない
//    (stopAllMotors した直後に制御ループが止まって落下する。実測 5.86 秒)。
// ============================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "quad/S5Vehicle.h"
#include "quad/FlightLog.h"
#include "quad/SdLog.h"
#include "quad/LogLink.h"
#include "quad/LogLinkProto.h"
#include "quad/SelfTest.h"
#include "quad/StallLog.h"

namespace S5 {
namespace Console {

// ------------------------------------------------------------
//  I2C バススキャン。0x08..0x77 を叩いて ACK を返したアドレスを列挙する。
//   期待値:  0x29 = VL53L1X (距離)   0x68 = MPU6050 (IMU)
//   0x68 だけ見えて 0x29 が見えない → VL53L1X が配線に乗れていない
//   両方見えない → バスが Low に張り付いている (モジュール故障 or 電源ショート)
//  戻り値: 見つかったデバイス数 (BLE の ActAck に載せる)
// ------------------------------------------------------------
inline int i2cScan() {
    Serial.println("\n--- I2C scan (Wire: SDA=18 / SCL=19) ---");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        Wire.beginTransmission(addr);
        const uint8_t err = Wire.endTransmission();
        if (err == 0) {
            const char *tag = (addr == 0x29) ? "  <- VL53L1X (距離)"
                            : (addr == 0x68) ? "  <- MPU6050 (IMU)"
                                             : "";
            Serial.printf("  0x%02X  ACK%s\n", addr, tag);
            ++found;
        }
    }
    if (found == 0)
        Serial.println("  応答なし。SDA/SCL が Low 固着 or 全モジュール断線。");
    Serial.printf("--- %d device(s) ---\n", found);
    return found;
}

// ------------------------------------------------------------
//  ゲイン調整メニュー ('p')。ブロッキング。
// ------------------------------------------------------------
inline void tuningMenu(Vehicle& v) {
    stopAllMotors(v);
    resetControllers(v);

    const unsigned long old_timeout = Serial.getTimeout();
    Serial.setTimeout(30000);
    while (Serial.available()) Serial.read();

    Quad::Axis& ra = v.roll_axis;
    Quad::Axis& pa = v.pitch_axis;
    Quad::Axis& ya = v.yaw_axis;

    Serial.println("\n\n!! モーターを停止しました。制御ループも止まっています !!");
    Serial.println("========== PID Tuning ==========");
    Serial.println("-- Rate (内側) --");
    Serial.printf(" [1] Roll  P %9.5f  [2] Roll  I %9.5f  [3] Roll  D %9.5f\n",
                  ra.rate.kp(), ra.rate.ki(), ra.rate.kd());
    Serial.printf(" [4] Pitch P %9.5f  [5] Pitch I %9.5f  [6] Pitch D %9.5f\n",
                  pa.rate.kp(), pa.rate.ki(), pa.rate.kd());
    Serial.printf(" [7] Yaw   P %9.5f  [8] Yaw   I %9.5f  [9] Yaw   D %9.5f\n",
                  ya.rate.kp(), ya.rate.ki(), ya.rate.kd());
    Serial.println("-- Angle (外側) --");
    Serial.printf(" [a] Roll  P %9.4f  [b] Roll  I %9.4f  [c] Roll  D %9.4f\n",
                  ra.angle.kp(), ra.angle.ki(), ra.angle.kd());
    Serial.printf(" [d] Pitch P %9.4f  [e] Pitch I %9.4f  [f] Pitch D %9.4f\n",
                  pa.angle.kp(), pa.angle.ki(), pa.angle.kd());
    Serial.println("-- ヘディングホールド (ヨー) --");
    Serial.printf(" [h] Hold  P %9.4f   (0 にすると保持を切って従来のレートのみになる)\n",
                  v.heading.kp());
    Serial.println("-- s5b フロー位置ホールド --");
    Serial.printf(" [i] Vel P %9.4f  [j] Vel I %9.4f  [o] Pos P %9.4f\n",
                  v.flow_vel_kp, v.flow_vel_ki, v.flow_pos_kp);
    Serial.println("-- s5c 高度ホールド --");
    Serial.printf(" [s] Pos P %9.4f  [t] Rate P %9.4f  [u] Rate I %9.4f  [v] Rate D %9.5f\n",
                  v.althold.posKp(), v.althold.ratePid().kp(),
                  v.althold.ratePid().ki(), v.althold.ratePid().kd());
    Serial.println(" [q] 抜ける");
    Serial.print("選択 > ");

    while (!Serial.available()) { /* 入力待ち */ }
    String sel = Serial.readStringUntil('\n');
    sel.trim();
    sel.toLowerCase();

    auto resume = [&]() {
        resetControllers(v);
        Serial.setTimeout(old_timeout);
        Serial.println("再開します。");
    };

    if (sel.length() == 0 || sel[0] == 'q') { Serial.println(); resume(); return; }

    Serial.print("新しい値 > ");
    const float val = Serial.parseFloat();
    Serial.println(val, 5);

    // ヘディングホールドは Pid クラスではないので先に処理する
    if (sel[0] == 'h') {
        v.heading.setKp(constrain(val, 0.0f, 20.0f));
        Serial.printf("更新: ヘディングホールド kp=%.4f\n", v.heading.kp());
        resume(); return;
    }

    // フロー位置ホールドのゲイン (vx/vy 両方に同じ値を入れる)
    if (sel[0] == 'i' || sel[0] == 'j' || sel[0] == 'o') {
        if (sel[0] == 'i') v.flow_vel_kp = constrain(val, 0.0f, 40.0f);
        if (sel[0] == 'j') v.flow_vel_ki = constrain(val, 0.0f, 20.0f);
        if (sel[0] == 'o') v.flow_pos_kp = constrain(val, 0.0f, 5.0f);
        v.poshold.setVelGains(v.flow_vel_kp, v.flow_vel_ki, Quad::FLOW_VEL_KD);
        v.poshold.setPosKp(v.flow_pos_kp);
        Serial.printf("更新: Flow vel P=%.3f I=%.3f  pos P=%.3f\n",
                      v.flow_vel_kp, v.flow_vel_ki, v.flow_pos_kp);
        resume(); return;
    }

    // 高度ホールドのゲイン
    if (sel[0] == 's' || sel[0] == 't' || sel[0] == 'u' || sel[0] == 'v') {
        Quad::Pid& arp = v.althold.ratePid();
        if (sel[0] == 's') v.althold.setPosKp(constrain(val, 0.0f, 100.0f));
        // rate ゲインは上限クランプなし (負値だけ弾く。負だと正帰還で即発散)
        if (sel[0] == 't') arp.set_gains(max(val, 0.0f), arp.ki(), arp.kd());
        if (sel[0] == 'u') arp.set_gains(arp.kp(), max(val, 0.0f), arp.kd());
        if (sel[0] == 'v') arp.set_gains(arp.kp(), arp.ki(), max(val, 0.0f));
        Serial.printf("更新: Alt pos P=%.3f  rate P=%.3f I=%.3f D=%.5f\n",
                      v.althold.posKp(), arp.kp(), arp.ki(), arp.kd());
        resume(); return;
    }

    Quad::Pid* pid = nullptr;
    int which = -1;   // 0=P 1=I 2=D
    switch (sel[0]) {
        case '1': pid = &ra.rate;  which = 0; break;
        case '2': pid = &ra.rate;  which = 1; break;
        case '3': pid = &ra.rate;  which = 2; break;
        case '4': pid = &pa.rate;  which = 0; break;
        case '5': pid = &pa.rate;  which = 1; break;
        case '6': pid = &pa.rate;  which = 2; break;
        case '7': pid = &ya.rate;  which = 0; break;
        case '8': pid = &ya.rate;  which = 1; break;
        case '9': pid = &ya.rate;  which = 2; break;
        case 'a': pid = &ra.angle; which = 0; break;
        case 'b': pid = &ra.angle; which = 1; break;
        case 'c': pid = &ra.angle; which = 2; break;
        case 'd': pid = &pa.angle; which = 0; break;
        case 'e': pid = &pa.angle; which = 1; break;
        case 'f': pid = &pa.angle; which = 2; break;
        default: break;
    }

    if (pid) {
        const float p = (which == 0) ? val : pid->kp();
        const float i = (which == 1) ? val : pid->ki();
        const float d = (which == 2) ? val : pid->kd();
        pid->set_gains(p, i, d);
        Serial.printf("更新: P=%.5f I=%.5f D=%.5f\n", p, i, d);
    } else {
        Serial.println("不明な選択です");
    }
    resume();
}

// ------------------------------------------------------------
//  単発メンテナンス指令の実処理 (S5Cmd.h の Action)
// ------------------------------------------------------------
//  呼び出し元は BLE (handleBleAction) だけ (IM920 は操縦専用)。GUIDED の
//  成否ゲートを一切通さない: PID reset / IMU 校正 / デバイス確認は「GUIDED
//  に入れる状態か」と無関係にいつでも試せてよく、むしろ GUIDED に入る前
//  (地上で機体を整えている最中) にこそ使う。
struct ActionResult { uint8_t result; uint8_t imu_ok; uint8_t i2c_found; };

inline ActionResult runMaintenanceAction(Vehicle& v, uint8_t action) {
    switch (action) {
        case S5C::ACT_PID_RESET:
            // シリアル 'r' と同じく非アーム/アーム問わず実行する。積分器を 0 に
            // 戻すだけで、モーターを止めたりしない。
            resetControllers(v);
            Serial.println("\n>>> BLEからの指令: PID reset");
            return {S5T::ACK_OK, 0, 0};

        case S5C::ACT_IMU_CAL: {
            // ★ シリアル 'k' と違い、遠隔操作者は機体に触れていない。飛行中に
            //   誤って送られたら致命的なので「非アーム限定」ゲートを足す。
            if (isArmed(v)) {
                Serial.println("\n!! BLEからの IMU_CAL を拒否: アーム中です");
                return {S5T::ACK_REFUSED_ARMED, 0, 0};
            }
            stopAllMotors(v);
            Serial.println("\n>>> BLEからの指令: CALIBRATE "
                           "(機体を水平に置いて動かさないでください)");
            bool ok = true;
            if (USE_MPU) ok = v.mpu.recalibrate();
            resetControllers(v);
            return {ok ? S5T::ACK_OK : S5T::ACK_CAL_REJECTED, 0, 0};
        }

        case S5C::ACT_SELFTEST: {
            // i2cScan() は ACK の無い addr ごとに Wire のタイムアウト分ループが
            // 伸びうる。1000Hz ループを乱さないよう非アーム限定。
            if (isArmed(v)) {
                Serial.println("\n!! BLEからの SELFTEST を拒否: アーム中です");
                return {S5T::ACK_REFUSED_ARMED, 0, 0};
            }
            Serial.println("\n>>> BLEからの指令: SELFTEST (I2C再走査)");
            const int found = i2cScan();
            const uint8_t imu_ok = (USE_MPU && v.mpu.connected()) ? 1 : 0;
            return {S5T::ACK_OK, imu_ok, (uint8_t)constrain(found, 0, 255)};
        }

        default:
            return {S5T::ACK_OK, 0, 0};   // 呼び出し側で ACT_NONE は弾いている
    }
}

// BLE (log_recorder 経由。デバッグ専用) からの単発指令を 1 つ処理する。
inline void handleBleAction(Vehicle& v) {
    static uint8_t last_seq = 0;
    if (!v.link_ok) return;
    LogLinkProto::ActReq req;
    if (!LogLink::pollAction(req)) return;
    if (req.action == S5C::ACT_NONE) return;
    if (req.action_seq == last_seq) return;   // 重複排除
    last_seq = req.action_seq;

    const ActionResult r = runMaintenanceAction(v, req.action);
    LogLink::sendAck(req.action, req.action_seq, r.result, r.imu_ok, r.i2c_found);
}

// ------------------------------------------------------------
//  USB シリアルの 1 文字キー
// ------------------------------------------------------------
inline void handleSerial(Vehicle& v) {
    if (!Serial.available()) return;
    const char c = (char)tolower(Serial.read());

    switch (c) {
        case 'p':
            // アーム中は入れない (モーター停止 + 制御ループ停止で落下する)
            if (isArmed(v)) {
                Serial.println("\n!! アーム中は 'p' メニューに入れません "
                               "(モーター停止 + 制御ループ停止で落下するため)。"
                               "ディスアームしてから押してください。");
            } else {
                tuningMenu(v);
            }
            break;
        case 'i':
            i2cScan();
            break;
        case 'm':
            v.dry_run = !v.dry_run;
            stopAllMotors(v);
            Serial.printf("\n>>> ドライラン = %s\n",
                          v.dry_run
                            ? "ON  (ESCへは0のみ。g_out は計算/記録。飛行前に必ずOFF)"
                            : "OFF (通常。モーターが回る)");
            break;
        case 'k':
            stopAllMotors(v);
            Serial.println("CALIBRATE: 機体を水平に置いて動かさないでください");
            // 成功したら EEPROM に保存され、次回起動時に自動で読み込まれる。
            if (USE_MPU) v.mpu.recalibrate();
            resetControllers(v);
            break;
        case 'x':
            // EEPROM の保存値を捨てて Config.h のハードコード値に戻す。
            // EEPROM はファーム書き込みでは消えないので、変な値を保存して
            // しまったときの唯一の逃げ道がこれ。
            stopAllMotors(v);
            if (USE_MPU) v.mpu.clearCalibration();
            Serial.println("INFO: 再起動すると Config.h の値に戻ります");
            break;
        case 'r':
            resetControllers(v);
            Serial.println("PID reset");
            break;
        case 'l':
            // 500Hz ログの開始/停止 (USB 直結時のみ)。scripts/logger.py と対で使う。
            FlightLog::Usb::toggle();
            break;
        case 'v':
            FlightLog::Ram::dump();      // RAM ログを USB へダンプ
            break;
        case 'y':
            FlightLog::Ram::status();    // RAM ログの状態だけ見る
            break;
        case 'n':
            FlightLog::Ram::forceTrigger();   // RAM ログの手動トリガ (armed 不問で 8 秒)
            break;
        case 'w':
            StallLog::dump();
            break;
        case 'g': {
            // 高度ホールド (スロットルPID) の ON/OFF トグル。
            //  ★ OFF にした瞬間、スロットルは物理プロポの値へ戻る。ホバー中に
            //    切るときは、スティックを今のホバー位置に戻してから押すこと。
            v.alt_hold_enable = !v.alt_hold_enable;
            if (!v.alt_hold_enable) v.althold.reset(v.range.hOrZero());
            Serial.printf("\n>>> 高度ホールド = %s%s\n",
                          v.alt_hold_enable ? "ON" : "OFF (スロットル手動)",
                          (v.alt_hold_enable && !v.range.valid)
                            ? "  ※ただし今は測距が無効なので効きません" : "");
            break;
        }
        case 'd':
            SelfTest::printFull(Serial);   // 起動時デバイスチェックの再表示
            break;
        case 's':
            // SD の状態表示。ディスアーム中は単体テストも走る。
            //  ★ USE_SD=false のときは selftest が SPI0 を再初期化してフローを
            //    潰すので、状態表示だけにする。
            if (USE_LOGLINK) {
                LogLink::status();   // ログは RP2040 側。こちらの SPI0 は触らない
                break;
            }
            if (!USE_SD) {
                Serial.println("SD: 無効 (USE_FLOW=true)。selftest は SPI0 競合を避けてスキップ");
                break;
            }
            SdLog::status();
            if (!isArmed(v)) SdLog::selftest(Serial);
            break;
        case 'z':
            // フロー積算のゼロ。既知距離キャリブレーションの開始点。
            v.flowobs.zeroAccum();
            Serial.println("FLOW: 積算をゼロにしました");
            break;
        case 'h': {
            // フローのスケール高度 [m] を手で入れる。測距が有効なら毎ループ
            // 上書きされるので、測距を切っている / 失探しているときの手動用。
            const unsigned long old_to = Serial.getTimeout();
            Serial.setTimeout(15000);
            Serial.print("\nフロー高度 h [m] > ");
            const float h = Serial.parseFloat();
            Serial.println(h, 3);
            v.flow.setHeight(h);
            Serial.printf("FLOW: h = %.3f m にしました", v.flow.height());
            if (USE_RANGE && v.range.valid)
                Serial.print(" (※測距が有効なので次のループで上書きされます)");
            Serial.println();
            Serial.setTimeout(old_to);
            break;
        }
        default:
            break;
    }
}

} // namespace Console
} // namespace S5
