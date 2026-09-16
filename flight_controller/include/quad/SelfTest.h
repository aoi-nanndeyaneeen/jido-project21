// ============================================================
//  SelfTest.h  -  起動時に「どのデバイスがつながっているか」を 1 回だけ調べ、
//                 結果を保持して起動後も画面に残す
// ============================================================
//  setup() 末尾で probe() を呼ぶ。結果は:
//    ・その場で詳細ブロックを Serial に出す (printFull)
//    ・printStatus() が毎フレーム 1 行 (printCompact) を出すので画面に残り続ける
//    ・シリアル 'd' で詳細ブロックを再表示できる
//  ※ IMU/測距/フロー/SD は「起動時に応答したか」の固定スナップショット。
//    SBUS/IM920 は起動時に信号が来ていたか。飛行中の生死は printStatus の
//    link= / IM920 link= 行が別に出す。
//
//  ★ ブロッキングで待つ箇所がある (SBUS 250ms / IM920 300ms / ロガー 700ms)。
//    まだ飛んでいない setup() の中でしか呼ばないこと。
// ============================================================
#pragma once
#include <Arduino.h>
#include "quad/S5Vehicle.h"
#include "quad/LogLink.h"

namespace S5 {
namespace SelfTest {

enum St : uint8_t { ST_SKIP, ST_OK, ST_FAIL, ST_NOSIG };

struct Dev {
    const char* name;
    const char* bus;
    St          st;
    char        note[64];   // UTF-8 の日本語は 1 文字 3B。strcpy を使うので余裕をもって
};

// 並びはバス順 (I2C → SPI → UART)
static Dev devs[] = {
    { "IMU MPU6050",  "I2C 0x68", ST_SKIP, "" },
    { "Rangefinder",  "I2C/PW",   ST_SKIP, "" },
    { "PMW3901 flow", "SPI CS10", ST_SKIP, "" },
    { "SD HW-125",    "SPI CS9",  ST_SKIP, "" },
    { "RP2040 logger","Serial2",  ST_SKIP, "" },
    { "SBUS RX",      "Serial5",  ST_SKIP, "" },
    { "IM920",        "Serial3",  ST_SKIP, "" },
};
constexpr int N = sizeof(devs) / sizeof(devs[0]);
enum { D_IMU, D_RANGE, D_FLOW, D_SD, D_LINK, D_SBUS, D_IM920 };

inline const char* tag(St s) {
    switch (s) {
        case ST_OK:    return "OK ";
        case ST_FAIL:  return "FAIL";
        case ST_NOSIG: return "--  ";
        default:       return "skip";
    }
}

// 起動時に1回。ブロッキングで良い (まだ飛んでいない)。
inline void probe(Vehicle& v) {
    // --- IMU (I2C 0x68, WHO_AM_I) ---
    if (USE_MPU) {
        devs[D_IMU].st = v.mpu.connected() ? ST_OK : ST_FAIL;
        if (devs[D_IMU].st == ST_FAIL)
            strcpy(devs[D_IMU].note, "応答なし SDA18/SCL19");
    }

    // --- 測距 (VL53L1X I2C 0x29 / MaxBotix PW) ---
    if (USE_RANGE) {
        const bool sonar = (Quad::RANGE_BACKEND == Quad::RangeBackend::Sonar_EZ);
        devs[D_RANGE].bus = sonar ? "PW pin" : "I2C 0x29";
        devs[D_RANGE].st  = v.range.ok ? ST_OK : ST_FAIL;
        strcpy(devs[D_RANGE].note, sonar ? "MaxBotix EZ"
                                          : (v.range.ok ? "VL53L1X" : "VL53L1X 応答なし/電源未接続"));
    }

    // --- PMW3901 (SPI CS10) ---
    if (USE_FLOW) {
        devs[D_FLOW].st = v.flowobs.ok ? ST_OK : ST_FAIL;
        if (devs[D_FLOW].st == ST_FAIL) strcpy(devs[D_FLOW].note, "応答なし");
    }

    // --- SD HW-125 (SPI CS9) ---
    //  USE_LOGLINK / USE_FLOW のときは「オンボード SD は意図的に無し」なので
    //  FAIL 扱いにしない (誤警報になる)。ログの実体は RP2040 logger 行を見る。
    if (USE_SD) {
        devs[D_SD].st = v.sd_ok ? ST_OK : ST_FAIL;
        strcpy(devs[D_SD].note, v.sd_ok ? "" : "VCC=5V/FAT32/配線");
    } else {
        devs[D_SD].st = ST_SKIP;
        strcpy(devs[D_SD].note,
               USE_LOGLINK ? "RP2040 logger へ移設 (下の行)"
                           : "USE_FLOW=true (RAM/USB ログ)");
    }

    // --- RP2040 ロガー (Serial2: RX7/TX8) ---
    //  UART は開いただけでは相手の生死が分からない。相手が 2Hz で返す
    //  状態フレームが届いているかで判定する (RX を配線していないと
    //  NOSIG になるが、送信は片方向でも成立するので致命ではない)。
    if (USE_LOGLINK) {
        if (!v.link_ok) {
            devs[D_LINK].st = ST_FAIL;
            strcpy(devs[D_LINK].note, "UART開けず");
        } else {
            // ロガーは T_STAT を 2Hz (STAT_MS=500ms) で返す。probe の時点では
            // まだ 1 発も取り込めていないことがあるので、ここで最大 700ms 待つ。
            const uint32_t t0 = millis();
            while (millis() - t0 < 700 && !LogLink::statFresh()) {
                LogLink::service();
                delay(5);
            }
            if (!LogLink::statFresh()) {
                devs[D_LINK].st = ST_NOSIG;
                strcpy(devs[D_LINK].note, "応答なし (RX7 未配線?)");
            } else if (!LogLink::loggerSdOk()) {
                devs[D_LINK].st = ST_FAIL;
                strcpy(devs[D_LINK].note, "ロガーの SD が NG");
            } else {
                devs[D_LINK].st = ST_OK;
                strcpy(devs[D_LINK].note, "ロガー SD=OK 双方向OK");
            }
        }
    }

    // --- SBUS: ~250ms 受信機のフレームを待つ ---
    if (USE_SBUS) {
        devs[D_SBUS].st = ST_NOSIG;
        const uint32_t t0 = millis();
        while (millis() - t0 < 250) {
            v.sbus.update();
            if (v.sbus.failCount() == 0) { devs[D_SBUS].st = ST_OK; break; }
        }
        if (devs[D_SBUS].st == ST_NOSIG)
            strcpy(devs[D_SBUS].note, "信号なし(受信機OFF/未接続?)");
    }

    // --- スティック中央の取り込み (QuadConfig § 4 STICK_CENTER_*) ---
    //  送信機のサブトリムがずれると、des[] が中央スナップ帯 (±0.04) から
    //  外れた瞬間に 0.00 -> ±0.05 へ跳ぶ。0.05 は YAW_STICK_DEAD(0.03) も
    //  FLOW_STICK_DEAD(0.05) も超えるので、FC は「ずっと操作中」と判定し、
    //  ヘディングホールドと位置ホールドが丸ごと効かなくなる (LOG0005)。
    //  ★ 握ったまま起動した位置を「中央」として焼き込むのが最悪なので、
    //    棄却されたら 0 (従来動作) のままにして、ここで必ず警告を出す。
    if (USE_SBUS && Quad::STICK_CENTER_ENABLE && devs[D_SBUS].st == ST_OK) {
        const Sbus::CenterCal cc = v.sbus.calibrateCenter(
            Quad::STICK_CENTER_CAL_MS, Quad::STICK_CENTER_MAX_OFS, Quad::STICK_CENTER_MAX_MOVE);
        switch (cc.st) {
            case Sbus::CC_OK:
                Serial.printf("  スティック中央 取込 OK: R%+.3f P%+.3f Y%+.3f "
                              "(%d frames, 振れ %.3f)\n",
                              cc.roll, cc.pitch, cc.yaw, cc.n, cc.worst_move);
                snprintf(devs[D_SBUS].note, sizeof(devs[D_SBUS].note),
                         "center R%+.3f P%+.3f Y%+.3f", cc.roll, cc.pitch, cc.yaw);
                break;
            case Sbus::CC_MOVING:
                Serial.printf("  !! スティック中央 取込 棄却: 動いています "
                              "(振れ %.3f > %.3f)。オフセット 0 のまま !!\n",
                              cc.worst_move, Quad::STICK_CENTER_MAX_MOVE);
                strcpy(devs[D_SBUS].note, "中央取込 棄却(スティックが動いていた)");
                break;
            case Sbus::CC_TOOFAR:
                Serial.printf("  !! スティック中央 取込 棄却: ずれが大きすぎます "
                              "R%+.3f P%+.3f Y%+.3f (上限 %.2f)。"
                              "握ったまま起動していませんか? オフセット 0 のまま !!\n",
                              cc.roll, cc.pitch, cc.yaw, Quad::STICK_CENTER_MAX_OFS);
                strcpy(devs[D_SBUS].note, "中央取込 棄却(ずれ過大/握ったまま?)");
                break;
            default:   // CC_NOSIG
                Serial.println("  !! スティック中央 取込 棄却: フレーム不足。"
                               "オフセット 0 のまま !!");
                strcpy(devs[D_SBUS].note, "中央取込 棄却(フレーム不足)");
                break;
        }
    }

    // --- IM920: RDID を送って ~300ms 応答を待つ ---
    if (USE_IM920) {
        while (Serial3.available()) Serial3.read();      // 掃除
        Serial3.print("RDID\r\n");
        String resp;
        const uint32_t t0 = millis();
        while (millis() - t0 < 300) {
            while (Serial3.available()) resp += (char)Serial3.read();
        }
        resp.trim();
        if (resp.length() > 0) {
            devs[D_IM920].st = ST_OK;
            resp.replace("\r", " "); resp.replace("\n", " ");
            snprintf(devs[D_IM920].note, sizeof(devs[D_IM920].note), "resp:%s", resp.c_str());
        } else {
            devs[D_IM920].st = ST_NOSIG;
            strcpy(devs[D_IM920].note, "無応答 (baud/配線/電源?)");
        }
        while (Serial3.available()) Serial3.read();      // 応答の残りを main ループへ持ち越さない
    }
}

// 詳細ブロック (setup 末尾 と 'd' で表示)
inline void printFull(Print& out) {
    out.println();
    out.println("=== 起動時デバイスチェック ===");
    for (int i = 0; i < N; ++i) {
        out.printf("  [%s] %-13s %-9s %s\n",
                   tag(devs[i].st), devs[i].name, devs[i].bus, devs[i].note);
    }
    out.println("  (IMU/測距/フロー/SD は起動時スナップショット。"
                "SBUS/IM920/RP2040 の現在値は下の link= / LOGLINK 行)");
    out.println("=============================");
}

// printStatus() の 1 行 (画面に残す用)
inline void printCompact(Print& out) {
    out.print("DEV(boot): ");
    for (int i = 0; i < N; ++i) {
        const char* s = devs[i].st == ST_OK ? "OK"
                      : devs[i].st == ST_FAIL ? "X"
                      : devs[i].st == ST_NOSIG ? "--" : "sk";
        // name の先頭語だけ短く出す
        char short_name[8];
        int k = 0;
        for (const char* p = devs[i].name; *p && *p != ' ' && k < 7; ++p) short_name[k++] = *p;
        short_name[k] = 0;
        out.printf("%s:%s ", short_name, s);
    }
    out.println();
}

} // namespace SelfTest
} // namespace S5
