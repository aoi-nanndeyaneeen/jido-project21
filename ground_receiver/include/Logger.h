#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include "Config.h"

class FlashLogger {
private:
    File logFile;
    bool active = false;
    const char* filename = "/flight_log.csv";
    unsigned long lastFlush = 0;

public:
    FlashLogger() {}

    bool begin() {
        if (!LittleFS.begin()) {
            Serial.println("【エラー】LittleFSの初期化に失敗しました。");
            return false;
        }
        Serial.println("【成功】LittleFS接続完了。");
        return true;
    }

    void start() {
        // 追記モードで開く
        logFile = LittleFS.open(filename, "a");
        if (!logFile) {
            Serial.println("【エラー】ログファイルを開けませんでした。");
            return;
        }
        
        // ファイルが空ならヘッダーを書き込む
        if (logFile.size() == 0) {
            logFile.println("Time,Roll,Pitch,Yaw,Ax,Ay,Az,Alt,P_adj,I_adj,D_adj");
        }
        
        active = true;
        Serial.println(">>> ログ記録を開始しました。");
    }

    void stop() {
        if (active) {
            logFile.close();
            active = false;
            Serial.println(">>> ログ記録を停止しました。");
        }
    }

    void clear() {
        stop();
        if (LittleFS.remove(filename)) {
            Serial.println(">>> ログファイルを削除しました。");
        } else {
            Serial.println(">>> ログファイルが見つかりません。");
        }
    }

    void dump() {
        stop();
        File f = LittleFS.open(filename, "r");
        if (!f) {
            Serial.println(">>> ログファイルが空、または存在しません。");
            return;
        }
        Serial.println("--- LOG DATA START ---");
        while (f.available()) {
            Serial.write(f.read());
        }
        Serial.println("\n--- LOG DATA END ---");
        f.close();
    }

    void log(const PlaneData& p, const GroundData& g, unsigned long timestamp) {
        if (!active) return;

        logFile.printf("%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.3f,%.3f,%.3f\n",
                       timestamp, p.gx, p.gy, p.gz, p.ax, p.ay, p.az, p.altitude,
                       g.p_adj, g.i_adj, g.d_adj);
        
        // 5秒おきにフラッシュして書き込みを確定させる
        if (millis() - lastFlush > 5000) {
            logFile.flush();
            lastFlush = millis();
        }
    }

    bool isActive() const { return active; }
};
