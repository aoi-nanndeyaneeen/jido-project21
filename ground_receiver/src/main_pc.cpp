#include <Arduino.h>
#include "Config.h"
#include "Telemetry.h"
#include "Serial_monitor.h"
#include "Serial_com.h"

unsigned long dt;
int counter;

PlaneData  Plane_Data;
GroundData Ground_Data;
IM920SL_Generic<GroundData, PlaneData> im920(&Serial1);
Serial_monitor serial(GROUND_DATA_NUM);

const char* gain_labels[] = {"", "Roll  Rate ", "Pitch Rate ", "Yaw   Rate ", "Roll  Angle", "Pitch Angle"};

void getGainsFromPlaneData(uint8_t sel, float &p, float &i, float &d) {
    if (Plane_Data.packet_type != 1) return; // ゲインデータじゃない時は無視
    switch (sel) {
        case 1: p=Plane_Data.data[0]; i=Plane_Data.data[1]; d=Plane_Data.data[2]; break;
        case 2: p=Plane_Data.data[3]; i=Plane_Data.data[4]; d=Plane_Data.data[5]; break;
        case 4: p=Plane_Data.data[6]; i=0; d=0; break;
        case 5: p=Plane_Data.data[7]; i=0; d=0; break;
        default: p=i=d=0; break;
    }
}

bool readFloatOrQuit(const char* prompt, float &out) {
    Serial.print(prompt);
    Serial.print(" (Q=中止) > ");
    while (!Serial.available());
    String s = Serial.readStringUntil('\n');
    s.trim(); s.toUpperCase();
    if (s == "Q") return false;
    out = s.toFloat();
    return true;
}

void handleGroundPIDTuning() {
    long old_timeout = Serial.getTimeout();
    Serial.setTimeout(30000);
    while (Serial.available()) Serial.read(); // バッファ掃除

    Serial.println("\n========== Ground PID Tuning Menu  [Q]=どこでも中止 ==========");
    float dp, di, dd;
    for (int idx = 1; idx <= 5; idx++) {
        if(idx == 3) continue; // Yawはスキップ
        getGainsFromPlaneData(idx, dp, di, dd);
        Serial.printf(" [%d] %s  P=%+8.4f  I=%+8.4f  D=%+8.4f\n", idx, gain_labels[idx], dp, di, dd);
    }
    Serial.println(" [Q] Quit (変更せずに通信再開)");
    Serial.print("Selection > ");

    while (!Serial.available());
    String sel = Serial.readStringUntil('\n');
    sel.trim(); sel.toUpperCase();
    
    if (sel == "Q" || sel == "") { Serial.setTimeout(old_timeout); return; }

    uint8_t param = sel.toInt();
    if (param < 1 || param > 5 || param == 3) {
        Serial.println("Invalid.");
        Serial.setTimeout(old_timeout);
        return;
    }

    float p, i, d;
    getGainsFromPlaneData(param, p, i, d);
    Serial.printf("--- %s を編集 ---\n", gain_labels[param]);

    if (!readFloatOrQuit("P", p)) { Serial.setTimeout(old_timeout); return; }
    if (!readFloatOrQuit("I", i)) { Serial.setTimeout(old_timeout); return; }
    if (!readFloatOrQuit("D", d)) { Serial.setTimeout(old_timeout); return; }

    // 機体へ新しいゲインを送信
    Serial.printf("\nSending: %s  P=%.4f  I=%.4f  D=%.4f\n", gain_labels[param], p, i, d);
    Ground_Data.param_sel = param;
    Ground_Data.p_adj = p; Ground_Data.i_adj = i; Ground_Data.d_adj = d;
    im920.write(Ground_Data);

    // 送信後クリア
    Ground_Data.param_sel = 0; Ground_Data.p_adj = 0.0f; Ground_Data.i_adj = 0.0f; Ground_Data.d_adj = 0.0f;

    Serial.println("INFO: Sent new gains to plane.");
    Serial.setTimeout(old_timeout);
}

void setup() {
    im920.begin();
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    Serial.println("\n\n[RECEIVER START]");
    Serial.printf("sizeof(PlaneData)=%d  sizeof(GroundData)=%d\n",
        sizeof(PlaneData), sizeof(GroundData));
    serial.begin(115200);
}

void loop() {
    if (!frec()) return;
    bool need_tx = false;

    if (Serial.available()) {
        char c = toupper(Serial.peek());
        if (c == 'R') {
            Serial.read();
            Ground_Data.reset_cmd = 1;
            need_tx = true;
            Serial.println("\n>>> INFO: Sending Remote Reset to Plane...");
            while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) Serial.read();
        } 
        else if (c == 'P') {
            Serial.read();
            while (Serial.available()) Serial.read(); // バッファクリア

            // 1. 機体に「テレメトリ停止＆ゲイン要求 (10)」を送る
            Ground_Data.param_sel = 10;
            im920.write(Ground_Data);
            Ground_Data.param_sel = 0;
            Serial.println("\n>>> INFO: Requesting current gains from plane...");

            // 2. 機体から packet_type == 1 のゲインデータが届くのを待つ
            unsigned long wait_start = millis();
            bool got_gains = false;
            while (millis() - wait_start < 2000) { // 2秒タイムアウト
                if (im920.read(Plane_Data)) {
                    if (Plane_Data.packet_type == 1) { // ゲインデータ到着！
                        got_gains = true;
                        break;
                    }
                }
                delay(10);
            }

            // 3. ゲインが届いたらメニューを開く
            if (got_gains) {
                handleGroundPIDTuning();
            } else {
                Serial.println("ERROR: No response from plane. Timeout.");
            }

            // 4. メニューを抜けたら必ず「テレメトリ再開 (11)」を送る
            Ground_Data.param_sel = 11;
            im920.write(Ground_Data);
            Ground_Data.param_sel = 0;
            Serial.println(">>> INFO: Resuming Telemetry...");
        } 
        else {
            serial.read();
            serial.update(&Ground_Data);
            need_tx = true;
        }
    }

    if (need_tx) {
        im920.write(Ground_Data);
        Ground_Data.reset_cmd = 0;
    }

    // 通常の受信処理
    static unsigned long last_rx_ms = 0;
    if (im920.read(Plane_Data)) {
        last_rx_ms = millis();
    }

    if (counter % 100 == 0) {
        // もし受信したのが姿勢データなら画面更新
        if (Plane_Data.packet_type == 0) {
            Serial.print("\033[2J\033[H");
            Serial.println("=== GROUND STATION  [R]=Reset  [P]=PID Tuning ===");
            
            unsigned long delay_ms = millis() - last_rx_ms;
            if (last_rx_ms == 0 || delay_ms > 2000) {
                Serial.printf(">>> [WARNING] NO DATA RECEIVED! (Silence: %lu ms) <<<\n", delay_ms);
            } else {
                Serial.printf(">>> Data Stream OK (Last RX: %lu ms ago) <<<\n", delay_ms);
            }
            
            print_timing(dt);
            print_MPU(Plane_Data.data[3], Plane_Data.data[4], Plane_Data.data[5], 0.0, 0.0, 0.0);
            print_ACC(Plane_Data.data[0], Plane_Data.data[1], Plane_Data.data[2]);
            Serial.printf("Alt: %+7.2f m\n", Plane_Data.data[6]);
            Serial.println("-------------------------------------------");
            Ground_Data.print();
        }
    }
}