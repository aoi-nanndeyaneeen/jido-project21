#include <Arduino.h>
#include "Config.h"
#include "Telemetry.h"
#include "Serial_com.h"

unsigned long dt;
int counter;

PlaneData  Plane_Data;
GroundData Ground_Data;
IM920SL_Generic<GroundData, PlaneData> im920(&Serial1);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    Serial.println("\n\n[GROUND STATION START]");
    Serial.printf("sizeof(PlaneData)=%d  sizeof(GroundData)=%d\n",
        sizeof(PlaneData), sizeof(GroundData));
    im920.begin();
}

void loop() {
    if (!frec()) return;

    // ========== Rキーでリセット送信 ==========
    if (Serial.available()) {
        char c = toupper(Serial.read());
        // バッファ残りを掃除
        while (Serial.available() && 
               (Serial.peek() == '\n' || Serial.peek() == '\r')) Serial.read();

        if (c == 'R') {
            Ground_Data.reset_cmd = 1;
            Ground_Data.param_sel = 0;
            im920.write(Ground_Data);
            Ground_Data.reset_cmd = 0;
            Serial.println(">>> INFO: Reset sent to plane.");
        }
        // PIDチューニングは現在無効
        // else if (c == 'P') { ... }
    }

    // ========== 機体からの受信 ==========
    if (im920.read(Plane_Data)) {
        // 受信できた時だけ即座にログ（デバッグ用）
        Serial.printf("[RX] type=%d  d0=%+6.2f d1=%+6.2f d2=%+6.2f d3=%+6.2f d4=%+6.2f d5=%+6.2f d6=%+6.2f\n",
            (int)Plane_Data.packet_type,
            Plane_Data.data[0], Plane_Data.data[1], Plane_Data.data[2],
            Plane_Data.data[3], Plane_Data.data[4], Plane_Data.data[5],
            Plane_Data.data[6]);
    }

    // ========== 1Hz で生存確認 ==========
    static uint32_t last_alive = 0;
    if (millis() - last_alive > 1000) {
        last_alive = millis();
        Serial.printf("--- Alive  PlaneData=%d GroundData=%d  [R]=Reset ---\n",
            sizeof(PlaneData), sizeof(GroundData));
    }
}