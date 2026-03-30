#include <Arduino.h>
#include "Config.h"
#include "Telemetry.h"
#include "Serial_com.h"

unsigned long dt;
int counter;

PlaneData  Plane_Data;
GroundData Ground_Data;
IM920SL_Generic<GroundData, PlaneData> im920(&Serial1);

// auto_flight.cpp の値と合わせておく（Config変更時はここも更新）
float g_rp  =  0.15f,  g_ri = 0.0f,   g_rd = 0.0f;   // Roll  Rate
float g_pp  =  0.10f,  g_pi = 0.001f, g_pd = 0.0f;   // Pitch Rate
float g_yp  = -0.03f;                                  // Yaw   Rate P
float g_rap = 10.0f,   g_pap = 7.0f;                  // Angle P

const char* gain_labels[] = {"","Roll  Rate ","Pitch Rate ","Yaw   Rate ","Roll  Angle","Pitch Angle"};

bool readFloatOrQuit(const char* prompt, float &out) {
    Serial.printf("%s (Q=中止) > ", prompt);
    while (!Serial.available());
    String s = Serial.readStringUntil('\n');
    s.trim(); s.toUpperCase();
    if (s == "Q") return false;
    out = s.toFloat();
    return true;
}

void handleGroundPIDTuning() {
    long old_to = Serial.getTimeout();
    Serial.setTimeout(30000);
    while (Serial.available()) Serial.read();

    Serial.println("\n========== PID Tuning  [Q]=どこでも中止 ==========");
    Serial.println("  ※ 値は最後に送信した値です。起動直後は0です");
    Serial.printf(" [1] %s P=%+8.4f I=%+8.4f D=%+8.4f\n", gain_labels[1], g_rp, g_ri, g_rd);
    Serial.printf(" [2] %s P=%+8.4f I=%+8.4f D=%+8.4f\n", gain_labels[2], g_pp, g_pi, g_pd);
    Serial.printf(" [3] %s P=%+8.4f\n",                    gain_labels[3], g_yp);
    Serial.printf(" [4] %s P=%+8.4f\n",                    gain_labels[4], g_rap);
    Serial.printf(" [5] %s P=%+8.4f\n",                    gain_labels[5], g_pap);
    Serial.println(" [Q] Quit");
    Serial.print("Selection > ");

    while (!Serial.available());
    String sel = Serial.readStringUntil('\n');
    sel.trim(); sel.toUpperCase();
    if (sel == "Q" || sel == "") { Serial.setTimeout(old_to); return; }

    uint8_t param = sel.toInt();
    if (param < 1 || param > 5) { Serial.println("Invalid."); Serial.setTimeout(old_to); return; }

    float p=0, i=0, d=0;
    switch (param) {
        case 1: p=g_rp;  i=g_ri;  d=g_rd;  break;
        case 2: p=g_pp;  i=g_pi;  d=g_pd;  break;
        case 3: p=g_yp;  break;
        case 4: p=g_rap; break;
        case 5: p=g_pap; break;
    }
    Serial.printf("--- %s (現在 P=%.4f I=%.4f D=%.4f) ---\n", gain_labels[param], p, i, d);

    if (!readFloatOrQuit("P", p)) { Serial.setTimeout(old_to); return; }
    if (!readFloatOrQuit("I", i)) { Serial.setTimeout(old_to); return; }
    if (!readFloatOrQuit("D", d)) { Serial.setTimeout(old_to); return; }

    Ground_Data.param_sel = param;
    Ground_Data.p_adj = p; Ground_Data.i_adj = i; Ground_Data.d_adj = d;
    im920.write(Ground_Data);
    Ground_Data.param_sel = 0;
    Ground_Data.p_adj = Ground_Data.i_adj = Ground_Data.d_adj = 0.0f;

    // ローカル記録更新
    switch (param) {
        case 1: g_rp=p;  g_ri=i;  g_rd=d;  break;
        case 2: g_pp=p;  g_pi=i;  g_pd=d;  break;
        case 3: g_yp=p;  break;
        case 4: g_rap=p; break;
        case 5: g_pap=p; break;
    }
    Serial.printf("INFO: Sent %s P=%.4f I=%.4f D=%.4f\n", gain_labels[param], p, i, d);
    Serial.setTimeout(old_to);
}

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

    if (Serial.available()) {
        char c = toupper(Serial.read());
        while (Serial.available() &&
               (Serial.peek()=='\n' || Serial.peek()=='\r')) Serial.read();
        if (c == 'R') {
            Ground_Data.reset_cmd = 1;
            Ground_Data.param_sel = 0;
            im920.write(Ground_Data);
            Ground_Data.reset_cmd = 0;
            Serial.println(">>> INFO: Reset sent.");
        } else if (c == 'P') {
            handleGroundPIDTuning();
        }
    }

    // 受信したときだけ表示用バッファを更新（受信なし時は前回値を保持）
    static PlaneData disp = {};
    if (im920.read(Plane_Data)) {
        disp = Plane_Data;
    }

    if (counter % 100 == 0) {
        Serial.print("\033[2J\033[H");
        Serial.println("=== GROUND STATION  [R]=Reset  [P]=PID Tuning ===");
        Serial.printf("sizeof(PlaneData)=%d sizeof(GroundData)=%d\n",
            sizeof(PlaneData), sizeof(GroundData));
        print_timing(dt);
        print_MPU(disp.roll, disp.pitch, disp.yaw, 0, 0, 0);
        print_ACC(disp.ax, disp.ay, disp.az);
        Serial.printf("Alt: %+7.2f m\n", disp.altitude);
        Serial.println("-------------------------------------------");
        Ground_Data.print();
        Serial.printf("Gains(local): RollRate P=%.4f  PitchRate P=%.4f  RollAng P=%.4f\n",
            g_rp, g_pp, g_rap);
    }
}