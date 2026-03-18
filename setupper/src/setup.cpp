// ============================================================
// main.cpp  -  自律飛行対応版
//
// 【モード切替チャンネル割り当て】(プロポ側で要確認)
//   Aux2 (ch7/index=6) : 上 → 自動水平旋回
//   Aux3 (ch8/index=7) : 上 → 8の字
//   優先度: 8の字 > 水平旋回 > マニュアル
//   ※両スイッチがOFFでマニュアルに戻る
//
// 【GroundDataによる地上局からのパラメータ変更】
//   Ground_Data.roll  : バンク角 [deg] (0なら変更なし)
//   Ground_Data.pitch : 8の字の片道時間 [s] (0なら変更なし)
// ============================================================

#include <Arduino.h>
#include "Config.h"
#include "Actuators.h"
#include "Serial_com.h"


namespace T = Config::Timing; // T に凝縮


// ============================================================
//  インスタンス生成
// ============================================================


//RC_servo(int pin,float offset, float end1, float end2,bool reverse = false, int minPWM = 1000, int maxPWM = 2000)
//RC_servo(int pin,float offset, float end1, float end2,float endp1,float endp2,bool reverse = false,bool p_reverse = false, int minPWM = 1000, int maxPWM = 2000)  
RC_servo Ele_von1(5, 0.0, -1.0, 1.0,-0.9,0.85,false),
         Ele_von2(6, 0.0, -1.0,0.5,-0.9,1.0,true),
         Ele (24, 0.0, -1.0, 1.0),
         Rud (25, 0.0, -1.0, 1.0);
RC_motor Thr(9, 1.0);

// ============================================================
//  プロトタイプ宣言
// ============================================================
void       updateSensorsAndComms();
void       writeServos();

// ============================================================
//  setup
// ============================================================
void setup() {
    Ele_von1.begin(); Ele_von2.begin();
    Ele.begin();
    Rud.begin();
    Thr.begin();

    //im920.begin();
    Serial.begin(115200);
    //if (!barometer.begin()) Serial.println("Barometer init failed!");
}

// ============================================================
//  loop
// ============================================================
void loop() {
    if (Serial.available() > 0) {
        // 1. まず「どのサーボか」を数字で読み取る
        int id = Serial.parseInt(); 
        
        // 2. 次に「どの設定か」を1文字読み取る
        char cmd = Serial.read(); 
        while(cmd == ' ' || cmd == ',') cmd = Serial.read(); // 空白飛ばし
        
        // 3. 最後に「設定値」を読み取る
        float val = Serial.parseFloat();

        // 4. IDに応じて対象を振り分け
        switch (id) {
            case 1: Ele_von1.updateParam(cmd, val); break;
            case 2: Ele_von2.updateParam(cmd, val); break;
            case 3: Ele.updateParam(cmd, val);      break;
            case 4: Rud.updateParam(cmd, val);      break;

            case 9: 
                if (cmd == 's') {
                    Serial.println("\n--- Copy & Paste these lines to your Config.h ---");
                    Ele_von1.exportConfig("Ele_von1");
                    Ele_von2.exportConfig("Ele_von2");
                    Ele.exportConfig("Ele");
                    Rud.exportConfig("Rud");
                    Serial.println("--------------------------------------------------\n");
                }
                break;
                
            default: Serial.println("ID Error (1-4)"); break;
        }
    }
}



