//入力系
#pragma once
#include <Arduino.h>
#include <sbus.h>
#include "Config.h"

class Sbus {
private:

    bfs::SbusRx *_sbus;
    bfs::SbusData _data;
    int i;

    int connection_fail;

    float scaleToUnit(float raw) {
        const float minVal = 360.0f;
        const float midVal = 1040.0f;
        const float maxVal = 1680.0f;

        // 範囲外をガード
        raw = constrain(raw, minVal, maxVal);

        if (raw >= midVal) {
            // 1040〜1680 を 0.5〜1.0 にマップ
            // (現在の値 - 中央) / (最大 - 中央) * 0.5 + 0.5
            return (raw - midVal) / (maxVal - midVal) * 0.5f + 0.5f;
        } 
        else {
            // 360〜1040 を 0.0〜0.5 にマップ
            // (現在の値 - 最小) / (中央 - 最小) * 0.5
            return (raw - minVal) / (midVal - minVal) * 0.5f;
        }
    }

public:
    float des[16];

    Sbus(HardwareSerial *ser) {
        _sbus = new bfs::SbusRx(ser, true);
    }

    void begin() {
        _sbus->Begin();
    }

    void update() {

      if (_sbus->Read()) {
        _data = _sbus->data();
        connection_fail = 0;
      }
      else connection_fail++;//通信途絶検知用

      // SBUSの172-1811を0.0-1.0にマッピング
      for(i=0;i<16;i++){
        //if(i == 1)Serial.println(_data.ch[i]);
        float val = scaleToUnit(_data.ch[i]);//取得して0-1にマッピング
        val = (val < 0.0f) ? 0 : (val > 1.0f ? 1.0f : val);//?はif,:はelse,そのため(条件)? 結果 : 結果　みたいな感じ

        if(i!=Ch::THR) {
          val = (abs(val-0.50f)) < 0.02f ? 0.0f : val*2.0f-1.0f;
        }

        else {
          val = ((val) < 0.02f) ? 0.00 : val;
        }//thrだけ例外だね
        des[i]= val;
      }
    }

    Sw Ch_state(int ch) {
        if (des[ch] > 0.25)     return up;                //ここで条件式をそのままreturnする発想は私にはなかった(´・ω・｀)
        if (des[ch] < -0.25)    return down;
        return cen;
    }

    bool th_cut() {
        if(Ch_state(Ch::THR_CUT)==up)   return true;   
        return false;
    }

    bool isSafe() {
        return !_data.failsafe && !(connection_fail>3000);
    }
};