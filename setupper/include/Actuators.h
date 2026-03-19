// 出力系
#pragma once
#include "Config.h"
#include <Arduino.h>
#include <Servo.h>
class RC_servo {
private:
  int _pin;
  float _des = 1500, _offset, _end1, _end2, _endp1, _endp2, _inv, _invp;
  int _minPWM, _maxPWM;
  Servo _servo;
  bool elevon_flag = false;
  float float_to_microsec(float in) { return in * 500 + 1500; }
  float sbus_constrain(float input, float offset, float end1, float end2) {
    if (input < 0) {
      input = input * fabs(end1 - offset);
    } else {
      input = input * fabs(end2 - offset);
    }
    return input;
  }

public:
  // コンストラクタで設定を流し込む
  RC_servo(int pin, float offset, float end1, float end2, bool reverse = false,
           int minPWM = 1000, int maxPWM = 2000)
      : _pin(pin), _offset(offset), _end1(end1), _end2(end2),
        _inv(reverse ? -1 : 1), _minPWM(minPWM), _maxPWM(maxPWM) {}

  RC_servo(int pin, float offset, float end1, float end2, float endp1,
           float endp2, bool reverse = false, bool p_reverse = false,
           int minPWM = 1000, int maxPWM = 2000)
      : _pin(pin), _offset(offset), _end1(end1), _end2(end2), _endp1(endp1),
        _endp2(endp2), _inv(reverse ? -1 : 1), _invp(p_reverse ? -1 : 1),
        _minPWM(minPWM), _maxPWM(maxPWM) {elevon_flag=true;}

  void begin() {
    _servo.attach(_pin, _minPWM, _maxPWM); // ここで先ほどの3引数attachを活用！
  }
  // --- コピペ用コードを出力するメソッド ---
  void exportConfig(const char* instanceName) {
    Serial.print("RC_servo ");
    Serial.print(instanceName);
    
    if (!elevon_flag) {
      // 標準サーボ形式: (pin, offset, end1, end2, reverse)
      Serial.print("(");
      Serial.print(_pin); Serial.print(", ");
      Serial.print(_offset, 3); Serial.print(", ");
      Serial.print(_end1, 2); Serial.print(", ");
      Serial.print(_end2, 2); Serial.print(", ");
      Serial.print(_inv < 0 ? "true" : "false");
      Serial.println(");");
    } else {
      // エレボン形式: (pin, offset, end1, end2, endp1, endp2, reverse, p_reverse)
      Serial.print("(");
      Serial.print(_pin); Serial.print(", ");
      Serial.print(_offset, 3); Serial.print(", ");
      Serial.print(_end1, 2); Serial.print(", ");
      Serial.print(_end2, 2); Serial.print(", ");
      Serial.print(_endp1, 2); Serial.print(", ");
      Serial.print(_endp2, 2); Serial.print(", ");
      Serial.print(_inv < 0 ? "true" : "false"); Serial.print(", ");
      Serial.print(_invp < 0 ? "true" : "false");
      Serial.println(");");
    }
  }
  void write(float input) {
    input = sbus_constrain(input, _offset, _end1, _end2);
    _des = float_to_microsec(input * _inv + _offset);
    _servo.writeMicroseconds(int(_des));
  }

  void setReverse(bool rev) { 
    _inv = rev ? -1.0 : 1.0; 
    printSettings("Update: Reverse");
  }

  void setOffset(float off) { 
    _offset = off; 
    printSettings("Update: Offset");
  }

  void setEnd1(float e1) { 
    _end1 = e1; 
    printSettings("Update: End1");
  }

  void setEnd2(float e2) { 
    _end2 = e2; 
    printSettings("Update: End2");
  }

  // エレボン等の第2軸（Pitchなど）用
  void setEndP1(float ep1) {
    _endp1 = ep1;
    printSettings("Update: EndP1");
  }

  void setEndP2(float ep2) {
    _endp2 = ep2;
    printSettings("Update: EndP2");
  }

  void updateParam(char cmd, float val) {
    switch (cmd) {
        case 'r': setReverse(val > 0.5);      break;
        case 'o': setOffset(val); write(0.0); break;
        case '1': setEnd1(val); elevon_flag ? elevon(-1.0, 0) : write(-1.0); break;
        case '2': setEnd2(val); elevon_flag ? elevon(1.0, 0)  : write(1.0);  break;
        case 'a': if(elevon_flag){ setEndP1(val); elevon(0, -1.0); } break;
        case 'b': if(elevon_flag){ setEndP2(val); elevon(0, 1.0); }  break;
        case 'p': printSettings("Manual Check"); break;
    }
  }

  void printSettings(const char* name) {
    Serial.print("[Servo: "); Serial.print(name); Serial.println("]");
    Serial.print("  Reverse: "); Serial.println(_inv < 0 ? "TRUE" : "FALSE");
    Serial.print("  Offset : "); Serial.println(_offset, 3);
    Serial.print("  End1/2 : "); Serial.print(_end1, 2); Serial.print(" / "); Serial.println(_end2, 2);
    if (elevon_flag) { // エレボン設定がある場合
        Serial.print("  EndP1/2: "); Serial.print(_endp1, 2); Serial.print(" / "); Serial.println(_endp2, 2);
        Serial.print("  P-Rev  : "); Serial.println(_invp < 0 ? "TRUE" : "FALSE");
    }
    Serial.println("--------------------");
  }


  void flap(Sw input) {
    if (input == up)
      write(1.0);
    if (input == cen)
      write(0.0);
    if (input == down)
      write(-1.0);
  }

  void flapelon(Sw input, float off_up, float off_cen, float off_down) {
    if (input == up)
      write(1.0);
    if (input == cen)
      write(0.0);
    if (input == down)
      write(-1.0);
  }

  void elevon(float R_input, float P_input) {
    R_input = sbus_constrain(R_input, _offset, _end1, _end2);
    P_input = sbus_constrain(P_input, _offset, _endp1, _endp2);


    float output = constrain(R_input*_inv + P_input*_invp, -1.0, 1.0);
    _des = float_to_microsec(output + _offset);

    _servo.writeMicroseconds(int(_des));
  }
};

class RC_motor {
private:
  int _pin, _minPWM, _maxPWM;
  float _des = 0, _end2;
  Servo _servo;

  float float_to_microsec(float in) { return in * 1000 + 1000; }

public:
  // コンストラクタで設定を流し込む
  RC_motor(int pin, float end2 = 1.0, int minPWM = 600, int maxPWM = 2000)
      : _pin(pin), _minPWM(minPWM), _maxPWM(maxPWM), _end2(end2) {
  } // リミットend2(end2_)

  void begin() {
    _servo.attach(_pin, _minPWM, _maxPWM); // ここで先ほどの3引数attachを活用！
  }

  void write(float input) {

    input = input * fabs(_end2);

    _des = float_to_microsec(input);

    _servo.writeMicroseconds(int(_des));
  }
};
