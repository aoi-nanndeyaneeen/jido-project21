//出力系
#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "Config.h"
class RC_servo{
  private:
  int _pin;
  float _des=1500,_offset,_end1,_end2,_endp1,_endp2,_inv,_invp;
  int _minPWM, _maxPWM ;
  Servo _servo;

  float float_to_microsec(float in) { return in*500+1500; }
  float sbus_constrain(float input,float offset,float end1,float end2){
      if(input<0){
        input = input*fabs(end1-offset);
      }
      else{
        input = input*fabs(end2-offset);
      }
      return input;
  }
  public:
  // コンストラクタで設定を流し込む
    RC_servo(int pin,float offset, float end1, float end2,bool reverse = false, int minPWM = 1000, int maxPWM = 2000) 
    : _pin(pin), _offset(offset), _end1(end1), _end2(end2),  _inv(reverse ? -1 : 1), _minPWM(minPWM),_maxPWM(maxPWM){}

    RC_servo(int pin,float offset, float end1, float end2,float endp1,float endp2,bool reverse = false,bool p_reverse = false, int minPWM = 1000, int maxPWM = 2000) 
    : _pin(pin), _offset(offset), _end1(end1), _end2(end2), _endp1(endp1), _endp2(endp2),  _inv(reverse ? -1 : 1), _invp(p_reverse ? -1 : 1), _minPWM(minPWM),_maxPWM(maxPWM){}

    void begin() {
      _servo.attach(_pin, _minPWM, _maxPWM); // ここで先ほどの3引数attachを活用！
    }

    void write(float input){
      input = sbus_constrain(input,_offset,_end1,_end2);

      _des = float_to_microsec(input*_inv + _offset);

      _servo.writeMicroseconds(int(_des));
    }

    void flap(Sw input){
      if(input ==up  ) write(1.0);
      if(input ==cen ) write(0.0);
      if(input ==down) write(-1.0);
    }

    void flapelon(Sw input,float off_up,float off_cen,float off_down){
      if(input ==up  ) write(1.0);
      if(input ==cen ) write(0.0);
      if(input ==down) write(-1.0);
    }

    void elevon(float R_input, float P_input){
      R_input = sbus_constrain(R_input,_offset,_end1,_end2);
      P_input = sbus_constrain(P_input,_offset,_endp1,_endp2);
      R_input = (_inv == true) ? -R_input : R_input;
      P_input = (_invp == true) ? -P_input : P_input;

      float output = constrain(R_input + P_input, -1.0, 1.0);
      _des = float_to_microsec(output + _offset);

      _servo.writeMicroseconds(int(_des));
    }
};

class RC_motor{
  private:
  int _pin, _minPWM, _maxPWM;
  float _des=0,_end2;
  Servo _servo;

  float float_to_microsec(float in) { return in*1000+1000; }

  public:
  // コンストラクタで設定を流し込む
    RC_motor(int pin,float end2 = 1.0, int minPWM = 600, int maxPWM = 2000) 
      :_pin(pin),_minPWM(minPWM),_maxPWM(maxPWM),_end2(end2){}//リミットend2(end2_)
    

    void begin() {
      _servo.attach(_pin, _minPWM, _maxPWM); // ここで先ほどの3引数attachを活用！
    }

    void write(float input){

      input = input*fabs(_end2);

      _des = float_to_microsec(input);

      _servo.writeMicroseconds(int(_des));
    }
};
