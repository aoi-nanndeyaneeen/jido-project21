//出力系
#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "Config.h"
class RC_servo{
  private:
  int _pin;
  float _des=1500,_offset,_end1,_end2,_inv;
  int _minPWM, _maxPWM ;
  Servo _servo;

  float float_to_microsec(float in) { return in*500+1500; }
  
  public:
  // コンストラクタで設定を流し込む
    RC_servo(int pin,float offset, float end1, float end2,bool reverse = false, int minPWM = 1000, int maxPWM = 2000) 
    : _pin(pin), _offset(offset), _end1(end1), _end2(end2),  _inv(reverse ? -1 : 1), _minPWM(minPWM),_maxPWM(maxPWM){}
    

    void begin() {
      _servo.attach(_pin, _minPWM, _maxPWM); // ここで先ほどの3引数attachを活用！
    }

    void write(float input){
      if(input<0){
        input = input*fabs(_end1-_offset);
      }
      else{
        input = input*fabs(_end2-_offset);
      }
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

    void elevon(float R_input, float P_input,float R_mix_rate, float P_mix_rate,bool R_inv, bool P_inv){
      R_input = (R_inv == true) ? -R_input : R_input;
      P_input = (P_inv == true) ? -P_input : P_input;
      
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
