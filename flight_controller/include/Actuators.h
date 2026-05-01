//出力系
#pragma once
#include <Servo.h>
#include "Config.h"
class RC_servo{
  private:
  int _pin;
  float _des=1500,_offset,_end1,_end2,_endp1,_endp2,_inv,_invp;
  int _minPWM, _maxPWM ;
  Servo _servo;

  float float_to_microsec(float in);
  float sbus_constrain(float input,float offset,float end1,float end2);
  public:
  // コンストラクタで設定を流し込む
    RC_servo(int pin,float offset, float end1, float end2,bool reverse = false, int minPWM = 1000, int maxPWM = 2000);
    

    RC_servo(int pin,float offset, float end1, float end2,float endp1,float endp2,bool reverse = false,bool p_reverse = false, int minPWM = 1000, int maxPWM = 2000);
   

    void begin();

    void write(float input);

    void flap(Sw input);
    void flapelon(Sw input,float off_up,float off_cen,float off_down);


    void elevon(float R_input, float P_input);
};

class RC_motor{
  private:
  int _pin, _minPWM, _maxPWM;
  float _des=0,_end2;
  Servo _servo;

  float float_to_microsec(float in);

  public:
  // コンストラクタで設定を流し込む
    RC_motor(int pin,float end2 = 1.0, int minPWM = 600, int maxPWM = 2000);

    void begin();

    void write(float input);
};
