//出力系
#pragma once
#include <Servo.h>
#include "Config.h"

// class RC_servo{
//   private:
//   int _pin;
//   float _des=1500,_offset,_end1,_end2,_endp1,_endp2,_inv,_invp;
//   int _minPWM, _maxPWM ;
//   Servo _servo;

//   float float_to_microsec(float in);
//   float sbus_constrain(float input,float offset,float end1,float end2);

//   public:
//   // コンストラクタで設定を流し込む
//     RC_servo(int pin,float offset, float end1, float end2,bool reverse = false, int minPWM = 1000, int maxPWM = 2000);
//     RC_servo(int pin,float offset, float end1, float end2,float endp1,float endp2,bool reverse = false,bool p_reverse = false, int minPWM = 1000, int maxPWM = 2000);
   
//     void begin();
//     void write(float input);
//     void flap(Sw input);
//     void flapelon(Sw input,float off_up,float off_cen,float off_down);
//     void elevon(float R_input, float P_input);
// };

// class RC_motor{
//   private:
//   int _pin, _minPWM, _maxPWM;
//   float _des=0,_end2;
//   Servo _servo;

//   float float_to_microsec(float in);

//   public:
//   // コンストラクタで設定を流し込む
//     RC_motor(int pin,float end2 = 1.0, int minPWM = 600, int maxPWM = 2000);

//     void begin();

//     void write(float input);
// };
template <typename T>
class Actuator {
  protected:
    bool _built = false;
    float _off=0, _end_1=-1.0, _end_2=1.0;  // ← -1.0に修正
    int _pin=-1, _minPWM=600, _maxPWM=2000;
    Servo _servo;

  public:
    T& set_pin(int pin)            { if(!_built) _pin=pin;         return static_cast<T&>(*this); }
    T& set_minPWM(int v)           { if(!_built) _minPWM=v;        return static_cast<T&>(*this); }
    T& set_maxPWM(int v)           { if(!_built) _maxPWM=v;        return static_cast<T&>(*this); }
    T& set_offset(float v)         { if(!_built) _off=v;           return static_cast<T&>(*this); }
    T& set_endpoints(float e1, float e2) { if(!_built){_end_1=e1; _end_2=e2;} return static_cast<T&>(*this); }

    void begin() {
        if(_pin==-1) return;
        _servo.attach(_pin, _minPWM, _maxPWM);
        _built = true;
    }
    bool is_ready() { return _built; }
};

class Norm_Servo : public Actuator<Norm_Servo> {
private:
public:
    void write(float input) ;
};

class Elevon_Servo : public Actuator<Elevon_Servo> {
private:
    float p_ratio = 1.0, r_ratio = 1.0;
public:
    Elevon_Servo& set_ratio(float p, float r) { if(!_built){p_ratio=p; r_ratio=r;} return *this; }
    void write(float p_input,float r_input) ;
};

class Flap_Servo : public Actuator<Flap_Servo> {
private:
public:
    void write(Sw input) ;
};

class motor : public Actuator<motor> {
private:
public:
    void write(float input) ;
};