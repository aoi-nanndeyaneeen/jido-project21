//出力系
#pragma once
#include <Servo.h>
#include <DShot.h>
#include "Config.h"


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
        _servo.writeMicroseconds(1000);
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
    void begin();             // 親クラスのServo用begin()を上書きするために追加
    void write(float input) ;
};

