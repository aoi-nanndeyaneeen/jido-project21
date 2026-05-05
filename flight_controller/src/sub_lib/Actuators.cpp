#include "Actuators.h"
#include "Config.h"
#include <Arduino.h>
#include <Servo.h>
float s_map(float input,float offset, float end1, float end2){
    if(input>0) return offset + input * (end2 - offset);
    else        return offset + input * (offset - end1);
}

int s_float_to_microseconds(float input,int minPWM, int maxPWM){
    float cenPWM = (minPWM + maxPWM) / 2;
    return (int)(input * (maxPWM - cenPWM) + cenPWM);
}

void Norm_Servo::write(float input){
    if(is_ready()) _servo.writeMicroseconds(s_float_to_microseconds(s_map(input, _off, _end_1, _end_2), _minPWM, _maxPWM));
}

void Elevon_Servo::write(float p_input,float r_input){
        if(!is_ready()) return;
        float mixed = p_input * p_ratio + r_input * r_ratio;
        mixed = constrain(mixed, -1.0f, 1.0f);
        _servo.writeMicroseconds(
            s_float_to_microseconds(
                s_map(mixed, _off, _end_1, _end_2),_minPWM, _maxPWM
            )
        );
}

void Flap_Servo::write(Sw input){
    if(!is_ready()) return;
    float val = (input == up) ? 1.0f : (input == cen) ? 0.0f : -1.0f; // upで1.0、cenで0.0、downで-1.0
    _servo.writeMicroseconds(
        s_float_to_microseconds(
            s_map(val, _off, _end_1, _end_2),_minPWM, _maxPWM
        )
    );
}

void motor::write(float input){
    if(is_ready()) _servo.writeMicroseconds(input*(_maxPWM - _minPWM) + _minPWM);
}