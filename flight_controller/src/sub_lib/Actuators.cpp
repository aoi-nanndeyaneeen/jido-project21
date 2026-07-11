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

// ============================================================
// motorクラス - Teensyハードウェアタイマーを使った400Hz Fast PWM
// ============================================================
void motor::begin() {
    if(_pin == -1) return;
    pinMode(_pin, OUTPUT);
    
    // TeensyのハードウェアPWM設定
    analogWriteResolution(12);       // 12-bit分解能 (0-4095)
    analogWriteFrequency(_pin, 400); // PWM周波数を400Hzに設定
    
    write(0.0f); // 初期化 (スロットル0のパルスを出力)
    _built = true;
}

void motor::write(float input) {
    if(!is_ready()) return;
    
    // inputは 0.0(スロットル0) ~ 1.0(フルスロットル) を想定
    input = constrain(input, 0.0f, 1.0f);
    
    // 1000us ~ 2000us の幅を 12-bit (0-4095) の 1638 ~ 3277 にマッピング
    int pwm_val = 1638 + (int)(input * (3277 - 1638));
    
    analogWrite(_pin, pwm_val);
}