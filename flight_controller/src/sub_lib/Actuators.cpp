#include "Actuators.h"
#include "Config.h"
#include <Arduino.h>
#include <Servo.h>

float RC_servo::float_to_microsec(float in) { return in * 500 + 1500; }

float RC_servo::sbus_constrain(float input, float offset, float end1,
                               float end2) {
  if (input < 0) {
    input = input * fabs(end1 - offset);
  } else {
    input = input * fabs(end2 - offset);
  }
  return input;
}
RC_servo::RC_servo(int pin, float offset, float end1, float end2,
                   bool reverse, int minPWM, int maxPWM)
    : _pin(pin), _offset(offset), _end1(end1), _end2(end2),
      _inv(reverse ? -1 : 1), _minPWM(minPWM), _maxPWM(maxPWM) {}

RC_servo::RC_servo::RC_servo(int pin, float offset, float end1, float end2,
                             float endp1, float endp2, bool reverse,
                             bool p_reverse, int minPWM,
                             int maxPWM)
    : _pin(pin), _offset(offset), _end1(end1), _end2(end2), _endp1(endp1),
      _endp2(endp2), _inv(reverse ? -1 : 1), _invp(p_reverse ? -1 : 1),
      _minPWM(minPWM), _maxPWM(maxPWM) {}

void RC_servo::begin() {
  _servo.attach(_pin, _minPWM, _maxPWM); // ここで先ほどの3引数attachを活用！
}

void RC_servo::write(float input) {
  input = sbus_constrain(input, _offset, _end1, _end2);

  _des = float_to_microsec(input * _inv + _offset);

  _servo.writeMicroseconds(int(_des));
}

void RC_servo::flap(Sw input) {
  if (input == up)
    write(1.0);
  if (input == cen)
    write(0.0);
  if (input == down)
    write(-1.0);
}

void RC_servo::flapelon(Sw input, float off_up, float off_cen, float off_down) {
  if (input == up)
    write(1.0);
  if (input == cen)
    write(0.0);
  if (input == down)
    write(-1.0);
}

void RC_servo::elevon(float R_input, float P_input) {
  R_input = sbus_constrain(R_input, _offset, _end1, _end2);
  P_input = sbus_constrain(P_input, _offset, _endp1, _endp2);

  float output = constrain(R_input * _inv + P_input * _invp, -1.0, 1.0);
  _des = float_to_microsec(output + _offset);

  _servo.writeMicroseconds(int(_des));
}

float RC_motor::float_to_microsec(float in) { return in * 1000 + 1000; }

// コンストラクタで設定を流し込む
RC_motor::RC_motor(int pin, float end2, int minPWM,
                   int maxPWM)
    : _pin(pin), _minPWM(minPWM), _maxPWM(maxPWM), _end2(end2) {
} // リミットend2(end2_)

void RC_motor::begin() {
  _servo.attach(_pin, _minPWM, _maxPWM); // ここで先ほどの3引数attachを活用！
}

void RC_motor::write(float input) {

  input = input * fabs(_end2);

  _des = float_to_microsec(input);

  _servo.writeMicroseconds(int(_des));
}
