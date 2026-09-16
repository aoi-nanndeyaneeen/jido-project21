#pragma once
#include <Arduino.h>
#include "Config.h"

class Serial_monitor {
private:
  String inputString_ = "";
  int data_num_;

public:
  float *data;
  Serial_monitor(int data_num) {
    data_num_ = data_num;
    data = new float[data_num_];
  }

  void begin(int baud) {
    Serial.begin(baud);
    Serial.setTimeout(10);
    inputString_.reserve(50);
  }

  void update(void *dest){
    memcpy(dest, data, sizeof(float) * data_num_);
  }

  void read() {
    if (get()) {
      for (int i = 0; i < data_num_; i++) {
        data[i] = inputString_.toFloat();
        int commaIndex = inputString_.indexOf(',');
        if (commaIndex != -1) inputString_.remove(0,commaIndex+1);
        else break;
      }
      inputString_ = ""; // 念のため最後もクリア（次の受信に備える）
    }
  }

  bool get() {
    while (Serial.available() > 0) {
      char inChar = (char)Serial.read();
      if (inChar == '\n' || inChar == '\r') { // 改行が来たら「入力完了」
        return true;
      }
      inputString_ += inChar; // 文字を貯める
    }
    return false;
  }
};