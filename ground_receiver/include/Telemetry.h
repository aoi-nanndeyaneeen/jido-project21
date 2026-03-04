//通信系
#pragma once
#include <Arduino.h>
#include "Config.h"

template <typename Sender, typename Reciver>
class IM920SL_Generic {
private:
    HardwareSerial *_IM920SL_Serial;
    String _raw = "";
    String _temp = "";
    String _Byte = "";

public:
    IM920SL_Generic(HardwareSerial *IM920SL_Serial) : _IM920SL_Serial(IM920SL_Serial) {}

    void begin() {
        _IM920SL_Serial->begin(19200);
        _raw.reserve(300);
        _temp.reserve(300);
        _Byte.reserve(300);
    }

    void write(Sender &data) {
        uint8_t *p = (uint8_t *)&data;
        size_t size = sizeof(Sender);
        uint32_t _check_sum = 0;
        _IM920SL_Serial->print("TXDA");
        for (size_t i = 0; i < size; i++) {
            char buf[3];
            _check_sum += p[i];
            sprintf(buf, "%02X", p[i]);
            _IM920SL_Serial->print(buf);
        }
        _check_sum = ~_check_sum + 1;
        uint8_t *q = (uint8_t *)&(_check_sum);
        for (size_t i = 0; i < sizeof(_check_sum); i++) {
            char buf[3];
            sprintf(buf, "%02X", q[i]);
            _IM920SL_Serial->print(buf);
        }
        _IM920SL_Serial->print("\r\n");
    }

    bool read(Reciver &data) {
        uint8_t *p = (uint8_t *)&data;
        size_t size = sizeof(Reciver);
        uint32_t _check_sum = 0;
        if (get_stl()) {
            for (size_t i = 0; i < size; i++) {
                _Byte = _temp;
                _Byte.remove(2, _Byte.length());
                p[i] = (uint8_t)strtoul(_Byte.c_str(), NULL, 16);
                _check_sum += p[i];
                _temp.remove(0, 2);
            }
            uint32_t q;
            uint8_t *r = (uint8_t *)&q;
            for (size_t i = 0; i < sizeof(_check_sum); i++) {
                _Byte = _temp;
                _Byte.remove(2, _Byte.length());
                r[i] = (uint8_t)strtoul(_Byte.c_str(), NULL, 16);
                _temp.remove(0, 2);
            }
            if (_check_sum + q == 0) return true;
        }
        return false;
    }

    bool get_stl() {
        while (_IM920SL_Serial->available() > 0) {
            _raw += (char)_IM920SL_Serial->read();
        }
        if (_raw.indexOf("\r\n") == -1) return false;
        if (_raw.indexOf(':') == -1) {
            Serial.println(_raw);
            _raw.remove(0, _raw.indexOf("\r\n") + 2);
            return false;
        } else {
            int line_end = _raw.indexOf("\r\n");
            int colon = _raw.indexOf(':');
            _temp = _raw;
            _raw.remove(0, line_end + 2);
            _temp.remove(line_end, _temp.length());
            _temp.remove(0, colon + 1);
            _temp.replace(",", "");
            return true;
        }
    }
};