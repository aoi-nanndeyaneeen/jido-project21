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
        _IM920SL_Serial->print("TXDA "); // Added space
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
        if (!get_stl()) return false;

        uint8_t buffer[sizeof(Reciver)];
        uint32_t _check_sum = 0;

        // パースした行（_temp）から実データを読み取る
        for (size_t i = 0; i < sizeof(Reciver); i++) {
            _Byte = _temp.substring(0, 2);
            buffer[i] = (uint8_t)strtoul(_Byte.c_str(), NULL, 16);
            _check_sum += buffer[i];
            _temp.remove(0, 2);
        }

        // チェックサム部分（最後の4バイト）を読み取る
        uint32_t q = 0;
        uint8_t *r = (uint8_t *)&q;
        for (size_t i = 0; i < sizeof(q); i++) {
            _Byte = _temp.substring(0, 2);
            r[i] = (uint8_t)strtoul(_Byte.c_str(), NULL, 16);
            _temp.remove(0, 2);
        }

        // 正しければ構造体に反映
        if (_check_sum + q == 0) {
            memcpy(&data, buffer, sizeof(Reciver));
            return true;
        }

        return false;
    }

    bool get_stl() {
        while (_IM920SL_Serial->available() > 0) {
            char c = (char)_IM920SL_Serial->read();
            _raw += c;
        }

        while (true) {
            
            int line_end = _raw.indexOf("\r\n");
            if (line_end == -1) {
                return false; // 改行まで受信していない
            }

            // 1行切り出す
            String line = _raw.substring(0, line_end);
            _raw.remove(0, line_end + 2);

            int colon = line.indexOf(':');
            if (colon == -1) {
                Serial.print(_raw);
                // ':'が含まれていない行はデータではないのでスキップ (例: "OK", 起動メッセージ等)
                continue;
            }

            // ':' 以降をデータ文字列として抽出
            _temp = line.substring(colon + 1);
            _temp.replace(",", ""); // RSSIや他の付加情報を消す場合など
            
            // 十分な長さがあるかチェック (データ部 + チェックサム4バイト(8文字))
            if (_temp.length() >= (sizeof(Reciver) * 2 + 8)) {
                return true;
            }
        }
    }
};