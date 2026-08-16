//通信系
#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Control.h"
#include "sensor/IMU.h"
#include "sensor/Barometer.h"

// ============================================================
//  低レベル IM920SL 通信クラス (変更なし)
// ============================================================
template <typename Sender, typename Reciver>
class IM920SL_Generic {
private:
    HardwareSerial *_IM920SL_Serial;
    String _raw = "", _temp = "", _Byte = "";

public:
    IM920SL_Generic(HardwareSerial *s) : _IM920SL_Serial(s) {}

    void begin() {
        _IM920SL_Serial->begin(19200);
        _raw.reserve(300); _temp.reserve(300); _Byte.reserve(300);
    }

    void write(Sender &data) {
        uint8_t *p = (uint8_t *)&data;
        size_t size = sizeof(Sender);
        uint32_t cs = 0;
        _IM920SL_Serial->print("TXDA ");
        for (size_t i = 0; i < size; i++) {
            char buf[3]; cs += p[i];
            sprintf(buf, "%02X", p[i]); _IM920SL_Serial->print(buf);
        }
        cs = ~cs + 1;
        uint8_t *q = (uint8_t *)&cs;
        for (size_t i = 0; i < sizeof(cs); i++) {
            char buf[3]; sprintf(buf, "%02X", q[i]); _IM920SL_Serial->print(buf);
        }
        _IM920SL_Serial->print("\r\n");
    }

    bool read(Reciver &data) {
        if (!get_stl()) return false;
        uint8_t buffer[sizeof(Reciver)];
        uint32_t cs = 0;
        for (size_t i = 0; i < sizeof(Reciver); i++) {
            _Byte = _temp.substring(0, 2);
            buffer[i] = (uint8_t)strtoul(_Byte.c_str(), NULL, 16);
            cs += buffer[i]; _temp.remove(0, 2);
        }
        uint32_t q = 0; uint8_t *r = (uint8_t *)&q;
        for (size_t i = 0; i < sizeof(q); i++) {
            _Byte = _temp.substring(0, 2);
            r[i] = (uint8_t)strtoul(_Byte.c_str(), NULL, 16);
            _temp.remove(0, 2);
        }
        if (cs + q == 0) { memcpy(&data, buffer, sizeof(Reciver)); return true; }
        return false;
    }

    bool get_stl() {
        while (_IM920SL_Serial->available() > 0)
            _raw += (char)_IM920SL_Serial->read();
        while (true) {
            int le = _raw.indexOf("\r\n");
            if (le == -1) return false;
            String line = _raw.substring(0, le);
            _raw.remove(0, le + 2);
            int colon = line.indexOf(':');
            if (colon == -1) { Serial.print(_raw); continue; }
            _temp = line.substring(colon + 1);
            _temp.replace(",", "");
            if (_temp.length() >= (sizeof(Reciver) * 2 + 8)) return true;
        }
    }
};

// ============================================================
//  FlightTelemetry  -  機体側の送受信を一括管理
// ============================================================
class FlightTelemetry {
private:
    IM920SL_Generic<PlaneData, GroundData> _im920;
    PlaneData  _plane_data;
    GroundData _ground_data;
    uint32_t   _last_ground_rx_ms = 0; // 最後にGroundDataを正常受信した時刻

public:
    FlightTelemetry(HardwareSerial* serial) : _im920(serial) {}

    void begin() { _im920.begin(); }

    // 姿勢データを送信 (28 bytes)
    void sendAttitude(float ax, float ay, float az,
                      float roll, float pitch, float yaw, float alt) {
        _plane_data.ax       = ax;   _plane_data.ay    = ay;  _plane_data.az = az;
        _plane_data.roll     = roll; _plane_data.pitch = pitch; _plane_data.yaw = yaw;
        _plane_data.altitude = alt;
        _im920.write(_plane_data);
    }

    // ------------------------------------------------------------
    //  受信のみ (GroundData を取り込んで鮮度を更新するだけ)
    //
    //  receiveAndProcess() は Axis_value / BarometerSensor を引数に取るため、
    //  それらを持たないコード (drone_s4.cpp のクアッド系は quad/QuadPID.h の
    //  Quad::Pid を使い、気圧計も積んでいない) からは呼べません。
    //  自律制御コマンド (ap_*) の受信だけが目的ならこちらを使います。
    //  PIDゲインのリモート調整やリモートリセットは行いません。
    // ------------------------------------------------------------
    bool receive() {
        if (!_im920.read(_ground_data)) return false;
        _last_ground_rx_ms = millis();
        return true;
    }

    // 姿勢データを送信 (roll/pitch/yaw のみ。加速度と高度は 0 埋め)
    void sendAttitudeOnly(float roll, float pitch, float yaw) {
        sendAttitude(0.0f, 0.0f, 0.0f, roll, pitch, yaw, 0.0f);
    }

    // 地上局からの受信・処理
    // 戻り値: 受信して何らかの処理をした場合 true
    bool receiveAndProcess(Axis_value &Roll, Axis_value &Pitch, Axis_value &Yaw,
                           IMU &mpu, BarometerSensor &barometer,
                           float &bank_angle, unsigned long &turn_ms) {
        if (!_im920.read(_ground_data)) return false;
        _last_ground_rx_ms = millis();

        // リモートリセット
        if (_ground_data.reset_cmd == 1) {
            mpu.recalibrate();
            barometer.reset();
            Config::Timing::resetTiming();
            Roll.pid_reset(); Pitch.pid_reset(); Yaw.pid_reset();
            Serial.println("INFO: Remote Reset Complete.");
            _ground_data.reset_cmd = 0;
        }

        // 飛行パラメータ更新
        if (_ground_data.roll  != 0.0f) bank_angle = fabsf(_ground_data.roll);
        if (_ground_data.pitch != 0.0f) turn_ms    = (unsigned long)(_ground_data.pitch * 1000.0f);

        // PIDゲイン更新
        if (_ground_data.param_sel != 0) {
            float p = _ground_data.p_adj, i = _ground_data.i_adj, d = _ground_data.d_adj;
            switch (_ground_data.param_sel) {
                case 1: Roll.set_rate_gains(p,i,d);
                        Serial.printf("INFO: Roll  Rate  P=%.4f I=%.4f D=%.4f\n",p,i,d); break;
                case 2: Pitch.set_rate_gains(p,i,d);
                        Serial.printf("INFO: Pitch Rate  P=%.4f I=%.4f D=%.4f\n",p,i,d); break;
                case 3: Yaw.set_rate_gains(p,i,d);
                        Serial.printf("INFO: Yaw   Rate  P=%.4f I=%.4f D=%.4f\n",p,i,d); break;
                case 4: Roll.set_angle_gains(p,i,d);
                        Serial.printf("INFO: Roll  Angle P=%.4f I=%.4f D=%.4f\n",p,i,d); break;
                case 5: Pitch.set_angle_gains(p,i,d);
                        Serial.printf("INFO: Pitch Angle P=%.4f I=%.4f D=%.4f\n",p,i,d); break;
                default: break;
            }
            _ground_data.param_sel = 0;
        }
        return true;
    }

    // デバッグ用: 最後に受信した GroundData を返す
    const GroundData& lastGroundData() const { return _ground_data; }

    // 地上局からのリンクが max_age_ms 以内に新規受信されているか
    // (MODE_AUTONOMOUS への安全なフォールバック判定に使う)
    bool groundLinkFresh(uint32_t max_age_ms = 500) const {
        return _last_ground_rx_ms != 0 &&
               (millis() - _last_ground_rx_ms) < max_age_ms;
    }
};