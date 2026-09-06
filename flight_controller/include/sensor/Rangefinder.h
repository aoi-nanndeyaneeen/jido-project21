// ============================================================
//  Rangefinder.h  -  対地距離センサの機体側ラッパ (バックエンド切替式)
// ============================================================
//  Stage 5 (drone_s5.cpp) の s5c で使用。
//
//  QuadConfig.h の RANGE_BACKEND で 2 種類から選ぶ:
//    ToF_VL53L1X : I2C(Wire) ToF。IMU と同じバス共有。SDA=18 / SCL=19。
//    Sonar_EZ    : MaxBotix LV-MaxSonar-EZ 超音波。PW パルス幅を割り込みで
//                  測る (EZ2.h)。I2C を使わないので ToF のバス問題を回避。
//                  配線 3本:  +5→3V3   GND→GND   PW→RANGE_SONAR_PW_PIN
//                  ★5V 給電時は PW を分圧すること (Teensy 4.0 は 5V 非対応)
//
//  やっていること (バックエンド共通):
//    1. 「斜め距離」[m] を非ブロッキングで取得
//    2. レンジ外 / ステータス異常 / 失探 を弾く
//    3. 機体の傾き (roll/pitch) で cos 補正し「鉛直方向の対地高度」に直す
//    4. 高度を LPF、その微分から上昇速度 [m/s] も出す
//
//  ★ この高度を毎ループ OpticalFlow::setHeight() に渡すことで、
//    s5b までの FLOW_ASSUMED_HEIGHT_M 固定を実測値に置き換える。
//
//  依存: pololu/VL53L1X (ToF)  /  EZ2.h + 割り込みピン (Sonar)
//
//  ※ 既存の他機体コードには影響しません (このヘッダを include するのは
//    drone_s5.cpp だけ)。EZ2Sensor は他では未使用。
// ============================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <VL53L1X.h>
#include "quad/QuadConfig.h"
#include "sensor/EZ2.h"

class Rangefinder {
public:
    Rangefinder() = default;

    // 戻り値 false = センサが応答しない (ToF のみ判定可能。Sonar は常に true を返し、
    //         実際の失探は update() の stale 判定で valid() が落ちる)
    bool begin() {
        _have_h    = false;
        _bad_ms    = 0;
        _last_ms   = 0;
        _last_h_ms = 0;

        if (Quad::RANGE_BACKEND == Quad::RangeBackend::Sonar_EZ) {
            _min_m = Quad::RANGE_SONAR_MIN_M;
            _max_m = Quad::RANGE_SONAR_MAX_M;
            _sonar.begin();
            _sonar_seq   = _sonar.seq();
            _last_new_ms = millis();
            _ok_init = true;          // ソナーは初期化応答が無い
            return true;
        }

        // ---- ToF (VL53L1X) ----
        _min_m = Quad::RANGE_MIN_M;
        _max_m = Quad::RANGE_MAX_M;
        // IMU::begin() が先に Wire.begin() 済みのはずだが、単体でも動くよう保険。
        // ★ 2026-09-05: Teensy の TwoWire::begin(void) は内部で無条件に
        //   setClock(100000) する (WireIMXRT.cpp)。IMU::begin() が先に
        //   400kHz にセットしていても、ここで Wire.begin() を呼ぶと
        //   サイレントに100kHzへ巻き戻ってしまい、以後の全I2C通信
        //   (IMUの毎ループ読み取りも含む) が4倍遅くなっていた
        //   (実測: getMotion6() が理論値約400usに対して実測1566us)。
        //   Wire.begin() を消しても単体動作は壊さない (setBus/setTimeout/
        //   init は Wire オブジェクトの既存状態を使うだけ) ので、ここでは
        //   呼ばずに setClock() だけ再アサートする。
        Wire.setClock(400000);
        _sensor.setBus(&Wire);
        _sensor.setTimeout(500);
        if (!_sensor.init()) { _ok_init = false; return false; }

        _sensor.setDistanceMode(VL53L1X::Medium);
        _sensor.setMeasurementTimingBudget(Quad::RANGE_TIMING_BUDGET_US);
        _sensor.startContinuous(Quad::RANGE_CONTINUOUS_MS);

        _ok_init = true;
        return true;
    }
    bool initialized() const { return _ok_init; }

    // ------------------------------------------------------------
    //  update()
    //    roll_deg / pitch_deg : 機体姿勢角 [deg] (BodyFrame の g_att をそのまま)
    //
    //  非ブロッキング。新しい測距が来ていなければ何もしない (前回値を保持)。
    //  戻り値: このループで「有効な新サンプル」を取り込んだら true。
    // ------------------------------------------------------------
    bool update(float roll_deg, float pitch_deg) {
        _fresh = false;
        if (!_ok_init) return false;

        float slant_m   = 0.0f;
        bool  status_ok = false;
        if (!readSlant(slant_m, status_ok)) {
            // 新サンプル無し。ソナーは長く途切れたら失探にする
            // (ToF は dataReady() 待ちなので、ここは単に前回値保持)。
            if (Quad::RANGE_BACKEND == Quad::RangeBackend::Sonar_EZ &&
                _have_h && (millis() - _last_new_ms) > Quad::RANGE_SONAR_STALE_MS) {
                _have_h = false;
            }
            return false;
        }

        const uint32_t now = millis();
        _last_new_ms = now;

        // --- 外れ値判定 ---
        if (!status_ok || slant_m < _min_m || slant_m > _max_m) {
            if (_have_h) _bad_ms += (now - _last_ms);
            _last_ms = now;
            if (_bad_ms > Quad::RANGE_FAULT_MS) _have_h = false;  // 失探
            return false;
        }
        _raw_m = slant_m;

        // --- 傾き補正: 斜め距離 → 鉛直高度 ---
        //  大きく傾いている間は測距点が横にずれて信用できないので前回値を保持。
        const bool tilt_ok = (fabsf(roll_deg)  < Quad::RANGE_TILT_LIMIT_DEG &&
                              fabsf(pitch_deg) < Quad::RANGE_TILT_LIMIT_DEG);
        if (!tilt_ok) {
            _bad_ms  = 0;             // センサ自体は生きている
            _last_ms = now;
            return false;
        }
        const float cr = cosf(roll_deg  * DEG2RAD);
        const float cp = cosf(pitch_deg * DEG2RAD);
        float h = slant_m * cr * cp + Quad::RANGE_OFFSET_M;
        h = constrain(h, _min_m, _max_m);

        // --- 高度 LPF ---
        if (!_have_h) {
            _height_m = h;
            _vz_mps   = 0.0f;
            _have_h   = true;
        } else {
            const float a  = Quad::RANGE_H_ALPHA;
            const float h_prev = _height_m;
            _height_m += a * (h - _height_m);

            // --- 上昇速度 = 高度の微分 (さらに LPF) ---
            //  ★ dt は「前回 _height_m を更新した時刻」から測る。
            //    旧コードは _last_ms を使っていたが、_last_ms は外れ値や
            //    傾き過大で棄却した回にも now へ進めていたため、
            //    棄却をはさむと分母だけが短くなり vz が最大で数倍に
            //    過大評価されていた (その vz が高度PIDの測定値になる)。
            const float dt = (now - _last_h_ms) * 0.001f;
            if (dt > 0.0f && dt < 0.5f) {
                const float vz_raw = (_height_m - h_prev) / dt;
                _vz_mps += Quad::RANGE_VZ_ALPHA * (vz_raw - _vz_mps);
            }
        }
        _bad_ms    = 0;
        _last_h_ms = now;
        _last_ms   = now;
        _fresh   = true;
        return true;
    }

    // 直近の結果 -------------------------------------------------
    bool  valid()    const { return _ok_init && _have_h; }  // 高度が信用できるか
    float heightM()  const { return _height_m; }            // 鉛直対地高度 [m]
    float climbMps() const { return _vz_mps; }              // 上昇速度 [m/s] (上 +)
    float rawM()     const { return _raw_m; }               // 傾き補正前の斜め距離 [m]
    bool  consumeFresh() { const bool f = _fresh; _fresh = false; return f; }

private:
    static constexpr float DEG2RAD = 0.01745329252f;

    // バックエンドから「新しい斜め距離サンプル」を1個取り出す。
    //  戻り値 true = slant_m / status_ok をセットした (レンジ判定は呼び出し側)
    bool readSlant(float& slant_m, bool& status_ok) {
        if (Quad::RANGE_BACKEND == Quad::RangeBackend::Sonar_EZ) {
            _sonar.update();
            const uint32_t s = _sonar.seq();
            if (s == _sonar_seq) return false;   // 新パルスまだ
            _sonar_seq = s;
            slant_m    = _sonar.get_distance_m();
            status_ok  = true;                   // ソナーに range_status 相当は無い
            return true;
        }
        // ---- ToF (VL53L1X) ----
        if (!_sensor.dataReady()) return false;
        const uint16_t mm = _sensor.read(false); // 非ブロッキング読み出し
        status_ok = (_sensor.ranging_data.range_status == VL53L1X::RangeValid);
        slant_m   = mm * 0.001f;
        return true;
    }

    VL53L1X   _sensor;
    EZ2Sensor _sonar{Quad::RANGE_SONAR_PW_PIN, Quad::RANGE_SONAR_ALPHA};

    bool     _ok_init  = false;
    bool     _have_h   = false;
    bool     _fresh    = false;
    float    _raw_m    = 0.0f;
    float    _height_m = 0.0f;
    float    _vz_mps   = 0.0f;
    float    _min_m    = 0.03f;
    float    _max_m    = 3.5f;
    uint32_t _last_ms      = 0;   // 直近「サンプルを見た」時刻 (bad_ms の積算用)
    uint32_t _last_h_ms    = 0;   // 直近「_height_m を更新した」時刻 (climb 用 dt)
    uint32_t _last_new_ms  = 0;   // 直近「新サンプル到着」の時刻 (stale 判定用)
    uint32_t _bad_ms       = 0;
    uint32_t _sonar_seq    = 0;
};
