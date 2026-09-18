// ============================================================
//  Rangefinder.h  -  対地距離センサの機体側ラッパ (バックエンド切替式)
// ============================================================
//  Stage 5 (drone_s5.cpp) の s5c で使用。
//
//  QuadConfig.h の RANGE_BACKEND で 3 種類から選ぶ (レンジ・失探時間・表示名は
//  QuadConfig.h の RANGE_INFO に集約してあるので、増やすときはそこも足す):
//    ToF_VL53L1X : I2C(Wire) ToF。IMU と同じバス共有。SDA=18 / SCL=19。
//    Sonar_EZ    : MaxBotix LV-MaxSonar-EZ 超音波。PW パルス幅を割り込みで
//                  測る (EZ2.h)。I2C を使わないので ToF のバス問題を回避。
//                  配線 3本:  +5→3V3   GND→GND   PW→RANGE_SONAR_PW_PIN
//                  ★5V 給電時は PW を分圧すること (Teensy 4.0 は 5V 非対応)
//    Lidar_TSD20 : I2C(Wire) 単点 DTOF LiDAR。IMU と同じバス共有 (0x52)。
//                  0.05〜20m / 200Hz。上昇旋回 (ポール 3m 超) のために導入。
//                  ★出荷時は UART モード。I2C 化してあることが前提 (QuadConfig 参照)
//
//  やっていること (バックエンド共通):
//    1. 「斜め距離」[m] を非ブロッキングで取得
//    2. レンジ外 / ステータス異常 / 失探 を弾く
//    3. 機体の傾き (roll/pitch) で cos 補正し「鉛直方向の対地高度」に直す
//    4. 高度を LPF、その微分から上昇速度 [m/s] も出す
//
//  ★ 2026-09-18: 飛びゲート (stepGate) は「捨てる」だけ。オフセットで高度を貼り付ける
//    方式は天井張り付きを起こしたのでやめた (QuadConfig.h の RANGE_STEP_*)。
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
        _min_m     = Quad::RANGE_INFO.min_m;
        _max_m     = Quad::RANGE_INFO.max_m;
        _n_io_fail = _n_zero = _n_oor = _n_tilt = _n_range = 0;

        if (Quad::RANGE_BACKEND == Quad::RangeBackend::Sonar_EZ) {
            _sonar.begin();
            _sonar_seq = _sonar.seq();
            _ok_init = true;          // ソナーは初期化応答が無い
            return true;
        }

        if (Quad::RANGE_BACKEND == Quad::RangeBackend::Lidar_TSD20) {
            // IMU と同じバス。TSD20 の I2C は最大 400kHz なので IMU と同じで良い。
            Wire.setClock(400000);
            // ID (0x03 == 0x4A) で疎通を見る。返らなければ配線か、まだ UART モード。
            uint8_t id = 0;
            if (!tsdRead(Quad::RANGE_TSD20_REG_ID, &id, 1) || id != Quad::RANGE_TSD20_ID) {
                _ok_init = false;
                return false;
            }
            tsdWrite(Quad::RANGE_TSD20_REG_LASER, 1);   // レーザ ON
            // VL53L1X と同じ理由で、実サンプルが 1 回来るまで待って電源を確認する。
            const uint32_t t0 = millis();
            _ok_init = false;
            while (millis() - t0 < Quad::RANGE_PROBE_TIMEOUT_MS) {
                uint16_t mm = 0;
                if (tsdDistance(mm) && mm != Quad::RANGE_TSD20_ZERO_MM
                                    && mm != Quad::RANGE_TSD20_OOR_MM) {
                    _ok_init = true;
                    break;
                }
                delay(2);
            }
            return _ok_init;
        }

        // ---- ToF (VL53L1X) ----
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

        _sensor.setDistanceMode(Quad::RANGE_TOF_LONG ? VL53L1X::Long : VL53L1X::Medium);
        _sensor.setMeasurementTimingBudget(Quad::RANGE_TIMING_BUDGET_US);
        _sensor.startContinuous(Quad::RANGE_CONTINUOUS_MS);

        // ★ 2026-09-15: ここまでの init() はレジスタ I/O が通るかしか
        //   見ておらず、VIN 未接続でも SDA/SCL の漏れ電流だけで応答して
        //   しまうことがある (実際のレンジングは電源不足で進まず、高度が
        //   凍結したまま気づけない)。実サンプルが1回来るまで待って初めて
        //   「電源も来ている」とみなす。QuadConfig の RANGE_PROBE_TIMEOUT_MS
        //   参照。
        const uint32_t t0 = millis();
        bool got_sample = false;
        while (millis() - t0 < Quad::RANGE_PROBE_TIMEOUT_MS) {
            if (_sensor.dataReady()) { got_sample = true; break; }
            delay(2);
        }
        if (!got_sample) { _ok_init = false; return false; }

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

        //  --- 失探判定: 「_height_m が更新されていない」という一点だけで見る ---
        //  ★ 2026-09-09: VL53L1X が固まると readSlant() が false を返し続ける
        //    だけで _have_h が true のまま残り、valid() が凍った高度を有効と
        //    言い続けた。そこで「新サンプルが来ない」枝に stale 判定を入れた。
        //  ★ 2026-09-18 LOG0087: 同じ凍結が別の枝で再発した。ToF は応答しつつ
        //    status 無効を 472ms 返し、_last_new_ms は毎回更新されるので上の
        //    判定が一度も発火しない。_height_m は 0.504m で凍ったまま valid()
        //    は true、高度ホールドは誤差ゼロと見て基準以下のスロットルを出し
        //    続け、機体は 0.7m/s で沈んで接地した。
        //    サンプル無し / status 無効 / レンジ外 / 傾き過大 — どの枝で捨てても
        //    「高度が凍る」ことは同じなので、枝ごとにタイマーを置くのをやめる。
        //  段差ゲートで捨てている間は valid() が既に false なのでここでは触らない
        //  (ゲート自身の RANGE_STEP_GIVEUP_MS に任せる)。
        if (_have_h && !_step_suspect
            && (millis() - _last_h_ms) > Quad::RANGE_INFO.stale_ms) {
            _have_h = false;
        }

        float slant_m   = 0.0f;
        bool  status_ok = false;
        if (!readSlant(slant_m, status_ok)) return false;

        const uint32_t now = millis();

        // --- 外れ値判定 ---
        _too_close = (status_ok && slant_m < _min_m);
        if (!status_ok || slant_m < _min_m || slant_m > _max_m) {
            if (status_ok) ++_n_range;      // センサは答えたが _min_m.._max_m の外
            if (_have_h) _bad_ms += (now - _last_ms);
            _last_ms = now;
            if (_bad_ms > Quad::RANGE_FAULT_MS) _have_h = false;  // 失探
            return false;
        }
        _raw_m = slant_m;
        _too_close = false;

        // --- 傾き補正: 斜め距離 → 鉛直高度 ---
        //  大きく傾いている間は測距点が横にずれて信用できないので前回値を保持。
        const bool tilt_ok = (fabsf(roll_deg)  < Quad::RANGE_TILT_LIMIT_DEG &&
                              fabsf(pitch_deg) < Quad::RANGE_TILT_LIMIT_DEG);
        if (!tilt_ok) {
            ++_n_tilt;
            _bad_ms  = 0;             // センサ自体は生きている
            _last_ms = now;
            return false;
        }
        const float cr = cosf(roll_deg  * DEG2RAD);
        const float cp = cosf(pitch_deg * DEG2RAD);
        float h = slant_m * cr * cp + Quad::RANGE_OFFSET_M;
        h = constrain(h, _min_m, _max_m);

        // --- 段差ゲート (QuadConfig RANGE_STEP_*) ---
        if (_have_h && !stepGate(h, now)) return false;

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
        _h_acc     = h;
        _bad_ms    = 0;
        _last_h_ms = now;
        _last_ms   = now;
        _fresh   = true;
        return true;
    }

    // 飛びゲートの状態を捨てる (アーム / モード切替の resetControllers から)
    void resetStep() { _step_suspect = false; }

    // I2C バスを復旧した直後に呼ぶ (IMU::recoverBus から drone_s5.cpp 経由)。
    //  同じバスなので、電源が落ちていれば VL53L1X も初期状態に戻っている。
    //  連続測距を設定し直し、高度は失探扱いにして次の有効サンプルで取り直す。
    void busRecovered() {
        _have_h = false;
        _step_suspect = false;
        if (!Quad::RANGE_INFO.on_i2c) return;          // ソナーは I2C に居ない

        if (Quad::RANGE_BACKEND == Quad::RangeBackend::Lidar_TSD20) {
            // 電源が落ちていればレーザも切れている。クロックごと入れ直す。
            Wire.setClock(400000);
            tsdWrite(Quad::RANGE_TSD20_REG_LASER, 1);
            return;
        }
        _sensor.setBus(&Wire);
        _sensor.setTimeout(50);
        if (_sensor.init()) {
            _sensor.setDistanceMode(Quad::RANGE_TOF_LONG ? VL53L1X::Long : VL53L1X::Medium);
            _sensor.setMeasurementTimingBudget(Quad::RANGE_TIMING_BUDGET_US);
            _sensor.startContinuous(Quad::RANGE_CONTINUOUS_MS);
        }
    }

    // 直近の結果 -------------------------------------------------
    //  高度が信用できるか。段差を疑ってサンプルを捨てている間も false。
    bool  valid()    const { return _ok_init && _have_h && !_step_suspect; }
    bool  stepRejecting() const { return _step_suspect; }      // 飛びゲートで捨て中
    uint16_t stepCount()  const { return _step_count; }        // 捨て続けて同期し直した回数
    // 直近サンプルが「センサは応答しているが RANGE_MIN_M 未満」= 地面に置いてある。
    // AltHold の地上からの自動離陸 (ground_start) に使う。失探とは区別できる。
    bool  tooClose() const { return _ok_init && _too_close; }
    float heightM()  const { return _height_m; }            // 鉛直対地高度 [m]
    float climbMps() const { return _vz_mps; }              // 上昇速度 [m/s] (上 +)
    float rawM()     const { return _raw_m; }               // 傾き補正前の斜め距離 [m]
    bool  consumeFresh() { const bool f = _fresh; _fresh = false; return f; }

    //  失探の「理由」の累計。begin() で 0 に戻る。
    //  ★ ioFail が増える = I2C が通っていない (配線/接触不良/電源)。
    //    センサが生きていて 0mm や範囲外を返しているのとは原因が別物なので分けて数える。
    uint32_t ioFailCount() const { return _n_io_fail; }   // I2C 転送そのものが失敗
    uint32_t zeroCount()   const { return _n_zero; }      // 0mm (測距できていない)
    uint32_t oorCount()    const { return _n_oor; }       // 50000 (レンジ外)
    uint32_t rangeCount()  const { return _n_range; }     // min..max の外で棄却
    uint32_t tiltCount()   const { return _n_tilt; }      // 傾き過大で凍結

private:
    static constexpr float DEG2RAD = 0.01745329252f;

    // 飛びゲート本体。h は新しいサンプル (レンジ内に丸めたもの)。
    //  戻り値 true = このサンプルを採用してよい。
    //  ★ 捨てるだけ。オフセットで「貼り付ける」ことはしない (LOG0064 の天井張り付き。
    //    QuadConfig.h の RANGE_STEP_* のコメント参照)。捨て続けたら失探にして、
    //    次の有効サンプルでセンサの値に取り直す。
    bool stepGate(float& h, uint32_t now) {
        if (Quad::RANGE_STEP_M <= 0.0f) return true;
        const float dt   = constrain((now - _last_h_ms) * 0.001f, 0.0f, 0.2f);
        // 地面より下へは外挿しない (着地の瞬間は降下速度が残っていて、LOG0037 の
        //  リプレイで予測 -0.12m → 接地の 0.03m を飛びと誤認した)
        const float pred = constrain(_h_acc + _vz_mps * dt, _min_m, _max_m);
        // 上下に速く動いているほど予測は外れる。その分だけしきい値を広げる
        const float thr = Quad::RANGE_STEP_M
                        + fabsf(_vz_mps) * Quad::RANGE_STEP_VZ_ALLOW_S;
        if (fabsf(h - pred) <= thr) {
            _step_suspect = false;
            return true;
        }
        if (!_step_suspect) {
            _step_suspect  = true;
            _step_first_ms = now;
        }
        _bad_ms  = 0;             // センサ自体は応答している
        _last_ms = now;

        if (now - _step_first_ms >= Quad::RANGE_STEP_GIVEUP_MS) {
            // 戻ってこない = 本当にその高さになった (or センサが変わった)。
            // 失探にして、次の有効サンプルでそのまま取り直す (オフセットは持たない)。
            _have_h = false;
            _step_suspect = false;
            _step_count++;
        }
        return false;
    }

    // バックエンドから「新しい斜め距離サンプル」を1個取り出す。
    //  戻り値 true = slant_m / status_ok をセットした (レンジ判定は呼び出し側)
    bool readSlant(float& slant_m, bool& status_ok) {
        if (Quad::RANGE_BACKEND == Quad::RangeBackend::Lidar_TSD20) {
            uint16_t mm = 0;
            if (!tsdDistance(mm)) { ++_n_io_fail; return false; }  // I2C 自体が失敗
            if (mm == Quad::RANGE_TSD20_ZERO_MM)     ++_n_zero;    // 測距できていない
            else if (mm == Quad::RANGE_TSD20_OOR_MM) ++_n_oor;     // レンジ外
            // ★ 2026-09-19: 「前回と同じ値なら新サンプルではない」としてはいけない。
            //   TSD20 は静止した的に対して同じ mm を返し続ける。ベンチ実測で
            //   値が変わる回数は 0〜14回/秒 まで落ち、まるまる 1 秒変化しない区間が
            //   あった (RANGE_TSD20_STALE_MS=200ms の 5 倍)。同値を捨てると
            //   **高度が安定しているときほど失探する** という逆立ちした挙動になる。
            //   センサ 200Hz > 読み出し 100Hz なので、読めた値は常に新しい。
            status_ok = (mm != Quad::RANGE_TSD20_ZERO_MM &&
                         mm != Quad::RANGE_TSD20_OOR_MM);
            slant_m   = mm * 0.001f;
            return true;
        }
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

    // ---- TSD20 の I2C (レジスタアドレスを書いてから読む) ----
    static bool tsdWrite(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(Quad::RANGE_TSD20_ADDR);
        Wire.write(reg);
        Wire.write(val);
        return Wire.endTransmission() == 0;
    }
    static bool tsdRead(uint8_t reg, uint8_t* buf, uint8_t n) {
        Wire.beginTransmission(Quad::RANGE_TSD20_ADDR);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return false;      // repeated start
        if (Wire.requestFrom((int)Quad::RANGE_TSD20_ADDR, (int)n) != n) return false;
        for (uint8_t i = 0; i < n; ++i) buf[i] = (uint8_t)Wire.read();
        return true;
    }
    // 距離 2 バイト (上位 -> 下位)。★ UART 側はリトルエンディアンなので逆。間違えないこと。
    static bool tsdDistance(uint16_t& mm) {
        uint8_t b[2];
        if (!tsdRead(Quad::RANGE_TSD20_REG_DIST, b, 2)) return false;
        mm = (uint16_t)((uint16_t)b[0] << 8 | b[1]);
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
    bool     _too_close = false;
    float    _max_m    = 3.5f;
    uint32_t _last_ms      = 0;   // 直近「サンプルを見た」時刻 (bad_ms の積算用)
    uint32_t _last_h_ms    = 0;   // 直近「_height_m を更新した」時刻 (climb 用 dt / 失探判定)
    uint32_t _bad_ms       = 0;
    uint32_t _sonar_seq    = 0;

    // 失探の理由べつ累計 (接触不良の切り分け用)
    uint32_t _n_io_fail    = 0;
    uint32_t _n_zero       = 0;
    uint32_t _n_oor        = 0;
    uint32_t _n_range      = 0;
    uint32_t _n_tilt       = 0;

    // 飛びゲート
    float    _h_acc        = 0.0f;   // 直近に採用した高度 (LPF 前)
    bool     _step_suspect = false;
    uint32_t _step_first_ms = 0;     // 捨て始めた時刻
    uint16_t _step_count   = 0;      // 捨て続けて同期し直した回数
};
