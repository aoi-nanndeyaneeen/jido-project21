#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <MadgwickAHRS.h>
#include "Config.h"

// ============================================================
//  IMU チップの選択 (I2C レジスタ直叩き。ライブラリは使わない)
//
//  -D IMU_CHIP_LSM6DSV16X : 秋月 AE-LSM6DSV16X (2026-09-18〜 drone_s5_rp2040)
//  未定義                 : MPU6050 (GY-521。Teensy 時代の各機体)
//
//  チップ依存はこの ImuChip 構造体に閉じ込める。IMU クラス本体 (Madgwick 融合・
//  キャリブレーション・EEPROM・I2C 復旧) はどちらでも同じ。
//  ★ レジスタの設定値と換算スケール (LSB/g, LSB/dps) は必ずここで対にしておく。
//    2026-09-14 にレンジ設定の書き込みが黙って落ちて、ソフトのスケールと
//    センサの実レンジが食い違ったまま何日も気づけなかった。begin() は
//    CONFIG[] を書いた後に読み戻して一致を確認する。
// ============================================================
struct ImuChip {
    struct RegVal { uint8_t reg; uint8_t val; const char* what; };

#if defined(IMU_CHIP_LSM6DSV16X)
    // ------------------------------------------------------------
    //  ST LSM6DSV16X (秋月 AE-LSM6DSV16X)
    //   ・3.3V 専用 (基板にレギュレータ無し。5V を入れると壊れる)
    //   ・I2C アドレスは基板のジャンパで決まる: J3 短絡 = 0x6A / J2 短絡 = 0x6B。
    //     どちらも開放だと SA0 が浮いてアドレスが定まらない。必ずどちらか短絡。
    //     ソフトは両方を WHO_AM_I で探して応答した方を使う (probe())。
    //   ・CS は基板で VDD にプルアップ済み (J4 開放) = I2C モード。
    //   ・SDA/SCL のプルアップは J1 短絡で 10kΩ が入る。XIAO 側には無いので短絡する。
    //   ・ODR 960Hz (メインループ 1000Hz 設計 / 実効 630〜700Hz より上)。
    //     LPF は MPU6050 の DLPF 42Hz に寄せる: ジャイロ LPF1 ≈58Hz、加速度 LPF2 = ODR/20 = 48Hz。
    //   ・出力は OUTX_L_G(0x22) からジャイロ→加速度の順、リトルエンディアン 12 バイト。
    // ------------------------------------------------------------
    static constexpr const char* NAME     = "LSM6DSV16X";
    static constexpr const char* I2C_DESC = "I2C 0x6A/0x6B";
    static constexpr uint8_t ADDRS[]      = { 0x6A, 0x6B };
    static constexpr uint8_t REG_WHO_AM_I = 0x0F;
    static constexpr uint8_t WHO_AM_I     = 0x70;
    static constexpr uint8_t REG_OUT      = 0x22;     // OUTX_L_G
    static constexpr uint8_t OUT_LEN      = 12;
    // ソフトリセット (CTRL3.SW_RESET)。ビットは自動で戻るので読み戻し検証はしない。
    static constexpr RegVal  RESET        = { 0x12, 0x01, "CTRL3 SW_RESET" };
    static constexpr uint8_t RESET_WAIT_MS = 20;
    // ODR (CTRL1/2) は最後に書く。設定が固まる前に動き出させない。
    static constexpr RegVal CONFIG[] = {
        { 0x12, 0x44, "CTRL3 BDU + IF_INC"                    },  // bit6 BDU, bit2 IF_INC
        { 0x15, 0x51, "CTRL6 gyro LPF1 58Hz + ±250dps"        },  // LPF1_G_BW=101, FS_G=0001
        { 0x16, 0x01, "CTRL7 gyro LPF1 有効"                   },  // LPF1_G_EN
        { 0x17, 0x42, "CTRL8 accel LPF2 ODR/20 + ±8g"         },  // HP_LPF2_XL_BW=010, FS_XL=10
        { 0x18, 0x08, "CTRL9 accel LPF2 有効"                  },  // LPF2_XL_EN
        { 0x10, 0x09, "CTRL1 accel 960Hz high-performance"    },  // ODR_XL=1001
        { 0x11, 0x09, "CTRL2 gyro 960Hz high-performance"     },  // ODR_G=1001
    };
    // 上の FS 設定と対。±8g = 0.244 mg/LSB、±250dps = 8.75 mdps/LSB
    static constexpr float ACCEL_SCALE = 4096.0f;            // LSB/g
    static constexpr float GYRO_SCALE  = 1000.0f / 8.75f;    // LSB/dps ≈ 114.29
    static constexpr uint32_t CAL_MAGIC = 0x4C534D36;        // "LSM6" (MPU6050 の値を読まないため)

    static void decode(const uint8_t* b, int16_t& ax, int16_t& ay, int16_t& az,
                                         int16_t& gx, int16_t& gy, int16_t& gz) {
        auto le = [&](int i) { return (int16_t)((uint16_t)b[2 * i + 1] << 8 | b[2 * i]); };
        gx = le(0); gy = le(1); gz = le(2);
        ax = le(3); ay = le(4); az = le(5);
    }
#else
    // ------------------------------------------------------------
    //  InvenSense MPU6050 (GY-521)
    //   ・出力は ACCEL_XOUT_H(0x3B) から 加速度→温度→ジャイロ、ビッグエンディアン 14 バイト。
    //   ・±8g にした経緯 (上下逆マウント + s_az_bias≈-2 で上向き余裕が 1g しか無かった) は
    //     2026-09-09。Madgwick は加速度を正規化するので姿勢推定の挙動は不変。
    // ------------------------------------------------------------
    static constexpr const char* NAME     = "MPU6050";
    static constexpr const char* I2C_DESC = "I2C 0x68";
    static constexpr uint8_t ADDRS[]      = { 0x68 };
    static constexpr uint8_t REG_WHO_AM_I = 0x75;
    static constexpr uint8_t WHO_AM_I     = 0x68;
    static constexpr uint8_t REG_OUT      = 0x3B;     // ACCEL_XOUT_H
    static constexpr uint8_t OUT_LEN      = 14;
    static constexpr RegVal  RESET        = { 0, 0, nullptr };   // リセットしない (従来どおり)
    static constexpr uint8_t RESET_WAIT_MS = 0;
    static constexpr RegVal CONFIG[] = {
        { 0x6B, 0x01, "PWR_MGMT_1 スリープ解除 / PLL X gyro" },
        { 0x1C, 0x10, "ACCEL_CONFIG ±8g"                    },
        { 0x1B, 0x00, "GYRO_CONFIG ±250dps"                 },
        { 0x1A, 0x03, "CONFIG DLPF 42Hz"                    },
    };
    static constexpr float ACCEL_SCALE = 4096.0f;   // LSB/g   (±8g)
    static constexpr float GYRO_SCALE  = 131.0f;    // LSB/dps (±250dps)
    static constexpr uint32_t CAL_MAGIC = 0x43414C32;  // "CAL2"

    static void decode(const uint8_t* b, int16_t& ax, int16_t& ay, int16_t& az,
                                         int16_t& gx, int16_t& gy, int16_t& gz) {
        auto be = [&](int i) { return (int16_t)((uint16_t)b[2 * i] << 8 | b[2 * i + 1]); };
        ax = be(0); ay = be(1); az = be(2);          // be(3) = 温度
        gx = be(4); gy = be(5); gz = be(6);
    }
#endif
    static constexpr uint8_t N_ADDRS  = sizeof(ADDRS) / sizeof(ADDRS[0]);
    static constexpr uint8_t N_CONFIG = sizeof(CONFIG) / sizeof(CONFIG[0]);
    static bool isAddr(uint8_t a) {
        for (uint8_t i = 0; i < N_ADDRS; ++i) if (ADDRS[i] == a) return true;
        return false;
    }
};

// ------------------------------------------------------------
//  取付け向き: センサ生軸 → FLU (前・左・上)
//
//  IMU_MOUNT_X/Y/Z (±1) を定義した環境では、加速度とジャイロに「同じ」符号をかける。
//  Madgwick は加速度 (重力) とジャイロ (回転) が同じ右手系に載っている前提で
//  融合するので、片方だけ反転すると両者が逆方向を主張し、静止中も角度が
//  流れ続ける (2026-09-17 log_048/049 で実測)。
//  ★ 符号の積は +1 (= 回転) でなければならない。-1 (鏡映) だと角速度の向きが
//    加速度と整合しなくなる。上下逆に付けたなら Z と X (または Y) の 2 軸を反転。
//  ★ 静止・水平で FLU の az が +1g (重力反力が上向き) になること。
//    recalibrate() はこれを満たさないと却下する。
//  ★ 符号反転しかできない (軸の入れ替えは無い)。チップの X 軸が機体の前後、
//    Y 軸が左右に沿うように基板を載せること。90° 回して載せると X/Y の入れ替えが
//    要るので、そのときはここに swap を足す。
//
//  未定義の環境は旧来の符号のまま (他の機体用。ジャイロ X/Y だけ反転している)。
// ------------------------------------------------------------
#if defined(IMU_MOUNT_X)
static constexpr float IMU_ACC_SX = IMU_MOUNT_X, IMU_ACC_SY = IMU_MOUNT_Y, IMU_ACC_SZ = IMU_MOUNT_Z;
static constexpr float IMU_GYR_SX = IMU_MOUNT_X, IMU_GYR_SY = IMU_MOUNT_Y, IMU_GYR_SZ = IMU_MOUNT_Z;
static_assert(IMU_MOUNT_X * IMU_MOUNT_Y * IMU_MOUNT_Z > 0.0f,
              "IMU_MOUNT の符号の積は +1 (回転) にすること。鏡映だとジャイロと加速度が食い違う");
#else
static constexpr float IMU_ACC_SX = +1.0f, IMU_ACC_SY = +1.0f, IMU_ACC_SZ = +1.0f;
static constexpr float IMU_GYR_SX = -1.0f, IMU_GYR_SY = -1.0f, IMU_GYR_SZ = +1.0f;
#endif

class IMU {
private:
    // recalibrate() の妥当性チェックのしきい値。
    //  傾き: 水平な床に置いたつもりでも脚の高さで数度は出るので 5度まで許す。
    //        それを超えたら「水平ではない」と見なして採用しない。
    //  ジャイロ振れ幅: 手で持っているとすぐ数 deg/s 動く。静止なら 1 未満。
    //
    //  ★ (IMU_MOUNT 未定義の旧来環境) 加速度センサを上下逆に載せた機体があり、
    //    水平時の az は -1g になるのが正常だった。そのため |az| で見ている。
    //    IMU_MOUNT 定義環境 (drone_s5_rp2040) は符号込みで az>0 を要求する。
    //    (2026-09-04 に az<0 を却下条件にしてしまい、正常な姿勢での 'k' が
    //     通らなくなった。その修正)
    static constexpr float MAX_CAL_TILT_DEG  = 5.0f;
    static constexpr float MAX_CAL_GYRO_SPAN = 3.0f;
    // recalibrate() 中に読めなかった回数がこれを超えたら中断 (バスが死んでいる)
    static constexpr int   MAX_CAL_IO_FAIL   = 50;

    Madgwick filter;
    int16_t ax_raw, ay_raw, az_raw;
    int16_t _p_ax = 0, _p_ay = 0, _p_az = 0, _p_gx = 0, _p_gy = 0, _p_gz = 0;
    uint32_t _same_since_ms = 0;
    uint8_t  _io_fail = 0;              // 連続で読めなかった回数
    uint32_t _io_fail_total = 0;
    uint16_t _recover_count = 0;
    uint32_t _last_recover_ms = 0;
    bool     _bus_recovered = false;
    bool     _recover_ok = false;       // 直近の復旧で WHO_AM_I と設定書き戻しが通ったか
    int16_t gx_raw, gy_raw, gz_raw;
    TwoWire *wire;
    uint8_t  _addr = ImuChip::ADDRS[0];   // probe() で応答したアドレスに更新

    // 出力レジスタから 6 軸を連続読み。戻り値 false = I2C 失敗。
    bool readRaw() {
        wire->beginTransmission(_addr);
        wire->write(ImuChip::REG_OUT);
        if (wire->endTransmission(false) != 0) return false;
        if (wire->requestFrom(_addr, ImuChip::OUT_LEN) != ImuChip::OUT_LEN) return false;
        uint8_t buf[ImuChip::OUT_LEN];
        for (uint8_t i = 0; i < ImuChip::OUT_LEN; ++i) buf[i] = (uint8_t)wire->read();
        ImuChip::decode(buf, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);
        return true;
    }

    bool readReg(uint8_t addr, uint8_t reg, uint8_t& out) {
        wire->beginTransmission(addr);
        wire->write(reg);
        if (wire->endTransmission(false) != 0) return false;
        if (wire->requestFrom(addr, (uint8_t)1) != 1) return false;
        out = (uint8_t)wire->read();
        return true;
    }

    void writeReg(uint8_t reg, uint8_t value) {
        wire->beginTransmission(_addr);
        wire->write(reg);
        wire->write(value);
        wire->endTransmission();
    }

    // レジスタに書き込み、読み戻して一致するまで数回リトライする。
    //  ★ 2026-09-14: レンジ設定の書き込みが黙って反映されないケースがあり、
    //    ソフトとセンサーのスケール食い違いに何日も気づけなかった。
    //    ここで検証しないと同じ事故が再発する。
    bool writeRegVerified(uint8_t reg, uint8_t value, uint8_t retries = 3) {
        for (uint8_t attempt = 0; attempt <= retries; ++attempt) {
            writeReg(reg, value);
            delay(2);
            uint8_t readback;
            if (readReg(_addr, reg, readback) && readback == value) return true;
            delay(5);
        }
        return false;
    }

    // WHO_AM_I で候補アドレスを順に探し、応答した方を _addr にする。
    bool probe() {
        for (uint8_t i = 0; i < ImuChip::N_ADDRS; ++i) {
            uint8_t who;
            if (readReg(ImuChip::ADDRS[i], ImuChip::REG_WHO_AM_I, who) && who == ImuChip::WHO_AM_I) {
                _addr = ImuChip::ADDRS[i];
                return true;
            }
        }
        return false;
    }

    // ImuChip::CONFIG[] を全部書いて読み戻す。1 つでも通らなければ false。
    //  ★ 反映されないと、ソフトは ±8g/±250dps のつもりで割り続けるのに実際の
    //    センサは既定レンジのまま → 読み値のスケールが数倍ずれる。この状態だと
    //    recalibrate() の |a|≒1g チェックに必ず REJECTED され、古いバイアスの
    //    まま姿勢推定が発散する (2026-09-14: roll_ang が 180° 付近に張り付いた原因)。
    bool configureChip(uint8_t retries) {
        bool ok = true;
        for (uint8_t i = 0; i < ImuChip::N_CONFIG; ++i) {
            const auto& c = ImuChip::CONFIG[i];
            if (!writeRegVerified(c.reg, c.val, retries)) {
                Serial.printf("!!!! FATAL: %s %s (0x%02X<-0x%02X) が反映されていません。"
                              "スケールが実際と食い違うため姿勢推定が壊れます。"
                              "I2C 配線・IMU の電源を確認してください。\n",
                              ImuChip::NAME, c.what, c.reg, c.val);
                ok = false;
            }
        }
        return ok;
    }

    // I2C バスの復旧。SDA を掴んだままのスレーブを SCL のパルスで解放し、
    // Wire を入れ直してからチップを「リセット → 設定」し直す。
    //  ★ 電源が一瞬落ちた IMU はスリープ/パワーダウン + 既定レンジに戻っている。
    //    設定し直さないと、読めても値のスケールが違う/ゼロのままになる。
    //    キャリブレーション値 (バイアス) は EEPROM 由来なのでそのまま使える。
    //  ★ begin() と同じ手順 (probe → リセット → CONFIG[] 検証) を踏む。中途半端に
    //    落ちた状態から書くと、書けたつもりで反映されないことがある (2026-09-14)。
    //    チップが違っても手順は同じ (ImuChip に閉じている)。
    void recoverBus() {
        _last_recover_ms = millis();
        _recover_count++;

        wire->end();
#if defined(PIN_WIRE_SDA) && defined(PIN_WIRE_SCL)
        const uint8_t sda_pin = PIN_WIRE_SDA, scl_pin = PIN_WIRE_SCL;
#elif defined(SDA) && defined(SCL)
        const uint8_t sda_pin = SDA, scl_pin = SCL;
#else
        const int sda_pin = -1, scl_pin = -1;
#endif
        if (sda_pin >= 0) {
            pinMode(sda_pin, INPUT_PULLUP);
            pinMode(scl_pin, OUTPUT);
            for (int i = 0; i < 9; ++i) {           // 9 クロックで転送途中のスレーブを解放
                digitalWrite(scl_pin, LOW);  delayMicroseconds(5);
                digitalWrite(scl_pin, HIGH); delayMicroseconds(5);
            }
            pinMode(scl_pin, INPUT_PULLUP);
        }
        wire->begin();
        wire->setClock(400000);
#ifdef ARDUINO_ARCH_RP2040
        wire->setTimeout(I2C_TIMEOUT_MS, true);
#endif
        // チップが戻ってきたか (WHO_AM_I)。戻っていなければ設定しても無駄なので、
        // 次の周期でまた試す (frozenMs が伸びれば S5Failsafe がモーターを止める)。
        _recover_ok = probe();
        if (_recover_ok) {
            if (ImuChip::RESET_WAIT_MS > 0) {
                writeReg(ImuChip::RESET.reg, ImuChip::RESET.val);
                delay(ImuChip::RESET_WAIT_MS);
            }
            _recover_ok = configureChip(/*retries=*/1);
        }
        _bus_recovered = true;                       // 測距センサも入れ直す (drone_s5.cpp)
    }

public:
    IMU(TwoWire *wire_i = &Wire) : wire(wire_i) {};

    // 画面 / セルフテスト表示用
    static constexpr const char* CHIP_NAME = ImuChip::NAME;
    static constexpr const char* I2C_DESC  = ImuChip::I2C_DESC;
    static bool isChipAddr(uint8_t a) { return ImuChip::isAddr(a); }
    uint8_t i2cAddr() const { return _addr; }

    // ------------------------------------------------------------
    //  キャリブレーション値の EEPROM 保存 / 読み出し
    //
    //  Config.h の s_*_bias は inline 変数 = RAM なので、これまで 'k' の
    //  結果は電源を切ると消え、毎回ハードコード値に戻っていた。飛ばす直前に
    //  必ず 'k' を押す運用が要るうえ、押し忘れても何も言われない。
    //  ★ Teensy の EEPROM は flash エミュレーションなので書き換え回数に
    //    限りがある。保存するのは 'k' が成功したときだけ。
    //  ★ magic はチップごとに違う (ImuChip::CAL_MAGIC)。IMU を載せ替えたら
    //    前のチップのバイアスは読まれず、'k' を取り直すまで Config.h の値になる。
    // ------------------------------------------------------------
    struct CalStore {
        uint32_t magic;
        float ax, ay, az, gx, gy, gz;
        uint32_t sum;      // 単純なチェックサム (化けた値を読まないため)
    };
    static constexpr uint32_t CAL_MAGIC = ImuChip::CAL_MAGIC;
    static constexpr int      CAL_ADDR  = 0;

    static uint32_t calSum(const CalStore& c) {
        const uint8_t* p = (const uint8_t*)&c;
        uint32_t s = 0;
        for (size_t i = 0; i < offsetof(CalStore, sum); ++i) s += p[i];
        return s;
    }

    void saveCalibration() {
        CalStore c{};
        c.magic = CAL_MAGIC;
        c.ax = Config::sensor::s_ax_bias;  c.ay = Config::sensor::s_ay_bias;
        c.az = Config::sensor::s_az_bias;  c.gx = Config::sensor::s_gx_bias;
        c.gy = Config::sensor::s_gy_bias;  c.gz = Config::sensor::s_gz_bias;
        c.sum = calSum(c);
        EEPROM.put(CAL_ADDR, c);
#ifdef ARDUINO_ARCH_RP2040
        EEPROM.commit();   // arduino-pico はフラッシュエミュ。commit しないと書かれない
#endif
        Serial.println("INFO: キャリブレーション値を EEPROM に保存しました (次回起動時に読み込みます)");
    }

    // 戻り値: 読み込めたら true
    bool loadCalibration() {
        CalStore c{};
        EEPROM.get(CAL_ADDR, c);
        if (c.magic != CAL_MAGIC || c.sum != calSum(c)) return false;
        Config::sensor::s_ax_bias = c.ax;  Config::sensor::s_ay_bias = c.ay;
        Config::sensor::s_az_bias = c.az;  Config::sensor::s_gx_bias = c.gx;
        Config::sensor::s_gy_bias = c.gy;  Config::sensor::s_gz_bias = c.gz;
        return true;
    }

    // EEPROM の保存値を捨てて Config.h のハードコード値へ戻す
    void clearCalibration() {
        CalStore c{};
        EEPROM.put(CAL_ADDR, c);   // magic が壊れるので次回は読まれない
#ifdef ARDUINO_ARCH_RP2040
        EEPROM.commit();
#endif
        Serial.println("INFO: EEPROM のキャリブレーション値を消しました "
                       "(次回起動から Config.h の値に戻ります)");
    }

    // I2C の 1 トランザクションあたりの上限 [ms]。
    //  ★ 2026-09-18: arduino-pico の TwoWire はタイムアウトを設定しないと Stream の既定
    //    1000ms のまま。IMU の電源が不安定になってバスが応答しなくなると、1 回の
    //    読み書きで最大 1 秒ブロックする。IMU+測距で数回叩くので 1 ループが数秒 =
    //    実機で見えていた「0.2Hz」。短くしておけば、バスが死んでもループは回り続け、
    //    失敗として検出できる (下の update / recoverBus)。
    //    第 2 引数 true = タイムアウトしたら I2C ブロックをリセットする。
    //    (Teensy 時代は I2Cdev::readTimeout=20ms で同じことをしていた。2026-09-05)
    static constexpr uint32_t I2C_TIMEOUT_MS = 10;
    // 連続でこの回数読めなければバス復旧を試みる (10ms タイムアウト × 3 = 30ms)
    static constexpr uint8_t  IO_FAIL_RECOVER = 3;
    // 復旧を試みる最短間隔 [ms] (壊れっぱなしのとき復旧処理でループを埋めない)
    static constexpr uint32_t RECOVER_MIN_INTERVAL_MS = 100;

    void begin() {
#ifdef ARDUINO_ARCH_RP2040
        EEPROM.begin(256);   // arduino-pico はサイズ指定の begin() が必須 (CAL_ADDR + sizeof(Cal) が収まる)
#endif
        wire->begin();
        wire->setClock(400000);
#ifdef ARDUINO_ARCH_RP2040
        wire->setTimeout(I2C_TIMEOUT_MS, /*reset_with_timeout=*/true);
#endif

        if (probe()) {
            Serial.printf("INFO: %s @0x%02X (WHO_AM_I=0x%02X OK)\n", ImuChip::NAME, _addr, ImuChip::WHO_AM_I);
        } else {
            Serial.printf("!!!! FATAL: %s が %s で応答しません (WHO_AM_I 不一致 or 無応答)。"
                          "配線・電源・アドレスジャンパを確認してください。\n",
                          ImuChip::NAME, ImuChip::I2C_DESC);
        }

        // ソフトリセット → 既知の初期状態から設定する (チップが対応していれば)
        if (ImuChip::RESET_WAIT_MS > 0) {
            writeReg(ImuChip::RESET.reg, ImuChip::RESET.val);
            delay(ImuChip::RESET_WAIT_MS);
        }
        configureChip(/*retries=*/3);

        filter.begin(Config::Timing::MAIN_Hz);

        if (loadCalibration()) {
            Serial.println("INFO: EEPROM のキャリブレーション値を読み込みました");
        } else {
            Serial.println("INFO: EEPROM に有効なキャリブレーション値がありません "
                           "→ Config.h の値を使います ('k' で取り直してください)");
        }
        Serial.printf("INFO: Biases ax=%.4f ay=%.4f az=%.4f gx=%.4f gy=%.4f gz=%.4f\n",
                      Config::sensor::s_ax_bias, Config::sensor::s_ay_bias,
                      Config::sensor::s_az_bias, Config::sensor::s_gx_bias,
                      Config::sensor::s_gy_bias, Config::sensor::s_gz_bias);
    }

    // I2C 上でチップが応答し WHO_AM_I が一致するか (起動時のデバイスチェック用)。
    bool connected() { return probe(); }

    // ------------------------------------------------------------
    //  姿勢融合 (Madgwick) の加速度補正の強さ beta
    // ------------------------------------------------------------
    //  Madgwick の補正は「大きさ一定 (2·beta rad/s) で加速度の向きへ寄せ続ける」
    //  スルー。beta=0.1 なら 11.5deg/s。誤差が小さくても同じ速さで寄せるので、
    //  ホバー中の小さな傾きほど相対的に強く加速度側へ引かれる。
    //  クアッドは横に動くと比力 (加速度計の読み) が推力方向 = 機体 z 軸のまま
    //  なので、加速度から見た傾きは「0」に近い嘘になる。beta が大きいと推定角は
    //  真の傾きより小さく・位相が進んで出る。
    //  ★ 2026-09-18 LOG0064 (RP2040 POSHOLD) をジャイロ積分と比べた実測:
    //      0.3〜0.7Hz (位置ループの揺れ帯) で |推定角|/|∫ジャイロ| = 0.34, 位相 +38deg
    //      2〜6Hz                          で                     = 0.57 (← 固定 dt 1ms の分)
    //    同じログを Python の Madgwick に食わせ直すと (dt 正しく):
    //      beta 0.10 → 0.57 (+46deg)   beta 0.03 → 0.92 (+17deg)   beta 0.01 → 0.99 (+6deg)
    //    = 位置ループが「2度傾けろ」と言うと機体は本当は 4度傾いていて 46deg 遅れる。
    //    速度ループのゲイン余裕が半分に見えていた主因で、0.3〜0.45Hz のリミット
    //    サイクルを FLOW_VEL_KP/POS_KP で追いかけ続けていた理由。
    //  運用: 地上 (アーム前) は beta 大で素早く水平に収束させ、アーム中は小さくして
    //  ジャイロ主体にする (QuadConfig.h IMU_FUSION_BETA_GROUND / _FLIGHT)。
    //  ジャイロバイアスは起動時の 'k' で取ってあるので、飛行中の補正は
    //  温度ドリフト分 (1deg/s 未満) を吸えれば足りる。
    void  setFusionBeta(float beta) { filter.setBeta(beta); }
    float fusionBeta() const        { return filter.getBeta(); }

    // ------------------------------------------------------------
    //  update(dt_s)  — 毎メインループ
    //    dt_s : 前回 update() からの実測経過 [s]。0 以下なら begin() の固定周期
    //           (Config::Timing::MAIN_Hz) で積分する (他機体の旧呼び出し互換)。
    //  ★ 2026-09-18: RP2040 では 1000Hz が回らず (実効 630〜700Hz)、固定 1ms で
    //    積分していた Madgwick の角度が真値の 0.6〜0.7 倍に縮んでいた
    //    (LOG0056〜0064 で 2〜6Hz の |角度|/|∫ジャイロ| = 0.57〜0.70)。
    //    レート/角度 PID は実測 dt を使っていたので、姿勢推定だけが「1ms のつもり」だった。
    // ------------------------------------------------------------
    void update(float dt_s = 0.0f) {
        // センサから生データを読み出す。失敗した回は前回値を残したまま何もしない
        // (古い値で姿勢推定を進めない)。
        //  ★ 2026-09-18 LOG0065: IMU の電源が不安定 → I2C が応答しなくなり、
        //    ループが「IMU 読み」の区間で止まってウォッチドッグがリセットした。
        if (!readRaw()) {
            _io_fail++;
            _io_fail_total++;
            const uint32_t now = millis();
            if (_io_fail >= IO_FAIL_RECOVER && (now - _last_recover_ms) >= RECOVER_MIN_INTERVAL_MS)
                recoverBus();
            return;
        }
        _io_fail = 0;

        // 読めていても値が動かない (センサが固まっている) ことがある。6 軸が完全に
        // 同じ = 新しい値が来ていない。センサはノイズで必ず揺れるので正常では起きない。
        const uint32_t now_ms = millis();
        if (ax_raw == _p_ax && ay_raw == _p_ay && az_raw == _p_az &&
            gx_raw == _p_gx && gy_raw == _p_gy && gz_raw == _p_gz) {
            if (_same_since_ms == 0) _same_since_ms = now_ms;
        } else {
            _same_since_ms = 0;
        }
        _p_ax = ax_raw; _p_ay = ay_raw; _p_az = az_raw;
        _p_gx = gx_raw; _p_gy = gy_raw; _p_gz = gz_raw;

        // --- ソフトウェア・キャリブレーション補正 ---
        float c_ax = getAccX();
        float c_ay = getAccY();
        float c_az = getAccZ();
        float c_gx = getGyroX();
        float c_gy = getGyroY();
        float c_gz = getGyroZ();

        // Madgwickフィルタの更新。実測 dt で積分する。
        //  I2C 復旧などでループが長く止まった直後の dt は上限で切る (その間の回転は
        //  どうせ分からない。加速度補正が数百 ms で引き戻す)。
        if (dt_s > 0.0f) filter.updateIMU(c_gx, c_gy, c_gz, c_ax, c_ay, c_az, fminf(dt_s, MAX_FUSION_DT_S));
        else             filter.updateIMU(c_gx, c_gy, c_gz, c_ax, c_ay, c_az);
    }
    static constexpr float MAX_FUSION_DT_S = 0.05f;

    // 生データスケール変換のみ (Raw Scaled)
    float getAccX_Raw() { return (float)ax_raw / ImuChip::ACCEL_SCALE; }
    float getAccY_Raw() { return (float)ay_raw / ImuChip::ACCEL_SCALE; }
    float getAccZ_Raw() { return (float)az_raw / ImuChip::ACCEL_SCALE; }
    float getGyroX_Raw() { return (float)gx_raw / ImuChip::GYRO_SCALE; }
    float getGyroY_Raw() { return (float)gy_raw / ImuChip::GYRO_SCALE; }
    float getGyroZ_Raw() { return (float)gz_raw / ImuChip::GYRO_SCALE; }

    // ソフトウェア補正適用済みの値 (機体座標系: Forward, Left, Up)。符号は上の IMU_ACC_* / IMU_GYR_*
    float getAccX()  { return IMU_ACC_SX * (getAccX_Raw() - Config::sensor::s_ax_bias); }
    float getAccY()  { return IMU_ACC_SY * (getAccY_Raw() - Config::sensor::s_ay_bias); }
    float getAccZ()  { return IMU_ACC_SZ * (getAccZ_Raw() - Config::sensor::s_az_bias); }

    float getGyroX() { return IMU_GYR_SX * (getGyroX_Raw() - Config::sensor::s_gx_bias); }
    float getGyroY() { return IMU_GYR_SY * (getGyroY_Raw() - Config::sensor::s_gy_bias); }
    float getGyroZ() { return IMU_GYR_SZ * (getGyroZ_Raw() - Config::sensor::s_gz_bias); }

    // 生データが変わらないまま続いている時間 [ms] (0 = 直近で変化あり)
    uint32_t frozenMs() const { return (_same_since_ms == 0) ? 0 : (millis() - _same_since_ms); }

    // --- I2C の健康状態 (画面 / ログ用) ---
    uint32_t ioFailTotal()  const { return _io_fail_total; }   // 読み出しに失敗した回数
    uint16_t recoverCount() const { return _recover_count; }   // バス復旧を試みた回数
    // バス復旧した直後に 1 回だけ true (同じバスの測距センサも入れ直すため)
    bool consumeBusRecovered() { const bool f = _bus_recovered; _bus_recovered = false; return f; }
    bool recoverOk() const { return _recover_ok; }   // 直近の復旧でチップが戻ったか

    float getRoll()  { return filter.getRoll(); }
    float getPitch() { return filter.getPitch(); }
    float getYaw()   { return filter.getYaw(); }

    // 戻り値: 採用されたら true。妥当性チェックで却下されたら false
    //  (呼び出し側が discard しても既存の呼び方はそのまま動く)。
    //  ★ 2026-09-14: 遠隔からの再キャリブレーション (drone_s5.cpp
    //    handleRemoteAction()) が、拒否されたことを地上局へ伝えるために
    //    戻り値を見る。シリアル 'k' は従来どおり画面のメッセージだけ見る。
    //  onProgress: サンプリング中に定期的に呼ばれる (LED演出などの視覚フィードバック用)。
    //    省略可。呼び出し頻度は samples 内部で決める実装依存。
    bool recalibrate(void (*onProgress)() = nullptr) {
        Serial.printf("INFO: %s Recalibration (ax=0, ay=0, az=1 mode)...\n", ImuChip::NAME);

        // 却下したときに戻せるよう、今の値を退避しておく
        const float old_ax = Config::sensor::s_ax_bias;
        const float old_ay = Config::sensor::s_ay_bias;
        const float old_az = Config::sensor::s_az_bias;
        const float old_gx = Config::sensor::s_gx_bias;
        const float old_gy = Config::sensor::s_gy_bias;
        const float old_gz = Config::sensor::s_gz_bias;

        Config::sensor::s_ax_bias = 0.0f;
        Config::sensor::s_ay_bias = 0.0f;
        Config::sensor::s_az_bias = 0.0f;
        Config::sensor::s_gx_bias = 0.0f;
        Config::sensor::s_gy_bias = 0.0f;
        Config::sensor::s_gz_bias = 0.0f;

        double sum_ax=0, sum_ay=0, sum_az=0;
        double sum_gx=0, sum_gy=0, sum_gz=0;
        // 「動いていないか」を見るためにジャイロの振れ幅も取る
        float min_gx=1e9f, max_gx=-1e9f, min_gy=1e9f, max_gy=-1e9f,
              min_gz=1e9f, max_gz=-1e9f;
        const int samples = 400;
        int n_fail = 0;

        for(int i=0; i<samples; i++) {
            if (!readRaw()) {
                // 読めなかった回は数に入れない。続くようならバスが死んでいるので中断。
                if (++n_fail > MAX_CAL_IO_FAIL) break;
                --i;
                delay(2);
                continue;
            }
            sum_ax += getAccX_Raw();
            sum_ay += getAccY_Raw();
            sum_az += getAccZ_Raw();
            const float gxv = getGyroX_Raw();
            const float gyv = getGyroY_Raw();
            const float gzv = getGyroZ_Raw();
            sum_gx += gxv;  sum_gy += gyv;  sum_gz += gzv;
            min_gx = fminf(min_gx, gxv);  max_gx = fmaxf(max_gx, gxv);
            min_gy = fminf(min_gy, gyv);  max_gy = fmaxf(max_gy, gyv);
            min_gz = fminf(min_gz, gzv);  max_gz = fmaxf(max_gz, gzv);
            if (i % 100 == 0) Serial.print(".");
            if (onProgress && (i % 40 == 0)) onProgress();
            delay(2);
        }
        Serial.println(" Done.");

        const float m_ax = (float)(sum_ax / samples);
        const float m_ay = (float)(sum_ay / samples);
        const float m_az = (float)(sum_az / samples);
        const float m_gx = (float)(sum_gx / samples);
        const float m_gy = (float)(sum_gy / samples);
        const float m_gz = (float)(sum_gz / samples);

        // ------------------------------------------------------------
        //  ★ 妥当性チェック (2026-09-04 追加)
        //
        //  以前はここで無条件に採用していた。そのため、傾いた床・手に持った
        //  状態・裏返しで 'k' を押すと、その姿勢が黙って「水平」として
        //  登録され、機体はその方向へ飛んでいく。エラーも警告も出ないので
        //  外からは絶対に気づけない。実機で az = -1.01g (裏返し) のまま
        //  キャリブレーションが通ってしまうのを確認した。
        //
        //  水平に静止していれば必ず ax≈0, ay≈0, az≈+1, ジャイロの振れ幅も
        //  小さい。そうでなければ採用せず、元の値を残す。
        // ------------------------------------------------------------
        // FLU の上向き成分。IMU_MOUNT 定義環境では符号込みで +1g でなければならない
        // (Madgwick は静止時の重力反力が +Z にある前提。逆だと 180° ずれた姿勢に収束する)。
        // 旧来の環境は上下逆マウントを |az| で通していたので、その挙動を残す。
#if defined(IMU_MOUNT_X)
        const float az_up = IMU_ACC_SZ * m_az;
#else
        const float az_up = fabsf(m_az);
#endif
        const float tilt_deg = atan2f(sqrtf(m_ax * m_ax + m_ay * m_ay),
                                      az_up) * 57.2957795f;
        const float g_norm   = sqrtf(m_ax * m_ax + m_ay * m_ay + m_az * m_az);
        const float g_span   = fmaxf(fmaxf(max_gx - min_gx, max_gy - min_gy),
                                     max_gz - min_gz);

        const char* reason = nullptr;
        if (n_fail > MAX_CAL_IO_FAIL)          reason = "IMU が I2C で読めません (配線 / 電源 / アドレスジャンパ)";
        else if (g_norm < 0.85f || g_norm > 1.15f)
                                               reason = "加速度の大きさが 1g から外れています (動いている / センサ異常)";
        else if (az_up <= 0.0f)                reason = "重力が -Z 側に出ています (IMU_MOUNT の符号と実機の上下が逆 / 裏返し)";
        else if (tilt_deg > MAX_CAL_TILT_DEG)  reason = "機体が傾いています";
        else if (g_span > MAX_CAL_GYRO_SPAN)   reason = "機体が動いています";

        if (reason) {
            // 採用しない。退避しておいた元の値を戻す。
            Config::sensor::s_ax_bias = old_ax;
            Config::sensor::s_ay_bias = old_ay;
            Config::sensor::s_az_bias = old_az;
            Config::sensor::s_gx_bias = old_gx;
            Config::sensor::s_gy_bias = old_gy;
            Config::sensor::s_gz_bias = old_gz;
            Serial.println();
            Serial.printf("!! CALIBRATION REJECTED: %s\n", reason);
            Serial.printf("   実測 ax=%+.4f ay=%+.4f az=%+.4f |a|=%.4f g   "
                          "傾き %.1f deg   ジャイロ振れ幅 %.2f deg/s\n",
                          m_ax, m_ay, m_az, g_norm, tilt_deg, g_span);
            Serial.printf("   許容: 傾き < %.1f deg / ジャイロ振れ幅 < %.1f deg/s / |a| ≒ 1g\n",
                          MAX_CAL_TILT_DEG, MAX_CAL_GYRO_SPAN);
#if defined(IMU_MOUNT_X)
            Serial.printf("   (IMU_MOUNT 符号込みで az≈+1g が必要。符号 Z=%+.0f)\n", IMU_ACC_SZ);
#else
            Serial.println("   (az の符号は問わない。IMU 上下逆マウントでも az≈-1g で通る)");
#endif
            Serial.println("   → 水平な床に置き、手を離して静止させてから 'k' を押し直してください。");
            Serial.println("   キャリブレーション値は変更していません (前の値のまま)。");
            return false;
        }

        Config::sensor::s_ax_bias = m_ax;
        Config::sensor::s_ay_bias = m_ay;
        // getAccZ() = SZ * (raw - bias) が +1g になるバイアス。SZ=+1 なら従来の m_az - 1
        Config::sensor::s_az_bias = m_az - IMU_ACC_SZ;

        Config::sensor::s_gx_bias = m_gx;
        Config::sensor::s_gy_bias = m_gy;
        Config::sensor::s_gz_bias = m_gz;

        saveCalibration();   // 電源を切っても残るように EEPROM へ

        filter.reset();
        filter.begin(Config::Timing::MAIN_Hz);
        Serial.printf("INFO: %s Recalibration Finished.\n", ImuChip::NAME);
        Serial.printf("INFO: New Biases: ax=%.4f ay=%.4f az=%.4f gx=%.4f gy=%.4f gz=%.4f\n",
                      Config::sensor::s_ax_bias, Config::sensor::s_ay_bias, Config::sensor::s_az_bias,
                      Config::sensor::s_gx_bias, Config::sensor::s_gy_bias, Config::sensor::s_gz_bias);
        return true;
    }
};
