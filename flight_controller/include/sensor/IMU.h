#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>
#include "Config.h"

class IMU {
private:
    // recalibrate() の妥当性チェックのしきい値。
    //  傾き: 水平な床に置いたつもりでも脚の高さで数度は出るので 5度まで許す。
    //        それを超えたら「水平ではない」と見なして採用しない。
    //  ジャイロ振れ幅: 手で持っているとすぐ数 deg/s 動く。静止なら 1 未満。
    //
    //  ★ この機体は配線の都合で加速度センサを上下逆に載せている。水平時の
    //    az は -1g になるのが正常。したがって「az が負なら裏返し」という
    //    判定はできない。|az| が 1g 付近にあるか、で見る。
    //    (2026-09-04 に az<0 を却下条件にしてしまい、正常な姿勢での 'k' が
    //     通らなくなった。その修正)
    static constexpr float MAX_CAL_TILT_DEG  = 5.0f;
    static constexpr float MAX_CAL_GYRO_SPAN = 3.0f;

    MPU6050 mpu;
    Madgwick filter;
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    TwoWire *wire;

    // MPU6050 のレジスタに書き込み、読み戻して一致するまで数回リトライする。
    //  ★ 2026-09-14: レンジ設定 (0x1C) の書き込みが黙って反映されないケースが
    //    あり、ソフトとセンサーのスケール食い違いに何日も気づけなかった。
    //    ここで検証しないと同じ事故が再発する。
    bool writeRegVerified(uint8_t reg, uint8_t value, uint8_t retries = 3) {
        for (uint8_t attempt = 0; attempt <= retries; ++attempt) {
            wire->beginTransmission(0x68);
            wire->write(reg);
            wire->write(value);
            wire->endTransmission();
            delay(2);

            wire->beginTransmission(0x68);
            wire->write(reg);
            wire->endTransmission(false);
            wire->requestFrom((uint8_t)0x68, (uint8_t)1);
            if (wire->available()) {
                const uint8_t readback = wire->read();
                if (readback == value) return true;
            }
            delay(5);
        }
        return false;
    }

public:
    IMU(TwoWire *wire_i = &Wire) : mpu(0x68), wire(wire_i) {};

    // ------------------------------------------------------------
    //  キャリブレーション値の EEPROM 保存 / 読み出し
    //
    //  Config.h の s_*_bias は inline 変数 = RAM なので、これまで 'k' の
    //  結果は電源を切ると消え、毎回ハードコード値に戻っていた。飛ばす直前に
    //  必ず 'k' を押す運用が要るうえ、押し忘れても何も言われない。
    //  ★ Teensy の EEPROM は flash エミュレーションなので書き換え回数に
    //    限りがある。保存するのは 'k' が成功したときだけ。
    // ------------------------------------------------------------
    struct CalStore {
        uint32_t magic;
        float ax, ay, az, gx, gy, gz;
        uint32_t sum;      // 単純なチェックサム (化けた値を読まないため)
    };
    static constexpr uint32_t CAL_MAGIC = 0x43414C32;  // "CAL2"
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
        Serial.println("INFO: EEPROM のキャリブレーション値を消しました "
                       "(次回起動から Config.h の値に戻ります)");
    }

    void begin() {
        wire->begin();
        wire->setClock(400000);

        // ★ 2026-09-05: I2Cdev のデフォルト読み取りタイムアウトは1000ms。
        //   getMotion6() はメインループの先頭で毎回呼ぶブロッキング呼び出しなので、
        //   I2Cバスが一時的に応答しなくなる (振動による接触不良など) と、
        //   そのたびに最大1秒まるごと制御ループが停止する。実機ログで
        //   約5.86秒(≒1000ms×6回)の完全停止を確認した。
        //   20msに縮めておけば、同じグリッチが起きても被害を数十msに抑えられる。
        //   根本原因 (配線・振動) は別途対策が必要。
        I2Cdev::readTimeout = 20;

        mpu.initialize();

        // --- スケール強制設定 MPU6050 (±8g / ±250dps) ---
        //  ★ 2026-09-09: Accel を ±2g(0x00) -> ±8g(0x10) に。理由は Config.h の
        //    ACCEL_SCALE のコメント参照 (上下逆マウント + s_az_bias≈-2 で上向き
        //    加速の余裕が 1g しか無く、AltEstimator が飽和していた)。
        //    ACCEL_SCALE = 4096.0f と対で変えること。
        //  ★ 2026-09-14: この書き込みが (I2Cバスのノイズ or MPU6050 が
        //    initialize() 直後の内部リセット中で書き込みを無視した等の理由で)
        //    黙って反映されないケースを実機で確認した。ソフトは ±8g のつもりで
        //    ACCEL_SCALE=4096 で割り続けるが、実際のセンサーが ±2g のままだと
        //    読み値が正確に4倍(16384/4096)に膨らみ、az が水平でも -4g 付近に
        //    見える。この状態だと recalibrate() の |a|≒1g チェックに必ず
        //    REJECTED され、EEPROM のバイアスは (スケールが正しかった頃の)
        //    古い値のまま固定されてしまい、姿勢推定が発散する
        //    (roll_ang が 180° 付近に張り付く不具合の原因になった)。
        //    書き込み後に読み戻して確認し、ズレていたら再送・それでもダメなら
        //    起動時に大きく警告を出す。
        if (!writeRegVerified(0x1C, 0x10)) {           // Accel ±8g
            Serial.println("!!!! FATAL: MPU6050 ACCEL_CONFIG (±8g) が反映されていません。"
                           "加速度スケールが実際と食い違うため、姿勢推定が壊れます。"
                           "I2C配線・MPU6050の電源を確認してください。");
        }
        if (!writeRegVerified(0x1B, 0x00)) {           // Gyro ±250dps
            Serial.println("!!!! WARN: MPU6050 GYRO_CONFIG (±250dps) の書き込み確認に失敗しました。");
        }
        if (!writeRegVerified(0x1A, 0x03)) {           // DLPF 42Hz
            Serial.println("!!!! WARN: MPU6050 DLPF 設定の書き込み確認に失敗しました。");
        }

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

    // I2C 上で MPU6050 が応答し WHO_AM_I が一致するか (起動時のデバイスチェック用)。
    bool connected() { return mpu.testConnection(); }

    void update() {
        // センサから生データを読み出す
        mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

        // --- ソフトウェア・キャリブレーション補正 ---
        float c_ax = getAccX();
        float c_ay = getAccY();
        float c_az = getAccZ();
        float c_gx = getGyroX();
        float c_gy = getGyroY();
        float c_gz = getGyroZ();

        // Madgwickフィルタの更新
        filter.updateIMU(c_gx, c_gy, c_gz, c_ax, c_ay, c_az);
    }

    // 生データスケール変換のみ (Raw Scaled)
    float getAccX_Raw() { return (float)ax_raw / Config::sensor::ACCEL_SCALE; }
    float getAccY_Raw() { return (float)ay_raw / Config::sensor::ACCEL_SCALE; }
    float getAccZ_Raw() { return (float)az_raw / Config::sensor::ACCEL_SCALE; }
    float getGyroX_Raw() { return (float)gx_raw / Config::sensor::GYRO_SCALE; }
    float getGyroY_Raw() { return (float)gy_raw / Config::sensor::GYRO_SCALE; }
    float getGyroZ_Raw() { return (float)gz_raw / Config::sensor::GYRO_SCALE; }
    
    // ソフトウェア補正適用済みの値 (機体座標系: Forward, Left, Up)
    float getAccX()  { return (getAccX_Raw() - Config::sensor::s_ax_bias); } // 前 = -X
    float getAccY()  { return (getAccY_Raw() - Config::sensor::s_ay_bias); } // 左 = -Y
    float getAccZ()  { return (getAccZ_Raw() - Config::sensor::s_az_bias); } // 上 = Z

    float getGyroX() { return -(getGyroX_Raw() - Config::sensor::s_gx_bias); }
    float getGyroY() { return -(getGyroY_Raw() - Config::sensor::s_gy_bias); }
    float getGyroZ() { return  (getGyroZ_Raw() - Config::sensor::s_gz_bias); }

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
        Serial.println("INFO: MPU6050 Recalibration (ax=0, ay=0, az=1 mode)...");
        
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

        for(int i=0; i<samples; i++) {
            int16_t r_ax, r_ay, r_az, r_gx, r_gy, r_gz;
            mpu.getMotion6(&r_ax, &r_ay, &r_az, &r_gx, &r_gy, &r_gz);
            sum_ax += (float)r_ax / Config::sensor::ACCEL_SCALE;
            sum_ay += (float)r_ay / Config::sensor::ACCEL_SCALE;
            sum_az += (float)r_az / Config::sensor::ACCEL_SCALE;
            const float gxv = (float)r_gx / Config::sensor::GYRO_SCALE;
            const float gyv = (float)r_gy / Config::sensor::GYRO_SCALE;
            const float gzv = (float)r_gz / Config::sensor::GYRO_SCALE;
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
        // 重力ベクトルが Z 軸からどれだけ離れているか。センサの上下向きに
        // 依存しないよう |az| で測る (上下逆マウントでも同じ式が使える)。
        const float tilt_deg = atan2f(sqrtf(m_ax * m_ax + m_ay * m_ay),
                                      fabsf(m_az)) * 57.2957795f;
        const float g_norm   = sqrtf(m_ax * m_ax + m_ay * m_ay + m_az * m_az);
        const float g_span   = fmaxf(fmaxf(max_gx - min_gx, max_gy - min_gy),
                                     max_gz - min_gz);

        const char* reason = nullptr;
        if (g_norm < 0.85f || g_norm > 1.15f)
                                               reason = "加速度の大きさが 1g から外れています (動いている / センサ異常)";
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
            Serial.println("   (az の符号は問わない。IMU 上下逆マウントでも az≈-1g で通る)");
            Serial.println("   → 水平な床に置き、手を離して静止させてから 'k' を押し直してください。");
            Serial.println("   キャリブレーション値は変更していません (前の値のまま)。");
            return false;
        }

        Config::sensor::s_ax_bias = m_ax;
        Config::sensor::s_ay_bias = m_ay;
        Config::sensor::s_az_bias = m_az - 1.0f;

        Config::sensor::s_gx_bias = m_gx;
        Config::sensor::s_gy_bias = m_gy;
        Config::sensor::s_gz_bias = m_gz;

        saveCalibration();   // 電源を切っても残るように EEPROM へ

        filter.reset();
        filter.begin(Config::Timing::MAIN_Hz);
        Serial.println("INFO: MPU6050 Recalibration Finished.");
        Serial.printf("INFO: New Biases: ax=%.4f ay=%.4f az=%.4f gx=%.4f gy=%.4f gz=%.4f\n",
                      Config::sensor::s_ax_bias, Config::sensor::s_ay_bias, Config::sensor::s_az_bias,
                      Config::sensor::s_gx_bias, Config::sensor::s_gy_bias, Config::sensor::s_gz_bias);
        return true;
    }
};
