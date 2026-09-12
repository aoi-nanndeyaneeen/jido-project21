// ============================================================
//  OpticalFlow.h  -  PMW3901 オプティカルフローの機体側ラッパ
// ============================================================
//  Stage 5 (drone_s5.cpp) で使用。
//
//  やっていること:
//    1. PMW3901 から「前回読み出しからの移動カウント(ピクセル)」を取得
//    2. QuadConfig.h の符号 / スワップで機体座標 (FRD: 前=x, 右=y) に合わせる
//    3. de-rotation: 機体の回転が作る「見かけの流れ」をジャイロで差し引く
//       (これを省くと、傾いた瞬間に偽の並進速度が出て発振する)
//    4. 高度を与えて対地速度 [m/s] に換算する
//
//  ★ 高度は外から与える。距離センサが無い間 (s5a / s5b) は
//    QuadConfig.h の FLOW_ASSUMED_HEIGHT_M が初期値として入る。
//    距離センサが載ったら (s5c) 測距値を毎ループ setHeight() するだけ。
//
//  依存: bitcraze/Bitcraze PMW3901  (platformio.ini の lib_deps に既に有り)
//        グローバル SPI              (Teensy 4.0: SCK=13 MOSI=11 MISO=12)
//
//  ※ 既存の他機体コードには影響しません (このヘッダを include するのは
//    drone_s5.cpp だけ)。
// ============================================================
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "Bitcraze_PMW3901.h"
#include "quad/QuadConfig.h"

class OpticalFlow {
public:
    explicit OpticalFlow(uint8_t cs_pin = (uint8_t)Quad::FLOW_CS_PIN)
        : _cs(cs_pin), _sensor(cs_pin) {}

    // 戻り値 false = PMW3901 が応答しない (配線 / SPI / 電源を確認)
    bool begin() {
        _ok_init  = _sensor.begin();
        _height_m = Quad::FLOW_ASSUMED_HEIGHT_M;
        return _ok_init;
    }
    bool initialized() const { return _ok_init; }

    // 換算スケールに使う高度 [m] をセットする。
    // 極端な値は無視して直前の値を保持する (測距センサの外れ値対策)。
    void setHeight(float h_m) {
        if (h_m > 0.05f && h_m < 40.0f) _height_m = h_m;
    }
    float height() const { return _height_m; }

    // ------------------------------------------------------------
    //  update()  — FLOW_LOOP_HZ (読み出しレート) で毎回呼ぶ
    //    dt_s                : 前回 update() からの経過 [s]
    //    gyro_roll_rate_dps  : 機体 X軸まわり角速度 [deg/s]  (FRD: 右バンク +)
    //    gyro_pitch_rate_dps : 機体 Y軸まわり角速度 [deg/s]  (FRD: 機首上げ +)
    //
    //  ★ 2026-09-11: 読み出しと制御を分離。毎回センサを読んで
    //    「de-rotate 済み変位」を _acc_* に積算し、FLOW_CTRL_DIV 読みごとに
    //    1 回だけ窓を締めて vx/vy を作る (そのとき _fresh=true)。
    //    呼び出し側は consumeFresh() が true の回だけ PosHold を回す。
    //    FLOW_CTRL_DIV==1 なら毎回窓が締まる = 旧挙動。
    //  ジャイロは BodyFrame.h の Attitude.roll_rate / pitch_rate をそのまま渡す。
    // ------------------------------------------------------------
    void update(float dt_s, float gyro_roll_rate_dps, float gyro_pitch_rate_dps) {
        if (!_ok_init || dt_s <= 0.0f) return;

        int16_t dx = 0, dy = 0;
        if (Quad::FLOW_USE_BURST) { readMotionBurst(_cs, &dx, &dy, &_squal); }
        else                      { _sensor.readMotionCount(&dx, &dy); _squal = 255; }

        // 生カウント → 機体座標 (FRD)
        float fx = (float)dx, fy = (float)dy;
        if (Quad::FLOW_SWAP_XY) { const float t = fx; fx = fy; fy = t; }
        fx *= Quad::FLOW_SIGN_X;
        fy *= Quad::FLOW_SIGN_Y;

        // --- SQUAL 床下のサンプルは窓に入れない (追える模様が無い = ゼロを混ぜる害) ---
        const bool lowq = Quad::FLOW_USE_BURST && (_squal < Quad::FLOW_SQUAL_MIN);
        if (lowq) {
            _lowqual_s += dt_s;
        } else {
            _lowqual_s = 0.0f;
            _acc_raw_x += fx;
            _acc_raw_y += fy;
            // de-rotation はサンプルごとに引いてから積算する。
            //  見かけ流量 [px] = PX_PER_RAD * 角速度[rad/s] * dt。窓内で姿勢が
            //  変わっても各サンプルの dt/ジャイロで正しく打ち消せる。
            if (Quad::FLOW_DEROTATE) {
                const float gr = gyro_roll_rate_dps  * DEG2RAD;
                const float gp = gyro_pitch_rate_dps * DEG2RAD;
                _acc_gyro_x += Quad::FLOW_DEROT_SIGN_X * Quad::FLOW_PX_PER_RAD * gp * dt_s;
                _acc_gyro_y += Quad::FLOW_DEROT_SIGN_Y * Quad::FLOW_PX_PER_RAD * gr * dt_s;
            }
            _acc_dt += dt_s;
        }

        // --- 窓がまだ埋まっていなければここまで ---
        if (++_read_ctr < Quad::FLOW_CTRL_DIV) return;
        _read_ctr = 0;

        const float T = _acc_dt;
        if (T <= 0.0f) {
            // 窓が丸ごと lowq だった: 速度 0 として制御へ (機体が動いても
            //  補正しない = 危険だが、SQUAL が戻るまでの短時間の話。
            //  _lowqual_s が FLOW_DEAD_S を超えたら suspectDead で POSHOLD 解除)。
            raw_x = raw_y = derot_x = derot_y = vx = vy = 0.0f;
            _last_dt = (float)Quad::FLOW_CTRL_DIV * dt_s;
            _acc_raw_x = _acc_raw_y = _acc_gyro_x = _acc_gyro_y = _acc_dt = 0.0f;
            _zero_run_s += _last_dt;
            _fresh = true;
            return;
        }

        raw_x   = _acc_raw_x;
        raw_y   = _acc_raw_y;
        derot_x = _acc_raw_x + _acc_gyro_x;    // de-rotation 済み変位 [px]
        derot_y = _acc_raw_y + _acc_gyro_y;

        // 対地速度 [m/s] = (変位[px] / T) / PX_PER_RAD * height
        const float k = _height_m / (Quad::FLOW_PX_PER_RAD * T);
        vx = derot_x * k;
        vy = derot_y * k;

        // 窓合計が (0,0) に丸まった = センサ凍結の疑い。SQUAL 判定の保険
        //  (burst 前の readMotionCount 経路ではこちらだけが効く)。
        if (lroundf(raw_x) == 0 && lroundf(raw_y) == 0) _zero_run_s += T;
        else                                            _zero_run_s = 0.0f;

        _last_dt = T;
        _acc_raw_x = _acc_raw_y = _acc_gyro_x = _acc_gyro_y = _acc_dt = 0.0f;
        _fresh = true;
    }

    // 直近 update() の結果 -----------------------------------------
    float raw_x   = 0.0f, raw_y   = 0.0f;   // 機体座標に直しただけの生カウント [px]
    float derot_x = 0.0f, derot_y = 0.0f;   // de-rotation 後 [px]
    float vx      = 0.0f, vy      = 0.0f;   // 対地速度 [m/s] (前 +, 右 +)

    float lastDt() const { return _last_dt; }
    // 表示 / ログで「今ループで更新されたか」を知りたいとき用
    bool  consumeFresh() { const bool f = _fresh; _fresh = false; return f; }

    // 生カウントが厳密に (0,0) のまま続いている時間 [s]。
    //  「センサが死んでいる」判定そのものは呼び出し側が行う (上のコメント参照)。
    //  地上で静止していても伸びるので、これ単体では異常を意味しない。
    float zeroRunS() const { return _zero_run_s; }

    // Quad::FLOW_DEAD_S を超えて「模様なし」が続いたか。飛行中なら固まったとみなす。
    //  burst: SQUAL 床下が続く / または窓合計 (0,0) が続く のどちらか。
    //  非burst: 窓合計 (0,0) が続く のみ (SQUAL は 255 固定なので効かない)。
    bool  suspectDead() const {
        return _zero_run_s >= Quad::FLOW_DEAD_S
            || (Quad::FLOW_USE_BURST && _lowqual_s >= Quad::FLOW_DEAD_S);
    }

    // 直近の窓積算に使った秒数 (= 制御 dt)。窓 dt。
    float lowQualS() const { return _lowqual_s; }

    // 直近バースト読みの表面品質 (SQUAL)。低い = フローが当てにならない。
    // FLOW_USE_BURST=false のときは 0 のまま。
    uint8_t squal() const { return _squal; }

private:
    static constexpr float DEG2RAD = 0.01745329252f;

    // --- PMW3901 モーションバースト読み出し -------------------------------
    //  ライブラリの readMotionCount() は registerRead を5回、各 200us の
    //  delayMicroseconds を挟むので 1 回 ~1ms ブロックする。FLOW_LOOP_HZ を
    //  上げるとレートループ (1kHz) を潰すため 20Hz に抑えられていた。
    //  バーストは 1 トランザクションで 12 バイトをバイト間ディレイ無しで
    //  読むので ~80us。手順 (データシート §MOTION Burst):
    //    CS LOW → 0x16 送信 → tSRAD 待ち → 12 バイト連続読み → CS HIGH。
    //  CS はバースト中トグルしないこと。SPI 設定はライブラリと同一
    //  (4MHz / MSBFIRST / MODE3)。
    //    buf[0]=Motion buf[1]=Observation
    //    buf[2..3]=DeltaX_L,H  buf[4..5]=DeltaY_L,H  buf[6]=SQUAL ...
    static void readMotionBurst(uint8_t cs, int16_t* dx, int16_t* dy,
                                uint8_t* squal) {
        uint8_t buf[12];
        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
        digitalWrite(cs, LOW);
        delayMicroseconds(50);
        SPI.transfer(0x16);            // Motion_Burst レジスタ
        delayMicroseconds(50);         // tSRAD (データシート 35us、余裕をみて 50)
        for (int i = 0; i < 12; ++i) buf[i] = SPI.transfer(0);
        digitalWrite(cs, HIGH);
        SPI.endTransaction();
        *dx = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
        *dy = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
        *squal = buf[6];
    }

    uint8_t _cs;
    uint8_t _squal = 0;
    Bitcraze_PMW3901 _sensor;
    bool  _ok_init    = false;
    float _height_m   = 1.0f;
    float _last_dt    = 0.0f;
    bool  _fresh      = false;
    float _zero_run_s = 0.0f;   // 窓合計が (0,0) のまま続いた時間 [s]
    float _lowqual_s  = 0.0f;   // SQUAL 床下が続いた時間 [s]

    // 制御窓の積算 (FLOW_CTRL_DIV 読みぶん)
    float    _acc_raw_x  = 0.0f, _acc_raw_y  = 0.0f;   // 生カウント合計 [px]
    float    _acc_gyro_x = 0.0f, _acc_gyro_y = 0.0f;   // de-rotation 補正合計 [px]
    float    _acc_dt     = 0.0f;                       // 窓の経過 [s]
    uint16_t _read_ctr   = 0;                          // 窓内の読み回数
};
