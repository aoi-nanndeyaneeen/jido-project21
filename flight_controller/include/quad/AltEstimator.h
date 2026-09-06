// ============================================================
//  AltEstimator.h  -  加速度Z × 測距 の相補フィルタ (高度・上昇速度の推定)
// ============================================================
//  【なぜ要るのか】
//
//  従来の climb (上昇速度) は 100% 測距センサ由来だった:
//      測距 → cos補正 → H-LPF → 差分 → VZ-LPF → climb
//  この経路の遅れを log_037 で実測すると、実効スロットル → climb で 420ms。
//  内訳は
//      センサ露光      約 30 ms   (VL53L1X の timing budget)
//      ZOH             約 15 ms   (サンプル周期の半分)
//      H/VZ の二重LPF  約 100 ms  (33Hz 時。16Hz 時は約 200 ms だった)
//      物理応答        100〜150 ms (スロットル→推力→速度。これは消せない)
//  フィードバックループに 0.4 秒の遅れがある系は、どんなゲインでも
//  「遅くて振れる」以外にならない。
//
//  【考え方】
//  加速度計と測距センサは、弱点がちょうど逆:
//      加速度計 : 1000Hz・遅れほぼゼロ。ただし積分するとドリフトする
//      測距     : 絶対値は正確でドリフトしない。ただし遅くてノイジー
//  そこで「速い成分は加速度から、遅い成分は測距から」取る。これが相補フィルタ。
//  こうすると測距の微分に重い LPF をかける必要そのものが無くなるので、
//  上の 100ms がまるごと消える。残るのは物理応答だけ。
//
//  【構成】3状態の相補フィルタ (三重極を -w に置いたもの)
//      状態: 高度 h [m] / 上昇速度 vz [m/s] / 加速度バイアス b [m/s^2]
//
//      predict()  毎ループ (1000Hz):
//          a = a_up - b
//          h += vz*dt + 0.5*a*dt^2
//          vz += a*dt
//
//      correct()  測距が来たとき (約33Hz):
//          err = h_meas - h
//          h  += 3w   * err * dt
//          vz += 3w^2 * err * dt
//          b  -= w^3  * err * dt
//
//  b があるおかげで、加速度計のバイアス (キャリブ残差・温度ドリフト・
//  取付角のわずかなずれ) を勝手に学習して消してくれる。これが無いと
//  積分が青天井にドリフトする。
//
//  【重要: まだ制御には使わないこと】
//  Q::ALT_USE_ACC_FUSION = false の間は、この推定値はログに出るだけで
//  AltHold は従来どおり測距由来の climb を使う。理由は2つ:
//    1. 加速度の符号 (ALT_ACC_Z_SIGN) が実機で確認できていない。
//       IMU が上下逆マウントだと符号が反転する。
//       -> analyze_alt_pid.py が次のログで自動判定して教えてくれる。
//    2. いまの機体は 14Hz の振動が乗っている。振動している加速度計を
//       積分するのは相補フィルタが一番失敗するパターン。
//       まずログで est_vz と climb を並べて、素性を確認してから有効化する。
// ============================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include "quad/QuadConfig.h"

namespace Quad {

class AltEstimator {
public:
    // 起動時 / アーム時に呼ぶ。h0 は分かっていれば実測高度。
    void reset(float h0_m) {
        _h    = (h0_m > 0.0f) ? h0_m : 0.0f;
        _vz   = 0.0f;
        _bias = 0.0f;
        _acc_up = 0.0f;
        _have   = false;
    }

    // ------------------------------------------------------------
    //  predict()  — メインループで毎回 (RATE_LOOP_HZ)
    //    roll_deg / pitch_deg : 機体姿勢 [deg]
    //    ax / ay / az         : 機体座標 FRD の加速度 [g] (g_att.acc_*)
    //
    //  FRD なので静止・水平で az ≈ -1.0 g。
    //  地球座標の「下向き」比力は回転行列の3行目:
    //      f_down = -sin(p)*ax + cos(p)sin(r)*ay + cos(p)cos(r)*az
    //  重力ぶん (-1g) を引いて符号を返すと「上向き正の加速度」になる。
    // ------------------------------------------------------------
    void predict(float dt_s, float roll_deg, float pitch_deg,
                 float ax, float ay, float az) {
        if (dt_s <= 0.0f || dt_s > 0.1f) return;   // 異常な dt は捨てる

        const float r = roll_deg  * DEG2RAD;
        const float p = pitch_deg * DEG2RAD;
        const float sr = sinf(r), cr = cosf(r);
        const float sp = sinf(p), cp = cosf(p);

        const float f_down = -sp * ax + cp * sr * ay + cp * cr * az;

        //  静止時 f_down = -1g なので、+1 して重力を除去。
        //  ALT_ACC_Z_SIGN は IMU が上下逆マウントの場合に -1 にする。
        _acc_up = ALT_ACC_Z_SIGN * (-(f_down + 1.0f)) * GRAVITY_MPS2;

        //  推定を進める前に、まだ一度も測距で初期化していなければ何もしない
        //  (適当な h から積分を始めるとバイアス推定が暴れる)。
        if (!_have) return;

        const float a = _acc_up - _bias;
        _h  += _vz * dt_s + 0.5f * a * dt_s * dt_s;
        _vz += a * dt_s;
    }

    // ------------------------------------------------------------
    //  correct()  — 新しい測距サンプルが来たときだけ
    //    h_meas : 測距由来の鉛直高度 [m]  (cos補正済み。LPF は掛かっていてよい)
    //    dt_s   : 前回 correct() からの経過 [s]
    // ------------------------------------------------------------
    void correct(float h_meas, float dt_s) {
        if (!_have) {                 // 初回は測距値をそのまま採用
            _h = h_meas; _vz = 0.0f; _bias = 0.0f; _have = true;
            return;
        }
        if (dt_s <= 0.0f || dt_s > 0.5f) return;

        float err = h_meas - _h;

        //  大きく食い違ったら推定を捨てて測距に合わせ直す。
        //  (測距の失探明け・着地の衝撃など)
        if (fabsf(err) > ALT_EST_RESET_ERR_M) {
            _h = h_meas; _vz = 0.0f; _bias = 0.0f;
            return;
        }

        const float w  = ALT_EST_W;
        _h    += 3.0f * w       * err * dt_s;
        _vz   += 3.0f * w * w   * err * dt_s;
        _bias -=        w * w * w * err * dt_s;
        _bias = constrain(_bias, -ALT_EST_BIAS_LIM, ALT_EST_BIAS_LIM);
    }

    // --- 出力 -----------------------------------------------------
    bool  valid()     const { return _have; }
    float heightM()   const { return _h; }        // 推定高度 [m]
    float climbMps()  const { return _vz; }       // 推定上昇速度 [m/s] (上 +)
    float biasMps2()  const { return _bias; }     // 学習した加速度バイアス [m/s^2]
    float accUp()     const { return _acc_up; }   // 重力除去後の鉛直加速度 [m/s^2]

private:
    static constexpr float DEG2RAD     = 0.01745329252f;
    static constexpr float GRAVITY_MPS2 = 9.80665f;

    float _h      = 0.0f;
    float _vz     = 0.0f;
    float _bias   = 0.0f;
    float _acc_up = 0.0f;
    bool  _have   = false;
};

} // namespace Quad
