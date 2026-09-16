// ============================================================
//  S5Vehicle.h  -  drone_s5 の「機体まるごと」: デバイス + 制御器 + 共有状態
// ============================================================
//  drone_s5.cpp に 40 個以上あったグローバル (g_att / g_out / g_mode /
//  g_flow_* / g_range_* / …) と、IMU・SBUS・フロー・測距・PosHold・AltHold …
//  のインスタンスを 1 つの構造体に集めたもの。
//
//  なぜ 1 個の構造体か:
//    テレメトリ (S5Telemetry.h)・画面 (S5Status.h)・ログ 1 行 (S5LogFill.h)・
//    起動時チェック (SelfTest.h)・シリアルキー (S5Console.h) は「機体の今の
//    全状態」を読む。これらを別ヘッダに出すには、読む相手を 1 個の引数で
//    渡せる形が要る。extern を並べるより、依存が引数に見えるこの形にする。
//
//  制御の中身 (どう更新するか) は drone_s5.cpp。ここは置き場と初期値だけ。
// ============================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "Actuators.h"
#include "Receiver.h"
#include "sensor/IMU.h"
#include "sensor/OpticalFlow.h"
#include "sensor/Rangefinder.h"
#include "S5Telem.h"
#include "S5Cmd.h"

#include "quad/QuadConfig.h"
#include "quad/QuadPID.h"
#include "quad/Mixer.h"
#include "quad/BodyFrame.h"
#include "quad/PosHold.h"
#include "quad/AltHold.h"
#include "quad/AltEstimator.h"
#include "quad/BodyDv.h"
#include "quad/HeadingHold.h"
#include "quad/Guided.h"
#include "quad/Scheduler.h"
#include "quad/S5Features.h"

namespace S5 {

// ---- オプティカルフロー観測 (表示・ログ・キャリブ積算) --------------------
//  制御に使う値は Q::PositionHold の中にある。ここは「センサが今何を返したか」。
struct FlowObs {
    bool  ok = false;                          // PMW3901 が初期化できたか (起動時スナップショット)
    float raw_x = 0.0f, raw_y = 0.0f;          // 機体座標の生カウント [px]
    float dx    = 0.0f, dy    = 0.0f;          // de-rotation 後 [px]
    float vx    = 0.0f, vy    = 0.0f;          // 対地速度 [m/s] (前 +, 右 +)
    float vx_f  = 0.0f, vy_f  = 0.0f;          // 表示用 LPF (ログは生値)

    // 既知距離キャリブ用の積算 ([z] でゼロ)。acc_px と実測距離を突き合わせて
    // QuadConfig.h の FLOW_PX_PER_RAD を決める。
    double acc_raw_x = 0.0, acc_raw_y = 0.0;   // 生ピクセル (de-rotation 前)
    double acc_px_x  = 0.0, acc_px_y  = 0.0;   // de-rotation 後ピクセル
    double acc_m_x   = 0.0, acc_m_y   = 0.0;   // 推定変位 [m]

    void zeroAccum() {
        acc_raw_x = acc_raw_y = 0.0;
        acc_px_x  = acc_px_y  = 0.0;
        acc_m_x   = acc_m_y   = 0.0;
    }

    // flow.consumeFresh() が true の回に呼ぶ。窓の積算秒数 dt_s を渡す。
    void take(const OpticalFlow& f, float dt_s) {
        raw_x = f.raw_x;   raw_y = f.raw_y;
        dx    = f.derot_x; dy    = f.derot_y;
        vx    = f.vx;      vy    = f.vy;
        acc_raw_x += f.raw_x;        acc_raw_y += f.raw_y;
        acc_px_x  += f.derot_x;      acc_px_y  += f.derot_y;
        acc_m_x   += f.vx * dt_s;    acc_m_y   += f.vy * dt_s;
        constexpr float A = 0.2f;
        vx_f += A * (f.vx - vx_f);
        vy_f += A * (f.vy - vy_f);
    }
};

// ---- 測距観測 --------------------------------------------------------------
struct RangeObs {
    bool  ok    = false;     // センサが初期化できたか (起動時スナップショット)
    bool  valid = false;     // 高度が信用できるか (失探すると false)
    bool  fresh = false;     // 今回のポーリングで新しいサンプルが入ったか
    float raw_m = 0.0f;      // 傾き補正前の斜め距離 [m]
    float h_m   = 0.0f;      // 鉛直対地高度 [m]
    float climb_mps = 0.0f;  // 上昇速度 [m/s] (上 +)

    void take(const Rangefinder& r, bool fresh_now) {
        fresh     = fresh_now;
        valid     = r.valid();
        raw_m     = r.rawM();
        h_m       = r.heightM();
        climb_mps = r.climbMps();
    }
    // AltHold / AltEstimator の reset に渡す「今の高度 (無効なら 0)」
    float hOrZero() const { return valid ? h_m : 0.0f; }
};

// ---- 機体まるごと ----------------------------------------------------------
struct Vehicle {
    // --- デバイス ---
    IMU   mpu{&Wire};
    Sbus  sbus{&Serial5};
    motor motors[Quad::MOTOR_COUNT];
    // IM920SL は Serial3。送信 (下りテレメトリ) と受信 (上りコマンド) が同じポート。
    // ★ 送信は必ず非ブロッキング (s5tx.service() を毎ループ)。受信は s5rx だけが読む。
    S5T::Tx s5tx{&Serial3};
    S5C::Rx s5rx{&Serial3};
    OpticalFlow flow;          // PMW3901 (SPI, CS = QuadConfig FLOW_CS_PIN)
    Rangefinder rangefinder;   // ToF/SONAR (QuadConfig RANGE_BACKEND)

    // --- 制御器 ---
    Quad::Axis          roll_axis, pitch_axis, yaw_axis;   // 角度PID + レートPID
    Quad::HeadingHold   heading;    // ジャイロ積分ヨーとそのホールド
    Quad::PositionHold  poshold;    // フロー水平ホールド (速度/位置ループ)
    Quad::AltitudeHold  althold;    // 測距高度ホールド (engage 状態遷移込み)
    Quad::AltEstimator  altest;     // 加速度Z×測距 相補フィルタ (ALT_USE_ACC_FUSION=false の間はログ専用)
    Quad::Guided        guided;     // 地上局要求 → 目標速度/高度 の翻訳
    Quad::BodyDvAccumulator body_dv; // 地上局ヨー推定用の機体Δv (D フレームで送る)

    // --- 毎ループの共有状態 ---
    Quad::Attitude att;                          // IMU から読んだ姿勢・角速度・加速度
    float          out[Quad::MOTOR_COUNT] = {0}; // ミキサー出力 (ドライラン中も計算する)
    Quad::MixInfo  mix;                          // ミキサーの報告 (飽和・thr_used)

    Mode mode       = MODE_ANGLE;
    Mode prev_mode  = MODE_ANGLE;
    bool prev_armed = false;
    bool arm_latched = false;    // アームの瞬間に SW_HOVER が ANGLE 側だったか (armGate)

    // 角度ループ (200Hz) の分周と、実測 dt 用の前回時刻
    Quad::Divider angle_div{Quad::ANGLE_LOOP_DIV};
    uint32_t      angle_prev_us = 0;

    FlowObs  flowobs;
    RangeObs range;

    bool alt_hold_enable = USE_ALT_HOLD;   // シリアル 'g' でトグル
    bool dry_run         = DRY_RUN;        // シリアル 'm' でトグル
    bool sd_ok   = false;                  // SdLog::begin() が成功したか
    bool link_ok = false;                  // LogLink::begin() が成功したか

    // ミキサーが実際に使ったスロットルの積算。1000Hz のミキサーと 100Hz の
    // 高度ループをつなぐ。瞬時値だと姿勢振動 (14Hz 級) とエイリアシングする
    // ので、区間平均で「機体が実際に受け取った推力」を渡す。
    float    thr_used_sum = 0.0f;
    uint32_t thr_used_n   = 0;

    // シリアル 'p' メニューで変えるフロー水平ゲイン (QuadConfig の初期値をコピー)
    float flow_vel_kp = Quad::FLOW_VEL_KP;
    float flow_vel_ki = Quad::FLOW_VEL_KI;
    float flow_pos_kp = Quad::FLOW_POS_KP;

    // --- よく使う派生値 ---
    float thrStick() const { return USE_SBUS ? constrain(sbus.des[Ch::THR], 0.0f, 1.0f) : 0.0f; }
    bool  holdMode()  const { return mode == MODE_POSHOLD || mode == MODE_GUIDED; }
    bool  flowAlive() const {
        // 起動時 OK かつ、空中で凍結 (生カウント 0 が続く) していない。
        // 地上 (離陸検知前) は静止でカウント 0 が正常なので suspectDead を見ない。
        return USE_FLOW && flowobs.ok && !(flow.suspectDead() && althold.airborne());
    }
};

// ============================================================
//  アーム判定
// ============================================================
//  ★ POSHOLD の位置ではアームさせない (2026-09-04)。
//    操縦者が ANGLE のつもりで飛ばした 6 本すべてが実際には POSHOLD だった
//    (mode=3 が 94〜100%)。POSHOLD では角度目標がフロー位置ループから来るので
//    スティックで姿勢を直せず、離陸検知が立った瞬間に最大リーンが出て飛んで
//    いく。「アームの瞬間は必ず ANGLE 側」を強制する。いったんアームした後に
//    POSHOLD へ切り替えるのは自由 (bail-out も従来どおり)。
inline bool armGateOk(Vehicle& v) {
    if (!USE_SBUS) return true;
    return v.sbus.Ch_state(Ch::SW_HOVER) != up;   // up = POSHOLD/GUIDED 側
}

inline bool isArmed(Vehicle& v) {
    if (!USE_SBUS) return false;
    if (!v.sbus.isSafe()) return false;
    if (v.sbus.Ch_state(Ch::THR_CUT) != Quad::ARM_SWITCH_STATE) {
        v.arm_latched = false;
        return false;
    }
    // アーム操作の瞬間だけ SW_HOVER の位置を見る。
    if (!v.arm_latched) {
        if (!armGateOk(v)) {
            static uint32_t last_warn_ms = 0;
            if (millis() - last_warn_ms > 2000) {
                last_warn_ms = millis();
                Serial.println("\n!! ARM 拒否: SW_HOVER が POSHOLD 側です。");
                Serial.println("   ANGLE 側 (bail-out 位置) に戻してからアームしてください。");
                Serial.println("   ★スイッチの向きが思っているのと逆になっていないか確認を。");
                Serial.println("   地上局の画面の MODE= 表示が ANGLE であることを見てください。");
            }
            return false;
        }
        v.arm_latched = true;
    }
    return true;
}

// ============================================================
//  出力とリセット (制御ループ / シリアルメニュー / メンテナンス指令が共用)
// ============================================================
inline void writeMotors(Vehicle& v) {
    if (!USE_MOTOR) return;
    // ドライラン中は out[] をそのまま残して (ログ/表示用)、ESC へは 0 だけ送る。
    if (v.dry_run) {
        for (int i = 0; i < Quad::MOTOR_COUNT; ++i) v.motors[i].write(0.0f);
        return;
    }
    for (int i = 0; i < Quad::MOTOR_COUNT; ++i) v.motors[i].write(v.out[i]);
}

inline void stopAllMotors(Vehicle& v) {
    for (int i = 0; i < Quad::MOTOR_COUNT; ++i) v.out[i] = 0.0f;
    v.mix = Quad::MixInfo{};
    writeMotors(v);
}

// PID の内部状態と目標値の両方をクリアする (アーム / モード切替 / 'r')。
//  目標値 (tar) を残すと、前のモードの目標角が次のモードに生き残る。
inline void resetControllers(Vehicle& v) {
    v.roll_axis.reset();
    v.pitch_axis.reset();
    v.yaw_axis.reset();
    v.angle_div.reset();
    v.angle_prev_us = micros();

    // ヘディングホールドも「今の向き」を基準に取り直す。
    v.heading.reset();
    v.guided.resetYawCorrSeq();    // 次に届いたパケットで即座に再基準する

    // D フレームは POSHOLD/GUIDED でしか送らない。ここで捨てておかないと、
    // ANGLE で溜まったままの古いΔvが、次に POSHOLD へ入った瞬間の 1 発目に乗る。
    v.body_dv.reset();

    // フロー水平ホールドと高度ホールドも「今ここ」を基準に取り直す。
    v.poshold.reset();
    v.althold.reset(v.range.hOrZero());
    v.altest.reset(v.range.hOrZero());
}

} // namespace S5
