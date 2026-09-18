// ============================================================
//  drone_s5.cpp  -  Stage 5d : 1スイッチ完全自動ホバリング + 地上局ガイド飛行
// ============================================================
//  Stage 4 (drone_s4.cpp) の角度PID + モード切替 + ヘディングホールドを土台に、
//  オプティカルフロー水平ホールド (s5b) / 測距高度ホールド (s5c) / 1 スイッチ
//  完全自動 (s5d) / 地上局ガイド飛行 GUIDED (2026-09-11〜) を載せたもの。
//
//  ------------------------------------------------------------
//  【運用】 SW_HOVER (ch8, 3 段) の 1 本だけで決める。THR_CUT (ch7) は常に最上位。
//      down → ANGLE   完全手動 (スロットルも手動)。これが bail-out
//      cen  → POSHOLD 完全自動 (水平位置 + 高度 + ヘディング)。
//                     フロー喪失時は ALTHOLD へ自動フォールバック
//      up   → GUIDED  機体単独の周回 (2026-09-17〜。地上局ミッションはコメントアウト中)。
//                     飛行中に cen → up へ上げた瞬間、今いる点を円の左端・機首方向を
//                     接線として半径 1m の右旋回を 1 周し、POSHOLD に戻る (quad/Guided.h)。
//                     GUIDED_PATTERN=Figure8 なら直径 1m の右旋回 → 起点で左旋回の 8 の字。
//                     GUIDED_PATTERN=ClimbTurn なら半径 0.75m で低 2 周 → 上昇 1 周 → 高 2 周
//                     → その場で開始高度へ降下 (QuadConfig.h CLIMB_*)。
//                     資格が無い間 / 終わった後は cen と同じ POSHOLD
//    アームは必ず ANGLE 側で (S5Vehicle.h の armGateOk)。
//    ★ 初回は広い床で、指をモードスイッチに。符号ミス = 即壁/天井行き。
//    ★ 地上確認はドライラン ('m') で。フロー制御は「アーム && POSHOLD &&
//      スロットル > FLOW_ENABLE_THR && 離陸検知済」でしか出力しない。
//
//  ------------------------------------------------------------
//  【制御の構造】 カスケード。GUIDED も POSHOLD と同じ経路を通る (目標の出所が違うだけ)
//
//    地上局 (setpoint のみ) ─┐
//    スティック ─────────────┴→ PosHold  目標速度 → 速度PID → 目標リーン角   25Hz
//                               AltHold  目標高度 → 位置/速度PID → スロットル   100Hz (測距の回だけ PID)
//                               角度PID  目標リーン角 → 目標角速度              毎ループ
//                               レートPID 目標角速度 → トルク → ミキサー → ESC  毎ループ (目標 1000Hz)
//    ヨー: ジャイロ積分の相対方位を保持 (HeadingHold)。定型機動中は一定レート。
//
//  ★ 2026-09-18 (RP2040 移行後の見直し): メインループは 1000Hz を目標にしているが
//    RP2040 (FPU なし) では実効 630〜700Hz。すべての PID・積分・姿勢推定 (Madgwick) は
//    実測 dt で計算し、回数分周 (÷5 / ÷10) は使わない。周期は Ticker (時間基準) だけ。
//    姿勢推定は固定 1ms で積分していて角度が 0.6〜0.7 倍に縮んでいた (sensor/IMU.h)。
//
//  ------------------------------------------------------------
//  【ファイル構成】 このファイルは「配線」だけ。中身は include/quad/ の各ヘッダ。
//    S5Features.h   搭載デバイスと周期の一覧 (★ loop() の周期表はここ)
//    S5Gains.h      姿勢ループゲイン / トリム (経緯は TUNING_HISTORY.md)
//    S5Vehicle.h    機体まるごと (デバイス + 制御器 + 共有状態) と arm/reset/motor
//    Scheduler.h    Ticker
//    Guided.h       地上局要求 → 目標速度/高度 の翻訳 (状態機械)
//    HeadingHold.h  ヨーの相対方位とホールド
//    PosHold.h / AltHold.h / AltEstimator.h / Maneuver.h   制御器の中身
//    S5Telemetry.h  下りテレメトリの枠割り (A,B,A,B,C,A,B,D)
//    S5LogFill.h    125Hz ログ 1 行の作成
//    S5Console.h    シリアルキー / ゲインメニュー / BLE メンテナンス指令
//    S5Status.h     デバッグ画面 (RP2040 ではコア1。§ 9 と quad/UsbOwner.h)
//    FcWatchdog.h / LoopProfile.h / SafeLog.h   ループ停止対策 / 区間計測 / 待たない出力
//    SelfTest.h / StallLog.h   起動時デバイスチェック / 停止調査
//    protocol/S5Cmd.h, S5Telem.h   無線パケット (地上局・PC と共有)
// ============================================================

#include <Arduino.h>
#include <math.h>

#include "quad/S5Vehicle.h"
#include "quad/S5Gains.h"
#include "quad/S5Telemetry.h"
#include "quad/S5LogFill.h"
#include "quad/S5Console.h"
#include "quad/S5Status.h"
#include "quad/SelfTest.h"
#include "quad/StallLog.h"
#include "quad/FlightLog.h"
#include "quad/SdLog.h"
#include "quad/LogLink.h"
#include "quad/StatusLed.h"
#include "quad/FcWatchdog.h"
#include "quad/SafeLog.h"
#include "quad/LoopProfile.h"
#include "quad/UsbOwner.h"
#include "quad/S5Failsafe.h"

namespace Q = Quad;
using S5::Vehicle;
namespace WDT = S5::FcWatchdog;

static_assert(sizeof(S5C::CmdFrame) == LogLinkProto::CMD_PAYLOAD,
              "quad/LogLinkProto.h の CMD_PAYLOAD を protocol/S5Cmd.h の CmdFrame に合わせること");

// ============================================================
//  § 1  インスタンス
// ============================================================
static Vehicle          v;        // 機体まるごと (S5Vehicle.h)
static S5::TelemetryTx  telem;    // 下りテレメトリの枠割り

// 周期 (一覧は S5Features.h)。すべて時間基準の Ticker (回数分周はしない。Scheduler.h)。
static Q::Ticker  main_tick    (S5::MAIN_HZ);        // 1000Hz (目標。RP2040 は実効 630〜700Hz)
static Q::Ticker  flow_tick    (Q::FLOW_LOOP_HZ);    //  100Hz  フロー読み (制御は FLOW_CTRL_HZ)
static Q::Ticker  range_tick   (Q::RANGE_LOOP_HZ);   //  100Hz  測距ポーリング
static Q::Ticker  telem_rx_tick(S5::TELEM_RX_HZ);    //  200Hz  上りコマンド受信
static Q::Ticker  telem_tick   (S5::USE_BLE_LINK ? S5::BLE_TELEM_HZ   //   20Hz  下りテレメトリ (BLE)
                                                  : S5::TELEM_TX_HZ);   //    8Hz  下りテレメトリ (IM920)
static Q::Ticker  debug_tick   (S5::DEBUG_HZ);       //   10Hz  画面
static Q::Ticker  guided_tick  (S5::GUIDED_HZ);      //  100Hz  地上局要求の翻訳 / 周回の目標
static Q::Ticker  log_tick     (FlightLog::LOG_HZ);  //  125Hz  ログ 1 行 (RP2040。Teensy 500Hz)

// ============================================================
//  § 2  ホールド用スロットルゲート
// ============================================================
//  ホールドの enable 判定に渡すスロットル。閾値を割る方向だけ
//  HOLD_THR_DROP_DEBOUNCE_MS 続くまで直前の値で保持する (QuadConfig 参照)。
//  上げる方向と、閾値より上での変化は素通し。ミキサーへ渡す thr_stick には
//  使わない (ホールドが release された後のスロットルは生値のまま)。
struct HoldThrottleGate {
    float    gate      = 0.0f;
    uint32_t low_since = 0;
    bool     glitch    = false;   // 1フレームで飛んだ下がり方だったか

    float update(const Vehicle& veh) {
        const float raw = veh.thrStick();
        const float th  = fminf(Q::FLOW_ENABLE_THR, Q::ALT_ENABLE_THR);
        const uint32_t now = millis();
        if (raw > th || Q::HOLD_THR_DROP_DEBOUNCE_MS == 0) {
            low_since = 0;
            glitch    = false;
            gate      = raw;
            return raw;
        }
        if (low_since == 0) {
            low_since = now;
            // 直前の値からいきなり HOLD_THR_GLITCH_STEP 以上落ちた = 人の手では出ない
            // 遷移 (LOG0007/0017/0018 で 3 回とも 1 フレームで 0.064 へ)。受信機の
            // プリセット値/再起動の疑いが強いので長めに保持する。
            glitch = (gate - raw) > Q::HOLD_THR_GLITCH_STEP;
        }
        // 他のスティックが動いた = パイロットが操作している → 通常の短い猶予に戻す
        const bool pilot_active = S5::USE_SBUS &&
            (fabsf(veh.sbus.des[Ch::ROLL]) > 0.10f || fabsf(veh.sbus.des[Ch::PITCH]) > 0.10f ||
             fabsf(veh.sbus.des[Ch::YAW])  > 0.10f);
        const uint32_t hold_ms = (glitch && !pilot_active)
                               ? Q::HOLD_THR_GLITCH_HOLD_MS : Q::HOLD_THR_DROP_DEBOUNCE_MS;
        if (now - low_since >= hold_ms) {
            gate = raw;
            return raw;
        }
        return gate;
    }
};
static HoldThrottleGate hold_gate;

// ============================================================
//  § 3  モード判定 — SW_HOVER の 1 本だけで決める
// ============================================================
//    SW_HOVER=down → ANGLE   (完全手動。bail-out)
//    SW_HOVER=cen  → POSHOLD (完全自動)。フローが死んでいれば ALTHOLD
//    SW_HOVER=up   → GUIDED  (Guided が engage しているとき)。資格が無ければ cen と同じ
//  ・SW_AUTO / RATE(アクロ) は封印。THR_CUT は selectMode より上位で常にモーターを止める。
static S5::Mode selectMode() {
    if (!S5::USE_SBUS) return S5::MODE_ANGLE;

    // GUIDED の資格判定は全部 Guided::update() 側。ここは結果を読むだけ。
    if (v.guided.engaged()) return S5::MODE_GUIDED;

    const Sw sw = v.sbus.Ch_state(Ch::SW_HOVER);
    if (sw == up || sw == cen) {
        // ★ flowobs.ok は起動時の 1 回きりのスナップショット。飛行中に PMW3901 が
        //   固まると (0,0) を返し続けて「静止している」と誤認するので、空中でだけ
        //   suspectDead() (生カウント 0 が FLOW_DEAD_S 続く) も見て ALTHOLD へ落とす。
        //   地上 (離陸検知前) は静止でカウント 0 が正常なので見ない (2026-09-15:
        //   見てしまうと地上からの GUIDED 自動離陸が ALTHOLD 止まりで始まらない)。
        if (v.flowAlive()) return S5::MODE_POSHOLD;
        return S5::MODE_ALTHOLD;   // フロー未初期化 or 実行時に凍結
    }
    return S5::MODE_ANGLE;
}

// ============================================================
//  § 4  センサ読み → 各ホールド制御器  (loop() から周期ごとに呼ぶ)
// ============================================================

// --- 毎ループ: IMU → 姿勢、高度推定の predict ---------------------------
static void readImu(float dt_s) {
    if (!S5::USE_MPU) return;
    v.mpu.update(dt_s);   // 実測 dt で Madgwick を積分する (固定 1ms だと RP2040 で角度が縮む)
    // I2C バスを復旧したら、同じバスの測距センサも入れ直す (sensor/IMU.h recoverBus)
    if (v.mpu.consumeBusRecovered()) {
        if (S5::USE_RANGE) v.rangefinder.busRecovered();
        Q::SafeLog::logf("\n!! I2C バス復旧 %u 回目: IMU %s (読み失敗 %lu 回)\n",
                         (unsigned)v.mpu.recoverCount(),
                         v.mpu.recoverOk() ? "復帰OK (リセット+設定し直し)" : "まだ無応答",
                         (unsigned long)v.mpu.ioFailTotal());
    }
    v.att = Q::readAttitude(v.mpu);
    // フローの de-rotation 用に機体の回転角を積む (フロー読みの回に取り出す)
    v.flow_rot.add(v.att, dt_s);
    // 加速度は毎ループ・遅れほぼゼロなので、測距 (遅れ大) より先に積分を進める。
    // ALT_USE_ACC_FUSION=false の間は結果をログに出すだけ。
    v.altest.predict(dt_s, v.att);
}

// --- 25Hz (FLOW_CTRL_HZ): フロー水平ホールド --------------------------------
//  中身は quad/PosHold.h。ここは「効かせる条件」と「目標の出所」を組み立てて渡す薄い層。
//  出力は poshold.leanRoll()/leanPitch() [deg] で、updateControl() が角度ループの目標にする。
static void updateFlowHold(float dt_s) {
    const float thr = hold_gate.update(v);

    // 離陸するまでは効かせない (FLOW_REQUIRE_AIRBORNE)。地上ではフロー速度が常に 0 で
    // 位置積分がノイズを溜め、速度I項が巻き上がる (地上ドライランで -3〜-5deg まで育つ)。
    // 測距が無い構成では離陸を検知できないのでゲートを無効にする (永久に立ち上がらない)。
    const bool can_detect_takeoff = S5::USE_RANGE && v.range.ok;
    const bool airborne = !Q::FLOW_REQUIRE_AIRBORNE || !can_detect_takeoff || v.althold.airborne();

    const bool active = S5::isArmed(v) && v.holdMode()
                     && (thr > Q::FLOW_ENABLE_THR) && airborne
                     && S5::USE_FLOW && v.flowobs.ok;

    // GUIDED 中は Guided の出力を PosHold に渡す。スティックは渡さない
    // (Guided がスティック操作を検出したら GUIDED 自体を降りるので混ぜる必要がない)。
    //   地上局ミッション  … 機体座標の目標速度 (setVelCommand)。定型機動もこの経路
    //   機体単独パターン  … 円の目標点・接線速度・向心加速度 (setTrajectory)
    float sx = 0.0f, sy = 0.0f;
    if (v.mode == S5::MODE_GUIDED && S5::GUIDED_MISSION) {
        v.poshold.clearTrajectory();
        v.poshold.setVelCommand(v.guided.vx(), v.guided.vy());
    } else if (v.mode == S5::MODE_GUIDED && v.guided.tracking()) {
        const Q::Guided::Trajectory tr = v.guided.trajectory();   // 周回 / 直進
        v.poshold.clearVelCommand();
        v.poshold.setTrajectory(tr.ref_n, tr.ref_e, tr.vel_n, tr.vel_e, tr.acc_n, tr.acc_e);
    } else {
        v.poshold.clearTrajectory();
        v.poshold.clearVelCommand();
        sx = Q::STICK_SIGN_PITCH * v.sbus.des[Ch::PITCH];   // 機体座標のまま (機首基準)
        sy = Q::STICK_SIGN_ROLL  * v.sbus.des[Ch::ROLL];
    }

    // 位置積分を地面固定フレームで行うため、ヘディング (アーム時 0 の相対方位) を渡す。
    v.poshold.update(dt_s, v.flow.vx, v.flow.vy, v.heading.est(), sx, sy, active);
}

// --- 100Hz (RANGE_LOOP_HZ): 高度ホールド ----------------------------------
//  中身は quad/AltHold.h。POSHOLD 中、プロポのスロットルは enable ゲートにしか使わない。
static void updateAltHold(float dt_s) {
    const float thr = hold_gate.update(v);

    // 前回の呼び出し以降にミキサーが実際に使ったスロットルの平均 (-1 = 情報なし)
    const float thr_applied = (v.thr_used_n > 0) ? (v.thr_used_sum / (float)v.thr_used_n) : -1.0f;
    v.thr_used_sum = 0.0f;
    v.thr_used_n   = 0;

    // 上昇速度の出所: 加速度融合 (遅れ小) か測距の微分か
    const bool  use_est = Q::ALT_USE_ACC_FUSION && v.altest.valid();
    const float climb   = use_est ? v.altest.climbMps() : v.range.climb_mps;

    // GUIDED 中は目標高度も地上局から。自動離陸・着陸はこの目標をスルーレート付きで
    // 動かしているだけで、専用の制御経路は無い。
    const bool guided_alt = (v.mode == S5::MODE_GUIDED && v.guided.altM() > 0.0f);
    if (v.fs == S5::FS_LAND)   // SBUS 途絶の自動着陸 (S5Failsafe.h)
        v.althold.commandTarget(Q::GUIDED_LAND_FLOOR_M, Q::GUIDED_LAND_SLEW_MPS);
    else if (guided_alt) v.althold.commandTarget(v.guided.altM(), v.guided.slew());
    else                 v.althold.clearCommandedTarget();

    v.althold.update(dt_s,
                     v.alt_hold_enable && S5::USE_RANGE,
                     S5::isArmed(v),
                     v.holdMode() || v.mode == S5::MODE_ALTHOLD,
                     v.range.valid, v.range.fresh, v.range.h_m, climb,
                     thr, thr_applied,
                     // 地上からの自動離陸: GUIDED で目標高度が来ていて、測距が
                     // 「近すぎて無効」の間だけ (AltHold::update の ground_start)
                     guided_alt && v.rangefinder.tooClose());
}

// --- 100Hz (GUIDED_HZ): 周回の目標を進める ------------------------------------
static void updateGuided(float dt_s) {
    Q::Guided::Inputs in;
    in.now_ms      = millis();
    in.dt_s        = dt_s;
    in.armed       = S5::isArmed(v);
    in.sbus_ok     = S5::USE_SBUS && v.fs == S5::FS_NONE;   // フェイルセーフ中は GUIDED を降りる
    in.sw_hover_up = S5::USE_SBUS && (v.sbus.Ch_state(Ch::SW_HOVER) == up);
    in.flow_alive  = v.flowAlive();
    in.range_ok    = S5::USE_RANGE && v.range.ok;
    in.range_valid = v.range.valid;
    in.range_h_m   = v.range.h_m;
    in.airborne    = v.althold.airborne();
    in.hold_ready  = v.holdMode() && v.althold.active() && (v.thrStick() > Q::FLOW_ENABLE_THR);
    in.pos_n       = v.poshold.posN();
    in.pos_e       = v.poshold.posE();
    in.stick_roll  = v.sbus.des[Ch::ROLL];
    in.stick_pitch = v.sbus.des[Ch::PITCH];
    in.stick_yaw   = v.sbus.des[Ch::YAW];
    in.yaw_est_deg = v.heading.est();
    in.yaw_since_arm_deg = v.heading.sinceArm();
    if (S5::GUIDED_MISSION) v.guided.update(in, v.s5rx, v.poshold, v.heading);
    else                    v.guided.update(in);
}

// ============================================================
//  § 5  姿勢制御  (毎ループ。目標 1000Hz)
// ============================================================
static void updateControl(float dt_s) {
    v.mode = selectMode();
    const bool armed = S5::isArmed(v);

    // --- アーム状態やモードが変わったらクリア ---
    if (armed != v.prev_armed) {
        S5::resetControllers(v);
        if (armed) v.heading.markArm();   // 置いた向き = 直進の基準 (HeadingHold::sinceArm)
        // 姿勢推定の加速度補正: 地上は強く (水平へ素早く収束)、飛行中は弱く
        // (横移動の加速度に引かれない。sensor/IMU.h setFusionBeta のコメント)
        v.mpu.setFusionBeta(armed ? Q::IMU_FUSION_BETA_FLIGHT : Q::IMU_FUSION_BETA_GROUND);
        v.prev_armed = armed;
        Q::SafeLog::logf("%s", armed ? "\n>>> ARMED\n" : "\n>>> DISARMED\n");
        // ループ停止のウォッチドッグはアーム中だけ (quad/FcWatchdog.h)
        if (armed) WDT::arm();
        else       WDT::disarm();
        S5::LoopProfile::armEdge(armed);   // ディスアームで「この飛行」の区間計測を出す
        if (armed) {
            v.sbus_gap_max_ms = 0;
        } else {
            v.sbus_gap_last_ms = v.sbus_gap_max_ms;
            Q::SafeLog::logf("[SBUS この飛行] フレーム間隔の最大 %lu ms (途絶判定 %lu ms)\n",
                             (unsigned long)v.sbus_gap_last_ms, (unsigned long)Q::SBUS_LOST_MS);
        }
        // アームの瞬間にスティック中央を取り直す (ディスアーム中に trackCenter() で
        // 溜めた直近フレーム。棄却規則は起動時と同じ)。
        if (armed && S5::USE_SBUS && Q::STICK_CENTER_ENABLE) {
            const Sbus::CenterCal cc = v.sbus.applyTrackedCenter(
                Q::STICK_CENTER_MAX_OFS, Q::STICK_CENTER_MAX_MOVE);
            if (cc.st == Sbus::CC_OK)
                Q::SafeLog::logf("    スティック中央 再取込: R%+.3f P%+.3f Y%+.3f (%d frames)\n",
                              cc.roll, cc.pitch, cc.yaw, cc.n);
            else
                Q::SafeLog::logf("    !! スティック中央 再取込 棄却 (%s) — 前の値のまま !!\n",
                              cc.st == Sbus::CC_MOVING ? "動いていた" :
                              cc.st == Sbus::CC_TOOFAR ? "ずれ過大" : "フレーム不足");
        }
    }
    if (!armed && S5::USE_SBUS) v.sbus.trackCenter();
    if (v.mode != v.prev_mode) {
        // POSHOLD ⇔ GUIDED は同じ制御経路で目標の出所が変わるだけなので、何もリセット
        // しない。リセットすると飛行中に高度ホールドが engage し直し (I 項・離陸検知が
        // 消える)、フロー位置の原点も飛ぶので、周回の起点が取れなくなる。
        const bool hold_to_hold = v.holdMode() &&
            (v.prev_mode == S5::MODE_POSHOLD || v.prev_mode == S5::MODE_GUIDED);
        if (!hold_to_hold) S5::resetControllers(v);
        v.prev_mode = v.mode;
        Q::SafeLog::logf("\n>>> MODE = %s\n", S5::modeLabel(v.mode));
    }

    // --- 測定値はアーム前から入れておく (飛行前にジャイロの符号を画面で確認できる) ---
    v.roll_axis.rate_meas  = v.att.roll_rate;
    v.pitch_axis.rate_meas = v.att.pitch_rate;
    v.yaw_axis.rate_meas   = v.att.yaw_rate;
    //  姿勢基準トリムはここで 1 回だけ引く (S5Gains.h)。ang_meas はこの 1 箇所でしか
    //  作られないので、角度ループ・ログ・テレメトリすべてが補正後の値を見る。
    //  v.att 自体は触らない (AltHold の cos 補正など「生の姿勢」が要るところを壊さない)。
    v.roll_axis.ang_meas   = v.att.roll  - Gain::ROLL_TRIM_DEG;
    v.pitch_axis.ang_meas  = v.att.pitch - Gain::PITCH_TRIM_DEG;

    // 地上局ヨー推定用の機体Δv (D フレームで送る)。アーム前から積んでよい。
    v.body_dv.update(dt_s, v.att);

    if (!armed) { S5::stopAllMotors(v); return; }

    // 自動着陸が完了したら、ディスアームされるまで出力を切ったままにする。
    // 接地後も高度ループが「まだ 5cm 届いていない」と押し続けると機体が地面を蹴って転がる。
    if (v.guided.landed()) { S5::stopAllMotors(v); return; }

    // --- スロットル ---
    //  高度ホールドが active なら althold.throttle()。POSHOLD/GUIDED で engage 前は
    //  POSHOLD_THR_CAP で頭打ち (S5Features.h)。それ以外はプロポの値をそのまま。
    const float thr_stick = v.thrStick();
    float thr;
    if (v.althold.active())  thr = v.althold.throttle();
    else if (v.holdMode())   thr = constrain(thr_stick, 0.0f, S5::POSHOLD_THR_CAP);
    else                     thr = thr_stick;
    const bool integrate = (thr > S5::I_ENABLE_THR);

    // --- スティック入力 ---
    //  ★ スロットルは全モード共通で常に物理プロポから取る (上の thr)。安全上の不変条件。
    //  地上局からの指令は GUIDED で「目標速度・目標高度」としてしか受けない
    //  (姿勢指令を無線で受ける経路は存在しない。protocol/S5Cmd.h 冒頭)。
    v.roll_axis.stick  = Q::STICK_SIGN_ROLL  * v.sbus.des[Ch::ROLL];
    v.pitch_axis.stick = Q::STICK_SIGN_PITCH * v.sbus.des[Ch::PITCH];
    v.yaw_axis.stick   = Q::STICK_SIGN_YAW   * v.sbus.des[Ch::YAW];

    // --- 外側ループ: 角度 → 目標角速度 (毎ループ。実測 dt) ---
    if (v.mode != S5::MODE_RATE) {
        if (v.holdMode()) {
            // 目標角は updateFlowHold() が計算済み (クランプ済み)
            v.roll_axis.ang_tar  = v.poshold.leanRoll();
            v.pitch_axis.ang_tar = v.poshold.leanPitch();
        } else {
            // エクスポでセンター付近の細かい操作をしやすくする (最大角度は変えない)
            v.roll_axis.ang_tar  = Q::stickExpo(v.roll_axis.stick,  Q::STICK_EXPO_ANGLE) * Q::MAX_ANGLE_ROLL;
            v.pitch_axis.ang_tar = Q::stickExpo(v.pitch_axis.stick, Q::STICK_EXPO_ANGLE) * Q::MAX_ANGLE_PITCH;
        }

        // 2026-09-18: ÷5 の分周をやめて毎ループ回す (S5Gains.h ANG_*)。P 主体なので
        // 値は同じで、目標角速度の 5ms 階段が消えるだけ。dt はレートループと同じ実測値。
        v.roll_axis.rate_tar = constrain(
            v.roll_axis.angle.update(v.roll_axis.ang_tar, v.roll_axis.ang_meas, dt_s, integrate),
            -S5::ANGLE_OUT_LIMIT, S5::ANGLE_OUT_LIMIT);
        v.pitch_axis.rate_tar = constrain(
            v.pitch_axis.angle.update(v.pitch_axis.ang_tar, v.pitch_axis.ang_meas, dt_s, integrate),
            -S5::ANGLE_OUT_LIMIT, S5::ANGLE_OUT_LIMIT);
    } else {
        // RATE モード (封印): スティックが直接、目標角速度になる
        v.roll_axis.ang_tar  = 0.0f;
        v.pitch_axis.ang_tar = 0.0f;
        v.roll_axis.rate_tar  = v.roll_axis.stick  * Q::MAX_RATE_ROLL;
        v.pitch_axis.rate_tar = v.pitch_axis.stick * Q::MAX_RATE_PITCH;
    }

    // --- ヨー: ヘディングホールド (HeadingHold.h) ---
    v.heading.integrate(v.yaw_axis.rate_meas, dt_s);
    //  周回中は接線方向へ向けるレート (CircleTrack::yawRateCmd)、それ以外は NAN
    const float maneuver_rate = v.guided.yawRate();
    v.yaw_axis.rate_tar = v.heading.update(v.yaw_axis.stick, integrate, maneuver_rate, Q::MAX_RATE_YAW);
    // 表示用 (ヨーには角度PIDを通していないが、保持誤差をここに入れておく)
    v.yaw_axis.ang_tar  = v.heading.hold();
    v.yaw_axis.ang_meas = v.heading.est();

    // --- 内側ループ: 角速度 → トルク指令 (毎ループ) ---
    v.roll_axis.cmd  = v.roll_axis.rate .update(v.roll_axis.rate_tar,  v.roll_axis.rate_meas,  dt_s, integrate);
    v.pitch_axis.cmd = v.pitch_axis.rate.update(v.pitch_axis.rate_tar, v.pitch_axis.rate_meas, dt_s, integrate);
    v.yaw_axis.cmd   = v.yaw_axis.rate  .update(v.yaw_axis.rate_tar,   v.yaw_axis.rate_meas,   dt_s, integrate);

    // --- ミキサー → ESC ---
    Q::mix(thr, v.roll_axis.cmd, v.pitch_axis.cmd, v.yaw_axis.cmd, v.out, &v.mix);
    v.thr_used_sum += v.mix.thr_used;    // 高度ホールドへ返すぶんを積算
    v.thr_used_n   += 1;
    S5::writeMotors(v);
}

// ============================================================
//  § 6  ログ / LED / ファイル開閉  (loop() 後半のサービス群)
// ============================================================

// LOG_HZ: ログ 1 行を作って USB / RAM / SD / LogLink の 4 シンクへ配る
static void serviceFlightLog(bool armed_now, float thr_now) {
    // RAM トリガの追加ゲート。フロー試験時は SW_HOVER=up からの 8 秒を録る
    // (地上待機や上昇でバッファを食い潰さない)。SD 運用では thr>0.20 だけ。
    const bool ram_gate = !S5::USE_FLOW || v.sbus.Ch_state(Ch::SW_HOVER) == up;
    FlightLog::Ram::tick(armed_now, thr_now, ram_gate);
    // どのシンクも動いていなければ量子化そのものを省く
    if (FlightLog::Usb::active || FlightLog::Ram::recording ||
        SdLog::recording() || LogLink::recording()) {
        FlightLog::Rec r;
        S5::fillRec(r, v, main_tick.dt_us, armed_now, thr_now);
        FlightLog::Usb::sample(r);      // 'l' 中のみ。USB が詰まっていたら捨てる
        FlightLog::Ram::push(r);        // 記録中のみ
        SdLog::push(&r, sizeof(r));     // アーム中のみ (中で recording を見る)
        LogLink::push(&r, sizeof(r));   // 同上。RP2040 ロガーへ
    }
}

// 再起動の報告ログを閉じる時刻 (0 = 報告中でない)。setup() で開始する
static uint32_t boot_report_until_ms = 0;

// アーム/ディスアームのエッジでログファイルを開閉する
static void serviceLogFiles(bool armed_now) {
    if (v.sd_ok) {
        static bool was_armed = false;
        if (armed_now && !was_armed) {          // アーム: 新規ファイル
            SdLog::startFile();                 // open + preAllocate で数十ms ブロック
            main_tick.prime();                  // ↑で基準がずれるので取り直す (dt 暴れ防止)
        }
        if (!armed_now && was_armed) SdLog::stopFile();   // CSV 化は PC (bin2csv.py)
        was_armed = armed_now;
    }
    if (v.link_ok) {
        // SD 版と違い open/preAllocate をこちら側でやらないのでブロックしない
        static bool was_armed = false;
        if (armed_now && !was_armed) { boot_report_until_ms = 0; LogLink::startFile(); }
        if (!armed_now && was_armed) LogLink::stopFile();
        was_armed = armed_now;
        // 再起動時の報告ログ (setup で開始) をアームせずに閉じる
        if (boot_report_until_ms != 0 && !armed_now && millis() >= boot_report_until_ms) {
            boot_report_until_ms = 0;
            LogLink::stopFile();
        }
    }
}

// モード表示 LED (pin 5/6/9) と機体検出用 LED (pin 21/22/23)
static void serviceLeds(bool armed_now) {
    const uint32_t now = millis();
    // ============================================================
    //  ★ ルールブック 2.2.8 (機体審査の項目)
    //    ・ハンズオフ飛行中      : 青または緑で **2Hz 程度の点滅**
    //    ・それ以外              : **赤点灯**
    //    ・操縦者の介入があったら : 赤点灯へ遷移
    //    ・審判から視認できない場合は赤とみなされる
    //
    //  したがって「ハンズオフ = GUIDED のときだけ緑 2Hz 点滅、それ以外は全部
    //  赤点灯」にする。POSHOLD/ALTHOLD は **操縦者がスイッチかスティックで
    //  介入した結果** 入るモードなので、自動系ではあるが赤が正しい。
    //  2Hz = 周期 500ms = 半周期 250ms。
    //
    //  ★ 2026-09-19 まではここが 黄(DISARM) / 緑4Hz(GUIDED) / 青2Hz(POSHOLD)
    //    だった。デバッグには便利だがルールには合っていない。練習でモードを
    //    色で見分けたいときだけ、下の COMP_LED_POLICY を false にする。
    // ============================================================
    constexpr bool COMP_LED_POLICY = true;   // true = 本番 (ルール 2.2.8)

    if (COMP_LED_POLICY) {
        if (armed_now && v.mode == S5::MODE_GUIDED) {
            if (Q::blinkOn(now, 250)) StatusLed::green(); else StatusLed::off();
        } else {
            StatusLed::red();
        }
    }
    //  練習用: 黄(点灯)=DISARM 赤(点灯)=ANGLE 青(2Hz)=POSHOLD/ALTHOLD 緑(4Hz)=GUIDED
    else if (!armed_now)                                             StatusLed::yellow();
    else if (v.mode == S5::MODE_GUIDED)  { if (Q::blinkOn(now, 125)) StatusLed::green(); else StatusLed::off(); }
    else if (v.mode == S5::MODE_POSHOLD ||
             v.mode == S5::MODE_ALTHOLD) { if (Q::blinkOn(now, 250)) StatusLed::blue();  else StatusLed::off(); }
    else if (v.mode == S5::MODE_ANGLE)                               StatusLed::red();
    else                                                             StatusLed::green();

    //  白色 6Hz 点滅 (カメラの点滅ロックイン検出用)。ディスアーム中は消灯。
    BlinkLed::white(armed_now && (now % 167u) < 83u);
}

// ============================================================
//  § 7  setup
// ============================================================
static void statusLedRainbowStep() {
    // StatusLed は 8 色しか出せないので、呼ばれるたびに次の色へ = 疑似レインボー
    static void (*const colors[])() = { StatusLed::red,  StatusLed::yellow, StatusLed::green,
                                         StatusLed::cyan, StatusLed::blue,   StatusLed::magenta };
    static uint8_t idx = 0;
    colors[idx]();
    idx = (idx + 1) % (sizeof(colors) / sizeof(colors[0]));
}

void setup() {
    StatusLed::begin();   // 他の初期化より先に。起動直後から状態が見えるように
    BlinkLed::begin();

    Serial.begin(115200);
    const uint32_t start_ms = millis();
    while (!Serial && (millis() - start_ms < 2000)) { }

    WDT::begin();   // 前回が「ループ停止によるリセット」なら、どの区間で止まったかを読む
    if (WDT::rebooted()) v.wdt_rebooted = true;

    Serial.println("\n\n=== Stage 5d : 1スイッチ完全自動ホバリング ===");
    if (v.wdt_rebooted)
        Serial.printf("\n!!!!! 前回はループ停止でウォッチドッグがリセットしました: 区間「%s」 "
                      "(起動 %lu ms 後) !!!!!\n!!!!! THR_CUT を一度切るまでアームしません !!!!!\n",
                      WDT::sectionName(WDT::lastSection()), (unsigned long)WDT::lastMs());
    Serial.println("!! SW_HOVER: down=ANGLE(手動) / cen=POSHOLD(完全自動) / up=GUIDED(地上局) !!");
    Serial.println("!! bail-out = SW_HOVER を下げる or THR_CUT。初回は広い床で指をスイッチに !!");

#ifndef ARDUINO_ARCH_RP2040
    // ★ 共有 SPI0: 全 CS を「最初に」HIGH へ固定する。flow.begin() 中に SD の CS(9)
    //   がフロートで Low に落ちると SD も選択されて両方のバスが壊れる。
    //   (RP2040 では FC に SPI デバイスが無い。PMW3901 は C3 側。quad/BoardPins.h)
    pinMode(10, OUTPUT); digitalWrite(10, HIGH);   // PMW3901 CS (Quad::FLOW_CS_PIN)
    if (S5::USE_SD) { pinMode(9, OUTPUT); digitalWrite(9, HIGH); }   // USE_LOGLINK 時は 9 = StatusLed B
#endif

    // --- 姿勢ループのゲイン (S5Gains.h) ---
    v.roll_axis.rate .set_gains(Gain::RATE_ROLL [0], Gain::RATE_ROLL [1], Gain::RATE_ROLL [2]);
    v.pitch_axis.rate.set_gains(Gain::RATE_PITCH[0], Gain::RATE_PITCH[1], Gain::RATE_PITCH[2]);
    v.yaw_axis.rate  .set_gains(Gain::RATE_YAW  [0], Gain::RATE_YAW  [1], Gain::RATE_YAW  [2]);
    v.roll_axis.angle .set_gains(Gain::ANG_ROLL [0], Gain::ANG_ROLL [1], Gain::ANG_ROLL [2]);
    v.pitch_axis.angle.set_gains(Gain::ANG_PITCH[0], Gain::ANG_PITCH[1], Gain::ANG_PITCH[2]);
    for (Q::Axis* ax : { &v.roll_axis, &v.pitch_axis, &v.yaw_axis }) {
        ax->rate.set_d_tau(Gain::RATE_D_TAU_S);
        ax->rate.set_i_limit(Gain::RATE_I_LIMIT);
        ax->angle.set_d_tau(Gain::ANG_D_TAU_S);
        ax->angle.set_i_limit(Gain::ANG_I_LIMIT);
    }

    // --- ホールド制御器 ---
    v.poshold.begin();
    v.poshold.setVelGains(v.flow_vel_kp, v.flow_vel_ki, Q::FLOW_VEL_KD);
    v.poshold.setPosKp(v.flow_pos_kp);
    v.althold.begin();

    // --- デバイス ---
    if (S5::USE_MOTOR) {
        Serial.println("Init motors...");
        for (int i = 0; i < Q::MOTOR_COUNT; ++i) v.motors[i].set_pin(Q::MOTOR_PIN[i]).begin();
        delay(500);
        S5::stopAllMotors(v);
    }
    if (v.dry_run) Serial.println("!! DRY-RUN 有効: モーターは回りません ('m' で解除) !!");

    if (S5::USE_SBUS)  {
        Serial.println("Init SBUS (" BOARD_SBUS_DESC ")...");
#ifdef ARDUINO_ARCH_RP2040
        // bolderflight SBUS は RP2040 では反転しない。begin() より前に必ず (trainer.cpp と同じ)。
        BOARD_SBUS_SERIAL.setInvertRX(true);
#endif
        v.sbus.begin();
    }
    if (S5::USE_MPU)   { Serial.println("Init IMU...");   v.mpu.begin();
                         v.mpu.setFusionBeta(Q::IMU_FUSION_BETA_GROUND); }   // 飛行中は arm で切替
    if (S5::USE_IM920) { Serial.println("Init IM920..."); v.s5tx.begin(); }   // Serial3 19200 (受信 s5rx も同じポート)
    if (S5::USE_FLOW && !S5::FLOW_VIA_LINK) {
        Serial.println("Init OpticalFlow (PMW3901)...");
        v.flowobs.ok = v.flow.begin();
        Serial.println(v.flowobs.ok ? "  PMW3901 OK"
                                    : "  !! PMW3901 応答なし (CSピン/SPI配線/電源を確認) !!");
    }
    // FLOW_VIA_LINK のときは LogLink::begin の後で評価する (下)。
    if (S5::USE_RANGE) {
        Serial.printf("Init Rangefinder (%s %s)...\n", Q::RANGE_INFO.name, Q::RANGE_INFO.device);
        v.range.ok = v.rangefinder.begin();
        if (!Q::RANGE_INFO.on_i2c) {
            // ソナーは初期化応答が無いので begin() の戻り値では判定できない
            Serial.printf("  %s PW=pin %d / 3V3給電。数百ms後に [距離] に値が出れば配線OK\n",
                          Q::RANGE_INFO.device, Q::RANGE_SONAR_PW_PIN);
        } else if (v.range.ok) {
            Serial.printf("  %s OK (%s)\n", Q::RANGE_INFO.device, Q::RANGE_INFO.bus);
        } else {
            Serial.printf("  !! %s 応答なし (%s) !!\n", Q::RANGE_INFO.device, Q::RANGE_INFO.hint);
        }
        Serial.printf("  高度ホールド: %s (POSHOLD で自動)。ホバースロットル=%s  目標高度=%s\n",
                      v.alt_hold_enable ? "有効" : "無効(POSHOLDでも手動)",
                      (Q::ALT_HOVER_THR > 0.01f) ? "実測値" : "未設定(engageしない)",
                      (Q::ALT_TARGET_M  > 0.0f)  ? "固定"   : "突入時の高度");
    }

    // --- ログの出口: SD (SPI0 共有。USE_FLOW と排他) or LogLink (RP2040 ロガーへ UART) ---
    if (S5::USE_SD) {
        Serial.println("Init SD (HW-125, CS=9, SPI0 共有)...");
        v.sd_ok = SdLog::begin(/*cs=*/9, sizeof(FlightLog::Rec), FlightLog::REC_VER,
                               (uint16_t)FlightLog::LOG_HZ);
        if (v.sd_ok) Serial.println("  SD OK");
        else { Serial.println("  !! SD 応答なし (CS=9 / VCC=5V / SCK13 MOSI11 MISO12 / FAT32) !!");
               SdLog::selftest(Serial); }
    } else {
        Serial.println(S5::USE_LOGLINK ? "SD: オンボード無効 (USE_LOGLINK=true。SD は RP2040 logger 側)"
                                       : "SD: 無効 (USE_FLOW=true のため。ログは RAM 'n'/'v' と USB 'l')");
    }
    if (S5::USE_LOGLINK) {
        // ポートは板ごと (quad/BoardPins.h)。2Mbaud。配線と移設の経緯は quad/LogLink.h。
        Serial.println("Init LOGLINK (" BOARD_LOGLINK_DESC ", 2Mbaud)...");
        v.link_ok = LogLink::begin(BOARD_LOGLINK_SERIAL, sizeof(FlightLog::Rec), FlightLog::REC_VER,
                                   (uint16_t)FlightLog::LOG_HZ);
        delay(600);
        LogLink::service();      // 溜まっている状態フレームを取り込む
        LogLink::status();
        // ログヘッダ後半にリセット原因と「前回ループが止まった区間」を載せる (全ファイル)。
        // バッテリー接続中は USB が使えないので、異常な再起動なら BLE で 1 本送る。
        uint8_t extra[16];
        WDT::fillHeaderExtra(extra);
        LogLink::setHeaderExtra(extra, sizeof(extra));
        if (v.link_ok && WDT::bootReportWanted()) {
            LogLink::startFile();
            boot_report_until_ms = millis() + Q::BOOT_REPORT_MS;
            Serial.printf(">>> 再起動の報告ログを BLE へ送ります (%lu 秒。リセット原因 %u)\n",
                          (unsigned long)(Q::BOOT_REPORT_MS / 1000), (unsigned)WDT::resetReason());
        }
    }
    if (S5::USE_FLOW && S5::FLOW_VIA_LINK) {
        // PMW3901 はロガー側。ここでは SPI に触らず、T_STAT の FLOW_OK と T_FLOW の鮮度を見る。
        Serial.println("Init OpticalFlow (PMW3901 @ ロガー, T_FLOW 経由)...");
        v.flow.beginLinked();
        v.flowobs.ok = LogLink::loggerFlowOk() && LogLink::flowFresh();
        Serial.println(v.flowobs.ok ? "  T_FLOW 受信中 OK"
                                    : "  !! T_FLOW 未受信 (ロガー側の PMW3901 / UART RX 配線 を確認。"
                                      "飛行中に来れば自動で POSHOLD 可になる) !!");
    }

    // 起動時デバイスチェック (結果は画面に残り、'd' で再表示できる)
    S5::SelfTest::probe(v);
    S5::SelfTest::printFull(Serial);

    S5::resetControllers(v);

    // ---- 電源投入 5秒後: 自動で 'k'+'r'+'z' (水平キャリブ → PIDリセット → フロー積算ゼロ) ----
    //  機体を水平に置いたまま待つだけで済むようにする。待機中にアームされたら中止。
    //  ★ ウォッチドッグのリセット直後は飛行中/墜落直後で水平とは限らないのでやらない
    //    (EEPROM の値のまま)。
    if (v.wdt_rebooted) {
        Serial.println(">>> ウォッチドッグリセット後のため自動キャリブレーションは省略します");
    } else {
        Serial.println("\n>>> 5秒後に自動キャリブレーション (k -> r -> z) を実行します。"
                       "機体を水平に置いて動かさないでください (アームすると中止)");
        const uint32_t wait_start = millis();
        bool aborted = false;
        while (millis() - wait_start < 5000) {
            if (S5::isArmed(v)) {
                aborted = true;
                Serial.println(">>> アーム検出: 自動キャリブレーションを中止します");
                break;
            }
            statusLedRainbowStep();
            delay(60);
        }
        if (!aborted) {
            Serial.println(">>> 自動キャリブレーション実行中 (k -> r -> z)...");
            S5::stopAllMotors(v);
            if (S5::USE_MPU) v.mpu.recalibrate(statusLedRainbowStep);   // 'k'
            S5::resetControllers(v);                                   // 'r'
            v.flowobs.zeroAccum();                                     // 'z'
            Serial.println(">>> 自動キャリブレーション完了 (k/r/z)");
        }
        StatusLed::off();
    }

    // 全 Ticker の基準時刻をそろえる (初回 dt が「起動からの経過」にならないように)
    for (Q::Ticker* t : { &main_tick, &flow_tick, &range_tick, &telem_rx_tick, &telem_tick, &debug_tick,
                          &guided_tick, &log_tick })
        t->prime();

    Serial.println("--- Setup complete ---");
    if (S5::UsbOwner::DUAL)
        Serial.println("--- USB シリアル (画面/キー) はコア1 に移ります ---");
    S5::UsbOwner::setupDone();
}

// ============================================================
//  § 8  loop  — 周期の骨組み (一覧は S5Features.h)
// ============================================================
void loop() {
    if (!main_tick.ready()) return;                     // 1000Hz 目標 (dt は実測)
    const uint32_t t0 = micros();
    const float dt_s = main_tick.dt_s();
    // ループ停止のウォッチドッグ (アーム中だけ有効)。WDT::mark() は「今どの区間か」を
    // リセットで消えないレジスタに残す。止まってリセットされたら起動時に表示する。
    WDT::feed();

    // --- センサ ---
    WDT::mark(WDT::SEC_IMU);
    readImu(dt_s);                                      // 毎ループ 姿勢 / 高度推定 predict
    const uint32_t t1 = micros();
    WDT::mark(WDT::SEC_SBUS);
    if (S5::USE_SBUS) v.sbus.update();                  // 毎ループ プロポ (換算は新フレームの回だけ)
    WDT::mark(WDT::SEC_FAILSAFE);
    S5::Failsafe::update(v);                            // SBUS 途絶 / IMU 固まり (S5Failsafe.h)
    const uint32_t t2 = micros();

    WDT::mark(WDT::SEC_FLOW);
    if (S5::USE_FLOW && flow_tick.ready()) {                          // 100Hz  フロー読み
        // de-rotation には「前回のフロー読みからメインループで積分した回転角」を渡す
        // (レートループと同じジャイロ値の積分。区間がカウントと一致する。OpticalFlow.h)
        float d_roll_deg, d_pitch_deg;
        v.flow_rot.take(d_roll_deg, d_pitch_deg);
        if (S5::FLOW_VIA_LINK) {
            // ロガー側 PMW3901。リンクの鮮度をそのまま flowobs.ok にする → 途切れれば
            // flowAlive() が false になり POSHOLD → ALTHOLD へ縮退 (既存の経路)。
            v.flowobs.ok = LogLink::loggerFlowOk() && LogLink::flowFresh();
            if (v.flowobs.ok) {
                int16_t dx, dy; uint8_t sq;
                LogLink::pollFlow(dx, dy, sq);                       // 新着なしなら 0,0
                v.flow.updateFrom(flow_tick.dt_s(), d_roll_deg, d_pitch_deg, dx, dy, sq);
            }
        } else if (v.flowobs.ok) {
            v.flow.update(flow_tick.dt_s(), d_roll_deg, d_pitch_deg);
        }
        if (v.flowobs.ok && v.flow.consumeFresh()) {                  //  25Hz  窓が締まった回だけ
            v.flowobs.take(v.flow, v.flow.lastDt());
            WDT::mark(WDT::SEC_FLOWHOLD);
            updateFlowHold(v.flow.lastDt());
        }
    }
    const uint32_t t3 = micros();

    WDT::mark(WDT::SEC_RANGE);
    if (S5::USE_RANGE && v.range.ok && range_tick.ready()) {          // 100Hz  測距
        v.range.take(v.rangefinder, v.rangefinder.update(v.att.roll, v.att.pitch));
        // 失探しても flow の height は「最後に有効だった値」を保持する (急に 1.0m へ飛ぶより安全)
        if (S5::USE_FLOW && v.range.valid) v.flow.setHeight(v.range.h_m);
        // 高度推定の correct。dt は「前回 correct からの経過」(呼び出し周期ではない)
        if (v.range.fresh && v.range.valid) {
            static uint32_t est_last_us = 0;
            const uint32_t now_us = micros();
            const float est_dt = (est_last_us == 0) ? 0.0f : (float)(now_us - est_last_us) * 1e-6f;
            est_last_us = now_us;
            v.altest.correct(v.range.h_m, est_dt);
        }
        WDT::mark(WDT::SEC_ALTHOLD);
        updateAltHold(range_tick.dt_s());
    }
    const uint32_t t4 = micros();

    // --- 地上局 ---
    WDT::mark(WDT::SEC_GROUND);
    if (S5::USE_IM920 && telem_rx_tick.ready() && v.s5rx.poll())     // 200Hz  上りコマンド受信 (IM920)
        v.guided.onCommand(v.s5rx.last());
    if (S5::USE_BLE_LINK && v.link_ok) {                              // 毎ループ 上りコマンド (BLE)
        // log_recorder が中継してきた T_CMD (前のループの LogLink::service() が
        // mailbox に置いたもの) を IM920 と同じ s5rx へ。統計・鮮度も共通。
        uint8_t cmd[LogLinkProto::CMD_PAYLOAD];
        // 新しい seq なら Guided へ (パターン選択 / 開始 / 中止。quad/Guided.h onCommand)
        if (LogLink::pollCmd(cmd) && v.s5rx.acceptRaw(cmd, sizeof(cmd))) v.guided.onCommand(v.s5rx.last());
    }
    WDT::mark(WDT::SEC_GUIDED);
    if (guided_tick.ready()) updateGuided(guided_tick.dt_s());       // 100Hz  要求 → 目標 (制御はしない)
    const uint32_t t5 = micros();

    // --- 姿勢制御 → ESC ---
    WDT::mark(WDT::SEC_CONTROL);
    updateControl(dt_s);                                              // 毎ループ (実測 dt)
    const uint32_t t6 = micros();

    WDT::mark(WDT::SEC_CONSOLE);
    if (S5::UsbOwner::DUAL) {
        // RP2040: キーはコア1 が読んで置いていく。ここで処理する (その間コア1 は USB から退いている)
        char key;
        if (S5::UsbOwner::keyPending(key)) {
            S5::Console::handleKey(v, key);
            S5::UsbOwner::keyDone();
        }
    } else {
        S5::Console::handleSerial(v);                                 // USB キー (あれば)
    }
    const uint32_t t7 = micros();

    // --- サービス (ログ / LED / 無線送信) ---
    WDT::mark(WDT::SEC_ARMCHECK);
    const bool  armed_now = S5::isArmed(v);
    v.armed_now = armed_now;                                          // コア1 の画面はこれを読む
    WDT::markState((uint8_t)v.mode, (uint8_t)v.guided.phase(), v.fs, armed_now);
    const float thr_now   = S5::USE_SBUS ? v.sbus.des[Ch::THR] : 0.0f;
    WDT::mark(WDT::SEC_LEDS);
    serviceLeds(armed_now);
    WDT::mark(WDT::SEC_LOGFILES);
    serviceLogFiles(armed_now);
    WDT::mark(WDT::SEC_FLIGHTLOG);
    if (log_tick.ready()) serviceFlightLog(armed_now, thr_now);      // LOG_HZ (125Hz)
    WDT::mark(WDT::SEC_SERVICE);
    if (v.sd_ok)   SdLog::service();                                  // 毎ループ、有界の書き出し
    WDT::mark(WDT::SEC_LOGLINK);
    if (v.link_ok && S5::STATUS_LED_VIA_LINK) LogLink::serviceLed(StatusLed::rgbBits());
    if (v.link_ok) LogLink::service();                                // 毎ループ、有界の UART 送信
    WDT::mark(WDT::SEC_BLEACT);
    S5::Console::handleBleAction(v);                                  // BLE 経由のデバッグ指令 (あれば)

    const uint32_t t8 = micros();
    WDT::mark(WDT::SEC_TELEM);
    if (S5::USE_IM920 && telem_tick.ready()) telem.tick(v);           //   8Hz  下りテレメトリ (積むだけ)
    if (S5::USE_BLE_LINK && v.link_ok && telem_tick.ready()) telem.tickBle(v);   // 20Hz  同 (BLE。束ねて LogLink へ)
    if (S5::USE_IM920) v.s5tx.service();                              // 送りかけを毎ループ吐き出す (非ブロッキング)

    const uint32_t t9 = micros();

    // デバッグ画面。RP2040 はコア1 (§ 9) が出すのでここでは何もしない。
    // Teensy: ログ中は止める (同じ USB を奪い合うとログが落ちる)。アーム中も出さない
    //  (1 回 2〜3KB の書式化で 10ms 級のループ抜けが 10Hz で出る / PC が読まないと
    //  USB の書き込みが最大 1 秒待つ。quad/SafeLog.h)。
    WDT::mark(WDT::SEC_STATUS);
    if (!S5::UsbOwner::DUAL && !armed_now && !FlightLog::Usb::active && debug_tick.ready())
        S5::printStatus(v, main_tick.dt_us);                          // 10Hz (地上のみ)
    WDT::mark(WDT::SEC_NONE);
    const uint32_t t10 = micros();

    // 区間ごとの所要時間 (quad/LoopProfile.h)。並びは LoopProfile::Sec
    const uint32_t sec[S5::LoopProfile::N] = {
        t1 - t0, t2 - t1, t3 - t2, t4 - t3, t5 - t4, t6 - t5, t7 - t6, t8 - t7, t9 - t8, t10 - t9 };
    S5::LoopProfile::add(sec, t10 - t0, main_tick.dt_us, armed_now);

// 記録はアーム中だけ (地上の画面表示で 32 件が埋まるのを防ぐ)
    if (armed_now) S5::StallLog::maybeLog(t10 - t0, t1 - t0, t2 - t1, t3 - t2, t4 - t3, t5 - t4, t6 - t5,
                           t7 - t6, t8 - t7, t9 - t8);
}

// ============================================================
//  § 9  コア1 (RP2040 のみ) — USB シリアルの持ち主: 画面 / キー / SafeLog
// ============================================================
//  ★ 2026-09-17: USB の書き込みは PC が読まないと最大 1 秒待ち、しかも 2 コアで
//    ロックを共有しているので、コア0 から USB に触ると制御ループが止まる。
//    USB を触るのはここだけにし、コア0 とは quad/UsbOwner.h のフラグで受け渡す。
//    ここが何秒止まっても制御 (コア0) には影響しない。
//
//  コア1 は setup() より前に起動する。setup() が終わる (UsbOwner::setupDone) まで
//  USB には触らない (起動メッセージはコア0 が出す)。
#if defined(ARDUINO_ARCH_RP2040) && !defined(S5_SINGLE_CORE)
// コア1 のスタックを 8KB に (既定は 2KB。printStatus の浮動小数 printf が深い)
bool core1_separate_stack = true;

static Q::Ticker core1_status_tick(S5::DEBUG_HZ);

void setup1() {
    // setup() 側の初期化を待つ (USB に触らない)
    while (!S5::UsbOwner::setupIsDone()) delay(10);
    core1_status_tick.prime();
}

void loop1() {
    if (!S5::UsbOwner::core1MayUse()) { delay(1); return; }   // コア0 が USB を使っている

    // キー: 読んだらコア0 に渡して、処理が終わるまで USB から退く
    if (Serial.available()) {
        S5::UsbOwner::core1PostKey((char)Serial.read());
        return;
    }
    // USB 直結ログ ('l') の間は、コア0 が USB に書いている。画面と SafeLog は止める
    if (FlightLog::Usb::active) { delay(1); return; }

    Q::SafeLog::drainTo(Serial);                             // コア0 の飛行中メッセージ
    if (core1_status_tick.ready()) S5::printStatus(v, main_tick.dt_us);
    else                           delay(1);
}
#endif
