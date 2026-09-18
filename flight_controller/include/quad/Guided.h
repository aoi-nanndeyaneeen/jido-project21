// ============================================================
//  Guided.h  -  GUIDED (SW_HOVER=up) の状態機械
// ============================================================
//  ★ 2026-09-17: GUIDED の中身を「機体単独の周回」に差し替えた。
//    SW_HOVER を cen(POSHOLD) → up に上げた瞬間、今いる点を円の左端として
//    機首方向を接線に、半径 CIRCLE_RADIUS_M の右旋回を 1 周する
//    (quad/CircleTrack.h)。位置はフロー積分、方位はジャイロ積分だけで決まり、
//    地上局のリンクは一切見ない。1 周したら POSHOLD に戻ってその場ホールド。
//    GUIDED_PATTERN=Figure8 では半径 FIG8_RADIUS_M で右旋回 1 周 → 起点に
//    戻ったら左旋回 1 周 (起点で接する 2 円 = 8 の字) してから POSHOLD。
//    GUIDED_PATTERN=ClimbTurn では半径 CLIMB_RADIUS_M を止まらずに 5 周し、高度を
//    開始高度 (2 周) → 上昇 (1 周) → CLIMB_ALT_M (2 周) と動かし、最後にその場で
//    開始高度まで降りて (GP_DESCEND) から POSHOLD。
//    GUIDED_PATTERN=Straight (2026-09-18) では、まずその場でアーム時の機首 (= 置いた
//    向き) へ向き直し (GP_ALIGN)、その向きへ STRAIGHT_DIST_M 直進 (GP_STRAIGHT,
//    quad/StraightTrack.h) して止まり POSHOLD。
//    もう一度回すには SW_HOVER を一度 cen に戻してから up。
//  ★ 2026-09-18: 地上局 (ble_monitor.py) の REQ_CIRCLE / REQ_FIGURE8 / REQ_CLIMB_TURN で
//    パターン・半径・速度を選べる (onCommand)。SW_HOVER=up で待っている間に届けば
//    cen に戻さなくてもその場で開始する。REQ_ABORT / REQ_HOLD で中止 → POSHOLD。
//    上の安全の骨組みはそのまま (開始のゲートも同じものを通る)。
//
//    これまでの「地上局の要求 (S5C::CmdFrame) を目標速度/高度へ翻訳する」
//    ミッション経路 (離陸 / 巡航 / 着陸 / 定型機動 / 位置・ヨー補正) は
//    下の `#if 0` ブロックにコメントアウトして残してある。戻すときは
//    drone_s5.cpp の updateFlowHold()/updateControl() の該当箇所も一緒に戻す。
//
//  ここがやるのは **目標を作るだけ**:
//      CircleTrack  ->  poshold.setTrajectory() (目標点・接線速度・向心加速度)
//                       yawRate()               (ヨーのレート指令)
//                       altM()/slew()           (AltHold の目標高度 = 開始時の高度)
//  制御経路は POSHOLD と同一 (PosHold → 角度 → レート)。
//
//  ★ 安全の骨組み (ここを崩さないこと)
//    1. スロットルスティックは常にパイロットのもの。GUIDED でも
//       FLOW_ENABLE_THR / ALT_ENABLE_THR (15%) を下回れば全部手放す。
//    2. SW_HOVER を下げれば ANGLE (完全手動)。これが最終の bail-out。
//    3. SW_HOVER を cen に戻せば POSHOLD (その場ホールド)。
//    4. ロール/ピッチ/ヨースティックを動かせば GUIDED から自動で抜ける。
//    5. 周回は「上げた瞬間」にしか始まらない。途中で抜けたら、スイッチを
//       cen に戻すまで再開しない (スティックを離した途端に勝手に回り出さない)。
//
//  呼び出しは S5::GUIDED_HZ (100Hz)。定数は QuadConfig.h § 9。
// ============================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include "quad/QuadConfig.h"
#include "quad/PosHold.h"
#include "quad/HeadingHold.h"
#include "quad/Maneuver.h"
#include "quad/CircleTrack.h"
#include "quad/StraightTrack.h"
#include "quad/SafeLog.h"
#include "S5Cmd.h"

namespace Quad {

enum GuidedPhase : uint8_t {
    GP_OFF = 0,     // 使っていない (SW_HOVER が up でない / 条件を満たさない)
    GP_HOLD,        // その場ホールド (水平 0。高度は最後の目標を保持)
    GP_TAKEOFF,     // 目標高度まで自動上昇中。水平 0
    GP_CRUISE,      // 地上局の速度指令に従って移動中
    GP_LAND,        // 自動着陸中。水平 0
    GP_LANDED,      // 接地を検知して出力を切った (ディスアームするまで保持)
    GP_MANEUVER,    // 定型機動 (CIRCLE/FIGURE8/CLIMB_TURN)。開始後は機体単独で進む
    GP_CIRCLE,      // 機体単独の周回 (CircleTrack)。2026-09-17〜 GUIDED はこれだけ
    GP_DESCEND,     // 上昇旋回の後、その場で開始高度まで降下中 (水平はホールド)
    GP_ALIGN,       // 直進の前に、その場で進む方位へ向き直り中
    GP_STRAIGHT,    // 機体単独の直進 (StraightTrack)
};

inline const char* guidedPhaseName(GuidedPhase p) {
    static const char* const N[] = { "OFF", "HOLD", "TAKEOFF", "CRUISE", "LAND", "LANDED",
                                     "MANEUV", "CIRCLE", "DESCEND", "ALIGN", "STRAIGHT" };
    return N[(int)p];
}

class Guided {
public:
    // update() が毎回見る「機体側の事実」。drone_s5.cpp が詰めて渡す。
    struct Inputs {
        uint32_t now_ms;
        float dt_s;           // 前回 update() からの実測経過 [s] (Ticker)。周回/直進の目標を進める
        bool  armed;
        bool  sbus_ok;        // S5::USE_SBUS
        bool  sw_hover_up;    // SW_HOVER == up (GUIDED の入り口)
        bool  flow_alive;     // フローが使える (初期化済み && 空中で凍結していない)
        bool  range_ok;       // 測距センサが初期化できている
        bool  range_valid;    // 今この瞬間の測距が信用できる
        float range_h_m;      // 鉛直対地高度 [m]
        bool  airborne;       // AltHold が離陸を検知済み (= 飛行中)
        bool  hold_ready;     // POSHOLD の水平ループが今効いている (スロットル十分)
        float pos_n, pos_e;   // PosHold の推定位置 (地面固定) [m]
        float stick_roll;     // sbus.des 生値 (符号適用前)
        float stick_pitch;
        float stick_yaw;
        float yaw_est_deg;    // HeadingHold::est() (機体の実測ヨー)
        float yaw_since_arm_deg;  // HeadingHold::sinceArm() (置いた向きからの回転)
    };

    // PosHold::setTrajectory() へ渡す目標 (地面固定フレーム)
    struct Trajectory { float ref_n, ref_e, vel_n, vel_e, acc_n, acc_e; };

    // ------------------------------------------------------------
    //  地上局の CmdFrame (新しい seq が届いた回だけ drone_s5.cpp が呼ぶ)。
    //  ★ log_recorder は PC が黙っている間も 200ms ごとに最後の指令を seq を
    //    振り直して再送してくるので、「req が変わった瞬間」だけを要求として扱う。
    //    同じ機動をもう一度頼むときは、PC 側が間に IDLE を挟む (ble_monitor.py)。
    //  ここでは要求を溜めるだけ。開始/中止の判断は update() がゲートを見てから。
    // ------------------------------------------------------------
    void onCommand(const S5C::CmdFrame& c) {
        const bool edge = !_cmd_seen || c.req != _cmd_req_prev;
        _cmd_seen = true;
        _cmd_req_prev = c.req;
        if (!edge) return;

        switch (c.req) {
            case S5C::REQ_CIRCLE:
            case S5C::REQ_FIGURE8:
            case S5C::REQ_CLIMB_TURN: {
                const GuidedPattern pat = (c.req == S5C::REQ_CIRCLE)  ? GuidedPattern::Circle
                                        : (c.req == S5C::REQ_FIGURE8) ? GuidedPattern::Figure8
                                                                      : GuidedPattern::ClimbTurn;
                const float speed = constrain(fabsf((float)c.vx_mmps / S5C::SC_MMPS),
                                              GUIDED_CMD_SPEED_MIN_MPS, GUIDED_CMD_SPEED_MAX_MPS);
                const float rate  = (float)c.yaw_rate_cdps / S5C::SC_CDPS;
                float radius = defaultRadius(pat);
                if (fabsf(rate) > 0.1f) radius = speed / (fabsf(rate) * DEG_TO_RAD);
                _req.pat    = pat;
                _req.speed  = speed;
                _req.radius = constrain(radius, GUIDED_CMD_RADIUS_MIN_M, GUIDED_CMD_RADIUS_MAX_M);
                _req.dir    = (rate < -0.1f) ? -1 : (rate > 0.1f) ? +1 : CIRCLE_DIR;
                _start_req  = true;
                break;
            }
            case S5C::REQ_ABORT:
            case S5C::REQ_HOLD:
                _stop_req = true;
                break;
            default:
                break;
        }
    }

    // ------------------------------------------------------------
    //  100Hz。機体単独の周回。
    // ------------------------------------------------------------
    void update(const Inputs& in) {
        if (!GUIDED_ENABLE) { _engaged = false; _gp = GP_OFF; return; }
        const uint32_t now = in.now_ms;
        // 目標を進める dt は呼び出し側の Ticker の実測値 (2026-09-18〜。以前は millis の差で
        // 1ms 刻み。100Hz なら 1 回 10% の丸め誤差が乗っていた)。異常値は捨てる。
        const float dt_s = (in.dt_s > 0.0f && in.dt_s < 0.5f) ? in.dt_s : 0.0f;
        _yaw_est_deg = in.yaw_est_deg;

        // --- 地上局の要求 (onCommand が溜めたもの) は毎回ここで消費する -------
        if (_stop_req) {
            _stop_req = false;
            if (_engaged) disengage("地上局から中止");
        }
        if (_start_req) {
            _start_req = false;
            if (_engaged) {
                // 飛行中のパターンを途中で差し替えない (8 の字の 2 円目などが化ける)
                SafeLog::logf("!! 実行中のため地上局の開始要求は無視\n");
            } else {
                _sel = _req;
                SafeLog::logf("\n>>> 地上局: %s を選択 (半径 %.2f m, %.2f m/s, %s旋回)%s\n",
                              patternName(), _sel.radius, _sel.speed, (_sel.dir < 0) ? "左" : "右",
                              in.sw_hover_up ? "" : " -> SW_HOVER を up に上げると開始");
                if (in.sw_hover_up) _await_rearm = false;   // up のまま待っていた → 下の開始処理へ
            }
        }

        // スイッチを up 以外に戻したら、次に上げたとき周回を受け付ける
        if (!in.sw_hover_up) _await_rearm = false;

        // --- 0) 続ける/入る資格 (毎回全部見る) ----------------------
        if (!in.armed)        { disengage("ディスアーム");         return; }
        if (!in.sbus_ok)      { disengage("SBUS 無効");           return; }
        if (!in.sw_hover_up)  { disengage("SW_HOVER が下");       return; }
        if (_await_rearm)     { return; }   // 途中で抜けた/終わった。cen に戻すまで待つ
        if (!in.flow_alive)   { refuse("フローが死んでいる");      return; }
        if (!in.range_ok)     { refuse("測距が無い");              return; }
        if (!in.airborne)     { refuse("離陸していない");          return; }
        if (!in.hold_ready)   { refuse("POSHOLD が効いていない");  return; }
        if (fabsf(in.stick_roll) > FLOW_STICK_DEAD || fabsf(in.stick_pitch) > FLOW_STICK_DEAD) {
            refuse("スティック操作を検出");
            return;
        }
        if (fabsf(in.stick_yaw) > Gain::YAW_STICK_DEAD) {
            refuse("ヨースティック操作を検出");
            return;
        }

        // --- 1) 開始 (up に上げた瞬間の 1 回だけ) ----------------------
        if (!_engaged) {
            const float alt_now = (in.range_valid && in.range_h_m > 0.05f) ? in.range_h_m : ALT_TARGET_M;
            if (_sel.pat == GuidedPattern::ClimbTurn && alt_now > CLIMB_ALT_M - CLIMB_MIN_RISE_M) {
                refuse("開始高度が到達高度 (CLIMB_ALT_M) に近すぎる");
                return;
            }
            if (_sel.pat == GuidedPattern::Straight) {
                startStraight(in, alt_now);
                return;
            }
            _engaged = true;
            _why     = "";
            _gp      = GP_CIRCLE;
            _alt_m   = alt_now;
            _alt_low = alt_now;
            _slew    = GUIDED_CRUISE_SLEW_MPS;
            // 起点は全部の円で共通。2 つ目以降の円もここから始める (機体の実測位置では
            // なく目標の起点を使うので、1 つ目の円の閉じ誤差が 2 つ目に持ち越されない)
            _p0_n = in.pos_n;
            _p0_e = in.pos_e;
            _leg  = 0;
            const bool fig8 = (_sel.pat == GuidedPattern::Figure8);
            _n_legs = fig8 ? 2 : 1;
            SafeLog::logf("\n>>> %s開始: 起点(N,E)=(%+.2f, %+.2f) 機首 %.1f deg, 高度 %.2f m\n",
                          patternName(), _p0_n, _p0_e, in.yaw_est_deg, _alt_m);
            beginLeg(in.yaw_est_deg);
            return;   // 目標は次の回から進める (dt を開始時刻から数える)
        }

        if (_gp == GP_ALIGN || _gp == GP_STRAIGHT) {
            updateStraight(in, dt_s, now);
            return;
        }

        // --- 2') 上昇旋回の後の降下 (水平は drone_s5 がその場ホールド) --------
        if (_gp == GP_DESCEND) {
            _alt_m = _alt_low;
            _slew  = CLIMB_DESCEND_SLEW_MPS;
            const bool near = in.range_valid && fabsf(in.range_h_m - _alt_low) < CLIMB_DESCEND_TOL_M;
            if (!near)                    _near_since_ms = 0;
            else if (_near_since_ms == 0) _near_since_ms = now;
            if (_near_since_ms != 0 && now - _near_since_ms >= CLIMB_DESCEND_SETTLE_MS) {
                disengage("上昇旋回完了 (開始高度へ降下済み)");
            } else if (now - _descend_start_ms >= CLIMB_DESCEND_TIMEOUT_MS) {
                disengage("降下が時間上限 (高度はその時の目標のまま)");
            }
            return;
        }

        // --- 2) 進める -----------------------------------------------
        _circle.update(dt_s, in.pos_n, in.pos_e);
        if (_sel.pat == GuidedPattern::ClimbTurn) {
            // 実測の周回数で高度目標を動かす: 低 → (1 周で直線的に) → 高
            const float lap  = _circle.lapsMeas();
            const float rise = (float)CLIMB_RISE_LAPS;
            const float f    = constrain((lap - (float)CLIMB_LOW_LAPS) / rise, 0.0f, 1.0f);
            _alt_m = _alt_low + (CLIMB_ALT_M - _alt_low) * f;
        }
        if (_circle.done()) {
            const bool last = (_leg + 1 >= _n_legs);
            SafeLog::logf("\n>>> 円 %d/%d %s: %.1f s, 実測 %.0f deg, 起点までのずれ %.2f m\n",
                          _leg + 1, _n_legs, _circle.timedOut() ? "打ち切り (時間上限)" : "完了",
                          _circle.elapsedS(), _circle.progressMeasDeg(),
                          hypotf(in.pos_n - _p0_n, in.pos_e - _p0_e));
            if (_sel.pat == GuidedPattern::ClimbTurn) {
                // 打ち切りでも完了でも、その場で開始高度へ降りてから抜ける
                _gp = GP_DESCEND;
                _descend_start_ms = now;
                _near_since_ms    = 0;
                SafeLog::logf(">>> 開始高度 %.2f m へ降下します (今 %.2f m)\n", _alt_low, in.range_h_m);
                return;
            }
            if (_circle.timedOut()) { disengage("円が時間上限"); return; }
            if (last)               { disengage(_n_legs > 1 ? "8の字完了" : "周回完了"); return; }
            // 次の円: 目標機首は前の円の終わり (= 起点で接線方向) から連続させる
            _leg++;
            beginLeg(_circle.yawRefDeg());
        }
    }

    // 途中で抜けた場合と、そもそも入れなかった場合の両方。スイッチを戻すまで再開しない。
    void refuse(const char* why) {
        if (!_engaged) {
            SafeLog::logf("\n!! 周回を開始できません: %s (SW_HOVER を cen に戻してやり直し)\n", why);
            _why = why;
            _await_rearm = true;
            return;
        }
        disengage(why);
    }

    // resetControllers() から (地上局ミッションのヨー補正用。今は何もしない)
    void resetYawCorrSeq() { _yaw_corr_seq_init = false; }

    // --- 出力 (drone_s5 が PosHold / AltHold / ヨー へ渡す) -----------
    bool        engaged()     const { return _engaged; }
    GuidedPhase phase()       const { return _gp; }
    bool        landed()      const { return _gp == GP_LANDED; }
    bool        inManeuver()  const {
        return (_gp == GP_CIRCLE && _circle.active()) || _gp == GP_ALIGN
            || (_gp == GP_STRAIGHT && _straight.active());
    }
    // 目標を trajectory() で PosHold に渡している間
    bool        tracking()    const { return inManeuver(); }
    // ヨーのレート指令 [deg/s]。追従中でなければ NAN (= HeadingHold の通常動作)
    float       yawRate()     const {
        if (!tracking()) return NAN;
        if (_gp == GP_CIRCLE) return _circle.yawRateCmd(_yaw_est_deg);
        if (_gp == GP_STRAIGHT) return _straight.yawRateCmd(_yaw_est_deg);
        // GP_ALIGN: その場で進む方位へ
        const float err = constrain(_line_deg - _yaw_est_deg,
                                    -CIRCLE_YAW_ERR_LIM_DEG, CIRCLE_YAW_ERR_LIM_DEG);
        return constrain(CIRCLE_YAW_KP * err, -STRAIGHT_YAW_RATE_LIM_DPS, STRAIGHT_YAW_RATE_LIM_DPS);
    }
    Trajectory  trajectory()  const {
        if (_gp == GP_CIRCLE)
            return { _circle.refN(), _circle.refE(), _circle.velN(), _circle.velE(),
                     _circle.accN(), _circle.accE() };
        if (_gp == GP_STRAIGHT)
            return { _straight.refN(), _straight.refE(), _straight.velN(), _straight.velE(),
                     _straight.accN(), _straight.accE() };
        return { _p0_n, _p0_e, 0.0f, 0.0f, 0.0f, 0.0f };   // GP_ALIGN: 起点で止まる
    }
    float       vx()          const { return _vx; }      // 機体座標 前+ [m/s] (周回では未使用)
    float       vy()          const { return _vy; }      // 同 右+ [m/s]
    float       altM()        const { return _alt_m; }   // 目標対地高度 [m] (0 = 指令なし)
    float       slew()        const { return _slew; }    // 目標高度を動かしてよい速さ [m/s]
    const char* why()         const { return _why; }     // 直前に GUIDED を抜けた理由
    const CircleTrack& circle() const { return _circle; }
    const StraightTrack& straight() const { return _straight; }
    float lineDeg() const { return _line_deg; }   // 直進の方位 (HeadingHold::est() の系)
    int legIndex() const { return _leg; }        // 今の円 (0 始まり)
    int legCount() const { return _n_legs; }

private:
    // _leg 番目の円を起点 (_p0) から始める。8 の字は円ごとに向きを反転する。
    void beginLeg(float yaw_deg) {
        const float radius = _sel.radius, speed = _sel.speed;
        const int   laps   = (_sel.pat == GuidedPattern::ClimbTurn)
                           ? CLIMB_LOW_LAPS + CLIMB_RISE_LAPS + CLIMB_HIGH_LAPS : 1;
        const int dir = ((_leg % 2) == 0) ? _sel.dir : -_sel.dir;
        _circle.begin(_p0_n, _p0_e, yaw_deg, radius, speed, CIRCLE_ACCEL_MPS2, dir, laps);
        SafeLog::logf(">>> 円 %d/%d 開始: 半径 %.2f m %s旋回 %.2f m/s %d 周 (想定 %.1f s) "
                      "機首 %.1f deg 中心(N,E)=(%+.2f, %+.2f)\n",
                      _leg + 1, _n_legs, radius, (dir < 0) ? "左" : "右", speed, laps,
                      _circle.expectS(), yaw_deg, _circle.centerN(), _circle.centerE());
        if (_sel.pat == GuidedPattern::ClimbTurn)
            SafeLog::logf("    高度: %.2f m で %d 周 → %d 周で %.2f m へ → %d 周\n", _alt_low,
                          CLIMB_LOW_LAPS, CLIMB_RISE_LAPS, CLIMB_ALT_M, CLIMB_HIGH_LAPS);
    }

    // 直進の開始 (up に上げた瞬間)。進む方位を決めて、まずその場で向き直る。
    void startStraight(const Inputs& in, float alt_now) {
        const float off = STRAIGHT_USE_ARM_HEADING ? in.yaw_since_arm_deg : 0.0f;
        if (fabsf(off) > STRAIGHT_ALIGN_MAX_DEG) {
            refuse("置いた向きから機首が回りすぎ (STRAIGHT_ALIGN_MAX_DEG)");
            return;
        }
        _engaged = true;
        _why     = "";
        _gp      = GP_ALIGN;
        _alt_m   = alt_now;
        _alt_low = alt_now;
        _slew    = GUIDED_CRUISE_SLEW_MPS;
        _p0_n    = in.pos_n;
        _p0_e    = in.pos_e;
        _line_deg = in.yaw_est_deg - off;
        _phase_start_ms = in.now_ms;
        _near_since_ms  = 0;
        SafeLog::logf("\n>>> 直進開始: 起点(N,E)=(%+.2f, %+.2f) 高度 %.2f m, %s\n"
                      "    機首は置いた向きから %+.1f deg → 方位 %.1f deg (est 系) へ向き直ってから "
                      "%.1f m / %.2f m/s\n",
                      _p0_n, _p0_e, _alt_m,
                      STRAIGHT_USE_ARM_HEADING ? "アーム時の機首へ進む" : "今の機首へ進む",
                      in.yaw_since_arm_deg, _line_deg, STRAIGHT_DIST_M, STRAIGHT_SPEED_MPS);
    }

    void updateStraight(const Inputs& in, float dt_s, uint32_t now) {
        if (_gp == GP_ALIGN) {
            const bool near = fabsf(_line_deg - in.yaw_est_deg) < STRAIGHT_ALIGN_TOL_DEG;
            if (!near)                    _near_since_ms = 0;
            else if (_near_since_ms == 0) _near_since_ms = now;
            if (_near_since_ms != 0 && now - _near_since_ms >= STRAIGHT_ALIGN_SETTLE_MS) {
                _gp = GP_STRAIGHT;
                _straight.begin(_p0_n, _p0_e, _line_deg,
                                STRAIGHT_DIST_M, STRAIGHT_SPEED_MPS, STRAIGHT_ACCEL_MPS2);
                SafeLog::logf(">>> 向き合わせ完了 (%.1f s, 置いた向きから %+.2f deg) → 直進 (想定 %.1f s)\n",
                              (now - _phase_start_ms) * 1e-3f, in.yaw_since_arm_deg,
                              _straight.expectS());
            } else if (now - _phase_start_ms >= STRAIGHT_ALIGN_TIMEOUT_MS) {
                disengage("向き合わせが時間上限 (出発しない)");
            }
            return;
        }

        _straight.update(dt_s, in.pos_n, in.pos_e);
        if (_straight.done()) {
            SafeLog::logf("\n>>> 直進%s: %.1f s, フロー上で 前 %.2f m / 横 %+.2f m (最大 %+.2f m), "
                          "機首は置いた向きから %+.2f deg\n"
                          "    ★ 実際の横ずれは巻尺で測ること (ジャイロ/フロー取付/置き方はこの数字に出ない)\n",
                          _straight.timedOut() ? "打ち切り (時間上限)" : "完了",
                          _straight.elapsedS(), _straight.along(), _straight.cross(),
                          _straight.crossMax(), in.yaw_since_arm_deg);
            disengage(_straight.timedOut() ? "直進が時間上限" : "直進完了");
        }
    }

    const char* patternName() const {
        switch (_sel.pat) {
            case GuidedPattern::Figure8:   return "8の字";
            case GuidedPattern::ClimbTurn: return "上昇旋回";
            case GuidedPattern::Straight:  return "直進";
            default:                       return "周回";
        }
    }

    // パターンごとの既定 (QuadConfig.h)。地上局が半径/速度を指定しなかったとき用。
    static float defaultRadius(GuidedPattern p) {
        switch (p) {
            case GuidedPattern::Figure8:   return FIG8_RADIUS_M;
            case GuidedPattern::ClimbTurn: return CLIMB_RADIUS_M;
            default:                       return CIRCLE_RADIUS_M;
        }
    }
    static float defaultSpeed(GuidedPattern p) {
        switch (p) {
            case GuidedPattern::Figure8:   return FIG8_SPEED_MPS;
            case GuidedPattern::ClimbTurn: return CLIMB_SPEED_MPS;
            default:                       return CIRCLE_SPEED_MPS;
        }
    }

    // 何をどう飛ぶか。起動時は QuadConfig.h の GUIDED_PATTERN、地上局の要求で差し替わる。
    struct Selection { GuidedPattern pat; float radius, speed; int dir; };
    static Selection defaultSelection() {
        return { GUIDED_PATTERN, defaultRadius(GUIDED_PATTERN), defaultSpeed(GUIDED_PATTERN), CIRCLE_DIR };
    }

    void disengage(const char* why) {
        if (_engaged) {
            _engaged = false;
            _why     = why;
            _await_rearm = true;      // スイッチを cen に戻すまで再開しない
            SafeLog::logf("\n>>> GUIDED 解除: %s\n", why);
        }
        _gp    = GP_OFF;
        _vx = _vy = 0.0f;
        _alt_m = 0.0f;
        _slew  = 0.0f;
        _circle.abort();
        _straight.abort();
    }

    Selection _sel = defaultSelection();   // 実際に飛ぶもの (非実行中にだけ書き換える)
    Selection _req = defaultSelection();   // onCommand が受けた最新の要求
    bool    _start_req = false, _stop_req = false;
    bool    _cmd_seen = false;
    uint8_t _cmd_req_prev = 0;

    GuidedPhase _gp = GP_OFF;
    bool  _engaged = false;         // GUIDED に入っているか (selectMode が見る)
    bool  _await_rearm = false;     // 抜けた後、SW_HOVER が up から外れるまで true
    float _vx = 0.0f, _vy = 0.0f;   // 機体座標の目標速度 [m/s] (地上局ミッション用。周回では 0)
    float _alt_m = 0.0f;            // 目標対地高度 [m] (0 = 指令なし)
    float _slew  = 0.0f;            // 目標高度を動かしてよい速さ [m/s]
    const char* _why = "";
    float    _yaw_est_deg = 0.0f;   // 直近の実測ヨー

    CircleTrack _circle;
    float   _p0_n = 0.0f, _p0_e = 0.0f;   // 起点 (全部の円で共通)
    int     _leg = 0, _n_legs = 1;
    float   _alt_low = 0.0f;              // 開始高度 (上昇旋回の低い方 / 降下の目標)
    uint32_t _descend_start_ms = 0;
    uint32_t _near_since_ms    = 0;       // 開始高度 / 進む方位の ±TOL に入り続けている開始時刻 (0 = 入っていない)
    StraightTrack _straight;
    float    _line_deg = 0.0f;            // 直進の方位 (HeadingHold::est() の系)
    uint32_t _phase_start_ms = 0;         // GP_ALIGN に入った時刻
    bool    _yaw_corr_seq_init = false;

#if 0
// ============================================================
//  ここから下: 地上局ミッション (2026-09-17 コメントアウト)
//    離陸 / 巡航 / 着陸 / 定型機動 (CIRCLE/FIGURE8/CLIMB_TURN) / 位置・ヨー補正。
//    position_estimator の core/program.py (本番プログラム) がこの経路で飛ぶ。
//    戻すときは上の update()/disengage() とメンバをこちらに差し替え、
//    drone_s5.cpp の updateGuided()/updateFlowHold()/updateControl() も戻すこと。
// ============================================================
public:
    // ------------------------------------------------------------
    //  100Hz。地上局の要求を目標速度/目標高度へ翻訳する。
    //    rx      : 上りコマンド受信器 (最新フレームと鮮度)
    //    poshold : 位置補正 / 原点合わせの適用先
    //    heading : カメラ絶対ヨーでの再基準の適用先
    // ------------------------------------------------------------
    void update(const Inputs& in, const S5C::Rx& rx, PositionHold& poshold, HeadingHold& heading) {
        if (!GUIDED_ENABLE) { _engaged = false; _gp = GP_OFF; return; }
        const uint32_t now = in.now_ms;
        _yaw_est_deg = in.yaw_est_deg;   // 機動の周回判定 (Maneuver::update) が使う

        // --- 0) 入る資格があるか (毎回全部見る) ----------------------
        if (!in.armed) { disengage("ディスアーム"); _landed_latch = false; return; }
        if (_landed_latch && in.sw_hover_up && !_sw_hover_was_up) {
            _landed_latch = false;      // 下げてから上げ直した = 次の便
            Serial.println("\n>>> 着陸ラッチ解除 (SW_HOVER 上げ直し)");
        }
        _sw_hover_was_up = in.sw_hover_up;
        if (_landed_latch) {
            // ★ 自動着陸完了のラッチ。GP_LANDED は disengage() で GP_OFF に消えるため、
            //   着陸後に地上でフローが「死んだ」判定になった瞬間に 解除→再エンゲージ→
            //   高度ホールド再起動→モーターが回って跳ねる事故があった (2026-09-16 00:33)。
            //   ディスアーム、または SW_HOVER を一度下げてから上げ直すまで解けない。
            _gp = GP_LANDED;
            _vx = _vy = 0.0f;
            return;
        }
        if (!in.sbus_ok)      { disengage("SBUS 無効");           return; }
        if (!in.sw_hover_up)  { disengage("SW_HOVER が下");       return; }
        if (!in.flow_alive)   { disengage("フローが死んでいる");   return; }
        if (!in.range_ok)     { disengage("測距が無い");           return; }

        // パイロットがスティックを触ったら自動を降りる (スイッチを探さずに戻せる)
        if (fabsf(in.stick_roll) > FLOW_STICK_DEAD || fabsf(in.stick_pitch) > FLOW_STICK_DEAD) {
            disengage("スティック操作を検出");
            return;
        }
        // 通常の GUIDED はヨースティックを生かしたまま (機首だけ手動で振れる)。
        // 定型機動中はヨー経路をこちらが握っているので、触れたら「回しすぎ・変な方向」
        // への意思表示とみなして即座に降りる。
        if (_gp == GP_MANEUVER && fabsf(in.stick_yaw) > Gain::YAW_STICK_DEAD) {
            disengage("機動中にヨースティック操作を検出");
            return;
        }

        // 着陸完了は、ディスアームするまで保持する (上の !armed で解ける)
        if (_gp == GP_LANDED) { _vx = _vy = 0.0f; return; }

        const S5C::CmdFrame& c   = rx.last();
        const uint32_t       age = rx.ageMs();
        const bool fresh_hold = (age < GUIDED_STALE_HOLD_MS);
        const bool fresh_land = (age < GUIDED_STALE_LAND_MS);

        // --- 1) 初回エンゲージ -----------------------------------------
        //   「新鮮な、意味のある指令」が1つ届くまでは入らない。地上局が
        //   起動していないのに SW_HOVER を上げてしまっても何も起きない。
        if (!_engaged) {
            if (!fresh_hold) return;
            if (c.req == S5C::REQ_IDLE || c.req == S5C::REQ_ABORT) return;
            _engaged = true;
            _why     = "";
            // 入った瞬間は必ずホールドから。高度目標は「今の高度」。
            _gp    = GP_HOLD;
            _alt_m = (in.range_valid && in.range_h_m > 0.05f) ? in.range_h_m : ALT_TARGET_M;
            _slew  = GUIDED_CRUISE_SLEW_MPS;
            _vx = _vy = 0.0f;
            _touch_since_ms = 0;
            Serial.printf("\n>>> GUIDED 開始 (目標高度 %.2f m から保持)\n", _alt_m);
        }

        if (_gp == GP_MANEUVER) {
            // --- 2') 定型機動中はリンクの新鮮さでフォールバックしない ------
            //   「ウェイポイントまでは地上局、そこから先は機体単独」の要。開始した
            //   瞬間の目標を Maneuver が持ったまま、新しいコマンドを待たずに進む。
            //   地上局が明示的に HOLD/ABORT/LAND を送ってくれば、新鮮な間だけ即反映。
            //   進行は Maneuver が「実測ヨーの積分」で数える (2026-09-17。機体が実際に 360°
//   回るまで脚を終えないので、立ち上がりや揺れで円が閉じきる前に完了扱いになる
//   のを防ぐ。回れないときのために脚ごとの時間上限あり。Maneuver.h 参照)。
            _maneuver.update(now, _yaw_est_deg);
            _vx    = _maneuver.fwd();
            _vy    = 0.0f;
            _alt_m = _maneuver.altTarget();   // CLIMB_TURN は進行に合わせて動く

            if (_maneuver.done()) {
                // 完了。同じ REQ が来続けても再開しないようラッチしてから HOLD へ。
                _maneuver_done_req = c.req;
                Serial.printf("\n>>> 機動完了 %s (%.0f deg) -> ホールドへ復帰 (高度 %.2f m)\n",
                              _maneuver.name(), _maneuver.totalDeg(), _alt_m);
                _maneuver.abort();
                _gp = GP_HOLD;
                _vx = _vy = 0.0f;
            } else if (fresh_hold && (c.req == S5C::REQ_HOLD || c.req == S5C::REQ_ABORT
                                      || c.req == S5C::REQ_LAND)) {
                applyCommand(c, now, in, poshold, heading);
            }
        } else if (!fresh_land) {
            // --- 2) リンク断のフェイルセーフ (要求より先に見る) ---------------
            if (_gp != GP_LAND) {
                Serial.printf("\n!! 地上局リンク断 %lu ms -> 自動着陸\n", (unsigned long)age);
                _gp             = GP_LAND;
                _land_start_ms  = now;
                _touch_since_ms = 0;
            }
        } else if (!fresh_hold) {
            // 瞬断。水平だけ止めて、高度目標はそのまま保持する。
            _vx = _vy = 0.0f;
            if (_gp == GP_CRUISE) _gp = GP_HOLD;
        } else {
            // --- 3) 新鮮な指令に従う -------------------------------------
            applyCommand(c, now, in, poshold, heading);
        }

        // --- 4) 着陸フェーズの面倒を見る --------------------------------
        if (_gp == GP_LAND) {
            _vx = _vy = 0.0f;                    // 降りる間は必ず水平ホールド
            _alt_m = GUIDED_LAND_FLOOR_M;
            _slew  = GUIDED_LAND_SLEW_MPS;

            // 接地判定: 規定高度を下回った状態が続いたら着いたとみなす。
            //  一瞬の測距の化けで切らないよう、必ず継続時間を見る。
            if (in.range_valid && in.range_h_m > 0.0f && in.range_h_m < GUIDED_LAND_TOUCH_M) {
                if (_touch_since_ms == 0) _touch_since_ms = now;
            } else {
                _touch_since_ms = 0;
            }
            const bool touched = (_touch_since_ms != 0) &&
                                 (now - _touch_since_ms >= GUIDED_LAND_TOUCH_MS);
            const bool timeout = (_land_start_ms != 0) &&
                                 (now - _land_start_ms >= GUIDED_LAND_TIMEOUT_MS);
            if (touched || timeout) {
                _gp = GP_LANDED;
                _landed_latch = true;
                _vx = _vy = 0.0f;
                Serial.printf("\n>>> 着陸完了 (%s)。出力を切りました。"
                              "THR_CUT でディスアームしてください\n",
                              touched ? "接地検知" : "タイムアウト");
            }
        }
    }

    // resetControllers() から。次に届いたパケットで即座にヨーを再基準する。
    void resetYawCorrSeq() { _yaw_corr_seq_init = false; }

private:
    void disengage(const char* why) {
        if (_engaged) {
            _engaged = false;
            _why     = why;
            Serial.printf("\n>>> GUIDED 解除: %s\n", why);
        }
        _gp    = GP_OFF;
        _vx = _vy = 0.0f;
        _alt_m = 0.0f;
        _slew  = 0.0f;
        _maneuver.abort();
        _maneuver_done_req = 0xFF;
        _touch_since_ms = 0;
    }

    void applyCommand(const S5C::CmdFrame& c, uint32_t now, const Inputs& in,
                      PositionHold& poshold, HeadingHold& heading) {
        if (c.flags & S5C::CF_POS_CORR) {
            const float n = (float)c.corr_n_mm / S5C::SC_MM;
            const float e = (float)c.corr_e_mm / S5C::SC_MM;
            if (c.flags & S5C::CF_POS_SHIFT) {
                // 原点合わせ (hold も一緒に動くので機体は動かない)。以降フェンス有効。
                if (!poshold.frameOk())
                    Serial.printf("\n>>> フレーム原点合わせ: pos=(%.2f, %.2f) -> フェンス有効 "
                                  "(N±%.1f E±%.1f)\n", n, e, FENCE_N_LIM, FENCE_E_LIM);
                poshold.shiftFrame(n, e);
            } else {
                poshold.correctPosition(n, e);
            }
        }

        // ---- ヨー補正: カメラが実測した絶対ヨーで推定値を再基準する ----
        //  CF_YAW_VALID は「カメラのヨー推定が収束している」間ずっと立っているので、
        //  同じ値を何ループも受け続ける。毎回適用するとジャイロの高速積分を無線の
        //  低いレートで踏みつぶすので、c.seq が進んだ = 新しいパケットの回だけ適用。
        //  推定値だけを書き換え、目標 hold には触らない (HeadingHold.h 冒頭)。
        if ((c.flags & S5C::CF_YAW_VALID) &&
            (!_yaw_corr_seq_init || c.seq != _yaw_corr_last_seq)) {
            _yaw_corr_seq_init = true;
            _yaw_corr_last_seq = c.seq;
            heading.rebase((float)c.yaw_abs_cdeg / S5C::SC_CDEG);
        }

        const float cmd_alt = (c.alt_cm > 0) ? (float)c.alt_cm / S5C::SC_CM : 0.0f;
        switch (c.req) {
            case S5C::REQ_TAKEOFF:
                _gp = GP_TAKEOFF;
                _vx = _vy = 0.0f;
                if (cmd_alt > 0.0f) _alt_m = cmd_alt;
                _slew = GUIDED_TAKEOFF_SLEW_MPS;
                break;

            case S5C::REQ_GUIDED:
                _gp = GP_CRUISE;
                _vx = constrain((float)c.vx_mmps / S5C::SC_MMPS, -GUIDED_MAX_VEL, GUIDED_MAX_VEL);
                _vy = constrain((float)c.vy_mmps / S5C::SC_MMPS, -GUIDED_MAX_VEL, GUIDED_MAX_VEL);
                if (cmd_alt > 0.0f) _alt_m = cmd_alt;
                _slew = GUIDED_CRUISE_SLEW_MPS;
                break;

            case S5C::REQ_LAND:
                if (_gp != GP_LAND) {
                    _gp = GP_LAND;
                    _land_start_ms  = now;
                    _touch_since_ms = 0;
                    Serial.println("\n>>> GUIDED 自動着陸を開始");
                }
                break;

            // 定型機動 (水平旋回 / 8の字 / 上昇旋回)。開始は1回だけ:
            //   ・すでに実行中なら進行はリセットしない (無線の再送に耐える)
            //   ・直前に完了/中断したのと同じ REQ が来続けていても再開しない
            //     (_maneuver_done_req)。別の REQ が1回届けば解除。
            case S5C::REQ_CIRCLE:
            case S5C::REQ_FIGURE8:
            case S5C::REQ_CLIMB_TURN: {
                if (c.req == _maneuver_done_req) break;
                if (_gp != GP_MANEUVER) {
                    const Maneuver::Kind kind =
                          (c.req == S5C::REQ_CIRCLE)  ? Maneuver::CIRCLE
                        : (c.req == S5C::REQ_FIGURE8) ? Maneuver::FIGURE8
                                                      : Maneuver::CLIMB_TURN;
                    const float fwd  = constrain((float)c.vx_mmps / S5C::SC_MMPS,
                                                 -GUIDED_MAX_VEL, GUIDED_MAX_VEL);
                    const float rate = constrain((float)c.yaw_rate_cdps / S5C::SC_CDPS,
                                                 -MANEUVER_MAX_YAW_RATE_DPS, +MANEUVER_MAX_YAW_RATE_DPS);
                    const float alt_now = (in.range_valid && in.range_h_m > 0.05f) ? in.range_h_m : _alt_m;
                    _maneuver.begin(kind, fwd, rate, alt_now, cmd_alt, (int)c.laps, now);
                    _gp = GP_MANEUVER;
                    Serial.printf("\n>>> 機動開始 %s: 前進 %.2f m/s, ヨーレート %+.1f deg/s "
                                  "(半径 %.2f m), %d 脚 (%.0f deg), 高度 %.2f -> %.2f m "
                                  "(機体センサのみで完結)\n",
                                  _maneuver.name(), fwd, rate,
                                  (fabsf(rate) > 0.1f) ? fwd / (fabsf(rate) * DEG_TO_RAD) : 0.0f,
                                  _maneuver.legs(), _maneuver.totalDeg(), alt_now, cmd_alt);
                }
                _vx = _maneuver.fwd();
                _vy = 0.0f;                     // 機動中は前進のみ。横方向は使わない
                _alt_m = _maneuver.altTarget();
                _slew  = GUIDED_CRUISE_SLEW_MPS;
                break;
            }

            case S5C::REQ_HOLD:
            case S5C::REQ_ABORT:
            case S5C::REQ_IDLE:
            default:
                if (_gp == GP_MANEUVER)
                    Serial.printf("\n>>> 機動中断 (%s) by %s\n", _maneuver.name(), S5C::reqName(c.req));
                if (_gp == GP_CRUISE || _gp == GP_TAKEOFF || _gp == GP_MANEUVER) _gp = GP_HOLD;
                _maneuver.abort();
                _maneuver_done_req = 0xFF;      // 別の REQ が来た = 次の機動を受け付けてよい
                _vx = _vy = 0.0f;
                _slew = GUIDED_CRUISE_SLEW_MPS;
                break;
        }
    }

    // ---- 地上局ミッションの状態 ----
    bool  _landed_latch = false;    // 着陸完了ラッチ (ディスアーム or SW_HOVER 上げ直しで解除)
    bool  _sw_hover_was_up = false;
    Maneuver _maneuver;
    uint8_t  _maneuver_done_req = 0xFF;   // 直前に完了/中断した REQ。0xFF = なし

    uint32_t _land_start_ms  = 0;   // 降下を始めた時刻 (タイムアウト用)
    uint32_t _touch_since_ms = 0;   // 接地高度を下回り続けている開始時刻 (0=未満たず)

    bool    _yaw_corr_seq_init = false;
    uint8_t _yaw_corr_last_seq = 0;
    float   _yaw_est_deg = 0.0f;   // 直近の実測ヨー (update で control 側から受ける)
#endif  // 地上局ミッション (コメントアウト)
};

} // namespace Quad
