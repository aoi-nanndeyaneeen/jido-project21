// ============================================================
//  Guided.h  -  地上局ガイド飛行 (GUIDED) の状態機械
// ============================================================
//  drone_s5.cpp § 6-4 にあった updateGuided() / applyGuidedCommand() /
//  guidedDisengage() と、その周りの g_gp / g_guided_* / g_landed_latch /
//  maneuver / 着陸タイマ を 1 クラスにまとめたもの。
//
//  ここがやるのは **翻訳だけ**:
//      地上局の要求 (S5C::CmdFrame)  ->  vx()/vy()   (PosHold の目標速度へ)
//                                        altM()/slew() (AltHold の目標高度へ)
//                                        maneuver().yawRate() (ヨーのレート指令へ)
//  制御そのものは一切しない。GUIDED 中に通る制御経路は POSHOLD と完全に
//  同一で、違うのは「目標速度と目標高度を誰が決めるか」だけ。
//
//  ★ 安全の骨組み (ここを崩さないこと)
//    1. スロットルスティックは常にパイロットのもの。GUIDED でも
//       FLOW_ENABLE_THR / ALT_ENABLE_THR (15%) を下回れば全部手放す。
//    2. SW_HOVER を下げれば ANGLE (完全手動)。これが最終の bail-out。
//    3. SW_HOVER を cen に戻せば POSHOLD (その場ホールド)。地上局だけ切れる。
//    4. ロール/ピッチスティックを動かせば GUIDED から自動で抜ける。
//    5. リンクが切れたら 1秒でその場ホールド、4秒で自動着陸。
//       指令が消えて暴走する経路は存在しない (指令 0 = ホールド)。
//
//  呼び出しは S5::GUIDED_HZ (100Hz)。中身は millis() の比較と代入だけなので
//  1000Hz で回す必要は無い。上りコマンドは別に 200Hz でポーリングしている。
//
//  パケット定義と設計方針は protocol/S5Cmd.h、定数は QuadConfig.h § 9。
// ============================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include "quad/QuadConfig.h"
#include "quad/PosHold.h"
#include "quad/HeadingHold.h"
#include "quad/Maneuver.h"
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
};

inline const char* guidedPhaseName(GuidedPhase p) {
    static const char* const N[] = { "OFF", "HOLD", "TAKEOFF", "CRUISE", "LAND", "LANDED", "MANEUV" };
    return N[(int)p];
}

class Guided {
public:
    // update() が毎回見る「機体側の事実」。drone_s5.cpp が詰めて渡す。
    struct Inputs {
        uint32_t now_ms;
        bool  armed;
        bool  sbus_ok;        // S5::USE_SBUS
        bool  sw_hover_up;    // SW_HOVER == up (GUIDED の入り口)
        bool  flow_alive;     // フローが使える (初期化済み && 空中で凍結していない)
        bool  range_ok;       // 測距センサが初期化できている
        bool  range_valid;    // 今この瞬間の測距が信用できる
        float range_h_m;      // 鉛直対地高度 [m]
        float stick_roll;     // sbus.des 生値 (符号適用前)
        float stick_pitch;
        float stick_yaw;
        float yaw_est_deg;    // HeadingHold::est() (機体の実測ヨー。機動の周回判定に使う)
    };

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

    // --- 出力 (drone_s5 が PosHold / AltHold / ヨー へ渡す) -----------
    bool        engaged()     const { return _engaged; }
    GuidedPhase phase()       const { return _gp; }
    bool        landed()      const { return _gp == GP_LANDED; }
    bool        inManeuver()  const { return _gp == GP_MANEUVER; }
    float       vx()          const { return _vx; }      // 機体座標 前+ [m/s]
    float       vy()          const { return _vy; }      // 同 右+ [m/s]
    float       altM()        const { return _alt_m; }   // 目標対地高度 [m] (0 = 指令なし)
    float       slew()        const { return _slew; }    // 目標高度を動かしてよい速さ [m/s]
    const char* why()         const { return _why; }     // 直前に GUIDED を抜けた理由
    const Maneuver& maneuver() const { return _maneuver; }

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

    GuidedPhase _gp = GP_OFF;
    bool  _engaged = false;         // GUIDED に入っているか (selectMode が見る)
    bool  _landed_latch = false;    // 着陸完了ラッチ (ディスアーム or SW_HOVER 上げ直しで解除)
    bool  _sw_hover_was_up = false;
    float _vx = 0.0f, _vy = 0.0f;   // 機体座標の目標速度 [m/s]
    float _alt_m = 0.0f;            // 目標対地高度 [m] (0 = 指令なし)
    float _slew  = 0.0f;            // 目標高度を動かしてよい速さ [m/s]
    const char* _why = "";

    Maneuver _maneuver;
    uint8_t  _maneuver_done_req = 0xFF;   // 直前に完了/中断した REQ。0xFF = なし

    uint32_t _land_start_ms  = 0;   // 降下を始めた時刻 (タイムアウト用)
    uint32_t _touch_since_ms = 0;   // 接地高度を下回り続けている開始時刻 (0=未満たず)

    bool    _yaw_corr_seq_init = false;
    uint8_t _yaw_corr_last_seq = 0;
    float   _yaw_est_deg = 0.0f;   // 直近の実測ヨー (update で control 側から受ける)
};

} // namespace Quad
