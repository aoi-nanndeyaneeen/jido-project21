// ============================================================
//  Maneuver.h  -  機体単独で完結する定型機動 (水平旋回 / 8の字 / 上昇旋回)
// ============================================================
//  地上局からは「開始」の1発 (S5Cmd の REQ_CIRCLE / REQ_FIGURE8 /
//  REQ_CLIMB_TURN) だけを受け、以降はリンクの新鮮さに関係なく
//  機体の時計だけで進める。構造は全部同じ:
//
//      前進速度 (機体座標 +x) 一定  +  ヨーレート一定  →  地面座標で円になる
//      半径 r = v / ω     (v=0.4m/s, ω=15deg/s で r≈1.53m)
//
//  地面座標への変換は一切しない (S5Cmd.h 冒頭の注記)。機動は「脚 (leg)」の
//  列で表す。1 脚 = 360°。脚ごとに 回転方向 と 高度 (開始→終了) を持つ。
//
//    CIRCLE(laps)     : [+ , alt→alt] × laps
//    FIGURE8          : [+ , alt→alt], [− , alt→alt]           (接する2円 = 8)
//    CLIMB_TURN(laps) : [+ , low→low] × laps,
//                       [+ , low→high]                          (回りながら上昇)
//                       [+ , high→high] × laps
//
//  ルールブック (2026) との対応:
//    水平旋回  半径 1.5m 以上・1周 400点・連続2周 1000点  → CIRCLE laps=2
//    8の字     1周 → 逆回り1周、半径同じ                    → FIGURE8
//    上昇旋回  3m 以下で2周 → ポール以上へ上昇 → ポール以上で2周 → CLIMB laps=2
//
//  進行は「指令したヨーレート × 経過時間」で数える (実測ジャイロではない)。
//  ・無線が完全に止まっても進み続けられる
//  ・レートループが追従していれば実測と大差ない (2026-09-16 実測: 目標30deg/s
//    に対し yaw_gyr 29〜31deg/s、1周指令で実測 352〜356deg)
//  ・実測で数えると、外乱で振られたぶんまで「進んだ」ことになり、円が閉じない
//
//  ★ このクラスは制御を持たない。出力 (fwd/yawRate/altTarget) を PosHold /
//    ヨーのレート指令 / AltHold に渡すのは drone_s5.cpp。中断判定
//    (スティック・スイッチ・地上局の HOLD) も drone_s5.cpp 側。
// ============================================================
#pragma once
#include <Arduino.h>
#include <math.h>

namespace Quad {

class Maneuver {
public:
    enum Kind : uint8_t { NONE = 0, CIRCLE, FIGURE8, CLIMB_TURN };

    static constexpr int MAX_LEGS = 8;   // CLIMB laps=3 でも 3+1+3=7
    static constexpr int DEFAULT_CIRCLE_LAPS = 1;
    static constexpr int DEFAULT_CLIMB_LAPS  = 2;

    static const char* kindName(Kind k) {
        switch (k) {
            case CIRCLE:     return "CIRCLE";
            case FIGURE8:    return "FIGURE8";
            case CLIMB_TURN: return "CLIMB";
            default:         return "-";
        }
    }

    // 開始。yaw_rate_dps の符号が最初の旋回方向 (右旋回 +)。
    //   alt_now_m    : 今の対地高度 (保持高度 / CLIMB の低高度)
    //   alt_target_m : CLIMB_TURN の到達高度。他の種類では alt_now の代わりに
    //                  保持する高度 (0 以下なら alt_now)
    //   laps         : 周回数 (0 = 既定)。FIGURE8 では無視
    void begin(Kind kind, float fwd_mps, float yaw_rate_dps,
               float alt_now_m, float alt_target_m, int laps, uint32_t now_ms) {
        _kind     = kind;
        _fwd      = fwd_mps;
        _rate_abs = fabsf(yaw_rate_dps);
        const float s0 = (yaw_rate_dps < 0.0f) ? -1.0f : 1.0f;
        _n_legs = 0;
        _leg    = 0;
        _turned = 0.0f;
        _turned_meas = 0.0f;
        _have_yaw = false;
        _prev_yaw = 0.0f;
        _leg_start_ms = now_ms;
        _prev_ms = now_ms;
        _done    = false;

        const float hold = (kind != CLIMB_TURN && alt_target_m > 0.0f) ? alt_target_m : alt_now_m;
        switch (kind) {
            case CIRCLE: {
                const int n = constrain(laps > 0 ? laps : DEFAULT_CIRCLE_LAPS, 1, MAX_LEGS);
                for (int i = 0; i < n; ++i) addLeg(s0, hold, hold);
                break;
            }
            case FIGURE8:
                addLeg( s0, hold, hold);
                addLeg(-s0, hold, hold);
                break;
            case CLIMB_TURN: {
                const int n = constrain(laps > 0 ? laps : DEFAULT_CLIMB_LAPS, 1, (MAX_LEGS - 1) / 2);
                const float hi = (alt_target_m > 0.0f) ? alt_target_m : alt_now_m;
                for (int i = 0; i < n; ++i) addLeg(s0, alt_now_m, alt_now_m);
                addLeg(s0, alt_now_m, hi);              // 回りながら上昇 (1周ぶん)
                for (int i = 0; i < n; ++i) addLeg(s0, hi, hi);
                break;
            }
            default:
                _kind = NONE;
                break;
        }
    }

    // 毎ループ呼ぶ。全脚を消化したら done() が立つ。
    //   yaw_meas_deg : 機体の実測ヨー (HeadingHold::est()。アーム基準の相対方位)。
    //
    // ★ 2026-09-17: 周回の判定を「指令レート × 経過時間」から「実測ヨーの積分」に
    //   変えた。旧方式は _rate_abs*dt を積算していたので、機体が指令レートに追いつく
    //   前 (立ち上がり) や外乱で遅れたぶんを数えず、**円が閉じきる前に完了扱い**に
    //   なっていた (2026-09-16: 水平旋回2周で指令720°に対し実測609°=1.69周。判定不成立)。
    //   実測ヨーの絶対増分を積むことで、機体が実際に 360° 回るまで脚を終えない。
    //
    // ★ 「実測で数えると外乱で振られたぶんも進んだ扱いになり円が閉じない」という旧
    //   コメントの懸念は、意図した一定レートの旋回では deliberate 成分が支配的なので
    //   小さい。暴走 (回れないのに永久に続く) を防ぐため、脚ごとに時間上限を持たせる:
    //   その脚の想定時間 (360/_rate_abs) の LEG_TIME_CAP 倍を超えたら、実測が 360 に
    //   届いていなくても打ち切って次の脚へ進む。
    void update(uint32_t now_ms, float yaw_meas_deg) {
        if (!active()) return;
        const float dt = (now_ms - _prev_ms) * 1e-3f;
        _prev_ms = now_ms;

        _turned += _rate_abs * dt;              // 指令基準 (表示 progressDeg / doneDeg 用に残す)

        // 実測ヨーの絶対増分を積む (±180 で折り返しを畳む。8の字の反転も abs で吸収)
        if (_have_yaw) {
            float d = yaw_meas_deg - _prev_yaw;
            while (d >  180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            _turned_meas += fabsf(d);
        }
        _prev_yaw = yaw_meas_deg;
        _have_yaw = true;

        const float leg_s   = (_rate_abs > 1.0f) ? 360.0f / _rate_abs : 1e9f;
        const bool  closed  = _turned_meas >= 360.0f;                       // 実際に1周した
        const bool  timeout = (now_ms - _leg_start_ms) * 1e-3f >= leg_s * LEG_TIME_CAP;
        if (closed || timeout) {
            _leg++;
            _turned      -= 360.0f;             // 端数を次の脚へ持ち越す (指令基準)
            _turned_meas  = (closed && _turned_meas > 360.0f) ? _turned_meas - 360.0f : 0.0f;
            _leg_start_ms = now_ms;
            if (_leg >= _n_legs) _done = true;
        }
    }

    void abort() { _kind = NONE; _done = false; _n_legs = 0; }

    bool  active()  const { return _kind != NONE && !_done; }
    bool  done()    const { return _kind != NONE && _done; }
    Kind  kind()    const { return _kind; }
    const char* name() const { return kindName(_kind); }

    // ---- 出力 (drone_s5.cpp が毎ループ読む) ----
    float fwd()     const { return active() ? _fwd : 0.0f; }                 // 機体座標 前+ [m/s]
    float yawRate() const { return active() ? cur().sign * _rate_abs : 0.0f; } // [deg/s]
    // 目標高度 [m]。上昇脚では進行に比例して開始→終了へ直線で動く。
    //  ★ 進行の割合は実測ヨー基準 (_turned_meas)。機体が物理的に回った分だけ高度を
    //    動かす (指令基準だと、機体がまだ回っていないのに高度だけ先に上がってしまう)。
    float altTarget() const {
        if (_n_legs == 0) return 0.0f;
        const Leg& L = _done ? _legs[_n_legs - 1] : cur();
        if (_done) return L.alt1;
        const float f = constrain(_turned_meas / 360.0f, 0.0f, 1.0f);
        return L.alt0 + (L.alt1 - L.alt0) * f;
    }

    // ---- 表示 / テレメトリ ----
    float progressDeg() const { return _turned_meas; }        // 今の脚で実測した回転 [deg]
    int   leg()         const { return _leg; }
    int   legs()        const { return _n_legs; }
    float totalDeg()    const { return 360.0f * _n_legs; }
    float doneDeg()     const { return 360.0f * _leg + _turned_meas; }   // 実測基準の総進行
    bool  climbing()    const { return active() && cur().alt0 != cur().alt1; }

private:
    // 脚ごとの時間上限 = 想定時間 (360/rate) の何倍まで許すか。回れないのに永久に
    // 回り続けるのを防ぐ安全弁。1.6 なら想定 12s の脚を最大 19s で打ち切る。
    static constexpr float LEG_TIME_CAP = 1.6f;

    struct Leg { float sign; float alt0; float alt1; };

    void addLeg(float sign, float a0, float a1) {
        if (_n_legs < MAX_LEGS) _legs[_n_legs++] = Leg{sign, a0, a1};
    }
    const Leg& cur() const { return _legs[constrain(_leg, 0, _n_legs - 1)]; }

    Kind     _kind     = NONE;
    bool     _done     = false;
    float    _fwd      = 0.0f;
    float    _rate_abs = 0.0f;
    Leg      _legs[MAX_LEGS] = {};
    int      _n_legs   = 0;
    int      _leg      = 0;
    float    _turned   = 0.0f;      // 指令基準の積算 [deg] (表示・端数持ち越し用)
    float    _turned_meas = 0.0f;   // 実測ヨーの積算 [deg] (完了判定・高度補間はこちら)
    bool     _have_yaw = false;
    float    _prev_yaw = 0.0f;
    uint32_t _leg_start_ms = 0;     // 今の脚の開始時刻 (時間上限用)
    uint32_t _prev_ms  = 0;
};

} // namespace Quad
