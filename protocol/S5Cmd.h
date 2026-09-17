// ============================================================
//  S5Cmd.h  -  地上局 -> 機体 の上りコマンド (IM920SL / BLE)
// ============================================================
//  ★ 2026-09-17: 地上局リンクを BLE でも運べるようにした (flight_controller
//    quad/S5Features.h の GROUND_LINK)。CmdFrame の中身は同じで、運び方だけが
//    「IM920 の TXDA 16進行」か「LogLink の T_CMD フレーム」(quad/LogLinkProto.h)
//    かが違う。受信は Rx::poll() (IM920) か Rx::acceptRaw() (BLE) のどちらか
//    一方だけを使う。統計・重複排除は共通。下の【帯域】は IM920 のときの話。
//  ★ このファイルはリポジトリ直下の protocol/ にあり、flight_controller と
//    ground_receiver の両方が platformio.ini の -I ../protocol で同じ実体を
//    読む (以前は両プロジェクトに手コピーしていた)。Python 側の定数は
//      python protocol/gen_py_protocol.py
//    で position_estimator/src/core/s5_protocol.py に生成する。
//
// ------------------------------------------------------------
//  【なぜ新しいパケットを作るのか】
//
//  従来の上りは Config.h の GroundData (42 + チェックサム4 = 46 バイト) で、
//  IM920sL の実効ペイロード 32 バイトを超えている。モジュールは OK を返す
//  のに受信側には先頭 32 バイトしか届かないので、ap_roll 以降 (オフセット
//  26 バイト目〜) は **一度も機体に届いたことがない**。
//  地上局オートパイロットを動かすには、32 バイトに収まる専用パケットが要る。
//
// ------------------------------------------------------------
//  【設計方針】  ★ ここが一番大事
//
//  地上局から「スティック相当の角度指令」を送ってはいけない。
//    ・IM920sL は半二重 19200bps、実効 15Hz、往復の遅れは 100〜200ms ある
//    ・姿勢ループは 1000Hz。そこへ 15Hz・遅れ 200ms の指令を入れると、
//      位相遅れがそのまま位相余裕を食って発振する
//    ・リンクが切れた瞬間に「最後の傾け指令」が残り、機体は飛んでいく
//
//  そこで送るのは **設定値 (setpoint) だけ** にする。
//    ・水平: 機体座標の目標速度 [mm/s] (前 +, 右 +)
//    ・垂直: 対地の目標高度 [cm]
//    ・状態: 離陸 / 巡航 / 着陸 / ホールド の要求
//
//  内側 (速度ループ・高度ループ・姿勢ループ) は今まで通り全部機体側。
//  つまり地上局は PosHold のスティック入力と AltHold の目標高度を
//  差し替えているだけで、飛行中の制御経路は POSHOLD と完全に同じ。
//  リンクが切れたら目標速度を 0 にするだけで「その場ホールド」に戻る。
//  これが安全側に倒れる唯一の構造。
//
//  ★ 速度指令が「機体座標」なのは、機体が絶対方位を持たないから
//    (6軸IMU。機体のヨー (HeadingHold) はアーム時を 0 とした相対方位)。地上局は
//    カメラでヘディングを推定しているので (position_estimator の
//    yaw_estimator)、地面座標 -> 機体座標の回転は地上局側でやる。
//    ※ そのぶんヘディング推定を外すと指令の向きも外れる。ウェイポイント
//      飛行中は機首を回さない (ヨー指令 0) 運用にすること。
//
//  ★ REQ_CIRCLE / REQ_FIGURE8 / REQ_CLIMB_TURN (定型機動) はこの前提の例外。地面座標へは一切変換
//    しない (機体座標のまま前進+回転を保ち続けるだけで、地面座標での
//    円軌道は勝手にできる)。だからカメラのヘディング推定にもリンクの
//    新鮮さにも依存せず、機体単独 (ジャイロ+オプティカルフロー) で完結
//    させてよい。詳細は flight_controller/include/quad/Guided.h の GP_MANEUVER 参照。
//
// ------------------------------------------------------------
//  【帯域】  ★ 2026-09-16 現在の割り当て
//    1 パケット 22+4 = 26 バイト = "TXDA " + 52桁 + CRLF = 59 文字 =
//    19200bps で 30.7ms。上り 8Hz で UART 占有率 25%。
//    下り (S5Telem) は 8Hz x 71文字 37ms = 30%。合計 53%。
//
//    レートを決めているのは 2 箇所だけ:
//      上り … ground_receiver/src/main.cpp (GroundStation) の CMD_MIN_GAP_MS
//      下り … flight_controller/src/drone_s5.cpp の TELEM_TX_HZ
//    IM920 は半二重 (送信中は受信できない) なので、片方だけ上げると
//    もう片方が落ちる。必ず 2 つセットで、合計 55% 程度を上限に見ること。
//
//    ★ 旧: 下り 12Hz (44%) + 上り 5Hz (14%) = 58%。この配分では下りの
//      実測が 10.5Hz しか通っておらず (12% 欠落)、しかも B(水平位置) は
//      5枠巡回の1枠しか無かったので実測 1.5Hz。PC が「自分の指令が効いた
//      か」を見る枠がそこだったため、体感の遅れの主犯は上りではなく
//      **下りの観測遅れ** だった。帯域を上りへ付け替え、枠割りも
//      A,B,A,B,C,A,B,D に変えてある (quad/S5Telemetry.h の tick())。
// ============================================================
#pragma once
#include <Arduino.h>
#include "Im920Frame.h"

namespace S5C {

// 構造体を変えたら必ずインクリメントすること (相手側が不一致を検出する)
// VERSION 4 (2026-09-14): action/action_seq を削除 (IM920は操縦専用に戻し、
//   単発メンテナンス指令はBLE経由に限定したため。下の Action 列挙子の
//   コメント参照)。
constexpr uint8_t VERSION = 9;  // laps (定型機動の周回数) added

// 下りテレメトリ (S5T) の type と衝突しない値。'K' = command
constexpr uint8_t MAGIC = 0x4B;

constexpr size_t IM920SL_MAX_PAYLOAD = IM920::MAX_PAYLOAD;      // 32
constexpr size_t CHECKSUM_BYTES      = IM920::CHECKSUM_BYTES;   // 4

// ------------------------------------------------------------
//  要求する飛行フェーズ
//    機体側はこれを「お願い」として受け取るだけで、実際に入れるかは
//    機体側のゲート (アーム済み / スイッチ位置 / センサ健全) が決める。
// ------------------------------------------------------------
enum Req : uint8_t {
    REQ_IDLE    = 0,  // 何もしない (地上局は生きているが指令なし)
    REQ_HOLD    = 1,  // その場ホールド (水平速度 0、高度は現状維持)
    REQ_TAKEOFF = 2,  // alt_cm まで自動上昇。水平は 0 固定
    REQ_GUIDED  = 3,  // 巡航: vx/vy と alt_cm に従う
    REQ_LAND    = 4,  // 自動着陸。水平は 0 固定
    REQ_ABORT   = 5,  // 中断: 即 HOLD して地上局の指令を捨てる
    // ★ 2026-09-16: 自動水平旋回。GUIDED と違い「一度届けば機体単独で
    //   完結する」設計 (下の注記参照)。REQ_FIGURE8 / REQ_CLIMB_TURN も
    //   同じ形 (vx_mmps=巡航速度, yaw_rate_cdps=旋回レート, alt_cm=高度)
    //   で足す予定なので、値を詰めて並べてある。
    REQ_CIRCLE  = 6,  // 水平旋回1周: vx_mmps で前進しつつ yaw_rate_cdps で
                      // 回り続け、機体が自分で360°を数えて自動的に HOLD へ戻る。
                      // alt_cm は保持する高度。
    REQ_FIGURE8 = 7,  // 8の字: 1周 + 逆回りで1周 (半径は同じ)。
    REQ_CLIMB_TURN = 8, // 上昇旋回: 開始高度で laps 周 → 回りながら alt_cm へ上昇
                      // (1周分) → alt_cm で laps 周。(quad/Maneuver.h)
    // ★ 機体は同じ REQ_* が連続して届いても1回しか始めない (完了後に同じ
    //   要求が来続けても再開しない)。次を始めるには別の REQ (IDLE/HOLD 等)
    //   を1回挟むこと。2026-09-16 の初飛行で、console が 10Hz で REQ_CIRCLE
    //   を送り続けたため完了→即再開を繰り返し 2.4 周回った。
};

inline const char* reqName(uint8_t r) {
    switch (r) {
        case REQ_IDLE:    return "IDLE";
        case REQ_HOLD:    return "HOLD";
        case REQ_TAKEOFF: return "TAKEOFF";
        case REQ_GUIDED:  return "GUIDED";
        case REQ_LAND:    return "LAND";
        case REQ_ABORT:   return "ABORT";
        case REQ_CIRCLE:  return "CIRCLE";
        case REQ_FIGURE8: return "FIGURE8";
        case REQ_CLIMB_TURN: return "CLIMB";
    }
    return "?";
}

// ------------------------------------------------------------
//  単発メンテナンス指令の種類 (Action)
// ------------------------------------------------------------
//  ★ 2026-09-14: 元はここ (IM920 の CmdFrame) に action/action_seq を
//    足して地上局から送っていたが、「IM920は操縦専用にする / PID reset・
//    IMU校正・デバイス確認のようなデバッグ用の単発操作はBLE経由に限定する」
//    という方針にしたため、CmdFrame からは action/action_seq を削除した。
//    今このファイルに残っているのは、値の意味 (Action の列挙子) を
//    BLE 側 (log_recorder/src/main.cpp の Cmd:: / LogLinkProto.h の
//    ActReq / drone_s5.cpp の handleBleAction()) と共有するためだけ。
//    IM920 経由でこれらの値が機体に届くことはもう無い。
// ------------------------------------------------------------
enum Action : uint8_t {
    ACT_NONE      = 0,  // 何もしない (通常の巡航中はずっとこれ)
    ACT_PID_RESET = 1,  // PID内部状態のリセット (シリアル 'r' と同じ)
    ACT_IMU_CAL   = 2,  // IMU再キャリブレーション (シリアル 'k' と同じ)
                        //   ★ 機体側は非アーム中のみ実行する (シリアル
                        //     'k' と違い、遠隔操作者は機体に触れていない
                        //     ため。詳細は drone_s5.cpp のコメント参照)
    ACT_SELFTEST  = 3,  // I2Cバス再走査 + IMU疎通確認 (シリアル 'i' 相当)
                        //   結果は BLE 側 (LogLinkProto.h の ActAck) で返る
};

inline const char* actionName(uint8_t a) {
    switch (a) {
        case ACT_NONE:      return "-";
        case ACT_PID_RESET: return "PID_RESET";
        case ACT_IMU_CAL:   return "IMU_CAL";
        case ACT_SELFTEST:  return "SELFTEST";
    }
    return "?";
}

// ------------------------------------------------------------
//  コマンドパケット (20 byte + チェックサム 4 = 24 byte)
// ------------------------------------------------------------
//  ★ corr_n_mm/corr_e_mm (2026-09-14 追加): 地上局が周期的に送る
//    「機体の自己位置(フロー積分)はここにいるはず」という絶対値の補正。
//    床の模様が薄いフィールドではフローが流れやすく、機体単独の位置推定
//    (PosFrame.pos_n_cm 等) がじわじわ真値からずれる。カメラの絶対位置
//    (誤差 cm オーダー) で定期的に上書きして流れを消す。
//
//    ★★ 速度・姿勢のようにクローズドループへは絶対に混ぜない。
//      IM920sL は往復 100〜200ms 遅れるので、この遅れを含んだ位置を
//      毎ループ (100Hz) 使うと発振する。だから
//        ・数秒に1回だけ (position_estimator/utils/config.py の
//          POS_CORR_PERIOD_S)
//        ・PositionHold._pos_n/_pos_e を書き換えるだけ (quad/PosHold.h)
//        ・GUIDED 巡航中 (stick_active) は hold が pos に毎ループ
//          追従するので実質無効。効くのは静止保持中だけ
//        ・結果の速度は FLOW_POS_VEL_LIM で必ずクランプされる
//      という「たまに位置をこっそり書き換えるだけ」に留める。
struct __attribute__((__packed__)) CmdFrame {
    uint8_t  magic;         //  1  MAGIC
    uint8_t  ver;           //  2  VERSION
    uint8_t  seq;           //  3  連番。機体は重複/逆行を捨てる
    uint8_t  req;           //  4  Req
    int16_t  vx_mmps;       //  6  機体座標の目標速度 前+ [mm/s]
    int16_t  vy_mmps;       //  8  同 右+ [mm/s]
    int16_t  alt_cm;        // 10  目標対地高度 [cm] (0以下 = 現状維持)
    int16_t  yaw_rate_cdps; // 12  目標ヨーレート [0.01 deg/s] (0 = 機首固定)
    uint16_t flags;         // 14  CmdFlag
    int16_t  corr_n_mm;     // 16  位置補正の絶対目標 pos_n [mm]。CF_POS_CORR 時のみ有効
    int16_t  corr_e_mm;     // 18  同 pos_e [mm]
    int16_t  yaw_abs_cdeg;  // 20  カメラ絶対ヨー [0.01 deg]。CF_YAW_VALID 時のみ
    // ★ 2026-09-16: 定型機動の周回数。REQ_CIRCLE = この回数だけ同じ向きに回る
    //   (ルールブック: 連続2周で1000点)。REQ_CLIMB_TURN = 低高度で laps 周 →
    //   上昇しながら1周 → 高高度で laps 周。REQ_FIGURE8 では無視 (常に1+1)。
    //   0 は既定 (CIRCLE=1, CLIMB=2)。他の REQ では無視。
    uint8_t  laps;          // 21
    uint8_t  rsv;           // 22  予約 (0)
};
static_assert(sizeof(CmdFrame) == 22, "CmdFrame は 22 byte");
static_assert(sizeof(CmdFrame) + CHECKSUM_BYTES <= IM920SL_MAX_PAYLOAD,
              "IM920sL の 32 バイト制限を超えています");

enum CmdFlag : uint16_t {
    CF_ARMED_OK  = 1u << 0,  // 地上局が「飛ばしてよい」と思っている
    CF_POS_VALID = 1u << 1,  // カメラの自己位置推定が生きている
    CF_YAW_VALID = 1u << 2,  // ヘディング推定が収束している
    CF_ALT_ABS   = 1u << 3,  // alt_cm を絶対目標として扱う (落ちていれば現状維持)
    CF_POS_CORR  = 1u << 4,  // corr_n_mm/corr_e_mm を pos_n/pos_e へ適用する
    // ★ 2026-09-16: CF_POS_CORR と一緒に立てると「補正」ではなく「原点合わせ」。
    //   pos だけでなく hold も同じ量だけ動かすので機体は動かず、以降の
    //   pos_n/pos_e が地上局の座標系 (フィールド座標) になる。機体側フェンス
    //   (QuadConfig FENCE_*) はこれを受けてから効き始める。PosHold::shiftFrame()。
    CF_POS_SHIFT = 1u << 5,
};

// スケール
constexpr float SC_MMPS = 1000.0f;   // 1 mm/s
constexpr float SC_CM   = 100.0f;    // 1 cm
constexpr float SC_CDPS = 100.0f;    // 0.01 deg/s
constexpr float SC_MM   = 1000.0f;   // 1 mm (位置補正用)
constexpr float SC_CDEG = 100.0f;    // 0.01 deg (カメラ絶対ヨー)

inline int16_t q16(float v, float scale) {
    const float x = v * scale;
    if (x >  32767.0f) return  32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)lroundf(x);
}

// チェックサム (S5Telem.h と同じ方式: 総和の2の補数)
// 行の組み立て / チェックサム / 送信は IM920 層 (Im920Frame.h) に一本化。
using IM920::checksum;
using Tx = IM920::Tx;       // 地上局側の非ブロッキング送信

// ============================================================
//  Rx  -  機体側の非ブロッキング受信
// ============================================================
//  poll() を毎ループ (TELEM_RX_HZ) 呼ぶ。1 回の呼び出しで available() のぶんだけ
//  進め、完全な行が出来たらデコードして last() を更新する。String は使わない
//  (Im920Frame.h 冒頭)。
//
//  ★ 同じ Serial ポートを他の受信処理と二重に読ませないこと。片方が先に
//    バイトを抜くと、もう片方は永久に行を組み立てられない。
// ============================================================
class Rx {
public:
    explicit Rx(HardwareSerial* ser) : _ser(ser) {}

    // 毎ループ呼ぶ。新しいコマンドを取り込んだら true。
    bool poll() {
        bool got = false;
        while (_ser->available()) {
            if (_line.feed((char)_ser->read()) && decodeLine()) got = true;
        }
        return got;
    }

    const CmdFrame& last()  const { return _last; }
    uint32_t lastRxMs()     const { return _last_rx_ms; }
    bool     everReceived() const { return _last_rx_ms != 0; }

    // BLE 経由 (LogLink の T_CMD) で届いた CmdFrame の生バイト列を取り込む。
    //  IM920 の行デコード (16進・4B チェックサム) を通らないだけで、magic/ver/seq
    //  の検査と統計は poll() と同じ (accept())。新しいコマンドなら true。
    //  ★ poll() と同じインスタンスで両方を呼ばないこと (seq が混ざる)。
    bool acceptRaw(const uint8_t* p, size_t n) {
        if (n != sizeof(CmdFrame)) { _bad_len++; return false; }
        _rssi = -1;                      // BLE では機体側に RSSI が無い
        CmdFrame f;
        memcpy(&f, p, sizeof(f));
        return accept(f);
    }

    // max_age_ms 以内に正常なコマンドが届いているか
    bool fresh(uint32_t max_age_ms) const {
        return _last_rx_ms != 0 && (millis() - _last_rx_ms) < max_age_ms;
    }
    uint32_t ageMs() const {
        return _last_rx_ms ? (millis() - _last_rx_ms) : 0xFFFFFFFFu;
    }

    uint32_t nGood()   const { return _n_good;  }
    uint32_t nBadCs()  const { return _bad_cs;  }
    uint32_t nBadLen() const { return _bad_len + _line.overflows(); }
    uint32_t nBadVer() const { return _bad_ver; }
    uint32_t nLost()   const { return _n_lost;  }
    int      rssi()    const { return _rssi;    }

private:
    bool decodeLine() {
        uint8_t buf[sizeof(CmdFrame)];
        const IM920::Decoded d = IM920::decodeLine(_line.line(), buf, sizeof(buf));
        switch (d.st) {
            case IM920::Decode::OK:       break;
            case IM920::Decode::NOT_DATA: return false;        // "OK" / "NG" / 起動メッセージ
            case IM920::Decode::BAD_CS:   _bad_cs++;  return false;
            default:                      _bad_len++; return false;   // BAD_HEX / BAD_LEN
        }
        if (d.n != sizeof(CmdFrame)) { _bad_len++; return false; }
        _rssi = d.rssi;

        CmdFrame f;
        memcpy(&f, buf, sizeof(f));
        return accept(f);
    }

    // IM920 / BLE 共通の検査。magic/ver を見て、seq で重複と欠落を数える。
    bool accept(const CmdFrame& f) {
        if (f.magic != MAGIC)   { _bad_len++; return false; }
        if (f.ver   != VERSION) { _bad_ver++; return false; }

        if (_n_good) {
            const uint8_t gap = (uint8_t)(f.seq - _last.seq);
            if (gap == 0) return false;              // 重複は捨てる
            if (gap > 1)  _n_lost += (uint32_t)(gap - 1);
        }
        _last       = f;
        _last_rx_ms = millis();
        _n_good++;
        return true;
    }

    HardwareSerial* _ser;
    IM920::LineAssembler<96> _line;
    CmdFrame _last{};
    uint32_t _last_rx_ms = 0;
    uint32_t _n_good = 0, _bad_cs = 0, _bad_len = 0, _bad_ver = 0, _n_lost = 0;
    int      _rssi = -1;
};

} // namespace S5C
