// ============================================================
//  S5Cmd.h  -  地上局 -> 機体 の上りコマンド (IM920SL)
// ============================================================
//  ★ このファイルは flight_controller/ と ground_receiver/ の両方に
//    同じ内容で置いてある。片方だけ編集すると構造体が食い違って
//    チェックサムは通るのに値だけ壊れる。必ず両方そろえること。
//      flight_controller/include/S5Cmd.h
//      ground_receiver/include/S5Cmd.h
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
//    (6軸IMU。g_yaw_est はアーム時を 0 とした相対方位)。地上局は
//    カメラでヘディングを推定しているので (position_estimator の
//    yaw_estimator)、地面座標 -> 機体座標の回転は地上局側でやる。
//    ※ そのぶんヘディング推定を外すと指令の向きも外れる。ウェイポイント
//      飛行中は機首を回さない (ヨー指令 0) 運用にすること。
//
// ------------------------------------------------------------
//  【帯域】
//    下り (テレメトリ) は 15Hz で UART 占有率 55%。上りを足すぶん、
//    上りは 5Hz に抑える。1 パケット 14+4=18 バイト =
//    "TXDA " + 36桁 + CRLF = 43 文字 = 19200bps で 22ms。5Hz で 11%。
//    合計 66%。それ以上入れるなら下りを 12Hz へ落とすこと。
// ============================================================
#pragma once
#include <Arduino.h>

namespace S5C {

// 構造体を変えたら必ずインクリメントすること (相手側が不一致を検出する)
constexpr uint8_t VERSION = 1;

// 下りテレメトリ (S5T) の type と衝突しない値。'K' = command
constexpr uint8_t MAGIC = 0x4B;

constexpr size_t IM920SL_MAX_PAYLOAD = 32;
constexpr size_t CHECKSUM_BYTES      = 4;

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
};

inline const char* reqName(uint8_t r) {
    switch (r) {
        case REQ_IDLE:    return "IDLE";
        case REQ_HOLD:    return "HOLD";
        case REQ_TAKEOFF: return "TAKEOFF";
        case REQ_GUIDED:  return "GUIDED";
        case REQ_LAND:    return "LAND";
        case REQ_ABORT:   return "ABORT";
    }
    return "?";
}

// ------------------------------------------------------------
//  コマンドパケット (14 byte + チェックサム 4 = 18 byte)
// ------------------------------------------------------------
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
};
static_assert(sizeof(CmdFrame) == 14, "CmdFrame は 14 byte");
static_assert(sizeof(CmdFrame) + CHECKSUM_BYTES <= IM920SL_MAX_PAYLOAD,
              "IM920sL の 32 バイト制限を超えています");

enum CmdFlag : uint16_t {
    CF_ARMED_OK  = 1u << 0,  // 地上局が「飛ばしてよい」と思っている
    CF_POS_VALID = 1u << 1,  // カメラの自己位置推定が生きている
    CF_YAW_VALID = 1u << 2,  // ヘディング推定が収束している
    CF_ALT_ABS   = 1u << 3,  // alt_cm を絶対目標として扱う (落ちていれば現状維持)
};

// スケール
constexpr float SC_MMPS = 1000.0f;   // 1 mm/s
constexpr float SC_CM   = 100.0f;    // 1 cm
constexpr float SC_CDPS = 100.0f;    // 0.01 deg/s

inline int16_t q16(float v, float scale) {
    const float x = v * scale;
    if (x >  32767.0f) return  32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)lroundf(x);
}

// チェックサム (S5Telem.h と同じ方式: 総和の2の補数)
inline uint32_t checksum(const uint8_t* p, size_t n) {
    uint32_t s = 0;
    for (size_t i = 0; i < n; ++i) s += p[i];
    return ~s + 1u;
}

// ============================================================
//  Tx  -  地上局側の非ブロッキング送信
// ============================================================
//  S5T::Tx と同じ流儀。send() は組み立てるだけ、service() が
//  availableForWrite() の空きぶんだけ進める。
// ============================================================
class Tx {
public:
    explicit Tx(HardwareSerial* ser) : _ser(ser) {}

    bool send(const CmdFrame& pkt) {
        if (busy()) { _dropped++; return false; }
        const uint8_t* p  = (const uint8_t*)&pkt;
        const uint32_t cs = checksum(p, sizeof(pkt));
        const uint8_t* q  = (const uint8_t*)&cs;

        char* w = _buf;
        *w++ = 'T'; *w++ = 'X'; *w++ = 'D'; *w++ = 'A'; *w++ = ' ';
        for (size_t i = 0; i < sizeof(pkt); ++i) w = hex2(w, p[i]);
        for (size_t i = 0; i < sizeof(cs);  ++i) w = hex2(w, q[i]);
        *w++ = '\r'; *w++ = '\n';

        _len = (size_t)(w - _buf);
        _pos = 0;
        _sent++;
        service();
        return true;
    }

    void service() {
        while (_pos < _len) {
            const int room = _ser->availableForWrite();
            if (room <= 0) return;
            size_t n = _len - _pos;
            if ((int)n > room) n = (size_t)room;
            _ser->write((const uint8_t*)_buf + _pos, n);
            _pos += n;
        }
    }

    bool     busy()    const { return _pos < _len; }
    uint32_t sent()    const { return _sent; }
    uint32_t dropped() const { return _dropped; }

private:
    static char* hex2(char* w, uint8_t b) {
        static const char H[] = "0123456789ABCDEF";
        *w++ = H[b >> 4];
        *w++ = H[b & 0x0F];
        return w;
    }
    HardwareSerial* _ser;
    char     _buf[64];         // "TXDA " + 36 + CRLF = 43
    size_t   _len = 0, _pos = 0;
    uint32_t _sent = 0, _dropped = 0;
};

// ============================================================
//  Rx  -  機体側の非ブロッキング受信
// ============================================================
//  ★ Telemetry.h の IM920SL_Generic::read() は String を使う。1000Hz の
//    制御ループから String の連結・substring を呼ぶと、ヒープ断片化と
//    数百us のジッタを持ち込む。ここは固定長の char バッファだけで組む。
//
//  IM920 の受信行:  "<ノード>,<モジュールID>,<RSSI>:<データ16進>\r\n"
//  poll() を毎ループ呼ぶ。1回の呼び出しで available() のぶんだけ進め、
//  完全な行が出来たらデコードして last() を更新する。
//
//  ★ 同じ Serial ポートを Telemetry.h の受信と二重に読ませないこと。
//    片方が先にバイトを抜くと、もう片方は永久に行を組み立てられない。
// ============================================================
class Rx {
public:
    explicit Rx(HardwareSerial* ser) : _ser(ser) {}

    // 毎ループ呼ぶ。新しいコマンドを取り込んだら true。
    bool poll() {
        bool got = false;
        while (_ser->available()) {
            const char c = (char)_ser->read();
            if (c == '\n') {
                if (decodeLine()) got = true;
                _n = 0;
            } else if (c != '\r') {
                if (_n < sizeof(_line) - 1) _line[_n++] = c;
                else { _n = 0; _bad_len++; }   // 長すぎる = 化けている
            }
        }
        return got;
    }

    const CmdFrame& last()  const { return _last; }
    uint32_t lastRxMs()     const { return _last_rx_ms; }
    bool     everReceived() const { return _last_rx_ms != 0; }

    // max_age_ms 以内に正常なコマンドが届いているか
    bool fresh(uint32_t max_age_ms) const {
        return _last_rx_ms != 0 && (millis() - _last_rx_ms) < max_age_ms;
    }
    uint32_t ageMs() const {
        return _last_rx_ms ? (millis() - _last_rx_ms) : 0xFFFFFFFFu;
    }

    uint32_t nGood()   const { return _n_good;  }
    uint32_t nBadCs()  const { return _bad_cs;  }
    uint32_t nBadLen() const { return _bad_len; }
    uint32_t nBadVer() const { return _bad_ver; }
    uint32_t nLost()   const { return _n_lost;  }
    int      rssi()    const { return _rssi;    }

private:
    static int hexVal(char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    }

    bool decodeLine() {
        _line[_n] = '\0';
        char* colon = strchr(_line, ':');
        if (!colon) return false;        // "OK" / "NG" / 起動メッセージ

        // ヘッダ3番目のフィールドが RSSI
        {
            *colon = '\0';
            const char* c1 = strchr(_line, ',');
            const char* c2 = c1 ? strchr(c1 + 1, ',') : nullptr;
            if (c2) _rssi = (int)strtoul(c2 + 1, nullptr, 16);
            *colon = ':';
        }

        // データ部を16進デコード (バイト間のカンマは読み飛ばす)
        uint8_t buf[sizeof(CmdFrame) + CHECKSUM_BYTES];
        size_t  n  = 0;
        int     hi = -1;
        for (const char* p = colon + 1; *p; ++p) {
            if (*p == ',' || *p == ' ') continue;
            const int v = hexVal(*p);
            if (v < 0) return false;
            if (hi < 0) { hi = v; continue; }
            if (n >= sizeof(buf)) { _bad_len++; return false; }  // 想定より長い
            buf[n++] = (uint8_t)((hi << 4) | v);
            hi = -1;
        }
        if (hi >= 0 || n != sizeof(buf)) { _bad_len++; return false; }

        uint32_t sum = 0;
        for (size_t i = 0; i < sizeof(CmdFrame); ++i) sum += buf[i];
        uint32_t cs = 0;
        memcpy(&cs, buf + sizeof(CmdFrame), CHECKSUM_BYTES);
        if ((uint32_t)(sum + cs) != 0u) { _bad_cs++; return false; }

        CmdFrame f;
        memcpy(&f, buf, sizeof(f));
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
    char     _line[96];
    size_t   _n = 0;
    CmdFrame _last{};
    uint32_t _last_rx_ms = 0;
    uint32_t _n_good = 0, _bad_cs = 0, _bad_len = 0, _bad_ver = 0, _n_lost = 0;
    int      _rssi = -1;
};

} // namespace S5C
