// ============================================================
//  Im920Frame.h  -  IM920sL の 1 行 (TXDA / 受信行) の組み立てと分解
// ============================================================
//  機体 (flight_controller) と地上局 (ground_receiver) が両方使う。
//  S5Telem.h / S5Cmd.h のパケットは全部この 1 層を通る:
//
//    送信:  "TXDA " + hex(payload) + hex(checksum4) + "\r\n"     → IM920::Tx
//    受信:  "<ノード>,<モジュールID>,<RSSI>:<hex...>\r\n"         → IM920::decodeLine
//            データ部はモジュール設定によってバイト間にカンマが入るので落とす。
//            ":" が無い行は "OK" / "NG" / 起動メッセージ (NOT_DATA)。
//
//  チェックサム: 全バイトの総和の 2 の補数を little-endian 4 バイトで後ろに付ける。
//  受信側は「データ部の総和 + チェックサム値 == 0」で検証する。
//
//  ★ 実効ペイロードは 32 バイト (実測)。33 バイト以上を TXDA すると OK を返す
//    のに受信側には先頭 32 バイトしか届かない。Tx::send は static_assert で弾く。
//  ★ 送信は必ず非ブロッキング (Tx::service を毎ループ)。HardwareSerial::print を
//    直接呼ぶと TX バッファ (40B) が溢れた時点でブロックし、1000Hz の制御ループが
//    数十 ms 止まる = それ自体が振動源になる。
//  ★ 受信は固定長 char バッファだけで組む (String は 1000Hz ループでヒープ断片化と
//    数百 us のジッタを持ち込む)。
// ============================================================
#pragma once
#include <Arduino.h>
#include <string.h>

namespace IM920 {

constexpr size_t MAX_PAYLOAD    = 32;   // IM920sL の実効ペイロード [byte]
constexpr size_t CHECKSUM_BYTES = 4;
constexpr size_t MAX_DATA       = MAX_PAYLOAD - CHECKSUM_BYTES;   // 28

inline uint32_t checksum(const uint8_t* p, size_t n) {
    uint32_t s = 0;
    for (size_t i = 0; i < n; ++i) s += p[i];
    return ~s + 1u;
}

inline char* hex2(char* w, uint8_t b) {
    static const char H[] = "0123456789ABCDEF";
    *w++ = H[b >> 4];
    *w++ = H[b & 0x0F];
    return w;
}

inline int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// ------------------------------------------------------------
//  受信行の分解
// ------------------------------------------------------------
enum class Decode : uint8_t {
    OK = 0,
    NOT_DATA,   // ':' が無い (OK / NG / 起動メッセージ)。line はそのまま読める
    BAD_HEX,    // 16 進でない文字 / 奇数桁
    BAD_LEN,    // 短すぎる or cap を超える
    BAD_CS,     // チェックサム不一致
};

struct Decoded {
    Decode  st   = Decode::NOT_DATA;
    size_t  n    = 0;     // チェックサムを除いたデータ長 [byte]
    int     rssi = -1;    // ヘッダ 3 番目のフィールド (16 進)。無ければ -1
};

// line: NUL 終端の 1 行 (CR/LF 無し)。破壊的に触らない。
// out : データ (チェックサム除く) を書く。cap はその容量。
inline Decoded decodeLine(const char* line, uint8_t* out, size_t cap) {
    Decoded r;
    const char* colon = strchr(line, ':');
    if (!colon) return r;   // NOT_DATA

    // ヘッダ "<node>,<id>,<rssi>" の 3 番目が RSSI
    {
        const char* c1 = strchr(line, ',');
        const char* c2 = (c1 && c1 < colon) ? strchr(c1 + 1, ',') : nullptr;
        if (c2 && c2 < colon) r.rssi = (int)strtoul(c2 + 1, nullptr, 16);
    }

    // データ部を 16 進デコード (バイト間のカンマ・空白は読み飛ばす)
    uint8_t buf[MAX_PAYLOAD];
    size_t  n  = 0;
    int     hi = -1;
    for (const char* p = colon + 1; *p; ++p) {
        if (*p == ',' || *p == ' ' || *p == '\r' || *p == '\n') continue;
        const int v = hexVal(*p);
        if (v < 0) { r.st = Decode::BAD_HEX; return r; }
        if (hi < 0) { hi = v; continue; }
        if (n >= sizeof(buf)) { r.st = Decode::BAD_LEN; return r; }
        buf[n++] = (uint8_t)((hi << 4) | v);
        hi = -1;
    }
    if (hi >= 0) { r.st = Decode::BAD_HEX; return r; }
    if (n <= CHECKSUM_BYTES || n - CHECKSUM_BYTES > cap) { r.st = Decode::BAD_LEN; return r; }

    const size_t payload = n - CHECKSUM_BYTES;
    uint32_t sum = 0;
    for (size_t i = 0; i < payload; ++i) sum += buf[i];
    uint32_t cs = 0;
    memcpy(&cs, buf + payload, CHECKSUM_BYTES);
    if ((uint32_t)(sum + cs) != 0u) { r.st = Decode::BAD_CS; return r; }

    memcpy(out, buf, payload);
    r.st = Decode::OK;
    r.n  = payload;
    return r;
}

// ------------------------------------------------------------
//  行の組み立て (Serial から 1 文字ずつ受けて、完全な行が出来たら true)
// ------------------------------------------------------------
//  poll() の呼び出し側が available() のぶんだけ feed() する。
template <size_t CAP = 128>
class LineAssembler {
public:
    // 1 文字入れる。行が完成したら true (line() で読める。次の feed で消える)。
    bool feed(char c) {
        if (_done) { _n = 0; _done = false; }
        if (c == '\n') {
            _buf[_n] = '\0';
            _done = true;
            return true;
        }
        if (c == '\r') return false;
        if (_n < CAP - 1) { _buf[_n++] = c; return false; }
        _n = 0;                 // 長すぎる = 化けている。捨ててやり直す
        _overflow++;
        return false;
    }
    const char* line() const { return _buf; }
    uint32_t overflows() const { return _overflow; }

private:
    char     _buf[CAP];
    size_t   _n = 0;
    bool     _done = false;
    uint32_t _overflow = 0;
};

// ------------------------------------------------------------
//  Tx  -  非ブロッキング送信 (機体・地上局共通)
// ------------------------------------------------------------
//  send() は文字列を内部バッファに組み立てるだけ。実際の UART 書き込みは
//  service() が availableForWrite() の空きぶんだけ進める。前のパケットを送り
//  切る前に send() が来たら、そのパケットは捨てて dropped() を増やす。
class Tx {
public:
    explicit Tx(HardwareSerial* ser) : _ser(ser) {}

    void begin(unsigned long baud = 19200) { _ser->begin(baud); }

    template <typename T>
    bool send(const T& pkt) {
        static_assert(sizeof(T) + CHECKSUM_BYTES <= MAX_PAYLOAD,
                      "IM920sL の 32 バイト制限を超えています "
                      "(超えた分は黙って切り捨てられ、値が壊れます)");
        if (busy()) { _dropped++; return false; }
        const uint8_t* p  = (const uint8_t*)&pkt;
        const uint32_t cs = checksum(p, sizeof(T));
        const uint8_t* q  = (const uint8_t*)&cs;

        char* w = _buf;
        *w++ = 'T'; *w++ = 'X'; *w++ = 'D'; *w++ = 'A'; *w++ = ' ';
        for (size_t i = 0; i < sizeof(T);  ++i) w = hex2(w, p[i]);
        for (size_t i = 0; i < sizeof(cs); ++i) w = hex2(w, q[i]);
        *w++ = '\r'; *w++ = '\n';

        _len = (size_t)(w - _buf);
        _pos = 0;
        _sent++;
        service();          // 空いていればこの場で書けるだけ書く
        return true;
    }

    // 送りかけのバイトを可能なぶんだけ吐き出す。ブロックしない。毎ループ呼ぶ。
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
    HardwareSerial* _ser;
    char     _buf[96];      // "TXDA " + 32*2 + CRLF = 71。余裕を見て 96
    size_t   _len = 0, _pos = 0;
    uint32_t _sent = 0, _dropped = 0;
};

} // namespace IM920
