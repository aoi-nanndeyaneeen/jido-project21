// ============================================================
//  S5Telem.h  -  s5 (POSHOLD) 解析用 IM920SL テレメトリ
// ============================================================
//  目的:
//    実飛行中は USB を挿せないので、drone_s5.cpp の 500Hz ログ
//    (Log:: 名前空間) が取れない。その代わりに「POSHOLD の解析に
//    必要な信号だけ」を無線で落とし、地上局側で CSV にする。
//    姿勢(数十Hz)の解析はできないが、高度・位置ホールドのループは
//    1〜2Hz の現象なので、これで足りる。
//
//  ★ このファイルは flight_controller/ と ground_receiver/ の両方に
//    同じ内容で置いてある。片方だけ編集すると構造体が食い違って
//    チェックサムは通るのに値だけ壊れる。必ず両方そろえること。
//      flight_controller/include/S5Telem.h
//      ground_receiver/include/S5Telem.h
//    ずれの検出用に VERSION と static_assert(sizeof) を入れてある。
//
// ------------------------------------------------------------
//  【実測した制約】 2026-09-04 実機で測定
//
//  1. IM920sL の実効ペイロードは 32 バイト。
//     33 バイト以上を TXDA すると、モジュールは OK を返すのに
//     受信側には先頭 32 バイトしか届かない (黙って切り詰められる)。
//     ※ Config.h に「64-4=60バイト」とあるのは誤り。あれは IM920 /
//       IM920s の値で、IM920sL は 32 バイト。旧 PlaneData(28+4=32) が
//       たまたま上限ちょうどだったので動いていただけ。
//       GroundData(42+4=46) は最初から切り詰められている。
//     → データ 28 バイト + チェックサム 4 バイト = 32 バイト厳守。
//
//  2. 持続できるパケットレートは 15Hz まで。
//       10Hz 欠落 0% / 12Hz 0% / 15Hz 0% / 20Hz 5% / 25Hz 22.5%
//     (32バイト = "TXDA " + 64桁 + CRLF = 71文字 = 19200bps で 37ms。
//      15Hz で UART 占有率 55%)
//     → drone_s5.cpp の TELEM_TX_HZ = 15。
//
//  この 2 つから、1 パケットに全部は載らない。そこでフレームを分けて
//  順番に送る。0.3〜2Hz のループを見るにはこれで十分。
//    A = 高度ループ + 姿勢角
//    B = 水平位置ループ
//    C = 姿勢ループの内部 (モーター出力 / 角速度 実測・目標 / ミキサー飽和)
//    P = ゲイン一覧 (数秒に1回、枠を1つ borrow する)
//
//  ★ 送る順番はモードで変える (drone_s5.cpp の S5Tel::tick):
//      ANGLE / RATE  … A,C の交互          (各 7.5Hz)
//        B(水平位置) は POSHOLD 以外では全部 0 なので送っても無駄。
//        代わりに C を厚くして、離陸時の転倒やモーター飽和を追えるようにする。
//      POSHOLD/AUTO  … A,B,A,C の4枠巡回   (A 7.5Hz / B 3.75Hz / C 3.75Hz)
//        位置ループは 0.3〜1Hz なので 3.75Hz あれば足りる。
//
//  ★ 送信は必ず非同期 (S5T::Tx::service)。HardwareSerial::print を直接
//    呼ぶと TXバッファ(40B)が溢れた時点でブロックし、1000Hz の制御
//    ループが数十ms止まる = それ自体が振動源になる。
//
//  【スケーリング】
//    float をそのまま並べると入らないので int16 に量子化する。
//    分解能は下の SC_* (位置 1cm / 速度 1mm/s / 角度 0.01deg)。
// ============================================================
#pragma once
#include <Arduino.h>

namespace S5T {

// 構造体を変えたら必ずインクリメントすること (地上局が不一致を検出する)
constexpr uint8_t VERSION = 4;

// IM920sL の実効ペイロード上限 [byte]。これを超えると黙って切られる。
constexpr size_t IM920SL_MAX_PAYLOAD = 32;
constexpr size_t CHECKSUM_BYTES      = 4;
constexpr size_t PACKET_BYTES        = IM920SL_MAX_PAYLOAD - CHECKSUM_BYTES;  // 28

constexpr uint8_t TYPE_ALT   = 0x41;  // 'A'  高度ループ + 姿勢
constexpr uint8_t TYPE_POS   = 0x42;  // 'B'  水平位置ループ
constexpr uint8_t TYPE_ATT   = 0x43;  // 'C'  姿勢ループ内部 (モーター出力/レート)
constexpr uint8_t TYPE_PARAM = 0x50;  // 'P'  ゲイン一覧

// ---- 量子化スケール (物理値 = 整数値 / SC_xxx) ----
constexpr float SC_CDEG = 100.0f;    // 0.01 deg
constexpr float SC_DDEG = 10.0f;     // 0.1  deg  (ヨー積分値は ±3276deg まで)
constexpr float SC_MM   = 1000.0f;   // 1 mm / 1 mm/s
constexpr float SC_CM   = 100.0f;    // 1 cm      (位置は ±327m まで)
constexpr float SC_1E4  = 10000.0f;  // スロットル割合 0.0001
constexpr float SC_GAIN = 1000.0f;   // ゲイン 0.001 刻み (±32.7)
constexpr float SC_STICK = 100.0f;   // スティック 0.01 刻み (int8, ±1.27)

// flags のビット割り当て (A/B 共通)
enum Flag : uint16_t {
    F_ARMED       = 1u << 0,
    F_FLOW_OK     = 1u << 1,   // PMW3901 が初期化できている
    F_RANGE_OK    = 1u << 2,   // 測距センサが初期化できている
    F_RANGE_VALID = 1u << 3,   // 今この瞬間の測距が信用できる
    F_ALT_EN      = 1u << 4,   // 高度ホールドが有効化されている (シリアル 'g')
    F_ALT_ACT     = 1u << 5,   // 高度ホールドが実際にスロットルを握っている
    F_POS_HOLD    = 1u << 6,   // 位置ホールドが基準を保持中 (スティック中立)
    F_AIRBORNE    = 1u << 7,   // 離陸判定が立っている
    F_DRY_RUN     = 1u << 8,   // ESC へ 0 しか出していない
    F_SAT         = 1u << 9,   // ミキサーがどこかで飽和した
    F_TX_DROP     = 1u << 10,  // 送信バッファが空かず直前に1回捨てた
};

// mode と alt_state を 1 バイトに詰める (上位=alt_state 下位=mode)
inline uint8_t packModes(uint8_t mode, uint8_t alt_state) {
    return (uint8_t)(((alt_state & 0x0F) << 4) | (mode & 0x0F));
}
inline uint8_t unpackMode(uint8_t m)     { return (uint8_t)(m & 0x0F); }
inline uint8_t unpackAltState(uint8_t m) { return (uint8_t)((m >> 4) & 0x0F); }

// ------------------------------------------------------------
//  共通ヘッダ (6 byte)
//    t_cs は millis()/10 [10ms単位]。655.35 秒で一周するので、地上側で
//    自前の受信時刻を使って巻き戻しを展開する (s5_log.cpp の unwrap)。
//    uint32 の生 millis を載せる余裕は 28 バイトには無い。
// ------------------------------------------------------------
struct __attribute__((__packed__)) Header {
    uint8_t  type;    // TYPE_ALT / TYPE_POS / TYPE_PARAM
    uint8_t  seq;     // A/B 通しの連番。地上で欠落を数える
    uint16_t flags;   // Flag のビット論理和
    uint16_t t_cs;    // 機体側 millis()/10
};
static_assert(sizeof(Header) == 6, "Header は 6 byte");

// ------------------------------------------------------------
//  A: 高度ループ + 姿勢  (28 byte)
// ------------------------------------------------------------
struct __attribute__((__packed__)) AltFrame {
    Header  h;                 //  6
    uint8_t modes;             //  7  packModes(mode, alt_state)
    uint8_t thr;               //  8  スロットルスティック 0..250 (=0.000..1.000)
    int16_t roll_cd;           // 10  [0.01 deg]
    int16_t pitch_cd;          // 12
    int16_t yaw_dd;            // 14  ヘディング積分値 [0.1 deg]
    int16_t range_h_mm;        // 16  傾き補正後の対地高度
    int16_t range_raw_mm;      // 18  補正前の斜め距離 (センサ生値の健全性確認用)
    int16_t alt_hold_mm;       // 20  目標高度
    int16_t climb_mmps;        // 22  上昇速度 実測
    int16_t alt_vz_tar_mmps;   // 24  上昇速度 目標 (位置ループ出力)
    int16_t alt_thr_corr;      // 26  スロットル補正 (速度PID出力) [1e-4]
    uint16_t alt_thr_out;      // 28  実際に出したスロットル [1e-4]
};
static_assert(sizeof(AltFrame) == PACKET_BYTES, "AltFrame が 28 byte ではありません");

// ------------------------------------------------------------
//  B: 水平位置ループ  (28 byte)
// ------------------------------------------------------------
struct __attribute__((__packed__)) PosFrame {
    Header  h;                 //  6
    uint8_t modes;             //  7
    uint8_t bad;               //  8  poshold.badCount() (フロー失探の連続回数)
    int16_t vx_mmps;           // 10  制御に使う対地速度 (機体座標 前+) [mm/s]
    int16_t vy_mmps;           // 12  (右+)
    int16_t vx_tar_mmps;       // 14  速度ループの目標
    int16_t vy_tar_mmps;       // 16
    int16_t pos_n_cm;          // 18  地面固定フレームの推定位置 [cm]
    int16_t pos_e_cm;          // 20
    int16_t hold_n_cm;         // 22  保持したい地面位置 [cm]
    int16_t hold_e_cm;         // 24
    int16_t lean_roll_cd;      // 26  位置ループが出した目標リーン角 [0.01deg]
    int16_t lean_pitch_cd;     // 28
};
static_assert(sizeof(PosFrame) == PACKET_BYTES, "PosFrame が 28 byte ではありません");

// ------------------------------------------------------------
//  C: 姿勢ループの内部  (28 byte)
// ------------------------------------------------------------
//  離陸時の転倒や発振の原因を追うためのフレーム。A/B だけでは
//  「どのモーターが飽和したか」「レートループが指令に追従しているか」が
//  分からず、接地したまま転がったのか空中で発振したのかを切り分けられない。
//
//  ★ 角速度は 0.1 deg/s 刻み (±3276 deg/s)。トルク指令 cmd は [-1,1] を
//    1e-4 刻みで。モーター出力は 0..250 = 0.000..1.000 (分解能 0.004)。
//  ★ yaw_cmd は入っていない (28バイトに入らなかった)。ヨーの問題は
//    yaw_rate_dd で見る。枠はスティック2本に使っている。
struct __attribute__((__packed__)) AttFrame {
    Header  h;                 //  6
    uint8_t modes;             //  7
    uint8_t sat;               //  8  Q::MixInfo::sat  bit0..3 = M1..M4 が張り付いた
    uint8_t m1, m2, m3, m4;    // 12  各モーター出力 0..250 (= 0.000..1.000)
    int16_t roll_rate_dd;      // 14  実測角速度 [0.1 deg/s]
    int16_t pitch_rate_dd;     // 16
    int16_t yaw_rate_dd;       // 18
    int16_t roll_rate_tar_dd;  // 20  角度ループが出した目標角速度 [0.1 deg/s]
    int16_t pitch_rate_tar_dd; // 22
    int16_t roll_cmd;          // 24  ミキサーへ渡したトルク指令 [1e-4]
    int16_t pitch_cmd;         // 26
    // ★ 受信機から読んだ生のスティック値 (STICK_SIGN を掛ける前)。
    //   トリムずれの検出用。2026-09-04、ロールスティックが中立のつもりで
    //   +0.318 ずれており、角度ループが常時「+9.5度傾け」と言われていた。
    //   これが「離陸しようとすると必ず一方向へ流れる」の正体だった。
    //   逆算でしか分からず時間を溶かしたので、以後は直接載せる。
    //   ※ 非アーム中でも入る (sbus から直接読む)。飛ばす前に地上で確認できる。
    int8_t  roll_stick;        // 27  [0.01] -1.00..+1.00
    int8_t  pitch_stick;       // 28
};
static_assert(sizeof(AttFrame) == PACKET_BYTES, "AttFrame が 28 byte ではありません");

// ------------------------------------------------------------
//  P: ゲイン一覧  (28 byte)
//    シリアル 'p' メニューで飛行中に変えられる値があるので、CSV だけ
//    見て「どのゲインの結果か」が分かるようにするため数秒に1回混ぜる。
//    float は入らないので 0.001 刻みの int16 にする (±32.7 まで)。
// ------------------------------------------------------------
enum ParamFlag : uint8_t {
    PF_ALT_HOLD_EN = 1u << 0,
    PF_DRY_RUN     = 1u << 1,
    PF_SONAR       = 1u << 2,   // 1=MaxBotix EZ (PW) / 0=VL53L1X (I2C)
    PF_STICK_VZ    = 1u << 3,
};

struct __attribute__((__packed__)) ParamFrame {
    uint8_t type;          //  1  TYPE_PARAM
    uint8_t seq;           //  2
    uint8_t ver;           //  3  VERSION
    uint8_t cfg_flags;     //  4  ParamFlag
    int16_t flow_vel_kp;   //  6  すべて [x1000]
    int16_t flow_vel_ki;   //  8
    int16_t flow_vel_kd;   // 10
    int16_t flow_pos_kp;   // 12
    int16_t alt_pos_kp;    // 14
    int16_t alt_rate_kp;   // 16
    int16_t alt_rate_ki;   // 18
    int16_t alt_rate_kd;   // 20
    int16_t alt_hover_thr; // 22
    int16_t alt_target_m;  // 24
    int16_t flow_max_lean; // 26
    int16_t alt_thr_auth;  // 28
};
static_assert(sizeof(ParamFrame) == PACKET_BYTES, "ParamFrame が 28 byte ではありません");

// ------------------------------------------------------------
//  量子化ヘルパ (飽和付き)
// ------------------------------------------------------------
inline int16_t q16(float v, float scale) {
    const float x = v * scale;
    if (x >  32767.0f) return  32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)lroundf(x);
}
inline uint16_t qu16(float v, float scale) {
    const float x = v * scale;
    if (x > 65535.0f) return 65535;
    if (x < 0.0f)     return 0;
    return (uint16_t)lroundf(x);
}
inline uint8_t qu8(float v, float scale) {
    const float x = v * scale;
    if (x > 255.0f) return 255;
    if (x < 0.0f)   return 0;
    return (uint8_t)lroundf(x);
}

// ------------------------------------------------------------
//  チェックサム (既存 Telemetry.h と同じ方式)
//    全バイトの総和の2の補数を little-endian 4バイトで後ろに付ける。
//    受信側は「データ部の総和 + チェックサム値 == 0」で検証する。
// ------------------------------------------------------------
inline uint32_t checksum(const uint8_t* p, size_t n) {
    uint32_t s = 0;
    for (size_t i = 0; i < n; ++i) s += p[i];
    return ~s + 1u;
}

// ============================================================
//  Tx  -  機体側の非ブロッキング送信
// ============================================================
//  send() は文字列を内部バッファに組み立てるだけ。実際の UART 書き込みは
//  service() が availableForWrite() の空きぶんだけ進める。毎ループ
//  (1000Hz) 呼ぶこと。前のパケットを送り切る前に send() が来たら、その
//  パケットは捨てて dropped() を増やす (制御ループを止めるより良い)。
// ============================================================
class Tx {
public:
    explicit Tx(HardwareSerial* ser) : _ser(ser) {}

    // ★ Serial3 を FlightTelemetry::begin() が既に開いているなら呼ばなくてよい。
    void begin(unsigned long baud = 19200) { _ser->begin(baud); }

    template <typename T>
    bool send(const T& pkt) {
        static_assert(sizeof(T) + CHECKSUM_BYTES <= IM920SL_MAX_PAYLOAD,
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

    // 送りかけのバイトを可能なぶんだけ吐き出す。ブロックしない。
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
    // "TXDA " + 32*2 + CRLF = 71。余裕を見て 96。
    char     _buf[96];
    size_t   _len = 0, _pos = 0;
    uint32_t _sent = 0, _dropped = 0;
};

} // namespace S5T
