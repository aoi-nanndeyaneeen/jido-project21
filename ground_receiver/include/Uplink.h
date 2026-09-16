// ============================================================
//  Uplink.h  -  上りコマンド (PC -> ここ -> IM920 -> 機体) の mailbox と送信
// ============================================================
//  PC (position_estimator) が USB へ 1 行ずつ投げてくる:
//      CMD,<req>,<vx_mmps>,<vy_mmps>,<alt_cm>,<yaw_rate_cdps>[,<flags>[,<corr_n_mm>,<corr_e_mm>[,<yaw_abs_cdeg>[,<laps>]]]]
//  ここは最新値を mailbox に持ち、**蓋 (CMD_MIN_GAP_MS) が開いた次のループで即** 無線へ流す。
//  PC はカメラのフレームレートで送ってきてよい (最新値だけが生き残る)。
//
//  ★ 新しい指令が無いあいだは CMD_KEEPALIVE_MS ごとに最後の指令を送り直す
//    (機体の cmd_fresh を維持する保険)。同じ値の連打を蓋いっぱいの速さで電波に
//    乗せても、下りを削るだけで何も良くならない。
//  ★ PC が CMD_PC_TIMEOUT_MS 黙ったら送信も止める。機体側は「コマンドが来ない」
//    = リンク断として その場ホールド -> 自動着陸 に落ちる。
//  ★ 送れたときだけ時刻を進める。旧実装は送信バッファが詰まって send() が false を
//    返しても時刻を更新していたので、その回の指令は黙って消えていた。
// ============================================================
#pragma once
#include <Arduino.h>
#include "S5Cmd.h"
#include "GroundConfig.h"

namespace Ground {

class Uplink {
public:
    explicit Uplink(HardwareSerial* im920) : _tx(im920) {}

    // ------------------------------------------------------------
    //  "CMD,..." 行のパース → mailbox
    //  ★ 欠けているフィールドは 0 として扱わず、行ごと捨てる。数値が 1 個ずれただけで
    //    「目標高度」が「速度」になる。黙って飛ばすほうが危ない。
    //    ただし flags 以降は後から足したぶんなので省略可 (短い指令行を保つため)。
    // ------------------------------------------------------------
    void handleCmdLine(const char* line) {
        n_lines++;
        long v[10];
        int  n = 0;
        const char* p = line + 4;              // "CMD," の次から
        while (n < 10 && *p) {
            char* end = nullptr;
            v[n] = strtol(p, &end, 10);
            if (end == p) break;               // 数字が無い
            n++;
            p = end;
            if (*p == ',') p++;
            else break;
        }
        if (n < 5) {                           // flags/corr_* は省略可
            n_bad++;
            Serial.printf("# CMD 行が短い (%d 個)。"
                          "CMD,req,vx,vy,alt,yawrate[,flags[,corrN,corrE[,yawabs[,laps]]]]\n", n);
            return;
        }

        _box.magic         = S5C::MAGIC;
        _box.ver           = S5C::VERSION;
        _box.req           = (uint8_t)v[0];
        _box.vx_mmps       = (int16_t)constrain(v[1], -32768, 32767);
        _box.vy_mmps       = (int16_t)constrain(v[2], -32768, 32767);
        _box.alt_cm        = (int16_t)constrain(v[3], -32768, 32767);
        _box.yaw_rate_cdps = (int16_t)constrain(v[4], -32768, 32767);
        _box.flags         = (uint16_t)((n >= 6) ? v[5] : 0);
        _box.corr_n_mm     = (int16_t)constrain((n >= 8) ? v[6] : 0, -32768, 32767);
        _box.corr_e_mm     = (int16_t)constrain((n >= 8) ? v[7] : 0, -32768, 32767);
        _box.yaw_abs_cdeg  = (int16_t)constrain((n >= 9) ? v[8] : 0, -32768, 32767);
        _box.laps          = (uint8_t)constrain((n >= 10) ? v[9] : 0, 0, 255);
        _box.rsv           = 0;
        _have       = true;
        _dirty      = true;                    // 次に蓋が開いた瞬間に出す
        _last_pc_ms = millis();
    }

    // 毎ループ呼ぶ。mailbox を無線へ流す。
    void service() {
        _tx.service();                         // 送りかけを吐き出す (非ブロッキング)
        if (!_have) return;

        const uint32_t now = millis();
        if (now - _last_pc_ms > CMD_PC_TIMEOUT_MS) {
            _have  = false;
            _dirty = false;
            Serial.println("# PC からの CMD が途切れました。上り送信を停止します "
                           "(機体はホールド -> 自動着陸へ)");
            return;
        }
        // 帯域の蓋。上りのレート上限を決めるのはここだけ。
        if (now - _last_tx_ms < CMD_MIN_GAP_MS) return;
        // 新しい指令が無いなら、キープアライブの間隔まで待つ
        if (!_dirty && (now - _last_tx_ms < CMD_KEEPALIVE_MS)) return;

        _box.seq = _seq;
        if (_tx.send(_box)) {
            _seq++;
            _last_tx_ms = now;
            _dirty      = false;
            n_tx++;
        }
    }

    bool have() const { return _have; }        // PC からの指令を持っている (送信中)
    void clearStats() { n_lines = n_tx = n_bad = 0; }

    uint32_t n_lines = 0;   // PC から受けた CMD 行
    uint32_t n_tx    = 0;   // 無線へ出した回数
    uint32_t n_bad   = 0;   // パースできなかった行

private:
    S5C::Tx       _tx;
    S5C::CmdFrame _box{};       // 最新の指令 (mailbox)
    bool     _have  = false;
    bool     _dirty = false;    // mailbox に「まだ送っていない指令」があるか
    uint32_t _last_pc_ms = 0;   // PC から最後に行が来た時刻
    uint32_t _last_tx_ms = 0;
    uint8_t  _seq = 0;
};

} // namespace Ground
