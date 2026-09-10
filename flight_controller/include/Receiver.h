//入力系
#pragma once
#include <Arduino.h>
#include <sbus.h>
#include "Config.h"

class Sbus {
private:

    bfs::SbusRx *_sbus;
    // ★ {} を追加: SbusData は初期化子を持たない POD なので、
    //   これが無いと最初の Read() が成功するまで failsafe / ch[] が不定値でした。
    //   起動直後に isSafe() が不定値を読んでしまうのを防ぎます。
    bfs::SbusData _data{};
    int i;

    int connection_fail;

    // ★ 起動時に取り込むスティック中央 (ROLL/PITCH/YAW のみ使う)。
    //   0 のままなら従来と完全に同じ挙動。
    float _center[16];
    // キャリブ前の生の正規化値 (-1..+1)。calibrateCenter() が平均を取る対象。
    // des[] は中央を引いた「後」の値なので、これで取ると自分の補正が入って
    // しまう。生値を別に持っておく。
    float _raw[16];

    // 中央付近を 0.0 へスナップする幅 (des[] の単位)。
    //  元の実装は 0..1 スケールで abs(val-0.50)<0.02 だった。-1..+1 では 0.04。
    //  値は変えていない (中央を引く位置が変わっただけ)。
    static constexpr float CENTER_SNAP = 0.04f;

    float scaleToUnit(float raw) {
        constexpr float minVal = 360.0f;
        constexpr float midVal = 1040.0f;
        constexpr float maxVal = 1680.0f;

        // 範囲外をガード
        raw = constrain(raw, minVal, maxVal);

        if (raw >= midVal) {
            // 1040〜1680 を 0.5〜1.0 にマップ
            // (現在の値 - 中央) / (最大 - 中央) * 0.5 + 0.5
            return (raw - midVal) / (maxVal - midVal) * 0.5f + 0.5f;
        } 
        else {
            // 360〜1040 を 0.0〜0.5 にマップ
            // (現在の値 - 最小) / (中央 - 最小) * 0.5
            return (raw - minVal) / (midVal - minVal) * 0.5f;
        }
    }

public:
    float des[16];

    Sbus(HardwareSerial *ser) {
        _sbus = new bfs::SbusRx(ser, true);
        connection_fail = 3000;
        for(int i=0; i<16; i++) { des[i] = 0.0f; _center[i] = 0.0f; _raw[i] = 0.0f; }
    }

    void begin() {
        _sbus->Begin();
    }

    void update() {

      if (_sbus->Read()) {
        _data = _sbus->data();
        connection_fail = 0;
      }
      else connection_fail++;//通信途絶検知用

      // SBUSの172-1811を0.0-1.0にマッピング
      for(i=0;i<16;i++){
        //if(i == 1)Serial.println(_data.ch[i]);
        float val = scaleToUnit(_data.ch[i]);//取得して0-1にマッピング
        val = (val < 0.0f) ? 0 : ((val > 1.0f) ? 1.0f : val);//?はif,:はelse,そのため(条件)? 結果 : 結果　みたいな感じ

        if(i!=Ch::THR) {
          // 0..1 -> -1..+1 にしてから、起動時に取り込んだ中央を引く。
          // スナップ判定は「中央を引いた後」に効かせる。こうすると
          // サブトリムがずれていても、手を離した状態がちゃんと 0.0 になる。
          // _center が 0 のときは元の式と完全に一致する。
          const float u = val*2.0f-1.0f;
          _raw[i] = u;
          const float c = u - _center[i];
          val = (fabsf(c) < CENTER_SNAP) ? 0.0f : c;
        }

        else {
          _raw[i] = val;
          val = ((val) < 0.02f) ? 0.00 : val;
        }//thrだけ例外だね
        des[i]= val;
      }
    }

    // ---- スティック中央の起動時キャリブ ----------------------------------
    //  ROLL/PITCH/YAW の静止値を ms ミリ秒ぶん実測し、以後 des[] から引く。
    //  ★ THR とスイッチ ch は触らない (THR の中央は下端、スイッチは
    //    Ch_state() が ±0.25 で判定するので原点を動かすと誤判定する)。
    //  ★ 棄却条件を 2 つ持つ。どちらかに引っかかったらオフセット 0 のまま
    //    (= 従来動作) にして、呼び出し側が警告を出せるよう理由を返す:
    //      Moving : 取り込み中の振れ幅が max_move 超 = 動かしている
    //      TooFar : 平均が max_ofs 超 = スティックを握ったまま起動した疑い
    //    黙って握った位置を「中央」として焼き込むのが一番危ないので、
    //    ここは通さずに従来動作へ落とす。
    enum CenterSt : uint8_t { CC_OK, CC_NOSIG, CC_MOVING, CC_TOOFAR };
    struct CenterCal {
        CenterSt st;
        float    roll, pitch, yaw;   // 取り込んだ (or 取り込もうとした) 中央
        int      n;                  // 使った有効フレーム数
        float    worst_move;         // 3軸で最大の振れ幅
    };

    CenterCal calibrateCenter(uint16_t ms, float max_ofs, float max_move) {
        CenterCal r{};
        r.st = CC_OK;
        const int CH[3] = { Ch::ROLL, Ch::PITCH, Ch::YAW };
        double sum[3] = { 0.0, 0.0, 0.0 };
        float  lo[3], hi[3];
        for (int k = 0; k < 3; ++k) { lo[k] = 1e9f; hi[k] = -1e9f; }

        const uint32_t t0 = millis();
        while (millis() - t0 < ms) {
            update();
            // failCount()==0 は「今の update() でフレームが取れた」の意味。
            // 取れていないときは _data が前回のままなので数えない。
            if (connection_fail == 0) {
                for (int k = 0; k < 3; ++k) {
                    const float v = _raw[CH[k]];
                    sum[k] += v;
                    if (v < lo[k]) lo[k] = v;
                    if (v > hi[k]) hi[k] = v;
                }
                ++r.n;
            }
            delay(1);
        }
        if (r.n < 10) { r.st = CC_NOSIG; return r; }

        for (int k = 0; k < 3; ++k) {
            const float mv = hi[k] - lo[k];
            if (mv > r.worst_move) r.worst_move = mv;
        }
        const float m0 = (float)(sum[0] / r.n);
        const float m1 = (float)(sum[1] / r.n);
        const float m2 = (float)(sum[2] / r.n);
        r.roll = m0; r.pitch = m1; r.yaw = m2;

        if (r.worst_move > max_move) { r.st = CC_MOVING; return r; }
        if (fabsf(m0) > max_ofs || fabsf(m1) > max_ofs || fabsf(m2) > max_ofs) {
            r.st = CC_TOOFAR; return r;
        }
        _center[CH[0]] = m0;
        _center[CH[1]] = m1;
        _center[CH[2]] = m2;
        return r;
    }

    // 今適用されている中央オフセット (デバッグ表示用)
    float center(int ch) const { return _center[ch]; }

    Sw Ch_state(int ch) {
        if (des[ch] > 0.25)     return down;                //ここで条件式をそのままreturnする発想は私にはなかった(´・ω・｀)
        if (des[ch] < -0.25)    return up;
        return cen;
    }

    bool th_cut() {
        if(Ch_state(Ch::THR_CUT)==up)   return true;   
        return false;
    }

    bool isSafe() {
        return !_data.failsafe && !(connection_fail>3000);
    }

    // 直近 update() でフレームを取れなかった連続回数。0 = 今まさに受信できている。
    // 起動時に「受信機がつながっているか」を判定するのに使う。
    int failCount() const { return connection_fail; }
};