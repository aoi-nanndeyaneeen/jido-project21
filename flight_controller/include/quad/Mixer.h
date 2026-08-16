// ============================================================
//  Mixer.h  -  姿勢指令 → 4モーターのスロットル
// ============================================================
//  旧 drone.cpp の問題:
//    ・4本の式が手書きで並んでいて、ヨーの組み合わせが対角ペアと一致していなかった
//    ・補正量を一律 ±0.1 で切っていたので、飽和したときに姿勢が破綻する
//    ・スロットルが 0.1 以下だと補正が一切入らない
//
//  ここでは係数テーブル (QuadConfig.h) を使い、飽和処理も含めて1箇所にまとめます。
// ============================================================
#pragma once
#include <Arduino.h>
#include "quad/QuadConfig.h"

namespace Quad {

// ------------------------------------------------------------
//  MixInfo : ミキサーが実際に何をしたかの報告 (ログ用)
//
//  drone.cpp のロガーが出していた corr_limit / sat と同じ役割です。
//  ・span_limit : 姿勢補正に使える振れ幅
//  ・scale      : 補正が入りきらず縮小したときの倍率 (1.0 なら無縮小)
//  ・sat        : bit0..3 = そのモーターが 0 または 1 に張り付いた
//
//  sat が立ちっぱなしなら「ゲインが高すぎて常に飽和している」証拠です。
// ------------------------------------------------------------
struct MixInfo {
    float   span_limit = 0.0f;
    float   scale      = 1.0f;
    uint8_t sat        = 0;
};

// ------------------------------------------------------------
//  mix() : 姿勢指令とスロットルを 4 モーターの出力 [0.0, 1.0] に変換する
//
//    thr    : スロットル [0.0, 1.0]
//    roll   : ロールトルク指令 [-1.0, 1.0] 相当 (PID出力)
//    pitch  : ピッチトルク指令
//    yaw    : ヨートルク指令
//    out[4] : 結果 (M1..M4)
//
//  飽和の考え方:
//    姿勢制御を優先し、スロットルを譲る。
//    どのモーターも 0..1 に収まらない場合は「姿勢補正の振れ幅」を縮め、
//    それでも収まらない分はスロットルをずらして吸収する。
//    こうすると、フルスロットル付近でも姿勢が保てる。
// ------------------------------------------------------------
inline void mix(float thr, float roll, float pitch, float yaw,
                float out[MOTOR_COUNT], MixInfo* info = nullptr) {

    if (info) *info = MixInfo{};

    // --- スロットルが低いときは全停止 (地上でプロペラが暴れるのを防ぐ) ---
    if (thr <= THR_MIN_MIX) {
        for (int i = 0; i < MOTOR_COUNT; ++i) out[i] = 0.0f;
        return;
    }

    // --- 0) 姿勢補正の権限を、下限のすぐ上からなめらかに立ち上げる ---
    //     ここが無いと thr が THR_MIN_MIX をまたいだ瞬間に補正が全開になり、
    //     出力が階段状にジャンプする (低スロットルでモーターがピクつく原因)。
    const float auth = constrain((thr - THR_MIN_MIX) / THR_RAMP_RANGE, 0.0f, 1.0f);
    roll  *= auth;
    pitch *= auth;
    yaw   *= auth;

    // --- 1) 姿勢補正だけを先に計算する ---
    float lo = +1e9f, hi = -1e9f;
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        out[i] = roll  * MIX_ROLL [i]
               + pitch * MIX_PITCH[i]
               + yaw   * MIX_YAW  [i];
        if (out[i] < lo) lo = out[i];
        if (out[i] > hi) hi = out[i];
    }

    // --- 2) 補正の振れ幅が出せる範囲を超えるなら、全体を比例縮小する ---
    //     (どれか1軸だけ切るより、軸の比率を保ったまま縮めるほうが姿勢が崩れない)
    //     出せる幅は 1.0 からアイドル分を引いた値。
    const float span_limit = 1.0f - THR_IDLE;
    const float span = hi - lo;
    if (info) info->span_limit = span_limit;
    if (span > span_limit) {
        const float k = span_limit / span;
        for (int i = 0; i < MOTOR_COUNT; ++i) out[i] *= k;
        lo *= k;
        hi *= k;
        if (info) info->scale = k;
    }

    // --- 3) スロットルの許容範囲 ---
    const float thr_lo = THR_IDLE - lo;   // 最小モーターがアイドルを下回らない下限
    const float thr_hi = 1.0f     - hi;   // 最大モーターが 1.0 を超えない上限

    if (ATTITUDE_PRIORITY) {
        // 姿勢優先: 補正が全部入るようにスロットルをずらす
        thr = constrain(thr, thr_lo, thr_hi);
    } else {
        // スロットル優先: スロットルは動かさず、はみ出した分は最後の
        // constrain で切り捨てられる (姿勢は崩れやすくなる)
        (void)thr_lo;
        (void)thr_hi;
    }

    // --- 4) 合成 ---
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        const float raw = out[i] + thr;
        out[i] = constrain(raw, 0.0f, 1.0f);
        if (info && (raw <= 0.0f || raw >= 1.0f)) info->sat |= (uint8_t)(1 << i);
    }
}

} // namespace Quad
