// ============================================================
//  QuadConfig.h  -  クアッドコプター機体固有の設定を1箇所に集約
// ============================================================
//  このファイルは Stage 2 以降の全ステージが参照します。
//  Stage 1 / Stage 2 のベンチ確認で分かったことを、ここに書き込んでください。
//  制御ロジック側 (drone_s*.cpp) は一切書き換えなくて済むようにしてあります。
//
//  ※ 既存の Config.h / Control.h / Actuators.h は変更していません。
//     auto_flight.cpp や uchida.cpp など他の機体のコードには影響しません。
// ============================================================
#pragma once
#include <Arduino.h>
#include "Config.h"   // enum Sw / enum Ch を使う

namespace Quad {

// ============================================================
//  § 1  機体座標系の定義  (FRD: Forward-Right-Down)
// ============================================================
//    X : 前   roll  = X軸まわり   右へバンクするのが +
//    Y : 右   pitch = Y軸まわり   機首が上がるのが   +
//    Z : 下   yaw   = Z軸まわり   右へ首を振るのが   +
//
//  スティックもこの符号に合わせます。
//    ロールスティックを右に倒す  → roll  指令 +
//    ピッチスティックを引く      → pitch 指令 +
//    ラダーを右に踏む            → yaw   指令 +
//
//  「前」がまだ決まっていないので、シャシーが固まってから
//  実際に前にしたい向きを決め、下の § 3 の符号で辻褄を合わせてください。

// ============================================================
//  § 2  モーター配置  (X配置)
// ============================================================
//  機体を「上から」見て、前方を上とした図:
//
//              前
//        M1 \      / M2          M1 : 左前   CW  (上から見て時計回り)
//             \  /               M2 : 右前   CCW
//              XX                M3 : 右後   CW
//             /  \               M4 : 左後   CCW
//        M4 /      \ M3
//              後
//
//  X配置では対角のペアが同じ回転方向になります (M1&M3 = CW, M2&M4 = CCW)。
//  隣り合うモーターは必ず逆回転です。これが崩れているとヨーが効きません。
//
//  【Stage 1 でやること】
//    1. drone_s1 を書き込み、シリアルで 1〜4 を打って1個ずつ回す
//    2. 実際に回った物理位置を確認し、下の MOTOR_PIN を並べ替える
//       (配線を直すのではなく、この配列を直すほうが確実です)
//    3. 回転方向を目視し、上図と違うモーターは ESC の3本のうち
//       任意の2本を入れ替えて反転させる

constexpr int MOTOR_COUNT = 4;

// MOTOR_PIN[0] = M1(左前), [1] = M2(右前), [2] = M3(右後), [3] = M4(左後)
// ↓ 旧 drone.cpp の motor1..motor4 の割り当てをそのまま初期値にしてあります。
//    Stage 1 の実測結果で必ず並べ替えてください。
constexpr int MOTOR_PIN[MOTOR_COUNT] = { 1,2,3,4 };

// ※ motor::write() (src/sub_lib/Actuators.cpp) は 0.0-1.0 を 1000-2000us 相当に
//   固定でマッピングしており、set_minPWM / set_maxPWM は参照していません。
//   ESC のパルス幅を変えたい場合は Actuators.cpp を直す必要がありますが、
//   auto_flight.cpp が set_minPWM(600) で呼んでいるため、共通の write() を直すと
//   別機体の出力が変わります。今は触っていません。
constexpr int MOTOR_MIN_US = 1000;  // (参考値。現状 write() 側にハードコード)
constexpr int MOTOR_MAX_US = 2000;  // (同上)
constexpr int MOTOR_PWM_HZ = 400;   // (同上)

// ---- ミキサー係数 --------------------------------------------
//   出力[i] = throttle + roll*MIX_ROLL[i] + pitch*MIX_PITCH[i] + yaw*MIX_YAW[i]
//
//   roll +  (右バンク)  → 左側 (M1,M4) を上げ、右側 (M2,M3) を下げる
//   pitch + (機首上げ)  → 後側 (M3,M4) を上げ、前側 (M1,M2) を下げる
//   yaw +   (右旋回)    → CCW 側 (M2,M4) を上げ、CW 側 (M1,M3) を下げる
//
//   ★ yaw の符号は「プロペラの回転方向」で決まります。
//     反作用トルクを使うので、CW で回るプロペラを速くすると機体は CCW に回ります。
//     MOTOR_PIN を並べ替えたら、この行も回転方向に合わせて見直してください。
//
//                                       M1     M2     M3     M4
constexpr float MIX_ROLL [MOTOR_COUNT] = { +1.0f, -1.0f, -1.0f, +1.0f };
constexpr float MIX_PITCH[MOTOR_COUNT] = { -1.0f, -1.0f, +1.0f, +1.0f };
constexpr float MIX_YAW  [MOTOR_COUNT] = { -1.0f, +1.0f, -1.0f, +1.0f };

// + 配置に載せ替える場合はこちら (M1=前, M2=右, M3=後, M4=左) :
//   MIX_ROLL  = {  0, -1,  0, +1 }
//   MIX_PITCH = { -1,  0, +1,  0 }
//   MIX_YAW   = { -1, +1, -1, +1 }

// ---- スロットル ----------------------------------------------
// アイドル: アーム中でもこの値までは常に回す。0 にすると空中で完全停止して
// 復帰できなくなるので、実飛行では 0.05〜0.10 程度を入れる。
// Stage 1/2 (プロペラ無し) の間は 0.0f のままで良い。
constexpr float THR_IDLE     = 0.00f;
// スロットルがこれ以下なら姿勢補正を入れず全モーター停止 (地上での暴れ防止)
constexpr float THR_MIN_MIX  = 0.05f;

// THR_MIN_MIX を境に「姿勢補正なし→全開」が段差でジャンプしないように、
// この幅ぶんかけて姿勢補正の権限を 0%→100% に立ち上げる。
//   thr <= THR_MIN_MIX                       : 補正なし (完全停止)
//   THR_MIN_MIX < thr < THR_MIN_MIX+THR_RAMP : 補正を比例的に効かせる
//   thr >= THR_MIN_MIX+THR_RAMP              : 通常通りフル
//
// これが無いと、スティックを0に戻した直後に残るわずかな補正量(ノイズやD項の
// 残り)だけで、ATTITUDE_PRIORITY がスロットルの下限を急に押し上げてしまい、
// モーターが小刻みに動いたり止まったりする(ピクつく)原因になっていた。
constexpr float THR_RAMP_RANGE = 0.05f;

// 姿勢優先モード (いわゆる airmode)
//   true  : 姿勢補正を必ず全部入れる。入りきらない場合はスロットルをずらして吸収する。
//           → 低スロットルでも姿勢を保てる。代わりに、指定より平均スロットルが
//             上がることがある (thr=0.10 でロール全開なら平均 0.20 になる等)。
//   false : スロットルを優先し、はみ出す姿勢補正は切り捨てる。
//           → スロットル通りに動くが、低スロットルで姿勢が破綻しやすい。
//
//  初飛行では true を推奨します。裏返るより多少浮くほうが安全です。
constexpr bool ATTITUDE_PRIORITY = true;

// ============================================================
//  § 3  IMU の取り付け向き
// ============================================================
//  IMU.h の getAccX/Y/Z, getGyroX/Y/Z は FLU 系 (前・左・上) で値を返します。
//  ここで FRD 系 (前・右・下) に変換します。
//
//  【Stage 2 でやること】プロペラを外し、機体を手で動かして確認:
//    ・機体を右に傾ける          → roll  の表示が +
//    ・機首を持ち上げる          → pitch の表示が +
//    ・機首を右へ回す            → yaw   レートの表示が +
//  合わないものは、下の SIGN を -1 にしてください。
//  軸そのものが入れ替わっている場合 (前後に傾けたのにロールが動く等) は
//  SWAP_XY を true にします。

constexpr bool  SWAP_XY    = false;  // IMU が機体に対して90度回って付いている場合 true

constexpr float GYRO_SIGN_ROLL  = -1.0f;  // X軸まわり
constexpr float GYRO_SIGN_PITCH = -1.0f;  // Y軸まわり (FLU の左 → FRD の右で反転)
constexpr float GYRO_SIGN_YAW   = +1.0f;  // Z軸まわり (FLU の上 → FRD の下で反転)

constexpr float ANG_SIGN_ROLL   = -1.0f;  // Madgwick の出力 [deg]
constexpr float ANG_SIGN_PITCH  = -1.0f;
constexpr float ANG_SIGN_YAW    = +1.0f;

// ============================================================
//  § 4  スティック → 目標値 のスケール
// ============================================================
// SBUS の des[] は -1.0 〜 +1.0 に正規化済み (THR のみ 0.0 〜 1.0)。

// レートモード: スティック全開で何 deg/s を要求するか
constexpr float MAX_RATE_ROLL  = 200.0f;  // [deg/s]
constexpr float MAX_RATE_PITCH = 200.0f;  // [deg/s]
constexpr float MAX_RATE_YAW   = 150.0f;  // [deg/s]

// 角度モード: スティック全開で何 deg 傾けるか
constexpr float MAX_ANGLE_ROLL  = 30.0f;  // [deg]
constexpr float MAX_ANGLE_PITCH = 30.0f;  // [deg]

// スティックの符号 (プロポの設定と合わなければ -1 にする)
constexpr float STICK_SIGN_ROLL  = +1.0f;
constexpr float STICK_SIGN_PITCH = +1.0f;
constexpr float STICK_SIGN_YAW   = +1.0f;

// ============================================================
//  § 5  スロットルカット
// ============================================================
//  旧 drone.cpp は「Ch_state(THR_CUT) == down のとき出力する」でしたが、
//  Receiver.h の th_cut() は「up でカット」と定義されていて食い違っていました。
//  ここで1本化します。Stage 1 でプロペラを外して必ず向きを確認すること。
//
//  Sw の判定 (Receiver.h): des > +0.25 → down / des < -0.25 → up / それ以外 cen
//  ★ 中央 (cen) では出力しません。安全側です。
constexpr Sw ARM_SWITCH_STATE = down;   // このスイッチ位置のときだけモーターに出力する

// ============================================================
//  § 6  制御ループ
// ============================================================
constexpr int RATE_LOOP_HZ  = 1000;  // レートPID (= メインループ)
constexpr int ANGLE_LOOP_HZ = 200;   // 角度PID (レートループを間引いて実行)
constexpr int DEBUG_HZ      = 10;    // シリアル表示

static_assert(RATE_LOOP_HZ % ANGLE_LOOP_HZ == 0,
              "ANGLE_LOOP_HZ は RATE_LOOP_HZ の約数にしてください");
constexpr int ANGLE_LOOP_DIV = RATE_LOOP_HZ / ANGLE_LOOP_HZ;

} // namespace Quad
