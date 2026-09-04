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
//   pitch + (機首上げ)  → 後側 (M3,M4) を下げ、前側 (M1,M2) を上げる
//   yaw +   (右旋回)    → CCW 側 (M2,M4) を上げ、CW 側 (M1,M3) を下げる
//
//   ★ yaw の符号は「プロペラの回転方向」で決まります。
//     反作用トルクを使うので、CW で回るプロペラを速くすると機体は CCW に回ります。
//     MOTOR_PIN を並べ替えたら、この行も回転方向に合わせて見直してください。
//
//                                       M1     M2     M3     M4
constexpr float MIX_ROLL [MOTOR_COUNT] = { +1.0f, -1.0f, -1.0f, +1.0f };
constexpr float MIX_PITCH[MOTOR_COUNT] = { +1.0f, +1.0f, -1.0f, -1.0f };
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

// ============================================================
//  § 7  オプティカルフロー (PMW3901)   ※ Stage 5 (drone_s5) で使用
// ============================================================
//  バス: グローバル SPI を使う。Teensy 4.0 では SCK=13 / MOSI=11 / MISO=12。
//        IMU は I2C (Wire) なので競合しない。モーターは 1..4、
//        SBUS=Serial5(20/21)、IM920=Serial3(14/15) なので下の CS と重ならない。
//
//  【s5a でやること】プロペラを外し、drone_s5 を書き込んでシリアル表示とログを見る:
//   1. 機体を手で「前」へ平行移動  → flow_dx が一方向に大きく動くか
//   2. 機体を手で「右」へ平行移動  → flow_dy
//      前後に動かしたのに flow_dy が動くなら軸が入れ替わっている → FLOW_SWAP_XY = true
//   3. 前進で flow_dx が + になるように FLOW_SIGN_X、右移動で + になるように FLOW_SIGN_Y
//   4. 機体を「回転だけ」させる (その場でヨー以外に傾ける)。
//      de-rotation が正しければ flow_dx / flow_dy がほぼ 0 のまま。
//      逆に振れるなら FLOW_DEROT_SIGN_* を反転。量が合わなければ FLOW_PX_PER_RAD を調整。

constexpr int FLOW_CS_PIN = 10;   // ★ PMW3901 の CS。実配線に合わせて変更すること

// センサ X/Y 軸 → 機体座標 (FRD: 前=x, 右=y)
constexpr bool  FLOW_SWAP_XY = false;
constexpr float FLOW_SIGN_X  = -1.0f;   // 前進で + になる向き
constexpr float FLOW_SIGN_Y  = -1.0f;   // 右移動で + になる向き

// 生カウント ⇔ 角度 の換算 [raw count / rad]。
//  ★ 幾何(画素数/FOV)から出る 40 前後は間違い。PMW3901 の生カウントは
//    内部でサブピクセル化されていて、実効値は 100倍ほど大きい。
//    (Crazyflie の flowdeck は換算すると raw ≒ 4098 count/rad 相当)
//  de-rotation と 速度換算の両方で使う。★必ず s5a のベンチで実測する:
//    [z] でゼロ → 実測高度 h で実測距離 D を1方向スライド → 積算raw を読む
//    FLOW_PX_PER_RAD = (積算raw ÷ D[m]) × h[m]
//  この値が桁で外れていると de-rotation がほぼ効かず (derot ≒ raw)、
//  速度換算も桁でズレる。
constexpr float FLOW_PX_PER_RAD = 450.0f;   // ← 仮値。ベンチ実測に置き換えること

// de-rotation: 機体の角速度が作る「見かけの流れ」をジャイロで差し引く。
//  pitch(Y軸まわり)レート → flow_x に乗る / roll(X軸まわり)レート → flow_y に乗る
constexpr bool  FLOW_DEROTATE     = true;
constexpr float FLOW_DEROT_SIGN_X = -1.0f;
constexpr float FLOW_DEROT_SIGN_Y = -1.0f;

// IMU (≒回転中心) から フローレンズ までのオフセット [m]  (機体座標 FRD: 前+ / 右+)
//  水平オフセットがあると、機体の回転がテコで並進フローに化ける
//  (de-rotation では消えない)。新シャシーでは 0.02m 未満を目標。
//  詰められない場合は ±5mm で実測してここに記録 → s5b でテコ補正に使う。
constexpr float FLOW_OFFSET_X = 0.00f;   // 前方向のずれ
constexpr float FLOW_OFFSET_Y = 0.00f;   // 右方向のずれ

// s5b で使う「仮定高度」[m]。距離センサ搭載後 (s5c) は測距値を毎ループ渡す。
//  ピクセル速度→対地速度の換算はこの値に比例する。テストホバリング高度に合わせること。
constexpr float FLOW_ASSUMED_HEIGHT_M = 1.0f;

// フロー更新レート [Hz]。1000Hz メインから間引く。
//  PMW3901 の1カウント分解能は粗い。レートを下げるほど1サンプルに溜まる
//  カウントが増えて SNR が上がる (代わりに遅延が増える)。50Hz で 20ms 遅延。
//  ※ Bitcraze ライブラリの readMotionCount は内部 delayMicroseconds で
//    1回あたり ~1ms ブロックする。レートを下げるとその取りこぼしも減る。
constexpr int FLOW_LOOP_HZ = 50;
static_assert(RATE_LOOP_HZ % FLOW_LOOP_HZ == 0,
              "FLOW_LOOP_HZ は RATE_LOOP_HZ の約数にしてください");
constexpr int FLOW_LOOP_DIV = RATE_LOOP_HZ / FLOW_LOOP_HZ;

// ============================================================
//  § 7-2  s5b : フロー速度・位置ホールド
// ============================================================
//  構造 (カスケード):
//    位置誤差[m] ──[POS_KP]──► 目標速度[m/s] ──[VEL PID]──► 目標リーン角[deg]
//                                                        └─► 既存の角度ループへ
//  スティックを触ると「目標速度」を直接指令し、離すとその場の位置を保持する
//  (ヘディングホールドと同じ考え方)。スロットルは全モード手動。
//
//  ★★ 初回テストは必ず「係留 or 広い床の上・指をモードスイッチに」。
//     符号(下の *_SIGN)を1つ間違えると即座に壁へ突っ込みます。
//     まず地面近く 10〜20cm で、手で押したら押し返してくるか確認すること。

// 速度ループ: (目標速度 - 実測速度[m/s]) → 目標リーン角[deg]
//   kp=6 なら「1 m/s ずれていたら 6度 傾けて戻す」
constexpr float FLOW_VEL_KP      = 6.0f;
constexpr float FLOW_VEL_KI      = 2.0f;    // 定常風・機体の取り付け傾きを吸収
constexpr float FLOW_VEL_KD      = 0.0f;
constexpr float FLOW_VEL_I_LIMIT = 4.0f;    // I項の上限 [deg]
constexpr float FLOW_VEL_D_ALPHA = 0.6f;

// 位置ループ(外側): 位置誤差[m] → 目標速度[m/s]
//   kp=1.0 なら「1m ずれていたら 1 m/s で戻る」。0 にすると純粋な速度ホールド。
constexpr float FLOW_POS_KP      = 1.0f;
constexpr float FLOW_POS_VEL_LIM = 0.8f;    // 位置ループが出す目標速度の上限 [m/s]

// 保持基準 (g_pos_hold) を「今の推定位置」へゆっくり緩和する時定数 [s]。
//  これが無いと、フローのノイズや残留誤差が積分されて位置推定がズレたとき、
//  ズレがどれだけ経っても消えず (積分にリークが無い)、静止していても
//  目標リーン角が0に戻らなくなる。この時定数で「一度ズレた分は数秒で
//  忘れる」ようにし、短時間のホバリングで実用になる範囲に収める。
//  0 にするとリーク無し (旧挙動)。
constexpr float FLOW_POS_HOLD_TAU_S = 6.0f;

// スティック → 目標速度 [m/s] (全開で)
constexpr float FLOW_STICK_VEL  = 1.0f;
constexpr float FLOW_STICK_DEAD = 0.05f;    // これ以下は「手を離した」と判定

// リーン角の出力上限 [deg] (レートループが追える範囲で小さく)
constexpr float FLOW_MAX_LEAN   = 8.0f;

// 目標リーン角の符号。手で押して「押し返す」向きにならなければ反転する。
//  vx(前進ドリフト) を止めるには機首上げ(pitch +)、vy(右ドリフト) を止めるには左バンク(roll -)。
constexpr float FLOW_LEAN_SIGN_ROLL  = -1.0f;
constexpr float FLOW_LEAN_SIGN_PITCH = +1.0f;

// スティックの符号 (プロポに合わせる)。ピッチを引いて機体が「後退」する向きが正。
constexpr float FLOW_STICK_SIGN_X = -1.0f;  // pitch stick → 前後 目標速度
constexpr float FLOW_STICK_SIGN_Y = -1.0f;  // roll  stick → 左右 目標速度

// 制御に使う速度の LPF (0=なし, 1に近いほど強い)
constexpr float FLOW_VEL_MEAS_ALPHA = 0.4f;

// |速度| がこれを超えたら異常値。0.5秒続いたらリーン0(水平)に固めて失探とみなす [m/s]
constexpr float FLOW_VEL_SANE = 3.0f;

// このスロットル以上で「浮いている」とみなしフロー制御を有効化
constexpr float FLOW_ENABLE_THR = 0.15f;

// ============================================================
//  § 7-3  距離センサ   ※ Stage 5 s5c で使用
// ============================================================
//  バックエンドを2種類から選べる (RANGE_BACKEND):
//
//   A) ToF_VL53L1X : I2C(Wire) ToF。IMU(MPU6050 @0x68) と同じバス共有。
//        Teensy 4.0 の Wire ピン SDA=18 / SCL=19。アドレス 0x29 固定で
//        MPU6050 と重ならないので XSHUT 不要。配線 4本:
//          VIN→3V3   GND→GND   SDA→18   SCL→19
//        (Teensy 4.0 は 5V 非対応。ブレークアウトのレギュレータ経由で 3V3)
//
//   B) Sonar_EZ    : MaxBotix LV-MaxSonar-EZ 超音波。PW(パルス幅)出力を
//        割り込みで測る。I2C を使わないので ToF のバス問題を回避できる。
//        配線 3本:
//          +5 → 3V3   (LV版は 2.5〜5.5V 可。3V3 給電なら PW も 0〜3V3 で
//                      Teensy に直結して安全。★5V 給電時は PW に分圧必須)
//          GND → GND
//          PW  → RANGE_SONAR_PW_PIN (下)   ※ TX/RX/AN/BW は未接続でよい
//        注意: 最短 ~0.16m 以下は測れない / ~20Hz / ビーム広め /
//              プロペラ後流・モーター音に弱い。真下向き・後流回避・防振。
//
//  【s5c でやること】プロペラを外し、drone_s5 を書き込んでシリアル表示を見る:
//   1. 機体を手で持ち上げ下げ → [距離] の h[m] が実測とほぼ一致するか
//      (定規で 0.3 / 0.5 / 1.0 m を実測して突き合わせる)
//   2. ずれる場合は RANGE_OFFSET_M で平行移動を合わせる
//      (センサレンズと機体最下端/フローレンズの高さ違いを吸収)
//   3. 機体をゆっくり上下 → climb[m/s] の符号が「上昇で +」か確認
//   4. その場で傾ける → RANGE_TILT_LIMIT_DEG を超えると h が凍結するのを確認

// 使う測距バックエンド。ハードを載せ替えたらここだけ変える。
enum class RangeBackend { ToF_VL53L1X, Sonar_EZ };
constexpr RangeBackend RANGE_BACKEND = RangeBackend::Sonar_EZ;

// --- Sonar_EZ (MaxBotix LV-MaxSonar-EZ) 専用 ---
constexpr uint8_t  RANGE_SONAR_PW_PIN   = 23;     // PW 出力 → 空き割り込みピン
constexpr float    RANGE_SONAR_ALPHA    = 0.35f;  // 生距離(EZ2側)の軽い1次LPF
constexpr float    RANGE_SONAR_MIN_M    = 0.16f;  // これ以下は測れない(仕様)
constexpr float    RANGE_SONAR_MAX_M    = 4.00f;  // LV-EZ の実用上限あたり
constexpr uint32_t RANGE_SONAR_STALE_MS = 300;    // 新パルスがこの時間来なければ失探

// 測距の1回あたり積分時間 [us]。長いほど精度↑/レート↓。
//  Medium モードは 33ms 前後が下限。
constexpr uint32_t RANGE_TIMING_BUDGET_US = 33000;
// 連続測距の周期 [ms] (timing budget 以上にする)
constexpr uint16_t RANGE_CONTINUOUS_MS    = 33;

// 読み出しポーリングレート [Hz]。1000Hz メインから間引く。
//  センサ側が ~30Hz なので、それより速く回して dataReady() で拾えばよい。
constexpr int RANGE_LOOP_HZ = 50;
static_assert(RATE_LOOP_HZ % RANGE_LOOP_HZ == 0,
              "RANGE_LOOP_HZ は RATE_LOOP_HZ の約数にしてください");

// 有効とみなす斜め距離のレンジ [m] (これ外は外れ値として捨てる)
constexpr float RANGE_MIN_M = 0.03f;
constexpr float RANGE_MAX_M = 3.5f;

// 高度に足す平行移動オフセット [m] (センサレンズ → 基準高さ の差。上に付いていれば +)
constexpr float RANGE_OFFSET_M = 0.00f;

// この傾きを超えている間は測距点が横にずれるので高度を凍結する [deg]
constexpr float RANGE_TILT_LIMIT_DEG = 25.0f;

// 高度 LPF (0=なし, 1に近いほど強い) と、上昇速度(高度の微分)の LPF
constexpr float RANGE_H_ALPHA  = 0.30f;
constexpr float RANGE_VZ_ALPHA = 0.20f;

// 外れ値がこの時間続いたら「失探」= valid() を false に落とす [ms]
//  失探中は OpticalFlow は FLOW_ASSUMED_HEIGHT_M に自動フォールバック。
constexpr uint32_t RANGE_FAULT_MS = 1000;

// ============================================================
//  § 7-4  s5c : 高度ホールド (スロットルPID)
// ============================================================
//  構造 (カスケード。位置ホールドと同じ考え方):
//    高度誤差[m] ──[ALT_POS_KP]──► 目標上昇速度[m/s] ──[ALT_RATE PID]──►
//        スロットル補正[割合] ──► (係留時に掴んだホバースロットル) + 補正
//
//  ★★ これまで「スロットルは全モード手動」が安全上の不変条件だった。
//     s5c ではこのループを足す。実飛行では PC を繋げないので、ON/OFF は
//     シリアル 'g' ではなく SW_AUTO スイッチで行う
//     (drone_s5.cpp の S5::ALT_HOLD_ON_SW_AUTO = true)。
//       SW_HOVER=up  → POSHOLD (水平ホールド)。ここまではスロットル手動。
//       + SW_AUTO=up → 高度ホールドも ON。
//       SW_AUTO=down → 高度だけ手動に戻る (水平ホールドは維持)。
//       SW_HOVER=down→ ANGLE + 完全手動 (最終 bail-out)。
//
//  ★ ホバースロットルの基準:
//     ALT_HOVER_THR (>0) にあらかじめ実測値を入れておく。これが
//     基準スロットルになり、PID はそこから ±ALT_THR_AUTH だけ動かす。
//     ALT_HOVER_THR = 0 のままだと高度ホールドは engage しない (安全)。
//     ※ 実測方法: ANGLE で安定ホバリングさせ、そのときのスロットル
//        スティック位置 (プロポの%) を読む。0.45〜0.55 が一般的。
//
//  ★ 保持高度: ALT_TARGET_M。0 なら「engage した瞬間の実測高度」。
//     飛行中はスロットルスティックを基準から動かすと上昇/下降速度指令、
//     戻すとその高度を保持する。

// 外側 (位置): 高度誤差[m] → 目標上昇速度[m/s]。kp=1 なら 1m ずれで 1m/s。
constexpr float ALT_POS_KP     = 2.0f;
constexpr float ALT_POS_VZ_LIM = 0.4f;     // 位置ループが出す上昇速度の上限 [m/s]

// 内側 (速度): (目標 - 実測 上昇速度[m/s]) → スロットル補正[割合]
//  kp=0.25 なら「0.4 m/s ずれていたら +0.10 スロットル」
constexpr float ALT_RATE_KP      = 0.25f;
constexpr float ALT_RATE_KI      = 0.15f;   // ホバースロットルの誤差を吸収
constexpr float ALT_RATE_KD      = 0.02f;
constexpr float ALT_RATE_I_LIMIT = 0.15f;   // I項の上限 [割合]
constexpr float ALT_RATE_D_ALPHA = 0.60f;

// PID がホバースロットルから動かしてよい最大量 [割合]。
//  ブリングアップ中は余裕を持って 0.30。挙動が信用できたら 0.20 に絞る。
constexpr float ALT_THR_AUTH = 0.30f;

// ホバースロットルの基準 [割合]。★実飛行前に ANGLE ホバリングで実測して入れる。
//  0 のままだと高度ホールドは engage しない (SW_AUTO を上げても効かない)。
constexpr float ALT_HOVER_THR = 0.0f;

// 保持したい対地高度 [m]。0 = engage した瞬間の実測高度をそのまま目標にする。
constexpr float ALT_TARGET_M  = 0.5f;

// スロットルスティックを基準 (ALT_HOVER_THR) からこれだけ動かしたら「上昇/下降指令」 [割合]
constexpr float ALT_STICK_DEAD = 0.08f;
// スティック偏差 1.0 あたりの上昇速度指令 [m/s]
constexpr float ALT_STICK_VZ   = 0.8f;

// このスロットル以上で高度ホールドを有効化 (地上での暴走防止)
constexpr float ALT_ENABLE_THR = 0.15f;

} // namespace Quad
