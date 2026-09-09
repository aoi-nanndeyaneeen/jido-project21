// ============================================================
//  drone_s5.cpp  -  Stage 5 : オプティカルフロー / 距離センサ / 自動ホバリング
// ============================================================
//  Stage 4 (drone_s4.cpp) の角度PID + モード切替 + ヘディングホールドを
//  そのまま土台にして、位置・高度の保持を段階的に足していく。
//
//  【このファイルの現在地: s5d — 1スイッチ完全自動ホバリング】
//    ・s5a (PMW3901 の符号/スケール/de-rotation のベンチ確定) 完了。
//    ・s5b (フロー速度・位置ホールド) 完了。調整値は QuadConfig.h § 7 / § 7-2。
//    ・s5c (対地距離センサ + 高度ホールド) 完了。§ 7-3 / § 7-4。
//        測距は RANGE_BACKEND で ToF(VL53L1X, I2C) / SONAR(MaxBotix EZ, PW) を選択。
//        傾き cos 補正した鉛直高度を毎ループ flow.setHeight() へも渡す
//        (失探したら FLOW_ASSUMED_HEIGHT_M へ自動フォールバック)。
//    ・s5d で足したもの:
//        1. 運用を SW_HOVER 1本だけで決める。
//           ★ 2026-09-05: down/cen 兼用の2段階から3段階に変更。
//             down → ANGLE   完全手動 (スロットルも手動)。これが bail-out。
//             cen  → ALTHOLD 姿勢は手動、高度だけ自動保持 (水平位置はまだ手動)。
//             up   → POSHOLD 完全自動 (水平位置 + 高度 + ヘディングを同時)。
//                    フロー喪失時は ALTHOLD へ自動フォールバック。
//           SW_AUTO / 地上局AUTO / RATE は封印。実飛行では PC を繋げないので
//           シリアル 'g' 等に依存しない運用にしてある。
//        2. 位置積分を「地面固定フレーム (N/E)」化。s5c までは機体座標のまま
//           積分していたので、機体がヨーすると保持基準ごと回って位置が流れた。
//           ヘディング g_yaw_est で回してから積分し、位置ループの出力を
//           機体座標へ戻して速度PIDへ渡す。
//        3. フロー水平ホールドを quad/PosHold.h (Q::PositionHold) に切り出し。
//           速度LPF / 失探検出 / 位置積分 / 位置ループ / 速度PID / スティック
//           処理を1クラスに集約し、drone_s5.cpp 側のグローバルを一掃した。
//    ★ 高度ホールドの基準スロットルは Q::ALT_HOVER_THR (実測値)。0 のときは
//      POSHOLD 突入時のスティック値を掴むので、ANGLE で安定ホバリングしてから
//      SW_HOVER を上げること。POSHOLD 中に測距だけ失った場合はスティック値へ
//      飛ばさず直前のホバースロットルを保持する (落下防止)。
//    ・調整は QuadConfig.h § 7-2 / § 7-4 と シリアル 'p' メニュー
//      ([i][j][o] = フロー水平, [s][t][u][v] = 高度)。
//    ★ 初回は広い床で、指をモードスイッチに。符号ミス = 即壁/天井行き。
//    ★ 地上確認は 'm' のドライラン (ESCへ0のみ / g_out は計算・表示) を使う。
//      フロー制御は「アーム && POSHOLD && スロットル > FLOW_ENABLE_THR」でしか
//      出力しないので、ドライラン中もスティックを 35% 程度まで上げること。
//
//  ------------------------------------------------------------
//  以下、Stage 4 から引き継いだ説明:
//
//  【前提】Stage 3 のレートPIDが飛べる状態になっていること。
//    角度ループは、レートループが安定していない限り絶対に安定しません。
//    Stage 3 で機体がふらつくなら、ここへ来ても悪化するだけです。
//
//  【制御の構造】カスケード
//
//    ANGLE モード:
//      スティック → 目標角度[deg] → 角度PID(200Hz) → 目標角速度[deg/s]
//                                 → レートPID(1000Hz) → トルク指令 → ミキサー
//    RATE モード:
//      スティック → 目標角速度[deg/s] → レートPID(1000Hz) → ...
//
//    ヨーはスティックを触っている間だけレート指令です。
//    中立に戻すと「ヘディングホールド」に切り替わり、そのときの機首方位を
//    保持します (§ 7-2 を参照)。
//
//  【モード】
//    SW_HOVER が UP  → ANGLE (手を離すと水平に戻る)
//    それ以外        → RATE  (アクロ。Stage 3 と同じ挙動)
//
//    旧コードの MODE_LEVEL_TURN / MODE_LEVEL_FLIGHT は固定翼用なので
//    削除しました (クアッドで 25度バンクを保持し続けるのは危険です)。
//    MODE_MANUAL (PIDを通さない素通し) も、クアッドでは意味がないので
//    削除しています。
//
//  【旧コードから直したところ】
//    ・角度PIDの dt を、実際の呼び出し周期 (200Hz) で渡すようにした
//      (旧: 200Hz で呼びながら dt は 1ms 固定で、I項とD項が5倍ずれていた)
//    ・SEMI_MANUAL の単位不整合を解消
//      (旧: 目標「角度」30deg を、そのままレートPIDの目標「角速度」に渡していた)
//    ・モード遷移時に目標値もクリアするようにした
//      (旧: pid_reset() は呼ぶが tar は残るので、
//       LEVEL_FLIGHT の前傾10度が次のモードに持ち越されていた)
//    ・ヨーのレートPIDに I項を入れ、ヘディングホールドを追加した (§ 7-2)
//      (旧: ヨーは kp のみ。P制御は定常偏差を消せないので、モーター取付角の
//       わずかな傾きやプロペラの個体差が作る一定のヨートルクに負けて、
//       機首がじわじわ回り続けていた)
// ============================================================

#include <Arduino.h>
#include <math.h>

#include "Config.h"
#include "Actuators.h"
#include "Receiver.h"
#include "sensor/IMU.h"   // 気圧・超音波は使わないので Sensors.h ごとは読まない
#include "sensor/OpticalFlow.h"  // PMW3901 (SPI)
#include "sensor/Rangefinder.h"  // 測距 (QuadConfig の RANGE_BACKEND で ToF/SONAR 切替)
#include "Telemetry.h"    // IM920SL (地上局との送受信)
#include "S5Telem.h"     // s5 解析用テレメトリ (POSHOLD を地上局でCSV化)

#include "quad/QuadConfig.h"
#include "quad/QuadPID.h"
#include "quad/Mixer.h"
#include "quad/BodyFrame.h"
#include "quad/PosHold.h"
#include "quad/AltHold.h"        // s5c: 高度ホールド (カスケード + engage 状態遷移)
#include "quad/AltEstimator.h"   // s5c: 加速度Z×測距の相補フィルタ (高度・上昇速度の推定)
#include "quad/SdLog.h"          // 飛行まるごとSDへストリーミング (HW-125, CS=9)
#include "quad/FlightLog.h"      // ログ1行の定義/整形 + USBストリーム + RAMリング

namespace Q = Quad;

// ============================================================
//  § 1  ゲイン
// ============================================================
namespace Gain {

// ---- レート (内側ループ) ----  Stage 3 で決めた値をここに写す
// ★ I項について (棒・紐での調整では 0 のままだった理由と、入れた理由)
//   支点のあるリグでは、機体に残る一定トルクを支点が受け止めてしまうので
//   定常偏差が現れない。フリーフライトでは受け止めるものが無く、
//   残留トルクはそのまま姿勢の定常偏差になる。
//
//   P だけのカスケードの定常状態 (角速度 0、姿勢一定) は
//     rate_tar = d / kp_rate,   rate_tar = -kp_ang * theta
//   なので   theta = -d / (kp_rate * kp_ang) = -250 * d
//   となり、わずか 1% の差動推力 (d=0.01) でも 2.5deg 傾いたまま釣り合う。
//   2.5deg は横加速度 0.43 m/s^2 で、当然その方向へ流れていく。
//   CG のずれ / モーターのKV差 / プロペラ個体差では 1% は簡単に出る。
//
//   ki = kp / Ti (このPIDは _i_term += ki*error*dt なので Ti = kp/ki)。
//   0.0005 は積分時定数 2 秒相当。戻りが遅ければ 0.0010 (Ti=1秒) まで上げてよい。
//   上げすぎると 0.5〜2Hz のゆっくりした揺れが出る。
//
//   ★ 積分は角度ループには入れない (ANG_* の ki は 0 のまま)。
//     両方に入れると I 同士が干渉して低周波で揺れる。
//
//   ★ 安定ホバリング中の I項の収束値 = 機体の機械的非対称量。
//     画面の [レートループ] I項 の列で読める。
//       ±0.01 程度 : 正常 / ±0.05 以上 : アーム・モーター取付角を疑う
//       RATE_I_LIMIT に張り付く : I項では吸収しきれない。機械を直すこと
//                                kp       ki       kd
// ★ 2026-09-04: 0.0010 -> 0.0020 に戻した。
//   s5_008 の離陸直後 (t=8.74) で角速度誤差 -337 deg/s に対し
//     cmd = 0.0010 * 337 = 0.337  (実測 -0.3491 と一致)
//   ミキサーが出せる純ピッチ指令の上限は span_limit/2 = (1-THR_IDLE)/2 = 0.45。
//   つまり 248 deg/s で回転しているのに権限の 78% しか使えていなかった。
//   0.0020 なら上限に張り付き、フル権限で止めにかかる。
//   「s5 に入ってすぐの頃のほうがよく飛んだ」という体感とも一致する
//   (その頃の値が 0.0020)。
// ★ 2026-09-04: ki を 0 -> 0.0020 に。
//   重心のずれ・モーター取付角・プロペラの個体差は「一定のトルク外乱」。
//   P だけのループは一定外乱を定常偏差でしか釣り合わせられない
//   (角度ループも ki=0 なので、系全体に積分器が1つも無かった)。
//   つまり「後ろに流れる」のを制御で消す手段が存在しなかった。
//   それを消すのが I 項。ki = kp は積分時定数 1秒に相当する。
//   ワインドアップ対策は既にある: integrate = (thr > I_ENABLE_THR) と
//   RATE_I_LIMIT = 0.15 (出力の 15% で頭打ち)。
// ★ 2026-09-09: ki を 0.0000 -> 0.0020 にしたが、同じコミットで ANG ki=0 と
//   RATE_D_ALPHA=0.95 も同時に変えた結果、LOG0057 で roll gyro_rms が
//   17 -> 97 dps、レート追従相関 +0.49 -> +0.10 に崩壊した (元々 roll は
//   一番安定していた軸)。LOG0054 は「旧ゲインのまま同じ非対称機で 15 秒
//   ホバーできていた」ので、3点同時変更が回帰だった。飛んでいた値に戻す。
//   → 積分の置き場所 (レート vs 角度) や D フィルタは、M1/M4 非対称を
//     直した機体で 1 つずつ検証すること。
constexpr float RATE_ROLL [3] = { 0.0015f, 0.0000f, 0.00004f };
constexpr float RATE_PITCH[3] = { 0.0015f, 0.0000f, 0.00004f };

// ★ ヨーの I項。P制御だけでは定常偏差が残る。
//   機体には必ず一定のヨートルクが残っている:
//     ・モーター取付角のわずかな傾き (推力ベクトルが真上を向いていない)
//     ・CW/CCW プロペラの特性差、モーターのKV差、ESCの個体差
//   これらは P では釣り合った角速度で回り続けるだけで、消えない。
//   ki = kp は積分時定数 1秒に相当する。まずこの値で試し、
//   戻りが遅ければ 0.005 まで上げてよい (上げすぎると 1Hz 前後で揺れる)。
// ★ 2026-09-05: kp を 0.0010 -> 0.0015 に。s5 立ち上げ時のコミット(s5d追加)で
//   s4 の 0.0020 から無言で半分に落とされていた (理由の記載なし)。s4/s5 で
//   ANGLE モードの挙動が違って感じるという指摘を受けて洗い出した差分の一つ。
//   s4 と揃えて 0.0020 に戻す案もあったが、まずは中間の 0.0015 で様子を見る。
constexpr float RATE_YAW  [3] = { 0.0015f, 0.0020f, 0.00004f };

// ★ 2026-09-09: 一度 0.80 -> 0.95 にしたが (LOG0054 の 17Hz リミットサイクル
//   対策)、LOG0057 で roll のレートループが発散気味になった。a=0.95 は
//   遮断 ~8Hz で、ループのクロスオーバー帯 (1〜5Hz) に D 項の位相遅れが
//   入り、D が「制動」ではなく「加振」に回ったと見られる。0.80 に戻す。
//   17Hz リミットサイクルは本物だが、LOG0054 は 15 秒ホバーできていたので
//   優先度は低い。潰すなら kd を 0.00004 -> 0.00002 に半減する方が安全
//   (位相を触らずゲインだけ下げる)。まず M1/M4 非対称を直してから。
//   参考: LOG0054 の 17Hz 内訳 = D 寄与 0.067 / P 寄与 0.023 (D が 74%)。
constexpr float RATE_D_ALPHA = 0.80f;
constexpr float RATE_I_LIMIT = 0.15f;

// ---- ヘディングホールド (ヨー) ----
//  ジャイロZを積分した「相対」方位を保持する。絶対方位 (磁気センサ/GPS) は
//  使わない。市販のフライトコントローラのヨーもこれと同じ仕組み。
//
//  ・ドリフトは無関係: 積分値がどれだけずれても「今向いている方向」を
//    保ち続けることに変わりはない
//  ・±180度の折り返し処理も不要: 相対値なので連続量のまま扱える
//
//  Madgwick の getYaw() は使わない。あれは updateIMU() (6軸) なので
//  ヨーに補正項が一切入らず、結局ジャイロの純積分でしかない上に、
//  制御に使うジャイロ値と位相がずれる。

// 方位誤差 1度あたり何 deg/s で戻すか [(deg/s)/deg]
constexpr float YAW_HOLD_KP       = 3.0f;
// ヘディングホールドが出してよい角速度の上限 [deg/s]
constexpr float YAW_HOLD_RATE_LIM = 60.0f;
// これ以上の方位誤差は追わない [deg]
//  大きく振られたときに全力で振り戻すと危ないので、ここで頭打ちにする。
constexpr float YAW_HOLD_ERR_LIM  = 20.0f;
// ラダースティックの不感帯。これを超えたら「操作中」と判定する
constexpr float YAW_STICK_DEAD    = 0.03f;

// ---- 角度 (外側ループ) ----
//  出力の単位は [deg/s] です。kp = 4.0 なら「10度傾いていたら 40deg/s で戻す」。
//  まず kp だけで調整し、ki / kd は基本 0 のままで構いません。
//  ★ 積分はレートループ (RATE_*) 側だけで持たせています。ここに ki を
//    入れるとレートループの I項と干渉して低周波の揺れが出ます。0 のままに。
//                               kp     ki    kd
// ★ 2026-09-09: 一度 ki 0.04 -> 0 にしたが、RATE ki と D_ALPHA も同時に
//   変えたため回帰の切り分けができなくなった (LOG0057)。飛んでいた
//   LOG0054 の値 (0.04) に戻す。pitch の 0.7〜1.3Hz 揺れ (pitch_ang rms
//   4.4deg / 追従誤差 roll の 4倍) は本物なので、非対称を直したあと
//   「ANG ki だけ」を 0.04 -> 0 にして 1 本録って検証すること。
constexpr float ANG_ROLL [3] = { 30.0f, 0.04f, 0.0f };
constexpr float ANG_PITCH[3] = { 30.0f, 0.04f, 0.0f };

constexpr float ANG_D_ALPHA = 0.70f;
// 角度ループの積分項の上限 [deg/s]
constexpr float ANG_I_LIMIT = 30.0f;

} // namespace Gain

// ============================================================
//  § 2  このステージの設定
// ============================================================
namespace S5 {

constexpr bool USE_MPU   = true;
constexpr bool USE_SBUS  = true;
constexpr bool USE_MOTOR = true;
constexpr bool USE_IM920 = true;   // 地上局(position_estimator)との無線
constexpr bool USE_FLOW  = true;   // PMW3901 オプティカルフロー
constexpr bool USE_RANGE = true;   // s5c: VL53L1X 測距 (高度を flow へ供給)

// ★ SD ログ (HW-125) と PMW3901 は SPI0 を共有していて共存できない
//   (HW-125 クローンの 74LVC125 が MISO を Hi-Z にせず、フローを潰す)。
//   なので「フローを使うときは SD を切り、RAM ログで録る」運用にする。
//   USE_SD を明示的に持ち、既定を !USE_FLOW にしておけば、USE_FLOW を
//   切り替えるだけで自動的に排他になる。
//   ・USE_FLOW=true  → USE_SD=false : ログは RAM ('n'手動 / スロットルで自動) と
//                                     USB 'l'。8秒制限あり
//   ・USE_FLOW=false → USE_SD=true  : ログは SD (飛行まるごと、制限なし)
//   どうしても両方 true にしたい (別 SPI に逃がした等) ときは下を手で書き換える。
constexpr bool USE_SD = !USE_FLOW;
static_assert(!(USE_FLOW && USE_SD),
              "USE_FLOW と USE_SD は同時に true にできない (SPI0 共有)。"
              "別バスに分けたなら、この static_assert を消して自己責任で。");

// s5c: 高度ホールド (スロットルPID)。
//  ★ 運用は2モードのみ:
//      SW_HOVER=down → ANGLE   … 完全手動 (スロットルも手動)。bail-out はこれ。
//      SW_HOVER=up   → POSHOLD … 完全自動 (水平位置ホールド + 高度ホールド)。
//    POSHOLD に入れば高度ホールドは自動で effective。専用スイッチは不要。
//    基準ホバースロットルは Q::ALT_HOVER_THR (実測値)。0 のときは POSHOLD
//    突入時のスロットルスティック値を掴む → ANGLE で安定ホバリングしてから
//    SW_HOVER を上げること。
//  ・USE_ALT_HOLD = false にすると、POSHOLD でも高度は手動のまま
//    (フロー水平ホールドだけ効かせたいとき)。シリアル 'g' でも切替可 (ベンチ用)。
constexpr bool USE_ALT_HOLD = true;

// ★ ドライラン (地上検証用)。true / シリアル 'm' で ON にすると:
//     ・制御パイプラインは通常どおり全部回る (mix() まで計算する)
//     ・ESC へは 0 (アイドル) しか送らない = プロペラが付いていても回らない
//     ・g_out[] (各モーターの「出すはずだった値") はログと printStatus に出る
//   プロペラを外さずに、傾けたときの各モーター配分の符号を確認できる。
//   ※ アーム状態でも一切回らないので、飛ばす前に必ず OFF に戻すこと。
constexpr bool DRY_RUN = false;

// s5b: SW_AUTO UP のとき、地上局AUTO ではなく「フロー位置ホールド」に入る。
//  地上局(position_estimator)を使う構成に戻すときは true にする。
constexpr bool USE_GROUND_AUTO = false;

// 運用は2モード固定。SW_HOVER の1本だけで決める (selectMode 参照):
//     SW_HOVER UP   → POSHOLD (完全自動。フローが死んでいたら ANGLE に自動フォールバック)
//     SW_HOVER 以外 → ANGLE   (完全手動。bail-out)
//   RATE(アクロ) と SW_AUTO/地上局AUTO は封印。どちらの位置も自己水平は効く。
constexpr bool POSHOLD_ON_HOVER_SW = true;  // (歴史的フラグ。今は常に true 運用)

constexpr float I_ENABLE_THR = 0.15f;

// s5 解析テレメトリの送信レート [Hz]。
//  ★ 2026-09-04 実機実測: IM920sL が持続できるのは 15Hz まで。
//       10Hz 欠落0% / 12Hz 0% / 15Hz 0% / 20Hz 5% / 25Hz 22.5% (≒19Hzで頭打ち)
//     1パケット32B = "TXDA "+64桁+CRLF = 71文字 = 19200bps で 37ms。
//     15Hz で UART 占有率 55%。
//  ★ A(高度)/B(水平) を交互に送るので、各フレームは実質 7.5Hz。
//     0.3〜2Hz の高度・位置ループを見るには足りる。
//  地上局の欠落率が上がるようなら 12 -> 10 と下げる。
constexpr int TELEM_TX_HZ = 15;

// ゲイン一覧 (S5T::Param) を送る周期 [秒]。
//  シリアル 'p' メニューで飛行中にゲインを変えられるので、CSV だけ見て
//  「どのゲインの結果か」が分かるように定期的に混ぜる。
//  この回だけ State を1つ落とすので、あまり短くしないこと。
constexpr uint32_t TELEM_PARAM_MS = 5000;

// 受信のポーリングレート [Hz]
//  IM920SL は 19200 baud なので 200Hz でも 1回あたり最大12バイト程度。
//  Teensy の受信バッファ(64B)を溢れさせずに、1000Hz の制御ループから
//  String 操作を追い出せる。地上局からの更新は10Hz程度なので十分。
constexpr int TELEM_RX_HZ = 200;

// 角度ループが出せる角速度の上限 [deg/s]
// (大きく傾いたときに、レートループが追えない目標を出さないための蓋)
constexpr float ANGLE_OUT_LIMIT = 300.0f;

constexpr int MAIN_HZ  = Q::RATE_LOOP_HZ;
constexpr int DEBUG_HZ = Q::DEBUG_HZ;

//  ★ 2026-09-05: SW_HOVER を3段階で使うように変更 (旧: down/cen どちらも
//    ANGLE 扱いで実質2段階だった)。
//  MODE_ANGLE    : 完全手動 (自己水平のみ、高度もスティック直結)。SW_HOVER=down。bail-out。
//  MODE_ALTHOLD  : 姿勢は手動 (ANGLEと同じ) + 高度だけ自動保持。SW_HOVER=cen(中央)。
//    水平位置(フロー)ホールドはまだ効かない。ALTHOLDとPOSHOLDの中間の練習用。
//  MODE_POSHOLD  : 完全自動。フローで水平位置保持 + 測距で高度保持。SW_HOVER=up。
//    センサ喪失の瞬間に ANGLE へ自動フォールバック。
//  MODE_RATE / MODE_AUTO : 封印 (enum は互換のため残置。selectMode は返さない)。
enum Mode : uint8_t { MODE_RATE = 0, MODE_ANGLE = 1, MODE_AUTO = 2, MODE_POSHOLD = 3,
                       MODE_ALTHOLD = 4 };

} // namespace S5

// ============================================================
//  § 3  インスタンス
// ============================================================
IMU   mpu(&Wire);
Sbus  sbus(&Serial5);
motor motors[Q::MOTOR_COUNT];

// IM920SL。drone.cpp と同じ Serial3 に合わせてある。
// 配線が違う場合はここを変えること。
FlightTelemetry telemetry(&Serial3);

// s5 解析テレメトリの送信側。Serial3 は telemetry.begin() が開くので、
// こちらは begin() を呼ばない (同じポートを二重に開かない)。
// ★ 送信は必ず非ブロッキング。s5tx.service() を毎ループ回すこと。
S5T::Tx s5tx(&Serial3);

// PMW3901 オプティカルフロー。CS ピンは QuadConfig.h の FLOW_CS_PIN。
// グローバル SPI (Teensy 4.0: SCK=13 MOSI=11 MISO=12) を使う。
OpticalFlow flow;

// s5c: 対地距離センサ。QuadConfig の RANGE_BACKEND で ToF(VL53L1X, I2C) と
//  SONAR(MaxBotix LV-MaxSonar-EZ, PW=pin RANGE_SONAR_PW_PIN) を切り替える。
Rangefinder rangefinder;

// s5b/s5d: フロー水平ホールド。速度ループ・位置ループ・地面固定フレーム積分を
//  すべて内包する (quad/PosHold.h)。出力は leanRoll()/leanPitch() [deg]。
Q::PositionHold poshold;

// s5c: 高度ホールド。位置ループ・上昇速度PID・engage 状態遷移をすべて内包する
//  (quad/AltHold.h)。出力は active() / throttle()。
Q::AltitudeHold althold;

//  s5c: 高度推定 (加速度Z × 測距)。ALT_USE_ACC_FUSION=false の間はログ専用。
Q::AltEstimator altest;

Q::Axis roll_axis, pitch_axis, yaw_axis;

// ============================================================
//  § 4  状態
// ============================================================
namespace {

Q::Attitude g_att;
float       g_out[Q::MOTOR_COUNT] = {0};

S5::Mode g_mode      = S5::MODE_ANGLE;
S5::Mode g_prev_mode = S5::MODE_ANGLE;
bool     g_prev_armed = false;
// アームの瞬間に SW_HOVER が ANGLE 側だったか (isArmed / armGateOk 参照)
bool     g_arm_latched = false;

// 角度ループの間引きカウンタと、実際の経過時間
int      g_angle_div_count = 0;
uint32_t g_angle_prev_us   = 0;

// ---- ヘディングホールド ----
float g_yaw_est   = 0.0f;   // ジャイロZを積分した相対方位 [deg]
float g_yaw_hold  = 0.0f;   // 保持したい相対方位 [deg]
bool  g_yaw_holding = false; // 表示用: いま保持中か

// シリアルから調整できるようにゲインだけ変数で持つ
float g_yaw_hold_kp = Gain::YAW_HOLD_KP;

// ---- ミキサーの報告 (ログ用) ----
Q::MixInfo g_mix;

// ---- オプティカルフロー観測 ----
bool  g_flow_ok    = false;   // PMW3901 が初期化できたか
float g_flow_raw_x = 0.0f, g_flow_raw_y = 0.0f;  // 機体座標の生カウント [px]
float g_flow_dx    = 0.0f, g_flow_dy    = 0.0f;  // de-rotation 後 [px]
float g_flow_vx    = 0.0f, g_flow_vy    = 0.0f;  // 対地速度 [m/s] (前 +, 右 +)
float g_flow_vx_f  = 0.0f, g_flow_vy_f  = 0.0f;  // 表示用にLPFした速度 (ログは生値)

// ---- s5b/s5d: フロー水平ホールドの状態は Q::PositionHold (poshold) が保持 ----
//  以前ここにあった g_flow_vx_ctl / g_pos_* / g_flow_lean_* などのグローバルは
//  すべて quad/PosHold.h の中に移した。参照は poshold.xxx() を使う。

// シリアルから調整するゲイン (QuadConfig.h の初期値から起動時にコピー)
float g_flow_vel_kp = Q::FLOW_VEL_KP;
float g_flow_vel_ki = Q::FLOW_VEL_KI;
float g_flow_pos_kp = Q::FLOW_POS_KP;

// ---- キャリブレーション用の積算 ----
//  既知距離をゆっくり動かし、acc_px と実測距離を突き合わせて
//  QuadConfig.h の FLOW_PX_PER_RAD を決める ([z] でゼロ)。
double g_flow_acc_raw_x = 0.0, g_flow_acc_raw_y = 0.0; // 生ピクセルの積算 (de-rotation 前)
double g_flow_acc_px_x  = 0.0, g_flow_acc_px_y  = 0.0; // de-rotation 後ピクセルの積算
double g_flow_acc_m_x   = 0.0, g_flow_acc_m_y   = 0.0; // 推定変位の積算 [m]

// ---- s5c: 距離センサ (VL53L1X) 観測 ----
bool  g_range_ok    = false;   // VL53L1X が初期化できたか
bool  g_range_valid = false;   // 高度が信用できるか (失探すると false)
float g_range_raw_m = 0.0f;    // 傾き補正前の斜め距離 [m]
float g_range_h_m   = 0.0f;    // 鉛直対地高度 [m] (flow.setHeight() に渡す値)
float g_climb_mps   = 0.0f;    // 上昇速度 [m/s] (上 +)

// ---- s5c: 高度ホールド ----
//  状態は Q::AltitudeHold (althold) が保持する。参照は althold.xxx()。
bool  g_alt_hold_enable = S5::USE_ALT_HOLD;  // シリアル 'g' でトグル
bool  g_range_fresh     = false;             // 今回 新しい測距が入ったか

//  ミキサーが実際に使ったスロットルの積算。1000Hz のミキサーと 100Hz の
//  高度ループをつなぐ。
//  ★ なぜ平均か: 機体の高度は「モーター出力の平均」に応答する。姿勢優先の
//    押し上げは姿勢の振動と同じ周波数 (log_037 では 14Hz) で出入りするので、
//    高度ループの周期で瞬時値を1点だけ拾うとエイリアシングする。
//    区間平均なら、機体が実際に受け取った推力と同じものを見られる。
float    g_thr_used_sum = 0.0f;
uint32_t g_thr_used_n   = 0;

// ドライラン: true の間は ESC へ 0 しか送らない (g_out は計算・記録する)
bool  g_dry_run = S5::DRY_RUN;

// ---- SD ログ (HW-125, CS=9, SPI0 を PMW3901 と共有) ----
bool  g_sd_ok              = false;   // SdLog::begin() が成功したか

} // anonymous namespace

// ============================================================
//  § 4-2  ログ 1 行の作成  (定義とフォーマットは quad/FlightLog.h)
// ============================================================
//  「今の全グローバル状態」を FlightLog::Rec に量子化して詰める。
//  この 1 個の Rec を loop() が USB / RAM / SD の 3 つのシンクへ配る。
//  drone_s5.cpp のグローバルを触るのはここだけなので、この関数だけが
//  FlightLog.h に入らず残っている。
//
//  ★ 列を足すときに触るのは、ここと FlightLog.h の Rec / HEADER /
//    formatRow、それに scripts/bin2csv.py の合計 5 箇所。
//    型か並びを変えたら FlightLog::REC_VER を +1 すること。
static void fillRec(FlightLog::Rec& r, uint32_t dt_us, int mode, bool armed, float thr) {
    r.t_ms = millis();
    r.dt_us = (uint16_t)constrain(dt_us, 0u, 65535u);

    uint16_t f = 0;
    if (armed)               f |= FlightLog::RF_ARMED;
    if (g_flow_ok)           f |= FlightLog::RF_FLOW_OK;
    if (g_range_ok)          f |= FlightLog::RF_RANGE_OK;
    if (g_alt_hold_enable)   f |= FlightLog::RF_ALT_EN;
    if (althold.active())    f |= FlightLog::RF_ALT_ACT;
    if (poshold.holding())   f |= FlightLog::RF_HOLDING;
    r.flags = f;

    r.mode   = (uint8_t)mode;
    r.mixsat = g_mix.sat;
    r.thr    = S5T::qu8(thr, 250.0f);

    r.roll_stick  = (int8_t)constrain(lroundf(roll_axis.stick  * 100.0f), -127L, 127L);
    r.pitch_stick = (int8_t)constrain(lroundf(pitch_axis.stick * 100.0f), -127L, 127L);
    r.yaw_stick   = (int8_t)constrain(lroundf(yaw_axis.stick   * 100.0f), -127L, 127L);

    r.roll_ang  = S5T::q16(roll_axis.ang_meas,  S5T::SC_CDEG);
    r.pitch_ang = S5T::q16(pitch_axis.ang_meas, S5T::SC_CDEG);
    r.yaw_est   = S5T::q16(g_yaw_est,           S5T::SC_CDEG);

    r.roll_rate  = S5T::q16(roll_axis.rate_meas,  S5T::SC_DDEG);
    r.pitch_rate = S5T::q16(pitch_axis.rate_meas, S5T::SC_DDEG);
    r.yaw_rate   = S5T::q16(yaw_axis.rate_meas,   S5T::SC_DDEG);

    r.roll_cmd = S5T::q16(roll_axis.cmd,  S5T::SC_1E4);
    r.pitch_cmd = S5T::q16(pitch_axis.cmd, S5T::SC_1E4);
    r.yaw_cmd   = S5T::q16(yaw_axis.cmd,   S5T::SC_1E4);

    r.m1 = S5T::qu8(g_out[0], 250.0f);
    r.m2 = S5T::qu8(g_out[1], 250.0f);
    r.m3 = S5T::qu8(g_out[2], 250.0f);
    r.m4 = S5T::qu8(g_out[3], 250.0f);

    r.span_limit = S5T::q16(g_mix.span_limit, 1000.0f);

    r.roll_ratetar  = S5T::q16(roll_axis.rate_tar,  S5T::SC_DDEG);
    r.pitch_ratetar = S5T::q16(pitch_axis.rate_tar, S5T::SC_DDEG);
    r.roll_angtar   = S5T::q16(roll_axis.ang_tar,   S5T::SC_CDEG);
    r.pitch_angtar  = S5T::q16(pitch_axis.ang_tar,  S5T::SC_CDEG);

    r.flow_raw_x = S5T::q16(g_flow_raw_x, 10.0f);
    r.flow_raw_y = S5T::q16(g_flow_raw_y, 10.0f);
    r.flow_dx    = S5T::q16(g_flow_dx,    10.0f);
    r.flow_dy    = S5T::q16(g_flow_dy,    10.0f);
    r.flow_vx    = S5T::q16(g_flow_vx, S5T::SC_MM);
    r.flow_vy    = S5T::q16(g_flow_vy, S5T::SC_MM);
    r.flow_h     = S5T::q16(flow.height(), S5T::SC_MM);
    r.flow_accx  = (float)g_flow_acc_m_x;
    r.flow_accy  = (float)g_flow_acc_m_y;

    r.fh_vxc = S5T::q16(poshold.vxCtl(), S5T::SC_MM);
    r.fh_vyc = S5T::q16(poshold.vyCtl(), S5T::SC_MM);
    r.fh_vxt = S5T::q16(poshold.vxTar(), S5T::SC_MM);
    r.fh_vyt = S5T::q16(poshold.vyTar(), S5T::SC_MM);
    r.fh_leanr = S5T::q16(poshold.leanRoll(),  S5T::SC_CDEG);
    r.fh_leanp = S5T::q16(poshold.leanPitch(), S5T::SC_CDEG);
    r.fh_posn  = S5T::q16(poshold.posN(),  S5T::SC_MM);
    r.fh_pose  = S5T::q16(poshold.posE(),  S5T::SC_MM);
    r.fh_holdn = S5T::q16(poshold.holdN(), S5T::SC_MM);
    r.fh_holde = S5T::q16(poshold.holdE(), S5T::SC_MM);

    r.range_raw = S5T::q16(g_range_raw_m, S5T::SC_MM);
    r.range_h   = S5T::q16(g_range_h_m,   S5T::SC_MM);
    r.climb     = S5T::q16(g_climb_mps,   S5T::SC_MM);

    r.alt_holdm     = S5T::q16(althold.holdM(),  S5T::SC_MM);
    r.alt_vzt       = S5T::q16(althold.vzTar(),  S5T::SC_MM);
    r.alt_corr      = S5T::q16(althold.thrCorr(), S5T::SC_1E4);
    r.alt_thr_out   = S5T::qu8(althold.thrOut(), 250.0f);
    r.alt_used      = S5T::qu8(g_mix.thr_used,   250.0f);

    // 2026-09-06: 加速度Z相補フィルタ検討用 (制御には未使用)。
    // ★ 2026-09-09: scale を SC_1E4(±3.27g で飽和) -> 1000(±32.7g) に。
    //   ±8g 化 + s_az_bias≈-2 で acc_z は [-10,+6] を取りうるため、
    //   従来スケールだと BIN 上で頭打ちしていた (REC_VER 2)。
    r.accx = S5T::q16(g_att.acc_x, 1000.0f);
    r.accy = S5T::q16(g_att.acc_y, 1000.0f);
    r.accz = S5T::q16(g_att.acc_z, 1000.0f);
    //  acc_up / est_bias は m/s^2。1e3 倍で ±32 m/s^2 まで入る。
    r.acc_up   = S5T::q16(altest.accUp(),    1000.0f);
    r.est_h    = S5T::q16(altest.heightM(),  S5T::SC_MM);
    r.est_vz   = S5T::q16(altest.climbMps(), S5T::SC_MM);
    r.est_bias = S5T::q16(altest.biasMps2(), 1000.0f);
}




// ============================================================
//  SelfTest  -  起動時に「どのデバイスがつながっているか」を 1 回だけ調べ、
//               結果を保持して 起動後も画面に残す
// ============================================================
//  setup() 末尾で probe() を呼ぶ。結果は:
//    ・その場で詳細ブロックを Serial に出す
//    ・printStatus() が毎フレーム 1 行 (compact) を出すので画面に残り続ける
//    ・シリアル 'd' で詳細ブロックを再表示できる
//  ※ IMU/測距/フロー/SD は「起動時に応答したか」の固定スナップショット。
//    SBUS/IM920 は起動時に信号が来ていたか。飛行中の生死は printStatus の
//    link= / IM920 link= 行が別に出す。
// ============================================================
namespace SelfTest {

enum St : uint8_t { ST_SKIP, ST_OK, ST_FAIL, ST_NOSIG };

struct Dev {
    const char* name;
    const char* bus;
    St          st;
    char        note[40];
};

// 並びはバス順 (I2C → SPI → UART)
Dev devs[] = {
    { "IMU MPU6050",  "I2C 0x68", ST_SKIP, "" },
    { "Rangefinder",  "I2C/PW",   ST_SKIP, "" },
    { "PMW3901 flow", "SPI CS10", ST_SKIP, "" },
    { "SD HW-125",    "SPI CS9",  ST_SKIP, "" },
    { "SBUS RX",      "Serial5",  ST_SKIP, "" },
    { "IM920",        "Serial3",  ST_SKIP, "" },
};
constexpr int N = sizeof(devs) / sizeof(devs[0]);
enum { D_IMU, D_RANGE, D_FLOW, D_SD, D_SBUS, D_IM920 };

inline const char* tag(St s) {
    switch (s) {
        case ST_OK:    return "OK ";
        case ST_FAIL:  return "FAIL";
        case ST_NOSIG: return "--  ";
        default:       return "skip";
    }
}

// 起動時に1回。ブロッキングで良い (まだ飛んでいない)。
inline void probe() {
    // --- IMU (I2C 0x68, WHO_AM_I) ---
    if (S5::USE_MPU) {
        devs[D_IMU].st = mpu.connected() ? ST_OK : ST_FAIL;
        if (devs[D_IMU].st == ST_FAIL)
            strcpy(devs[D_IMU].note, "応答なし SDA18/SCL19");
    }

    // --- 測距 (VL53L1X I2C 0x29 / MaxBotix PW) ---
    if (S5::USE_RANGE) {
        const bool sonar = (Q::RANGE_BACKEND == Q::RangeBackend::Sonar_EZ);
        devs[D_RANGE].bus = sonar ? "PW pin" : "I2C 0x29";
        devs[D_RANGE].st  = g_range_ok ? ST_OK : ST_FAIL;
        strcpy(devs[D_RANGE].note, sonar ? "MaxBotix EZ" : "VL53L1X");
    }

    // --- PMW3901 (SPI CS10) ---
    if (S5::USE_FLOW) {
        devs[D_FLOW].st = g_flow_ok ? ST_OK : ST_FAIL;
        if (devs[D_FLOW].st == ST_FAIL) strcpy(devs[D_FLOW].note, "応答なし");
    }

    // --- SD HW-125 (SPI CS9) ---
    devs[D_SD].st = g_sd_ok ? ST_OK : ST_FAIL;
    strcpy(devs[D_SD].note, g_sd_ok ? "" : "VCC=5V/FAT32/配線");

    // --- SBUS: ~250ms 受信機のフレームを待つ ---
    if (S5::USE_SBUS) {
        devs[D_SBUS].st = ST_NOSIG;
        const uint32_t t0 = millis();
        while (millis() - t0 < 250) {
            sbus.update();
            if (sbus.failCount() == 0) { devs[D_SBUS].st = ST_OK; break; }
        }
        if (devs[D_SBUS].st == ST_NOSIG)
            strcpy(devs[D_SBUS].note, "信号なし(受信機OFF/未接続?)");
    }

    // --- IM920: RDID を送って ~300ms 応答を待つ ---
    if (S5::USE_IM920) {
        while (Serial3.available()) Serial3.read();      // 掃除
        Serial3.print("RDID\r\n");
        String resp;
        const uint32_t t0 = millis();
        while (millis() - t0 < 300) {
            while (Serial3.available()) resp += (char)Serial3.read();
        }
        resp.trim();
        if (resp.length() > 0) {
            devs[D_IM920].st = ST_OK;
            resp.replace("\r", " "); resp.replace("\n", " ");
            snprintf(devs[D_IM920].note, sizeof(devs[D_IM920].note), "resp:%s", resp.c_str());
        } else {
            devs[D_IM920].st = ST_NOSIG;
            strcpy(devs[D_IM920].note, "無応答 (baud/配線/電源?)");
        }
        while (Serial3.available()) Serial3.read();      // 応答の残りを main ループへ持ち越さない
    }
}

// 詳細ブロック (setup 末尾 と 'd' で表示)
inline void printFull(Print& out) {
    out.println();
    out.println("=== 起動時デバイスチェック ===");
    for (int i = 0; i < N; ++i) {
        out.printf("  [%s] %-13s %-9s %s\n",
                   tag(devs[i].st), devs[i].name, devs[i].bus, devs[i].note);
    }
    out.println("  (IMU/測距/フロー/SD は起動時スナップショット。"
                "SBUS/IM920 の現在値は下の link= 行)");
    out.println("=============================");
}

// printStatus() の 1 行 (画面に残す用)
inline void printCompact(Print& out) {
    out.print("DEV(boot): ");
    for (int i = 0; i < N; ++i) {
        const char* s = devs[i].st == ST_OK ? "OK"
                      : devs[i].st == ST_FAIL ? "X"
                      : devs[i].st == ST_NOSIG ? "--" : "sk";
        // name の先頭語だけ短く出す
        char short_name[8];
        int k = 0;
        for (const char* p = devs[i].name; *p && *p != ' ' && k < 7; ++p) short_name[k++] = *p;
        short_name[k] = 0;
        out.printf("%s:%s ", short_name, s);
    }
    out.println();
}

} // namespace SelfTest

// ============================================================
//  StallLog  -  ループが異常に長くかかった回だけ、どの処理が原因かを
//               区間ごとの所要時間で記録する (原因調査用)
// ============================================================
//  I2Cバスの瞬断とみられる長時間停止 (実測 818ms〜4.3秒) が発生している。
//  I2Cdev::readTimeout短縮 (IMU.h) だけでは直らなかった。VL53L1Xは
//  I2Cdevを経由せず生のWire呼び出しを使っているため対象外だった可能性が
//  高いが、確証が無いので実際にどの区間で止まっているかを記録する。
//  'w' キーで USB へダンプ。
namespace StallLog {

constexpr uint32_t THRESHOLD_US = 5000;  // これを超えた回だけ記録
constexpr int       CAPACITY    = 32;    // 最初の32件だけ残す (以後は無視)

struct Rec {
    uint32_t t_ms;
    uint32_t total_us;
    uint32_t imu_us, sbus_us, flow_us, range_us, rx_us, ctrl_us;
};

DMAMEM Rec buf[CAPACITY];
int count = 0;

inline void maybeLog(uint32_t total_us, uint32_t imu_us, uint32_t sbus_us,
                      uint32_t flow_us, uint32_t range_us, uint32_t rx_us,
                      uint32_t ctrl_us) {
    if (total_us < THRESHOLD_US) return;
    if (count >= CAPACITY) return;   // 最初の数件が分かれば十分
    Rec& r = buf[count++];
    r.t_ms = millis();
    r.total_us = total_us;
    r.imu_us = imu_us; r.sbus_us = sbus_us; r.flow_us = flow_us;
    r.range_us = range_us; r.rx_us = rx_us; r.ctrl_us = ctrl_us;
}

inline void dump() {
    Serial.printf("StallLog: %d 件 (閾値 %lu us)\n", count,
                  (unsigned long)THRESHOLD_US);
    for (int i = 0; i < count; ++i) {
        Rec& r = buf[i];
        Serial.printf(" [%2d] t=%lums total=%luus  imu=%lu sbus=%lu flow=%lu"
                      " range=%lu rx=%lu ctrl=%lu (us)\n",
                      i, (unsigned long)r.t_ms, (unsigned long)r.total_us,
                      (unsigned long)r.imu_us, (unsigned long)r.sbus_us,
                      (unsigned long)r.flow_us, (unsigned long)r.range_us,
                      (unsigned long)r.rx_us, (unsigned long)r.ctrl_us);
    }
    if (count == 0) Serial.println("  (異常な停止は記録されていません)");
}

} // namespace StallLog

// ============================================================
//  § 4-3  s5 解析テレメトリ (IM920SL -> 地上局CSV)
// ============================================================
//  FlightLog::Usb の 500Hz USBログは「PCを繋いだ地上テスト」でしか取れない。
//  実飛行では USB が無いので、POSHOLD の解析に要る信号だけを 10Hz で
//  無線に落とし、地上局 (ground_receiver の env:xiao_s5_log) が CSV に
//  書く。列の意味と分解能は S5Telem.h を参照。
//
//  ★ 姿勢ループ (数十Hz) の解析には使えない。10Hz で見えるのは
//    高度ホールド / 位置ホールドの 1〜2Hz の挙動まで。それがまさに
//    今調整したいループなので、この帯域で足りる。
// isArmed() の実体は § 6。フラグを詰めるためにここで前方宣言しておく。
static bool isArmed();

namespace S5Tel {

uint8_t  seq           = 0;
uint32_t last_param_ms = 0;
bool     tx_drop_flag  = false;  // 直前の送信が捨てられたか (次パケットで通知)
uint8_t  slot          = 0;      // A/B/C の送信枠カウンタ (tick 参照)

// A/B 共通のヘッダを埋める。
inline void fillHeader(S5T::Header& h, uint8_t type) {
    h.type = type;
    h.seq  = seq++;

    uint16_t f = 0;
    if (isArmed())              f |= S5T::F_ARMED;
    if (g_flow_ok)              f |= S5T::F_FLOW_OK;
    if (g_range_ok)             f |= S5T::F_RANGE_OK;
    if (g_range_valid)          f |= S5T::F_RANGE_VALID;
    if (g_alt_hold_enable)      f |= S5T::F_ALT_EN;
    if (althold.active())       f |= S5T::F_ALT_ACT;
    if (poshold.holding())      f |= S5T::F_POS_HOLD;
    if (althold.airborne())     f |= S5T::F_AIRBORNE;
    if (g_dry_run)              f |= S5T::F_DRY_RUN;
    if (g_mix.sat)              f |= S5T::F_SAT;
    if (tx_drop_flag)         { f |= S5T::F_TX_DROP; tx_drop_flag = false; }
    h.flags = f;

    // 28バイトに uint32 の millis は載らないので 10ms 単位。
    // 655.35秒で一周する。地上側 (s5_log.cpp) が展開する。
    h.t_cs = (uint16_t)(millis() / 10u);
}

// A: 高度ループ + 姿勢
inline void sendAlt(int mode, float thr_stick) {
    S5T::AltFrame a{};
    fillHeader(a.h, S5T::TYPE_ALT);
    a.modes = S5T::packModes((uint8_t)mode, (uint8_t)althold.state());
    a.thr   = S5T::qu8(thr_stick, 250.0f);

    a.roll_cd  = S5T::q16(g_att.roll,  S5T::SC_CDEG);
    a.pitch_cd = S5T::q16(g_att.pitch, S5T::SC_CDEG);
    a.yaw_dd   = S5T::q16(g_yaw_est,   S5T::SC_DDEG);

    a.range_h_mm      = S5T::q16(g_range_h_m,       S5T::SC_MM);
    a.range_raw_mm    = S5T::q16(g_range_raw_m,     S5T::SC_MM);
    a.alt_hold_mm     = S5T::q16(althold.holdM(),   S5T::SC_MM);
    a.climb_mmps      = S5T::q16(g_climb_mps,       S5T::SC_MM);
    a.alt_vz_tar_mmps = S5T::q16(althold.vzTar(),   S5T::SC_MM);
    a.alt_thr_corr    = S5T::q16(althold.thrCorr(), S5T::SC_1E4);
    a.alt_thr_out     = S5T::qu16(althold.active() ? althold.thrOut() : 0.0f,
                                  S5T::SC_1E4);

    if (!s5tx.send(a)) tx_drop_flag = true;
}

// B: 水平位置ループ
inline void sendPos(int mode) {
    S5T::PosFrame b{};
    fillHeader(b.h, S5T::TYPE_POS);
    b.modes = S5T::packModes((uint8_t)mode, (uint8_t)althold.state());
    b.bad   = (uint8_t)constrain(poshold.badCount(), 0, 255);

    b.vx_mmps      = S5T::q16(poshold.vxCtl(), S5T::SC_MM);
    b.vy_mmps      = S5T::q16(poshold.vyCtl(), S5T::SC_MM);
    b.vx_tar_mmps  = S5T::q16(poshold.vxTar(), S5T::SC_MM);
    b.vy_tar_mmps  = S5T::q16(poshold.vyTar(), S5T::SC_MM);
    b.pos_n_cm     = S5T::q16(poshold.posN(),  S5T::SC_CM);
    b.pos_e_cm     = S5T::q16(poshold.posE(),  S5T::SC_CM);
    b.hold_n_cm    = S5T::q16(poshold.holdN(), S5T::SC_CM);
    b.hold_e_cm    = S5T::q16(poshold.holdE(), S5T::SC_CM);
    b.lean_roll_cd  = S5T::q16(poshold.leanRoll(),  S5T::SC_CDEG);
    b.lean_pitch_cd = S5T::q16(poshold.leanPitch(), S5T::SC_CDEG);

    if (!s5tx.send(b)) tx_drop_flag = true;
}

// C: 姿勢ループの内部 (モーター出力 / 角速度 / ミキサー飽和)
//  離陸時の転倒や発振の切り分け用。A/B だけでは「どのモーターが飽和したか」
//  「レートループが指令に追従しているか」が分からない。
inline void sendAtt(int mode) {
    S5T::AttFrame c{};
    fillHeader(c.h, S5T::TYPE_ATT);
    c.modes = S5T::packModes((uint8_t)mode, (uint8_t)althold.state());
    c.sat   = g_mix.sat;

    c.m1 = S5T::qu8(g_out[0], 250.0f);
    c.m2 = S5T::qu8(g_out[1], 250.0f);
    c.m3 = S5T::qu8(g_out[2], 250.0f);
    c.m4 = S5T::qu8(g_out[3], 250.0f);

    c.roll_rate_dd      = S5T::q16(roll_axis.rate_meas,  S5T::SC_DDEG);
    c.pitch_rate_dd     = S5T::q16(pitch_axis.rate_meas, S5T::SC_DDEG);
    c.yaw_rate_dd       = S5T::q16(yaw_axis.rate_meas,   S5T::SC_DDEG);
    c.roll_rate_tar_dd  = S5T::q16(roll_axis.rate_tar,   S5T::SC_DDEG);
    c.pitch_rate_tar_dd = S5T::q16(pitch_axis.rate_tar,  S5T::SC_DDEG);

    c.roll_cmd  = S5T::q16(roll_axis.cmd,  S5T::SC_1E4);
    c.pitch_cmd = S5T::q16(pitch_axis.cmd, S5T::SC_1E4);

    // ★ roll_axis.stick ではなく sbus から直接読む。roll_axis.stick は
    //   updateControl() の !armed で return する前には入らないので、
    //   非アーム中は 0 のままになる。トリム確認は飛ばす前にやりたい。
    if (S5::USE_SBUS) {
        const float rs = constrain(sbus.des[Ch::ROLL],  -1.0f, 1.0f);
        const float ps = constrain(sbus.des[Ch::PITCH], -1.0f, 1.0f);
        c.roll_stick  = (int8_t)constrain(lroundf(rs * S5T::SC_STICK), -127L, 127L);
        c.pitch_stick = (int8_t)constrain(lroundf(ps * S5T::SC_STICK), -127L, 127L);
    }

    if (!s5tx.send(c)) tx_drop_flag = true;
}

// P: 今どのゲインで飛んでいるか。数秒に1回。
inline void sendParam() {
    S5T::ParamFrame p{};
    p.type = S5T::TYPE_PARAM;
    p.seq  = seq++;
    p.ver  = S5T::VERSION;
    p.cfg_flags = (uint8_t)((g_alt_hold_enable ? S5T::PF_ALT_HOLD_EN : 0) |
                            (g_dry_run         ? S5T::PF_DRY_RUN     : 0) |
                            ((Q::RANGE_BACKEND == Q::RangeBackend::Sonar_EZ)
                                               ? S5T::PF_SONAR       : 0) |
                            (Q::ALT_STICK_VZ_ENABLE ? S5T::PF_STICK_VZ : 0));

    // 実際に効いている値を読む (シリアル 'p' で飛行中に変えられるため)
    const Quad::Pid& vp = poshold.velPid();
    p.flow_vel_kp = S5T::q16(vp.kp(), S5T::SC_GAIN);
    p.flow_vel_ki = S5T::q16(vp.ki(), S5T::SC_GAIN);
    p.flow_vel_kd = S5T::q16(vp.kd(), S5T::SC_GAIN);
    p.flow_pos_kp = S5T::q16(poshold.posKp(), S5T::SC_GAIN);

    const Quad::Pid& ap = althold.ratePid();
    p.alt_pos_kp  = S5T::q16(althold.posKp(), S5T::SC_GAIN);
    p.alt_rate_kp = S5T::q16(ap.kp(), S5T::SC_GAIN);
    p.alt_rate_ki = S5T::q16(ap.ki(), S5T::SC_GAIN);
    p.alt_rate_kd = S5T::q16(ap.kd(), S5T::SC_GAIN);

    p.alt_hover_thr = S5T::q16(Q::ALT_HOVER_THR, S5T::SC_GAIN);
    p.alt_target_m  = S5T::q16(Q::ALT_TARGET_M,  S5T::SC_GAIN);
    p.flow_max_lean = S5T::q16(Q::FLOW_MAX_LEAN, S5T::SC_GAIN);
    p.alt_thr_auth  = S5T::q16(Q::ALT_THR_AUTH,  S5T::SC_GAIN);

    if (!s5tx.send(p)) tx_drop_flag = true;
}

// telem_tick から呼ぶ。1回につき1パケットしか送れない (IM920sL は 32B 制限)。
//  何を送るかはモードで変える:
//    ANGLE / RATE  … A,C の交互 (各 7.5Hz)
//        B(水平位置) は POSHOLD 以外では中身が全部 0 なので送るだけ無駄。
//        その枠を C に回して、離陸時の転倒やモーター飽和を追えるようにする。
//    POSHOLD / AUTO … A,B,A,C の4枠巡回 (A 7.5Hz / B 3.75Hz / C 3.75Hz)
//        位置ループは 0.3〜1Hz なので B は 3.75Hz で足りる。
//  P(ゲイン一覧) は数秒に1回、その回の枠を borrow する。
inline void tick(int mode, float thr_stick) {
    const uint32_t now = millis();
    if (now - last_param_ms >= S5::TELEM_PARAM_MS) {
        last_param_ms = now;
        sendParam();
        return;
    }

    slot = (uint8_t)((slot + 1) & 0x03);   // 0,1,2,3 の巡回

    if (mode != (int)S5::MODE_POSHOLD && mode != (int)S5::MODE_AUTO) {
        // ANGLE / RATE: A と C の交互
        if (slot & 1) sendAtt(mode);
        else          sendAlt(mode, thr_stick);
        return;
    }

    // POSHOLD: A,B,A,C
    switch (slot) {
        case 0: case 2: sendAlt(mode, thr_stick); break;
        case 1:         sendPos(mode);            break;
        default:        sendAtt(mode);            break;
    }
}

} // namespace S5Tel

// ============================================================
//  § 5  周期実行のヘルパ
// ============================================================
struct Ticker {
    uint32_t period_us;
    uint32_t prev_us = 0;
    uint32_t dt_us   = 0;
    explicit Ticker(uint32_t hz) : period_us(1000000UL / hz) {}
    // setup() の最後に呼ぶ。これが無いと prev_us=0 のまま起動するので、
    // 初回 ready() の dt_us が「起動からの経過時間」(数秒) になり、
    // その dt で PID や積分が一気に進んでしまう。
    void prime() { prev_us = micros(); }
    bool ready() {
        const uint32_t now = micros();
        if (now - prev_us < period_us) return false;
        dt_us   = now - prev_us;
        prev_us = now;
        return true;
    }
};

static Ticker main_tick (S5::MAIN_HZ);
static Ticker debug_tick(S5::DEBUG_HZ);
static Ticker telem_tick(S5::TELEM_TX_HZ);
static Ticker telem_rx_tick(S5::TELEM_RX_HZ);
static Ticker flow_tick (Q::FLOW_LOOP_HZ);   // オプティカルフロー読み出し (FLOW_LOOP_HZ)
static Ticker range_tick(Q::RANGE_LOOP_HZ);  // s5c: VL53L1X ポーリング (RANGE_LOOP_HZ)

// ============================================================
//  § 6  出力
// ============================================================
static void writeMotors() {
    if (!S5::USE_MOTOR) return;
    // ドライラン中は g_out をそのまま残して (ログ/表示用)、ESC へは 0 だけ送る。
    if (g_dry_run) {
        for (int i = 0; i < Q::MOTOR_COUNT; ++i) motors[i].write(0.0f);
        return;
    }
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) motors[i].write(g_out[i]);
}

static void stopAllMotors() {
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) g_out[i] = 0.0f;
    g_mix = Q::MixInfo{};
    writeMotors();
}

// ★ POSHOLD の位置ではアームさせない (2026-09-04 追加)
//
//  2026-09-04 の飛行ログ 008〜013 で、操縦者が ANGLE のつもりで飛ばしていた
//  6本すべてが実際には POSHOLD だった (mode=3 が 94〜100%)。POSHOLD では
//  角度目標がフロー位置ループから来るのでスティックで姿勢を直せず、離陸
//  検知が立った瞬間に最大リーン (FLOW_MAX_LEAN=8deg) が出て機体が飛んで
//  いく。外からは「ANGLE が飛ばない」としか見えない。
//
//  SW_HOVER の向きが思っているのと逆でもアームできてしまうのが原因なので、
//  「アームの瞬間は必ず ANGLE 側」を強制する。いったんアームしたあとに
//  POSHOLD へ切り替えるのは従来どおり自由 (bail-out も従来どおり)。
static bool armGateOk() {
    if (!S5::USE_SBUS) return true;
    return sbus.Ch_state(Ch::SW_HOVER) != up;   // up = POSHOLD 側
}

static bool isArmed() {
    if (!S5::USE_SBUS) return false;
    if (!sbus.isSafe()) return false;
    if (sbus.Ch_state(Ch::THR_CUT) != Q::ARM_SWITCH_STATE) {
        g_arm_latched = false;
        return false;
    }
    // アーム操作の瞬間だけ SW_HOVER の位置を見る。
    if (!g_arm_latched) {
        if (!armGateOk()) {
            static uint32_t last_warn_ms = 0;
            if (millis() - last_warn_ms > 2000) {
                last_warn_ms = millis();
                Serial.println("\n!! ARM 拒否: SW_HOVER が POSHOLD 側です。");
                Serial.println("   ANGLE 側 (bail-out 位置) に戻してからアームしてください。");
                Serial.println("   ★スイッチの向きが思っているのと逆になっていないか確認を。");
                Serial.println("   地上局の画面の MODE= 表示が ANGLE であることを見てください。");
            }
            return false;
        }
        g_arm_latched = true;
    }
    return true;
}

// PIDの内部状態と目標値の両方をクリアする。
// 旧コードは目標値 (tar) を残していたため、モードをまたいで
// 前のモードの目標角が生き残っていました。
static void resetControllers() {
    roll_axis.reset();
    pitch_axis.reset();
    yaw_axis.reset();
    g_angle_div_count = 0;
    g_angle_prev_us   = micros();

    // ヘディングホールドも「今の向き」を基準に取り直す。
    // 相対値なので 0 に戻すだけでよい。
    g_yaw_est     = 0.0f;
    g_yaw_hold    = 0.0f;
    g_yaw_holding = false;

    // s5b/s5d: フロー水平ホールドも「今ここ」を基準に取り直す。
    poshold.reset();

    // s5c: 高度ホールドも「今ここ」を基準に取り直す。
    althold.reset(g_range_valid ? g_range_h_m : 0.0f);
    altest.reset(g_range_valid ? g_range_h_m : 0.0f);
}

static const char* modeName(S5::Mode m) {
    switch (m) {
        case S5::MODE_AUTO:     return "AUTO   (地上局)";
        case S5::MODE_POSHOLD:  return "POSHOLD(フロー位置保持)";
        case S5::MODE_ALTHOLD:  return "ALTHOLD(高度保持のみ)";
        case S5::MODE_ANGLE:    return "ANGLE  (水平維持)";
        default:                return "RATE   (アクロ)";
    }
}

// ------------------------------------------------------------
//  モード判定  — 3段階。SW_HOVER の1本だけで決める。
//
//    SW_HOVER=down(手前) → ANGLE   (完全手動。これが bail-out)
//    SW_HOVER=cen (中央) → ALTHOLD (姿勢は手動、高度だけ自動保持)
//    SW_HOVER=up  (奥)   → POSHOLD (完全自動: 水平位置 + 高度)。
//                          フローが死んでいるときは ALTHOLD に自動フォールバック
//                          (ALTHOLDにも高度手段が無ければ ANGLE 相当の動作になる)
//
//  ・SW_AUTO / 地上局AUTO / RATE(アクロ) は使わない (封印)。
//  ・スロットルカット (THR_CUT) は selectMode より上位で、常にモーターを止める。
// ------------------------------------------------------------
static S5::Mode selectMode() {
    if (!S5::USE_SBUS) return S5::MODE_ANGLE;

    const Sw sw = sbus.Ch_state(Ch::SW_HOVER);
    if (sw == up) {
        // ★ 2026-09-09: g_flow_ok は起動時 flow.begin() の 1 回きりのスナップ
        //   ショットなので、「フロー喪失時のフォールバック」と書いてあっても
        //   実際には起動時の初期化失敗しか拾えていなかった。飛行中に PMW3901
        //   が固まると (0,0) を返し続け、PosHold の失探検出 (|速度| が大きい)
        //   にも掛からないまま「静止している」と誤認して機体が流れる。
        //   suspectDead() (生カウントが FLOW_DEAD_S 秒ずっと厳密に 0) を
        //   足して、実行時の凍結でも ALTHOLD へ落ちるようにする。
        //   ※ 地上で完全静止していても suspectDead() は立つ。その場合は
        //     MODE 表示が ALTHOLD になるので「フローが値を出していない」と
        //     すぐ分かる (この機体は今それに当たる)。
        if (S5::USE_FLOW && g_flow_ok && !flow.suspectDead()) return S5::MODE_POSHOLD;
        return S5::MODE_ALTHOLD;   // フロー未初期化 or 実行時に凍結
    }
    if (sw == cen) return S5::MODE_ALTHOLD;
    return S5::MODE_ANGLE;
}

// ------------------------------------------------------------
//  § 6-2  s5b/s5d : フロー水平ホールド  (FLOW_LOOP_HZ で呼ぶ)
//
//   中身は quad/PosHold.h (Q::PositionHold) に移した。ここは
//   「効かせる条件」と「スティックの取り込み」だけを組み立てて渡す薄い層。
//
//   出力は poshold.leanRoll() / leanPitch() [deg]。updateControl() が POSHOLD の
//   ときに角度ループの目標角として使う。スロットルは触らない (高度は § 6-3)。
//
//   ★ active の条件: アーム済 && POSHOLD && スロットル > FLOW_ENABLE_THR
//                    && 離陸検知済 (FLOW_REQUIRE_AIRBORNE)。
//     地上でスロットルを下げたままだとフロー制御は一切出力しない (安全)。
//     ドライラン('m')で符号を確認するときも、スティックを上げる必要がある。
// ------------------------------------------------------------
static void updateFlowHold(float dt_s) {
    const float thr = S5::USE_SBUS ? constrain(sbus.des[Ch::THR], 0.0f, 1.0f) : 0.0f;

    // ★ 離陸するまでは効かせない (FLOW_REQUIRE_AIRBORNE)。
    //   地上ではフロー速度が常に 0 なので、位置積分がノイズを溜め、
    //   速度I項が FLOW_VEL_I_LIMIT まで巻き上がる。実際、地上のドライランで
    //   目標リーン角が -3〜-5deg まで育つのを確認している。そのまま浮くと
    //   離陸直後に飛び出すため、AltHold の離陸検知が立つまで待つ。
    //   ★ 測距が無い構成 (USE_RANGE=false / センサ初期化失敗) では離陸を
    //     検知できないので、このゲートは無効にして s5b と同じ動作に戻す。
    //     ゲートしたままだと水平ホールドが永久に立ち上がらない。
    const bool can_detect_takeoff = S5::USE_RANGE && g_range_ok;
    const bool airborne = !Q::FLOW_REQUIRE_AIRBORNE || !can_detect_takeoff
                        || althold.airborne();

    const bool  active = isArmed() && (g_mode == S5::MODE_POSHOLD)
                      && (thr > Q::FLOW_ENABLE_THR)
                      && airborne
                      && S5::USE_FLOW && g_flow_ok;

    // スティックは機体座標のまま渡す (パイロットの前後左右 = 機首基準)
    const float sx = Q::STICK_SIGN_PITCH * sbus.des[Ch::PITCH];
    const float sy = Q::STICK_SIGN_ROLL  * sbus.des[Ch::ROLL];

    // s5d: 位置積分を地面固定フレームで行うため、ヘディングを渡す。
    //  g_yaw_est はアーム時を 0 とした相対方位 (§ 7-2 が積分している)。
    poshold.update(dt_s, flow.vx, flow.vy, g_yaw_est, sx, sy, active);
}

// ------------------------------------------------------------
//  § 6-3  s5c : 高度ホールド  (RANGE_LOOP_HZ で呼ぶ)
//
//   中身は quad/AltHold.h (Q::AltitudeHold) に移した。ここは入力を集めて
//   渡すだけの薄い層。カスケードと engage 状態遷移はクラス側にある。
//
//   ★ POSHOLD 中、プロポのスロットルは「ALT_ENABLE_THR を超えているか」の
//     enable ゲートにしか使わない。出力は測距だけで決まる。
//   ★ fresh (新しい測距サンプルが入ったか) を渡す。ソナーは ~20Hz なので、
//     50Hz で無条件に PID を回すと同じ climb 値で D項がスパイクする。
// ------------------------------------------------------------
static void updateAltHold(float dt_s) {
    const float thr = S5::USE_SBUS ? constrain(sbus.des[Ch::THR], 0.0f, 1.0f) : 0.0f;

    // 前回の呼び出し以降にミキサーが実際に使ったスロットルの平均。
    // まだ1回も回っていなければ -1 (= 情報なし) を渡す。
    const float thr_applied = (g_thr_used_n > 0)
                            ? (g_thr_used_sum / (float)g_thr_used_n)
                            : -1.0f;
    g_thr_used_sum = 0.0f;
    g_thr_used_n   = 0;

    //  上昇速度をどこから取るか。
    //  ALT_USE_ACC_FUSION=true なら加速度融合の推定値 (遅れがほぼ無い)、
    //  false なら従来どおり測距の微分。切り替えても他は一切変えない。
    const bool  use_est = Q::ALT_USE_ACC_FUSION && altest.valid();
    const float climb   = use_est ? altest.climbMps() : g_climb_mps;

    althold.update(dt_s,
                   g_alt_hold_enable && S5::USE_RANGE,
                   isArmed(),
                   g_mode == S5::MODE_POSHOLD || g_mode == S5::MODE_ALTHOLD,
                   g_range_valid,
                   g_range_fresh,
                   g_range_h_m,
                   climb,
                   thr,
                   thr_applied);
}

// ============================================================
//  § 7  制御
// ============================================================
static void updateControl(float dt_s) {
    // --- モード判定 ---
    g_mode = selectMode();

    const bool armed = isArmed();

    // --- アーム状態やモードが変わったらクリア ---
    if (armed != g_prev_armed) {
        resetControllers();
        g_prev_armed = armed;
        Serial.println(armed ? "\n>>> ARMED" : "\n>>> DISARMED");
    }
    if (g_mode != g_prev_mode) {
        resetControllers();
        g_prev_mode = g_mode;
        Serial.printf("\n>>> MODE = %s\n", modeName(g_mode));
    }

    // --- 測定値はアーム前から入れておく ---
    //  ここより下は !armed で return するので、以前はアームするまで
    //  rate_meas / ang_meas が 0 のままだった。テレメトリの C フレームも
    //  全部 0 になり、飛行前に「機体を傾けてジャイロの符号を見る」という
    //  地上確認ができなかった。これは測定値の代入だけで出力には触らない。
    roll_axis.rate_meas  = g_att.roll_rate;
    pitch_axis.rate_meas = g_att.pitch_rate;
    yaw_axis.rate_meas   = g_att.yaw_rate;
    roll_axis.ang_meas   = g_att.roll;
    pitch_axis.ang_meas  = g_att.pitch;

    if (!armed) { stopAllMotors(); return; }

    // s5c: 高度ホールドが active なら、ミキサーへ渡すスロットルを
    //  ホバースロットル±PID補正 (althold.throttle()) に差し替える。
    //  それ以外は従来どおり物理プロポのスロットルをそのまま使う。
    const float thr_stick = constrain(sbus.des[Ch::THR], 0.0f, 1.0f);

    // ★ POSHOLD で高度ホールドが engage していないときのスロットル上限。
    //
    //  2026-09-04 に見つかった穴: 地面に置いた状態では測距が RANGE_MIN_M
    //  (0.03m) を割って 1秒で失探するため、POSHOLD に入れて離陸させようと
    //  しても AltHold は NoRange のまま engage しない。その間 active() が
    //  false なので、以前はスロットルがプロポの値のまま素通しだった。
    //  POSHOLD ではスティックで姿勢を直せない (ang_tar は位置ループが出す)
    //  ので、「姿勢を当てられないのに全開で上がれる」状態になっていた。
    //  さらに測距が有効になった瞬間に ALT_HOVER_THR へ跳ぶ。
    //
    //  そこで、engage 前は「engage 後に自動制御が出せる上限」と同じところで
    //  頭打ちにする。パイロットが自動制御より大きなスロットルを出せない。
    //   ・地上から: 0.45 まで出せるので離陸はできる。5cm ほど浮けば測距が
    //     有効になり、そこから通常の高度ホールドへ引き継がれる。
    //   ・空中で ANGLE から切り替えた場合: ホバー付近なので頭打ちに当たらず、
    //     測距を失っていても落ちない。
    //  ★ 2026-09-07: 定義を ALT_HOVER_THR + ALT_THR_AUTH からベタ値に変えた。
    //    ALT_THR_AUTH を 0.20 -> 0.55 に広げた時 (log_034 の転倒対策) に、この
    //    上限も 0.80 -> 1.15 へ道連れで動いてしまい、「頭打ち」が実質無効化
    //    されていた (上のコメントが書いている保護が消えていた)。
    //  ★★ 2026-09-09: 0.55 -> 0.65。ALT_HOVER_THR を実測で 0.42 -> 0.50 に
    //    上げた副作用で、余裕が 0.13 -> 0.05 しか無くなっていた。
    //    LOG0054 の実測ホバーは飛行前半 0.485 / 後半 0.515 で、電池が減ると
    //    さらに上がる。0.55 のままだと飛行後半に「上限 < 実ホバー」となり、
    //    POSHOLD で engage 前に沈み始めてもスロットルで救えなくなる。
    //    推力は指令の 2 乗なので 0.65 なら (0.65/0.50)^2 = 1.69 倍 = 上向き
    //    0.69g の余裕があり、実ホバーが 0.55 まで悪化しても 0.40g 残る。
    //    それでいて「姿勢を当てられないまま全開 (1.0)」は防げる。
    constexpr float POSHOLD_THR_CAP = 0.65f;
    //  ★ 単なる > ではなく「余裕 0.10 以上」を強制する。ALT_HOVER_THR を
    //    次に触った人が、ここを黙って際どくしてしまわないように。
    static_assert(POSHOLD_THR_CAP >= Q::ALT_HOVER_THR + 0.10f,
                  "POSHOLD_THR_CAP はホバースロットル +0.10 以上にすること "
                  "(離陸と電池消耗ぶんの余裕が要る)");

    float thr;
    if (althold.active())               thr = althold.throttle();
    else if (g_mode == S5::MODE_POSHOLD) thr = constrain(thr_stick, 0.0f, POSHOLD_THR_CAP);
    else                                 thr = thr_stick;
    const bool  integrate = (thr > S5::I_ENABLE_THR);

    // --- 測定値は上 (アーム判定の前) で入れてある ---
    // yaw_axis.ang_meas は § 7-2 でヘディングホールドの積分値を入れる。
    // g_att.yaw (Madgwick の6軸ヨー) は絶対方位として意味を持たないので使わない。

    // ------------------------------------------------------------
    //  スティック入力
    //
    //  AUTO モードでは、地上局(position_estimator)が計算した RC相当コマンドを
    //  スティック値の「代わりに」使う。以降の制御経路は ANGLE モードと完全に
    //  同じなので、自律制御専用のPID経路は存在しない。
    //
    //  ★ スロットルだけは全モード共通で常に物理プロポから取る (下の thr)。
    //    このコード全体の安全上の不変条件として崩さない。
    //    g.ap_throttle はあえて使わない。
    // ------------------------------------------------------------
    if (g_mode == S5::MODE_AUTO) {
        const GroundData& g = telemetry.lastGroundData();
        roll_axis.stick  = constrain(g.ap_roll,  -1.0f, 1.0f);
        pitch_axis.stick = constrain(g.ap_pitch, -1.0f, 1.0f);
        yaw_axis.stick   = constrain(g.ap_yaw,   -1.0f, 1.0f);
    } else {
        roll_axis.stick  = Q::STICK_SIGN_ROLL  * sbus.des[Ch::ROLL];
        pitch_axis.stick = Q::STICK_SIGN_PITCH * sbus.des[Ch::PITCH];
        yaw_axis.stick   = Q::STICK_SIGN_YAW   * sbus.des[Ch::YAW];
    }

    // ------------------------------------------------------------
    //  外側ループ: 角度 → 目標角速度   (ANGLE / AUTO / ALTHOLD / POSHOLD, 200Hz)
    // ------------------------------------------------------------
    if (g_mode == S5::MODE_ANGLE || g_mode == S5::MODE_AUTO ||
        g_mode == S5::MODE_ALTHOLD || g_mode == S5::MODE_POSHOLD) {

        if (g_mode == S5::MODE_POSHOLD) {
            // 目標角は updateFlowHold() が FLOW_LOOP_HZ で計算済み (すでにクランプ済み)。
            // スティックはそこで「目標速度」として使っている。
            roll_axis.ang_tar  = poshold.leanRoll();
            pitch_axis.ang_tar = poshold.leanPitch();
        } else {
            // ★ 2026-09-05: エクスポを追加。センター付近の細かい操作をしやすくする。
            //   フルスティックでの最大角度・追従速度(角度ループのkp)は変えない。
            roll_axis.ang_tar  = Q::stickExpo(roll_axis.stick,  Q::STICK_EXPO_ANGLE) * Q::MAX_ANGLE_ROLL;
            pitch_axis.ang_tar = Q::stickExpo(pitch_axis.stick, Q::STICK_EXPO_ANGLE) * Q::MAX_ANGLE_PITCH;
        }

        if (++g_angle_div_count >= Q::ANGLE_LOOP_DIV) {
            g_angle_div_count = 0;

            // ★ 実測の経過時間を渡す。
            //   旧コードはここで常に 1ms (メインループのdt) を使っていたため、
            //   角度ループの I項と D項が 5倍ずれていました。
            const uint32_t now = micros();
            const float ang_dt_s = (float)(now - g_angle_prev_us) * 1e-6f;
            g_angle_prev_us = now;

            roll_axis.rate_tar = constrain(
                roll_axis.angle.update(roll_axis.ang_tar, roll_axis.ang_meas,
                                       ang_dt_s, integrate),
                -S5::ANGLE_OUT_LIMIT, S5::ANGLE_OUT_LIMIT);

            pitch_axis.rate_tar = constrain(
                pitch_axis.angle.update(pitch_axis.ang_tar, pitch_axis.ang_meas,
                                        ang_dt_s, integrate),
                -S5::ANGLE_OUT_LIMIT, S5::ANGLE_OUT_LIMIT);
        }
        // 間引かれたループでは、前回の rate_tar をそのまま使う
    } else {
        // RATE モード: スティックが直接、目標角速度になる
        roll_axis.ang_tar  = 0.0f;
        pitch_axis.ang_tar = 0.0f;
        roll_axis.rate_tar  = roll_axis.stick  * Q::MAX_RATE_ROLL;
        pitch_axis.rate_tar = pitch_axis.stick * Q::MAX_RATE_PITCH;
    }

    // ------------------------------------------------------------
    //  § 7-2  ヨー: ヘディングホールド
    //
    //  ・スティックを触っている間 → 素直にレート指令 (従来どおり)
    //  ・中立に戻した瞬間          → そのときの方位を目標として保持
    //
    //  レートPIDの I項 (§1) は「回転速度を0にする」ところまでしか
    //  保証しない。突風で30度振られたら、その30度は戻ってこない。
    //  振られた分まで戻したいので、相対方位の外側ループを足す。
    // ------------------------------------------------------------
    //  積分は「制御に使っているジャイロ値」をそのまま使う。
    //  こうするとレートループと位相が完全に揃う。
    g_yaw_est += yaw_axis.rate_meas * dt_s;

    const bool yaw_stick_active = (fabsf(yaw_axis.stick) > Gain::YAW_STICK_DEAD);

    if (yaw_stick_active || !integrate) {
        // 操作中、または低スロットル(地上)。
        // 目標方位を現在値に追従させておくことで、スティックを離した
        // 瞬間から「今の向き」の保持が始まる。
        yaw_axis.rate_tar = yaw_axis.stick * Q::MAX_RATE_YAW;
        g_yaw_hold        = g_yaw_est;
        g_yaw_holding     = false;
    } else {
        // 中立: 保持方位との差を消しにいく
        const float err = constrain(g_yaw_hold - g_yaw_est,
                                    -Gain::YAW_HOLD_ERR_LIM,
                                    +Gain::YAW_HOLD_ERR_LIM);
        yaw_axis.rate_tar = constrain(g_yaw_hold_kp * err,
                                      -Gain::YAW_HOLD_RATE_LIM,
                                      +Gain::YAW_HOLD_RATE_LIM);
        g_yaw_holding     = true;
    }

    // 表示用 (ヨーには角度PIDを通していないが、保持誤差をここに入れておく)
    yaw_axis.ang_tar  = g_yaw_hold;
    yaw_axis.ang_meas = g_yaw_est;

    // ------------------------------------------------------------
    //  内側ループ: 角速度 → トルク指令   (1000Hz)
    // ------------------------------------------------------------
    roll_axis.cmd  = roll_axis.rate .update(roll_axis.rate_tar,  roll_axis.rate_meas,  dt_s, integrate);
    pitch_axis.cmd = pitch_axis.rate.update(pitch_axis.rate_tar, pitch_axis.rate_meas, dt_s, integrate);
    yaw_axis.cmd   = yaw_axis.rate  .update(yaw_axis.rate_tar,   yaw_axis.rate_meas,   dt_s, integrate);

    // --- ミキサー ---
    Q::mix(thr, roll_axis.cmd, pitch_axis.cmd, yaw_axis.cmd, g_out, &g_mix);

    // 高度ホールドへ返すぶんを積算 (次の updateAltHold() で平均を取る)。
    g_thr_used_sum += g_mix.thr_used;
    g_thr_used_n   += 1;

    writeMotors();
}

// ============================================================
//  § 8  シリアル (ゲイン調整)
// ============================================================
static void tuningMenu() {
    stopAllMotors();
    resetControllers();

    const unsigned long old_timeout = Serial.getTimeout();
    Serial.setTimeout(30000);
    while (Serial.available()) Serial.read();

    Serial.println("\n\n!! モーターを停止しました。制御ループも止まっています !!");
    Serial.println("========== PID Tuning ==========");
    Serial.println("-- Rate (内側) --");
    Serial.printf(" [1] Roll  P %9.5f  [2] Roll  I %9.5f  [3] Roll  D %9.5f\n",
                  roll_axis.rate.kp(), roll_axis.rate.ki(), roll_axis.rate.kd());
    Serial.printf(" [4] Pitch P %9.5f  [5] Pitch I %9.5f  [6] Pitch D %9.5f\n",
                  pitch_axis.rate.kp(), pitch_axis.rate.ki(), pitch_axis.rate.kd());
    Serial.printf(" [7] Yaw   P %9.5f  [8] Yaw   I %9.5f  [9] Yaw   D %9.5f\n",
                  yaw_axis.rate.kp(), yaw_axis.rate.ki(), yaw_axis.rate.kd());
    Serial.println("-- Angle (外側) --");
    Serial.printf(" [a] Roll  P %9.4f  [b] Roll  I %9.4f  [c] Roll  D %9.4f\n",
                  roll_axis.angle.kp(), roll_axis.angle.ki(), roll_axis.angle.kd());
    Serial.printf(" [d] Pitch P %9.4f  [e] Pitch I %9.4f  [f] Pitch D %9.4f\n",
                  pitch_axis.angle.kp(), pitch_axis.angle.ki(), pitch_axis.angle.kd());
    Serial.println("-- ヘディングホールド (ヨー) --");
    Serial.printf(" [h] Hold  P %9.4f   (0 にすると保持を切って従来のレートのみになる)\n",
                  g_yaw_hold_kp);
    Serial.println("-- s5b フロー位置ホールド --");
    Serial.printf(" [i] Vel P %9.4f  [j] Vel I %9.4f  [o] Pos P %9.4f\n",
                  g_flow_vel_kp, g_flow_vel_ki, g_flow_pos_kp);
    Serial.println("-- s5c 高度ホールド --");
    Serial.printf(" [s] Pos P %9.4f  [t] Rate P %9.4f  [u] Rate I %9.4f  [v] Rate D %9.5f\n",
                  althold.posKp(), althold.ratePid().kp(),
                  althold.ratePid().ki(), althold.ratePid().kd());
    Serial.println(" [q] 抜ける");
    Serial.print("選択 > ");

    while (!Serial.available()) { /* 入力待ち */ }
    String sel = Serial.readStringUntil('\n');
    sel.trim();
    sel.toLowerCase();

    if (sel.length() == 0 || sel[0] == 'q') {
        Serial.setTimeout(old_timeout);
        Serial.println("\n再開します。");
        return;
    }

    Serial.print("新しい値 > ");
    const float v = Serial.parseFloat();
    Serial.println(v, 5);

    // ヘディングホールドは Pid クラスではないので先に処理する
    if (sel[0] == 'h') {
        g_yaw_hold_kp = constrain(v, 0.0f, 20.0f);
        Serial.printf("更新: ヘディングホールド kp=%.4f\n", g_yaw_hold_kp);
        resetControllers();
        Serial.setTimeout(old_timeout);
        Serial.println("再開します。");
        return;
    }

    // s5b フロー位置ホールドのゲイン (vx/vy 両方に同じ値を入れる)
    if (sel[0] == 'i' || sel[0] == 'j' || sel[0] == 'o') {
        if (sel[0] == 'i') g_flow_vel_kp = constrain(v, 0.0f, 40.0f);
        if (sel[0] == 'j') g_flow_vel_ki = constrain(v, 0.0f, 20.0f);
        if (sel[0] == 'o') g_flow_pos_kp = constrain(v, 0.0f, 5.0f);
        poshold.setVelGains(g_flow_vel_kp, g_flow_vel_ki, Q::FLOW_VEL_KD);
        poshold.setPosKp(g_flow_pos_kp);
        Serial.printf("更新: Flow vel P=%.3f I=%.3f  pos P=%.3f\n",
                      g_flow_vel_kp, g_flow_vel_ki, g_flow_pos_kp);
        resetControllers();
        Serial.setTimeout(old_timeout);
        Serial.println("再開します。");
        return;
    }

    // s5c 高度ホールドのゲイン
    if (sel[0] == 's' || sel[0] == 't' || sel[0] == 'u' || sel[0] == 'v') {
        Q::Pid& arp = althold.ratePid();
        if (sel[0] == 's') althold.setPosKp(constrain(v, 0.0f, 100.0f));
        // rate ゲインは上限クランプなし (負値だけ弾く。負だと正帰還で即発散)
        if (sel[0] == 't') arp.set_gains(max(v, 0.0f), arp.ki(), arp.kd());
        if (sel[0] == 'u') arp.set_gains(arp.kp(), max(v, 0.0f), arp.kd());
        if (sel[0] == 'v') arp.set_gains(arp.kp(), arp.ki(), max(v, 0.0f));
        Serial.printf("更新: Alt pos P=%.3f  rate P=%.3f I=%.3f D=%.5f\n",
                      althold.posKp(), arp.kp(), arp.ki(), arp.kd());
        resetControllers();
        Serial.setTimeout(old_timeout);
        Serial.println("再開します。");
        return;
    }

    Q::Pid* pid = nullptr;
    int which = -1;   // 0=P 1=I 2=D
    switch (sel[0]) {
        case '1': pid = &roll_axis.rate;   which = 0; break;
        case '2': pid = &roll_axis.rate;   which = 1; break;
        case '3': pid = &roll_axis.rate;   which = 2; break;
        case '4': pid = &pitch_axis.rate;  which = 0; break;
        case '5': pid = &pitch_axis.rate;  which = 1; break;
        case '6': pid = &pitch_axis.rate;  which = 2; break;
        case '7': pid = &yaw_axis.rate;    which = 0; break;
        case '8': pid = &yaw_axis.rate;    which = 1; break;
        case '9': pid = &yaw_axis.rate;    which = 2; break;
        case 'a': pid = &roll_axis.angle;  which = 0; break;
        case 'b': pid = &roll_axis.angle;  which = 1; break;
        case 'c': pid = &roll_axis.angle;  which = 2; break;
        case 'd': pid = &pitch_axis.angle; which = 0; break;
        case 'e': pid = &pitch_axis.angle; which = 1; break;
        case 'f': pid = &pitch_axis.angle; which = 2; break;
        default: break;
    }

    if (pid) {
        const float p = (which == 0) ? v : pid->kp();
        const float i = (which == 1) ? v : pid->ki();
        const float d = (which == 2) ? v : pid->kd();
        pid->set_gains(p, i, d);
        Serial.printf("更新: P=%.5f I=%.5f D=%.5f\n", p, i, d);
    } else {
        Serial.println("不明な選択です");
    }

    resetControllers();
    Serial.setTimeout(old_timeout);
    Serial.println("再開します。");
}

// I2C バススキャン。0x08..0x77 を叩いて ACK を返したアドレスを列挙する。
//  期待値:  0x29 = VL53L1X (距離)   0x68 = MPU6050 (IMU)
//  0x68 だけ見えて 0x29 が見えない → VL53L1X が配線に乗れていない
//    (SDA/SCL 断線・VIN 断線・XSHUT が Low・センサ故障)。
//  両方見えない → バスが Low に張り付いている (モジュール故障 or 電源ショート)。
static void i2cScan() {
    Serial.println("\n--- I2C scan (Wire: SDA=18 / SCL=19) ---");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        Wire.beginTransmission(addr);
        const uint8_t err = Wire.endTransmission();
        if (err == 0) {
            const char *tag = (addr == 0x29) ? "  <- VL53L1X (距離)"
                            : (addr == 0x68) ? "  <- MPU6050 (IMU)"
                                             : "";
            Serial.printf("  0x%02X  ACK%s\n", addr, tag);
            ++found;
        }
    }
    if (found == 0)
        Serial.println("  応答なし。SDA/SCL が Low 固着 or 全モジュール断線。");
    Serial.printf("--- %d device(s) ---\n", found);
}

static void handleSerial() {
    if (!Serial.available()) return;
    const char c = (char)tolower(Serial.read());

    switch (c) {
        case 'p':
            // ★ 2026-09-05: アーム中に誤って 'p' を押すと、tuningMenu() が
            //   stopAllMotors() した直後、次のキー入力が来るまで制御ループを
            //   丸ごと止めてしまう (実測5.86秒停止した例あり)。飛行中は推力
            //   ゼロのままただ落ちるだけなので、アーム中は入れないようにする。
            if (isArmed()) {
                Serial.println("\n!! アーム中は 'p' メニューに入れません "
                               "(モーター停止 + 制御ループ停止で落下するため)。"
                               "ディスアームしてから押してください。");
            } else {
                tuningMenu();
            }
            break;
        case 'i':
            i2cScan();
            break;
        case 'm':
            g_dry_run = !g_dry_run;
            stopAllMotors();
            Serial.printf("\n>>> ドライラン = %s\n",
                          g_dry_run
                            ? "ON  (ESCへは0のみ。g_out は計算/記録。飛行前に必ずOFF)"
                            : "OFF (通常。モーターが回る)");
            break;
        case 'k':
            stopAllMotors();
            Serial.println("CALIBRATE: 機体を水平に置いて動かさないでください");
            // 成功したら EEPROM に保存され、次回起動時に自動で読み込まれる。
            // 姿勢がおかしい / 動いている場合は採用せず理由を出す。
            if (S5::USE_MPU) mpu.recalibrate();
            resetControllers();
            break;
        case 'x':
            // EEPROM の保存値を捨てて Config.h のハードコード値に戻す。
            // ★ handleSerial は tolower() しているので大文字キーは使えない。
            // ★ EEPROM はファーム書き込みでは消えないので、変な値を保存して
            //   しまったときの唯一の逃げ道がこれ。
            stopAllMotors();
            if (S5::USE_MPU) mpu.clearCalibration();
            Serial.println("INFO: 再起動すると Config.h の値に戻ります");
            break;
        case 'r':
            resetControllers();
            Serial.println("PID reset");
            break;
        case 'l':
            // 500Hz ログの開始/停止 (USB 直結時のみ)。scripts/logger.py と対で使う。
            FlightLog::Usb::toggle();
            break;
        case 'v':
            // RAM ログ (実飛行中に自動記録した分) を USB へダンプする。
            // 記録中 (スロットルがまだ THR_GATE を超えている) なら
            // 拒否メッセージを出すだけで安全に呼べる。
            FlightLog::Ram::dump();
            break;
        case 'y':
            // RAM ログの状態だけ見る (何行溜まっているか)。ダンプはしない。
            FlightLog::Ram::status();
            break;
        case 'n':
            // RAM ログの手動トリガ。armed/スロットル不問で 8 秒録る。
            //  ★ 2026-09-09: forceTrigger() は前から定義されていたのに
            //    switch に case が無く、'n' を押しても何も起きなかった
            //    (関数のコメントは「'n' キーでこれを呼ぶ」と書いてあった)。
            //  加速度Z相補フィルタのベンチ検証など、モーターを回さず
            //  機体を手で動かして録りたいときに使う。録った後は 'v' でダンプ。
            FlightLog::Ram::forceTrigger();
            break;
        case 'w':
            // 原因調査用: ループが異常に長くかかった回の内訳をダンプする。
            StallLog::dump();
            break;
        case 'g': {
            // s5c: 高度ホールド (スロットルPID) の ON/OFF トグル。
            //  ★ OFF にした瞬間、スロットルは物理プロポの値へ戻る。
            //    ホバー中に切るときは、スロットルスティックを今の
            //    ホバー位置に戻してから 'g' を押すこと (段差防止)。
            g_alt_hold_enable = !g_alt_hold_enable;
            if (!g_alt_hold_enable) althold.reset(g_range_valid ? g_range_h_m : 0.0f);
            Serial.printf("\n>>> 高度ホールド = %s%s\n",
                          g_alt_hold_enable ? "ON" : "OFF (スロットル手動)",
                          (g_alt_hold_enable && !g_range_valid)
                            ? "  ※ただし今は測距が無効なので効きません" : "");
            break;
        }
        case 'd':
            // 起動時デバイスチェックの結果を再表示する (画面クリアで流れたとき用)
            SelfTest::printFull(Serial);
            break;
        case 's':
            // SD の状態表示。ディスアーム中は単体テスト (再init + 書込/読戻 + エラーコード) も走る。
            //  ★ USE_SD=false (USE_FLOW=true) のときは selftest が SPI0 を
            //    再初期化してフローを潰すので、状態表示だけにする。
            if (!S5::USE_SD) {
                Serial.println("SD: 無効 (USE_FLOW=true)。selftest は SPI0 競合を避けてスキップ");
                break;
            }
            SdLog::status();
            if (!isArmed()) SdLog::selftest(Serial);
            break;
        case 'z':
            // フロー積算のゼロ。既知距離キャリブレーションの開始点。
            g_flow_acc_raw_x = g_flow_acc_raw_y = 0.0;
            g_flow_acc_px_x  = g_flow_acc_px_y  = 0.0;
            g_flow_acc_m_x   = g_flow_acc_m_y   = 0.0;
            Serial.println("FLOW: 積算をゼロにしました");
            break;
        case 'h': {
            // フローのスケール高度 [m] を手で入れる。
            // s5c 以降は VL53L1X が有効なら毎ループ上書きするので、これは
            // 測距を切っている / 失探しているときの手動フォールバック用。
            const unsigned long old_to = Serial.getTimeout();
            Serial.setTimeout(15000);
            Serial.print("\nフロー高度 h [m] > ");
            const float v = Serial.parseFloat();
            Serial.println(v, 3);
            flow.setHeight(v);
            Serial.printf("FLOW: h = %.3f m にしました", flow.height());
            if (S5::USE_RANGE && g_range_valid)
                Serial.print(" (※測距が有効なので次のループで上書きされます)");
            Serial.println();
            Serial.setTimeout(old_to);
            break;
        }
        default:
            break;
    }
}

// ============================================================
//  § 9  デバッグ表示
// ============================================================
static void printStatus(uint32_t dt_us) {
    Serial.print("\033[2J\033[H");
    Serial.println("=== Stage 5d : 1スイッチ完全自動ホバリング ===");
    if (g_dry_run)
        Serial.println(">>> DRY-RUN: ESCへは0のみ (モーターは回らない)  ['m']で解除 <<<");
    Serial.printf("loop dt = %6lu us (%6.1f Hz)   %s   link=%s\n",
                  (unsigned long)dt_us, 1000000.0f / (float)dt_us,
                  isArmed() ? "ARMED" : "DISARMED",
                  sbus.isSafe() ? "OK" : "LOST");
    Serial.printf("MODE = %s   (SW_HOVER: down=ANGLE / cen=ALTHOLD / up=POSHOLD)\n",
                  modeName(g_mode));
    SelfTest::printCompact(Serial);   // 起動時に何がつながっていたか (画面に残す)

    if (S5::USE_IM920) {
        const GroundData& g = telemetry.lastGroundData();
        Serial.printf("IM920 link=%s   AP: roll=%+.3f pitch=%+.3f yaw=%+.3f (thr=%+.3f 未使用)\n",
                      telemetry.groundLinkFresh() ? "FRESH" : "STALE",
                      g.ap_roll, g.ap_pitch, g.ap_yaw, g.ap_throttle);
        // s5 解析テレメトリの送信状況。drop が増え続けるなら
        // S5::TELEM_TX_HZ が UART の帯域(19200bps) に対して速すぎる。
        Serial.printf("TELEM tx=%lu drop=%lu %s\n",
                      (unsigned long)s5tx.sent(), (unsigned long)s5tx.dropped(),
                      s5tx.busy() ? "(sending)" : "");
    }

    if (g_sd_ok) SdLog::brief(Serial);

    if (g_mode == S5::MODE_ANGLE || g_mode == S5::MODE_AUTO ||
        g_mode == S5::MODE_ALTHOLD || g_mode == S5::MODE_POSHOLD) {
        Serial.println("\n[角度ループ]  目標[deg]  実測[deg]  → 角速度目標[deg/s]");
        Serial.printf("  roll  %10.1f %10.1f %18.1f\n",
                      roll_axis.ang_tar, roll_axis.ang_meas, roll_axis.rate_tar);
        Serial.printf("  pitch %10.1f %10.1f %18.1f\n",
                      pitch_axis.ang_tar, pitch_axis.ang_meas, pitch_axis.rate_tar);
    }

    if (g_mode == S5::MODE_POSHOLD) {
        Serial.printf("\n[フロー位置ホールド] %s  vel P=%.2f I=%.2f  pos P=%.2f  失探=%d\n",
                      poshold.holding() ? "HOLD " : "STICK",
                      g_flow_vel_kp, g_flow_vel_ki, g_flow_pos_kp, poshold.badCount());
        Serial.printf("  速度[m/s]  実測(前,右)=%+6.2f %+6.2f   目標(前,右)=%+6.2f %+6.2f\n",
                      poshold.vxCtl(), poshold.vyCtl(), poshold.vxTar(), poshold.vyTar());
        Serial.printf("  位置[m]    現在(N,E)=%+6.2f %+6.2f   保持(N,E)=%+6.2f %+6.2f"
                      "   (地面固定, ψ=%+.1fdeg)\n",
                      poshold.posN(), poshold.posE(),
                      poshold.holdN(), poshold.holdE(), g_yaw_est);
        Serial.printf("  → 目標リーン角[deg]  roll=%+5.1f  pitch=%+5.1f\n",
                      poshold.leanRoll(), poshold.leanPitch());
    }

    // --- s5a: フローセンサの生死 (常時表示) ---
    //  ★ 2026-09-09 追加。以前はフローの状態が POSHOLD のときしか出ず、
    //    「フローが死んでいるので POSHOLD に入れない」状態だと画面のどこにも
    //    出なかった (実際それで長期間気づかれていなかった)。常に出す。
    if (S5::USE_FLOW) {
        Serial.printf("\n[フロー] %s  生カウント(%.0f,%.0f)  0連続=%.1fs  h=%.2fm\n",
                      !g_flow_ok        ? "FAIL(起動時に応答なし)"
                    : flow.suspectDead()? "凍結?(生カウントが0のまま)"
                                        : "OK  ",
                      g_flow_raw_x, g_flow_raw_y, flow.zeroRunS(), flow.height());
    }

    // --- s5c: 距離センサ + 高度ホールド ---
    if (S5::USE_RANGE) {
        Serial.printf("\n[距離:%s] %s  斜め=%.2fm  → 鉛直h=%.2fm  上昇=%+.2fm/s\n",
                      (Q::RANGE_BACKEND == Q::RangeBackend::Sonar_EZ) ? "SONAR" : "ToF",
                      !g_range_ok ? "FAIL "
                                  : (g_range_valid ? "OK   " : "失探 "),
                      g_range_raw_m, g_range_h_m, g_climb_mps);
        Serial.printf("[高度ホールド] %s  %s  hold=%.2fm  vz_tar=%+.2fm/s  "
                      "base=%.2f corr=%+.3f → thr=%.2f\n",
                      g_alt_hold_enable ? "ENABLED" : "OFF(手動)",
                      althold.stateName(),
                      althold.holdM(), althold.vzTar(),
                      althold.thrBase(), althold.thrCorr(), althold.thrOut());
        Serial.printf("  base=ALT_HOVER_THR(%.2f)固定  スティックvz=%s  "
                      "離陸=%s (engage時h=%.2fm)  → スロットルは高度のみで決まる\n",
                      Q::ALT_HOVER_THR,
                      Q::ALT_STICK_VZ_ENABLE ? "有効" : "無効",
                      althold.airborne() ? "検知済" : "未検知(水平ホールド待機)",
                      althold.engageH());
        Serial.printf("  gains: pos P=%.2f  rate P=%.2f I=%.2f D=%.3f   ['g']切替  ['p']-[s..v]調整\n",
                      althold.posKp(), althold.ratePid().kp(),
                      althold.ratePid().ki(), althold.ratePid().kd());
    }

    Serial.println("\n[レートループ] 目標[deg/s] 実測[deg/s]      cmd      I項");
    Serial.printf("  roll  %11.1f %11.1f %9.4f %8.4f\n",
                  roll_axis.rate_tar, roll_axis.rate_meas,
                  roll_axis.cmd, roll_axis.rate.i_term());
    Serial.printf("  pitch %11.1f %11.1f %9.4f %8.4f\n",
                  pitch_axis.rate_tar, pitch_axis.rate_meas,
                  pitch_axis.cmd, pitch_axis.rate.i_term());
    Serial.printf("  yaw   %11.1f %11.1f %9.4f %8.4f\n",
                  yaw_axis.rate_tar, yaw_axis.rate_meas,
                  yaw_axis.cmd, yaw_axis.rate.i_term());

    // --- ヘディングホールド ---
    //  err がじわじわ片側に増え続けるなら、機体が回っているのではなく
    //  ジャイロZのバイアスが残っている (= [k] で再キャリブレーションが必要)。
    Serial.printf("\n[ヨー保持] %s  kp=%.2f  目標%+8.2f  推定%+8.2f  誤差%+7.2f [deg]\n",
                  g_yaw_holding ? "HOLD  " : "STICK ",
                  g_yaw_hold_kp, g_yaw_hold, g_yaw_est,
                  g_yaw_hold - g_yaw_est);

    Serial.printf("\n  attitude roll=%+7.2f pitch=%+7.2f [deg]  thr=%.2f\n",
                  g_att.roll, g_att.pitch, sbus.des[Ch::THR]);

    Serial.println("\n[モーター]  M1=左前 M2=右前 M3=右後 M4=左後");
    for (int i = 0; i < Q::MOTOR_COUNT; ++i) {
        Serial.printf("  M%d %5.3f  ", i + 1, g_out[i]);
        const int bar = (int)(g_out[i] * 40.0f);
        for (int j = 0; j < bar; ++j) Serial.print('#');
        Serial.println();
    }

    Serial.printf("\n[ミキサー] span_limit=%.3f  scale=%.3f  sat=0b%c%c%c%c (M4..M1)\n",
                  g_mix.span_limit, g_mix.scale,
                  (g_mix.sat & 8) ? '1' : '0', (g_mix.sat & 4) ? '1' : '0',
                  (g_mix.sat & 2) ? '1' : '0', (g_mix.sat & 1) ? '1' : '0');

    // --- オプティカルフロー (s5a: 観測のみ) ---
    //  ① 符号/軸: 机の上などで機体を「ゆっくり大きく」平行移動させて acc_px を見る
    //     ・前へ動かす → acc_px の x が一方向に大きく伸びる (逆なら FLOW_SIGN_X=-1)
    //     ・右へ動かす → acc_px の y                        (逆なら FLOW_SIGN_Y=-1)
    //     ・前後に動かして y が伸びる / 左右で x が伸びる     → FLOW_SWAP_XY=true
    //  ③ スケール ([z]でゼロ → 定規で測った高さ h で、定規で測った距離 D を
    //     一定高さでスライド → 止めて acc_px を読む):
    //        counts_per_m    = acc_px ÷ D
    //        FLOW_PX_PER_RAD = counts_per_m × h        ← これを QuadConfig.h に
    //     x(前後) と y(左右) を別々に測る。ほぼ同じ値になるはず。
    //     ※ dt にも仮定高度にも依存しない (counts/m がセンサの素の性質)。
    //  ② de-rotation: ③のあとで。機体の「位置は固定・向きだけ」前後/左右に傾ける。
    //     raw は振れるが derot(x,y) が ~0 のままなら OK。
    //     raw と逆向きに振れる → FLOW_DEROT_SIGN_* を反転。
    if (S5::USE_FLOW) {
        Serial.printf("\n[フロー] %s  h=%.2fm  raw(x,y)=%+6.1f %+6.1f  "
                      "derot(x,y)=%+6.1f %+6.1f\n",
                      g_flow_ok ? "OK  " : "FAIL", flow.height(),
                      g_flow_raw_x, g_flow_raw_y, g_flow_dx, g_flow_dy);
        Serial.printf("         v=%+6.2f %+6.2f m/s (LPF %+6.2f %+6.2f)   [z]ゼロ\n",
                      g_flow_vx, g_flow_vy, g_flow_vx_f, g_flow_vy_f);
        Serial.printf("         積算 raw=(%+9.0f,%+9.0f)  derot=(%+9.0f,%+9.0f)  "
                      "m=(%+7.3f,%+7.3f)\n",
                      g_flow_acc_raw_x, g_flow_acc_raw_y,
                      g_flow_acc_px_x,  g_flow_acc_px_y,
                      g_flow_acc_m_x,   g_flow_acc_m_y);
    }

    Serial.println("\n[p]ゲイン [k]IMUキャリブ(EEPROM保存) [x]キャリブ消去 [r]PIDリセット "
                   "[l]ログ(USB直結時) [n]RAMログ手動トリガ [v]RAMログdump [y]RAMログ状態 "
                   "[w]停止調査ログdump "
                   "[z]フロー積算ゼロ "
                   "[h]フロー高度(手動) [g]高度ホールド切替 [i]I2Cスキャン [m]ドライラン切替 "
                   "[s]SD状態/単体テスト [d]起動時デバイスチェック再表示");
}

// ============================================================
//  § 10  setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    const uint32_t start_ms = millis();
    while (!Serial && (millis() - start_ms < 2000)) { }

    Serial.println("\n\n=== Stage 5d : 1スイッチ完全自動ホバリング ===");
    Serial.println("!! 2モード: SW_HOVER UP = POSHOLD(完全自動) / それ以外 = ANGLE(手動) !!");
    Serial.println("!! bail-out = SW_HOVER を下げる or THR_CUT。初回は広い床で指をスイッチに !!");

    // ★ 共有 SPI0: 全 CS を「最初に」HIGH へ固定する。
    //   これが無いと、flow.begin() が PMW3901 を初期化している間、SD の CS(9) は
    //   まだ入力(フロート)のまま。9 が Low に落ちていると SD も選択されてしまい、
    //   PMW3901 宛ての SPI クロックに SD が応答して両方のバスが壊れる
    //   (SD 追加後に PMW3901 まで FAIL するのはこれが典型)。CS を分けるだけでは
    //   足りず、未初期化のあいだに Low へ落とさないことが要る。
    pinMode(10, OUTPUT); digitalWrite(10, HIGH);   // PMW3901 CS (Quad::FLOW_CS_PIN)
    pinMode(9,  OUTPUT); digitalWrite(9,  HIGH);   // SD HW-125 CS

    roll_axis.rate .set_gains(Gain::RATE_ROLL [0], Gain::RATE_ROLL [1], Gain::RATE_ROLL [2]);
    pitch_axis.rate.set_gains(Gain::RATE_PITCH[0], Gain::RATE_PITCH[1], Gain::RATE_PITCH[2]);
    yaw_axis.rate  .set_gains(Gain::RATE_YAW  [0], Gain::RATE_YAW  [1], Gain::RATE_YAW  [2]);

    roll_axis.angle .set_gains(Gain::ANG_ROLL [0], Gain::ANG_ROLL [1], Gain::ANG_ROLL [2]);
    pitch_axis.angle.set_gains(Gain::ANG_PITCH[0], Gain::ANG_PITCH[1], Gain::ANG_PITCH[2]);

    for (Q::Axis* ax : { &roll_axis, &pitch_axis, &yaw_axis }) {
        ax->rate.set_d_alpha(Gain::RATE_D_ALPHA);
        ax->rate.set_i_limit(Gain::RATE_I_LIMIT);
        ax->angle.set_d_alpha(Gain::ANG_D_ALPHA);
        ax->angle.set_i_limit(Gain::ANG_I_LIMIT);
    }

    // s5b/s5d: フロー水平ホールド (速度ループ + 地面固定フレーム位置ループ)
    poshold.begin();
    poshold.setVelGains(g_flow_vel_kp, g_flow_vel_ki, Q::FLOW_VEL_KD);
    poshold.setPosKp(g_flow_pos_kp);

    // s5c: 高度ホールドの内側ループ (上昇速度[m/s] → スロットル補正[割合])
    althold.begin();

    if (S5::USE_MOTOR) {
        Serial.println("Init motors...");
        for (int i = 0; i < Q::MOTOR_COUNT; ++i) {
            motors[i].set_pin(Q::MOTOR_PIN[i]).begin();
        }
        delay(500);
        stopAllMotors();
    }
    if (g_dry_run)
        Serial.println("!! DRY-RUN 有効: モーターは回りません ('m' で解除) !!");

    if (S5::USE_SBUS)  { Serial.println("Init SBUS...");  sbus.begin(); }
    if (S5::USE_MPU)   { Serial.println("Init IMU...");   mpu.begin();  }
    if (S5::USE_IM920) { Serial.println("Init IM920..."); telemetry.begin(); }
    if (S5::USE_FLOW) {
        Serial.println("Init OpticalFlow (PMW3901)...");
        g_flow_ok = flow.begin();
        Serial.println(g_flow_ok
            ? "  PMW3901 OK"
            : "  !! PMW3901 応答なし (CSピン/SPI配線/電源を確認) !!");
    }
    if (S5::USE_RANGE) {
        const bool sonar = (Q::RANGE_BACKEND == Q::RangeBackend::Sonar_EZ);
        Serial.printf("Init Rangefinder (%s)...\n",
                      sonar ? "SONAR MaxBotix LV-MaxSonar-EZ" : "ToF VL53L1X");
        g_range_ok = rangefinder.begin();
        if (sonar) {
            Serial.printf("  SONAR PW=pin %d / 3V3給電。数百ms後に [距離] に値が出れば配線OK\n",
                          Q::RANGE_SONAR_PW_PIN);
            Serial.println("  (I2Cは使わないので VL53L1X のバス問題とは無関係)");
        } else {
            Serial.println(g_range_ok
                ? "  VL53L1X OK"
                : "  !! VL53L1X 応答なし (SDA=18/SCL=19/3V3/GND 配線を確認) !!");
        }
        Serial.printf("  高度ホールド: %s (POSHOLD で自動)。ホバースロットル=%s  目標高度=%s\n",
                      g_alt_hold_enable ? "有効" : "無効(POSHOLDでも手動)",
                      (Q::ALT_HOVER_THR > 0.01f) ? "実測値" : "未設定(engageしない)",
                      (Q::ALT_TARGET_M  > 0.0f)  ? "固定"   : "突入時の高度");
    }

    // SD ログ (HW-125 / CS=9 / SPI0 を PMW3901 と共有)。飛行まるごとを
    // LOGnnnn.BIN へストリーミングする。CSV 化は PC で scripts/bin2csv.py。
    //  ★ USE_FLOW=true のときは SPI0 競合を避けるため SD を初期化しない。
    //    そのフライトのログは RAM ('n' / スロットル自動トリガ, 8秒) と USB 'l'。
    if (S5::USE_SD) {
        Serial.println("Init SD (HW-125, CS=9, SPI0 共有)...");
        g_sd_ok = SdLog::begin(/*cs=*/9, sizeof(FlightLog::Rec), FlightLog::REC_VER,
                               (uint16_t)FlightLog::LOG_HZ);
        if (g_sd_ok) {
            Serial.println("  SD OK");
        } else {
            Serial.println("  !! SD 応答なし (CS=9 / VCC=5V / SCK13 MOSI11 MISO12 / FAT32 を確認) !!");
            SdLog::selftest(Serial);   // 起動時にもエラーコード付きの詳細を出す
        }
    } else {
        g_sd_ok = false;
        Serial.println("SD: 無効 (USE_FLOW=true のため。ログは RAM 'n'/'v' と USB 'l')");
    }

    // 起動時デバイスチェック (どのデバイスが応答するか)。結果は画面に残り、
    // シリアル 'd' で再表示できる。
    SelfTest::probe();
    SelfTest::printFull(Serial);

    resetControllers();

    // 全 Ticker の基準時刻をここでそろえる。これが無いと初回 ready() の
    // dt_us が「起動からの経過時間」になり、その dt で積分が一気に進む。
    for (Ticker* t : { &main_tick, &debug_tick, &telem_tick, &telem_rx_tick,
                       &flow_tick, &range_tick }) t->prime();

    Serial.println("--- Setup complete ---");
}

void loop() {
    if (!main_tick.ready()) return;
    const uint32_t _t0 = micros();

    const float dt_s = (float)main_tick.dt_us * 1e-6f;

    if (S5::USE_MPU) {
        mpu.update();
        g_att = Q::readAttitude(mpu);

        // s5c: 高度推定の predict。加速度は 1000Hz・遅れほぼゼロなので、
        //  測距 (33Hz・遅れ大) より先にここで積分を進めておく。
        //  ALT_USE_ACC_FUSION=false の間は結果をログに出すだけ。
        altest.predict(dt_s, g_att.roll, g_att.pitch,
                       g_att.acc_x, g_att.acc_y, g_att.acc_z);
    }
    const uint32_t _t1 = micros();

    if (S5::USE_SBUS) sbus.update();
    const uint32_t _t2 = micros();

    // --- オプティカルフロー (FLOW_LOOP_HZ) ---
    //  読んで機体座標化 + de-rotation + 対地速度換算 → updateFlowHold() で
    //  s5b の速度・位置ホールドを計算 (POSHOLD 以外では出力せず基準を保持)。
    //  de-rotation にはレートループと同じジャイロ値 (g_att) を渡して位相を揃える。
    if (S5::USE_FLOW && g_flow_ok && flow_tick.ready()) {
        const float flow_dt_s = (float)flow_tick.dt_us * 1e-6f;
        flow.update(flow_dt_s, g_att.roll_rate, g_att.pitch_rate);
        g_flow_raw_x = flow.raw_x;   g_flow_raw_y = flow.raw_y;
        g_flow_dx    = flow.derot_x; g_flow_dy    = flow.derot_y;
        g_flow_vx    = flow.vx;      g_flow_vy    = flow.vy;

        // キャリブレーション用の積算 (生px / de-rot後px / 推定変位[m])
        g_flow_acc_raw_x += flow.raw_x;          g_flow_acc_raw_y += flow.raw_y;
        g_flow_acc_px_x  += flow.derot_x;        g_flow_acc_px_y  += flow.derot_y;
        g_flow_acc_m_x   += flow.vx * flow_dt_s; g_flow_acc_m_y   += flow.vy * flow_dt_s;

        // 表示用の軽い LPF (生値は g_flow_vx/vy とログに残す)
        constexpr float A = 0.2f;
        g_flow_vx_f += A * (flow.vx - g_flow_vx_f);
        g_flow_vy_f += A * (flow.vy - g_flow_vy_f);

        // s5b: 速度・位置ホールド
        updateFlowHold(flow_dt_s);
    }
    const uint32_t _t3 = micros();

    // --- s5c: 距離センサ (RANGE_LOOP_HZ) ---
    //  測距を非ブロッキングで読み、傾き補正した鉛直高度を flow へ渡す。
    //  ★ 失探しても flow の height は「最後に有効だった値」を保持する
    //    (OpticalFlow::setHeight が範囲外を弾くだけで、
    //     FLOW_ASSUMED_HEIGHT_M へ戻る処理はどこにも無い)。
    //    急に 1.0m へ飛ぶより最後の値を持つほうが安全なので、これで良い。
    //  ★ fresh = このループで新しいサンプルが入ったか。センサ (ソナー ~20Hz)
    //    より速い RANGE_LOOP_HZ で回すので、PID を進めてよい回を区別する。
    if (S5::USE_RANGE && g_range_ok && range_tick.ready()) {
        const float range_dt_s = (float)range_tick.dt_us * 1e-6f;
        g_range_fresh = rangefinder.update(g_att.roll, g_att.pitch);
        g_range_valid = rangefinder.valid();
        g_range_raw_m = rangefinder.rawM();
        g_range_h_m   = rangefinder.heightM();
        g_climb_mps   = rangefinder.climbMps();

        if (S5::USE_FLOW && g_range_valid) flow.setHeight(g_range_h_m);

        // s5c: 高度推定の correct。新しい測距が来た回だけ、そのサンプルで
        //  推定を引き戻す。dt は「前回 correct からの経過」でなければ
        //  ならない (呼び出し周期ではない) ので、ここで測る。
        if (g_range_fresh && g_range_valid) {
            static uint32_t est_last_us = 0;
            const uint32_t now_us = micros();
            const float    est_dt = (est_last_us == 0)
                                  ? 0.0f : (float)(now_us - est_last_us) * 1e-6f;
            est_last_us = now_us;
            altest.correct(g_range_h_m, est_dt);
        }

        // s5c: 高度ホールド (スロットルPID)
        updateAltHold(range_dt_s);
    }
    const uint32_t _t4 = micros();

    // 地上局からの受信。groundLinkFresh() の判定に使うので制御より前に読む。
    // (PIDゲインのリモート調整やリモートリセットは行わない。receive() は
    //  GroundData を取り込んで鮮度を更新するだけ)
    if (S5::USE_IM920 && telem_rx_tick.ready()) telemetry.receive();
    const uint32_t _t5 = micros();

    updateControl(dt_s);
    const uint32_t _t6 = micros();
    handleSerial();
    const uint32_t _t7 = micros();

    StallLog::maybeLog(_t7 - _t0, _t1 - _t0, _t2 - _t1, _t3 - _t2,
                       _t4 - _t3, _t5 - _t4, _t6 - _t5);

    // --- SD ログのファイル開閉 (アーム/ディスアームのエッジ) ---------------
    //  アーム〜ディスアームの全区間を LOGnnnn.BIN へ。RAM ログと違い 8 秒制限なし。
    //  push() はリングへ memcpy するだけ、service() は毎ループ有界サイズだけ書く
    //  (詳細は SdLog.h)。CSV 化は PC で scripts/bin2csv.py。
    const bool armed_now = isArmed();
    const float thr_now  = S5::USE_SBUS ? sbus.des[Ch::THR] : 0.0f;
    if (g_sd_ok) {
        static bool s_sd_was_armed = false;
        if (armed_now && !s_sd_was_armed) {          // アーム: 新規ファイル
            SdLog::startFile();                      // open + preAllocate で数十ms
            // ↑のブロックで main_tick の基準がずれる。次の ready() が
            //   dt=数十ms を返すと updateControl がその dt で積分を一気に
            //   進めてしまう (アーム直後にモーターがピクつく) ので基準を取り直す。
            main_tick.prime();
        }
        if (!armed_now && s_sd_was_armed) {          // ディスアーム: BIN を閉じる
            SdLog::stopFile();
            //  ★ 2026-09-09: オンボードの BIN→CSV 変換は廃止 (1万行で数十秒
            //    かかる上、共有 SPI を長時間占有する)。カードを PC に挿して
            //    scripts/bin2csv.py で変換する運用に一本化した。
        }
        s_sd_was_armed = armed_now;
    }

    // --- 500Hz ログ: 1 レコードを作って USB / RAM / SD の 3 つへ配る --------
    //  ★ 2026-09-09: 以前は Log::sample() が USB 用に独自の printf を持ち、
    //    RamLog::update() と SD 用にそれぞれ fillRec() を呼んでいた
    //    (= 同じ 1 行ぶんの量子化を 2 回、printf 書式は 2 本)。
    //    Rec を 1 個だけ作って配る形に統一した。書式は
    //    FlightLog::formatRow() の 1 本だけになり、USB と SD のログは
    //    完全に同じ内容になる (USB 側も量子化を通るようになった。
    //    角度 0.01deg / 角速度 0.1dps / 出力 1/250 の分解能で、解析には十分)。
    {
        static int s_log_div = 0;
        if (++s_log_div >= FlightLog::DIV) {
            s_log_div = 0;
            // RAM トリガの追加ゲート。フロー試験時 (USE_FLOW=true) は
            //  「SW_HOVER=up = フロー水平ホールドに入れた瞬間」からの 8 秒を
            //  録る。ANGLE で離陸してから上空でスイッチを上げるので、
            //  地上待機や上昇でバッファを食い潰さずに済む。
            //  SD 運用 (USE_FLOW=false) では従来どおり thr>0.20 だけで録る。
            const bool ram_gate = !S5::USE_FLOW
                                || sbus.Ch_state(Ch::SW_HOVER) == up;
            FlightLog::Ram::tick(armed_now, thr_now, ram_gate);   // トリガ状態機械 (軽い)
            // どのシンクも動いていなければ量子化そのものを省く
            if (FlightLog::Usb::active || FlightLog::Ram::recording ||
                SdLog::recording()) {
                FlightLog::Rec r;
                fillRec(r, main_tick.dt_us, (int)g_mode, armed_now, thr_now);
                FlightLog::Usb::sample(r);   // 'l' 中のみ。USB が詰まっていたら捨てる
                FlightLog::Ram::push(r);     // 記録中のみ
                SdLog::push(&r, sizeof(r));  // アーム中のみ (中で recording を見る)
            }
        }
    }
    if (g_sd_ok) SdLog::service();                   // 毎ループ、有界の書き出し

    // s5 解析テレメトリの送信 (TELEM_TX_HZ = 10Hz)。地上局が CSV に落とす。
    //  yaw はヘディングホールドの積分値を送る。Madgwick の6軸ヨーは
    //  絶対方位として意味を持たないため。
    //  ★ ここでは送信バッファに積むだけ。実際の UART 書き込みは下の
    //    s5tx.service() が空きぶんずつ進める (ブロックさせない)。
    if (S5::USE_IM920 && telem_tick.ready()) {
        S5Tel::tick((int)g_mode, S5::USE_SBUS ? sbus.des[Ch::THR] : 0.0f);
    }
    // 送りかけのテレメトリを毎ループ吐き出す (非ブロッキング)
    if (S5::USE_IM920) s5tx.service();

    // ログ中は画面表示を止める。同じUSBシリアルを奪い合うと
    // ログが落ちるうえ、logger.py 側のパースも乱れる。
    if (!FlightLog::Usb::active && debug_tick.ready()) printStatus(main_tick.dt_us);
}
