# Jido-Project Flight Controller

Teensy を用いたカスタムフライトコントローラー。PlatformIO でビルド・依存関係管理を行っています。
1つのリポジトリで複数の機体（クアッドコプター / 双発機 / オートジャイロ / トレーナー機）を扱います。

---

## 1. ビルドの仕組み — 「全部のファイルが動いている」わけではない

`src/` には `.cpp` が10個以上ありますが、**同時にビルドされるのは常に1つだけ**です。
`platformio.ini` のこの行がすべてを決めています。

```ini
build_src_filter = -<*> +<sub_lib/*.cpp> +<${PIOENV}.cpp>
```

意味は「まず `src/` 以下を**全部除外**し、そのあと `sub_lib/` と
**選んだ環境名と同じ名前の .cpp だけ**を戻す」です。

つまり:

| 実行するコマンド | 実際にビルドされるファイル |
|---|---|
| `pio run -e drone` | `src/drone.cpp` + `src/sub_lib/*.cpp` |
| `pio run -e drone_s4` | `src/drone_s4.cpp` + `src/sub_lib/*.cpp` |
| `pio run -e auto_flight` | `src/auto_flight.cpp` + `src/sub_lib/*.cpp` |

`drone.cpp` と `drone_s1`〜`s4.cpp` の5つが**同時に動くことはありません**。
それぞれが独立した `setup()` / `loop()` を持つ、別々のファームウェアです。

何もオプションを付けずに `pio run` とだけ打った場合は、`default_envs` の環境が選ばれます。

```ini
[platformio]
default_envs = auto_flight     ; ← 現在の既定。クアッドを触るときは -e で明示すること
```

**注意:** 既定が `auto_flight`（双発固定翼）のままなので、クアッドをビルドするときは
必ず `-e drone_s4` のように環境を明示してください。

---

## 2. なぜ drone_s1〜s4 が増えたのか（経緯）

git の記録では次の順です。

| コミット | 日付 | 内容 |
|---|---|---|
| `ca0eb62` | 2026-05-28 | `drone.cpp` 追加。クアッドはこれ1本だった |
| `ff7bd30` | — | 周波数向上とPID調整 |
| **`c420f18`** | **2026-08-04** | **`drone_s1`〜`s4.cpp` と `include/quad/` を新規追加**（+2205行） |
| `92f387a` | — | `drone.cpp` に 500Hz ロガーとモーター単体テストを追加 |
| `930e4fb` | — | IM920SL のペイロードを 26→42 byte に拡張 |

`c420f18` で「**クアッドの制御を1本の巨大な `drone.cpp` から、段階的に検証できる4本に分割する**」
リファクタリングが入りました。`drone.cpp` は消さずに残してあるので、いつでも戻れます。

`drone.cpp` はその後も編集されていますが、**追加されたのはロガーとテレメトリだけ**で、
制御則そのものは `c420f18` 以前のままです。

### 段階分割（Stage）の考え方

各ステージは前のステージの前提の上に立っています。**順番を飛ばすと必ず事故ります。**

| 環境 | 内容 | プロペラ | 目的 |
|---|---|---|---|
| `drone_s1` | 出力段とセンサの生存確認 | **外す** | どのピンがどのモーターか、回転方向、SBUSチャンネル |
| `drone_s2` | X配置ミキサーとセンサ軸の確認 | **外す** | ロール指令で正しいモーターが速くなるか、符号は合っているか |
| `drone_s3` | レートPID（アクロ） | 付ける | **初飛行はここ**。内側ループのゲイン決め |
| `drone_s4` | 角度PID + モード切替 + ヨー保持 + 自律 + ログ | 付ける | 実運用はここ。RATE / ANGLE / AUTO の3モード |
| `esc_calib` | ESCキャリブレーション専用 | **外す** | 起動後10秒間 2000us を出力 |

Stage 1/2 で判明した機体固有の値（ピン配置、ミキサー係数、センサ符号）は
すべて **`include/quad/QuadConfig.h` の1箇所**に書きます。
制御ロジック側（`drone_s*.cpp`）は書き換えなくて済むようにしてあります。

---

## 3. 現在のステータス

**クアッドの開発対象は `src/drone_s4.cpp`（環境 `drone_s4`）です。**

`src/drone.cpp` は**レガシー**として残していますが、以下の理由で新規の作業には使いません。

- **ヨーがミキサーに入っていない。** `drone.cpp` のモーター混合は
  `raw1 = -Pitch.cmd + Roll.cmd` … という形で **`Yaw.cmd` を一切参照していません**。
  `Yaw.cmd` は計算されログにも出ますが、モーターには届いていません。
  さらに `writeServos()` は空関数です。つまり**ヨーは物理的に無制御**でした。
- レートPIDのゲインスケールが `drone_s4` と 2000倍違う（`kp_rate = 2.0` vs `0.0010`）。
  `drone.cpp` 側は誤差 10 deg/s で出力が上限に張り付く、実質バンバン制御になっています。
- 固定翼由来のコード（`BANK_ANGLE`, `RUDDER_COORD`, `LEVEL_PITCH_ANGLE`）が
  コピペで残っていて、クアッドでは意味を持ちません。

`drone.cpp` を使い続ける必要が出た場合は、まずミキサーへのヨー項の追加が必須です
（符号を間違えると正帰還になり離陸直後にスピンするので、必ず `drone_s2` の
手順でプロペラを外して確認してから）。

### 8/15 の作業内容は `drone_s4` に移植済み

2026-08-15 に `drone.cpp` へ入った2つの機能は、`drone_s4.cpp` に移植してあります。
`drone_s1〜s4` の追加（8/4）より後に `drone.cpp` 側だけに入っていたためです。

| 機能 | 元コミット | `drone_s4` での実装 |
|---|---|---|
| 500Hz ロガー | `92f387a` | `namespace Log`。**列名は `drone.cpp` と完全に同一**なので `scripts/analyze_log.py` を変更せずそのまま使える |
| 地上局からの自律制御 | `930e4fb` | `S4::MODE_AUTO`（`drone.cpp` の `MODE_AUTONOMOUS` 相当） |

移植にあたって以下を追加しました。

- `include/quad/Mixer.h`: `MixInfo` 構造体（`span_limit` / `scale` / `sat`）を
  出力する省略可能な引数を追加。`drone.cpp` のロガーが出していた `corr_limit` / `sat`
  と同じ役割。既存の呼び出しは変更不要
- `include/Telemetry.h`: `receive()` と `sendAttitudeOnly()` を追加。
  既存の `receiveAndProcess()` は `Axis_value` と `BarometerSensor` を引数に取るため、
  それらを持たない `drone_s4`（`Quad::Pid` を使い気圧計も積んでいない）からは呼べなかった

### AUTO モードの安全設計

`drone.cpp` の考え方をそのまま引き継いでいます。

- **`SW_AUTO`（ch9）が UP かつ IM920 の受信が 500ms 以内**のときだけ AUTO に入る。
  スイッチOFF、またはリンク途絶の瞬間に ANGLE へ自動フォールバックする
- **スロットルは全モード共通で常に物理プロポから取る。** `GroundData.ap_throttle` は
  あえて使っていません。これはコード全体の安全上の不変条件です
- 地上局のコマンドは「スティック値の代わり」として入るだけで、以降の制御経路は
  ANGLE モードと完全に同じ。自律制御専用のPID経路は存在しません

### その他の機体

| 環境 | 機体 | ボード |
|---|---|---|
| `auto_flight` | 双発固定翼・自律飛行 | Teensy 4.0 |
| `trainer` | トレーナー機 | Seeed XIAO RP2040 |
| `Zunrocoptor` | オートジャイロ | Teensy 4.1 |
| `delta_auto` | デルタ翼 | Teensy 4.1 |
| `uchida` / `test` | 実験用 | — |

> ⚠️ **`auto_flight` は現在ビルドできません。** `auto_flight.cpp` が `Ch::Aux1/Aux2/Aux3` を
> 参照していますが、これらは `ff7bd30`（2026-07-11）で `Config.h` の `enum Ch` から
> 削除されています（`SW_TURN` / `SW_LEVEL` / `SW_HOVER` などへの改名）。
> `platformio.ini` の `default_envs` がこの環境のままなので、**引数なしの `pio run` は失敗します。**
> クアッドを触るときは必ず `-e drone_s4` を明示してください。

---

## 4. ディレクトリ構成

```text
flight_controller/
├── platformio.ini          # ★ ビルド環境の定義。build_src_filter がすべての鍵
├── DEVELOPMENT_PLAN.md     # 振動・発熱問題の切り分け計画 (2026-08-09)
├── src/
│   ├── drone_s1..s4.cpp    # ★ クアッド 段階的開発 (現行)
│   ├── drone.cpp           #   クアッド 旧版 (レガシー。ヨー無制御)
│   ├── esc_calib.cpp       #   ESCキャリブレーション
│   ├── auto_flight.cpp     #   双発固定翼
│   ├── Zunrocoptor.cpp / trainer.cpp / uchida.cpp / test.cpp
│   └── sub_lib/
│       └── Actuators.cpp   #   全環境で共通リンクされる (motor::write の実体)
├── include/
│   ├── quad/               # ★ クアッド専用 (drone_s2 以降が参照)
│   │   ├── QuadConfig.h    #   機体固有の設定を集約: ピン/ミキサー係数/センサ符号/レート上限
│   │   ├── QuadPID.h       #   dt を引数で受け取る PID。測定値微分 (derivative kick 回避)
│   │   ├── Mixer.h         #   X配置ミキサー + 飽和処理 (airmode)
│   │   └── BodyFrame.h     #   IMU (FLU) → 機体座標 (FRD) の変換
│   ├── Config.h            # 制御周期 (MAIN_Hz = 1000)、センサスケール、チャンネル定義
│   ├── sensor/IMU.h        # MPU6050 + Madgwick
│   ├── sensor/Barometer.h  # BMP280
│   ├── Control.h           # 旧PID (drone.cpp / auto_flight.cpp が使用)
│   ├── Receiver.h          # SBUS
│   ├── Telemetry.h         # IM920
│   └── Actuators.h, flight_mode.h, serial_com.h
├── lib/                    # ローカルライブラリ (MPU6050, Madgwick, SBUS など)
├── scripts/
│   ├── logger.py           # 500Hz ログ収集 (drone.cpp のロガーと対)
│   ├── analyze_log.py      # FFT で振動のピーク周波数を出す
│   └── monitor.py / gui.py
└── logs/
```

---

## 5. ハードウェア構成（クアッド）

| 役割 | 部品 | 備考 |
|---|---|---|
| マイコン | Teensy 4.0 | `drone_s*` は 4.0。`Zunrocoptor` などは 4.1 |
| IMU | MPU6050 | **6軸（磁気センサなし）**。I2C 400kHz、±2g / ±250dps、DLPF 42Hz |
| 気圧 | BMP280 | オプション（`drone_s4` では未使用） |
| 受信機 | SBUS | `drone_s4` は `Serial5` |
| テレメトリ | IM920SL | `drone_s4` は `Serial3`（`drone.cpp` と同じ）。19200 baud、`GroundData` 42 byte |
| 出力 | ESC ×4 (PWM 1000-2000us) | ピンは `QuadConfig.h` の `MOTOR_PIN` |

制御ループ: **レートPID 1000Hz / 角度PID 200Hz / シリアル表示 10Hz**

---

## 6. ヨー方向の安定（`drone_s4`）

### 設計方針: 絶対方位は使わない

市販のフライトコントローラも同じですが、**ヨーの姿勢制御に絶対方位（磁気センサ/GPS）は使いません。**
必要なのは「今向いている方向を保つ」ことだけで、それが北から何度かを知る必要はないからです。
絶対方位は Return-to-Home やウェイポイント飛行など、**フィールド座標が要る機能**のためだけに使います。

`IMU.h` は `filter.updateIMU()`（Madgwick の**6軸版**）を呼んでいるため、
`mpu.getYaw()` にはヨーの補正項が一切入りません。**ジャイロZの純積分そのもの**で、
絶対角としては最初から意味を持ちません。`drone_s4` はこの値を制御に使っていません。

### 実装（2段構え）

**① レートPIDの I項**（`Gain::RATE_YAW`）

```cpp
constexpr float RATE_YAW[3] = { 0.0020f, 0.0020f, 0.0f };  // kp, ki, kd
```

機体には必ず一定のヨートルクが残っています（モーター取付角のわずかな傾き、
CW/CCW プロペラの特性差、モーターのKV差、ESCの個体差）。
**P制御は定常偏差を消せない**ので、これらが残る限り機体は一定の角速度で回り続けます。
I項がこれを吸収します。`ki = kp` は積分時定数 1秒に相当。

**② ヘディングホールド**（`§ 7-2` in `drone_s4.cpp`）

I項は「角速度を0にする」ところまでしか保証しません。突風で30度振られたら、
その30度は戻ってきません。そこでジャイロZを積分した**相対**方位を保持します。

```
スティックを触っている間 → 素直にレート指令（従来どおり）
中立に戻した瞬間        → そのときの方位を目標として保持
```

- 積分値のドリフトは**無関係**です。「今向いている方向を保つ」ことに変わりはありません
- 相対値なので **±180度の折り返し処理が不要**（連続量のまま扱える）
- 積分には制御に使っているジャイロ値をそのまま使うので、レートループと位相が完全に揃います
- `YAW_HOLD_ERR_LIM`（±20度）で頭打ちにしてあり、大きく振られても全力で振り戻しません

### 調整パラメータ

`src/drone_s4.cpp` の `namespace Gain`:

| 定数 | 既定値 | 意味 |
|---|---|---|
| `RATE_YAW[1]` (ki) | `0.0020` | 遅ければ 0.005 まで。上げすぎると1Hz前後で揺れる |
| `YAW_HOLD_KP` | `3.0` | 方位誤差1度あたり何 deg/s で戻すか |
| `YAW_HOLD_RATE_LIM` | `60.0` | 保持が出してよい角速度の上限 [deg/s] |
| `YAW_HOLD_ERR_LIM` | `20.0` | これ以上の方位誤差は追わない [deg] |
| `YAW_STICK_DEAD` | `0.03` | ラダースティックの不感帯 |

`YAW_HOLD_KP` はシリアルの `p` → `h` から飛行中でも変更できます（**0 にすると保持が切れて
従来のレートのみに戻る**ので、切り分けに使えます）。

### 残る誤差要因: ジャイロZのバイアス

ヘディングホールドは「推定ヨーを止める」制御なので、**ジャイロにバイアス `b` があると
機体は実際には `-b` の速度で回り続けます。** MPU6050 のZ軸バイアスは温度で 0.5〜2 deg/s
動くので、0.5 deg/s でも1分で30度です。対策:

1. **離陸直前に `k` で再キャリブレーション**（電源投入から1〜2分暖機してから）
2. **IMUのソフトマウント**（制振フォーム/ゲル）— 振動の整流でバイアスが飛行中だけシフトするのを防ぐ。断熱にもなり温度ドリフトも緩む
3. **プロペラのバランス取り** — 一番安く効く
4. 温度補償（MPU6050 レジスタ `0x41-0x42` の内蔵温度計で `gz_bias` を一次補正）

シリアル画面の `[ヨー保持]` 行の「誤差」がじわじわ片側に増え続けるなら、
機体が回っているのではなくバイアスが残っている証拠です。

### 絶対方位が必要になったら

外部カメラ（`position_estimator`）の位置推定から機首方位を割り出して IM920 で降ろし、
弱いゲインで `g_yaw_est` を引っ張る形を想定しています。**レートループには入れません。**

```cpp
const float CORR_GAIN = 0.02f;              // 弱く
float diff = wrap180(yaw_abs - g_yaw_est);
g_yaw_est  += CORR_GAIN * diff;
g_yaw_hold += CORR_GAIN * diff;             // ★ 目標も一緒に動かすのが肝
```

`g_yaw_hold` も同量ずらすのがポイントです。推定値の座標系が絶対座標へゆっくり合っていくだけで、
**保持している物理的な向きは動かない** → 補正が入っても機体が振れません。

---

## 7. 開発・ビルド方法

VSCode + PlatformIO 拡張機能を推奨。CLI では:

```bash
# クアッド（現行）
pio run -e drone_s4 -t upload
pio device monitor -e drone_s4

# 段階を戻す場合
pio run -e drone_s1 -t upload    # ★プロペラを外すこと
```

### drone_s4 のシリアルコマンド

| キー | 動作 |
|---|---|
| `p` | ゲイン調整メニュー（**モーターを停止し制御ループも止める**） |
| `k` | IMU再キャリブレーション（機体を水平に置いて動かさない） |
| `r` | PID内部状態とヘディングホールドのリセット |
| `l` | 500Hz ログの開始/停止 |

ゲイン調整メニュー: `1-9` レートPID、`a-f` 角度PID、`h` ヘディングホールドkp、`q` 抜ける。

### ログ解析

```bash
pio run -e drone_s4 -t upload
python scripts/logger.py         # 別ターミナル。シリアルモニタは閉じる
                                 # 'L' + Enter で記録開始 / もう一度で停止
python scripts/analyze_log.py --plot
```

ログ記録中は10Hzの画面表示が自動的に止まります（同じUSBシリアルを奪い合うと
ログが落ち、`logger.py` のパースも乱れるため）。

`analyze_log.py` が出すFFTのピーク周波数で振動の原因が切り分けられます
（5-30Hz = 制御発振 / 30-80Hz = 構造共振 / 80Hz- = プロペラ・モーター）。
詳細は `DEVELOPMENT_PLAN.md`。
