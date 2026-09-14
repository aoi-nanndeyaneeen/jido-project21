# 🛩️ Jido-Project (RC Plane / Drone 3D Tracking & Autopilot System)

自律飛行機能を持つラジコン飛行機・ドローンのための総合システムプロジェクトです。
機体に搭載したセンサ群からのテレメトリデータと、地上に設置した2台のカメラ（ステレオビジョン）による画像認識を統合することで、リアルタイムな3D自己位置推定およびオートパイロット（自動操縦）を実現します。

## 🌟 システム全体構成 (System Architecture)

本プロジェクトは大きく分けて3つのサブシステムから構成されています。

### 1. 🛩️ Flight Controller (機体側制御システム)
* **ディレクトリ**: `flight_controller/`
* **概要**: Teensy等を用いたカスタムフライトコントローラー。PlatformIOベースで開発。
* **主要ハードウェア**: Teensy 4.1 / 4.0, MPU6050 (6軸センサ), BMP280 (気圧センサ), SBUS受信機, IM920 (920MHz無線通信), PWMサーボ / ESC
* **機能**: 
  - 機体の姿勢（Roll, Pitch, Yaw）と加速度、高度の取得
  - テレメトリデータ（センサ値）の地上への送信
  - 地上からのRCコマンドを受信し、自律飛行やオートジャイロ機能などの実行。各種機体（ドローン、双発機、トレーナー、デルタ翼）用のファームウェア環境を切り替え可能。
  - 現在のメイン開発ターゲットは `src/drone.cpp` (自律飛行対応版)

### 2. 📡 Ground Receiver (地上局通信レシーバー)
* **ディレクトリ**: `ground_receiver/`
* **概要**: 機体とPC間の通信を中継するレシーバー。
* **主要ハードウェア**: RP2040 (Raspberry Pi Pico等), IM920SL
* **機能**:
  - 機体から送られてくるIM920無線パケットを受信。
  - 受信データをPC（Python）が読み取りやすいCSV形式等にパースし、USBシリアル通信でPosition Estimatorへと渡す。

### 3. 💻 Position Estimator (PC側システム・オートパイロット)
* **ディレクトリ**: `position_estimator/`
* **概要**: ステレオビジョンによる3D自己位置推定および、機体を制御するオートパイロットシステム。
* **環境**: Python 3.x (OpenCV, numpy, matplotlib, pyserial)
* **構成**:
  - **Camera 1 (Laptop)**: PC直結のUSBカメラ。
  - **Camera 2 (Raspberry Pi)**: リモートカメラ（`camera_server.py`を実行）。ソケット通信でPCへ検知データを送信。
* **機能**:
  - フレーム差分法による動体検知と、2台のカメラを用いた三角測量（3Dトラッキング）。
  - ノイズ除去（PixelJumpFilterやresidualフィルタ）、未検出時の仮想円軌道フォールバック。
  - **Autopilot**: 自己位置推定結果を元に、目標軌道（例: 四角形パトロール）を追従するRCコマンド（スロットル、ピッチ、ロール、ヨー）の算出。

---

## 🛠️ ハードウェアのセットアップ

### Flight Controller
* **マイコン**: Teensy 4.1 または 4.0
* **配線例 (MPU6050 & BMP280)**: I2C (SDA -> Pin 18, SCL -> Pin 19)
* **IM920SL**: UART (Serial3 TX/RX など)

### Ground Receiver
* **マイコン**: RP2040
* **IM920SL**: UART (クロス接続)

### Position Estimator (カメラシステム)
* **PC側**: `pip install opencv-python numpy matplotlib pyserial` を実行
* **RPi側**: `sudo apt install python3-opencv -y` を実行
* 双方をLANで直結し、固定IPを割り当てて通信を行います。

---

## 🚀 ビルド＆実行方法

### 1. Flight Controller / Ground Receiver のビルド
VSCode + PlatformIO を推奨します。
1. `flight_controller` または `ground_receiver` フォルダを開く。
2. PlatformIO の Project Tasks から対象の環境 (例: `env:drone`) を選択。
3. `Build` 後、マイコンへ `Upload` します。

### 2. トラッキングとオートパイロットの実行
1. **RPi側**: `position_estimator/camera_server.py` を起動。
2. **PC側**: `position_estimator/src` ディレクトリに移動し、`python main.py` を実行。
3. **キャリブレーション**: 各カメラ映像でフィールドの基準5点をクリックし、3D空間をキャリブレーションします。
4. トラッキングが開始されると、各種UI（Velocity、Graph、RC Command等）が表示されます。

---

## 🎮 地上局コンソール (見ながら指令を出す)

テレメトリを1画面に出しながら、キー入力で機体へ指令を出せる対話コンソールです。
「少しだけ動かして挙動を見る」ための道具で、`main.py` (ウェイポイント自動飛行) とは
**同時に起動できません** (地上局のUSBポートは1プロセスしか開けないため)。

詳しい使い方 (画面の見方・キー一覧・ログの統合) は
[GROUND_CONSOLE_GUIDE.md](GROUND_CONSOLE_GUIDE.md) を参照。

```bash
cd position_estimator/src
python console.py            # 地上局を自動検出して起動。ログは自動で全部取る
python console.py --no-ble   # BLE(機体125Hzログ)だけ止めたいとき
```

`main.py` (ウェイポイント自動飛行) も同様に、起動するだけでテレメトリ・
カメラ・ミッションログに加えてBLE(機体125Hzログ)も自動で取る
(`utils/config.py` の `BLE_LOG_ENABLED`)。

| キー | 動作 |
|------|------|
| `h` / `t` / `g` / `l` | HOLD / TAKEOFF (2回押し) / GUIDED / LAND |
| `SPACE` | ABORT (= 機体を即その場ホールド。**緊急停止ではない**) |
| `x` | 送信停止 (機体は1秒でホールド → 4秒で自動着陸) |
| `w` `a` `s` `d` / `0` | 目標速度 (機体座標) の増減 / ゼロ |
| `r` `f` | 目標高度の増減 |
| `P` | PIDリセット (シリアル `r` キーと同じ) |
| `k` | IMU再キャリブレーション (2回押し。**機体が非アーム中のときだけ実行される**) |
| `i` | デバイス確認 / I2C再走査 (**機体が非アーム中のときだけ実行される**) |
| `S` `D` `Z` `C` | 地上局XIAOへ `s` `d` `z` `1` を転送 |
| `q` | 終了 (ABORTを送って切断) |

★ 緊急停止はプロポです。コンソールは「黙れば機体が降りてくる」構造に寄せてあります。

★ `P`/`k`/`i` は無線経由の単発指令 (`S5Cmd.h` の `Action`)。巡航指令 (`h`/`g` など) を
1度も送っていなくても使えるので、離陸前の地上チェックに使える。実行結果は画面の
`ACK` 行に出る (成功/アーム中で拒否/IMU校正が妥当性チェックで却下、のいずれか)。
`k`/`i` は遠隔操作者が機体に触れていないぶん、直結シリアルの `k`/`i` キーより
安全側に倒してあり、アーム中は機体側が黙って拒否する。

## 📊 ログ解析

飛行1回で3系統のログが出ます。時計が別々なので、`merge_logs.py` で1本に揃えてから
解析します。

| ログ | 出所 | 時刻 | レート |
|------|------|------|--------|
| `LOGnnnn.BIN` | 機体 → BLE (log_recorder) | 機体の `millis()` | 125Hz |
| `s5_link_*.csv` | 機体 → IM920 → 地上局 | **PC時刻 + 機体 `t_ms`** | 10〜15Hz |
| `flight_*.csv` / `mission_*.csv` / `console_*.csv` | PC | PC時刻 | 5〜60Hz |

機体にRTCは無いので、BIN単体では西暦の時刻が分かりません。橋渡しになるのは
`s5_link_*.csv` だけで、これが「受信した瞬間のPC時刻」と「機体の millis」を同じ行に
持っています。

```bash
cd flight_controller
python scripts/merge_logs.py                 # 既定の置き場から最新の1組を拾う
python scripts/merge_logs.py --dry-run       # 時刻合わせの結果だけ見る
python scripts/merge_logs.py --plot          # 合っているかを図で確認する
```

出力 `*_merged.csv` は 1行 = 機体ログ1レコードで、`epoch_s` / `t_rel_s` に続けて
機体の全列・`tl_`(テレメトリ)・`cam_`(カメラ)・`ms_`(ミッション)・`cmd_`(指令) が並びます。

★ 絶対時刻は「真の時刻 + 無線の片道最小遅延」です (往復を測っていないため数十msぶん
全体が遅い方向に揃う)。ログどうしの相対比較には効きません。

個別のログを見るだけなら `position_estimator/graph.py`、`flight_controller/scripts/plot_s5.py`、
`ground_receiver/tools/analyze_poshold.py` がそれぞれ使えます。