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

## 📊 ログ解析
PC側でトラッキング・フライトを行ったデータは保存され、`position_estimator/graph.py` 等を実行することで、飛行軌跡の3Dグラフや時系列データ分析が可能です。