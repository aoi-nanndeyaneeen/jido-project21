# 🛩️ ラジコン飛行機 3D自己位置推定システム (RC Plane 3D Tracking System)

一定のフィールド内を飛行するラジコン飛行機の自己位置推定・可視化プログラムです。
機体に搭載した気圧センサと姿勢センサのデータを無線で地上に送信し、地上のPCカメラ（一眼レフ）で取得した2D座標と統合することで、リアルタイムな3Dトラッキングを実現します。

## 🌟 システムアーキテクチャ

システムは以下の3つのモジュールで構成されています。

1. **Flight Controller (機体側 - Teensy 4.0)**
   * MPU6050 (6軸センサ) で機体の姿勢（Roll, Pitch, Yaw）と加速度を取得。
   * BMP280 (気圧センサ) で機体の高度（Z座標）を算出。
   * 取得データを IM920SL (920MHz帯無線モジュール) で地上へ送信。
2. **Ground Receiver (地上側 - RP2040)**
   * 機体から飛んでくる無線パケットを IM920SL で受信。
   * 受信したデータをPC（Python）が読み取りやすいCSV形式(`DATA,高度,ax,ay,az`)に変換し、USBシリアル通信で出力。
3. **Position Estimator (PC側 - Python 3.x)**
   * 一眼レフカメラの映像から動体検知を用いて機体の画面内座標(2D)を追跡。
   * OpenCV (`solvePnP`) を用いたカメラキャリブレーションと射影変換による方向ベクトルの算出。
   * 受信機(RP2040)から取得した高度(Z)と角度を交差させ、3D空間上の絶対座標(X, Y, Z)をリアルタイムに計算・描画。

## 🛠️ セットアップ (Setup)

### 1. 依存ライブラリのインストール
PC側で以下のコマンドを実行してください。
```bash
pip install opencv-python pyserial matplotlib numpy pandas
```

## 🔌 接続構成 (Hardware Connections)

### Flight Controller (Teensy 4.0)
* **MPU6050 (I2C)**: 
    * SDA -> Pin 18, SCL -> Pin 19
* **BMP280 (I2C)**: 
    * SDA -> Pin 18, SCL -> Pin 19
* **IM920SL (UART)**: 
    * TX -> Pin 14 (Serial3 TX), RX -> Pin 15 (Serial3 RX)

### Ground Receiver (RP2040 / Raspberry Pi Pico)
* **IM920SL (UART)**: 
    * 近傍のUARTピン（C++コード上で `Serial1` を使用）に接続。
    * TX/RX をクロスで接続。

## 📁 ディレクトリ構成

\`\`\`text
.
├── flight_controller/   # 機体側プログラム (PlatformIO / C++)
├── ground_receiver/     # 地上側プログラム (PlatformIO / C++)
└── position_estimator/  # PC側プログラム (Python)
    ├── main.py          # メイン実行ファイル
    ├── camera.py        # カメラ制御・動体検知モジュール
    ├── communication.py # シリアル通信モジュール
    └── graph.py         # ログデータ分析・グラフ化
\`\`\`

## 🚀 使い方 (How to Use)

### 1. ハードウェアの準備
* 機体側のTeensy 4.0に電源を入れます。
* 地上側のRP2040を受信PCのUSBポートに接続します。

### 2. ソフトウェアの起動
PC側のターミナルを開き、`position_estimator` ディレクトリ内で以下のコマンドを実行します。

\`\`\`bash
python main.py
\`\`\`
*(※ `communication.py` 内の `COM` ポート番号がRP2040のものと一致しているか確認してください)*

### 3. キャリブレーションとトラッキング
1. カメラのウィンドウが開いたら、現実のフィールドの基準点4箇所（左下→右下→右上→左上）を順番にクリックします。
2. `Enter` キーを押すとキャリブレーションが完了し、3Dトラッキングが開始されます。
3. トラッキング中のデータは自動的に `3d_tracking_log.csv` に保存されます。

## 📊 ログの分析
飛行テスト終了後、以下のコマンドで保存されたCSVデータから飛行軌跡の3Dグラフと時間変化グラフを生成できます。

\`\`\`bash
python graph.py
\`\`\`