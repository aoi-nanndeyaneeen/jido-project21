# Position Estimator

複数台のカメラ（ローカル接続およびリモート接続）を用いて対象の位置推定を行うシステムです。
Raspberry Piなどのリモート環境にカメラサーバーを配置し、ソケット通信で座標データのみを取得することで、ネットワーク負荷を抑えつつ高速に位置を算出できます。

---

## 📂 フォルダ構成と役割

```text
position_estimator/
├── camera_server.py        # [リモートカメラ用] Raspberry Pi等のカメラ側で動かすソケットサーバー
├── remote_camera.py        # [クライアント用] camera_server から座標データを受け取る通信クラス
├── test_remote.py          # [テスト用] リモートカメラからの接続および受信テストスクリプト
├── graph.py                # 保存されたフライトログ(CSV)を可視化するグラフプロットツール
├── standalone_logger.py    # フライトデータをシリアル通信で受信しログ保存するTkinter製GUIツール
├── requirements.txt        # 依存Pythonライブラリ一覧
├── src/                    # 本システムのメインソースコード群
│   ├── main.py             # メイン実行エントリーポイント（位置推定と表示）
│   ├── core/               # システムのコアロジック層
│   │   ├── camera.py       # ローカルカメラの制御、フレームキャプチャ、画像処理
│   │   ├── communication.py # 外部システムやPLC等とのシリアル/ネットワーク通信制御
│   │   ├── controller.py   # 全体フローの制御・統括
│   │   ├── geometry.py     # 2次元・3次元幾何計算、三角測量、位置推定アルゴリズム
│   │   └── tracker.py      # 物体追跡（トラッキング）ロジック
│   ├── ui/                 # ユーザーインターフェース層
│   │   ├── dashboard.py    # 総合ダッシュボードGUI
│   │   ├── view_3d.py      # 3Dによる位置表示ビュー
│   │   └── view_graph.py   # 2Dグラフ描画ビュー
│   └── utils/              # 共通ユーティリティ層
│       ├── config.py       # 各種設定パラメータの読み込みと管理
│       └── logger.py       # システム共通ログ出力モジュール
└── tools/                  # 開発・デバッグ用ツール群
    ├── dummy_source.py     # テスト用のダミー光源・カメラデータシミュレーター
    ├── find_camera.py      # 接続されているローカルカメラのデバイスID探索スクリプト
    ├── receiver.py         # ソケット通信データ受信テスト用スクリプト
    ├── sensor_receiver.py  # センサーデータ受信テスト用スクリプト
    └── test.py             # 個別コンポーネント用テストスクリプト
```

---

## 🛠️ 主要コンポーネント詳細

### 1. リモートカメラ通信 (`camera_server.py` / `remote_camera.py`)
ネットワーク経由でカメラ画像を解析し、座標だけをメインシステムに送信する仕組みです。
- **`camera_server.py` (送信側/リモート)**: 
  - Raspberry Pi等で動作します。
  - OpenCVで取得したフレーム間で差分検出を行い、検出した座標（`pt`）のみを JSON 形式で送信します。
  - TCPバッファ詰まりによるフリーズを防ぐため、プレビュー画像転送 (`SEND_PREVIEW`) はデフォルトで `False` になっています。
- **`remote_camera.py` (受信側/メイン機)**:
  - `RemoteCamera` クラスを提供し、ネットワーク経由で送信側の座標データをリアルタイムにパースします。

### 2. メインシステム (`src/`)
- **位置推定のアルゴリズム (`src/core/geometry.py`)**:
  - 複数カメラの画像座標から、三角測量などの幾何計算を用いてリアルタイムに実空間上の位置を推定します。
- **3Dビジュアライザ (`src/ui/view_3d.py`)**:
  - 推定された物体の位置を3次元空間上にプロットし、リアルタイムにアニメーション描画します。

### 3. フライトデータロガー & グラフ化 (`standalone_logger.py` / `graph.py`)
- **`standalone_logger.py`**:
  - マイコン等から送信されるシリアル通信テレメトリ（Roll, Pitch, Yaw, Altitude）をリアルタイム表示し、CSVファイルに記録します。
- **`graph.py`**:
  - 保存されたCSVログファイルを読み込み、姿勢角（角度）と相対高度をMatplotlibでグラフ描画・解析します。

---

## 🚀 使い方（リモートカメラのテスト手順）

### 1. リモート側 (例: Raspberry Pi) でサーバー起動
```bash
# RPi端末で実行
python camera_server.py
```
*※ `camera_server.py` 内の `CAMERA_ID` や `PORT` は環境に合わせて調整してください。*

### 2. ローカル側 (メインPC) でテスト受信
`test_remote.py` の `sock.connect(("192.168.11.13", 5555))` のIPアドレスを、サーバーのIPアドレスに書き換えます。

```bash
# Windows PowerShell等で実行
python test_remote.py
```

接続が成功すると、サーバー側で検出された座標情報 `{'pt': [x, y]}` または `{'pt': None}` がリアルタイムにコンソールへ出力されます。
