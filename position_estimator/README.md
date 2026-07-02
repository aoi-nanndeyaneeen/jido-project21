# 🛩️ RC Plane / Drone 3D Position Estimator & Autopilot

ステレオビジョン（2カメラ）による自己位置推定と、簡易オートパイロットを組み合わせたシステムです。
1台はラップトップ直結USBカメラ、もう1台はRaspberry Pi経由のリモートカメラで、フィールド上の機体をリアルタイムに3D追跡します。

## 🌟 システムアーキテクチャ

```
[Camera 1: Laptop USB]         [Camera 2: RPi USB]
        │                              │
   フレーム差分法                 フレーム差分法
   (core/camera.py)          (camera_server.py, RPi上)
        │                              │
        └──────────┬───────────────────┘
                    │ ソケット通信 (JSON over TCP)
                    ▼
         [core/tracker.py] 別スレッドで実行
                    │
        ① PixelJumpFilter（画像空間で誤検知除去）
        ② 三角測量 (core/geometry.py)
        ③ residual フィルタ（3D空間の外れ値除去）
        ④ 30フレーム未検出でダミー円軌道にフォールバック
                    │
                    ▼
         [core/autopilot.py] SquarePatrol
         位置PIDで四角形パトロールのRCコマンド算出
                    │
                    ▼
      ui/view_rc.py で操縦スティックを可視化（現状は表示のみ）
```

## 📁 ディレクトリ構成

```text
position_estimator/
├── camera_server.py         # [RPi上で実行] カメラサーバー（CALIB/STREAMモード対応）
├── calib/                   # 保存済みキャリブレーションデータ（JSON）
├── logs/                    # フライトログ（CSV）
└── src/
    ├── main.py               # エントリーポイント（初期化～メインループ）
    ├── core/
    │   ├── camera.py          # ローカルUSBカメラ制御・フレーム差分検知
    │   ├── remote_camera.py   # RPiカメラとのソケット通信クライアント
    │   ├── tracker.py         # 3D位置推定スレッド（ジャンプフィルタ・ダミーフォールバック含む）
    │   ├── geometry.py        # レイキャスト・三角測量・座標変換
    │   ├── autopilot.py       # 位置PIDによる四角形パトロール（SquarePatrol）
    │   ├── dummy_flight.py    # カメラ未検出時の仮想円軌道
    │   ├── controller.py      # 高度制御PID（シリアル送信用）
    │   └── communication.py   # RP2040とのシリアル通信（高度・加速度受信）
    ├── ui/
    │   ├── dashboard.py       # 高度・XYグラフ表示ウィンドウ
    │   ├── view_graph.py      # matplotlibグラフ描画
    │   ├── view_velocity.py   # トップビュー・速度・姿勢指示器（ADI）
    │   └── view_rc.py         # RCスティックプレビュー（純OpenCV描画）
    ├── utils/
    │   ├── config.py          # 全パラメータ設定
    │   ├── calib_store.py     # キャリブレーション結果のJSON保存・復元
    │   └── logger.py          # CSVフライトログ
    └── tools/
        └── test_cam2.py       # Camera2接続の単独診断スクリプト
```

## 🛠️ セットアップ

### PC側（ラップトップ）

```bash
pip install opencv-python numpy matplotlib pyserial
```

### RPi側

```bash
sudo apt install python3-opencv -y
```

## 🔌 ネットワーク構成

- ラップトップとRPiを有線LANで直結、双方に固定IPを設定（例: `192.168.11.1` / `192.168.11.13`）
- SSH公開鍵認証を設定しておくとパスワード入力なしで運用できる
- `camera_server.py` はsystemdサービス化して自動起動させるのを推奨

## 🚀 使い方

### 1. RPi側でカメラサーバー起動

```bash
cd ~/position_estimator
python camera_server.py
```

- 起動直後からラズパイのディスプレイにフルスクリーンでカメラ映像が表示される
- 接続前でも画面をクリックしてキャリブレーション点を事前選択できる
- 画面右上に `EXIT` / `RESET` ボタンあり

### 2. ラップトップ側でメインプログラム起動

```bash
cd position_estimator/src
python main.py
```

起動シーケンス：

1. **カメラ初期化** - Camera1（ローカル）・Camera2（RPi疎通確認）
2. **シリアル通信**（オプション） - 接続失敗時は自動スキップ
3. **ログファイル準備**
4. **キャリブレーション** - 保存済みデータがあれば使用するか選択可能。新規の場合はCamera1はPC画面で、Camera2はRPi画面でそれぞれ5点をクリック

### キー操作

| キー | 動作 |
|------|------|
| `B` | 背景差分のリセット（両カメラ） |
| `Q` | 終了 |
| `T` | 目標高度を入力（ターミナル） |

### 表示ウィンドウ

| ウィンドウ | 内容 |
|-----------|------|
| Camera 1 / Camera 2 | 各カメラの映像・検知点 |
| Velocity | トップビュー俯瞰・速度/高度メトリクス・姿勢指示器(ADI) |
| Graph | 高度・XY位置の時系列グラフ |
| RC Command | オートパイロットが算出する操縦スティック値のプレビュー |

## 🎯 誤検知対策（PixelJumpFilter）

各カメラの画像内で、前フレームからの移動距離が `PIXEL_SPEED_LIMIT_PX_S × 経過時間` を超えた場合、その検知をノイズとして棄却します。ただし同じ新しい位置に `JUMP_RECOVERY_FRAMES` 回連続で検知され続けた場合は「本当の移動」とみなして受理します。3D空間での`residual`フィルタと組み合わせた二段構えです。

## 🔁 カメラ切断への耐性

- RPi側 `camera_server.py` はクライアント切断後も自動でaccept待機に戻るため再起動不要
- ラップトップ側で `main.py` を再起動しても、RPi側はそのまま起動し続けていて問題ない
- キャリブレーションは `calib/Camera1.json` / `calib/Camera2.json` に保存され、次回起動時に再利用するか選べる

## 🤖 オートパイロット（現状）

`core/autopilot.py` の `SquarePatrol` が、高度3m・1辺6mの正方形を反時計回りに周回するウェイポイントを生成し、位置PIDでRCコマンド（throttle/pitch/roll/yaw）を算出します。機首方向は9軸センサ未導入のため、暫定的に速度ベクトルから推定しています。

現在は `RC Command` ウィンドウで算出値を表示するのみで、実機への送信（IM920SL経由）は未実装です。

## 📊 ログの分析

```bash
python graph.py
```

保存されたCSVから飛行軌跡の3Dグラフと時間変化グラフを生成します。

## ⚙️ 主要config.pyパラメータ

| パラメータ | 内容 |
|-----------|------|
| `FIELD_POINTS` | フィールドの基準5点（キャリブレーション用） |
| `MAX_RESIDUAL_M` | 3D三角測量の外れ値許容誤差 [m] |
| `PIXEL_SPEED_LIMIT_PX_S` | 画像空間でのジャンプ検知速度上限 [px/s] |
| `JUMP_RECOVERY_FRAMES` | ジャンプが何フレーム連続したら「本物の移動」と認めるか |
| `DUMMY_FALLBACK_FRAMES` | ダミー軌道に切り替えるまでの未検出フレーム数 |
| `DUMMY_ORBIT_RADIUS` / `DUMMY_ORBIT_ALT` / `DUMMY_ORBIT_PERIOD` | ダミー円軌道のパラメータ |
| `VELOCITY_SMOOTH_FRAMES` | 速度計算の平滑化フレーム数 |
| `DISP_W` / `DISP_H` / `VELOCITY_W` / `VELOCITY_H` | 各ウィンドウの表示サイズ |

## 📌 今後の課題

- PIDゲインのホットリロード（JSONファイル経由での実行中調整）
- 9軸センサ導入後、機首方向をセンサ値に切り替え
- IM920SL経由での実機へのRCコマンド送信

## 🗺️ ファイル依存関係

`main.py` を頂点に、下位モジュールへ一方向に依存する構造になっています。矢印は「import している側 → import されている側」を表します。

```
src/main.py
  ├─→ init/camera_setup.py
  │      ├─→ core/camera.py         (CameraTracker)
  │      └─→ utils/config.py
  │
  ├─→ init/calibration_flow.py
  │      ├─→ core/remote_camera.py  (RemoteCamera)
  │      ├─→ utils/calib_store.py   (save/load/ask_use_saved)
  │      └─→ utils/config.py
  │
  ├─→ app/main_loop.py
  │      ├─→ core/tracker.py        (camera_thread_func)
  │      │      ├─→ core/geometry.py       (get_ray, intersect_rays)
  │      │      ├─→ core/dummy_flight.py   (DummyFlight)
  │      │      └─→ utils/logger.py        (FlightLogger)
  │      │
  │      ├─→ core/controller.py     (AltitudeController)
  │      ├─→ core/autopilot.py      (SquarePatrol, RCCommand)
  │      ├─→ core/geometry.py       (accel_to_angles)
  │      │
  │      ├─→ ui/dashboard.py
  │      │      └─→ ui/view_graph.py
  │      ├─→ ui/view_velocity.py
  │      └─→ ui/view_rc.py
  │             └─→ core/autopilot.py  (RCCommand 型のみ参照)
  │
  └─→ core/communication.py         (SerialReceiver、SERIAL_ENABLED時のみ)
```

### レイヤーごとの役割

| レイヤー | フォルダ | 役割 | 依存してよい方向 |
|---------|---------|------|-----------------|
| エントリーポイント | `main.py` | 起動シーケンスの呼び出しのみ | `init/`, `app/` |
| 初期化 | `init/` | カメラ接続・キャリブレーションの一度きりの処理 | `core/`, `utils/` |
| 実行ループ | `app/` | メインループ・ウィンドウ管理・呼び出しの統括 | `core/`, `ui/`, `utils/` |
| ドメインロジック | `core/` | 位置推定・制御・通信などの中核処理 | `utils/` のみ |
| 表示 | `ui/` | OpenCV/matplotlibによる可視化 | `core/`（型参照のみ）, `utils/` |
| 共通基盤 | `utils/` | 設定・保存・ログ。他レイヤーに依存しない | なし（最下層） |

**原則**：矢印は下方向にのみ流れます。`core/` が `ui/` や `init/` を import することはありません。`utils/` は誰にも依存しない最下層です。この向きが崩れる変更（例：`core/tracker.py` が `ui/dashboard.py` を直接呼ぶ、など）をする場合は設計上の警告サインと考えてください。

### RPi側（別プロセス）

```
camera_server.py（RPi上で単独実行、position_estimator本体には import されない）
  └─→ OpenCV, socket, json のみに依存
```

`camera_server.py` は独立したPythonプロセスとしてRPi上で動くため、`src/` 以下のどのモジュールからも import されません。通信は `core/remote_camera.py` がソケット経由で行うのみです。