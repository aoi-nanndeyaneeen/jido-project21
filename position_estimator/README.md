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
├── detection_params.json    # ★検知パラメータの唯一の正（PC/RPi共通。RPiにも配置する）
├── standalone_logger.py     # Tkinter製の単独シリアルロガーGUI（main.pyとは独立して使用可）
├── graph.py                 # 保存済みCSVから飛行軌跡を可視化するスクリプト
├── calib/                   # 保存済みキャリブレーションデータ（JSON）
├── logs/                    # フライトログ（CSV）
├── used/                    # 過去の可視化コードなど、現行フローでは未使用のレガシー置き場（例: view_3d.py）
├── tools/                   # 単体診断・補助スクリプト群
│   ├── calibrate_intrinsics.py # ★事前作業: チェッカーボードでK/歪みを実測
│   ├── test_cam2.py          # Camera2接続の単独診断スクリプト
│   ├── test_remote.py        # RPiのCALIB→STREAMフロー単独テスト
│   ├── find_camera.py        # 接続中のUSBカメラID探索
│   ├── dummy_source.py       # カメラ/センサ代替の合成データ生成モジュール
│   ├── receiver.py           # UDP受信の単独テスト
│   ├── sensor_receiver.py    # 高度センサ受信の単独テスト
│   └── test.py                # シリアル/カメラの雑多な動作確認スクリプト
└── src/
    ├── main.py               # エントリーポイント（初期化～メインループ）
    ├── test_view_rc.py       # RCスティック表示（ui/view_rc.py）の単体テスト
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
    └── utils/
        ├── config.py          # 全パラメータ設定
        ├── calib_store.py     # キャリブレーション結果のJSON保存・復元
        └── logger.py          # CSVフライトログ
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

## 📐 事前準備：内部パラメータの実測（大会当日は不要）

カメラの較正には性質の違う2つがあり、**やる場所とタイミングが違います**。

| | 内部パラメータ (K, dist) | 外部パラメータ (R, tvec) |
|---|---|---|
| 何を表すか | レンズ自体の性質（焦点距離・光学中心・歪み） | カメラをどこに置いてどこを向けたか |
| 変わるタイミング | レンズ・解像度を変えた時だけ | 置き直すたび |
| どうやって | チェッカーボード撮影 | フィールド5点クリック |
| **いつ** | **自宅で事前に1回。以後使い回す** | 会場で毎回（準備1分以内） |

```bash
# 事前に1回だけ。各カメラについて実行する
python tools/calibrate_intrinsics.py --label Camera1 --source 1
python tools/calibrate_intrinsics.py --label Camera2 --source 0
```

結果は `calib/intrinsics_<label>.json` に保存され、当日は自動で読み込まれます。

**重要**: 撮影は本番と同じ解像度・同じ無限遠フォーカスで行ってください。オートフォーカスが動くと焦点距離そのものが変わり、測った K が無意味になります。

> 実測しない場合は `FALLBACK_HFOV_DEG` の公称画角から概算しますが、これは暫定値です。数値実験では、K を誤ると**クリック精度をどれだけ上げても3D誤差が約2m残り**、正しい K なら同条件で約0.06mでした。ここが精度の支配要因です。

## 🎯 誤検知対策

### 検知（各カメラ）

背景差分（MOG2）で前景を取り、輪郭を**候補リストとして複数返します**（最大 `max_candidates` 個）。1点に絞りません。42m先だと機体も人も20px程度で、大きさも形も1枚の画像からは区別できないためです。

学習率 `bg_learning_rate` はほぼ0にしてあります。ドローンはホバリングするため、学習率が高いと静止した機体が背景に吸収されて消えます。

前景が画面全体の `vibration_reject_ratio` を超えたフレームは丸ごと破棄します（三脚の揺れ・照明の急変・自動露出の追従対策）。

### 選択（2カメラの幾何整合）

どれが機体かは**2台の幾何整合**で決めます。審査員席はフィールド外にあるため、正しくペアリングされれば `GATE_X/Y/Z` の範囲外として自動的に落ち、誤ったペアリングは2本のレイが空間で交わらないため `MAX_RESIDUAL_M` で落ちます。

> 現在 `tracker.py` は移行期のため、暫定的に `PixelJumpFilter`（画像空間での速度制限）を使っています。Phase B で上記の3Dゲートに置き換えます。

## ✅ キャリブレーション品質の検証

キャリブ完了時に2つの指標が自動表示されます。**当日、数秒でやり直すべきか判断できます。**

**再投影誤差 [px]** — 求めた R,tvec で3D点を画像に投影し直し、クリック位置とのずれを測ったもの。各カメラ内部の辻褄を見ます。

| 値 | 判断 |
|---|---|
| 〜2 px | 良好 |
| 2〜5 px | 許容（精度は期待できない） |
| 5 px 超 | やり直し推奨 |
| 20 px 超 | ほぼ確実にクリック順序ミスか座標入力ミス |

**三角測量誤差 [m]** — 両カメラでクリックした同じ5点を三角測量し、config の3D座標と比較。2台の相対関係を検証するもので、位置推定で実際に使う精度に直結します。

## 🧭 カメラ設置の指針

フィールド42m×26mの4隅を1台に収めるのに必要な水平画角（角から**対角線方向**への後退距離ごと）:

| 対角後退 | 必要な水平画角 | C920 (90°) | α6400+16mm (72.6°) |
|---|---|---|---|
| 0 m | 90.0° | ぎりぎり | ✕ |
| 4 m | 80.8° | ○ | ✕ |
| 8 m | 73.1° | ○ | ほぼ限界 |
| 10 m | 69.7° | ○ | ○ |

**後退方向ごとに必要な距離**（α6400 72.6° で4隅を収める場合）:

| 後退方向 | 必要距離 |
|---|---|
| 角から対角線方向 | 8.3 m |
| 角から奥行方向（−y）のみ | 8.1 m |
| 角から横方向（−x）のみ | 13.2 m |
| 手前辺の中央から後方 | 17.7 m |

**対角と真後ろはほぼ同等**（どちらも約8m）なので、体育館で場所が取れる方を選んでください。横方向のみ・辺の中央は大きく不利です。

光軸は**フィールド4隅の方位角の二等分線**に向けます。フィールド中央に向けるのは最適ではなく、数値実験では隅が画角から外れました。

4隅が入らない場合は、隅にこだわらず**画像内で広く散った既知点**を使ってください（`CALIB_PRESET` を `inner_85` などに変更）。

### 5点目（高さの点）について

4点が床の同一平面上にあるため、姿勢とスケールを決めているのは実質5点目だけです。ただし数値実験では**思ったより寛容**でした:

| 条件 | 3D誤差 |
|---|---|
| 高さ2mを正確にクリック | 0.08 m |
| 実際は1.80mだった（20cm誤差） | 0.11 m |
| 目印なしで±2m程度の当てずっぽう | 0.69 m |

**体育館の規格構造物を使うのが確実です**（バスケットゴール3.05m、バレーネット支柱など、高さが規格で決まっているもの）。人に棒を持ってもらう必要はありません。`CALIB_POLE_H` と `CALIB_PRESET` で調整できます。

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

## 🧰 単独ツール

- `standalone_logger.py` : `main.py`とは独立に、シリアルポートからのテレメトリ（Roll/Pitch/Yaw/Ax/Ay/Az/Alt）をTkinter GUIで記録するロガー。`logs/`にCSV出力。
- `tools/` : カメラ探索・ダミーデータ生成・RPi通信の単独テストなど、開発時の動作確認用スクリプト群（`main.py`からはimportされない）。

## ⚙️ 主要config.pyパラメータ

| パラメータ | 内容 |
|-----------|------|
| `FIELD_W` / `FIELD_D` | フィールド寸法 26m（幅）× 42m（奥行）。原点はフィールド中央 |
| **`CALIB_PRESET`** | **当日ここだけ書き換える**。基準5点の3D座標プリセット選択 |
| `CALIB_PRESETS` | 5点の3D座標そのもの。`custom` に直接書いてもよい |
| `CALIB_POLE_H` / `CALIB_POLE_AT_NEAR` | 5点目の高さ [m] と、手前／奥どちらの隅に立てるか |
| `REPROJ_WARN_PX` / `REPROJ_FAIL_PX` | 再投影誤差の警告・失格しきい値 |
| `TRIANG_WARN_M` / `TRIANG_FAIL_M` | 2カメラ三角測量誤差のしきい値 |
| `CAMERA_AUTOFOCUS` / `CAMERA_FOCUS_VALUE` | **必ず False / 0（無限遠）**。動くと K が無効になる |
| `LOCK_EXPOSURE_AFTER_CALIB` | キャリブ完了時点の露出値で固定するか |
| `USE_MEASURED_INTRINSICS` / `FALLBACK_HFOV_DEG` | 実測 K を使うか、公称画角から概算するか |
| `MAX_RESIDUAL_M` | 3D三角測量の外れ値許容誤差 [m]。実測K導入後は1.0〜1.5に絞れる |
| `GATE_X` / `GATE_Y` / `GATE_Z` | 3D空間ゲート。フィールド外・異常高度の候補を棄却（Phase B） |
| `ALT_SENSOR_*` | 高度センサの有効範囲・観測ノイズ・傾き補正（Phase C） |
| `TRACK_*` | トラック確立条件・coast時間・外観相関の設定（Phase C） |
| `DUMMY_FALLBACK_FRAMES` | ダミー軌道に切り替えるまでの未検出フレーム数 |
| `DUMMY_ORBIT_RADIUS` / `DUMMY_ORBIT_ALT` / `DUMMY_ORBIT_PERIOD` | ダミー円軌道のパラメータ |
| `VELOCITY_SMOOTH_FRAMES` | 速度計算の平滑化フレーム数 |
| `DISP_W` / `DISP_H` / `VELOCITY_W` / `VELOCITY_H` | 各ウィンドウの表示サイズ |

### `detection_params.json`（PC / RPi 共通）

検知パラメータは**プロジェクト直下の `detection_params.json` が唯一の正**です。`src/utils/config.py` と `camera_server.py` の両方がこのファイルを読みます（以前は両者に別々の値がハードコードされていてずれていました）。**RPi にもこのファイルを配置してください。**

| キー | 内容 |
|---|---|
| `diff_threshold` / `min_area_px` / `max_area_px` | 二値化しきい値と、候補として扱う輪郭面積の範囲 |
| `max_candidates` | 各カメラが1フレームに返す候補の最大数 |
| `use_background_subtractor` / `bg_*` | MOG2背景差分の設定。`bg_learning_rate` はほぼ0にする |
| `vibration_reject_ratio` | 前景がこの割合を超えたフレームを丸ごと破棄 |

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