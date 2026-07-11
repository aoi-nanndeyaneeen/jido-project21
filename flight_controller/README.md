# Jido-Project Flight Controller

このプロジェクトは、Teensy等を用いたカスタムフライトコントローラーの開発プロジェクトです。PlatformIOを使用してビルドおよび依存関係の管理を行っています。

## 現在の開発ステータス
現在メインで開発が行われているのは **`src/drone.cpp`** です。
（自律飛行対応版・軽量化・カルマンフィルタ削除版）

## プロジェクト構造とビルドシステム

本プロジェクトは `platformio.ini` を活用し、1つのリポジトリで複数の機体やテスト用プログラムを柔軟に切り替えてビルドできる構成になっています。

ビルド環境（`env`）を指定することで、自動的に `src/` 以下の対応する `.cpp` ファイルのみがメインとしてコンパイルされます。
（例: `env:drone` でビルドすると、他の `main.cpp` や `auto_flight.cpp` などは除外され `src/drone.cpp` がビルド対象になります）

### 主なビルド環境 (Environments)
- **drone**: オートジャイロ / 自律飛行対応 (Teensy 4.1) - **現在メイン**
- **auto_flight**: 双発機用 (Teensy 4.0)
- **trainer**: トレーナー機用 (Teensy 4.0)
- **Zunrocoptor**: (Teensy 4.1)
- **delta_auto**: デルタ翼等 (Teensy 4.1)
- **test**: テスト用

### ディレクトリ構成
- `src/` : 各機体・プログラムごとのエントリーポイント（メインコード）群。
  - `drone.cpp` (メイン開発対象)
  - `auto_flight.cpp`, `Zunrocoptor.cpp`, `uchida.cpp`, `test.cpp`, `trainer.cpp` など。
- `include/` : プロジェクト共通のヘッダファイル群。センサ、制御、通信、アクチュエータなどのクラス・関数定義や設定が含まれます。
  - `Config.h`, `Actuators.h`, `Control.h`, `Sensors.h`, `Telemetry.h`, `flight_mode.h` など。
- `lib/` : プライベートライブラリやローカルの依存関係。
- `test/` : ユニットテスト等。

## ハードウェア・センサー構成 (drone.cppの例)
現在メインの `drone.cpp` では以下のデバイスに対応・搭載を想定しています。

- **マイコン**: Teensy 4.1 (または Teensy 4.0)
- **IMU (加速度・ジャイロ)**: MPU6050
- **気圧センサー**: BMP280 (オプション)
- **受信機**: SBUS対応レシーバー
- **テレメトリ**: IM920 無線モジュール
- **アクチュエータ**: PWMサーボ / ESC

## 開発・ビルド方法
VSCode + PlatformIO 拡張機能を使用することを推奨します。

1. VSCodeでこのプロジェクトフォルダを開きます。
2. 左側のPlatformIOアイコン（アリのマーク）をクリックし、「Project Tasks」を展開します。
3. `env:drone` などの対象環境を選択し、`Build`（コンパイル）または `Upload`（マイコンへの書き込み）を実行します。
