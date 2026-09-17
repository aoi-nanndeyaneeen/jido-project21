# 自動操縦システムの処理の流れと周期 (SYSTEM_FLOW.md)

作成 2026-09-16 (リファクタリング時)。機体 (flight_controller) / 地上局 (ground_receiver) /
PC (position_estimator) にまたがる自動操縦の **どこで何が何 Hz で回っているか** を 1 枚に
まとめたもの。個々の使い方は README.md / GROUND_CONSOLE_GUIDE.md / WAYPOINT_BRINGUP.md。

★ 2026-09-17: 地上局リンクは既定で **BLE** になった。下の図の「地上局 ground_receiver +
IM920」の段は `GROUND_LINK=IM920` に戻したときの経路。BLE のときはその段が次に置き換わる
(切り替え方は README「地上局リンクの経路」):

```
 PC  core/ble_link.py (BleS5Link。S5Link と同じ API) ── BLE (core/ble_tap.py と接続を共有)
      上り: CmdFrame 22B を Write (SEND_HZ=10Hz のまま)
      下り: T_TELEM を分解 → TelemetryStore と同じ列の state() / s5_link_*.csv
 log_recorder (XIAO ESP32C3)  src/main.cpp
      上り: mailbox → UART T_CMD (新指令は即 / 200ms キープアライブ / PC 1.5s 無音 or BLE 切断で停止)
      下り: UART の T_TELEM を Notify (REC と同じ FIFO。滞留 16KB 超で古いものから捨てる)
 機体  毎ループ LogLink::pollCmd → s5rx.acceptRaw  /  20Hz TelemetryTx::tickBle (A+B+(C|D) を 1 束)
```

```
 ┌─ PC (position_estimator) ─────────────────────────────────────────────────────┐
 │  カメラスレッド core/tracker.py           15〜60Hz (カメラの fps)               │
 │    read_and_detect ×2 → 点滅ロックイン → ペア選択 (三角測量) → P [m]           │
 │  メインスレッド app/main_loop.py  (FlightLoop)                                 │
 │    毎周   YawEstimator に位置 / 機体Δv (D フレーム) を入れる                   │
 │    新フレーム  YawArbiter (core/yaw_source.py) → 機首方位 "camera" / "fixed"   │
 │              MissionRunner.update (core/mission.py)   段階を順に実行           │
 │                └ SEND_HZ=10Hz 上限で間引き → link.send_command("CMD,...")       │
 │    5Hz    Velocity / Graph / Link Status の描画       2Hz  端末の状態画面        │
 │  core/s5_link.py  受信スレッド: DATA/PARAM/STAT 行 → state() スナップショット    │
 └───────────────┬──────────────────────────────────────────────▲─────────────────┘
                 │ USB 115200  "CMD,req,vx,vy,alt,yawrate,flags[...]"   │ "DATA,...", "STAT,..."
 ┌───────────────▼──────────────────────────────────────────────┴─────────────────┐
 │  地上局 ground_receiver  (XIAO RP2040 + IM920sL)  src/main.cpp                 │
 │    全速   USB 1 文字 → キー / CMD 行 → Uplink mailbox (最新値だけ生き残る)      │
 │    毎周   Uplink::service  蓋 CMD_MIN_GAP_MS=125ms (=8Hz) が開けば即 TXDA       │
 │                            新指令が無ければ 200ms ごとにキープアライブ          │
 │                            PC が 1.5s 黙れば送信停止 (機体をフェイルセーフへ)    │
 │    全速   IM920 1 文字 → 行 → IM920::decodeLine → TelemetryStore (forward-fill) │
 │    1Hz    STAT 行 / 人間向け表示                                                │
 └───────────────┬──────────────────────────────────────────────▲─────────────────┘
                 │ IM920sL 19200bps 半二重  上り 8Hz (26B)        │ 下り 8Hz (≤32B) A,B,A,B,C,A,B,D
 ┌───────────────▼──────────────────────────────────────────────┴─────────────────┐
 │  機体 flight_controller  (Teensy 4.0)  src/drone_s5.cpp  loop()                │
 │   1000Hz  IMU → 姿勢 / AltEstimator.predict / SBUS                             │
 │    100Hz  PMW3901 読み + de-rotation ─┐                                          │
 │     25Hz  窓を締めて PosHold          │ 目標速度 → 速度PID → 目標リーン角        │
 │    100Hz  測距ポーリング → AltHold    │ 目標高度 → 位置/速度PID → スロットル     │
 │    200Hz  s5rx.poll (上りコマンド)    │ (新サンプルの回だけ PID を進める)        │
 │    100Hz  Guided::update  要求 → 目標速度/高度 の翻訳 (制御はしない)            │
 │    200Hz  角度PID  目標リーン角 → 目標角速度                                    │
 │   1000Hz  レートPID → ミキサー → ESC   (HeadingHold: ヨーはジャイロ積分を保持)   │
 │    500Hz  FlightLog 1 行 → USB / RAM / SD / LogLink(RP2040 → BLE 125Hz)         │
 │      8Hz  TelemetryTx::tick  下りテレメトリ 1 パケット                          │
 │     10Hz  printStatus                                                           │
 └────────────────────────────────────────────────────────────────────────────────┘
```

## 1. 三層の役割 (ここを崩さない)

| 層 | 決めるもの | 決めないもの |
|---|---|---|
| PC | **目標速度 (機体座標) と目標高度** = setpoint。ミッションの順序、到達判定、安全打ち切り (LAND) | 姿勢、スロットル。無線遅延を姿勢ループへ持ち込まないため (protocol/S5Cmd.h 冒頭) |
| 地上局 | 上りの**帯域** (125ms 蓋)、PC 途絶時の送信停止 | 指令の中身。CSV 化と中継だけ |
| 機体 | 全ての制御。GUIDED に入る**資格** (アーム / SW_HOVER / フロー / 測距 / スティック)。リンク断 1s でホールド・4s で着陸 | ミッション (どこへ行くか) |

GUIDED 中に通る制御経路は POSHOLD と完全に同一。違うのは「目標速度と目標高度を、
スティックが決めるか地上局が決めるか」だけ (`quad/Guided.h`)。

## 1-2. 本番で何を飛ぶか (core/program.py)

PC 側は「Step の列」を順に実行する。各 Step に制限時間があり、超えたら打ち切って
次へ進む。さらに 2 段の締切で必ず着陸へ倒す (着陸は 1.2x1.5^4 = 4860点 で最も高い)。

```
  TAKEOFF 滑走路内離陸 ─ GOTO 進入 ─ MANEUVER 水平旋回 ─ GOTO ─ MANEUVER 上昇旋回
                                   ─ GOTO ─ MANEUVER 8の字 ─ GOTO 帰投 ─ LAND ─ (STILL 5秒)
     budget 20s        25s        40s       20s        80s       20s     40s    30s   40s
     COMP_LAND_BY_S       = 185s … 何をしていても帰投・着陸へ
     COMP_FORCE_LAND_BY_S = 205s … 帰投を諦めてその場で降りる
     ルールの締切          = 235s (3分55秒)。これを過ぎた着陸は 0 点
```

機動の開始地点は **機首の向きから逆算**して、円の中心が必ずフィールド中心に
来るようにする (`core/program.entry_point`)。起動時に軌跡がフェンスに収まるかを
実寸で検算して表示する (`check_geometry`)。手順は [COMPETITION_RUNBOOK.md](COMPETITION_RUNBOOK.md)。

## 2. 周期を決めている定数 (触るときはここ)

| 周期 | 定数 | 場所 |
|---|---|---|
| 機体メイン 1000Hz | `RATE_LOOP_HZ` | `flight_controller/include/quad/QuadConfig.h` §6 |
| 角度ループ 200Hz | `ANGLE_LOOP_HZ` | 同上 |
| フロー読み 100Hz / 制御 25Hz | `FLOW_LOOP_HZ` / `FLOW_CTRL_HZ` | 同上 §7 |
| 測距 100Hz | `RANGE_LOOP_HZ` | 同上 §7-3 |
| GUIDED 翻訳 100Hz | `S5::GUIDED_HZ` | `quad/S5Features.h` |
| 上り受信ポーリング 200Hz | `S5::TELEM_RX_HZ` | 同上 |
| 下りテレメトリ 8Hz (+枠割り) | `S5::TELEM_TX_HZ` / `TelemetryTx::tick` | `S5Features.h` / `S5Telemetry.h` |
| 機体ログ 500Hz | `FlightLog::LOG_HZ` | `quad/FlightLog.h` |
| 上り帯域 8Hz / キープアライブ / PC 途絶 | `CMD_MIN_GAP_MS` / `CMD_KEEPALIVE_MS` / `CMD_PC_TIMEOUT_MS` | `ground_receiver/include/GroundConfig.h` |
| リンク断フェイルセーフ 1s / 4s | `GUIDED_STALE_HOLD_MS` / `GUIDED_STALE_LAND_MS` | `QuadConfig.h` §9 |
| PC 指令 10Hz (上限) | `MissionRunner.SEND_HZ` | `position_estimator/src/core/mission.py` |
| 本番の段階と制限時間 | `COMP_BUDGET_S` / `COMP_LAND_BY_S` / `COMP_FORCE_LAND_BY_S` | `position_estimator/src/utils/config.py` |
| RPi の起動待ち 60s | `RPI_WAIT_S` | 同上 |
| PC 描画 5Hz / 画面 2Hz | `MPL_RENDER_HZ` / `STATUS_REDRAW_S` | `app/main_loop.py` |
| 位置補正 2s に 1 回 | `POS_CORR_PERIOD_S` | `utils/config.py` |

★ (IM920 のとき) IM920 は半二重。上り (地上局 `CMD_MIN_GAP_MS`) と下り (機体 `TELEM_TX_HZ`) は
  **必ずセットで**見て、UART 占有率 55% 程度を上限にする (2026-09-16 現在 53%)。

★ PC の `SEND_HZ` は**上限**。実際の送信はカメラのフレームが来たときにしか起きないので、
  Camera1 が 15fps なら 7.5Hz になる (tests/test_mission_sim.py で確認できる)。

## 3. 1 サイクルの遅れの内訳 (2026-09-16 の実測ベース)

```
 カメラ露光〜検知      15〜60fps → 17〜67ms
 PC 指令の間引き       0〜100ms (SEND_HZ)
 地上局の蓋            0〜125ms (CMD_MIN_GAP_MS)
 IM920 上り            ~31ms (26B) + 半二重の取り合い
 機体 Guided 翻訳      ≤10ms (100Hz)
 機体 PosHold 窓       ≤40ms (25Hz)
 ---- 機体が動く ----
 下り B フレーム       ~330ms 間隔 (3Hz。PC が「効いたか」を見る唯一の枠)
```

往復で 0.35〜0.45 秒が狙い (WAYPOINT_BRINGUP.md §6)。PC 位置ループのゲイン
(`KP_POS`/`KD_VEL`) はこの遅れを前提に決めてある。

## 4. ファイルの対応

| 役割 | 機体 | 地上局 | PC |
|---|---|---|---|
| 無線パケット定義 | `protocol/S5Cmd.h` `S5Telem.h` `Im920Frame.h` (共有) | 同左 | `core/s5_protocol.py` (生成: `python protocol/gen_py_protocol.py`) |
| 周期・配線 | `quad/S5Features.h` | `include/GroundConfig.h` | `app/main_loop.py` 冒頭 |
| ループ本体 | `src/drone_s5.cpp` | `src/main.cpp` | `app/main_loop.py` `FlightLoop` |
| 地上局要求の翻訳 / ミッション | `quad/Guided.h` | `include/Uplink.h` | `core/mission.py` (実行) + `core/program.py` (何を飛ぶか) |
| テレメトリ | `quad/S5Telemetry.h` | `include/TelemetryStore.h` | `core/s5_link.py` |
| 手動操縦コンソール | — | — | `console.py` |
| 使わなくなったもの | `src/archive/` | `used/` | `used/` |

パケット構造を変えたら: `protocol/*.h` の `VERSION` を上げる → `gen_py_protocol.py` を
実行 → 機体と地上局を**両方**焼き直す (片方だけだと `# !! パケットバージョン不一致`)。
