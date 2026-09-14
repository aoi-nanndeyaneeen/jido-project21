# log_recorder — XIAO ESP32C3 を BLE ログ中継機にする

FC (Teensy 4.0 / `drone_s5`) から **UART でレコードを受け取り、BLE Notify で
PC (または他の受信機) へ常時送信する**専用ファーム。

## なぜこうなったか

元々は SD (HW-125) へ書く専用機だった (PMW3901 とのSPI0共有問題を解消するため)。
その後 **SD カードが壊れた**ため、SD/SPI 周りを撤去して BLE 常時送信に置き換えた。
ボードも RP2040/ESP32C3 両対応をやめ、内蔵 BLE を持つ ESP32C3 専用にした。

## 帯域の制約 ★重要

`FlightLog::Rec` = 116B、フレーム 6B、500Hz → 61 kB/s。これをそのまま BLE
Notify すると、実用上のスループット上限 (だいたい 20〜80 kB/s、接続間隔次第) を
確実に超えて破綻する。そのため `src/main.cpp` の `BLE_REC_DECIM` (既定 4) で
T_REC を間引いて送信している (既定: 実質 125Hz ≒ 15 kB/s)。`T_START`/`T_STOP`
は間引かない。

間引かれた分は受信側から見ると seq が飛んで見えるが、これは損失ではなく意図的な
間引き。値を調整するときは `src/main.cpp` 冒頭コメントを参照。

さらに以下が実測を左右する:
- **MTU 交渉**: `BLEDevice::setMTU(247)` は要求するだけで、セントラル (PC/スマホ)
  側が応じるとは限らない。23B のままだと 122B フレームが分割され破綻しやすい。
- **接続間隔はセントラル側が決める**。特に iOS は短い間隔を許可しないことが多い。
  PC (Windows/Linux) の方が融通が利きやすい。
- ESP32 側の `notify()` は時間ベースの間引き (`BLE_MIN_TX_INTERVAL_MS`) だけで
  輻輳制御しており、送信キューの実際の空きは見ていない。

## 配線 (Seeed XIAO ESP32C3)

| XIAO | GPIO | 相手 |
|---|---|---|
| D7 | GPIO20 | RX ← Teensy **TX 17** (Serial4) |
| D6 | GPIO21 | TX → Teensy **RX 16** ※状態返信用。省略しても記録は動く |
| GND | — | Teensy GND **(必須)** |

- **XIAO の電源は FC と分ける。** GND だけ共通にすること。
- UART の線は短く (10cm 程度まで)。2Mbaud なので、長いと `crc_err` が増える。

## 帯域 (UART 区間)

`FlightLog::Rec` = 116B、フレーム 6B、500Hz → **61 kB/s = 610 kbps**。
8N1 なので最低 700kbaud 必要。既定は **2 Mbaud** で 3 倍の余裕を取っている。
化けるようなら両側を 1000000 に落とす (`LogLink::BAUD` と `LINK_BAUD` の両方)。

## 使い方

```
# ロガー側 (XIAO ESP32C3)
cd log_recorder
pio run -e xiao_logger_esp32c3 -t upload
pio device monitor -e xiao_logger_esp32c3     # 's' で状態、'r' で統計リセット

# FC 側: drone_s5.cpp の S5::USE_LOGLINK を true にする
cd flight_controller
pio run -e drone_s5 -t upload

# PC 側: BLE で受信して .BIN に保存
cd log_recorder/scripts
pip install -r requirements.txt
python ble_receiver.py --out logs
```

`ble_receiver.py` が保存する `LOGnnnn.BIN` は SD 版と同じ形式なので、
`flight_controller/scripts/bin2csv.py` でそのまま CSV 化できる。

FC 側のシリアルで `s` を押すとリンクとロガーの状態が出る (`SD=OK/NG` の表示は
実際には「BLE 接続中か」を指す。表示文言は旧SD時代のまま残っている点に注意)。

## ブリングアップ手順

1. **BLE 単体**: 電源投入 → USB シリアルで `s` → `advertising中` が出ることを確認。
   `python ble_receiver.py` を起動して `[CONNECT]` が出れば OK。
   FC が無くても、USB シリアルで `t` を送るとダミーの START/REC×200/STOP を
   直接リングへ注入できる (UART 受信をバイパス)。`ble_receiver.py` 側に
   `[OPEN]` → `[CLOSE]` が出て `LOGnnnn.BIN` ができれば BLE 経路は正常。
2. **リンク単体**: Teensy と TX/RX/GND を繋ぐ。FC 側で `s` →
   ロガー応答が出れば双方向 OK。「応答なし」なら RX 配線かボーレート。
3. **地上でアーム→ディスアーム** (プロペラを外す)。`ble_receiver.py` に
   `[OPEN]` → `[CLOSE]` が出て `LOGnnnn.BIN` ができることを確認。
4. `python ../flight_controller/scripts/bin2csv.py logs/` が通ることを確認。
5. ここまで通ってから飛ばす。

## 落ちたとき / BLE が切れたとき

- `T_STOP` が化けても、レコードが `IDLE_CLOSE_MS` (2 秒) 途切れた時点で
  ロガー・受信スクリプトの双方が勝手にセッションを終える。
- BLE が切れても `ble_receiver.py` は自動で再接続を試みる。再接続後に
  同じ `t0_ms` の `T_START` が来れば同じファイルに続きを書く (ロガー側が
  1Hz で `T_START` を再送する仕組みと対応)。切断中のフレームはリングに
  留まって送信を待つので、リング容量 (128KB ≒ 全レート換算で約2秒ぶん) を
  超えて初めて `drop_ring` として失われる。

## 診断値の読み方 (FC 側 `s` / ロガー側 `s`)

| 値 | 意味 | 増えたら |
|---|---|---|
| `crc_err` | UART フレームの CRC 不一致 | UART が化けている。配線を詰める / GND を太く / BAUD を落とす |
| `seq_gap` | seq の飛び = UART の取りこぼし | 同上 |
| `drop_ring` | ロガーのリング溢れ (BLE未接続/輻輳がリング容量分続いた) | BLE が遅すぎて追いつけていない。間引き率を上げる / 受信側との距離を詰める |
| `orphan` | START 前に来た REC | ロガー起動/BLE接続が遅れただけなら正常 |
| `txdrop` (FC側) | FC のリング溢れ | UART が遅すぎる。BAUD を上げる |
| リング peak | ロガーのリング使用ピーク | 50% を超えたら BLE が追いついていない |
