# log_recorder — RP2040 を SD ログ専用機にする

FC (Teensy 4.0 / `drone_s5`) から **UART でレコードを受け取り、RP2040 の SPI0 で
SD へ書く**だけの専用ファーム。

## なぜ作ったか

`drone_s5.cpp` では PMW3901 (フロー) と HW-125 (SD) が Teensy の SPI0 を共有していて
共存できなかった (HW-125 クローンの 74LVC125 が MISO を Hi-Z にせず、フローを潰す)。
そのため `USE_FLOW` と `USE_SD` が排他になり、**フロー試験のフライトは RAM ログ 8 秒
しか録れない**という制約があった。

SD 書き込みを丸ごと RP2040 に引っ越すと:

- FC の SPI0 は PMW3901 専用になり、`USE_FLOW=true` のままフルログが録れる
- ロガー側は SD 単独バスなので `DEDICATED_SPI` が使える (マルチブロック書き込みが効き、
  FC の `SHARED_SPI` より速い)
- アーム時の `open`+`preAllocate` による数十 ms のブロックが FC から消える
- できあがる `LOGnnnn.BIN` は従来と**バイト単位で同一**。変換は今までどおり
  `flight_controller/scripts/bin2csv.py`

## 配線 (Seeed XIAO RP2040)

| XIAO | GPIO | 相手 |
|---|---|---|
| D7 | GP1 | RX ← Teensy **TX 17** (Serial4) |
| D6 | GP0 | TX → Teensy **RX 16** ※状態返信用。省略しても記録は動く |
| GND | — | Teensy GND **(必須)** |
| D8 | GP2 | HW-125 SCK |
| D10 | GP3 | HW-125 MOSI |
| D9 | GP4 | HW-125 MISO |
| D2 | GP28 | HW-125 CS |
| 5V | — | HW-125 VCC (モジュール上でレギュレータ + レベル変換) |
| GND | — | HW-125 GND |

- **RP2040 の電源は FC と分ける。** SD 書き込みの突入電流で FC を巻き込まないため。
  GND だけ共通にすること。
- UART の線は短く (10cm 程度まで)。2Mbaud なので、長いと `crc_err` が増える。
- Teensy 側のピン選択に注意: ピン 1〜4 はモーター、Serial5(20/21)=SBUS、
  Serial3(14/15)=IM920。空いているのは Serial2(7/8) / **Serial4(16/17)** / Serial6(24/25)。

## 帯域

`FlightLog::Rec` = 116B、フレーム 6B、500Hz → **61 kB/s = 610 kbps**。
8N1 なので最低 700kbaud 必要。既定は **2 Mbaud** で 3 倍の余裕を取っている。
化けるようなら両側を 1000000 に落とす (`LogLink::BAUD` と `LINK_BAUD` の両方)。

## 使い方

```
# ロガー側
cd log_recorder
pio run -t upload
pio device monitor          # 's' で状態、'r' で統計リセット

# FC 側: drone_s5.cpp の S5::USE_LOGLINK を true にする
cd flight_controller
pio run -e drone_s5 -t upload
```

FC 側のシリアルで `s` を押すとリンクとロガーの状態が出る。起動時のデバイス一覧
(`d`) にも `RP2040 logger / Serial4` の行が出る。

## LED (オンボード、負論理)

| 色 | 意味 |
|---|---|
| 青 | 待機 (SD OK) |
| 緑 | 記録中 |
| 赤 | SD NG |
| 緑+赤点滅 | 記録中だがロス発生 (`crc_err` / `seq_gap` / `drop`) |

## ブリングアップ手順

1. **SD 単体**: SD だけ挿して電源投入 → 青が点けば `s_sd.begin()` 成功。
   赤なら CS/配線/FAT32 を疑う (`s` でエラー内容)。
2. **リンク単体**: Teensy と TX/RX/GND を繋ぐ。FC 側で `s` →
   「ロガー: SD=OK」が出れば双方向 OK。「応答なし」なら RX 配線かボーレート。
3. **地上でアーム→ディスアーム** (プロペラを外す)。緑点灯 → `LOGnnnn.BIN` が
   できることを確認。`s` で `seq_gap=0 crc_err=0` を確認する。
4. カードを PC に挿して `python scripts/bin2csv.py E:\` が通ることを確認。
5. ここまで通ってから飛ばす。

## 落ちたとき / 電源を先に抜いたとき

- 2 秒ごとに `sync()` しているので、失うのは最後の 2 秒ぶんだけ。
- `T_STOP` が化けても、レコードが `IDLE_CLOSE_MS` (2 秒) 途切れた時点で
  ロガーが勝手にファイルを閉じる。`preAllocate` した 16MB がゴミとして
  残ることはない。
- ロガーだけ後から電源投入しても拾える (FC が `T_START` ヘッダを 1Hz で再送し、
  ロガーはそれを見てファイルを開く)。その飛行の頭のぶんは落ちる。

## 診断値の読み方 (FC 側 `s` / ロガー側 `s`)

| 値 | 意味 | 増えたら |
|---|---|---|
| `crc_err` | フレームの CRC 不一致 | UART が化けている。配線を詰める / GND を太く / BAUD を落とす |
| `seq_gap` | seq の飛び = UART の取りこぼし | 同上 |
| `drop (ring)` | ロガーのリング溢れ | SD が遅い。カードを替える |
| `drop (wbuf)` | 整列バッファ溢れ | 通常起きない。起きたら要調査 |
| `orphan` | START 前に来た REC | ロガーの起動が遅れただけなら正常 (最大 1 秒ぶん) |
| `txdrop` (FC側) | FC のリング溢れ | UART が遅すぎる。BAUD を上げる |
| リング peak | ロガーのリング使用ピーク | 50% を超えたら SD が追いついていない |
