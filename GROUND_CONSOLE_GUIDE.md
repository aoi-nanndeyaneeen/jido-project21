# 地上局コンソール 使い方ガイド

作成 2026-09-14。`position_estimator/src/console.py` (見ながら指令を出す)
と `flight_controller/scripts/merge_logs.py` (ログを1本に統合する) の
使い方をまとめたもの。`console.py` は `main.py` (ウェイポイント自動飛行)
とは別の使い道の道具で、**併用はできない** (地上局のUSBポートは1プロセス
しか開けないため)。

★ `console.py` も `main.py` も、**起動するだけでログは自動で取り始める**。
別途フラグを付ける必要はない (BLEの機体125Hzログを含めて)。

---

## 1. 地上局コンソール (`console.py`)

### これは何か

今まで「見る」(`pio device monitor` / `s5_logger.py`) と「指令を出す」
(`main.py` の自動飛行) は別プロセスで、地上局のUSBが1本しかないぶん
**同時に使えなかった**。結果、「手で少しだけ動かして挙動を見る」が
できなかった。

`console.py` はテレメトリ表示・上りコマンド送信・ログ保存を1プロセスに
まとめたもの。`pio device monitor` の代わりとして常用できる。

### 起動

```bash
cd position_estimator/src
python console.py                 # 地上局 XIAO を自動検出。ログは自動で全部取る
python console.py --no-ble        # BLE (機体125Hzログ) だけ止める
python console.py --port COM7     # 自動検出が外れるとき
python console.py --plain         # 画面制御を使わず1行ずつ流す (端末がANSI非対応のとき)
python console.py --alt 0.8       # 目標高度の初期値を変える (既定は MISSION_TAKEOFF_ALT_M)
```

`main.py` や `s5_logger.py` を同時に起動していないことを確認してから
起動すること (同じCOMポートは1プロセスしか開けない。BLEは別デバイスなので
これには含まれない)。

★ BLEは既定でON。`bleak` が入っていない、または `log_recorder` (XIAO) の
電源が入っていない/見つからないときは、黙って諦めてテレメトリ・指令だけで
続行する (エラーにはならない)。明示的に止めたいときだけ `--no-ble`。

### 画面の見方

```
============================================================================
 S5 CONSOLE   COM7   14:32:10    送信 812 / 受信 1044
============================================================================
 LINK   OK   age 0.08s   12.3Hz   RSSI -72
 BLE    接続   124.8Hz   LOG0009.BIN   38213rec   seq_gap 0
----------------------------------------------------------------------------
 MODE   POSHOLD  alt:HOLD     ARMED  GUIDED  AIRBORNE  CMD_FRESH  FLOW  RANGE
 ALT    対地  0.52 m  目標  0.50 m  上昇 -0.03 m/s  thr 0.440  (alt_thr 0.460)
 POS    n +0.12 e -0.05 m   hold n +0.10 e -0.04   v +0.02/-0.01 目標 +0.00/+0.00
 ATT    roll   +1.2 pitch   -0.4 yaw  +12.3 deg   lean  +0.8/ -0.2
 MOT    0.440 0.450 0.430 0.440   sat 0   lost 2  bad 0  t_ms 123456
----------------------------------------------------------------------------
 TX     >>> 送信中 <<<  REQ=GUIDED   vx +0.10  vy -0.05  alt 0.50 m  flags 0x09
----------------------------------------------------------------------------
 h HOLD   t TAKEOFF(2回押し)   g GUIDED   l LAND   SPACE ABORT(=即HOLD)
 x 送信停止   w/s 前後   a/d 左右   0 速度ゼロ   r/f 目標高度   q 終了
 S 地上局の状態   D 生データ表示   Z 統計クリア   C CSV出力ON   ? ヘルプ
 ★ w/a/s/d は機体座標 (機首向き基準)。緊急停止はプロポ。
 ★ PIDリセット/IMU校正/デバイス確認は ble_monitor.py (BLE) へ移動しました。
----------------------------------------------------------------------------
 [S5Link] 地上局に接続: COM7
```

- `LINK` : テレメトリの受信状況 (`age` が1秒以上なら機体が自動着陸へ落ちかけている)
- `BLE`  : 125Hz生ログの受信状況 (`--no-ble` のときは表示されない)
- `TX`   : 今まさに送っている内容。`送信停止` のときは何も送っていない
- `?` キーでヘルプ行の表示/非表示を切り替え

### キー一覧

| キー | 動作 |
|------|------|
| `h` | HOLD (その場ホールド) |
| `t` | TAKEOFF (**2回押し**。3秒以内にもう一度) |
| `g` | GUIDED (巡航。以後 `w/a/s/d` で速度、`r/f` で高度) |
| `l` | LAND (自動着陸) |
| `SPACE` | ABORT (= 機体を即その場ホールド。**緊急停止ではない**) |
| `x` | 送信停止 (機体は1秒でホールド → 4秒で自動着陸) |
| `w` `s` | 前進/後退 (機体座標、`VEL_STEP_MPS`=0.05刻み) |
| `a` `d` | 左/右 |
| `0` | 速度をゼロに戻す |
| `r` `f` | 目標高度を上げる/下げる (`ALT_STEP_M`=0.05刻み) |
| `S` | 地上局XIAOの状態表示 (`s5_log.cpp` の `s` を転送) |
| `D` | 地上局の生データ表示切替 (`d` を転送) |
| `Z` | 地上局の統計クリア (`z` を転送) |
| `C` | 地上局のCSV出力ON (`1` を転送) |
| `?` | ヘルプ表示の切り替え |
| `q` | 終了 (ABORTを送ってから切断) |

★ 緊急停止はプロポです。コンソールは「黙れば機体が自動着陸へ落ちる」
構造に寄せてあるので、フリーズしたらそのままキーボードから手を離して
プロポで対処すれば安全側に倒れます。

★ PID reset / IMU再キャリブレーション / デバイス確認 (I2C再走査) は
`console.py` (IM920) には無い。**2026-09-14 に IM920 から削除し、BLE
専用の `ble_monitor.py` に一本化した** (IM920は操縦専用に戻した)。
下記「おまけ」参照。

### ログの保存先

既定では `position_estimator/src/logs/` に、同じセッションぶんが揃う。

| ファイル | 内容 |
|---|---|
| `s5_link_YYYYmmdd_HHMMSS.csv` | テレメトリ生ログ (`Epoch_s` + 機体 `t_ms`) |
| `console_YYYYmmdd_HHMMSS.csv` | 送った指令とキー操作 (`Epoch_s` 付き) |
| `LOGnnnn.BIN` | 機体125Hzログ。既定で自動取得 (`--no-ble` で止めない限り)。`bin2csv.py` でCSV化できる |

後で `merge_logs.py` (下記) に渡すと1本の時系列になる。

### おまけ: BLE だけで完結するデバッグツール (`ble_monitor.py`)

`console.py` の画面には BLE の最新値が1行だけ (上書きで) 出るが、
値の推移をそのまま目で追いたいときは `ble_monitor.py` を使う。
BLE 経由で既に受け取っている機体125Hzログ (`bin2csv.py` が解釈している
のと同じバイナリ) を、間引かずに1行ずつテキストで流す。

**PID リセット / IMU再キャリブレーション / デバイス確認 (I2C再走査) は
このツールだけが持っている** (2026-09-14: IM920から削除しBLEに一本化)。
地上局 (IM920) を一切使わない完全に独立した経路で、機体側の実装も
専用の最小フレーム (action + action_seq の2byteのみ) を新設してある。
**操縦系 (速度・高度・離着陸要求) はプロトコル上そもそも運べない** ので、
このツールから機体を飛ばすことは構造的にできない。IM920の地上局が
無い/繋がっていないベンチ上でも、BLEさえ繋がればこれらの確認ができる。

```bash
cd position_estimator/src
python ble_monitor.py             # 表示 + P/k/i でデバッグ指令 (.BIN は残さない)
python ble_monitor.py --save      # ついでに .BIN も残す
python ble_monitor.py --no-keys   # 表示専用にする (誤操作防止)
```

| キー | 動作 |
|---|---|
| `P` | PIDリセット (単押し) |
| `k` | IMU再キャリブレーション (2回押し。**機体が非アーム中のときだけ実行される**) |
| `i` | デバイス確認 / I2C再走査 (**機体が非アーム中のときだけ実行される**) |
| `q` | 終了 |

実行結果は `[BLE] ACK ...` として画面に流れる (成功/アーム中で拒否/
IMU校正が妥当性チェックで却下、のいずれか)。

`console.py` / `main.py` と同時に起動しても構わない (BLE Notify は
複数クライアントが繋がっても平気)。地上局 (IM920) の COM ポートには
一切触らない。

---

## 2. ログ統合 (`merge_logs.py`)

### これは何か

1回の飛行で複数のログができる。**3系統**あり、時計がバラバラなのは
このうち1つ (BLEの機体ログ) だけ:

| # | ログ | 誰が作るか | 時刻 | レート |
|---|------|-----------|------|--------|
| 1 | `LOGnnnn.BIN` | `console.py` / `main.py` が**既定で自動取得**<br>(`log_recorder/scripts/ble_receiver.py` を単体でも起動可) | 機体の `millis()` のみ | 125Hz |
| 2 | `s5_link_*.csv` | `S5Link` クラス<br>(`console.py` でも `main.py` でも同じものが出る) | **PC時刻 + 機体 `t_ms`** | 10〜15Hz |
| 3 | `flight_*.csv` / `mission_*.csv`<br>(カメラ・ウェイポイント指令) | `main.py` のみ<br>(`console.py`単体では出ない) | PC時刻 | 5〜60Hz |

機体にはRTCが無いので、①のBIN単体では西暦の時刻が分からない。橋渡しに
なるのは②`s5_link_*.csv` だけ (受信した瞬間のPC時刻と機体の `t_ms` を
同じ行に持っている)。③のログは元からPC時刻 (`time.time()`) で打刻されて
いるので、②と③は素直に突き合わせられる。

①は`console.py`/`main.py`のどちらでも既定で自動取得するので、
**普通に飛ばせば3系統そろう**。`bleak`未導入や`log_recorder`の電源
オフなど、①だけ取れなかった場合でも②③は問題なく残るので、その場合は
下記の `--no-ble` で2系統だけの統合になる。

### 使い方

```bash
cd flight_controller

# 3系統そろっているとき (①BLE + ②テレメトリ + ③カメラ/ミッション)
python scripts/merge_logs.py                 # 既定の置き場から最新の1組を自動選択

# ①BLEが無いとき (②テレメトリ + ③カメラ/ミッションの2系統だけ)
#  ★ main.py だけで飛ばした場合はこちら。①のBINが無ければ自動でもこの
#    動きになるが、明示したいときや誤って古いBINを拾わせたくないときに使う。
python scripts/merge_logs.py --no-ble

python scripts/merge_logs.py --dry-run        # 時刻合わせの結果だけ見る (CSVは書かない)
python scripts/merge_logs.py --plot           # 合っているかを図で確認する (①ありのときのみ)

# 明示指定
python scripts/merge_logs.py \
    --bin     ../log_recorder/scripts/logs/LOG0009.BIN \
    --telem   ../position_estimator/src/logs/s5_link_20260914_121314.csv \
    --vision  ../position_estimator/src/logs/flight_20260914_121320.csv \
    --mission ../position_estimator/src/logs/mission_20260914_121320.csv \
    --cmd     ../position_estimator/src/logs/console_20260914_121320.csv
```

実行すると画面に時刻合わせの根拠が出て、`*_merged.csv` が
**`position_estimator/src/logs/`** にできる (他のログと同じ場所。
`-o` で変更可)。

- ①ありのとき: 1行 = 機体ログ1レコード (既定125Hz)。列は `epoch_s`/`t_rel_s`
  に続けて機体の全列、`tl_`(テレメトリ)、`cam_`(カメラ)、`ms_`(ミッション)、
  `cmd_`(コンソール指令) の順。ドリフト・無線遅延・ラップ補正・相互相関の
  時計合わせが入る。
- `--no-ble` のとき: 1行 = テレメトリ1レコード (10〜15Hz)。列は
  `epoch_s`/`t_rel_s` に続けてテレメトリの全列、`cam_`/`ms_`/`cmd_`。
  時計合わせは不要 (テレメトリが最初からPC時刻)。

★ ①ありのときの絶対時刻は「真の時刻 + 無線の片道最小遅延」(往復未計測の
ため数十ms系統的に遅い)。ログどうしの相対比較には影響しない。`--no-ble`
のときも同様に、テレメトリのEpoch_s自体が無線受信の遅延ぶん少し遅い。

`pip install numpy`。`--plot` にはさらに `matplotlib` が要る。

---

## 3. トラブルシュート

| 症状 | 見るところ |
|---|---|
| `console.py` が地上局を見つけない | USBの挿し直し、`--port COM?` で明示指定 |
| `TX` は送信中なのに機体が反応しない | `LINK` の `age`。1秒以上ならリンク切れ |
| `ble_monitor.py` の `k`/`i` を押しても `ACK` が「拒否」のまま | 機体がアーム中。ディスアームしてから |
| `BLE 未接続` のまま (`--no-ble` を付けていないのに) | `pip install bleak`。`log_recorder` 側が起動 (`BLE_REC_DECIM` 等) しているか |
| `merge_logs.py` が古い/別飛行のBINを拾う | `--no-ble` を明示するか `--bin` で無いことを指定 |
| `merge_logs.py` が `s5_link_*.csv が見つかりません` | `console.py` (または旧 `s5_link.py` 利用コード) を1回動かしてログを作ってから |
