"""
core/ble_tap.py
log_recorder (XIAO ESP32C3) の BLE ログを「保存しながら覗く」ためのタップ。

--------------------------------------------------------------------------
なぜ ble_receiver.py をそのまま使わないのか
--------------------------------------------------------------------------
log_recorder/scripts/ble_receiver.py は「つないで .BIN に保存し続ける」
単体スクリプトで、止める口も、今どんな値が流れているかを外から見る口も
持っていない。コンソール (console.py) から使うには

    * バックグラウンドで回って、メインスレッドを止めないこと
    * 最新の 1 レコードを覗けること (画面に出すため)
    * 終了時にきちんとファイルを閉じられること

が要る。フレーム分解 (FrameParser) と .BIN の書き出し (BinLogger) は
ble_receiver.py のものを **そのまま import して使う**。ここで作り直すと、
プロトコルを変えたときに直す場所が 2 つに増えて必ず片方が腐る。

--------------------------------------------------------------------------
レコードの解釈も bin2csv.py に任せる
--------------------------------------------------------------------------
Rec の並びとスケールは flight_controller/scripts/bin2csv.py が持っている
(REC_VER つき)。表示のためにここで struct を書き直すと、ファームの
REC_VER が上がったときに「CSV は正しいのに画面だけ嘘をつく」状態になる。
そこで bin2csv の _REC_STRUCT / _row() を借りて 1 行 CSV を作り、列名と
zip して dict にしている。125Hz 全部を文字列化すると無駄なので、画面が
読める程度 (DECODE_HZ) に間引いて解釈する。

--------------------------------------------------------------------------
使い方
--------------------------------------------------------------------------
    tap = BleTap(outdir=Path("logs"), on_event=print)
    if tap.start():                 # bleak が無ければ False
        ...
        st  = tap.status()          # 接続/レート/ファイル
        rec = tap.state()           # 最新レコード (dict, 物理値)
        seq = tap.next_action_seq()
        tap.fire_action(ACT_PID_RESET, seq)   # デバッグ指令 (BLE Write)
        ack, age = tap.last_ack()             # 実行結果 (来ていれば dict)

--------------------------------------------------------------------------
地上局リンク (2026-09-17 追加)
--------------------------------------------------------------------------
IM920 の代わりに BLE で機体と操縦指令・テレメトリをやり取りするとき
(utils/config.py の GROUND_LINK_BACKEND = "ble")、core/ble_link.py の
BleS5Link がこのタップの上に乗る。ここが持つのは運び方だけ:
    add_telem_listener(fn)  … T_TELEM フレームの payload を fn に渡す
    send_ctrl(payload)      … CmdFrame (22B) を操縦用 characteristic へ Write
★ 1 台の PC から同じ機体への BLE 接続は 1 本しか張れない。BleS5Link は
  必ずこのタップを共有する (別に BleakClient を作らない)。
    tap.stop()
"""

import asyncio
import importlib.util
import struct
import sys
import threading
import time
from pathlib import Path

# position_estimator/src/core/ble_tap.py -> ... -> リポジトリ直下
REPO_ROOT = Path(__file__).resolve().parents[3]
BLE_RECEIVER_PY = REPO_ROOT / "log_recorder" / "scripts" / "ble_receiver.py"
BIN2CSV_PY = REPO_ROOT / "flight_controller" / "scripts" / "bin2csv.py"

# 最新レコードを解釈する頻度 [Hz]。画面が読める速さで十分。
DECODE_HZ = 10.0


# ★ 値は protocol/S5Cmd.h / S5Telem.h が唯一の定義 (core/s5_protocol.py は生成物)。
#   BLE 側は Action の値をそのまま運ぶだけ。
#   2026-09-16 まではここに手書きの _MODE_NAME があり、機体の mode (0=RATE 1=ANGLE
#   2=GUIDED 3=POSHOLD 4=ALTHOLD) を REQ 名 (IDLE/HOLD/TAKEOFF/...) で表示していた。
from core.s5_protocol import (MODE_NAME as _MODE_NAME,
                              ACT_NONE, ACT_PID_RESET, ACT_IMU_CAL, ACT_SELFTEST,
                              ACTION_NAME as ACT_NAME,
                              ACK_OK, ACK_REFUSED_ARMED, ACK_CAL_REJECTED)
ACK_RESULT_NAME = {ACK_OK: "OK", ACK_REFUSED_ARMED: "拒否(アーム中)",
                   ACK_CAL_REJECTED: "却下(妥当性チェック)"}


def _format_rec_line(rec):
    """decode 済みレコード 1 件を、シリアルモニタ風の 1 行にする。"""
    def g(k, fmt="{:.2f}"):
        v = rec.get(k)
        if v is None or isinstance(v, str):
            return "--"
        try:
            return fmt.format(v)
        except (ValueError, TypeError):
            return str(v)

    mode = rec.get("mode")
    mode_s = _MODE_NAME.get(int(mode), str(mode)) if isinstance(mode, float) else "--"
    return (f"t={g('t_ms','{:.0f}')}ms  {mode_s:<7} "
            f"armed={g('armed','{:.0f}')}  thr={g('thr')}  "
            f"att(r/p/y)={g('roll_ang')}/{g('pitch_ang')}/{g('yaw_ang')}  "
            f"m=[{g('m1')},{g('m2')},{g('m3')},{g('m4')}]  "
            f"h={g('range_h')}  vz={g('est_vz')}  "
            f"flow=({g('flow_vx')},{g('flow_vy')})")


def _load_module(name, path):
    """パス直指定で 1 ファイルを import する (どちらもパッケージではないため)。"""
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"{path} を読み込めません")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


class BleTap:
    """BLE ログを別スレッドで受けながら .BIN に保存し、最新値を覗かせる。"""

    def __init__(self, outdir, name="S5-LogBLE", on_event=None, verbose=False):
        self.outdir = Path(outdir)
        self.name = name
        self._say = on_event if on_event is not None else print
        # True にすると、解釈できたレコードを DECODE_HZ で 1 行ずつ on_event に
        # 流す (= シリアルモニタのような使い方ができる)。.BIN への保存は
        # verbose に関係なく常に行う (これは「表示」だけの機能)。
        self.verbose = verbose

        self._thread = None
        self._loop = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self._logger = None          # ble_receiver.BinLogger
        self._connected = False
        self._client = None          # bleak.BleakClient (接続中のみ)
        self._last_rec = {}          # 解釈済みの最新レコード
        self._t_decode = 0.0
        self._n_rec = 0              # 受け取った T_REC の総数
        self._rate_n = 0             # レート計算用
        self._rate_t = time.time()
        self._rate_hz = 0.0
        self._rec_size_ok = None     # None=未確認 / True / False
        self._err = ""

        # デバッグ指令 (PID reset/IMU校正/デバイス確認) の ACK
        self._last_ack = None        # dict または None
        self._last_ack_t = 0.0
        self._act_seq = 0            # next_action_seq() のカウンタ

        self._ble = None             # ble_receiver モジュール
        self._b2c = None             # bin2csv モジュール

        # 地上局リンク (core/ble_link.py の BleS5Link) 用
        self._telem_listeners = []   # T_TELEM の payload を受け取る関数
        self._n_telem = 0            # 受け取った T_TELEM の総数
        self._ctrl_latest = None     # まだ書いていない最新の操縦指令 (bytes)
        self._ctrl_busy = False      # 書き込みコルーチンが走っているか
        self._n_ctrl_tx = 0          # BLE へ書けた操縦指令の数
        self._n_ctrl_err = 0         # 書き込みに失敗した数
        self._names = None           # CSV の列名

    # ------------------------------------------------------------ 起動
    def start(self):
        """受信スレッドを起こす。bleak が無ければ False (呼び出し側は続行可)。"""
        try:
            self._b2c = _load_module("s5_bin2csv", BIN2CSV_PY)
            self._names = self._b2c.HEADER.split(",")
        except Exception as e:
            self._err = f"bin2csv.py を読めません: {e}"
            self._say(f"[BLE] {self._err}")
            return False

        try:
            # bleak が無いと ble_receiver.py は import 時に sys.exit する
            self._ble = _load_module("s5_ble_receiver", BLE_RECEIVER_PY)
        except SystemExit as e:
            self._err = "bleak が入っていません (pip install bleak)"
            self._say(f"[BLE] {self._err}  -- BLE 無しで続行します")
            return False
        except Exception as e:
            self._err = f"ble_receiver.py を読めません: {e}"
            self._say(f"[BLE] {self._err}  -- BLE 無しで続行します")
            return False

        self.outdir.mkdir(parents=True, exist_ok=True)
        self._logger = self._ble.BinLogger(self.outdir, on_event=self._say)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        # ★ 先にフレームの取り込みを止める。BLE のスキャンは最大 10 秒
        #   ブロックするので、join を待たずに閉じた .BIN へ書きに行く
        #   スレッドが残りうる。
        self._stop.set()
        if self._loop is not None:
            # 走っているループを起こして finally までたどり着かせる
            try:
                self._loop.call_soon_threadsafe(lambda: None)
            except RuntimeError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._logger is not None:
            self._logger.close()

    # ------------------------------------------------------------ 状態
    def status(self):
        with self._lock:
            st = {"connected": self._connected, "rate_hz": self._rate_hz,
                  "n_rec": self._n_rec, "err": self._err,
                  "n_telem": self._n_telem, "n_ctrl_tx": self._n_ctrl_tx,
                  "n_ctrl_err": self._n_ctrl_err,
                  "rec_size_ok": self._rec_size_ok,
                  "path": None, "seq_gap": 0, "open": False}
        if self._logger is not None:
            ls = self._logger.status()
            st["path"] = ls["path"]
            st["seq_gap"] = ls["seq_gap"]
            st["open"] = ls["open"]
        return st

    def state(self):
        """最新レコード (物理値の dict)。まだ来ていなければ空 dict。"""
        with self._lock:
            return dict(self._last_rec)

    def last_ack(self):
        """(dict または None, 受信してからの経過秒)。まだ無ければ (None, inf)。"""
        with self._lock:
            if self._last_ack is None:
                return None, float("inf")
            return dict(self._last_ack), time.time() - self._last_ack_t

    def next_action_seq(self):
        """1..255 を巡回するカウンタ。0 は機体側で無視されるので使わない。"""
        with self._lock:
            self._act_seq = (self._act_seq % 255) + 1
            return self._act_seq

    # ------------------------------------------------------------ 送信
    def fire_action(self, action, action_seq):
        """デバッグ指令 (PID reset/IMU校正/デバイス確認) を BLE Write で送る。
        接続していなければ False (呼び出し側がメッセージを出すこと)。
        ★ ここで運べるのは action + action_seq の 2byte だけ。速度・高度・
          離着陸要求のような操縦系は絶対に足さないこと。操縦指令は
          send_ctrl() (別の characteristic・別のフレーム型) だけが運ぶ。
          デバッグ用の口と操縦の口を分けておくことで、ble_monitor.py の
          ようなデバッグツールからは構造的に操縦が出ないままにしている。"""
        if self._loop is None or not self._connected:
            return False
        payload = bytes([int(action) & 0xFF, int(action_seq) & 0xFF])
        try:
            asyncio.run_coroutine_threadsafe(self._write_action(payload), self._loop)
        except RuntimeError:
            return False
        return True

    def add_telem_listener(self, fn):
        """T_TELEM (地上局リンクの下りテレメトリ) の payload を fn(bytes) に渡す。
        fn は BLE の受信スレッドから呼ばれるので、中でロックを取ること。"""
        self._telem_listeners.append(fn)

    def send_ctrl(self, payload):
        """操縦指令 (S5Cmd.h の CmdFrame 22B) を BLE Write で送る。
        未接続なら False。

        ★ 最新値だけを送る。前の書き込みが終わる前に次が来たら、前の未送信分は
          捨てて新しいほうを書く (指令は毎回「今どう飛ばしたいか」の全量なので、
          古いものを律儀に全部送るほど遅れるだけ)。
        ★ 応答なし Write を使う (遅延を減らすため)。1 発落ちても次で上書き
          されるし、機体側は seq と鮮度で欠落を数えている。"""
        if self._loop is None or not self._connected:
            return False
        payload = bytes(payload)
        start = False
        with self._lock:
            self._ctrl_latest = payload
            if not self._ctrl_busy:
                self._ctrl_busy = True
                start = True
        if start:
            try:
                asyncio.run_coroutine_threadsafe(self._ctrl_writer(), self._loop)
            except RuntimeError:
                with self._lock:
                    self._ctrl_busy = False
                return False
        return True

    async def _ctrl_writer(self):
        try:
            while True:
                with self._lock:
                    payload = self._ctrl_latest
                    self._ctrl_latest = None
                    if payload is None:
                        self._ctrl_busy = False
                        return
                client = self._client
                if client is None or not client.is_connected:
                    continue            # 捨てる (切断中の指令は送らない)
                try:
                    await client.write_gatt_char(self._ble.BLE_CHAR_CTRL_UUID,
                                                 payload, response=False)
                    with self._lock:
                        self._n_ctrl_tx += 1
                except Exception as e:
                    with self._lock:
                        self._n_ctrl_err += 1
                        first = (self._n_ctrl_err == 1)
                    if first:
                        self._say(f"[BLE] 操縦指令の送信に失敗: {e}  "
                                  "(log_recorder のファームが古い = 操縦用 "
                                  "characteristic が無い可能性)")
        except BaseException:
            with self._lock:
                self._ctrl_busy = False
            raise

    async def _write_action(self, payload: bytes):
        client = self._client
        if client is None or not client.is_connected:
            self._say("[BLE] 未接続のため指令を送れませんでした")
            return
        try:
            await client.write_gatt_char(self._ble.BLE_CHAR_CMD_UUID, payload, response=True)
        except Exception as e:
            self._say(f"[BLE] 指令の送信に失敗: {e}")

    # ------------------------------------------------------------ 内部
    def _on_frame(self, type_, seq, payload):
        if self._stop.is_set():
            return

        if type_ == self._ble.T_ACT_ACK:
            # デバッグ指令の実行結果。.BIN には書かない (ログレコードではないため)。
            self._on_act_ack(payload)
            return

        # まず保存 (ble_receiver.BinLogger と完全に同じ .BIN ができる)
        self._logger.on_frame(type_, seq, payload)

        if type_ == getattr(self._ble, "T_TELEM", None):
            # 地上局リンクの下りテレメトリ。.BIN には書かれない (BinLogger は
            # 型を見て無視する。seq の数え上げのためだけに上で渡している)。
            with self._lock:
                self._n_telem += 1
                listeners = list(self._telem_listeners)
            for fn in listeners:
                try:
                    fn(payload)
                except Exception as e:
                    self._say(f"[BLE] テレメトリの解釈で例外: {e}")
            return

        if type_ == self._ble.T_START:
            # 32B ヘッダの rec_size が今の bin2csv と食い違っていたら、
            # 解釈した値は嘘になる。保存は続けつつ画面には出さない。
            if len(payload) >= 10:
                rec_size = int.from_bytes(payload[8:10], "little")
                ok = (rec_size == self._b2c._REC_STRUCT.size)
                with self._lock:
                    self._rec_size_ok = ok
                if not ok:
                    self._say(f"[BLE] rec_size={rec_size} が bin2csv "
                              f"({self._b2c._REC_STRUCT.size}) と不一致。"
                              "保存はしますが画面表示はできません")
            return

        if type_ != self._ble.T_REC:
            return

        now = time.time()
        with self._lock:
            self._n_rec += 1
            self._rate_n += 1
            if now - self._rate_t >= 1.0:
                self._rate_hz = self._rate_n / (now - self._rate_t)
                self._rate_n = 0
                self._rate_t = now
            if self._rec_size_ok is False or now - self._t_decode < 1.0 / DECODE_HZ:
                return
            self._t_decode = now

        if len(payload) != self._b2c._REC_STRUCT.size:
            return
        try:
            row = self._b2c._row(payload, self._b2c.DEFAULT_HOVER_THR).split(",")
        except Exception:
            return
        rec = {}
        for k, v in zip(self._names, row):
            try:
                rec[k] = float(v)
            except ValueError:
                rec[k] = v
        with self._lock:
            self._last_rec = rec
        if self.verbose:
            self._say(_format_rec_line(rec))

    def _on_act_ack(self, payload):
        try:
            action, action_seq, result, imu_ok, i2c_found = struct.unpack(
                self._ble.ACT_ACK_STRUCT, payload)
        except struct.error:
            return
        ack = {"action": action, "action_seq": action_seq, "result": result,
               "imu_ok": imu_ok, "i2c_found": i2c_found}
        with self._lock:
            self._last_ack = ack
            self._last_ack_t = time.time()
        extra = ""
        if action == ACT_SELFTEST and result == ACK_OK:
            extra = f"  IMU疎通={'OK' if imu_ok else 'NG'} I2C={i2c_found}個"
        self._say(f"[BLE] ACK  {ACT_NAME.get(action, action)} "
                  f"-> {ACK_RESULT_NAME.get(result, result)}{extra}")

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._scan_loop())
        finally:
            self._loop.close()

    async def _scan_loop(self):
        """ble_receiver.run() と同じ流れ。止められる点と、接続状態を
        外へ出す点だけが違う。"""
        ble = self._ble
        parser = ble.FrameParser(self._on_frame)

        while not self._stop.is_set():
            try:
                dev = await ble.BleakScanner.find_device_by_name(self.name, timeout=10.0)
            except Exception as e:
                with self._lock:
                    self._err = f"スキャン失敗: {e}"
                await asyncio.sleep(3)
                continue

            if dev is None:
                with self._lock:
                    self._err = f"'{self.name}' が見つかりません"
                await asyncio.sleep(2)
                continue

            try:
                async with ble.BleakClient(dev) as client:
                    with self._lock:
                        self._connected = True
                        self._client = client
                        self._err = ""
                    self._say(f"[BLE] 接続: {dev.address}")

                    def handle_notify(_, data: bytearray) -> None:
                        parser.feed(bytes(data))

                    await client.start_notify(ble.BLE_CHAR_UUID, handle_notify)
                    while client.is_connected and not self._stop.is_set():
                        await asyncio.sleep(0.2)
            except Exception as e:
                with self._lock:
                    self._err = str(e)
            finally:
                with self._lock:
                    self._connected = False
                    self._client = None
                    self._ctrl_latest = None

            if not self._stop.is_set():
                self._say("[BLE] 切断。再接続します")
                await asyncio.sleep(1.5)
