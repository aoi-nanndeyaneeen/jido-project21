"""
console.py  -  機体の状態を見ながら指令を出す地上局コンソール

    機体 (drone_s5) ──IM920 15Hz──> 地上局 XIAO ──USB──> ここ ──> 画面
                    <──IM920 5Hz───          <──USB──  ここ <── キー入力
    機体 (drone_s5) ──UART──> log_recorder XIAO ──BLE 125Hz──> ここ (--ble)

==========================================================================
これは何か
==========================================================================
今まで「見る」のは pio device monitor / s5_logger.py、「指令を出す」のは
position_estimator の main.py (ウェイポイント飛行) しか無かった。地上局の
USB は 1 本しかないので **この 2 つは同時に使えない**。結果、手で少しだけ
動かして挙動を見る、という一番やりたいことができなかった。

このコンソールは s5_link.S5Link を 1 つだけ開いて、

    * 機体テレメトリを 1 画面に整形して出し続ける (pio device monitor 相当)
    * キー入力をその場で上りコマンドへ変える (TAKEOFF/GUIDED/LAND/...)
    * 同じセッションのログを 1 か所へ揃えて落とす (後で突き合わせるため)

の 3 つを 1 プロセスでやる。

★ main.py (ウェイポイント飛行) とは同時に起動できない。同じ COM ポートは
  1 プロセスしか開けない。s5_logger.py も同様。

==========================================================================
安全のきまり
==========================================================================
★ 送り続けているあいだだけ機体は GUIDED でいられる。
  機体は「指令が 1 秒来ない = その場ホールド」「4 秒来ない = 自動着陸」
  として扱う (QuadConfig.h GUIDED_STALE_HOLD_MS / _LAND_MS)。
  このコンソールは x キー (送信停止) や終了で黙るので、そのまま放って
  おけば機体は勝手に降りてくる。これが唯一の安全側の構造。

★ 緊急停止はプロポ。 SPACE の ABORT は「機体を即その場ホールドへ戻す」
  であって、モーターを切るものではない (機体側 updateGuided() の
  REQ_ABORT は HOLD と同じ扱い)。落とすなら THR_CUT / スティック介入。

★ 速度指令は機体座標。w = 機首方向へ前進であって、画面の上ではない。
  機体は 6 軸 IMU で絶対方位を持たないので、ここでの前後左右は必ず
  「機体から見て」になる。機首がどこを向いているか分からないなら
  速度を出さないこと。

==========================================================================
使い方
==========================================================================
    cd position_estimator/src
    python console.py                 # 地上局を自動検出。ログは自動で全部取る
    python console.py --no-ble        # BLE (機体125Hzログ) だけ止める
    python console.py --port COM7
    python console.py --plain         # 画面制御を使わず1行ずつ流す

ログ (既定では全部 src/logs/ に揃う。後で merge_logs.py で突き合わせる)
    s5_link_YYYYmmdd_HHMMSS.csv   テレメトリ生ログ (Epoch_s + 機体 t_ms)
    console_YYYYmmdd_HHMMSS.csv   送った指令とキー操作 (Epoch_s)
    LOGnnnn.BIN                   機体125Hzログ (BLE)。bin2csv.py で CSV 化できる。
                                   bleak未導入 / log_recorder が見つからないときは
                                   黙って諦めて続行する (--no-ble を付けたのと同じ)
"""

import argparse
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from core.s5_link import (S5Link, REQ_ABORT, REQ_GUIDED, REQ_HOLD, REQ_IDLE,
                          REQ_LAND, REQ_TAKEOFF, REQ_NAME,
                          CF_ARMED_OK, CF_ALT_ABS,
                          ACT_NONE, ACT_PID_RESET, ACT_IMU_CAL, ACT_SELFTEST,
                          ACT_NAME, ACK_OK, ACK_RESULT_NAME)
from utils.config import GROUND_LINK_PORT, LOG_DIR, MISSION_TAKEOFF_ALT_M
from utils.logger import CsvLogger

# ---- 指令の刻み -----------------------------------------------------------
# ★ 上限は機体側 Q::GUIDED_MAX_VEL (0.5 m/s) でも必ずクランプされる。
#   ここはそれより内側に取る (手で押す以上、行き過ぎを機体任せにしない)。
VEL_STEP_MPS = 0.05
VEL_MAX_MPS = 0.40
ALT_STEP_M = 0.05
ALT_MAX_M = 2.00

# 上りコマンドの送信レート [Hz]。地上局 XIAO が 5Hz に間引くので、
# それ以上送っても無線には出ない。
SEND_HZ = 5.0

# 単発メンテナンス指令 (PID reset / IMU校正 / デバイス確認) を、
# 届く確率を上げるためにこの秒数だけ同じ action_seq で送り続ける。
# 機体側は action_seq の重複を排除するので、届いた回数分は実行されない。
ACTION_LINGER_S = 1.0

# 機体側のフェイルセーフ (QuadConfig.h)。画面の注意書きに使う。
STALE_HOLD_S = 1.0
STALE_LAND_S = 4.0

# モード番号 -> 名前 (utils/logger.py の MODE_NAME と同じ対応)
MODE_NAME = {0: "RATE", 1: "ANGLE", 2: "GUIDED", 3: "POSHOLD", 4: "ALTHOLD"}

# 機体の高度サブ状態 (S5Telem の AltState)
ALT_STATE_NAME = {0: "OFF", 1: "HOLD", 2: "TAKEOFF", 3: "LAND", 4: "LANDED"}


# ==========================================================================
#  キー入力  -  Enter を押さずに 1 文字取る
# ==========================================================================
class KeyReader:
    """Windows は msvcrt、それ以外は termios。どちらも無ければ無効化する。"""

    def __init__(self):
        self._mode = None
        self._fd = None
        self._saved = None
        if os.name == "nt":
            import msvcrt  # noqa: F401
            self._mode = "nt"
        elif sys.stdin.isatty():
            self._mode = "posix"

    def __enter__(self):
        if self._mode == "posix":
            import termios
            import tty
            self._fd = sys.stdin.fileno()
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        if self._mode == "posix" and self._saved is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)

    def get(self):
        """押されていれば 1 文字、無ければ None。ブロックしない。"""
        if self._mode == "nt":
            import msvcrt
            if not msvcrt.kbhit():
                return None
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):   # 方向キーなどの 2 バイト目を捨てる
                msvcrt.getwch()
                return None
            return ch
        if self._mode == "posix":
            import select
            if not select.select([sys.stdin], [], [], 0)[0]:
                return None
            return sys.stdin.read(1)
        return None


# ==========================================================================
#  画面
# ==========================================================================
class Screen:
    """毎回カーソルを左上へ戻して上書きする。行末は消してから書くので、
    短い行になったときにゴミが残らない。

    ★ --plain では ANSI を一切使わない。機体側 s5_log.cpp が
      「PlatformIO のシリアルモニタでは ANSI が処理されず、画面が崩れた
      のかキーが効いていないのか区別できなくなる」としてクリアを避けて
      いるのと同じ理由で、逃げ道を用意しておく。
    """

    def __init__(self, plain=False):
        self.plain = plain
        if not plain and os.name == "nt":
            self._enable_vt()

    @staticmethod
    def _enable_vt():
        """Windows のコンソールで ANSI を有効にする (Win10 1511 以降)。"""
        try:
            import ctypes
            k32 = ctypes.windll.kernel32
            h = k32.GetStdHandle(-11)
            mode = ctypes.c_uint32()
            if k32.GetConsoleMode(h, ctypes.byref(mode)):
                k32.SetConsoleMode(h, mode.value | 0x0004)
        except Exception:
            pass

    def draw(self, lines):
        out = ["\033[H"]
        for line in lines:
            out.append(line + "\033[K\n")
        out.append("\033[J")
        sys.stdout.write("".join(out))
        sys.stdout.flush()

    def clear(self):
        if not self.plain:
            sys.stdout.write("\033[2J\033[H")
            sys.stdout.flush()


# ==========================================================================
#  送信ログ
# ==========================================================================
class ConsoleLogger(CsvLogger):
    """送った指令と、そのときの機体の言い分を 1 行に並べる。

    ★ 先頭列は Epoch_s。utils/logger.py の決まりに合わせてある
      (これがカメラログ・テレメトリログとの唯一の突き合わせキー)。
    """

    HEADER = ["Epoch_s", "Time", "Event",
              "Tx_On", "Req", "Cmd_Vx(m/s)", "Cmd_Vy(m/s)", "Cmd_Alt(m)", "Flags",
              "Action", "Action_Seq",
              "Mode", "Armed", "Guided", "Cmd_Fresh", "Landed", "Airborne",
              "Range_H(m)", "Alt_Hold(m)", "Climb(m/s)", "Thr",
              "Fh_PosN(m)", "Fh_PosE(m)", "Flow_OK", "Range_Valid",
              "Tel_Age(s)", "RSSI", "Drone_t_ms"]

    def __init__(self, log_path):
        super().__init__(log_path, self.HEADER, mode="w")

    def write(self, cmd, tel, age, event=""):
        epoch, hms = self._stamp()
        mode = tel.get("mode")
        self._writer.writerow([
            epoch, hms, event,
            int(cmd["tx_on"]), REQ_NAME.get(cmd["req"], cmd["req"]),
            round(cmd["vx"], 3), round(cmd["vy"], 3), round(cmd["alt"], 3),
            cmd["flags"],
            ACT_NAME.get(cmd.get("action", ACT_NONE), cmd.get("action", "")),
            cmd.get("action_seq", 0),
            MODE_NAME.get(int(mode), mode) if mode is not None else "",
            int(bool(tel.get("armed"))), int(bool(tel.get("guided"))),
            int(bool(tel.get("cmd_fresh"))), int(bool(tel.get("landed"))),
            int(bool(tel.get("airborne"))),
            tel.get("range_h", ""), tel.get("alt_hold", ""),
            tel.get("climb", ""), tel.get("thr", ""),
            tel.get("fh_posn", ""), tel.get("fh_pose", ""),
            int(bool(tel.get("flow_ok"))), int(bool(tel.get("range_valid"))),
            round(age, 3) if age != float("inf") else "",
            tel.get("rssi", ""), tel.get("t_ms", ""),
        ])
        self._fh.flush()


# ==========================================================================
#  指令の保持と送信
# ==========================================================================
class Commander:
    """「今どう飛ばしたいか」を持って、SEND_HZ で送り続ける。

    ★ キープアライブとして勝手に前回値を送り直すのではない。操縦者が
      止める (x) か、このプロセスが死ぬまで送る、という意味。s5_link の
      docstring にあるとおり「止まったプロセスの最後の指令を送り続ける」
      のが一番危ないので、送信スレッドはデーモンにして必ず一緒に死ぬ。
    """

    def __init__(self, link, logger, alt_m):
        self.link = link
        self.logger = logger
        self._lock = threading.Lock()
        self._req = REQ_HOLD
        self._vx = 0.0
        self._vy = 0.0
        self._alt = alt_m
        self._tx = False              # ★ 起動時は必ず送信 OFF
        self._event = ""
        self._n_tx = 0
        self._action = ACT_NONE
        self._action_seq = 0
        self._action_until = 0.0      # この時刻まで action を送り続ける
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ---- 状態 ----------------------------------------------------------
    def snapshot(self):
        with self._lock:
            action_live = self._action if time.time() < self._action_until else ACT_NONE
            return {"req": self._req, "vx": self._vx, "vy": self._vy,
                    "alt": self._alt, "tx_on": self._tx,
                    "flags": CF_ARMED_OK | CF_ALT_ABS, "n_tx": self._n_tx,
                    "action": action_live,
                    "action_seq": self._action_seq if action_live != ACT_NONE else 0}

    # ---- 操作 ----------------------------------------------------------
    def set_req(self, req, event=""):
        with self._lock:
            self._req = req
            if req in (REQ_HOLD, REQ_TAKEOFF, REQ_LAND, REQ_ABORT):
                self._vx = self._vy = 0.0   # 水平は機体側でも 0 にされる
            self._tx = True
            self._event = event or f"REQ={REQ_NAME.get(req, req)}"

    def nudge(self, dvx=0.0, dvy=0.0, dalt=0.0):
        with self._lock:
            self._vx = max(-VEL_MAX_MPS, min(VEL_MAX_MPS, self._vx + dvx))
            self._vy = max(-VEL_MAX_MPS, min(VEL_MAX_MPS, self._vy + dvy))
            self._alt = max(0.0, min(ALT_MAX_M, self._alt + dalt))
            self._event = (f"SET vx={self._vx:+.2f} vy={self._vy:+.2f} "
                           f"alt={self._alt:.2f}")

    def zero_vel(self):
        with self._lock:
            self._vx = self._vy = 0.0
            self._event = "SET vx=0 vy=0"

    def stop_tx(self):
        with self._lock:
            self._tx = False
            self._req = REQ_IDLE
            self._vx = self._vy = 0.0
            self._event = "TX STOP (機体は 1s でホールド -> 4s で自動着陸)"

    def fire_action(self, action, event=""):
        """単発メンテナンス指令 (PID reset / IMU校正 / デバイス確認) を1つ発行する。

        ★ 巡航指令 (req) を送っていない (tx_on=False) 状態でも送れる。
          これが主用途: 飛ばす前に地上でPID resetやデバイス確認をする。
          その場合 _loop() は req を REQ_IDLE に強制する
          (勝手に GUIDED へ入ってしまわないように)。
        """
        with self._lock:
            self._action = action
            self._action_seq = self.link.next_action_seq()
            self._action_until = time.time() + ACTION_LINGER_S
            self._event = event or f"ACTION={ACT_NAME.get(action, action)}"

    def take_event(self):
        with self._lock:
            ev, self._event = self._event, ""
            return ev

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)

    # ---- 本体 ----------------------------------------------------------
    def _loop(self):
        period = 1.0 / SEND_HZ
        while self._running:
            t0 = time.time()
            snap = self.snapshot()
            event = self.take_event()
            # action は tx_on と無関係に送る (飛ばす前の地上チェックが主用途)。
            #  ★ tx_on=False のときは req を REQ_IDLE に強制する。self._req の
            #    既定値 REQ_HOLD をそのまま送ると、GUIDED に入る資格さえ
            #    揃っていれば「意味のある指令」として勝手に engage されうる
            #    (drone_s5.cpp updateGuided() の初回エンゲージ条件)。
            #    デバイス確認のつもりが離陸資格を与えてしまう事故を防ぐ。
            send_now = snap["tx_on"] or snap["action"] != ACT_NONE
            if send_now:
                req = snap["req"] if snap["tx_on"] else REQ_IDLE
                self.link.send_command(req, vx_mps=snap["vx"],
                                       vy_mps=snap["vy"], alt_m=snap["alt"],
                                       flags=snap["flags"],
                                       action=snap["action"],
                                       action_seq=snap["action_seq"])
                with self._lock:
                    self._n_tx += 1
            if self.logger is not None and (send_now or event):
                self.logger.write(snap, self.link.state(), self.link.age(), event)
            time.sleep(max(0.0, period - (time.time() - t0)))


# ==========================================================================
#  キーの割り当て
# ==========================================================================
class KeyHandler:
    """1 文字を指令に変える。押し方の状態 (離陸の 2 回押し) はここが持つ。"""

    # 離陸の確認をやり直すまでの時間 [s]
    TAKEOFF_CONFIRM_S = 3.0

    def __init__(self, cmdr, link, say):
        self.cmdr = cmdr
        self.link = link
        self.say = say
        self.help_on = True
        self._t_takeoff = 0.0
        self._t_imu_cal = 0.0

    def handle(self, ch):
        """処理して True。q が押されたら False (呼び出し側が終了する)。"""
        cmdr = self.cmdr
        now = time.time()

        if ch == "q":
            return False

        if ch == "h":
            cmdr.set_req(REQ_HOLD, "KEY h -> HOLD")
            self.say("[Console] HOLD")
        elif ch == "t":
            # ★ 離陸は 2 回押し。1 キーで機体が上がるのは危ない。
            if now - self._t_takeoff < self.TAKEOFF_CONFIRM_S:
                cmdr.set_req(REQ_TAKEOFF, "KEY t -> TAKEOFF")
                self.say(f"[Console] TAKEOFF 送信 (目標 {cmdr.snapshot()['alt']:.2f} m)")
                self._t_takeoff = 0.0
            else:
                self._t_takeoff = now
                self.say(f"[Console] TAKEOFF? {self.TAKEOFF_CONFIRM_S:.0f} 秒以内に"
                         "もう一度 t を押してください")
        elif ch == "g":
            cmdr.set_req(REQ_GUIDED, "KEY g -> GUIDED")
            self.say("[Console] GUIDED (w/a/s/d で速度、r/f で高度)")
        elif ch == "l":
            cmdr.set_req(REQ_LAND, "KEY l -> LAND")
            self.say("[Console] LAND (自動着陸)")
        elif ch == " ":
            cmdr.set_req(REQ_ABORT, "KEY SPACE -> ABORT")
            self.say("[Console] ABORT = 機体は即その場ホールド。落とすならプロポ")
        elif ch == "x":
            cmdr.stop_tx()
            self.say(f"[Console] 送信停止。機体は {STALE_HOLD_S:.0f}s でホールド "
                     f"-> {STALE_LAND_S:.0f}s で自動着陸")
        elif ch == "w":
            cmdr.nudge(dvx=+VEL_STEP_MPS)
        elif ch == "s":
            cmdr.nudge(dvx=-VEL_STEP_MPS)
        elif ch == "a":
            cmdr.nudge(dvy=-VEL_STEP_MPS)
        elif ch == "d":
            cmdr.nudge(dvy=+VEL_STEP_MPS)
        elif ch == "0":
            cmdr.zero_vel()
        elif ch == "r":
            cmdr.nudge(dalt=+ALT_STEP_M)
        elif ch == "f":
            cmdr.nudge(dalt=-ALT_STEP_M)
        elif ch == "P":
            # PID reset。シリアル 'r' キーと同じ操作で、アーム中でも実行される
            # (機体側もそこは serial と同じ挙動にそろえてある)。
            cmdr.fire_action(ACT_PID_RESET, "KEY P -> PID_RESET")
            self.say("[Console] PID reset を送信")
        elif ch == "k":
            # ★ IMU校正は 2 回押し。機体を水平に静止させてから押すこと。
            #   機体側は非アーム中しか実行しない (アーム中は ACK が拒否を返す)。
            if now - self._t_imu_cal < self.TAKEOFF_CONFIRM_S:
                cmdr.fire_action(ACT_IMU_CAL, "KEY k -> IMU_CAL")
                self.say("[Console] IMU校正を送信 (機体は非アーム中のみ実行)")
                self._t_imu_cal = 0.0
            else:
                self._t_imu_cal = now
                armed = self.link.flag("armed")
                warn = "  ※テレメトリはアーム中と表示しています。拒否されます" if armed else ""
                self.say(f"[Console] IMU校正? 機体を水平な床に置いて静止させてから "
                         f"{self.TAKEOFF_CONFIRM_S:.0f} 秒以内にもう一度 k を"
                         f"押してください{warn}")
        elif ch == "i":
            # デバイス確認 (I2C再走査)。機体側は非アーム中のみ実行する。
            cmdr.fire_action(ACT_SELFTEST, "KEY i -> SELFTEST")
            self.say("[Console] デバイス確認 (I2C再走査) を送信")
        elif ch in ("S", "D", "Z", "C"):
            # 地上局 XIAO のキーをそのまま転送する (s5_log.cpp の handleKey)
            fwd = {"S": "s", "D": "d", "Z": "z", "C": "1"}[ch]
            self.link.send_key(fwd)
            self.say(f"[Console] 地上局へ '{fwd}' を送りました")
        elif ch == "?":
            self.help_on = not self.help_on
        return True


# ==========================================================================
#  画面の組み立て
# ==========================================================================
def _fmt(tel, key, fmt="{:+.2f}", default="  --  "):
    v = tel.get(key)
    if v is None or v == "":
        return default
    try:
        return fmt.format(float(v))
    except (TypeError, ValueError):
        return str(v)


def _onoff(tel, key, label):
    """立っているフラグだけ名前を出す。全部並べると読めなくなる。"""
    return label if tel.get(key) else " " * len(label)


def build_lines(link, cmd, tap, messages, rate_hz, help_on):
    tel = link.state()
    age = link.age()
    now = datetime.now().strftime("%H:%M:%S")
    bar = "=" * 78
    thin = "-" * 78

    lines = [bar,
             f" S5 CONSOLE   {link.port or '(ポート無し)'}   {now}"
             f"    送信 {cmd['n_tx']} / 受信 {link.n_data()}",
             bar]

    # ---- リンク -------------------------------------------------------
    if age == float("inf"):
        link_s = "テレメトリ未受信"
    elif age < 1.0:
        link_s = f"OK   age {age:4.2f}s   {rate_hz:4.1f}Hz   RSSI {_fmt(tel, 'rssi', '{:.0f}')}"
    else:
        link_s = f"!! 途切れています  age {age:4.1f}s"
    lines.append(f" LINK   {link_s}")

    if tap is not None:
        st = tap.status()
        name = st["path"].name if st["path"] else "-"
        if st["connected"]:
            ble_s = (f"接続   {st['rate_hz']:5.1f}Hz   {name}   "
                     f"{st['n_rec']}rec   seq_gap {st['seq_gap']}")
        else:
            ble_s = f"未接続 ({st['err'] or 'スキャン中'})"
        lines.append(f" BLE    {ble_s}")
    lines.append(thin)

    # ---- 機体の状態 ---------------------------------------------------
    mode = tel.get("mode")
    mode_s = MODE_NAME.get(int(mode), str(mode)) if mode is not None else "--"
    alt_state = tel.get("alt_state")
    alt_s = ALT_STATE_NAME.get(int(alt_state), str(alt_state)) if alt_state is not None else "--"
    flags = "  ".join(x for x in [
        _onoff(tel, "armed", "ARMED"), _onoff(tel, "guided", "GUIDED"),
        _onoff(tel, "airborne", "AIRBORNE"), _onoff(tel, "cmd_fresh", "CMD_FRESH"),
        _onoff(tel, "landed", "LANDED"), _onoff(tel, "flow_ok", "FLOW"),
        _onoff(tel, "range_valid", "RANGE"), _onoff(tel, "pos_hold", "HOLD"),
    ] if x.strip())
    lines.append(f" MODE   {mode_s:<8} alt:{alt_s:<8} {flags}")
    lines.append(f" ALT    対地 {_fmt(tel, 'range_h', '{:5.2f}')} m  "
                 f"目標 {_fmt(tel, 'alt_hold', '{:5.2f}')} m  "
                 f"上昇 {_fmt(tel, 'climb', '{:+5.2f}')} m/s  "
                 f"thr {_fmt(tel, 'thr', '{:5.3f}')}  "
                 f"(alt_thr {_fmt(tel, 'alt_thr', '{:5.3f}')})")
    lines.append(f" POS    n {_fmt(tel, 'fh_posn', '{:+5.2f}')} "
                 f"e {_fmt(tel, 'fh_pose', '{:+5.2f}')} m   "
                 f"hold n {_fmt(tel, 'fh_holdn', '{:+5.2f}')} "
                 f"e {_fmt(tel, 'fh_holde', '{:+5.2f}')}   "
                 f"v {_fmt(tel, 'fh_vxc', '{:+5.2f}')}/{_fmt(tel, 'fh_vyc', '{:+5.2f}')} "
                 f"目標 {_fmt(tel, 'fh_vxt', '{:+5.2f}')}/{_fmt(tel, 'fh_vyt', '{:+5.2f}')}")
    lines.append(f" ATT    roll {_fmt(tel, 'roll', '{:+6.1f}')} "
                 f"pitch {_fmt(tel, 'pitch', '{:+6.1f}')} "
                 f"yaw {_fmt(tel, 'yaw', '{:+6.1f}')} deg   "
                 f"lean {_fmt(tel, 'fh_leanr', '{:+5.1f}')}/{_fmt(tel, 'fh_leanp', '{:+5.1f}')}")
    lines.append(f" MOT    "
                 f"{_fmt(tel, 'm1', '{:5.3f}')} {_fmt(tel, 'm2', '{:5.3f}')} "
                 f"{_fmt(tel, 'm3', '{:5.3f}')} {_fmt(tel, 'm4', '{:5.3f}')}   "
                 f"sat {_fmt(tel, 'mixsat', '{:.0f}')}   "
                 f"lost {_fmt(tel, 'lost', '{:.0f}')}  bad {_fmt(tel, 'bad', '{:.0f}')}  "
                 f"t_ms {_fmt(tel, 't_ms', '{:.0f}')}")

    # ---- BLE の高レート側 ---------------------------------------------
    if tap is not None:
        rec = tap.state()
        if rec:
            lines.append(f" BLE125 est_h {_fmt(rec, 'est_h', '{:5.2f}')} "
                         f"est_vz {_fmt(rec, 'est_vz', '{:+5.2f}')} "
                         f"flow {_fmt(rec, 'flow_vx', '{:+5.2f}')}/"
                         f"{_fmt(rec, 'flow_vy', '{:+5.2f}')}  "
                         f"dt {_fmt(rec, 'dt_us', '{:.0f}')}us  "
                         f"t_ms {_fmt(rec, 't_ms', '{:.0f}')}")
    lines.append(thin)

    # ---- 送信状態 -----------------------------------------------------
    tx_mark = ">>> 送信中 <<<" if cmd["tx_on"] else "--- 送信停止 ---"
    alt_txt = "現状維持" if cmd["alt"] <= 0.0 else f"{cmd['alt']:.2f} m"
    lines.append(f" TX     {tx_mark}  REQ={REQ_NAME.get(cmd['req'], cmd['req']):<8}"
                 f" vx {cmd['vx']:+.2f}  vy {cmd['vy']:+.2f}  alt {alt_txt}"
                 f"  flags 0x{cmd['flags']:02X}")
    if not cmd["tx_on"]:
        lines.append(f"        (黙っている間、機体は {STALE_HOLD_S:.0f}s でホールド"
                     f" -> {STALE_LAND_S:.0f}s で自動着陸)")

    # ---- 単発メンテナンス指令の実行結果 --------------------------------
    ack, ack_age = link.last_ack()
    if ack is not None:
        result_s = ACK_RESULT_NAME.get(ack["result"], str(ack["result"]))
        extra = ""
        if ack["action"] == ACT_SELFTEST and ack["result"] == ACK_OK:
            extra = f"  IMU疎通={'OK' if ack['imu_ok'] else 'NG'} I2C={ack['i2c_found']}個"
        age_s = f"{ack_age:4.1f}s前" if ack_age < 999 else "  --  "
        lines.append(f" ACK    {ACT_NAME.get(ack['action'], ack['action']):<10}"
                     f"-> {result_s}{extra}   ({age_s})")
    lines.append(thin)

    # ---- キー ---------------------------------------------------------
    if help_on:
        lines += [
            " h HOLD   t TAKEOFF(2回押し)   g GUIDED   l LAND   SPACE ABORT(=即HOLD)",
            " x 送信停止   w/s 前後   a/d 左右   0 速度ゼロ   r/f 目標高度   q 終了",
            " P PIDリセット   k IMU校正(2回押し、非アーム時のみ)   i デバイス確認(I2C)",
            " S 地上局の状態   D 生データ表示   Z 統計クリア   C CSV出力ON   ? ヘルプ",
            " ★ w/a/s/d は機体座標 (機首向き基準)。緊急停止はプロポ。",
        ]
    else:
        lines.append(" ? キーでヘルプ")
    lines.append(thin)

    # ---- メッセージ ---------------------------------------------------
    for m in messages[-6:]:
        lines.append(" " + m[:77])
    return lines


def build_plain_line(link, cmd, tap, rate_hz):
    """--plain のときの 1 行。流れていく前提なので、時刻と要点だけ。"""
    tel = link.state()
    mode = tel.get("mode")
    mode_s = MODE_NAME.get(int(mode), str(mode)) if mode is not None else "--"
    age = link.age()
    age_s = "---" if age == float("inf") else f"{age:4.2f}"
    ble = ""
    if tap is not None:
        st = tap.status()
        ble = f" BLE {'OK' if st['connected'] else '--'} {st['n_rec']}rec"
    return (f"{datetime.now().strftime('%H:%M:%S')} "
            f"{mode_s:<7} "
            f"{'ARM' if tel.get('armed') else '   '} "
            f"{'GUID' if tel.get('guided') else '    '} "
            f"h {_fmt(tel, 'range_h', '{:5.2f}')}/{_fmt(tel, 'alt_hold', '{:5.2f}')} "
            f"pos {_fmt(tel, 'fh_posn', '{:+5.2f}')},{_fmt(tel, 'fh_pose', '{:+5.2f}')} "
            f"thr {_fmt(tel, 'thr', '{:5.3f}')} "
            f"| age {age_s}s {rate_hz:4.1f}Hz{ble} "
            f"| TX {'ON ' if cmd['tx_on'] else 'off'} "
            f"{REQ_NAME.get(cmd['req'], cmd['req'])} "
            f"v {cmd['vx']:+.2f},{cmd['vy']:+.2f} alt {cmd['alt']:.2f}")


# ==========================================================================
#  本体
# ==========================================================================
def main():
    ap = argparse.ArgumentParser(
        description="機体テレメトリを見ながら上りコマンドを出す地上局コンソール")
    ap.add_argument("--port", default=GROUND_LINK_PORT,
                    help="地上局 XIAO の COM ポート (既定: VID:PID で自動検出)")
    ap.add_argument("--no-ble", action="store_true",
                    help="log_recorder の BLE 125Hzログを受けない。"
                        "既定では自動で探しにいく (bleak 未導入 / 機体が"
                        "見つからないときは黙って諦めて続行するので、"
                        "付けなくても害はない)")
    ap.add_argument("--ble-name", default="S5-LogBLE", help="BLE デバイス名")
    ap.add_argument("--log-dir", default=str(LOG_DIR),
                    help="ログの保存先 (既定: position_estimator/src/logs)")
    ap.add_argument("--alt", type=float, default=MISSION_TAKEOFF_ALT_M,
                    help="目標高度の初期値 [m]")
    ap.add_argument("--plain", action="store_true",
                    help="画面制御を使わず 1 行ずつ流す (端末が ANSI を解さないとき)")
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    messages = []

    def say(msg):
        messages.append(str(msg))
        del messages[:-200]
        if args.plain:
            # 全画面を描かないので、メッセージ枠も無い。その場で流す。
            print(str(msg), flush=True)

    screen = Screen(plain=args.plain)
    screen.clear()
    print("地上局を探しています...", flush=True)

    link = S5Link(port=args.port, log_dir=log_dir, on_message=say)
    if not link.ok:
        print("\n".join(messages))
        print("\n地上局につながりません。USB とファーム (env:xiao_s5_log) を確認してください。")
        return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    clogger = ConsoleLogger(log_dir / f"console_{ts}.csv")
    say(f"[Console] 操作ログ: {clogger.path.name}")

    # ★ 既定で自動的に BLE も受けにいく (--no-ble で止められる)。起動時に
    #   毎回フラグを付けなくても、コンソールを立ち上げるだけで3系統
    #   (テレメトリ・操作ログ・機体125Hz) が全部そろうようにするため。
    #   bleak が無い / log_recorder が見つからないだけなら黙って諦めて
    #   続行する (BleTap.start() 参照) ので、付けっぱなしでも害はない。
    tap = None
    if not args.no_ble:
        from core.ble_tap import BleTap
        tap = BleTap(log_dir, name=args.ble_name, on_event=say)
        if not tap.start():
            tap = None

    cmdr = Commander(link, clogger, args.alt)
    keymap = KeyHandler(cmdr, link, say)

    n_prev, t_prev, rate_hz = link.n_data(), time.time(), 0.0
    t_plain = 0.0

    try:
        with KeyReader() as keys:
            while True:
                ch = keys.get()
                if ch and not keymap.handle(ch):
                    say("[Console] 終了します (ABORT を送って切断)")
                    break

                now = time.time()
                if now - t_prev >= 1.0:
                    rate_hz = (link.n_data() - n_prev) / (now - t_prev)
                    n_prev, t_prev = link.n_data(), now

                if args.plain:
                    # 流していく表示。10Hz で出すと読めないので 1Hz。
                    if now - t_plain >= 1.0:
                        t_plain = now
                        print(build_plain_line(link, cmdr.snapshot(), tap, rate_hz),
                              flush=True)
                else:
                    screen.draw(build_lines(link, cmdr.snapshot(), tap,
                                            messages, rate_hz, keymap.help_on))
                time.sleep(0.1)
    except KeyboardInterrupt:
        say("[Console] Ctrl+C")
    finally:
        cmdr.close()
        if tap is not None:
            tap.stop()
        link.close()          # 最後に必ず ABORT を送って閉じる
        clogger.close()
        if not args.plain:
            # 全画面表示だと、終了後の画面に何も残らない。最後の数行だけ戻す。
            screen.clear()
            print("\n".join(messages[-20:]))
        print(f"\nログ: {log_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
