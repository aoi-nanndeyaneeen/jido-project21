"""
console.py  -  機体の状態を見ながら指令を出す地上局コンソール

    [BLE 版 (既定。utils/config.py の GROUND_LINK_BACKEND = "ble")]
    機体 (drone_s5) ──UART──> log_recorder XIAO ──BLE──> ここ ──> 画面
                    <──UART──                  <──BLE──  ここ <── キー入力
      (テレメトリ 20Hz / 指令 10Hz / 機体125Hzログを 1 本の BLE 接続で)

    [IM920 版 (--link im920 / 旧経路)]
    機体 (drone_s5) ──IM920 8Hz──> 地上局 XIAO ──USB──> ここ ──> 画面
                    <──IM920 8Hz──          <──USB──  ここ <── キー入力
    機体 (drone_s5) ──UART──> log_recorder XIAO ──BLE 125Hz──> ここ

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
  であって、モーターを切るものではない (機体側 quad/Guided.h の
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
import math
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from core.keyreader import KeyReader
from core.ble_link import open_ground_link
from core.s5_link import (REQ_ABORT, REQ_GUIDED, REQ_HOLD,
                          REQ_IDLE, REQ_LAND, REQ_TAKEOFF, REQ_NAME,
                          CF_ARMED_OK, CF_ALT_ABS, CF_POS_CORR, CF_POS_SHIFT)
from core.maneuver import Circle, FigureEight, ClimbTurn, ManeuverRunner
from utils.config import (GROUND_LINK_BACKEND, BLE_LOG_NAME,
                          GROUND_LINK_PORT, LOG_DIR, MISSION_TAKEOFF_ALT_M,
                          COMP_MANEUVER_SPEED, COMP_TURN_RADIUS_M,
                          COMP_TURN_RIGHT, COMP_CIRCLE_LAPS, COMP_CLIMB_ALT_M)
from utils.logger import CsvLogger
from utils.screen import Screen

# ---- 指令の刻み -----------------------------------------------------------
# ★ 上限は機体側 Q::GUIDED_MAX_VEL (0.5 m/s) でも必ずクランプされる。
#   ここはそれより内側に取る (手で押す以上、行き過ぎを機体任せにしない)。
VEL_STEP_MPS = 0.05
VEL_MAX_MPS = 0.40
ALT_STEP_M = 0.05
ALT_MAX_M = 2.00

# 上りコマンドの送信レート [Hz]。
#  ★ 2026-09-16: 5 -> 10。地上局 XIAO は「届いた順に、最短 125ms 間隔」で
#    流すようになった (ground_receiver GroundConfig.h の CMD_MIN_GAP_MS)。速く送るほど、
#    キーを押してから無線に出るまでの待ちが短くなる。無線の帯域を決めて
#    いるのは地上局側だけなので、ここを上げても混まない。
SEND_HZ = 10.0

# 機体側のフェイルセーフ (QuadConfig.h)。画面の注意書きに使う。
STALE_HOLD_S = 1.0
STALE_LAND_S = 4.0

# ---- 定型機動 (c 水平旋回 / 8 8の字 / u 上昇旋回) --------------------------
#  ★ 本番 (main.py) と同じ値を使う。ここで別の値にすると、練習で確かめた挙動と
#    本番の挙動が食い違う。変えたいときは utils/config.py の COMP_* を直すこと。
#  半径 r[m] = 速度 / (ヨーレート * pi/180)。ルールブックは「概ね 1.5m 以上」。
#    実測は指令より 2〜3割大きく出る (2026-09-16: 指令 0.57m、フィット 0.7〜0.78m)。
#  ★ 押した時点で機体が GUIDED に入って静止ホバリング中であること。
#    円の中心は「開始地点から旋回方向へ r 横」。右回り(+)なら左寄りで始める。
#    (main.py は開始地点を機首から逆算して円の中心をフィールド中心に置くが、
#     console.py は「今いる場所から」なので、位置は操縦者が決める)
#    送り方は core/maneuver.py の ManeuverRunner (1秒バースト → IDLE)。
MANEUVER_SPEED_MPS    = COMP_MANEUVER_SPEED
MANEUVER_YAW_RATE_DPS = (1.0 if COMP_TURN_RIGHT else -1.0) * math.degrees(
    COMP_MANEUVER_SPEED / COMP_TURN_RADIUS_M)
MANEUVER_LAPS         = COMP_CIRCLE_LAPS   # c: 連続2周 (1000点) / u: 低・高それぞれ
CLIMB_TURN_ALT_M      = COMP_CLIMB_ALT_M   # 上昇旋回の到達高度 [m] (MISSION_FENCE_Z 未満に)

# ---- デッドマン --------------------------------------------------------
#  ★ 2026-09-16: w/a/s/d で入れた速度指令は「次に押すまで残り続ける」ので、
#    手を離しても機体は同じ速度で進み続け、フィールド外 (フロー積分で
#    E=+2.9m) まで出た。GUIDED 中にこの秒数キー操作が無ければ速度を 0 に
#    戻す (= その場ホールド)。動かし続けたければ押し続ける (連打する)。
DEADMAN_S = 1.5

# モード番号 / 高度サブ状態 -> 名前。protocol/S5Telem.h から生成 (core/s5_protocol.py)。
#  ★ 2026-09-16 までここに手書きされていた ALT_STATE_NAME は機体の実体
#    (1=STANDBY 2=NO_HOVER_THR 3=NO_RANGE 4=HOLDING 5=RANGE_LOST) と食い違っていた。
from core.s5_protocol import MODE_NAME, ALT_STATE_NAME   # noqa: E402



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
        self._yaw_rate = 0.0
        self._alt = alt_m
        self._runner = None           # 実行中の定型機動 (core.maneuver.ManeuverRunner)
        self._t_key = 0.0             # 最後に速度を触った時刻 (デッドマン用)
        self._origin_pending = False  # 次の1回だけ CF_POS_SHIFT(0,0) を乗せる
        self._tx = False              # ★ 起動時は必ず送信 OFF
        self._event = ""
        self._n_tx = 0
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ---- 状態 ----------------------------------------------------------
    def snapshot(self):
        with self._lock:
            req, vx, vy, yr, alt = self._req, self._vx, self._vy, self._yaw_rate, self._alt
            mst = None
            r = self._runner
            if r is not None and r.active():
                req, vx, yr, alt_m, laps = r.request()
                vy = 0.0
                if alt_m > 0.0:
                    alt = alt_m
                mst = (r.m.name, r.state, r.elapsed_s())
            else:
                laps = 0
            return {"req": req, "vx": vx, "vy": vy, "yaw_rate": yr, "laps": laps,
                    "alt": alt, "tx_on": self._tx, "maneuver": mst,
                    "flags": CF_ARMED_OK | CF_ALT_ABS, "n_tx": self._n_tx}

    # ---- 操作 ----------------------------------------------------------
    def set_req(self, req, event=""):
        with self._lock:
            if self._runner is not None and self._runner.active():
                self._event = f"機動 {self._runner.m.name} を中断 -> {REQ_NAME.get(req, req)}"
            self._runner = None       # 手動操作は機動より優先 (HOLD/ABORT は機体側でも割り込む)
            self._req = req
            if req in (REQ_HOLD, REQ_TAKEOFF, REQ_LAND, REQ_ABORT):
                self._vx = self._vy = 0.0   # 水平は機体側でも 0 にされる
            self._yaw_rate = 0.0
            self._tx = True
            self._event = event or f"REQ={REQ_NAME.get(req, req)}"

    def set_maneuver(self, maneuver):
        """定型機動を1回だけ実行する。送り方 (バースト→IDLE) と完了判定は ManeuverRunner。"""
        with self._lock:
            self._runner = ManeuverRunner(maneuver, self.link)
            self._runner.start()
            self._vx = self._vy = 0.0
            self._yaw_rate = 0.0
            self._tx = True
            self._event = f"MANEUVER {maneuver.describe()}"

    def nudge(self, dvx=0.0, dvy=0.0, dalt=0.0):
        with self._lock:
            self._vx = max(-VEL_MAX_MPS, min(VEL_MAX_MPS, self._vx + dvx))
            self._vy = max(-VEL_MAX_MPS, min(VEL_MAX_MPS, self._vy + dvy))
            self._alt = max(0.0, min(ALT_MAX_M, self._alt + dalt))
            self._t_key = time.time()
            self._event = (f"SET vx={self._vx:+.2f} vy={self._vy:+.2f} "
                           f"alt={self._alt:.2f}")

    def zero_vel(self):
        with self._lock:
            self._vx = self._vy = 0.0
            self._event = "SET vx=0 vy=0"

    def set_origin(self):
        """今いる場所を機体側フェンスの中心 (0,0) にする。機体は動かない。"""
        with self._lock:
            self._origin_pending = True
            self._event = "ORIGIN (CF_POS_SHIFT 0,0) -> 機体側フェンス有効"

    def stop_tx(self):
        with self._lock:
            self._tx = False
            self._req = REQ_IDLE
            self._vx = self._vy = 0.0
            self._event = "TX STOP (機体は 1s でホールド -> 4s で自動着陸)"

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
            # 定型機動の進行 (完了/失敗したら IDLE に落として HOLD に任せる)
            with self._lock:
                r = self._runner
                if r is not None and r.active():
                    st = r.update(t0)
                    if st == "done":
                        self._event = f"MANEUVER {r.m.name} 完了 ({r.elapsed_s(t0):.1f}s) -> HOLD"
                        self._req = REQ_IDLE
                        self._runner = None
                    elif st == "failed":
                        self._event = f"MANEUVER {r.m.name} 失敗: {r.reason} -> HOLD"
                        self._req = REQ_HOLD
                        self._runner = None
            # デッドマン: GUIDED で速度が入ったまま DEADMAN_S 触られていなければ 0 へ
            with self._lock:
                if (self._tx and self._req == REQ_GUIDED
                        and (self._vx != 0.0 or self._vy != 0.0)
                        and t0 - self._t_key > DEADMAN_S):
                    self._vx = self._vy = 0.0
                    self._event = f"DEADMAN {DEADMAN_S:.1f}s 操作なし -> vx=0 vy=0 (ホールド)"
                origin = self._origin_pending
                self._origin_pending = False
            snap = self.snapshot()
            event = self.take_event()
            send_now = snap["tx_on"]
            if send_now:
                flags = snap["flags"]
                corr_n = corr_e = None
                if origin:
                    flags |= CF_POS_CORR | CF_POS_SHIFT
                    corr_n = corr_e = 0.0
                self.link.send_command(snap["req"], vx_mps=snap["vx"],
                                       vy_mps=snap["vy"], alt_m=snap["alt"],
                                       yaw_rate_dps=snap["yaw_rate"], laps=snap["laps"],
                                       flags=flags, corr_n_m=corr_n, corr_e_m=corr_e)
                with self._lock:
                    self._n_tx += 1
            elif origin:
                with self._lock:
                    self._event = "ORIGIN は送信中 (h/g など) にしか送れません"
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
        elif ch in ("c", "8", "u"):
            if not self.link.flag("guided"):
                self.say("[Console] 機動は機体が GUIDED に入ってから (h で HOLD を送って静止させてから)")
            else:
                alt_now = float(self.link.state().get("range_h", 0.0) or 0.0)
                hold_alt = cmdr.snapshot()["alt"]
                if ch == "c":
                    m = Circle(MANEUVER_SPEED_MPS, MANEUVER_YAW_RATE_DPS, hold_alt, MANEUVER_LAPS)
                elif ch == "8":
                    m = FigureEight(MANEUVER_SPEED_MPS, MANEUVER_YAW_RATE_DPS, hold_alt)
                else:
                    m = ClimbTurn(MANEUVER_SPEED_MPS, MANEUVER_YAW_RATE_DPS, CLIMB_TURN_ALT_M, MANEUVER_LAPS)
                cmdr.set_maneuver(m)
                self.say(f"[Console] {m.describe()} を開始 (今の高度 {alt_now:.2f}m)。"
                         "機体センサのみで完結し、終わると自動でホールド")
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
        elif ch == "o":
            if self.link.flag("airborne"):
                cmdr.set_origin()
                self.say("[Console] ここをフェンス中心 (0,0) にしました。"
                         "画面の FRAME が点いたら機体側フェンス有効")
            else:
                self.say("[Console] o は離陸後 (airborne) にだけ効きます "
                         "(地上では機体が位置を毎ループ 0 に戻すため)")
        elif ch == "r":
            cmdr.nudge(dalt=+ALT_STEP_M)
        elif ch == "f":
            cmdr.nudge(dalt=-ALT_STEP_M)
        elif ch in ("P", "k", "i"):
            self._maintenance(ch, now)
        elif ch in ("S", "D", "Z", "C"):
            # 地上局 XIAO のキーをそのまま転送する (ground_receiver main.cpp の handleKey)
            fwd = {"S": "s", "D": "d", "Z": "z", "C": "1"}[ch]
            self.link.send_key(fwd)
            self.say(f"[Console] 地上局へ '{fwd}' を送りました")
        elif ch == "?":
            self.help_on = not self.help_on
        return True

    def _maintenance(self, ch, now):
        """PID reset / IMU校正 / デバイス確認 (BLE のデバッグ指令)。

        ★ IM920 版: 使えない (IM920 は操縦専用)。ble_monitor.py を別に起動する。
        ★ BLE 版: 地上局リンクと同じ BLE 接続を使うので、ble_monitor.py は
          同時に接続できない (1 台の PC から同じ機体へは 1 本だけ)。代わりに
          ここから送る。キーと確認手順は ble_monitor.py と同じ。
        """
        tap = getattr(self.link, "tap", None)
        if tap is None:
            self.say("[Console] P/k/i は IM920 経由では使えません。"
                     "ble_monitor.py を使ってください (BLE経由)")
            return
        from core.ble_tap import ACT_PID_RESET, ACT_IMU_CAL, ACT_SELFTEST
        if ch == "k" and now - self._t_imu_cal >= self.TAKEOFF_CONFIRM_S:
            self._t_imu_cal = now
            self.say("[Console] IMU校正? 機体を水平に置いて静止させ、"
                     f"{self.TAKEOFF_CONFIRM_S:.0f}秒以内にもう一度 k (アーム中は機体が拒否)")
            return
        action, label = {"P": (ACT_PID_RESET, "PID reset"),
                         "k": (ACT_IMU_CAL, "IMU校正"),
                         "i": (ACT_SELFTEST, "デバイス確認")}[ch]
        if ch == "k":
            self._t_imu_cal = 0.0
        seq = tap.next_action_seq()
        if tap.fire_action(action, seq):
            self.say(f"[Console] {label} を送信 (BLE, seq={seq})。結果は [BLE] ACK 行に出ます")
        else:
            self.say(f"[Console] {label} を送れませんでした (BLE 未接続)")


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
        _onoff(tel, "landed", "LANDED"), _onoff(tel, "maneuver", "CIRCLE中"),
        _onoff(tel, "frame_ok", "FRAME"), _onoff(tel, "flow_ok", "FLOW"),
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
    yaw_txt = (f"  yaw_rate {cmd['yaw_rate']:+.1f}deg/s"
              if cmd.get("yaw_rate", 0.0) != 0.0 else "")
    mst = cmd.get("maneuver")
    if mst:
        yaw_txt += f"  [{mst[0]} {mst[1]} {mst[2]:.1f}s]"
    lines.append(f" TX     {tx_mark}  REQ={REQ_NAME.get(cmd['req'], cmd['req']):<8}"
                 f" vx {cmd['vx']:+.2f}  vy {cmd['vy']:+.2f}  alt {alt_txt}"
                 f"  flags 0x{cmd['flags']:02X}{yaw_txt}")
    if not cmd["tx_on"]:
        lines.append(f"        (黙っている間、機体は {STALE_HOLD_S:.0f}s でホールド"
                     f" -> {STALE_LAND_S:.0f}s で自動着陸)")
    lines.append(thin)

    # ---- キー ---------------------------------------------------------
    if help_on:
        lines += [
            " h HOLD   t TAKEOFF(2回押し)   g GUIDED   l LAND   SPACE ABORT(=即HOLD)",
            f" c 水平旋回{MANEUVER_LAPS}周  8 8の字  u 上昇旋回({MANEUVER_LAPS}周→{CLIMB_TURN_ALT_M:.1f}m→{MANEUVER_LAPS}周) "
            f"v={MANEUVER_SPEED_MPS:.2f} ω={MANEUVER_YAW_RATE_DPS:+.0f} r~1.5m ※GUIDED中",
            " x 送信停止   w/s 前後   a/d 左右   0 速度ゼロ   r/f 目標高度   q 終了",
            f" o ここをフェンス中心に (離陸後)   ★ {DEADMAN_S:.1f}s 操作が無いと速度は自動で 0",
            " S 地上局の状態   D 生データ表示   Z 統計クリア   C CSV出力ON   ? ヘルプ"
            "  (S/D/C は IM920 版のみ)",
            " P PIDリセット  k IMU校正(2回押し)  i デバイス確認  (BLE版のみ。IM920版は ble_monitor.py)",
            " ★ w/a/s/d は機体座標 (機首向き基準)。緊急停止はプロポ。",
            " ★ 旋回中にヨースティックへ触れると機体側が即中断する (安全網)。",
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
    ap.add_argument("--link", choices=("ble", "im920"), default=GROUND_LINK_BACKEND,
                    help="地上局リンクの経路 (既定: utils/config.py の "
                         "GROUND_LINK_BACKEND)。機体側 S5Features.h の "
                         "GROUND_LINK とそろえること")
    ap.add_argument("--port", default=GROUND_LINK_PORT,
                    help="[im920] 地上局 XIAO の COM ポート (既定: VID:PID で自動検出)")
    ap.add_argument("--no-ble", action="store_true",
                    help="log_recorder の BLE 125Hzログを受けない。"
                        "既定では自動で探しにいく (bleak 未導入 / 機体が"
                        "見つからないときは黙って諦めて続行するので、"
                        "付けなくても害はない)")
    ap.add_argument("--ble-name", default=BLE_LOG_NAME, help="BLE デバイス名")
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

    tap = None
    if args.link == "ble":
        # BLE 版: 地上局リンクと機体125Hzログは同じ BLE 接続 (link 側が起こす)。
        if args.no_ble:
            say("[Console] --link ble では地上局リンクに BLE を使うので --no-ble は無視します")
        link = open_ground_link("ble", ble_name=args.ble_name, log_dir=log_dir,
                                on_message=say)
        if not link.ok:
            print("\n".join(messages))
            print("\nBLE を使えません。bleak (pip install bleak) を確認してください。"
                  "IM920 に戻すなら --link im920 (機体側も戻すこと)。")
            return 1
        tap = link.tap
    else:
        link = open_ground_link("im920", port=args.port, log_dir=log_dir, on_message=say)
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
    if args.link == "im920" and not args.no_ble:
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
        link.close()          # 最後に必ず ABORT を送って閉じる (BLE 版はタップもここで止まる)
        if tap is not None and args.link == "im920":
            tap.stop()
        clogger.close()
        if not args.plain:
            # 全画面表示だと、終了後の画面に何も残らない。最後の数行だけ戻す。
            screen.clear()
            print("\n".join(messages[-20:]))
        print(f"\nログ: {log_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
