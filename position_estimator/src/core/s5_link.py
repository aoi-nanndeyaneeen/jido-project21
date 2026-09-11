"""
core/s5_link.py
地上局 (XIAO RP2040 / env:xiao_s5_log) との USB シリアルを一手に引き受ける。

--------------------------------------------------------------------------
なぜ 1 つのクラスに下りと上りを両方持たせるのか
--------------------------------------------------------------------------
XIAO の USB シリアルは 1 本しかない。従来は
    ground_receiver/tools/s5_logger.py   … テレメトリを CSV へ
という別プロセスが占有していたので、position_estimator からは開けなかった。

ウェイポイント飛行では、PC が
    「機体の今の高度・モード・GUIDED の状態」を見ながら次の指令を出す
必要がある。見ながら出す以上、同じプロセスが両方を持つしかない。

    → s5_logger.py は **同時に起動しないこと**。このクラスが CSV も書く。

--------------------------------------------------------------------------
プロトコル (ground_receiver/src/tools/s5_log.cpp と対)
--------------------------------------------------------------------------
受信 (XIAO -> PC):
    "HEADER,<列名...>"   CSV の列定義。これを見て列を動的に覚える
    "LOG_START" / "LOG_STOP"
    "DATA,<値...>"       テレメトリ 1 パケット = 1 行
    "PARAM,k=v,..."      機体のゲイン (数秒に1回)
    "# ..."              人間向けメッセージ

送信 (PC -> XIAO):
    "1" / "0"            CSV 出力の ON / OFF
    "CMD,<req>,<vx_mmps>,<vy_mmps>,<alt_cm>,<yaw_rate_cdps>,<flags>"
                         上りコマンド。XIAO が 5Hz に間引いて無線へ流す

--------------------------------------------------------------------------
★ 送り続けることが生存確認を兼ねている
--------------------------------------------------------------------------
機体は「コマンドが 1 秒来ない = その場ホールド」「4 秒来ない = 自動着陸」
として扱う (QuadConfig.h § 9)。つまり PC は飛行中ずっと送り続ける必要が
あり、逆に PC が落ちれば機体は勝手に降りてくる。
このクラスは send_command() が呼ばれなくなった時点で何も送らない。
キープアライブのために勝手に前回値を送り直すことは **しない**
(止まったプロセスの最後の指令を無限に送り続けるのが一番危ない)。
"""

import threading
import time
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

# ground_receiver/src/tools/s5_log.cpp / include/S5Cmd.h の Req と同じ値
REQ_IDLE    = 0
REQ_HOLD    = 1
REQ_TAKEOFF = 2
REQ_GUIDED  = 3
REQ_LAND    = 4
REQ_ABORT   = 5

REQ_NAME = {REQ_IDLE: "IDLE", REQ_HOLD: "HOLD", REQ_TAKEOFF: "TAKEOFF",
            REQ_GUIDED: "GUIDED", REQ_LAND: "LAND", REQ_ABORT: "ABORT"}

# S5Cmd.h の CmdFlag
CF_ARMED_OK  = 1 << 0
CF_POS_VALID = 1 << 1
CF_YAW_VALID = 1 << 2
CF_ALT_ABS   = 1 << 3

# XIAO RP2040 の USB CDC。Windows では description で判別できないので VID:PID。
VID_PID_RP2040 = (0x2E8A, 0x000A)
VID_PID_TEENSY = (0x16C0, 0x0483)


def find_ground_port():
    """XIAO RP2040 を VID:PID で探す。見つからなければ None。"""
    for p in serial.tools.list_ports.comports():
        if (p.vid, p.pid) == VID_PID_RP2040:
            return p.device
    return None


class S5Link:
    """
    地上局 XIAO との双方向リンク。

    使い方:
        link = S5Link()                  # ポート自動検出
        if link.ok:
            link.send_command(REQ_TAKEOFF, 0.0, 0.0, 1.0)
            st = link.state()            # 最新テレメトリ (dict)
        link.close()

    state() が返す dict の中身は機体の CSV 列そのまま (s5_log.cpp の
    CSV_HEADER)。よく使うのは:
        mode      0=RATE 1=ANGLE 2=GUIDED 3=POSHOLD 4=ALTHOLD
        armed     0/1
        guided    0/1   GUIDED に入っているか
        cmd_fresh 0/1   機体から見て上りコマンドが新鮮か
        landed    0/1   自動着陸が完了して出力を切った
        range_h   対地高度 [m]
        alt_hold  目標高度 [m]
        flow_ok / range_valid / airborne
    """

    def __init__(self, port=None, baud=115200, log_dir=None, autostart_csv=True):
        self.port = port or find_ground_port()
        self.ok = False
        self._state = {}
        self._lock = threading.Lock()
        self._cols = None
        self._last_rx = 0.0
        self._n_data = 0
        self._csv = None
        self._running = False
        self._n_sent = 0
        self._last_param_line = None

        if self.port is None:
            print("[S5Link] 地上局 (XIAO RP2040 2E8A:000A) が見つかりません。")
            self._print_port_hint()
            return

        try:
            self.ser = serial.Serial(self.port, baud, timeout=0.2)
        except Exception as e:
            print(f"[S5Link] {self.port} を開けません: {e}")
            print("         s5_logger.py を同時に起動していませんか? "
                  "(同じポートは1プロセスしか開けません)")
            return

        self.ok = True
        self._running = True
        print(f"[S5Link] 地上局に接続: {self.port}")

        if log_dir is not None:
            log_dir = Path(log_dir)
            log_dir.mkdir(parents=True, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._csv_path = log_dir / f"s5_link_{ts}.csv"
            self._csv = open(self._csv_path, "w", encoding="utf-8", newline="")
            print(f"[S5Link] テレメトリ保存先: {self._csv_path}")

        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

        if autostart_csv:
            # '1' は冪等な ON (トグルの 'l' だと、既に ON のとき止めてしまう)
            self._write_raw("1\n")

    # ---------------------------------------------------------------- 受信
    @staticmethod
    def _print_port_hint():
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("         COM ポートが1つも見えていません。USB を挿し直してください。")
            return
        print("         接続中のポート:")
        for p in ports:
            tag = "  <- Teensy (機体側。地上局ではありません)" \
                  if (p.vid, p.pid) == VID_PID_TEENSY else ""
            vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "    -    "
            print(f"           {p.device}  {vidpid}  {p.description}{tag}")

    def _rx_loop(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except Exception as e:
                print(f"[S5Link] 受信エラー: {e}")
                time.sleep(0.2)
                continue
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if self._csv is not None and (line.startswith("DATA,")
                                          or line.startswith("HEADER,")
                                          or line.startswith("PARAM,")):
                self._csv.write(line + "\n")

            if line.startswith("HEADER,"):
                self._cols = line[len("HEADER,"):].split(",")
                continue

            if line.startswith("DATA,"):
                self._parse_data(line)
                continue

            if line.startswith("PARAM,"):
                self._last_param_line = line
                continue

            if line.startswith("#"):
                # 受信機からの人間向けメッセージ。切り分けに効くのでそのまま出す。
                print(f"[地上局] {line[1:].strip()}")

    def _parse_data(self, line):
        if self._cols is None:
            return
        vals = line[len("DATA,"):].split(",")
        if len(vals) != len(self._cols):
            # 列数が合わない = 受信機のファーム更新後に HEADER を取り損ねた。
            # 次の HEADER を待つ (s5_log.cpp は '1' を受けるたびに出し直す)。
            return
        d = {}
        for k, v in zip(self._cols, vals):
            try:
                d[k] = float(v)
            except ValueError:
                d[k] = v
        with self._lock:
            self._state = d
            self._last_rx = time.time()
            self._n_data += 1

    # ---------------------------------------------------------------- 状態
    def state(self):
        """最新テレメトリの dict (コピー)。未受信なら空 dict。"""
        with self._lock:
            return dict(self._state)

    def age(self):
        """最後にテレメトリを受けてからの秒数。未受信なら inf。"""
        with self._lock:
            if self._last_rx == 0.0:
                return float("inf")
            return time.time() - self._last_rx

    def telemetry_ok(self, max_age_s=1.0):
        return self.age() < max_age_s

    def n_data(self):
        with self._lock:
            return self._n_data

    def flag(self, name):
        """mode/armed/guided など 0/1 系の列を bool で取る。無ければ False。"""
        return bool(self.state().get(name, 0))

    # ---------------------------------------------------------------- 送信
    def _write_raw(self, s):
        if not self.ok:
            return
        try:
            self.ser.write(s.encode("utf-8"))
        except Exception as e:
            print(f"[S5Link] 送信エラー: {e}")

    def send_command(self, req, vx_mps=0.0, vy_mps=0.0, alt_m=0.0,
                     yaw_rate_dps=0.0, flags=0):
        """
        上りコマンドを 1 行送る。

        Args:
            req        : REQ_* のどれか
            vx_mps     : 機体座標の目標速度 前+ [m/s]
            vy_mps     : 機体座標の目標速度 右+ [m/s]
            alt_m      : 目標対地高度 [m]。0 以下 = 「現状維持」
            yaw_rate_dps: 目標ヨーレート [deg/s]。現状 機体側は未使用
            flags      : CF_* の論理和

        ★ これを呼び続けるあいだだけ機体は GUIDED でいられる。呼ぶのを
          止めれば機体はホールド -> 自動着陸に落ちる (それが仕様)。
        """
        if not self.ok:
            return
        self._n_sent += 1
        self._write_raw(
            "CMD,{:d},{:d},{:d},{:d},{:d},{:d}\n".format(
                int(req),
                int(round(vx_mps * 1000.0)),
                int(round(vy_mps * 1000.0)),
                int(round(alt_m * 100.0)),
                int(round(yaw_rate_dps * 100.0)),
                int(flags),
            )
        )

    def n_sent(self):
        return self._n_sent

    # ---------------------------------------------------------------- 終了
    def close(self):
        if not self.ok:
            return
        # 最後に必ず ABORT を送る。異常終了でなくても、機体側に「地上局は
        # もう指令を出さない」と明示的に伝えてからリンクを閉じる。
        try:
            self.send_command(REQ_ABORT)
            time.sleep(0.05)
            self._write_raw("0\n")          # 受信機の CSV 出力も止める
            time.sleep(0.05)
        except Exception:
            pass
        self._running = False
        try:
            self.ser.close()
        except Exception:
            pass
        if self._csv is not None:
            self._csv.close()
            print(f"[S5Link] テレメトリを保存しました: {self._csv_path}")
        print(f"[S5Link] 切断 (受信 {self._n_data} 行 / 送信 {self._n_sent} 行)")
