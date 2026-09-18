"""
ble_monitor.py  -  log_recorder (XIAO ESP32C3) 経由で、BLE だけで完結する
                    デバッグ / 試験飛行用ツール。

    機体 (drone_s5) ──UART──> log_recorder XIAO ──BLE 125Hz Notify──> ここ (見る)
    機体 (drone_s5) <──UART── log_recorder XIAO <──BLE Write──────── ここ (指令)

==========================================================================
これは何か
==========================================================================
1. Arduino IDE のシリアルモニタみたいに、機体状態を流しっぱなしで見る
   (console.py の画面は BLE の最新値を1行だけ上書き表示するが、値の
   推移をそのまま目で追いたいときはそれでは物足りない)。
2. PID reset / IMU再キャリブレーション / デバイス確認 (I2C再走査) を
   BLE 経由で送る (P / k / i キー)。
3. (2026-09-18) 機体単独の定型機動 (水平旋回 / 8の字 / 上昇旋回) を選んで
   開始・中止する (c / 8 / u / スペース)。半径・速度は下の MANEUVER_*。

★ 3. で送るのは「どのパターンをどの半径・速度で飛ぶか」と「開始/中止」だけ。
  速度や高度をリアルタイムに操縦する口ではない。軌道は機体が自分で作り
  (flight_controller quad/Guided.h)、開始のゲートも機体側が見る:
    ・SW_HOVER=up で POSHOLD 待機中 (空中・スティック中立) ならその場で開始
    ・SW_HOVER が cen/down なら選択だけ。次に up に上げたときに飛ぶ
    ・実行中に別のパターンを送っても無視 (途中で差し替えない)
    ・スペース = 中止 → その場 POSHOLD。プロポ (SW_HOVER / スティック) は常に優先
  操縦指令は ble_tap の操縦用 characteristic (send_ctrl) に CmdFrame で載せる。
  デバッグ指令 (fire_action) とは別の口のまま。
  半径は CmdFrame の vx_mmps (速度) と yaw_rate_cdps (= v/r) で伝える。
  上昇旋回の到達高度・周回数は機体側 (QuadConfig.h CLIMB_*) の値。

★ 新しい無線プロトコルを画面表示のために足したわけではない。BLE 経由で
  既に受け取っている FlightLog::Rec (bin2csv.py が解釈しているのと同じ
  バイナリ) を文字列にしているだけ。指令のほうは log_recorder 側に
  新設した Write 用 characteristic を使う (詳細は core/ble_tap.py)。
  機体の選択/開始ログ (">>> 地上局: ...") は FC の USB シリアルにだけ出る。
  ここではモード表示 (GUIDED になったか) で確認する。

★ 地上局 (IM920 / COM ポート) には一切触らない。
★ console.py / main.py と同時に起動できるのは GROUND_LINK_BACKEND="im920"
  のときだけ。"ble" (既定) では console.py / main.py が同じ機体へ BLE 接続を
  張っているので、ここからは接続できない (1 台の PC から同じ機体へは 1 本)。
  その場合 P/k/i は console.py の同じキーから送れる。

==========================================================================
使い方
==========================================================================
    cd position_estimator/src
    python ble_monitor.py                  # 既定の名前 (S5-LogBLE) を自動で探す
    python ble_monitor.py --name S5-LogBLE
    python ble_monitor.py --save            # ついでに .BIN も残す (既定は残さない)
    python ble_monitor.py --no-keys         # キーを全部無効化し、表示専用にする

キー:
    P       PID リセット (単押し)
    k       IMU 再キャリブレーション (2回押し。機体は非アーム中のみ実行する)
    i       デバイス確認 / I2C再走査 (機体は非アーム中のみ実行する)
    c       水平旋回を送る (2回押し)
    8       8の字を送る (2回押し)
    u       上昇旋回を送る (2回押し)
    スペース 機動を中止 → POSHOLD (単押し)
    q       終了 (終了時にも中止を送る)
"""
#普通にlog取ってなくてもBLE経由で状態が見たい！！！
#デバイスチェックはもっと正確に状態がわからないかな？
import argparse
import math
import time
from pathlib import Path

from core.ble_tap import BleTap, ACT_PID_RESET, ACT_IMU_CAL, ACT_SELFTEST
from core.ble_link import BleS5Link
from core.keyreader import KeyReader
from core.s5_protocol import (REQ_IDLE, REQ_ABORT, REQ_CIRCLE, REQ_FIGURE8,
                              REQ_CLIMB_TURN)
from utils.config import LOG_DIR

# 'k' (IMU校正) / 機動を確定させるまでの2回目待ち時間 [s]。console.py と揃えてある。
CONFIRM_S = 3.0
# キー入力のポーリング間隔。人間の反応速度に対して十分速く、CPUも食わない。
POLL_S = 0.05

# 定型機動の幾何。3 つとも同じ。機体側でも範囲外はクランプされる
# (QuadConfig.h GUIDED_CMD_RADIUS_* / GUIDED_CMD_SPEED_*)。
MANEUVER_RADIUS_M = 0.75
MANEUVER_SPEED_MPS = 0.40
# +1 右旋回 / -1 左旋回 (8の字は最初の円の向き)
MANEUVER_DIR = +1

# キー -> (REQ, 表示名)
MANEUVER_KEYS = {
    "c": (REQ_CIRCLE, "水平旋回"),
    "8": (REQ_FIGURE8, "8の字"),
    "u": (REQ_CLIMB_TURN, "上昇旋回"),
}

# 機体は「req が変わった瞬間」だけを要求として扱う (Guided::onCommand)。
# 応答なし Write が1発落ちても届くよう REQ を数回書き、その後 IDLE に戻す
# (IDLE を挟むので、同じ機動をもう一度送れる)。
BURST_TIMES_S = (0.0, 0.15, 0.30)
IDLE_AFTER_S = 0.6


class CommandSender:
    """REQ を短く連打してから IDLE に戻す。メインループから tick() を呼ぶ。"""

    def __init__(self, link):
        self.link = link
        self._req = None
        self._args = {}
        self._t0 = 0.0
        self._sent = 0

    def start(self, req, **args):
        self._req = req
        self._args = args
        self._t0 = time.time()
        self._sent = 0
        self.tick()

    def tick(self):
        if self._req is None:
            return
        dt = time.time() - self._t0
        while self._sent < len(BURST_TIMES_S) and dt >= BURST_TIMES_S[self._sent]:
            self.link.send_command(self._req, **self._args)
            self._sent += 1
        if dt >= IDLE_AFTER_S:
            self.link.send_command(REQ_IDLE)
            self._req = None


def maneuver_args():
    rate_dps = math.degrees(MANEUVER_SPEED_MPS / MANEUVER_RADIUS_M) * MANEUVER_DIR
    return {"vx_mps": MANEUVER_SPEED_MPS, "yaw_rate_dps": rate_dps}


def main():
    ap = argparse.ArgumentParser(
        description="BLE 経由で機体ログを見て、デバッグ指令 (PID reset/"
                    "IMU校正/デバイス確認) と定型機動 (水平旋回/8の字/上昇旋回) を送るツール")
    ap.add_argument("--name", default="S5-LogBLE", help="BLE デバイス名")
    ap.add_argument("--save", action="store_true",
                    help="表示だけでなく .BIN にも保存する (既定: 保存しない)")
    ap.add_argument("--log-dir", default=str(LOG_DIR),
                    help="--save のときの保存先 (既定: position_estimator/src/logs)")
    ap.add_argument("--no-keys", action="store_true",
                    help="キーを全部無効化し、表示専用にする (誤操作防止)")
    args = ap.parse_args()

    outdir = Path(args.log_dir) if args.save else Path(".ble_monitor_tmp")

    tap = BleTap(outdir, name=args.name, on_event=print, verbose=True)
    if not tap.start():
        print("[ble_monitor] 起動に失敗しました (上のメッセージを参照)")
        return

    # 表示専用では操縦用の口を開かない (close() が ABORT を送るため)
    link = None if args.no_keys else BleS5Link(tap=tap, on_message=print)
    sender = CommandSender(link) if link is not None else None

    print(f"[ble_monitor] '{args.name}' を探しています... (Ctrl+C または q で終了)")
    if not args.no_keys:
        print("[ble_monitor] P=PIDリセット  k=IMU校正(2回押し)  i=デバイス確認  q=終了")
        print(f"[ble_monitor] c=水平旋回  8=8の字  u=上昇旋回 (各2回押し)  スペース=中止   "
              f"半径 {MANEUVER_RADIUS_M:.2f} m / {MANEUVER_SPEED_MPS:.2f} m/s / "
              f"{'右' if MANEUVER_DIR > 0 else '左'}旋回")

    t_k_armed = 0.0   # 'k' の1回目を押した時刻 (0 = 未確定)
    pending_key = None  # 機動キーの1回目
    t_pending = 0.0

    def fire(action, label):
        seq = tap.next_action_seq()
        if tap.fire_action(action, seq):
            print(f"[ble_monitor] {label} を送信 (action_seq={seq})")
        else:
            print(f"[ble_monitor] {label} を送れませんでした (BLE未接続)")

    def send_req(req, label, **kw):
        if not tap.status().get("connected"):
            print(f"[ble_monitor] {label} を送れませんでした (BLE未接続)")
            return
        sender.start(req, **kw)
        print(f"[ble_monitor] {label} を送信")

    try:
        with KeyReader() as kr:
            while True:
                if args.no_keys:
                    # 何も読まずにただ待つ (表示専用)。
                    time.sleep(1.0)
                    continue

                sender.tick()
                ch = kr.get()
                now = time.time()
                if pending_key is not None and now - t_pending >= CONFIRM_S:
                    print("[ble_monitor] 機動の送信を取り消しました (時間切れ)")
                    pending_key = None

                if ch == "q":
                    break
                elif ch == " ":
                    pending_key = None
                    send_req(REQ_ABORT, "中止 (→POSHOLD)")
                elif ch in MANEUVER_KEYS:
                    req, label = MANEUVER_KEYS[ch]
                    if pending_key == ch:
                        pending_key = None
                        send_req(req, f"{label} (半径 {MANEUVER_RADIUS_M:.2f} m, "
                                      f"{MANEUVER_SPEED_MPS:.2f} m/s)", **maneuver_args())
                    else:
                        pending_key = ch
                        t_pending = now
                        armed = tap.state().get("armed")
                        where = ("SW_HOVER=up で待機中ならすぐ飛びます"
                                 if armed else "非アーム: 選択だけ。飛行中に SW_HOVER を up で開始")
                        print(f"[ble_monitor] {label}? ({where}) "
                              f"{CONFIRM_S:.0f} 秒以内にもう一度 {ch} を押してください")
                elif ch == "P":
                    fire(ACT_PID_RESET, "PID reset")
                elif ch == "k":
                    if now - t_k_armed < CONFIRM_S:
                        fire(ACT_IMU_CAL, "IMU校正")
                        t_k_armed = 0.0
                    else:
                        t_k_armed = now
                        rec = tap.state()
                        armed = rec.get("armed")
                        warn = ("  ※テレメトリはアーム中と表示しています。"
                                "拒否されます" if armed else "")
                        print(f"[ble_monitor] IMU校正? 機体を水平な床に置いて"
                              f"静止させてから {CONFIRM_S:.0f} 秒以内に"
                              f"もう一度 k を押してください{warn}")
                elif ch == "i":
                    fire(ACT_SELFTEST, "デバイス確認(I2C再走査)")
                time.sleep(POLL_S)
    except KeyboardInterrupt:
        pass
    finally:
        if link is not None:
            link.close()   # ABORT を送ってから閉じる (tap は下で止める)
        tap.stop()
        if not args.save:
            # --save 無しのときは一時 .BIN を残さない (中身は画面に出し切っている)。
            for f in outdir.glob("LOG*.BIN"):
                try:
                    f.unlink()
                except OSError:
                    pass
            try:
                outdir.rmdir()
            except OSError:
                pass
        print("[ble_monitor] 終了しました")


if __name__ == "__main__":
    main()
