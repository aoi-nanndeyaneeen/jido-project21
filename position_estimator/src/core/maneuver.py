"""
core/maneuver.py
機体単独で完結する定型機動 (水平旋回 / 8の字 / 上昇旋回) の、地上局側の表現。

機体側 (flight_controller quad/Maneuver.h) が実際に飛ぶ。こちらが持つのは
  1) 何を送るか      … Maneuver (REQ とパラメータ)
  2) いつ何を送るか  … ManeuverRunner (開始バースト → 待つ → 完了/失敗を判定)
だけで、飛行中の制御には一切関与しない。

★ 送り方が肝 (2026-09-16 初飛行の教訓):
  console が REQ_CIRCLE を 10Hz で送り続けたため、機体は 1 周完了 → 即再開を
  繰り返して 2.4 周回った。機体側にも「同じ REQ の連続では再開しない」ラッチを
  入れたが、地上局側も「開始要求は短いバーストで送り、その後は REQ_IDLE で
  リンクだけ生かす」形にする。IDLE は機動中の機体には無視され (HOLD/ABORT/LAND
  だけが割り込める)、完了後は HOLD として扱われる。

使い方:
    m = Circle(fwd_mps=0.4, yaw_rate_dps=15.0, alt_m=0.5, laps=2)   # r≈1.5m
    runner = ManeuverRunner(m, link)
    runner.start()
    ...毎ティック...
    req, vx, yaw_rate, alt, laps = runner.request()  # これを送る
    state = runner.update()                         # "burst"/"running"/"done"/"failed"
"""

import time
from dataclasses import dataclass

from core.s5_link import REQ_CIRCLE, REQ_FIGURE8, REQ_CLIMB_TURN, REQ_IDLE


@dataclass(frozen=True)
class Maneuver:
    """1 回の定型機動。値は機体側の Q::Maneuver::begin() にそのまま渡る。"""
    req: int
    name: str
    fwd_mps: float          # 機体座標 前+ の巡航速度 [m/s]
    yaw_rate_dps: float     # 旋回レート [deg/s]。符号が最初の旋回方向 (右 +)
    alt_m: float            # CIRCLE/FIGURE8: 保持高度。CLIMB_TURN: 到達高度
    laps: int = 1           # CIRCLE: 周回数 / CLIMB: 低高度・高高度それぞれの周回数 / FIGURE8: 無視

    @property
    def n_legs(self) -> int:
        """機体側 Q::Maneuver が作る脚 (360°) の数。"""
        if self.req == REQ_FIGURE8:
            return 2
        if self.req == REQ_CLIMB_TURN:
            return 2 * self.laps + 1          # 低 laps + 上昇1 + 高 laps
        return self.laps

    @property
    def radius_m(self) -> float:
        import math
        w = abs(self.yaw_rate_dps) * math.pi / 180.0
        return self.fwd_mps / w if w > 0 else float("inf")

    @property
    def expected_duration_s(self) -> float:
        if abs(self.yaw_rate_dps) <= 0:
            return float("inf")
        return 360.0 * self.n_legs / abs(self.yaw_rate_dps)

    def describe(self) -> str:
        return (f"{self.name}x{self.n_legs}: v={self.fwd_mps:.2f}m/s ω={self.yaw_rate_dps:+.0f}deg/s "
                f"(r~{self.radius_m:.2f}m, 約{self.expected_duration_s:.0f}s) alt={self.alt_m:.2f}m")


def Circle(fwd_mps: float, yaw_rate_dps: float, alt_m: float, laps: int = 1) -> Maneuver:
    """水平旋回 laps 周 (ルール: 1周400点、連続2周1000点、半径1.5m以上)。"""
    return Maneuver(REQ_CIRCLE, "CIRCLE", fwd_mps, yaw_rate_dps, alt_m, laps=max(1, int(laps)))


def FigureEight(fwd_mps: float, yaw_rate_dps: float, alt_m: float) -> Maneuver:
    """8の字: 1周 → 逆回り1周。半径は同じ (ルール: 半径が著しく異なると不可)。"""
    return Maneuver(REQ_FIGURE8, "FIGURE8", fwd_mps, yaw_rate_dps, alt_m, laps=1)


def ClimbTurn(fwd_mps: float, yaw_rate_dps: float, alt_target_m: float, laps: int = 2) -> Maneuver:
    """上昇旋回: 開始高度で laps 周 → 回りながら alt_target へ上昇 → alt_target で laps 周。"""
    return Maneuver(REQ_CLIMB_TURN, "CLIMB", fwd_mps, yaw_rate_dps, alt_target_m, laps=max(1, int(laps)))


def from_config(kind, fwd_mps: float, yaw_rate_dps: float, alt_m: float, laps: int = 1):
    """config の文字列 ("circle"/"figure8"/"climb"/None) から作る。None なら None。"""
    if not kind:
        return None
    k = kind.lower()
    if k == "circle":
        return Circle(fwd_mps, yaw_rate_dps, alt_m, laps)
    if k in ("figure8", "figure_eight", "8"):
        return FigureEight(fwd_mps, yaw_rate_dps, alt_m)
    if k in ("climb", "climb_turn"):
        return ClimbTurn(fwd_mps, yaw_rate_dps, alt_m, laps)
    raise ValueError(f"unknown maneuver kind: {kind!r}")


class ManeuverRunner:
    """
    地上局側の進行管理。機体の telemetry `maneuver` フラグ (F_MANEUVER) の
    立ち上がり/立ち下がりで開始・完了を判定する。

    状態:
        idle    : start() 前
        burst   : 開始要求を BURST_S 秒送っている (無線の欠落に耐えるため複数回)
        running : 機体が maneuver=1 を返している
        done    : maneuver が 1 → 0 に落ちた (完了)
        failed  : 開始が確認できない / 想定時間を大きく超えた
    """

    BURST_S         = 1.0    # 開始要求を送り続ける時間 [s]
    START_TIMEOUT_S = 3.0    # burst 開始からこの時間 maneuver=1 が見えなければ失敗
    EXTRA_TIMEOUT_S = 10.0   # 想定所要時間 + これ を超えたら失敗

    def __init__(self, maneuver: Maneuver, link):
        self.m = maneuver
        self.link = link
        self.state = "idle"
        self.reason = ""
        self._t_start = None
        self._t_running = None
        self._seen_running = False

    # ------------------------------------------------------------
    def start(self, now=None):
        self._t_start = now if now is not None else time.time()
        self._t_running = None
        self._seen_running = False
        self.state = "burst"
        self.reason = ""

    def active(self) -> bool:
        return self.state in ("burst", "running")

    def request(self):
        """このティックに送るべき (req, vx_mps, yaw_rate_dps, alt_m, laps)。"""
        if self.state == "burst":
            return self.m.req, self.m.fwd_mps, self.m.yaw_rate_dps, self.m.alt_m, self.m.laps
        # running / done / failed: リンクだけ生かす。IDLE は機動を邪魔しない。
        return REQ_IDLE, 0.0, 0.0, 0.0, 0

    def update(self, now=None) -> str:
        if not self.active():
            return self.state
        now = now if now is not None else time.time()
        flag = bool(self.link.flag("maneuver"))

        if flag and not self._seen_running:
            self._seen_running = True
            self._t_running = now

        if self.state == "burst":
            if self._seen_running and now - self._t_start >= self.BURST_S:
                self.state = "running"
            elif not self._seen_running and now - self._t_start > self.START_TIMEOUT_S:
                self.state, self.reason = "failed", "機体が機動を開始しませんでした (maneuver フラグが立たない)"
            return self.state

        # running
        if not flag:
            self.state = "done"
            return self.state
        if now - self._t_running > self.m.expected_duration_s + self.EXTRA_TIMEOUT_S:
            self.state, self.reason = "failed", (
                f"機動が想定時間 ({self.m.expected_duration_s:.0f}s) を大きく超えました")
        return self.state

    def elapsed_s(self, now=None) -> float:
        if self._t_running is None:
            return 0.0
        return (now if now is not None else time.time()) - self._t_running
