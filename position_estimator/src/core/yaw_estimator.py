"""
core/yaw_estimator.py
カメラのΔvと機体のΔvを照合してドローンの絶対ヨー方位を推定する。

YAW_HANDOFF.md §3〜§4 の position_estimator 側実装。

--------------------------------------------------------------------------
原理
--------------------------------------------------------------------------
クアッドは傾いた方向に加速する。機体は自分の roll/pitch を正確に知っている
（加速度計で補正されるためヨーと違ってドリフトしない）ので、
「機体座標系での速度変化」は既知。カメラは「フィールド座標系での速度変化」を
測れる。この2つのベクトルのなす角がヨーである。

ArduPilot EKF3 が磁気センサなしで飛ぶときの GSF yaw estimator と同じ原理。

--------------------------------------------------------------------------
座標系と符号（YAW_HANDOFF.md §8 の確定事項）
--------------------------------------------------------------------------
    フィールド : x=幅(右), y=奥行(奥), z=上。右手系
    機体       : FRD (前・右・下)。ヨーは右へ首を振るのが +

    psi = atan2(nx, ny)      n = 機首方向の水平単位ベクトル

    +y を北とみなした方位角。上から見て時計回りが正で、機体側の
    「右へ首を振るのが +」と一致する。

    ★ atan2(y, x) ではない。逆にすると回転方向が反転し、
      位置制御ループが正帰還になって機体が飛んでいく。

導出:
    機体前方 n = (nx, ny)、機体右方 r = (ny, -nx)  ※上から見て n を時計回りに90度
    Δv_field = dvx_body * n + dvy_body * r
    両辺の方位角をとると
        bearing(Δv_field) = psi + atan2(dvy_body, dvx_body)
    よって
        psi = bearing(Δv_field) - atan2(dvy_body, dvx_body)

--------------------------------------------------------------------------
なぜ速度ベクトルではなく「速度の変化」なのか
--------------------------------------------------------------------------
飛行機は横滑りしないので進行方向＝機首方向だが、クアッドは真横にも真後ろにも
飛べるため一致しない。autopilot.py の atan2(vy, vx) は固定翼の仮定であり、
クアッドでは使えない（YAW_HANDOFF.md §4-2）。
"""

import math
import time
from collections import deque
from typing import NamedTuple, Optional

import numpy as np

from utils.config import (YAW_VEL_WINDOW_S, YAW_DV_WINDOW_S,
                          YAW_MIN_DV_CAMERA, YAW_MIN_DV_BODY,
                          YAW_DV_RATIO_TOL, YAW_MAX_TURN_DEG,
                          YAW_AVG_COUNT, YAW_CONFIRM_N, YAW_CONFIRM_TOL_DEG,
                          YAW_HOLD_SEC, YAW_MEASURE_HZ,
                          YAW_ENTRY_MODE_ENABLED, YAW_ENTRY_MAX_DVY_BODY,
                          YAW_INITIAL_ALIGN_DEG)


# ============================================================
# 小道具
# ============================================================

def wrap180(deg: float) -> float:
    """角度を -180 〜 +180 に畳む。"""
    return (deg + 180.0) % 360.0 - 180.0


def bearing_deg(x: float, y: float) -> float:
    """
    フィールド座標のベクトル (x, y) の方位角 [deg]。
    +y を 0 とし、上から見て時計回りが正（機体のヨー符号に一致）。
    """
    return math.degrees(math.atan2(x, y))


def circular_mean_deg(angles_deg) -> Optional[float]:
    """
    角度の円形平均。単純平均は ±180° をまたぐと壊れるため必須。
    """
    if not len(angles_deg):
        return None
    s = sum(math.sin(math.radians(a)) for a in angles_deg)
    c = sum(math.cos(math.radians(a)) for a in angles_deg)
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return None
    return math.degrees(math.atan2(s, c))


class BodyDeltaV(NamedTuple):
    """機体から届いた200ms窓の速度変化（FRD の 前・右）。"""
    t_pc: float     # PC時刻に換算した窓の終端時刻
    t_ms: int       # 機体の millis()（生値）
    dvx: float      # 前方向 [m/s]
    dvy: float      # 右方向 [m/s]
    yaw_body: Optional[float] = None   # 機体のジャイロ積分ヨー [deg]（相対値）


class YawEstimate(NamedTuple):
    yaw_deg: float          # フィールド座標系での機首方位
    valid: bool             # 機体が使ってよいか
    n_measurements: int     # 円形平均に使った測定数
    spread_deg: float       # 直近測定のばらつき
    last_update: float      # 最後に測定が入った時刻
    source: str             # "entry" / "match" / "align" / "none"


# ============================================================
# 時刻合わせ
# ============================================================

class ClockSync:
    """
    機体の millis() と PC 時刻のオフセットを推定する。

    IM920 のリトライ等で受信遅延がばらつくため、平均ではなく
    **最小遅延**を採用する（最小遅延のパケットが最も真値に近い）。
    YAW_HANDOFF.md §5-3。
    """

    def __init__(self, warmup: int = 20):
        self.warmup = warmup
        self._samples = deque(maxlen=200)
        self.offset: Optional[float] = None   # t_pc = t_ms/1000 + offset

    def observe(self, t_ms: int, t_recv: float):
        self._samples.append(t_recv - t_ms / 1000.0)
        # 最小 = 最小遅延のパケット。外れ値に強い
        self.offset = min(self._samples)

    @property
    def ready(self) -> bool:
        return self.offset is not None and len(self._samples) >= min(3, self.warmup)

    def to_pc_time(self, t_ms: int) -> Optional[float]:
        if self.offset is None:
            return None
        return t_ms / 1000.0 + self.offset


# ============================================================
# カメラ側の速度履歴
# ============================================================

class VelocityHistory:
    """
    3D位置履歴から、窓内の最小二乗フィットで速度を出す。

    2点差分（main_loop.py の vel_ap）はオートパイロット用でノイズが大きすぎる。
    ヨー推定には別系統の平滑化した速度を使う（YAW_HANDOFF.md §4-4-1）。
    最小二乗は窓内の全点を使うので、2点差分より誤差が小さい。
    """

    def __init__(self, window_s: float = YAW_VEL_WINDOW_S, maxlen: int = 600):
        self.window_s = window_s
        self._buf = deque(maxlen=maxlen)   # (t, x, y, ok)

    def add(self, t: float, pos, tracking_ok: bool):
        if pos is None:
            self._buf.append((t, np.nan, np.nan, False))
        else:
            self._buf.append((t, float(pos[0]), float(pos[1]), bool(tracking_ok)))

    def velocity_at(self, t_end: float):
        """
        t_end で終わる窓の水平速度 (vx, vy) を返す。
        窓内に棄却区間や欠測があれば None（YAW_HANDOFF.md §4-4-4）。
        """
        t0 = t_end - self.window_s
        ts, xs, ys = [], [], []
        for (t, x, y, ok) in self._buf:
            if t < t0 - 1e-9 or t > t_end + 1e-9:
                continue
            if not ok or math.isnan(x):
                return None          # 窓内に棄却/欠測があれば窓ごと捨てる
            ts.append(t); xs.append(x); ys.append(y)

        if len(ts) < 5:
            return None
        ts = np.asarray(ts); span = ts[-1] - ts[0]
        if span < self.window_s * 0.6:
            return None              # 窓が十分埋まっていない

        tc = ts - ts.mean()
        denom = float((tc * tc).sum())
        if denom < 1e-9:
            return None
        vx = float((tc * (np.asarray(xs) - np.mean(xs))).sum() / denom)
        vy = float((tc * (np.asarray(ys) - np.mean(ys))).sum() / denom)
        return vx, vy


# ============================================================
# ヨー推定本体
# ============================================================

class YawEstimator:
    """
    使い方:
        est = YawEstimator()
        # カメラスレッド/メインループから毎フレーム
        est.add_position(t, P, tracking_ok=...)
        # シリアル受信から機体のΔvが届くたび
        est.add_body_dv(t_ms, dvx, dvy, t_recv, yaw_body)
        # 定期的に
        e = est.update()
        if e.valid: ...送信...
    """

    def __init__(self):
        self.clock = ClockSync()
        self.vel = VelocityHistory()
        self._body = deque(maxlen=200)          # BodyDeltaV
        self._measurements = deque(maxlen=YAW_AVG_COUNT)   # (t, yaw_deg)
        self._recent = deque(maxlen=YAW_CONFIRM_N)         # 収束判定用

        self._last_measure_t = 0.0
        self._last_valid_t = 0.0
        self._yaw = float(YAW_INITIAL_ALIGN_DEG)
        self._source = "align"
        self._valid = False
        self._reject_reason = ""
        self.stats = {"tried": 0, "accepted": 0, "rejected": {}}

    # ---------------- 入力 ----------------

    def add_position(self, t: float, pos, tracking_ok: bool = True):
        """カメラの3D位置。tracking_ok=False なら棄却区間として扱う。"""
        self.vel.add(t, pos, tracking_ok)

    def add_body_dv(self, t_ms: int, dvx: float, dvy: float,
                    t_recv: Optional[float] = None,
                    yaw_body: Optional[float] = None):
        """
        機体から届いた200ms窓のΔv（FRD の 前・右）。

        Args:
            t_ms  : 機体の millis()。窓の終端時刻
            dvx   : 前方向の速度変化 [m/s]
            dvy   : 右方向の速度変化 [m/s]
            yaw_body: 機体のジャイロ積分ヨー [deg]（相対値でよい。旋回検出用）
        """
        t_recv = time.time() if t_recv is None else t_recv
        self.clock.observe(t_ms, t_recv)
        t_pc = self.clock.to_pc_time(t_ms)
        if t_pc is None:
            return
        self._body.append(BodyDeltaV(t_pc, t_ms, float(dvx), float(dvy), yaw_body))

    # ---------------- 集計 ----------------

    def _body_dv_over(self, t_start: float, t_end: float):
        """
        t_start〜t_end の機体Δvを、200ms刻みのサンプルを足し合わせて作る。

        機体座標系での合成なので、窓の間に機体が大きく首を振っていると
        意味がなくなる。yaw_body があればその変化量で弾く。
        """
        picked = [b for b in self._body if t_start < b.t_pc <= t_end]
        if len(picked) < 2:
            return None, "body_samples"

        span = picked[-1].t_pc - picked[0].t_pc
        if span < (t_end - t_start) * 0.5:
            return None, "body_gap"

        yaws = [b.yaw_body for b in picked if b.yaw_body is not None]
        if len(yaws) >= 2:
            turn = abs(wrap180(yaws[-1] - yaws[0]))
            if turn > YAW_MAX_TURN_DEG:
                return None, "turning"

        dvx = sum(b.dvx for b in picked)
        dvy = sum(b.dvy for b in picked)
        return (dvx, dvy), None

    def _measure_once(self, now: float):
        """1回分の生ヨー測定。成功すれば角度[deg]、失敗すれば None。"""
        self.stats["tried"] += 1

        # ── カメラ側のΔv ─────────────────────────────
        v2 = self.vel.velocity_at(now)
        v1 = self.vel.velocity_at(now - YAW_DV_WINDOW_S)
        if v1 is None or v2 is None:
            return None, "camera_vel"

        dvx_f = v2[0] - v1[0]
        dvy_f = v2[1] - v1[1]
        mag_cam = math.hypot(dvx_f, dvy_f)
        if mag_cam < YAW_MIN_DV_CAMERA:
            return None, "dv_camera_small"

        # ── 機体側のΔv ───────────────────────────────
        body, why = self._body_dv_over(now - YAW_DV_WINDOW_S, now)
        if body is None:
            return None, why
        dvx_b, dvy_b = body
        mag_body = math.hypot(dvx_b, dvy_b)
        if mag_body < YAW_MIN_DV_BODY:
            return None, "dv_body_small"

        # ── 大きさの整合 ─────────────────────────────
        if abs(mag_cam - mag_body) / mag_cam > YAW_DV_RATIO_TOL:
            return None, "magnitude_mismatch"

        # ── ヨー算出 ─────────────────────────────────
        # 進入フェーズ: ほぼ純前進なら、カメラΔvの方位がそのまま機首方位。
        # 機体側のdvyノイズを噛まないぶん頑健（YAW_HANDOFF.md §6-②）。
        if YAW_ENTRY_MODE_ENABLED and abs(dvy_b) < YAW_ENTRY_MAX_DVY_BODY and dvx_b > 0:
            return wrap180(bearing_deg(dvx_f, dvy_f)), "entry"

        psi = bearing_deg(dvx_f, dvy_f) - math.degrees(math.atan2(dvy_b, dvx_b))
        return wrap180(psi), "match"

    def current(self) -> YawEstimate:
        """
        現在の推定値を返すだけの参照専用アクセサ。状態を一切変えない。

        ★ 表示やコマンド送信からは必ずこちらを呼ぶこと。
          update() は測定を1回進めるので、表示のたびに呼ぶと
          測定が二重に入り、収束判定も壊れる。
        """
        spread = 0.0
        if len(self._recent) >= 2:
            ref = circular_mean_deg(list(self._recent))
            if ref is not None:
                spread = max(abs(wrap180(a - ref)) for a in self._recent)
        return YawEstimate(
            yaw_deg=self._yaw,
            valid=self._valid,
            n_measurements=len(self._measurements),
            spread_deg=spread,
            last_update=self._last_valid_t,
            source=self._source if self._valid else "none",
        )

    def update(self, now: Optional[float] = None) -> YawEstimate:
        """
        測定を1回ぶん進める。内部でレート制限するので毎フレーム呼んでよい。

        ★ 1つのループから1回だけ呼ぶこと。参照だけなら current() を使う。
        """
        now = time.time() if now is None else now

        if now - self._last_measure_t >= 1.0 / YAW_MEASURE_HZ:
            self._last_measure_t = now
            raw, tag = self._measure_once(now)
            if raw is None:
                self._reject_reason = tag
                self.stats["rejected"][tag] = self.stats["rejected"].get(tag, 0) + 1
            else:
                self.stats["accepted"] += 1
                self._reject_reason = ""
                self._measurements.append((now, raw))
                self._recent.append(raw)
                self._source = tag

                mean = circular_mean_deg([m for _, m in self._measurements])
                if mean is not None:
                    self._yaw = wrap180(mean)
                self._last_valid_t = now

        # ── 収束判定 ─────────────────────────────────
        # 連続 N 回が互いに ±YAW_CONFIRM_TOL_DEG に収まったら有効とする
        if len(self._recent) >= YAW_CONFIRM_N:
            ref = circular_mean_deg(list(self._recent))
            if ref is not None:
                spread = max(abs(wrap180(a - ref)) for a in self._recent)
                if spread <= YAW_CONFIRM_TOL_DEG:
                    self._valid = True

        # 一定時間 測定が入らなければ無効へ落とす
        # （ホバリング中は加速度が無くヨーが観測できない。嘘の値を出さない）
        if self._valid and (now - self._last_valid_t) > YAW_HOLD_SEC:
            self._valid = False
            self._recent.clear()

        return self.current()

    # ---------------- 表示用 ----------------

    @property
    def last_reject_reason(self) -> str:
        return self._reject_reason

    def status_line(self) -> str:
        e = self.current()          # ★ update() を呼ばない（測定が二重に入る）
        head = f"YAW {e.yaw_deg:+7.1f}deg {'VALID' if e.valid else '  -  '}"
        if e.valid:
            return f"{head}  n={e.n_measurements} spread={e.spread_deg:.1f} ({e.source})"
        return f"{head}  reject={self._reject_reason or 'warming up'}"
