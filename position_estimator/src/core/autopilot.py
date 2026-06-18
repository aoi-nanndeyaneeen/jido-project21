"""
autopilot.py
位置PIDによる自律飛行コントローラ。

機首方向:
    9軸センサ未導入時は速度ベクトルから推定（暫定）。
    センサ導入後は update() の heading_rad 引数に渡すだけで切り替え可能。

RCCommand の throttle/pitch/roll/yaw は -1.0〜1.0（throttleは0〜1）に
正規化し、呼び出し元で PWM 変換する。
"""

import math
import time
import numpy as np
from dataclasses import dataclass, field


# ── データ型 ───────────────────────────────────────────────────
@dataclass
class RCCommand:
    throttle: float = 0.0   # 0 ~ 1
    pitch:    float = 0.0   # 前後 -1 ~ 1
    roll:     float = 0.0   # 左右 -1 ~ 1
    yaw:      float = 0.0   # 旋回 -1 ~ 1（現状未使用）

    def __str__(self):
        return (f"CMD thr={self.throttle:.2f} "
                f"pit={self.pitch:.2f} "
                f"rol={self.roll:.2f} "
                f"yaw={self.yaw:.2f}")


# ── PID コントローラ ───────────────────────────────────────────
class PID:
    def __init__(self, kp: float, ki: float, kd: float, limit: float = 1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self._prev_err  = 0.0
        self._integral  = 0.0

    def update(self, error: float, dt: float) -> float:
        self._integral = max(-self.limit,
                             min(self.limit, self._integral + error * dt))
        deriv = (error - self._prev_err) / max(dt, 1e-6)
        self._prev_err = error
        out = self.kp * error + self.ki * self._integral + self.kd * deriv
        return max(-self.limit, min(self.limit, out))

    def reset(self):
        self._prev_err = 0.0
        self._integral = 0.0


# ── 四角形旋回ミッション ──────────────────────────────────────
class SquarePatrol:
    """
    指定した起点から1辺 SIDE_LEN [m] の正方形を
    反時計回りに繰り返し旋回する。

    ウェイポイント順（起点を左下として）:
        WP0(start) → WP1(+X) → WP2(+X+Y) → WP3(+Y) → WP0 ...

    config で変更可能なパラメータ:
        SIDE_LEN, ALTITUDE, ARRIVAL_R,
        MIN_SPEED_FOR_HEADING, HOVER_THROTTLE
    """

    # ── チューニングパラメータ ─────────────────────────────────
    ALTITUDE              = 3.0   # 飛行高度 [m]
    SIDE_LEN              = 6.0   # 1辺の長さ [m]
    ARRIVAL_R             = 0.5   # 到達判定半径（水平）[m]
    ARRIVAL_Z             = 0.3   # 到達判定半径（高度）[m]
    MIN_SPEED_FOR_HEADING = 0.3   # heading推定に使う最小速度 [m/s]
    HOVER_THROTTLE        = 0.5   # ホバリング時スロットル（重量に応じて調整）

    # ── PID ゲイン ─────────────────────────────────────────────
    KP_H, KI_H, KD_H = 0.40, 0.01, 0.10   # 水平（前後/左右共通）
    KP_Z, KI_Z, KD_Z = 0.80, 0.05, 0.20   # 高度

    def __init__(self, start_x: float = 0.0, start_y: float = 0.0):
        L = self.SIDE_LEN
        x0, y0, z = start_x, start_y, self.ALTITUDE
        self.waypoints = [
            (x0,     y0,     z),   # WP0: 起点
            (x0 + L, y0,     z),   # WP1: +X
            (x0 + L, y0 + L, z),   # WP2: +X+Y
            (x0,     y0 + L, z),   # WP3: +Y
        ]
        self.wp_idx  = 0
        self.heading = 0.0          # 推定機首方向 [rad]

        self.pid_fwd   = PID(self.KP_H, self.KI_H, self.KD_H, limit=0.6)
        self.pid_right = PID(self.KP_H, self.KI_H, self.KD_H, limit=0.6)
        self.pid_z     = PID(self.KP_Z, self.KI_Z, self.KD_Z, limit=0.5)

        print(f"[SquarePatrol] ウェイポイント:")
        for i, wp in enumerate(self.waypoints):
            print(f"  WP{i}: ({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})")

    # ── メイン更新（毎制御サイクル呼ぶ） ─────────────────────
    def update(self,
               pos: np.ndarray,
               vel: np.ndarray,
               dt: float,
               heading_rad: float | None = None) -> RCCommand:
        """
        Args:
            pos         : 現在位置 [x, y, z] (m)
            vel         : 現在速度 [vx, vy, vz] (m/s)
            dt          : 前回呼び出しからの経過時間 (s)
            heading_rad : 機首方向 (rad)。None なら速度ベクトルから推定。
        Returns:
            RCCommand
        """
        px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
        vx, vy     = float(vel[0]), float(vel[1])

        # ── 機首方向の推定 ────────────────────────────────────
        if heading_rad is not None:
            self.heading = heading_rad          # センサ値を優先
        else:
            horiz_speed = math.sqrt(vx**2 + vy**2)
            if horiz_speed > self.MIN_SPEED_FOR_HEADING:
                self.heading = math.atan2(vy, vx)
            # else: 低速時は前回値を保持

        # ── ウェイポイント到達チェック ────────────────────────
        tx, ty, tz = self.waypoints[self.wp_idx]
        dist_h = math.sqrt((px - tx)**2 + (py - ty)**2)
        dist_z = abs(pz - tz)

        if dist_h < self.ARRIVAL_R and dist_z < self.ARRIVAL_Z:
            prev_idx = self.wp_idx
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            self._reset_pids()
            tx, ty, tz = self.waypoints[self.wp_idx]
            print(f"[SquarePatrol] WP{prev_idx} 到達 → WP{self.wp_idx} "
                  f"({tx:.1f}, {ty:.1f}, {tz:.1f})")

        # ── 位置誤差 → 機体座標系に変換 ──────────────────────
        dx = tx - px
        dy = ty - py
        dz = tz - pz

        ch, sh     = math.cos(self.heading), math.sin(self.heading)
        fwd_err    =  dx * ch + dy * sh    # 機首方向の誤差 → pitch
        right_err  = -dx * sh + dy * ch   # 右方向の誤差   → roll

        # ── PID 計算 ──────────────────────────────────────────
        cmd = RCCommand()
        cmd.pitch    = self.pid_fwd.update(fwd_err,    dt)
        cmd.roll     = self.pid_right.update(right_err, dt)
        cmd.throttle = self.HOVER_THROTTLE + self.pid_z.update(dz, dt)
        cmd.throttle = max(0.0, min(1.0, cmd.throttle))

        return cmd

    # ── 現在の目標ウェイポイント ──────────────────────────────
    @property
    def target(self) -> tuple:
        return self.waypoints[self.wp_idx]

    def _reset_pids(self):
        for pid in [self.pid_fwd, self.pid_right, self.pid_z]:
            pid.reset()