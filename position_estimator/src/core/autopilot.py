"""
autopilot.py
位置PIDによる自律飛行コントローラ。

機首方向:
    9軸センサ未導入時は速度ベクトルから推定（暫定）。
    センサ導入後は update() の heading_rad 引数に渡すだけで切り替え可能。

RCCommand の throttle/pitch/roll/yaw は -1.0〜1.0（throttleは0〜1）に
正規化し、呼び出し元で PWM 変換する。

ログ:
    update() を呼ぶたびに、目標値・現在位置・実際に送信したRCコマンドを
    自動でCSVに記録する（autopilot_YYYYMMDD_HHMMSS.csv）。
    tracker.py 側のログとは別ファイルになるが、Time列で後から突き合わせ可能。
    dummyモード中は呼び出し側で is_dummy=True を渡すことでログを止められる。
"""

import csv
import math
import time
import numpy as np
from dataclasses import dataclass
from pathlib import Path
from datetime import datetime


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


# ── ログ記録 ───────────────────────────────────────────────────
class AutopilotLogger:
    """
    autopilot の入出力を CSV に記録する。

    列構成:
        Time, WP_idx, Heading(deg),
        Pos_X(m), Pos_Y(m), Pos_Z(m),
        Vel_X(m/s), Vel_Y(m/s),
        Target_X(m), Target_Y(m), Target_Z(m),
        Cmd_Throttle, Cmd_Pitch, Cmd_Roll, Cmd_Yaw
    """

    HEADER = [
        "Time", "WP_idx", "Heading(deg)",
        "Pos_X(m)", "Pos_Y(m)", "Pos_Z(m)",
        "Vel_X(m/s)", "Vel_Y(m/s)",
        "Target_X(m)", "Target_Y(m)", "Target_Z(m)",
        "Cmd_Throttle", "Cmd_Pitch", "Cmd_Roll", "Cmd_Yaw",
    ]

    def __init__(self, log_dir: str = "logs", prefix: str = "autopilot"):
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = Path(log_dir) / f"{prefix}_{ts}.csv"

        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)

        print(f"[AutopilotLogger] 記録開始: {self.path}")

    def write(self,
              wp_idx: int,
              heading_rad: float,
              pos: np.ndarray,
              vel: np.ndarray,
              target: tuple,
              cmd: RCCommand):
        row = [
            datetime.now().isoformat(timespec="milliseconds"),
            wp_idx,
            f"{math.degrees(heading_rad):.2f}",
            f"{float(pos[0]):.4f}", f"{float(pos[1]):.4f}", f"{float(pos[2]):.4f}",
            f"{float(vel[0]):.4f}", f"{float(vel[1]):.4f}",
            f"{target[0]:.4f}", f"{target[1]:.4f}", f"{target[2]:.4f}",
            f"{cmd.throttle:.4f}", f"{cmd.pitch:.4f}",
            f"{cmd.roll:.4f}", f"{cmd.yaw:.4f}",
        ]
        self._writer.writerow(row)

    def close(self):
        self._file.close()
        print(f"[AutopilotLogger] 記録終了: {self.path}")


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

    def __init__(self,
                 start_x: float = 0.0,
                 start_y: float = 0.0,
                 enable_logging: bool = True,
                 log_dir: str = "logs"):
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

        self.logger = AutopilotLogger(log_dir=log_dir) if enable_logging else None

        print(f"[SquarePatrol] ウェイポイント:")
        for i, wp in enumerate(self.waypoints):
            print(f"  WP{i}: ({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})")

    # ── メイン更新（毎制御サイクル呼ぶ） ─────────────────────
    def update(self,
               pos: np.ndarray,
               vel: np.ndarray,
               dt: float,
               heading_rad: float | None = None,
               is_dummy: bool = False) -> RCCommand:
        """
        Args:
            pos         : 現在位置 [x, y, z] (m)
            vel         : 現在速度 [vx, vy, vz] (m/s)
            dt          : 前回呼び出しからの経過時間 (s)
            heading_rad : 機首方向 (rad)。None なら速度ベクトルから推定。
            is_dummy    : True の場合、このフレームはログに記録しない
                          （tracker.py が dummy フォールバック中のとき呼び出し側から渡す）
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

        # ── ログ記録 ──────────────────────────────────────────
        if self.logger is not None and not is_dummy:
            self.logger.write(
                wp_idx=self.wp_idx,
                heading_rad=self.heading,
                pos=pos,
                vel=vel,
                target=(tx, ty, tz),
                cmd=cmd,
            )

        return cmd

    # ── 現在の目標ウェイポイント ──────────────────────────────
    @property
    def target(self) -> tuple:
        return self.waypoints[self.wp_idx]

    def close(self):
        """終了時に呼ぶ（ログファイルをきちんと閉じる）"""
        if self.logger is not None:
            self.logger.close()

    def _reset_pids(self):
        for pid in [self.pid_fwd, self.pid_right, self.pid_z]:
            pid.reset()