"""
autopilot.py
位置PIDによる自律飛行コントローラ。

機首方向:
    カメラ由来のヨー推定（core/yaw_estimator.py）を update() の heading_rad に渡す。
    未確定のあいだはアーム時の初期アラインメント値を保持する。
    ★ クアッドでは速度ベクトルから機首方向を推定してはいけない。
      進行方向と機首方向が一致しないため（YAW_HANDOFF.md §4-2）。

RCCommand の throttle/pitch/roll/yaw は -1.0〜1.0（throttleは0〜1）に
正規化し、呼び出し元で PWM 変換する。

ログ:
    update() を呼ぶたびに、目標値・現在位置・実際に送信したRCコマンドを
    自動でCSVに記録する（autopilot_YYYYMMDD_HHMMSS.csv）。
    tracker.py 側のログとは別ファイルになるが、Time列で後から突き合わせ可能。
    dummyモード中は呼び出し側で is_dummy=True を渡すことでログを止められる。
"""

import math
import time
import numpy as np
from dataclasses import dataclass
from pathlib import Path
from datetime import datetime

from utils.config import YAW_INITIAL_ALIGN_DEG
from utils.logger import CsvLogger


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


# ── 座標系と指令の符号 ─────────────────────────────────────────
# ★★ 飛行前に必ずベンチで確認すること ★★
#
# フィールド座標系 (utils/config.py):
#     x = 幅(右), y = 奥行(奥), z = 上。右手系
#
# ヨー角 yaw_rad (core/yaw_estimator.py と同一の定義):
#     yaw = atan2(nx, ny)   n = 機首方向の水平単位ベクトル
#     yaw = 0     → 機首がフィールド奥 (+y)
#     yaw = +90度 → 機首がフィールド右 (+x)   ※右回りが正
#
# 機体の指令符号 (flight_controller/include/quad/QuadConfig.h §1):
#     roll  + = 右へバンク  → 右へ加速
#     pitch + = 機首が上がる → **後ろへ加速**   ← 前進させたいときは負
#
# ⚠️ 2026-08-16 修正: 従来の実装は
#        fwd_err   =  dx*cos(h) + dy*sin(h)
#        right_err = -dx*sin(h) + dy*cos(h)
#        cmd.pitch = PID(fwd_err);  cmd.roll = PID(right_err)
#    となっており、**ピッチ・ロールともに符号が反転**していた。
#    (-sin h, cos h) は機体右方向ではなく**左方向**であり、
#    さらに「前進 = pitch 正」は固定翼の感覚で、クアッドでは逆。
#    この状態で位置制御ループを閉じると横にも前後にも正帰還になり、
#    YAW_HANDOFF.md §1 が警告する「機体が飛んでいく」状態になる。
SIGN_ROLL_FOR_RIGHT    = +1.0   # 右へ動かしたいとき roll に掛ける符号
SIGN_PITCH_FOR_FORWARD = -1.0   # 前へ動かしたいとき pitch に掛ける符号


def body_frame_errors(dx: float, dy: float, yaw_rad: float):
    """
    フィールド座標系の位置誤差 (dx, dy) を
    機体座標系の (前方誤差, 右方誤差) に変換する。

        n = (sin yaw, cos yaw)      機首方向
        r = (ny, -nx)               機体右方向（上から見て n を時計回りに90度）
    """
    nx, ny = math.sin(yaw_rad), math.cos(yaw_rad)
    rx, ry = ny, -nx
    return dx * nx + dy * ny, dx * rx + dy * ry


# 速度ベクトルから機首方向を推定してよいか。
# ★ クアッドでは必ず False。
#   飛行機は横滑りしないので進行方向＝機首方向だが、クアッドは真横にも
#   真後ろにも飛べるため一致しない。速度から推定すると機首方向を取り違え、
#   位置制御ループが正帰還になる（YAW_HANDOFF.md §4-2）。
HEADING_FROM_VELOCITY = False

_warned_no_yaw = False


def _resolve_yaw(self, yaw_rad, vx, vy):
    """
    使用するヨー角を決める。

    1. カメラ由来のヨーが渡されていればそれを使う（最優先）
    2. 無ければ前回値を保持する
       （初期値は config.YAW_INITIAL_ALIGN_DEG ＝ アーム時の初期アラインメント）

    クアッドでは速度ベクトルからの推定にフォールバックしない。
    """
    global _warned_no_yaw

    if yaw_rad is not None:
        return yaw_rad

    if HEADING_FROM_VELOCITY:
        if math.hypot(vx, vy) > self.MIN_SPEED_FOR_HEADING:
            # 固定翼用。クアッドでは到達しない
            return math.atan2(vy, vx)
        return self.heading

    if not _warned_no_yaw:
        print("[Autopilot] カメラ由来のヨーが未確定です。"
              "初期アラインメント値を保持します "
              f"({math.degrees(self.heading):+.1f}deg)。")
        print("            速度ベクトルからの推定はクアッドでは行いません"
              "（正帰還になるため）。")
        _warned_no_yaw = True
    return self.heading


# ── ログ記録 ───────────────────────────────────────────────────
class AutopilotLogger(CsvLogger):
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
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = Path(log_dir) / f"{prefix}_{ts}.csv"
        super().__init__(path, self.HEADER, mode="w")

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
        super().close()
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
        # 機首方位 [rad]。定義は上部 SIGN_* のコメント参照
        # 初期値はアーム時の初期アラインメント（YAW_HANDOFF.md §6-1）
        self.heading = math.radians(YAW_INITIAL_ALIGN_DEG)

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
            heading_rad : 機首方位 (rad)。yaw_estimator の psi と同一定義
                          (0=フィールド奥+y、右回りが正)。
                          None なら前回値を保持する（速度からは推定しない）。
            is_dummy    : True の場合、このフレームはログに記録しない
                          （tracker.py が dummy フォールバック中のとき呼び出し側から渡す）
        Returns:
            RCCommand
        """
        px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
        vx, vy     = float(vel[0]), float(vel[1])

        # ── 機首方向の決定 ────────────────────────────────────
        self.heading = _resolve_yaw(self, heading_rad, vx, vy)

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

        fwd_err, right_err = body_frame_errors(dx, dy, self.heading)

        # ── PID 計算 ──────────────────────────────────────────
        # 符号の根拠は上部の SIGN_* 定数のコメントを参照
        cmd = RCCommand()
        cmd.pitch    = SIGN_PITCH_FOR_FORWARD * self.pid_fwd.update(fwd_err, dt)
        cmd.roll     = SIGN_ROLL_FOR_RIGHT    * self.pid_right.update(right_err, dt)
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


# ── ホバリング（その場保持） ───────────────────────────────────
class HoverHold:
    """
    生成された時点の位置（またはpos省略時は原点）でその場ホバリングを維持する。

    SquarePatrol と同じ PID／ヘディング推定／ロギングの作り方を流用し、
    update()/close() のインターフェースを揃えることで、main_loop.py 側は
    どのモードが有効かを意識せずに autopilot オブジェクトを差し替えられる。
    """

    ALTITUDE               = SquarePatrol.ALTITUDE
    MIN_SPEED_FOR_HEADING  = SquarePatrol.MIN_SPEED_FOR_HEADING
    HOVER_THROTTLE         = SquarePatrol.HOVER_THROTTLE
    KP_H, KI_H, KD_H = SquarePatrol.KP_H, SquarePatrol.KI_H, SquarePatrol.KD_H
    KP_Z, KI_Z, KD_Z = SquarePatrol.KP_Z, SquarePatrol.KI_Z, SquarePatrol.KD_Z

    def __init__(self,
                 pos=None,
                 enable_logging: bool = True,
                 log_dir: str = "logs"):
        if pos is None:
            self.target = (0.0, 0.0, self.ALTITUDE)
        else:
            self.target = (float(pos[0]), float(pos[1]), float(pos[2]))
        self.heading = math.radians(YAW_INITIAL_ALIGN_DEG)

        self.pid_fwd   = PID(self.KP_H, self.KI_H, self.KD_H, limit=0.6)
        self.pid_right = PID(self.KP_H, self.KI_H, self.KD_H, limit=0.6)
        self.pid_z     = PID(self.KP_Z, self.KI_Z, self.KD_Z, limit=0.5)

        self.logger = AutopilotLogger(log_dir=log_dir) if enable_logging else None

        print(f"[HoverHold] 保持位置: "
              f"({self.target[0]:.1f}, {self.target[1]:.1f}, {self.target[2]:.1f})")

    def update(self,
               pos: np.ndarray,
               vel: np.ndarray,
               dt: float,
               heading_rad: float | None = None,
               is_dummy: bool = False) -> RCCommand:
        px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
        vx, vy     = float(vel[0]), float(vel[1])

        self.heading = _resolve_yaw(self, heading_rad, vx, vy)

        tx, ty, tz = self.target
        dx, dy, dz = tx - px, ty - py, tz - pz

        fwd_err, right_err = body_frame_errors(dx, dy, self.heading)

        cmd = RCCommand()
        cmd.pitch    = SIGN_PITCH_FOR_FORWARD * self.pid_fwd.update(fwd_err, dt)
        cmd.roll     = SIGN_ROLL_FOR_RIGHT    * self.pid_right.update(right_err, dt)
        cmd.throttle = self.HOVER_THROTTLE + self.pid_z.update(dz, dt)
        cmd.throttle = max(0.0, min(1.0, cmd.throttle))

        if self.logger is not None and not is_dummy:
            self.logger.write(
                wp_idx=0,
                heading_rad=self.heading,
                pos=pos,
                vel=vel,
                target=self.target,
                cmd=cmd,
            )

        return cmd

    @property
    def target_point(self) -> tuple:
        return self.target

    def close(self):
        if self.logger is not None:
            self.logger.close()


# ── ウェイポイント飛行（今後実装予定） ──────────────────────────
class WaypointMission(HoverHold):
    """
    ウェイポイント飛行モード（未実装）。

    ルート追従ロジックが実装されるまでは、安全側としてその場ホバリング
    （HoverHold）にフォールバックする。update()/close() は HoverHold から
    そのまま継承しているため、main_loop.py からは他モードと同じ扱いで良い。
    """

    _warned = False

    def __init__(self,
                 pos=None,
                 enable_logging: bool = True,
                 log_dir: str = "logs"):
        if not WaypointMission._warned:
            print("[WaypointMission] 未実装のため、現在位置でホバリングします。")
            WaypointMission._warned = True
        super().__init__(pos=pos, enable_logging=enable_logging, log_dir=log_dir)