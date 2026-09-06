import cv2
import numpy as np
import math
import time
from collections import deque

from utils.config import VELOCITY_SMOOTH_FRAMES, VIEW_X, VIEW_Y


def _nice_step(span, target_ticks=6):
    """目盛り間隔を 1/2/2.5/5 × 10^n に丸める。"""
    if span <= 0:
        return 1.0
    raw = span / target_ticks
    mag = 10 ** math.floor(math.log10(raw))
    for mult in (1, 2, 2.5, 5, 10):
        if raw <= mult * mag:
            return mult * mag
    return 10 * mag


class ViewVelocity:
    W, H = 1000, 500
    _TOP_BOX = (55, 620, 45, 455)   # left, right, top, bottom

    def __init__(self, field_points):
        self.field_points = np.asarray(field_points[:4], dtype=np.float32)
        self._history = deque()
        self._vx = (float(VIEW_X[0]), float(VIEW_X[1]))
        self._vy = (float(VIEW_Y[0]), float(VIEW_Y[1]))

    def _smooth_velocity(self, P):
        now = time.time()
        self._history.append((now, P.copy()))
        while len(self._history) > VELOCITY_SMOOTH_FRAMES + 20:
            self._history.popleft()
        recent = list(self._history)[-VELOCITY_SMOOTH_FRAMES:]
        if len(recent) < 2:
            return np.zeros(3)
        t0, P0 = recent[0]
        t1, P1 = recent[-1]
        dt = t1 - t0
        if dt < 1e-6:
            return np.zeros(3)
        return (P1 - P0) / dt

    def get_image(self, P, roll_deg=0.0, pitch_deg=0.0, imu_available=False):
        image = np.full((self.H, self.W, 3), (245, 245, 245), dtype=np.uint8)
        velocity = np.zeros(3)
        if P is not None:
            velocity = self._smooth_velocity(P)
        speed_h = float(np.linalg.norm(velocity[:2]))
        speed_3d = float(np.linalg.norm(velocity))
        altitude_rate = float(velocity[2])
        altitude = float(P[2]) if P is not None else 0.0
        self._draw_top(image, P, velocity)
        self._draw_metrics(image, speed_h, speed_3d, altitude_rate, altitude)
        self._draw_adi(image, roll_deg, pitch_deg, imu_available)
        return image

    def _top_point(self, x, y):
        """フィールド座標 [m] → トップビューのピクセル座標。X/Y 等倍。"""
        left, right, top, bottom = self._TOP_BOX
        span_x = self._vx[1] - self._vx[0]
        span_y = self._vy[1] - self._vy[0]
        scale = min((right - left) / span_x, (bottom - top) / span_y)
        mid_x = 0.5 * (self._vx[0] + self._vx[1])
        mid_y = 0.5 * (self._vy[0] + self._vy[1])
        px = 0.5 * (left + right) + (x - mid_x) * scale
        py = 0.5 * (top + bottom) - (y - mid_y) * scale
        return int(round(px)), int(round(py))

    def _draw_top(self, image, P, velocity):
        left, right, top, bottom = self._TOP_BOX
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.rectangle(image, (left, top), (right, bottom), (255, 255, 255), -1)

        # データ座標に沿ったグリッド + 目盛り値
        step_x = _nice_step(self._vx[1] - self._vx[0])
        tick = math.ceil(self._vx[0] / step_x) * step_x
        while tick <= self._vx[1] + 1e-6:
            px, _ = self._top_point(tick, self._vy[0])
            if left <= px <= right:
                cv2.line(image, (px, top), (px, bottom), (228, 228, 228), 1)
                cv2.putText(image, f"{tick:.0f}", (px - 8, bottom + 15), font, 0.4, (110, 110, 110), 1)
            tick += step_x

        step_y = _nice_step(self._vy[1] - self._vy[0])
        tick = math.ceil(self._vy[0] / step_y) * step_y
        while tick <= self._vy[1] + 1e-6:
            _, py = self._top_point(self._vx[0], tick)
            if top <= py <= bottom:
                cv2.line(image, (left, py), (right, py), (228, 228, 228), 1)
                cv2.putText(image, f"{tick:.0f}", (left - 28, py + 4), font, 0.4, (110, 110, 110), 1)
            tick += step_y

        cv2.rectangle(image, (left, top), (right, bottom), (120, 120, 120), 1)

        field = np.array([self._top_point(point[0], point[1])
                          for point in self.field_points], dtype=np.int32)
        cv2.polylines(image, [field], True, (120, 120, 120), 1, cv2.LINE_AA)
        origin = self._top_point(0.0, 0.0)
        cv2.line(image, (origin[0], top), (origin[0], bottom), (205, 205, 205), 1)
        cv2.line(image, (left, origin[1]), (right, origin[1]), (205, 205, 205), 1)

        cv2.putText(image, "Top view (XY)", (left, 28), font, 0.7, (35, 35, 35), 2)
        cv2.putText(image, "X [m]", (right - 45, bottom + 30), font, 0.5, (80, 80, 80), 1)
        cv2.putText(image, "Y [m]", (4, top + 6), font, 0.5, (80, 80, 80), 1)

        points = [(t, p) for t, p in self._history if p is not None]
        for previous, current in zip(points, points[1:]):
            cv2.line(image, self._top_point(previous[1][0], previous[1][1]),
                     self._top_point(current[1][0], current[1][1]),
                     (40, 160, 115), 2, cv2.LINE_AA)
        if P is not None:
            point = self._top_point(P[0], P[1])
            if left <= point[0] <= right and top <= point[1] <= bottom:
                cv2.circle(image, point, 7, (40, 160, 115), -1, cv2.LINE_AA)
                if np.linalg.norm(velocity[:2]) > 0.1:
                    end = self._top_point(P[0] + velocity[0], P[1] + velocity[1])
                    cv2.arrowedLine(image, point, end, (60, 70, 220), 2, cv2.LINE_AA, tipLength=0.2)

    def _draw_metrics(self, image, speed_h, speed_3d, altitude_rate, altitude):
        x = 660
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image, "Velocity & Altitude", (x, 35), font, 0.7, (35, 35, 35), 2)
        rows = [
            ("Speed (H)", f"{speed_h:.1f} m/s"),
            ("Speed (3D)", f"{speed_3d:.1f} m/s"),
            ("Climb rate", f"{altitude_rate:+.1f} m/s"),
            ("Altitude", f"{altitude:.2f} m"),
        ]
        for index, (label, value) in enumerate(rows):
            y = 80 + index * 38
            cv2.putText(image, label, (x, y), font, 0.55, (100, 100, 100), 1)
            cv2.putText(image, value, (x + 180, y), font, 0.65, (20, 20, 20), 2)
            cv2.line(image, (x, y + 12), (980, y + 12), (210, 210, 210), 1)

    def _draw_adi(self, image, roll_deg, pitch_deg, imu_available):
        font = cv2.FONT_HERSHEY_SIMPLEX
        center = (820, 355)
        radius = 95
        sky = (180, 155, 90) if imu_available else (193, 166, 138)
        ground = (10, 95, 140) if imu_available else (83, 107, 122)
        cv2.circle(image, center, radius, sky, -1, cv2.LINE_AA)

        pitch = float(np.clip(pitch_deg / 60.0, -0.95, 0.95))
        horizon_y = -pitch * radius
        angle = np.radians(roll_deg)
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])

        def to_screen(pts):
            return (np.asarray(pts, dtype=np.float32) @ rotation.T) + np.array(center)

        overlay = image.copy()
        ground_poly = to_screen([[-radius * 2, horizon_y], [radius * 2, horizon_y],
                                 [radius * 2, radius * 2], [-radius * 2, radius * 2]])
        cv2.fillConvexPoly(overlay, np.round(ground_poly).astype(np.int32), ground)

        # ピッチ目盛り（±10, ±20 deg）
        for deg, half in ((10, 13), (20, 20)):
            for sign in (1, -1):
                yy = horizon_y - sign * (deg / 60.0) * radius
                seg = to_screen([[-half, yy], [half, yy]])
                cv2.line(overlay, tuple(np.round(seg[0]).astype(int)),
                         tuple(np.round(seg[1]).astype(int)), (235, 235, 235), 1, cv2.LINE_AA)

        # 地平線（地面ポリゴンと同じ変換）
        horizon = to_screen([[-radius, horizon_y], [radius, horizon_y]])
        cv2.line(overlay, tuple(np.round(horizon[0]).astype(int)),
                 tuple(np.round(horizon[1]).astype(int)), (255, 255, 255), 2, cv2.LINE_AA)

        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.circle(mask, center, radius, 255, -1)
        image[mask == 255] = overlay[mask == 255]

        cv2.circle(image, center, radius, (80, 80, 80), 2, cv2.LINE_AA)
        cv2.line(image, (center[0] - 42, center[1]), (center[0] - 12, center[1]), (0, 200, 255), 3)
        cv2.line(image, (center[0] + 12, center[1]), (center[0] + 42, center[1]), (0, 200, 255), 3)
        cv2.circle(image, center, 3, (0, 200, 255), -1)

        label = f"Attitude  R:{roll_deg:+.0f} deg  P:{pitch_deg:+.0f} deg"
        if not imu_available:
            label += "  (NO SENSOR)"
        cv2.putText(image, label, (660, 486), font, 0.46, (70, 70, 70), 1)

    def close(self):
        pass
