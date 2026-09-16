"""
dummy_source.py
カメラ・センサの代わりに合成データを生成するモジュール。
インターフェースは CameraTracker / SerialReceiver と完全に同一。
"""

import time
import numpy as np
import cv2


def make_lookat_camera(eye, target, width=1920, height=1080):
    eye    = np.array(eye,    dtype=np.float64)
    target = np.array(target, dtype=np.float64)

    focal = float(width)
    K = np.array([[focal, 0, width  / 2],
                  [0, focal, height / 2],
                  [0,     0,          1]], dtype=np.float32)

    fwd = target - eye
    fwd /= np.linalg.norm(fwd)

    world_up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(fwd, world_up)) > 0.99:
        world_up = np.array([0.0, 1.0, 0.0])

    right = np.cross(world_up, fwd)
    right /= np.linalg.norm(right)
    down  = np.cross(fwd, right)
    down  /= np.linalg.norm(down)

    R    = np.stack([right, down, fwd]).astype(np.float32)
    tvec = (-R @ eye).reshape(3, 1).astype(np.float32)
    return K, R, tvec


class DummySensor:
    """SerialReceiver と同インターフェース"""
    def __init__(self, altitude: float = 3.0):
        self._altitude = altitude
        self._accel    = (0.0, 0.0, -9.8)   # 水平飛行 = 重力のみ

    def get_altitude(self) -> float:
        return self._altitude

    def get_accel(self) -> tuple:
        return self._accel

    def stop(self):
        pass


class DummyCamera:
    """
    CameraTracker と同インターフェース。
    フィールド中心(0,0)を原点とした定常円旋回を合成する。
    """

    def __init__(self,
                 K:        np.ndarray,
                 R:        np.ndarray,
                 tvec:     np.ndarray,
                 width:    int   = 1920,
                 height:   int   = 1080,
                 center:   tuple = (0.0, 0.0),   # フィールド中心 = 原点
                 radius:   float = 4.0,
                 altitude: float = 3.0,
                 omega:    float = 0.6):
        self.K        = K
        self.R        = R
        self.tvec     = tvec
        self.width    = width
        self.height   = height
        self._cx, self._cy = center
        self._radius  = radius
        self._alt     = altitude
        self._omega   = omega
        self._t0      = time.time()
        self._rvec, _ = cv2.Rodrigues(R)

        self.prev_gray = None   # CameraTracker との互換

    def read_and_track(self):
        theta   = self._omega * (time.time() - self._t0)
        P_world = np.array([[
            self._cx + self._radius * np.cos(theta),
            self._cy + self._radius * np.sin(theta),
            self._alt,
        ]], dtype=np.float32)

        pts, _ = cv2.projectPoints(P_world, self._rvec, self.tvec, self.K, None)
        u, v   = pts[0][0]
        iu, iv = int(round(u)), int(round(v))

        # ── フレーム生成 ───────────────────────────
        frame = np.full((self.height, self.width, 3), 30, dtype=np.uint8)

        # フィールド枠（4隅を投影）
        field_3d = np.array([
            [-8., -8., 0.], [ 8., -8., 0.],
            [ 8.,  8., 0.], [-8.,  8., 0.],
        ], dtype=np.float32)
        corners, _ = cv2.projectPoints(field_3d, self._rvec, self.tvec, self.K, None)
        corners = corners.reshape(-1, 2).astype(int)
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1) % 4]),
                     (60, 60, 60), 2)

        # フィールド中心マーカー（= 原点）
        orig_3d = np.array([[0., 0., 0.]], dtype=np.float32)
        op, _   = cv2.projectPoints(orig_3d, self._rvec, self.tvec, self.K, None)
        opx, opy = int(op[0][0][0]), int(op[0][0][1])
        cv2.drawMarker(frame, (opx, opy), (100, 100, 100), cv2.MARKER_CROSS, 24, 2)
        cv2.putText(frame, "O(0,0)", (opx+8, opy-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100,100,100), 1)

        # 機体ドット（座標テキストは tracker.py が描くので二重にしない）
        if 0 <= iu < self.width and 0 <= iv < self.height:
            cv2.circle(frame, (iu, iv), 18, (0, 200, 0), -1)
            cv2.circle(frame, (iu, iv), 19, (255, 255, 255), 2)
            cv2.putText(frame, "[SIM]",
                        (iu + 22, iv - 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 200, 200), 2)
            return frame, (iu, iv)

        cv2.putText(frame, "[SIM] plane out of view",
                    (50, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 200), 2)
        return frame, None

    def release(self):
        pass