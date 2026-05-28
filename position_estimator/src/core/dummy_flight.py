"""
dummy_flight.py
カメラ未検出時のフォールバック用、仮想定常円旋回軌道を生成するクラス。
config.py の DUMMY_* パラメータで軌道を設定できる。
"""

import numpy as np
import time


class DummyFlight:
    def __init__(self, radius: float, altitude: float, period: float):
        """
        Args:
            radius   : 旋回半径 [m]
            altitude : 飛行高度 [m]
            period   : 1周にかかる時間 [s]
        """
        self.radius   = radius
        self.altitude = altitude
        self.period   = period
        self.t_start  = time.time()

    def reset(self):
        """現在時刻を起点にリセット"""
        self.t_start = time.time()

    def get_position(self) -> np.ndarray:
        """現在時刻に対応する3D座標 [X, Y, Z] を返す"""
        t     = time.time() - self.t_start
        omega = 2.0 * np.pi / self.period
        return np.array([
            self.radius * np.cos(omega * t),
            self.radius * np.sin(omega * t),
            self.altitude,
        ], dtype=np.float64)