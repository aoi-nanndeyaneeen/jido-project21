import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import cv2
import time
from collections import deque

class ViewGraph:
    def __init__(self):
        self.fig, (self.ax_z, self.ax_xy) = plt.subplots(2, 1, figsize=(6, 6), dpi=100)
        
        # ★ ZとXYの履歴を別々に管理する
        self.history_z = deque()  # (time, z, target_z) を保存
        self.history_xy = deque() # (time, x, y) を保存
        
        self.window_sec = 10.0
        self.fig.tight_layout(pad=2.0)

    def get_image(self, P, current_z, target_alt):
        now = time.time()
        
        # ★ Z と Target_Z は【常に】記録する
        self.history_z.append((now, current_z, target_alt))
        
        # ★ X, Y は【検知した時だけ】記録する
        if P is not None:
            self.history_xy.append((now, P[0], P[1]))
        
        # 10秒以上古いデータを削除
        while self.history_z and now - self.history_z[0][0] > self.window_sec:
            self.history_z.popleft()
        while self.history_xy and now - self.history_xy[0][0] > self.window_sec:
            self.history_xy.popleft()

        self.ax_z.cla()
        self.ax_xy.cla()

        # --- 上段: 高度(Z)グラフの描画 ---
        if self.history_z:
            times_z = [d[0] - now for d in self.history_z]
            zs = [d[1] for d in self.history_z]
            targets = [d[2] for d in self.history_z]

            self.ax_z.set_ylim(0, 15)
            self.ax_z.set_xlim(-self.window_sec, 0)
            self.ax_z.plot(times_z, zs, color='blue', label='Current Alt(m)')
            self.ax_z.plot(times_z, targets, color='red', linestyle='--', label='Target Alt(m)')
            self.ax_z.set_title("Altitude (Z)", fontsize=10)
            self.ax_z.legend(loc='upper right', fontsize=8)
            self.ax_z.grid(True)
        else:
            self.ax_z.set_xlim(-self.window_sec, 0)
            self.ax_z.grid(True)

        # --- 下段: 水平(X, Y)グラフの描画 ---
        if self.history_xy:
            times_xy = [d[0] - now for d in self.history_xy]
            xs = [d[1] for d in self.history_xy]
            ys = [d[2] for d in self.history_xy]

            self.ax_xy.set_ylim(-10, 10)
            self.ax_xy.set_xlim(-self.window_sec, 0)
            self.ax_xy.plot(times_xy, xs, color='tab:red', label='X (m)')
            self.ax_xy.plot(times_xy, ys, color='tab:green', label='Y (m)')
            self.ax_xy.set_title("Horizontal Position (X, Y)", fontsize=10)
            self.ax_xy.legend(loc='upper right', fontsize=8)
            self.ax_xy.grid(True)
        else:
            self.ax_xy.set_xlim(-self.window_sec, 0)
            self.ax_xy.grid(True)

        return self._to_opencv_image()

    def _to_opencv_image(self):
        self.fig.tight_layout()
        self.fig.canvas.draw()
        img = np.asarray(self.fig.canvas.buffer_rgba())
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

    def close(self):
        plt.close(self.fig)