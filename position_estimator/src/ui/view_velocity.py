import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Circle, Polygon
import numpy as np
import cv2
import time
from collections import deque

from utils.config import VELOCITY_SMOOTH_FRAMES, FIELD_POINTS


class ViewVelocity:
    def __init__(self, field_points):
        self.fig = plt.figure(figsize=(10, 5), dpi=100)
        gs = gridspec.GridSpec(2, 2, figure=self.fig,
                               width_ratios=[1.4, 1],
                               height_ratios=[1, 1.4],
                               hspace=0.45, wspace=0.35)
        self.ax_top = self.fig.add_subplot(gs[:, 0])   # 左列全体: トップビュー
        self.ax_met = self.fig.add_subplot(gs[0, 1])   # 右上: 数値
        self.ax_adi = self.fig.add_subplot(gs[1, 1])   # 右下: 姿勢指示器

        fp = field_points[:4]
        self.field_xs = np.append(fp[:, 0], fp[0, 0])
        self.field_ys = np.append(fp[:, 1], fp[0, 1])

        self._history = deque()   # (timestamp, P)

    # ── 速度計算 ────────────────────────────────────────────────
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

    # ── メイン描画 ──────────────────────────────────────────────
    # 修正箇所: imu_available=False を引数に追加しました
    def get_image(self, P, roll_deg=0.0, pitch_deg=0.0, imu_available=False):
        vel = np.zeros(3)
        if P is not None:
            vel = self._smooth_velocity(P)

        speed_h  = float(np.linalg.norm(vel[:2]))
        speed_3d = float(np.linalg.norm(vel))
        vz  = float(vel[2])
        alt = float(P[2]) if P is not None else 0.0

        self._draw_top(P, vel)
        self._draw_metrics(speed_h, speed_3d, vz, alt)
        self._draw_adi(roll_deg, pitch_deg, imu_available)

        self.fig.canvas.draw()
        img = np.asarray(self.fig.canvas.buffer_rgba())
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

    # ── トップビュー（XY俯瞰） ───────────────────────────────────
    def _draw_top(self, P, vel):
        ax = self.ax_top
        ax.cla()

        ax.plot(self.field_xs, self.field_ys, color='gray',
                lw=1, linestyle='--', alpha=0.7)
        ax.axhline(0, color='gray', lw=0.4, alpha=0.3)
        ax.axvline(0, color='gray', lw=0.4, alpha=0.3)

        # 軌跡（古いほど薄い）
        pts = [(t, p) for t, p in self._history if p is not None]
        if len(pts) > 1:
            n   = len(pts)
            xs  = [p[0] for _, p in pts]
            ys  = [p[1] for _, p in pts]
            for i in range(n - 1):
                alpha = (i + 1) / n * 0.8
                ax.plot(xs[i:i+2], ys[i:i+2],
                        color='#1D9E75', alpha=alpha, lw=1.5)

        # 現在位置 + 速度ベクトル矢印
        if P is not None:
            ax.scatter(P[0], P[1], color='#1D9E75', s=70, zorder=5)
            if np.linalg.norm(vel[:2]) > 0.1:
                ax.annotate('',
                    xy=(P[0] + vel[0], P[1] + vel[1]),
                    xytext=(P[0], P[1]),
                    arrowprops=dict(arrowstyle='->', color='#E24B4A', lw=2.5),
                    zorder=6)

        ax.set_xlim(-15, 15)
        ax.set_ylim(-15, 15)
        ax.set_aspect('equal')
        ax.set_title('Top view (XY)', fontsize=10)
        ax.set_xlabel('X [m]', fontsize=8)
        ax.set_ylabel('Y [m]', fontsize=8)
        ax.grid(True, alpha=0.2)

    # ── 数値パネル ───────────────────────────────────────────────
    def _draw_metrics(self, speed_h, speed_3d, vz, alt):
        ax = self.ax_met
        ax.cla()
        ax.axis('off')

        rows = [
            ('Speed (H)',  f'{speed_h:.1f} m/s'),
            ('Speed (3D)', f'{speed_3d:.1f} m/s'),
            ('Climb rate', f'{vz:+.1f} m/s'),
            ('Altitude',   f'{alt:.2f} m'),
        ]
        for i, (label, val) in enumerate(rows):
            y = 0.88 - i * 0.24
            ax.text(0.05, y, label, transform=ax.transAxes,
                    fontsize=9,  color='#666666', va='top')
            ax.text(0.97, y, val, transform=ax.transAxes,
                    fontsize=13, fontweight='bold', ha='right', va='top',
                    color='#111111')
            if i < len(rows) - 1:
                ax.plot([0.0, 1.0], [y - 0.19, y - 0.19],
                        color='#cccccc', lw=0.5,
                        transform=ax.transAxes, clip_on=False)

        ax.set_title('Velocity & Altitude', fontsize=10)

    # ── 姿勢指示器（ADI） ────────────────────────────────────────
    # 修正箇所: imu_available を使ってセンサ未接続時はグレーアウト等にすることも可能です
    def _draw_adi(self, roll_deg, pitch_deg, imu_available=True):
        ax = self.ax_adi
        ax.cla()
        ax.set_xlim(-1.3, 1.3)
        ax.set_ylim(-1.3, 1.3)
        ax.set_aspect('equal')
        ax.axis('off')

        roll_rad    = np.radians(roll_deg)
        pitch_frac  = np.clip(pitch_deg / 60.0, -0.95, 0.95)
        horizon_y   = -pitch_frac   # ピッチ上昇 → 地平線が下がる

        cos_r, sin_r = np.cos(roll_rad), np.sin(roll_rad)
        rot = np.array([[cos_r, -sin_r], [sin_r, cos_r]])

        clip_c = Circle((0, 0), 1.0, transform=ax.transData)

        # 空（青） - センサ未接続時は色をくすませる
        sky_color = '#5B9BD5' if imu_available else '#8AA6C1'
        gnd_color = '#8B5E0A' if imu_available else '#7A6B53'
        
        ax.add_patch(Circle((0, 0), 1.0, fc=sky_color, ec='none', zorder=1))

        # 地面（茶）
        gnd_pts = np.array([[-2, -2], [2, -2], [2, horizon_y], [-2, horizon_y]])
        gnd_rot = (rot @ gnd_pts.T).T
        gnd = Polygon(gnd_rot, fc=gnd_color, ec='none', zorder=2)
        gnd.set_clip_path(clip_c)
        ax.add_patch(gnd)

        # 地平線
        h_raw = np.array([[-2, 2], [horizon_y, horizon_y]])
        h_rot = (rot @ h_raw).T
        hl, = ax.plot(h_rot[:, 0], h_rot[:, 1], 'white', lw=2, zorder=3)
        hl.set_clip_path(clip_c)

        # ピッチ目盛り
        for pm in [-20, -10, 10, 20]:
            py = horizon_y + pm / 60.0
            if abs(py) > 1.1:
                continue
            w = 0.25 if abs(pm) == 10 else 0.42
            m_raw = np.array([[-w, w], [py, py]])
            m_rot = (rot @ m_raw).T
            ml, = ax.plot(m_rot[:, 0], m_rot[:, 1],
                          'white', lw=0.9, alpha=0.75, zorder=4)
            ml.set_clip_path(clip_c)

        # ロール弧 + ポインタ
        arc_t = np.linspace(np.radians(30), np.radians(150), 80)
        ax.plot(np.cos(arc_t) * 0.88, np.sin(arc_t) * 0.88,
                color='white', lw=0.8, alpha=0.55, zorder=5)
        tri_a = np.pi / 2 - roll_rad
        ax.plot([np.cos(tri_a) * 0.80, np.cos(tri_a) * 0.93],
                [np.sin(tri_a) * 0.80, np.sin(tri_a) * 0.93],
                color='white', lw=2, zorder=6)

        # 機体シンボル（固定）
        ax.plot([-0.46, -0.13], [0, 0], color='#F5C400', lw=3.5, zorder=8)
        ax.plot([0.13,  0.46],  [0, 0], color='#F5C400', lw=3.5, zorder=8)
        ax.plot([0, 0], [-0.08, 0.08],  color='#F5C400', lw=2.5, zorder=8)
        ax.scatter(0, 0, color='#F5C400', s=18, zorder=9)

        # 外枠
        ax.add_patch(Circle((0, 0), 1.0, fill=False, ec='#555555',
                             lw=2.5, zorder=10))

        title_text = f'Attitude   R: {roll_deg:+.0f}°   P: {pitch_deg:+.0f}°'
        if not imu_available:
            title_text += ' (NO SENSOR)'
            
        ax.set_title(title_text, fontsize=9)

    def close(self):
        plt.close(self.fig)