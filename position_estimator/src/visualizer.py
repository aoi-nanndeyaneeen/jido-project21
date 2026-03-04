import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class Visualizer3D:
    def __init__(self, field_points):
        self.field_points = field_points
        self.xs = np.append(field_points[:,0], field_points[0,0])
        self.ys = np.append(field_points[:,1], field_points[0,1])
        self.zs = np.append(field_points[:,2], field_points[0,2])

        # 3D表示の固定範囲
        self.limits = {
            'X': (-2.0, 18.0),
            'Y': (-2.0, 18.0),
            'Z': ( 0.0, 15.0)
        }

        # グラフの初期化
        plt.ion()
        self.fig = plt.figure(figsize=(9, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.show(block=False)

    def _draw_plane_on_ax(self, pos, roll, pitch, scale=0.5):
        """機体の3Dモデル描画（内部メソッド）"""
        body = np.array([[-1,0,0],[1,0,0]]) * scale
        wing = np.array([[0,-1.2,0],[0,1.2,0]]) * scale
        tail = np.array([[-0.8,-0.4,0],[-0.8,0.4,0]]) * scale
        vert = np.array([[-0.8,0,0],[-0.8,0,0.4]]) * scale
        
        r_, p_ = np.radians(roll), np.radians(pitch)
        Rx = np.array([[1,0,0],[0,np.cos(r_),-np.sin(r_)],[0,np.sin(r_),np.cos(r_)]])
        Ry = np.array([[np.cos(p_),0,np.sin(p_)],[0,1,0],[-np.sin(p_),0,np.cos(p_)]])
        
        for part, color in [(body,'blue'),(wing,'blue'),(tail,'red'),(vert,'red')]:
            pts = ((Ry @ Rx) @ part.T).T + pos
            self.ax.plot(pts[:,0], pts[:,1], pts[:,2], color=color, linewidth=3)

    def update(self, P, O, roll, pitch, current_z):
        """whileループ内で毎フレーム呼び出す更新メソッド"""
        self.ax.cla() # 画面クリア
        self.ax.set_xlim(*self.limits['X'])
        self.ax.set_ylim(*self.limits['Y'])
        self.ax.set_zlim(*self.limits['Z'])
        self.ax.set_box_aspect((
            self.limits['X'][1] - self.limits['X'][0],
            self.limits['Y'][1] - self.limits['Y'][0],
            self.limits['Z'][1] - self.limits['Z'][0]
        ))

        # フィールドとカメラ位置の描画
        self.ax.plot(self.xs, self.ys, self.zs, color='gray', linewidth=2, label="Field 16x16m")
        self.ax.scatter(*O, color='red', s=80, zorder=5, label="Camera")

        # 機体の描画
        if P is not None:
            self._draw_plane_on_ax(P, roll, pitch, scale=0.5)
            self.ax.plot([O[0],P[0]], [O[1],P[1]], [O[2],P[2]],
                         color='orange', linestyle='--', linewidth=1, alpha=0.6)
            self.ax.scatter(*P, color='green', s=80, zorder=5,
                            label=f"({P[0]:.1f},{P[1]:.1f},{current_z:.1f}m)")

        self.ax.set_xlabel('X(m)')
        self.ax.set_ylabel('Y(m)')
        self.ax.set_zlabel('Z(m)')
        self.ax.set_title(f"Roll:{roll:+.1f}deg  Pitch:{pitch:+.1f}deg  Alt:{current_z:.2f}m")
        self.ax.legend(fontsize=7, loc='upper right')

        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception:
            pass
        plt.pause(0.05)

    def close(self):
        plt.close('all')