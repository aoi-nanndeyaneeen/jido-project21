import matplotlib
matplotlib.use('Agg') # ウィンドウを出さず裏で画像生成する設定
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cv2

class View3D:
    def __init__(self, field_points):
        self.limits = {'X': (-5.0, 20.0), 'Y': (-5.0, 20.0), 'Z': (0.0, 15.0)}
        self.fig = plt.figure(figsize=(8, 8), dpi=100) # 800x800 px
        self.ax = self.fig.add_subplot(111, projection='3d')
        # フィールド枠の準備
        self.xs = np.append(field_points[:,0], field_points[0,0])
        self.ys = np.append(field_points[:,1], field_points[0,1])
        self.zs = np.append(field_points[:,2], field_points[0,2])

    def get_image(self, P, O, roll, pitch, current_z):
        self.ax.cla()
        self.ax.set_xlim(*self.limits['X'])
        self.ax.set_ylim(*self.limits['Y'])
        self.ax.set_zlim(*self.limits['Z'])
        
        # 描画
        self.ax.plot(self.xs, self.ys, self.zs, color='gray', label="Field")
        self.ax.scatter(*O, color='red', s=80, label="Camera")
        if P is not None:
            self.ax.scatter(*P, color='green', s=100, label=f"Plane ({current_z:.1f}m)")
            self.ax.plot([O[0],P[0]], [O[1],P[1]], [O[2],P[2]], color='orange', linestyle='--')
            
        self.ax.set_title(f"3D View - Roll:{roll:+.1f} Pitch:{pitch:+.1f}")
        self.ax.legend(loc='upper right')

        # Matplotlibの図をOpenCVの画像(Numpy配列)に変換
        self.fig.canvas.draw()
        img = np.asarray(self.fig.canvas.buffer_rgba())
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

    def close(self):
        plt.close(self.fig)