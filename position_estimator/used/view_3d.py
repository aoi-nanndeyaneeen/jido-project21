import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cv2

class View3D:
    def __init__(self, field_points):
        self.limits = {'X': (-10.0, 10.0), 'Y': (-10.0, 10.0), 'Z': (0.0, 15.0)}
        
        # ★ 800x800 から 600x600 に縮小し、画面に収まりやすくします
        self.fig = plt.figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        ground_points = field_points[:4] 
        self.xs = np.append(ground_points[:,0], ground_points[0,0])
        self.ys = np.append(ground_points[:,1], ground_points[0,1])
        self.zs = np.append(ground_points[:,2], ground_points[0,2])

    def get_image(self, P, O, roll, pitch, current_z):
        self.ax.cla()
        self.ax.set_xlim(*self.limits['X'])
        self.ax.set_ylim(*self.limits['Y'])
        self.ax.set_zlim(*self.limits['Z'])
        
        self.ax.plot(self.xs, self.ys, self.zs, color='gray', label="Field")
        self.ax.scatter(*O, color='red', s=80, label="Camera")
        if P is not None:
            self.ax.scatter(*P, color='green', s=100, label=f"Plane ({current_z:.1f}m)")
            self.ax.plot([O[0],P[0]], [O[1],P[1]], [O[2],P[2]], color='orange', linestyle='--')
            
        self.ax.set_title(f"3D View - Roll:{roll:+.1f} Pitch:{pitch:+.1f}")
        self.ax.legend(loc='upper right', fontsize=8)

        self.fig.tight_layout()
        self.fig.canvas.draw()
        img = np.asarray(self.fig.canvas.buffer_rgba())
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

    def close(self):
        plt.close(self.fig)