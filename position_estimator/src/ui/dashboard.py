import cv2
import numpy as np
from ui.view_3d import View3D
from ui.view_graph import ViewGraph
from ui.view_camera import ViewCamera

class Dashboard:
    def __init__(self, field_points):
        self.view_3d = View3D(field_points)
        self.view_graph = ViewGraph()
        self.view_camera = ViewCamera()

    def render_and_show(self, P, O, roll, pitch, current_z, target_alt, frame):
        # 1. 各モジュールから画像(Numpy配列)を取得
        img_3d = self.view_3d.get_image(P, O, roll, pitch, current_z) # 800x800
        img_graph = self.view_graph.get_image(current_z, target_alt)  # 600x400
        img_cam = self.view_camera.get_image(frame)                   # 600x400

        # 2. 右側のグラフとカメラを縦に結合 (600x800 になる)
        img_right = np.vstack((img_graph, img_cam))

        # 3. 左の3D(800x800)と右の列(600x800)を横に結合 (1400x800 のダッシュボード完成！)
        dashboard_img = np.hstack((img_3d, img_right))

        # 4. 表示
        cv2.imshow("Dashboard", dashboard_img)

    def close(self):
        self.view_3d.close()
        self.view_graph.close()
        cv2.destroyWindow("Dashboard")