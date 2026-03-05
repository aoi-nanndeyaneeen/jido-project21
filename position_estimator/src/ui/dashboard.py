import cv2
import numpy as np
from ui.view_3d import View3D
from ui.view_graph import ViewGraph

class Dashboard:
    def __init__(self, field_points):
        self.view_3d = View3D(field_points)
        self.view_graph = ViewGraph()
        
        # ★ cv2.namedWindow("Dashboard", cv2.WINDOW_NORMAL) は削除！
        # これにより、文字が一切潰れない「等倍の固定サイズ表示」になります。

    def render_and_show(self, P, O, roll, pitch, current_z, target_alt):
        # 1. 画像を取得 (それぞれ 600x600 になります)
        img_3d = self.view_3d.get_image(P, O, roll, pitch, current_z)
        img_graph = self.view_graph.get_image(P, current_z, target_alt)

        # 2. 横に結合 (1200x600 のダッシュボード完成)
        dashboard_img = np.hstack((img_3d, img_graph))

        # 3. 等倍で表示
        cv2.imshow("Dashboard", dashboard_img)

    def close(self):
        self.view_3d.close()
        self.view_graph.close()
        cv2.destroyWindow("Dashboard")