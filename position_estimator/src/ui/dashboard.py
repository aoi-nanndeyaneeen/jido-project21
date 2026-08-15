import cv2
import numpy as np
from ui.view_graph import ViewGraph
from utils.config import DISP_W, DISP_H, VELOCITY_W, VELOCITY_H


class Dashboard:
    def __init__(self, field_points):
        self.view_graph = ViewGraph()
        cv2.namedWindow("Graph", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Graph", 600, 600)
        # Velocityウィンドウの右隣、カメラウィンドウの下段に配置
        cv2.moveWindow("Graph", VELOCITY_W + 20, DISP_H + 40)
        # 初回imshowで即座に応答可能にする
        cv2.imshow("Graph", np.zeros((600, 600, 3), dtype=np.uint8))

    def render_and_show(self, P, current_z, target_alt):
        img = self.view_graph.get_image(P, current_z, target_alt)
        cv2.imshow("Graph", img)

    def close(self):
        self.view_graph.close()
        cv2.destroyWindow("Graph")