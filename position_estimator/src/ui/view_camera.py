import cv2
import numpy as np

class ViewCamera:
    def __init__(self):
        self.target_size = (600, 400) # グラフと同じサイズにリサイズ

    def get_image(self, frame):
        if frame is None:
            img = np.zeros((self.target_size[1], self.target_size[0], 3), dtype=np.uint8)
            cv2.putText(img, "No Camera Signal", (150, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            return img
        # リサイズして返す
        return cv2.resize(frame, self.target_size)