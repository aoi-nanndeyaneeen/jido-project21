# test_view_rc.py
import cv2
import numpy as np
from core.autopilot import RCCommand
from ui.view_rc import ViewRC

view = ViewRC()
cmd = RCCommand(throttle=0.6, pitch=0.3, roll=-0.2, yaw=0.1)

img = view.get_image(cmd)
print(f"画像shape: {img.shape}, dtype: {img.dtype}, 平均輝度: {img.mean()}")

cv2.namedWindow("Test RC", cv2.WINDOW_NORMAL)
cv2.imshow("Test RC", img)
cv2.waitKey(0)
cv2.destroyAllWindows()