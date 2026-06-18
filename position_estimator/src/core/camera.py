# カメラ制御
# ネットワークストリーム (MJPEG over HTTP) およびローカルUSBカメラに対応

import cv2
import numpy as np


class CameraTracker:
    def __init__(self, camera_url, width=1280, height=720, label="Camera"):
        self.label = label
        self.camera_url = camera_url

        if isinstance(camera_url, int):
            self.cap = cv2.VideoCapture(camera_url, cv2.CAP_DSHOW)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            self.cap = cv2.VideoCapture(camera_url)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"[{label}] 初期化完了: 要求 {width}x{height} -> 実際 {int(actual_w)} x {int(actual_h)}")

        self.width  = int(actual_w) if actual_w > 0 else width
        self.height = int(actual_h) if actual_h > 0 else height
        self.prev_gray = None

        # ── 差分パラメータ（camera_server.py と統一） ──────────
        self.blur_size      = (5, 5)   # GaussianBlur カーネル
        self.diff_threshold = 25       # 差分2値化の閾値
        self.min_area       = 100      # 最小輪郭面積 [px^2]
        self._morph_kernel  = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5)
        )

    def get_approx_camera_matrix(self):
        focal_length = self.width
        cx = self.width  / 2.0
        cy = self.height / 2.0
        return np.array([[focal_length, 0,            cx],
                         [0,            focal_length, cy],
                         [0,            0,             1]], dtype=np.float32)

    def read_and_track(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None, None

        gray         = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, self.blur_size, 0)

        center_uv = None

        if self.prev_gray is None:
            # 初回フレーム：背景として保存
            self.prev_gray = gray_blurred
        else:
            # ── フレーム差分（camera_server.py と同じ方式） ────
            diff = cv2.absdiff(self.prev_gray, gray_blurred)
            _, thresh = cv2.threshold(
                diff, self.diff_threshold, 255, cv2.THRESH_BINARY
            )
            # MORPH_OPEN = 収縮→膨張（小ノイズ除去、大ブロブ保持）
            thresh = cv2.morphologyEx(
                thresh, cv2.MORPH_OPEN, self._morph_kernel
            )

            contours, _ = cv2.findContours(
                thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > self.min_area:
                    M = cv2.moments(largest)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        center_uv = (cx, cy)
                        x, y, w, h = cv2.boundingRect(largest)
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.circle(frame, center_uv, 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"({cx},{cy})", (cx+10, cy-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # 前フレームを更新（単純入れ替え。加重平均なし）
            self.prev_gray = gray_blurred

        cv2.putText(frame, self.label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 0), 2)

        return frame, center_uv

    def reset_background(self):
        self.prev_gray = None

    def release(self):
        self.cap.release()