import cv2
from core.geometry import approx_camera_matrix
from utils.config import DIFF_THRESHOLD, MIN_AREA_PX, BLUR_KERNEL, MORPH_KERNEL, CAMERA_FPS


class CameraTracker:
    def __init__(self, camera_url, width=1280, height=720, label="Camera"):
        self.label = label
        self.camera_url = camera_url

        if isinstance(camera_url, int):
            self.cap = cv2.VideoCapture(camera_url, cv2.CAP_DSHOW)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            self.cap = cv2.VideoCapture(camera_url)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"[{label}] 初期化完了: 要求 {width}x{height}@{CAMERA_FPS}fps "
              f"-> 実際 {int(actual_w)}x{int(actual_h)}@{actual_fps:.1f}fps")

        self.width  = int(actual_w) if actual_w > 0 else width
        self.height = int(actual_h) if actual_h > 0 else height
        self.prev_gray = None

        # ── 差分パラメータ（config.py で調整可能） ──────────
        self.blur_size      = (BLUR_KERNEL, BLUR_KERNEL)
        self.diff_threshold = DIFF_THRESHOLD
        self.min_area       = MIN_AREA_PX
        self._morph_kernel  = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (MORPH_KERNEL, MORPH_KERNEL)
        )

    def get_approx_camera_matrix(self):
        return approx_camera_matrix(self.width, self.height)

    def read_and_track(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None, None

        gray         = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, self.blur_size, 0)

        center_uv = None

        if self.prev_gray is None:
            self.prev_gray = gray_blurred
        else:
            diff = cv2.absdiff(self.prev_gray, gray_blurred)
            _, thresh = cv2.threshold(
                diff, self.diff_threshold, 255, cv2.THRESH_BINARY
            )
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

            self.prev_gray = gray_blurred

        cv2.putText(frame, self.label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 0), 2)

        return frame, center_uv

    def reset_background(self):
        self.prev_gray = None

    def release(self):
        self.cap.release()