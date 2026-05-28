# カメラ制御
# ネットワークストリーム (MJPEG over HTTP) およびローカルUSBカメラに対応

import cv2
import numpy as np


class CameraTracker:
    def __init__(self, camera_url, width=1280, height=720, label="Camera"):
        """
        カメラの初期化。

        Args:
            camera_url: カメラのURL (str: "http://..." など) またはID (int: 0, 1, ...)
            width (int): 要求解像度 (幅)
            height (int): 要求解像度 (高さ)
            label (str): ログ表示用ラベル (例: "Camera1", "Camera2")
        """
        self.label = label
        self.camera_url = camera_url

        # ローカルカメラの場合は CAP_DSHOW を使用 (Windows での高速化)
        if isinstance(camera_url, int):
            self.cap = cv2.VideoCapture(camera_url, cv2.CAP_DSHOW)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            # バッファを最小にしてフレーム遅延を低減
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            # ネットワークストリーム (MJPEG etc.)
            self.cap = cv2.VideoCapture(camera_url)
            # バッファを最小にして最新フレームを確実に取得
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 実際の解像度を確認
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"[{label}] 初期化完了: 要求 {width}x{height} -> 実際 {int(actual_w)} x {int(actual_h)}")

        self.width  = int(actual_w) if actual_w > 0 else width
        self.height = int(actual_h) if actual_h > 0 else height
        self.prev_gray = None

        # フレーム差分用パラメータ
        self.blur_size      = (15, 15)
        self.diff_threshold = 15
        self.min_area       = 50

    def get_approx_camera_matrix(self):
        """
        画角の中央を基準とした簡易カメラ行列を取得。
        焦点距離は画像幅をそのまま使用 (対角画角 ≈ 53° 相当)。
        カメラ固有のFOVが分かっている場合は focal_length を計算してください:
            focal_length = (width / 2) / math.tan(math.radians(hfov_deg / 2))
        """
        focal_length = self.width   # 近似値
        cx = self.width  / 2.0
        cy = self.height / 2.0
        return np.array([[focal_length, 0,            cx],
                         [0,            focal_length, cy],
                         [0,            0,             1]], dtype=np.float32)

    def read_and_track(self):
        """
        フレームを読み込み、フレーム差分法で動体を検知してピクセル座標(u,v)を返す。

        Returns:
            frame (np.ndarray or None): 描画済みのフレーム画像
            center_uv (tuple or None): 検知した動体の重心ピクセル座標 (u, v)
        """
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None, None

        gray          = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blurred  = cv2.GaussianBlur(gray, self.blur_size, 0)

        center_uv = None

        if self.prev_gray is None:
            # 初回フレーム: 背景として保存
            self.prev_gray = np.float32(gray_blurred)
        else:
            # 差分計算
            bg_uint8 = cv2.convertScaleAbs(self.prev_gray)
            diff     = cv2.absdiff(bg_uint8, gray_blurred)
            _, thresh = cv2.threshold(diff, self.diff_threshold, 255, cv2.THRESH_BINARY)
            thresh    = cv2.dilate(thresh, None, iterations=2)

            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > self.min_area:
                    M = cv2.moments(largest)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        center_uv = (cx, cy)
                        # 描画
                        x, y, w, h = cv2.boundingRect(largest)
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.circle(frame, center_uv, 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"({cx},{cy})", (cx+10, cy-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # 背景を現在フレームで更新（混合率0.5）
            cv2.accumulateWeighted(gray_blurred, self.prev_gray, 0.5)

        # カメララベルを表示
        cv2.putText(frame, self.label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 0), 2)

        return frame, center_uv

    def reset_background(self):
        """背景をリセットし、次フレームから再学習する"""
        self.prev_gray = None

    def release(self):
        """カメラリソースを解放する"""
        self.cap.release()