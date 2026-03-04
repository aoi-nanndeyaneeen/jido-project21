# カメラ制御

import cv2
import numpy as np

class CameraTracker:
    def __init__(self, camera_id=0, width=1920, height=1080):
        """カメラの初期化（1080p高画質設定）"""
        self.cap = cv2.VideoCapture(camera_id)
        
        # 1080pとMJPGフォーマットを要求（USB3.0の帯域を活かして高画質・高FPSを出すため）
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # 実際の解像度を確認
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"[Camera] 初期化完了: 要求 {width}x{height} -> 実際 {actual_w} x {actual_h}")

        self.width = int(actual_w)
        self.height = int(actual_h)
        self.prev_gray = None
        
        # フレーム差分用のパラメータ
        self.blur_size = (15, 15)
        self.diff_threshold = 15
        self.min_area = 50

    def get_approx_camera_matrix(self):
        """画角の中央を基準とした簡易カメラマトリクスを取得"""
        focal_length = self.width
        return np.array([[focal_length, 0, self.width / 2],
                         [0, focal_length, self.height / 2],
                         [0, 0, 1]], dtype=np.float32)

    def read_and_track(self):
        """フレームを読み込み、動体を検知してピクセル座標(u,v)を返す"""
        ret, frame = self.cap.read()
        if not ret:
            return None, None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, self.blur_size, 0)
        
        center_uv = None
        
        # 背景（prev_gray）が空の場合、float32型に変換して初期化する（エラー対策）
        if self.prev_gray is None:
            self.prev_gray = np.float32(gray_blurred)
        else:
            # 差分計算のために、float32型の背景をuint8型（通常の画像形式）に戻す
            bg_uint8 = cv2.convertScaleAbs(self.prev_gray)
            
            # 差分を計算
            diff = cv2.absdiff(bg_uint8, gray_blurred)
            _, thresh = cv2.threshold(diff, self.diff_threshold, 255, cv2.THRESH_BINARY)
            thresh = cv2.dilate(thresh, None, iterations=2)
            
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > self.min_area:
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        center_uv = (cx, cy)
                        # 描画用
                        x, y, w, h = cv2.boundingRect(largest_contour)
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.circle(frame, center_uv, 5, (0, 0, 255), -1)

            # 背景の更新（現在のフレームを0.5の割合で背景にブレンド）
            cv2.accumulateWeighted(gray_blurred, self.prev_gray, 0.5)

        return frame, center_uv

    def release(self):
        self.cap.release()