"""
remote_camera.py
RPi上のcamera_server.pyからソケット経由で
座標とプレビューフレームを受け取るクライアントクラス。
CameraTrackerと同一インターフェースを持つ。
"""
import socket, json, base64, cv2
import numpy as np


class RemoteCamera:
    def __init__(self, host: str, port: int = 5555, label: str = "RemoteCamera"):
        self.host  = host
        self.port  = port
        self.label = label
        self.sock  = None
        self.buf   = ""
        self.width  = None
        self.height = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(1.0)

        # 最初の1行 = 解像度情報
        info = self._readline()
        d = json.loads(info)
        self.width, self.height = d["width"], d["height"]
        print(f"[{self.label}] 接続完了: {self.width}x{self.height}")

    def _readline(self) -> str:
        while "\n" not in self.buf:
            chunk = self.sock.recv(65536).decode()
            if not chunk:
                raise ConnectionError("Remote camera disconnected")
            self.buf += chunk
        line, self.buf = self.buf.split("\n", 1)
        return line

    def read_and_track(self):
        """
        CameraTrackerと同一インターフェース。
        Returns:
            frame (np.ndarray or None): プレビューフレーム（SEND_PREVIEW=Trueの場合）
            center_uv (tuple or None): 検出座標
        """
        try:
            line = self._readline()
            d = json.loads(line)

            pt = d.get("pt")
            center_uv = (pt[0], pt[1]) if pt else None

            frame = None
            if "frame" in d:
                img_bytes = base64.b64decode(d["frame"])
                arr = np.frombuffer(img_bytes, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None and center_uv:
                    cx, cy = int(center_uv[0]), int(center_uv[1])
                    # プレビューはスケールダウン済みなので元座標で描画するとズレる
                    # → フル解像度座標をスケール変換して描画
                    scale_x = frame.shape[1] / self.width
                    scale_y = frame.shape[0] / self.height
                    dx = int(cx * scale_x)
                    dy = int(cy * scale_y)
                    cv2.circle(frame, (dx, dy), 8, (0, 255, 0), 2)
                    cv2.rectangle(frame, (dx-10, dy-10), (dx+10, dy+10), (0,255,0), 1)
                    cv2.putText(frame, f"({cx},{cy})",
                                (dx+12, dy-12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                if frame is not None:
                    cv2.putText(frame, self.label, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,200,0), 2)
            return frame, center_uv

        except (socket.timeout, json.JSONDecodeError):
            return None, None

    def get_approx_camera_matrix(self):
        """CameraTrackerと同一インターフェース"""
        focal = self.width
        return np.array([[focal, 0,     self.width/2],
                         [0,     focal, self.height/2],
                         [0,     0,     1           ]], dtype=np.float32)

    def reset_background(self):
        """RPi側ではサーバー内で管理のためno-op"""
        pass

    def release(self):
        if self.sock:
            self.sock.close()