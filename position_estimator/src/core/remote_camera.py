import socket, json, base64, cv2
import numpy as np
from core.geometry import approx_camera_matrix


class RemoteCamera:
    def __init__(self, host: str, port: int = 5555, label: str = "RemoteCamera"):
        self.host  = host
        self.port  = port
        self.label = label
        self.sock  = None
        self.buf   = ""
        self.width  = None
        self.height = None

    def connect(self, mode: str = "STREAM"):
        """
        サーバーに接続してモードを通知する。
        mode: "STREAM"（通常）または "CALIB"（キャリブレーション）
        """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(2.0)

        # 解像度情報を受信
        info = self._readline()
        d = json.loads(info)
        self.width, self.height = d["width"], d["height"]

        # ★ モードをサーバーに送信（必須）
        self.sock.sendall(f"{mode}\n".encode())
        self.sock.settimeout(1.0)   # 通常通信用に戻す
        print(f"[{self.label}] 接続完了: {self.width}x{self.height} (mode={mode})")

    # ── キャリブレーション専用クラスメソッド ────────────────────
    @classmethod
    def request_calibration(cls, host: str, port: int,
                            timeout: float = 120.0) -> tuple[list, int, int]:
        """
        RPiにCALIBモードで接続し、ユーザーがRPi画面で
        クリックした5点の座標を受け取る。

        Returns:
            pts    : [[x,y], ...] 5点のリスト（フル解像度座標）
            width  : カメラ幅
            height : カメラ高さ
        """
        print(f"[RemoteCamera] キャリブレーション接続中 ({host}:{port})...")
        sock = socket.socket()
        sock.connect((host, port))
        sock.settimeout(timeout)

        buf = ""
        # 解像度受信
        while "\n" not in buf:
            buf += sock.recv(1024).decode()
        line, buf = buf.split("\n", 1)
        info = json.loads(line)
        w, h = info["width"], info["height"]

        # CALIBモードを要求
        sock.sendall(b"CALIB\n")
        print("[RemoteCamera] ラズパイのディスプレイで5点をクリックしてください...")

        # 5点データを待つ（タイムアウトまで待機）
        while "\n" not in buf:
            chunk = sock.recv(4096).decode()
            if not chunk:
                raise ConnectionError("RPiが切断されました")
            buf += chunk
        line, _ = buf.split("\n", 1)
        data = json.loads(line)
        sock.close()

        pts = data["calib_pts"]
        print(f"[RemoteCamera] キャリブ点受信: {pts}")
        return pts, w, h

    def _readline(self) -> str:
        while "\n" not in self.buf:
            chunk = self.sock.recv(65536).decode()
            if not chunk:
                raise ConnectionError("Remote camera disconnected")
            self.buf += chunk
        line, self.buf = self.buf.split("\n", 1)
        return line

    def read_and_track(self):
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
                    scale_x = frame.shape[1] / self.width
                    scale_y = frame.shape[0] / self.height
                    dx = int(center_uv[0] * scale_x)
                    dy = int(center_uv[1] * scale_y)
                    cv2.circle(frame, (dx, dy), 8, (0, 255, 0), 2)
                    cv2.putText(frame, f"({int(center_uv[0])},{int(center_uv[1])})",
                                (dx + 12, dy - 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                if frame is not None:
                    cv2.putText(frame, self.label, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 0), 2)
            return frame, center_uv

        except (socket.timeout, json.JSONDecodeError):
            return None, None

    def get_approx_camera_matrix(self):
        return approx_camera_matrix(self.width, self.height)

    def reset_background(self):
        pass

    def release(self):
        if self.sock:
            self.sock.close()