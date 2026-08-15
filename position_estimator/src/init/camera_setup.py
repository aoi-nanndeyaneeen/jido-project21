"""
init/camera_setup.py
Camera1（ローカルUSB）・Camera2（RPi）の初期化と疎通確認を担当。
Camera2の本接続はキャリブレーション完了後に calibration_flow.py 側で行うため、
ここではダミースタブを返す。
"""

import socket
from core.camera import CameraTracker
from core.geometry import approx_camera_matrix
from utils.config import CAMERA_1_URL, CAMERA_W, CAMERA_H, RPI_HOST, RPI_PORT


class _DummyCam2:
    """Camera2本接続前のプレースホルダー。tracker.pyから同じインターフェースで呼べる。"""
    width, height = 1280, 720

    def read_and_track(self):
        return None, None

    def reset_background(self):
        pass

    def release(self):
        pass

    def get_approx_camera_matrix(self):
        return approx_camera_matrix(self.width, self.height)


def init_cameras():
    """
    Returns:
        cam1        : CameraTracker（ローカルUSBカメラ）
        cam2_ok_rpi : bool - RPiに疎通できたか
        cam2        : _DummyCam2 スタブ（本接続はキャリブ後に差し替え）
    """
    print("\n[INIT 1/4]  カメラ起動中...")
    cam1 = CameraTracker(CAMERA_1_URL, width=CAMERA_W, height=CAMERA_H, label="Camera1")

    ret, test_frame = cam1.cap.read()
    if not ret or test_frame is None:
        print("  [WARN] Camera1 映像取得失敗。ダミーモードで続行します。")

    cam2_ok_rpi = True
    print("  [INFO] Camera2 (RPi): キャリブレーション時に接続確認します")
    try:
        test_sock = socket.socket()
        test_sock.settimeout(3.0)
        test_sock.connect((RPI_HOST, RPI_PORT))
        test_sock.recv(256)
        test_sock.close()
        print("  [OK]  Camera2 (RPi) 到達確認")
    except Exception as e:
        cam2_ok_rpi = False
        print(f"  [WARN] Camera2 到達失敗: {e}")
        print("         ダミーモードで続行します。")

    cam2 = _DummyCam2()
    return cam1, cam2_ok_rpi, cam2