"""
init/camera_setup.py
Camera1 (ローカルUSB) と Camera2 (RPi) の初期化・疎通確認。

Camera2 (RPi) の本接続はキャリブレーション完了後に calibration_flow.py 側で
行うため、ここではダミースタブを返す。ここでやるのは「RPi が listen して
いるか」の確認だけ。

==========================================================================
★ 2026-09-17: 「1回目はラズパイの入力が来ない。main.py を終了してやり直すと動く」
==========================================================================
原因はここにあった。従来の実装は

    3秒のタイムアウトで1回だけ connect を試す
      -> 失敗したら cam2_ok_rpi = False
      -> calibration_flow._calibrate_camera() が即 dummy() を返す
      -> Camera2 が **ダミーカメラのまま固定** され、以後どれだけ待っても回復しない

という作りだった。本番の手順は「2人が同時に camera_server.py と main.py を
起動する」なので、RPi のカメラ初期化 (cap.read の立ち上がり + ウィンドウ生成)
が終わる前に PC 側の1回きりの probe が走り、ほぼ必ず外れる。
操縦者が見ていたのは「ダミーカメラで三角測量できず、位置が出ない」状態で、
main.py を再起動すると RPi が立ち上がり済みなので通っていた。

対策は 2 つ:
  1. **待つ**。1回で諦めず RPI_WAIT_S 秒まで繰り返し試し、経過を1行で見せる。
  2. **黙ってダミーに落とさない**。時間切れでも、続けるか待ち直すかを聞く
     (main.py --no-wait-rpi のときは聞かずに続行。Camera1 だけの確認用)。
"""

import socket
import sys
import time

from core.camera import CameraTracker
from core.geometry import approx_camera_matrix
from utils.config import (CAMERA_1_URL, CAMERA_2_URL, CAMERA2_SOURCE,
                          CAMERA_W, CAMERA_H, RPI_HOST, RPI_PORT,
                          RPI_WAIT_S, FALLBACK_HFOV_DEFAULT)


class _DummyCam2:
    """Camera2本接続前のプレースホルダー。tracker.pyから同じインターフェースで呼べる。"""
    width, height = 1280, 720
    label = "Camera2"
    last_frame_time = 0.0
    vibration_rejected = False

    def read_and_detect(self):
        return None, [], 0.0

    def draw_candidates(self, frame, candidates, best_index=None):
        pass

    def reset_background(self):
        pass

    def release(self):
        pass

    def get_intrinsics(self):
        return approx_camera_matrix(self.width, self.height,
                                    FALLBACK_HFOV_DEFAULT), None

    def get_approx_camera_matrix(self):
        return self.get_intrinsics()[0]


def probe_rpi(timeout_s=1.0) -> bool:
    """RPi の camera_server.py が listen しているかを1回だけ見る。

    ★ 接続して解像度の1行を受け取ったら即切る。サーバー側はモード行を待たずに
      切られた接続を「モード受信前に切断 (無視)」として捨てて accept に戻るので、
      何度叩いても状態は汚れない。
    """
    sk = None
    try:
        sk = socket.socket()
        sk.settimeout(timeout_s)
        sk.connect((RPI_HOST, RPI_PORT))
        return bool(sk.recv(256))
    except Exception:
        return False
    finally:
        if sk is not None:
            try:
                sk.close()
            except OSError:
                pass


def wait_for_rpi(wait_s=None, interactive=True) -> bool:
    """RPi が立ち上がるまで待つ。立ち上がったら True。

    本番は 1 分の準備時間のうちに 2 人が同時に起動するので、どちらが先でも
    よいように **PC 側が待つ**。経過は1行を上書きして出す (流れると読めない)。
    """
    wait_s = RPI_WAIT_S if wait_s is None else wait_s
    t0 = time.time()
    if probe_rpi():
        print(f"  [OK]  Camera2 (RPi {RPI_HOST}:{RPI_PORT}) 到達確認")
        return True

    print(f"  [WAIT] Camera2 (RPi {RPI_HOST}:{RPI_PORT}) を待っています "
          f"(最大 {wait_s:.0f} 秒)。ラズパイで camera_server.py を起動してください")
    while time.time() - t0 < wait_s:
        if probe_rpi():
            print(f"\r  [OK]  Camera2 (RPi) 到達確認 ({time.time() - t0:.0f} 秒待ちました)"
                  + " " * 20)
            return True
        left = wait_s - (time.time() - t0)
        sys.stdout.write(f"\r         ... 残り {left:4.0f} 秒 "
                         f"(Ctrl+C で中止)   ")
        sys.stdout.flush()
        time.sleep(0.5)
    print(f"\r  [NG]  Camera2 (RPi) に {wait_s:.0f} 秒待っても繋がりませんでした。"
          + " " * 10)
    print("        確認: ラズパイで camera_server.py が動いているか / "
          f"IP が {RPI_HOST} か (config.py RPI_HOST) / 同じネットワークか")
    if not interactive:
        return False
    # ★ ここで黙ってダミーに落とすと、飛ばす直前まで気づけない。必ず聞く。
    #   ただし入力が閉じている (自動化スクリプト / パイプ) ときは聞き返さずに
    #   続行する。無限ループになるとカメラと COM を掴んだままプロセスが残る。
    for _ in range(20):
        try:
            ans = input("        [r] もう一度待つ / [d] Camera2 なしで続行 "
                        "(三角測量できません) > ").strip().lower()
        except (EOFError, OSError):
            print("        (入力なし) Camera2 なしで続行します")
            return False
        if ans in ("r", ""):
            return wait_for_rpi(wait_s, interactive)
        if ans == "d":
            print("        Camera2 なしで続行します (位置推定はできません)")
            return False
        print("        r か d を入力してください")
    return False


def init_cameras(interactive=True):
    """
    Returns:
        cam1        : CameraTracker (ローカルUSBカメラ)
        cam2_ok_rpi : bool - Camera2 が使えるか
        cam2        : _DummyCam2 スタブ (本接続はキャリブ後に差し替え)
    """
    print("\n[INIT 1/3]  カメラ起動中...")
    cam1 = CameraTracker(CAMERA_1_URL, width=CAMERA_W, height=CAMERA_H, label="Camera1")

    ret, test_frame = cam1.cap.read()
    if not ret or test_frame is None:
        print("  [WARN] Camera1 映像取得失敗。ダミーモードで続行します。")
    else:
        print(f"  [OK]  Camera1 {cam1.width}x{cam1.height}")

    if CAMERA2_SOURCE == "USB":
        print(f"  [INFO] Camera2 (USB): デバイス番号 {CAMERA_2_URL}")
        cam2 = CameraTracker(CAMERA_2_URL, width=CAMERA_W, height=CAMERA_H,
                             label="Camera2")
        ret, test_frame = cam2.cap.read()
        cam2_ok = bool(ret and test_frame is not None)
        if not cam2_ok:
            print("  [WARN] Camera2 USB映像取得失敗。ダミーモードで続行します。")
        return cam1, cam2_ok, cam2

    cam2_ok = wait_for_rpi(interactive=interactive)
    return cam1, cam2_ok, _DummyCam2()
