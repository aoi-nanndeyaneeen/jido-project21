"""
init/calibration_flow.py
Camera1・Camera2 のキャリブレーションフロー。
保存済みデータの再利用選択、新規キャリブレーション、
Camera2のCALIB/STREAMソケットハンドシェイクをまとめる。
"""

import cv2
import json
import socket
import time
import numpy as np

from utils.config import FIELD_POINTS, DISP_W, DISP_H, RPI_HOST, RPI_PORT
from core.remote_camera import RemoteCamera
from utils.calib_store import save_calibration, load_calibration, ask_use_saved


def run_calibration_single(cam, cam_label: str):
    """Camera1用: PC画面上で5点クリックしてsolvePnP"""
    points_2d = []
    window_name = f"Calibration: {cam_label}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, DISP_W, DISP_H)

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points_2d) < 5:
            points_2d.append([x, y])
            print(f"  [{cam_label}] 点{len(points_2d)}/5 クリック: ({x}, {y})")

    cv2.setMouseCallback(window_name, on_click)
    print(f"\n  【{cam_label}】 基準点5箇所をクリックしてください")
    print("       順序: 左下 → 右下 → 右上 → 左上 → 左上の2m上")

    while True:
        ret, frame = cam.cap.read()
        if not ret or frame is None:
            continue
        disp = frame.copy()
        for i, p in enumerate(points_2d):
            cv2.circle(disp, tuple(p), 6, (0, 0, 255), -1)
            cv2.putText(disp, str(i + 1), (p[0] + 12, p[1] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        remaining = 5 - len(points_2d)
        status = f"Click {remaining} more pts | Enter to confirm" if remaining > 0 else "Press Enter to confirm"
        cv2.putText(disp, status, (10, disp.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow(window_name, disp)

        key = cv2.waitKey(1) & 0xFF
        if key == 13 and len(points_2d) == 5:
            break
        elif key == ord('q'):
            raise RuntimeError(f"[{cam_label}] キャリブレーションがキャンセルされました。")

    K = cam.get_approx_camera_matrix()
    pts2d = np.array(points_2d, dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(FIELD_POINTS, pts2d, K, None, flags=cv2.SOLVEPNP_EPNP)
    if not ok:
        raise RuntimeError(f"[{cam_label}] solvePnP が失敗しました。")

    R, _ = cv2.Rodrigues(rvec)
    cam_pos = -R.T.dot(tvec)
    print(f"  [{cam_label}] キャリブレーション完了 "
          f"X={cam_pos[0,0]:.2f} Y={cam_pos[1,0]:.2f} Z={cam_pos[2,0]:.2f}m")

    cv2.destroyWindow(window_name)
    for _ in range(5):
        cv2.waitKey(1)
    return K, R, tvec


def _dummy_calib(label, cam_pos_xyz):
    K = np.array([[1280, 0, 640], [0, 1280, 360], [0, 0, 1]], dtype=np.float32)
    cam_pos = np.array(cam_pos_xyz, dtype=np.float64).reshape(3, 1)
    R = np.eye(3, dtype=np.float64)
    tvec = -R @ cam_pos
    print(f"  [DUMMY] {label} ダミーキャリブ使用")
    return K, R, tvec


def _rline(sock, buf=""):
    while "\n" not in buf:
        buf += sock.recv(4096).decode()
    line, rest = buf.split("\n", 1)
    return line, rest


def _connect_stream(w_default=1280, h_default=720, retries=5):
    """CALIBを経ずにSTREAMモードだけで接続（保存済みキャリブ利用時）"""
    for attempt in range(retries):
        try:
            sk = socket.socket()
            sk.settimeout(5.0)
            sk.connect((RPI_HOST, RPI_PORT))
            line, buf = _rline(sk)
            info = json.loads(line)
            sk.sendall(b"STREAM\n")
            sk.settimeout(1.0)

            real_cam2 = RemoteCamera(RPI_HOST, RPI_PORT, label="Camera2")
            real_cam2.sock, real_cam2.buf = sk, buf
            real_cam2.width  = info.get("width", w_default)
            real_cam2.height = info.get("height", h_default)
            print("  [Camera2] STREAM接続成功")
            return real_cam2
        except Exception as e:
            print(f"  [Camera2] 接続試行{attempt+1}/{retries}失敗: {e}")
            time.sleep(1.0)
    return None


def _run_camera2_fresh_calibration():
    """RPi画面で5点クリック → solvePnP → 保存 → STREAM接続"""
    print("\n  ─── Camera2 キャリブレーション ───")
    print("       ラズパイ画面で5点クリック")
    print("       順序: 左下→右下→右上→左上→左上+2m")

    pts, w2, h2 = None, 1280, 720
    sk = socket.socket()
    sk.settimeout(120.0)
    try:
        sk.connect((RPI_HOST, RPI_PORT))
        line, buf = _rline(sk)
        info = json.loads(line)
        w2, h2 = info["width"], info["height"]
        sk.sendall(b"CALIB\n")
        print("  [A] CALIB送信 - クリック待ち...")
        line, _ = _rline(sk, buf)
        pts = json.loads(line)["calib_pts"]
        print(f"  [A] {len(pts)}点受信: {pts}")
    except Exception as e:
        print(f"  [FAIL] CALIB失敗: {e}")
    finally:
        sk.close()

    if not pts or len(pts) != 5:
        print("  [FAIL] 点データ不正 → ダミー")
        K2, R2, tvec2 = _dummy_calib("Camera2", [8, -8, 2])
        return K2, R2, tvec2, None

    K2 = np.array([[w2, 0, w2/2], [0, w2, h2/2], [0, 0, 1]], dtype=np.float32)
    p2d = np.array(pts, dtype=np.float32)
    ok, rvec2, tvec2 = cv2.solvePnP(FIELD_POINTS, p2d, K2, None, flags=cv2.SOLVEPNP_EPNP)
    if not ok:
        print("  [FAIL] solvePnP失敗 → ダミー")
        K2, R2, tvec2 = _dummy_calib("Camera2", [8, -8, 2])
        return K2, R2, tvec2, None

    R2, _ = cv2.Rodrigues(rvec2)
    cp = -R2.T.dot(tvec2)
    print(f"  [B] solvePnP: X={cp[0,0]:.2f} Y={cp[1,0]:.2f} Z={cp[2,0]:.2f}m")
    save_calibration("Camera2", K2, R2, tvec2, pts, w2, h2)

    time.sleep(0.3)
    real_cam2 = _connect_stream(w2, h2, retries=1)
    return K2, R2, tvec2, real_cam2


def run_calibration_phase(cam1, cam1_ok: bool, cam2_ok_rpi: bool, cam2_stub):
    """
    Camera1・Camera2両方のキャリブレーションを実施し、
    (K1, R1, tvec1, K2, R2, tvec2, cam2) を返す。
    """
    print("\n[INIT 4/4]  フィールドキャリブレーション")

    # ── Camera1 ──────────────────────────────────────────────
    if cam1_ok and ask_use_saved("Camera1"):
        saved = load_calibration("Camera1")
        K1, R1, tvec1 = saved["K"], saved["R"], saved["tvec"]
        print("  [Camera1] 保存済みデータを使用")
    elif cam1_ok:
        print("\n  ─── Camera1 キャリブレーション ───")
        K1, R1, tvec1 = run_calibration_single(cam1, "Camera1")
        save_calibration("Camera1", K1, R1, tvec1)
    else:
        K1, R1, tvec1 = _dummy_calib("Camera1", [-8, -8, 2])

    # ── Camera2 ──────────────────────────────────────────────
    cam2 = cam2_stub
    if cam2_ok_rpi and ask_use_saved("Camera2"):
        saved = load_calibration("Camera2")
        K2, R2, tvec2 = saved["K"], saved["R"], saved["tvec"]
        print("  [Camera2] 保存済みデータを使用 → STREAM接続のみ実施")
        real_cam2 = _connect_stream(saved.get("width", 1280), saved.get("height", 720))
        if real_cam2 is not None:
            cam2 = real_cam2
        else:
            print("  [WARN] Camera2 接続失敗 → ダミーで続行")
            K2, R2, tvec2 = _dummy_calib("Camera2", [8, -8, 2])

    elif cam2_ok_rpi:
        K2, R2, tvec2, real_cam2 = _run_camera2_fresh_calibration()
        if real_cam2 is not None:
            cam2 = real_cam2
    else:
        K2, R2, tvec2 = _dummy_calib("Camera2", [8, -8, 2])

    return K1, R1, tvec1, K2, R2, tvec2, cam2