"""
app/main_loop.py
メインループ本体：ウィンドウ管理・カメラスレッド起動・
オートパイロット計算・各種可視化ウィンドウの更新を担当。
"""

import cv2
import numpy as np
import threading
import time
import msvcrt
from collections import deque

from utils.config import DISP_W, DISP_H, VELOCITY_W, VELOCITY_H
from core.tracker import camera_thread_func
from core.controller import AltitudeController
from core.geometry import accel_to_angles
from core.autopilot import SquarePatrol, RCCommand
from ui.dashboard import Dashboard
from ui.view_velocity import ViewVelocity
from ui.view_rc import ViewRC


def run_main_loop(cam1, cam2,
                  K1, R1, tvec1,
                  K2, R2, tvec2,
                  log_path,
                  alt_sensor,
                  field_points):

    plot_lock = threading.Lock()
    plot_data = {
        "P": None, "O1": None, "O2": None,
        "current_z": 0.0, "residual": None,
        "uv1": None, "uv2": None,
        "updated": False,
        "frame1": None, "frame2": None,
    }

    print()
    print("=" * 56)
    print("   ALL SYSTEMS GO  -  STEREO TRACKING STARTED")
    print("=" * 56)
    print()
    print("  [B]     背景リセット (両カメラ)")
    print("  [Q]     終了")
    print()

    shared = {"do_bg_reset": False, "quit": False}

    # ── ウィンドウ作成 ──────────────────────────────────────
    cv2.namedWindow("Camera 1", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera 1", DISP_W, DISP_H)
    cv2.moveWindow("Camera 1", 0, 0)

    cv2.namedWindow("Camera 2", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera 2", DISP_W, DISP_H)
    cv2.moveWindow("Camera 2", DISP_W + 10, 0)

    # ── ターミナルフォーカス時のキー入力スレッド ────────────
    def keyboard_thread():
        while not shared.get("quit", False):
            if msvcrt.kbhit():
                key = msvcrt.getch().lower()
                if key == b'q':
                    print("[KEY] Q → 終了")
                    shared["quit"] = True
                elif key == b'b':
                    print("[KEY] B → 背景リセット")
                    shared["do_bg_reset"] = True
            time.sleep(0.05)

    threading.Thread(target=keyboard_thread, daemon=True).start()

    # ── カメラスレッド起動 ──────────────────────────────────
    cam_thread = threading.Thread(
        target=camera_thread_func,
        args=(cam1, cam2, K1, R1, tvec1, K2, R2, tvec2,
              log_path, shared, plot_lock, plot_data),
        daemon=True)
    cam_thread.start()

    # ── 各ビュー初期化 ──────────────────────────────────────
    dashboard      = Dashboard(field_points)
    controller     = AltitudeController(p_gain=5.0)
    velocity_view  = ViewVelocity(field_points)
    patrol         = SquarePatrol(start_x=0.0, start_y=0.0)
    view_rc        = ViewRC()

    cv2.namedWindow("RC Command", cv2.WINDOW_NORMAL)
    cv2.imshow("RC Command", np.zeros((ViewRC.H, ViewRC.W, 3), dtype=np.uint8))
    cv2.waitKey(1)
    cv2.resizeWindow("RC Command", ViewRC.W, ViewRC.H)
    cv2.moveWindow("RC Command", VELOCITY_W + 640, DISP_H + 40)
    cv2.waitKey(1)

    cv2.namedWindow("Velocity", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Velocity", VELOCITY_W, VELOCITY_H)
    cv2.moveWindow("Velocity", 0, DISP_H + 40)

    _pos_hist = deque(maxlen=6)
    _last_cmd = RCCommand()
    _ap_time  = time.time()

    last_mpl_render = 0.0
    MPL_RENDER_HZ   = 5

    # ── メインループ ──────────────────────────────────────
    while not shared.get("quit", False):
        with plot_lock:
            updated   = plot_data["updated"]
            P         = plot_data["P"]
            O1        = plot_data["O1"]
            current_z = plot_data["current_z"]
            if updated:
                plot_data["updated"] = False

        if updated:
            with plot_lock:
                f1 = plot_data.get("frame1")
                f2 = plot_data.get("frame2")
            if f1 is not None:
                cv2.imshow("Camera 1", cv2.resize(f1, (DISP_W, DISP_H)))
            if f2 is not None:
                cv2.imshow("Camera 2", cv2.resize(f2, (DISP_W, DISP_H)))

            # ── オートパイロット計算 ────────────────────────
            now_ap = time.time()
            dt_ap  = now_ap - _ap_time
            _ap_time = now_ap

            if P is not None:
                _pos_hist.append((now_ap, P.copy()))

            vel_ap = np.zeros(3)
            if len(_pos_hist) >= 2:
                t0, p0 = _pos_hist[0]
                t1, p1 = _pos_hist[-1]
                dt_v = t1 - t0
                if dt_v > 1e-4:
                    vel_ap = (p1 - p0) / dt_v

            if P is not None and dt_ap > 0:
                _last_cmd = patrol.update(pos=P, vel=vel_ap, dt=dt_ap)

            rc_img = view_rc.get_image(_last_cmd)
            cv2.imshow("RC Command", rc_img)

            # ── matplotlib系（レート制限） ──────────────────
            now = time.time()
            if now - last_mpl_render > 1.0 / MPL_RENDER_HZ:
                last_mpl_render = now

                roll_deg, pitch_deg = 0.0, 0.0
                imu_available = alt_sensor is not None
                if imu_available:
                    roll_deg, pitch_deg = accel_to_angles(alt_sensor.get_accel())

                vel_img = velocity_view.get_image(P, roll_deg, pitch_deg, imu_available)
                cv2.imshow("Velocity", vel_img)

                if O1 is not None:
                    target_alt = controller.get_target()
                    dashboard.render_and_show(P, current_z, target_alt)
                    if imu_available and P is not None:
                        pitch_cmd = controller.calc_pitch_command(current_z)
                        alt_sensor.send_target_altitude(pitch_cmd)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            shared["quit"] = True
        elif key == ord('b'):
            shared["do_bg_reset"] = True
        elif key == ord('t'):
            def ask_target():
                try:
                    val = input("\n>>> 新しい目標高度を入力 (m): ")
                    controller.set_target(float(val))
                    print(f">>> 目標高度を {float(val):.1f}m に設定しました。")
                except ValueError:
                    print(">>> [エラー] 数値を入力してください。")
            threading.Thread(target=ask_target, daemon=True).start()

    # ── 終了処理 ──────────────────────────────────────────
    if alt_sensor is not None:
        alt_sensor.stop()
    dashboard.close()
    velocity_view.close()
    cv2.destroyAllWindows()
    for _ in range(10):
        cv2.waitKey(1)