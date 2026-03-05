#カメラスレッド、OpenCV描画

"""
tracker.py
カメラスレッド・OpenCV描画・ログ記録を担当。
"""

import cv2
import numpy as np
import threading
from pathlib import Path

from utils.config import MIN_TRACK_ALT
from core.geometry import get_ray, calc_tilt, accel_to_angles, is_valid_position
from utils.logger import FlightLogger


def draw_horizon(frame, roll_deg, pitch_deg):
    cx, cy, r = 120, 120, 90
    cv2.circle(frame, (cx, cy), r, (40, 40, 40), -1)
    pitch_px  = int(np.clip(pitch_deg / 90.0 * r, -r, r))
    cos_a = np.cos(np.radians(roll_deg))
    sin_a = np.sin(np.radians(roll_deg))

    def hp(sign):
        return (int(cx + sign*r*cos_a - pitch_px*sin_a),
                int(cy + sign*r*sin_a + pitch_px*cos_a))

    p1, p2 = hp(-1), hp(1)
    mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
    cv2.circle(mask, (cx, cy), r-1, 255, -1)
    sky_mask = np.zeros_like(mask)
    dx, dy = -(p2[1]-p1[1]), p2[0]-p1[0]
    pts = np.array([p1, p2, (p2[0]+dx*5, p2[1]+dy*5), (p1[0]+dx*5, p1[1]+dy*5)], dtype=np.int32)
    cv2.fillPoly(sky_mask, [pts], 255)
    cv2.bitwise_and(sky_mask, mask, sky_mask)
    frame[mask == 255]     = (100, 60, 30)
    frame[sky_mask == 255] = (180, 120, 40)
    cv2.line(frame, p1, p2, (255,255,255), 2)
    cv2.line(frame, (cx-30,cy), (cx-10,cy), (0,255,255), 3)
    cv2.line(frame, (cx+10,cy), (cx+30,cy), (0,255,255), 3)
    cv2.circle(frame, (cx,cy), 4, (0,255,255), -1)
    cv2.circle(frame, (cx,cy), r, (200,200,200), 2)
    cv2.putText(frame, f"Roll :{roll_deg:+.1f}deg",  (10,230), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
    cv2.putText(frame, f"Pitch:{pitch_deg:+.1f}deg", (10,260), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
    cv2.putText(frame, "Yaw : N/A",                  (10,290), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128,128,128), 2)


def camera_thread_func(cam, alt_sensor, K, R, tvec, log_path: Path,
                       shared: dict, plot_lock: threading.Lock, plot_data: dict):
    O_fixed = (-R.T.dot(tvec)).flatten()
    log     = FlightLogger(log_path)

    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Camera", lambda *a: None)

    try:
        while not shared.get("quit", False):
            raw_alt   = alt_sensor.get_altitude()
            raw_accel = alt_sensor.get_accel()

            alt_offset = shared["alt_offset"]
            current_z  = raw_alt - alt_offset
            roll, pitch = calc_tilt(raw_accel, shared["ref_roll"], shared["ref_pitch"])

            frame, center_uv = cam.read_and_track()
            if frame is None:
                break

            # --- OpenCV描画 ---
            draw_horizon(frame, roll, pitch)
            cv2.putText(frame,
                f"Alt(raw):{raw_alt:.2f}m  offset:{alt_offset:.2f}m  rel:{current_z:.2f}m",
                (10,340), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180,180,180), 2)
            cv2.putText(frame, "[SPACE]Calib(Z+tilt)  [B]BG Reset  [Q]Quit",
                (10, frame.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 2)

            # --- 3D位置推定 ---
            P_vec = None
            if center_uv is not None:
                # ★ 低高度では交点計算が発散するためスキップ
                if current_z < MIN_TRACK_ALT:
                    cv2.putText(frame,
                        f"ALT TOO LOW ({current_z:.2f}m < {MIN_TRACK_ALT}m)",
                        (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 100, 255), 3)
                else:
                    u, v = center_uv
                    O_ray, D = get_ray(u, v, K, R, tvec)
                    if abs(D[2]) > 1e-6:
                        t_val = (current_z - O_ray[2]) / D[2]
                        P_raw = O_ray + t_val * D
                        if is_valid_position(np.append(P_raw[:2], current_z)):
                            P_vec = P_raw   # ★ pos_offset は廃止
                            cv2.putText(frame,
                                f"X:{P_vec[0]:.2f} Y:{P_vec[1]:.2f} Z:{current_z:.2f}m",
                                (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,255,255), 4)
                        else:
                            cv2.putText(frame, "OUT OF RANGE",
                                (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)

            # --- ログ ---
            log.write(P_vec, current_z, roll, pitch, raw_alt, alt_offset)

            # --- キャリブレーション（Z・傾きのみ、X/Yリセットなし） ---
            if shared.get("do_calib", False):
                shared["do_calib"]   = False
                shared["alt_offset"] = raw_alt
                r0, p0 = accel_to_angles(raw_accel)
                shared["ref_roll"]   = r0
                shared["ref_pitch"]  = p0
                print(f"[Calib] Z={raw_alt:.2f}m→0  Roll={r0:.1f}°  Pitch={p0:.1f}°")

            if shared.get("do_bg_reset", False):
                shared["do_bg_reset"] = False
                cam.prev_gray = None
                print("背景をリセットしました")

            # --- plot_data 更新 ---
            with plot_lock:
                plot_data["P"]         = P_vec
                plot_data["O"]         = O_fixed
                plot_data["roll"]      = roll
                plot_data["pitch"]     = pitch
                plot_data["current_z"] = current_z
                plot_data["updated"]   = True
                plot_data["frame"]     = frame.copy() # ★ ここで画像を渡す！

            #cv2.imshow("Camera", frame)

    finally:
        log.close()
        cam.release()
        cv2.destroyAllWindows()