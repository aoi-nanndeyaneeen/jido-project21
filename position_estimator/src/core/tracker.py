"""
tracker.py
2カメラ対応のカメラスレッド・ログ記録を担当。

位置推定:
    両カメラの視線ベクトルの最近接点を3D位置とする。
    residual（2本のレイの最近接距離）が MAX_RESIDUAL_M を超える場合は外れ値として破棄。
    DUMMY_FALLBACK_FRAMES フレーム連続で未検出の場合、DummyFlight に切り替え。
"""

import cv2
import numpy as np
import threading
from pathlib import Path

from core.geometry   import get_ray, intersect_rays
from core.dummy_flight import DummyFlight
from utils.logger    import FlightLogger
from utils.config    import (MAX_RESIDUAL_M,
                              DUMMY_FALLBACK_FRAMES,
                              DUMMY_ORBIT_RADIUS,
                              DUMMY_ORBIT_ALT,
                              DUMMY_ORBIT_PERIOD)


def camera_thread_func(cam1, cam2,
                       K1, R1, tvec1,
                       K2, R2, tvec2,
                       log_path: Path,
                       shared: dict,
                       plot_lock: threading.Lock,
                       plot_data: dict):

    O1_fixed = (-R1.T.dot(tvec1)).flatten()
    O2_fixed = (-R2.T.dot(tvec2)).flatten()

    log   = FlightLogger(log_path)
    dummy = DummyFlight(DUMMY_ORBIT_RADIUS, DUMMY_ORBIT_ALT, DUMMY_ORBIT_PERIOD)

    no_detect_count = 0
    in_dummy_mode   = False

    try:
        while not shared.get("quit", False):

            # ── 両カメラからフレームを取得 ──────────────────────────
            frame1, uv1 = cam1.read_and_track()
            frame2, uv2 = cam2.read_and_track()

            # カメラ未接続時はプレースホルダー画像で継続
            if frame1 is None:
                frame1 = np.zeros((720, 1280, 3), dtype=np.uint8)
                cv2.putText(frame1, "Camera 1 - NO SIGNAL",
                            (350, 360), cv2.FONT_HERSHEY_SIMPLEX,
                            1.5, (80, 80, 80), 2)
            if frame2 is None:
                frame2 = np.zeros((720, 1280, 3), dtype=np.uint8)
                cv2.putText(frame2, "Camera 2 - NO SIGNAL",
                            (350, 360), cv2.FONT_HERSHEY_SIMPLEX,
                            1.5, (80, 80, 80), 2)

            # ── 3D位置推定 ──────────────────────────────────────────
            P_vec    = None
            residual = None
            status_label = ""
            status_color = (128, 128, 128)

            if uv1 is not None and uv2 is not None:
                _, D1 = get_ray(uv1[0], uv1[1], K1, R1, tvec1)
                _, D2 = get_ray(uv2[0], uv2[1], K2, R2, tvec2)
                P_raw, res = intersect_rays(O1_fixed, D1, O2_fixed, D2)

                if P_raw is None:
                    status_label = "RAYS PARALLEL"
                    status_color = (0, 0, 255)
                elif res > MAX_RESIDUAL_M:
                    status_label = f"HIGH RESIDUAL {res:.2f}m (>{MAX_RESIDUAL_M:.1f})"
                    status_color = (0, 140, 255)
                else:
                    P_vec    = P_raw
                    residual = res
                    status_label = (f"X:{P_vec[0]:.2f} Y:{P_vec[1]:.2f} "
                                    f"Z:{P_vec[2]:.2f}m  err:{res:.3f}m")
                    status_color = (0, 255, 255)

            elif uv1 is None and uv2 is None:
                status_label = "NO TARGET"
            elif uv1 is None:
                status_label = "Camera1 missing"
                status_color = (0, 165, 255)
            else:
                status_label = "Camera2 missing"
                status_color = (0, 165, 255)

            # ── ダミーへのフォールバック ─────────────────────────────
            if P_vec is not None:
                # 検出できた → ダミーモード解除
                if in_dummy_mode:
                    print("[Tracker] Camera detected again → REAL mode")
                    in_dummy_mode = False
                no_detect_count = 0
            else:
                no_detect_count += 1
                if no_detect_count >= DUMMY_FALLBACK_FRAMES:
                    if not in_dummy_mode:
                        print(f"[Tracker] No detection for {DUMMY_FALLBACK_FRAMES} frames"
                              f" → DUMMY mode (r={DUMMY_ORBIT_RADIUS}m "
                              f"alt={DUMMY_ORBIT_ALT}m)")
                        in_dummy_mode = True
                        dummy.reset()
                    P_vec    = dummy.get_position()
                    residual = 0.0
                    status_label = (f"[DUMMY] X:{P_vec[0]:.2f} Y:{P_vec[1]:.2f} "
                                    f"Z:{P_vec[2]:.2f}m")
                    status_color = (180, 255, 180)

            # ── フレームへの情報描画 ─────────────────────────────────
            for frm in (frame1, frame2):
                if frm is None:
                    continue
                h = frm.shape[0]
                cv2.putText(frm, status_label, (10, h - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
                cv2.putText(frm, "[B]BG Reset  [Q]Quit",
                            (10, h - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                if in_dummy_mode:
                    cv2.putText(frm, "-- DUMMY --", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 255, 180), 2)

            # ── ログ記録 ────────────────────────────────────────────
            current_z = float(P_vec[2]) if P_vec is not None else 0.0
            log.write(P_vec, current_z, 0.0, 0.0, 0.0, 0.0)

            # ── 背景リセット ─────────────────────────────────────────
            if shared.get("do_bg_reset", False):
                shared["do_bg_reset"] = False
                cam1.reset_background()
                cam2.reset_background()
                print("[Tracker] Background reset")

            # ── plot_data 更新 ────────────────────────────────────────
            with plot_lock:
                plot_data["P"]         = P_vec
                plot_data["O1"]        = O1_fixed
                plot_data["O2"]        = O2_fixed
                plot_data["current_z"] = current_z
                plot_data["residual"]  = residual
                plot_data["uv1"]       = uv1
                plot_data["uv2"]       = uv2
                plot_data["frame1"]    = frame1.copy()
                plot_data["frame2"]    = frame2.copy() if frame2 is not None else None
                plot_data["updated"]   = True

    finally:
        log.close()
        cam1.release()
        cam2.release()