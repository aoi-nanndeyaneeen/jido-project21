"""
tracker.py
2カメラ対応のカメラスレッド・ログ記録を担当。
※ imshow / namedWindow / destroyAllWindows はメインスレッド(main.py)で管理するため、
   このファイルには一切記述しない。
"""

import cv2
import numpy as np
import threading
from pathlib import Path

from core.geometry import get_ray, intersect_rays, is_valid_position
from utils.logger import FlightLogger


def camera_thread_func(cam1, cam2,
                       K1, R1, tvec1,
                       K2, R2, tvec2,
                       log_path: Path,
                       shared: dict,
                       plot_lock: threading.Lock,
                       plot_data: dict):
    """
    2台のカメラを並列で読み込み、ステレオビジョンで3D位置を推定するスレッド関数。
    フレームの表示は行わず、plot_data に結果を格納してメインスレッドに渡す。
    """
    O1_fixed = (-R1.T.dot(tvec1)).flatten()
    O2_fixed = (-R2.T.dot(tvec2)).flatten()

    log = FlightLogger(log_path)

    try:
        while not shared.get("quit", False):

            # ── 両カメラからフレームを取得 ──────────────────────────
            frame1, uv1 = cam1.read_and_track()
            frame2, uv2 = cam2.read_and_track()

            if frame1 is None:
                print("[Tracker] Camera1 フレーム取得失敗")
                break

            # ── 3D位置推定 ──────────────────────────────────────────
            P_vec    = None
            residual = None

            if uv1 is not None and uv2 is not None:
                u1, v1 = uv1
                u2, v2 = uv2

                _, D1 = get_ray(u1, v1, K1, R1, tvec1)
                _, D2 = get_ray(u2, v2, K2, R2, tvec2)

                P_raw, res = intersect_rays(O1_fixed, D1, O2_fixed, D2)

                if P_raw is not None and is_valid_position(P_raw):
                    P_vec    = P_raw
                    residual = res
                    label = f"X:{P_vec[0]:.2f} Y:{P_vec[1]:.2f} Z:{P_vec[2]:.2f}m  err:{res:.3f}m"
                    color = (0, 255, 255)
                else:
                    label = "OUT OF RANGE" if P_raw is not None else "RAYS PARALLEL"
                    color = (0, 0, 255)

                cv2.putText(frame1, label, (10, frame1.shape[0] - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                if frame2 is not None:
                    cv2.putText(frame2, label, (10, frame2.shape[0] - 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            elif uv1 is None and uv2 is None:
                msg = "NO TARGET"
                cv2.putText(frame1, msg, (10, frame1.shape[0] - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 128, 128), 2)
            else:
                missing = "Camera2 missing" if uv1 is not None else "Camera1 missing"
                cv2.putText(frame1, missing, (10, frame1.shape[0] - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

            # 操作ガイド
            guide = "[SPACE]Calib  [B]BG Reset  [Q]Quit"
            cv2.putText(frame1, guide, (10, frame1.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
            if frame2 is not None:
                cv2.putText(frame2, guide, (10, frame2.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            # ── ログ記録 ────────────────────────────────────────────
            current_z = P_vec[2] if P_vec is not None else 0.0
            log.write(P_vec, current_z, 0.0, 0.0, 0.0, 0.0)

            # ── 背景リセット ─────────────────────────────────────────
            if shared.get("do_bg_reset", False):
                shared["do_bg_reset"] = False
                cam1.reset_background()
                cam2.reset_background()
                print("[Tracker] 背景をリセットしました")

            # ── plot_data 更新（メインスレッドへ渡す） ───────────────
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
        # ※ destroyAllWindows はメインスレッドで呼ぶため、ここでは呼ばない