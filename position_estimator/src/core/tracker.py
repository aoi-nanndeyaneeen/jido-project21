"""
tracker.py
2カメラ対応のカメラスレッド・OpenCV描画・ログ記録を担当。

位置推定方式:
    カメラ1 と カメラ2 の両方で機体の重心ピクセル座標(u,v)を取得し、
    それぞれのカメラ行列(K)とカメラ姿勢(R, tvec)からワールド空間の視線ベクトルを生成。
    2本のレイの最近接点を3D位置 (X, Y, Z) として推定する。
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

    Args:
        cam1 (CameraTracker): カメラ1
        cam2 (CameraTracker): カメラ2
        K1, R1, tvec1: カメラ1のキャリブレーション結果
        K2, R2, tvec2: カメラ2のキャリブレーション結果
        log_path (Path): ログファイルパス
        shared (dict): スレッド間共有状態
        plot_lock (threading.Lock): plot_data 保護用ロック
        plot_data (dict): UIスレッドへの結果受け渡し用辞書
    """
    # カメラ位置 (ワールド座標) を事前計算
    O1_fixed = (-R1.T.dot(tvec1)).flatten()
    O2_fixed = (-R2.T.dot(tvec2)).flatten()

    log = FlightLogger(log_path)

    # 2画面表示ウィンドウ
    cv2.namedWindow("Camera 1", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Camera 2", cv2.WINDOW_NORMAL)

    try:
        while not shared.get("quit", False):

            # ── 両カメラからフレームを取得 ──────────────────────────────
            frame1, uv1 = cam1.read_and_track()
            frame2, uv2 = cam2.read_and_track()

            # 変更後（frame2はNoneでもOK）
            if frame1 is None:
                print("[Tracker] Camera1からフレームを取得できませんでした。")
                break

            # Camera2ウィンドウはフレームがある時だけ表示
            if frame2 is not None:
                cv2.imshow("Camera 2", frame2)

            # ── 3D位置推定 ──────────────────────────────────────────────
            P_vec    = None
            residual = None

            if uv1 is not None and uv2 is not None:
                # 両カメラで機体を検出 → 視線ベクトルを生成
                u1, v1 = uv1
                u2, v2 = uv2

                _, D1 = get_ray(u1, v1, K1, R1, tvec1)
                _, D2 = get_ray(u2, v2, K2, R2, tvec2)

                P_raw, res = intersect_rays(O1_fixed, D1, O2_fixed, D2)

                if P_raw is not None and is_valid_position(P_raw):
                    P_vec    = P_raw
                    residual = res

                    # 位置情報をフレームに描画
                    x_str = f"X:{P_vec[0]:.2f} Y:{P_vec[1]:.2f} Z:{P_vec[2]:.2f}m  err:{res:.3f}m"
                    cv2.putText(frame1, x_str,
                                (10, frame1.shape[0] - 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(frame2, x_str,
                                (10, frame2.shape[0] - 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                else:
                    _label = "OUT OF RANGE" if P_raw is not None else "RAYS PARALLEL"
                    cv2.putText(frame1, _label,
                                (10, frame1.shape[0] - 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            elif uv1 is None and uv2 is None:
                # 両カメラで機体未検出
                msg = "NO TARGET"
                cv2.putText(frame1, msg, (10, frame1.shape[0] - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 128, 128), 2)
                cv2.putText(frame2, msg, (10, frame2.shape[0] - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 128, 128), 2)
            else:
                # 片方のみ検出
                missing = "Camera2 missing" if uv1 is not None else "Camera1 missing"
                cv2.putText(frame1, missing, (10, frame1.shape[0] - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

            # 操作ガイドを表示
            guide = "[SPACE]Calib  [B]BG Reset  [Q]Quit"
            cv2.putText(frame1, guide,
                        (10, frame1.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
            cv2.putText(frame2, guide,
                        (10, frame2.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            # ── ログ記録 ────────────────────────────────────────────────
            current_z = P_vec[2] if P_vec is not None else 0.0
            log.write(P_vec, current_z, 0.0, 0.0, 0.0, 0.0)

            # ── コマンド処理 ─────────────────────────────────────────────
            if shared.get("do_bg_reset", False):
                shared["do_bg_reset"] = False
                cam1.reset_background()
                cam2.reset_background()
                print("[Tracker] 背景をリセットしました")

            # ── カメラウィンドウに表示 ────────────────────────────────────
            cv2.imshow("Camera 1", frame1)
            cv2.imshow("Camera 2", frame2)

            # ── plot_data 更新 ────────────────────────────────────────────
            with plot_lock:
                plot_data["P"]         = P_vec
                plot_data["O1"]        = O1_fixed
                plot_data["O2"]        = O2_fixed
                plot_data["current_z"] = current_z
                plot_data["residual"]  = residual
                plot_data["uv1"]       = uv1
                plot_data["uv2"]       = uv2
                plot_data["updated"]   = True
                plot_data["frame1"]    = frame1.copy()
                plot_data["frame2"]    = frame2.copy()

    finally:
        log.close()
        cam1.release()
        cam2.release()
        cv2.destroyAllWindows()