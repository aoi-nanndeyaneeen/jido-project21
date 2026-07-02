"""
tracker.py
2カメラ対応のカメラスレッド・ログ記録を担当。

位置推定:
    1) 各カメラの画像座標(u,v)についてPixelJumpFilterで急激なジャンプ（誤検知）を棄却
    2) 残った有効な検知のみで両カメラの視線ベクトルの最近接点を3D位置とする
    3) residual（2本のレイの最近接距離）が MAX_RESIDUAL_M を超える場合は外れ値として破棄
    4) DUMMY_FALLBACK_FRAMES フレーム連続で未検出の場合、DummyFlight に切り替え
"""

import cv2
import numpy as np
import threading
import time
import traceback
from pathlib import Path

from core.geometry     import get_ray, intersect_rays
from core.dummy_flight import DummyFlight
from utils.logger      import FlightLogger
from utils.config      import (MAX_RESIDUAL_M,
                                DUMMY_FALLBACK_FRAMES,
                                DUMMY_ORBIT_RADIUS,
                                DUMMY_ORBIT_ALT,
                                DUMMY_ORBIT_PERIOD,
                                PIXEL_SPEED_LIMIT_PX_S,
                                JUMP_RECOVERY_FRAMES)


class PixelJumpFilter:
    """
    画像座標(u,v)の急激なジャンプ（誤検知）を画像空間で棄却するフィルタ。

    固定ピクセル数ではなく px/秒 の速度で判定するため、
    フレームレートが変動しても（カメラ1とカメラ2で異なっても）機能する。
    ただし何フレームも連続して同じ「新しい場所」に検知され続けた場合は
    本当に対象が移動した／再取得したとみなして受理する。
    """
    def __init__(self, speed_limit_px_s: float, recovery_frames: int):
        self.speed_limit = speed_limit_px_s
        self.recovery_frames = recovery_frames
        self.last_uv = None
        self.last_t  = None
        self.reject_streak = 0

    def filter(self, uv):
        now = time.time()

        if uv is None:
            # 未検出はジャンプ判定と無関係。連続棄却カウントは維持する
            return None

        if self.last_uv is None or self.last_t is None:
            self.last_uv, self.last_t = uv, now
            return uv

        dt = max(now - self.last_t, 1e-3)
        dist = float(np.hypot(uv[0] - self.last_uv[0], uv[1] - self.last_uv[1]))
        allowed = self.speed_limit * dt

        if dist <= allowed:
            self.last_uv, self.last_t = uv, now
            self.reject_streak = 0
            return uv

        # 許容速度を超えるジャンプ → ノイズを疑う
        self.reject_streak += 1
        if self.reject_streak >= self.recovery_frames:
            # 同じ新しい場所に連続して出現＝本当の移動として受理
            self.last_uv, self.last_t = uv, now
            self.reject_streak = 0
            return uv

        # 棄却（前回位置は更新しない＝次回も同じ基準で判定）
        return None


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

    jf1 = PixelJumpFilter(PIXEL_SPEED_LIMIT_PX_S, JUMP_RECOVERY_FRAMES)
    jf2 = PixelJumpFilter(PIXEL_SPEED_LIMIT_PX_S, JUMP_RECOVERY_FRAMES)

    no_detect_count = 0
    in_dummy_mode   = False
    frame_count     = 0

    print(f"[Tracker] 開始 - ログ: {log_path}")

    try:
        while not shared.get("quit", False):
            try:
                # ── 両カメラからフレームを取得 ──────────────────
                frame1, uv1_raw = cam1.read_and_track()
                frame2, uv2_raw = cam2.read_and_track()

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

                # ── 画像空間ジャンプフィルタ（誤検知除去） ──────
                uv1 = jf1.filter(uv1_raw)
                uv2 = jf2.filter(uv2_raw)

                jump_rejected = (uv1_raw is not None and uv1 is None) or \
                                (uv2_raw is not None and uv2 is None)

                # ── 3D位置推定 ──────────────────────────────────
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
                    status_label = "JUMP REJECTED" if jump_rejected else "NO TARGET"
                    status_color = (0, 100, 255) if jump_rejected else (128, 128, 128)
                elif uv1 is None:
                    status_label = "Camera1 missing/rejected"
                    status_color = (0, 165, 255)
                else:
                    status_label = "Camera2 missing/rejected"
                    status_color = (0, 165, 255)

                # ── ダミーへのフォールバック ─────────────────────
                if P_vec is not None:
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
                            print("[Tracker] ※ダミーモード中はログを記録しません")
                            in_dummy_mode = True
                            dummy.reset()
                        P_vec    = dummy.get_position()
                        residual = 0.0
                        status_label = (f"[DUMMY] X:{P_vec[0]:.2f} Y:{P_vec[1]:.2f} "
                                        f"Z:{P_vec[2]:.2f}m")
                        status_color = (180, 255, 180)

                # ── フレームへの情報描画 ─────────────────────────
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

                # ── ログ記録（ダミーモード中は記録しない） ────────
                if not in_dummy_mode:
                    current_z = float(P_vec[2]) if P_vec is not None else 0.0
                    log.write(P_vec, current_z, 0.0, 0.0, 0.0, 0.0)
                else:
                    current_z = float(P_vec[2]) if P_vec is not None else 0.0

                frame_count += 1
                if frame_count % 300 == 0:
                    log_status = "ログ停止中(DUMMY)" if in_dummy_mode else "ログ書き込み中"
                    print(f"[Tracker] {frame_count}フレーム処理済み（{log_status}）")

                # ── 背景リセット ─────────────────────────────────
                if shared.get("do_bg_reset", False):
                    shared["do_bg_reset"] = False
                    cam1.reset_background()
                    cam2.reset_background()
                    jf1.last_uv = jf2.last_uv = None   # ジャンプ判定もリセット
                    print("[Tracker] Background reset")

                # ── plot_data 更新 ────────────────────────────────
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

            except Exception:
                # 1フレームの例外でスレッド全体を落とさない
                print("[Tracker] フレーム処理中に例外発生（継続します）:")
                traceback.print_exc()
                time.sleep(0.05)

    finally:
        print(f"[Tracker] 終了 - 合計{frame_count}フレーム処理")
        log.close()
        cam1.release()
        cam2.release()