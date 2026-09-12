"""
tracker.py
2カメラ対応のカメラスレッド・ログ記録を担当。

位置推定:
    1) 各カメラから「候補リスト」を受け取る（1枚では機体かどうか決められない）
    2) 全ペアを三角測量し、residual・3Dゲート・前回位置からの到達可能性を満たす
       ものだけ残して、最も辻褄の合う1組を機体として採用する
    3) 窓の反射や審査員席のような「片方のカメラにしか整合しない明点」は、
       ペアを組んだ瞬間に residual が跳ね上がるか、フィールド外に落ちて棄却される
    4) DUMMY_FALLBACK_FRAMES フレーム連続で未検出の場合、DummyFlight に切り替え
"""

import cv2
import numpy as np
import threading
import time
import traceback
from pathlib import Path

from core.geometry     import intersect_rays
from core.dummy_flight import DummyFlight
from utils.logger      import FlightLogger, PerformanceLogger
from utils.config      import (MAX_RESIDUAL_M,
                                DUMMY_FALLBACK_FRAMES,
                                DUMMY_ORBIT_RADIUS,
                                DUMMY_ORBIT_ALT,
                                DUMMY_ORBIT_PERIOD,
                                GATE_X, GATE_Y, GATE_Z,
                                TRACK_COAST_SEC,
                                TRACK_MAX_SPEED_MPS)


def _in_gate(P) -> bool:
    """三角測量した点が物理的にありえる空間内かどうか。"""
    return (GATE_X[0] <= P[0] <= GATE_X[1] and
            GATE_Y[0] <= P[1] <= GATE_Y[1] and
            GATE_Z[0] <= P[2] <= GATE_Z[1])


class PairSelector:
    """
    2カメラの候補リストから「機体である1組」を幾何整合で選ぶ。

    面積最大の候補を無条件に採る方式だと、窓の反射のように機体より大きく
    写る明点に必ず負ける。ここでは全ペアを三角測量して
      ・residual (2本のレイの最近接距離) が小さい
      ・3Dゲート内 (フィールド上空) にある
      ・前回位置から到達可能な距離にある
    を満たすものだけを残す。片方のカメラにしか無い誤検知は、どの候補と
    組ませても上の条件を同時には満たせないため自動的に落ちる。
    """

    def __init__(self):
        self.last_P = None
        self.last_t = 0.0

    def select(self, cands1, cands2, calib1, calib2):
        """
        Returns:
            (P, residual, index1, index2) — 採用できるペアが無ければ全て None
        """
        now = time.time()
        if self.last_P is not None and now - self.last_t > TRACK_COAST_SEC:
            # 見失って時間が経った。次は自由に再取得させる
            self.last_P = None

        rays1 = [calib1.ray(c.u, c.v) for c in cands1]
        rays2 = [calib2.ray(c.u, c.v) for c in cands2]

        # 到達可能半径。前回位置がある場合のみ効かせる
        reach = None
        if self.last_P is not None:
            reach = TRACK_MAX_SPEED_MPS * max(now - self.last_t, 1e-3) + 0.5

        best = (None, None, None, None)
        best_res = float("inf")
        for i, (O1, D1) in enumerate(rays1):
            for j, (O2, D2) in enumerate(rays2):
                P, res = intersect_rays(O1, D1, O2, D2)
                if P is None or res > MAX_RESIDUAL_M or not _in_gate(P):
                    continue
                if reach is not None and np.linalg.norm(P - self.last_P) > reach:
                    continue
                if res < best_res:
                    best_res = res
                    best = (P, res, i, j)

        if best[0] is not None:
            self.last_P, self.last_t = best[0], now
        return best

    def reset(self):
        self.last_P = None
        self.last_t = 0.0


def camera_thread_func(cam1, cam2,
                       calib1, calib2,
                       log_path: Path,
                       shared: dict,
                       plot_lock: threading.Lock,
                       plot_data: dict):

    O1_fixed = calib1.origin
    O2_fixed = calib2.origin

    log   = FlightLogger(log_path)
    perf_log = PerformanceLogger(log_path.with_name(log_path.stem + "_perf.csv"))
    dummy = DummyFlight(DUMMY_ORBIT_RADIUS, DUMMY_ORBIT_ALT, DUMMY_ORBIT_PERIOD)

    selector = PairSelector()

    no_detect_count = 0
    in_dummy_mode   = False
    frame_count     = 0
    perf_print_time = time.time()

    print(f"[Tracker] 開始 - ログ: {log_path}")
    for camera in (cam1, cam2):
        start_reader = getattr(camera, "start_latest_reader", None)
        if start_reader is not None:
            start_reader()

    try:
        while not shared.get("quit", False):
            try:
                loop_start = time.perf_counter()
                # ── 両カメラからフレームを取得 ──────────────────
                cam1_start = time.perf_counter()
                frame1, cands1, _ = cam1.read_and_detect()
                cam1_ms = (time.perf_counter() - cam1_start) * 1000.0
                cam2_start = time.perf_counter()
                frame2, cands2, _ = cam2.read_and_detect()
                cam2_ms = (time.perf_counter() - cam2_start) * 1000.0

                # ── 候補ペアの幾何整合で機体を1組選ぶ（Phase B） ──
                P_vec, residual, idx1, idx2 = selector.select(
                    cands1, cands2, calib1, calib2)

                if frame1 is not None:
                    cam1.draw_candidates(frame1, cands1, idx1)
                else:
                    frame1 = np.zeros((720, 1280, 3), dtype=np.uint8)
                    cv2.putText(frame1, "Camera 1 - NO SIGNAL",
                                (350, 360), cv2.FONT_HERSHEY_SIMPLEX,
                                1.5, (80, 80, 80), 2)
                if frame2 is not None:
                    cam2.draw_candidates(frame2, cands2, idx2)
                else:
                    frame2 = np.zeros((720, 1280, 3), dtype=np.uint8)
                    cv2.putText(frame2, "Camera 2 - NO SIGNAL",
                                (350, 360), cv2.FONT_HERSHEY_SIMPLEX,
                                1.5, (80, 80, 80), 2)

                uv1 = (cands1[idx1].u, cands1[idx1].v) if idx1 is not None else None
                uv2 = (cands2[idx2].u, cands2[idx2].v) if idx2 is not None else None

                # 候補はあるのにペアが成立しなかった＝窓の反射などを弾いた状態
                pair_rejected = (P_vec is None
                                 and bool(cands1) and bool(cands2))

                if P_vec is not None:
                    status_label = (f"X:{P_vec[0]:.2f} Y:{P_vec[1]:.2f} "
                                    f"Z:{P_vec[2]:.2f}m  err:{residual:.3f}m")
                    status_color = (0, 255, 255)
                elif pair_rejected:
                    status_label = (f"NO CONSISTENT PAIR "
                                    f"(cand {len(cands1)}/{len(cands2)})")
                    status_color = (0, 140, 255)
                else:
                    status_label = f"NO TARGET (cand {len(cands1)}/{len(cands2)})"
                    status_color = (128, 128, 128)

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
                    log.write(
                        P_vec, current_z,
                        residual if residual is not None else -1.0,
                        bool(cands1),
                        bool(cands2),
                        pair_rejected
                    )
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
                    selector.reset()   # 追従の基準位置もリセット
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
                    # ヨー推定用: この位置が「実測として信用できるか」。
                    # ダミー円軌道中や棄却フレームを窓に含めると
                    # 嘘のヨーが出るため、明示的に伝える
                    plot_data["in_dummy"]      = in_dummy_mode
                    plot_data["tracking_ok"]   = (P_vec is not None
                                                  and not in_dummy_mode)
                    plot_data["frame_time"]    = time.time()
                    plot_data["frame1"]    = frame1.copy()
                    plot_data["frame2"]    = frame2.copy() if frame2 is not None else None
                    plot_data["updated"]   = True

                stats1 = getattr(cam1, "get_performance_stats", lambda: {})()
                stats2 = getattr(cam2, "get_performance_stats", lambda: {})()
                perf_values = {
                    "Loop_ms": (time.perf_counter() - loop_start) * 1000.0,
                    "Cam1_ms": cam1_ms,
                    "Cam2_ms": cam2_ms,
                    "FrameAge1_ms": stats1.get("process_last_age_ms", 0.0),
                    "FrameAge2_ms": stats2.get("process_last_age_ms", 0.0),
                    "ReaderFPS1": stats1.get("reader_fps", 0.0),
                    "ReaderFPS2": stats2.get("reader_fps", 0.0),
                    "ReaderMs1": stats1.get("reader_avg_ms", 0.0),
                    "ReaderMs2": stats2.get("reader_avg_ms", 0.0),
                    "ReadErrors1": stats1.get("reader_errors", 0.0),
                    "ReadErrors2": stats2.get("reader_errors", 0.0),
                }
                perf_log.write("tracker", perf_values)
                if time.time() - perf_print_time >= 2.0:
                    print("[PERF] tracker "
                          f"loop={perf_values['Loop_ms']:.1f}ms "
                          f"cam1={cam1_ms:.1f}ms cam2={cam2_ms:.1f}ms "
                          f"age=({perf_values['FrameAge1_ms']:.1f},"
                          f"{perf_values['FrameAge2_ms']:.1f})ms "
                          f"reader=({perf_values['ReaderFPS1']:.1f},"
                          f"{perf_values['ReaderFPS2']:.1f})fps")
                    perf_print_time = time.time()

            except Exception:
                # 1フレームの例外でスレッド全体を落とさない
                print("[Tracker] フレーム処理中に例外発生（継続します）:")
                traceback.print_exc()
                time.sleep(0.05)

    finally:
        print(f"[Tracker] 終了 - 合計{frame_count}フレーム処理")
        log.close()
        perf_log.close()
        cam1.release()
        cam2.release()