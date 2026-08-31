"""
app/main_loop.py
メインループ本体：ウィンドウ管理・カメラスレッド起動・
オートパイロット計算・各種可視化ウィンドウの更新を担当。
"""

import cv2
import math
import numpy as np
import threading
import time
import msvcrt
from collections import deque

from utils.config import (DISP_W, DISP_H, VELOCITY_W, VELOCITY_H,
                          YAW_ENABLED, YAW_INITIAL_ALIGN_DEG,
                          YAW_SEND_HZ, YAW_SEND_INITIAL_ALIGN)
from core.tracker import camera_thread_func
from core.controller import AltitudeController
from core.geometry import accel_to_angles
from core.yaw_estimator import YawEstimator
from core.autopilot import SquarePatrol, HoverHold, WaypointMission, RCCommand
from ui.dashboard import Dashboard
from ui.view_velocity import ViewVelocity
from ui.view_rc import ViewRC


def run_main_loop(cam1, cam2,
                  calib1, calib2,
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
        "in_dummy": False, "tracking_ok": False, "frame_time": 0.0,
    }

    # ── ヨー方位推定 (YAW_HANDOFF.md) ───────────────────────
    yaw_est = YawEstimator() if YAW_ENABLED else None
    last_yaw_send = 0.0
    if yaw_est is not None:
        print(f"[Yaw] 推定を有効化 (初期アラインメント "
              f"{YAW_INITIAL_ALIGN_DEG:+.1f}deg = 機首をフィールド奥+yへ)")
        if alt_sensor is None:
            print("[Yaw] [WARN] シリアル未接続のため機体Δvが届きません。"
                  "ヨーは初期アラインメント値のまま固定されます。")

    print()
    print("=" * 56)
    print("   ALL SYSTEMS GO  -  STEREO TRACKING STARTED")
    print("=" * 56)
    print()
    print("  [B]     背景リセット (両カメラ)")
    print("  [H]     ホバリングモード")
    print("  [W]     waypoint飛行モード (今後実装予定 → 現状はホバリングにフォールバック)")
    print("  [C]     本番(競技)モード")
    print("  [Q]     終了")
    print()

    shared = {"do_bg_reset": False, "quit": False, "mode_request": None}

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
                elif key == b'h':
                    print("[KEY] H → ホバリングモード要求")
                    shared["mode_request"] = "hover"
                elif key == b'w':
                    print("[KEY] W → waypoint飛行モード要求")
                    shared["mode_request"] = "waypoint"
                elif key == b'c':
                    print("[KEY] C → 本番(競技)モード要求")
                    shared["mode_request"] = "competition"
            time.sleep(0.05)

    threading.Thread(target=keyboard_thread, daemon=True).start()

    # ── カメラスレッド起動 ──────────────────────────────────
    cam_thread = threading.Thread(
        target=camera_thread_func,
        args=(cam1, cam2, calib1, calib2,
              log_path, shared, plot_lock, plot_data),
        daemon=True)
    cam_thread.start()

    # ── 各ビュー初期化 ──────────────────────────────────────
    dashboard      = Dashboard(field_points)
    controller     = AltitudeController(p_gain=5.0)
    velocity_view  = ViewVelocity(field_points)
    autopilot_mode = "hover"                    # 起動直後は最も安全なホバリングから開始
    patrol         = HoverHold()
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
            updated     = plot_data["updated"]
            P           = plot_data["P"]
            O1          = plot_data["O1"]
            current_z   = plot_data["current_z"]
            tracking_ok = plot_data.get("tracking_ok", False)
            frame_time  = plot_data.get("frame_time", 0.0)
            in_dummy    = plot_data.get("in_dummy", False)
            if updated:
                plot_data["updated"] = False

        # ── ヨー推定への入力 ────────────────────────────────
        # カメラ位置は毎フレーム、機体Δvは届いたぶんをまとめて渡す。
        # 推定器は単一スレッドで扱う（受信スレッドはキューに積むだけ）。
        if yaw_est is not None:
            if updated and frame_time > 0.0:
                yaw_est.add_position(frame_time, P, tracking_ok)
            if alt_sensor is not None:
                body_yaw = alt_sensor.get_body_yaw()
                for (t_ms, dvx, dvy, t_recv) in alt_sensor.drain_body_dv():
                    yaw_est.add_body_dv(t_ms, dvx, dvy, t_recv, body_yaw)

        if updated:
            with plot_lock:
                f1 = plot_data.get("frame1")
                f2 = plot_data.get("frame2")
            if f1 is not None:
                disp1 = cv2.resize(f1, (DISP_W, DISP_H))
                if yaw_est is not None:
                    ye = yaw_est.current()
                    cv2.putText(disp1, yaw_est.status_line(), (10, DISP_H - 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (120, 255, 120) if ye.valid else (140, 140, 140), 2)
                cv2.imshow("Camera 1", disp1)
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

            # ── モード切替要求の反映（H/W/C キー） ──────────────
            mode_req = shared.get("mode_request")
            if mode_req is not None:
                shared["mode_request"] = None
                if mode_req != autopilot_mode:
                    patrol.close()
                    if mode_req == "hover":
                        patrol = HoverHold(pos=P)
                    elif mode_req == "waypoint":
                        patrol = WaypointMission(pos=P)
                    elif mode_req == "competition":
                        # 本番のウェイポイントはフィールド固定座標なので原点基準で生成する
                        patrol = SquarePatrol(start_x=0.0, start_y=0.0)
                    autopilot_mode = mode_req
                    print(f"[Mode] → {autopilot_mode}")

            # ── ヨー推定を1ステップ進める ───────────────────
            # ★ ここでの update() は1ループ1回だけ。参照は current() を使う
            yaw_rad = None
            if yaw_est is not None:
                ye = yaw_est.update(now_ap)
                if ye.valid:
                    yaw_rad = math.radians(ye.yaw_deg)

            if P is not None and dt_ap > 0:
                _last_cmd = patrol.update(pos=P, vel=vel_ap, dt=dt_ap,
                                          heading_rad=yaw_rad,
                                          is_dummy=in_dummy)

            # ── 自律制御コマンドをground_receiver経由でドローンへ送信 ──
            if alt_sensor is not None:
                alt_sensor.send_autopilot_command(_last_cmd)

                # ヨーは 0.5〜1Hz でよい。速くしても通信ジッタが姿勢に乗るだけ
                if yaw_est is not None and now_ap - last_yaw_send >= 1.0 / YAW_SEND_HZ:
                    last_yaw_send = now_ap
                    ye = yaw_est.current()
                    if ye.valid:
                        alt_sensor.send_yaw(ye.yaw_deg, True)
                    elif YAW_SEND_INITIAL_ALIGN:
                        # 未収束のあいだは初期アラインメント値を valid=0 で送る。
                        # 機体側は valid=0 を完全に無視すること
                        alt_sensor.send_yaw(YAW_INITIAL_ALIGN_DEG, False)

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
    patrol.close()
    if alt_sensor is not None:
        alt_sensor.stop()
    dashboard.close()
    velocity_view.close()
    cv2.destroyAllWindows()
    for _ in range(10):
        cv2.waitKey(1)