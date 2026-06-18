import cv2
import numpy as np
import threading
import datetime
import time
import msvcrt

from utils.config import (CAMERA_1_URL, 
                          RPI_HOST, RPI_PORT, 
                          CAMERA_W, CAMERA_H,
                          SERIAL_ENABLED, SERIAL_PORT, SERIAL_BAUD,
                          FIELD_POINTS, LOG_DIR,
                          DISP_W, DISP_H,
                          VELOCITY_W,VELOCITY_H)
from core.camera import CameraTracker
from core.tracker import camera_thread_func
from ui.dashboard import Dashboard
from core.controller import AltitudeController
from core.remote_camera import RemoteCamera
from ui.view_velocity import ViewVelocity                  
from core.geometry import accel_to_angles          
from collections import deque
from core.autopilot import SquarePatrol, RCCommand
from ui.view_rc     import ViewRC     

# ── オプション: シリアル通信 ─────────────────────────────
if SERIAL_ENABLED:
    from core.communication import SerialReceiver

# スレッド間共有
plot_lock = threading.Lock()
plot_data = {
    "P": None, "O1": None, "O2": None,
    "current_z": 0.0, "residual": None,
    "uv1": None, "uv2": None,
    "updated": False,
    "frame1": None, "frame2": None,
}


# ─────────────────────────────────────────────────────────────
#  キャリブレーション関数
# ─────────────────────────────────────────────────────────────
# camera1用
def run_calibration_single(cam: CameraTracker, cam_label: str):
    """
    指定されたカメラに対してキャリブレーションを実施する。
    画面上でFIELD_POINTSに対応する基準点5点をクリックし、
    solvePnP によりカメラ行列(K)・回転行列(R)・並進ベクトル(tvec)を返す。

    クリック順序: 左下→右下→右上→左上→左上の2m上
    """
    points_2d = []
    window_name = f"Calibration: {cam_label}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, DISP_W, DISP_H)

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points_2d) < 5:
            points_2d.append([x, y])
            print(f"  [{cam_label}] 点{len(points_2d)}/5 クリック: ({x}, {y})")

    cv2.setMouseCallback(window_name, on_click)
    print()
    print(f"  【{cam_label}】 基準点5箇所をクリックしてください")
    print(f"       順序: 左下 → 右下 → 右上 → 左上 → 左上の2m上")
    print(f"       5点クリック後に Enter を押してください")

    while True:
        ret, frame = cam.cap.read()
        if not ret or frame is None:
            continue

        disp = frame.copy()
        # クリック済みの点を描画
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
        if key == 13 and len(points_2d) == 5:   # Enter
            break
        elif key == ord('q'):
            raise RuntimeError(f"[{cam_label}] キャリブレーションがキャンセルされました。")

    K = cam.get_approx_camera_matrix()
    pts2d = np.array(points_2d, dtype=np.float32)
    ret_pnp, rvec, tvec = cv2.solvePnP(
        FIELD_POINTS, pts2d, K, None, flags=cv2.SOLVEPNP_EPNP)
    if not ret_pnp:
        raise RuntimeError(f"[{cam_label}] solvePnP が失敗しました。基準点の選択をやり直してください。")

    R, _ = cv2.Rodrigues(rvec)
    cam_pos = -R.T.dot(tvec)
    print(f"  [{cam_label}] キャリブレーション完了")
    print(f"             カメラ位置 (世界座標): X={cam_pos[0,0]:.2f}  Y={cam_pos[1,0]:.2f}  Z={cam_pos[2,0]:.2f} m")

    cv2.destroyWindow(window_name)
    for _ in range(5):
        cv2.waitKey(1)

    return K, R, tvec

#camera2用
def run_calibration_remote(remote_cam: RemoteCamera, cam_label: str):
    """
    RemoteCameraのプレビューストリームを表示しながらキャリブレーション。
    フレームはRPi側でスケールダウンされているので、
    クリック座標をフル解像度にスケールバックして使う。
    """
    points_2d = []
    window_name = f"Calibration: {cam_label}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, DISP_W, DISP_H)

    # スケール比（プレビューはSEND_PREVIEW_SCALEで縮小済み）
    PREVIEW_SCALE = 0.5  # camera_server.pyのPREVIEW_SCALEと合わせる

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points_2d) < 5:
            # プレビュー座標 → フル解像度座標に変換
            full_x = x / PREVIEW_SCALE
            full_y = y / PREVIEW_SCALE
            points_2d.append([full_x, full_y])
            print(f"  [{cam_label}] 点{len(points_2d)}/5: "
                  f"preview({x},{y}) → full({full_x:.0f},{full_y:.0f})")

    cv2.setMouseCallback(window_name, on_click)
    print(f"\n  【{cam_label}】 基準点5箇所をクリックしてください")
    print(f"       順序: 左下→右下→右上→左上→左上の2m上")

    while True:
        frame, _ = remote_cam.read_and_track()
        if frame is None:
            continue

        disp = frame.copy()
        for i, p in enumerate(points_2d):
            # フル解像度座標をプレビュースケールに変換して描画
            px = int(p[0] * PREVIEW_SCALE)
            py = int(p[1] * PREVIEW_SCALE)
            cv2.circle(disp, (px, py), 6, (0, 0, 255), -1)
            cv2.putText(disp, str(i+1), (px+10, py-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

        remaining = 5 - len(points_2d)
        status = f"Click {remaining} more pts | Enter to confirm" if remaining > 0 else "Press Enter to confirm"
        cv2.putText(disp, status, (10, disp.shape[0]-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        cv2.imshow(window_name, disp)

        key = cv2.waitKey(1) & 0xFF
        if key == 13 and len(points_2d) == 5:
            break
        elif key == ord('q'):
            raise RuntimeError(f"[{cam_label}] キャリブレーションキャンセル")

    K = remote_cam.get_approx_camera_matrix()
    pts2d = np.array(points_2d, dtype=np.float32)
    ret_pnp, rvec, tvec = cv2.solvePnP(
        FIELD_POINTS, pts2d, K, None, flags=cv2.SOLVEPNP_EPNP)
    if not ret_pnp:
        raise RuntimeError(f"[{cam_label}] solvePnP失敗")

    R, _ = cv2.Rodrigues(rvec)
    cam_pos = -R.T.dot(tvec)
    print(f"  [{cam_label}] キャリブ完了 "
          f"X={cam_pos[0,0]:.2f} Y={cam_pos[1,0]:.2f} Z={cam_pos[2,0]:.2f}m")
    cv2.destroyWindow(window_name)
    return K, R, tvec





# ─────────────────────────────────────────────────────────────
#  メイン
# ─────────────────────────────────────────────────────────────

def main():
    print()
    print("=" * 56)
    print("   POSITION ESTIMATOR  -  DUAL CAMERA STARTUP")
    print("=" * 56)
    time.sleep(0.3)

    # ── [1/4] カメラ初期化 ────────────────────────────────────
    print("\n[INIT 1/4]  カメラ起動中...")
    cam1 = CameraTracker(CAMERA_1_URL, width=CAMERA_W, height=CAMERA_H, label="Camera1")
    cam2 = None

    # Camera1 接続チェック
    ret, test_frame = cam1.cap.read()
    if not ret or test_frame is None:
        print("  [WARN] Camera1 映像取得失敗。ダミーモードで続行します。")

    # Camera2 接続チェック（失敗してもスキップ）
    try:
        cam2 = RemoteCamera(RPI_HOST, RPI_PORT)
        cam2.connect()
        test_frame2, _ = cam2.read_and_track()
        if test_frame2 is None:
            print("  [WARN] Camera2 フレーム取得失敗。ダミーモードで続行します。")
    except Exception as e:
        print(f"  [WARN] Camera2 接続失敗: {e}")
        print("         ダミーモードで続行します。")
        cam2 = None

    # Camera2がNoneの場合はダミー用スタブを作成
    if cam2 is None:
        from core.remote_camera import RemoteCamera as _RC
        class _DummyCam2:
            width, height = 1280, 720
            def read_and_track(self): return None, None
            def reset_background(self): pass
            def release(self): pass
            def get_approx_camera_matrix(self):
                import numpy as np
                f = self.width
                return np.array([[f,0,self.width/2],[0,f,self.height/2],[0,0,1]],
                                dtype=np.float32)
        cam2 = _DummyCam2()
        print("  [INFO] Camera2 スタブを使用します。")

    time.sleep(0.2)


    # ── [2/4] シリアル通信（自動検出・失敗時はスキップ） ─────────
    alt_sensor = None
    print(f"\n[INIT 2/4]  センサ接続試行中...  ({SERIAL_PORT} @ {SERIAL_BAUD}bps)")

    if SERIAL_ENABLED:
        from core.communication import SerialReceiver
        alt_sensor = SerialReceiver(port=SERIAL_PORT, baudrate=SERIAL_BAUD)

        if not alt_sensor.is_running:
            # ポートが開けなかった（デバイス未接続など）
            print("  [SKIP] ポートを開けませんでした。センサなしで続行します。")
            alt_sensor = None
        else:
            # データが来るまで最大2秒待つ
            print("  [WAIT] データ受信待ち...", end="", flush=True)
            for _ in range(20):
                time.sleep(0.1)
                if (alt_sensor.get_accel() != (0.0, 0.0, 0.0)
                        or alt_sensor.get_altitude() != 0.0):
                    print(" [OK]")
                    break
                print(".", end="", flush=True)
            else:
                print(" [TIMEOUT] データ未受信。センサなしで続行します。")
                alt_sensor.stop()
                alt_sensor = None
    else:
        print("  [SKIP] SERIAL_ENABLED=False のためスキップ")

    # ── [3/4] ログファイル準備 ────────────────────────────────
    print("\n[INIT 3/4]  ログファイル準備中...")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"flight_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"  [OK]    ログ保存先: {log_path}")
    time.sleep(0.2)

    # ── [4/4] キャリブレーション ──────────────────────────────
    print("\n[INIT 4/4]  フィールドキャリブレーション")

    def dummy_calib(label, cam_pos_xyz):
        """ダミーカメラ行列（位置だけ指定した簡易キャリブ）"""
        import numpy as np
        K = np.array([[1280, 0, 640],[0, 1280, 360],[0, 0, 1]],
                     dtype=np.float32)
        # カメラ位置からtvec/Rを逆算
        cam_pos = np.array(cam_pos_xyz, dtype=np.float64).reshape(3,1)
        R = np.eye(3, dtype=np.float64)
        tvec = -R @ cam_pos
        print(f"  [DUMMY] {label} ダミーキャリブ使用 pos={cam_pos_xyz}")
        return K, R, tvec

    cam1_ok = cam1.cap.isOpened() and cam1.width > 0
    cam2_ok = hasattr(cam2, 'sock')   # RemoteCameraはsockを持つ

    try:
        if cam1_ok:
            print("\n  ─── Camera1 キャリブレーション ───")
            K1, R1, tvec1 = run_calibration_single(cam1, "Camera1")
        else:
            K1, R1, tvec1 = dummy_calib("Camera1", [-8, -8, 2])

        if cam2_ok:
            print("\n  ─── Camera2 キャリブレーション ───")
            K2, R2, tvec2 = run_calibration_remote(cam2, "Camera2")
        else:
            K2, R2, tvec2 = dummy_calib("Camera2", [8, -8, 2])

    except RuntimeError as e:
        print(f"\n  [ERROR] {e}")
        cam1.release(); cam2.release()
        return

    print()
    print("=" * 56)
    print("   ALL SYSTEMS GO  -  STEREO TRACKING STARTED")
    print("=" * 56)
    print()
    print("  [B]     背景リセット (両カメラ)")
    print("  [Q]     終了")
    print()

    # スレッド間共有状態
    shared = {
        "do_bg_reset": False,
        "quit":        False,
    }

    # ── カメラウィンドウをメインスレッドで作成 ────────────────
    cv2.namedWindow("Camera 1", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera 1", DISP_W, DISP_H)
    cv2.moveWindow("Camera 1", 0, 0)

    cv2.namedWindow("Camera 2", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera 2", DISP_W, DISP_H)
    cv2.moveWindow("Camera 2", DISP_W + 10, 0)   # 横に並べる

    # ── ターミナルフォーカス時のキー入力スレッド ─────────────
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

    # ── カメラスレッド起動 ─────────────────────────────────────
    cam_thread = threading.Thread(
        target=camera_thread_func,
        args=(cam1, cam2,
              K1, R1, tvec1,
              K2, R2, tvec2,
              log_path, shared, plot_lock, plot_data),
        daemon=True)    
    cam_thread.start()

    # ── ダッシュボード準備 ────────────────────────────────────
    dashboard = Dashboard(FIELD_POINTS)
    controller = AltitudeController(p_gain=5.0)
    velocity_view = ViewVelocity(FIELD_POINTS)
    patrol   = SquarePatrol(start_x=0.0, start_y=0.0)
    view_rc  = ViewRC()

    cv2.namedWindow("RC Command", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("RC Command", ViewRC.W, ViewRC.H)
    cv2.moveWindow("RC Command", VELOCITY_W + 640, DISP_H + 40)
    # 初期表示（灰色ではなく黒で起動）
    cv2.imshow("RC Command", np.zeros((ViewRC.H, ViewRC.W, 3), dtype=np.uint8))

    # 速度推定用の位置履歴（main.py内で簡易計算）
    _pos_hist  = deque(maxlen=6)   # (time, np.array)
    _last_cmd  = RCCommand()       # 最後のコマンド（未検出時に保持）
    _ap_time   = time.time()       # オートパイロット前回呼出し時刻

    cv2.namedWindow("Velocity", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Velocity", VELOCITY_W, VELOCITY_H)
    cv2.moveWindow("Velocity", 0, DISP_H + 40)

    # ── メインループ ──────────────────────────────────────────
    last_mpl_render = 0.0
    MPL_RENDER_HZ   = 5          # matplotlibは最大5fps

    while not shared.get("quit", False):
        with plot_lock:
            updated   = plot_data["updated"]
            P         = plot_data["P"]
            O1        = plot_data["O1"]
            current_z = plot_data["current_z"]
            if updated:
                plot_data["updated"] = False

        if updated:
            # カメラ映像はフルレートで表示
            with plot_lock:
                f1 = plot_data.get("frame1")
                f2 = plot_data.get("frame2")
            if f1 is not None:
                cv2.imshow("Camera 1", cv2.resize(f1, (DISP_W, DISP_H)))
            if f2 is not None:
                cv2.imshow("Camera 2", cv2.resize(f2, (DISP_W, DISP_H)))

            # matplotlibビューはレートを制限
            now = time.time()
            if now - last_mpl_render > 1.0 / MPL_RENDER_HZ:
                last_mpl_render = now

                roll_deg, pitch_deg = 0.0, 0.0
                if alt_sensor is not None:
                    roll_deg, pitch_deg = accel_to_angles(alt_sensor.get_accel())

                vel_img = velocity_view.get_image(P, roll_deg, pitch_deg)
                cv2.imshow("Velocity", vel_img)

                # ── オートパイロット計算 ──────────────────────────────
                now_ap = time.time()
                dt_ap  = now_ap - _ap_time
                _ap_time = now_ap

                if P is not None:
                    _pos_hist.append((now_ap, P.copy()))

                # 速度推定（posHistから有限差分）
                vel_ap = np.zeros(3)
                if len(_pos_hist) >= 2:
                    t0, p0 = _pos_hist[0]
                    t1, p1 = _pos_hist[-1]
                    dt_v = t1 - t0
                    if dt_v > 1e-4:
                        vel_ap = (p1 - p0) / dt_v

                if P is not None and dt_ap > 0:
                    _last_cmd = patrol.update(
                        pos=P,
                        vel=vel_ap,
                        dt=dt_ap
                    )

                rc_img = view_rc.get_image(_last_cmd)
                cv2.imshow("RC Command", rc_img)

                if O1 is not None:
                    target_alt = controller.get_target()
                    dashboard.render_and_show(P, O1, 0.0, 0.0, current_z, target_alt)
                    if alt_sensor is not None and P is not None:
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

    # ── 終了処理 ──────────────────────────────────────────────
    if alt_sensor is not None:
        alt_sensor.stop()
    view_rc.close() if hasattr(view_rc, 'close') else None
    dashboard.close()
    velocity_view.close()
    cv2.destroyAllWindows()
    for _ in range(10):
        cv2.waitKey(1)


if __name__ == "__main__":
    main()