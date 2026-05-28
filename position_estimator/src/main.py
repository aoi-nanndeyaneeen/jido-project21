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
                          DISP_W, DISP_H)
from core.camera import CameraTracker
from core.tracker import camera_thread_func
from ui.dashboard import Dashboard
from core.controller import AltitudeController
from core.remote_camera import RemoteCamera

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
    cam2 = RemoteCamera(RPI_HOST, RPI_PORT)
    cam2.connect()

    # 接続チェック
    # Camera1
    ret, test_frame = cam1.cap.read()
    if not ret or test_frame is None:
        print("Camera1 接続失敗")
        return

    # Camera2
    test_frame, _ = cam2.read_and_track()
    if test_frame is None:
        print("Camera2 接続失敗")
        return

    time.sleep(0.2)

    # ── [2/4] シリアル通信 (オプション) ──────────────────────
    alt_sensor = None
    if SERIAL_ENABLED:
        print(f"\n[INIT 2/4]  センサ受信モジュール起動中...  ({SERIAL_PORT} @ {SERIAL_BAUD}bps)")
        alt_sensor = SerialReceiver(port=SERIAL_PORT, baudrate=SERIAL_BAUD)
        print("  [WAIT]   データ受信待ち...", end="", flush=True)
        for _ in range(20):
            time.sleep(0.1)
            alt = alt_sensor.get_altitude()
            acc = alt_sensor.get_accel()
            if acc != (0.0, 0.0, 0.0) or alt != 0.0:
                print(" [OK]")
                break
            print(".", end="", flush=True)
        else:
            print(" [TIMEOUT]  センサデータ未受信（スキップして続行）")
    else:
        print("\n[INIT 2/4]  シリアル通信: 無効 (config.py で SERIAL_ENABLED=True にすると有効)")

    # ── [3/4] ログファイル準備 ────────────────────────────────
    print("\n[INIT 3/4]  ログファイル準備中...")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"flight_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"  [OK]    ログ保存先: {log_path}")
    time.sleep(0.2)

    # ── [4/4] 2カメラキャリブレーション ──────────────────────
    print("\n[INIT 4/4]  フィールドキャリブレーション")
    print("           ★ Camera1 と Camera2 の順に各5点をクリックします")

    try:
        print("\n  ─── Camera1 キャリブレーション ───")
        K1, R1, tvec1 = run_calibration_single(cam1, "Camera1")

        print("\n  ─── Camera2 キャリブレーション ───")
        K2, R2, tvec2 = run_calibration_remote(cam2, "Camera2")
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

    # ── メインループ ──────────────────────────────────────────
    while not shared.get("quit", False):
        with plot_lock:
            updated   = plot_data["updated"]
            P         = plot_data["P"]
            O1        = plot_data["O1"]
            current_z = plot_data["current_z"]
            residual  = plot_data["residual"]
            if updated:
                plot_data["updated"] = False

        if updated:
            # フレーム表示（両カメラ）
            with plot_lock:
                f1 = plot_data.get("frame1")
                f2 = plot_data.get("frame2")

            if f1 is not None:
                disp1 = cv2.resize(f1, (DISP_W, DISP_H))
                cv2.imshow("Camera 1", disp1)

            if f2 is not None:
                disp2 = cv2.resize(f2, (DISP_W, DISP_H))
                cv2.imshow("Camera 2", disp2)

            if O1 is not None:
                target_alt = controller.get_target()
                dashboard.render_and_show(P, O1, 0.0, 0.0, current_z, target_alt)

                if alt_sensor is not None and P is not None:
                    pitch_cmd = controller.calc_pitch_command(current_z)
                    alt_sensor.send_target_altitude(pitch_cmd)

        # waitKeyはメインスレッドで（既存のキー処理と統合）
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
    dashboard.close()
    cv2.destroyAllWindows()
    for _ in range(10):
        cv2.waitKey(1)


if __name__ == "__main__":
    main()