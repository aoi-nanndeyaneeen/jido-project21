import cv2
import numpy as np
import threading
import datetime
import time

from config import (CAMERA_ID, CAMERA_W, CAMERA_H,
                    SERIAL_PORT, SERIAL_BAUD,
                    FIELD_POINTS, LOG_DIR)
from camera import CameraTracker
from communication import SerialReceiver
from tracker import camera_thread_func
from visualizer import Visualizer3D

# スレッド間共有
plot_lock = threading.Lock()
plot_data = {"P": None, "O": None, "roll": 0.0,
             "pitch": 0.0, "current_z": 0.0, "updated": False}


def run_calibration(cam):
    """基準点4点のクリックを受け付け、solvePnP の結果を返す"""
    points_2d = []
    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points_2d) < 4:
            points_2d.append([x, y])

    cv2.setMouseCallback("Camera", on_click)
    print("【準備】基準点4箇所（左下→右下→右上→左上）をクリックしてください。")

    while True:
        ret, frame = cam.cap.read()
        if not ret:
            continue
        disp = frame.copy()
        for i, p in enumerate(points_2d):
            cv2.circle(disp, tuple(p), 5, (0, 0, 255), -1)
            cv2.putText(disp, str(i+1), (p[0]+10, p[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow("Camera", disp)
        if cv2.waitKey(1) == 13 and len(points_2d) == 4:
            break

    K = cam.get_approx_camera_matrix()
    _, rvec, tvec = cv2.solvePnP(
        FIELD_POINTS, np.array(points_2d, dtype=np.float32), K, None)
    R, _ = cv2.Rodrigues(rvec)
    cv2.destroyAllWindows()
    return K, R, tvec


def main():
    print()
    print("=" * 52)
    print("   POSITION ESTIMATOR  -  STARTUP SEQUENCE")
    print("=" * 52)
    time.sleep(0.3)

    # ── カメラ初期化 ──────────────────────────────
    print("\n[INIT 1/4]  カメラ起動中...")
    cam = CameraTracker(camera_id=CAMERA_ID, width=CAMERA_W, height=CAMERA_H)
    ret, test_frame = cam.cap.read()
    if not ret or test_frame is None:
        print("  [ERROR]  カメラ映像の取得に失敗しました。接続を確認してください。")
        return
    h, w = test_frame.shape[:2]
    print(f"  [OK]     解像度: {w} x {h}  /  カメラID: {CAMERA_ID}")
    time.sleep(0.2)

    # ── シリアル（センサ）初期化 ──────────────────
    print(f"\n[INIT 2/4]  センサ受信モジュール起動中...  ({SERIAL_PORT} @ {SERIAL_BAUD}bps)")
    alt_sensor = SerialReceiver(port=SERIAL_PORT, baudrate=SERIAL_BAUD)
    time.sleep(0.5)  # 最初のデータが来るまで少し待つ

    alt  = alt_sensor.get_altitude()
    acc  = alt_sensor.get_accel()
    if acc == (0.0, 0.0, 0.0) and alt == 0.0:
        print(f"  [WARN]   センサデータ未受信（ケーブル・COMポートを確認）")
    else:
        print(f"  [OK]     高度     : {alt:.2f} m")
        print(f"  [OK]     加速度   : Ax={acc[0]:.2f}  Ay={acc[1]:.2f}  Az={acc[2]:.2f}  [m/s²]")
    time.sleep(0.2)

    # ── 行列・ログ準備 ────────────────────────────
    print("\n[INIT 3/4]  カメラ行列・ログファイル準備中...")
    K = cam.get_approx_camera_matrix()
    print(f"  [OK]     焦点距離(近似): {K[0,0]:.0f} px")

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"flight_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"  [OK]     ログ保存先: {log_path}")
    time.sleep(0.2)

    # ── フィールドキャリブレーション ─────────────
    print("\n[INIT 4/4]  フィールドキャリブレーション待機中...")
    print("  [INFO]   カメラウィンドウで基準点4箇所をクリックしてください。")
    K, R, tvec = run_calibration(cam)
    print("  [OK]     solvePnP 完了 → カメラ姿勢を推定しました。")
    time.sleep(0.2)

    print()
    print("=" * 52)
    print("   ALL SYSTEMS GO  -  TRACKING STARTED")
    print("=" * 52)
    print()
    print("  [SPACE] キャリブレーション")
    print("  [B]     背景リセット")
    print("  [Q]     終了")
    print()

    # スレッド間共有状態
    shared = {
        "alt_offset":  0.0,
        "pos_offset":  np.zeros(3),
        "ref_roll":    0.0,
        "ref_pitch":   0.0,
        "do_calib":    False,
        "do_bg_reset": False,
        "quit":        False,
    }

    # カメラスレッド起動
    cam_thread = threading.Thread(
        target=camera_thread_func,
        args=(cam, alt_sensor, K, R, tvec, log_path, shared, plot_lock, plot_data),
        daemon=True)
    cam_thread.start()

    # 3D描画（メインスレッド）
    viz = Visualizer3D(FIELD_POINTS)
    while not shared.get("quit", False):
        with plot_lock:
            updated   = plot_data["updated"]
            P         = plot_data["P"]
            O         = plot_data["O"]
            roll      = plot_data["roll"]
            pitch     = plot_data["pitch"]
            current_z = plot_data["current_z"]
            if updated:
                plot_data["updated"] = False

        if updated and O is not None:
            viz.update(P, O, roll, pitch, current_z)
        else:
            try:
                viz.fig.canvas.flush_events()
            except Exception:
                pass

    alt_sensor.stop()
    viz.close()


if __name__ == "__main__":
    main()