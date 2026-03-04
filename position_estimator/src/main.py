import cv2
import numpy as np
import threading
import os

# 分離した自作モジュールのインポート
from camera import CameraTracker
from communication import SerialReceiver
from visualizer import Visualizer3D

# ==========================================
# 設定
# ==========================================
FIELD_POINTS = np.array([
    [0.0,  0.0,  0.0],
    [16.0, 0.0,  0.0],
    [16.0, 16.0, 0.0],
    [0.0,  16.0, 0.0]
], dtype=np.float32)

# 外れ値フィルタ：この範囲外の検知は無視する（フィールドより少し広め）
VALID_X_MIN, VALID_X_MAX = -5.0, 21.0
VALID_Y_MIN, VALID_Y_MAX = -5.0, 21.0
VALID_Z_MIN, VALID_Z_MAX = -1.0, 20.0

# スレッド間共有データ
plot_lock = threading.Lock()
plot_data = {
    "P": None, "O": None,
    "roll": 0.0, "pitch": 0.0, "current_z": 0.0,
    "updated": False,
}

# ==========================================
# 計算関数
# ==========================================
def get_ray(u, v, K, R, tvec):
    pt_2d = np.array([[[u, v]]], dtype=np.float32)
    undistorted = cv2.undistortPoints(pt_2d, K, None)
    dir_cam = np.array([undistorted[0][0][0], undistorted[0][0][1], 1.0])
    cam_pos = -R.T.dot(tvec).flatten()
    dir_world = R.T.dot(dir_cam)
    dir_world /= np.linalg.norm(dir_world)
    return cam_pos, dir_world.flatten()

def accel_to_angles(accel):
    ax_, ay, az = accel
    norm = np.sqrt(ax_**2 + ay**2 + az**2)
    if norm < 1e-6:
        return 0.0, 0.0
    ax_, ay, az = ax_/norm, ay/norm, az/norm
    pitch = np.degrees(np.arctan2(-ax_, np.sqrt(ay**2 + az**2)))
    roll  = np.degrees(np.arctan2(ay, az))
    return roll, pitch

def calc_tilt(accel, ref_roll, ref_pitch):
    roll, pitch = accel_to_angles(accel)
    return roll - ref_roll, pitch - ref_pitch

def is_valid_position(P):
    return (VALID_X_MIN <= P[0] <= VALID_X_MAX and
            VALID_Y_MIN <= P[1] <= VALID_Y_MAX and
            VALID_Z_MIN <= P[2] <= VALID_Z_MAX)

# ==========================================
# OpenCVスレッド（カメラ・検知・ログ）
# ==========================================
# ※以前の main.py にあった camera_thread_func の中身と draw_horizon はここに入ります。
# （長くなるため中略しますが、前回のコードのまま変更不要です。ログの保存先パスだけ渡す時に変えます）
# ... (camera_thread_func の実装) ...

# ==========================================
# エントリポイント
# ==========================================
if __name__ == "__main__":
    print("システムを起動中...")

    # ログフォルダがなければ作成
    if not os.path.exists("logs"):
        os.makedirs("logs")

    cam        = CameraTracker(camera_id=0, width=1920, height=1080)
    alt_sensor = SerialReceiver(port="COM7", baudrate=115200)
    K          = cam.get_approx_camera_matrix()

    shared = {
        "points_2d":  [],
        "alt_offset": 0.0,
        "pos_offset": np.zeros(3),
        "ref_roll":   0.0,
        "ref_pitch":  0.0,
        "do_calib":   False,
        "do_bg_reset":False,
        "quit":       False,
    }

    # --- 基準点クリック ---
    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
    def on_click_init(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(shared["points_2d"]) < 4:
            shared["points_2d"].append([x, y])
    cv2.setMouseCallback("Camera", on_click_init)
    print("【準備】基準点4箇所（左下→右下→右上→左上）をクリックしてください。")

    while True:
        ret, frame = cam.cap.read()
        if not ret: continue
        disp = frame.copy()
        for i, p in enumerate(shared["points_2d"]):
            cv2.circle(disp, tuple(p), 5, (0,0,255), -1)
            cv2.putText(disp, str(i+1), (p[0]+10,p[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
        cv2.imshow("Camera", disp)
        if cv2.waitKey(1) == 13 and len(shared["points_2d"]) == 4:
            break

    success, rvec, tvec = cv2.solvePnP(FIELD_POINTS, np.array(shared["points_2d"], dtype=np.float32), K, None)
    R, _ = cv2.Rodrigues(rvec)
    cv2.destroyAllWindows()

    print("【開始】トラッキング開始！")

    # スレッド起動（ログの保存先を logs/ 内に変更）
    log_file_path = os.path.join("logs", "3d_tracking_log.csv")
    cam_thread = threading.Thread(
        target=camera_thread_func, # 上部で定義した関数
        args=(cam, alt_sensor, K, R, tvec, log_file_path, shared),
        daemon=True)
    cam_thread.start()

    # --- ここからが Visualizer クラスを使ったスッキリしたループ ---
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

    # 終了処理
    alt_sensor.stop()
    viz.close()