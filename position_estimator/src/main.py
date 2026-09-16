import time
import datetime

from utils.config import LOG_DIR, FIELD_POINTS
from init.camera_setup import init_cameras
from init.calibration_flow import run_calibration_phase
from app.main_loop import run_main_loop


def main():
    print()
    print("=" * 56)
    print("   POSITION ESTIMATOR  -  DUAL CAMERA STARTUP")
    print("=" * 56)
    time.sleep(0.3)

    cam1, cam2_ok_rpi, cam2 = init_cameras()
    time.sleep(0.2)

    print("\n[INIT 2/3]  ログファイル準備中...")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"flight_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"  [OK]    ログ保存先: {log_path}")
    time.sleep(0.2)

    cam1_ok = cam1.cap.isOpened() and cam1.width > 0
    try:
        calib1, calib2, cam2 = run_calibration_phase(
            cam1, cam1_ok, cam2_ok_rpi, cam2
        )
    except RuntimeError as e:
        print(f"\n  [ERROR] {e}")
        cam1.release()
        cam2.release()
        return

    try:
        run_main_loop(cam1, cam2, calib1, calib2, log_path, FIELD_POINTS)
    finally:
        for cam in (cam1, cam2):
            try:
                cam.release()
            except Exception as e:
                print(f"  [WARN] カメラ解放に失敗: {e}")


if __name__ == "__main__":
    import os
    import sys
    import traceback
    exit_code = 0
    try:
        main()
    except BaseException:
        traceback.print_exc()
        exit_code = 1
    # ★ 後始末は main() で済んでいる。Windows ではカメラ読み込み中の daemon
    #   スレッドがあるとインタプリタ終了が固まり、カメラ/COM を掴んだまま
    #   プロセスが残ることがあった。ログは各行 flush 済みなので強制終了してよい。
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)