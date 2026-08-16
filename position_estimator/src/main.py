import time
import datetime

from utils.config import SERIAL_ENABLED, SERIAL_PORT, SERIAL_BAUD, LOG_DIR, FIELD_POINTS
from init.camera_setup import init_cameras
from init.calibration_flow import run_calibration_phase
from app.main_loop import run_main_loop


def init_serial():
    print(f"\n[INIT 2/4]  センサ接続試行中...  ({SERIAL_PORT} @ {SERIAL_BAUD}bps)")
    if not SERIAL_ENABLED:
        print("  [SKIP] SERIAL_ENABLED=False のためスキップ")
        return None

    from core.communication import SerialReceiver
    alt_sensor = SerialReceiver(port=SERIAL_PORT, baudrate=SERIAL_BAUD)
    if not alt_sensor.is_running:
        print("  [SKIP] ポートを開けませんでした。センサなしで続行します。")
        return None

    print("  [WAIT] データ受信待ち...", end="", flush=True)
    for _ in range(20):
        time.sleep(0.1)
        if alt_sensor.get_accel() != (0.0, 0.0, 0.0) or alt_sensor.get_altitude() != 0.0:
            print(" [OK]")
            return alt_sensor
        print(".", end="", flush=True)
    print(" [TIMEOUT] データ未受信。センサなしで続行します。")
    alt_sensor.stop()
    return None


def main():
    print()
    print("=" * 56)
    print("   POSITION ESTIMATOR  -  DUAL CAMERA STARTUP")
    print("=" * 56)
    time.sleep(0.3)

    cam1, cam2_ok_rpi, cam2 = init_cameras()
    time.sleep(0.2)

    alt_sensor = init_serial()

    print("\n[INIT 3/4]  ログファイル準備中...")
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

    run_main_loop(cam1, cam2, calib1, calib2,
                  log_path, alt_sensor, FIELD_POINTS)


if __name__ == "__main__":
    main()