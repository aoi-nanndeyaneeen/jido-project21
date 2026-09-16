"""
main.py  -  本番のエントリーポイント (離陸から着陸まで全自動)

==========================================================================
本番の手順 (準備 1 分 / 飛行 4 分)
==========================================================================
準備が始まる前:
    機体にバッテリーを繋ぎ、プロポの電源を入れて待機。THR_CUT は入れたまま。
    機体は離着陸エリアに、**機首を config.YAW_INITIAL_ALIGN_DEG の向き**
    (既定 90deg = フィールド右 +x) へ向けて置く。

準備時間 (1 分):
    1. ラズパイ:  python camera_server.py
    2. こちら  :  python main.py            (どちらが先でもよい。PC 側が待つ)
    3. 2 人で同時にフィールドの基準 5 点をクリック
       (PC は Camera1 の画面、ラズパイはラズパイの画面。順序は画面の指示どおり)
    4. 追跡が始まり、飛行プログラムの一覧とフェンスの検算が表示される

飛行 (4 分。目標は 3 分で完了):
    5. プロポの THR_CUT を解除してアーム
       -> PC が自動でミッション待機に入り、REQ_HOLD を送り始める
    6. SW_HOVER を GUIDED (上) にして、スロットルを 15% 以上へ
       -> 機体が GUIDED に入った瞬間に競技時計が 0 から走り、離陸
    7. あとは全自動: 離陸 -> 水平旋回 -> 上昇旋回 -> 8の字 -> 帰投 -> 着陸 -> 静止判定
       ★ ここから先は PC に触らない。中断は [X]、緊急停止はプロポ。

    飛ぶ順番・時間配分・機動の半径は core/program.py と utils/config.py の COMP_*。

==========================================================================
起動オプション
==========================================================================
    python main.py                    本番 (キャリブは毎回測り直す)
    python main.py --calib saved      保存済みの外部パラメータを使う (練習用)
    python main.py --calib ask        毎回 y/n を聞く (従来の挙動)
    python main.py --no-wait-rpi      ラズパイを待たない (Camera1 だけの確認用)
"""

import argparse
import datetime
import time

from utils.config import LOG_DIR, FIELD_POINTS
from init.camera_setup import init_cameras
from init.calibration_flow import run_calibration_phase
from app.main_loop import run_main_loop


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="ステレオ追跡 + 自動飛行 (離陸から着陸まで)")
    ap.add_argument("--calib", choices=("fresh", "saved", "ask"), default="fresh",
                    help="外部パラメータ: fresh=毎回5点を測り直す (既定・本番) / "
                         "saved=保存済みを使う / ask=毎回 y/n を聞く")
    ap.add_argument("--no-wait-rpi", action="store_true",
                    help="ラズパイの立ち上がりを待たない (Camera1 だけで動かすとき)")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    print()
    print("=" * 56)
    print("   POSITION ESTIMATOR  -  DUAL CAMERA STARTUP")
    print("=" * 56)

    cam1, cam2_ok, cam2 = init_cameras(interactive=not args.no_wait_rpi)
    time.sleep(0.2)

    print("\n[INIT 2/3]  ログファイル準備中...")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"flight_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"  [OK]    ログ保存先: {log_path}")

    cam1_ok = cam1.cap.isOpened() and cam1.width > 0
    try:
        calib1, calib2, cam2 = run_calibration_phase(
            cam1, cam1_ok, cam2_ok, cam2, calib_mode=args.calib)
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
