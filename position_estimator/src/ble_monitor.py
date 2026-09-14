"""
ble_monitor.py  -  log_recorder (XIAO ESP32C3) の BLE ログを
                    Arduino IDE のシリアルモニタみたいに流しっぱなしで見る。

    機体 (drone_s5) ──UART──> log_recorder XIAO ──BLE 125Hz──> ここ ──> 画面

console.py の画面には BLE の最新値が 1 行だけ (上書きで) 出ているが、
「動かしながら値の推移をそのまま目で追いたい」ときはそれでは物足りない。
これは core.ble_tap.BleTap の decode 結果を間引かずそのまま 1 行ずつ
流すだけの、デバッグ専用の薄いラッパー。

★ 新しい無線プロトコルは何も足していない。BLE 経由で既に受け取っている
  FlightLog::Rec (bin2csv.py が解釈しているのと同じバイナリ) を、
  ここで文字列にしているだけ。console.py / main.py と同時に起動しても
  構わない (BLE 受信は Notify なので複数クライアントが繋がっても平気)。
  ただし地上局 (IM920) の COM ポートは触らないので、console.py / main.py
  の代わりにはならない。

使い方:
    cd position_estimator/src
    python ble_monitor.py                  # 既定の名前 (S5-LogBLE) を自動で探す
    python ble_monitor.py --name S5-LogBLE
    python ble_monitor.py --save            # ついでに .BIN も残す (既定は残さない)
"""

import argparse
import time
from pathlib import Path

from core.ble_tap import BleTap
from utils.config import LOG_DIR


def main():
    ap = argparse.ArgumentParser(
        description="BLE 経由の機体ログをシリアルモニタ風に流し続ける (デバッグ用)")
    ap.add_argument("--name", default="S5-LogBLE", help="BLE デバイス名")
    ap.add_argument("--save", action="store_true",
                    help="表示だけでなく .BIN にも保存する (既定: 保存しない)")
    ap.add_argument("--log-dir", default=str(LOG_DIR),
                    help="--save のときの保存先 (既定: position_estimator/src/logs)")
    args = ap.parse_args()

    outdir = Path(args.log_dir) if args.save else Path(".ble_monitor_tmp")

    tap = BleTap(outdir, name=args.name, on_event=print, verbose=True)
    if not tap.start():
        print("[ble_monitor] 起動に失敗しました (上のメッセージを参照)")
        return

    print(f"[ble_monitor] '{args.name}' を探しています... (Ctrl+C で終了)")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        tap.stop()
        if not args.save:
            # --save 無しのときは一時 .BIN を残さない (中身は画面に出し切っている)。
            for f in outdir.glob("LOG*.BIN"):
                try:
                    f.unlink()
                except OSError:
                    pass
            try:
                outdir.rmdir()
            except OSError:
                pass
        print("[ble_monitor] 終了しました")


if __name__ == "__main__":
    main()
