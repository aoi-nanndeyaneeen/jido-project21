"""
logger.py  -  RP2040地上局のシリアル出力からフライトデータをCSVに記録する

使い方:
    python logger.py               # COMポートを自動検出
    python logger.py --port COM7   # ポートを指定
    python logger.py --port COM7 --baud 115200

ログ保存先: logs/YYYYMMDD_HHMMSS.csv
終了: Ctrl+C

記録後はそのまま plot_3d.py に渡せます:
    python plot_3d.py logs/20250330_123456.csv
"""

import serial
import serial.tools.list_ports
import csv
import os
import re
import time
import argparse
from datetime import datetime

# ============================================================
#  パース対象の行フォーマット（main_pc.cpp の print_MPU / Alt 出力）
#  例:
#    Roll_Ang:  -3.37,Pitch_Ang:  -8.62,Yaw_Ang:+130.95,...
#    Alt:  +14.68 m
# ============================================================
RE_MPU = re.compile(
    r"Roll_Ang:\s*([+-]?\d+\.\d+).*?Pitch_Ang:\s*([+-]?\d+\.\d+).*?Yaw_Ang:\s*([+-]?\d+\.\d+)"
)
RE_ALT = re.compile(r"Alt:\s*([+-]?\d+\.\d+)\s*m")

VID_RP2040 = 0x2E8A

def find_port():
    """RP2040らしいポートを自動検出"""
    ports = list(serial.tools.list_ports.comports())

    # 1. VID で判定する。説明文は Windows の表示言語で変わる (日本語だと
    #    「USB シリアル デバイス」) ので、当てにしない。
    for p in ports:
        if p.vid == VID_RP2040:
            return p.device

    # 2. 説明文でのフォールバック (英語/日本語の両方を見る)
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ["xiao", "rp2040", "pico", "usb serial", "usb シリアル"]):
            return p.device

    # 3. 見つからないときに先頭を掴むと、Bluetooth の仮想COMを開いて
    #    "semaphore timeout" でハングする。勝手に選ばず --port を促す。
    print("自動検出できませんでした。--port COMx で指定してください。利用可能なポート:")
    for p in ports:
        vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "    -    "
        print(f"  {p.device}  {vidpid}  {p.description}")
    return None

def main():
    parser = argparse.ArgumentParser(description="Flight Data Logger")
    parser.add_argument("--port", default=None, help="COMポート (例: COM7 / /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    port = args.port or find_port()
    if port is None:
        print("エラー: シリアルポートが見つかりません。")
        return

    # ログ保存先: このスクリプトと同じフォルダ(tools/)の下に logs/ を作る
    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(script_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)
    fname = os.path.join(log_dir, datetime.now().strftime("%Y%m%d_%H%M%S.csv"))

    print(f"ポート: {port}  ボーレート: {args.baud}")
    print(f"保存先: {fname}")
    print("記録開始... 終了は Ctrl+C\n")

    start_time = time.time()
    pending = {}   # 1フレーム分のデータを一時保持

    with serial.Serial(port, args.baud, timeout=1) as ser, \
         open(fname, "w", newline="", encoding="utf-8") as f:

        writer = csv.writer(f)
        writer.writerow(["Time", "Roll(deg)", "Pitch(deg)", "Yaw(deg)", "Alt(m)",
                         "ax(g)", "ay(g)", "az(g)"])
        f.flush()

        record_count = 0
        try:
            while True:
                raw = ser.readline()
                try:
                    line = raw.decode("utf-8", errors="replace").strip()
                except Exception:
                    continue

                # Roll/Pitch/Yaw を検出
                m = RE_MPU.search(line)
                if m:
                    pending["roll"]  = float(m.group(1))
                    pending["pitch"] = float(m.group(2))
                    pending["yaw"]   = float(m.group(3))

                # Alt を検出 → 1レコード確定
                m = RE_ALT.search(line)
                if m and "roll" in pending:
                    pending["alt"] = float(m.group(1))
                    t = round(time.time() - start_time, 3)
                    writer.writerow([
                        t,
                        pending.get("roll",  0.0),
                        pending.get("pitch", 0.0),
                        pending.get("yaw",   0.0),
                        pending.get("alt",   0.0),
                        pending.get("ax",    0.0),
                        pending.get("ay",    0.0),
                        pending.get("az",    0.0),
                    ])
                    f.flush()
                    record_count += 1
                    pending.clear()
                    print(f"\r{record_count} レコード記録中...  t={t:.1f}s  "
                      f"Roll={pending.get('roll',0):+6.1f}  "
                      f"Pitch={pending.get('pitch',0):+6.1f}  "
                      f"Alt={pending.get('alt',0):+6.2f}m", end="")

        except KeyboardInterrupt:
            print(f"\n\n終了。{record_count} レコードを {fname} に保存しました。")
            print(f"グラフ表示: python tools/plot_3d.py \"{fname}\"")

if __name__ == "__main__":
    main()