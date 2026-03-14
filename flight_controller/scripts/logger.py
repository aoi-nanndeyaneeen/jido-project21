# scripts/logger.py
# Teensyのシリアル出力を受け取り、logsフォルダにCSVを保存する
#
# 【使い方】
#   1. Teensyを接続してアップロード済みにする
#   2. ターミナルで:  python scripts/logger.py
#   3. Teensyのシリアルモニタ(VSCode)は閉じておく (COMポートを占有するため)
#   4. TeensyのシリアルからLキーを押す代わりに、
#      このスクリプトの入力に "L" + Enter で開始/停止できる
#
# 【必要パッケージ】
#   pip install pyserial

import serial
import serial.tools.list_ports
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# ============================================================
#  設定
# ============================================================
BAUD_RATE   = 115200
LOGS_DIR    = Path(__file__).parent.parent / "logs"   # プロジェクトルート/logs/
AUTO_DETECT = True   # Trueにすると自動でTeensyのCOMポートを探す
MANUAL_PORT = "COM14" # AUTO_DETECT=FalseのときにここのCOMポートを使う

# ============================================================
#  COMポート自動検出
# ============================================================
def find_teensy_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        mfr  = (p.manufacturer or "").lower()
        if "teensy" in desc or "teensy" in mfr or "usb serial" in desc:
            return p.device
    # 見つからなければ一覧を表示して選ばせる
    print("Teensyが自動検出できませんでした。接続中のCOMポート一覧:")
    for i,p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    idx = input("番号を選択 > ").strip()
    return ports[int(idx)].device

# ============================================================
#  ファイル名の自動採番
# ============================================================
def new_log_path():
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    existing = sorted(LOGS_DIR.glob("log_*.csv"))
    idx = len(existing) + 1
    ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
    return LOGS_DIR / f"log_{idx:03d}_{ts}.csv"

# ============================================================
#  メイン
# ============================================================
def main():
    port = find_teensy_port() if AUTO_DETECT else MANUAL_PORT
    print(f"接続: {port} @ {BAUD_RATE}bps")

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] ポートを開けません: {e}")
        sys.exit(1)

    time.sleep(1.5)  # Teensy起動待ち

    log_file   = None
    log_writer = None
    logging    = False
    header     = None
    line_count = 0

    print("Teensy出力を受信中... (Ctrl+C で終了)")
    print("コマンド: L=ログ開始/停止  R=キャリブ  P=スナップショット  Q=終了\n")

    # キーボード入力を別スレッドで受け付ける
    def keyboard_input():
        while True:
            cmd = input().strip().upper()
            if cmd in ("L","R","P"):
                ser.write((cmd+"\n").encode())
            elif cmd == "Q":
                ser.write(b"L\n")  # ログ停止してから終了
                time.sleep(0.5)
                os._exit(0)

    t = threading.Thread(target=keyboard_input, daemon=True)
    t.start()

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="replace").rstrip()
            except Exception:
                continue

            # ---- ヘッダ行の保存 ----
            if line.startswith("HEADER,"):
                header = line[len("HEADER,"):]
                print(f"[ヘッダ取得] {header}")
                continue

            # ---- ログ開始シグナル ----
            if line == "LOG_START":
                log_path = new_log_path()
                log_file = open(log_path, "w", encoding="utf-8", newline="")
                if header:
                    log_file.write(header + "\n")
                logging    = True
                line_count = 0
                print(f"\n[LOG開始] → {log_path.name}")
                continue

            # ---- ログ停止シグナル ----
            if line == "LOG_STOP":
                if log_file:
                    log_file.flush()
                    log_file.close()
                    log_file = None
                logging = False
                print(f"\n[LOG停止] {line_count}行保存 → logs/{log_path.name}")
                continue

            # ---- データ行の保存 ----
            if line.startswith("DATA,"):
                csv_line = line[len("DATA,"):]
                if logging and log_file:
                    log_file.write(csv_line + "\n")
                    line_count += 1
                    if line_count % 100 == 0:
                        log_file.flush()
                        print(f"  {line_count}行...", end="\r")
                continue

            # ---- INFO行はそのままターミナルに表示 ----
            if line.startswith("INFO:"):
                print(line[5:].strip())
            else:
                print(line)  # その他の行もそのまま表示

    except KeyboardInterrupt:
        print("\n終了します...")
        if log_file:
            log_file.flush()
            log_file.close()
            print(f"[保存済] {line_count}行")
        ser.close()

if __name__ == "__main__":
    main()