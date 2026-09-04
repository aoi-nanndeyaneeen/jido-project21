"""
s5_logger.py  -  地上局 (xiao_s5_log) が吐く s5 テレメトリを CSV に保存する

    機体 drone_s5.cpp ──IM920sL 15Hz──> XIAO (env:xiao_s5_log) ──USB──> これ

    A(高度+姿勢角) / B(水平位置) / C(姿勢ループ内部) / P(ゲイン) の
    4種のフレームをモードに応じた順で受け、1行にまとめた CSV を保存する。

使い方:
    python tools/s5_logger.py                  # COMポート自動検出、すぐ記録開始
    python tools/s5_logger.py --port COM7
    python tools/s5_logger.py --no-autostart   # 手で 'l' を押すまで待つ

保存先:
    ground_receiver/logs/s5_NNN_YYYYMMDD_HHMMSS.csv        本体
    ground_receiver/logs/s5_NNN_YYYYMMDD_HHMMSS.params.txt 機体から届いたゲイン

終了: Ctrl+C  (自動でログを閉じ、受信機の CSV 出力も止める)

解析:
    python tools/analyze_poshold.py            # 最新のログを自動で開く

必要パッケージ:
    pip install pyserial
"""

import argparse
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

LOGS_DIR = Path(__file__).parent.parent / "logs"


# RP2040 の USB CDC。Windows では description が
# 「USB シリアル デバイス」としか出ず名前で判別できないので VID:PID で引く。
# (Teensy は 16C0:0483。Bluetooth の仮想COM を掴む事故もこれで防げる)
VID_PID_RP2040 = (0x2E8A, 0x000A)
VID_PID_TEENSY = (0x16C0, 0x0483)


def find_port():
    """XIAO RP2040 を VID:PID で探す。見つからなければ一覧を出して選ばせる。"""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    for p in ports:
        if (p.vid, p.pid) == VID_PID_RP2040:
            print(f"XIAO RP2040 を検出: {p.device}")
            return p.device

    print("XIAO RP2040 (VID:PID 2E8A:000A) が見つかりません。接続中のポート:")
    for i, p in enumerate(ports):
        tag = ""
        if (p.vid, p.pid) == VID_PID_TEENSY:
            tag = "  ← Teensy (機体側。地上局ではありません)"
        vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "     -   "
        print(f"  [{i}] {p.device}  {vidpid}  {p.description}{tag}")
    sel = input("番号を選択 (Enter で 0) > ").strip()
    return ports[int(sel) if sel else 0].device


def new_log_path():
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    idx = len(list(LOGS_DIR.glob("s5_*.csv"))) + 1
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return LOGS_DIR / f"s5_{idx:03d}_{ts}.csv"


def main():
    ap = argparse.ArgumentParser(description="s5 telemetry logger (ground station)")
    ap.add_argument("--port", default=None, help="COMポート (例: COM7)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-autostart", action="store_true",
                    help="起動時に 'l' を自動送信しない")
    args = ap.parse_args()

    port = args.port or find_port()
    if port is None:
        sys.exit("エラー: シリアルポートが見つかりません。")

    print(f"接続: {port} @ {args.baud}bps")
    try:
        ser = serial.Serial(port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"[ERROR] ポートを開けません: {e}")

    time.sleep(1.0)  # ボード起動待ち

    state = {
        "header": None,
        "file": None,
        "param_file": None,
        "path": None,
        "rows": 0,
        "logging": False,
    }

    def send(cmd):
        ser.write(cmd.encode())

    # ---- キーボード操作 (別スレッド) ----
    def keyboard():
        while True:
            try:
                cmd = input().strip().lower()
            except (EOFError, KeyboardInterrupt):
                return
            if cmd in ("l", "s", "z", "h", "d", "1", "0"):
                send(cmd)
            elif cmd == "q":
                send("0")          # 冪等に停止
                time.sleep(0.3)
                os._exit(0)

    threading.Thread(target=keyboard, daemon=True).start()

    def close_log():
        if state["file"]:
            state["file"].flush()
            state["file"].close()
            state["file"] = None
        if state["param_file"]:
            state["param_file"].close()
            state["param_file"] = None
        state["logging"] = False

    if not args.no_autostart:
        # ★ 'l' (トグル) ではなく '1' (ON) を送る。
        #   受信機が既に CSV 出力中のときにトグルを送ると「開始のつもりが停止」
        #   になり、0行のログができてしまう (実際に踏んだ)。'1' は冪等。
        send("1")
        print("記録開始を要求しました。('1' 開始 / '0' 停止 / 'q' 終了、Ctrl+C でも可)\n")
    else:
        print("受信待機中。'1' + Enter で記録開始します。\n")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip()

            if line.startswith("HEADER,"):
                state["header"] = line[len("HEADER,"):]
                continue

            if line == "LOG_START":
                close_log()
                path = new_log_path()
                state["path"] = path
                state["file"] = open(path, "w", encoding="utf-8", newline="")
                state["param_file"] = open(path.with_suffix(".params.txt"),
                                           "w", encoding="utf-8")
                if state["header"]:
                    state["file"].write(state["header"] + "\n")
                else:
                    print("[警告] HEADER 行を受け取っていません。"
                          "受信機を一度リセットしてください。")
                state["rows"] = 0
                state["logging"] = True
                print(f"\n[記録開始] -> {path.name}")
                continue

            if line == "LOG_STOP":
                path = state["path"]
                rows = state["rows"]
                close_log()
                print(f"\n[記録停止] {rows} 行 -> {path.name if path else '?'}")
                continue

            if line.startswith("DATA,"):
                if state["logging"] and state["file"]:
                    state["file"].write(line[len("DATA,"):] + "\n")
                    state["rows"] += 1
                    if state["rows"] % 10 == 0:
                        state["file"].flush()
                        # 進捗だけ1行で上書き表示 (10Hz なので 1秒ごと)
                        fields = line.split(",")
                        # DATA,rx_ms,t_ms,seq,lost,rssi,frame,mode,... の並び
                        # (s5_log.cpp の CSV_HEADER と対応。列を足したらここも直す)
                        try:
                            lost, rssi, mode = fields[4], fields[5], fields[7]
                            thr, h, tgt = fields[20], fields[25], fields[27]
                        except IndexError:
                            lost = rssi = mode = thr = h = tgt = "?"
                        print(f"\r  {state['rows']:6d} 行  lost={lost} "
                              f"rssi={rssi} mode={mode} thr={thr} h={h}/{tgt} m   ",
                              end="", flush=True)
                continue

            if line.startswith("PARAM,"):
                if state["param_file"]:
                    state["param_file"].write(line + "\n")
                    state["param_file"].flush()
                if not state["logging"]:
                    print(line)
                continue

            # '#' で始まる受信機のメッセージや、それ以外の行はそのまま表示
            if line:
                print(("\n" if state["logging"] else "") + line)

    except KeyboardInterrupt:
        print("\n終了します...")
        try:
            send("0")           # 受信機の CSV 出力も止める (冪等)
            time.sleep(0.2)
        except Exception:
            pass
        path, rows = state["path"], state["rows"]
        close_log()
        ser.close()
        if path:
            print(f"[保存] {rows} 行 -> {path}")
            print(f"解析:  python tools/analyze_poshold.py \"{path}\"")


if __name__ == "__main__":
    main()
