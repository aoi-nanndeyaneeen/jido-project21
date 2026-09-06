# scripts/logger.py
# Teensyのシリアル出力を受け取り、logsフォルダにCSVを保存する
#
# 【使い方】
#   1. Teensyを接続してアップロード済みにする
#   2. ターミナルで:  python scripts/logger.py
#   3. Teensyのシリアルモニタ(VSCode)は閉じておく (COMポートを占有するため)
#   4. キー入力:  L=ログ開始/停止  R=キャリブ  P=スナップショット
#                V=RamLogダンプ    Y=RamLog状態  W=停止調査ログ  Q=終了
#
# 【必要パッケージ】
#   pip install pyserial
#
# ─────────────────────────────────────────────────────────────
#  ★ 2026-09-06 受信経路を書き直した。
#    旧版は ser.readline() で1行ずつ・単一スレッド・100行ごとに flush()+print()
#    していた。500Hz(1行322B ≒ 160KB/s)ストリームや RamLog ダンプ
#    (無ペーシングの ~1.3MB 一括送信) に PC が追いつけず、Windows/pyserial の
#    受信バッファが溢れて「行が丸ごと消える」現象が出ていた
#    (log_028/030 の欠損は全てこれ。ファームのループは止まっていない)。
#
#    新版:
#      - 受信バッファを 4MB に拡大 (Windows は set_buffer_size が効く)
#      - readline() をやめ、in_waiting をまとめ読みして \n で分割
#      - 受信スレッド と 書き込みスレッド を分離 (Queue 経由)
#      - flush は 100行ごとではなく 0.5秒ごと (時間ベース)
#      - t_ms 列の飛びを検出して「取りこぼし」を警告 + 総数を集計
# ─────────────────────────────────────────────────────────────

import serial
import serial.tools.list_ports
import os
import sys
import threading
import queue
import time
from datetime import datetime
from pathlib import Path

# ============================================================
#  設定
# ============================================================
BAUD_RATE   = 115200          # Teensy はネイティブUSBなので実際には無視される
LOGS_DIR    = Path(__file__).parent.parent / "logs"
AUTO_DETECT = True
MANUAL_PORT = "COM14"
RX_BUF_BYTES = 4 * 1024 * 1024   # OS 受信バッファ要求サイズ
FLUSH_EVERY_S = 0.5              # ファイル flush 間隔 [s]


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
    print("Teensyが自動検出できませんでした。接続中のCOMポート一覧:")
    for i, p in enumerate(ports):
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
#  書き込みスレッド:  Queue から行を受け取り CSV に落とすだけ
#   受信スレッドをファイル I/O でブロックさせないために分離する。
# ============================================================
class Writer(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.q = queue.Queue(maxsize=200000)
        self._stop = threading.Event()
        self.file = None
        self.path = None
        self.header = None
        self.line_count = 0
        # 取りこぼし検出用
        self._last_t = None
        self._t_col = 0
        self.missing_rows = 0
        self.gap_events = 0

    # --- 受信スレッドから呼ぶ ---
    def open_log(self):
        self.q.put(("OPEN", None))

    def close_log(self):
        self.q.put(("CLOSE", None))

    def set_header(self, h):
        self.header = h
        # t_ms 列の位置を覚える (先頭想定だが一応探す)
        cols = h.split(",")
        self._t_col = cols.index("t_ms") if "t_ms" in cols else 0

    def data(self, csv_line):
        try:
            self.q.put_nowait(("DATA", csv_line))
        except queue.Full:
            # ここが詰まる = ディスクが遅い。運用上ほぼ起きないが記録はする。
            self.missing_rows += 1

    def stop(self):
        self._stop.set()
        self.q.put(("QUIT", None))

    # --- スレッド本体 ---
    def run(self):
        last_flush = time.time()
        while not self._stop.is_set():
            try:
                kind, payload = self.q.get(timeout=0.2)
            except queue.Empty:
                if self.file and (time.time() - last_flush) > FLUSH_EVERY_S:
                    self.file.flush()
                    last_flush = time.time()
                continue

            if kind == "QUIT":
                break
            if kind == "OPEN":
                self.path = new_log_path()
                self.file = open(self.path, "w", encoding="utf-8", newline="")
                if self.header:
                    self.file.write(self.header + "\n")
                self.line_count = 0
                self._last_t = None
                self.missing_rows = 0
                self.gap_events = 0
                print(f"\n[LOG開始] → {self.path.name}")
            elif kind == "CLOSE":
                if self.file:
                    self.file.flush()
                    self.file.close()
                    self.file = None
                    miss = f"  (取りこぼし {self.missing_rows} 行 / {self.gap_events} 箇所)" \
                           if self.missing_rows else "  (取りこぼしなし)"
                    print(f"\n[LOG停止] {self.line_count}行保存 → logs/{self.path.name}{miss}")
            elif kind == "DATA":
                if not self.file:
                    continue
                self.file.write(payload + "\n")
                self.line_count += 1
                # --- t_ms の飛び検出 ---
                try:
                    tval = int(payload.split(",", self._t_col + 1)[self._t_col])
                    if self._last_t is not None:
                        dt = tval - self._last_t
                        if dt > 6:   # 500Hz なら 2ms 刻み。6ms 超は取りこぼし
                            self.gap_events += 1
                            self.missing_rows += max(dt // 2 - 1, 0)
                    self._last_t = tval
                except (ValueError, IndexError):
                    pass
                if self.line_count % 500 == 0:
                    print(f"  {self.line_count}行  (取りこぼし {self.missing_rows})", end="\r")

        if self.file:
            self.file.flush()
            self.file.close()


# ============================================================
#  メイン (受信スレッド = ここ)
# ============================================================
def main():
    port = find_teensy_port() if AUTO_DETECT else MANUAL_PORT
    print(f"接続: {port} @ {BAUD_RATE}bps (Teensyネイティブ USB なので速度指定は名目上)")

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0)   # 非ブロッキング読み
    except serial.SerialException as e:
        print(f"[ERROR] ポートを開けません: {e}")
        sys.exit(1)

    try:
        ser.set_buffer_size(rx_size=RX_BUF_BYTES, tx_size=4096)
    except Exception as e:
        print(f"[warn] set_buffer_size 非対応: {e}")

    time.sleep(1.5)  # Teensy起動待ち
    ser.reset_input_buffer()

    writer = Writer()
    writer.start()

    logging = False

    print("Teensy出力を受信中... (Ctrl+C で終了)")
    print("コマンド: L=ログ開始/停止  R=キャリブ  P=スナップショット")
    print("         V=RamLogダンプ  Y=RamLog状態確認  W=停止調査ログダンプ  Q=終了\n")

    def keyboard_input():
        while True:
            try:
                cmd = input().strip().upper()
            except EOFError:
                return
            if cmd in ("L", "R", "P", "V", "Y", "W"):
                ser.write((cmd + "\n").encode())
            elif cmd == "Q":
                ser.write(b"L\n")
                time.sleep(0.5)
                os._exit(0)

    threading.Thread(target=keyboard_input, daemon=True).start()

    buf = bytearray()
    try:
        while True:
            # --- まとめ読み: 来ているぶんを一括で取る ---
            chunk = ser.read(65536)
            if not chunk:
                # in_waiting も見て、無ければ少しだけ待つ
                n = ser.in_waiting
                if n:
                    chunk = ser.read(n)
                else:
                    time.sleep(0.001)
                    continue
            buf.extend(chunk)

            # --- 行単位に切り出す ---
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                raw = bytes(buf[:nl])
                del buf[:nl + 1]
                line = raw.decode("utf-8", errors="replace").rstrip("\r")

                if line.startswith("HEADER,"):
                    h = line[len("HEADER,"):]
                    writer.set_header(h)
                    print(f"[ヘッダ取得] {h[:60]}...")
                elif line == "LOG_START":
                    writer.open_log()
                    logging = True
                elif line == "LOG_STOP":
                    writer.close_log()
                    logging = False
                elif line.startswith("DATA,"):
                    if logging:
                        writer.data(line[len("DATA,"):])
                elif line.startswith("INFO:"):
                    print(line[5:].strip())
                elif line:
                    print(line)

            # バッファが極端に育ったら警告 (通常あり得ない)
            if len(buf) > 2 * 1024 * 1024:
                print(f"[warn] 未処理バッファ {len(buf)//1024}KB — 処理が遅れています")

    except KeyboardInterrupt:
        print("\n終了します...")
        try:
            ser.write(b"L\n")
        except Exception:
            pass
        time.sleep(0.3)
        writer.stop()
        writer.join(timeout=2)
        ser.close()


if __name__ == "__main__":
    main()
