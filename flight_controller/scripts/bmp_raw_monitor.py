"""
bmp_raw_monitor.py
BMP280の生値だけをプロット。Kalman・IMU処理は一切なし。

【操作】
  B : 現在地を 0m にリセット
  Q : 終了
"""

import serial
import serial.tools.list_ports
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys

# ============================================================
#  設定（main_test.cpp と合わせる）
# ============================================================
BAUD_RATE   = 115200
AUTO_DETECT = True
MANUAL_PORT = "COM14"

PLOT_SECONDS = 10.0
PLOT_POINTS  = int(PLOT_SECONDS * 50)  # 50Hz送信なので

# ============================================================
#  状態変数
# ============================================================
buf_time = deque(maxlen=PLOT_POINTS)
buf_baro = deque(maxlen=PLOT_POINTS)
lock     = threading.Lock()
alt_base = None   # 最初の受信値を 0m の基準にする
t_start  = time.time()

# ============================================================
#  ポート検出
# ============================================================
def find_port():
    if not AUTO_DETECT:
        return MANUAL_PORT
    for p in serial.tools.list_ports.comports():
        if "teensy"     in (p.description or "").lower() or \
           "usb serial" in (p.description or "").lower():
            return p.device
    ports = list(serial.tools.list_ports.comports())
    print("Select port:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    return ports[int(input("> "))].device

# ============================================================
#  シリアル受信スレッド（パースのみ）
# ============================================================
def serial_thread(ser):
    global alt_base, t_start
    ser.reset_input_buffer()
    t_start = time.time()
    print("Receiving... (waiting for first BMP value)")

    while True:
        try:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw or raw == "READY":
                continue

            parts = raw.split(",")
            # フォーマット: ax_raw,ay_raw,az_raw,baro_alt
            if len(parts) < 4:
                continue
            baro = float(parts[3])

            # 最初の有効値でベースラインを設定
            if alt_base is None:
                if baro == 0.0:   # まだBMPが準備できていない
                    continue
                alt_base = baro
                print(f"[Baseline set] {alt_base:.2f} m")

            baro_rel = baro - alt_base
            t = time.time() - t_start

            with lock:
                buf_time.append(t)
                buf_baro.append(baro_rel)

        except Exception:
            continue

# ============================================================
#  キーボードスレッド
# ============================================================
def keyboard_thread():
    global alt_base
    while True:
        c = input().strip().upper()
        if c == "B":
            with lock:
                if buf_baro:
                    alt_base += list(buf_baro)[-1]
            print("[Baseline reset] Current altitude -> 0 m")
        elif c == "Q":
            sys.exit(0)

# ============================================================
#  グラフ
# ============================================================
fig, ax = plt.subplots(figsize=(11, 4))
fig.suptitle("BMP280 Raw Altitude (no Kalman)  |  B=Baseline  Q=Quit", fontsize=11)
ax.set_ylabel("[m]")
ax.set_xlabel("Time [s]")
ax.axhline(0.0, color="gray", lw=0.5)
ax.grid(True, alpha=0.3)

line_baro, = ax.plot([], [], "b-", lw=1.5, label="baro altitude (relative)")
info_text  = ax.text(0.02, 0.92, "", transform=ax.transAxes,
                     fontsize=9, bbox=dict(boxstyle="round", fc="white", alpha=0.7))
ax.legend(loc="upper right", fontsize=8)

def update_plot(_):
    with lock:
        t  = np.array(buf_time)
        br = np.array(buf_baro)

    if len(t) < 2:
        return line_baro, info_text

    t_plot = t - t[-1]   # 現在を 0 とする相対表示
    ax.set_xlim(-PLOT_SECONDS, 0)

    # Y軸は自動調整
    margin = max(0.3, (br.max() - br.min()) * 0.2)
    ax.set_ylim(br.min() - margin, br.max() + margin)

    line_baro.set_data(t_plot, br)

    # 直近 10 秒の統計
    mask = t_plot >= -10.0
    br10 = br[mask]
    if len(br10) > 5:
        pp  = br10.max() - br10.min()
        std = np.std(br10)
        info_text.set_text(
            f"current={br[-1]:.3f} m  |  10s: P-P={pp:.3f} m  std={std:.3f} m"
        )
    return line_baro, info_text

# ============================================================
#  メイン
# ============================================================
if __name__ == "__main__":
    port = find_port()
    print(f"Port: {port}")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=2)
    except serial.SerialException as e:
        print(f"[ERROR] {e}"); sys.exit(1)

    threading.Thread(target=serial_thread,  args=(ser,), daemon=True).start()
    threading.Thread(target=keyboard_thread,              daemon=True).start()

    ani = animation.FuncAnimation(fig, update_plot, interval=100,
                                  blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()