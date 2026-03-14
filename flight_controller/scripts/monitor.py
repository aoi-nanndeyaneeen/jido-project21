"""
scripts/monitor.py
MPU6050の6軸をリアルタイムグラフ表示 + CSVログ保存

【使い方】
  python scripts/monitor.py

【操作】
  R キー : キャリブレーション（静止させてから押す）
  L キー : ログ開始/停止
  Q キー : 終了

【必要パッケージ】
  pip install pyserial matplotlib numpy
"""

import serial
import serial.tools.list_ports
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from pathlib import Path
from datetime import datetime
import sys

# ============================================================
#  設定
# ============================================================
BAUD_RATE    = 115200
AUTO_DETECT  = True
MANUAL_PORT  = "COM14"
LOGS_DIR     = Path(__file__).parent.parent / "logs"

ACCEL_SCALE  = 16384.0   # ±2g (レジスタ確認後に要調整: ±8gなら4096.0)
GYRO_SCALE   = 131.0     # ±250dps
PLOT_SECONDS = 5.0
SAMPLE_RATE  = 50
PLOT_POINTS  = int(PLOT_SECONDS * SAMPLE_RATE)

# ============================================================
#  状態管理
# ============================================================
# オフセットは全軸0スタート（重力を引かない・生値をそのまま表示）
# Rキーで水平静止キャリブ後にaz_offset=-az_rawになり重力除去される
offset = np.zeros(6)
# ※ 水平静止時にazが1.0gになるかどうかでスケールを確認する
#   1.0g → ACCEL_SCALE=16384 正しい
#   0.25g → ACCEL_SCALE=4096 に変更（±8g設定になっている）
#   0.5g → ACCEL_SCALE=8192 に変更（±4g設定）

# リングバッファ
# dequeでリングバッファ（numpy rollより高速でフリーズしない）
buf_time  = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_ax    = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_ay    = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_az    = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_gx    = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_gy    = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_gz    = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_norm  = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)

lock         = threading.Lock()
calib_flag   = False    # Rキーでセット
logging_flag = False
log_file     = None
sample_count = 0
t_start      = time.time()

# キャリブレーション用バッファ
calib_buf    = []
CALIB_SAMPLES = 200

# ============================================================
#  COMポート自動検出
# ============================================================
def find_port():
    if not AUTO_DETECT:
        return MANUAL_PORT
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "teensy" in desc or "usb serial" in desc:
            return p.device
    ports = list(serial.tools.list_ports.comports())
    print("ポートを選択してください:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    return ports[int(input("> "))].device

# ============================================================
#  シリアル受信スレッド
# ============================================================
def serial_thread(ser):
    global offset, calib_flag, calib_buf, sample_count, t_start

    # Teensy4.0はUSB接続でリセットしないためREADY待ちは不要
    # バッファをフラッシュして即受信開始
    ser.reset_input_buffer()
    print("受信開始")
    t_start = time.time()

    while True:
        try:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw or raw.count(",") != 5:
                continue

            vals = list(map(int, raw.split(",")))
            ax_r, ay_r, az_r, gx_r, gy_r, gz_r = vals

            # キャリブレーション収集中
            if calib_flag:
                calib_buf.append(vals)
                if len(calib_buf) >= CALIB_SAMPLES:
                    arr = np.array(calib_buf, dtype=float)
                    mean = arr.mean(axis=0)
                    # az だけ理想値(+1g)との差をオフセットにする
                    new_offset = mean.copy()
                    new_offset[2] = mean[2] - ACCEL_SCALE
                    with lock:
                        offset = new_offset
                    norm_after = np.linalg.norm(
                        (mean[:3] - new_offset[:3]) / ACCEL_SCALE)
                    print(f"\n[Calib done] |a|={norm_after:.4f}g  "
                          f"offset={new_offset[:3].astype(int)}")
                    if norm_after > 1.8:
                        print("[WARN] still ~2g -> check scale setting")
                    elif norm_after < 0.5:
                        print("[ERROR] ~0g -> sensor may be broken")
                    else:
                        print("[OK] ~1g -> normal")
                    calib_buf.clear()
                    calib_flag = False
                continue

            # 補正値に変換
            with lock:
                off = offset.copy()
            ax = (ax_r - off[0]) / ACCEL_SCALE
            ay = -(ay_r - off[1]) / ACCEL_SCALE  # Y軸反転
            az = (az_r - off[2]) / ACCEL_SCALE
            gx = (gx_r - off[3]) / GYRO_SCALE
            gy = (gy_r - off[4]) / GYRO_SCALE
            gz = (gz_r - off[5]) / GYRO_SCALE
            norm = np.sqrt(ax**2 + ay**2 + az**2)
            t    = time.time() - t_start

            with lock:
                buf_time.append(t)
                buf_ax.append(ax); buf_ay.append(ay); buf_az.append(az)
                buf_gx.append(gx); buf_gy.append(gy); buf_gz.append(gz)
                buf_norm.append(norm)
                sample_count += 1

            # ログ書き込み
            if logging_flag and log_file:
                log_file.write(
                    f"{t:.3f},{ax:.4f},{ay:.4f},{az:.4f},"
                    f"{gx:.3f},{gy:.3f},{gz:.3f},{norm:.4f}\n")

        except Exception:
            continue

# ============================================================
#  キーボード入力スレッド
# ============================================================
def keyboard_thread():
    global calib_flag, calib_buf, logging_flag, log_file
    while True:
        c = input().strip().upper()
        if c == "R":
            print(f"[キャリブ開始] 静止させてください ({CALIB_SAMPLES}サンプル収集)...")
            calib_buf.clear()
            calib_flag = True
        elif c == "L":
            if not logging_flag:
                LOGS_DIR.mkdir(parents=True, exist_ok=True)
                existing = list(LOGS_DIR.glob("log_*.csv"))
                fname = LOGS_DIR / f"log_{len(existing)+1:03d}_{datetime.now():%Y%m%d_%H%M%S}.csv"
                log_file = open(fname, "w")
                log_file.write("time_s,ax,ay,az,gx,gy,gz,a_norm\n")
                logging_flag = True
                print(f"[ログ開始] → {fname.name}")
            else:
                logging_flag = False
                if log_file:
                    log_file.flush(); log_file.close(); log_file = None
                print("[ログ停止]")
        elif c == "Q":
            print("終了します")
            if log_file:
                log_file.flush(); log_file.close()
            sys.exit(0)

# ============================================================
#  グラフ設定
# ============================================================
fig, (ax_plot, gyro_plot) = plt.subplots(2, 1, figsize=(10, 7))
fig.suptitle("MPU6050 Monitor  |  R=Calib  L=Log  Q=Quit", fontsize=11)

ax_plot.set_title("Accelerometer [g]")
ax_plot.set_ylim(-2.2, 2.2)
ax_plot.axhline(1.0,  color="gray", lw=0.5, ls="--")
ax_plot.axhline(-1.0, color="gray", lw=0.5, ls="--")
ax_plot.axhline(0.0,  color="gray", lw=0.5)
ax_plot.set_ylabel("[g]")
ax_plot.grid(True, alpha=0.3)

gyro_plot.set_title("Gyroscope [dps]")
gyro_plot.set_ylim(-300, 300)
gyro_plot.axhline(0.0, color="gray", lw=0.5)
gyro_plot.set_ylabel("[dps]")
gyro_plot.set_xlabel("Time [s]")
gyro_plot.grid(True, alpha=0.3)

line_ax,   = ax_plot.plot([], [], "r-",  lw=1.2, label="ax")
line_ay,   = ax_plot.plot([], [], "g-",  lw=1.2, label="ay")
line_az,   = ax_plot.plot([], [], "b-",  lw=1.2, label="az")
line_norm, = ax_plot.plot([], [], "k--", lw=1.0, label="|a|")
line_gx,   = gyro_plot.plot([], [], "r-", lw=1.2, label="gx")
line_gy,   = gyro_plot.plot([], [], "g-", lw=1.2, label="gy")
line_gz,   = gyro_plot.plot([], [], "b-", lw=1.2, label="gz")

ax_plot.legend(loc="upper right", fontsize=8)
gyro_plot.legend(loc="upper right", fontsize=8)

# 右上に|a|テキスト表示
norm_text = ax_plot.text(0.02, 0.92, "", transform=ax_plot.transAxes,
                         fontsize=10, color="black",
                         bbox=dict(boxstyle="round", fc="white", alpha=0.7))

def update_plot(_):
    with lock:
        t   = np.array(buf_time)
        _ax = np.array(buf_ax); _ay = np.array(buf_ay); _az = np.array(buf_az)
        _gx = np.array(buf_gx); _gy = np.array(buf_gy); _gz = np.array(buf_gz)
        _nm = np.array(buf_norm)

    if t[-1] == 0:
        return line_ax, line_ay, line_az, line_norm, line_gx, line_gy, line_gz, norm_text

    # X軸を「直近N秒」でスクロール
    x_max = t[-1]
    x_min = x_max - PLOT_SECONDS
    ax_plot.set_xlim(x_min, x_max)
    gyro_plot.set_xlim(x_min, x_max)

    line_ax.set_data(t, _ax); line_ay.set_data(t, _ay); line_az.set_data(t, _az)
    line_norm.set_data(t, _nm)
    line_gx.set_data(t, _gx); line_gy.set_data(t, _gy); line_gz.set_data(t, _gz)

    cur_norm = _nm[-1]
    status = "正常" if 0.97 < cur_norm < 1.03 else ("スケール誤り?" if cur_norm > 1.8 else "要確認")
    norm_text.set_text(f"|a| = {cur_norm:.4f} g  [{status}]")
    norm_text.set_color("green" if "正常" in status else "red")

    return line_ax, line_ay, line_az, line_norm, line_gx, line_gy, line_gz, norm_text

# ============================================================
#  メイン
# ============================================================
if __name__ == "__main__":
    port = find_port()
    print(f"接続: {port}")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=2)
    except serial.SerialException as e:
        print(f"[ERROR] {e}"); sys.exit(1)

    print("操作: R=キャリブ  L=ログ開始/停止  Q=終了\n")

    threading.Thread(target=serial_thread,  args=(ser,), daemon=True).start()
    threading.Thread(target=keyboard_thread,               daemon=True).start()

    ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=True)
    plt.tight_layout()
    plt.show()