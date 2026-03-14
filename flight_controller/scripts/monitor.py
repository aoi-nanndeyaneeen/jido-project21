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

ACCEL_SCALE  = 16384.0   # ±2g
GYRO_SCALE   = 131.0     # ±250dps
PLOT_SECONDS = 5.0        # グラフの表示幅 [秒]
SAMPLE_RATE  = 50         # Teensy送信レート [Hz]
PLOT_POINTS  = int(PLOT_SECONDS * SAMPLE_RATE)

# ============================================================
#  状態管理
# ============================================================
# オフセット（キャリブレーションで更新）
offset = np.zeros(6)          # [ax,ay,az,gx,gy,gz] の生値オフセット
offset[2] = ACCEL_SCALE       # az は +1g(=16384)が理想なので差し引く

# リングバッファ
buf_time  = np.zeros(PLOT_POINTS)
buf_ax    = np.zeros(PLOT_POINTS)
buf_ay    = np.zeros(PLOT_POINTS)
buf_az    = np.zeros(PLOT_POINTS)
buf_gx    = np.zeros(PLOT_POINTS)
buf_gy    = np.zeros(PLOT_POINTS)
buf_gz    = np.zeros(PLOT_POINTS)
buf_norm  = np.zeros(PLOT_POINTS)

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

    # READY待ち
    print("Teensy起動待ち...", end="", flush=True)
    while True:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line == "READY":
            print(" OK")
            break

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
                    print(f"\n[キャリブ完了] |a|={norm_after:.4f}g  "
                          f"offset={new_offset[:3].astype(int)}")
                    if norm_after > 1.8:
                        print("[WARN] まだ≈2g → スケール設定を確認")
                    elif norm_after < 0.5:
                        print("[ERROR] ≈0g → センサ故障の可能性")
                    else:
                        print("[OK] ≈1g → 正常")
                    calib_buf.clear()
                    calib_flag = False
                continue

            # 補正値に変換
            with lock:
                off = offset.copy()
            ax = (ax_r - off[0]) / ACCEL_SCALE
            ay = (ay_r - off[1]) / ACCEL_SCALE
            az = (az_r - off[2]) / ACCEL_SCALE
            gx = (gx_r - off[3]) / GYRO_SCALE
            gy = (gy_r - off[4]) / GYRO_SCALE
            gz = (gz_r - off[5]) / GYRO_SCALE
            norm = np.sqrt(ax**2 + ay**2 + az**2)
            t    = time.time() - t_start

            # バッファ更新（ロールシフト）
            with lock:
                buf_time[:-1] = buf_time[1:]; buf_time[-1] = t
                buf_ax[:-1]   = buf_ax[1:];   buf_ax[-1]   = ax
                buf_ay[:-1]   = buf_ay[1:];   buf_ay[-1]   = ay
                buf_az[:-1]   = buf_az[1:];   buf_az[-1]   = az
                buf_gx[:-1]   = buf_gx[1:];   buf_gx[-1]   = gx
                buf_gy[:-1]   = buf_gy[1:];   buf_gy[-1]   = gy
                buf_gz[:-1]   = buf_gz[1:];   buf_gz[-1]   = gz
                buf_norm[:-1] = buf_norm[1:]; buf_norm[-1] = norm
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
fig.suptitle("MPU6050 リアルタイムモニタ  |  R=キャリブ  L=ログ  Q=終了",
             fontsize=11)

ax_plot.set_title("加速度 [g]")
ax_plot.set_ylim(-2.2, 2.2)
ax_plot.axhline(1.0,  color="gray", lw=0.5, ls="--")
ax_plot.axhline(-1.0, color="gray", lw=0.5, ls="--")
ax_plot.axhline(0.0,  color="gray", lw=0.5)
ax_plot.set_ylabel("[g]")
ax_plot.grid(True, alpha=0.3)

gyro_plot.set_title("ジャイロ [dps]")
gyro_plot.set_ylim(-300, 300)
gyro_plot.axhline(0.0, color="gray", lw=0.5)
gyro_plot.set_ylabel("[dps]")
gyro_plot.set_xlabel("時間 [s]")
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
        t   = buf_time.copy()
        _ax = buf_ax.copy(); _ay = buf_ay.copy(); _az = buf_az.copy()
        _gx = buf_gx.copy(); _gy = buf_gy.copy(); _gz = buf_gz.copy()
        _nm = buf_norm.copy()

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