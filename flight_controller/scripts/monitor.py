"""
scripts/monitor.py
加速度 + BMP280気圧高度 + カルマンフィルタ高度 リアルタイム表示

【操作】
  R : キャリブレーション（水平静止してから）
  L : ログ開始/停止
  B : 高度ベースラインリセット（現在地を0mにする）
  Q : 終了

【カルマンフィルタ設計】
  状態: [高度z, 垂直速度vz]
  予測: az_world（重力除去後の垂直加速度）で積分
  更新: BMP280気圧高度で補正
  R_baro小 → 気圧を信頼 → 速応答・ノイズ多め
  R_baro大 → 加速度予測を信頼 → 遅応答・滑らか
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

ACCEL_SCALE  = 16384.0  # ±2g。水平静止でaz≈0.25gなら4096.0に変更
GRAVITY      = 9.80665  # [m/s²]
SEA_LEVEL_HPA = 1013.25 # 天気予報の気圧に合わせると高度精度向上

PLOT_SECONDS = 8.0
SAMPLE_RATE  = 50        # Teensy送信レート
PLOT_POINTS  = int(PLOT_SECONDS * SAMPLE_RATE)

# ============================================================
#  カルマンフィルタ
#  目標: 3m→6mの変化を0.1秒以内に反映
#  R_baro=0.05 + BMP280 20Hz → 約50ms/サンプルで即追従
# ============================================================
class AltitudeKalman:
    def __init__(self):
        self.z   = 0.0   # 推定高度 [m]
        self.vz  = 0.0   # 推定垂直速度 [m/s]
        self.P   = np.eye(2)

        # BMP280 FILTER_X16 + SAMPLING_X16設定に合わせたパラメータ
        # BMP280が遅くて正確(2Hz, σ≈0.1m) → R_baro小さく強く信頼
        # 加速度が速くて積分誤差あり → Q_vz大きく速度変化を許容
        self.Q_z    = 0.001  # 高度プロセスノイズ
        self.Q_vz   = 0.1    # 速度プロセスノイズ
        self.R_baro = 0.01   # BMP280観測ノイズ(σ≈0.1m → σ²=0.01)

    def init(self, alt):
        self.z = alt; self.vz = 0.0
        self.P = np.eye(2)

    def predict(self, dt, az_world):
        """az_world: 重力除去済み垂直加速度 [m/s²]"""
        self.z  += self.vz * dt + 0.5 * az_world * dt**2
        self.vz += az_world * dt
        F = np.array([[1, dt], [0, 1]])
        Q = np.diag([self.Q_z, self.Q_vz])
        self.P = F @ self.P @ F.T + Q

    def update(self, baro_z):
        """baro_z: BMP280気圧高度 [m]"""
        H = np.array([[1, 0]])
        S = H @ self.P @ H.T + self.R_baro
        K = self.P @ H.T / S[0, 0]
        inn = baro_z - self.z
        self.z  += K[0, 0] * inn
        self.vz += K[1, 0] * inn
        self.P   = (np.eye(2) - K @ H) @ self.P

# ============================================================
#  状態変数
# ============================================================
offset   = np.zeros(3)      # [ax, ay, az] 生値オフセット
kalman   = AltitudeKalman()
alt_base = 0.0              # Bキーでリセットする高度ベースライン

buf_time  = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_ax    = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_ay    = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_az    = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_norm  = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_baro  = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)
buf_kalt  = deque([0.0] * PLOT_POINTS, maxlen=PLOT_POINTS)

lock          = threading.Lock()
calib_flag    = False
calib_buf     = []
logging_flag  = False
log_file      = None
t_start       = time.time()
t_last        = time.time()
baro_prev     = 0.0
kalman_ready  = False
CALIB_SAMPLES = 200
BASELINE_SAMPLES = 50   # 起動時に何サンプル平均でベースラインを決めるか
baseline_buf  = []      # 起動時のベースライン収集用

# ============================================================
#  ポート検出
# ============================================================
def find_port():
    if not AUTO_DETECT:
        return MANUAL_PORT
    for p in serial.tools.list_ports.comports():
        if "teensy" in (p.description or "").lower() or \
           "usb serial" in (p.description or "").lower():
            return p.device
    ports = list(serial.tools.list_ports.comports())
    print("Select port:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    return ports[int(input("> "))].device

# ============================================================
#  シリアル受信スレッド
# ============================================================
def serial_thread(ser):
    global offset, calib_flag, calib_buf, t_start, t_last
    global baro_prev, kalman_ready, alt_base

    ser.reset_input_buffer()
    print("Receiving...")
    t_start = time.time()
    t_last  = t_start

    while True:
        try:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw or raw == "READY":
                continue
            parts = raw.split(",")
            if len(parts) != 4:
                continue

            ax_r, ay_r, az_r = int(parts[0]), int(parts[1]), int(parts[2])
            baro_raw = float(parts[3])

            # --- キャリブ収集 ---
            if calib_flag:
                calib_buf.append([ax_r, ay_r, az_r])
                if len(calib_buf) >= CALIB_SAMPLES:
                    arr  = np.array(calib_buf, dtype=float)
                    mean = arr.mean(axis=0)
                    new_off = mean.copy()
                    new_off[2] = mean[2] - ACCEL_SCALE  # az: 重力1g分を残す
                    with lock:
                        offset[:] = new_off
                    calib_buf.clear()
                    calib_flag = False
                    _az_check = (mean[2] - new_off[2]) / ACCEL_SCALE
                    print(f"[Calib done] az_check={_az_check:.4f}g  "
                          f"(1.0=OK / 0.25=scale4096 / 0.5=scale8192)")
                continue

            # --- スケール・オフセット適用 ---
            with lock:
                off = offset.copy()

            ax =  (ax_r - off[0]) / ACCEL_SCALE
            ay = -(ay_r - off[1]) / ACCEL_SCALE   # Y軸反転
            az =  (az_r - off[2]) / ACCEL_SCALE   # キャリブ後: 水平で≈1.0g

            # 垂直加速度 (重力除去, 機体がほぼ水平の前提)
            az_world = (az - 1.0) * GRAVITY        # [m/s²]

            norm = np.sqrt(ax**2 + ay**2 + az**2)
            t    = time.time() - t_start

            # --- カルマンフィルタ ---
            dt = time.time() - t_last
            t_last = time.time()
            dt = min(dt, 0.1)  # 外れ値ガード

            baro_adj = baro_raw + (SEA_LEVEL_HPA - 1013.25) * 8.5  # 海面気圧補正[簡易]

            # --- ベースライン確定（起動時に50サンプル平均、以降は一切変更しない）---
            if not kalman_ready:
                baseline_buf.append(baro_adj)
                if len(baseline_buf) < BASELINE_SAMPLES:
                    continue  # 収集中はカルマン更新しない
                alt_base = float(np.mean(baseline_buf))
                kalman.init(0.0)
                kalman_ready = True
                print(f"[Baseline fixed] {alt_base:.2f} m  (will NOT auto-reset)")

            baro_rel = baro_adj - alt_base  # ベースラインからの相対高度

            kalman.predict(dt, az_world)
            kalman.update(baro_rel)

            with lock:
                buf_time.append(t)
                buf_ax.append(ax); buf_ay.append(ay); buf_az.append(az)
                buf_norm.append(norm)
                buf_baro.append(baro_rel)
                buf_kalt.append(kalman.z)

            if logging_flag and log_file:
                log_file.write(
                    f"{t:.3f},{ax:.4f},{ay:.4f},{az:.4f},"
                    f"{az_world:.4f},{baro_rel:.3f},{kalman.z:.3f},{kalman.vz:.3f}\n")

        except Exception:
            continue

# ============================================================
#  キーボード入力スレッド
# ============================================================
def keyboard_thread():
    global calib_flag, calib_buf, logging_flag, log_file, alt_base, kalman_ready

    while True:
        c = input().strip().upper()
        if c == "R":
            print(f"[Calib] Keep still ({CALIB_SAMPLES} samples)...")
            calib_buf.clear()
            calib_flag = True
        elif c == "B":
            # 現在地を高度0mにリセット
            with lock:
                if buf_baro:
                    alt_base += list(buf_baro)[-1]  # 現在の相対高度をベースに加算
            kalman.init(0.0)
            print("[Baseline reset] Current altitude -> 0 m")
        elif c == "L":
            if not logging_flag:
                LOGS_DIR.mkdir(parents=True, exist_ok=True)
                n = len(list(LOGS_DIR.glob("log_*.csv"))) + 1
                fname = LOGS_DIR / f"log_{n:03d}_{datetime.now():%Y%m%d_%H%M%S}.csv"
                log_file = open(fname, "w")
                log_file.write("time_s,ax,ay,az,az_world,baro_alt,kalman_alt,kalman_vz\n")
                logging_flag = True
                print(f"[Log start] -> {fname.name}")
            else:
                logging_flag = False
                if log_file:
                    log_file.flush(); log_file.close(); log_file = None
                print("[Log stop]")
        elif c == "Q":
            if log_file:
                log_file.flush(); log_file.close()
            sys.exit(0)

# ============================================================
#  グラフ設定
# ============================================================
fig, (ax_plot, alt_plot) = plt.subplots(2, 1, figsize=(11, 7))
fig.suptitle("IMU Monitor  |  R=Calib  B=BaselineReset  L=Log  Q=Quit", fontsize=11)

# 上段: 加速度
ax_plot.set_title("Accelerometer [g]")
ax_plot.set_ylim(-2.2, 2.2)
ax_plot.axhline( 1.0, color="gray", lw=0.5, ls="--")
ax_plot.axhline(-1.0, color="gray", lw=0.5, ls="--")
ax_plot.axhline( 0.0, color="gray", lw=0.5)
ax_plot.set_ylabel("[g]")
ax_plot.grid(True, alpha=0.3)

# 下段: 高度
alt_plot.set_title("Altitude [m]  (relative to baseline)")
alt_plot.set_ylim(-5, 20)
alt_plot.axhline(0.0, color="gray", lw=0.5)
alt_plot.set_ylabel("[m]")
alt_plot.set_xlabel("Time [s]")
alt_plot.grid(True, alpha=0.3)

line_ax,   = ax_plot.plot([], [], "r-",  lw=1.2, label="ax")
line_ay,   = ax_plot.plot([], [], "g-",  lw=1.2, label="ay")
line_az,   = ax_plot.plot([], [], "b-",  lw=1.2, label="az")
line_norm, = ax_plot.plot([], [], "k--", lw=0.8, label="|a|")
line_baro, = alt_plot.plot([], [], "c-",  lw=1.0, label="baro (raw)",   alpha=0.6)
line_kalt, = alt_plot.plot([], [], "r-",  lw=1.8, label="kalman alt")

ax_plot.legend(loc="upper right", fontsize=8)
alt_plot.legend(loc="upper right", fontsize=8)

norm_text = ax_plot.text(0.02, 0.92, "", transform=ax_plot.transAxes,
                         fontsize=9, bbox=dict(boxstyle="round", fc="white", alpha=0.7))
alt_text  = alt_plot.text(0.02, 0.92, "", transform=alt_plot.transAxes,
                          fontsize=9, bbox=dict(boxstyle="round", fc="white", alpha=0.7))

def update_plot(_):
    with lock:
        t    = np.array(buf_time)
        _ax  = np.array(buf_ax);  _ay = np.array(buf_ay); _az = np.array(buf_az)
        _nm  = np.array(buf_norm)
        _br  = np.array(buf_baro)
        _ka  = np.array(buf_kalt)

    if t[-1] == 0:
        return line_ax, line_ay, line_az, line_norm, line_baro, line_kalt, norm_text, alt_text

    x_max = t[-1]; x_min = x_max - PLOT_SECONDS
    ax_plot.set_xlim(x_min, x_max)
    alt_plot.set_xlim(x_min, x_max)

    # 高度レンジを自動調整（余白1m）
    if _ka.max() - _ka.min() > 0.1:
        margin = max(1.0, (_ka.max() - _ka.min()) * 0.2)
        alt_plot.set_ylim(_ka.min() - margin, _ka.max() + margin)

    line_ax.set_data(t, _ax); line_ay.set_data(t, _ay)
    line_az.set_data(t, _az); line_norm.set_data(t, _nm)
    line_baro.set_data(t, _br); line_kalt.set_data(t, _ka)

    cur_norm = _nm[-1]
    status = "OK" if 0.95 < cur_norm < 1.05 else ("scale?" if cur_norm > 1.5 else "check")
    norm_text.set_text(f"|a|={cur_norm:.3f}g [{status}]")
    norm_text.set_color("green" if status == "OK" else "red")

    alt_text.set_text(f"baro={_br[-1]:.2f}m  kalman={_ka[-1]:.2f}m  vz={kalman.vz:.2f}m/s")

    return line_ax, line_ay, line_az, line_norm, line_baro, line_kalt, norm_text, alt_text

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

    print("R=Calib  B=BaselineReset  L=Log  Q=Quit\n")

    threading.Thread(target=serial_thread,  args=(ser,), daemon=True).start()
    threading.Thread(target=keyboard_thread,               daemon=True).start()

    ani = animation.FuncAnimation(fig, update_plot, interval=100,
                                  blit=True, cache_frame_data=False)
    plt.tight_layout()
    plt.show()