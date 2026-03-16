"""
bmp_raw_monitor.py
BMP280の生値＋移動平均をプロット。Kalman・IMU処理は一切なし。

【操作】
  L : ログ開始（15秒自動終了）
  M : 移動平均Nをリアルタイム変更
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
from pathlib import Path
from datetime import datetime
import sys

# ============================================================
#  設定
# ============================================================
BAUD_RATE   = 115200
AUTO_DETECT = True
MANUAL_PORT = "COM14"
LOGS_DIR    = Path(__file__).parent.parent / "logs"

BMP_HZ = 50   # ★ main_test.cpp の BMP_HZ と合わせる（2 / 10 / 23 / 50 / 75）
MA_N   = 5    # 移動平均サンプル数（Mキーで変更可: 1=オフ, 推奨3〜15）

PLOT_SECONDS = 10.0
PLOT_POINTS  = int(PLOT_SECONDS * 50)

# ステップ応答テストの時間設定 [秒]
T_TOTAL = 15.0
T_DOWN  =  5.0
T_UP    = 10.0

# ============================================================
#  状態変数
# ============================================================
buf_time    = deque(maxlen=PLOT_POINTS)
buf_baro    = deque(maxlen=PLOT_POINTS)
buf_baro_ma = deque(maxlen=PLOT_POINTS)
lock        = threading.Lock()
alt_base    = None
t_start     = time.time()
score_history = []

log_active  = False
log_data    = []
log_t_start = 0.0

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
#  シリアル受信スレッド
# ============================================================
def serial_thread(ser):
    global alt_base, t_start, log_active, log_data, log_t_start

    ser.reset_input_buffer()
    t_start = time.time()
    print("Receiving... (waiting for first BMP value)")

    while True:
        try:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw or raw.startswith("READY"):
                continue

            parts = raw.split(",")
            if len(parts) < 4:
                continue
            baro = float(parts[3])

            if alt_base is None:
                if baro == 0.0:
                    continue
                alt_base = baro
                print(f"[Baseline set] {alt_base:.2f} m")

            baro_rel = baro - alt_base
            t = time.time() - t_start

            with lock:
                buf_time.append(t)
                buf_baro.append(baro_rel)
                recent = list(buf_baro)[-MA_N:]
                buf_baro_ma.append(sum(recent) / len(recent))

            if log_active:
                elapsed = time.time() - log_t_start
                ma_val  = list(buf_baro_ma)[-1]
                log_data.append((elapsed, baro_rel, ma_val))
                if elapsed >= T_TOTAL:
                    log_active = False
                    _save_log()

        except Exception:
            continue

# ============================================================
#  ログ保存＋解析
# ============================================================
def _save_log():
    if not log_data:
        return
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    fname = LOGS_DIR / f"step_{BMP_HZ}hz_ma{MA_N}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(fname, "w") as f:
        f.write("time_s,baro_raw_m,baro_ma_m\n")
        for row in log_data:
            f.write(f"{row[0]:.3f},{row[1]:.4f},{row[2]:.4f}\n")
    print(f"\n[Log saved] {fname.name}  (MA_N={MA_N}, delay≈{MA_N * round(1000 // BMP_HZ)}ms)")

    arr  = np.array(log_data)
    ts   = arr[:, 0]
    vals = arr[:, 2]  # MA値で解析

    phase0 = vals[ts <  T_DOWN]
    phase1 = vals[(ts >= T_DOWN) & (ts < T_UP)]
    phase2 = vals[ts >= T_UP]

    m0 = phase0.mean() if len(phase0) else float("nan")
    m1 = phase1.mean() if len(phase1) else float("nan")
    m2 = phase2.mean() if len(phase2) else float("nan")
    step = abs(m1 - m0)

    def response_time(t_event, direction):
        mask  = (ts >= t_event) & (ts < t_event + 5.0)
        t_seg = ts[mask]; v_seg = vals[mask]
        if len(t_seg) == 0:
            return float("nan")
        v_from = m0 if direction == "down" else m1
        v_to   = m1 if direction == "down" else m2
        target = v_from + (v_to - v_from) * 0.5
        cond   = (v_seg <= target) if direction == "down" else (v_seg >= target)
        idx    = np.argmax(cond)
        if not cond[idx]:
            return float("nan")
        return t_seg[idx] - t_event

    rt_down = response_time(T_DOWN, "down")
    rt_up   = response_time(T_UP,   "up")

    print(f"  Phase 0 (desk initial) avg : {m0:.3f} m")
    print(f"  Phase 1 (lower 15cm)   avg : {m1:.3f} m")
    print(f"  Phase 2 (desk return)  avg : {m2:.3f} m")
    print(f"  Step size              : {step:.3f} m  (ideal=0.15 m)")
    print(f"  50%% response time down: {rt_down:.2f} s")
    print(f"  50%% response time up  : {rt_up:.2f} s")

    log_data.clear()

# ============================================================
#  キーボードスレッド
# ============================================================
def keyboard_thread():
    global alt_base, log_active, log_data, log_t_start, MA_N

    while True:
        c = input().strip().upper()
        if c == "L":
            if log_active:
                print("[Log] already running")
            else:
                log_data.clear()
                log_t_start = time.time()
                log_active  = True
                print(f"[Log start] BMP_HZ={BMP_HZ}  MA_N={MA_N}  {T_TOTAL}s 自動終了")
                print(f"  0s  : このまま静止")
                print(f"  {T_DOWN:.0f}s  : 台に下ろす（-15cm）")
                print(f"  {T_UP:.0f}s : 机に戻す")
        elif c == "M":
            raw = input("MA_N (1=off, 推奨3〜15) > ").strip()
            try:
                n = int(raw)
                if 1 <= n <= 50:
                    MA_N = n
                    print(f"[MA] N={MA_N}  遅延≈{MA_N * round(1000 // BMP_HZ)}ms")
                else:
                    print("[MA] 1〜50 で入力してください")
            except ValueError:
                print("[MA] 数値を入力してください")
        elif c == "B":
            with lock:
                if buf_baro:
                    alt_base += list(buf_baro)[-1]
            print("[Baseline reset] Current altitude -> 0 m")
        elif c == "Q":
            sys.exit(0)

# ============================================================
#  グラフ
# ============================================================
fig, ax = plt.subplots(figsize=(12, 5))
fig.suptitle(f"BMP280  BMP_HZ={BMP_HZ}  |  L=Log(15s)  B=Baseline  M=MA_N  Q=Quit", fontsize=11)
ax.set_ylabel("[m]")
ax.set_xlabel("Time [s]")
ax.axhline(0.0, color="gray", lw=0.5)
ax.grid(True, alpha=0.3)

line_baro,    = ax.plot([], [], "b-", lw=0.8, alpha=0.4, label="baro raw")
line_baro_ma, = ax.plot([], [], "r-", lw=1.8,             label=f"baro MA(N={MA_N})")
info_text = ax.text(0.02, 0.92, "", transform=ax.transAxes,
                    fontsize=9, bbox=dict(boxstyle="round", fc="white", alpha=0.8))
log_text  = ax.text(0.98, 0.92, "", transform=ax.transAxes,
                    fontsize=10, ha="right", color="red",
                    bbox=dict(boxstyle="round", fc="lightyellow", alpha=0.9))
ax.legend(loc="upper left", fontsize=8, bbox_to_anchor=(0.0, 0.85))

def update_plot(_):
    with lock:
        t   = np.array(buf_time)
        br  = np.array(buf_baro)
        bma = np.array(buf_baro_ma)
        sh  = list(score_history)

    if len(t) < 2:
        return line_baro, line_baro_ma, info_text, log_text

    t_plot = t - t[-1]
    ax.set_xlim(-PLOT_SECONDS, 0)
    margin = max(0.3, (br.max() - br.min()) * 0.2)
    ax.set_ylim(br.min() - margin, br.max() + margin)
    line_baro.set_data(t_plot, br)
    line_baro_ma.set_data(t_plot, bma)
    line_baro_ma.set_label(f"baro MA(N={MA_N}, delay≈{MA_N * round(1000 // BMP_HZ)}ms)")
    ax.legend(loc="upper left", fontsize=8, bbox_to_anchor=(0.0, 0.85))

    # 安定性スコア（MA値で計算）
    mask  = t_plot >= -10.0
    bma10 = bma[mask]
    if len(bma10) > 5:
        pp    = bma10.max() - bma10.min()
        std   = np.std(bma10)
        score = max(0, min(100, round(100 * (1 - std / 0.3))))
        with lock:
            score_history.append(score)
        avg_score = round(sum(sh) / len(sh)) if sh else score

        if   score >= 80: grade, color = "EXCELLENT", "green"
        elif score >= 55: grade, color = "GOOD",      "blue"
        elif score >= 30: grade, color = "FAIR",      "orange"
        else:             grade, color = "POOR",      "red"

        info_text.set_text(
            f"current={bma[-1]:.3f} m  |  10s: P-P={pp:.3f} m  std={std:.3f} m\n"
            f"BMP {BMP_HZ} Hz  MA N={MA_N} (delay≈{MA_N * round(1000 // BMP_HZ)}ms)  |  "
            f"Stability: {score}/100 [{grade}]  avg: {avg_score}/100  (n={len(sh)})"
        )
        info_text.set_color(color)

    # ログ進行表示
    if log_active:
        elapsed = time.time() - log_t_start
        if   elapsed < T_DOWN: phase_msg = f"静止中... あと{T_DOWN - elapsed:.1f}s で台へ"
        elif elapsed < T_UP:   phase_msg = f"台の上... あと{T_UP   - elapsed:.1f}s で机へ"
        else:                  phase_msg = f"机に戻す... 終了まで{T_TOTAL - elapsed:.1f}s"
        log_text.set_text(f"● REC {elapsed:.1f}s / {T_TOTAL:.0f}s\n{phase_msg}")
    else:
        log_text.set_text("")

    return line_baro, line_baro_ma, info_text, log_text

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

    print("L=Log(15s)  B=BaselineReset  M=MA_N  Q=Quit\n")

    threading.Thread(target=serial_thread,  args=(ser,), daemon=True).start()
    threading.Thread(target=keyboard_thread,              daemon=True).start()

    ani = animation.FuncAnimation(fig, update_plot, interval=100,
                                  blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()