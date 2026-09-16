"""
plot_3d.py  -  フライトデータ時系列プロッター
使い方: python tools/plot_3d.py tools/logs/YYYYMMDD_HHMMSS.csv
"""

import csv
import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def main():
    parser = argparse.ArgumentParser(description="Flight Data Time-Series Plotter")
    parser.add_argument("csv_file", type=str, help="Path to the logged CSV file")
    args = parser.parse_args()

    times, roll, pitch, yaw, alt = [], [], [], [], []

    try:
        with open(args.csv_file, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    times.append(float(row["Time"]))
                    roll.append(float(row["Roll(deg)"]))
                    pitch.append(float(row["Pitch(deg)"]))
                    yaw.append(float(row["Yaw(deg)"]))
                    alt.append(float(row["Alt(m)"]))
                except (ValueError, KeyError):
                    continue
    except FileNotFoundError:
        print(f"エラー: ファイル '{args.csv_file}' が見つかりません。")
        sys.exit(1)

    if len(times) == 0:
        print("データがありませんでした。")
        sys.exit(1)

    t = np.array(times)
    roll  = np.array(roll)
    pitch = np.array(pitch)
    yaw   = np.array(yaw)
    alt   = np.array(alt)

    print(f"読み込み完了: {len(t)} レコード  "
          f"({t[-1]:.1f}s  Roll {roll.min():.1f}~{roll.max():.1f}deg  "
          f"Pitch {pitch.min():.1f}~{pitch.max():.1f}deg  "
          f"Alt {alt.min():.2f}~{alt.max():.2f}m)")

    fig = plt.figure(figsize=(12, 8))
    fig.canvas.manager.set_window_title("Flight Telemetry")
    fig.suptitle(args.csv_file.split("\\")[-1].split("/")[-1], fontsize=11)

    gs = gridspec.GridSpec(3, 1, hspace=0.45)

    # --- Roll ---
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, roll, color="#e05a5a", linewidth=1.2)
    ax1.axhline(0, color="gray", linewidth=0.6, linestyle="--")
    ax1.set_ylabel("Roll (deg)", fontsize=10)
    ax1.set_xlim(t[0], t[-1])
    ax1.grid(True, alpha=0.3)
    ax1.tick_params(labelbottom=False)

    # --- Pitch ---
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    ax2.plot(t, pitch, color="#5a9ee0", linewidth=1.2)
    ax2.axhline(0, color="gray", linewidth=0.6, linestyle="--")
    ax2.set_ylabel("Pitch (deg)", fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.tick_params(labelbottom=False)

    # --- Altitude ---
    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    ax3.plot(t, alt, color="#5ac88a", linewidth=1.2)
    ax3.set_ylabel("Altitude (m)", fontsize=10)
    ax3.set_xlabel("Time (s)", fontsize=10)
    ax3.grid(True, alpha=0.3)

    plt.show()

if __name__ == "__main__":
    main()