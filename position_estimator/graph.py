"""
graph.py
logs/ 内の FlightLogger 出力（flight_*.csv）を読み込み、
XY水平軌跡と高度・誤差の時系列グラフを表示する。

使い方:
    python graph.py                  # logs/ 内の最新 flight_*.csv を自動選択
    python graph.py <ファイル名>      # ファイルを明示指定
"""

import csv
import sys
from pathlib import Path
import matplotlib.pyplot as plt

LOG_DIR = Path(__file__).parent / "logs"


def find_log_file():
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
        return path if path.is_absolute() else LOG_DIR / path

    candidates = sorted(LOG_DIR.glob("flight_*.csv"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        return None
    return candidates[-1]


def load_log(path: Path):
    times, detected = [], []
    pos_x, pos_y, pos_z = [], [], []
    residual = []

    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(row["Time"])
            is_detected = row["Detected"] == "1"
            detected.append(is_detected)
            pos_x.append(float(row["Pos_X(m)"]) if is_detected else None)
            pos_y.append(float(row["Pos_Y(m)"]) if is_detected else None)
            pos_z.append(float(row["Pos_Z(m)"]))
            residual.append(float(row["Residual(m)"]))

    return times, detected, pos_x, pos_y, pos_z, residual


def main():
    path = find_log_file()
    if path is None or not path.exists():
        print(f"エラー: 読み込めるログ (flight_*.csv) が {LOG_DIR} に見つかりません。")
        return

    print(f"{path} を読み込んでいます...")
    times, detected, pos_x, pos_y, pos_z, residual = load_log(path)

    xs = [x for x in pos_x if x is not None]
    ys = [y for y in pos_y if y is not None]
    x_indices = range(len(times))

    fig, (ax_xy, ax_z) = plt.subplots(1, 2, figsize=(12, 6))
    fig.canvas.manager.set_window_title(f"Flight Data Analysis - {path.name}")

    # --- 左: XY水平軌跡（検知できたフレームのみ） ---
    if xs and ys:
        ax_xy.plot(xs, ys, color="#1D9E75", linewidth=1.5)
        ax_xy.scatter(xs[-1], ys[-1], color="red", zorder=5, label="Last")
        ax_xy.legend(loc="upper right")
    ax_xy.set_title("Horizontal Trajectory (X, Y)", fontsize=14)
    ax_xy.set_xlabel("X (m)", fontsize=12)
    ax_xy.set_ylabel("Y (m)", fontsize=12)
    ax_xy.set_aspect("equal")
    ax_xy.grid(True, linestyle="--", alpha=0.7)

    # --- 右: 高度・誤差の時系列 ---
    ax_z.plot(x_indices, pos_z, label="Altitude Z (m)", color="purple", linewidth=2)
    ax_res = ax_z.twinx()
    ax_res.plot(x_indices, residual, label="Residual (m)", color="#FF5733",
                linewidth=1, alpha=0.5)

    ax_z.set_title("Altitude & Triangulation Residual", fontsize=14)
    ax_z.set_xlabel("Data Points (Time ->)", fontsize=12)
    ax_z.set_ylabel("Altitude (m)", fontsize=12, color="purple")
    ax_res.set_ylabel("Residual (m)", fontsize=12, color="#FF5733")
    ax_z.grid(True, linestyle="--", alpha=0.7)
    ax_z.axhline(0, color="black", linewidth=1, linestyle="-")

    lines1, labels1 = ax_z.get_legend_handles_labels()
    lines2, labels2 = ax_res.get_legend_handles_labels()
    ax_z.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

    plt.tight_layout()
    print("グラフを描画しました！ウィンドウ下部の虫眼鏡マークで拡大できます。")
    plt.show()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"エラーが発生しました: {e}")
