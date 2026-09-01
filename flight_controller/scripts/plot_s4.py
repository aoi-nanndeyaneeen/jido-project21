# scripts/plot_s4.py
# s4 のログを時系列で見る。角度復帰の挙動を1回分ズームして確認する用。
#
#   python scripts/plot_s4.py                       # 最新ログ全体
#   python scripts/plot_s4.py logs/log_005_*.csv    # ファイル指定
#   python scripts/plot_s4.py --t 12 18             # 12〜18秒だけ拡大
#   python scripts/plot_s4.py --axis pitch          # pitch を見る (既定: roll)
#
#   pip install numpy matplotlib

import sys
import argparse
from pathlib import Path
import numpy as np

LOGS_DIR = Path(__file__).parent.parent / "logs"

# firmware の S4::ANGLE_OUT_LIMIT と合わせる（表示用の目安線）
ANGLE_OUT_LIMIT = 300.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", help="ログCSV (省略時は logs/ の最新)")
    ap.add_argument("--axis", default="roll", choices=["roll", "pitch"])
    ap.add_argument("--t", nargs=2, type=float, metavar=("T0", "T1"),
                    help="表示する時間範囲 [s]")
    args = ap.parse_args()

    if args.path:
        path = Path(args.path)
    else:
        cands = sorted(LOGS_DIR.glob("log_*.csv"), key=lambda p: p.stat().st_mtime)
        if not cands:
            sys.exit("[ERROR] logs/ にログがありません")
        path = cands[-1]

    d = np.genfromtxt(path, delimiter=",", names=True)
    ax = args.axis
    for col in (f"{ax}_ratetar", f"{ax}_angtar"):
        if col not in d.dtype.names:
            sys.exit(f"[ERROR] 列 {col} がありません。s4 を新しい firmware で焼き直してください。")

    t = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    m = np.ones_like(t, dtype=bool)
    if args.t:
        m = (t >= args.t[0]) & (t <= args.t[1])

    t = t[m]
    ang    = d[f"{ax}_ang"][m]
    angtar = d[f"{ax}_angtar"][m]
    ratetar = d[f"{ax}_ratetar"][m]
    gyr    = d[f"{ax}_gyr"][m]
    cmd    = d[f"{ax}_cmd"][m]
    thr    = d["thr"][m]
    mot    = [d[f"m{i}"][m] for i in range(1, 5)]

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib が要ります: pip install matplotlib")

    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)

    axes[0].plot(t, ang, label=f"{ax}_ang (実測)", lw=1.0)
    axes[0].plot(t, angtar, "--", label=f"{ax}_angtar (目標)", lw=1.0)
    axes[0].axhline(45, color="r", ls=":", lw=0.8); axes[0].axhline(-45, color="r", ls=":", lw=0.8)
    axes[0].set_ylabel("angle [deg]"); axes[0].legend(loc="upper right"); axes[0].grid(alpha=0.3)

    axes[1].plot(t, ratetar, "--", label=f"{ax}_ratetar (角度ループ出力)", lw=1.0)
    axes[1].plot(t, gyr, label=f"{ax}_gyr (実測角速度)", lw=1.0)
    axes[1].axhline(ANGLE_OUT_LIMIT, color="r", ls=":", lw=0.8)
    axes[1].axhline(-ANGLE_OUT_LIMIT, color="r", ls=":", lw=0.8)
    axes[1].set_ylabel("rate [deg/s]"); axes[1].legend(loc="upper right"); axes[1].grid(alpha=0.3)

    axes[2].plot(t, cmd, label=f"{ax}_cmd", lw=1.0)
    axes[2].plot(t, thr, label="thr", lw=0.8, color="gray")
    axes[2].set_ylabel("cmd / thr"); axes[2].legend(loc="upper right"); axes[2].grid(alpha=0.3)

    for i, mm in enumerate(mot):
        axes[3].plot(t, mm, label=f"m{i+1}", lw=0.8)
    axes[3].set_ylabel("motor [0-1]"); axes[3].set_xlabel("time [s]")
    axes[3].legend(loc="upper right", ncol=4); axes[3].grid(alpha=0.3)

    axes[0].set_title(f"{path.name}   axis={ax}")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
