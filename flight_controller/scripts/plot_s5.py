# scripts/plot_s5.py
# s5 のホバリングログを時系列で見る。高度ホールド・フロー位置ホールドの挙動確認用。
#
#   python scripts/plot_s5.py                         # 最新ログ全体
#   python scripts/plot_s5.py logs/log_022_*.csv      # ファイル指定
#   python scripts/plot_s5.py --t 2 6                 # 2〜6秒だけ拡大
#   python scripts/plot_s5.py --save                  # 表示せず logs/<name>_s5.png に保存
#   python scripts/plot_s5.py --raw-ylim              # 外れ値でスケールを潰させる (旧挙動)
#
#   pip install numpy matplotlib

import sys
import argparse
from pathlib import Path
import numpy as np

LOGS_DIR = Path(__file__).parent.parent / "logs"

ALT_TARGET_HINT = 0.50  # 表示用の目安線（今回の試験の目標高度）


def pick_log(path_arg):
    if path_arg:
        return Path(path_arg)
    cands = sorted(LOGS_DIR.glob("log_*.csv"), key=lambda p: p.stat().st_mtime)
    if not cands:
        sys.exit("[ERROR] logs/ にログがありません")
    return cands[-1]


def need(d, *cols):
    miss = [c for c in cols if c not in d.dtype.names]
    if miss:
        sys.exit(f"[ERROR] 列 {miss} がありません。s5 を新しい firmware で焼き直してください。")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", help="ログCSV (省略時は logs/ の最新)")
    ap.add_argument("--t", nargs=2, type=float, metavar=("T0", "T1"),
                    help="表示する時間範囲 [s]")
    ap.add_argument("--save", action="store_true",
                    help="表示せず PNG 保存 (logs/<name>_s5.png)")
    ap.add_argument("--raw-ylim", action="store_true",
                    help="Y軸を外れ値まで含めて自動スケール (既定はロバスト範囲にクリップ)")
    args = ap.parse_args()

    path = pick_log(args.path)
    d = np.genfromtxt(path, delimiter=",", names=True)
    need(d, "t_ms", "alt_hold", "est_h", "range_h", "alt_corr", "est_bias",
         "alt_vzt", "est_vz", "climb", "roll_ang", "pitch_ang", "yaw_ang",
         "fh_posn", "fh_pose", "flow_vx", "flow_vy", "fh_vxc")

    t = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    m = np.ones_like(t, dtype=bool)
    if args.t:
        m = (t >= args.t[0]) & (t <= args.t[1])
    t = t[m]

    def g(k):
        return d[k][m]

    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib が要ります: pip install matplotlib")

    # 日本語フォント (無ければ豆腐になるが処理は通す)
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for fam in ("Yu Gothic", "Meiryo", "MS Gothic", "Noto Sans CJK JP"):
        if fam in have:
            matplotlib.rcParams["font.family"] = fam
            break
    matplotlib.rcParams["axes.unicode_minus"] = False

    # (title, unit, [(col, label, style)])
    panels = [
        ("高度", "m", [
            ("alt_hold", "目標 alt_hold", "--"),
            ("est_h", "推定 est_h", "-"),
            ("range_h", "測距 range_h", "-"),
        ]),
        ("高度制御 内部量", "m", [
            ("alt_corr", "alt_corr", "-"),
            ("est_bias", "est_bias", "-"),
        ]),
        ("上下速度", "m/s", [
            ("alt_vzt", "目標 alt_vzt", "--"),
            ("est_vz", "est_vz", "-"),
            ("climb", "climb", "-"),
        ]),
        ("姿勢角", "deg", [
            ("roll_ang", "roll", "-"),
            ("pitch_ang", "pitch", "-"),
            ("yaw_ang", "yaw", "-"),
        ]),
        ("フロー位置ホールド誤差", "m", [
            ("fh_posn", "north fh_posn", "-"),
            ("fh_pose", "east fh_pose", "-"),
        ]),
        ("フロー速度", "m/s", [
            ("flow_vx", "flow_vx", "-"),
            ("flow_vy", "flow_vy", "-"),
            ("fh_vxc", "fh_vxc (補正後)", "-"),
        ]),
    ]

    fig, axes = plt.subplots(len(panels), 1, figsize=(12, 13), sharex=True)
    fig.suptitle(path.name, fontsize=10)

    # パネルごとの「物理的にありえる範囲」。離着陸直後に est_h / est_vz が
    # 数十 m 単位で発散したり、着地後に測距が飛んだりするので、この帯の外は
    # スケール決定から除外する (描画自体はする)。--raw-ylim で無効化。
    SANE = {
        "高度": (-0.3, 3.0),
        "高度制御 内部量": (-0.6, 0.6),
        "上下速度": (-3.0, 3.0),
        "姿勢角": (-60.0, 60.0),
        "フロー位置ホールド誤差": (-1.5, 1.5),
        "フロー速度": (-3.0, 3.0),
    }

    def robust_ylim(series_vals, sane, pad=0.08, floor=1e-3, extra=()):
        v = np.concatenate([np.asarray(x, float).ravel() for x in series_vals])
        v = v[np.isfinite(v)]
        if sane is not None:
            inside = v[(v >= sane[0]) & (v <= sane[1])]
            if inside.size:
                v = inside
        if v.size == 0:
            return None
        lo, hi = float(v.min()), float(v.max())
        for e in extra:
            lo, hi = min(lo, e), max(hi, e)
        span = hi - lo
        if span < floor:
            lo, hi, span = lo - floor, hi + floor, 2 * floor
        return lo - pad * span, hi + pad * span

    for ax, (title, unit, series) in zip(axes, panels):
        for col, label, style in series:
            ax.plot(t, g(col), style, lw=1.0, label=label)
        extra = (ALT_TARGET_HINT,) if title == "高度" else ()
        if title == "高度":
            ax.axhline(ALT_TARGET_HINT, color="r", ls=":", lw=0.7)
        if not args.raw_ylim:
            yl = robust_ylim([g(c) for c, _, _ in series],
                             SANE.get(title), extra=extra)
            if yl:
                ax.set_ylim(*yl)
        ax.set_ylabel(f"{title}\n[{unit}]")
        ax.legend(loc="upper right", fontsize=8, ncol=3)
        ax.grid(alpha=0.3)

    axes[-1].set_xlabel("t [s]")
    fig.tight_layout(rect=(0, 0, 1, 0.98))

    # サマリ（高度ホールド誤差など）
    eh, ah = d["est_h"][m], d["alt_hold"][m]
    err = np.abs(eh - ah)
    print(f"[{path.name}]  {t[-1] - t[0]:.1f}s / {t.size} 点")
    eh_fin = eh[np.isfinite(eh)]
    eh_hi = np.percentile(eh_fin, 99.7) if eh_fin.size else float("nan")
    print(f"  高度誤差   平均 {err.mean():.3f} m  最大 {err.max():.3f} m  "
          f"est_h p99.7 {eh_hi:.3f} m (生の最大 {eh.max():.3f} m)")
    print(f"  est_bias   {d['est_bias'][m][0]:.3f} -> {d['est_bias'][m][-1]:.3f} m")
    print(f"  位置ドリフト fh_posn {d['fh_posn'][m].min():.2f} m  fh_pose {d['fh_pose'][m].min():.2f} m")
    if "acc_up" in d.dtype.names:
        au = d["acc_up"][m]
        print(f"  acc_up RMS ±{np.sqrt((au ** 2).mean()):.1f}")

    if args.save:
        out = path.with_name(path.stem + "_s5.png")
        fig.savefig(out, dpi=130)
        print(f"  saved -> {out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
