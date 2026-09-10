# scripts/tune_s5.py
# s5 の PID 調整用。ループごとに「目標 vs 実測 + 誤差」を段で並べる。
#
#   python scripts/tune_s5.py                       # 最新ログ、全ループを画面表示
#   python scripts/tune_s5.py --loop att            # 姿勢 (角度/レート/出力) だけ
#   python scripts/tune_s5.py --loop alt            # 高度ホールド
#   python scripts/tune_s5.py --loop flow           # フロー位置ホールド
#   python scripts/tune_s5.py logs/log_022_*.csv --t 2 6
#   python scripts/tune_s5.py --save               # logs/<name>_tune_<loop>.png に保存
#
# ゲイン (kp/ki/kd) はログに入っていないので、値はファーム側の
# シリアル "PID Tuning" メニュー表示と突き合わせること。
#
#   pip install numpy matplotlib

import sys
import argparse
from pathlib import Path
import numpy as np

LOGS_DIR = Path(__file__).parent.parent / "logs"

ALT_TARGET_HINT = 0.50  # 表示用の目安線 (今回の試験の目標高度)


def pick_log(path_arg):
    if path_arg:
        return Path(path_arg)
    cands = sorted(LOGS_DIR.glob("log_*.csv"), key=lambda p: p.stat().st_mtime)
    if not cands:
        sys.exit("[ERROR] logs/ にログがありません")
    return cands[-1]


def setup_font(matplotlib):
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for fam in ("Yu Gothic", "Meiryo", "MS Gothic", "Noto Sans CJK JP"):
        if fam in have:
            matplotlib.rcParams["font.family"] = fam
            break
    matplotlib.rcParams["axes.unicode_minus"] = False


# ループ定義。
#   panels: (段タイトル, 単位, [(列, 凡例, スタイル), ...], 誤差ペア or None)
#   誤差ペア = (実測列, 目標列)  → 別トレースで resp-tar を薄く重ねる
LOOPS = {
    "att": ("姿勢 PID (角度ループ → レートループ → トルク)", [
        ("Roll 角度", "deg", [
            ("roll_angtar", "目標 angtar", "--"),
            ("roll_ang", "実測 ang", "-")], ("roll_ang", "roll_angtar")),
        ("Roll レート", "deg/s", [
            ("roll_ratetar", "目標 ratetar", "--"),
            ("roll_gyr", "実測 gyr", "-")], ("roll_gyr", "roll_ratetar")),
        ("Pitch 角度", "deg", [
            ("pitch_angtar", "目標 angtar", "--"),
            ("pitch_ang", "実測 ang", "-")], ("pitch_ang", "pitch_angtar")),
        ("Pitch レート", "deg/s", [
            ("pitch_ratetar", "目標 ratetar", "--"),
            ("pitch_gyr", "実測 gyr", "-")], ("pitch_gyr", "pitch_ratetar")),
        ("Yaw ヘディング保持誤差 / 角速度", "deg, deg/s", [
            ("yaw_ang", "yaw_ang (保持誤差)", "-"),
            ("yaw_gyr", "yaw_gyr", "-")], None),
        ("トルク指令 (ミキサー入力)", "-", [
            ("roll_cmd", "roll_cmd", "-"),
            ("pitch_cmd", "pitch_cmd", "-"),
            ("yaw_cmd", "yaw_cmd", "-")], None),
    ]),
    "alt": ("高度ホールド PID (位置ループ → 上昇速度PID → スロットル補正)", [
        ("高度", "m", [
            ("alt_hold", "目標 alt_hold", "--"),
            ("est_h", "推定 est_h", "-"),
            ("range_h", "測距 range_h", "-")], ("est_h", "alt_hold")),
        ("上昇速度", "m/s", [
            ("alt_vzt", "目標 alt_vzt (位置ループ出力)", "--"),
            ("est_vz", "推定 est_vz", "-"),
            ("climb", "climb", "-")], ("est_vz", "alt_vzt")),
        ("スロットル", "-", [
            ("alt_base", "alt_base (ホバリング)", "--"),
            ("alt_corr", "alt_corr (PID補正)", "-"),
            ("alt_used", "alt_used (最終)", "-")], None),
        ("推定加速度バイアス", "m", [
            ("est_bias", "est_bias", "-")], None),
    ]),
    "flow": ("フロー位置ホールド PID (位置ループ → 速度PID → リーン角)", [
        ("位置誤差", "m", [
            ("fh_posn", "north fh_posn", "-"),
            ("fh_pose", "east fh_pose", "-")], None),
        ("速度 X (body)", "m/s", [
            ("fh_vxt", "目標 fh_vxt (位置ループ出力)", "--"),
            ("flow_vx", "生 flow_vx", "-"),
            ("fh_vxc", "補正後 fh_vxc", "-")], ("fh_vxc", "fh_vxt")),
        ("速度 Y (body)", "m/s", [
            ("fh_vyt", "目標 fh_vyt", "--"),
            ("flow_vy", "生 flow_vy", "-"),
            ("fh_vyc", "補正後 fh_vyc", "-")], ("fh_vyc", "fh_vyt")),
        ("リーン角指令 (角度ループへ)", "deg", [
            ("fh_leanr", "fh_leanr (roll)", "-"),
            ("fh_leanp", "fh_leanp (pitch)", "-")], None),
    ]),
}


def need(d, loop_key):
    cols = set()
    for _, _, series, errpair in LOOPS[loop_key][1]:
        cols.update(c for c, _, _ in series)
        if errpair:
            cols.update(errpair)
    miss = sorted(c for c in cols if c not in d.dtype.names)
    if miss:
        sys.exit(f"[ERROR] loop={loop_key}: 列 {miss} がありません。"
                 "s5 を新しい firmware で焼き直してください。")


def draw_loop(plt, loop_key, d, m, t, path, save):
    title, panels = LOOPS[loop_key]
    fig, axes = plt.subplots(len(panels), 1,
                             figsize=(12, 2.1 * len(panels) + 1), sharex=True)
    if len(panels) == 1:
        axes = [axes]
    fig.suptitle(f"{title}\n{path.name}", fontsize=10)

    print(f"\n=== loop={loop_key}  [{path.name}]  {t[-1] - t[0]:.1f}s / {t.size} 点 ===")
    for ax, (ptitle, unit, series, errpair) in zip(axes, panels):
        for col, label, style in series:
            ax.plot(t, d[col][m], style, lw=1.0, label=label)
        if errpair:
            resp, tar = errpair
            e = d[resp][m] - d[tar][m]
            ax.plot(t, e, "-", color="0.55", lw=0.8, label="誤差 (実測-目標)")
            print(f"  {ptitle:24s} 誤差 RMS {np.sqrt((e ** 2).mean()):7.3f}  "
                  f"平均 {e.mean():+7.3f}  最大|e| {np.abs(e).max():7.3f}  [{unit}]")
        if loop_key == "alt" and ptitle == "高度":
            ax.axhline(ALT_TARGET_HINT, color="r", ls=":", lw=0.7)
        ax.axhline(0, color="0.8", lw=0.6, zorder=0)
        ax.set_title(ptitle, loc="left", fontsize=9)
        ax.set_ylabel(f"[{unit}]")
        ax.legend(loc="upper right", fontsize=8, ncol=4)
        ax.grid(alpha=0.3)

    axes[-1].set_xlabel("t [s]")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.subplots_adjust(hspace=0.35)

    if save:
        out = path.with_name(f"{path.stem}_tune_{loop_key}.png")
        fig.savefig(out, dpi=130)
        print(f"  saved -> {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", help="ログCSV (省略時は logs/ の最新)")
    ap.add_argument("--loop", default="all",
                    choices=["all", "att", "alt", "flow"],
                    help="表示するループ (既定: all)")
    ap.add_argument("--t", nargs=2, type=float, metavar=("T0", "T1"),
                    help="表示する時間範囲 [s]")
    ap.add_argument("--save", action="store_true",
                    help="表示せず PNG 保存 (logs/<name>_tune_<loop>.png)")
    args = ap.parse_args()

    path = pick_log(args.path)
    d = np.genfromtxt(path, delimiter=",", names=True)

    t_all = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    m = np.ones_like(t_all, dtype=bool)
    if args.t:
        m = (t_all >= args.t[0]) & (t_all <= args.t[1])
    t = t_all[m]
    if t.size < 2:
        sys.exit("[ERROR] 指定区間にデータがありません")

    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib が要ります: pip install matplotlib")
    setup_font(matplotlib)

    loops = ["att", "alt", "flow"] if args.loop == "all" else [args.loop]
    for k in loops:
        need(d, k)
        draw_loop(plt, k, d, m, t, path, args.save)

    if not args.save:
        plt.show()


if __name__ == "__main__":
    main()
