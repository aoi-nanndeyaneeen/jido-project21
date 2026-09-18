# scripts/analyze_attitude_est.py
# 姿勢推定 (Madgwick) が「ジャイロを積分した角度」に対してどれだけ縮んで・遅れて
# 出ているかを、飛行ログから帯域ごとに出す。RP2040 移行 (2026-09-17) で見つかった
# 2 つの問題を、次の便で直ったか確かめるための道具。
#
#   python scripts/analyze_attitude_est.py                  # 最新の LOG*.BIN / CSV
#   python scripts/analyze_attitude_est.py path/to/LOG0064.BIN
#   python scripts/analyze_attitude_est.py LOG0064.csv --seg 130 170   # 秒で区間指定
#
# ------------------------------------------------------------------------
#  何を見るか
# ------------------------------------------------------------------------
#   H(f) = 推定角 (roll_ang / pitch_ang) ÷ ∫ジャイロ (roll_gyr / pitch_gyr を実時刻で積分)
#   を 0.1〜15Hz の帯域ごとに複素最小二乗で出す (ゲインと位相)。
#
#   正しい姿勢推定ならジャイロ帯域 (>1Hz) でゲイン 1.0 / 位相 0 に近い。
#     ・2〜6Hz のゲインが 0.6〜0.7  → Madgwick の積分 dt が実際より短い
#         (RP2040 で固定 1ms のまま実効 1.5ms で回っていた。sensor/IMU.h。
#          LOG0056〜0064 で 0.57〜0.70。dt_us の平均の逆数に一致する)
#     ・0.3〜0.7Hz のゲインが 0.3〜0.6 / 位相 +30〜+46deg
#                                       → 加速度補正 (beta) が強すぎて、横移動の
#          加速度に推定角が引かれている (QuadConfig.h IMU_FUSION_BETA_FLIGHT)。
#          位置ループは「2度傾けろ」と言って本当は 4度傾いていて 46deg 遅れる =
#          0.3〜0.45Hz の横揺れ (リミットサイクル) の隠れた原因。
#
#   ついでに出すもの:
#     ・ループ周期 dt_us の統計 (1000Hz 目標に対する実効)
#     ・ホールド区間 (mode>=2) での roll_ang / fh_leanr / fh_vyc のピーク周波数
#       (横揺れが何 Hz か。位置ループの揺れなら 3 つが同じ周波数になる)
#
#   期待値 (2026-09-18 の修正後): 2〜6Hz で 0.95〜1.0、0.3〜0.7Hz で 0.9 以上 / 位相 +20deg 以内。
#   まだ 0.3〜0.7Hz が低ければ IMU_FUSION_BETA_FLIGHT をさらに下げる (0.03 → 0.02 → 0.01)。
#
#   pip install numpy

import argparse
import csv
import importlib.util
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
SEARCH_DIRS = [
    REPO / "log_recorder" / "scripts" / "logs",
    REPO / "position_estimator" / "src" / "logs",
    REPO / "flight_controller" / "logs",
]
COLS = ("t_ms", "dt_us", "mode", "armed",
        "roll_ang", "pitch_ang", "roll_gyr", "pitch_gyr",
        "fh_leanr", "fh_leanp", "fh_vxc", "fh_vyc")
BANDS = ((0.1, 0.3), (0.3, 0.7), (0.7, 2.0), (2.0, 6.0), (6.0, 15.0))
MIN_SEG_S = 8.0     # これより短い区間は帯域分解できない


# ------------------------------------------------------------------------
#  読み込み (analyze_circle.py と同じ流儀)
# ------------------------------------------------------------------------
def pick_log(arg):
    if arg:
        p = Path(arg)
        if not p.exists():
            sys.exit(f"[ERROR] 見つかりません: {arg}")
        return p
    cands = []
    for d in SEARCH_DIRS:
        if d.is_dir():
            cands += list(d.glob("LOG*.BIN")) + list(d.glob("LOG*.bin")) + list(d.glob("*.csv"))
    cands = [c for c in cands if c.suffix.lower() == ".bin" or _csv_has_rec(c)]
    if not cands:
        sys.exit("[ERROR] ログが見つかりません。パスを指定してください")
    return max(cands, key=lambda p: p.stat().st_mtime)


def _csv_has_rec(p):
    try:
        with p.open(encoding="utf-8") as f:
            return "roll_gyr" in f.readline()
    except (OSError, UnicodeDecodeError):
        return False


def load(path):
    if path.suffix.lower() == ".bin":
        spec = importlib.util.spec_from_file_location("s5_bin2csv", HERE / "bin2csv.py")
        b2c = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(b2c)
        try:
            rows, _ = b2c.decode_bin(path)
        except ValueError as e:
            sys.exit(f"[ERROR] {e}")
        lines = [b2c.HEADER] + rows
    else:
        lines = path.read_text(encoding="utf-8").splitlines()
        if lines and lines[0].startswith("HEADER,"):
            lines[0] = lines[0][len("HEADER,"):]
        lines = [ln[len("DATA,"):] if ln.startswith("DATA,") else ln for ln in lines]
    rd = csv.DictReader(lines)
    missing = [c for c in COLS if c not in (rd.fieldnames or [])]
    if missing:
        sys.exit(f"[ERROR] 列がありません: {missing}")
    data = {c: [] for c in COLS}
    for r in rd:
        try:
            vals = [float(r[c]) for c in COLS]
        except (TypeError, ValueError):
            continue
        for c, v in zip(COLS, vals):
            data[c].append(v)
    return {c: np.asarray(v) for c, v in data.items()}


# ------------------------------------------------------------------------
#  解析
# ------------------------------------------------------------------------
def transfer(x, y, fs, lo, hi):
    """y ≈ H·x を帯域 [lo,hi] Hz で複素最小二乗。戻り値 (|H|, 位相[deg], コヒーレンス)"""
    n = len(x)
    w = np.hanning(n)
    X = np.fft.rfft((x - x.mean()) * w)
    Y = np.fft.rfft((y - y.mean()) * w)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    b = (f >= lo) & (f <= hi)
    if not b.any() or (np.abs(X[b]) ** 2).sum() == 0:
        return float("nan"), float("nan"), 0.0
    cross = (Y[b] * np.conj(X[b])).sum()
    H = cross / (np.abs(X[b]) ** 2).sum()
    coh = abs(cross) ** 2 / ((np.abs(X[b]) ** 2).sum() * (np.abs(Y[b]) ** 2).sum())
    return abs(H), float(np.degrees(np.angle(H))), float(coh)


def peaks(x, fs, lo=0.1, hi=5.0, k=3):
    n = len(x)
    F = np.fft.rfft((x - x.mean()) * np.hanning(n))
    f = np.fft.rfftfreq(n, 1.0 / fs)
    P = np.abs(F) ** 2
    b = (f >= lo) & (f <= hi)
    if not b.any() or P[b].sum() == 0:
        return []
    idx = np.argsort(P[b])[::-1][:k]
    return [(float(f[b][i]), float(P[b][i] / P[b].sum() * 100)) for i in idx]


def longest_run(mask):
    idx = np.where(mask)[0]
    if len(idx) == 0:
        return None
    segs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
    return max(segs, key=len)


def analyze_segment(d, sel, label):
    t = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    tt = t[sel]
    if len(tt) < 20 or tt[-1] - tt[0] < MIN_SEG_S:
        print(f"  ({label}: {tt[-1] - tt[0] if len(tt) else 0:.0f}s は短すぎるので省略)")
        return
    fs = 1.0 / np.median(np.diff(tt))
    dt = d["dt_us"][sel]
    print(f"\n== {label}: {tt[0]:.0f}〜{tt[-1]:.0f}s  ログ実効 {fs:.0f}Hz  "
          f"ループ dt 平均 {dt.mean():.0f}us (実効 {1e6 / dt.mean():.0f}Hz) p90 {np.quantile(dt, .9):.0f} "
          f"p99 {np.quantile(dt, .99):.0f} 最大 {dt.max():.0f}")
    print("   帯域[Hz]     roll  |H|  位相    coh  |  pitch |H|  位相    coh")
    for lo, hi in BANDS:
        row = f"   {lo:4.1f}-{hi:4.1f}  "
        for ax in ("roll", "pitch"):
            g = d[ax + "_gyr"][sel]
            a = d[ax + "_ang"][sel]
            ig = np.concatenate([[0.0], np.cumsum(0.5 * (g[1:] + g[:-1]) * np.diff(tt))])
            gain, ph, coh = transfer(ig, a, fs, lo, hi)
            row += f"   {gain:5.2f} {ph:+5.0f}  {coh:4.2f}  |"
        print(row)
    print("   (|H| = 推定角 / ∫ジャイロ。1.0 が理想。coh が低い行は信用しない)")

    if (d["mode"][sel] >= 2).mean() > 0.5:
        print("   揺れのピーク (Hz, 帯域内パワー%):")
        for c in ("roll_ang", "fh_leanr", "fh_vyc", "pitch_ang", "fh_leanp", "fh_vxc"):
            pk = ", ".join(f"{f:.2f}Hz {p:.0f}%" for f, p in peaks(d[c][sel], fs))
            print(f"     {c:9s} std {d[c][sel].std():5.2f}  {pk}")


def main():
    ap = argparse.ArgumentParser(description="姿勢推定 (Madgwick) の縮み/遅れをログから出す")
    ap.add_argument("path", nargs="?", help="LOGnnnn.BIN か CSV (省略で最新)")
    ap.add_argument("--seg", nargs=2, type=float, metavar=("T0", "T1"),
                    help="解析する区間 [s] (省略でモードごとの最長区間)")
    a = ap.parse_args()

    path = pick_log(a.path)
    print(f"[INFO] {path}")
    d = load(path)
    t = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    print(f"[INFO] {t[-1]:.0f}s  {len(t)} 行  ループ dt 平均 {d['dt_us'].mean():.0f}us "
          f"(実効 {1e6 / d['dt_us'].mean():.0f}Hz)  モード内訳: "
          + ", ".join(f"{int(m)}:{int(c)}" for m, c in zip(*np.unique(d['mode'].astype(int), return_counts=True))))

    if a.seg:
        sel = (t >= a.seg[0]) & (t < a.seg[1])
        analyze_segment(d, sel, f"指定区間")
        return

    for mode, name in ((1, "ANGLE"), (3, "POSHOLD"), (2, "GUIDED")):
        run = longest_run((d["mode"] == mode) & (d["armed"] == 1))
        if run is None:
            continue
        sel = np.zeros(len(t), bool)
        sel[run] = True
        analyze_segment(d, sel, name)
    hold = longest_run((d["mode"] >= 2) & (d["armed"] == 1))
    if hold is not None and len(hold) > 20:
        sel = np.zeros(len(t), bool)
        sel[hold] = True
        analyze_segment(d, sel, "ホールド (POSHOLD+GUIDED 連続)")


if __name__ == "__main__":
    main()
