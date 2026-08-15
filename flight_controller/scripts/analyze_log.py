# scripts/analyze_log.py
# logs/ のCSVを読んで「振動の原因はどこか」を数字で出す
#
# 【使い方】
#   python scripts/analyze_log.py                  # 最新のログを自動で選ぶ
#   python scripts/analyze_log.py logs/log_007.csv # ファイル指定
#   python scripts/analyze_log.py --plot           # グラフも出す
#
# 【必要パッケージ】
#   pip install numpy
#   pip install matplotlib   # --plot を使う場合のみ

import sys
import argparse
from pathlib import Path

import numpy as np

LOGS_DIR = Path(__file__).parent.parent / "logs"


def load(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size == 0:
        sys.exit(f"[ERROR] データ行がありません: {path}")
    return data


def section(title):
    print()
    print("=" * 62)
    print(f"  {title}")
    print("=" * 62)


def check_loop_timing(d):
    section("1. 制御ループの実効レート")
    dt = d["dt_us"]
    hz = 1e6 / dt
    print(f"  dt      平均 {dt.mean():8.1f} us   "
          f"最小 {dt.min():6.0f}   最大 {dt.max():7.0f}")
    print(f"  実効Hz  平均 {hz.mean():8.1f} Hz   "
          f"最小 {hz.min():6.1f}   最大 {hz.max():7.1f}")
    print(f"  ジッタ  標準偏差 {dt.std():.1f} us")

    target = 1000.0
    if hz.mean() < target * 0.9:
        print(f"  → 想定の {target:.0f}Hz に届いていません "
              f"({hz.mean():.0f}Hz)。")
        print("     Madgwickフィルタは begin(MAIN_Hz) で 1000Hz 固定を前提に")
        print("     積分しているため、姿勢推定に誤差が出ます。")
    if dt.max() > dt.mean() * 3:
        print(f"  → 最大 {dt.max():.0f}us のコマ落ちがあります。")
        print("     この瞬間だけ制御が止まるので、周期的な脈動の原因になります。")


def check_saturation(d):
    section("2. 飽和 (ゲインが高すぎないか)")
    sat = d["sat"].astype(int)
    armed = d["armed"].astype(bool)
    if armed.sum() == 0:
        print("  ARMED のサンプルがありません。")
        return
    s = sat[armed]
    any_sat = (s != 0).mean() * 100
    print(f"  補正がクランプに当たっていた時間: {any_sat:5.1f} %")
    for i in range(4):
        pct = ((s >> i) & 1).mean() * 100
        print(f"    M{i+1}: {pct:5.1f} %")

    if any_sat > 30:
        print("  → 常時飽和しています。ゲインが高すぎます。")
        print("     この状態ではPIDは実質ON/OFF制御になり、必ず発振します。")
    elif any_sat > 5:
        print("  → 飽和が目立ちます。ゲインを下げる余地があります。")
    else:
        print("  → 飽和は許容範囲です。")

    # モーター出力が 0 / 1 に張り付いた割合
    print()
    for i, name in enumerate(["m1", "m2", "m3", "m4"]):
        m = d[name][armed]
        lo = (m <= 0.001).mean() * 100
        hi = (m >= 0.999).mean() * 100
        print(f"  {name}: 0に張り付き {lo:5.1f} %   1に張り付き {hi:5.1f} %")


def check_cmd_magnitude(d):
    section("3. PID出力の大きさ")
    armed = d["armed"].astype(bool)
    if armed.sum() == 0:
        return
    for ax in ["roll", "pitch", "yaw"]:
        c = d[f"{ax}_cmd"][armed]
        print(f"  {ax:6s} cmd: 平均 {c.mean():+8.4f}  "
              f"標準偏差 {c.std():7.4f}  "
              f"範囲 [{c.min():+7.3f}, {c.max():+7.3f}]")
    print()
    print("  目安: ホバリング付近では標準偏差 0.02〜0.10 程度が健全。")
    print("        0.3 を超えるならゲインが高すぎます。")


def check_gyro_noise(d):
    section("4. ジャイロのノイズ (振動)")
    # スロットルが低い=プロペラが回っていない区間と、回っている区間を比べる
    thr = d["thr"]
    quiet = thr < 0.05
    spin = thr > 0.15

    for ax in ["roll", "pitch", "yaw"]:
        g = d[f"{ax}_gyr"]
        q = g[quiet].std() if quiet.sum() > 50 else float("nan")
        s = g[spin].std() if spin.sum() > 50 else float("nan")
        print(f"  {ax:6s} gyro 標準偏差: "
              f"静止時 {q:7.3f}   回転時 {s:7.3f}  [deg/s]")

    print()
    print("  静止時が 1 deg/s を大きく超えるならセンサ/取付の問題。")
    print("  回転時だけ大きいなら機体振動 (プロペラ・シャシーのねじれ) です。")


def check_spectrum(d, plot=False):
    section("5. 振動の周波数 (FFT)")
    dt = d["dt_us"].mean() * 1e-6
    fs = 1.0 / dt
    armed = d["armed"].astype(bool)
    if armed.sum() < 256:
        print("  ARMED区間が短すぎて解析できません。")
        return

    print(f"  サンプリング {fs:.0f} Hz")
    results = {}
    for ax in ["roll", "pitch"]:
        g = d[f"{ax}_gyr"][armed]
        g = g - g.mean()
        n = len(g)
        win = np.hanning(n)
        spec = np.abs(np.fft.rfft(g * win))
        freq = np.fft.rfftfreq(n, dt)
        # DC付近を除外
        mask = freq > 2.0
        if mask.sum() == 0:
            continue
        peak = freq[mask][np.argmax(spec[mask])]
        results[ax] = (freq, spec, peak)
        print(f"  {ax:6s} gyro のピーク周波数: {peak:6.1f} Hz")

    print()
    print("  読み方:")
    print("    5〜30Hz  : 制御ループの発振。ゲイン(特にP)が高すぎる。")
    print("    30〜80Hz : 機体の構造共振。シャシーのねじれ・剛性不足。")
    print("    80Hz以上 : プロペラ/モーター由来。バランス取り・防振で対処。")

    if plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("\n  [警告] matplotlib が無いのでグラフは省略します。")
            return
        fig, axes = plt.subplots(2, 1, figsize=(10, 7))
        for ax_name, (freq, spec, peak) in results.items():
            mask = (freq > 1) & (freq < fs / 2)
            axes[0].semilogy(freq[mask], spec[mask], label=f"{ax_name} (peak {peak:.0f}Hz)")
        axes[0].set_xlabel("Frequency [Hz]")
        axes[0].set_ylabel("Magnitude")
        axes[0].set_title("Gyro spectrum")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        t = d["t_ms"] / 1000.0
        axes[1].plot(t, d["roll_cmd"], label="roll_cmd", lw=0.8)
        axes[1].plot(t, d["pitch_cmd"], label="pitch_cmd", lw=0.8)
        axes[1].plot(t, d["corr_limit"], "k--", label="corr_limit", lw=0.8)
        axes[1].plot(t, -d["corr_limit"], "k--", lw=0.8)
        axes[1].set_xlabel("time [s]")
        axes[1].set_ylabel("cmd")
        axes[1].set_title("PID output vs clamp")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", help="ログCSV (省略時は logs/ の最新)")
    ap.add_argument("--plot", action="store_true", help="グラフを表示")
    args = ap.parse_args()

    if args.path:
        path = Path(args.path)
    else:
        cands = sorted(LOGS_DIR.glob("log_*.csv"),
                       key=lambda p: p.stat().st_mtime)
        if not cands:
            sys.exit("[ERROR] logs/ にログがありません")
        path = cands[-1]

    print(f"解析対象: {path}")
    d = load(path)
    print(f"サンプル数: {len(d)}")

    check_loop_timing(d)
    check_saturation(d)
    check_cmd_magnitude(d)
    check_gyro_noise(d)
    check_spectrum(d, plot=args.plot)
    print()


if __name__ == "__main__":
    main()
