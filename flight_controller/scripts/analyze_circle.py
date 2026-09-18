# scripts/analyze_circle.py
# 機体単独の周回 / 8 の字 / 上昇旋回 (SW_HOVER cen→up。quad/CircleTrack.h, Guided.h) を
# BLE ログから円ごとに採点し、どのゲインを触るべきかを出す。
#
#   python scripts/analyze_circle.py                         # 最新の LOG*.BIN / CSV
#   python scripts/analyze_circle.py path/to/LOG0021.BIN
#   python scripts/analyze_circle.py LOG0021.csv --plot      # グラフ (matplotlib)
#   python scripts/analyze_circle.py --plot --save           # <log>_circle<n>_<円>.png に保存
#
# 入力は log_recorder の BLE ログ (ble_receiver.py / console の BleTap が保存する
# LOGnnnn.BIN) か、bin2csv.py で CSV にしたもの。BIN は bin2csv.decode_bin で読む。
#
# ------------------------------------------------------------------------
#  ログのどの列が何か (REC の列は増やしていない)
# ------------------------------------------------------------------------
#   mode == 2 (GUIDED)       周回/8 の字の区間 (GUIDED は今はこれしかしない)
#   fh_holdn / fh_holde      円の目標点 (PosHold::setTrajectory が hold に入れる)
#   fh_posn  / fh_pose       機体の推定位置 (フロー積分, 地面固定)
#   yaw_ang                  機体の推定機首 (ジャイロ積分。±180 に畳んで記録)
#   fh_vxt/vyt, fh_vxc/vyc   速度ループの目標/実測 (機体座標 前/右)
#   fh_leanr/leanp           速度ループが出したリーン角 (加速度 FF 込み)
#  円の中心と目標機首はログに無いので、区間の最初の行 (起点と機首) と目標点の
#  並びから復元する。8 の字は「目標点が起点に戻り、また離れた」所で円を分ける。
#  半径と回転方向は目標点に円をフィットして決める (--radius / --dir で上書き可)。
#
# ------------------------------------------------------------------------
#  調整の順番 (内側から)
# ------------------------------------------------------------------------
#   1. 速度ループ   vel_err_rms が大きい / 遅れる → FLOW_VEL_KP/KI (ホバーと共通)
#   2. 向心 FF      半径誤差の平均が + (外へ膨らむ) → CIRCLE_ACC_FF を上げる
#                                       - (内へ切れ込む) → 下げる
#   3. 位置 P       進行方向の遅れ (along 平均 > 0) が残る → CIRCLE_POS_KP を上げる
#                   半径誤差が 0.3Hz 前後で揺れる → CIRCLE_POS_KP を下げる
#   4. ヨー         yaw_err の平均が一方に寄る → CIRCLE_YAW_KP を上げる
#                   yaw_gyr が振動する → 下げる (またはヨーのレートループ)
#   5. 速さ         全部収まってから CIRCLE_SPEED_MPS を上げる
#  ★ ここで見る位置はフロー積分なので「機体が思っている円」。実際の軌跡は
#    カメラ (position_estimator) の録画と突き合わせること。
#
#   pip install numpy matplotlib

import argparse
import csv
import importlib.util
import math
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
MODE_GUIDED = 2
LEG_HOME_M = 0.03        # 目標点が起点からこれ以内 = 円の終わり (CircleTrack は起点で止まる)
LEG_FAR_MIN_M = 0.2      # 起点からこれ以上離れない区間は円とみなさない
MIN_ROWS = 50            # これより短い GUIDED 区間は誤操作とみなして捨てる

COLS = ("t_ms", "mode", "fh_posn", "fh_pose", "fh_holdn", "fh_holde", "yaw_ang", "yaw_gyr",
        "fh_vxt", "fh_vyt", "fh_vxc", "fh_vyc", "fh_leanr", "fh_leanp",
        "roll_ang", "pitch_ang", "range_h", "range_raw", "alt_hold", "alt_act",
        "roll_sbus", "pitch_sbus", "yaw_sbus", "thr")
SPEED_WIN_S = 0.25       # 速さは ±この時間の中心差分 (位置は 25Hz 更新、ログは 70〜125Hz)
STICK_DEAD = 0.05        # QuadConfig FLOW_STICK_DEAD
RANGE_STEP_M = 0.15      # QuadConfig RANGE_STEP_M (測距の飛びの判定)


# ------------------------------------------------------------------------
#  読み込み
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
            return "fh_holdn" in f.readline()
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


def segments(d):
    g = d["mode"] == MODE_GUIDED
    out, i, n = [], 0, len(g)
    while i < n:
        if g[i]:
            j = i
            while j < n and g[j]:
                j += 1
            if j - i >= MIN_ROWS:
                out.append((i, j))
            i = j
        else:
            i += 1
    return out


# ------------------------------------------------------------------------
#  1 区間の採点
# ------------------------------------------------------------------------
def fit_circle(x, y):
    """最小二乗の円 (Kasa)。中心と半径。"""
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    b = x * x + y * y
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    return cx, cy, math.sqrt(max(c + cx * cx + cy * cy, 0.0))


def split_legs(d, a, b):
    """GUIDED 区間 [a,b) を円ごとに分ける。目標点が起点から十分離れ → 起点に戻り →
    また離れた所が境目 (起点で待っている行は前の円に含める)。"""
    hn, he = d["fh_holdn"][a:b], d["fh_holde"][a:b]
    dist = np.hypot(hn - hn[0], he - he[0])
    legs, i, n = [], 0, len(dist)
    while i < n:
        rest = dist[i:]
        if rest.max() < LEG_FAR_MIN_M:
            if legs:
                legs[-1] = (legs[-1][0], b)
            else:
                legs.append((a + i, b))
            break
        j = i + int(np.argmax(rest >= 0.9 * rest.max()))
        back = np.flatnonzero(dist[j:] < LEG_HOME_M)
        if len(back) == 0:
            legs.append((a + i, b))
            break
        k = j + int(back[0])
        away = np.flatnonzero(dist[k:] >= LEG_HOME_M)
        if len(away) == 0:
            legs.append((a + i, b))
            break
        e = k + int(away[0])
        legs.append((a + i, a + e))
        i = e
    return legs


def leg_geometry(d, la, lb, p0, psi0_deg, radius=None, direction=None):
    """円 1 つの半径と回転方向。目標点に円をフィットし、中心が機首の右なら右旋回。"""
    fn, fe, fr = fit_circle(d["fh_holdn"][la:lb], d["fh_holde"][la:lb])
    psi = math.radians(psi0_deg)
    cross = math.cos(psi) * (fe - p0[1]) - math.sin(psi) * (fn - p0[0])
    R = radius if radius else fr
    D = direction if direction else (1 if cross >= 0 else -1)
    return R, D


def rel_angle(x):
    """起点からの角度 [rad] を連続値にする。先頭を ±π に収めてから unwrap
    (2 つ目の円は beta0 に 360° ぶん乗っているので、そのままだと 2π ずれる)。"""
    x = np.unwrap(x)
    return x - 2.0 * math.pi * round(float(x[0]) / (2.0 * math.pi))


def analyze(d, a, b, R, D, p0, psi0_deg, yaw):
    """円 1 つ [a,b) を採点する。p0 = 起点 (全部の円で共通)、psi0_deg = この円の開始時の
    目標機首 (連続値)、yaw = 機首の連続値 (この円の行ぶん)。"""
    s = {k: v[a:b] for k, v in d.items()}
    t = (s["t_ms"] - s["t_ms"][0]) / 1000.0
    pn, pe = s["fh_posn"], s["fh_pose"]
    hn, he = s["fh_holdn"], s["fh_holde"]

    psi0 = math.radians(psi0_deg)
    cn = p0[0] + R * D * -math.sin(psi0)
    ce = p0[1] + R * D * math.cos(psi0)
    beta0 = psi0 - D * math.pi / 2

    # 目標点の方位角 → 目標機首 (連続値)
    beta_ref = beta0 + rel_angle(np.arctan2(he - ce, hn - cn) - beta0)
    yaw_ref = psi0_deg + np.degrees(beta_ref - beta0)
    yaw_err = yaw_ref - yaw

    # 機体の方位角 (実測進行)
    beta_meas = beta0 + rel_angle(np.arctan2(pe - ce, pn - cn) - beta0)
    prog_meas = np.degrees(D * (beta_meas - beta0))
    prog_ref = np.degrees(D * (beta_ref - beta0))

    # 誤差: 目標点 - 機体 を 接線 (進行方向 +) / 半径 (外 +) に分解
    en, ee = hn - pn, he - pe
    tn, te = D * -np.sin(beta_ref), D * np.cos(beta_ref)
    along = en * tn + ee * te                       # + = 機体が目標点より遅れている
    rad_err = np.hypot(pn - cn, pe - ce) - R        # + = 外へ膨らんでいる
    pos_err = np.hypot(en, ee)

    vel_err = np.hypot(s["fh_vxt"] - s["fh_vxc"], s["fh_vyt"] - s["fh_vyc"])
    # 1 行ごとの差分だと、位置が 25Hz でしか更新されないので 0 と大きな値を行き来する
    i0 = np.clip(np.searchsorted(t, t - SPEED_WIN_S), 0, len(t) - 1)
    i1 = np.clip(np.searchsorted(t, t + SPEED_WIN_S), 0, len(t) - 1)
    span = t[i1] - t[i0]
    speed = np.where(span > 1e-3,
                     np.hypot(pn[i1] - pn[i0], pe[i1] - pe[i0]) / np.maximum(span, 1e-3), 0.0)

    # 巡航区間 (前後 15% を除く) の統計。立ち上がり/減速で数字が汚れないように
    lo, hi = int(len(t) * 0.15), int(len(t) * 0.85)
    mid = slice(lo, max(hi, lo + 1))
    fc = fit_circle(pn, pe)

    return dict(
        t=t, pn=pn, pe=pe, hn=hn, he=he, cn=cn, ce=ce, R=R, D=D, yaw=yaw, yaw_ref=yaw_ref,
        yaw_err=yaw_err, along=along, rad_err=rad_err, pos_err=pos_err, vel_err=vel_err,
        prog_meas=prog_meas, prog_ref=prog_ref, speed=speed, s=s, mid=mid, fit=fc,
        dur=t[-1], closure=math.hypot(pn[-1] - p0[0], pe[-1] - p0[1]),
    )


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def hover_baseline(d, a):
    """周回の直前の POSHOLD ホバー (高度ホールド中) の速度誤差 RMS。比較の物差し。"""
    t = d["t_ms"] / 1000.0
    m = (d["mode"] == 3) & (d["alt_act"] > 0.5) & (d["range_h"] > 0.2)
    m[a:] = False
    m &= t > t[a] - 20.0                 # 直前 20 秒まで
    if np.sum(m) < 50:
        return None
    return rms(np.hypot(d["fh_vxt"] - d["fh_vxc"], d["fh_vyt"] - d["fh_vyc"])[m])


def end_reason(d, a, b, r):
    """GUIDED を抜けた理由をログから推定する (シリアルの解除理由はログに残らない)。"""
    t = d["t_ms"] / 1000.0
    if r["prog_meas"][-1] >= 350.0:
        return "1 周完了", None
    reasons = []
    # 抜ける直前 1 秒の測距の飛び
    w = np.flatnonzero((t >= t[b - 1] - 1.0) & (t <= t[min(b, len(t) - 1)]))
    raw = d["range_raw"][w]
    chg = np.flatnonzero(np.diff(raw) != 0) + 1
    if len(chg) > 1:
        jumps = np.abs(np.diff(raw[chg]))
        k = np.argmax(jumps)
        if jumps[k] > RANGE_STEP_M:
            tj = t[w][chg][k + 1] - t[a]
            reasons.append(f"測距の飛び {raw[chg][k]:.2f}→{raw[chg][k + 1]:.2f} m "
                           f"(周回 {tj:.1f} s)")
    # 抜けた瞬間までのスティック (抜けた後の操作は原因ではないので見ない)
    k = np.arange(max(b - 5, a), min(b + 1, len(t)))
    for col, name in (("roll_sbus", "ロール"), ("pitch_sbus", "ピッチ"), ("yaw_sbus", "ヨー")):
        if np.max(np.abs(d[col][k])) > STICK_DEAD:
            reasons.append(f"{name}スティック操作")
    after = int(d["mode"][b]) if b < len(t) else None
    names = {1: "ANGLE", 3: "POSHOLD", 4: "ALTHOLD"}
    if after is not None:
        reasons.append(f"次のモード {names.get(after, after)}")
    if r["dur"] > 0 and not reasons:
        reasons.append("不明 (シリアルの解除理由を確認)")
    return "途中で解除", reasons


def report(i, j, n, r, base_vel, why):
    m = r["mid"]
    s = r["s"]
    alt_err = s["range_h"] - s["alt_hold"]
    done = why[0] == "1 周完了"
    close = (f"起点へのずれ {r['closure']:.2f} m" if done
             else "起点へのずれ -- (未完了)")
    turn = "右" if r["D"] > 0 else "左"
    print(f"\n=== #{i} 円 {j}/{n} ({turn}旋回)  {r['dur']:.1f} s  実測進行 "
          f"{r['prog_meas'][-1]:.0f} deg (目標 {r['prog_ref'][-1]:.0f})  {close} ===")
    print(f"  終わり方: {why[0]}" + ("" if not why[1] else "  ← " + " / ".join(why[1])))
    print(f"  中心 (N,E) = ({r['cn']:+.2f}, {r['ce']:+.2f})   フィット円: 中心 "
          f"({r['fit'][0]:+.2f}, {r['fit'][1]:+.2f}) 半径 {r['fit'][2]:.2f} m (目標 {r['R']:.2f})")
    print("  --- 巡航区間 (前後 15% 除く) ---")
    print(f"  半径誤差   平均 {np.mean(r['rad_err'][m]):+.3f}  RMS {rms(r['rad_err'][m]):.3f}  "
          f"最大 {np.max(np.abs(r['rad_err'][m])):.3f} m   (+ = 外へ膨らむ)")
    print(f"  進行遅れ   平均 {np.mean(r['along'][m]):+.3f}  RMS {rms(r['along'][m]):.3f} m   "
          f"(+ = 目標点より遅れ)")
    print(f"  位置誤差   RMS {rms(r['pos_err'][m]):.3f}  最大 {np.max(r['pos_err']):.3f} m")
    print(f"  速さ       平均 {np.mean(r['speed'][m]):.2f} m/s")
    base_txt = f" (直前のホバー {base_vel:.3f})" if base_vel is not None else ""
    print(f"  速度ループ 誤差 RMS {rms(r['vel_err'][m]):.3f} m/s{base_txt}")
    print(f"  ヨー誤差   平均 {np.mean(r['yaw_err'][m]):+.1f}  RMS {rms(r['yaw_err'][m]):.1f} deg  "
          f"ヨーレート平均 {np.mean(s['yaw_gyr'][m]):+.1f} deg/s")
    print(f"  リーン     最大 roll {np.max(np.abs(s['fh_leanr'])):.1f} / "
          f"pitch {np.max(np.abs(s['fh_leanp'])):.1f} deg")
    print(f"  高度       開始 {s['range_h'][0]:.2f} → 終了 {s['range_h'][-1]:.2f} m "
          f"(目標 {s['alt_hold'][0]:.2f} → {s['alt_hold'][-1]:.2f})  誤差 RMS {rms(alt_err[m]):.3f} m")

    # 半径誤差の揺れの周波数。0.25〜0.35Hz はホバーでも出る速度ループのリミットサイクル
    rad_d = r["rad_err"][m] - np.mean(r["rad_err"][m])
    tm = r["t"][m]
    f_peak = None
    if len(rad_d) > 64 and tm[-1] - tm[0] > 4.0:
        tu = np.linspace(tm[0], tm[-1], len(tm))
        ru = np.interp(tu, tm, rad_d)
        F = np.fft.rfftfreq(len(ru), tu[1] - tu[0])
        P = np.abs(np.fft.rfft(ru))
        f_peak = float(F[1 + np.argmax(P[1:])])
        f_res = float(F[1])               # 周波数分解能 = 1 / 解析区間の長さ
        print(f"  半径の揺れ 振幅 RMS {rms(rad_d):.3f} m  ピーク {f_peak:.2f} Hz "
              f"(分解能 ±{f_res:.2f})")

    hints = []
    if np.mean(r["rad_err"][m]) > 0.10:
        hints.append("外へ膨らむ → CIRCLE_ACC_FF を上げる (次に CIRCLE_POS_KP)")
    if np.mean(r["rad_err"][m]) < -0.10:
        hints.append("内へ切れ込む → CIRCLE_ACC_FF を下げる")
    if np.mean(r["along"][m]) > 0.15:
        hints.append("目標点より遅れる → 速度ループ (FLOW_VEL_KP/KI) か CIRCLE_POS_KP を上げる")
    v_lim = max(0.10, 1.2 * base_vel) if base_vel is not None else 0.25
    if rms(r["vel_err"][m]) > v_lim:
        hints.append("速度ループがホバー時より追えていない → FLOW_VEL_KP/KI から "
                     "(CIRCLE_SPEED_MPS を下げて切り分け)")
    if abs(np.mean(r["yaw_err"][m])) > 8.0:
        hints.append("機首が接線からずれたまま → CIRCLE_YAW_KP を上げる")
    if not done:
        hints.append("1 周に届かず終了 → 上の「終わり方」を確認。周回のゲインとは切り分けること")
    if f_peak is not None and rms(rad_d) > 0.06:
        # 区間が短いと分解能が粗いので、リミットサイクル帯を分解能ぶん広げて判定する
        if 0.25 - f_res <= f_peak <= 0.35 + f_res:
            hints.append(f"半径が {f_peak:.2f}Hz で揺れる = ホバーと同じ速度ループの揺れ → "
                         "CIRCLE_POS_KP ではなく FLOW_VEL_KP/KD")
        else:
            hints.append(f"半径が {f_peak:.2f}Hz で揺れる → CIRCLE_POS_KP を下げる")
    print("  --- 次に触るもの ---")
    for h in hints or ["目立つ問題なし。CIRCLE_SPEED_MPS を上げて再試験"]:
        print(f"   * {h}")


def plot(i, j, r, path, save, whole=None):
    import matplotlib
    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for fam in ("Yu Gothic", "Meiryo", "MS Gothic", "Noto Sans CJK JP"):
        if fam in have:
            matplotlib.rcParams["font.family"] = fam
            break

    s, t = r["s"], r["t"]
    fig = plt.figure(figsize=(14, 9))
    fig.suptitle(f"{path.name}  #{i} 円 {j} ({'右' if r['D'] > 0 else '左'}旋回)")

    ax = fig.add_subplot(2, 3, (1, 4))
    if whole is not None:   # 8 の字: 区間全体を薄く
        ax.plot(whole[1], whole[0], color="0.8", lw=1, label="区間全体 (機体)")
    ax.plot(r["he"], r["hn"], "--", label="目標点")
    ax.plot(r["pe"], r["pn"], label="機体 (フロー積分)")
    ax.plot([r["ce"]], [r["cn"]], "k+")
    ax.plot([r["pe"][0]], [r["pn"][0]], "go", label="起点")
    ax.set_xlabel("E [m]")
    ax.set_ylabel("N [m]")
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()

    ax = fig.add_subplot(2, 3, 2)
    ax.plot(t, r["rad_err"], label="半径誤差 (+外)")
    ax.plot(t, r["along"], label="進行遅れ (+遅れ)")
    ax.set_ylabel("m")
    ax.grid(True)
    ax.legend()

    ax = fig.add_subplot(2, 3, 3)
    ax.plot(t, r["yaw_ref"], "--", label="目標機首")
    ax.plot(t, r["yaw"], label="機首")
    ax2 = ax.twinx()
    ax2.plot(t, r["yaw_err"], "r", alpha=0.4, label="誤差")
    ax.set_ylabel("deg")
    ax.grid(True)
    ax.legend(loc="upper left")

    ax = fig.add_subplot(2, 3, 5)
    ax.plot(t, s["fh_vxt"], "--", label="vx 目標")
    ax.plot(t, s["fh_vxc"], label="vx 実測")
    ax.plot(t, s["fh_vyt"], "--", label="vy 目標")
    ax.plot(t, s["fh_vyc"], label="vy 実測")
    ax.set_ylabel("m/s (機体座標)")
    ax.set_xlabel("t [s]")
    ax.grid(True)
    ax.legend()

    ax = fig.add_subplot(2, 3, 6)
    ax.plot(t, s["fh_leanr"], "--", label="lean roll 指令")
    ax.plot(t, s["roll_ang"], label="roll")
    ax.plot(t, s["fh_leanp"], "--", label="lean pitch 指令")
    ax.plot(t, s["pitch_ang"], label="pitch")
    ax.set_ylabel("deg")
    ax.set_xlabel("t [s]")
    ax.grid(True)
    ax.legend()

    fig.tight_layout()
    if save:
        out = path.with_name(f"{path.stem}_circle{i}_{j}.png")
        fig.savefig(out, dpi=110)
        print(f"  [保存] {out}")
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="機体単独周回の BLE ログ解析")
    ap.add_argument("path", nargs="?", help="LOGnnnn.BIN または CSV (省略時は最新)")
    ap.add_argument("--radius", type=float, default=None, help="半径を固定 (既定: 目標点から推定)")
    ap.add_argument("--dir", type=int, default=None, choices=(-1, 1),
                    help="1 つ目の円の向きを固定 (+1 右 / -1 左。既定: 推定。2 つ目以降は交互)")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--save", action="store_true", help="--plot を PNG に保存")
    args = ap.parse_args()

    path = pick_log(args.path)
    print(f"[INFO] {path}")
    d = load(path)
    segs = segments(d)
    if not segs:
        sys.exit("[ERROR] GUIDED (mode=2) の区間がありません")
    for i, (a, b) in enumerate(segs, 1):
        # ログの yaw_ang は ±180 に畳んである (S5LogFill.h)。区間全体で連続値に戻す
        yaw = np.degrees(np.unwrap(np.radians(d["yaw_ang"][a:b])))
        # 起点: GUIDED の最初の行はまだ POSHOLD の保持点が残っていることがあるので、
        #  保持点が最初に書き換わった行 (= 軌道追従の 1 回目、目標は起点のまま) を使う
        hn, he = d["fh_holdn"][a:b], d["fh_holde"][a:b]
        moved = np.flatnonzero((hn != hn[0]) | (he != he[0]))
        k = a + (int(moved[0]) if len(moved) else 0)
        p0 = (d["fh_holdn"][k], d["fh_holde"][k])
        legs = split_legs(d, a, b)
        base = hover_baseline(d, a)
        psi = yaw[0]                      # 1 つ目の円の目標機首 = 開始時の機首
        if len(legs) > 1:
            print(f"\n##### #{i}: 円 {len(legs)} 個 (8 の字)  "
                  f"{(d['t_ms'][b - 1] - d['t_ms'][a]) / 1000:.1f} s #####")
        for j, (la, lb) in enumerate(legs, 1):
            fixed_dir = None if args.dir is None else args.dir * (1 if j % 2 else -1)
            R, D = leg_geometry(d, la, lb, p0, psi, args.radius, fixed_dir)
            r = analyze(d, la, lb, R, D, p0, psi, yaw[la - a:lb - a])
            why = end_reason(d, la, b, r) if j == len(legs) else ("1 周完了", None)
            report(i, j, len(legs), r, base, why)
            if args.plot or args.save:
                whole = (d["fh_posn"][a:b], d["fh_pose"][a:b]) if len(legs) > 1 else None
                plot(i, j, r, path, args.save, whole)
            psi += 360.0 * D              # 次の円は前の円の終わりの目標機首から
    if args.plot and not args.save:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
