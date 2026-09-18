# scripts/analyze_straight.py
# 着陸点ボーナスの直進性試験: 「カメラの外で離陸 → 22m まっすぐ → 幅 6m (±3m) に入る」
# を機体のセンサだけで満たせるかを、誤差の出どころごとに分けて測る。
#
#   python scripts/analyze_straight.py flight [LOG.BIN|CSV] [--plot]   直進飛行の採点 (フロー上)
#   python scripts/analyze_straight.py drift  [LOG.BIN|CSV]            静置ログからジャイロのドリフト
#   python scripts/analyze_straight.py budget measurements.csv         実測 (巻尺) から ±3m に入る確率
#   python scripts/analyze_straight.py model --bias 0.05               ジャイロバイアスだけの横ずれ予測
#
# ------------------------------------------------------------------------
#  何が横ずれを作るか (22m 先の許容 ±3m = 方位 ±7.8 deg)
# ------------------------------------------------------------------------
#   A. 置いた向き (人)         22m × tan(θ)。1 deg = 0.38 m。             → 試験 1
#   B. ジャイロのバイアス      方位が b[deg/s] で回り続ける。横ずれは飛行時間の 2 乗。
#                              アーム→出発までの時間 T0 の分は 22m × tan(b·T0)。  → 試験 2
#   C. フローの取り付けヨー角  機体に固定の一定角。往路/復路で同じ「機体の右/左」に出る。 → 試験 3
#   D. フローの横ノイズ・風    フロー上では 0 に戻すので、ログの cross が小さければ小さい。
#  A〜C はログの cross (フロー座標) には出ない。実際の位置は巻尺で測るしかない。
#
# ------------------------------------------------------------------------
#  試験 1: 置き方 (飛ばさない。1 人 10 回 × 何人か)
# ------------------------------------------------------------------------
#   1. 床に基準線 (本番の「まっすぐ」方向) をテープで 22m 引き、22m 地点に ±3m の印。
#      22m 取れなければ 10m でよい (角度に直して 22m に換算する)。
#   2. 機体の前後軸にレーザーポインタを仮止めし、治具 (基準線に当てた定規) に機体を
#      当てた状態でレーザーが基準線の先端に当たるよう合わせる (= 取り付け誤差 0 の校正)。
#   3. 本番と同じ手順・同じ手がかりで置く (機体を持って離れ、戻ってきて置く、を毎回)。
#      置く人はレーザーを見ない。見る人が先端での横ずれ [m] を読む (右 +)。
#   4. measurements.csv に kind=place で書く。本番で使う「目印」(テープの矢印、
#      照準棒など) を変えるなら note に書いて条件ごとに比べる。
#
# ------------------------------------------------------------------------
#  試験 2: ジャイロのドリフト (飛ばさない、数分)
# ------------------------------------------------------------------------
#   電源を入れて 5 秒の自動キャリブが終わったら、機体に触らず 2〜3 分置いてログを取る
#   (BLE REC)。`drift` が yaw_gyr の平均から b [deg/s] を出し、B の横ずれを予測する。
#   (プロペラを外してアームしたログなら yaw_ang の傾き = 実際に積分された方位を使う)
#   冷えた状態と、飛行直後 (モーター・基板が温まった状態) の両方で取ること。
#   ★ 飛行中の振動で変わる分はこれでは測れない (試験 3 の結果に含まれる)。
#
# ------------------------------------------------------------------------
#  試験 3: 直進飛行 (QuadConfig.h GUIDED_PATTERN=Straight)
# ------------------------------------------------------------------------
#   1. 試験 1 の治具で置く (レーザーで置き方の誤差を読んで place_offset_m に書く)。
#   2. 本番と同じ手順で離陸 → POSHOLD → SW_HOVER up。機体はアーム時の機首へ向き直り、
#      STRAIGHT_DIST_M 進んで止まる (POSHOLD)。★ STRAIGHT_DIST_M を場所に合わせること。
#   3. 止まってホバーしている機体の真下を、基準線から巻尺で測る (右 +)。下げ振り
#      (糸の先に重り) を持った人が真下に立つと速い。着陸させると降下中に流れる。
#   4. kind=flight, dist_m, offset_m, place_offset_m, dir (往/復) を書く。
#   5. 同じ線を逆向き (往路と復路) にも飛ぶ。風や床は向きで符号が変わり、
#      C (取り付け角) とジャイロの向きは「機体の右」で同じ符号に出る。最低 往3 復3。
#   6. `flight LOG` でフロー上の cross が小さいこと (= 制御は真っ直ぐ飛んだつもり) を
#      確かめる。フロー上で曲がっているならゲイン (CIRCLE_POS_KP) の問題で、
#      巻尺の値とは別に直す。
#
# ------------------------------------------------------------------------
#  measurements.csv (試験 1 と 3 の実測。右 + [m])
# ------------------------------------------------------------------------
#   kind,dist_m,offset_m,place_offset_m,who,dir,note
#   place,22,+0.40,,A,,テープ矢印のみ
#   place,22,-0.85,,B,,テープ矢印のみ
#   flight,10,+0.62,+0.05,,往,0.5m/s
#   flight,10,+0.48,-0.10,,復,0.5m/s
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
TARGET_M = 22.0          # 本番の直進距離
HALF_WIDTH_M = 3.0       # フィールド幅 6m の半分
MODE_GUIDED = 2
MOVING_M = 0.05          # 目標点が起点からこれ以上離れた行 = 直進中 (手前は向き合わせ)


def _circle_mod():
    """読み込み (BIN/CSV) は analyze_circle.py と共通。"""
    spec = importlib.util.spec_from_file_location("analyze_circle", HERE / "analyze_circle.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def t_seconds(d):
    return (d["t_ms"] - d["t_ms"][0]) * 1e-3


# ------------------------------------------------------------------------
#  誤差モデル: ジャイロのバイアスだけで出る横ずれ
# ------------------------------------------------------------------------
def gyro_lateral(bias_dps, dist=TARGET_M, speed=0.5, accel=0.2, t0=20.0, dt=0.01):
    """バイアス b で方位が回り続けるとき、dist 進んだ時点の横ずれ [m]。
    t0 = アームから出発までの秒数 (その分の方位ずれを持って出発する)。
    機体はフロー座標では真っ直ぐ進むので、実際の進行方向 = b·(t0 + t)。"""
    s = v = y = t = 0.0
    while s < dist:
        v = min(speed, v + accel * dt)
        v = max(min(v, math.sqrt(2.0 * accel * (dist - s))), 0.05)   # 終点で減速
        psi = math.radians(bias_dps * (t0 + t))
        s += v * math.cos(psi) * dt
        y += v * math.sin(psi) * dt
        t += dt
    return y, t


# ------------------------------------------------------------------------
#  flight: 直進区間の採点 (フロー座標 = 機体が思っている軌跡)
# ------------------------------------------------------------------------
def straight_segments(d, segs):
    out = []
    for a, b in segs:
        hn, he = d["fh_holdn"][a:b], d["fh_holde"][a:b]
        dist = np.hypot(hn - hn[0], he - he[0])
        mv = np.flatnonzero(dist > MOVING_M)
        if len(mv) < 10:
            continue
        out.append((a, a + int(mv[0]), b))   # (GUIDED 開始, 直進開始, 終了)
    return out


def analyze_flight(d, a, s, b):
    t = t_seconds(d)
    hn, he = d["fh_holdn"], d["fh_holde"]
    pn, pe = d["fh_posn"], d["fh_pose"]
    # 方向: 目標点の起点 → 最遠点 (目標点は直線上しか動かない)
    p0 = np.array([hn[a], he[a]])
    far = s + int(np.argmax(np.hypot(hn[s:b] - p0[0], he[s:b] - p0[1])))
    u = np.array([hn[far] - p0[0], he[far] - p0[1]])
    L = float(np.linalg.norm(u))
    u /= L
    line_deg = math.degrees(math.atan2(u[1], u[0]))

    rn, re_ = pn[s:b] - p0[0], pe[s:b] - p0[1]
    along = rn * u[0] + re_ * u[1]
    cross = -rn * u[1] + re_ * u[0]
    yaw = np.degrees(np.unwrap(np.radians(d["yaw_ang"][s:b])))
    yaw_err = yaw - line_deg
    yaw_err -= 360.0 * np.round(np.median(yaw_err) / 360.0)
    # 向き合わせの成績 (直進開始時点)
    align_s = t[s] - t[a]
    tt = t[s:b] - t[s]
    spd = np.gradient(along, tt, edge_order=1) if len(tt) > 2 else np.zeros_like(along)
    cruise = spd > 0.8 * np.percentile(spd, 90)
    return dict(
        L=L, line_deg=line_deg, align_s=align_s, dur=float(tt[-1]),
        along_end=float(along[-1]), cross_end=float(cross[-1]),
        cross_max=float(cross[np.argmax(np.abs(cross))]), cross_rms=float(np.sqrt(np.mean(cross ** 2))),
        yaw_mean=float(np.mean(yaw_err)), yaw_absmax=float(np.max(np.abs(yaw_err))),
        speed_cruise=float(np.median(spd[cruise])) if cruise.any() else float("nan"),
        tt=tt, along=along, cross=cross, yaw_err=yaw_err,
    )


def cmd_flight(args):
    ac = _circle_mod()
    path = ac.pick_log(args.path)
    d = ac.load(path)
    segs = straight_segments(d, ac.segments(d))
    print(f"ログ: {path}")
    if not segs:
        sys.exit("[ERROR] 直進した GUIDED 区間がありません (GUIDED_PATTERN=Straight で飛んだか)")
    for k, (a, s, b) in enumerate(segs, 1):
        r = analyze_flight(d, a, s, b)
        print(f"\n=== 直進 {k}: 目標 {r['L']:.2f} m, 方位 {r['line_deg']:+.1f} deg (est 系) ===")
        print(f"  向き合わせ   {r['align_s']:.1f} s")
        print(f"  直進         {r['dur']:.1f} s, 巡航 {r['speed_cruise']:.2f} m/s")
        print(f"  フロー上     前 {r['along_end']:.2f} m / 横 終点 {r['cross_end']:+.2f} m "
              f"最大 {r['cross_max']:+.2f} m RMS {r['cross_rms']:.2f} m")
        print(f"  機首ずれ     平均 {r['yaw_mean']:+.2f} deg, 最大 {r['yaw_absmax']:.2f} deg")
        ok = abs(r["cross_max"]) < 0.3 and r["yaw_absmax"] < 5.0
        print("  判定         " + ("制御は真っ直ぐ飛んだつもり。実際のずれは巻尺の値で見る"
                                   if ok else
                                   "★ フロー上で既に曲がっている/機首が振れている。"
                                   "CIRCLE_POS_KP・CIRCLE_YAW_KP・速度ループから先に直す"))
        if args.plot:
            plot_flight(r, path, k, args.save)


def plot_flight(r, path, k, save):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    ax[0].plot(r["tt"], r["along"]); ax[0].set_ylabel("along [m]")
    ax[1].plot(r["tt"], r["cross"]); ax[1].set_ylabel("cross (flow) [m]")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[2].plot(r["tt"], r["yaw_err"]); ax[2].set_ylabel("yaw err [deg]"); ax[2].set_xlabel("t [s]")
    fig.suptitle(f"{path.name} straight {k}")
    fig.tight_layout()
    if save:
        out = path.with_name(f"{path.stem}_straight{k}.png")
        fig.savefig(out, dpi=120)
        print(f"  -> {out}")
    else:
        plt.show()


# ------------------------------------------------------------------------
#  drift: 静置ログの方位の傾き
# ------------------------------------------------------------------------
def cmd_drift(args):
    ac = _circle_mod()
    path = ac.pick_log(args.path)
    d = ac.load(path)
    t = t_seconds(d)
    # 静置 = スロットル最低 かつ ジャイロが小さい。最長の連続区間を使う
    still = ((d["thr"] < 0.05) & (np.abs(d["yaw_gyr"]) < 2.0)).astype(int)
    edges = np.flatnonzero(np.diff(np.r_[0, still, 0]))
    runs = list(zip(edges[::2], edges[1::2]))          # [a, b) の連続区間
    if not runs:
        sys.exit("[ERROR] 静置区間がありません")
    a, b = max(runs, key=lambda r: t[r[1] - 1] - t[r[0]])
    if b - a < 50 or t[b - 1] - t[a] < 30.0:
        sys.exit("[ERROR] 30 秒以上の静置区間がありません")
    tt = t[a:b]
    gyr = d["yaw_gyr"][a:b]
    bias = float(np.mean(gyr))
    print(f"ログ: {path}")
    print(f"静置区間 {tt[-1] - tt[0]:.0f} s")
    # ディスアーム中は HeadingHold::integrate() まで届かないので yaw_ang は動かない。
    # 積分に使われる値そのもの (yaw_gyr) の平均をバイアスとする。yaw_gyr は 0.1 deg/s 刻み
    # なので、ノイズが刻みより小さいと平均が丸められる (σ を見る)。
    half = len(gyr) // 2
    print(f"yaw_gyr 平均 b = {bias:+.4f} deg/s ({bias * 60:+.2f} deg/min), σ {np.std(gyr):.3f} deg/s, "
          f"前半 {np.mean(gyr[:half]):+.4f} / 後半 {np.mean(gyr[half:]):+.4f}")
    if np.std(gyr) < 0.05:
        print("  ★ ノイズが記録の刻み (0.1 deg/s) より小さく、平均が丸められている可能性")
    yaw = np.degrees(np.unwrap(np.radians(d["yaw_ang"][a:b])))
    if np.ptp(yaw) > 0.2:   # アーム中 (プロペラを外して) なら方位も動く
        slope = float(np.polyfit(tt, yaw, 1)[0])
        print(f"yaw_ang の傾き  = {slope:+.4f} deg/s (アーム中のログ。こちらが実際の積分)")
        bias = slope
    print_model(bias, args.speed, args.t0)


def print_model(bias, speed, t0):
    print(f"\n予測 (22m, {speed:.2f} m/s, アーム→出発 {t0:.0f} s, このバイアスだけの場合):")
    y, t = gyro_lateral(bias, speed=speed, t0=t0)
    print(f"  横ずれ {y:+.2f} m  (直進 {t:.0f} s)  → 許容 ±{HALF_WIDTH_M:.0f} m の "
          f"{abs(y) / HALF_WIDTH_M * 100:.0f}%")
    for sp in (0.4, 0.5, 0.8):
        for tz in (10.0, 30.0):
            yy, _ = gyro_lateral(bias, speed=sp, t0=tz)
            print(f"    {sp:.1f} m/s, T0 {tz:3.0f} s: {yy:+.2f} m")


def cmd_model(args):
    print_model(args.bias, args.speed, args.t0)


# ------------------------------------------------------------------------
#  budget: 巻尺の実測から 22m 先で ±3m に入る確率
# ------------------------------------------------------------------------
def _norm_cdf(x):
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


def _stats(xs):
    xs = np.asarray(xs, float)
    sd = float(np.std(xs, ddof=1)) if len(xs) > 1 else float("nan")
    return float(np.mean(xs)), sd


def cmd_budget(args):
    place, flight = [], []
    with open(args.csv, encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            try:
                dist = float(r["dist_m"])
                off = float(r["offset_m"])
            except (KeyError, ValueError):
                continue
            k = TARGET_M / dist            # 角度の誤差として 22m に換算
            kind = (r.get("kind") or "").strip()
            if kind == "place":
                place.append((off * k, r.get("who", ""), r.get("note", "")))
            elif kind == "flight":
                po = r.get("place_offset_m") or ""
                po = float(po) if po.strip() else 0.0
                flight.append(((off - po) * k, (r.get("dir") or "").strip(), off * k))

    print(f"22m 換算 (右 +)。許容 ±{HALF_WIDTH_M:.1f} m\n")
    mp = sp = mf = sf = None
    if place:
        mp, sp = _stats([p[0] for p in place])
        print(f"[置き方] {len(place)} 回: 平均 {mp:+.2f} m, σ {sp:.2f} m "
              f"(角度 平均 {math.degrees(math.atan(mp / TARGET_M)):+.2f}, "
              f"σ {math.degrees(math.atan(sp / TARGET_M)):.2f} deg), 最大 |{max(abs(p[0]) for p in place):.2f}| m")
        for who in sorted({p[1] for p in place}):
            xs = [p[0] for p in place if p[1] == who]
            if who and len(xs) > 1:
                m, s = _stats(xs)
                print(f"    {who}: {len(xs)} 回 平均 {m:+.2f} σ {s:.2f}")
    if flight:
        mf, sf = _stats([x[0] for x in flight])
        print(f"[飛行 (置き方を引いた分)] {len(flight)} 回: 平均 {mf:+.2f} m, σ {sf:.2f} m")
        dirs = sorted({x[1] for x in flight if x[1]})
        if len(dirs) == 2:
            m1, _ = _stats([x[0] for x in flight if x[1] == dirs[0]])
            m2, _ = _stats([x[0] for x in flight if x[1] == dirs[1]])
            print(f"    {dirs[0]} 平均 {m1:+.2f} / {dirs[1]} 平均 {m2:+.2f}")
            print(f"    → 機体に固定の偏り (取付角・ジャイロ) ≈ {(m1 + m2) / 2:+.2f} m "
                  f"(トリムで消せる), 向きで反転する分 (風・床) ≈ {(m1 - m2) / 2:+.2f} m")
        print("    ★ 22m 未満で飛んだ場合、ジャイロのドリフト分は時間の 2 乗で増えるので"
              "この換算は楽観側。22m で 1 回は確かめること")

    if place and flight and len(place) > 1 and len(flight) > 1:
        m = mp + mf
        s = math.hypot(sp, sf)
        p_in = _norm_cdf((HALF_WIDTH_M - m) / s) - _norm_cdf((-HALF_WIDTH_M - m) / s)
        print(f"\n[合計] 平均 {m:+.2f} m, σ {s:.2f} m → ±{HALF_WIDTH_M:.0f} m に入る確率 ≈ {p_in * 100:.0f}% "
              f"(正規分布を仮定)")
        mb = mp + mf
        p_nobias = _norm_cdf(HALF_WIDTH_M / s) - _norm_cdf(-HALF_WIDTH_M / s)
        if abs(mb) > 0.3:
            print(f"    平均の偏り {mb:+.2f} m を消せれば (置く向きを逆に振る/フロー取付角を補正) "
                  f"≈ {p_nobias * 100:.0f}%")
        share_p = sp ** 2 / (sp ** 2 + sf ** 2)
        print(f"    ばらつきの内訳: 置き方 {share_p * 100:.0f}% / 飛行 {(1 - share_p) * 100:.0f}%")


def main():
    ap = argparse.ArgumentParser(description="着陸点 22m 直進性試験の解析")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("flight", help="直進飛行ログの採点 (フロー上)")
    p.add_argument("path", nargs="?")
    p.add_argument("--plot", action="store_true")
    p.add_argument("--save", action="store_true")
    p.set_defaults(func=cmd_flight)

    p = sub.add_parser("drift", help="静置ログからジャイロのドリフト")
    p.add_argument("path", nargs="?")
    p.add_argument("--speed", type=float, default=0.5, help="予測に使う直進速度 [m/s]")
    p.add_argument("--t0", type=float, default=20.0, help="予測に使う アーム→出発 [s]")
    p.set_defaults(func=cmd_drift)

    p = sub.add_parser("model", help="バイアス b [deg/s] だけの横ずれ予測")
    p.add_argument("--bias", type=float, required=True)
    p.add_argument("--speed", type=float, default=0.5)
    p.add_argument("--t0", type=float, default=20.0)
    p.set_defaults(func=cmd_model)

    p = sub.add_parser("budget", help="巻尺の実測から ±3m に入る確率")
    p.add_argument("csv")
    p.set_defaults(func=cmd_budget)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
