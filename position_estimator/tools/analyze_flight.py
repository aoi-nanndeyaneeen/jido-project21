"""
tools/analyze_flight.py
1回の飛行のログ3本をまとめて読み、「次に何を直すべきか」だけを出す。

    python tools/analyze_flight.py            # 最新の飛行を自動で選ぶ
    python tools/analyze_flight.py 20260914_121610

読むもの (すべて src/logs/):
    flight_<ts>.csv    カメラの検知と幾何
    mission_<ts>.csv   PCの判断と機体の言い分の突き合わせ
    s5_link_*.csv      機体テレメトリの生ログ (時刻が近いものを自動で対応付け)

★ 3本とも先頭列が Epoch_s なので、時刻で素直に並べられる。
  機体の millis() しか無かった頃はこれができず、解析が推測になっていた。
"""

import csv
import sys
from pathlib import Path

LOG_DIR = Path(__file__).resolve().parent.parent / "src" / "logs"


def _rows(path):
    if not path.exists():
        return []
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _f(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def _stat(vals):
    """(個数, 中央値, 最大) を返す。空なら (0, None, None)。"""
    v = sorted(x for x in vals if x is not None)
    if not v:
        return 0, None, None
    return len(v), v[len(v) // 2], v[-1]


def find_session(stamp=None):
    flights = sorted(LOG_DIR.glob("flight_*.csv"))
    flights = [p for p in flights if "_perf" not in p.name]
    if not flights:
        return None
    if stamp:
        flights = [p for p in flights if stamp in p.name]
        if not flights:
            return None
    return flights[-1]


def report_camera(rows):
    print("\n== カメラ検知 ==")
    if not rows:
        print("  flight ログがありません")
        return
    n = len(rows)
    det = sum(1 for r in rows if r["Detected"] == "1")
    dummy = sum(1 for r in rows if r.get("In_Dummy") == "1")
    pair_rej = sum(1 for r in rows if r.get("Pair_Rejected") == "1")
    t0, t1 = _f(rows[0], "Epoch_s"), _f(rows[-1], "Epoch_s")
    dur = (t1 - t0) if (t0 and t1) else 0.0

    print(f"  {dur:.0f}秒 / {n}フレーム ({n / max(dur, 1e-9):.0f}fps)")
    print(f"  検知率 {det / n * 100:5.1f}%   ダミー {dummy / n * 100:4.1f}%   "
          f"ペア棄却 {pair_rej / n * 100:4.1f}%")

    _, res_med, res_max = _stat([_f(r, "Residual(m)") for r in rows])
    if res_med is not None:
        print(f"  残差 中央{res_med:.3f}m 最大{res_max:.3f}m")

    # 候補が多い = 窓や反射を拾っている。1個に絞れているのが理想。
    c1 = [_f(r, "N_Cand1") for r in rows]
    c2 = [_f(r, "N_Cand2") for r in rows]
    _, m1, x1 = _stat(c1)
    _, m2, x2 = _stat(c2)
    if m1 is not None:
        print(f"  候補数 Cam1 中央{m1:.0f}/最大{x1:.0f}   Cam2 中央{m2:.0f}/最大{x2:.0f}"
              "   ← 多いほど誤検知を掴むリスク")

    # 見失いの連続区間
    gaps, cur = [], 0
    for r in rows:
        if r["Detected"] != "1":
            cur += 1
        elif cur:
            gaps.append(cur)
            cur = 0
    if cur:
        gaps.append(cur)
    if gaps:
        fps = n / max(dur, 1e-9)
        print(f"  見失い {len(gaps)}回  最長 {max(gaps)}フレーム "
              f"({max(gaps) / max(fps, 1e-9):.2f}秒)")


def report_mission(rows):
    print("\n== ミッション ==")
    if not rows:
        print("  mission ログがありません (= [M] を押していないか、"
              "地上局に繋がっていません)")
        return

    # フェーズの滞在時間と遷移
    print("  フェーズの推移:")
    prev, t_in = None, None
    for r in rows:
        ph, t = r["Phase"], _f(r, "Epoch_s")
        if ph != prev:
            if prev is not None:
                print(f"    {prev:8s} {t - t_in:5.1f}秒")
            prev, t_in = ph, t
    if prev is not None:
        print(f"    {prev:8s} {_f(rows[-1], 'Epoch_s') - t_in:5.1f}秒")

    events = [(r["Time"], r["Event"]) for r in rows if r.get("Event")]
    if events:
        print("  イベント:")
        for tm, ev in events:
            print(f"    {tm}  {ev}")

    # 機体が GUIDED でいた時間。ここが短ければ「入れて終わった」だけ。
    g = [r for r in rows if r.get("Guided") == "1"]
    if g:
        print(f"  GUIDED 滞在 {_f(g[-1], 'Epoch_s') - _f(g[0], 'Epoch_s'):.1f}秒 "
              f"({len(g)}指令)")
    else:
        print("  GUIDED に一度も入っていません "
              "(SW_HOVER が上か / cmd_fresh が立っているか を確認)")

    stale = sum(1 for r in rows if r.get("Cmd_Fresh") == "0")
    if stale:
        print(f"  [!] 指令が届いていない行 {stale}/{len(rows)} "
              "← 無線の上りが細っています")


def report_divergence(rows):
    """カメラの絶対位置と、機体自身のフロー推定のズレ。"""
    print("\n== 自己位置のズレ (カメラ基準 - 機体のフロー推定) ==")
    d = [r for r in rows if r.get("Diff_Norm(m)")]
    if not d:
        print("  比較できる行がありません (原点合わせが成立していない)")
        return
    n, med, mx = _stat([_f(r, "Diff_Norm(m)") for r in d])
    t0 = _f(d[0], "Epoch_s")
    print(f"  {n}点  中央 {med:.2f}m  最大 {mx:.2f}m")
    print("  時間  ずれX     ずれY     大きさ")
    step = max(1, len(d) // 8)
    for r in d[::step]:
        print(f"   {_f(r, 'Epoch_s') - t0:5.1f}s "
              f"{_f(r, 'Diff_X(m)'):+7.2f}m {_f(r, 'Diff_Y(m)'):+7.2f}m "
              f"{_f(r, 'Diff_Norm(m)'):7.2f}m")
    if mx > 0.3:
        print("  → 機体のフロー推定が流れています。機体だけで位置を保とうとすると"
              "この量だけずれます")


def report_tracking_error(rows):
    """目標にどれだけ寄れていたか (制御そのものの出来)。"""
    print("\n== 追従誤差 (目標 - カメラ実測) ==")
    cr = [r for r in rows if r["Phase"] in ("CRUISE", "DWELL")
          and r.get("Dist_H(m)")]
    if not cr:
        print("  CRUISE/DWELL の行がありません")
        return
    n, med, mx = _stat([_f(r, "Dist_H(m)") for r in cr])
    print(f"  水平距離 中央 {med:.2f}m  最大 {mx:.2f}m  ({n}点)")

    # 横ぶれ: 目標軸から外れた量の振れ幅
    xs = [_f(r, "Cam_X(m)") for r in cr if _f(r, "Cam_X(m)") is not None]
    ys = [_f(r, "Cam_Y(m)") for r in cr if _f(r, "Cam_Y(m)") is not None]
    if xs:
        print(f"  実測X {min(xs):+.2f} 〜 {max(xs):+.2f} m (振れ幅 {max(xs) - min(xs):.2f}m)")
        print(f"  実測Y {min(ys):+.2f} 〜 {max(ys):+.2f} m (振れ幅 {max(ys) - min(ys):.2f}m)")

    v = [(_f(r, "Cmd_Vx(m/s)"), _f(r, "Cmd_Vy(m/s)")) for r in cr]
    vx = [a for a, _ in v if a is not None]
    vy = [b for _, b in v if b is not None]
    if vx:
        print(f"  指令速度 前後 {min(vx):+.2f}〜{max(vx):+.2f}  "
              f"左右 {min(vy):+.2f}〜{max(vy):+.2f} m/s")


def main():
    stamp = sys.argv[1] if len(sys.argv) > 1 else None
    flight = find_session(stamp)
    if flight is None:
        print(f"ログが見つかりません: {LOG_DIR}")
        return
    ts = flight.stem[len("flight_"):]
    mission = LOG_DIR / f"mission_{ts}.csv"

    print("=" * 62)
    print(f"  飛行ログ解析: {ts}")
    print("=" * 62)

    f_rows = _rows(flight)
    m_rows = _rows(mission)

    report_camera(f_rows)
    report_mission(m_rows)
    if m_rows:
        report_tracking_error(m_rows)
        report_divergence(m_rows)
    print()


if __name__ == "__main__":
    main()
