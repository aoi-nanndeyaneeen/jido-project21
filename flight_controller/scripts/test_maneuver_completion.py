"""
test_maneuver_completion.py
include/quad/Maneuver.h の完了判定を PC で検証する (この環境に C++ ホストコンパイラが
無いため、update() のロジックを Python に忠実に移植して回す)。

    python scripts/test_maneuver_completion.py

検証すること:
  旧方式 (指令レート × 経過時間で数える) は、機体が指令ヨーレートに追いつく前の
  立ち上がりぶんを数えないので、**円が閉じきる前に完了扱い**になっていた
  (2026-09-16 実機: 水平旋回2周で指令720°に対し実測609° = 1.69周。判定不成立)。
  新方式 (実測ヨーの積分で数える) は、機体が実際に 360° 回るまで脚を終えないので
  円が閉じる。回れない機体は脚ごとの時間上限で打ち切られ暴走しない。

★ Maneuver.h の update() を変えたら、この移植も合わせること。
"""

LEG_TIME_CAP = 1.6   # Maneuver.h と同じ


def simulate(legs_signs, rate_abs, tau, dt=0.01, mode="meas", cap_s=None):
    """機体ヨーが指令レートに一次遅れ (時定数 tau) で追従するとしてマヌーバを回す。

    mode="meas" : 実測ヨーの積分で完了判定 (新)  /  "cmd" : 指令 × 時間 (旧)
    戻り値: (実測総回転 [周], 所要 [s])
    """
    yaw = rate = 0.0
    leg = 0
    turned = turned_meas = prev_yaw = leg_start = t = 0.0
    have = done = False
    n = len(legs_signs)
    guard = 0
    while not done and guard < 5_000_000:
        guard += 1
        t += dt
        cmd = legs_signs[min(leg, n - 1)] * rate_abs
        rate += (cmd - rate) * (dt / tau)     # レートが指令へ立ち上がる
        yaw += rate * dt
        # --- update() 本体 ---
        turned += rate_abs * dt
        if have:
            d = yaw - prev_yaw
            while d > 180:
                d -= 360
            while d < -180:
                d += 360
            turned_meas += abs(d)
        prev_yaw = yaw
        have = True
        leg_s = 360.0 / rate_abs if rate_abs > 1 else 1e9
        if mode == "meas":
            closed = turned_meas >= 360.0
            timeout = (t - leg_start) >= leg_s * LEG_TIME_CAP
        else:
            closed = turned >= 360.0
            timeout = False
        if closed or timeout:
            leg += 1
            turned -= 360.0
            turned_meas = (turned_meas - 360.0) if (mode == "meas" and closed and turned_meas > 360) else 0.0
            leg_start = t
            if leg >= n:
                done = True
    return abs(yaw) / 360.0, t


def main():
    fails = []

    # 水平旋回2周 (v=0.8, w=30.6deg/s = 半径1.5m)。遅れが大きいほど旧方式は短くなる。
    print("水平旋回2周 (指令720°):")
    for tau in (0.5, 1.0, 1.5, 2.0):
        old, _ = simulate([1, 1], 30.6, tau, mode="cmd")
        new, tt = simulate([1, 1], 30.6, tau, mode="meas")
        print(f"  遅れ tau={tau}s: 旧 {old:.2f}周   新 {new:.2f}周 / {tt:.1f}s")
        if new < 2.0 - 0.1:
            fails.append(f"CIRCLE tau={tau}: 新 {new:.2f}周 < 1.9")

    # 8の字 (右1周 + 左1周)。net はほぼ 0 が正常。各脚が閉じるかは所要時間で見る。
    print("8の字 (右1周+左1周):")
    for tau in (1.0, 1.5):
        new, tt = simulate([1, -1], 30.6, tau, mode="meas")
        # 各脚 360°/30.6 = 11.8s。実測基準なら遅れぶん少し伸びるが、2脚で ~24-27s。
        print(f"  tau={tau}s: net {new:.2f}周 (0付近が正常) / {tt:.1f}s")
        if not (22.0 <= tt <= 32.0):
            fails.append(f"FIGURE8 tau={tau}: 所要 {tt:.1f}s が想定外")

    # 上昇旋回 (低2周 + 上昇1周 + 高2周 = 5脚)
    print("上昇旋回 laps=2 (5脚):")
    new, tt = simulate([1, 1, 1, 1, 1], 30.6, 1.5, mode="meas")
    print(f"  実測 {new:.2f}周 / {tt:.1f}s")
    if new < 5.0 - 0.3:
        fails.append(f"CLIMB: 新 {new:.2f}周 < 4.7")

    # 回れない機体 (レートが立ち上がらない) で暴走しないか
    print("回れない機体 (tau=100s) の暴走防止:")
    new, tt = simulate([1, 1], 30.6, 100.0, mode="meas")
    print(f"  {new:.2f}周で {tt:.1f}s 打ち切り (2脚 x 11.8s x 1.6 = 37.7s が上限)")
    if tt > 40.0:
        fails.append(f"時間上限が効いていない: {tt:.1f}s")

    print()
    if fails:
        print("NG:")
        for f in fails:
            print("  -", f)
        return 1
    print("OK: 全件 円が閉じ、暴走もしない")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
