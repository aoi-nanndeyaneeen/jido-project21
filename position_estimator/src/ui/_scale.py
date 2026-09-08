"""目盛り関連の小さな共有ヘルパー（view_graph / view_velocity 共用）。"""

import math


def nice_step(span, target_ticks=6):
    """目盛り間隔を 1 / 2 / 2.5 / 5 × 10^n に丸める。"""
    if span <= 0:
        return 1.0
    raw = span / target_ticks
    mag = 10 ** math.floor(math.log10(raw))
    for mult in (1, 2, 2.5, 5, 10):
        if raw <= mult * mag:
            return mult * mag
    return 10 * mag


def tick_decimals(step):
    """目盛り値を誤差なく表示できる最小の小数桁数（2.5 のような刻みも考慮）。"""
    for d in range(0, 4):
        scaled = step * (10 ** d)
        if abs(scaled - round(scaled)) < 1e-9:
            return d
    return 4


def format_tick(value, decimals):
    """-0.0 を 0 に潰して固定小数で整形する。"""
    if abs(value) < 0.5 * 10 ** (-decimals):
        value = 0.0
    return f"{value:.{decimals}f}"


def ticks(lo, hi, step):
    """[lo, hi] を含む区間の目盛り位置を列挙する。"""
    out = []
    t = math.ceil(lo / step) * step
    while t <= hi + 1e-6:
        out.append(t)
        t += step
    return out
