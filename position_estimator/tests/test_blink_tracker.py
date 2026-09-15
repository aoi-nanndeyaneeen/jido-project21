import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.blink import BlinkTracker
from core.camera import Candidate

LED_HZ = 1000.0 / 167.0     # drone_s5.cpp: millis() % 167 < 83
U, V = 815.0, 483.0


def _led_frame(ts, exposure_s, amp=200.0, base=20.0):
    """静止した点滅LEDを1枚ぶん作る。露出中の点灯割合で輝度が決まる。"""
    tt = np.linspace(ts - exposure_s, ts, 40)
    on = float(np.mean((tt * LED_HZ) % 1.0 < 83.0 / 167.0))
    frame = np.full((720, 1280, 3), base, dtype=np.uint8)
    frame[int(V) - 3:int(V) + 4, int(U) - 3:int(U) + 4] = int(base + amp * on)
    return frame, on


def _run(fps, exposure_s, cand_prob, seconds=30.0, seed=0):
    """cand_prob: 点灯フレームで bright 検知が候補を出す確率 (しきい値ぎりぎり/マスク欠け)。"""
    rng = np.random.default_rng(seed)
    bt = BlinkTracker("cam", 6.0, match_dist_px=30, roi_px=8,
                      history_sec=1.2, min_score=0.5, min_depth=1.5)
    n = int(seconds * fps)
    hits = total = 0
    for k in range(n):
        ts = 100.0 + k / fps
        frame, on = _led_frame(ts, exposure_s)
        cands = []
        if on > 0.5 and rng.random() < cand_prob:
            cands = [Candidate(U, V, 20.0, int(U) - 3, int(V) - 3, 7, 7)]
        out = bt.filter(frame, cands, ts, 1280)
        if ts - 100.0 > 3.0:
            total += 1
            hits += bool(out)
    return hits / total


class BlinkTrackerStationaryTests(unittest.TestCase):
    def test_camera2_like_60fps_reliable_candidates(self):
        self.assertGreater(_run(60, 1 / 60, cand_prob=1.0), 0.99)

    def test_camera1_like_16fps_intermittent_candidates_stays_locked(self):
        # 2026-09-15 log: Camera1 16fps / 自動露出 62ms、静止機体で
        # 「1.2秒検知 -> 約4秒見失い」を繰り返した状態。
        self.assertGreater(_run(16, 0.062, cand_prob=0.15), 0.95)

    def test_confirmed_track_dropped_when_led_disappears(self):
        bt = BlinkTracker("cam", 6.0, history_sec=1.2)
        fps = 30
        for k in range(int(5 * fps)):
            ts = k / fps
            frame, on = _led_frame(ts, 1 / 60)
            cands = [Candidate(U, V, 20.0, int(U) - 3, int(V) - 3, 7, 7)] if on > 0.5 else []
            bt.filter(frame, cands, ts, 1280)
        dark = np.full((720, 1280, 3), 20, dtype=np.uint8)
        out = None
        for k in range(int(5 * fps), int(9 * fps)):
            out = bt.filter(dark, [], k / fps, 1280)
        self.assertEqual(out, [])


if __name__ == "__main__":
    unittest.main()
