"""
Camera1 の検知系を実録画で回帰テストする。

tests/画面録画 2026-09-16 164847.mp4:
  Camera1 (α6400) の窓の画面録画。フィールド対角の約 10m (カメラから最も
  遠い点) に 6Hz 点滅 LED の機体を静置。LED は 1280x720 で 3px 角、
  ピーク輝度 200 程度。共通の検知パラメータでは 0% だった (2026-09-16)。
  正解位置 (527,384) は録画を目視して決めた。

  tools/replay_detect.py と同じ経路 (detect -> BlinkTracker) で流し、
  静的輝点マスク学習後、点滅確認済み候補が正解 ±12px に出た割合を見る。
  ロックインに 1 秒ほど要するので、確定後の連続性も別に見る。
"""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for d in (ROOT / "src", ROOT / "tools"):
    if str(d) not in sys.path:
        sys.path.insert(0, str(d))

from utils.config import detection_params_for, blink_params_for   # noqa: E402

VIDEO = ROOT / "tests" / "画面録画 2026-09-16 164847.mp4"
TRUTH = (527.0, 384.0)


@unittest.skipUnless(VIDEO.exists(), "録画が無い")
class Camera1ReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import replay_detect
        cls.stats, cls.per_frame = replay_detect.replay(VIDEO, truth=TRUTH)

    def test_far_drone_is_confirmed_most_of_the_time(self):
        n = self.stats["scored"]
        self.assertGreater(n, 100)
        self.assertGreater(self.stats["blink_hit"] / n, 0.8)
        self.assertEqual(self.stats["false_confirmed"], 0)
        self.assertEqual(self.stats["rejected"], 0)

    def test_once_locked_it_stays_locked(self):
        hits = [bool(row[4]) for row in self.per_frame]
        first = hits.index(True)
        after = hits[first:]
        self.assertGreater(len(after), 80)
        self.assertGreater(sum(after) / len(after), 0.97)

    def test_static_lights_do_not_score_like_the_led(self):
        # 正解以外のトラックが確定しきい値に届いていない (余裕の確認)
        self.assertLess(self.stats["false_best_score"], blink_params_for("Camera1")["min_score"])


class PerCameraParamTests(unittest.TestCase):
    def test_camera1_overrides_do_not_leak_to_camera2(self):
        p1, p2 = detection_params_for("Camera1"), detection_params_for("Camera2")
        self.assertEqual(p1["bright_threshold"], 140)
        self.assertEqual(p2["bright_threshold"], 230)
        self.assertEqual(p2["blur_kernel"], 5)
        self.assertEqual(blink_params_for("Camera2")["roi_px"], 8)
        self.assertEqual(blink_params_for("Camera1")["roi_px"], 3)


if __name__ == "__main__":
    unittest.main()
