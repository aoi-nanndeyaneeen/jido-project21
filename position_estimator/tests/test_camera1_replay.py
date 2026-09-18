"""
Camera1 の検知系を実録画で回帰テストする。

tests/画面録画 2026-09-16 164847.mp4:
  Camera1 (α6400) の窓の画面録画。フィールド対角の約 10m (カメラから最も
  遠い点) に 6Hz 点滅 LED の機体を静置。LED は 1280x720 で 3px 角、
  ピーク輝度 200 程度。共通の検知パラメータでは 0% だった (2026-09-16)。
  正解位置 (527,384) は録画を目視して決めた。

  tools/replay_detect.py と同じ経路 (detect -> BlinkTracker) で流し、
  点滅確認済み候補が正解 ±12px に出た割合を見る。点滅の確定には履歴が
  1 秒ほど要るので、確定後の連続性も別に見る。
  2026-09-17 から Camera1 は flicker 検知 + 2値トグル判定 (静的輝点マスク無し)。
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

# 2026-09-17 09:36:51: α6400 が 1/15s +0.3EV の自動露出で会場が明るく写り、
# 実効 10fps・フレーム間隔もばらつく。bright + ロックインでは 0% だった。
# 機体は床に静置 (LED は点灯 200 / 消灯 113)。正解は6Hz 成分の振幅マップから。
VIDEO_LONG_EXPOSURE = ROOT / "tests" / "画面録画 2026-09-17 093651.mp4"
CROP_LONG_EXPOSURE = "15,55,1454,917"
TRUTH_LONG_EXPOSURE = (1127.0, 544.0)

# 2026-09-17 10:08:01: 同じ露出のまま、0.8m/s 以下でフィールド内を 30 秒動き続ける機体。
# 画面上 最大 ~500px/s。正解は連続フレームの輝度差ブロブを追って作り目視で確認した
# CSV (誤差 ~20px なので ±25px で見る)。修正前は点滅確定 0.4%。
VIDEO_MOVING = ROOT / "tests" / "画面録画 2026-09-17 100801.mp4"
CROP_MOVING = "15,55,1454,917"
TRUTH_MOVING = ROOT / "tests" / "画面録画 2026-09-17 100801_truth.csv"

# ★ 録画にしか無い誤検知源: 当時の窓に描かれていたスコア文字・DUMMY の機体アイコン
#   (実機では detect() は描き込み前の画像を見る)。これらが確定されることがあるので、
#   「正解以外の確定」は 0 ではなく、集計フレームあたりの割合で上限を見る。


@unittest.skipUnless(VIDEO.exists(), "録画が無い")
class Camera1ReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import replay_detect
        cls.stats, cls.per_frame = replay_detect.replay(VIDEO, truth=TRUTH)

    def test_far_drone_is_confirmed_most_of_the_time(self):
        n = self.stats["scored"]
        self.assertGreater(n, 100)
        self.assertGreater(self.stats["blink_hit"] / n, 0.85)
        self.assertLess(self.stats["false_confirmed"] / n, 0.15)
        self.assertEqual(self.stats["rejected"], 0)

    def test_once_locked_it_stays_locked(self):
        hits = [bool(row[4]) for row in self.per_frame]
        first = hits.index(True)
        after = hits[first:]
        self.assertGreater(len(after), 80)
        self.assertGreater(sum(after) / len(after), 0.95)


@unittest.skipUnless(VIDEO_LONG_EXPOSURE.exists(), "録画が無い")
class Camera1LongExposureReplayTests(unittest.TestCase):
    """明るい会場・長露出・不規則なフレーム間隔でも flicker + トグル判定で拾える。"""

    @classmethod
    def setUpClass(cls):
        import replay_detect
        cls.stats, cls.per_frame = replay_detect.replay(
            VIDEO_LONG_EXPOSURE, crop=CROP_LONG_EXPOSURE, truth=TRUTH_LONG_EXPOSURE)

    def test_drone_is_candidate_and_confirmed(self):
        n = self.stats["scored"]
        self.assertGreater(n, 80)
        # 候補は点灯したフレームだけ出る (消灯中はトラックが位置を持つ)
        self.assertGreater(self.stats["raw_hit"] / n, 0.4)
        # 最初の約 1 秒は点滅の履歴が溜まるまで確定できない
        self.assertGreater(self.stats["blink_hit"] / n, 0.75)
        self.assertLess(self.stats["false_confirmed"] / n, 0.15)
        self.assertEqual(self.stats["rejected"], 0)

    def test_once_locked_it_mostly_stays_locked(self):
        hits = [bool(row[4]) for row in self.per_frame]
        after = hits[hits.index(True):]
        self.assertGreater(sum(after) / len(after), 0.9)


@unittest.skipUnless(VIDEO_MOVING.exists() and TRUTH_MOVING.exists(), "録画が無い")
class Camera1MovingReplayTests(unittest.TestCase):
    """動き続ける機体に追従して点滅を確定し続ける。"""

    @classmethod
    def setUpClass(cls):
        import replay_detect
        cls.stats, cls.per_frame = replay_detect.replay(
            VIDEO_MOVING, crop=CROP_MOVING, truth=TRUTH_MOVING, truth_px=25.0)

    def test_moving_drone_is_confirmed_most_of_the_time(self):
        n = self.stats["scored"]
        self.assertGreater(n, 250)
        # 白飛びした窓・壁の前を横切る間は LED の明暗が消えるので 100% にはならない
        self.assertGreater(self.stats["blink_hit"] / n, 0.65)
        self.assertLess(self.stats["false_confirmed"] / n, 0.25)

    def test_no_long_gaps(self):
        # 見失っても 2 秒 (実効 10fps で 20 枚) 以内に復帰する
        gap = longest = 0
        for row in self.per_frame:
            gap = 0 if row[4] else gap + 1
            longest = max(longest, gap)
        self.assertLessEqual(longest, 20)


class PerCameraParamTests(unittest.TestCase):
    def test_camera1_overrides_do_not_leak_to_camera2(self):
        p1, p2 = detection_params_for("Camera1"), detection_params_for("Camera2")
        self.assertEqual(p1["bright_threshold"], 140)
        self.assertEqual(p2["bright_threshold"], 230)
        self.assertEqual(p2["blur_kernel"], 5)
        self.assertEqual(blink_params_for("Camera2")["roi_px"], 8)
        self.assertEqual(blink_params_for("Camera1")["roi_px"], 2)
        self.assertIsNone(blink_params_for("Camera2").get("max_speed_px_s"))
        self.assertEqual(p1["detect_mode"], "flicker")
        self.assertEqual(p2["detect_mode"], "bright")
        self.assertIsNone(blink_params_for("Camera2").get("toggle_min_hz"))
        self.assertIsNotNone(blink_params_for("Camera1")["toggle_min_hz"])


if __name__ == "__main__":
    unittest.main()
