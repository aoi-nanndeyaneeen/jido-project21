import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from init import calibration_flow


class FakeCam:
    def __init__(self, name):
        self.name = name
        self.reset_calls = 0

    def reset_background(self):
        self.reset_calls += 1


class CalibrationStaticFilterTests(unittest.TestCase):
    def test_calibrate_camera_resets_static_filter_on_saved_calibration(self):
        cam = FakeCam("cam")
        source = type("Source", (), {"open_live": lambda self, w, h: cam})()

        with mock.patch.object(calibration_flow, "load_calibration", return_value={
            "R": np.eye(3),
            "tvec": np.zeros((3, 1)),
            "points": None,
            "width": 1280,
            "height": 720,
            "reproj_px": None,
            "preset": None,
        }):
            with mock.patch.object(calibration_flow, "ask_use_saved", return_value=True):
                with mock.patch.object(calibration_flow, "resolve_intrinsics", return_value=(np.eye(3), np.zeros((5, 1)))):
                    K, dist, R, tvec, points, out_cam = calibration_flow._calibrate_camera(
                        "Camera1", source, True, [0, 0, 0], None
                    )

        self.assertIsNotNone(K)
        self.assertIs(out_cam, cam)
        self.assertEqual(cam.reset_calls, 1)


if __name__ == "__main__":
    unittest.main()
