"""
calib_store.py
カメラキャリブレーション結果（K, R, tvec）をJSONで保存・復元する。
"""

import json
import numpy as np
from pathlib import Path
from utils.config import LOG_DIR

CALIB_DIR = LOG_DIR.parent / "calib"
CALIB_DIR.mkdir(parents=True, exist_ok=True)


def save_calibration(label: str, K, R, tvec, points=None, w=None, h=None):
    path = CALIB_DIR / f"{label}.json"
    data = {
        "K":    K.tolist(),
        "R":    R.tolist(),
        "tvec": tvec.tolist(),
        "points": points,
        "width":  w,
        "height": h,
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"  [SAVE] {label} キャリブレーションを保存: {path}")


def load_calibration(label: str):
    """保存済みデータがあれば dict を、なければ None を返す"""
    path = CALIB_DIR / f"{label}.json"
    if not path.exists():
        return None
    try:
        with open(path) as f:
            d = json.load(f)
        K    = np.array(d["K"],    dtype=np.float32)
        R    = np.array(d["R"],    dtype=np.float64)
        tvec = np.array(d["tvec"], dtype=np.float64)
        return {"K": K, "R": R, "tvec": tvec,
                "points": d.get("points"),
                "width": d.get("width"), "height": d.get("height")}
    except Exception as e:
        print(f"  [WARN] {label} キャリブ読込失敗: {e}")
        return None


def ask_use_saved(label: str) -> bool:
    """保存データがあれば使うか確認。なければ False を返す"""
    saved = load_calibration(label)
    if saved is None:
        return False
    cam_pos = -saved["R"].T.dot(saved["tvec"])
    print(f"\n  [{label}] 保存済みキャリブレーションが見つかりました:")
    print(f"             位置: X={cam_pos[0,0]:.2f} "
          f"Y={cam_pos[1,0]:.2f} Z={cam_pos[2,0]:.2f}m")
    ans = input(f"  [{label}] このデータを使いますか？ (y/n): ").strip().lower()
    return ans == "y"