"""
calib_store.py
カメラキャリブレーション結果をJSONで保存・復元する。

2種類を明確に分けて扱う:

  内部パラメータ (K, dist)   … レンズ自体の性質。事前にチェッカーボードで実測し、
                               calib/intrinsics_<label>.json に置く。
                               レンズ・解像度が変わらない限り使い回す（当日は読むだけ）。
                               → tools/calibrate_intrinsics.py で作成

  外部パラメータ (R, tvec)   … カメラの設置位置と向き。置き直すたびに変わる。
                               calib/<label>.json に保存。当日5点クリックで求める。
"""

import json
import numpy as np
from utils.config import LOG_DIR

CALIB_DIR = LOG_DIR.parent / "calib"
CALIB_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# 内部パラメータ (事前測定)
# ============================================================

def load_intrinsics(label: str, width: int = None, height: int = None):
    """
    tools/calibrate_intrinsics.py が保存した内部パラメータを読む。

    実行時の解像度が測定時と違う場合は K をスケールして返す。
    （アスペクト比まで違う場合は画角が変わっているので警告し、None を返す）

    Returns:
        dict {"K", "dist", "width", "height", "rms_px"} または None
    """
    path = CALIB_DIR / f"intrinsics_{label}.json"
    if not path.exists():
        return None

    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        K    = np.array(d["K"], dtype=np.float32)
        dist = np.array(d["dist"], dtype=np.float32).reshape(-1, 1)
        cw, ch = int(d["width"]), int(d["height"])
        rms = d.get("rms_px")
    except (json.JSONDecodeError, KeyError, ValueError, OSError) as e:
        print(f"  [WARN] {label} 内部パラメータの読込に失敗: {e}")
        return None

    if width and height and (width, height) != (cw, ch):
        aspect_measured = cw / ch
        aspect_runtime  = width / height
        if abs(aspect_measured - aspect_runtime) > 0.02:
            print(f"  [WARN] {label} 内部パラメータのアスペクト比が実行時と違います "
                  f"（測定 {cw}x{ch} / 実行 {width}x{height}）。")
            print("         画角そのものが違うため、この値は使えません。")
            print("         本番と同じ解像度で撮り直してください。")
            return None

        sx, sy = width / cw, height / ch
        K = K.copy()
        K[0, 0] *= sx; K[0, 2] *= sx
        K[1, 1] *= sy; K[1, 2] *= sy
        print(f"  [{label}] 内部パラメータを {cw}x{ch} → {width}x{height} にスケールしました")
        cw, ch = width, height

    fx, hfov = K[0, 0], 0.0
    if fx > 0:
        hfov = 2 * np.degrees(np.arctan(cw / (2 * fx)))
    msg = f"  [{label}] 実測の内部パラメータを使用 (水平画角 {hfov:.1f}°"
    if rms is not None:
        msg += f", RMS {rms:.3f}px"
    print(msg + ")")

    return {"K": K, "dist": dist, "width": cw, "height": ch, "rms_px": rms}


# ============================================================
# 外部パラメータ (当日測定)
# ============================================================

def save_calibration(label: str, K, R, tvec,
                     points=None, w=None, h=None, dist=None,
                     reproj_px=None, preset=None):
    path = CALIB_DIR / f"{label}.json"
    data = {
        "K":      np.asarray(K).tolist(),
        "dist":   np.asarray(dist).ravel().tolist() if dist is not None else None,
        "R":      np.asarray(R).tolist(),
        "tvec":   np.asarray(tvec).tolist(),
        "points": points,
        "width":  w,
        "height": h,
        "reproj_px": reproj_px,
        "preset":    preset,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    print(f"  [SAVE] {label} キャリブレーションを保存: {path}")


def load_calibration(label: str):
    """保存済みデータがあれば dict を、なければ None を返す"""
    path = CALIB_DIR / f"{label}.json"
    if not path.exists():
        return None
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        K    = np.array(d["K"],    dtype=np.float32)
        R    = np.array(d["R"],    dtype=np.float64)
        tvec = np.array(d["tvec"], dtype=np.float64)

        raw_dist = d.get("dist")
        dist = (np.array(raw_dist, dtype=np.float32).reshape(-1, 1)
                if raw_dist else None)

        return {"K": K, "dist": dist, "R": R, "tvec": tvec,
                "points": d.get("points"),
                "width": d.get("width"), "height": d.get("height"),
                "reproj_px": d.get("reproj_px"), "preset": d.get("preset")}
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
    if saved.get("preset"):
        print(f"             使用プリセット: {saved['preset']}")
    if saved.get("reproj_px") is not None:
        print(f"             再投影誤差: {saved['reproj_px']:.2f} px")
    ans = input(f"  [{label}] このデータを使いますか？ (y/n): ").strip().lower()
    return ans == "y"
