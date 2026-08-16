"""
tools/calibrate_intrinsics.py

チェッカーボードによるカメラ内部パラメータ (K, distCoeffs) の実測。

【これは事前に自宅で1回だけ行う作業です。大会当日は実行しません。】

    内部パラメータ (K, dist)  = レンズ自体の性質（焦点距離・光学中心・歪み）
                                → レンズと解像度が変わらない限り不変。使い回す。
    外部パラメータ (R, tvec)  = カメラをどこに置いてどこを向けたか
                                → 置き直すたびに変わる。当日フィールド5点クリックで求める。

このスクリプトは前者だけを求めます。当日の準備時間には一切影響しません。

--------------------------------------------------------------------------
準備するもの
--------------------------------------------------------------------------
  ・チェッカーボード（格子模様）を印刷し、平らな板に貼る
      ★ 反りがあると精度が出ません。段ボールやスチレンボードに全面糊付けを推奨。
      ★ 大きいほど良い。A3は最低限、A2〜A1あれば余裕を持って離せます。
  ・マス目の一辺の実寸をノギス/定規で測っておく（mm）
      ※ K と dist の値自体はマス目サイズに依存しませんが、記録として保存します。

--------------------------------------------------------------------------
重要: 本番と同じ設定で撮ること
--------------------------------------------------------------------------
  1) 解像度を本番と一致させる（既定 1280x720）。
     違う解像度で測った K はそのままでは使えません。
  2) フォーカスを本番と同じ「無限遠固定」にする。
     ★ フォーカスが動くと焦点距離が変わるため、K が意味を失います。
     ★ 無限遠固定のままボードを近づけるとボケて検出できません。
        - C920 など小型センサ機は被写界深度が深いので 1〜2m でも大抵通ります。
        - α6400 + 16mm は F8 程度まで絞ってください
          （F8 の過焦点距離は約1.7m。F2.8 のままだと約4.8mまで必要）。
          そのうえでボードを3m以上離すため、大きめのボードが要ります。
  3) α6400 を OpenCV から直接読めない場合は --source にフォルダを指定し、
     ★ 本番と同じ解像度・同じ画角で撮った画像を入れてください。
        α6400 の静止画は3:2、動画は16:9（上下クロップ）で画角が違います。
        キャプチャカードや Imaging Edge Webcam 経由でライブ撮影できるなら
        そちらの方が確実です。

--------------------------------------------------------------------------
使い方
--------------------------------------------------------------------------
  # ライブ撮影（推奨）
  python tools/calibrate_intrinsics.py --label Camera1 --source 1

  # 撮影済み画像フォルダから
  python tools/calibrate_intrinsics.py --label Camera2 --source ./shots

  # ボードのサイズが違う場合（--cols/--rows は「内側の交点の数」）
  python tools/calibrate_intrinsics.py --label Camera1 --source 1 --cols 9 --rows 6 --square 25

  キー操作:  SPACE=撮影   C=計算して保存   U=直前を取消   Q=中断

--------------------------------------------------------------------------
撮り方のコツ
--------------------------------------------------------------------------
  ・20枚程度。画面の四隅・中央をまんべんなく埋める（歪み係数は周辺部で決まる）。
  ・ボードを傾ける（正対だけだと焦点距離と距離が分離できず解が不安定になる）。
  ・ブレを避ける。動かした直後は1秒待つ。
  ・画面下のカバレッジ表示が全体的に埋まったら十分です。
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT_DIR  = Path(__file__).resolve().parent.parent
CALIB_DIR = ROOT_DIR / "calib"

# カバレッジ表示の分割数
COV_COLS, COV_ROWS = 8, 5


def build_object_points(cols: int, rows: int, square_m: float) -> np.ndarray:
    """チェッカーボード1枚分の3Dモデル点（z=0 平面上の格子）を作る。"""
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return objp * square_m


def find_corners(gray: np.ndarray, cols: int, rows: int):
    """
    チェッカーボードの交点を探す。
    OpenCV 4.x の findChessboardCornersSB があればそちらを使う（高速かつ頑健）。
    """
    pattern = (cols, rows)

    if hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags)
        if ok:
            return True, corners

    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
             | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return False, None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def coverage_grid(all_corners, w: int, h: int) -> np.ndarray:
    """撮影済みコーナーが画面のどのセルを埋めたかを bool 格子で返す。"""
    grid = np.zeros((COV_ROWS, COV_COLS), dtype=bool)
    for corners in all_corners:
        for pt in corners.reshape(-1, 2):
            c = min(int(pt[0] / w * COV_COLS), COV_COLS - 1)
            r = min(int(pt[1] / h * COV_ROWS), COV_ROWS - 1)
            if c >= 0 and r >= 0:
                grid[r, c] = True
    return grid


def draw_coverage(frame: np.ndarray, grid: np.ndarray) -> None:
    """カバレッジ格子を画面右下に小さく重ねて描く。"""
    h, w = frame.shape[:2]
    cell = 18
    x0 = w - COV_COLS * cell - 20
    y0 = h - COV_ROWS * cell - 20

    cv2.rectangle(frame, (x0 - 6, y0 - 24),
                  (x0 + COV_COLS * cell + 6, y0 + COV_ROWS * cell + 6),
                  (0, 0, 0), -1)
    cv2.putText(frame, "coverage", (x0 - 2, y0 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    for r in range(COV_ROWS):
        for c in range(COV_COLS):
            p1 = (x0 + c * cell, y0 + r * cell)
            p2 = (p1[0] + cell - 2, p1[1] + cell - 2)
            color = (0, 200, 0) if grid[r, c] else (60, 60, 60)
            cv2.rectangle(frame, p1, p2, color, -1)


def open_live_source(source: str, width: int, height: int, focus: int):
    """ライブカメラを本番と同じ設定で開く。"""
    cap = cv2.VideoCapture(int(source), cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(f"カメラ {source} を開けません。")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # ★ 本番と同じフォーカス設定にする（これを外すと K が無意味になる）
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, focus)

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[live] 要求 {width}x{height} -> 実際 {aw}x{ah}")
    if (aw, ah) != (width, height):
        print("  [WARN] 解像度が要求と違います。本番も同じ実解像度になるか確認してください。")
    return cap, aw, ah


def collect_live(source, cols, rows, width, height, focus, target):
    """ライブプレビューで撮影し、検出できたコーナー列を集める。"""
    cap, w, h = open_live_source(source, width, height, focus)
    all_corners = []
    last_capture_t = 0.0

    print("\n  SPACE=撮影   C=計算して保存   U=直前を取消   Q=中断\n")
    win = "Intrinsics Calibration"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, min(w, 1280), min(h, 720))

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = find_corners(gray, cols, rows)

            disp = frame.copy()
            if found:
                cv2.drawChessboardCorners(disp, (cols, rows), corners, True)

            grid = coverage_grid(all_corners, w, h)
            filled = int(grid.sum())
            total = COV_COLS * COV_ROWS

            status = "BOARD FOUND - press SPACE" if found else "searching board..."
            color = (0, 255, 0) if found else (120, 120, 120)
            cv2.putText(disp, status, (12, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            cv2.putText(disp, f"captured: {len(all_corners)}/{target}   "
                              f"coverage: {filled}/{total}",
                        (12, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            if time.time() - last_capture_t < 0.6:
                cv2.putText(disp, "CAPTURED", (12, 104),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
            draw_coverage(disp, grid)

            cv2.imshow(win, disp)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' ') and found:
                all_corners.append(corners)
                last_capture_t = time.time()
                print(f"  撮影 {len(all_corners)}枚目")
            elif key == ord('u') and all_corners:
                all_corners.pop()
                print(f"  取消 → {len(all_corners)}枚")
            elif key == ord('c'):
                break
            elif key == ord('q'):
                all_corners = []
                break
    finally:
        cap.release()
        cv2.destroyWindow(win)
        for _ in range(5):
            cv2.waitKey(1)

    return all_corners, w, h


def collect_folder(folder: Path, cols: int, rows: int):
    """撮影済み画像フォルダからコーナーを集める。"""
    exts = ("*.jpg", "*.jpeg", "*.png", "*.JPG", "*.JPEG", "*.PNG")
    files = sorted({p for e in exts for p in folder.glob(e)})
    if not files:
        raise RuntimeError(f"{folder} に画像が見つかりません。")

    all_corners = []
    size = None
    for p in files:
        img = cv2.imread(str(p))
        if img is None:
            print(f"  [SKIP] 読込失敗: {p.name}")
            continue
        h, w = img.shape[:2]
        if size is None:
            size = (w, h)
        elif (w, h) != size:
            print(f"  [SKIP] 解像度が他と違う ({w}x{h}): {p.name}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = find_corners(gray, cols, rows)
        if found:
            all_corners.append(corners)
            print(f"  [OK]   {p.name}")
        else:
            print(f"  [MISS] ボード未検出: {p.name}")

    if size is None:
        raise RuntimeError("有効な画像がありませんでした。")
    return all_corners, size[0], size[1]


def calibrate(all_corners, objp, w, h):
    """calibrateCamera を実行し、K・dist・RMS・画像ごとの誤差を返す。"""
    obj_points = [objp for _ in all_corners]
    img_points = [c.reshape(-1, 1, 2).astype(np.float32) for c in all_corners]

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None
    )

    per_image = []
    for i in range(len(obj_points)):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(img_points[i], proj, cv2.NORM_L2) / len(proj)
        per_image.append(float(err))

    return rms, K, dist, per_image


def report(K, dist, rms, per_image, w, h):
    """結果を人間が判断できる形で表示する。"""
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    hfov = 2 * np.degrees(np.arctan(w / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(h / (2 * fy)))

    print("\n" + "=" * 62)
    print("  内部パラメータ 測定結果")
    print("=" * 62)
    print(f"  解像度      : {w} x {h}")
    print(f"  fx, fy      : {fx:.2f}, {fy:.2f} px")
    print(f"  cx, cy      : {cx:.2f}, {cy:.2f} px  (画像中心: {w/2:.1f}, {h/2:.1f})")
    print(f"  水平画角    : {hfov:.1f}°")
    print(f"  垂直画角    : {vfov:.1f}°")
    print(f"  歪み係数    : k1={dist[0]:.4f} k2={dist[1]:.4f} "
          f"p1={dist[2]:.4f} p2={dist[3]:.4f} k3={dist[4]:.4f}")
    print(f"  RMS再投影誤差: {rms:.4f} px")

    print("\n  判定:")
    if rms < 0.5:
        print("    [良好] そのまま使えます。")
    elif rms < 1.0:
        print("    [許容] 使えますが、周辺部の枚数を増やすと改善します。")
    else:
        print("    [要再撮影] 誤差が大きすぎます。")
        print("      - ボードが反っていないか（平面性が最重要）")
        print("      - ブレていないか、ピントが合っているか")
        print("      - 傾けたポーズが含まれているか（正対だけだと不安定）")

    worst = sorted(range(len(per_image)), key=lambda i: -per_image[i])[:3]
    print("\n  誤差の大きい画像 (0始まりの撮影順):")
    for i in worst:
        print(f"    #{i}: {per_image[i]:.3f} px")
    print("    ※ 突出して悪い1枚があれば、それを除いて撮り直すと改善します。")

    if abs(cx - w / 2) > w * 0.1 or abs(cy - h / 2) > h * 0.1:
        print("\n  [WARN] 光学中心が画像中心から大きくずれています。")
        print("         カバレッジ不足の可能性があります（枚数と配置を増やしてください）。")

    print("=" * 62)


def save(label, K, dist, rms, w, h, n, cols, rows, square_mm):
    CALIB_DIR.mkdir(parents=True, exist_ok=True)
    path = CALIB_DIR / f"intrinsics_{label}.json"
    data = {
        "label": label,
        "width": int(w),
        "height": int(h),
        "K": K.tolist(),
        "dist": np.asarray(dist).ravel().tolist(),
        "rms_px": float(rms),
        "n_images": int(n),
        "board": {"cols": cols, "rows": rows, "square_mm": square_mm},
        "captured_at": datetime.now().isoformat(timespec="seconds"),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    print(f"\n  [SAVE] {path}")
    print("  当日はこのファイルが自動的に読み込まれます（撮影は不要です）。")


def main():
    ap = argparse.ArgumentParser(
        description="チェッカーボードでカメラ内部パラメータを実測する（事前作業）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", required=True,
                    help="保存名。Camera1 / Camera2 のいずれか")
    ap.add_argument("--source", required=True,
                    help="カメラID（例 1）または撮影済み画像フォルダのパス")
    ap.add_argument("--cols", type=int, default=9,
                    help="ボード内側交点の列数（既定 9）")
    ap.add_argument("--rows", type=int, default=6,
                    help="ボード内側交点の行数（既定 6）")
    ap.add_argument("--square", type=float, default=25.0,
                    help="マス目の一辺 [mm]（既定 25.0）")
    ap.add_argument("--width",  type=int, default=1280, help="ライブ撮影の要求幅")
    ap.add_argument("--height", type=int, default=720,  help="ライブ撮影の要求高さ")
    ap.add_argument("--focus",  type=int, default=0,
                    help="固定フォーカス値。0=無限遠（本番と必ず一致させる）")
    ap.add_argument("--count",  type=int, default=20, help="目標撮影枚数")
    args = ap.parse_args()

    print(f"\n  ボード: 内側交点 {args.cols} x {args.rows}, マス目 {args.square}mm")
    print("  ※ --cols/--rows は『マスの数』ではなく『内側の交点の数』です。")
    print("     例: 10x7マスの市松模様 → 交点は 9x6\n")

    src_path = Path(args.source)
    if src_path.is_dir():
        print(f"  フォルダから読み込み: {src_path}")
        all_corners, w, h = collect_folder(src_path, args.cols, args.rows)
    else:
        all_corners, w, h = collect_live(
            args.source, args.cols, args.rows,
            args.width, args.height, args.focus, args.count)

    n = len(all_corners)
    print(f"\n  有効な撮影: {n} 枚")
    if n < 5:
        print("  [FAIL] 枚数が足りません（最低5枚、推奨20枚）。中断します。")
        return 1
    if n < 10:
        print("  [WARN] 枚数が少なめです。歪み係数の精度は期待できません。")

    objp = build_object_points(args.cols, args.rows, args.square / 1000.0)
    rms, K, dist, per_image = calibrate(all_corners, objp, w, h)

    report(K, dist, rms, per_image, w, h)
    save(args.label, K, dist, rms, w, h, n,
         args.cols, args.rows, args.square)
    return 0


if __name__ == "__main__":
    sys.exit(main())
