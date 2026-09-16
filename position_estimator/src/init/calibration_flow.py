"""
init/calibration_flow.py
Camera1・Camera2 の外部パラメータ（設置位置と向き）を求めるフロー。

内部パラメータ (K, dist) は事前にチェッカーボードで実測済みのものを
calib/intrinsics_<label>.json から読む（ここでは測らない）。

キャリブレーション完了時に、その結果が信用できるかを2つの指標で検証する:
  1) 再投影誤差 [px] … 各カメラ内部の辻褄。順序ミス・座標入力ミスが一発で分かる
  2) 三角測量誤差 [m] … 2台の相対関係。位置推定で実際に使う量そのもの
"""

import cv2
import json
import socket
import time
import numpy as np

from utils.config import (FIELD_POINTS, CALIB_POINT_LABELS, CALIB_PRESET,
                          DISP_W, DISP_H, RPI_HOST, RPI_PORT,
                          REPROJ_WARN_PX, REPROJ_FAIL_PX,
                          TRIANG_WARN_M, TRIANG_FAIL_M,
                          LOCK_EXPOSURE_AFTER_CALIB, CAMERA2_SOURCE)
from core.remote_camera import RemoteCamera
from core.geometry import (CameraCalib, solve_extrinsics, reprojection_error,
                           get_ray, intersect_rays)
from utils.calib_store import (save_calibration, load_calibration,
                               ask_use_saved, resolve_intrinsics)

# クリック位置をサブピクセル補正する際、これ以上動いたら補正を却下する [px]
SUBPIX_MAX_SHIFT_PX = 6.0
# ルーペ（拡大表示）の設定
LOUPE_SRC_PX  = 60    # 元画像から切り出す一辺
LOUPE_ZOOM    = 4


# ============================================================
# クリック補助
# ============================================================

def _refine_click(gray, pt):
    """
    クリック位置をライン交点にサブピクセル吸着させる。

    人手のクリック誤差±3〜5pxが±0.5px程度になる。
    ただし床のライン交点のような「コーナー」でないと誤動作するため、
    大きく動いた場合は補正を却下して元のクリック位置を返す。
    """
    try:
        corner = np.array([[list(map(float, pt))]], dtype=np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
        refined = cv2.cornerSubPix(gray, corner, (9, 9), (-1, -1), criteria)
        rp = refined[0][0]
        if np.hypot(rp[0] - pt[0], rp[1] - pt[1]) <= SUBPIX_MAX_SHIFT_PX:
            return [float(rp[0]), float(rp[1])]
    except cv2.error:
        pass
    return [float(pt[0]), float(pt[1])]


def _draw_loupe(disp, frame, mouse_xy):
    """マウス周辺を拡大表示する。クリック精度を上げるための補助。"""
    if mouse_xy is None:
        return
    mx, my = mouse_xy
    h, w = frame.shape[:2]
    half = LOUPE_SRC_PX // 2
    x0, y0 = max(0, mx - half), max(0, my - half)
    x1, y1 = min(w, mx + half), min(h, my + half)
    if x1 - x0 < 8 or y1 - y0 < 8:
        return

    patch = frame[y0:y1, x0:x1]
    zoom = cv2.resize(patch, None, fx=LOUPE_ZOOM, fy=LOUPE_ZOOM,
                      interpolation=cv2.INTER_NEAREST)
    zh, zw = zoom.shape[:2]

    # マウス位置に対応する拡大画像内の座標へ十字を描く
    cx = int((mx - x0) * LOUPE_ZOOM)
    cy = int((my - y0) * LOUPE_ZOOM)
    cv2.line(zoom, (cx, 0), (cx, zh), (0, 255, 255), 1)
    cv2.line(zoom, (0, cy), (zw, cy), (0, 255, 255), 1)
    cv2.rectangle(zoom, (0, 0), (zw - 1, zh - 1), (255, 255, 255), 1)

    px, py = disp.shape[1] - zw - 12, 12
    if py + zh <= disp.shape[0] and px >= 0:
        disp[py:py + zh, px:px + zw] = zoom


# ============================================================
# 品質検証
# ============================================================

def _report_reprojection(label, object_points, image_points, K, dist, rvec, tvec):
    """再投影誤差を計算して表示し、平均誤差[px]を返す。"""
    mean_px, per_point = reprojection_error(
        object_points, image_points, K, rvec, tvec, dist)

    if mean_px <= REPROJ_WARN_PX:
        verdict, mark = "良好", "OK"
    elif mean_px <= REPROJ_FAIL_PX:
        verdict, mark = "許容（精度は期待できない）", "WARN"
    else:
        verdict, mark = "やり直し推奨", "FAIL"

    print(f"  [{label}] 再投影誤差: 平均 {mean_px:.2f} px  [{mark}] {verdict}")
    worst = int(np.argmax(per_point))
    print(f"           点ごと: " +
          "  ".join(f"{i+1}:{e:.1f}" for i, e in enumerate(per_point)))

    if mean_px > REPROJ_FAIL_PX:
        print(f"           → 点{worst+1} が最も外れています ({per_point[worst]:.1f} px)")
        print("           → まずクリック順序と config の3D座標を疑ってください")
        print("             順序: 手前左 → 手前右 → 奥右 → 奥左 → 4番(奥左)の真上")
    return mean_px


def _verify_pair(K1, dist1, R1, tvec1, pts1,
                 K2, dist2, R2, tvec2, pts2, field_points):
    """
    両カメラでクリックした同じ5点を三角測量し、configの3D座標と比べる。

    再投影誤差は各カメラ内部の辻褄しか見ないため、K が間違っていても
    R,tvec が歪んで吸収してしまうことがある。こちらは2台の相対関係を
    検証するので、位置推定で実際に使う精度に直結する。
    """
    print("\n  ─── 2カメラ整合の検証（三角測量） ───")
    O1 = (-R1.T.dot(tvec1)).flatten()
    O2 = (-R2.T.dot(tvec2)).flatten()

    errors = []
    for i, (p1, p2) in enumerate(zip(pts1, pts2)):
        _, D1 = get_ray(p1[0], p1[1], K1, R1, tvec1, dist1)
        _, D2 = get_ray(p2[0], p2[1], K2, R2, tvec2, dist2)
        P, residual = intersect_rays(O1, D1, O2, D2)
        if P is None:
            print(f"    点{i+1}: レイが平行 → 計算不能")
            errors.append(float('inf'))
            continue

        truth = np.asarray(field_points[i], dtype=np.float64)
        err = float(np.linalg.norm(P - truth))
        errors.append(err)
        print(f"    点{i+1}: 復元 ({P[0]:+6.2f}, {P[1]:+6.2f}, {P[2]:+5.2f})  "
              f"正解 ({truth[0]:+6.2f}, {truth[1]:+6.2f}, {truth[2]:+5.2f})  "
              f"誤差 {err:.2f}m  残差 {residual:.2f}m")

    finite = [e for e in errors if np.isfinite(e)]
    if not finite:
        print("  [FAIL] 三角測量が成立しませんでした。カメラ配置を確認してください。")
        return float('inf')

    mean_err = float(np.mean(finite))
    print(f"\n  平均3D誤差: {mean_err:.2f} m", end="  ")
    if mean_err <= TRIANG_WARN_M:
        print("[OK] このまま本番に使えます。")
        print(f"       MAX_RESIDUAL_M を {max(1.0, mean_err * 3):.1f} 程度まで"
              "絞ると3Dゲートが効きます。")
    elif mean_err <= TRIANG_FAIL_M:
        print("[WARN] 動きますが精度は期待できません。")
        print("       5点目（高さの点）のクリック精度が全体を支配します。")
    else:
        print("[FAIL] やり直しを強く推奨します。")
        print("       - 内部パラメータを実測済みか")
        print("         (tools/calibrate_intrinsics.py)")
        print("       - クリック順序が両カメラで一致しているか")
        print("       - config の FIELD_POINTS が実際のフィールド寸法と合っているか")
    return mean_err


# ============================================================
# 5点の取得（PC画面でクリック）
# ============================================================

def _click_five_points(cam, cam_label: str):
    """PC画面で5点クリックし、サブピクセル補正した座標を返す。"""
    points_2d = []
    state = {"mouse": None}
    window_name = f"Calibration: {cam_label}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, DISP_W, DISP_H)
    cv2.moveWindow(window_name, 0, 0)

    def on_mouse(event, x, y, flags, param):
        state["mouse"] = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN and len(points_2d) < 5:
            points_2d.append([x, y])
            print(f"  [{cam_label}] {CALIB_POINT_LABELS[len(points_2d)-1]} "
                  f"→ ({x}, {y})")

    cv2.setMouseCallback(window_name, on_mouse)
    print(f"\n  【{cam_label}】 基準点5箇所をクリックしてください "
          f"(プリセット: {CALIB_PRESET})")
    for lbl in CALIB_POINT_LABELS:
        print(f"       {lbl}")
    print("       [BACKSPACE]=1つ戻す  [ENTER]=確定  [Q]=中断")

    last_frame = None
    while True:
        ret, frame = cam.cap.read()
        if not ret or frame is None:
            continue
        last_frame = frame
        disp = frame.copy()

        for i, p in enumerate(points_2d):
            cv2.circle(disp, (int(p[0]), int(p[1])), 6, (0, 0, 255), -1)
            cv2.putText(disp, str(i + 1), (int(p[0]) + 12, int(p[1]) - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        n = len(points_2d)
        if n < 5:
            cv2.putText(disp, f"CLICK -> {CALIB_POINT_LABELS[n]}", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        else:
            cv2.putText(disp, "5/5 DONE - press ENTER", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(disp, "[BACKSPACE] undo   [ENTER] confirm   [Q] abort",
                    (10, disp.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        _draw_loupe(disp, frame, state["mouse"])
        cv2.imshow(window_name, disp)

        key = cv2.waitKey(1) & 0xFF
        if key == 13 and n == 5:
            break
        elif key == 8 and points_2d:          # BACKSPACE
            points_2d.pop()
            print(f"  [{cam_label}] 1つ戻しました → {len(points_2d)}/5")
        elif key == ord('q'):
            cv2.destroyWindow(window_name)
            raise RuntimeError(f"[{cam_label}] キャリブレーションがキャンセルされました。")

    # ── クリック位置のサブピクセル補正 ────────────────────
    # 床のライン交点である1〜4番のみ。5番（棒の先端）はコーナーではないので触らない。
    gray = cv2.cvtColor(last_frame, cv2.COLOR_BGR2GRAY)
    refined = [_refine_click(gray, p) for p in points_2d[:4]]
    refined.append([float(points_2d[4][0]), float(points_2d[4][1])])

    cv2.destroyWindow(window_name)
    for _ in range(5):
        cv2.waitKey(1)
    return refined


def _dummy_calib(label, cam_pos_xyz):
    from core.geometry import approx_camera_matrix
    K = approx_camera_matrix(1280, 720, 78.0)
    cam_pos = np.array(cam_pos_xyz, dtype=np.float64).reshape(3, 1)
    R = np.eye(3, dtype=np.float64)
    tvec = -R @ cam_pos
    print(f"  [DUMMY] {label} ダミーキャリブ使用")
    return K, None, R, tvec


# ============================================================
# Camera2（RPi経由）
# ============================================================

def _rline(sock, buf=""):
    while "\n" not in buf:
        buf += sock.recv(4096).decode()
    line, rest = buf.split("\n", 1)
    return line, rest


def _connect_stream(w_default=1280, h_default=720, retries=5):
    """CALIBを経ずにSTREAMモードだけで接続（保存済みキャリブ利用時）"""
    for attempt in range(retries):
        try:
            sk = socket.socket()
            sk.settimeout(5.0)
            sk.connect((RPI_HOST, RPI_PORT))
            line, buf = _rline(sk)
            info = json.loads(line)
            sk.sendall(b"STREAM\n")
            sk.settimeout(1.0)

            real_cam2 = RemoteCamera(RPI_HOST, RPI_PORT, label="Camera2")
            real_cam2.sock, real_cam2.buf = sk, buf
            real_cam2.width  = info.get("width", w_default)
            real_cam2.height = info.get("height", h_default)
            print("  [Camera2] STREAM接続成功")
            return real_cam2
        except Exception as e:
            print(f"  [Camera2] 接続試行{attempt+1}/{retries}失敗: {e}")
            time.sleep(1.0)
    return None


def _request_rpi_points():
    """
    RPiにCALIBを投げ、ラズパイ画面でクリックされた5点を受け取る。

    Returns:
        (pts, width, height) — 失敗時 pts は None
    """
    print(f"\n  ─── Camera2 キャリブレーション (プリセット: {CALIB_PRESET}) ───")
    print("       ラズパイ画面でクリック:")
    for lbl in CALIB_POINT_LABELS:
        print(f"       {lbl}")

    pts, w2, h2 = None, 1280, 720
    sk = socket.socket()
    sk.settimeout(120.0)
    try:
        sk.connect((RPI_HOST, RPI_PORT))
        line, buf = _rline(sk)
        info = json.loads(line)
        w2, h2 = info["width"], info["height"]
        sk.sendall(b"CALIB\n")
        print("  [Camera2] CALIB送信 - クリック待ち...")
        line, _ = _rline(sk, buf)
        pts = json.loads(line)["calib_pts"]
        print(f"  [Camera2] {len(pts)}点受信: {pts}")
    except Exception as e:
        print(f"  [FAIL] CALIB失敗: {e}")
    finally:
        sk.close()

    return pts, w2, h2


# ============================================================
# カメラソース
# ============================================================
# 「5点をどうやって取るか」「ライブ映像をどうやって開くか」だけが
# USB と RPi で違う。その2点だけをここに閉じ込め、キャリブの手順そのもの
# (保存済みを使うか聞く → 内部パラメータを読む → 解く → 保存 → 接続) は
# 両カメラ・両経路で完全に同じコードを通す。
#
# ★ 順序が重要: RPi のサーバーは1接続1モードなので、CALIB で点を受けてから
#   STREAM で繋ぎ直す。USB は初期化時点で既に開いているので open_live は素通し。

class _UsbSource:
    """PCに直結したUSBカメラ。PC画面でクリックする。"""

    def __init__(self, cam):
        self.cam = cam

    def capture_points(self, label):
        print(f"\n  ─── {label} (USB) キャリブレーション ───")
        pts = _click_five_points(self.cam, label)
        return pts, self.cam.width, self.cam.height

    def open_live(self, width, height):
        return self.cam


class _RpiSource:
    """ラズパイ経由。CALIB接続で5点を受け取り、STREAM接続に張り直す。"""

    def capture_points(self, label):
        return _request_rpi_points()

    def open_live(self, width, height):
        # CALIB 側の接続をサーバーが畳むまでの猶予
        time.sleep(0.3)
        return _connect_stream(width, height)


# ============================================================
# 1台分のキャリブレーション
# ============================================================

def _check_resolution(label, cam, w, h):
    """外部パラメータを測った解像度と、いま開いている映像の解像度を突き合わせる。"""
    live_w, live_h = getattr(cam, "width", None), getattr(cam, "height", None)
    if not live_w or not live_h or (live_w, live_h) == (w, h):
        return
    print(f"  [{label}] [WARN] 外部パラメータは {w}x{h} で測られていますが、"
          f"いまの映像は {live_w}x{live_h} です。")
    print("             クリック座標の基準がずれるため、測り直してください。")


def _verify_saved(label, points, K, dist, R, tvec, saved_reproj):
    """
    保存済みの外部パラメータを、いまの内部パラメータで検算する。

    内部パラメータを測り直したあとは保存時の再投影誤差が当てにならない。
    ここで計算し直すことで、K の入れ替わりに気づかないまま飛ばすのを防ぐ。
    """
    rvec, _ = cv2.Rodrigues(R)
    reproj = _report_reprojection(label, FIELD_POINTS,
                                  np.array(points, dtype=np.float32),
                                  K, dist, rvec, tvec)
    if saved_reproj is not None and abs(reproj - saved_reproj) > 0.5:
        print(f"  [{label}] [WARN] 保存時 ({saved_reproj:.2f} px) と食い違っています。")
        print("             内部パラメータを測り直した可能性があります。"
              "誤差が悪化しているなら外部パラメータも測り直してください。")


def _calibrate_camera(label, source, available, fallback_pos, fallback_cam):
    """
    1台分のキャリブレーションを行い (K, dist, R, tvec, points, cam) を返す。

    保存済み利用 / 新規測定 / ダミー の3経路をここに集約する。
    points は2カメラ整合の検証に使う。取れなかった場合は None。
    """
    def dummy(reason=None):
        if reason:
            print(f"  [{label}] {reason}")
        K, dist, R, tvec = _dummy_calib(label, fallback_pos)
        return K, dist, R, tvec, None, fallback_cam

    if not available:
        return dummy()

    saved = load_calibration(label)
    if saved is not None and ask_use_saved(label, saved):
        points = saved.get("points")
        w = saved.get("width") or 1280
        h = saved.get("height") or 720
        # ★ K は保存データからではなく、必ず intrinsics_<label>.json から読み直す。
        K, dist = resolve_intrinsics(label, w, h)
        R, tvec = saved["R"], saved["tvec"]
        print(f"  [{label}] 保存済みの外部パラメータを使用")
        if points and len(points) == 5:
            _verify_saved(label, points, K, dist, R, tvec, saved.get("reproj_px"))
    else:
        points, w, h = source.capture_points(label)
        if not points or len(points) != 5:
            return dummy("[FAIL] 5点を取得できませんでした")

        K, dist = resolve_intrinsics(label, w, h)
        p2d = np.array(points, dtype=np.float32)
        R, tvec, rvec = solve_extrinsics(FIELD_POINTS, p2d, K, dist)
        if R is None:
            return dummy("[FAIL] solvePnP が失敗しました")

        cp = -R.T.dot(tvec)
        print(f"\n  [{label}] カメラ位置: X={cp[0,0]:.2f} "
              f"Y={cp[1,0]:.2f} Z={cp[2,0]:.2f}m")
        reproj = _report_reprojection(label, FIELD_POINTS, p2d, K, dist, rvec, tvec)
        # ★ ライブ接続より先に保存する。接続に失敗しても5点クリックは無駄にならない。
        save_calibration(label, R, tvec, points, w, h,
                         reproj_px=reproj, preset=CALIB_PRESET)

    cam = source.open_live(w, h)
    if cam is None:
        return dummy("[WARN] 映像の接続に失敗しました")

    # ★ 保存済みを使っても、静的フィルタは毎回ゼロから再学習する。
    #   これにより、前回のマスクが残ったまま「y でスキップ」してしまうのを防ぐ。
    cam.reset_background()
    print(f"  [{label}] 静的フィルタを初期化して再学習を開始")

    _check_resolution(label, cam, w, h)
    return K, dist, R, tvec, points, cam


# ============================================================
# フロー全体
# ============================================================

def run_calibration_phase(cam1, cam1_ok: bool, cam2_ok: bool, cam2_stub):
    """
    Camera1・Camera2両方のキャリブレーションを実施し、
    (calib1, calib2, cam2) を返す。calib は CameraCalib。
    """
    print("\n[INIT 3/3]  フィールドキャリブレーション")
    print(f"  使用プリセット: {CALIB_PRESET}")
    print(f"  基準点の3D座標:\n{FIELD_POINTS}")

    K1, dist1, R1, tvec1, pts1, cam1 = _calibrate_camera(
        "Camera1", _UsbSource(cam1), cam1_ok, [-8, -8, 2], cam1)

    source2 = _UsbSource(cam2_stub) if CAMERA2_SOURCE == "USB" else _RpiSource()
    K2, dist2, R2, tvec2, pts2, cam2 = _calibrate_camera(
        "Camera2", source2, cam2_ok, [8, -8, 2], cam2_stub)

    # ── 2カメラ整合の検証 ────────────────────────────────────
    if pts1 and pts2 and len(pts1) == 5 and len(pts2) == 5:
        _verify_pair(K1, dist1, R1, tvec1, pts1,
                     K2, dist2, R2, tvec2, pts2, FIELD_POINTS)
    else:
        print("\n  [WARN] 両カメラのクリック点が揃っていないため、"
              "2カメラ整合の検証をスキップしました。")

    # ── 露出の固定 ───────────────────────────────────────────
    # ここまでは自動露出で会場の明るさに合わせ、以降は固定して変動させない。
    # RPi 経由の Camera2 は STREAM 開始時に自分で固定するため、ここでは何もしない
    # (lock_exposure を持たないので自然に飛ばされる)。
    if LOCK_EXPOSURE_AFTER_CALIB:
        print()
        for cam in (cam1, cam2):
            if hasattr(cam, "lock_exposure"):
                cam.lock_exposure()

    calib1 = CameraCalib(K=K1, dist=dist1, R=R1, tvec=tvec1)
    calib2 = CameraCalib(K=K2, dist=dist2, R=R2, tvec=tvec2)
    return calib1, calib2, cam2
