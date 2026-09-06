"""
core/camera.py
ローカルUSBカメラの制御と動体検知。

検知は「候補を1個に絞って返す」のではなく「候補リストを返す」設計。
どれが機体かは1枚の画像では決められない（42m先だと機体も人も20px程度で
大きさも形も区別がつかない）ため、選択は2カメラの幾何整合に任せる。
"""

import time
import threading
from typing import NamedTuple

import cv2

from core.geometry import approx_camera_matrix
from utils.config import (DIFF_THRESHOLD, MIN_AREA_PX, MAX_AREA_PX,
                          BLUR_KERNEL, MORPH_KERNEL, CAMERA_FPS,
                          MAX_CANDIDATES, USE_BG_SUBTRACTOR,
                          BG_HISTORY, BG_VAR_THRESHOLD, BG_LEARNING_RATE,
                          VIBRATION_REJECT_RATIO,
                          CAMERA_AUTOFOCUS, CAMERA_FOCUS_VALUE,
                          USE_MEASURED_INTRINSICS,
                          FALLBACK_HFOV_DEG, FALLBACK_HFOV_DEFAULT)


class Candidate(NamedTuple):
    """1フレーム内の動体候補。どれが機体かはこの時点では未確定。"""
    u: float
    v: float
    area: float
    x: int
    y: int
    w: int
    h: int


# DirectShow の自動露出プロパティは 0.75=自動 / 0.25=手動 という慣習
DSHOW_EXPOSURE_AUTO   = 0.75
DSHOW_EXPOSURE_MANUAL = 0.25


class CameraTracker:
    def __init__(self, camera_url, width=1280, height=720, label="Camera"):
        self.label = label
        self.camera_url = camera_url

        if isinstance(camera_url, int):
            self.cap = self._open_local_camera(camera_url)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            self.cap = cv2.VideoCapture(camera_url)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"[{label}] 初期化完了: 要求 {width}x{height}@{CAMERA_FPS}fps "
              f"-> 実際 {int(actual_w)}x{int(actual_h)}@{actual_fps:.1f}fps")

        self.width  = int(actual_w) if actual_w > 0 else width
        self.height = int(actual_h) if actual_h > 0 else height

        self._apply_focus_settings()

        # ── 検知パラメータ（detection_params.json 由来） ────────
        self.blur_size      = (BLUR_KERNEL, BLUR_KERNEL)
        self.diff_threshold = DIFF_THRESHOLD
        self.min_area       = MIN_AREA_PX
        self.max_area       = MAX_AREA_PX
        self._morph_kernel  = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (MORPH_KERNEL, MORPH_KERNEL)
        )

        self.prev_gray = None
        self._bg = None
        if USE_BG_SUBTRACTOR:
            self._bg = self._make_bg_subtractor()

        self.exposure_locked = False
        self.last_frame_time = 0.0
        self.last_candidates = []
        self.vibration_rejected = False
        self._latest_lock = threading.Lock()
        self._latest_frame = None
        self._latest_frame_time = 0.0
        self._reader_stop = threading.Event()
        self._reader_thread = None
        self._latest_reader_started = False
        self._perf = {
            "reader_reads": 0,
            "reader_errors": 0,
            "reader_read_ms_total": 0.0,
            "reader_last_ms": 0.0,
            "reader_start_time": 0.0,
            "reader_last_frame_time": 0.0,
            "reader_seq": 0,
            "process_count": 0,
            "process_ms_total": 0.0,
            "process_last_ms": 0.0,
            "process_last_copy_ms": 0.0,
            "process_last_age_ms": 0.0,
            "process_seq": 0,
        }

    def _open_local_camera(self, requested_index):
        """Open a Windows camera, tolerating backend and index differences."""
        attempts = []
        for index in [requested_index, 0, 1, 2, 3]:
            if index not in [item[0] for item in attempts]:
                attempts.append((index, cv2.CAP_DSHOW))
                attempts.append((index, cv2.CAP_MSMF))

        for index, backend in attempts:
            cap = cv2.VideoCapture(index, backend)
            if cap.isOpened():
                backend_name = "DSHOW" if backend == cv2.CAP_DSHOW else "MSMF"
                if index != requested_index or backend != cv2.CAP_DSHOW:
                    print(f"  [{self.label}] 接続方法をフォールバック: "
                          f"カメラ{index} / {backend_name}")
                return cap
            cap.release()

        return cv2.VideoCapture(requested_index, cv2.CAP_ANY)

    # ------------------------------------------------------------------
    # カメラ設定
    # ------------------------------------------------------------------
    def _apply_focus_settings(self):
        """
        オートフォーカスを切って無限遠に固定する。

        ★ ノイズ対策ではなく幾何精度の話。フォーカスが動くと焦点距離が
          変わるため、チェッカーボードで測った K が意味を失う。
          対象は15〜60m先なので無限遠固定で問題ない。
        """
        if CAMERA_AUTOFOCUS:
            print(f"  [{self.label}] [WARN] オートフォーカスが有効のままです。"
                  "内部パラメータが飛行中に変化します。")
            return
        try:
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            self.cap.set(cv2.CAP_PROP_FOCUS, CAMERA_FOCUS_VALUE)
            af = self.cap.get(cv2.CAP_PROP_AUTOFOCUS)
            if af and af > 0:
                print(f"  [{self.label}] [WARN] オートフォーカスを無効化できませんでした。"
                      "カメラ本体側で固定してください。")
            else:
                print(f"  [{self.label}] オートフォーカス無効・無限遠固定")
        except cv2.error as e:
            print(f"  [{self.label}] [WARN] フォーカス設定に失敗: {e}")

    def lock_exposure(self):
        """
        現在の自動露出・自動WBの結果を読み取り、その値で固定する。

        キャリブレーション完了時点で呼ぶ想定。会場の明るさには自動で
        合わせたうえで、競技中は変動させない。当日の手作業はゼロ。
        """
        if not isinstance(self.camera_url, int):
            return False
        try:
            exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
            wb       = self.cap.get(cv2.CAP_PROP_WB_TEMPERATURE)

            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, DSHOW_EXPOSURE_MANUAL)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            if wb and wb > 0:
                self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)

            mode = self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
            ok = abs(mode - DSHOW_EXPOSURE_MANUAL) < 0.1
            if ok:
                print(f"  [{self.label}] 露出を固定 (exposure={exposure:.1f}, wb={wb:.0f})")
            else:
                print(f"  [{self.label}] [WARN] 露出の固定に失敗しました。"
                      "自動のまま続行します（振動フレーム破棄で吸収されます）。")
            self.exposure_locked = ok
            return ok
        except cv2.error as e:
            print(f"  [{self.label}] [WARN] 露出固定に失敗: {e}")
            return False

    def get_intrinsics(self):
        """
        内部パラメータ (K, dist) を返す。

        事前実測値（calib/intrinsics_<label>.json）があればそれを使い、
        無ければ公称画角からの概算にフォールバックする。
        """
        if USE_MEASURED_INTRINSICS:
            from utils.calib_store import load_intrinsics
            m = load_intrinsics(self.label, self.width, self.height)
            if m is not None:
                return m["K"], m["dist"]

        hfov = FALLBACK_HFOV_DEG.get(self.label, FALLBACK_HFOV_DEFAULT)
        print(f"  [{self.label}] [WARN] 実測の内部パラメータがありません。"
              f"公称画角 {hfov:.1f}° から概算します。")
        print(f"             tools/calibrate_intrinsics.py --label {self.label} "
              "で事前に実測してください。")
        return approx_camera_matrix(self.width, self.height, hfov), None

    def get_approx_camera_matrix(self):
        """後方互換。新しいコードは get_intrinsics() を使うこと。"""
        return self.get_intrinsics()[0]

    # ------------------------------------------------------------------
    # 検知
    # ------------------------------------------------------------------
    def _make_bg_subtractor(self):
        """
        MOG2 背景差分器を作る。

        前フレーム差分ではなく背景差分にする理由:
          ・前フレーム差分は移動前と移動後の両方が光るため（ゴースト）、
            重心が実際の機体より後ろにずれ、しかもずれ量が速度で変わる。
          ・背景差分ならその歪みが出ない。
        学習率はほぼ0に設定する（config側）。ドローンはホバリングするので
        学習率が高いと静止した機体が背景に吸収されて消えてしまう。
        """
        return cv2.createBackgroundSubtractorMOG2(
            history=BG_HISTORY,
            varThreshold=BG_VAR_THRESHOLD,
            detectShadows=False,
        )

    def _foreground_mask(self, gray_blurred):
        """前景マスクを作る。背景差分器が無効なら前フレーム差分にフォールバック。"""
        if self._bg is not None:
            mask = self._bg.apply(gray_blurred, learningRate=BG_LEARNING_RATE)
            _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
            return mask

        if self.prev_gray is None:
            self.prev_gray = gray_blurred
            return None
        diff = cv2.absdiff(self.prev_gray, gray_blurred)
        _, mask = cv2.threshold(diff, self.diff_threshold, 255, cv2.THRESH_BINARY)
        self.prev_gray = gray_blurred
        return mask

    def detect(self, frame):
        """
        1フレームから動体候補のリストを返す（面積の大きい順、最大 MAX_CANDIDATES 個）。

        ここでは「どれが機体か」を決めない。1枚の画像からは決められないため。
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, self.blur_size, 0)

        mask = self._foreground_mask(gray_blurred)
        self.vibration_rejected = False
        if mask is None:
            return []

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._morph_kernel)

        # ── 振動・照明急変フレームの破棄 ───────────────────
        # 三脚が揺れる／自動露出が追従する／照明が変わると画面全体が
        # 前景になる。そのフレームは検知結果を丸ごと捨てる。
        fg_ratio = cv2.countNonZero(mask) / float(mask.size)
        if fg_ratio > VIBRATION_REJECT_RATIO:
            self.vibration_rejected = True
            return []

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        candidates = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area or area > self.max_area:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cu = M["m10"] / M["m00"]
            cv_ = M["m01"] / M["m00"]
            x, y, w, h = cv2.boundingRect(c)
            candidates.append(Candidate(cu, cv_, float(area), x, y, w, h))

        candidates.sort(key=lambda c: -c.area)
        return candidates[:MAX_CANDIDATES]

    def draw_candidates(self, frame, candidates, best_index=None):
        """候補を重ねて描く。採用された候補だけ強調する。"""
        for i, c in enumerate(candidates):
            is_best = (best_index is not None and i == best_index)
            color = (0, 255, 0) if is_best else (110, 110, 110)
            thickness = 2 if is_best else 1
            cv2.rectangle(frame, (c.x, c.y), (c.x + c.w, c.y + c.h),
                          color, thickness)
            if is_best:
                cv2.circle(frame, (int(c.u), int(c.v)), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"({int(c.u)},{int(c.v)})",
                            (int(c.u) + 10, int(c.v) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.putText(frame, self.label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 0), 2)
        if self.vibration_rejected:
            cv2.putText(frame, "FRAME REJECTED (vibration/lighting)", (10, 66),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2)

    def read_and_detect(self):
        """
        フレームを読んで候補リストを返す。

        Returns:
            (frame, candidates, timestamp) — 取得失敗時は (None, [], 0.0)
        """
        process_start = time.perf_counter()
        if self._latest_reader_started:
            copy_start = time.perf_counter()
            with self._latest_lock:
                frame = None if self._latest_frame is None else self._latest_frame.copy()
                ts = self._latest_frame_time
                seq = self._perf["reader_seq"]
            copy_ms = (time.perf_counter() - copy_start) * 1000.0
            if frame is None:
                return None, [], 0.0
        else:
            ret, frame = self.cap.read()
            ts = time.time()
            seq = 0
            copy_ms = 0.0
            if not ret or frame is None:
                return None, [], 0.0

        candidates = self.detect(frame)
        process_ms = (time.perf_counter() - process_start) * 1000.0
        with self._latest_lock:
            self._perf["process_count"] += 1
            self._perf["process_ms_total"] += process_ms
            self._perf["process_last_ms"] = process_ms
            self._perf["process_last_copy_ms"] = copy_ms
            self._perf["process_last_age_ms"] = max(0.0, (time.time() - ts) * 1000.0)
            self._perf["process_seq"] = seq
        self.last_frame_time = ts
        self.last_candidates = candidates
        return frame, candidates, ts

    def read_and_track(self):
        """
        後方互換API。候補のうち最大面積のものを1点だけ返す。

        ※ Phase B で tracker 側を read_and_detect() に切り替えたら削除する。
        """
        frame, candidates, _ = self.read_and_detect()
        if frame is None:
            return None, None

        best_index = 0 if candidates else None
        self.draw_candidates(frame, candidates, best_index)
        center_uv = (candidates[0].u, candidates[0].v) if candidates else None
        return frame, center_uv

    def reset_background(self):
        self.prev_gray = None
        if self._bg is not None:
            self._bg = self._make_bg_subtractor()

    def start_latest_reader(self):
        """Continuously capture frames so processing always uses the newest one."""
        if self._latest_reader_started or not isinstance(self.camera_url, int):
            return
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(
            target=self._latest_reader_loop,
            name=f"{self.label}-capture",
            daemon=True,
        )
        self._latest_reader_started = True
        self._reader_thread.start()

    def _latest_reader_loop(self):
        with self._latest_lock:
            self._perf["reader_start_time"] = time.time()
        while not self._reader_stop.is_set():
            read_start = time.perf_counter()
            ret, frame = self.cap.read()
            read_ms = (time.perf_counter() - read_start) * 1000.0
            if not ret or frame is None:
                with self._latest_lock:
                    self._perf["reader_errors"] += 1
                time.sleep(0.005)
                continue
            with self._latest_lock:
                self._latest_frame = frame
                self._latest_frame_time = time.time()
                self._perf["reader_reads"] += 1
                self._perf["reader_read_ms_total"] += read_ms
                self._perf["reader_last_ms"] = read_ms
                self._perf["reader_last_frame_time"] = self._latest_frame_time
                self._perf["reader_seq"] += 1

    def get_performance_stats(self):
        with self._latest_lock:
            stats = dict(self._perf)
        elapsed = max(time.time() - stats["reader_start_time"], 1e-6)
        stats["reader_fps"] = stats["reader_reads"] / elapsed
        stats["reader_avg_ms"] = (
            stats["reader_read_ms_total"] / stats["reader_reads"]
            if stats["reader_reads"] else 0.0
        )
        return stats

    def release(self):
        self._reader_stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
        self.cap.release()
