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
import numpy as np

from utils.config import (TRACKING_EXPOSURE, CAMERA_FPS,
                          CAMERA_AUTOFOCUS, CAMERA_FOCUS_VALUE,
                          DETECTION_CAMERA_OVERRIDES, detection_params_for)


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


class StaticBrightMask:
    """
    「ずっと明るいまま動かない画素」を覚えて bright マスクから引く。

    窓・白い反射・照明は時間的に不変なので、学習フレームの大半で明るかった
    画素を構造物として記録し、以降の検知から除外する。機体のLEDは
    ホバリング中でも数px揺れるため、比率しきい値で残る。

    ★ 学習中は機体を画角に入れないこと（または消灯しておくこと）。
      静止した機体が写っていると、そのLEDごと構造物として覚えてしまう。
      照明が変わったときや覚え間違えたときは [B] キーで再学習できる。
    """

    def __init__(self, label: str, learn_frames: int, ratio: float, dilate_px: int):
        self.label = label
        self.learn_frames = learn_frames
        self.ratio = ratio
        self.dilate_px = dilate_px
        self._acc = None
        self._frames = 0
        self.mask = None      # 学習完了後の除外マスク (255=除外)

    @property
    def learned_frames(self) -> int:
        return self._frames

    def apply(self, bright_mask):
        """学習を進めつつ、学習済みなら静的画素を除いたマスクを返す。"""
        if self._frames < self.learn_frames:
            self._learn(bright_mask)
            return bright_mask
        if self.mask is None:
            return bright_mask
        return cv2.bitwise_and(bright_mask, cv2.bitwise_not(self.mask))

    def _learn(self, bright_mask):
        if self._acc is None:
            self._acc = np.zeros(bright_mask.shape, dtype=np.uint16)
        self._acc += (bright_mask > 0)
        self._frames += 1
        if self._frames < self.learn_frames:
            return

        hits = int(self.learn_frames * self.ratio)
        mask = ((self._acc >= hits) * 255).astype(np.uint8)
        if self.dilate_px > 0:
            k = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.dilate_px, self.dilate_px))
            mask = cv2.dilate(mask, k)
        self.mask = mask
        self._acc = None

        coverage = cv2.countNonZero(mask) / float(mask.size)
        print(f"  [{self.label}] 静的輝点マスクを学習完了 "
              f"({self.learn_frames}フレーム, 画面の{coverage * 100:.1f}%を除外)")
        if coverage > 0.20:
            print(f"  [{self.label}] [WARN] 除外領域が広すぎます。露出が明るすぎるか、"
                  "学習中に機体が写っていた可能性があります。[B]キーで再学習してください。")

    def reset(self):
        self._acc = None
        self._frames = 0
        self.mask = None


class FlickerMask:
    """
    「今、直近より明るくなった所」のマスク (detect_mode "flicker")。点滅 LED の点灯を拾う。

    ★ 2026-09-17 09:36 Camera1 (α6400 が 1/15s・+0.3EV の自動露出に戻っていた) で、
      bright 系は会場が明るいと成り立たなかった: しきい値 140 を床の反射や
      照明が超え、静的輝点マスクが画面の 43% を覆い、残った雑音で候補枠
      (max_candidates) が毎フレーム埋まって機体の LED が候補に入らなかった。
      明るさの絶対値ではなく「点滅して明るさが変わる所」を拾えば露出に依らない。

    画素ごとに2つの条件を両方満たす所を前景にする:
      1) 今の輝度 − 直近 window_sec 秒の最小 > threshold
         窓は LED の 1 周期 (167ms) より長く取り、消灯を必ず含める。
         枚数ではなく時間で持つのは、Camera1 の実効 fps が露出で 10〜30 と変わるため。
      2) 今の輝度 − ゆっくり追う背景 (指数移動平均, bg_alpha/フレーム) > bg_threshold
         窓の最小だけだと、白飛びした窓の前を人や物が一瞬横切って戻った所も
         「最小より明るい」になる。背景より明るいことも要求して、元から明るい所を落とす。

    ★ 2026-09-17 10:08 (0.8m/s 以下で動き続ける機体): 最初は「窓内の最大 − 最小」で
      拾っていたが、動く LED では 0.4 秒ぶんの軌跡が1つの細長い塊になり、重心が
      LED より 20px 以上遅れて点滅判定の ROI が LED を外した (点滅確定 0.4%)。
      「今」明るい所だけにすると塊が今の LED の位置に来る。消灯中のフレームでは
      候補が出ないが、core/blink.py のトラックが速度で位置を予測してつなぐ。
    止まっている照明・反射・OSD は変化が無いので出てこない。動く人・手も拾うが、
    それは core/blink.py の点滅判定が落とす。
    """

    def __init__(self, window_sec: float, threshold: int, bg_alpha: float, bg_threshold: int):
        self.window_sec = window_sec
        self.threshold = threshold
        self.bg_alpha = bg_alpha
        self.bg_threshold = bg_threshold
        self._frames = []      # (ts, gray)
        self._bg = None        # float32 の指数移動平均

    def apply(self, gray, ts):
        """窓が埋まるまでは None。以降は点灯した画素を 255 にしたマスク。"""
        self._frames.append((ts, gray))
        while self._frames and ts - self._frames[0][0] > self.window_sec:
            self._frames.pop(0)
        if self._bg is None:
            self._bg = gray.astype(np.float32)
        above_bg = cv2.subtract(gray, cv2.convertScaleAbs(self._bg))
        cv2.accumulateWeighted(gray, self._bg, self.bg_alpha)
        if len(self._frames) < 3 or ts - self._frames[0][0] < 0.5 * self.window_sec:
            return None

        lo = self._frames[0][1].copy()
        for _, g in self._frames[1:]:
            cv2.min(lo, g, dst=lo)
        _, mask = cv2.threshold(cv2.subtract(gray, lo), self.threshold, 255, cv2.THRESH_BINARY)
        _, above = cv2.threshold(above_bg, self.bg_threshold, 255, cv2.THRESH_BINARY)
        return cv2.bitwise_and(mask, above)

    def reset(self):
        self._frames = []
        self._bg = None


class CameraTracker:
    """
    検知パラメータは detection_params.json の共通値に、そのカメラ用の
    "camera_overrides" を重ねたもの (config.detection_params_for)。
    Camera1 (α6400) と Camera2 (RPi) はレンズ・露出・LED の写る大きさが
    違うので、係数は揃えない。

    camera_url=None なら映像デバイスを開かない。録画を detect() に流す
    オフライン再生 (tools/replay_detect.py, tests/) 用。
    """

    def __init__(self, camera_url, width=1280, height=720, label="Camera"):
        self.label = label
        self.camera_url = camera_url
        self.p = detection_params_for(label)

        if camera_url is None:
            self.cap = None
            actual_w, actual_h = width, height
        elif isinstance(camera_url, int):
            self.cap = self._open_local_camera(camera_url, width, height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            fourcc_s = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4))
            if fourcc_s != "MJPG":
                print(f"  [{label}] [WARN] 転送形式が {fourcc_s!r} です (MJPG ではない)。"
                      "非圧縮 YUY2 だと USB2.0 の帯域で 1280x720 は 10fps に落ち、"
                      "露出も伸びて残像が出ます。")
        else:
            self.cap = cv2.VideoCapture(camera_url)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if self.cap is not None:
            actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            print(f"[{label}] 初期化完了: 要求 {width}x{height}@{CAMERA_FPS}fps "
                  f"-> 実際 {int(actual_w)}x{int(actual_h)}@{actual_fps:.1f}fps")

        self.width  = int(actual_w) if actual_w > 0 else width
        self.height = int(actual_h) if actual_h > 0 else height

        if self.cap is not None:
            self._apply_focus_settings()

        # ── 検知パラメータ（detection_params.json 由来、カメラ別上書き込み） ──
        p = self.p
        # カーネル 1 以下は「かけない」。遠方の LED は 3px 角しか無く、
        # 5x5 のぼかしでピークが 7 割に落ち、5x5 のオープニングで消える。
        self.blur_size      = ((p["blur_kernel"], p["blur_kernel"])
                               if p["blur_kernel"] > 1 else None)
        self.diff_threshold = p["diff_threshold"]
        self.min_area       = p["min_area_px"]
        self.max_area       = p["max_area_px"]
        self._morph_kernel  = (cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (p["morph_kernel"], p["morph_kernel"]))
            if p["morph_kernel"] > 1 else None)
        overrides = {k: v for k, v in DETECTION_CAMERA_OVERRIDES.get(label, {}).items()
                     if not k.startswith("_")}
        if overrides:
            print(f"  [{label}] 検知パラメータのカメラ別上書き: "
                  + ", ".join(f"{k}={v}" for k, v in sorted(overrides.items())))

        self.prev_gray = None
        self._bg = None
        if p["use_background_subtractor"]:
            self._bg = self._make_bg_subtractor()

        uses_bright = p["detect_mode"] in ("bright", "bright_or_motion")
        self._static_bright = (StaticBrightMask(label, p["static_mask_learn_frames"],
                                                p["static_mask_ratio"],
                                                p["static_mask_dilate_px"])
                               if p["static_bright_mask"] and uses_bright else None)
        self._flicker = (FlickerMask(p["flicker_window_sec"], p["flicker_threshold"],
                                     p["flicker_bg_alpha"], p["flicker_bg_threshold"])
                         if p["detect_mode"] == "flicker" else None)

        self.exposure_locked = False
        self.last_frame_time = 0.0
        self.last_candidates = []
        self._last_detect_seq = None
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

    def _open_local_camera(self, requested_index, width, height):
        """
        Windows のカメラを開く。バックエンドや番号の違いには寛容にする。

        ★ DirectShow では形式・解像度・FPS を「開くときのパラメータ」で渡すこと。
          開いた後に set(FOURCC=MJPG) しても OpenCV 5.0 + C930e では YUY2 のまま
          残り、1280x720 が 10fps (1枚 100ms) になっていた (2026-09-15 実測)。
          パラメータ渡しなら MJPG 30fps (C930e の上限) / C920系 60fps が出る。
          MSMF はパラメータ渡しだと開けないので、開いた後に set する。
        """
        open_params = [cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"),
                       cv2.CAP_PROP_FRAME_WIDTH, width,
                       cv2.CAP_PROP_FRAME_HEIGHT, height,
                       cv2.CAP_PROP_FPS, CAMERA_FPS]
        attempts = []
        for index in [requested_index, 0, 1, 2, 3]:
            if index not in [item[0] for item in attempts]:
                attempts.append((index, cv2.CAP_DSHOW))
                attempts.append((index, cv2.CAP_MSMF))

        for index, backend in attempts:
            if backend == cv2.CAP_DSHOW:
                cap = cv2.VideoCapture(index, backend, open_params)
            else:
                cap = cv2.VideoCapture(index, backend)
                for prop, value in zip(open_params[::2], open_params[1::2]):
                    cap.set(prop, value)
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

            # bright 検知モード用: 自動が選んだ値ではなく、明示した暗い値で固定する。
            #  「画面の中でLEDだけが白飛びしている」状態を作るのが狙い。
            if TRACKING_EXPOSURE is not None:
                exposure = float(TRACKING_EXPOSURE)
                print(f"  [{self.label}] 追跡用に露出を {exposure:.1f} へ落とします "
                      f"(TRACKING_EXPOSURE)")

            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, DSHOW_EXPOSURE_MANUAL)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            if wb and wb > 0:
                self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)

            # ★ AUTO_EXPOSURE の読み戻しは OpenCV 5 + DSHOW だと設定に関係なく
            #   -1 が返り、実際には固定できているのに毎回「失敗」と出ていた
            #   (2026-09-15 Camera1 実測: -6 を設定すると 16fps -> 30fps に上がり
            #   EXPOSURE も -6 で読み戻せる)。露出値そのものの読み戻しで判定する。
            ok = abs(self.cap.get(cv2.CAP_PROP_EXPOSURE) - exposure) < 0.5
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

    # 追跡中の露出の見張り。2026-09-16 00:17 の便で、前 2 便は 30fps だった
    #  Camera1 が 15fps に戻っていた (lock_exposure 後にドライバ側で自動露出へ
    #  戻ったか、設定が乗らなかった)。直近の read() 1 枚の所要時間が
    #  長いままなら露出を設定し直す。露出値が読み戻せない場合は触らない。
    def recheck_exposure(self, read_ms):
        if TRACKING_EXPOSURE is None or not isinstance(self.camera_url, int):
            return
        if read_ms < 45.0:          # 30fps なら ~33ms、16fps だと ~62ms
            return
        try:
            cur = self.cap.get(cv2.CAP_PROP_EXPOSURE)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, DSHOW_EXPOSURE_MANUAL)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, float(TRACKING_EXPOSURE))
            print(f"  [{self.label}] [WARN] read() が {read_ms:.0f}ms/枚 "
                  f"(露出 {cur:.1f})。露出を {float(TRACKING_EXPOSURE):.1f} へ再設定しました")
        except cv2.error as e:
            print(f"  [{self.label}] [WARN] 露出の再設定に失敗: {e}")

    def get_intrinsics(self):
        """
        内部パラメータ (K, dist) を返す。

        事前実測値（calib/intrinsics_<label>.json）があればそれを使い、
        無ければ公称画角からの概算にフォールバックする。
        """
        from utils.calib_store import resolve_intrinsics
        return resolve_intrinsics(self.label, self.width, self.height)

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
            history=self.p["bg_history"],
            varThreshold=self.p["bg_var_threshold"],
            detectShadows=False,
        )

    def _foreground_mask(self, gray_blurred):
        """前景マスクを作る。背景差分器が無効なら前フレーム差分にフォールバック。"""
        if self._bg is not None:
            mask = self._bg.apply(gray_blurred, learningRate=self.p["bg_learning_rate"])
            _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
            return mask

        if self.prev_gray is None:
            self.prev_gray = gray_blurred
            return None
        diff = cv2.absdiff(self.prev_gray, gray_blurred)
        _, mask = cv2.threshold(diff, self.diff_threshold, 255, cv2.THRESH_BINARY)
        self.prev_gray = gray_blurred
        return mask

    def _bright_mask(self, gray_blurred):
        """
        輝度しきい値だけで前景を作る。機体に明るいLEDを載せている場合用。

        背景差分と違って「動いたか」を一切見ないので、**ホバリングで完全に
        静止していても消えない**。これが motion モードとの決定的な差。
        露出を絞って「画面の中でLEDだけが白飛びしている」状態にして使うこと。

        窓や白い反射も同じように光るため、StaticBrightMask で
        「ずっと明るいまま動かない画素」を差し引く。
        """
        _, mask = cv2.threshold(gray_blurred, self.p["bright_threshold"], 255,
                                cv2.THRESH_BINARY)
        if self._static_bright is not None:
            mask = self._static_bright.apply(mask)
        return mask

    def detect(self, frame, ts=None):
        """
        1フレームから候補のリストを返す（面積の大きい順、最大 MAX_CANDIDATES 個）。

        ここでは「どれが機体か」を決めない。1枚の画像からは決められないため。

        検知方式は DETECT_MODE (detection_params.json の detect_mode):
          motion           背景差分。従来動作
          bright           輝度しきい値のみ。静止ホバリングでも消えない
          bright_or_motion 両方の論理和。点滅LEDの消灯フレームを motion 側が埋める
          flicker          直近の輝度の振れ幅 (FlickerMask)。露出・会場の明るさに依らない

        ts はフレームの取得時刻。flicker の窓に使う (None なら今の時刻)。
        """
        p = self.p
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blurred = (cv2.GaussianBlur(gray, self.blur_size, 0)
                        if self.blur_size is not None else gray)

        self.vibration_rejected = False
        min_area, max_area = self.min_area, self.max_area

        mode = p["detect_mode"]
        if mode == "bright":
            mask = self._bright_mask(gray_blurred)
            min_area, max_area = p["bright_min_area_px"], p["bright_max_area_px"]
        elif mode == "bright_or_motion":
            bright = self._bright_mask(gray_blurred)
            motion = self._foreground_mask(gray_blurred)
            mask = bright if motion is None else cv2.bitwise_or(bright, motion)
            # 下限はLED側に合わせる (LEDは小さい)。上限は動体側の広いほうを使う。
            min_area = p["bright_min_area_px"]
        elif mode == "flicker":
            mask = self._flicker.apply(gray_blurred, time.time() if ts is None else ts)
            min_area, max_area = p["bright_min_area_px"], p["bright_max_area_px"]
        else:
            mask = self._foreground_mask(gray_blurred)
        if mask is None:
            return []

        if self._morph_kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._morph_kernel)

        # ── 振動・照明急変フレームの破棄 ───────────────────
        # 三脚が揺れる／自動露出が追従する／照明が変わると画面全体が
        # 前景になる。そのフレームは検知結果を丸ごと捨てる。
        fg_ratio = cv2.countNonZero(mask) / float(mask.size)
        if fg_ratio > p["vibration_reject_ratio"]:
            self.vibration_rejected = True
            return []

        # ★ 面積は「画素数」で数える (connectedComponents)。findContours の
        #   contourArea は輪郭の内側の面積なので、2x2 の光点で 1、3x3 で 4 と
        #   実際の画素数の半分以下になり、遠方の LED が min_area を割っていた。
        n, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        candidates = []
        for i in range(1, n):
            x, y, w, h, area = (int(v) for v in stats[i])
            if area < min_area or area > max_area:
                continue
            cu, cv_ = (float(v) for v in centroids[i])
            candidates.append(Candidate(cu, cv_, float(area), x, y, w, h))

        candidates.sort(key=lambda c: -c.area)
        return candidates[:p["max_candidates"]]

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
        elif self._static_bright is not None and self._static_bright.mask is None:
            # 学習中は機体を画角に入れてはいけないので、はっきり出す
            cv2.putText(frame,
                        f"LEARNING STATIC BRIGHT "
                        f"{self._static_bright.learned_frames}/{self._static_bright.learn_frames}"
                        " - keep drone out of view",
                        (10, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

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
            if self.cap is None:
                return None, [], 0.0
            ret, frame = self.cap.read()
            ts = time.time()
            seq = None
            copy_ms = 0.0
            if not ret or frame is None:
                return None, [], 0.0

        # ★ tracker のループはカメラより速い (Camera1 16fps に対し ~80Hz) ので、
        #   同じフレームが何度も来る。そのたびに detect() すると静的輝点マスクの
        #   学習が「呼び出し回数」で進み、120フレームのつもりが実時間 0.7 秒で
        #   終わる上、同じ1枚を何度も数えて比率が偏る。新しいフレームだけ検知する。
        if seq is not None and seq == self._last_detect_seq:
            candidates = self.last_candidates
        else:
            candidates = self.detect(frame, ts)
            self._last_detect_seq = seq
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

    def reset_background(self):
        self.prev_gray = None
        if self._bg is not None:
            self._bg = self._make_bg_subtractor()
        if self._static_bright is not None:
            self._static_bright.reset()
        if self._flicker is not None:
            self._flicker.reset()

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
        if self.cap is not None:
            self.cap.release()
