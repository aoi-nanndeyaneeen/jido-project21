"""
core/blink.py
LED点滅 (既知周波数) による候補の識別。ロックイン検波方式。

detect() の候補は「明るい/動いた」だけで、1フレームでは窓の反射や照明と
機体を区別できない。機体の白色LEDは flight_controller 側で 6Hz・50% duty で
点滅している (drone_s5.cpp)。そこで候補位置の周辺輝度を新フレームごとに
サンプリングし、その時系列のうち「LED_BLINK_HZ の成分」が占める割合を見る。

★ 候補の有無 (ON/OFF の2値) で立ち上がりを数える方式はうまくいかなかった:
  ・bright_or_motion では消灯フレームも motion 側が候補を出すので OFF にならない
  ・露出が長い (Camera1 は実測 100ms/フレーム) と点灯/消灯が1枚の中で
    平均され、輝度しきい値を常に超えて「ずっと点灯」に見える
  ・遷移をまたぐフレームで OFF 区間が 2〜3 フレームに縮み、デバウンスに潰される
  連続値の輝度に対するロックインなら、これらはすべて振幅が下がるだけで済む。

★ 実タイムスタンプで sin/cos と相関を取るので、フレームレートが 2×6Hz を
  下回っていても (例: 10fps) サンプルの位相が散っていれば検出できる。
  ただし露出時間が半周期 (83ms) に近づくと振幅が大きく減るため、
  追跡時の露出は短くすること (config.TRACKING_EXPOSURE)。

処理 (新しいフレームが来たときだけ):
  1) 候補を既存トラックに最近傍で割り当てる。割り当てのない候補は新規トラック。
  2) 全トラック (今フレームで候補が無い = 消灯中のものも含む) について、
     最後の位置まわり ROI の平均輝度を記録する。
  3) 直近 BLINK_HISTORY_SEC 秒のサンプルで
        score = 2(I²+Q²) / (N·Σx²)     x = 輝度 − 平均
     を計算する。全分散のうち目標周波数成分の割合 (0〜1)。純正弦波で1、
     50% duty の矩形波で約0.81、無相関ノイズで約 2/N。
  4) score と変調深さ (輝度の標準偏差) が閾値を超えたトラックだけを候補として返す。
     消灯中のフレームでも、確定済みトラックは最後の位置で候補を出し続ける。
"""

import math
from collections import deque

import cv2
import numpy as np


class _Track:
    __slots__ = ("cand", "last_seen_ts", "samples", "score", "depth", "confirmed")

    def __init__(self, cand, ts):
        self.cand = cand
        self.last_seen_ts = ts
        self.samples = deque()          # (ts, 平均輝度)
        self.score = 0.0
        self.depth = 0.0
        self.confirmed = False


class BlinkTracker:
    """1カメラぶんの候補列に、既知周波数の点滅によるフィルタをかける。"""

    MAX_TRACKS = 32

    def __init__(self, label, target_hz, match_dist_px=30.0, roi_px=8,
                 history_sec=1.2, min_score=0.5, min_depth=1.5, enabled=True):
        self.label = label
        self.enabled = enabled and target_hz > 0
        self.omega = 2.0 * math.pi * target_hz
        self.target_hz = target_hz
        self.match_dist_px = match_dist_px
        self.roi_px = roi_px
        self.history_sec = history_sec
        self.min_score = min_score
        self.min_depth = min_depth
        self._tracks = []
        self._last_ts = None
        self._last_output = []
        self._dts = []
        self._rate_warned = False

    # ------------------------------------------------------------------
    def filter(self, frame, candidates, ts, full_width=None):
        """
        Args:
            frame:      そのカメラの画像 (BGR)。縮小プレビューでもよい。
            candidates: detect() の候補 (座標はフル解像度基準)
            ts:         フレームの取得時刻
            full_width: 候補座標系の画像幅。frame が縮小されている場合のスケール計算用
        Returns:
            点滅が確認できたトラックの候補リスト
        """
        if not self.enabled:
            return candidates
        if frame is None or not ts:
            return []
        # tracker ループはカメラより速く回るので同じフレームが何度も来る。
        # 同じサンプルを重複して入れると統計が歪むので、前回の結果を返す。
        if ts == self._last_ts:
            return self._last_output
        if self._last_ts is not None:
            self._note_interval(ts - self._last_ts)
        self._last_ts = ts

        scale = frame.shape[1] / float(full_width) if full_width else 1.0

        self._associate(candidates, ts)
        for t in self._tracks:
            t.samples.append((ts, self._roi_mean(frame, t.cand, scale)))
            while t.samples and ts - t.samples[0][0] > self.history_sec:
                t.samples.popleft()
            self._update_score(t)

        self._last_output = [t.cand for t in self._tracks if t.confirmed]
        return self._last_output

    # ------------------------------------------------------------------
    def _associate(self, candidates, ts):
        pairs = []
        for ti, t in enumerate(self._tracks):
            for ci, c in enumerate(candidates):
                d = math.hypot(c.u - t.cand.u, c.v - t.cand.v)
                if d <= self.match_dist_px:
                    pairs.append((d, ti, ci))
        pairs.sort()
        used_t, used_c = set(), set()
        for _, ti, ci in pairs:
            if ti in used_t or ci in used_c:
                continue
            used_t.add(ti)
            used_c.add(ci)
            self._tracks[ti].cand = candidates[ci]
            self._tracks[ti].last_seen_ts = ts

        for ci, c in enumerate(candidates):
            if ci in used_c:
                continue
            # 既存トラックの近く (消灯中で割り当てが無かったもの等) には増やさない
            if any(math.hypot(c.u - t.cand.u, c.v - t.cand.v) <= self.match_dist_px
                   for t in self._tracks):
                continue
            if len(self._tracks) < self.MAX_TRACKS:
                self._tracks.append(_Track(c, ts))

        # 候補として一度も出てこなくなったトラックは捨てる。
        # 点滅中のLEDは半周期ごとに必ず候補になるので history_sec あれば十分。
        self._tracks = [t for t in self._tracks
                        if ts - t.last_seen_ts <= self.history_sec]

    def _roi_mean(self, frame, cand, scale):
        h, w = frame.shape[:2]
        r = max(2, int(round(self.roi_px * scale)))
        cu, cv_ = int(round(cand.u * scale)), int(round(cand.v * scale))
        x0, x1 = max(0, cu - r), min(w, cu + r + 1)
        y0, y1 = max(0, cv_ - r), min(h, cv_ + r + 1)
        if x0 >= x1 or y0 >= y1:
            return 0.0
        # 白色LEDなのでチャンネル平均でよい。最大値だと白飛び時に変調が消える。
        return float(frame[y0:y1, x0:x1].mean())

    def _update_score(self, t):
        n = len(t.samples)
        if n < 8 or t.samples[-1][0] - t.samples[0][0] < 0.75 * self.history_sec:
            t.score, t.depth, t.confirmed = 0.0, 0.0, False
            return
        arr = np.asarray(t.samples, dtype=np.float64)
        x = arr[:, 1] - arr[:, 1].mean()
        var_sum = float(np.dot(x, x))
        if var_sum <= 1e-9:
            t.score, t.depth, t.confirmed = 0.0, 0.0, False
            return
        phase = self.omega * (arr[:, 0] - arr[0, 0])
        i = float(np.dot(x, np.cos(phase)))
        q = float(np.dot(x, np.sin(phase)))
        t.score = min(1.0, 2.0 * (i * i + q * q) / (n * var_sum))
        t.depth = math.sqrt(var_sum / n)
        # ヒステリシス: 一瞬の遮蔽やブレでスコアが落ちても即座には外さない
        on_thr = self.min_score if not t.confirmed else self.min_score * 0.6
        t.confirmed = t.score >= on_thr and t.depth >= self.min_depth

    def _note_interval(self, dt):
        if self._rate_warned or dt <= 0:
            return
        self._dts.append(dt)
        if len(self._dts) < 60:
            return
        self._rate_warned = True
        med = sorted(self._dts)[len(self._dts) // 2]
        half_period = 0.5 / self.target_hz
        if med >= half_period:
            print(f"  [{self.label}] [WARN] フレーム間隔 {med * 1000:.0f}ms "
                  f"({1.0 / med:.1f}fps) が LED の半周期 {half_period * 1000:.0f}ms 以上です。"
                  "露出が長いと点灯/消灯が1枚の中で平均されて点滅が見えません。"
                  "TRACKING_EXPOSURE で露出を短くしてください。")

    # ------------------------------------------------------------------
    def draw(self, frame, full_width=None):
        """各トラックのスコアを描く。点滅確認済みは黄、未確認は灰。"""
        if not self.enabled or frame is None:
            return
        scale = frame.shape[1] / float(full_width) if full_width else 1.0
        for t in self._tracks:
            u, v = int(t.cand.u * scale), int(t.cand.v * scale)
            color = (0, 255, 255) if t.confirmed else (140, 140, 140)
            cv2.putText(frame, f"{t.score:.2f}/{t.depth:.0f}", (u + 8, v + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    def reset(self):
        self._tracks = []
        self._last_ts = None
        self._last_output = []
