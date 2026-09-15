"""
core/blink.py
LED点滅パターンによる候補フィルタ。

detect() が返す「明るい/動いた」候補は、1フレームだけでは機体かどうか
判定できない（窓の反射・照明も同じように光る）。機体のLEDを既知の周波数
で点滅させておけば、時間方向に「その周波数で明滅しているか」を見ることで
静止した明点を積極的に落とせる。PairSelector の幾何整合より前段の
フィルタとして使う。

標本化定理: LED_BLINK_HZ はカメラの実効フレームレートの半分未満でなければ
点滅を復元できない（エイリアシング）。さらに検知の取りこぼし（フレーム
ドロップ・露出の谷）を吸収するには半周期あたり数フレームの余裕が要るため、
実測FPSの1/4〜1/6程度を目安にする（例: 実測30fpsなら5〜7Hz程度）。
BlinkTracker.check_sampling() が実測FPSと照合して警告を出す。

アルゴリズム:
    1) フレーム間で候補を最近傍マッチングし、同一光点の「点灯/消灯」の
       時系列を追う（トラック）。
    2) OFF→ON の立ち上がり時刻を直近 BLINK_HISTORY_SEC 秒ぶん記録する。
    3) 立ち上がり間隔が目標周期 (1/LED_BLINK_HZ) に近いものの割合を
       スコアとする。静止した明点は立ち上がりが起きない（または起きても
       周期が揃わない）ため、スコアが上がらない。
    4) トラックが十分な期間生存し、かつスコアが閾値未満の候補は捨てる。
       生存期間が短いトラック（判定材料が足りない）はまだ捨てない
       （新規に入ってきた本物の機体候補を初速で殺さないため）。
"""

import math
from collections import deque


class _Track:
    __slots__ = ("u", "v", "state", "last_seen_ts", "born_ts",
                 "rising_edges", "score")

    def __init__(self, u, v, ts):
        self.u = u
        self.v = v
        self.state = True          # 生成された時点で「明るい」候補なのでON
        self.last_seen_ts = ts
        self.born_ts = ts
        self.rising_edges = deque([ts])
        self.score = 0.0

    def age(self, ts) -> float:
        return ts - self.born_ts

    def _prune(self, ts, history_sec):
        while self.rising_edges and ts - self.rising_edges[0] > history_sec:
            self.rising_edges.popleft()

    def _update_score(self, target_period, period_tol, min_cycles):
        edges = self.rising_edges
        if len(edges) < 2:
            self.score = 0.0
            return
        periods = [edges[i + 1] - edges[i] for i in range(len(edges) - 1)]
        good = [p for p in periods if abs(p - target_period) <= target_period * period_tol]
        ratio = len(good) / len(periods)
        # 周期数が min_cycles に満たない間は満点を出さない（早すぎる確定を防ぐ）
        confidence = min(1.0, len(good) / float(min_cycles))
        self.score = ratio * confidence

    def mark_seen(self, u, v, ts, target_period, period_tol, min_cycles, history_sec):
        if not self.state:
            self.rising_edges.append(ts)      # OFF -> ON の立ち上がり
        self.state = True
        self.u, self.v = u, v
        self.last_seen_ts = ts
        self._prune(ts, history_sec)
        self._update_score(target_period, period_tol, min_cycles)

    def mark_missed(self, ts, off_debounce_s, target_period, period_tol,
                     min_cycles, history_sec):
        if ts - self.last_seen_ts > off_debounce_s:
            self.state = False
        self._prune(ts, history_sec)
        self._update_score(target_period, period_tol, min_cycles)


class BlinkTracker:
    """
    1カメラぶんの候補列に対して、フレームをまたいだ点滅整合フィルタをかける。

    使い方:
        blink = BlinkTracker("Camera1", target_hz=6.0, camera_fps=30.0)
        ...
        candidates = blink.filter(candidates, timestamp)
    """

    def __init__(self, label, target_hz, camera_fps=None,
                 match_dist_px=30.0, history_sec=1.2, min_cycles=3,
                 period_tol=0.35, mature_sec=0.6, enabled=True):
        self.label = label
        self.enabled = enabled
        self.target_hz = target_hz
        self.target_period = 1.0 / target_hz if target_hz > 0 else None
        self.match_dist_px = match_dist_px
        self.history_sec = history_sec
        self.min_cycles = min_cycles
        self.period_tol = period_tol
        # 判定を確定させるまでの猶予。これより若いトラックはスコア不足でも通す。
        self.mature_sec = mature_sec
        self._tracks = []
        if camera_fps:
            self.check_sampling(camera_fps)

    def check_sampling(self, camera_fps):
        """標本化定理を満たしているか確認し、満たしていなければ警告する。"""
        if not self.enabled or not self.target_hz:
            return
        nyquist = camera_fps / 2.0
        if self.target_hz >= nyquist:
            print(f"  [{self.label}] [WARN] LED_BLINK_HZ={self.target_hz:.1f}Hz が "
                  f"実効FPS {camera_fps:.1f} のナイキスト周波数 {nyquist:.1f}Hz 以上です。"
                  "点滅を復元できません。LED_BLINK_HZ を下げてください。")
        elif camera_fps / self.target_hz < 4.0:
            print(f"  [{self.label}] [WARN] 実効FPS {camera_fps:.1f} に対し "
                  f"LED_BLINK_HZ={self.target_hz:.1f}Hz は余裕が少ないです "
                  f"(1周期あたり{camera_fps / self.target_hz:.1f}フレーム)。"
                  "フレーム取りこぼしで誤棄却しやすくなります。目安は4倍以上。")

    def filter(self, candidates, ts):
        """
        候補リストを点滅整合フィルタにかけて返す。

        BLINK 無効時、または目標周波数未設定時はそのまま素通しする。
        候補が0個のフレーム（点滅の消灯フェーズ）も、既存トラックを
        「消灯した」として更新する必要があるため、ここで早期returnしては
        いけない — 消灯を記録しないと立ち上がりエッジが一切積み上がらず、
        スコアが永久に0のままになる。
        """
        if not self.enabled or not self.target_period:
            return candidates

        matched = set()
        for track in self._tracks:
            best_i, best_d = None, self.match_dist_px
            for i, c in enumerate(candidates):
                if i in matched:
                    continue
                d = math.hypot(c.u - track.u, c.v - track.v)
                if d < best_d:
                    best_d, best_i = d, i
            if best_i is not None:
                track.mark_seen(candidates[best_i].u, candidates[best_i].v, ts,
                                 self.target_period, self.period_tol,
                                 self.min_cycles, self.history_sec)
                matched.add(best_i)
            else:
                track.mark_missed(ts, self._off_debounce(),
                                   self.target_period, self.period_tol,
                                   self.min_cycles, self.history_sec)

        for i, c in enumerate(candidates):
            if i not in matched:
                self._tracks.append(_Track(c.u, c.v, ts))

        self._age_out(ts)

        kept = []
        for i, c in enumerate(candidates):
            track = self._nearest_track(c.u, c.v)
            if track is None:
                kept.append(c)
                continue
            if track.age(ts) < self.mature_sec:
                kept.append(c)        # 判定材料が足りない間は殺さない
            elif track.score >= self._min_score():
                kept.append(c)
        return kept

    def _off_debounce(self):
        # 半周期未満の消灯は「点滅の谷」として無視し、状態をOFFにしない
        return (self.target_period or 0.0) * 0.4

    def _min_score(self):
        return 0.5

    def _nearest_track(self, u, v):
        best, best_d = None, self.match_dist_px
        for t in self._tracks:
            d = math.hypot(u - t.u, v - t.v)
            if d < best_d:
                best_d, best = d, t
        return best

    def _age_out(self, ts):
        limit = max(self.history_sec, self._off_debounce() * 4.0)
        self._tracks = [t for t in self._tracks if ts - t.last_seen_ts <= limit]

    def reset(self):
        self._tracks = []
