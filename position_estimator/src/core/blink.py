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

★ 2値トグル判定 (toggle_min_hz を与えたカメラだけ。今は Camera1)。
  ロックインは取得時刻が正確なことが前提。2026-09-17 の Camera1 は α6400 が
  1/15s の自動露出で実効 10fps、フレーム間隔も 33〜230ms とばらつき、
  ROI 輝度は「点灯 200 / 消灯 113」ときれいな2値なのにスコアは 0.28 だった
  (時刻が 50ms ずれると 6Hz では位相が 108° ずれる)。そこで時刻の精度に
  頼らない判定に置き換える (ロックインのスコアは表示・ログ用に計算だけする):
    ・明暗2つの輝度 (20/80 パーセンタイル) の差が toggle_min_contrast 以上
    ・サンプルの 6 割以上がどちらかの輝度の近く (差の 1/4 以内) にある = 2値的
    ・明/暗それぞれが 2 割以上ある (点滅は 50% duty)
    ・明暗の切り替わりが毎秒 toggle_min_hz 回以上 (6Hz の点滅で最大 12 回)
  同じ録画と 2026-09-16 の録画で、LED は差 58〜76・毎秒 5〜12 回、
  それ以外 (床の反射・照明・OSD) は差 30 以下・毎秒 5.3 回以下だった。
  人が手を振っている Camera2 の録画に同じ条件を当てると、ロックインでは
  人や画面上の文字の確定が 119 回出たのに対し、この判定では 1 回だった
  (動く物の ROI 輝度は連続的に変わり「2値的」にならない)。

★ 動く機体への追従 (max_speed_px_s を与えたカメラだけ。今は Camera1)。
  2026-09-17 10:08 の録画 (0.8m/s 以下で 30 秒動き続ける) では、機体が画面上で
  最大 ~500px/s 動き、候補が 1 枚で 30px 以上跳ぶとトラックが切れて作り直され、
  点滅の履歴が溜まらずに確定 0.4% だった。そこで:
    ・トラックが画面上の速度を持ち、予測位置との距離で候補を割り当てる。
      許容距離は経過時間ぶん広げる (速度不明なら max_speed_px_s×dt、既知なら半分)。
    ・消灯中で候補が来ないフレームも予測位置 (外挿は最長 MAX_EXTRAPOLATE_SEC) で測る。
    ・roi_peak_px: 予測のずれを吸収するため、半径内で roi_px 角平均が最大の所の値を使う。
    ・coast_sec: 確定済みトラックは、条件を外れても (白飛びした窓の前を横切る等)
      その秒数までは確定のまま予測位置を出す。
"""

import math
from collections import deque

import cv2
import numpy as np


class _Track:
    __slots__ = ("cand", "cand_ts", "vel", "pos", "last_seen_ts", "samples", "score",
                 "depth", "toggle_hz", "confirmed", "last_ok_ts")

    def __init__(self, cand, ts):
        self.cand = cand                # 最後に割り当てた候補 (その時刻が cand_ts)
        self.cand_ts = ts
        self.vel = None                 # 画面上の速度 [px/s]。2回割り当てるまで不明
        self.pos = cand                 # 今フレームで輝度を測った位置 (= 出力する候補)
        self.last_seen_ts = ts
        self.samples = deque()          # (ts, 平均輝度)
        self.score = 0.0
        self.depth = 0.0
        self.toggle_hz = 0.0            # 2値トグル判定を満たしたときの切り替わり回数/秒
        self.confirmed = False
        self.last_ok_ts = None          # 点滅の条件を最後に満たした時刻


class BlinkTracker:
    """1カメラぶんの候補列に、既知周波数の点滅によるフィルタをかける。"""

    MAX_TRACKS = 32
    MAX_EXTRAPOLATE_SEC = 0.3

    def __init__(self, label, target_hz, match_dist_px=30.0, roi_px=8,
                 history_sec=1.2, min_score=0.5, min_depth=1.5, enabled=True,
                 toggle_min_hz=None, toggle_min_contrast=35.0, max_speed_px_s=None,
                 roi_peak_px=None, min_samples=8, min_span_ratio=0.75, coast_sec=0.0):
        self.label = label
        self.enabled = enabled and target_hz > 0
        self.omega = 2.0 * math.pi * target_hz
        self.target_hz = target_hz
        self.match_dist_px = match_dist_px
        self.roi_px = roi_px
        self.history_sec = history_sec
        self.min_score = min_score
        self.min_depth = min_depth
        self.toggle_min_hz = toggle_min_hz
        self.toggle_min_contrast = toggle_min_contrast
        self.max_speed_px_s = max_speed_px_s
        self.roi_peak_px = roi_peak_px
        self.min_samples = min_samples
        self.min_span_ratio = min_span_ratio
        self.coast_sec = coast_sec
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
            # 候補が来なかった (消灯中の) トラックは、動いていれば予測位置で測る
            t.pos = self._predict(t, ts)
            t.samples.append((ts, self._roi_mean(frame, t.pos, scale)))
            while t.samples and ts - t.samples[0][0] > self.history_sec:
                t.samples.popleft()
            self._update_score(t, ts)
            # ★ 点滅が確認できている間は、bright 検知の候補が出なくても
            #   トラックを生かす。寿命を候補の有無だけで決めていたため、
            #   しきい値ぎりぎり/静的マスクに一部欠けた LED だと候補が
            #   途切れるたびに削除され、静止した機体でも
            #   「1.2秒検知 -> 数秒見失い」を繰り返した (2026-09-15 Camera1)。
            #   ROI の6Hz成分そのものが機体の証拠なので、それで延命する。
            if t.confirmed:
                t.last_seen_ts = ts

        self._last_output = [t.pos for t in self._tracks if t.confirmed]
        return self._last_output

    # ------------------------------------------------------------------
    def _predict(self, t, ts):
        """速度が分かっていれば等速で外挿した位置。外挿は最長 MAX_EXTRAPOLATE_SEC。"""
        if t.vel is None:
            return t.cand
        dt = min(ts - t.cand_ts, self.MAX_EXTRAPOLATE_SEC)
        return t.cand._replace(u=t.cand.u + t.vel[0] * dt, v=t.cand.v + t.vel[1] * dt)

    def _gate(self, t, ts):
        """割り当ての許容距離。動く機体では前回からの経過時間ぶん広げる。"""
        if self.max_speed_px_s is None:
            return self.match_dist_px
        dt = min(ts - t.cand_ts, self.MAX_EXTRAPOLATE_SEC)
        # 速度が分かっているトラックは予測位置からのずれ (加減速ぶん) だけ許す
        k = 0.5 if t.vel is not None else 1.0
        return self.match_dist_px + k * self.max_speed_px_s * dt

    def _associate(self, candidates, ts):
        pairs = []
        preds = [self._predict(t, ts) for t in self._tracks]
        for ti, t in enumerate(self._tracks):
            gate = self._gate(t, ts)
            for ci, c in enumerate(candidates):
                d = math.hypot(c.u - preds[ti].u, c.v - preds[ti].v)
                if d <= gate:
                    pairs.append((d, ti, ci))
        pairs.sort()
        used_t, used_c = set(), set()
        for _, ti, ci in pairs:
            if ti in used_t or ci in used_c:
                continue
            used_t.add(ti)
            used_c.add(ci)
            t, c = self._tracks[ti], candidates[ci]
            if self.max_speed_px_s is not None and ts > t.cand_ts:
                dt = ts - t.cand_ts
                vu, vv = (c.u - t.cand.u) / dt, (c.v - t.cand.v) / dt
                speed = math.hypot(vu, vv)
                if speed > self.max_speed_px_s:
                    vu, vv = vu * self.max_speed_px_s / speed, vv * self.max_speed_px_s / speed
                t.vel = ((vu, vv) if t.vel is None
                         else (0.5 * (t.vel[0] + vu), 0.5 * (t.vel[1] + vv)))
            t.cand = c
            t.cand_ts = ts
            t.last_seen_ts = ts

        for ci, c in enumerate(candidates):
            if ci in used_c:
                continue
            # 既存トラックの近く (消灯中で割り当てが無かったもの等) には増やさない
            if any(math.hypot(c.u - preds[ti].u, c.v - preds[ti].v) <= self.match_dist_px
                   for ti in range(len(preds))):
                continue
            if len(self._tracks) < self.MAX_TRACKS:
                self._tracks.append(_Track(c, ts))

        # 候補として一度も出てこなくなったトラックは捨てる。
        # 点滅中のLEDは半周期ごとに必ず候補になるので history_sec あれば十分。
        self._tracks = [t for t in self._tracks
                        if ts - t.last_seen_ts <= self.history_sec]

    def _roi_mean(self, frame, cand, scale):
        if self.roi_peak_px is not None:
            return self._roi_peak(frame, cand, scale)
        h, w = frame.shape[:2]
        r = max(2, int(round(self.roi_px * scale)))
        cu, cv_ = int(round(cand.u * scale)), int(round(cand.v * scale))
        x0, x1 = max(0, cu - r), min(w, cu + r + 1)
        y0, y1 = max(0, cv_ - r), min(h, cv_ + r + 1)
        if x0 >= x1 or y0 >= y1:
            return 0.0
        # 白色LEDなのでチャンネル平均でよい。最大値だと白飛び時に変調が消える。
        return float(frame[y0:y1, x0:x1].mean())

    def _roi_peak(self, frame, cand, scale):
        """半径 roi_peak_px の中で、roi_px 角の平均輝度がいちばん高い所の値。
        動く機体では予測位置が数 px ずれるので、固定 ROI の平均だと LED を外す。"""
        h, w = frame.shape[:2]
        R = max(2, int(round(self.roi_peak_px * scale)))
        k = max(1, int(round(self.roi_px * scale)))
        cu, cv_ = int(round(cand.u * scale)), int(round(cand.v * scale))
        x0, x1 = max(0, cu - R - k), min(w, cu + R + k + 1)
        y0, y1 = max(0, cv_ - R - k), min(h, cv_ + R + k + 1)
        if x1 - x0 <= 2 * k or y1 - y0 <= 2 * k:
            return 0.0
        g = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
        m = cv2.blur(g, (2 * k + 1, 2 * k + 1))[k:-k, k:-k]
        return float(m.max())

    def _update_score(self, t, ts):
        was_confirmed = t.confirmed
        self._judge(t)
        if t.confirmed:
            t.last_ok_ts = ts
        elif (was_confirmed and t.last_ok_ts is not None
              and ts - t.last_ok_ts <= self.coast_sec):
            # 確定済みの機体が白飛びした窓の前を横切る・1〜2枚ブレる間は、
            # 予測位置で確定のまま出し続ける (coast_sec まで)
            t.confirmed = True

    def _judge(self, t):
        n = len(t.samples)
        if (n < self.min_samples
                or t.samples[-1][0] - t.samples[0][0] < self.min_span_ratio * self.history_sec):
            t.score, t.depth, t.toggle_hz, t.confirmed = 0.0, 0.0, 0.0, False
            return
        arr = np.asarray(t.samples, dtype=np.float64)
        x = arr[:, 1] - arr[:, 1].mean()
        var_sum = float(np.dot(x, x))
        if var_sum <= 1e-9:
            t.score, t.depth, t.toggle_hz, t.confirmed = 0.0, 0.0, 0.0, False
            return
        phase = self.omega * (arr[:, 0] - arr[0, 0])
        i = float(np.dot(x, np.cos(phase)))
        q = float(np.dot(x, np.sin(phase)))
        t.score = min(1.0, 2.0 * (i * i + q * q) / (n * var_sum))
        t.depth = math.sqrt(var_sum / n)
        # ヒステリシス: 一瞬の遮蔽やブレでスコアが落ちても即座には外さない
        t.toggle_hz = self._toggle_hz(arr)
        if self.toggle_min_hz is not None:
            on_hz = self.toggle_min_hz if not t.confirmed else self.toggle_min_hz * 0.8
            t.confirmed = t.toggle_hz >= on_hz
        else:
            on_thr = self.min_score if not t.confirmed else self.min_score * 0.6
            t.confirmed = t.score >= on_thr and t.depth >= self.min_depth

    def _toggle_hz(self, arr):
        """2値トグル判定 (モジュール冒頭の説明)。満たさなければ 0、満たせば切り替わり回数/秒。"""
        if self.toggle_min_hz is None:
            return 0.0
        ts, x = arr[:, 0], arr[:, 1]
        lo, hi = np.percentile(x, (20, 80))
        contrast = hi - lo
        if contrast < self.toggle_min_contrast:
            return 0.0
        bright = x > 0.5 * (lo + hi)
        if min(bright.mean(), 1.0 - bright.mean()) < 0.2:
            return 0.0
        near_level = np.minimum(np.abs(x - lo), np.abs(x - hi)) < 0.25 * contrast
        if near_level.mean() < 0.6:
            return 0.0
        switches = int(np.count_nonzero(bright[1:] != bright[:-1]))
        return switches / max(ts[-1] - ts[0], 1e-6)

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
            u, v = int(t.pos.u * scale), int(t.pos.v * scale)
            color = (0, 255, 255) if t.confirmed else (140, 140, 140)
            text = (f"{t.score:.2f}/{t.depth:.0f}" if self.toggle_min_hz is None
                    else f"T{t.toggle_hz:.1f}Hz")
            cv2.putText(frame, text, (u + 8, v + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    def best(self):
        """ログ用: いちばんスコアの高いトラックの (score, depth)。トラックが無ければ (0, 0)。
        「候補はあるのに黄色にならない」とき、score が低い (点滅が潰れている) のか
        depth が低い (LED が暗い/小さい) のかをログから切り分けるため。"""
        if not self._tracks:
            return 0.0, 0.0
        t = max(self._tracks, key=lambda t: t.score)
        return t.score, t.depth

    def reset(self):
        self._tracks = []
        self._last_ts = None
        self._last_output = []
