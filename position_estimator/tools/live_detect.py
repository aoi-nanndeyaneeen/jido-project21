"""
tools/live_detect.py
USB カメラ 1 台の生映像を、本番と同じ検知系 (core/camera.py の detect -> core/blink.py)
に流して、点滅 LED を捉えられるかをその場で確かめる。main.py と違い、
キャリブ・RPi・シリアル・もう 1 台のカメラは要らない。

    python tools/live_detect.py                      # CAMERA_1_URL を Camera1 の係数で
    python tools/live_detect.py --device 0
    python tools/live_detect.py --exposure auto      # 露出に触らない
    python tools/live_detect.py --exposure -6        # 露出を数値で固定
    python tools/live_detect.py --label Camera2      # Camera2 の係数 (PC 側の値) で

本番との対応:
  ・カメラは CameraTracker(CAMERA_1_URL) と同じ開き方 (DSHOW + MJPG をパラメータ渡し)。
  ・露出は既定 (--exposure main) で、本番のキャリブ完了時と同じく
    自動露出で WARMUP_SEC 待ってから lock_exposure() (TRACKING_EXPOSURE) する。
    追跡中の recheck_exposure() も同じく 2 秒ごとに呼ぶ。
  ・検知係数は detection_params_for(label) / blink_params_for(label)。
    detection_params.json や config.py を直せばそのまま反映される。

画面:
  マゼンタ枠 = detect() の候補 (raw)
  灰/黄の文字 = blink のトラック (黄 = 点滅確定)。緑丸 = 確定した候補
  右下のグラフ = 輝度の時系列。クリックした点 (右クリックで解除)、
                 無ければいちばん点滅らしいトラックの ROI 輝度。
                 LED が 6Hz で 2 値に振れていれば「カメラは点滅を捉えている」。
                 振れていないなら検知係数ではなく露出・fps の問題。

キー:
  q / Esc  終了
  r        検知と点滅判定の状態をリセット (静的マスク再学習も)
  [ / ]    露出を 1 段暗く / 明るく (手動固定になる)
  l        本番と同じ露出固定 (lock_exposure)
  a        自動露出に戻す
  v        生フレームの録画 開始/停止 (tests/live_*.mp4)
           後から python tools/replay_detect.py <mp4> --crop none --ignore none --truth ""

ログ:
  src/logs/live_detect_YYYYMMDD_HHMMSS.csv に新フレームごとに 1 行 (列は LiveDetectLogger)。
  キー操作 (リセット・露出変更・録画) は Event 列に残る。--no-log で書かない。
  終了時 (q / 窓を閉じる / Ctrl+C) に集計を表示する。
"""

import argparse
import datetime
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from core.camera import (CameraTracker, DSHOW_EXPOSURE_AUTO,      # noqa: E402
                         DSHOW_EXPOSURE_MANUAL)
from core.blink import BlinkTracker                               # noqa: E402
from utils.config import (CAMERA_1_URL, CAMERA_W, CAMERA_H,       # noqa: E402
                          LED_BLINK_HZ, LOG_DIR, blink_params_for)
from utils.logger import CsvLogger, _f                            # noqa: E402

WINDOW = "live_detect"
WARMUP_SEC = 1.5          # 露出固定の前に自動露出を落ち着かせる時間
PLOT_SEC = 2.0            # グラフに出す時間幅
PLOT_W, PLOT_H = 420, 140
PROBE_R = 8               # クリック点の輝度を測る半径 [px] (この中の 3x3 平均の最大)
RATE_SEC = 5.0            # 「確定あり」割合を出す窓


class Probe:
    """クリックした点の輝度時系列。検知の有無と無関係に、カメラが明滅を写しているかを見る。"""

    def __init__(self):
        self.point = None
        self.samples = deque()

    def set(self, point):
        self.point = point
        self.samples.clear()

    def sample(self, frame, ts):
        if self.point is None:
            return
        u, v = self.point
        h, w = frame.shape[:2]
        x0, x1 = max(0, u - PROBE_R), min(w, u + PROBE_R + 1)
        y0, y1 = max(0, v - PROBE_R), min(h, v + PROBE_R + 1)
        if x1 - x0 < 3 or y1 - y0 < 3:
            return
        g = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
        self.samples.append((ts, float(cv2.blur(g, (3, 3)).max())))
        while ts - self.samples[0][0] > PLOT_SEC:
            self.samples.popleft()


def best_track(blink):
    """グラフに出すトラック: 確定済みを優先し、その中で切り替わりの多いもの。"""
    tracks = [t for t in blink._tracks if len(t.samples) >= 3]
    if not tracks:
        return None
    return max(tracks, key=lambda t: (t.confirmed, t.toggle_hz, t.score, t.depth))


def blink_stats(samples):
    """(明暗差, 明暗の切り替わり回数/秒)。blink.py の 2 値トグル判定と同じ数え方。"""
    if len(samples) < 3:
        return 0.0, 0.0
    arr = np.asarray(samples, dtype=np.float64)
    lo, hi = np.percentile(arr[:, 1], (20, 80))
    bright = arr[:, 1] > 0.5 * (lo + hi)
    switches = int(np.count_nonzero(bright[1:] != bright[:-1]))
    return hi - lo, switches / max(arr[-1, 0] - arr[0, 0], 1e-6)


def draw_plot(vis, samples, title, color):
    h, w = vis.shape[:2]
    x0, y0 = w - PLOT_W - 10, h - PLOT_H - 10
    roi = vis[y0:y0 + PLOT_H, x0:x0 + PLOT_W]
    roi[:] = (roi * 0.3).astype(np.uint8)
    cv2.rectangle(vis, (x0, y0), (x0 + PLOT_W, y0 + PLOT_H), (200, 200, 200), 1)
    contrast, hz = blink_stats(samples)
    cv2.putText(vis, f"{title}  diff {contrast:.0f}  switch {hz:.1f}/s "
                f"(6Hz blink = {2 * LED_BLINK_HZ:.0f}/s)",
                (x0 + 6, y0 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
    if len(samples) < 2:
        return
    arr = np.asarray(samples, dtype=np.float64)
    lo, hi = arr[:, 1].min(), arr[:, 1].max()
    span = max(hi - lo, 10.0)
    t_end = arr[-1, 0]
    top, bottom = y0 + 26, y0 + PLOT_H - 8
    pts = []
    for t, val in arr:
        px = x0 + PLOT_W - 6 - (t_end - t) / PLOT_SEC * (PLOT_W - 12)
        py = bottom - (val - lo) / span * (bottom - top)
        pts.append((int(px), int(py)))
    cv2.polylines(vis, [np.array(pts, dtype=np.int32)], False, color, 1)
    for p in pts:
        cv2.circle(vis, p, 2, color, -1)
    cv2.putText(vis, f"{hi:.0f}", (x0 + 4, top + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                (200, 200, 200), 1)
    cv2.putText(vis, f"{lo:.0f}", (x0 + 4, bottom), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                (200, 200, 200), 1)


class LiveDetectLogger(CsvLogger):
    """
    新フレーム 1 枚につき 1 行。src/logs/live_detect_*.csv。
    先頭列 Epoch_s はフレームの取得時刻 (CameraTracker の reader が付けた time.time())。
    flight_*.csv / s5_link_*.csv と同じく、これで他のログと突き合わせる。

    Best_* はグラフに出しているのと同じ「いちばん点滅らしいトラック」。
    確定しない原因の切り分けに使う:
      Best_Contrast が toggle_min_contrast 未満   -> LED が暗い/露出が長くて明暗が潰れている
      Best_ToggleHz が toggle_min_hz 未満         -> fps が足りない/取りこぼしている
      NRaw=0 が続く                               -> detect() 段階で拾えていない (係数の問題)
    Raw / Tracks は "u:v:面積" / "u:v:切替Hz:確定" を ; 区切り。
    """

    HEADER = ["Epoch_s", "Time", "Frame", "ReaderFPS", "Read_ms", "Detect_ms",
              "Exposure", "ExposureMode", "Rejected",
              "NRaw", "NTracks", "NConfirmed", "Conf_u", "Conf_v",
              "Best_u", "Best_v", "Best_Confirmed", "Best_ToggleHz", "Best_Contrast",
              "Best_Score", "Best_Depth", "Best_Level",
              "Probe_u", "Probe_v", "Probe_Level",
              "Raw", "Tracks", "Event"]

    def __init__(self, log_path: Path):
        super().__init__(log_path, self.HEADER)

    def write(self, ts, frame_no, st, detect_ms, exposure, exposure_mode, rejected,
              cands, blink, confirmed, probe, event):
        hms = datetime.datetime.fromtimestamp(ts).strftime("%H:%M:%S.%f")[:-3]
        conf = confirmed[0] if confirmed else None
        t = best_track(blink)
        contrast = blink_stats(t.samples)[0] if t is not None else None
        probe_level = (probe.samples[-1][1]
                       if probe.point is not None and probe.samples
                       and probe.samples[-1][0] == ts else None)
        self._writer.writerow([
            round(ts, 3), hms, frame_no,
            _f(st["reader_fps"], 1), _f(st["reader_last_ms"], 1), _f(detect_ms, 1),
            _f(exposure, 1), exposure_mode, int(rejected),
            len(cands), len(blink._tracks), len(confirmed),
            _f(conf.u if conf else None, 1), _f(conf.v if conf else None, 1),
            _f(t.pos.u if t else None, 1), _f(t.pos.v if t else None, 1),
            int(t.confirmed) if t else "", _f(t.toggle_hz if t else None, 2),
            _f(contrast, 1), _f(t.score if t else None, 3), _f(t.depth if t else None, 1),
            _f(t.samples[-1][1] if t and t.samples else None, 1),
            probe.point[0] if probe.point else "", probe.point[1] if probe.point else "",
            _f(probe_level, 1),
            ";".join(f"{c.u:.0f}:{c.v:.0f}:{c.area:.0f}" for c in cands),
            ";".join(f"{tr.pos.u:.0f}:{tr.pos.v:.0f}:{tr.toggle_hz:.1f}:{int(tr.confirmed)}"
                     for tr in blink._tracks),
            event,
        ])


def set_manual_exposure(cam, value):
    cam.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, DSHOW_EXPOSURE_MANUAL)
    cam.cap.set(cv2.CAP_PROP_EXPOSURE, float(value))
    print(f"  露出を手動 {value:.1f} に (読み戻し {cam.cap.get(cv2.CAP_PROP_EXPOSURE):.1f})")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", type=int, default=CAMERA_1_URL, help="USB カメラ番号")
    ap.add_argument("--label", default="Camera1",
                    help="検知係数のカメラ名 (detection_params.json の camera_overrides / "
                         "config.BLINK_CAMERA_OVERRIDES のキー)")
    ap.add_argument("--exposure", default="main",
                    help="main: 本番と同じ固定 (TRACKING_EXPOSURE) / auto: 触らない / 数値: その値で固定")
    ap.add_argument("--scale", type=float, default=1.0, help="表示の縮尺")
    ap.add_argument("--no-log", action="store_true", help="CSV ログを書かない")
    a = ap.parse_args()

    cam = CameraTracker(a.device, width=CAMERA_W, height=CAMERA_H, label=a.label)
    if not cam.cap.isOpened():
        raise SystemExit(f"カメラ {a.device} を開けません (tools/find_camera.py で番号を確認)")
    blink = BlinkTracker(a.label, LED_BLINK_HZ, **blink_params_for(a.label))
    print(f"  検知モード: {cam.p['detect_mode']}  点滅判定: {blink_params_for(a.label)}")

    exposure_mode = a.exposure            # "main" / "auto" / "manual"
    manual_value = None
    if exposure_mode not in ("main", "auto"):
        manual_value = float(exposure_mode)
        exposure_mode = "manual"
        set_manual_exposure(cam, manual_value)
    elif exposure_mode == "auto":
        cam.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, DSHOW_EXPOSURE_AUTO)

    cam.start_latest_reader()
    start = time.time()
    exposure_pending = exposure_mode == "main"   # WARMUP_SEC 後に lock_exposure

    probe = Probe()

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            probe.set((int(x / a.scale), int(y / a.scale)))
        elif event == cv2.EVENT_RBUTTONDOWN:
            probe.set(None)

    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, on_mouse)

    last_ts = None
    history = deque()          # (ts, 確定あり)
    totals = {"frames": 0, "confirmed": 0, "rejected": 0}
    detect_ms = 0.0
    recheck_time = time.time()
    writer = None
    rec_path = None
    rec_frames = 0
    key = 255                  # 新フレームが来るまで持ち越す (来ない周回の方が多い)
    events = []                # 次の行の Event 列に書くもの
    log = None
    if not a.no_log:
        log = LiveDetectLogger(
            LOG_DIR / f"live_detect_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        print(f"  ログ: {log.path}")

    try:
        while True:
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            # 窓の × で閉じた (imshow が窓を作り直すので放置すると終われない)
            if totals["frames"] and cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k != 255:
                key = k

            if exposure_pending and time.time() - start >= WARMUP_SEC:
                exposure_pending = False
                cam.lock_exposure()
                events.append("lock_exposure(warmup)")

            t0 = time.perf_counter()
            frame, cands, ts = cam.read_and_detect()
            if frame is None or ts == last_ts:
                continue
            last_ts = ts
            # ROI 輝度を測るので描き込み前のフレームを渡す (tracker.py と同じ順)
            confirmed = blink.filter(frame, cands, ts, cam.width)
            detect_ms = 0.9 * detect_ms + 0.1 * (time.perf_counter() - t0) * 1000.0
            probe.sample(frame, ts)

            if writer is not None:
                writer.write(frame)
                rec_frames += 1

            totals["frames"] += 1
            totals["confirmed"] += bool(confirmed)
            totals["rejected"] += cam.vibration_rejected
            history.append((ts, bool(confirmed)))
            while ts - history[0][0] > RATE_SEC:
                history.popleft()

            st = cam.get_performance_stats()
            exp_now = cam.cap.get(cv2.CAP_PROP_EXPOSURE)
            if log is not None:
                log.write(ts, totals["frames"], st, (time.perf_counter() - t0) * 1000.0,
                          exp_now, exposure_mode, cam.vibration_rejected,
                          cands, blink, confirmed, probe, " ".join(events))
                events.clear()

            if exposure_mode == "main" and time.time() - recheck_time >= 2.0:
                recheck_time = time.time()
                st = cam.get_performance_stats()
                if st["reader_reads"] > 30:
                    cam.recheck_exposure(st["reader_last_ms"])

            # ── 描画 ──────────────────────────────────────────
            vis = frame.copy()
            for c in cands:
                cv2.rectangle(vis, (c.x - 2, c.y - 2), (c.x + c.w + 2, c.y + c.h + 2),
                              (255, 0, 255), 1)
            blink.draw(vis, cam.width)
            cam.draw_candidates(vis, confirmed, 0 if confirmed else None)
            for c in confirmed[1:]:
                cv2.circle(vis, (int(c.u), int(c.v)), 12, (0, 255, 0), 2)

            rate = sum(h for _, h in history) / max(1, len(history)) * 100.0
            exp_text = {"main": "locked(main)" if not exposure_pending else "auto->lock",
                        "auto": "auto", "manual": "manual"}[exposure_mode]
            lines = [
                f"mode {cam.p['detect_mode']}  cam {st['reader_fps']:.1f}fps "
                f"({st['reader_last_ms']:.0f}ms/read)  detect+blink {detect_ms:.1f}ms",
                f"exposure {exp_now:.1f} [{exp_text}]  raw {len(cands)}  "
                f"blink tracks {len(blink._tracks)}  confirmed {len(confirmed)}",
                f"confirmed in last {RATE_SEC:.0f}s: {rate:.0f}%",
            ]
            if st["reader_fps"] and 1.0 / max(st["reader_fps"], 1e-6) >= 0.5 / LED_BLINK_HZ:
                lines.append(f"WARN: fps < {2 * LED_BLINK_HZ:.0f} - blink will blur")
            if writer is not None:
                lines.append(f"REC {rec_path.name} ({rec_frames})")
            for i, text in enumerate(lines):
                y = 100 + 24 * i
                color = (0, 0, 255) if text.startswith(("WARN", "REC")) else (255, 255, 255)
                cv2.putText(vis, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(vis, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

            if probe.point is not None:
                cv2.circle(vis, probe.point, PROBE_R, (255, 255, 0), 1)
                draw_plot(vis, probe.samples, f"probe {probe.point}", (255, 255, 0))
            else:
                t = best_track(blink)
                if t is not None:
                    color = (0, 255, 255) if t.confirmed else (170, 170, 170)
                    cv2.circle(vis, (int(t.pos.u), int(t.pos.v)), 16, color, 1)
                    draw_plot(vis, t.samples,
                              f"track ({t.pos.u:.0f},{t.pos.v:.0f})", color)
                else:
                    draw_plot(vis, [], "no track (click a point to probe)", (170, 170, 170))

            if a.scale != 1.0:
                vis = cv2.resize(vis, None, fx=a.scale, fy=a.scale, interpolation=cv2.INTER_AREA)
            cv2.imshow(WINDOW, vis)

            # ── キー ──────────────────────────────────────────
            if key == ord("r"):
                cam.reset_background()
                blink.reset()
                history.clear()
                events.append("reset")
                print("  リセットしました")
            elif key in (ord("["), ord("]")):
                base = manual_value if manual_value is not None else round(exp_now)
                manual_value = base + (1 if key == ord("]") else -1)
                exposure_mode, exposure_pending = "manual", False
                set_manual_exposure(cam, manual_value)
                events.append(f"exposure={manual_value:.0f}")
            elif key == ord("l"):
                exposure_mode, exposure_pending, manual_value = "main", False, None
                cam.lock_exposure()
                events.append("lock_exposure")
            elif key == ord("a"):
                exposure_mode, exposure_pending, manual_value = "auto", False, None
                cam.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, DSHOW_EXPOSURE_AUTO)
                events.append("auto_exposure")
                print("  自動露出に戻しました")
            elif key == ord("v"):
                if writer is None:
                    rec_path = ROOT / "tests" / time.strftime("live_%Y-%m-%d_%H%M%S.mp4")
                    fps = st["reader_fps"] if st["reader_fps"] > 1 else 30.0
                    writer = cv2.VideoWriter(str(rec_path), cv2.VideoWriter_fourcc(*"mp4v"),
                                             fps, (frame.shape[1], frame.shape[0]))
                    rec_frames = 0
                    events.append(f"rec_start={rec_path.name}")
                    print(f"  録画開始: {rec_path} ({fps:.1f}fps で記録)")
                else:
                    writer.release()
                    writer = None
                    events.append("rec_stop")
                    print(f"  録画停止: {rec_path} ({rec_frames} フレーム)")
            key = 255
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
            print(f"  録画停止: {rec_path} ({rec_frames} フレーム)")
        if log is not None:
            log.close()
        cam.release()
        cv2.destroyAllWindows()

        n = max(1, totals["frames"])
        elapsed = time.time() - start
        print(f"[{a.label}] {totals['frames']} フレーム / {elapsed:.1f}s "
              f"({totals['frames'] / max(elapsed, 1e-6):.1f}fps)  "
              f"点滅確定ありのフレーム {totals['confirmed'] / n * 100:.1f}%  "
              f"振動棄却 {totals['rejected']}")
        if log is not None:
            print(f"  ログ: {log.path}")


if __name__ == "__main__":
    main()
