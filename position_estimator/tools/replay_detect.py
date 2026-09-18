"""
tools/replay_detect.py
録画を Camera1 の検知系 (core/camera.py の detect -> core/blink.py) に流し、
実機を立ち上げずに検知率を測る。しきい値の調整はこれで回す。

    python tools/replay_detect.py "tests/画面録画 2026-09-16 164847.mp4"
    python tools/replay_detect.py <video> --truth 527,384 --label Camera1 --out annotated.mp4

tests/ の録画ごとの引数 (窓の大きさが違うので --crop も変わる):
    2026-09-16 164847  Camera1  (既定値のまま)
    2026-09-17 093651  Camera1  --crop 15,55,1454,917 --truth 1127,544
        α6400 が 1/15s の自動露出。機体は床に静置、右下。
    2026-09-17 100801  Camera1  --crop 15,55,1454,917 --truth-px 25
                                 --truth "tests/画面録画 2026-09-17 100801_truth.csv"
        同じ露出で 0.8m/s 以下で 30 秒動き続ける。正解はフレームごとの CSV。
        「床の反射」(機体の真下に映る LED) は誤検知とは別に数える。
    2026-09-17 093721  Camera2  --label Camera2 --crop 1,55,1422,917 --truth 203,497
                                 --ignore "0,0,520,140;0,540,900,720"
        RPi のプレビュー (640x360 を拡大) で、左の人が手を振る。窓に RPi が描いた
        緑丸・当時のスコア文字が写り込んでいるので、誤検知の目安にしかならない。

入力は 2 種類:
  ・cv2.imshow("Camera 1") の窓を画面録画したもの (既定)。
    窓のクライアント領域は DISP_W x DISP_H の画像を窓サイズに引き伸ばした
    ものなので、--crop で領域を切り出して 1280x720 に戻してから流す。
    既定の --crop は 2026-09-16 16:48 の録画 (1468x932, 窓 15..1455 x 55..917)。
  ・カメラの生映像 (--crop none)。

★ 画面録画には Camera1 窓のオーバーレイが写り込んでいる。実機では
  detect() は描き込み前の生フレームを受け取るので、これは録画にしか無い。
    ・ラベル "Camera1" / 状態行 "[DUMMY] X:.. Y:.." など位置が固定の文字:
      数値が毎フレーム変わるので静的マスクでは消えない。--ignore の矩形
      (既定は左上と左下の文字帯) を黒く塗って流す。
    ・DUMMY の機体アイコン: 動くので候補になるが、点滅しないので blink が落とす。
  α6400 自身の OSD (1/200, F3.5, ISO 100) はカメラ信号の一部なので残す。
  静的輝点マスクが覚える対象で、学習中 (static_mask_learn_frames) は
  候補に出るため、集計は学習完了後から始める。

★ 画面録画は表示より速い周期で撮られるので、同じ表示が 2 枚続くことがある
  (16:48 の録画では 30fps 中 1/3 が前フレームと同一)。実機では blink.py が
  同じ取得時刻のフレームを重複サンプルしないので、録画でも「前フレームと
  ほぼ同一」の枚は飛ばす (--dup-px)。飛ばさないと同じ輝度が 2 回入り、
  ロックインの位相が乱れてスコアが落ちる。

出力:
  フレームごとの候補数 / 点滅確認済み候補数 / 正解位置との距離を集計し、
  「学習完了後のフレームのうち、正解 ±truth_px 以内に確認済み候補が
  あった割合」を検知率として表示する。--out を付けると候補枠を描いた
  動画を書き出す。
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from core.camera import CameraTracker            # noqa: E402
from core.blink import BlinkTracker              # noqa: E402
from utils.config import (CAMERA_W, CAMERA_H, LED_BLINK_HZ,     # noqa: E402
                          blink_params_for)

DEFAULT_CROP = "15,55,1455,917"     # x0,y0,x1,y1  (2026-09-16 16:48 の録画)
# main_loop / tracker が窓に描く文字の帯 (1280x720 基準)。上: ラベルと DUMMY 表示、
# 下: YAW 行・[DUMMY] 座標行・キー案内。
DEFAULT_IGNORE = "0,0,300,80;0,600,460,720"
# 前フレームとの差が 20 階調を超える画素がこれ未満なら「同じ表示」とみなして飛ばす
DEFAULT_DUP_PX = 50
REFLECTION_DU_PX = 35
REFLECTION_DV_PX = 130


def _parse_crop(text):
    if text.lower() == "none":
        return None
    x0, y0, x1, y1 = (int(v) for v in text.split(","))
    return x0, y0, x1, y1


def _parse_rects(text):
    if not text or text.lower() == "none":
        return []
    return [tuple(int(v) for v in r.split(",")) for r in text.split(";") if r]


def _parse_xy(text):
    if not text:
        return None
    u, v = (float(v) for v in text.split(","))
    return u, v


def load_truth_csv(path):
    """動く機体の正解。行 "frame,u,v" (録画のフレーム番号) を、次の行のフレームまで有効として返す。"""
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("frame"):
                continue
            fr, u, v = line.split(",")
            rows.append((int(fr), float(u), float(v)))
    return rows


def _truth_fn(truth):
    """truth (None / (u, v) / CSV パス / load_truth_csv の結果) を「フレーム番号 -> (u, v) か None」に。
    CSV の最終行より後のフレームは正解無し (集計しない)。"""
    if truth is None:
        return None
    if isinstance(truth, (str, Path)):
        truth = load_truth_csv(truth)
    if isinstance(truth, tuple):
        return lambda i: truth
    rows = sorted(truth)
    frames = [r[0] for r in rows]
    last = frames[-1] + (frames[-1] - frames[-2] if len(frames) > 1 else 0)

    def at(i):
        import bisect
        k = bisect.bisect_right(frames, i) - 1
        if k < 0 or i > last:
            return None
        return rows[k][1], rows[k][2]
    return at


def replay(video, label="Camera1", crop=DEFAULT_CROP, truth=None, truth_px=12.0,
           fps=None, out=None, verbose=False, start_after_learn=True,
           ignore=DEFAULT_IGNORE, dup_px=DEFAULT_DUP_PX):
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise SystemExit(f"開けません: {video}")
    src_fps = fps or cap.get(cv2.CAP_PROP_FPS) or 30.0
    crop = _parse_crop(crop) if isinstance(crop, str) else crop
    ignore = _parse_rects(ignore) if isinstance(ignore, str) else (ignore or [])
    truth_at = _truth_fn(truth)

    cam = CameraTracker(None, width=CAMERA_W, height=CAMERA_H, label=label)
    blink = BlinkTracker(label, LED_BLINK_HZ, **blink_params_for(label))
    # flicker モードには静的輝点マスクが無く、学習待ちも無い
    learn_frames = cam.p["static_mask_learn_frames"] if cam._static_bright is not None else 0

    writer = None
    idx = 0
    stats = {"frames": 0, "scored": 0, "raw_hit": 0, "blink_hit": 0,
             "raw_cands": 0, "blink_cands": 0, "false_confirmed": 0,
             "rejected": 0, "duplicates": 0,
             "reflection_confirmed": 0,  # 正解の真下に写る床の反射 (本物の LED 像)。誤検知とは別に数える
             "false_best_score": 0.0}   # 正解以外のトラックが出した最高スコア (余裕の目安)
    first_hit = None
    per_frame = []
    prev_gray = None
    n_detect = 0          # detect() を呼んだ枚数。静的マスクの学習はこれで進む
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if crop is not None:
            x0, y0, x1, y1 = crop
            frame = frame[y0:y1, x0:x1]
        if frame.shape[1] != CAMERA_W or frame.shape[0] != CAMERA_H:
            frame = cv2.resize(frame, (CAMERA_W, CAMERA_H), interpolation=cv2.INTER_AREA)
        for x0, y0, x1, y1 in ignore:
            frame[y0:y1, x0:x1] = 0
        ts = 1000.0 + idx / src_fps
        idx += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if prev_gray is not None and dup_px is not None:
            changed = int(np.count_nonzero(cv2.absdiff(gray, prev_gray) > 20))
            if changed < dup_px:
                stats["duplicates"] += 1
                if writer is not None:
                    writer.write(vis)
                continue
        prev_gray = gray

        cands = cam.detect(frame, ts)
        n_detect += 1
        confirmed = blink.filter(frame, cands, ts, cam.width)
        truth = truth_at(idx - 1) if truth_at is not None else None

        def is_truth(c):
            return abs(c.u - truth[0]) <= truth_px and abs(c.v - truth[1]) <= truth_px

        def is_reflection(c):
            # 体育館の床に LED が映る。機体の真下 10〜130px (距離で変わる) に同じ点滅が出る。
            # 2 カメラの三角測量で床下になるので PairSelector が落とす。
            return abs(c.u - truth[0]) <= REFLECTION_DU_PX and 10 < c.v - truth[1] <= REFLECTION_DV_PX

        def near(cs):
            if truth is None:
                return bool(cs)
            return any(is_truth(c) for c in cs)

        scored = ((n_detect > learn_frames or not start_after_learn)
                  and (truth_at is None or truth is not None))
        stats["frames"] += 1
        if scored:
            stats["scored"] += 1
            stats["raw_cands"] += len(cands)
            stats["blink_cands"] += len(confirmed)
            stats["rejected"] += cam.vibration_rejected
            rh, bh = near(cands), near(confirmed)
            stats["raw_hit"] += rh
            stats["blink_hit"] += bh
            if truth is not None:
                stats["reflection_confirmed"] += sum(
                    1 for c in confirmed if not is_truth(c) and is_reflection(c))
                stats["false_confirmed"] += sum(
                    1 for c in confirmed if not is_truth(c) and not is_reflection(c))
                for t in blink._tracks:
                    if not is_truth(t.cand):
                        stats["false_best_score"] = max(stats["false_best_score"], t.score)
            if bh and first_hit is None:
                first_hit = idx
            per_frame.append((idx, len(cands), len(confirmed), rh, bh, *blink.best()))
            if verbose:
                print(f"{idx:4d} raw={len(cands):2d} blink={len(confirmed):2d} "
                      f"hit={int(rh)}/{int(bh)} best={blink.best()[0]:.2f}/{blink.best()[1]:.1f}")

        if out is not None:
            vis = frame.copy()
            blink.draw(vis, cam.width)
            cam.draw_candidates(vis, confirmed, 0 if confirmed else None)
            for c in cands:
                cv2.rectangle(vis, (c.x - 2, c.y - 2), (c.x + c.w + 2, c.y + c.h + 2),
                              (255, 0, 255), 1)
            if truth is not None:
                cv2.circle(vis, (int(truth[0]), int(truth[1])), int(truth_px), (0, 0, 255), 1)
            if writer is None:
                writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"),
                                         src_fps, (vis.shape[1], vis.shape[0]))
            writer.write(vis)

    cap.release()
    if writer is not None:
        writer.release()

    n = max(1, stats["scored"])
    print(f"[{label}] {video}")
    print(f"  フレーム {stats['frames']} (集計 {stats['scored']}, 静的マスク学習 {learn_frames}, "
          f"同一表示の重複 {stats['duplicates']} は飛ばした)  fps={src_fps:.1f}")
    print(f"  振動棄却: {stats['rejected']}  候補/フレーム: raw {stats['raw_cands'] / n:.2f}"
          f"  点滅確認 {stats['blink_cands'] / n:.2f}")
    if truth_at is not None:
        where = (f"({truth[0]:.0f},{truth[1]:.0f})" if isinstance(truth, tuple) and truth
                 else "(CSV)")
        print(f"  正解 {where} ±{truth_px:.0f}px:"
              f"  raw 候補あり {stats['raw_hit'] / n * 100:.1f}%"
              f"  点滅確認済み {stats['blink_hit'] / n * 100:.1f}%"
              f"  (初回確認 frame {first_hit})"
              f"  正解以外の確認済み候補 {stats['false_confirmed']} 個"
              f" (床の反射 {stats['reflection_confirmed']} 個は別)"
              f" (正解以外の最高スコア {stats['false_best_score']:.2f})")
    else:
        print(f"  候補あり {stats['raw_hit'] / n * 100:.1f}%  点滅確認済みあり "
              f"{stats['blink_hit'] / n * 100:.1f}%  (初回確認 frame {first_hit})")
    return stats, per_frame


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video")
    ap.add_argument("--label", default="Camera1")
    ap.add_argument("--crop", default=DEFAULT_CROP,
                    help="x0,y0,x1,y1 で窓の映像領域。生映像なら none")
    ap.add_argument("--ignore", default=DEFAULT_IGNORE,
                    help="黒く塗る矩形 x0,y0,x1,y1 を ; 区切りで。生映像なら none")
    ap.add_argument("--dup-px", type=int, default=DEFAULT_DUP_PX,
                    help="前フレームと同一とみなす変化画素数の上限。-1 で無効")
    ap.add_argument("--truth", default="527,384",
                    help="機体 LED の正解位置 u,v (1280x720 基準)、または動く機体用の CSV "
                         "(frame,u,v)。空なら集計しない")
    ap.add_argument("--truth-px", type=float, default=12.0)
    ap.add_argument("--fps", type=float, default=None, help="録画の fps を上書き")
    ap.add_argument("--out", default=None, help="候補を描いた動画の出力先")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    t0 = time.perf_counter()
    truth = a.truth if a.truth.lower().endswith(".csv") else _parse_xy(a.truth)
    replay(a.video, a.label, a.crop, truth, a.truth_px, a.fps, a.out, a.verbose,
           ignore=a.ignore, dup_px=None if a.dup_px < 0 else a.dup_px)
    print(f"  処理時間 {time.perf_counter() - t0:.1f}s")


if __name__ == "__main__":
    main()
