"""
scripts/merge_logs.py
BLE ログ (機体 125Hz) と PC 側のログ (テレメトリ / カメラ / 指令) を
1 本の時系列に並べ直して解析用の CSV にする。

==========================================================================
なぜ「時刻合わせ」が要るのか
==========================================================================
3 系統のログは、それぞれ別の時計で打たれている。

    LOGnnnn.BIN     機体の millis()      125Hz  (BLE 経由。t_ms 列)
    s5_link_*.csv   PC の time.time()    ~10Hz  (Epoch_s 列) + 機体 t_ms
    flight/mission/console/autopilot_*.csv  PC の time.time()  5〜60Hz

機体には RTC が無いので、BIN 単体では「西暦何時何分か」が分からない。
一方カメラのログは PC 時刻しか持たない。そのままでは並べられない。

★ 橋渡しになるのは s5_link_*.csv だけ。この 1 ファイルだけが
  「受信した瞬間の PC 時刻 (Epoch_s)」と「機体の millis (t_ms)」を
  同じ行に持っている (core/s5_link.py の _rx_loop)。
  ここから epoch = a * t_ms/1000 + b を作り、BIN の t_ms に適用する。

==========================================================================
素直にやると外す 3 点
==========================================================================
1. 無線の遅延は片側にしか出ない
   Epoch_s は「PC が受け取った時刻」なので、必ず 機体時刻 + 遅延。
   IM920sL は往復 100〜200ms で、混むほど遅れる。平均で引くと、その
   ログの混み具合ぶんだけ全体がずれる。→ 下側包絡 (遅延が最小だった
   パケット) を採用する。OFFSET_PCTL がその分位点。

2. テレメトリの t_ms は 655.36 秒で一周している
   下りパケットの時刻は 10ms 単位の uint16 (S5Telem)。地上局が展開して
   いるが、起点は「最初に受けたパケット」なので、真の millis() とは
   k * 655360 ms だけずれている (ground_receiver/src/tools/s5_log.cpp
   の unwrapTime)。→ BIN と重なるように k を選ぶ。

3. カメラの Time は「撮影時刻」ではない
   flight_*_perf.csv の FrameAge が示すとおり、取り込みから処理まで
   60〜130ms ある。→ 機体の高度とカメラの Z を相互相関にかけて実効
   遅延を測り、既定では差し引く (--vision-lag で固定もできる)。

==========================================================================
使い方
==========================================================================
    # 何も指定しなければ、既定の置き場から「最新の 1 組」を拾う
    python scripts/merge_logs.py

    # BLE(機体125Hz)を録っていない / 要らないとき (2系統だけ統合)
    #  ★ main.py だけで飛ばした場合はこちら。テレメトリ(s5_link)自身の
    #    Epoch_s (PCのtime.time()) を軸に、カメラ/ミッションを合わせる。
    #    機体側の時計合わせ (ドリフト・ラップ・相互相関) が丸ごと不要になる。
    python scripts/merge_logs.py --no-ble

    # 明示する
    python scripts/merge_logs.py --bin ../log_recorder/scripts/logs/LOG0009.BIN \\
        --telem ../position_estimator/src/logs/s5_link_20260914_121314.csv \\
        --vision ../position_estimator/src/logs/flight_20260914_121320.csv \\
        --mission ../position_estimator/src/logs/mission_20260914_121320.csv \\
        --plot

    # 時刻合わせだけ見たい (CSV を書かない)
    python scripts/merge_logs.py --dry-run

出力
    <telem/binの名前>_merged.csv  既定では position_estimator/src/logs/ に置く
                 (他のログと同じ場所。-o で変更可)
                 BINありのとき: 1行=BINの1レコード (既定125Hz)。
                     epoch_s / t_rel_s / 機体の全列 / tl_*(テレメトリ) /
                     cam_*(カメラ) / ms_*(ミッション) / cmd_*(コンソール)
                 --no-ble のとき: 1行=テレメトリの1レコード (10〜15Hz)。
                     epoch_s / t_rel_s / テレメトリの全列 /
                     cam_*(カメラ) / ms_*(ミッション) / cmd_*(コンソール)
    画面         時刻合わせの根拠 (ドリフト・最小遅延・相互相関のピーク)

    pip install numpy   (--plot には matplotlib も)
"""

import argparse
import csv
import importlib.util
import re
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
PE_LOGS = REPO / "position_estimator" / "src" / "logs"
BLE_LOGS = REPO / "log_recorder" / "scripts" / "logs"

# 機体時計 <-> PC 時計のオフセットに使う分位点 [%]。
# 0 に近いほど「最速で届いた 1 発」を信じる = 外れ値に弱い。5% 前後が妥当。
OFFSET_PCTL = 5.0

# 相互相関で探す範囲 [s] と、リサンプルの刻み [s]
XCORR_WINDOW_S = 2.0
XCORR_DT_S = 0.02
# 相関がこれ未満なら「合っている証拠なし」として採用しない
XCORR_MIN_CORR = 0.30
# 整合チェックのずれがこれを超えたら、回帰を信じずに相互相関で寄せる [s]。
# テレメトリの時刻は 10ms 刻み (S5Telem の t_cs) なので、数十 ms は誤差。
XCORR_APPLY_S = 0.05

# 最近傍で貼り付けるときの許容 [s] (これを超えたら空欄にする)
TOL_TELEM_S = 0.20     # テレメトリ 10〜15Hz
TOL_VISION_S = 0.10    # カメラ 30〜60Hz
TOL_EVENT_S = 0.30     # ミッション/コンソール 5Hz

MS_WRAP = 655360       # S5Telem の t_cs (10ms uint16) が一周する ms


# ==========================================================================
#  読み込み
# ==========================================================================
class Table:
    """CSV 1 枚。行は文字列のまま持ち、必要な列だけ float にして使う。

    ★ 文字列のまま持つのは、出力がそのまま「元のログの値」になるように
      するため。数値化して書き戻すと、桁が勝手に変わって元ログとの
      目視 diff が取れなくなる。
    """

    def __init__(self, names, rows, epoch=None, source=""):
        self.names = list(names)
        self.rows = rows
        self.epoch = epoch            # np.array [s] or None
        self.source = source
        self._cache = {}

    def __len__(self):
        return len(self.rows)

    def col(self, name):
        """float の np.array。数値でないセルは NaN。"""
        if name in self._cache:
            return self._cache[name]
        if name not in self.names:
            return None
        i = self.names.index(name)
        out = np.full(len(self.rows), np.nan)
        for r, row in enumerate(self.rows):
            if i < len(row):
                try:
                    out[r] = float(row[i])
                except (ValueError, TypeError):
                    pass
        self._cache[name] = out
        return out


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def load_fc_log(path, hover_thr=None, force=False):
    """LOGnnnn.BIN (または変換済み CSV) を読む。列は bin2csv の HEADER。"""
    b2c = _load_module("s5_bin2csv", HERE / "bin2csv.py")
    path = Path(path)
    if path.suffix.lower() == ".bin":
        rows, meta = b2c.decode_bin(
            path, hover_thr if hover_thr is not None else b2c.DEFAULT_HOVER_THR,
            force)
        names = b2c.HEADER.split(",")
        return Table(names, [r.split(",") for r in rows], source=str(path)), meta

    with path.open(encoding="utf-8") as f:
        rows = list(csv.reader(f))
    if not rows:
        raise ValueError(f"空のファイルです: {path}")
    return Table(rows[0], rows[1:], source=str(path)), {"path": path, "n_rec": len(rows) - 1}


def load_telemetry(path):
    """s5_link_*.csv を読む。'HEADER,' 行で列を決め、'DATA,' 行だけ拾う。"""
    path = Path(path)
    names, rows = None, []
    with path.open(encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.rstrip("\r\n")
            if line.startswith("HEADER,"):
                new = line[len("HEADER,"):].split(",")
                if names is not None and new != names:
                    # 受信機のファーム更新をまたいだファイル。混ぜると列がずれる。
                    print("[WARN] HEADER が途中で変わりました。後半を無視します。")
                    break
                names = new
            elif line.startswith("DATA,") and names is not None:
                vals = line[len("DATA,"):].split(",")
                if len(vals) == len(names):
                    rows.append(vals)

    if names is None:
        raise ValueError(
            f"{path} に HEADER 行がありません。\n"
            "  s5_link.py が書いた s5_link_*.csv を指定してください "
            "(旧 s5_logger.py の CSV には PC 時刻が入っていないため使えません)")
    if "Epoch_s" not in names:
        raise ValueError(f"{path} に Epoch_s 列がありません (古い形式)。")

    t = Table(names, rows, source=str(path))
    t.epoch = t.col("Epoch_s")
    return t


def _date_from_name(path):
    """flight_20260914_121320.csv -> その日の 0 時 (時刻だけの Time 列用)。"""
    m = re.search(r"(\d{8})_(\d{6})", Path(path).name)
    if not m:
        return None
    return datetime.strptime(m.group(1), "%Y%m%d")


def load_pc_csv(path):
    """PC 側の CSV (flight / mission / console / autopilot) を読む。

    Epoch_s があればそれを使う。無ければ Time 列から復元する:
      "2026-09-06T21:02:41.330" -> そのまま
      "21:02:41.085"            -> ファイル名の日付と合わせる
    """
    path = Path(path)
    with path.open(encoding="utf-8", errors="ignore") as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        raise ValueError(f"中身がありません: {path}")
    names, body = rows[0], rows[1:]
    t = Table(names, body, source=str(path))

    if "Epoch_s" in names:
        t.epoch = t.col("Epoch_s")
        return t

    if "Time" not in names:
        raise ValueError(f"{path} に Epoch_s も Time もありません。")

    day = _date_from_name(path)
    i = names.index("Time")
    ep = np.full(len(body), np.nan)
    for r, row in enumerate(body):
        if i >= len(row) or not row[i]:
            continue
        s = row[i]
        try:
            if "T" in s:
                ep[r] = datetime.fromisoformat(s).timestamp()
            elif day is not None:
                hh, mm, ss = s.split(":")
                ep[r] = (day.timestamp() + int(hh) * 3600 + int(mm) * 60 + float(ss))
        except (ValueError, TypeError):
            pass
    if not np.isfinite(ep).any():
        raise ValueError(f"{path} の Time 列を解釈できませんでした。")
    t.epoch = ep
    print(f"[WARN] {path.name} に Epoch_s がありません。Time 列から復元しました "
          "(ローカル時刻として解釈。夏時間や日付跨ぎには注意)")
    return t


# ==========================================================================
#  時刻合わせ
# ==========================================================================
def fit_clock(t_ms, epoch):
    """機体 t_ms [ms] と PC epoch [s] から epoch = a*(t_ms/1000) + b を作る。

    ★ 素の最小二乗ではなく「下側包絡」に当てる。Epoch_s には無線と USB の
      遅延が片側にだけ乗っているので、平均に当てるとその飛行の混み具合
      ぶんだけ全体が遅れる方向へずれる。傾き (= 水晶のドリフト) も、
      遅延のばらつきをそのまま食わせると数十 ppm 平気で振れるので、
      「速く届いた便だけ」を残して数回引き直す。

    ★ それでも残る誤差がある: 片道の **最小** 遅延そのものは、この
      ログからは分離できない (往復を測っていないため)。したがって
      b は「真の時刻 + 最小遅延」であって、全体が数十 ms 遅い方向へ
      揃っている。相対的な時間差の解析には効かないが、絶対時刻として
      使うときは頭に入れておくこと。

    Returns: dict(a, b, drift_ppm, drift_se_ppm, delay_*, n, n_used)
    """
    ok = np.isfinite(t_ms) & np.isfinite(epoch)
    x = t_ms[ok] / 1000.0
    y = epoch[ok]
    if len(x) < 20:
        raise ValueError(f"時刻合わせに使える行が {len(x)} 行しかありません")

    a, b = np.polyfit(x, y, 1)

    # ★ 「残差の小さい点だけ残して引き直す」を繰り返すと、点が全部ログの
    #   一部分に片寄って傾きが暴れる (実測: 821 行 -> 14 行に潰れ、傾きが
    #   符号ごと外れた)。そうではなく、時間で等分した窓ごとに「その窓で
    #   最も速く届いた便」を 1 点ずつ拾い、その点列に線を当てる。
    #   点数は窓の数だけだが、必ず全長にわたって散るので傾きが安定する。
    n_bins = int(np.clip(len(x) // 20, 6, 24))
    edges = np.linspace(x.min(), x.max(), n_bins + 1)
    env_x, env_y = [], []
    for i in range(n_bins):
        m = (x >= edges[i]) & (x <= edges[i + 1])
        if m.sum() < 3:
            continue
        d = y[m] - (a * x[m] + b)
        j = int(np.argmin(d))
        env_x.append(x[m][j])
        env_y.append(y[m][j])

    if len(env_x) >= 4:
        env_x = np.array(env_x)
        env_y = np.array(env_y)
        a, b = np.polyfit(env_x, env_y, 1)
        env_res = env_y - (a * env_x + b)
        denom = np.std(env_x) * np.sqrt(len(env_x))
        se_ppm = float(np.std(env_res) / denom * 1e6) if denom > 0 else float("nan")
        n_used = len(env_x)
    else:
        # 窓を作れないほど短い/疎なログ。傾きは 1 に固定し、切片だけ取る。
        a = 1.0
        b = float(np.percentile(y - x, OFFSET_PCTL))
        se_ppm = float("nan")
        n_used = 0

    resid = y - (a * x + b)
    shift = np.percentile(resid, OFFSET_PCTL)
    b += shift
    resid_lo = resid - shift            # 0 が「最速で届いた便」

    return {"a": float(a), "b": float(b),
            # 機体の millis が PC 時計よりどれだけ速く進むか。
            # a = dEpoch/dt_fc なので、a<1 (= PC 秒より機体 ms が多い) が「速い」。
            "fc_fast_ppm": float((1.0 / a - 1.0) * 1e6),
            "drift_se_ppm": se_ppm,
            "delay_med_s": float(np.median(resid_lo)),
            "delay_p95_s": float(np.percentile(resid_lo, 95)),
            "span_s": float(x.max() - x.min()),
            "n": int(len(x)), "n_used": n_used}


def resolve_wrap(fc_t_ms, tel_t_ms, fit):
    """テレメトリ側 t_ms の 655.36 秒ラップぶん k を決める。

    BIN とテレメトリは同じ飛行なので、時間窓が重なる k がただ 1 つある。
    """
    fc_mid = float(np.nanmedian(fc_t_ms))
    tel_mid = float(np.nanmedian(tel_t_ms))
    k = int(round((fc_mid - tel_mid) / MS_WRAP))
    return k


def to_epoch(fc_t_ms, fit, k):
    """機体 millis -> PC epoch。k はテレメトリ側ラップの補正。"""
    return fit["a"] * ((fc_t_ms - k * MS_WRAP) / 1000.0) + fit["b"]


def xcorr_lag(t1, v1, t2, v2, window_s=XCORR_WINDOW_S, dt=XCORR_DT_S):
    """信号 2 が信号 1 より **どれだけ遅れているか** [s] を相互相関で測る。

    返り値 lag > 0 なら「2 の方が lag だけ遅い」。t2 から lag を引くと
    1 に揃う。重なりが無い / 変化が無いときは (None, 0.0)。
    """
    ok1 = np.isfinite(t1) & np.isfinite(v1)
    ok2 = np.isfinite(t2) & np.isfinite(v2)
    if ok1.sum() < 10 or ok2.sum() < 10:
        return None, 0.0
    t1, v1, t2, v2 = t1[ok1], v1[ok1], t2[ok2], v2[ok2]

    lo = max(t1.min(), t2.min()) + window_s
    hi = min(t1.max(), t2.max()) - window_s
    if hi - lo < 3.0:                      # 3 秒も重ならないなら測れない
        return None, 0.0

    grid = np.arange(lo, hi, dt)
    s1 = np.interp(grid, t1, v1)
    s2 = np.interp(grid, t2, v2)
    s1 -= s1.mean()
    s2 -= s2.mean()
    if s1.std() < 1e-9 or s2.std() < 1e-9:  # 平らな信号では合わせようがない
        return None, 0.0
    s1 /= s1.std() * len(s1)
    s2 /= s2.std()

    n = int(window_s / dt)
    lags = np.arange(-n, n + 1)
    corr = np.array([np.dot(s1, np.roll(s2, int(l))) for l in lags])
    i = int(np.argmax(np.abs(corr)))
    # np.roll(s2, +l) は s2 を「さらに遅らせる」ので、ピークが負側に立つ
    # ときに「2 の方が遅い」。符号を返り値の定義 (2 の遅れ) に直す。
    return float(-lags[i] * dt), float(corr[i])


def pick_xcorr(fc, fc_epoch, other, other_epoch, pairs):
    """使える信号の組を順に試して、最初に相関の立った組を返す。"""
    for fc_name, ot_name in pairs:
        v1 = fc.col(fc_name)
        v2 = other.col(ot_name)
        if v1 is None or v2 is None:
            continue
        lag, corr = xcorr_lag(fc_epoch, v1, other_epoch, v2)
        if lag is None:
            continue
        if abs(corr) >= XCORR_MIN_CORR:
            return {"signal": f"{fc_name} <-> {ot_name}", "lag_s": lag, "corr": corr}
    return None


# ==========================================================================
#  結合
# ==========================================================================
def nearest_join(base, src, tol):
    """base の各時刻に一番近い src の行番号と、許容内かどうか。"""
    if len(src) == 0:
        return np.zeros(len(base), dtype=int), np.zeros(len(base), dtype=bool)
    order = np.argsort(src)
    s = src[order]
    idx = np.searchsorted(s, base)
    idx = np.clip(idx, 1, len(s) - 1)
    left, right = s[idx - 1], s[idx]
    pick = np.where(base - left <= right - base, idx - 1, idx)
    if len(s) == 1:
        pick = np.zeros(len(base), dtype=int)
    ok = np.abs(s[pick] - base) <= tol
    return order[pick], ok


def merge(base, base_epoch, sources, out_path, decimate=1):
    """base (機体ログ) を軸に、各ソースを最近傍で貼り付けて 1 枚にする。"""
    names = ["epoch_s", "t_rel_s", "wall_time"] + list(base.names)
    joins = []
    for prefix, table, tol in sources:
        idx, ok = nearest_join(base_epoch, table.epoch, tol)
        joins.append((prefix, table, idx, ok))
        names += [f"{prefix}{n}" for n in table.names] + [f"{prefix}dt_s"]

    t0 = float(np.nanmin(base_epoch))
    n_written = 0
    with Path(out_path).open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(names)
        for r in range(0, len(base), decimate):
            ep = base_epoch[r]
            row = [f"{ep:.3f}", f"{ep - t0:.3f}",
                   datetime.fromtimestamp(ep).strftime("%H:%M:%S.%f")[:-3]]
            row += base.rows[r]
            for prefix, table, idx, ok in joins:
                if ok[r]:
                    src = table.rows[idx[r]]
                    row += list(src) + [f"{table.epoch[idx[r]] - ep:+.3f}"]
                else:
                    row += [""] * (len(table.names) + 1)
            w.writerow(row)
            n_written += 1
    return n_written, names


# ==========================================================================
#  入力の自動選択
# ==========================================================================
def newest(globs):
    cands = []
    for d, pat in globs:
        if Path(d).is_dir():
            cands += list(Path(d).glob(pat))
    if not cands:
        return None
    return max(cands, key=lambda p: p.stat().st_mtime)


# ==========================================================================
#  本体
# ==========================================================================
def main():
    ap = argparse.ArgumentParser(
        description="BLE ログ (機体 125Hz) と PC 側ログを時刻で突き合わせて 1 枚にする")
    ap.add_argument("--bin", dest="binpath",
                    help="LOGnnnn.BIN か、bin2csv 済みの CSV (既定: 最新。"
                         "BLEを録っていないときは --no-ble で明示的に外す)")
    ap.add_argument("--no-ble", action="store_true",
                    help="BLE(機体125Hz)ログを使わない。テレメトリ(s5_link)自身の"
                         "Epoch_sを軸にカメラ/ミッション/指令だけ統合する"
                         "(2系統だけのときはこちら。main.py だけで飛ばした"
                         "ログはこれで十分揃う)")
    ap.add_argument("--telem", help="s5_link_*.csv (時刻の軸。既定: 最新)")
    ap.add_argument("--vision", help="flight_*.csv / autopilot_*.csv (既定: 最新)")
    ap.add_argument("--mission", help="mission_*.csv (既定: 最新)")
    ap.add_argument("--cmd", help="console_*.csv (既定: 最新)")
    ap.add_argument("-o", "--out",
                    help="出力 CSV (既定: position_estimator/src/logs/ に "
                         "<テレメトリ名>_merged.csv として置く)")
    ap.add_argument("--decimate", type=int, default=1,
                    help="機体ログを N 行に 1 行へ間引く (既定 1 = 125Hz のまま。"
                         "--no-ble のときは無視)")
    ap.add_argument("--vision-lag", default="auto",
                    help="カメラの実効遅延 [ms]。auto = 相互相関で測る / 0 = 補正しない")
    ap.add_argument("--vision-signal",
                    help="カメラ遅延を測る信号の組 '軸側の列:カメラ列' "
                         "(既定: est_h:Pos_Z(m) などを順に試す。--no-ble のときは"
                         " range_h:Pos_Z(m) などテレメトリ由来の列を試す)")
    ap.add_argument("--no-xcorr", action="store_true",
                    help="相互相関による微調整をせず、テレメトリの回帰だけで合わせる"
                         "(--no-ble のときは無関係)")
    ap.add_argument("--hover-thr", type=float, default=None,
                    help="bin2csv の alt_base 列に使うホバースロットル")
    ap.add_argument("--force", action="store_true", help="rec_ver 不一致でも強行する")
    ap.add_argument("--dry-run", action="store_true", help="時刻合わせだけ表示して終わる")
    ap.add_argument("--plot", action="store_true", help="合わせ具合を確認する図を出す"
                    " (--no-ble のときは出ません)")
    args = ap.parse_args()

    # ★ 明示されたファイルの存在は、何よりも先に見る。自動選択の後ろで
    #   見ていると「--bin の打ち間違い」が「テレメトリが無い」と報告される。
    for label, given in (("--bin", args.binpath), ("--telem", args.telem),
                         ("--vision", args.vision), ("--mission", args.mission),
                         ("--cmd", args.cmd)):
        if given and not Path(given).exists():
            sys.exit(f"[ERROR] {label} のファイルがありません: {given}")

    # ---- 入力を決める -------------------------------------------------
    telem = args.telem or newest([(PE_LOGS, "s5_link_*.csv")])
    if telem is None:
        sys.exit("[ERROR] s5_link_*.csv が見つかりません。--telem で指定してください。\n"
                 "        console.py か main.py を1回動かして作ってください。")

    binpath = None
    if not args.no_ble:
        binpath = args.binpath or newest([(BLE_LOGS, "LOG*.BIN"), (PE_LOGS, "LOG*.BIN")])
        if binpath is None and args.binpath is None:
            print("[INFO] 機体ログ (LOG*.BIN) が見つからないので --no-ble 相当で"
                  "続けます (テレメトリ自身のEpoch_sを軸にします)。")
            print("       BLEログも録るなら log_recorder/scripts/ble_receiver.py を"
                  "並行して起動しておいてください。\n")

    vision = args.vision or newest([(PE_LOGS, "flight_*.csv"),
                                    (PE_LOGS.parent.parent / "logs", "autopilot_*.csv")])
    if vision and Path(vision).name.endswith(("_perf.csv", "_display_perf.csv")):
        vision = None                      # perf ログは中身が別物なので拾わない
    mission = args.mission or newest([(PE_LOGS, "mission_*.csv")])
    cmdlog = args.cmd or newest([(PE_LOGS, "console_*.csv")])

    print("=" * 74)
    print(" merge_logs  -  入力")
    print("=" * 74)
    print(f"  機体 (BLE)   : {binpath or '(なし。テレメトリを軸にします)'}")
    print(f"  テレメトリ   : {telem}")
    print(f"  カメラ       : {vision or '(なし)'}")
    print(f"  ミッション   : {mission or '(なし)'}")
    print(f"  指令         : {cmdlog or '(なし)'}")
    print()

    tel = load_telemetry(telem)
    tel_t = tel.col("t_ms")
    if tel_t is None:
        sys.exit("[ERROR] テレメトリに t_ms 列がありません。")

    if binpath is not None:
        base, base_epoch, xcorr_ref, out_default = _prepare_with_bin(
            args, binpath, tel, tel_t)
    else:
        base, base_epoch, xcorr_ref, out_default = _prepare_pc_only(tel, telem)

    # ---- カメラの遅延 ---------------------------------------------------
    sources = []
    if binpath is not None:
        sources.append(("tl_", tel, TOL_TELEM_S))   # --no-ble ではテレメトリ自身が軸
    vis = None
    vis_lag = 0.0
    if vision:
        vis = load_pc_csv(vision)
        ref_table, ref_epoch = xcorr_ref
        if args.vision_lag == "auto":
            if args.vision_signal:
                ref_name, _, cam_name = args.vision_signal.partition(":")
                pairs = [(ref_name.strip(), cam_name.strip())]
            elif binpath is not None:
                pairs = [("est_h", "Pos_Z(m)"), ("range_h", "Pos_Z(m)"),
                         ("est_h", "Cam_Z(m)"), ("range_h", "Cam_Z(m)")]
            else:
                # --no-ble: 軸はテレメトリなので est_h (BINだけの列) は無い
                pairs = [("range_h", "Pos_Z(m)"), ("roll", "Pos_Z(m)"),
                         ("range_h", "Cam_Z(m)")]
            cam = pick_xcorr(ref_table, ref_epoch, vis, vis.epoch, pairs)
            if cam is not None:
                # lag>0 = カメラのログの方が遅れている = 引く
                vis_lag = cam["lag_s"]
                print(f"  カメラ遅延   : {vis_lag * 1000:+.0f} ms  "
                      f"({cam['signal']}  corr {cam['corr']:+.2f}) -> 差し引きます")
                if binpath is not None:
                    print("               ※ 機体ログの時間軸に対する遅れです。機体側の"
                          "時間軸自体が無線の片道最小遅延ぶん遅れているので、")
                    print("                 撮影から処理までの本当の遅れは、この値 + "
                          "その最小遅延 (数十 ms) になります。")
                if cam["corr"] < 0:
                    print("               ※ 相関が負。カメラの Z 軸の符号が"
                          "機体の高度と逆向きです (取り付け/座標定義を確認)")
            else:
                print("  カメラ遅延   : 測れません (高度が動いていない / 列名が違う?)。"
                      "0 とします")
                print("                 --vision-signal '軸の列:カメラ列' の形で "
                      "合わせる信号を指定できます")
        else:
            vis_lag = float(args.vision_lag) / 1000.0
            print(f"  カメラ遅延   : {vis_lag * 1000:+.0f} ms (--vision-lag 指定)")
        vis.epoch = vis.epoch - vis_lag
        sources.append(("cam_", vis, TOL_VISION_S))

    if mission:
        sources.append(("ms_", load_pc_csv(mission), TOL_EVENT_S))
    if cmdlog:
        sources.append(("cmd_", load_pc_csv(cmdlog), TOL_EVENT_S))

    if args.plot:
        if binpath is not None:
            _plot(base, base_epoch, tel, tel.epoch, vis)
        else:
            print("[INFO] --no-ble では --plot は出ません (機体125Hzの波形が無いため)")

    if args.dry_run:
        print("\n[dry-run] CSV は書きませんでした。")
        return 0

    out = Path(args.out) if args.out else out_default
    decimate = max(1, args.decimate) if binpath is not None else 1
    n, names = merge(base, base_epoch, sources, out, decimate)
    print("=" * 74)
    print(f"[OK] {n} 行 / {len(names)} 列 -> {out}")
    if binpath is not None:
        print("     列の頭: epoch_s (PC時刻) / t_rel_s (先頭からの秒) / 機体の列 / "
              "tl_(テレメトリ) cam_(カメラ) ms_(ミッション) cmd_(指令)")
        print("     *_dt_s は「貼り付けた行が何秒ずれていたか」。大きい行は参考値。")
        print()
        print("  ※ 絶対時刻は「真の時刻 + 無線の片道最小遅延」。往復を測っていない")
        print("    ので数十 ms ぶん全体が遅い方向に揃っています。ログどうしの相対")
        print("    比較には効きませんが、絶対時刻として使うときは頭に入れてください。")
    else:
        print("     列の頭: epoch_s (PC時刻) / t_rel_s (先頭からの秒) / テレメトリの列 / "
              "cam_(カメラ) ms_(ミッション) cmd_(指令)")
        print("     *_dt_s は「貼り付けた行が何秒ずれていたか」。大きい行は参考値。")
        print()
        print("  ※ テレメトリの Epoch_s は PC の受信時刻なので、無線の遅延ぶん"
              "実際の機体挙動よりわずかに遅い (数十〜200ms)。BLEを使う場合ほど"
              "厳密ではないが、カメラ/ミッションとの相対比較には十分。")
    return 0


def _prepare_with_bin(args, binpath, tel, tel_t):
    """機体125Hzログ(BIN)を軸にする、従来の時刻合わせ一式。

    Returns: (base_table, base_epoch, (xcorr参照table, xcorr参照epoch), 既定出力先)
    """
    fc, meta = load_fc_log(binpath, args.hover_thr, args.force)
    fc_t = fc.col("t_ms")

    # ---- 1) テレメトリから時計を作る ----------------------------------
    try:
        fit = fit_clock(tel_t, tel.epoch)
    except ValueError as e:
        sys.exit(f"[ERROR] {e}")

    # ---- 2) ラップを解く ----------------------------------------------
    k = resolve_wrap(fc_t, tel_t, fit)
    fc_epoch = to_epoch(fc_t, fit, k)
    tel_epoch = tel.epoch

    # ---- 3) 相互相関で「合っているか」を確かめる ------------------------
    #  ★ 相手は Epoch_s (受信時刻) ではなく、テレメトリ自身の t_ms を
    #    同じ式で写した時刻。受信時刻に合わせてしまうと、せっかく
    #    下側包絡で落とした無線遅延をもう一度足し込むことになる。
    #    どちらも機体の millis を写しただけなので、ここは 0 になるのが
    #    正常。大きくずれたら k か回帰が外れているので、そのとき初めて
    #    採用する。
    tel_mapped = fit["a"] * (tel_t / 1000.0) + fit["b"]
    xc = None
    if not args.no_xcorr:
        xc = pick_xcorr(fc, fc_epoch, tel, tel_mapped,
                        [("range_h", "range_h"), ("roll_ang", "roll"),
                         ("pitch_ang", "pitch"), ("thr", "thr"), ("m1", "m1")])
        if xc is not None and abs(xc["lag_s"]) >= XCORR_APPLY_S:
            fc_epoch = fc_epoch + xc["lag_s"]
            xc["applied"] = True

    overlap_lo = max(np.nanmin(fc_epoch), np.nanmin(tel_epoch))
    overlap_hi = min(np.nanmax(fc_epoch), np.nanmax(tel_epoch))

    print("=" * 74)
    print(" 時刻合わせ")
    print("=" * 74)
    print(f"  テレメトリ   : {fit['n']} 行 (下側包絡 {fit['n_used']} 点で回帰) / "
          f"{fit['span_s']:.1f} s")
    print(f"  機体時計     : PC より {fit['fc_fast_ppm']:+.0f} ± {fit['drift_se_ppm']:.0f} ppm 速い "
          f"(この窓の端で ±{fit['drift_se_ppm'] * 1e-6 * fit['span_s'] * 1000:.0f} ms 相当)")
    print(f"  無線+USB 遅延: 中央 {fit['delay_med_s'] * 1000:.0f} ms / "
          f"95% {fit['delay_p95_s'] * 1000:.0f} ms  "
          f"(最速便を 0 とした相対値)")
    print(f"  ラップ補正   : k={k}  ({k * MS_WRAP / 1000:.2f} s)")
    if xc is not None and xc.get("applied"):
        print(f"  整合チェック : {xc['signal']}  ずれ {xc['lag_s'] * 1000:+.0f} ms  "
              f"corr {xc['corr']:+.2f}  -> ★ 大きいので補正しました "
              "(k か回帰を疑ってください)")
    elif xc is not None:
        print(f"  整合チェック : {xc['signal']}  ずれ {xc['lag_s'] * 1000:+.0f} ms  "
              f"corr {xc['corr']:+.2f}  -> OK (±{XCORR_APPLY_S * 1000:.0f} ms 以内)")
    elif args.no_xcorr:
        print("  整合チェック : --no-xcorr のため未実施")
    else:
        print(f"  整合チェック : 相関 {XCORR_MIN_CORR} 未満で確認できず "
              "(機体が動いていないログだと普通)")
    if overlap_hi > overlap_lo:
        print(f"  重なり       : "
              f"{datetime.fromtimestamp(overlap_lo).strftime('%Y-%m-%d %H:%M:%S')} "
              f"〜 {datetime.fromtimestamp(overlap_hi).strftime('%H:%M:%S')}  "
              f"({overlap_hi - overlap_lo:.1f} s)")
    else:
        print("  重なり       : ★ ありません。別々の飛行のログを混ぜていませんか?")
    print(f"  機体ログ     : {len(fc)} 行  "
          f"{(np.nanmax(fc_t) - np.nanmin(fc_t)) / 1000.0:.1f} s")
    print()

    out_default = PE_LOGS / (Path(binpath).stem + "_merged.csv")
    return fc, fc_epoch, (fc, fc_epoch), out_default


def _prepare_pc_only(tel, telem):
    """--no-ble (2系統だけ) のとき: テレメトリ自身の Epoch_s をそのまま軸にする。

    BINが無い = 時計合わせの回帰そのものが要らない (テレメトリはPCの
    time.time()で打刻済み)。カメラ/ミッション/指令ログも同じ time.time()
    なので、Epoch_s どうしの最近傍合わせだけで十分。
    """
    print("=" * 74)
    print(" 時刻合わせ (--no-ble: テレメトリの Epoch_s をそのまま使用)")
    print("=" * 74)
    print(f"  テレメトリ   : {len(tel)} 行  "
          f"{(np.nanmax(tel.epoch) - np.nanmin(tel.epoch)):.1f} s")
    print("  ★ PCの time.time() で打刻済みなので、機体時計との突き合わせは不要。"
          "無線の受信遅延(数十〜200ms)ぶんだけ実際の機体挙動より遅れている。")
    print()

    out_default = PE_LOGS / (Path(telem).stem + "_merged.csv")
    return tel, tel.epoch, (tel, tel.epoch), out_default


def _plot(fc, fc_epoch, tel, tel_epoch, vis):
    """合わせ具合の目視確認。数字だけ見て納得しないための図。"""
    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib が無いので --plot は飛ばします")
        return

    # 日本語フォント (無ければ豆腐になるが処理は通す)。plot_s5.py と同じ。
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for fam in ("Yu Gothic", "Meiryo", "MS Gothic", "Noto Sans CJK JP"):
        if fam in have:
            matplotlib.rcParams["font.family"] = fam
            break
    matplotlib.rcParams["axes.unicode_minus"] = False

    t0 = float(np.nanmin(fc_epoch))
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    h_fc = fc.col("range_h")
    if h_fc is not None:
        ax[0].plot(fc_epoch - t0, h_fc, lw=0.8, label="機体 range_h (BLE 125Hz)")
    h_tel = tel.col("range_h")
    if h_tel is not None:
        ax[0].plot(tel_epoch - t0, h_tel, ".", ms=3, label="テレメトリ range_h")
    if vis is not None:
        for name in ("Pos_Z(m)", "Cam_Z(m)"):
            z = vis.col(name)
            if z is not None and np.isfinite(z).any():
                ax[0].plot(vis.epoch - t0, z, ".", ms=3, label=f"カメラ {name}")
                break
    ax[0].set_ylabel("height [m]")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    ax[0].set_title("時刻合わせの確認: 同じ山が重なっていれば合っている")

    r_fc = fc.col("roll_ang")
    r_tel = tel.col("roll")
    if r_fc is not None:
        ax[1].plot(fc_epoch - t0, r_fc, lw=0.8, label="機体 roll (BLE)")
    if r_tel is not None:
        ax[1].plot(tel_epoch - t0, r_tel, ".", ms=3, label="テレメトリ roll")
    ax[1].set_ylabel("roll [deg]")
    ax[1].set_xlabel("t [s] (先頭から)")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    sys.exit(main())
