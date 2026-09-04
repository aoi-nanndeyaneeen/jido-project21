"""
analyze_poshold.py  -  s5 (POSHOLD) テレメトリ CSV の解析

    python tools/analyze_poshold.py                 # logs/ の最新を自動で開く
    python tools/analyze_poshold.py logs/s5_003_*.csv
    python tools/analyze_poshold.py --plot          # グラフも出す

s5_logger.py が保存した CSV (15Hz) を読んで、
  1. リンク品質 (欠落率・実効レート・RSSI)      … 数字を信用してよいかの判定
  2. 高度ホールド (ALT)                          … 誤差 / 上昇速度追従 / 権限飽和
  2b. 姿勢ループ / モーター (ATT)                … 離陸したか / 飽和 / 転倒 / I項の要否
  3. 水平位置ホールド (POS)                      … 位置ドリフト / 速度追従 / リーン飽和
  4. センサ健全性 (測距失探・フロー失探)
  5. 振動の周期                                  … P が高すぎるかどうか
を出し、ゲインをどちらへ動かすべきかを言葉で書く。

★ IM920sL の 32バイト制限のため、機体は A(高度) と B(水平) を交互に
  15Hz で送っている。つまり各ループの実効レートは 7.5Hz で、見えるのは
  3.7Hz まで (ナイキスト)。姿勢ループ (数十Hz) の発振はここでは分からない。
  それは USB を挿して flight_controller/scripts/logger.py の 500Hz ログで
  見ること。ここで扱うのは高度・位置ループ (0.1〜2Hz) の話。

必要パッケージ:
    pip install numpy
    pip install matplotlib   # --plot のときだけ
"""

import argparse
import sys
from pathlib import Path

import numpy as np

LOGS_DIR = Path(__file__).parent.parent / "logs"


# ============================================================
#  読み込み
# ============================================================
def latest_log():
    files = sorted(LOGS_DIR.glob("s5_*.csv"))
    if not files:
        sys.exit(f"[ERROR] {LOGS_DIR} に s5_*.csv がありません。"
                 "先に tools/s5_logger.py で記録してください。")
    return files[-1]


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    if d.size == 0:
        sys.exit(f"[ERROR] データ行がありません: {path}")
    if d.ndim == 0:                      # 1行だけのとき
        d = d.reshape(1)
    return d


def load_params(path):
    """s5_logger.py が書いた .params.txt を辞書のリストで返す。"""
    p = Path(path).with_suffix(".params.txt")
    if not p.exists():
        return []
    out = []
    for line in p.read_text(encoding="utf-8").splitlines():
        if not line.startswith("PARAM,"):
            continue
        rec = {}
        for kv in line[len("PARAM,"):].split(","):
            if "=" in kv:
                k, v = kv.split("=", 1)
                rec[k] = v
        out.append(rec)
    return out


def section(title):
    print()
    print("=" * 66)
    print(f"  {title}")
    print("=" * 66)


def stat_line(label, v, unit=""):
    if len(v) == 0:
        print(f"  {label:<22} (データなし)")
        return
    print(f"  {label:<22} 平均 {np.mean(v):+8.3f}  RMS {np.sqrt(np.mean(v**2)):7.3f}  "
          f"最大|{np.max(np.abs(v)):7.3f}| {unit}")


def dominant_freq(sig, fs):
    """信号の主要な振動周波数 [Hz] を返す。直流成分は落とす。"""
    n = len(sig)
    if n < 16:
        return None, 0.0
    x = sig - np.mean(sig)
    if np.allclose(x, 0):
        return None, 0.0
    w = np.hanning(n)
    sp = np.abs(np.fft.rfft(x * w))
    fr = np.fft.rfftfreq(n, d=1.0 / fs)
    sp[0] = 0.0                       # DC は無視
    k = int(np.argmax(sp))
    total = np.sum(sp)
    share = float(sp[k] / total) if total > 0 else 0.0
    return float(fr[k]), share


# ============================================================
#  1. リンク品質
# ============================================================
def check_link(d):
    section("1. 無線リンク (この数字が悪いと以下の解析も当てにならない)")
    rx = d["rx_ms"]
    n = len(rx)
    span_s = (rx[-1] - rx[0]) / 1000.0 if n > 1 else 0.0
    rate = (n - 1) / span_s if span_s > 0 else 0.0

    lost = d["lost"]
    lost_in_log = float(lost[-1] - lost[0]) if n > 1 else 0.0
    total = lost_in_log + n
    loss_pct = 100.0 * lost_in_log / total if total > 0 else 0.0

    print(f"  受信パケット   {n} 個 / {span_s:.1f} 秒  = {rate:.2f} Hz")
    print(f"  欠落 (seq飛び) {lost_in_log:.0f} 個  ({loss_pct:.1f} %)")

    if "rssi" in d.dtype.names:
        r = d["rssi"]
        r = r[r >= 0]
        if len(r):
            print(f"  RSSI           平均 {np.mean(r):.0f}  最小 {np.min(r):.0f}")

    gaps = np.diff(rx)
    if len(gaps):
        print(f"  受信間隔       中央値 {np.median(gaps):.0f} ms  "
              f"最大 {np.max(gaps):.0f} ms")
        long_gaps = np.sum(gaps > 500)
        if long_gaps:
            print(f"  → 0.5秒以上の途切れが {long_gaps} 回あります。")

    if loss_pct > 30:
        print("  → 欠落が多すぎます。drone_s5.cpp の S5::TELEM_TX_HZ を"
              " 15 → 12 → 10 と下げてください。")
        print("     (実測の上限は約19Hz。32バイト=71文字は19200bpsのUARTで37ms、"
              "15Hzで占有率55%)")
    elif loss_pct > 10:
        print("  → 欠落がやや多めです。波形の細部は信用しすぎないこと。")
    else:
        print("  → リンクは良好です。")

    if "tx_drop" in d.dtype.names and np.any(d["tx_drop"] > 0):
        cnt = int(np.sum(d["tx_drop"]))
        print(f"  ※ 機体側の送信バッファ詰まりが {cnt} 回。"
              "TELEM_TX_HZ が UART に対して速すぎるサインです。")

    return rate if rate > 0 else 10.0


# ============================================================
#  区間の切り出し
# ============================================================
def poshold_segments(d, min_len=20):
    """mode==POSHOLD(3) かつ ARMED の連続区間をスライスで返す。"""
    m = (d["mode"] == 3) & (d["armed"] > 0.5)
    segs, start = [], None
    for i, v in enumerate(m):
        if v and start is None:
            start = i
        elif not v and start is not None:
            if i - start >= min_len:
                segs.append(slice(start, i))
            start = None
    if start is not None and len(m) - start >= min_len:
        segs.append(slice(start, len(m)))
    return segs


# ============================================================
#  2. 高度ホールド
# ============================================================
def check_alt(d, seg, fs, thr_auth):
    act = d["alt_act"][seg] > 0.5
    if act.sum() < 10:
        print("  高度ホールドが engage している区間がありません。")
        print("    alt_state を確認: 1=STANDBY(未アーム/POSHOLDでない/スロットル不足)")
        print("                      2=NO_HOVER_THR(ALT_HOVER_THR未設定)")
        print("                      3=NO_RANGE(測距が一度も取れていない)")
        st = d["alt_state"][seg]
        vals, cnt = np.unique(st.astype(int), return_counts=True)
        print("    この区間の alt_state 内訳: " +
              "  ".join(f"{v}:{c}" for v, c in zip(vals, cnt)))
        return

    h = d["range_h"][seg][act]
    tgt = d["alt_hold"][seg][act]
    err = tgt - h
    vz = d["climb"][seg][act]
    vzt = d["alt_vzt"][seg][act]
    corr = d["alt_corr"][seg][act]
    thr = d["alt_thr"][seg][act]

    print(f"  engage 時間 {act.sum() / fs:.1f} 秒   高度 {np.mean(h):.2f} m "
          f"(min {np.min(h):.2f} / max {np.max(h):.2f})")
    stat_line("高度誤差 (目標-実測)", err, "m")
    stat_line("上昇速度 実測", vz, "m/s")
    stat_line("速度追従誤差 (目標-実測)", vzt - vz, "m/s")
    print(f"  スロットル             平均 {np.mean(thr):.3f}  "
          f"範囲 {np.min(thr):.3f}〜{np.max(thr):.3f}")

    # --- 権限飽和 (ALT_THR_AUTH に張り付いているか) ---
    sat = np.mean(np.abs(corr) >= thr_auth * 0.98) * 100.0
    print(f"  補正が権限±{thr_auth:.2f} に張り付いた割合: {sat:.1f} %")
    if sat > 20:
        print("  → 飽和が多い。原因は次のどちらか:")
        print("     (a) ALT_HOVER_THR がホバー実測とずれている "
              "→ 平均スロットルの値に合わせる")
        print("     (b) ALT_THR_AUTH が小さすぎる → 0.05 刻みで広げる")

    # --- 定常偏差 ---
    bias = np.mean(err)
    if abs(bias) > 0.10:
        lo = "低い" if bias > 0 else "高い"
        print(f"  → 目標より平均 {abs(bias):.2f} m {lo}状態が続いています。")
        print("     ALT_RATE_KI を上げる (今の 1.5倍程度) か、"
              "ALT_HOVER_THR を実測ホバー値へ寄せてください。")

    # --- 振動 ---
    f_err, share = dominant_freq(err, fs)
    if f_err and share > 0.15 and np.std(err) > 0.03:
        print(f"  → 高度が {f_err:.2f} Hz で振動しています "
              f"(振幅 RMS {np.std(err):.3f} m)")
        if f_err > 0.8:
            print("     速い振動 = 内側 (速度) ループが強い。"
                  "ALT_RATE_KP を 0.7倍。それでも残れば ALT_RATE_KD を少し。")
        else:
            print("     遅い振動 = 外側 (位置) ループが強い。"
                  "ALT_POS_KP を 0.7倍。")

    # --- 上昇速度の追従 ---
    if np.std(vzt) > 0.02:
        track = np.corrcoef(vzt, vz)[0, 1]
        print(f"  上昇速度の追従相関 r = {track:+.2f}")
        if track < 0.3:
            print("  → 目標上昇速度に全く追従できていません。"
                  "ALT_RATE_KP が低すぎるか、climb (測距の微分) が"
                  "ノイズだらけです。RANGE_VZ_ALPHA を強めてください。")



# ============================================================
#  2b. 姿勢ループ / モーター (C フレーム)
# ============================================================
#  ANGLE のブリングアップで一番知りたいのは「浮いたのか」「どのモーターが
#  飽和したか」「レートループが指令に追従しているか」。A/B だけでは
#  接地したまま転がったのか空中で発振したのかを切り分けられない。
def check_attitude(d, fs, hover_thr):
    section("2b. 姿勢ループ / モーター (ARMED 区間)")
    if "m1" not in d.dtype.names:
        print("  C フレームが入っていない古いログです (機体を焼き直してください)。")
        return

    armed = d["armed"] > 0.5
    if armed.sum() < 5:
        print("  ARMED の区間がありません。")
        return

    a = {k: d[k][armed] for k in
         ("thr", "roll", "pitch", "m1", "m2", "m3", "m4", "mixsat",
          "roll_gyr", "pitch_gyr", "roll_ratetar", "pitch_ratetar",
          "roll_cmd", "pitch_cmd", "yaw_cmd", "range_raw", "airborne")}
    t = (d["rx_ms"][armed] - d["rx_ms"][armed][0]) / 1000.0

    # --- 離陸したか ---
    print(f"  ARMED {armed.sum() / fs:.1f} 秒   "
          f"スロットル 最大 {a['thr'].max():.3f} / 平均 {a['thr'].mean():.3f}")
    if a["airborne"].max() < 0.5:
        print("  ★ 一度も離陸していません (airborne が立っていない)。")
        if hover_thr > 0 and a["thr"].max() < hover_thr * 0.9:
            print(f"     スロットル最大 {a['thr'].max():.2f} に対し "
                  f"ホバー基準 ALT_HOVER_THR = {hover_thr:.2f}。推力が足りません。")
            print("     地面近くで半端な推力のまま粘ると、片脚だけ荷重が抜けて")
            print("     支点になり転がります。ホバー付近まで1秒以内で上げること。")

    # --- 転倒の検出 ---
    tilt = np.hypot(a["roll"], a["pitch"])
    if tilt.max() > 45:
        i = int(np.argmax(tilt > 45))
        print(f"  ★ t={t[i]:.2f}s で傾き {tilt[i]:.0f} 度を超えました (転倒)。")
        j = max(0, i - int(fs * 2))
        rate = (tilt[i] - tilt[j]) / max(t[i] - t[j], 1e-3)
        print(f"     直前2秒の傾き変化 {rate:+.0f} deg/s   "
              f"そのときのスロットル {a['thr'][i]:.2f}")
        if a["airborne"][:i + 1].max() < 0.5:
            print("     接地したまま転がっています。姿勢ゲインではなく")
            print("     推力不足・重心・脚の引っかかりを先に疑ってください。")

    # --- モーター ---
    print()
    for i, k in enumerate(("m1", "m2", "m3", "m4"), 1):
        v = a[k]
        hi = np.mean(v >= 0.99) * 100
        lo = np.mean(v <= 0.01) * 100
        print(f"  M{i}  平均 {v.mean():.3f}  最大 {v.max():.3f}   "
              f"上限張り付き {hi:4.1f} %  下限 {lo:4.1f} %")
    ms = a["mixsat"].astype(int)
    print(f"  ミキサー飽和 (どれか) {np.mean(ms != 0) * 100:.1f} %   "
          + "  ".join(f"M{i+1}:{np.mean((ms >> i) & 1) * 100:.1f}%" for i in range(4)))
    if np.mean(ms != 0) * 100 > 20:
        print("  → 飽和が多すぎます。姿勢ゲインが高いか、機体が重いか、")
        print("     ホバースロットルが上限に近すぎます。")

    # --- 左右差 (推力の偏り) ---
    mm = np.array([a[k].mean() for k in ("m1", "m2", "m3", "m4")])
    spread = mm.max() - mm.min()
    print(f"  モーター平均の開き {spread:.3f}")
    if spread > 0.10:
        print("  → 特定のモーターだけ働いています。重心のずれ、モーター取付角、")
        print("     プロペラの CW/CCW 取り違え、ESC の個体差を疑ってください。")

    # --- レートループの追従 ---
    print()
    for ax, meas, tar in (("roll", "roll_gyr", "roll_ratetar"),
                          ("pitch", "pitch_gyr", "pitch_ratetar")):
        m_, t_ = a[meas], a[tar]
        err = t_ - m_
        print(f"  {ax:<6} 角速度 実測 RMS {np.sqrt(np.mean(m_**2)):6.1f} / "
              f"目標 RMS {np.sqrt(np.mean(t_**2)):6.1f} / "
              f"追従誤差 RMS {np.sqrt(np.mean(err**2)):6.1f} [deg/s]")
        if np.std(t_) > 1.0:
            r = np.corrcoef(t_, m_)[0, 1]
            if r < 0.2:
                print(f"    → 目標にまったく追従していません (r={r:+.2f})。"
                      "レートPIDの符号かゲインを疑ってください。")

    # --- 定常的な傾き (I項の要否はここで判断する) ---
    air = a["airborne"] > 0.5
    if air.sum() > fs * 2:
        print()
        print(f"  離陸中の平均傾き: roll {a['roll'][air].mean():+.2f} deg  "
              f"pitch {a['pitch'][air].mean():+.2f} deg")
        if max(abs(a["roll"][air].mean()), abs(a["pitch"][air].mean())) > 2.0:
            print("  → 空中で一方向に傾き続けています。ここで初めて")
            print("     角度ループの I項 (Gain::ANG_ROLL/ANG_PITCH の ki) が効きます。")
        else:
            print("  → 定常偏差は小さいので、I項を足す理由はまだありません。")
    else:
        print()
        print("  ★ 離陸している区間がないので、I項の要否はこのログでは判断できません。")
        print("    接地中は地面が姿勢を拘束していて誤差が消えないため、I を入れると")
        print("    溜まり続け、浮いた瞬間に一気に出て反対側へ蹴ります。")
        print("    まず浮かせること。I項の検討はそのあとです。")


# ============================================================
#  3. 水平位置ホールド
# ============================================================
def check_pos(d, seg, fs, max_lean):
    hold = d["pos_hold"][seg] > 0.5
    air = d["airborne"][seg] > 0.5
    use = hold & air
    if use.sum() < 10:
        print("  スティック中立で保持している区間がありません "
              "(pos_hold=1 かつ airborne=1)。")
        if air.sum() < 10:
            print("    airborne が立っていません。ALT_AIRBORNE_RISE_M まで"
                  "上昇できていないか、測距が来ていません。")
        return

    dn = d["fh_posn"][seg][use] - d["fh_holdn"][seg][use]
    de = d["fh_pose"][seg][use] - d["fh_holde"][seg][use]
    dist = np.hypot(dn, de)
    vx, vy = d["fh_vxc"][seg][use], d["fh_vyc"][seg][use]
    vxt, vyt = d["fh_vxt"][seg][use], d["fh_vyt"][seg][use]
    lr, lp = d["fh_leanr"][seg][use], d["fh_leanp"][seg][use]

    print(f"  保持時間 {use.sum() / fs:.1f} 秒")
    print(f"  保持点からのずれ       平均 {np.mean(dist):.2f} m  "
          f"RMS {np.sqrt(np.mean(dist**2)):.2f} m  最大 {np.max(dist):.2f} m")
    stat_line("ずれ N (前後)", dn, "m")
    stat_line("ずれ E (左右)", de, "m")
    stat_line("速度 vx (前+)", vx, "m/s")
    stat_line("速度 vy (右+)", vy, "m/s")
    stat_line("速度追従誤差 vx", vxt - vx, "m/s")
    stat_line("速度追従誤差 vy", vyt - vy, "m/s")

    lean = np.hypot(lr, lp)
    sat = np.mean(lean >= max_lean * 0.98) * 100.0
    print(f"  リーン角               平均 {np.mean(lean):.2f} deg  "
          f"最大 {np.max(lean):.2f} deg   上限±{max_lean:.1f} 張り付き {sat:.1f} %")
    if sat > 20:
        print("  → リーン上限に張り付いています。FLOW_MAX_LEAN が小さすぎて"
              "位置を戻す力が足りないか、そもそも流されすぎです。")

    # --- 一方向に流れているか (符号ミス / 取り付け傾きの疑い) ---
    for name, v in (("N (前後)", dn), ("E (左右)", de)):
        if len(v) > 5:
            t = np.arange(len(v)) / fs
            slope = np.polyfit(t, v, 1)[0]
            if abs(slope) > 0.05:
                print(f"  → {name} 方向へ {slope:+.2f} m/s で一定に流れています。")
                print("     一定方向のドリフト = 風、機体の取り付け傾き、"
                      "または FLOW_SIGN/FLOW_LEAN_SIGN の符号ミス。")
                print("     まず無風の屋内で再現するか確かめてください。"
                      "再現するなら符号、しないなら風です。")

    # --- 発散していないか ---
    if len(dist) > 20:
        first = np.mean(dist[: len(dist) // 4])
        last = np.mean(dist[-len(dist) // 4:])
        if last > first * 2 and last > 0.5:
            print(f"  → ずれが拡大しています ({first:.2f} → {last:.2f} m)。")
            print("     位置ループが効いていない (FLOW_POS_KP が低い)、"
                  "または符号が逆で正帰還になっています。")
            print("     ★ 符号確認はドライラン ('m') + 手で機体を動かして、"
                  "lean が「戻す向き」に出るかで見ること。")

    # --- 振動 ---
    for name, v, fast_hint, slow_hint in (
        ("前後 (N)", dn, "FLOW_VEL_KP", "FLOW_POS_KP"),
        ("左右 (E)", de, "FLOW_VEL_KP", "FLOW_POS_KP"),
    ):
        f, share = dominant_freq(v, fs)
        if f and share > 0.15 and np.std(v) > 0.05:
            print(f"  → {name} が {f:.2f} Hz で振動 (RMS {np.std(v):.2f} m)")
            key = fast_hint if f > 0.6 else slow_hint
            print(f"     {key} を 0.7倍にしてください"
                  f"{' (速い振動 = 速度ループ)' if f > 0.6 else ' (遅い振動 = 位置ループ)'}")

    # --- 速度ループの追従 ---
    for name, tv, mv in (("vx", vxt, vx), ("vy", vyt, vy)):
        if np.std(tv) > 0.02:
            r = np.corrcoef(tv, mv)[0, 1]
            if r < 0.2:
                print(f"  → {name} が目標速度に追従していません (r={r:+.2f})。"
                      "FLOW_VEL_KP を上げるか、符号を疑ってください。")


# ============================================================
#  4. センサ健全性
# ============================================================
def check_sensors(d, fs):
    section("4. センサの健全性")
    n = len(d["rx_ms"])

    for key, label in (("flow_ok", "PMW3901 初期化"),
                       ("range_ok", "測距センサ 初期化")):
        ok = np.mean(d[key]) * 100
        print(f"  {label:<18} OK 割合 {ok:5.1f} %")
        if ok < 99:
            print("    → 起動していません。配線と電源を確認してください。")

    rv = d["range_valid"] > 0.5
    print(f"  測距が valid だった割合 {np.mean(rv) * 100:5.1f} %")
    if np.mean(rv) < 0.95:
        # 失探の最長連続時間
        worst, cur = 0, 0
        for v in rv:
            cur = 0 if v else cur + 1
            worst = max(worst, cur)
        print(f"    → 失探あり。最長 {worst / fs:.1f} 秒 連続で見失いました。")
        print("       ソナーなら地面の材質 (草・カーペットは弱い) と "
              "プロペラ後流。ToF なら屋外の太陽光。")

    bad = d["bad"]
    if np.max(bad) > 0:
        print(f"  フロー失探カウンタ bad  最大 {np.max(bad):.0f}  "
              f"(0以外だった割合 {np.mean(bad > 0) * 100:.1f} %)")
        print("    → PMW3901 が地面の模様を掴めていません。"
              "床が無地なら模様のあるマットを敷いてください。")

    if "sat" in d.dtype.names:
        armed = d["armed"] > 0.5
        if armed.sum():
            s = np.mean(d["sat"][armed]) * 100
            print(f"  ミキサー飽和 (ARMED中) {s:5.1f} %")
            if s > 20:
                print("    → 姿勢ゲインが高すぎるか、機体が重すぎます。"
                      "この状態では位置・高度の調整をしても意味がありません。")


# ============================================================
#  グラフ
# ============================================================
def plot(d, path):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[--plot には matplotlib が必要です]  pip install matplotlib")
        return

    t = (d["rx_ms"] - d["rx_ms"][0]) / 1000.0
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

    ax[0].plot(t, d["range_h"], label="h (m)")
    ax[0].plot(t, d["alt_hold"], "--", label="target (m)")
    ax[0].set_ylabel("altitude [m]")
    ax[0].legend(loc="upper right")
    ax[0].grid(alpha=0.3)

    ax[1].plot(t, d["climb"], label="vz meas")
    ax[1].plot(t, d["alt_vzt"], "--", label="vz target")
    ax[1].plot(t, d["alt_thr"], ":", label="throttle out")
    ax[1].set_ylabel("vz [m/s] / thr")
    ax[1].legend(loc="upper right")
    ax[1].grid(alpha=0.3)

    ax[2].plot(t, d["fh_posn"] - d["fh_holdn"], label="dN (m)")
    ax[2].plot(t, d["fh_pose"] - d["fh_holde"], label="dE (m)")
    ax[2].set_ylabel("position error [m]")
    ax[2].legend(loc="upper right")
    ax[2].grid(alpha=0.3)

    ax[3].plot(t, d["fh_vxc"], label="vx")
    ax[3].plot(t, d["fh_vyc"], label="vy")
    ax[3].plot(t, d["fh_vxt"], "--", label="vx target")
    ax[3].plot(t, d["fh_vyt"], "--", label="vy target")
    ax[3].set_ylabel("velocity [m/s]")
    ax[3].set_xlabel("time [s]")
    ax[3].legend(loc="upper right")
    ax[3].grid(alpha=0.3)

    # POSHOLD 区間を塗る
    for s in poshold_segments(d):
        for a in ax:
            a.axvspan(t[s][0], t[s][-1], color="tab:green", alpha=0.08)

    fig.suptitle(Path(path).name)
    fig.tight_layout()
    plt.show()


# ============================================================
def main():
    ap = argparse.ArgumentParser(description="s5 POSHOLD telemetry analyzer")
    ap.add_argument("csv", nargs="?", default=None)
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    path = Path(args.csv) if args.csv else latest_log()
    d = load(path)
    print(f"ログ: {path}   {len(d)} サンプル")

    params = load_params(path)
    thr_auth, max_lean, hover_thr = 0.40, 8.0, 0.0  # QuadConfig.h の既定値
    if params:
        p = params[-1]
        section("0. 飛行時のゲイン (機体から届いた PARAM パケット)")
        print(f"  flow  vel  P={p.get('flow_vel_kp')} I={p.get('flow_vel_ki')} "
              f"D={p.get('flow_vel_kd')}   pos P={p.get('flow_pos_kp')}")
        print(f"  alt   rate P={p.get('alt_rate_kp')} I={p.get('alt_rate_ki')} "
              f"D={p.get('alt_rate_kd')}   pos P={p.get('alt_pos_kp')}")
        print(f"  hover_thr={p.get('alt_hover_thr')}  target={p.get('alt_target_m')} m  "
              f"max_lean={p.get('flow_max_lean')} deg  thr_auth={p.get('alt_thr_auth')}")
        try:
            thr_auth = float(p["alt_thr_auth"])
            max_lean = float(p["flow_max_lean"])
            hover_thr = float(p["alt_hover_thr"])
        except (KeyError, ValueError):
            pass
        if len(params) > 1:
            uniq = {tuple(sorted(x.items())) for x in params}
            if len(uniq) > 1:
                print("  ※ ログ中にゲインが変更されています "
                      f"({len(uniq)} 通り)。区間ごとに見てください。")
    else:
        print("(.params.txt がありません。権限・リーン上限は QuadConfig の"
              "既定値を仮定します)")

    fs = check_link(d)

    # ★ IM920sL の 32バイト制限のため、機体は A(高度) と B(水平) を交互に
    #   送っている。地上局は届くたびに「もう片方は前回値」で1行を作る
    #   (forward-fill) ので、CSV には同じ値が2行続く箇所ができる。
    #   統計と FFT を正しく出すため、ここで実際に更新された行だけを取る。
    #     frame == 0 -> A が新しい行 / frame == 1 -> B が新しい行
    if "frame" in d.dtype.names:
        d_alt = d[d["frame"] < 0.5]                             # A
        d_pos = d[(d["frame"] > 0.5) & (d["frame"] < 1.5)]      # B
        d_att = d[d["frame"] > 1.5]                             # C
    else:                                   # 旧形式 (単一フレーム) の CSV
        d_alt = d_pos = d_att = d

    def rate_of(x):
        # 空 / 1行のときは 0 を返す。fs で埋めると「無いフレーム」が
        # 全体レートで表示されてしまい、有無の判断を誤る。
        if len(x) < 2:
            return 0.0
        span = (x["rx_ms"][-1] - x["rx_ms"][0]) / 1000.0
        return (len(x) - 1) / span if span > 0 else 0.0

    fs_alt, fs_pos, fs_att = rate_of(d_alt), rate_of(d_pos), rate_of(d_att)
    print(f"  実効レート: A(高度) {fs_alt:.1f} Hz / B(水平) {fs_pos:.1f} Hz / "
          f"C(姿勢) {fs_att:.1f} Hz")
    # 表示は実測のまま。以降の「秒数 = サンプル数 / fs」でゼロ割しないよう
    # ここから先だけ全体レートで埋める。
    fs_alt = fs_alt or fs
    fs_pos = fs_pos or fs
    fs_att = fs_att or fs
    if len(d_att) < 5:
        print("  ※ C(姿勢) フレームがありません。機体側が古いファームです")
        print("     (drone_s5 を焼き直すと m1..m4 / 角速度 / ミキサー飽和が入ります)。")

    segs_a = poshold_segments(d_alt)
    segs_b = poshold_segments(d_pos)
    if not segs_a and not segs_b:
        section("2/3. POSHOLD 区間")
        print("  POSHOLD かつ ARMED の区間がありません "
              "(mode: 1=ANGLE / 3=POSHOLD)。")
        print("  ANGLE のブリングアップ中ならこれで正常です。下の 2b を見てください。")
    for i, sl in enumerate(segs_a, 1):
        dur = (d_alt["rx_ms"][sl][-1] - d_alt["rx_ms"][sl][0]) / 1000.0
        section(f"2. 高度ホールド  [POSHOLD 区間 {i}/{len(segs_a)}  {dur:.1f} 秒]")
        check_alt(d_alt, sl, fs_alt, thr_auth)
    for i, sl in enumerate(segs_b, 1):
        dur = (d_pos["rx_ms"][sl][-1] - d_pos["rx_ms"][sl][0]) / 1000.0
        section(f"3. 水平位置ホールド  [POSHOLD 区間 {i}/{len(segs_b)}  {dur:.1f} 秒]")
        check_pos(d_pos, sl, fs_pos, max_lean)

    if len(d_att) > 5:
        check_attitude(d_att, fs_att, hover_thr)

    check_sensors(d, fs)

    section("まとめ")
    print("  調整の順番は 高度 → 水平 です。高度が上下していると")
    print("  フローの見かけ速度が高さ変化で汚れ、水平の調整が成立しません。")
    print("  値は QuadConfig.h の § 7-2 (FLOW_*) / § 7-4 (ALT_*)、")
    print("  飛行中はシリアル 'p' メニューでも変えられます。")

    if args.plot:
        plot(d, path)


if __name__ == "__main__":
    main()
