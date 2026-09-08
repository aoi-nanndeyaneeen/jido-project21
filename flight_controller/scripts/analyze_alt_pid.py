# -*- coding: utf-8 -*-
"""
analyze_alt_pid.py  -  高度ホールド (距離ループ / 速度ループ) の解析とグラフ化

使い方:
    python analyze_alt_pid.py ../logs/log_030_20260906_162350.csv
    python analyze_alt_pid.py ../logs/log_030_20260906_162350.csv --png out.png

やること:
  1. armed && mode==ALTHOLD/POSHOLD && alt_act==1 の区間だけを解析対象にする
  2. 距離ループ (外側): 目標高度 alt_hold, 実測 range_h, 誤差, 出力 alt_vzt(目標上昇速度)
  3. 速度ループ (内側): 目標 alt_vzt, 実測 climb, 誤差, 出力 alt_corr / alt_thr
  4. IMU(姿勢) と 距離センサ の整合性チェック:
       - range_h  ?=  range_raw * cos(roll) * cos(pitch) + OFFSET       (cos補正が効いているか)
       - climb    ?=  d/dt(range_h)                                     (PIDが見ている速度が高度微分と一致するか)
       - climb の対 d/dt(range_h) 遅れ / ゲイン (二重LPFの位相遅れを定量化)
  横軸はすべて時間 [s]。誤差信号は各パネルの右軸に出す。

ファーム側の対応定数 (QuadConfig.h より。変えたらここも直す):
"""
import sys, csv, argparse
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# --- QuadConfig.h と揃える ---
RANGE_OFFSET_M   = 0.00
RANGE_H_ALPHA    = 0.45
RANGE_VZ_ALPHA   = 0.45
ALT_POS_KP       = 2.0
ALT_POS_VZ_LIM   = 0.8
ALT_RATE_KP      = 0.3
ALT_RATE_KI      = 0.06
ALT_RATE_KD      = 0.03
ALT_RATE_I_LIMIT = 0.12
ALT_THR_AUTH     = 0.55
ALT_HOVER_THR    = 0.42
THR_IDLE         = 0.10
THR_MIN_MIX      = 0.05
THR_RAMP_RANGE   = 0.05
MIX_ROLL         = np.array([+1.0, -1.0, -1.0, +1.0])
MIX_PITCH        = np.array([+1.0, +1.0, -1.0, -1.0])
MIX_YAW          = np.array([-1.0, +1.0, -1.0, +1.0])
DEG2RAD          = np.pi / 180.0

MODE_POSHOLD = 3
MODE_ALTHOLD = 4


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    d = {k: np.array([float(r[k]) for r in rows]) for k in rows[0].keys()}
    return d


def central_diff(y, t):
    """非等間隔 t に対する中心差分。端は前方/後方差分。"""
    v = np.zeros_like(y)
    v[1:-1] = (y[2:] - y[:-2]) / (t[2:] - t[:-2])
    v[0]    = (y[1] - y[0]) / (t[1] - t[0])
    v[-1]   = (y[-1] - y[-2]) / (t[-1] - t[-2])
    return v


def lag_via_xcorr(a, b, dt_med, max_lag_s=0.6):
    """a を何秒遅らせると b に最も一致するか (正 = a が b より進んでいる=bが遅れている)。"""
    a = a - a.mean()
    b = b - b.mean()
    n = len(a)
    max_k = int(max_lag_s / dt_med)
    best_k, best_c = 0, -1e18
    for k in range(-max_k, max_k + 1):
        if k >= 0:
            c = np.dot(a[k:], b[:n - k]) if n - k > 0 else 0.0
            denom = n - k
        else:
            c = np.dot(a[:n + k], b[-k:]) if n + k > 0 else 0.0
            denom = n + k
        c /= max(denom, 1)
        if c > best_c:
            best_c, best_k = c, k
    return best_k * dt_med, best_c


def newest_log():
    """引数省略時: scripts/ から見た ../logs/ の最新 log_*.csv を返す。"""
    import glob, os
    here = os.path.dirname(os.path.abspath(__file__))
    cands = glob.glob(os.path.join(here, "..", "logs", "log_*.csv"))
    cands = [c for c in cands if os.path.getsize(c) > 200]
    if not cands:
        return None
    return max(cands, key=os.path.getmtime)


# ------------------------------------------------------------------
#  ログの全チャンネルを、意味ごとにグループ分けして時系列プロット
# ------------------------------------------------------------------
#  各タプル: (パネル見出し, [(列名, 表示ラベル), ...], y軸ラベル)
FULL_GROUPS = [
    ("ループ周期",        [("dt_us", "dt_us [us]")], "us"),
    ("モード / アーム",   [("mode", "mode (1=ANGLE 3=POSHOLD 4=ALTHOLD)"),
                           ("armed", "armed"), ("alt_en", "alt_en"),
                           ("alt_act", "alt_act"), ("range_ok", "range_ok"),
                           ("flow_ok", "flow_ok"), ("sat", "mixer sat")], "flag / 値"),
    ("スティック",        [("roll_sbus", "roll"), ("pitch_sbus", "pitch"),
                           ("yaw_sbus", "yaw"), ("thr", "throttle")], "正規化 (-1..1 / thr 0..1)"),
    ("姿勢角",            [("roll_ang", "roll"), ("pitch_ang", "pitch"),
                           ("yaw_ang", "yaw")], "deg"),
    ("角度ループ 目標 vs 実測", [("roll_angtar", "roll 目標"), ("roll_ang", "roll 実測"),
                           ("pitch_angtar", "pitch 目標"), ("pitch_ang", "pitch 実測")], "deg"),
    ("角速度 (ジャイロ)", [("roll_gyr", "roll"), ("pitch_gyr", "pitch"),
                           ("yaw_gyr", "yaw")], "deg/s"),
    ("レートループ 目標 vs 実測", [("roll_ratetar", "roll 目標"), ("roll_gyr", "roll 実測"),
                           ("pitch_ratetar", "pitch 目標"), ("pitch_gyr", "pitch 実測")], "deg/s"),
    ("PID 出力 (ミキサー入力)", [("roll_cmd", "roll"), ("pitch_cmd", "pitch"),
                           ("yaw_cmd", "yaw"), ("corr_limit", "corr_limit")], "割合"),
    ("モーター出力",      [("m1", "M1"), ("m2", "M2"), ("m3", "M3"), ("m4", "M4")], "割合 0..1"),
    ("フロー 生値",       [("flow_raw_x", "raw_x"), ("flow_raw_y", "raw_y"),
                           ("flow_dx", "derot_x"), ("flow_dy", "derot_y")], "px"),
    ("フロー 速度 / 高度", [("flow_vx", "vx [m/s]"), ("flow_vy", "vy [m/s]"),
                           ("flow_h", "flow_h [m]")], "m/s, m"),
    ("フロー積算",        [("flow_accx", "acc_x [m]"), ("flow_accy", "acc_y [m]")], "m"),
    ("位置ホールド 速度", [("fh_vxc", "vx 実測"), ("fh_vxt", "vx 目標"),
                           ("fh_vyc", "vy 実測"), ("fh_vyt", "vy 目標")], "m/s"),
    ("位置ホールド リーン / 位置", [("fh_leanr", "lean roll [deg]"), ("fh_leanp", "lean pitch [deg]"),
                           ("fh_posn", "pos N [m]"), ("fh_pose", "pos E [m]"),
                           ("fh_holdn", "hold N [m]"), ("fh_holde", "hold E [m]"),
                           ("fh_hold", "holding")], "deg / m / flag"),
    ("距離センサ",        [("range_raw", "range_raw [m]"), ("range_h", "range_h [m]"),
                           ("climb", "climb [m/s]")], "m, m/s"),
    ("高度ホールド",      [("alt_hold", "目標高度 [m]"), ("range_h", "実測高度 [m]"),
                           ("alt_vzt", "目標上昇速度 [m/s]"), ("climb", "実測上昇速度 [m/s]"),
                           ("alt_base", "base thr"), ("alt_corr", "corr thr"),
                           ("alt_thr", "out thr")], "m / (m/s) / 割合"),
    ("高度ホールド 指令 vs 実効", [("alt_thr", "alt_thr 高度ループの指令"),
                           ("alt_used", "alt_used ミキサーが実際に使った値")], "割合"),
    ("高度推定 相補フィルタ", [("range_h", "range_h 測距(LPF後)"), ("est_h", "est_h 推定高度"),
                           ("climb", "climb 測距の微分"), ("est_vz", "est_vz 推定上昇速度"),
                           ("est_bias", "est_bias 加速度バイアス")], "m / (m/s) / (m/s^2)"),
    ("鉛直加速度",        [("acc_up", "acc_up 重力除去後 [m/s^2]")], "m/s^2"),
]


def plot_full(d, t, out, stalls):
    groups = [g for g in FULL_GROUPS if all(c in d for c, _ in g[1])]
    ng = len(groups)
    fig, ax = plt.subplots(ng, 1, figsize=(15, 2.6 * ng), sharex=True)
    if ng == 1:
        ax = [ax]

    brk = np.zeros(len(t), dtype=bool)
    brk[1:] = np.diff(t) > 0.008

    for a, (title, cols, ylab) in zip(ax, groups):
        for c, lab in cols:
            y = d[c].astype(float).copy()
            y[brk] = np.nan
            a.plot(t, y, lw=1.0, label=lab)
        for (t0, t1) in stalls:
            a.axvspan(t0, t1, color="red", alpha=0.08, lw=0)
        a.set_ylabel(ylab, fontsize=8)
        a.set_title(title, fontsize=10, loc="left")
        a.grid(alpha=0.3)
        a.legend(loc="upper right", fontsize=7, ncol=4)
    ax[-1].set_xlabel("時間 [s]")
    ax[0].set_xlim(t[0], t[-1])
    fig.suptitle(f"全チャンネル時系列  {out}   (赤帯 = ループ停止/ログ欠損区間)", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.99])
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"全チャンネルグラフ: {out}  ({ng} パネル)")


def mixer_thr_used(d):
    """Mixer::mix() を再現して「実際に使われたスロットル」を出す。

    ファームが alt_used を吐く前のログでも、姿勢優先でスロットルが
    どれだけずらされたかを後から復元できる。
    log_037 でこの再現はモーター平均と rms 0.0009 / 相関 1.000 で一致した。
    """
    thr = d["alt_thr"]
    auth = np.clip((thr - THR_MIN_MIX) / THR_RAMP_RANGE, 0.0, 1.0)
    corr = (np.outer(d["roll_cmd"]  * auth, MIX_ROLL)
          + np.outer(d["pitch_cmd"] * auth, MIX_PITCH)
          + np.outer(d["yaw_cmd"]   * auth, MIX_YAW))
    lo, hi = corr.min(axis=1), corr.max(axis=1)
    span = hi - lo
    span_limit = 1.0 - THR_IDLE
    k = np.where(span > span_limit, span_limit / np.maximum(span, 1e-9), 1.0)
    lo, hi = lo * k, hi * k
    return np.clip(thr, THR_IDLE - lo, 1.0 - hi)


def diagnose(d, t, m):
    """PIDゲインを見る前に確認すべき「仕組み」側の健全性チェック。

    ここが赤なら、どんなゲインを入れても高度は落ち着かない。
    """
    print("\n============ 仕組みの健全性チェック (ゲインより先に見る) ============")
    mot = np.vstack([d[f"m{i}"] for i in (1, 2, 3, 4)])
    m_mean = mot.mean(axis=0)

    # --- 1) 実測ホバースロットル -----------------------------------
    #  「空中で高度がほぼ一定」の区間のモーター出力平均。スティック位置ではない。
    hov = (d["armed"] > 0.5) & (d["range_ok"] > 0.5) & \
          (d["range_h"] > 0.3) & (np.abs(d["climb"]) < 0.10)
    print("\n [1] 実測ホバースロットル (= 空中で |climb|<0.1 の区間のモーター平均)")
    if hov.sum() < 50:
        print(f"     区間が短すぎて測れない ({hov.sum()} 行)。定常ホバーを録ること。")
    else:
        hv = m_mean[hov].mean()
        print(f"     実測 {hv:.3f}   設定 ALT_HOVER_THR = {ALT_HOVER_THR:.3f}"
              f"   ({hov.sum()} 行 / {hov.sum()*np.median(np.diff(t)):.1f}s)")
        gap = ALT_HOVER_THR - hv
        if abs(gap) > ALT_RATE_I_LIMIT:
            print(f"     !! ずれ {gap:+.3f} が I項の上限 ({ALT_RATE_I_LIMIT}) を超えている。")
            print(f"        P項は誤差ゼロで 0 なので、この差を埋められるのは I項だけ。")
            print(f"        構造的に定常偏差が残る -> ALT_HOVER_THR を {hv:.2f} に直すこと。")
        else:
            print(f"     ずれ {gap:+.3f}  (I項の上限 {ALT_RATE_I_LIMIT} 以内。吸収できる)")

    # --- 2) 測距の実サンプルレート ---------------------------------
    ch = np.where(np.abs(np.diff(d["range_raw"])) > 1e-9)[0] + 1
    print("\n [2] 測距の実サンプルレート")
    if len(ch) < 5:
        print("     サンプル数不足。")
        fs_range = None
    else:
        iv = np.diff(t[ch])
        iv = iv[iv < 0.5]
        fs_range = 1.0 / np.median(iv)
        print(f"     range_raw が更新される間隔: 中央値 {np.median(iv)*1000:.1f} ms"
              f"  -> {fs_range:.1f} Hz  ({len(ch)} サンプル)")
        if fs_range < 25:
            print(f"     !! 設定上の想定 (VL53L1X ~30Hz) より遅い。")
            print(f"        VL53L1X は 連続測距周期 > timing budget + 4ms を要求する。")
            print(f"        33ms budget + 33ms 周期のような設定だと1回おきに落ちる。")
        # LPF の群遅延はサンプルレートで決まる
        Ts = 1.0 / fs_range
        tau_h  = -Ts / np.log(1 - RANGE_H_ALPHA) * 1000
        tau_vz = -Ts / np.log(1 - RANGE_VZ_ALPHA) * 1000
        print(f"     この Ts での LPF 群遅延: H {tau_h:.0f} ms + VZ {tau_vz:.0f} ms"
              f" + ZOH {0.5*Ts*1000:.0f} ms + センサ 40 ms"
              f" = {tau_h+tau_vz+0.5*Ts*1000+40:.0f} ms")

    # --- 3) 高度PIDが実際に使った dt -------------------------------
    #  ファームは fresh の回だけ PID を進める。その間隔が PID の真の dt。
    chc = np.where(np.abs(np.diff(d["alt_corr"])) > 1e-9)[0] + 1
    chc = chc[m[chc]] if m.sum() else chc
    print("\n [3] 高度PID が実際に進む間隔 (= PID に渡すべき dt)")
    if len(chc) < 5:
        print("     サンプル数不足。")
    else:
        ivc = np.diff(t[chc]); ivc = ivc[ivc < 0.5]
        print(f"     alt_corr が更新される間隔: 中央値 {np.median(ivc)*1000:.1f} ms"
              f"  (p90 {np.percentile(ivc,90)*1000:.1f} ms)")
        print(f"     ★ ファームがここに RANGE_LOOP_HZ の周期を渡していると、"
              f"D項が {np.median(ivc)/0.010:.1f}倍(100Hz時)過大になる。"
              f" AltHold は fresh 間の経過を自前で積算すること。")

    # --- 4) ミキサーに奪われたスロットル ---------------------------
    print("\n [4] ミキサーがスロットルを奪っていないか (ATTITUDE_PRIORITY)")
    if "alt_used" in d:
        used = d["alt_used"]
        src = "ログの alt_used (ファーム実測)"
    else:
        used = mixer_thr_used(d)
        src = "Mixer::mix() の再現 (ログに alt_used が無いため)"
    steal = used - d["alt_thr"]
    if m.sum() < 20:
        print("     高度ホールドが効いている区間が無い。")
    else:
        s = steal[m]
        print(f"     出典: {src}")
        print(f"     押し上げ量 alt_used - alt_thr : 平均 {s.mean():+.3f}"
              f"  最大 {s.max():+.3f}  最小 {s.min():+.3f}")
        print(f"     |ずれ|>0.02 の割合 {100*np.mean(np.abs(s)>0.02):.1f} %"
              f" / >0.10 {100*np.mean(np.abs(s)>0.10):.1f} %"
              f" / >0.20 {100*np.mean(np.abs(s)>0.20):.1f} %")
        if np.mean(np.abs(s) > 0.02) > 0.10:
            print("     !! 高度ループの出力が実際には出ていない。制御が閉じていない。")
            print("        原因は姿勢PID出力の大きさ。下の [5] を見ること。")

    # --- 5) 姿勢の振動 (ミキサーが奪う原因) ------------------------
    print("\n [5] 姿勢ループの振動 (ここが震えるほどミキサーがスロットルを奪う)")
    if m.sum() > 512:
        fs = 1.0 / np.median(np.diff(t))
        for ax in ("roll", "pitch"):
            y = d[f"{ax}_gyr"][m]; y = y - y.mean()
            P = np.abs(np.fft.rfft(y)) ** 2
            f = np.fft.rfftfreq(len(y), 1 / fs)
            peak = f[np.argmax(P[1:]) + 1]
            cmd = d[f"{ax}_cmd"][m]
            print(f"     {ax:5s}: gyro rms {np.sqrt(np.mean(y**2)):5.1f} dps"
                  f"  主ピーク {peak:5.1f} Hz"
                  f"  |cmd| 平均 {np.abs(cmd).mean():.3f} 最大 {np.abs(cmd).max():.3f}")
        print("     ※ 目安: |cmd| 平均が 0.05 を超え、5〜30Hz に鋭いピークがあれば")
        print("       レートループのD項由来の振動を疑う (ミキサー権限を食い潰す)。")

    # --- 6) 高度推定 (相補フィルタ) の素性 --------------------------
    print("\n [6] 高度推定 (加速度Z × 測距の相補フィルタ)")
    if "acc_up" not in d:
        print("     このログに推定チャンネルが無い (ファームが古い)。")
        print("     ALT_USE_ACC_FUSION=false のまま1本飛ばせば出る。")
    elif m.sum() < 200:
        print("     高度ホールドが効いている区間が短すぎる。")
    else:
        acc_up = d["acc_up"]; est_vz = d["est_vz"]; est_h = d["est_h"]
        bias = d["est_bias"]; climb = d["climb"]; range_h = d["range_h"]
        dt_med = np.median(np.diff(t))

        # (a) 符号チェック: 上昇中に acc_up は正か
        #  高度の2階微分と acc_up が同符号でなければ ALT_ACC_Z_SIGN が逆。
        dh = np.gradient(range_h, t)
        ddh = np.gradient(np.convolve(dh, np.ones(51)/51, mode="same"), t)
        au_s = np.convolve(acc_up, np.ones(51)/51, mode="same")
        good = m & np.isfinite(ddh) & np.isfinite(au_s)
        r_sign = np.corrcoef(au_s[good], ddh[good])[0, 1] if good.sum() > 100 else 0.0
        print(f"     [符号] acc_up と d2(range_h)/dt2 の相関: {r_sign:+.2f}")
        if r_sign < -0.15:
            print("     !! 符号が逆。QuadConfig.h の ALT_ACC_Z_SIGN を -1.0 にすること。")
        elif r_sign > 0.15:
            print("        -> ALT_ACC_Z_SIGN は現在の値で正しい。")
        else:
            print("        -> 相関が弱く判定できない。上下に動かす飛行を録ること。")

        # (b) バイアス収束
        print(f"     [バイアス] est_bias: 最終 {bias[m][-1]:+.3f} m/s2"
              f"  範囲 [{bias[m].min():+.3f}, {bias[m].max():+.3f}]")
        if np.abs(bias[m]).max() > 2.5:
            print("     !! バイアスが上限近くまで振れている。IMU キャリブを取り直すこと。")

        # (c) 遅れの比較。
        #  ★ 参照は「生の測距に cos補正だけ掛けたもの」を零位相平滑して微分する。
        #    range_h から作ってはいけない。range_h はファーム内で既に H-LPF を
        #    通っているので、そこから作った参照は climb と同じ遅れを共有してしまい、
        #    climb の遅れが 0ms に見えてしまう (実際そうなった)。
        #  零位相平滑 = 前向き移動平均 → 反転 → もう一度 → 反転。位相が打ち消える。
        def zero_phase(y, w=41):
            kk = np.ones(w) / w
            return np.convolve(np.convolve(y, kk, mode="same")[::-1], kk,
                               mode="same")[::-1]
        h_raw = (d["range_raw"] * np.cos(d["roll_ang"] * DEG2RAD)
                                * np.cos(d["pitch_ang"] * DEG2RAD)) + RANGE_OFFSET_M
        ref = np.gradient(zero_phase(h_raw), t)

        def lag_ms(sig):
            """正 = sig が参照より遅れている / 負 = 先行している。
            戻り値 (遅れ[ms], そのときの相関, 参照との rms差)。"""
            a = sig[m] - sig[m].mean(); b = ref[m] - ref[m].mean()
            n = len(a)
            sa, sb = np.std(a), np.std(b)
            if sa < 1e-9 or sb < 1e-9:
                return float("nan"), 0.0, float("nan")
            best = (0, -1e18)
            span = int(0.6 / dt_med)
            for kk in range(-span, span + 1):     # 先行もありうるので負まで探す
                if kk >= 0:
                    c = np.dot(a[kk:], b[:n - kk]) / max(n - kk, 1)
                else:
                    c = np.dot(a[:n + kk], b[-kk:]) / max(n + kk, 1)
                c /= (sa * sb)
                if c > best[1]: best = (kk, c)
            return best[0] * dt_med * 1000, best[1], np.sqrt(np.mean((a - b) ** 2))

        l_climb, c_climb, e_climb = lag_ms(climb)
        l_est, c_est, e_est = lag_ms(est_vz)
        print("     [遅れ] 位相ゼロ参照 (生測距のcos補正) に対する遅れ:")
        print(f"        従来 climb  (測距の微分) : {l_climb:6.0f} ms"
              f"  相関 {c_climb:.2f}  参照とのrms差 {e_climb:.3f} m/s")
        print(f"        新   est_vz (加速度融合) : {l_est:6.0f} ms"
              f"  相関 {c_est:.2f}  参照とのrms差 {e_est:.3f} m/s")
        lag_ok = (c_climb > 0.7 and c_est > 0.7)
        if not lag_ok:
            print("     !! 相関が低く、この飛行では遅れを測れない。")
            print("        高度が一方向にゆっくり動くだけの記録だと、位相差が")
            print("        小さすぎて相互相関に出ない (log_037 がまさにこれ)。")
            print("        1〜2Hz で上下に振る飛行を5秒ほど録ると測れる。")
            print("        ※ 参照とのrms差なら短い記録でも比較できる。小さいほど良い。")
        else:
            print(f"        -> 短縮 {l_climb - l_est:.0f} ms")
        print(f"     [一致] est_vz vs climb: 相関 {np.corrcoef(est_vz[m], climb[m])[0,1]:+.2f}"
              f"  rms差 {np.sqrt(np.mean((est_vz[m]-climb[m])**2)):.3f} m/s")
        print(f"     [高度] est_h vs range_h: rms差"
              f" {np.sqrt(np.mean((est_h[m]-range_h[m])**2)):.3f} m"
              f"  (0.1m 以内なら追従できている)")
        ok_sign = (r_sign > 0.15)
        ok_bias = (np.abs(bias[m]).max() < 2.5)
        ok_h    = (np.sqrt(np.mean((est_h[m] - range_h[m]) ** 2)) < 0.10)
        ok_lag  = (e_est < e_climb) if not lag_ok else (l_est < l_climb - 30)
        if ok_sign and ok_bias and ok_h and ok_lag:
            print("     ==> 素性は良好。ALT_USE_ACC_FUSION = true にしてよい。")
        else:
            ng = [n for n, o in [("符号", ok_sign), ("バイアス", ok_bias),
                                 ("高度追従", ok_h), ("速さ", ok_lag)] if not o]
            print(f"     ==> まだ有効化しないこと。未達: {' / '.join(ng)}")
    print()


def main():
    ap = argparse.ArgumentParser(
        description="高度ホールド(距離/速度PID)解析。csv 省略時は ../logs の最新 log_*.csv を使う。")
    ap.add_argument("csv", nargs="?", default=None, help="ログCSV (省略で最新を自動選択)")
    ap.add_argument("--png", default=None, help="高度PID解析グラフの出力先")
    ap.add_argument("--all", action="store_true", help="区間を絞らず全行を使う")
    ap.add_argument("--no-full", action="store_true", help="全チャンネルグラフを出さない")
    ap.add_argument("--full-only", action="store_true", help="全チャンネルグラフだけ出す")
    args = ap.parse_args()

    if args.csv is None:
        args.csv = newest_log()
        if args.csv is None:
            ap.error("../logs に log_*.csv が見つかりません。CSV を引数で指定してください。")
        print(f"(引数省略 -> 最新ログを使用: {args.csv})")

    d = load(args.csv)
    t = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    n = len(t)

    # --- ループ停止 (ログ欠損) の検出。これがあると PID 評価は無意味 ---
    gap = np.diff(d["t_ms"])
    big = [(d["t_ms"][i] - d["t_ms"][0], gap[i]) for i in range(len(gap)) if gap[i] >= 10]
    stalls = [((d["t_ms"][i] - d["t_ms"][0]) / 1000.0,
               (d["t_ms"][i + 1] - d["t_ms"][0]) / 1000.0)
              for i in range(len(gap)) if gap[i] >= 10]
    if big:
        # 各ギャップを「本物のループ停止」か「受信取りこぼし」に分類する。
        #  - r.dt_us は 16bit で 65535us に飽和 -> ギャップ直後の dt_us が >= ~60000 なら
        #    ファーム自身が長い1周を計測した = 本物の停止。
        #  - ギャップをまたいで dt_us が ~1000-2000 のまま & ギャップが 2ms の整数倍
        #    -> 行そのものは生成されていた = USB/PC 側で取りこぼした (RamLog でも起きる)。
        dtus = d["dt_us"]
        idx_map = {int(d["t_ms"][i] - d["t_ms"][0]): i for i in range(n)}
        real_stall = drop = 0
        print(f"!! データ欠損 {len(big)} 箇所, 合計 {sum(g for _,g in big):.0f} ms:")
        for tm, g in big[:20]:
            j = idx_map.get(int(tm))
            after = dtus[j + 1] if (j is not None and j + 1 < n) else 0
            before = dtus[j] if j is not None else 0
            miss = max(int(round(g / 2.0)) - 1, 0)   # 500Hz(2ms)前提で抜けた行数
            if max(after, before) >= 20000:          # dt_us が 20ms 以上 = 本物の長い1周
                kind, tag = "停止", f"ループが固まった (dt_us={max(after,before):.0f}us)"
                real_stall += 1
            else:
                kind, tag = "取りこぼし", f"~{miss}行が受信されず (前後 dt_us {before:.0f}/{after:.0f}us = 正常)"
                drop += 1
            print(f"     t={tm/1000:6.2f}s  gap={g:5.0f}ms  [{kind}]  {tag}")
        print(f"   --> 本物のループ停止 {real_stall} / 受信取りこぼし {drop}")
        if drop and not real_stall:
            print("   ==> ファームは回り続けている。RAM/生成は正常で、USB受信(logger.py)で欠落。")
            print("       RamLog ダンプでも同じ: ダンプは無ペーシングの ~1.3MB 一括送信なので")
            print("       pyserial の1行ずつ読みが追いつかず Windows 受信バッファが溢れる。")
        print()

    # --- 全チャンネル時系列グラフ ---
    if not args.no_full:
        full_out = (args.csv.rsplit(".", 1)[0] + "_full.png")
        plot_full(d, t, full_out, stalls)
    if args.full_only:
        return

    # --- 解析区間: armed かつ 高度ホールドが実際に効いている所 ---
    if args.all:
        m = np.ones(n, dtype=bool)
    else:
        m = (d["armed"] > 0.5) & (d["alt_act"] > 0.5) & \
            ((d["mode"] == MODE_ALTHOLD) | (d["mode"] == MODE_POSHOLD))
    if m.sum() < 20:
        print("!! alt_act 有効な区間が短すぎます。--all で全体を見ます。")
        m = np.ones(n, dtype=bool)

    diagnose(d, t, m)

    # ループ停止をまたぐと PID 数値が壊れるので、サンプル間隔が正常な所だけ残す
    good = np.ones(n, dtype=bool)
    good[1:] = np.diff(d["t_ms"]) < 8.0
    m = m & good

    # 連続した最長ブロックを取る
    idx = np.where(m)[0]
    splits = np.where(np.diff(idx) > 1)[0]
    blocks = np.split(idx, splits + 1)
    blk = max(blocks, key=len)
    s, e = blk[0], blk[-1] + 1
    print(f"解析区間: row {s}..{e-1}  ({t[s]:.2f}s .. {t[e-1]:.2f}s, {e-s} rows"
          f" / 全 {t[-1]:.1f}s 中。停止区間は除外済み)")

    sl = slice(s, e)
    tt      = t[sl]
    dt_med  = np.median(np.diff(tt))
    hold    = d["alt_hold"][sl]
    range_h = d["range_h"][sl]
    range_r = d["range_raw"][sl]
    climb   = d["climb"][sl]
    vzt     = d["alt_vzt"][sl]
    corr    = d["alt_corr"][sl]
    athr    = d["alt_thr"][sl]
    thr     = d["thr"][sl]
    roll    = d["roll_ang"][sl]
    pitch   = d["pitch_ang"][sl]
    m_all   = np.vstack([d[k][sl] for k in ("m1", "m2", "m3", "m4")])
    m_mean  = m_all.mean(axis=0)
    m_max   = m_all.max(axis=0)
    m_min   = m_all.min(axis=0)

    # --- IMU姿勢から cos 補正高度を再計算 ---
    h_geom = range_r * np.cos(roll * DEG2RAD) * np.cos(pitch * DEG2RAD) + RANGE_OFFSET_M
    # ファームは h_geom をさらに 1次LPF(alpha=RANGE_H_ALPHA)して range_h にしている。
    h_lpf = np.zeros_like(h_geom)
    h_lpf[0] = range_h[0]
    for i in range(1, len(h_geom)):
        h_lpf[i] = h_lpf[i-1] + RANGE_H_ALPHA * (h_geom[i] - h_lpf[i-1])

    # --- 速度の突き合わせ ---
    #  range_h は ~30Hz でしか更新されない階段状。500Hz で素の中心差分を取ると
    #  「ほぼ 0 + たまに巨大スパイク」になる。実際に値が変わった点だけで微分し、
    #  元の時間軸へ線形補間で戻す (= ファームの climb と同じ土俵で比べる)。
    def step_deriv(y, tt_):
        chg = np.concatenate(([0], np.where(np.abs(np.diff(y)) > 1e-6)[0] + 1))
        if len(chg) < 3:
            return central_diff(y, tt_)
        tc, yc = tt_[chg], y[chg]
        vc = np.gradient(yc, tc)
        return np.interp(tt_, tc, vc)

    dh_dt   = step_deriv(range_h, tt)        # PIDが見ている高度の微分 (更新点ベース)
    dhg_dt  = step_deriv(h_geom,  tt)        # cos補正のみ(LPF前)の微分
    lag_s, _ = lag_via_xcorr(dh_dt, climb, dt_med)

    def rms(x):
        return float(np.sqrt(np.mean(x**2)))

    print("\n================ 距離ループ (外側 / 位置) ================")
    perr = hold - range_h
    print(f"  目標高度 alt_hold      : {hold.mean():.3f} m (min {hold.min():.3f} / max {hold.max():.3f})")
    print(f"  実測    range_h        : mean {range_h.mean():.3f}  min {range_h.min():.3f}  max {range_h.max():.3f} m")
    print(f"  位置誤差 (hold-range_h): mean {perr.mean():+.3f}  rms {rms(perr):.3f}  |max| {np.abs(perr).max():.3f} m")
    print(f"  出力 alt_vzt           : mean {vzt.mean():+.3f}  min {vzt.min():+.3f}  max {vzt.max():+.3f} m/s"
          f"   (上限 ±{ALT_POS_VZ_LIM})")
    sat = np.mean(np.abs(np.abs(vzt) - ALT_POS_VZ_LIM) < 1e-3) * 100
    print(f"  alt_vzt が上限に張り付いている割合: {sat:.0f} %")
    # kp 整合: vzt ?= clamp(kp*perr)
    vzt_pred = np.clip(ALT_POS_KP * perr, -ALT_POS_VZ_LIM, ALT_POS_VZ_LIM)
    print(f"  kp={ALT_POS_KP} 再現との一致 rms誤差: {rms(vzt - vzt_pred):.3f} m/s")

    print("\n================ 速度ループ (内側 / レート) ================")
    verr = vzt - climb
    print(f"  目標 alt_vzt : mean {vzt.mean():+.3f}  rms {rms(vzt):.3f} m/s")
    print(f"  実測 climb   : mean {climb.mean():+.3f}  rms {rms(climb):.3f}  |max| {np.abs(climb).max():.3f} m/s")
    print(f"  速度誤差     : mean {verr.mean():+.3f}  rms {rms(verr):.3f}  |max| {np.abs(verr).max():.3f} m/s")
    print(f"  出力 alt_corr: mean {corr.mean():+.4f}  min {corr.min():+.4f}  max {corr.max():+.4f}"
          f"   (上限 ±{ALT_THR_AUTH})")
    csat = np.mean(np.abs(np.abs(corr) - ALT_THR_AUTH) < 1e-3) * 100
    print(f"  alt_corr が権限に張り付いている割合: {csat:.0f} %")
    print(f"  alt_thr (出力スロットル): mean {athr.mean():.3f}  min {athr.min():.3f}  max {athr.max():.3f}")

    print("\n============ IMU(姿勢) × 距離センサ 整合性 ============")
    resid = range_h - h_lpf
    print(f"  cos補正の再現   range_h vs (raw*cosR*cosP をLPF): rms {rms(resid):.4f}  |max| {np.abs(resid).max():.4f} m")
    print(f"  傾き range       roll [{roll.min():+.1f}, {roll.max():+.1f}]  pitch [{pitch.min():+.1f}, {pitch.max():+.1f}] deg")
    over = np.mean(range_h > range_r + 1e-3) * 100
    print(f"  range_h > range_raw になっている割合: {over:.1f} %  (>0 なら cos補正の符号/スケール異常)")
    ratio = np.median(range_h / np.maximum(range_r, 1e-3))
    print(f"  median(range_h / range_raw): {ratio:.3f}  (傾きが小さければ ~1.0 のはず)")
    print()
    print(f"  climb vs d/dt(range_h):")
    print(f"     相関 {np.corrcoef(climb, dh_dt)[0,1]:+.3f}   "
          f"std比 climb/dhdt = {np.std(climb)/max(np.std(dh_dt),1e-9):.2f}   "
          f"rms差 {rms(climb - dh_dt):.3f} m/s")
    print(f"     climb の遅れ (二重LPFの位相遅れ推定): {lag_s*1000:+.0f} ms")
    print(f"  climb vs d/dt(raw*cos) : 相関 {np.corrcoef(climb, dhg_dt)[0,1]:+.3f}   "
          f"rms差 {rms(climb - dhg_dt):.3f} m/s  (生微分はノイズ大)")
    print(f"  ※ このログに IMU 由来の鉛直速度チャンネルは無い。climb は 100% 距離センサ由来"
          f"（加速度の相補フィルタは未実装）。")

    # ---------------- 遅れ内訳 (どこで 0.5秒 遅れるのか) ----------------
    print("\n============ 遅れ内訳  (信号がどの段で遅れているか) ============")

    import math
    Ts = 1.0 / 30.0    # VL53L1X continuous ~33ms

    # --- 解析値: 1次LPF の群遅延 tau = -Ts/ln(1-alpha)。ここは設定だけで決まる確定値 ---
    tau_h   = -Ts / math.log(1 - RANGE_H_ALPHA)  * 1000
    tau_vz  = -Ts / math.log(1 - RANGE_VZ_ALPHA) * 1000
    zoh     = 0.5 * Ts * 1000
    sensor  = 40.0
    print("  [解析] 設定から決まる確定遅れ:")
    print(f"     VL53L1X 測距 (budget33 + continuous33)     : ~{sensor:.0f} ms")
    print(f"     30Hz 更新の ZOH                            : ~{zoh:.0f} ms")
    print(f"     H-LPF   (RANGE_H_ALPHA={RANGE_H_ALPHA})  群遅延           : ~{tau_h:.0f} ms  -> range_h")
    print(f"     VZ-LPF  (RANGE_VZ_ALPHA={RANGE_VZ_ALPHA}) 群遅延          : ~{tau_vz:.0f} ms  -> climb (H-LPFに上乗せ)")
    print(f"     合計 (climb がリアル高度変化に遅れる量)     : ~{sensor+zoh+tau_h+tau_vz:.0f} ms")
    print(f"     + スロットル→鉛直速度の物理応答 ~100-150 ms  => 全体 ~0.4-0.5 s  (体感と一致)")

    print("\n  >>> 距離センサ素の遅れ (~40ms) は主因ではない。")
    print("      主因は H/VZ の二重ソフトLPF (~240ms) + 30Hz更新。RANGE_*_ALPHA を上げるか")
    print("      加速度Zの相補フィルタで vz を作れば、この 240ms はほぼ消せる。")
    print("      ※ ループ停止中は測距読みが数百ms凍るので、その区間だけは実質センサ遅れが支配。")
    print("      ※ このログは『1回登って落ちる』単発 + 停止多発で、相互相関による実測ラグは")
    print("        分離不能だった (相関 <0.1)。定常ホバリングが録れたら実測を足せる。")

    # ================= 作図 =================
    # 窓内に残った小さな停止でも微分がスパイクするので、線を切る
    brk = np.zeros(len(tt), dtype=bool)
    brk[1:] = np.diff(tt) > 0.008

    def cut(y):
        z = y.astype(float).copy()
        z[brk] = np.nan
        return z

    fig, ax = plt.subplots(5, 1, figsize=(13, 18), sharex=True)

    # (1) 距離ループ
    a = ax[0]
    a.plot(tt, cut(hold),    lw=2.0, label="alt_hold  目標高度 [m]")
    a.plot(tt, cut(range_h), lw=1.6, label="range_h  実測(LPF後) [m]")
    a.plot(tt, cut(range_r), lw=0.8, alpha=0.5, label="range_raw  斜め距離 [m]")
    a.set_ylabel("高度 [m]")
    a.set_title("(1) 距離ループ = 外側/位置  :  目標高度 vs 実測  (右軸 = 位置誤差)")
    a.grid(alpha=0.3); a.legend(loc="upper left", fontsize=8)
    ar = a.twinx()
    ar.plot(tt, cut(perr), color="tab:red", lw=0.9, alpha=0.7, label="誤差 hold-range_h [m]")
    ar.axhline(0, color="tab:red", lw=0.5, alpha=0.4)
    ar.set_ylabel("位置誤差 [m]", color="tab:red")
    ar.tick_params(axis="y", colors="tab:red")

    # (2) 距離ループ出力 = 速度ループ目標
    a = ax[1]
    a.plot(tt, cut(vzt),   lw=1.8, label="alt_vzt  目標上昇速度 [m/s] (= 距離ループ出力)")
    a.plot(tt, cut(climb), lw=1.4, label="climb  実測上昇速度 [m/s]")
    a.plot(tt, cut(dh_dt), lw=0.7, alpha=0.5, label="d/dt(range_h)  独立参照 [m/s]")
    a.axhline( ALT_POS_VZ_LIM, color="k", ls=":", lw=0.8)
    a.axhline(-ALT_POS_VZ_LIM, color="k", ls=":", lw=0.8, label=f"±ALT_POS_VZ_LIM ({ALT_POS_VZ_LIM})")
    _lo = min(vzt.min(), climb.min(), -ALT_POS_VZ_LIM) - 0.1
    _hi = max(vzt.max(), climb.max(),  ALT_POS_VZ_LIM) + 0.1
    a.set_ylim(_lo, _hi)
    a.set_ylabel("上昇速度 [m/s]")
    a.set_title("(2) 速度ループ = 内側/レート  :  目標 alt_vzt vs 実測 climb  (右軸 = 速度誤差)")
    a.grid(alpha=0.3); a.legend(loc="upper left", fontsize=8)
    ar = a.twinx()
    ar.plot(tt, cut(vzt - climb), color="tab:red", lw=0.9, alpha=0.7, label="誤差 vzt-climb [m/s]")
    ar.axhline(0, color="tab:red", lw=0.5, alpha=0.4)
    ar.set_ylabel("速度誤差 [m/s]", color="tab:red")
    ar.tick_params(axis="y", colors="tab:red")

    # (3) 速度ループ出力
    a = ax[2]
    a.plot(tt, cut(corr), lw=1.6, label="alt_corr  速度PID出力(補正) [割合]")
    a.plot(tt, cut(athr), lw=1.4, label="alt_thr  = HOVER_THR + corr [割合]")
    a.plot(tt, cut(thr),  lw=0.9, alpha=0.5, label="thr  スロットルスティック [割合]")
    a.axhline( ALT_HOVER_THR + ALT_THR_AUTH, color="k", ls=":", lw=0.8)
    a.axhline( ALT_HOVER_THR - ALT_THR_AUTH, color="k", ls=":", lw=0.8,
              label=f"HOVER {ALT_HOVER_THR} ± AUTH {ALT_THR_AUTH}")
    a.axhline( ALT_HOVER_THR, color="gray", ls="--", lw=0.7)
    a.set_ylabel("スロットル [割合]")
    a.set_title("(3) 速度ループ出力  :  alt_corr / alt_thr")
    a.grid(alpha=0.3); a.legend(loc="upper left", fontsize=8)

    # (4) スロットル(実出力) vs 上昇速度  -- 応答遅れが目で見える
    a = ax[3]
    a.plot(tt, cut(athr),   lw=1.6, color="tab:orange", label="alt_thr  高度PID出力 [割合]")
    a.plot(tt, cut(m_mean), lw=1.4, color="tab:blue",   label="モーター平均 (m1..m4) [割合]")
    a.fill_between(tt, cut(m_min), cut(m_max), color="tab:blue", alpha=0.15,
                   label="モーター min..max (姿勢制御ぶんの開き)")
    a.axhline(ALT_HOVER_THR, color="gray", ls="--", lw=0.7, label=f"ALT_HOVER_THR {ALT_HOVER_THR}")
    a.set_ylabel("スロットル [割合]")
    a.set_title("(4) スロットル実出力 vs 上昇速度  (右軸=climb)  ── 指令してから速度が動くまでの遅れを見る")
    a.grid(alpha=0.3); a.legend(loc="upper left", fontsize=8)
    ar = a.twinx()
    ar.plot(tt, cut(climb), color="tab:green", lw=1.5, label="climb  実測上昇速度 [m/s]")
    ar.plot(tt, cut(vzt),   color="tab:green", lw=0.8, ls=":", alpha=0.7, label="alt_vzt  目標 [m/s]")
    ar.axhline(0, color="gray", lw=0.5, alpha=0.4)
    ar.set_ylabel("上昇速度 [m/s]", color="tab:green")
    ar.tick_params(axis="y", colors="tab:green")
    ar.legend(loc="upper right", fontsize=8)

    # (5) 整合性: 傾きと残差
    a = ax[4]
    a.plot(tt, cut(roll),  lw=1.0, label="roll_ang [deg]")
    a.plot(tt, cut(pitch), lw=1.0, label="pitch_ang [deg]")
    a.set_ylabel("姿勢角 [deg]")
    a.set_title("(5) IMU×距離 整合性  :  climb(PIDが見る速度) vs d/dt(range_h)  と cos補正残差")
    a.grid(alpha=0.3); a.legend(loc="upper left", fontsize=8)
    ar = a.twinx()
    ar.plot(tt, cut(climb),  color="tab:green", lw=1.3, label="climb [m/s]")
    ar.plot(tt, cut(dh_dt),  color="tab:red",   lw=0.8, alpha=0.6, label="d/dt(range_h) [m/s]")
    ar.set_ylim(-0.8, 0.8)
    ar.axhline(0, color="gray", lw=0.5, alpha=0.4)
    ar.set_ylabel("上昇速度 [m/s]")
    ar.legend(loc="upper right", fontsize=8)

    ax[-1].set_xlabel("時間 [s]")
    for _a in ax:
        try:
            _a.set_xlim(tt[0], tt[-1])
        except Exception:
            pass
    fig.suptitle(f"高度ホールド解析  {args.csv}", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.985])

    out = args.png or (args.csv.rsplit(".", 1)[0] + "_altpid.png")
    fig.savefig(out, dpi=120)
    print(f"\nグラフを書き出しました: {out}")


if __name__ == "__main__":
    # 日本語フォント (無ければ豆腐になるが処理は通す)
    for fam in ["Yu Gothic", "Meiryo", "MS Gothic", "Noto Sans CJK JP"]:
        try:
            matplotlib.rcParams["font.family"] = fam
            break
        except Exception:
            pass
    matplotlib.rcParams["axes.unicode_minus"] = False
    main()
