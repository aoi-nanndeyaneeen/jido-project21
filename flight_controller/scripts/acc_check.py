# scripts/acc_check.py
# 加速度センサの動作確認 + 水平キャリブ (リセット) を USB シリアルで行う。s5 (drone_s5 / drone_s5_rp2040) 用。
#
#   python scripts/acc_check.py              # ポート自動検出
#   python scripts/acc_check.py --port COM7
#
#   ★ VSCode のシリアルモニタ / logger.py は閉じておく (COM ポートの取り合いになる)
#   ★ 電源投入 5 秒後にファームが自動で 'k' を走らせるので、起動直後は触らないこと
#
# 【何を見ているか】
#   ファームの 'l' (500Hz USB ログ) を流し、accx/accy/accz と roll_ang/pitch_ang を読む。
#   accx/y/z はファーム内で バイアス除去 → IMU_MOUNT 符号 → FRD 変換 まで済んだ値
#   (= 姿勢推定と高度推定が実際に使っている値) なので、ここが合っていれば配線・符号・スケールは正しい。
#
#   FRD (前・右・下) で「重力の反力」が出るので、静止時の正解は:
#     [1] 水平 (普通に置く)       accz = -1g
#     [2] 裏返し                  accz = +1g
#     [3] 機首を真上に向ける      accx = +1g
#     [4] 機首を真下に向ける      accx = -1g
#     [5] 右側面を下 (右90°バンク) accy = -1g
#     [6] 左側面を下 (左90°バンク) accy = +1g
#
# 【キー】
#   1〜6 : 上の姿勢で静止させて押す → 1 秒平均を取り、軸と符号を判定
#          対になる 2 面 (1+2 / 3+4 / 5+6) が揃うと、その軸のオフセットとスケールを出す
#   k    : 水平キャリブ (ファームの 'k')。水平な床に置いて静止させてから押す。成功すると EEPROM に保存
#   x    : EEPROM のキャリブ値を消去 (ファームの 'x')。再起動後 Config.h の値に戻る
#   c    : 1〜6 の記録をクリア
#   q    : 終了
#
#   pip install pyserial

import argparse
import math
import msvcrt
import sys
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports

BAUD = 115200               # USB CDC なので実際には無視される
WINDOW_S = 0.3              # ライブ表示の平均窓 [s]
CAPTURE_S = 1.0             # 1〜6 の静止計測時間 [s]
STILL_STD_G = 0.03          # これより揺れていたら「動いた」とみなす [g]
AXIS_TOL_G = 0.10           # 主軸の |値| が 1g からこれ以上ずれたら NG
CROSS_TOL_G = 0.26          # 他軸がこれを超えたら「90° 置けていない」(約15°)
LEVEL_TILT_WARN_DEG = 2.0   # 水平置きで加速度から求めた傾きがこれを超えたら注意
FUSION_DIFF_WARN_DEG = 3.0  # 静止時に Madgwick 角度と加速度の傾きがこれ以上違ったら注意

# key: (名前, 軸 index 0=x 1=y 2=z, 期待符号)
POSES = {
    "1": ("水平",           2, -1),
    "2": ("裏返し",         2, +1),
    "3": ("機首を真上",     0, +1),
    "4": ("機首を真下",     0, -1),
    "5": ("右側面を下",     1, -1),
    "6": ("左側面を下",     1, +1),
}
AXIS_NAME = ("accx(前)", "accy(右)", "accz(下)")
PAIRS = (("3", "4", 0), ("6", "5", 1), ("2", "1", 2))   # (+1g 側, -1g 側, 軸)


# ============================================================
#  シリアル受信 (別スレッド)
# ============================================================
class Link:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.05)
        try:
            self.ser.set_buffer_size(rx_size=4 * 1024 * 1024)
        except Exception:
            pass
        self.cols = None
        self.samples = deque()          # (受信時刻, ax, ay, az, roll, pitch)
        self.logging = False
        self.msgs = deque()             # DATA 以外の行 (メインスレッドが表示)
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._rx, daemon=True).start()

    def send(self, ch):
        self.ser.write(ch.encode())

    def _rx(self):
        buf = b""
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                self.msgs.append(f"!! シリアル切断: {e}")
                self.running = False
                return
            if not chunk:
                continue
            buf += chunk
            *lines, buf = buf.split(b"\n")
            now = time.monotonic()
            for raw in lines:
                line = raw.decode("utf-8", errors="replace").strip()
                if line.startswith("DATA,"):
                    self._data(now, line)
                elif line.startswith("HEADER,"):
                    names = line[len("HEADER,"):].split(",")
                    self.cols = {n: i for i, n in enumerate(names)}
                elif line == "LOG_START":
                    self.logging = True
                elif line == "LOG_STOP":
                    self.logging = False
                elif line:
                    self.msgs.append(line)

    def _data(self, now, line):
        if self.cols is None:
            return
        f = line[len("DATA,"):].split(",")
        c = self.cols
        try:
            s = (now, float(f[c["accx"]]), float(f[c["accy"]]), float(f[c["accz"]]),
                 float(f[c["roll_ang"]]), float(f[c["pitch_ang"]]))
        except (KeyError, IndexError, ValueError):
            return
        with self.lock:
            self.samples.append(s)
            while self.samples and now - self.samples[0][0] > 5.0:
                self.samples.popleft()

    def recent(self, seconds):
        t0 = time.monotonic() - seconds
        with self.lock:
            return [s for s in self.samples if s[0] >= t0]

    def close(self):
        self.running = False
        self.ser.close()


# ============================================================
#  計算
# ============================================================
def mean_std(rows, i):
    v = [r[i] for r in rows]
    m = sum(v) / len(v)
    return m, math.sqrt(sum((x - m) ** 2 for x in v) / len(v))


def acc_tilt(ax, ay, az):
    """FRD の比力から (roll, pitch) [deg]。右バンク +, 機首上げ +"""
    roll = math.degrees(math.atan2(-ay, -az))
    pitch = math.degrees(math.atan2(ax, math.hypot(ay, az)))
    return roll, pitch


# ============================================================
#  UI
# ============================================================
def say(msg=""):
    sys.stdout.write("\r" + " " * 118 + "\r" + msg + "\n")
    sys.stdout.flush()


def find_port(arg):
    if arg:
        return arg
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if p.vid == 0x2E8A or p.vid == 0x16C0 or "teensy" in desc:   # RP2040 / Teensy
            return p.device
    if not ports:
        sys.exit("[ERROR] COM ポートがありません")
    print("自動検出できませんでした。接続中の COM ポート:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    return ports[int(input("番号 > ").strip())].device


def ensure_logging(link, want):
    """'l' はトグルなので、LOG_START/LOG_STOP を見て目的の状態にする"""
    for _ in range(3):
        if link.logging == want and (not want or link.recent(0.3)):
            return True
        link.send("l")
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            time.sleep(0.05)
            if link.logging == want and (not want or link.recent(0.2)):
                return True
    return link.logging == want


def flush_msgs(link):
    while link.msgs:
        say("  fw> " + link.msgs.popleft())


def capture(link, key, results):
    name, axis, sign = POSES[key]
    say(f"[{key}] {name}: {CAPTURE_S:.0f} 秒計測中... 動かさないでください")
    time.sleep(CAPTURE_S)
    rows = link.recent(CAPTURE_S)
    if len(rows) < 50:
        say(f"  !! サンプルが {len(rows)} 個しか来ていません (USB ログが流れていない?)")
        return
    m = [mean_std(rows, i) for i in (1, 2, 3)]
    mean = [x[0] for x in m]
    std = max(x[1] for x in m)
    norm = math.sqrt(sum(v * v for v in mean))
    say(f"  accx={mean[0]:+.3f} accy={mean[1]:+.3f} accz={mean[2]:+.3f} g   "
        f"|a|={norm:.3f} g   揺れ(最大std)={std:.3f} g   n={len(rows)}")

    ng = []
    if std > STILL_STD_G:
        ng.append(f"動いています (std {std:.3f} > {STILL_STD_G}) → 置き直して再計測")
    dom = max(range(3), key=lambda i: abs(mean[i]))
    if dom != axis:
        ng.append(f"重力が {AXIS_NAME[dom]} に出ています。期待は {AXIS_NAME[axis]} "
                  f"→ 軸の入れ替わり (SWAP_XY / 取付け向き) を疑う")
    elif (mean[axis] > 0) != (sign > 0):
        ng.append(f"{AXIS_NAME[axis]} の符号が逆 (期待 {sign:+d}g) → IMU_MOUNT の符号を疑う")
    elif abs(abs(mean[axis]) - 1.0) > AXIS_TOL_G:
        ng.append(f"{AXIS_NAME[axis]} の大きさが {abs(mean[axis]):.3f} g "
                  f"(±{AXIS_TOL_G} 外) → スケール (±8g 設定) かバイアスを疑う")
    for i in range(3):
        if dom == axis and i != axis and abs(mean[i]) > CROSS_TOL_G:
            ng.append(f"{AXIS_NAME[i]} が {mean[i]:+.3f} g → 90° に置けていない (傾き約 "
                      f"{math.degrees(math.asin(min(1.0, abs(mean[i])))):.0f}°)")
    if key == "1":
        r, p = acc_tilt(*mean)
        tilt = math.hypot(r, p)
        if tilt > LEVEL_TILT_WARN_DEG:
            ng.append(f"水平置きなのに加速度の傾き roll={r:+.1f}° pitch={p:+.1f}° "
                      f"→ 床が傾いているか、キャリブ ('k') がずれている")

    if ng:
        for s in ng:
            say("  NG: " + s)
    else:
        say(f"  OK: {name} で {AXIS_NAME[axis]} = {mean[axis]:+.3f} g")
    if std > STILL_STD_G or dom != axis:
        say("  (この計測は記録しません)")
        return
    results[key] = mean
    report_pair(results, key)


def report_pair(results, key):
    for kp, km, axis in PAIRS:
        if key in (kp, km) and kp in results and km in results:
            plus, minus = results[kp][axis], results[km][axis]
            offset = (plus + minus) / 2
            scale = (plus - minus) / 2
            ok = abs(offset) < 0.05 and abs(scale - 1.0) < 0.05
            say(f"  == {AXIS_NAME[axis]}: オフセット {offset:+.3f} g  スケール {scale:.3f}  "
                f"{'OK' if ok else 'NG (目安: |オフセット|<0.05, スケール 0.95〜1.05)'}")
            if axis == 2 and abs(offset) >= 0.05:
                say("     (accz のオフセットは 'k' で水平を 1g に合わせた結果の誤差。"
                    "裏返しとの差が大きいならセンサ固有のスケール誤差)")


def status_line(link):
    rows = link.recent(WINDOW_S)
    if not rows:
        return "  (データ待ち... 'l' のログが来ていません)"
    ax, sx = mean_std(rows, 1)
    ay, sy = mean_std(rows, 2)
    az, sz = mean_std(rows, 3)
    roll, _ = mean_std(rows, 4)
    pitch, _ = mean_std(rows, 5)
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    ar, ap = acc_tilt(ax, ay, az)
    hz = len(rows) / WINDOW_S
    std = max(sx, sy, sz)
    s = (f"acc x{ax:+.3f} y{ay:+.3f} z{az:+.3f} |a|{norm:.3f}g ±{std:.3f} | "
         f"傾き(加速度) R{ar:+6.1f} P{ap:+6.1f} | 姿勢推定 R{roll:+6.1f} P{pitch:+6.1f} | {hz:3.0f}Hz")
    if std < STILL_STD_G and max(abs(ar - roll), abs(ap - pitch)) > FUSION_DIFF_WARN_DEG \
            and abs(ap) < 60:
        s += " ←推定と不一致"
    return s


def main():
    ap = argparse.ArgumentParser(description="s5 加速度センサ確認")
    ap.add_argument("--port")
    args = ap.parse_args()

    port = find_port(args.port)
    print(f"接続: {port}")
    link = Link(port)
    time.sleep(0.3)
    if not ensure_logging(link, True):
        print("!! USB ログ ('l') が始まりません。ファームが drone_s5 系か、起動直後でないか確認してください")

    print("キー: 1=水平 2=裏返し 3=機首上 4=機首下 5=右面下 6=左面下 | k=水平キャリブ x=EEPROM消去 c=記録クリア q=終了")
    print("正解 (静止時, FRD): 水平 accz=-1 / 機首上 accx=+1 / 右面下 accy=-1。 "
          "表示の R/P は 右バンク+, 機首上げ+ (姿勢推定側はトリム差し引き後)")
    results = {}

    try:
        while link.running:
            flush_msgs(link)
            if msvcrt.kbhit():
                key = msvcrt.getwch().lower()
                if key == "q":
                    break
                elif key in POSES:
                    capture(link, key, results)
                elif key == "c":
                    results.clear()
                    say("記録をクリアしました")
                elif key == "k":
                    say(">>> 水平キャリブ: 水平な床に置いて手を離してください (2 秒後に開始)")
                    time.sleep(2.0)
                    ensure_logging(link, False)   # ログを止めないとキャリブ中の表示が埋もれる
                    link.send("k")
                    t_end = time.monotonic() + 4.0
                    while time.monotonic() < t_end:
                        flush_msgs(link)
                        time.sleep(0.05)
                    ensure_logging(link, True)
                    results.clear()
                    say(">>> 完了。上の fw> 行で REJECTED が出ていないか確認。1〜6 の記録はクリアしました")
                elif key == "x":
                    say("EEPROM のキャリブ値を消去します。よろしいですか? (y/n)")
                    if msvcrt.getwch().lower() == "y":
                        ensure_logging(link, False)
                        link.send("x")
                        time.sleep(0.5)
                        flush_msgs(link)
                        ensure_logging(link, True)
                        say(">>> 消去しました。電源を入れ直すと Config.h の値に戻ります "
                            "(起動 5 秒後の自動 'k' が成功すればまた保存されます)")
                    else:
                        say("キャンセル")
            sys.stdout.write("\r" + status_line(link)[:118].ljust(118))
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        say()
        ensure_logging(link, False)
        link.close()
        print("終了")


if __name__ == "__main__":
    main()
