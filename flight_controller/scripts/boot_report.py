# scripts/boot_report.py
# BLE ログ (LOGnnnn.BIN) のヘッダから「その起動のリセット原因」と
# 「前回ループが止まった区間 (ウォッチドッグの記録)」を読む。
#
#   python scripts/boot_report.py                 # position_estimator/src/logs の最新 15 本
#   python scripts/boot_report.py -n 40
#   python scripts/boot_report.py path/to/logs    # フォルダを指定
#   python scripts/boot_report.py LOG0062.BIN     # 1 本だけ
#
# ★ 2026-09-18: RP2040 版 FC はバッテリー接続中に USB を認識しないので、再起動の
#   理由を USB で見られない。FC は起動のたびにログヘッダ後半 16B へ記録を載せ、
#   電源投入以外の再起動なら、アームしなくても起動後 BOOT_REPORT_MS の間 BLE ログを
#   1 本送る (flight_controller/include/quad/FcWatchdog.h fillHeaderExtra)。
#
# 見方:
#   リセット原因 PWRON   … 普通の電源投入 (バッテリーを挿した)
#   リセット原因 WDT     … ループが止まってウォッチドッグがリセットした
#                          → 「止まった区間」と「そのときの状態」を見る
#   それ以外 (SOFT は書き込み直後など)
#
# ★ 下の名前の並びはファームの enum と同じ順番。ファーム側を増やしたらここも足す。
#   区間: quad/FcWatchdog.h Section / 段階: quad/Guided.h GuidedPhase / モード: protocol/S5Telem.h

import argparse
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_DIR = HERE.parent.parent / "position_estimator" / "src" / "logs"

RESET = ["UNKNOWN", "PWRON (電源投入)", "RUN_PIN", "SOFT", "WDT (ウォッチドッグ)", "DEBUG",
         "GLITCH", "BROWNOUT"]
SECTION = [
    "-", "IMU 読み (I2C)", "SBUS", "フロー (LogLink)", "測距 (I2C)",
    "地上局受信/GUIDED", "姿勢制御/ESC", "USB キー入力", "ログ/LED/LogLink 送信",
    "テレメトリ", "デバッグ画面 (USB)",
    "フェイルセーフ判定", "フロー位置ホールド (PosHold)", "高度ホールド (AltHold)",
    "GUIDED 更新 (周回/直進)", "LED", "ログファイル開閉", "ログ 1 行作成",
    "LogLink UART 送受信", "BLE メンテ指令", "アーム判定",
]
PHASE = ["OFF", "HOLD", "TAKEOFF", "CRUISE", "LAND", "LANDED", "MANEUV", "CIRCLE", "DESCEND",
         "ALIGN", "STRAIGHT"]
MODE = ["RATE", "ANGLE", "GUIDED", "POSHOLD", "ALTHOLD"]
FS = ["-", "自動着陸", "モーター停止"]


def name(tbl, i):
    return tbl[i] if 0 <= i < len(tbl) else f"?({i})"


def read_one(p: Path) -> str:
    raw = p.read_bytes()[:32]
    if len(raw) < 32 or raw[:6] != b"S5LOG\0":
        return f"{p.name}: (S5LOG ヘッダではない / 空)"
    t0 = struct.unpack_from("<I", raw, 12)[0]
    size = p.stat().st_size
    mtime = time.strftime("%m-%d %H:%M:%S", time.localtime(p.stat().st_mtime))
    head = f"{p.name}  {mtime}  {size // 1024:>5} KB  記録開始 起動+{t0 / 1000:.1f}s"
    ex = raw[16:32]
    if ex[:2] != b"WD":
        return head + "  | (記録なし: 古いファーム)"
    reason, wd, sec = ex[3], ex[4], ex[5]
    st = struct.unpack_from("<H", ex, 6)[0]
    ms = struct.unpack_from("<I", ex, 8)[0]
    line = head + f"  | リセット原因 {name(RESET, reason)}"
    if wd:
        armed = (st >> 12) & 1
        fs = (st >> 8) & 0xF
        mode = (st >> 4) & 0xF
        gp = st & 0xF
        line += (f"\n      ★ 前回ループが止まった: 区間「{name(SECTION, sec)}」 起動 {ms / 1000:.1f}s 後"
                 f"  mode={name(MODE, mode)} guided={name(PHASE, gp)} "
                 f"failsafe={name(FS, fs)} armed={armed}")
    return line


def main():
    ap = argparse.ArgumentParser(description="BLE ログのヘッダからリセット原因を読む")
    ap.add_argument("path", nargs="?", default=str(DEFAULT_DIR), help="LOG*.BIN かフォルダ")
    ap.add_argument("-n", type=int, default=15, help="フォルダのとき新しい順に何本")
    args = ap.parse_args()

    p = Path(args.path)
    if p.is_file():
        files = [p]
    elif p.is_dir():
        # Windows は大文字小文字を区別しないので BIN/bin の両方で同じファイルが出る
        found = {f.resolve() for f in list(p.glob("LOG*.BIN")) + list(p.glob("LOG*.bin"))}
        files = sorted(found, key=lambda f: f.stat().st_mtime)[-args.n:]
    else:
        sys.exit(f"[ERROR] 見つかりません: {p}")
    for f in files:
        print(read_one(f))


if __name__ == "__main__":
    main()
