# scripts/bin2csv.py
# SD カード (HW-125) に貯めた LOGnnnn.BIN を、analyze_log.py がそのまま読める
# CSV に変換する。
#
# 通常は機体側 (drone_s5) がディスアーム時に LOGnnnn.CSV を自動生成するので
# これは不要。使うのは:
#   * 変換が途中で切れた (電源を先に抜いた) とき
#   * ファームの Rec 形式が変わって、古い BIN を機体が弾いたとき
#     (--force で当時のフィールド定義を指定して吸い出す想定。既定は現行 v1)
#
# 【使い方】
#   python scripts/bin2csv.py  path/to/LOG0007.BIN
#   python scripts/bin2csv.py  E:\           # ドライブ直下の最新 .BIN を変換
#   python scripts/bin2csv.py  E:\ -o logs\  # 出力先を logs/ に
#
# 【バイナリ形式】 include/quad/SdLog.h と厳密に一致させること。
#   [32B ヘッダ]  b"S5LOG\0" | u8 fmt_ver | u8 rec_ver | u16 rec_size |
#                 u16 rate_hz | u32 t0_ms | 0 埋め
#   [rec_size バイトのレコード] × N     (RamLog::Rec そのまま, packed / little-endian)

import argparse
import struct
import sys
from pathlib import Path

# --- 現行フォーマット (drone_s5.cpp の RamLog::REC_VER と一致させる) -----------
# v2 (2026-09-09): accx/accy/accz の量子化スケール SC_1E4(1e4) -> 1000。
#   ±8g 化で acc_z が [-10,+6] を取りうるようになったため。v1 の BIN は
#   --force で吸えるが accx/accy/accz が 10 倍ずれるので注意。
# v3 (2026-09-12): GUIDED 診断用に cmd_req/cmd_age_ms/cmd_vx_mmps/cmd_vy_mmps/
#   cmd_alt_cm を末尾に追加。flags に RF_GUIDED_ENGAGED/RF_SW_AUTO_UP を追加。
REC_VER = 3

# QuadConfig.h の Q::ALT_HOVER_THR。alt_base 列に使う (機体側 formatRow と同じ)。
# ここは実機の設定とずれることがある。--hover-thr で上書きできる。
DEFAULT_HOVER_THR = 0.42

# RamLog::Rec の並び。(struct 文字, 名前) を宣言順に。packed なので '<' で詰める。
_FIELDS = [
    ("I", "t_ms"), ("H", "dt_us"), ("H", "flags"),
    ("B", "mode"), ("B", "mixsat"), ("B", "thr"),
    ("b", "roll_stick"), ("b", "pitch_stick"), ("b", "yaw_stick"),
    ("h", "roll_ang"), ("h", "pitch_ang"), ("h", "yaw_est"),
    ("h", "roll_rate"), ("h", "pitch_rate"), ("h", "yaw_rate"),
    ("h", "roll_cmd"), ("h", "pitch_cmd"), ("h", "yaw_cmd"),
    ("B", "m1"), ("B", "m2"), ("B", "m3"), ("B", "m4"),
    ("h", "span_limit"),
    ("h", "roll_ratetar"), ("h", "pitch_ratetar"),
    ("h", "roll_angtar"), ("h", "pitch_angtar"),
    ("h", "flow_raw_x"), ("h", "flow_raw_y"), ("h", "flow_dx"), ("h", "flow_dy"),
    ("h", "flow_vx"), ("h", "flow_vy"), ("h", "flow_h"),
    ("f", "flow_accx"), ("f", "flow_accy"),
    ("h", "fh_vxc"), ("h", "fh_vyc"), ("h", "fh_vxt"), ("h", "fh_vyt"),
    ("h", "fh_leanr"), ("h", "fh_leanp"),
    ("h", "fh_posn"), ("h", "fh_pose"), ("h", "fh_holdn"), ("h", "fh_holde"),
    ("h", "range_raw"), ("h", "range_h"), ("h", "climb"),
    ("h", "alt_holdm"), ("h", "alt_vzt"), ("h", "alt_corr"),
    ("B", "alt_thr_out"), ("B", "alt_used"),
    ("h", "accx"), ("h", "accy"), ("h", "accz"),
    ("h", "acc_up"), ("h", "est_h"), ("h", "est_vz"), ("h", "est_bias"),
    ("B", "cmd_req"), ("H", "cmd_age_ms"),
    ("h", "cmd_vx_mmps"), ("h", "cmd_vy_mmps"), ("h", "cmd_alt_cm"),
]
_REC_STRUCT = struct.Struct("<" + "".join(c for c, _ in _FIELDS))
_NAMES = [n for _, n in _FIELDS]

# 出力する CSV の列名 (drone_s5.cpp の Log::HEADER と同一・同順)
HEADER = (
    "t_ms,dt_us,mode,armed,thr,"
    "roll_sbus,pitch_sbus,yaw_sbus,"
    "roll_ang,pitch_ang,yaw_ang,"
    "roll_gyr,pitch_gyr,yaw_gyr,"
    "roll_cmd,pitch_cmd,yaw_cmd,"
    "m1,m2,m3,m4,corr_limit,sat,"
    "roll_ratetar,pitch_ratetar,roll_angtar,pitch_angtar,"
    "flow_ok,flow_raw_x,flow_raw_y,flow_dx,flow_dy,flow_vx,flow_vy,flow_h,"
    "flow_accx,flow_accy,"
    "fh_vxc,fh_vyc,fh_vxt,fh_vyt,fh_leanr,fh_leanp,fh_posn,fh_pose,"
    "fh_holdn,fh_holde,fh_hold,"
    "range_ok,range_raw,range_h,climb,alt_en,alt_act,alt_hold,alt_vzt,"
    "alt_base,alt_corr,alt_thr,"
    "alt_used,"
    "accx,accy,accz,"
    "acc_up,est_h,est_vz,est_bias,"
    "guided_engaged,sw_auto,cmd_req,cmd_age_ms,cmd_vx,cmd_vy,cmd_alt_cm"
)

# flags のビット (RamLog::RFlag)
(RF_ARMED, RF_FLOW_OK, RF_RANGE_OK, RF_ALT_EN, RF_ALT_ACT, RF_HOLDING,
 RF_GUIDED_ENGAGED, RF_SW_AUTO_UP) = (
    1 << 0, 1 << 1, 1 << 2, 1 << 3, 1 << 4, 1 << 5, 1 << 6, 1 << 7)


def _row(rec, hover_thr):
    d = dict(zip(_NAMES, _REC_STRUCT.unpack(rec)))
    f = d["flags"]
    armed   = 1 if f & RF_ARMED    else 0
    flow_ok = 1 if f & RF_FLOW_OK  else 0
    rng_ok  = 1 if f & RF_RANGE_OK else 0
    alt_en  = 1 if f & RF_ALT_EN   else 0
    alt_act = 1 if f & RF_ALT_ACT  else 0
    holding = 1 if f & RF_HOLDING  else 0
    guided_engaged = 1 if f & RF_GUIDED_ENGAGED else 0
    sw_auto        = 1 if f & RF_SW_AUTO_UP     else 0
    alt_base = hover_thr if alt_act else 0.0

    # 機体側 formatRow() と同じ桁数で並べる (analyze_log.py は列名参照なので
    # 桁数は本質ではないが、目視差分を取りやすいよう合わせておく)。
    return ",".join(str(x) for x in [
        d["t_ms"], d["dt_us"], d["mode"], armed, f'{d["thr"]/250:.3f}',
        f'{d["roll_stick"]/100:.3f}', f'{d["pitch_stick"]/100:.3f}', f'{d["yaw_stick"]/100:.3f}',
        f'{d["roll_ang"]/100:.2f}', f'{d["pitch_ang"]/100:.2f}', f'{d["yaw_est"]/100:.2f}',
        f'{d["roll_rate"]/10:.2f}', f'{d["pitch_rate"]/10:.2f}', f'{d["yaw_rate"]/10:.2f}',
        f'{d["roll_cmd"]/1e4:.4f}', f'{d["pitch_cmd"]/1e4:.4f}', f'{d["yaw_cmd"]/1e4:.4f}',
        f'{d["m1"]/250:.3f}', f'{d["m2"]/250:.3f}', f'{d["m3"]/250:.3f}', f'{d["m4"]/250:.3f}',
        f'{d["span_limit"]/1000:.3f}', d["mixsat"],
        f'{d["roll_ratetar"]/10:.1f}', f'{d["pitch_ratetar"]/10:.1f}',
        f'{d["roll_angtar"]/100:.1f}', f'{d["pitch_angtar"]/100:.1f}',
        flow_ok,
        f'{d["flow_raw_x"]/10:.1f}', f'{d["flow_raw_y"]/10:.1f}',
        f'{d["flow_dx"]/10:.1f}', f'{d["flow_dy"]/10:.1f}',
        f'{d["flow_vx"]/1000:.3f}', f'{d["flow_vy"]/1000:.3f}', f'{d["flow_h"]/1000:.2f}',
        f'{d["flow_accx"]:.4f}', f'{d["flow_accy"]:.4f}',
        f'{d["fh_vxc"]/1000:.3f}', f'{d["fh_vyc"]/1000:.3f}',
        f'{d["fh_vxt"]/1000:.3f}', f'{d["fh_vyt"]/1000:.3f}',
        f'{d["fh_leanr"]/100:.2f}', f'{d["fh_leanp"]/100:.2f}',
        f'{d["fh_posn"]/1000:.3f}', f'{d["fh_pose"]/1000:.3f}',
        f'{d["fh_holdn"]/1000:.3f}', f'{d["fh_holde"]/1000:.3f}', holding,
        rng_ok, f'{d["range_raw"]/1000:.3f}', f'{d["range_h"]/1000:.3f}', f'{d["climb"]/1000:.3f}',
        alt_en, alt_act,
        f'{d["alt_holdm"]/1000:.3f}', f'{d["alt_vzt"]/1000:.3f}', f'{alt_base:.3f}',
        f'{d["alt_corr"]/1e4:.4f}', f'{d["alt_thr_out"]/250:.3f}',
        f'{d["alt_used"]/250:.3f}',
        f'{d["accx"]/1000:.4f}', f'{d["accy"]/1000:.4f}', f'{d["accz"]/1000:.4f}',
        f'{d["acc_up"]/1000:.3f}', f'{d["est_h"]/1000:.3f}',
        f'{d["est_vz"]/1000:.3f}', f'{d["est_bias"]/1000:.3f}',
        guided_engaged, sw_auto, d["cmd_req"], d["cmd_age_ms"],
        f'{d["cmd_vx_mmps"]/1000:.3f}', f'{d["cmd_vy_mmps"]/1000:.3f}',
        f'{d["cmd_alt_cm"]:.1f}',
    ])


def _pick_bin(arg):
    p = Path(arg)
    if p.is_file():
        return p
    if p.is_dir():
        bins = sorted(p.glob("LOG*.BIN")) + sorted(p.glob("LOG*.bin"))
        if not bins:
            sys.exit(f"[ERROR] {p} に LOG*.BIN がありません")
        return bins[-1]
    sys.exit(f"[ERROR] 見つかりません: {arg}")


def main():
    ap = argparse.ArgumentParser(description="LOGnnnn.BIN -> CSV (analyze_log.py 用)")
    ap.add_argument("path", help="BIN ファイル、または .BIN を含むフォルダ (最新を選ぶ)")
    ap.add_argument("-o", "--out", help="出力先フォルダ or ファイル (既定: BIN と同じ場所)")
    ap.add_argument("--hover-thr", type=float, default=DEFAULT_HOVER_THR,
                    help=f"alt_base 列に使うホバースロットル (既定 {DEFAULT_HOVER_THR})")
    ap.add_argument("--force", action="store_true",
                    help="rec_ver / rec_size 不一致でも強行する")
    args = ap.parse_args()

    binpath = _pick_bin(args.path)
    raw = binpath.read_bytes()
    if len(raw) < 32 or raw[:6] != b"S5LOG\0":
        sys.exit(f"[ERROR] ヘッダが S5LOG ではありません: {binpath}")

    fmt_ver = raw[6]
    rec_ver = raw[7]
    rec_size = raw[8] | (raw[9] << 8)
    rate_hz = raw[10] | (raw[11] << 8)
    t0_ms = struct.unpack_from("<I", raw, 12)[0]

    print(f"[INFO] {binpath.name}  fmt_ver={fmt_ver} rec_ver={rec_ver} "
          f"rec_size={rec_size} rate={rate_hz}Hz t0={t0_ms}ms")

    if (rec_ver != REC_VER or rec_size != _REC_STRUCT.size) and not args.force:
        sys.exit(f"[ERROR] このスクリプトは rec_ver={REC_VER} / "
                 f"rec_size={_REC_STRUCT.size} 用です。--force で強行できますが "
                 f"値がずれる可能性があります。")

    body = raw[32:]
    step = rec_size if args.force else _REC_STRUCT.size
    n_full = len(body) // step
    tail = len(body) - n_full * step
    if tail:
        print(f"[WARN] 末尾 {tail} バイトが半端です (電源断で切れた?)。無視します。")

    # --- preAllocate の未使用領域を切り落とす -------------------------------
    #  SdLog::startFile() は PREALLOC (16MB) を先に確保し、stopFile() が
    #  truncate して余りを返す。ディスアームせずに電源を抜くと stopFile() が
    #  走らないので、実データの後ろに未書き込み領域がまるごと残る。
    #  未書き込みセクタは 0xFF (または 0x00) で読めるので、そこで打ち切る。
    #  (LOG0054 は 9.1MB の実データ + 7MB の 0xFF で、そのまま変換すると
    #   t_ms=4294967295 のゴミ行が 7 万行できていた)
    def _is_blank(rec):
        return rec.count(0xFF) == len(rec) or rec.count(0x00) == len(rec)

    n_valid = n_full
    for i in range(n_full):
        if _is_blank(body[i * step:i * step + step]):
            n_valid = i
            break
    if n_valid < n_full:
        print(f"[WARN] {n_valid} 行目以降が未書き込み領域 (preAllocate の余り) でした。"
              f"{n_full - n_valid} 行を切り捨てます。")
        print("       → ディスアームしてから電源を切ると truncate されます。")
        n_full = n_valid
    if n_full == 0:
        sys.exit("[ERROR] 有効なレコードがありません。")

    if args.out:
        outp = Path(args.out)
        outp = outp / (binpath.stem + ".csv") if outp.is_dir() or args.out.endswith(("\\", "/")) else outp
    else:
        outp = binpath.with_suffix(".csv")
    outp.parent.mkdir(parents=True, exist_ok=True)

    with outp.open("w", encoding="utf-8", newline="") as fo:
        fo.write(HEADER + "\n")
        for i in range(n_full):
            off = i * step
            fo.write(_row(body[off:off + _REC_STRUCT.size], args.hover_thr) + "\n")

    print(f"[OK] {n_full} 行 -> {outp}")


if __name__ == "__main__":
    main()
