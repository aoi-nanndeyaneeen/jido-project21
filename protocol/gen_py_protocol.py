#!/usr/bin/env python3
"""
gen_py_protocol.py  -  S5Cmd.h / S5Telem.h から Python 側の定数モジュールを生成する。

    python protocol/gen_py_protocol.py            # 生成 (position_estimator/src/core/s5_protocol.py)
    python protocol/gen_py_protocol.py --check    # 生成物が最新か検査するだけ (焼く前の確認)

なぜ生成するか:
    REQ_* / CF_* / MODE 名 / AltState 名 が s5_link.py・mission.py・console.py・
    utils/logger.py・ble_tap.py に手書きで散らばっていて、console.py の
    ALT_STATE_NAME が機体の実体とずれたまま表示されていた (2026-09-16 発見)。
    C++ ヘッダを唯一の定義とし、Python はそこから機械的に作る。

拾うもの (正規表現。ヘッダの書き方に依存するので、書式を大きく変えたらここも直す):
    namespace S5C / S5T の
      - constexpr uint8_t VERSION = N;
      - constexpr uint8_t MAGIC / TYPE_* = 0x..;   constexpr size_t IM920SL_MAX_PAYLOAD 等
      - constexpr float SC_* = ...;
      - enum <Name> [: type] { MEMBER = EXPR, ... }   (EXPR は整数 / 0x / 1u << n)
      - inline const char* xxxName(uint8_t) { switch { case MEMBER: return "文字列"; } }
        → 表示名の対応表 (XXX_NAME: 値 -> 文字列) にする
"""

import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
HEADERS = [HERE / "S5Cmd.h", HERE / "S5Telem.h"]
OUT = HERE.parent / "position_estimator" / "src" / "core" / "s5_protocol.py"

_INT_CONST = re.compile(
    r"constexpr\s+(?:uint8_t|uint16_t|size_t|int)\s+([A-Z][A-Z0-9_]*)\s*=\s*([^;]+);")
_FLOAT_CONST = re.compile(r"constexpr\s+float\s+(SC_[A-Z0-9_]*)\s*=\s*([0-9.]+)f?\s*;")
_ENUM = re.compile(r"enum\s+(?:class\s+)?(\w+)\s*(?::\s*\w+)?\s*\{(.*?)\}\s*;", re.S)
_ENUM_MEMBER = re.compile(r"^\s*([A-Za-z_]\w*)\s*=\s*([^,/]+?)\s*,?\s*(?://.*)?$")
_NAME_FN = re.compile(
    r"inline\s+const\s+char\*\s+(\w+)\s*\(\s*uint8_t\s+\w+\s*\)\s*\{(.*?)\n\}", re.S)
_CASE = re.compile(r"case\s+([\w:]+)\s*:\s*return\s+\"([^\"]*)\"\s*;")
_NAMESPACE = re.compile(r"namespace\s+(S5[CT])\s*\{")

# enum class は Python に名前空間が無いので、メンバ名にプレフィクスを付ける
_ENUM_CLASS_PREFIX = {"AltState": "ALT_"}


def _im920_consts() -> dict:
    """Im920Frame.h の整数定数 (MAX_PAYLOAD / CHECKSUM_BYTES / MAX_DATA)。S5*.h が参照する。"""
    out = {}
    text = _strip_comments((HERE / "Im920Frame.h").read_text(encoding="utf-8"))
    for m in _INT_CONST.finditer(text):
        try:
            out[m.group(1)] = _eval_int(m.group(2), out)
        except ValueError:
            pass
    return out


def _strip_comments(src: str) -> str:
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


def _eval_int(expr: str, known: dict) -> int:
    e = expr.strip().replace("IM920::", "")     # Im920Frame.h の定数を参照する式
    for k, v in known.items():                            # 既知の定数名を展開
        e = re.sub(rf"\b{k}\b", str(v), e)
    e = re.sub(r"(\d)[uU][lL]?[lL]?\b", r"\1", e)        # 1u / 32u
    if not re.fullmatch(r"[0-9a-fA-Fx<>|()+\-* ]+", e):
        raise ValueError(f"not an integer expression: {expr!r}")
    return int(eval(e, {"__builtins__": {}}, {}))        # noqa: S307 (定数式のみ)


def _py_name(ename: str, member: str) -> str:
    pre = _ENUM_CLASS_PREFIX.get(ename, "")
    return pre + member.upper() if pre else member


_IM920_CONSTS = _im920_consts()


def parse_header(path: Path):
    text = path.read_text(encoding="utf-8")
    ns_m = _NAMESPACE.search(text)
    if not ns_m:
        raise ValueError(f"{path.name}: namespace S5C/S5T not found")
    ns = ns_m.group(1)
    clean = _strip_comments(text)

    ints, floats, enums, names = {}, {}, {}, {}
    known = dict(_IM920_CONSTS)                    # IM920::MAX_PAYLOAD 等を展開できるように
    for m in _INT_CONST.finditer(clean):
        try:
            ints[m.group(1)] = _eval_int(m.group(2), known)
            known[m.group(1)] = ints[m.group(1)]
        except ValueError:
            pass    # sizeof(...) 等は Python 側に要らない
    for m in _FLOAT_CONST.finditer(clean):
        floats[m.group(1)] = float(m.group(2))
    for m in _ENUM.finditer(clean):
        ename, body = m.group(1), m.group(2)
        members = {}
        for line in body.split("\n"):
            mm = _ENUM_MEMBER.match(line)
            if mm:
                members[mm.group(1)] = _eval_int(mm.group(2), {})
        enums[ename] = members
    member_owner = {mem: ename for ename, mems in enums.items() for mem in mems}
    for m in _NAME_FN.finditer(text):      # 文字列を拾うので元テキストで
        fn, body = m.group(1), m.group(2)
        table = {}
        for c in _CASE.finditer(body):
            member = c.group(1).split("::")[-1]
            ename = member_owner.get(member, "")
            table[_py_name(ename, member)] = c.group(2)
        names[fn] = table
    return ns, ints, floats, enums, names


def render(parsed) -> str:
    seen = set()
    out = ['"""', "s5_protocol.py  -  ★ 自動生成。手で編集しないこと ★",
           "", "  python protocol/gen_py_protocol.py",
           "", "protocol/S5Cmd.h / S5Telem.h (機体と地上局が共有する無線プロトコル) から",
           "Python 側で使う定数を生成したもの。値の意味は元ヘッダのコメントを読む。",
           "XXX_NAME は 値 -> 表示名 の dict (C++ の xxxName() と同じ文字列)。", '"""', ""]
    for ns, ints, floats, enums, names in parsed:
        out.append("# " + "=" * 70)
        out.append(f"#  namespace {ns}  ({'S5Cmd.h' if ns == 'S5C' else 'S5Telem.h'})")
        out.append("# " + "=" * 70)
        if "VERSION" in ints:
            out.append(f"{ns}_VERSION = {ints['VERSION']}")
        for k, v in ints.items():
            if k == "VERSION" or k in seen:
                continue
            seen.add(k)
            out.append(f"{k} = {v:#04x}" if k.startswith(("TYPE_", "MAGIC")) else f"{k} = {v}")
        if floats:
            out.append("")
            for k, v in floats.items():
                out.append(f"{ns}_{k} = {v}")
        for ename, members in enums.items():
            out.append("")
            out.append(f"# enum {ename}")
            for k, v in members.items():
                name = _py_name(ename, k)
                if ename in ("CmdFlag", "Flag") and v > 0 and (v & (v - 1)) == 0:
                    out.append(f"{name} = 1 << {v.bit_length() - 1}")
                else:
                    out.append(f"{name} = {v}")
        for fn, table in names.items():
            out.append("")
            const = re.sub(r"Name$", "", fn)
            const = re.sub(r"([a-z])([A-Z])", r"\1_\2", const).upper() + "_NAME"
            out.append(f"{const} = {{")
            for k, v in table.items():
                out.append(f"    {k}: {v!r},")
            out.append("}")
        out.append("")
    return "\n".join(out)


def build_source() -> str:
    return render([parse_header(h) for h in HEADERS])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="生成物が最新か検査するだけ")
    ap.add_argument("-o", "--out", default=str(OUT))
    args = ap.parse_args()
    src = build_source()
    out = Path(args.out)
    if args.check:
        cur = out.read_text(encoding="utf-8") if out.exists() else ""
        if cur != src:
            print(f"[gen_py_protocol] STALE: {out}  -> run: python protocol/gen_py_protocol.py")
            return 1
        print(f"[gen_py_protocol] OK: {out} is up to date")
        return 0
    out.write_text(src, encoding="utf-8", newline="\n")
    print(f"[gen_py_protocol] wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
