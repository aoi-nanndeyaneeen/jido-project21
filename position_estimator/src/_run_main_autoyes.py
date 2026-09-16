"""
_run_main_autoyes.py  -  立ち上げ確認用。main.py を対話なしで動かす。

キャリブレーションの y/n は main.py --calib で決まるので、ここで潰すのは
「ラズパイを待つか」等の残りの問い合わせだけ。本番では使わない。

    python _run_main_autoyes.py            # --calib fresh (既定) で起動
    python _run_main_autoyes.py --calib saved
"""

import builtins
import sys


def _auto_input(prompt=""):
    # d = 「そのまま続行」。待ち直しを選ぶと自動実行が終わらなくなる。
    print(prompt, end="")
    print("d  [auto-answered by bringup script]")
    return "d"


builtins.input = _auto_input

import main   # noqa: E402

main.main(sys.argv[1:])
