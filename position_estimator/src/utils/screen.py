"""
utils/screen.py

毎回カーソルを左上へ戻して上書きする、流れない端末表示。
console.py で使っていたものを共通化し、main_loop.py からも使えるようにした
(2026-09-16: 「シリアル表示が流れて今の状態が読めない」)。
"""

import os
import sys


class Screen:
    """毎回カーソルを左上へ戻して上書きする。行末は消してから書くので、
    短い行になったときにゴミが残らない。

    ★ --plain / plain=True では ANSI を一切使わない。地上局 (ground_receiver StatusView.h) が
      「PlatformIO のシリアルモニタでは ANSI が処理されず、画面が崩れた
      のかキーが効いていないのか区別できなくなる」としてクリアを避けて
      いるのと同じ理由で、逃げ道を用意しておく。
    """

    def __init__(self, plain=False):
        self.plain = plain
        if not plain and os.name == "nt":
            self._enable_vt()

    @staticmethod
    def _enable_vt():
        """Windows のコンソールで ANSI を有効にする (Win10 1511 以降)。"""
        try:
            import ctypes
            k32 = ctypes.windll.kernel32
            h = k32.GetStdHandle(-11)
            mode = ctypes.c_uint32()
            if k32.GetConsoleMode(h, ctypes.byref(mode)):
                k32.SetConsoleMode(h, mode.value | 0x0004)
        except Exception:
            pass

    def draw(self, lines):
        if self.plain:
            sys.stdout.write("\n".join(lines) + "\n")
            sys.stdout.flush()
            return
        out = ["\033[H"]
        for line in lines:
            out.append(line + "\033[K\n")
        out.append("\033[J")
        sys.stdout.write("".join(out))
        sys.stdout.flush()

    def clear(self):
        if not self.plain:
            sys.stdout.write("\033[2J\033[H")
            sys.stdout.flush()
