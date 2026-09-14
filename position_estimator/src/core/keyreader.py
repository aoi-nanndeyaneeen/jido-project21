"""
core/keyreader.py
非ブロッキングな1文字キー入力。console.py と ble_monitor.py の両方が使う
(全画面コンソールと、流れっぱなしのシリアルモニタ風ツールの両方に必要な、
最小限の「押されていれば1文字返す」という機能だけを切り出したもの)。
"""
import os
import sys


class KeyReader:
    """Windows は msvcrt、それ以外は termios。どちらも無ければ無効化する。"""

    def __init__(self):
        self._mode = None
        self._fd = None
        self._saved = None
        if os.name == "nt":
            import msvcrt  # noqa: F401
            self._mode = "nt"
        elif sys.stdin.isatty():
            self._mode = "posix"

    def __enter__(self):
        if self._mode == "posix":
            import termios
            import tty
            self._fd = sys.stdin.fileno()
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        if self._mode == "posix" and self._saved is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)

    def get(self):
        """押されていれば 1 文字、無ければ None。ブロックしない。"""
        if self._mode == "nt":
            import msvcrt
            if not msvcrt.kbhit():
                return None
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):   # 方向キーなどの 2 バイト目を捨てる
                msvcrt.getwch()
                return None
            return ch
        if self._mode == "posix":
            import select
            if not select.select([sys.stdin], [], [], 0)[0]:
                return None
            return sys.stdin.read(1)
        return None
