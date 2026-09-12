import builtins

_orig_input = builtins.input


def _auto_input(prompt=""):
    print(prompt, end="")
    print("n  [auto-answered by bringup script: re-calibrate]")
    return "n"


builtins.input = _auto_input

import main

main.main()
