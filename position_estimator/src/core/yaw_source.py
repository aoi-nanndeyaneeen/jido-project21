"""
core/yaw_source.py
ミッションに渡す「機首方位」をどこから取るかの裁定。

main_loop.py の中に [False] セル 4 個で埋め込まれていた隠れ状態機械を
1 クラスにしたもの (2026-09-16)。毎フレーム 1 回 arbitrate() を呼ぶ。

    入力: カメラのヨー推定 (YawEstimator.update() の結果、収束していれば rad)
          機体テレメトリ (armed / yaw = アーム時基準のジャイロ積分ヨー [deg])
    出力: (yaw_rad, yaw_valid, yaw_src)   yaw_src は "camera" か "fixed"

優先順位:
  1. カメラ推定が収束していて、かつ機体のジャイロヨーと食い違っていない → "camera"。
     この値は CF_YAW_VALID 付きで機体へ送られ、機体は自分のヨー推定を再基準する。
     ★ 2026-09-15 20:49: 推定が -86.3deg (実際は -2deg) に「収束」し、機体の
       ヨーを -86 に書き換えた結果ヘディング保持が 60deg/s で機首を回し続けて
       0.6m 流れた。機体のヨーは一日中 ±5deg 以内で正しかったので、
       YAW_CAMERA_MAX_DISAGREE_DEG 以上食い違うカメラ推定は採用しない。
  2. それ以外で MISSION_YAW_MODE == "fixed" → 機体のジャイロヨー + 初期アラインメント。
     ★ 決め打ちの 0deg ではなく機体の実測ヨーを使う。2026-09-15: アーム後に機首が
       -17.8deg 回っていたのに 0deg で飛ばし、目標と違う向きへ進んだ。
       機体のヨーはアーム時基準の相対値 (カメラで再基準された後は絶対値) なので、
       再基準前だけ YAW_INITIAL_ALIGN_DEG を足して絶対方位にする。
  3. MISSION_YAW_MODE == "camera" で未収束 → yaw_valid=False (ミッションは水平移動しない)。

★ この裁定は PC 側のミッション (mission.py) が「前がどっちか」を決めるためのもの。
  機体のヘディングホールドそのものはジャイロ積分で動いており、ここが誤っても
  機体が勝手に回ることは無い (回すのは CF_YAW_VALID の再基準だけ)。
"""

import math

from utils.config import (MISSION_YAW_MODE, YAW_INITIAL_ALIGN_DEG,
                          YAW_CAMERA_MAX_DISAGREE_DEG)


class YawArbiter:
    def __init__(self, say=print):
        self._say = say
        # 機体のヨー推定がカメラの絶対ヨーで再基準済みか (アーム中のみ有効)。
        # 機体はアーム時にヨー推定を 0 へ戻すので、ディスアームで落とす。
        self._rebased = False
        self._warned_fixed = False      # 「初期アラインメントで飛んでいます」は 1 回だけ
        self._reject_warned = False     # カメラ不採用の警告は状態が変わるまで 1 回だけ

    def arbitrate(self, camera_yaw_rad, link_state):
        """
        Args:
            camera_yaw_rad : YawEstimator が valid なら rad、未収束なら None
            link_state     : S5Link.state() (空 dict でもよい)
        Returns:
            (yaw_rad, yaw_valid, yaw_src)
        """
        armed = bool(link_state.get("armed", 0))
        if not armed:
            self._rebased = False
        dev_yaw = link_state.get("yaw")
        dev_ok = isinstance(dev_yaw, float)
        # 再基準前は「アーム時の向き = 初期アラインメント」を足して絶対方位にする
        base = 0.0 if self._rebased else YAW_INITIAL_ALIGN_DEG

        yaw = camera_yaw_rad
        valid = yaw is not None
        src = "camera"

        # 1) カメラ推定を機体のジャイロヨーと突き合わせる
        if valid and dev_ok:
            disagree = (math.degrees(yaw) - (base + dev_yaw) + 180.0) % 360.0 - 180.0
            if abs(disagree) > YAW_CAMERA_MAX_DISAGREE_DEG:
                if not self._reject_warned:
                    self._reject_warned = True
                    self._say(f"[Yaw] カメラ推定 {math.degrees(yaw):+.1f}deg が機体ヨー "
                              f"{base + dev_yaw:+.1f}deg と {disagree:+.0f}deg 食い違うため不採用")
                yaw, valid = None, False
            else:
                self._reject_warned = False
        if valid and armed:
            self._rebased = True    # この値が CF_YAW_VALID で機体へ送られ、再基準される

        # 2) 未収束のフォールバック
        if not valid and MISSION_YAW_MODE == "fixed":
            yaw = math.radians(base + dev_yaw) if dev_ok else math.radians(YAW_INITIAL_ALIGN_DEG)
            valid = True
            src = "fixed"
            if not self._warned_fixed:
                self._warned_fixed = True
                self._say(f"[Mission] ヨー推定が未収束のため、初期アラインメント "
                          f"{YAW_INITIAL_ALIGN_DEG:+.1f}deg を機首方位として使います。"
                          f"機首をフィールド奥(+y)へ向けたまま飛ばしてください "
                          f"(GUIDEDに入るたびに機体側の実測yawと一緒に再警告します)")
        return yaw, valid, src
