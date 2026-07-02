"""
view_rc.py
RCコマンドをプロポのスティック風に可視化する。
純OpenCV描画のため軽量・matplotlib不使用。

左スティック : Throttle（縦） × Yaw（横）
右スティック : Pitch（縦）    × Roll（横）
"""

import cv2
import numpy as np
from core.autopilot import RCCommand


class ViewRC:
    W, H = 900, 420

    # スティック中心・半径
    _LX, _RX = 220, 680
    _CY, _R  = 200, 155

    # カラー
    _BG       = ( 26,  20,  40)
    _RING     = ( 90,  85, 120)
    _GRID     = ( 55,  52,  78)
    _LINE     = ( 40, 180, 100)
    _DOT      = ( 30, 220, 120)
    _DOT_RIM  = (200, 255, 220)
    _CENTER   = (140, 140, 160)
    _TITLE    = (230, 230, 255)
    _LABEL    = (160, 200, 160)
    _VAL      = (100, 210, 255)
    _DIVIDER  = ( 70,  65,  95)

    def get_image(self, cmd: RCCommand) -> np.ndarray:
        img = np.full((self.H, self.W, 3), self._BG, dtype=np.uint8)

        # ヘッダ
        cv2.putText(img, "RC COMMAND PREVIEW",
                    (self.W // 2 - 165, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, self._TITLE, 2)

        # 区切り線（縦）
        cv2.line(img,
                 (self.W // 2, 50), (self.W // 2, self.H - 10),
                 self._DIVIDER, 1)

        # 左スティック: Yaw(X) × Pitch(Y)
        self._stick(img,
                    cx=self._LX, cy=self._CY, r=self._R,
                    sx=cmd.yaw,
                    sy=cmd.pitch,
                    title="LEFT  STICK",
                    x_name="YAW",
                    x_val=cmd.yaw,
                    y_name="PITCH",
                    y_val=cmd.pitch,
                    y_str=f"{cmd.pitch:+.3f}")

        # 右スティック: Roll(X) × Throttle(Y)
        self._stick(img,
                    cx=self._RX, cy=self._CY, r=self._R,
                    sx=cmd.roll,
                    sy=cmd.throttle * 2.0 - 1.0,
                    title="RIGHT STICK",
                    x_name="ROLL",
                    x_val=cmd.roll,
                    y_name="THROTTLE",
                    y_val=cmd.throttle,
                    y_str=f"{cmd.throttle * 100:.0f} %")

        return img

    # ─────────────────────────────────────────────────────────
    def _stick(self, img, cx, cy, r,
               sx, sy,
               title,
               x_name, x_val,
               y_name, y_val, y_str):
        """1本のスティックを描画"""

        # ── 外枠・グリッド ──────────────────────────────────
        cv2.circle(img, (cx, cy), r, self._RING, 2)
        cv2.circle(img, (cx, cy), r // 5, self._GRID, 1)   # 内側目安円
        cv2.line(img, (cx - r, cy), (cx + r, cy), self._GRID, 1)
        cv2.line(img, (cx, cy - r), (cx, cy + r), self._GRID, 1)

        # ── スティック位置（円外クランプ） ──────────────────
        raw_dx = sx * r
        raw_dy = sy * r   # 画面Y軸反転
        length = np.hypot(raw_dx, raw_dy)
        if length > r:
            raw_dx = raw_dx / length * r
            raw_dy = raw_dy / length * r
        dx, dy = int(round(raw_dx)), int(round(raw_dy))

        # センター→ドットへの線
        cv2.line(img, (cx, cy), (cx + dx, cy + dy), self._LINE, 2)
        # ドット
        cv2.circle(img, (cx + dx, cy + dy), 13, self._DOT,    -1)
        cv2.circle(img, (cx + dx, cy + dy), 15, self._DOT_RIM,  1)
        # センターマーク
        cv2.circle(img, (cx, cy), 4, self._CENTER, -1)

        # ── タイトル ────────────────────────────────────────
        (tw, _), _ = cv2.getTextSize(title,
                                     cv2.FONT_HERSHEY_SIMPLEX, 0.62, 2)
        cv2.putText(img, title,
                    (cx - tw // 2, cy - r - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, self._TITLE, 2)

        # ── 数値ラベル（スティック下） ───────────────────────
        y_base = cy + r + 28
        entries = [
            (x_name, f"{x_val:+.3f}"),
            (y_name, y_str),
        ]
        for i, (lbl, val) in enumerate(entries):
            row_y = y_base + i * 28
            cv2.putText(img, f"{lbl:<10}",
                        (cx - r + 5, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, self._LABEL, 1)
            cv2.putText(img, val,
                        (cx + 20, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.56, self._VAL, 1)