import cv2
import numpy as np
import time
from collections import deque

from utils.config import VIEW_X, VIEW_Y, VIEW_Z


class ViewGraph:
    W, H = 600, 600

    def __init__(self):
        self.history_z = deque()
        self.history_xy = deque()
        self.window_sec = 10.0

    def get_image(self, P, current_z, target_alt):
        now = time.time()
        self.history_z.append((now, current_z, target_alt))
        if P is not None:
            self.history_xy.append((now, float(P[0]), float(P[1])))
        while self.history_z and now - self.history_z[0][0] > self.window_sec:
            self.history_z.popleft()
        while self.history_xy and now - self.history_xy[0][0] > self.window_sec:
            self.history_xy.popleft()

        image = np.full((self.H, self.W, 3), 255, dtype=np.uint8)
        # 水平位置は X/Y を同じスケールで見たいので、フィールド全体が入る対称レンジにする
        xy_limit = max(abs(VIEW_X[0]), abs(VIEW_X[1]), abs(VIEW_Y[0]), abs(VIEW_Y[1]))
        self._draw_plot(
            image, (58, 40, 572, 270), list(self.history_z), now,
            VIEW_Z[0], VIEW_Z[1], "m",
            [(1, "Altitude", (220, 70, 40)), (2, "Target", (40, 40, 210))],
            "Altitude Z",
        )
        self._draw_plot(
            image, (58, 328, 572, 558), list(self.history_xy), now,
            -xy_limit, xy_limit, "m",
            [(1, "X", (40, 70, 220)), (2, "Y", (40, 160, 70))],
            "Horizontal Position X / Y",
        )
        return image

    def _draw_plot(self, image, rect, values, now, y_min, y_max, unit, series, title):
        left, top, right, bottom = rect
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.rectangle(image, (left, top), (right, bottom), (250, 250, 250), -1)
        cv2.rectangle(image, (left, top), (right, bottom), (100, 100, 100), 1)

        # 横グリッド + Y軸目盛り（値）
        rows = 5
        for i in range(rows + 1):
            y = int(top + i * (bottom - top) / rows)
            if 0 < i < rows:
                cv2.line(image, (left, y), (right, y), (225, 225, 225), 1)
            val = y_max - i * (y_max - y_min) / rows
            cv2.putText(image, f"{val:.1f}", (4, y + 4), font, 0.38, (90, 90, 90), 1)

        # 縦グリッド + X軸目盛り（時間 [s]、右端が現在）
        cols = 5
        for i in range(cols + 1):
            x = int(left + i * (right - left) / cols)
            if 0 < i < cols:
                cv2.line(image, (x, top), (x, bottom), (235, 235, 235), 1)
            t_label = -self.window_sec + i * self.window_sec / cols
            cv2.putText(image, f"{t_label:+.0f}", (x - 10, bottom + 15), font, 0.38, (90, 90, 90), 1)
        cv2.putText(image, "time [s]", (int((left + right) / 2) - 28, bottom + 28), font, 0.4, (90, 90, 90), 1)

        # タイトル（単位つき）
        cv2.putText(image, f"{title} [{unit}]", (left, top - 12), font, 0.55, (35, 35, 35), 1)

        # 凡例 + 現在値
        legend_y = top + 16
        for value_index, name, color in series:
            cv2.line(image, (left + 8, legend_y - 4), (left + 30, legend_y - 4), color, 2, cv2.LINE_AA)
            latest = values[-1][value_index] if values else None
            text = name if latest is None else f"{name}: {latest:+.2f} {unit}"
            cv2.putText(image, text, (left + 38, legend_y), font, 0.42, (60, 60, 60), 1)
            legend_y += 18

        if not values:
            cv2.putText(image, "no data", (int((left + right) / 2) - 28, int((top + bottom) / 2)),
                        font, 0.5, (150, 150, 150), 1)
            return

        def point(timestamp, value):
            age = max(0.0, min(self.window_sec, now - timestamp))
            x = right - int(age / self.window_sec * (right - left))
            ratio = (value - y_min) / max(y_max - y_min, 1e-6)
            y = bottom - int(np.clip(ratio, 0.0, 1.0) * (bottom - top))
            return x, y

        for value_index, _name, color in series:
            points = [point(row[0], row[value_index]) for row in values]
            if len(points) > 1:
                cv2.polylines(image, [np.array(points, dtype=np.int32)], False, color, 2, cv2.LINE_AA)

    def close(self):
        pass
