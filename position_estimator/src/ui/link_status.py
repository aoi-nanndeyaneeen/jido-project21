"""地上局リンクの最小限の常時診断表示。"""

import math

import cv2
import numpy as np


class LinkStatusView:
    """PC が得られる通信事実だけを、区間ごとに表示する。"""

    W, H = 760, 344

    _GREEN = (55, 150, 70)
    _YELLOW = (30, 180, 230)
    _RED = (55, 55, 220)
    _GRAY = (125, 125, 125)

    def __init__(self):
        cv2.namedWindow("Link Status", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Link Status", self.W, self.H)
        cv2.imshow("Link Status", np.zeros((self.H, self.W, 3), dtype=np.uint8))

    @staticmethod
    def _number(data, name, default=0.0):
        try:
            return float(data.get(name, default))
        except (TypeError, ValueError):
            return default

    @classmethod
    def _line(cls, image, y, label, value, state, detail=""):
        color = {"ok": cls._GREEN, "warn": cls._YELLOW,
                 "bad": cls._RED, "idle": cls._GRAY}[state]
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.circle(image, (28, y - 6), 8, color, -1, cv2.LINE_AA)
        cv2.putText(image, label, (48, y), font, 0.58, (65, 65, 65), 1, cv2.LINE_AA)
        cv2.putText(image, value, (250, y), font, 0.62, color, 2, cv2.LINE_AA)
        if detail:
            cv2.putText(image, detail, (410, y), font, 0.47,
                        (90, 90, 90), 1, cv2.LINE_AA)

    def render_and_show(self, link):
        image = np.full((self.H, self.W, 3), (247, 247, 247), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image, "COMMUNICATION DIAGNOSTICS", (18, 32), font,
                    0.72, (35, 35, 35), 2, cv2.LINE_AA)

        if link is None or not link.ok:
            self._line(image, 78, "PC - Ground station USB", "NOT CONNECTED", "bad",
                       "Check USB cable / selected ground receiver")
            self._line(image, 122, "Ground station - IM920 UART", "UNKNOWN", "idle")
            self._line(image, 166, "Drone -> PC telemetry", "UNKNOWN", "idle")
            self._line(image, 210, "PC -> Drone command", "UNKNOWN", "idle")
            self._line(image, 254, "  ... as seen by aircraft", "UNKNOWN", "idle")
            self._line(image, 298, "Flight controller", "UNKNOWN", "idle")
            cv2.imshow("Link Status", image)
            return

        stat = link.diagnostics()
        telemetry_age = link.age()
        if math.isfinite(telemetry_age):
            down_state = "ok" if telemetry_age < 1.0 else "bad"
            down_value = "OK" if down_state == "ok" else "LOST"
            down_detail = f"last packet {telemetry_age * 1000:.0f} ms ago"
        else:
            down_state, down_value, down_detail = "bad", "WAITING", "no DATA packet yet"

        im920_ok = stat.get("im920_ok")
        if im920_ok is None:
            uart_state, uart_value, uart_detail = "idle", "CHECKING", "STAT not received yet"
        elif im920_ok:
            uart_state, uart_value = "ok", "OK"
            uart_detail = f"{int(stat.get('rx_bytes', 0))} UART bytes"
        else:
            uart_state, uart_value = "bad", "NO RESPONSE"
            uart_detail = "check 3V3, wiring, UART baud"

        received = int(stat.get("alt", 0) + stat.get("pos", 0) + stat.get("att", 0)
                       + stat.get("param", 0))
        lost = int(stat.get("lost", 0))
        total = received + lost
        loss = 100.0 * lost / total if total else 0.0
        rssi = link.state().get("rssi")
        radio_detail = f"RSSI {rssi:.0f}  loss {loss:.1f}%" if rssi is not None else f"loss {loss:.1f}%"
        if received and loss >= 10.0:
            down_state = "warn"
        elif received and (stat.get("badcs", 0) or stat.get("badlen", 0)):
            down_state = "warn"

        cmd_tx = int(stat.get("cmd_tx", 0))
        im_ng = int(stat.get("im_ng", 0))
        cmd_have = bool(stat.get("cmd_have", False))
        fc_fresh = bool(link.flag("cmd_fresh"))
        up_detail = f"gs sent {cmd_tx}, IM920 refused {im_ng}"
        if not cmd_have:
            up_state, up_value, up_detail = "idle", "IDLE", "PC is not commanding the aircraft"
        elif fc_fresh:
            up_state, up_value = "ok", "OK"
        else:
            up_state, up_value = "bad", "NOT ACKNOWLEDGED"

        # 機体が自分で数えた上りの受信状況 (D フレーム同乗。S5Telem.h)。
        #  ★ 地上局の cmd_tx とは数え始めが違うので、絶対数の差は比較しない。
        #    機体の good/lost だけで完結する欠落率と、最後に受けてからの
        #    経過時間を見る。ここが「上りが遅いのか落ちているのか」の答え。
        fc_good = stat.get("fc_cmd_good")
        fc_lost = int(stat.get("fc_cmd_lost", 0))
        fc_age_cs = int(stat.get("fc_cmd_age_cs", 0xFFFF))
        if fc_good is None:
            fc_up_state, fc_up_value = "idle", "NO D FRAME"
            fc_up_detail = "waiting for the aircraft's uplink report"
        elif fc_age_cs == 0xFFFF:
            fc_up_state, fc_up_value = "bad", "NEVER RECEIVED"
            fc_up_detail = "the aircraft has not seen a single command"
        else:
            fc_good = int(fc_good)
            fc_total = fc_good + fc_lost
            fc_loss = 100.0 * fc_lost / fc_total if fc_total else 0.0
            fc_age_s = fc_age_cs / 100.0
            fc_up_value = f"{fc_loss:.0f}% lost"
            fc_up_detail = (f"got {fc_good}, lost {fc_lost}, "
                            f"last {fc_age_s:.2f}s ago")
            if fc_age_s > 1.0 or fc_loss >= 25.0:
                fc_up_state = "bad"
            elif fc_age_s > 0.5 or fc_loss >= 10.0:
                fc_up_state = "warn"
            else:
                fc_up_state = "ok"

        st = link.state()
        if not st:
            fc_state, fc_value, fc_detail = "idle", "WAITING", "no flight-controller telemetry"
        else:
            flags = []
            if bool(st.get("armed", 0)):
                flags.append("ARMED")
            if bool(st.get("guided", 0)):
                flags.append("GUIDED")
            if bool(st.get("flow_ok", 0)):
                flags.append("FLOW")
            if bool(st.get("range_valid", 0)):
                flags.append("RANGE")
            fc_value = " ".join(flags) if flags else "NOT READY"
            fc_state = "ok" if bool(st.get("flow_ok", 0)) and bool(st.get("range_valid", 0)) else "warn"
            fc_detail = f"mode {int(st.get('mode', -1))}"

        self._line(image, 78, "PC - Ground station USB", "OK", "ok", link.port)
        self._line(image, 122, "Ground station - IM920 UART", uart_value, uart_state, uart_detail)
        self._line(image, 166, "Drone -> PC telemetry", down_value, down_state,
                   f"{down_detail}; {radio_detail}")
        self._line(image, 210, "PC -> Drone command", up_value, up_state, up_detail)
        self._line(image, 254, "  ... as seen by aircraft", fc_up_value, fc_up_state,
                   fc_up_detail)
        self._line(image, 298, "Flight controller", fc_value, fc_state, fc_detail)
        cv2.imshow("Link Status", image)

    def close(self):
        cv2.destroyWindow("Link Status")
