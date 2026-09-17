"""
core/ble_link.py
地上局リンク (機体への操縦指令 + 機体テレメトリ) の BLE 版。

--------------------------------------------------------------------------
これは何か
--------------------------------------------------------------------------
2026-09-17 に、IM920 (ground_receiver の xiao_s5_log + core/s5_link.py) が
担っていた通信を BLE へ移した。経路は

    機体 drone_s5 ──UART──> log_recorder XIAO ESP32C3 ──BLE Notify──> ここ
                  <─UART──                            <─BLE Write───

で、機体125Hzログ (core/ble_tap.py) と同じ BLE 接続を共有する。

**S5Link と同じ公開インターフェース** (ok / port / state() / age() /
telemetry_ok() / n_data() / diagnostics() / flag() / send_command() /
send_key() / n_sent() / close()) を持たせてあるので、mission.py /
console.py / main_loop.py は中身が IM920 か BLE かを意識しない。
どちらを使うかは utils/config.py の GROUND_LINK_BACKEND だけで決まる。

--------------------------------------------------------------------------
中身は IM920 と同じ
--------------------------------------------------------------------------
  上り: protocol/S5Cmd.h の CmdFrame (22B) をそのまま BLE Write する。
        seq は log_recorder が振る (IM920 版で地上局 XIAO が振っていたのと同じ)。
  下り: protocol/S5Telem.h の A/B/C/D/P フレームが T_TELEM に束ねられて届く。
        ここで ground_receiver/include/TelemetryStore.h の emitData() と
        **同じ列・同じ書式** の行に直す。s5_link_*.csv の形式も同じなので、
        merge_logs.py / analyze_poshold.py はそのまま使える。
        違うのは rx_ms (地上局 XIAO の millis ではなく、このリンクを開いて
        からの PC 時刻 [ms]) と rssi (BLE では常に -1) だけ。

--------------------------------------------------------------------------
★ 安全のきまり (S5Link と同じ)
--------------------------------------------------------------------------
機体は「コマンドが 1 秒来ない = その場ホールド」「4 秒来ない = 自動着陸」。
このクラスは send_command() が呼ばれなくなったら何も送らない
(log_recorder は PC が黙ってから 1.5 秒だけ最後の指令を再送し、BLE が
切れたら即止める)。キープアライブのために勝手に前回値を送り直すことは
しない。
"""

import struct
import threading
import time
from datetime import datetime
from pathlib import Path

# ★ 定数 (VERSION / MAGIC / TYPE_* / F_* / PF_*) は protocol/ から生成した
#   core/s5_protocol.py を使う。構造体の並び (struct の書式) だけはここに手で書く
#   (生成スクリプトは構造体を読まない) ので、下の assert でサイズを突き合わせる。
from core.s5_protocol import (
    MAGIC as CMD_MAGIC, S5C_VERSION as CMD_VERSION, S5T_VERSION as TELEM_VERSION,
    PACKET_BYTES, TYPE_ALT, TYPE_POS, TYPE_ATT, TYPE_DV, TYPE_PARAM,
    F_ARMED, F_FLOW_OK, F_RANGE_OK, F_RANGE_VALID, F_ALT_EN, F_ALT_ACT, F_POS_HOLD,
    F_AIRBORNE, F_DRY_RUN, F_SAT, F_TX_DROP, F_GUIDED, F_CMD_FRESH, F_LANDED,
    F_MANEUVER, F_FRAME_OK,
    PF_ALT_HOLD_EN, PF_DRY_RUN, PF_SONAR, PF_STICK_VZ,
    REQ_ABORT,
)

# ---- CmdFrame (protocol/S5Cmd.h) -----------------------------------------
# magic ver seq req vx vy alt yawrate flags corr_n corr_e yaw_abs laps rsv
_CMD_STRUCT = struct.Struct("<BBBBhhhhHhhhBB")
assert _CMD_STRUCT.size == 22, "protocol/S5Cmd.h の CmdFrame (22B) とずれている"

# ---- S5Telem.h のフレーム ------------------------------------------------

_HDR = "BBHH"                                   # type seq flags t_cs
_ALT = struct.Struct("<" + _HDR + "BB" + "h" * 9 + "H")
_POS = struct.Struct("<" + _HDR + "BB" + "h" * 10)
_ATT = struct.Struct("<" + _HDR + "BBBBBB" + "h" * 7 + "bb")
_DV = struct.Struct("<" + _HDR + "hhh" + "HHHH" + "Bb")
_PARAM = struct.Struct("<BBBB" + "h" * 12)
_FRAME_STRUCT = {TYPE_ALT: _ALT, TYPE_POS: _POS, TYPE_ATT: _ATT,
                 TYPE_DV: _DV, TYPE_PARAM: _PARAM}
assert (_ALT.size, _POS.size, _ATT.size, _PARAM.size) == (PACKET_BYTES,) * 4 \
    and _DV.size == 22, "protocol/S5Telem.h のフレームサイズとずれている"

# ground_receiver/include/TelemetryStore.h の CSV_HEADER と同じ並び
CSV_COLUMNS = (
    "rx_ms,t_ms,seq,lost,rssi,frame,"
    "mode,alt_state,armed,flow_ok,range_ok,range_valid,"
    "alt_en,alt_act,pos_hold,airborne,dry_run,sat,tx_drop,"
    "guided,cmd_fresh,landed,maneuver,frame_ok,"
    "thr,bad,"
    "roll,pitch,yaw,"
    "range_h,range_raw,alt_hold,climb,alt_vzt,alt_corr,alt_thr,"
    "fh_vxc,fh_vyc,fh_vxt,fh_vyt,"
    "fh_posn,fh_pose,fh_holdn,fh_holde,"
    "fh_leanr,fh_leanp,"
    "m1,m2,m3,m4,mixsat,"
    "roll_gyr,pitch_gyr,yaw_gyr,roll_ratetar,pitch_ratetar,"
    "roll_cmd,pitch_cmd,roll_stick,pitch_stick,"
    "dvx,dvy,dv_yaw,"
    "cmd_age,cmd_good,cmd_lost,cmd_bad,cmd_seq,cmd_rssi"
).split(",")


def _q16(v, scale):
    x = int(round(v * scale))
    return max(-32768, min(32767, x))


def _bit(flags, m):
    return 1 if flags & m else 0


class _TelemDecoder:
    """ground_receiver TelemetryStore.h の handleLine() + emitData() を Python に写したもの。
    フレームを受けるたびに「他のフレームは前回値」で 1 行を組み立てる。"""

    def __init__(self):
        self.last = {TYPE_ALT: None, TYPE_POS: None, TYPE_ATT: None,
                     TYPE_DV: None, TYPE_PARAM: None}
        self.live_flags = 0
        self.live_modes = 0
        self.t_cs_prev = None
        self.t_ms = 0
        self.seq_prev = None
        self.n = {TYPE_ALT: 0, TYPE_POS: 0, TYPE_ATT: 0, TYPE_DV: 0, TYPE_PARAM: 0}
        self.n_lost = 0
        self.n_badlen = 0
        self.ver_warned = False

    def _unwrap(self, t_cs):
        if self.t_cs_prev is None:
            self.t_ms = t_cs * 10
        else:
            self.t_ms += ((t_cs - self.t_cs_prev) & 0xFFFF) * 10
        self.t_cs_prev = t_cs
        return self.t_ms

    def feed(self, payload, rx_ms):
        """T_TELEM の payload (フレームの連結) を分解する。
        戻り値: [("DATA", [str...]) | ("PARAM", str) | ("WARN", str), ...]"""
        out = []
        i = 0
        while i < len(payload):
            typ = payload[i]
            st = _FRAME_STRUCT.get(typ)
            if st is None or i + st.size > len(payload):
                # 型が分からない = 残りの切れ目も分からない。この束は捨てる
                self.n_badlen += 1
                if self.n_badlen <= 3:
                    out.append(("WARN", f"未知の type=0x{typ:02X} / 長さ不足 "
                                        "(S5Telem.h が機体とずれている?)"))
                break
            vals = st.unpack_from(payload, i)
            i += st.size

            seq = vals[1]
            if self.seq_prev is not None:
                gap = (seq - self.seq_prev) & 0xFF
                if gap > 1:
                    self.n_lost += gap - 1
            self.seq_prev = seq

            self.n[typ] += 1
            self.last[typ] = vals
            if typ == TYPE_PARAM:
                if vals[2] != TELEM_VERSION and not self.ver_warned:
                    self.ver_warned = True
                    out.append(("WARN", f"パケットバージョン不一致: 機体={vals[2]} "
                                        f"PC={TELEM_VERSION}"))
                out.append(("PARAM", self._param_line(vals)))
                continue

            self.live_flags = vals[2]
            if typ != TYPE_DV:
                self.live_modes = vals[4]
            t_ms = self._unwrap(vals[3])
            frame = {TYPE_ALT: 0, TYPE_POS: 1, TYPE_ATT: 2, TYPE_DV: 3}[typ]
            out.append(("DATA", self._data_row(rx_ms, t_ms, seq, frame)))
        return out

    @staticmethod
    def _param_line(p):
        (_t, _s, ver, cfg, fvkp, fvki, fvkd, fpkp, apkp, arkp, arki, arkd,
         hover, target, maxlean, auth) = p
        g = 1000.0
        return (f"ver={ver},alt_en={_bit(cfg, PF_ALT_HOLD_EN)},dry={_bit(cfg, PF_DRY_RUN)},"
                f"sonar={_bit(cfg, PF_SONAR)},stick_vz={_bit(cfg, PF_STICK_VZ)},"
                f"flow_vel_kp={fvkp / g:.3f},flow_vel_ki={fvki / g:.3f},"
                f"flow_vel_kd={fvkd / g:.3f},flow_pos_kp={fpkp / g:.3f},"
                f"alt_pos_kp={apkp / g:.3f},alt_rate_kp={arkp / g:.3f},"
                f"alt_rate_ki={arki / g:.3f},alt_rate_kd={arkd / g:.3f},"
                f"alt_hover_thr={hover / g:.3f},alt_target_m={target / g:.3f},"
                f"flow_max_lean={maxlean / g:.3f},alt_thr_auth={auth / g:.3f}")

    def _data_row(self, rx_ms, t_ms, seq, frame):
        a = self.last[TYPE_ALT] or (0,) * 16
        b = self.last[TYPE_POS] or (0,) * 16
        c = self.last[TYPE_ATT] or (0,) * 19
        d = self.last[TYPE_DV] or (0,) * 13
        f = self.live_flags
        m = self.live_modes
        # A: type seq flags t_cs modes thr roll pitch yaw rh rraw hold climb vzt corr thr_out
        # B: type seq flags t_cs modes bad vx vy vxt vyt pn pe hn he lr lp
        # C: type seq flags t_cs modes sat m1 m2 m3 m4 rr pr yr rrt prt rc pc rs ps
        # D: type seq flags t_cs dvx dvy yaw age good lost bad cseq crssi
        cmd_age = -1.0 if d[7] == 0xFFFF else d[7] / 100.0   # 0xFFFF = 未受信
        return [
            str(int(rx_ms)), str(int(t_ms)), str(seq), str(self.n_lost), "-1", str(frame),
            str(m & 0x0F), str((m >> 4) & 0x0F), str(_bit(f, F_ARMED)), str(_bit(f, F_FLOW_OK)),
            str(_bit(f, F_RANGE_OK)), str(_bit(f, F_RANGE_VALID)),
            str(_bit(f, F_ALT_EN)), str(_bit(f, F_ALT_ACT)), str(_bit(f, F_POS_HOLD)),
            str(_bit(f, F_AIRBORNE)), str(_bit(f, F_DRY_RUN)), str(_bit(f, F_SAT)),
            str(_bit(f, F_TX_DROP)),
            str(_bit(f, F_GUIDED)), str(_bit(f, F_CMD_FRESH)), str(_bit(f, F_LANDED)),
            str(_bit(f, F_MANEUVER)), str(_bit(f, F_FRAME_OK)),
            f"{a[5] / 250.0:.3f}", str(b[5]),
            f"{a[6] / 100.0:.2f}", f"{a[7] / 100.0:.2f}", f"{a[8] / 10.0:.1f}",
            f"{a[9] / 1000.0:.3f}", f"{a[10] / 1000.0:.3f}", f"{a[11] / 1000.0:.3f}",
            f"{a[12] / 1000.0:.3f}", f"{a[13] / 1000.0:.3f}",
            f"{a[14] / 1e4:.4f}", f"{a[15] / 1e4:.4f}",
            f"{b[6] / 1000.0:.3f}", f"{b[7] / 1000.0:.3f}",
            f"{b[8] / 1000.0:.3f}", f"{b[9] / 1000.0:.3f}",
            f"{b[10] / 100.0:.2f}", f"{b[11] / 100.0:.2f}",
            f"{b[12] / 100.0:.2f}", f"{b[13] / 100.0:.2f}",
            f"{b[14] / 100.0:.2f}", f"{b[15] / 100.0:.2f}",
            f"{c[6] / 250.0:.3f}", f"{c[7] / 250.0:.3f}",
            f"{c[8] / 250.0:.3f}", f"{c[9] / 250.0:.3f}", str(c[5]),
            f"{c[10] / 10.0:.1f}", f"{c[11] / 10.0:.1f}", f"{c[12] / 10.0:.1f}",
            f"{c[13] / 10.0:.1f}", f"{c[14] / 10.0:.1f}",
            f"{c[15] / 1e4:.4f}", f"{c[16] / 1e4:.4f}",
            f"{c[17] / 100.0:.2f}", f"{c[18] / 100.0:.2f}",
            f"{d[4] / 1000.0:.3f}", f"{d[5] / 1000.0:.3f}", f"{d[6] / 10.0:.1f}",
            f"{cmd_age:.2f}", str(d[8]), str(d[9]), str(d[10]), str(d[11]), str(d[12]),
        ]


class BleS5Link:
    """
    log_recorder (BLE) 経由の地上局リンク。S5Link と差し替えて使う。

        link = BleS5Link(tap=ble_tap, log_dir=LOG_DIR)   # 既存の BleTap を共有
        link = BleS5Link(log_dir=LOG_DIR)                # 自前で BleTap を起こす
        if link.ok:
            link.send_command(REQ_TAKEOFF, 0.0, 0.0, 1.0)
            st = link.state()
        link.close()

    ★ tap を渡したときは、その tap の停止は呼び出し側の責任
      (close() は tap を止めない)。自前で起こした tap は close() が止める。
    """

    backend = "ble"

    def __init__(self, tap=None, name="S5-LogBLE", log_dir=None, on_message=None):
        self._say = on_message if on_message is not None else print
        self.port = f"BLE {name}"
        self.ok = False
        self._lock = threading.Lock()
        self._state = {}
        self._last_rx = 0.0
        self._n_data = 0
        self._n_sent = 0
        self._t_last_send = 0.0
        self._last_param_line = None
        self._dec = _TelemDecoder()
        self._t_open = time.time()
        self._csv = None
        self._closed = False

        self._own_tap = tap is None
        if tap is None:
            from core.ble_tap import BleTap
            tap = BleTap(Path(log_dir) if log_dir is not None else Path("logs"),
                         name=name, on_event=self._say)
            if not tap.start():
                self._say("[BleLink] BLE を使えません (上のメッセージ参照)。"
                          "機体へは何も送りません")
                self.tap = None
                return
        self.tap = tap
        self.name = tap.name
        self.port = f"BLE {tap.name}"

        if log_dir is not None:
            log_dir = Path(log_dir)
            log_dir.mkdir(parents=True, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._csv_path = log_dir / f"s5_link_{ts}.csv"
            self._csv = open(self._csv_path, "w", encoding="utf-8", newline="")
            self._csv.write("HEADER,Epoch_s," + ",".join(CSV_COLUMNS) + "\n")
            self._say(f"[BleLink] テレメトリ保存先: {self._csv_path}")

        tap.add_telem_listener(self._on_telem)
        self.ok = True
        self._say(f"[BleLink] 地上局リンク = BLE ('{tap.name}')。"
                  "接続はバックグラウンドで待ちます")

    # ---------------------------------------------------------------- 受信
    def _on_telem(self, payload):
        now = time.time()
        rx_ms = (now - self._t_open) * 1000.0
        with self._lock:
            if self._closed:
                return
            items = self._dec.feed(payload, rx_ms)
            for kind, body in items:
                if kind == "DATA":
                    if self._csv is not None:
                        self._csv.write(f"DATA,{now:.3f}," + ",".join(body) + "\n")
                    d = {}
                    for k, v in zip(CSV_COLUMNS, body):
                        try:
                            d[k] = float(v)
                        except ValueError:
                            d[k] = v
                    self._state = d
                    self._last_rx = now
                    self._n_data += 1
                elif kind == "PARAM":
                    self._last_param_line = "PARAM," + body
                    if self._csv is not None:
                        self._csv.write(f"PARAM,{now:.3f},{body}\n")
        for kind, body in items:
            if kind == "WARN":
                self._say(f"[BleLink] {body}")

    # ---------------------------------------------------------------- 状態
    def state(self):
        with self._lock:
            return dict(self._state)

    def age(self):
        with self._lock:
            if self._last_rx == 0.0:
                return float("inf")
            return time.time() - self._last_rx

    def telemetry_ok(self, max_age_s=1.0):
        return self.age() < max_age_s

    def n_data(self):
        with self._lock:
            return self._n_data

    def diagnostics(self):
        """S5Link.diagnostics() に相当するもの (ui/link_status.py 用)。
        地上局 XIAO の STAT は無いので、PC で数えた値と BLE の状態で埋める。"""
        st = self.tap.status() if self.tap is not None else {}
        with self._lock:
            dec = self._dec
            d = {
                "ble_connected": 1 if st.get("connected") else 0,
                "ble_err": st.get("err", ""),
                "telem_frames": st.get("n_telem", 0),
                "ctrl_tx": st.get("n_ctrl_tx", 0),
                "ctrl_err": st.get("n_ctrl_err", 0),
                "alt": dec.n[TYPE_ALT], "pos": dec.n[TYPE_POS], "att": dec.n[TYPE_ATT],
                "dv": dec.n[TYPE_DV], "param": dec.n[TYPE_PARAM],
                "lost": dec.n_lost, "badcs": 0, "badlen": dec.n_badlen,
                "cmd_lines": self._n_sent, "cmd_tx": st.get("n_ctrl_tx", 0),
                "cmd_have": 1 if (time.time() - self._t_last_send) < 1.5 else 0,
            }
            dv = dec.last[TYPE_DV]
            if dv is not None:
                d["fc_cmd_age_cs"] = dv[7]
                d["fc_cmd_good"] = dv[8]
                d["fc_cmd_lost"] = dv[9]
                d["fc_cmd_bad"] = dv[10]
        return d

    def flag(self, name):
        return bool(self.state().get(name, 0))

    # ---------------------------------------------------------------- 送信
    def send_command(self, req, vx_mps=0.0, vy_mps=0.0, alt_m=0.0,
                     yaw_rate_dps=0.0, flags=0, corr_n_m=None, corr_e_m=None,
                     yaw_abs_deg=None, laps=0):
        """S5Link.send_command() と同じ引数・同じ意味。
        省略された corr_* / yaw_abs / laps は 0 で埋める (ground_receiver Uplink.h と同じ)。
        それらを機体に適用させるかどうかは flags (CF_POS_CORR / CF_YAW_VALID) が決める。"""
        if not self.ok or self.tap is None:
            return
        corr_n = corr_e = 0
        if corr_n_m is not None and corr_e_m is not None:
            corr_n = _q16(corr_n_m, 1000.0)
            corr_e = _q16(corr_e_m, 1000.0)
        yaw_abs = _q16(yaw_abs_deg, 100.0) if yaw_abs_deg is not None else 0
        payload = _CMD_STRUCT.pack(
            CMD_MAGIC, CMD_VERSION, 0, int(req) & 0xFF,
            _q16(vx_mps, 1000.0), _q16(vy_mps, 1000.0),
            _q16(alt_m, 100.0), _q16(yaw_rate_dps, 100.0),
            int(flags) & 0xFFFF, corr_n, corr_e, yaw_abs,
            max(0, min(255, int(laps))), 0)
        with self._lock:
            self._n_sent += 1
            self._t_last_send = time.time()
        self.tap.send_ctrl(payload)

    def send_key(self, ch):
        """IM920 地上局 XIAO へのキー転送 (S/D/Z/C) に相当するもの。
        BLE 版には地上局 XIAO が無いので、'z' (統計クリア) だけ PC 側で行う。"""
        if str(ch).lower() == "z":
            with self._lock:
                self._dec.n_lost = 0
                self._dec.n_badlen = 0
                for k in self._dec.n:
                    self._dec.n[k] = 0
            self._say("[BleLink] 受信統計をクリアしました")
            return
        self._say(f"[BleLink] '{ch}' は IM920 地上局用のキーです (BLE 版では無効。"
                  "log_recorder の状態は XIAO の USB シリアル 's' で見る)")

    def n_sent(self):
        with self._lock:
            return self._n_sent

    # ---------------------------------------------------------------- 終了
    def close(self):
        if not self.ok:
            return
        # 最後に必ず ABORT を送る (S5Link.close() と同じ)。BLE の書き込みは
        # 別スレッドで非同期に出るので、少し待ってから閉じる。
        try:
            self.send_command(REQ_ABORT)
            time.sleep(0.3)
        except Exception:
            pass
        with self._lock:
            self._closed = True
            if self._csv is not None:
                self._csv.close()
        if self._csv is not None:
            self._say(f"[BleLink] テレメトリを保存しました: {self._csv_path}")
        if self._own_tap and self.tap is not None:
            self.tap.stop()
        self._say(f"[BleLink] 切断 (受信 {self._n_data} 行 / 送信 {self._n_sent} 回)")
        self.ok = False


def open_ground_link(backend, *, port=None, tap=None, ble_name="S5-LogBLE",
                     log_dir=None, on_message=None):
    """utils/config.py の GROUND_LINK_BACKEND に従って地上局リンクを開く。
        "ble"   -> BleS5Link (tap があれば共有)
        "im920" -> S5Link    (ground_receiver の xiao_s5_log を USB で開く)
    """
    backend = str(backend).lower()
    if backend == "ble":
        return BleS5Link(tap=tap, name=ble_name, log_dir=log_dir, on_message=on_message)
    if backend == "im920":
        from core.s5_link import S5Link
        return S5Link(port=port, log_dir=log_dir, on_message=on_message)
    raise ValueError(f"GROUND_LINK_BACKEND は 'ble' か 'im920' (今: {backend!r})")
