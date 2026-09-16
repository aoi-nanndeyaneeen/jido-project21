"""
app/main_loop.py
PC 側メインループ: カメラ追跡の結果を受けて、ヨー推定 → ミッション (10Hz 指令) →
可視化 を回す。ウィンドウ管理と終了処理もここ。

==========================================================================
処理の流れと周期 (2026-09-16 整理)
==========================================================================

    カメラスレッド (core/tracker.py)  15〜60Hz (カメラの fps)
        read_and_detect ×2 → 点滅ロックイン → ペア選択 (三角測量) → plot_data["updated"]=True
              │
              ▼  メインスレッド (このファイル)。plot_data を毎周ポーリング
    [毎周]      yaw_est.add_position() / D フレームの Δv を yaw_est.add_body_dv()
    [新フレームのとき]
                cv2.imshow (Camera 1/2)
                yaw_est.update()                 → カメラ絶対ヨー (収束していれば)
                YawArbiter.arbitrate()           → ミッションに渡す機首方位 ("camera"/"fixed")
                mission.update()                 → 内部で SEND_HZ (10Hz) に間引いて CMD 送信
                mission_log (指令が出た回だけ 1 行)
    [5Hz]       Velocity / Link Status / Graph の描画 (matplotlib 系は重いので間引く)
    [2Hz]       端末の状態画面 (utils/screen.Screen で上書き)
    [毎周]      cv2.waitKey(1) → q / b キー

    地上局リンク (core/s5_link.py) は別スレッドで USB を読み、state() のスナップショットを返す。
    BLE (core/ble_tap.py) は機体 125Hz ログの受信だけで、追跡・ミッションには関与しない。

★ ミッション指令のレートを決めているのは mission.py の SEND_HZ (10Hz) であって
  カメラの fps ではない。カメラが 60fps でも指令は 10Hz、15fps でも 10Hz。
★ 機体へ渡るのは setpoint (目標速度/高度) だけ。ここでは姿勢を一切計算しない
  (理由は protocol/S5Cmd.h 冒頭と WAYPOINT_BRINGUP.md §0)。
"""

import cv2
import math
import os
import signal
import threading
import time
import msvcrt

from utils.config import (DISP_W, DISP_H,
                          YAW_ENABLED, YAW_INITIAL_ALIGN_DEG,
                          GROUND_LINK_ENABLED, GROUND_LINK_PORT,
                          MISSION_WAYPOINTS, MISSION_TAKEOFF_ALT_M,
                          MISSION_AUTOSTART, LOG_DIR,
                          BLE_LOG_ENABLED, BLE_LOG_NAME)
from core.tracker import camera_thread_func
from core.yaw_estimator import YawEstimator
from core.yaw_source import YawArbiter
from core.s5_link import S5Link
from core.mission import WaypointMission as Mission, Phase as MissionPhase
from utils.logger import PerformanceLogger, MissionLogger
from ui.dashboard import Dashboard
from ui.view_velocity import ViewVelocity
from ui.link_status import LinkStatusView
from ui.window_layout import primary_display_size, flight_window_layout, apply_window_layout
from utils.screen import Screen

# ---- 周期 --------------------------------------------------------------------
MPL_RENDER_HZ   = 5.0    # Velocity / Graph / Link Status の描画。速くしても読めない
STATUS_REDRAW_S = 0.5    # 端末の状態画面 (2Hz)
KEY_POLL_S      = 0.05   # msvcrt キー入力のポーリング
MESSAGE_LINES   = 8      # 状態画面の下に残すイベント行数


def _fmt_pos(P, residual, tracking_ok, in_dummy):
    if in_dummy:
        return "[DUMMY 円軌道] カメラ見失い中"
    if P is None:
        return "未検出"
    tag = "REAL" if tracking_ok else "reject"
    err = f"  err:{residual:.3f}m" if residual is not None else ""
    return f"X:{P[0]:+.2f} Y:{P[1]:+.2f} Z:{P[2]:+.2f}m{err}  [{tag}]"


class FlightLoop:
    """run_main_loop() の中身。段階ごとにメソッドを分けてある (読む順 = 呼ぶ順)。"""

    def __init__(self, cam1, cam2, calib1, calib2, log_path, field_points):
        self.cam1, self.cam2 = cam1, cam2
        self.calib1, self.calib2 = calib1, calib2
        self.log_path = log_path
        self.field_points = field_points

        # ---- カメラスレッドとの共有 ----
        self.plot_lock = threading.Lock()
        self.plot_data = {
            "P": None, "O1": None, "O2": None,
            "current_z": 0.0, "residual": None,
            "uv1": None, "uv2": None,
            "updated": False,
            "frame1": None, "frame2": None,
            "in_dummy": False, "tracking_ok": False, "frame_time": 0.0,
        }
        self.shared = {"do_bg_reset": False, "quit": False,
                       "mission_request": None, "tracker_status": {}}

        # ---- 流れない端末表示 (console.py と同じ Screen) ----
        #  起動時のバナー・カメラ初期化ログは流して問題ない (1 回きり)。流れると困るのは
        #  ループ中の定期ログとイベント通知で、後者は say() で溜めて画面下に出す。
        self.messages = []
        self._msg_lock = threading.Lock()
        self.screen = Screen()
        self.shared["say"] = self.say     # カメラスレッドからも同じ経路で

        # ---- 推定・制御 ----
        self.yaw_est = YawEstimator() if YAW_ENABLED else None
        self.yaw_arbiter = YawArbiter(say=self.say)
        self._last_dv_t_ms = None         # D フレームの重複排除 (state() はキューではない)
        self.link = None
        self.mission = None
        self.mission_log = None
        self._last_n_tx = 0
        self._mission_aligned_logged = False
        self._prev_guided = False         # MISSION_AUTOSTART のエッジ検出用
        self.ble_tap = None
        self.cam_thread = None

        # ---- 可視化 ----
        self.dashboard = None
        self.velocity_view = None
        self.link_view = None
        self.display_perf_log = PerformanceLogger(
            log_path.with_name(log_path.stem + "_display_perf.csv"))
        self._last_mpl_render = 0.0
        self._last_status_draw = time.time()
        self._display_values = {"Display_ms": 0.0, "Velocity_ms": 0.0, "Graph_ms": 0.0}

    # ------------------------------------------------------------------ 補助
    def say(self, msg):
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        with self._msg_lock:
            self.messages.append(line)
            del self.messages[:-MESSAGE_LINES]

    # ------------------------------------------------------------------ 起動
    def setup(self):
        if self.yaw_est is not None:
            print(f"[Yaw] 推定を有効化 (初期アラインメント "
                  f"{YAW_INITIAL_ALIGN_DEG:+.1f}deg = 機首をフィールド奥+yへ)")
            print("[Yaw] 機体Δvは地上局リンク (S5Link, IM920のDフレーム) から受け取ります。"
                  "GROUND_LINK_ENABLED=False またはリンク未接続だと収束しません")

        print()
        print("=" * 56)
        print("   ALL SYSTEMS GO  -  STEREO TRACKING STARTED")
        print("=" * 56)
        print()
        print("  [M]     ミッション待機の開始/やり直し (自動開始が有効なら押す必要はありません)")
        print("  [X]     ミッション中断 (その場から自動着陸)")
        print("  [C]/[8]/[U]  巡航中の今の場所から 水平旋回 / 8の字 / 上昇旋回 "
              "(終了後は離陸地点へ帰投。半径・周回数は config MISSION_MANEUVER_*)")
        print("  [B]     背景リセット (両カメラ)")
        print("  [Q]     終了")
        print()

        # Ctrl+C は例外で抜けずに [Q] と同じ終了処理へ通す。例外で抜けるとミッション中断・
        # リンクの ABORT・カメラ解放が飛ばされ、プロセスが裏に残ってカメラと COM を掴み続けた
        # (2026-09-15)。2 回押したら終了処理を待たずに強制終了する。
        def _on_sigint(signum, frame):
            if self.shared["quit"]:
                print("\n[Main] Ctrl+C 2回目 → 強制終了")
                os._exit(1)
            self.say("[Main] Ctrl+C → 終了処理に入ります (もう一度押すと強制終了)")
            self.shared["quit"] = True
        signal.signal(signal.SIGINT, _on_sigint)

        # ---- ウィンドウ ----
        cv2.namedWindow("Camera 1", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Camera 2", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Velocity", cv2.WINDOW_NORMAL)
        self.dashboard     = Dashboard(self.field_points)
        self.velocity_view = ViewVelocity(self.field_points)
        self.link_view     = LinkStatusView()
        screen_w, screen_h = primary_display_size()
        apply_window_layout(flight_window_layout(screen_w, screen_h))
        print(f"[UI] 主ディスプレイ {screen_w}x{screen_h}: "
              "上段=Camera 1/2、下段=Velocity:Graph (2:1) に配置しました")

        # ---- スレッド ----
        threading.Thread(target=self._keyboard_thread, daemon=True).start()
        self.cam_thread = threading.Thread(
            target=camera_thread_func,
            args=(self.cam1, self.cam2, self.calib1, self.calib2,
                  self.log_path, self.shared, self.plot_lock, self.plot_data),
            daemon=True)
        self.cam_thread.start()

        # ---- BLE (機体 125Hz ログ)。地上局とは別デバイスなので、繋がらなくても続行 ----
        if BLE_LOG_ENABLED:
            from core.ble_tap import BleTap
            tap = BleTap(LOG_DIR, name=BLE_LOG_NAME)
            self.ble_tap = tap if tap.start() else None

        # ---- 地上局リンク (機体へ指令を送る唯一の経路) ----
        #  ここが None のままだと、機体は一切動かない (見ているだけ)。
        if GROUND_LINK_ENABLED:
            print()
            print("[INIT] 地上局 (XIAO / xiao_s5_log) へ接続中...")
            link = S5Link(port=GROUND_LINK_PORT, log_dir=LOG_DIR)
            if link.ok:
                self.link = link
                self.mission = Mission(link, MISSION_WAYPOINTS)
                self.mission.TAKEOFF_ALT_M = MISSION_TAKEOFF_ALT_M
                self.mission_log = MissionLogger(
                    self.log_path.with_name(
                        self.log_path.stem.replace("flight_", "mission_") + ".csv"))
                print(f"       ミッションログ: {self.mission_log.path.name}")
                if MISSION_AUTOSTART:
                    # start() はここでは呼ばない。SW_HOVER を GUIDED (up) に上げた瞬間
                    # (= link.flag("guided") の立ち上がり) を _step_mission が見て始める。
                    print("  [OK] 自動開始が有効です。プロポを GUIDED (SW_HOVERを上)"
                          "にするたびにミッションが最初から走ります ([X] で中断)")
                else:
                    print("  [OK] ミッション準備完了。[M] で開始します")
                for i, wp in enumerate(MISSION_WAYPOINTS):
                    print(f"       WP{i}: ({wp[0]:+.2f}, {wp[1]:+.2f}, {wp[2]:.2f})")
            else:
                print("  [SKIP] 地上局に接続できません。ミッション指令は停止し、"
                      "ダミー追跡を含む表示のみで続行します")
        else:
            print("[INIT] GROUND_LINK_ENABLED=False のため機体へは何も送りません。"
                  "ダミー追跡を含む表示のみで続行します")

    def _keyboard_thread(self):
        simple = {b'q': ("[KEY] Q → 終了", "quit", True),
                  b'b': ("[KEY] B → 背景リセット", "do_bg_reset", True),
                  b'm': ("[KEY] M → ミッション開始要求", "mission_request", "start"),
                  b'x': ("[KEY] X → ミッション中断要求", "mission_request", "abort")}
        maneuvers = {b'c': "circle", b'8': "figure8", b'u': "climb"}
        while not self.shared.get("quit", False):
            if msvcrt.kbhit():
                key = msvcrt.getch().lower()
                if key in simple:
                    msg, name, value = simple[key]
                    self.say(msg)
                    self.shared[name] = value
                elif key in maneuvers:
                    self.say(f"[KEY] {key.decode().upper()} → 定型機動 {maneuvers[key]} 要求")
                    self.shared["mission_request"] = "maneuver:" + maneuvers[key]
            time.sleep(KEY_POLL_S)

    # ------------------------------------------------------------------ ループ
    def run(self):
        while not self.shared.get("quit", False):
            snap = self._poll_tracker()
            self._feed_yaw_estimator(snap)
            if snap["updated"]:
                t0 = time.perf_counter()
                self._show_cameras()
                camera_yaw = self._update_yaw()
                self._step_mission(snap, camera_yaw)
                self._render_views(snap)
                self._display_values["Display_ms"] = (time.perf_counter() - t0) * 1000.0
                self.display_perf_log.write("display", self._display_values)
                self._redraw_status(snap)
            self._handle_cv_keys()
        self._shutdown()

    def _poll_tracker(self):
        """カメラスレッドの最新結果を 1 回だけ読む (updated フラグは消費する)。"""
        with self.plot_lock:
            snap = {k: self.plot_data.get(k) for k in
                    ("updated", "P", "O1", "current_z", "tracking_ok",
                     "frame_time", "in_dummy", "residual")}
            if snap["updated"]:
                self.plot_data["updated"] = False
        snap["tracking_ok"] = bool(snap["tracking_ok"])
        snap["in_dummy"] = bool(snap["in_dummy"])
        snap["frame_time"] = snap["frame_time"] or 0.0
        return snap

    def _feed_yaw_estimator(self, snap):
        """カメラ位置は毎フレーム、機体Δv (D フレーム) は届いたぶんを推定器へ。単一スレッドで扱う。"""
        if self.yaw_est is None:
            return
        if snap["updated"] and snap["frame_time"] > 0.0:
            self.yaw_est.add_position(snap["frame_time"], snap["P"], snap["tracking_ok"])
        if self.link is not None:
            st = self.link.state()
            if st.get("frame") == 3:      # 0:A 1:B 2:C 3:D
                t_ms, dvx, dvy = st.get("t_ms"), st.get("dvx"), st.get("dvy")
                if (t_ms is not None and dvx is not None and dvy is not None
                        and t_ms != self._last_dv_t_ms):
                    self._last_dv_t_ms = t_ms
                    self.yaw_est.add_body_dv(int(t_ms), float(dvx), float(dvy),
                                             yaw_body=st.get("dv_yaw"))

    def _show_cameras(self):
        with self.plot_lock:
            f1, f2 = self.plot_data.get("frame1"), self.plot_data.get("frame2")
        if f1 is not None:
            disp1 = cv2.resize(f1, (DISP_W, DISP_H))
            if self.yaw_est is not None:
                ye = self.yaw_est.current()
                cv2.putText(disp1, self.yaw_est.status_line(), (10, DISP_H - 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (120, 255, 120) if ye.valid else (140, 140, 140), 2)
            cv2.imshow("Camera 1", disp1)
        if f2 is not None:
            cv2.imshow("Camera 2", cv2.resize(f2, (DISP_W, DISP_H)))

    def _update_yaw(self):
        """ヨー推定を 1 ステップ進める (1 ループ 1 回)。収束していれば rad、なければ None。"""
        if self.yaw_est is None:
            return None
        ye = self.yaw_est.update(time.time())
        return math.radians(ye.yaw_deg) if ye.valid else None

    def _step_mission(self, snap, camera_yaw_rad):
        if self.mission is None:
            return
        mission, link = self.mission, self.link

        # SW_HOVER を GUIDED (up) に上げた瞬間 (立ち上がりエッジ) に自動で開始。
        # 入りっぱなしの間 (着陸直後でまだ THR_CUT していない) に毎フレーム start() を
        # 呼ぶと着陸完了の瞬間にまた離陸するので、エッジだけを見る。
        guided_now = bool(link.flag("guided"))
        if (MISSION_AUTOSTART and guided_now and not self._prev_guided
                and mission.phase in (MissionPhase.IDLE, MissionPhase.DONE, MissionPhase.ABORT)):
            self.say("[Mission] GUIDED 検出 (SW_HOVER 上) -> ミッションを最初から開始")
            mission.start()
        self._prev_guided = guided_now

        # キー要求は update() より先に処理する (同じフレームで「開始 → 1 回目の指令」まで進む)
        req = self.shared.get("mission_request")
        if req is not None:
            self.shared["mission_request"] = None
            if req == "start":
                mission.start()
            elif req == "abort":
                mission.abort("キー操作")
            elif req.startswith("maneuver:"):
                mission.request_maneuver(req.split(":", 1)[1])

        # カメラが機体を捉えているか。in_dummy (仮想円軌道) は「捉えていない」。
        # ここを True にすると架空の位置で位置ループを閉じることになる。
        P = snap["P"]
        pos_valid = snap["tracking_ok"] and not snap["in_dummy"] and P is not None

        # 機首方位の出所を決める (core/yaw_source.py)
        m_yaw, m_yaw_valid, yaw_src = self.yaw_arbiter.arbitrate(camera_yaw_rad, link.state())

        mission.update(pos=P, yaw_rad=m_yaw, pos_valid=pos_valid,
                       yaw_valid=m_yaw_valid, yaw_src=yaw_src)

        # ミッションログは指令が出た回だけ (カメラの 60Hz で書くと同じ指令が 12 行並ぶ)
        if self.mission_log is not None and mission.n_tx() != self._last_n_tx:
            self._last_n_tx = mission.n_tx()
            tel = link.state()
            tel["age"] = link.age()
            cam = {"x": None if P is None else float(P[0]),
                   "y": None if P is None else float(P[1]),
                   "z": None if P is None else float(P[2]),
                   "valid": pos_valid, "in_dummy": snap["in_dummy"],
                   "residual": snap["residual"]}
            # 原点合わせと Diff_* は mission.py が位置補正のために持っている値をそのまま使う
            mission_snap = mission.snapshot()
            if mission_snap["aligned"] and not self._mission_aligned_logged:
                self._mission_aligned_logged = True
                self.say("[Mission] カメラと機体フローの原点を合わせました "
                         "(以後 Diff_* が機体側のドリフト量)")
            self.mission_log.write(
                mission_snap, cam, tel,
                {"deg": None if m_yaw is None else math.degrees(m_yaw),
                 "valid": m_yaw_valid, "src": yaw_src},
                event=mission.take_event())

    def _target_alt_for_graph(self):
        """Graph の Target 線。ミッションの目標高度 → 機体の保持高度 → 既定、の順で拾う。
        (以前は [T] キーで手入力する表示専用の値だった。機体には無関係だったので廃止)"""
        if self.mission is not None and self.mission.phase not in (MissionPhase.IDLE, MissionPhase.DONE):
            tgt = self.mission.snapshot().get("tgt")
            if tgt is not None:
                return float(tgt[2])
        if self.link is not None:
            alt_hold = self.link.state().get("alt_hold")
            if isinstance(alt_hold, float) and alt_hold > 0.0:
                return alt_hold
        return MISSION_TAKEOFF_ALT_M

    def _render_views(self, snap):
        """matplotlib 系 (重い) は MPL_RENDER_HZ に間引く。"""
        now = time.time()
        self._display_values["Velocity_ms"] = 0.0
        self._display_values["Graph_ms"] = 0.0
        if now - self._last_mpl_render <= 1.0 / MPL_RENDER_HZ:
            return
        self._last_mpl_render = now

        # ADI へ渡す姿勢は IM920 で届く実機の roll/pitch
        roll_deg, pitch_deg, imu_available, attitude_source = 0.0, 0.0, False, None
        tel = self.link.state() if self.link is not None else {}
        if self.link is not None and self.link.telemetry_ok() and "roll" in tel and "pitch" in tel:
            roll_deg, pitch_deg = float(tel["roll"]), float(tel["pitch"])
            imu_available, attitude_source = True, "IM920"

        t = time.perf_counter()
        cv2.imshow("Velocity", self.velocity_view.get_image(
            snap["P"], roll_deg, pitch_deg, imu_available, attitude_source))
        self._display_values["Velocity_ms"] = (time.perf_counter() - t) * 1000.0
        self.link_view.render_and_show(self.link)

        if snap["O1"] is not None:
            t = time.perf_counter()
            self.dashboard.render_and_show(snap["P"], snap["current_z"], self._target_alt_for_graph())
            self._display_values["Graph_ms"] = (time.perf_counter() - t) * 1000.0

    def _redraw_status(self, snap):
        if time.time() - self._last_status_draw < STATUS_REDRAW_S:
            return
        self._last_status_draw = time.time()
        self.screen.draw(self._status_lines(snap))

    def _status_lines(self, snap):
        """流れる print() の代わりに、毎フレーム同じ場所へ上書きする状態画面。"""
        bar, thin = "=" * 72, "-" * 72
        ts = self.shared.get("tracker_status", {})
        b1 = ts.get("blink1", (0.0, 0.0))
        b2 = ts.get("blink2", (0.0, 0.0))
        lines = [bar,
                 f" POSITION ESTIMATOR  -  FLIGHT MONITOR    {time.strftime('%H:%M:%S')}"
                 f"    frame {ts.get('frame_count', 0)}",
                 bar,
                 f" TRACK  {_fmt_pos(snap['P'], snap['residual'], snap['tracking_ok'], snap['in_dummy'])}",
                 f" CAM1   cand {ts.get('n_cand1', '-')}   blink {b1[0]:.2f}/{b1[1]:.1f}   "
                 f"{ts.get('cam1_ms', 0.0):.1f}ms   fps {ts.get('reader_fps1', 0.0):.1f}",
                 f" CAM2   cand {ts.get('n_cand2', '-')}   blink {b2[0]:.2f}/{b2[1]:.1f}   "
                 f"{ts.get('cam2_ms', 0.0):.1f}ms   fps {ts.get('reader_fps2', 0.0):.1f}",
                 thin]
        if self.mission is not None:
            lines.append(f" MISSION  {self.mission.status_line()}")
        if self.yaw_est is not None:
            lines.append(f" {self.yaw_est.status_line()}")
        if self.link is not None:
            age = self.link.age()
            age_s = "未受信" if age == float("inf") else f"{age:4.2f}s"
            lines.append(f" LINK   {self.link.port or '(ポート無し)'}   age {age_s}   "
                         f"受信 {self.link.n_data()}")
        if self.ble_tap is not None:
            st = self.ble_tap.status()
            ble_s = (f"接続 {st['rate_hz']:5.1f}Hz {st['n_rec']}rec"
                     if st["connected"] else f"未接続 ({st['err'] or 'スキャン中'})")
            lines.append(f" BLE    {ble_s}")
        lines.append(f" PERF   display {self._display_values['Display_ms']:.1f}ms   "
                     f"loop {ts.get('loop_ms', 0.0):.1f}ms")
        lines.append(thin)
        lines.append(" [M]開始/やり直し  [X]中断  [C]/[8]/[U]定型機動  [B]背景リセット  [Q]終了")
        lines.append(thin)
        with self._msg_lock:
            lines += [" " + m[:70] for m in self.messages[-MESSAGE_LINES:]]
        return lines

    def _handle_cv_keys(self):
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.shared["quit"] = True
        elif key == ord('b'):
            self.shared["do_bg_reset"] = True

    # ------------------------------------------------------------------ 終了
    def _shutdown(self):
        # リンクを閉じる前に必ずミッションを止める。close() が ABORT を送るので機体は
        # 自動着陸へ落ちるが、飛行中に Q を押した場合はそれでも「降りてくるまで見ていること」。
        # 機体が既にディスアーム済み/地上なら待たない (以前は着陸後の Q で 20 秒待っていた)。
        link, mission = self.link, self.mission
        in_air = (link is not None and link.ok and link.n_data() > 0
                  and link.flag("armed") and link.flag("airborne"))
        if mission is not None and in_air and mission.phase not in (MissionPhase.IDLE, MissionPhase.DONE):
            print("[Mission] 終了要求 → 自動着陸を指示します。着地を見届けてください")
            mission.abort("プログラム終了")
            for _ in range(200):                 # 最大 20 秒だけ着陸に付き合う
                mission.update()
                if mission.phase is MissionPhase.DONE or not link.flag("armed"):
                    break
                time.sleep(0.1)
        if link is not None:
            link.close()
        if self.mission_log is not None:
            self.mission_log.close()
            print(f"[Mission] ミッションログを保存しました: {self.mission_log.path}")
        if self.ble_tap is not None:
            self.ble_tap.stop()

        self.dashboard.close()
        self.velocity_view.close()
        self.link_view.close()
        self.display_perf_log.close()
        self.cam_thread.join(timeout=2.0)   # カメラを解放する前に追跡スレッドを止める
        cv2.destroyAllWindows()
        for _ in range(10):
            cv2.waitKey(1)


def run_main_loop(cam1, cam2, calib1, calib2, log_path, field_points):
    loop = FlightLoop(cam1, cam2, calib1, calib2, log_path, field_points)
    loop.setup()
    loop.run()
