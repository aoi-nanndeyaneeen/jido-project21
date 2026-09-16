"""
app/main_loop.py
メインループ本体：ウィンドウ管理・カメラスレッド起動・
オートパイロット計算・各種可視化ウィンドウの更新を担当。
"""

import cv2
import math
import numpy as np
import os
import signal
import threading
import time
import msvcrt

from utils.config import (DISP_W, DISP_H, VELOCITY_W, VELOCITY_H,
                          YAW_ENABLED, YAW_INITIAL_ALIGN_DEG,
                          YAW_SEND_HZ, YAW_SEND_INITIAL_ALIGN,
                          GROUND_LINK_ENABLED, GROUND_LINK_PORT,
                          MISSION_WAYPOINTS, MISSION_TAKEOFF_ALT_M,
                          MISSION_YAW_MODE, MISSION_AUTOSTART, LOG_DIR,
                          YAW_CAMERA_MAX_DISAGREE_DEG,
                          BLE_LOG_ENABLED, BLE_LOG_NAME)
from core.tracker import camera_thread_func
from core.controller import AltitudeController
from core.geometry import accel_to_angles
from core.yaw_estimator import YawEstimator
from core.s5_link import S5Link
from core.mission import WaypointMission as Mission, Phase as MissionPhase
from utils.logger import PerformanceLogger, MissionLogger
from ui.dashboard import Dashboard
from ui.view_velocity import ViewVelocity
from ui.link_status import LinkStatusView
from ui.window_layout import primary_display_size, flight_window_layout, apply_window_layout


def run_main_loop(cam1, cam2,
                  calib1, calib2,
                  log_path,
                  alt_sensor,
                  field_points):

    plot_lock = threading.Lock()
    plot_data = {
        "P": None, "O1": None, "O2": None,
        "current_z": 0.0, "residual": None,
        "uv1": None, "uv2": None,
        "updated": False,
        "frame1": None, "frame2": None,
        "in_dummy": False, "tracking_ok": False, "frame_time": 0.0,
    }

    # ── ヨー方位推定 (YAW_HANDOFF.md) ───────────────────────
    yaw_est = YawEstimator() if YAW_ENABLED else None
    last_yaw_send = 0.0
    if yaw_est is not None:
        print(f"[Yaw] 推定を有効化 (初期アラインメント "
              f"{YAW_INITIAL_ALIGN_DEG:+.1f}deg = 機首をフィールド奥+yへ)")
        print("[Yaw] 機体Δvは地上局リンク (S5Link, IM920のDフレーム) から受け取ります。"
              "GROUND_LINK_ENABLED=False またはリンク未接続だと収束しません "
              "(この下の [INIT] 地上局... を見てください)")

    print()
    print("=" * 56)
    print("   ALL SYSTEMS GO  -  STEREO TRACKING STARTED")
    print("=" * 56)
    print()
    print("  [M]     ミッション待機の開始/やり直し "
          "(自動開始が有効なら押す必要はありません)")
    print("  [X]     ミッション中断 (その場から自動着陸)")
    print("  [C]/[8]/[U]  巡航中の今の場所から 水平旋回 / 8の字 / 上昇旋回 "
          "(終了後は離陸地点へ帰投。半径・周回数は config MISSION_MANEUVER_*)")
    print("  [B]     背景リセット (両カメラ)")
    print("  [Q]     終了")
    print()

    shared = {"do_bg_reset": False, "quit": False,
              "mission_request": None}

    # ★ Ctrl+C は例外で抜けずに [Q] と同じ終了処理へ通す。例外で抜けると
    #   ミッション中断・リンクの ABORT・カメラ解放が飛ばされ、2026-09-15 には
    #   プロセスが裏に残ってカメラと COM を掴み続け、次の起動がカメラ2を
    #   開けずに止まった。2回押したら終了処理を待たずに強制終了する。
    def _on_sigint(signum, frame):
        if shared["quit"]:
            print("\n[Main] Ctrl+C 2回目 → 強制終了")
            os._exit(1)
        print("\n[Main] Ctrl+C → 終了処理に入ります (もう一度押すと強制終了)")
        shared["quit"] = True
    signal.signal(signal.SIGINT, _on_sigint)

    # ── ウィンドウ作成 ──────────────────────────────────────
    cv2.namedWindow("Camera 1", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Camera 2", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Velocity", cv2.WINDOW_NORMAL)

    def keyboard_thread():
        while not shared.get("quit", False):
            if msvcrt.kbhit():
                key = msvcrt.getch().lower()
                if key == b'q':
                    print("[KEY] Q → 終了")
                    shared["quit"] = True
                elif key == b'b':
                    print("[KEY] B → 背景リセット")
                    shared["do_bg_reset"] = True
                elif key == b'm':
                    print("[KEY] M → ミッション開始要求")
                    shared["mission_request"] = "start"
                elif key == b'x':
                    print("[KEY] X → ミッション中断要求")
                    shared["mission_request"] = "abort"
                elif key in (b'c', b'8', b'u'):
                    kind = {b'c': "circle", b'8': "figure8", b'u': "climb"}[key]
                    print(f"[KEY] {key.decode().upper()} → 定型機動 {kind} 要求")
                    shared["mission_request"] = "maneuver:" + kind
            time.sleep(0.05)

    threading.Thread(target=keyboard_thread, daemon=True).start()

    # ── カメラスレッド起動 ──────────────────────────────────
    cam_thread = threading.Thread(
        target=camera_thread_func,
        args=(cam1, cam2, calib1, calib2,
              log_path, shared, plot_lock, plot_data),
        daemon=True)
    cam_thread.start()
    display_perf_log = PerformanceLogger(
        log_path.with_name(log_path.stem + "_display_perf.csv")
    )

    # ── 各ビュー初期化 ──────────────────────────────────────
    dashboard      = Dashboard(field_points)
    controller     = AltitudeController(p_gain=5.0)
    velocity_view  = ViewVelocity(field_points)
    link_view      = LinkStatusView()
    screen_w, screen_h = primary_display_size()
    apply_window_layout(flight_window_layout(screen_w, screen_h))
    print(f"[UI] 主ディスプレイ {screen_w}x{screen_h}: "
          "上段=Camera 1/2、下段=Velocity:Graph (2:1) に配置しました")
    # ── 地上局リンク (機体へ指令を送る唯一の経路) ───────────────
    #  ★ ここが None のままだと、機体は一切動かない (見ているだけ)。
    #    s5_logger.py を同時起動していると開けないので、その旨を出す。
    link = None
    mission = None
    mission_log = None
    _last_n_tx = [0]
    _mission_aligned_logged = [False]

    # ── BLE (機体125Hzログ) ── 地上局リンクとは別デバイスなので、
    #  つながらなくても追跡・ミッションはそのまま続く。
    ble_tap = None
    if BLE_LOG_ENABLED:
        from core.ble_tap import BleTap
        ble_tap = BleTap(LOG_DIR, name=BLE_LOG_NAME)
        if not ble_tap.start():
            ble_tap = None

    _prev_guided = [False]   # MISSION_AUTOSTART のエッジ検出用
    if GROUND_LINK_ENABLED:
        print()
        print("[INIT] 地上局 (XIAO / xiao_s5_log) へ接続中...")
        link = S5Link(port=GROUND_LINK_PORT, log_dir=LOG_DIR)
        if link.ok:
            mission = Mission(link, MISSION_WAYPOINTS)
            mission.TAKEOFF_ALT_M = MISSION_TAKEOFF_ALT_M
            mission_log = MissionLogger(
                log_path.with_name(log_path.stem.replace("flight_", "mission_")
                                   + ".csv"))
            print(f"       ミッションログ: {mission_log.path.name}")
            if MISSION_AUTOSTART:
                #  ★ ここでは start() しない。SW_HOVER を GUIDED (up) に
                #    上げた瞬間 (= link.flag("guided") の立ち上がりエッジ)
                #    をメインループ側で見て、そのたびに start() し直す。
                #    プログラム起動時点ではまだ何も送らない (IDLE のまま)。
                print("  [OK] 自動開始が有効です。プロポを GUIDED (SW_HOVERを上)"
                      "にするたびにミッションが最初から走ります")
                print("       ([X] で中断)")
            else:
                print(f"  [OK] ミッション準備完了。[M] で開始します")
            for i, wp in enumerate(MISSION_WAYPOINTS):
                print(f"       WP{i}: ({wp[0]:+.2f}, {wp[1]:+.2f}, {wp[2]:.2f})")
        else:
            link = None
            print("  [SKIP] 地上局に接続できません。ミッション指令は停止し、"
                  "ダミー追跡を含む表示のみで続行します")
    else:
        print("[INIT] GROUND_LINK_ENABLED=False のため機体へは何も送りません。"
              "ダミー追跡を含む表示のみで続行します")

    # 「初期アラインメントで飛んでいます」の警告を1回だけ出すための箱
    _warned_fixed_yaw = [False]
    # 機体の g_yaw_est がカメラの絶対ヨーで再基準済みか (アーム中のみ有効)
    _yaw_rebased = [False]
    _yaw_reject_warned = [False]
    # 直近に yaw_est へ渡した D フレームの t_ms。同じ値を2回渡さないための重複排除。
    #  ★ S5Link.state() は「最新のCSV行」のスナップショットを返すだけで
    #    キューではない。ここのポーリング周期(カメラフレーム毎)は
    #    D フレームの送信周期(POSHOLD/AUTOで3Hz)よりずっと速いので、
    #    取りこぼしはほぼ無い。
    _last_dv_t_ms = [None]

    last_mpl_render = 0.0
    MPL_RENDER_HZ   = 5
    display_perf_time = time.time()

    # ── メインループ ──────────────────────────────────────
    while not shared.get("quit", False):
        with plot_lock:
            updated     = plot_data["updated"]
            P           = plot_data["P"]
            O1          = plot_data["O1"]
            current_z   = plot_data["current_z"]
            tracking_ok = plot_data.get("tracking_ok", False)
            frame_time  = plot_data.get("frame_time", 0.0)
            in_dummy    = plot_data.get("in_dummy", False)
            residual    = plot_data.get("residual")
            if updated:
                plot_data["updated"] = False

        # ── ヨー推定への入力 ────────────────────────────────
        # カメラ位置は毎フレーム、機体Δvは届いたぶんをまとめて渡す。
        # 推定器は単一スレッドで扱う（受信スレッドはキューに積むだけ）。
        if yaw_est is not None:
            if updated and frame_time > 0.0:
                yaw_est.add_position(frame_time, P, tracking_ok)

            # ★ 本命: S5Link (IM920 の D フレーム、drone_s5.cpp が計算)。
            #   GROUND_LINK_ENABLED の通常運用ではこちらだけが動く。
            if link is not None and link.ok:
                st = link.state()
                if st.get("frame") == 3:      # 0:A 1:B 2:C 3:D (s5_log.cpp)
                    t_ms = st.get("t_ms")
                    dvx  = st.get("dvx")
                    dvy  = st.get("dvy")
                    if (t_ms is not None and dvx is not None and dvy is not None
                            and t_ms != _last_dv_t_ms[0]):
                        _last_dv_t_ms[0] = t_ms
                        yaw_est.add_body_dv(int(t_ms), float(dvx), float(dvy),
                                            yaw_body=st.get("dv_yaw"))

            # ★ 旧経路: main_pc.cpp 世代の別シリアル接続 (SERIAL_ENABLED)。
            #   現行の drone_s5.cpp では使わないが、レガシー機体での
            #   動作確認用に残している。
            if alt_sensor is not None:
                body_yaw = alt_sensor.get_body_yaw()
                for (t_ms, dvx, dvy, t_recv) in alt_sensor.drain_body_dv():
                    yaw_est.add_body_dv(t_ms, dvx, dvy, t_recv, body_yaw)

        if updated:
            display_loop_start = time.perf_counter()
            with plot_lock:
                f1 = plot_data.get("frame1")
                f2 = plot_data.get("frame2")
            if f1 is not None:
                disp1 = cv2.resize(f1, (DISP_W, DISP_H))
                if yaw_est is not None:
                    ye = yaw_est.current()
                    cv2.putText(disp1, yaw_est.status_line(), (10, DISP_H - 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (120, 255, 120) if ye.valid else (140, 140, 140), 2)
                cv2.imshow("Camera 1", disp1)
            if f2 is not None:
                cv2.imshow("Camera 2", cv2.resize(f2, (DISP_W, DISP_H)))

            now_ap = time.time()

            # ── ヨー推定を1ステップ進める ───────────────────
            # ★ ここでの update() は1ループ1回だけ。参照は current() を使う
            yaw_rad = None
            if yaw_est is not None:
                ye = yaw_est.update(now_ap)
                if ye.valid:
                    yaw_rad = math.radians(ye.yaw_deg)

            # ── ウェイポイントミッション ────────────────────
            #  ★ 順番が大事: 先にキー要求を処理し、そのあと update() する。
            #    同じフレームで「開始 -> 1回目の指令」まで進むので、
            #    キーを押してから機体が反応するまでの遅れが 1 フレームで済む。
            if mission is not None:
                #  ── SW_HOVER を GUIDED (up) に上げた瞬間に自動で開始 ──
                #  ★ 立ち上がりエッジだけを見る。GUIDED に入りっぱなしの
                #    間 (例: 着陸直後でまだ THR_CUT していない) に毎フレーム
                #    start() を呼ぶと、着陸完了の瞬間にまた離陸してしまう。
                #    スイッチを一度戻して上げ直す = 次の便を飛ばす意思表示。
                guided_now = bool(link.flag("guided"))
                if (MISSION_AUTOSTART and guided_now and not _prev_guided[0]
                        and mission.phase in (MissionPhase.IDLE,
                                              MissionPhase.DONE,
                                              MissionPhase.ABORT)):
                    print("[Mission] GUIDED 検出 (SW_HOVER 上) -> ミッションを最初から開始")
                    mission.start()
                _prev_guided[0] = guided_now

                req = shared.get("mission_request")
                if req is not None:
                    shared["mission_request"] = None
                    if req == "start":
                        mission.start()
                    elif req == "abort":
                        mission.abort("キー操作")
                    elif req.startswith("maneuver:"):
                        mission.request_maneuver(req.split(":", 1)[1])

                #  カメラが機体を捉えているか。
                #  ★ in_dummy (仮想円軌道へのフォールバック中) は「捉えていない」。
                #    ここを True にすると、架空の位置で位置ループを閉じることになる。
                pos_valid = bool(tracking_ok) and not in_dummy and P is not None

                #  ── ミッションに渡す機首方位 ──────────────────
                #  カメラのヨー推定が収束していればそれが最優先。
                #  収束していないときに何を使うかが MISSION_YAW_MODE。
                #    "fixed"  : 初期アラインメント値を使って飛ぶ
                #               (機首をフィールド奥へ向けて置いた前提)
                #    "camera" : 使わない = 水平移動しない (安全側)
                #  ★ ここを "camera" のままにすると、ヨー推定は機体が
                #    動いていないと収束しないため、永久に動き出さない。
                #  ★ 前提が崩れていないかの警告は mission.py 側が GUIDED に
                #    入るたびに出す (2026-09-14: プログラム起動中1回だけの
                #    警告だと、複数回リトライする間に手動操作で機首がズレて
                #    いくのを見逃した)。ここでは yaw_src を渡すだけでよい。
                m_yaw, m_yaw_valid = yaw_rad, (yaw_rad is not None)
                yaw_src = "camera"
                st_now = link.state() if (link is not None and link.ok) else {}
                armed_now = bool(st_now.get("armed", 0))
                if not armed_now:
                    _yaw_rebased[0] = False    # 機体はアーム時に g_yaw_est を 0 へ戻す
                dev_yaw = st_now.get("yaw")
                if m_yaw_valid and isinstance(dev_yaw, float):
                    # ★ カメラ推定が機体のジャイロヨーと大きく食い違ったら使わない。
                    #   2026-09-15 20:49: 推定が -86.3deg (実際は -2deg) に「収束」し、
                    #   CF_YAW_VALID で機体の g_yaw_est を -86 に書き換えた結果、
                    #   ヘディング保持が 60deg/s で機首を回し続け 0.6m 流れた。
                    #   機体のヨーは今日一日 ±5deg 以内で正しかったので、そちらを信じる。
                    base = 0.0 if _yaw_rebased[0] else YAW_INITIAL_ALIGN_DEG
                    disagree = (math.degrees(yaw_rad) - (base + dev_yaw) + 180.0) % 360.0 - 180.0
                    if abs(disagree) > YAW_CAMERA_MAX_DISAGREE_DEG:
                        if not _yaw_reject_warned[0]:
                            _yaw_reject_warned[0] = True
                            print(f"[Yaw] カメラ推定 {math.degrees(yaw_rad):+.1f}deg が機体ヨー "
                                  f"{base + dev_yaw:+.1f}deg と {disagree:+.0f}deg 食い違うため不採用")
                        m_yaw, m_yaw_valid = None, False
                    else:
                        _yaw_reject_warned[0] = False
                if m_yaw_valid and armed_now:
                    _yaw_rebased[0] = True     # この値が CF_YAW_VALID で機体へ送られる
                if not m_yaw_valid and MISSION_YAW_MODE == "fixed":
                    # ★ 決め打ちの 0deg ではなく機体のジャイロ積分ヨーを使う。
                    #   2026-09-15: アーム後に機首が -17.8deg 回っていたのに 0deg で
                    #   飛ばし、目標と違う向きへ進んだ。機体の g_yaw_est はアーム時
                    #   基準の相対値 (カメラで再基準された後は絶対値) なので、
                    #   アーム時の向き = YAW_INITIAL_ALIGN_DEG を足して絶対方位にする。
                    dev_yaw = st_now.get("yaw")
                    if isinstance(dev_yaw, float):
                        base = 0.0 if _yaw_rebased[0] else YAW_INITIAL_ALIGN_DEG
                        m_yaw = math.radians(base + dev_yaw)
                    else:
                        m_yaw = math.radians(YAW_INITIAL_ALIGN_DEG)
                    m_yaw_valid = True
                    yaw_src = "fixed"
                    if not _warned_fixed_yaw[0]:
                        _warned_fixed_yaw[0] = True
                        print(f"[Mission] ヨー推定が未収束のため、初期アラインメント "
                              f"{YAW_INITIAL_ALIGN_DEG:+.1f}deg を機首方位として使います。"
                              f"機首をフィールド奥(+y)へ向けたまま飛ばしてください "
                              f"(GUIDEDに入るたびに機体側の実測yawと一緒に再警告します)")

                mission.update(pos=P, yaw_rad=m_yaw,
                               pos_valid=pos_valid,
                               yaw_valid=m_yaw_valid,
                               yaw_src=yaw_src)

                # ── ミッションログ ──────────────────────────────
                #  PC の判断・カメラの見立て・機体の言い分を1行にまとめる。
                #  ★ 指令レートに合わせて書く (カメラの60Hzで書くと、同じ
                #    指令が12行並ぶだけで差分が読めなくなる)。
                if mission_log is not None and mission.n_tx() != _last_n_tx[0]:
                    _last_n_tx[0] = mission.n_tx()
                    tel = link.state()
                    tel["age"] = link.age()
                    cam = {"x": None if P is None else float(P[0]),
                           "y": None if P is None else float(P[1]),
                           "z": None if P is None else float(P[2]),
                           "valid": pos_valid, "in_dummy": in_dummy,
                           "residual": residual}
                    # 原点合わせと Diff_* の計算は mission.py 側が
                    # 位置補正のために持っている値をそのまま使う
                    # (ここで別に計算すると2つの基準がずれかねない)。
                    snap = mission.snapshot()
                    if snap["aligned"] and not _mission_aligned_logged[0]:
                        _mission_aligned_logged[0] = True
                        print("[Mission] カメラと機体フローの原点を合わせました "
                              "(以後 Diff_* が機体側のドリフト量)")
                    mission_log.write(
                        snap, cam, tel,
                        {"deg": None if m_yaw is None else math.degrees(m_yaw),
                         "valid": m_yaw_valid, "src": yaw_src},
                        event=mission.take_event())

            # ── ヨー推定値をground_receiver経由でドローンへ送信 ──
            if alt_sensor is not None:
                # ヨーは 0.5〜1Hz でよい。速くしても通信ジッタが姿勢に乗るだけ
                if yaw_est is not None and now_ap - last_yaw_send >= 1.0 / YAW_SEND_HZ:
                    last_yaw_send = now_ap
                    ye = yaw_est.current()
                    if ye.valid:
                        alt_sensor.send_yaw(ye.yaw_deg, True)
                    elif YAW_SEND_INITIAL_ALIGN:
                        # 未収束のあいだは初期アラインメント値を valid=0 で送る。
                        # 機体側は valid=0 を完全に無視すること
                        alt_sensor.send_yaw(YAW_INITIAL_ALIGN_DEG, False)

            # ── matplotlib系（レート制限） ──────────────────
            now = time.time()
            velocity_ms = 0.0
            graph_ms = 0.0
            if now - last_mpl_render > 1.0 / MPL_RENDER_HZ:
                last_mpl_render = now

                roll_deg, pitch_deg = 0.0, 0.0
                imu_available = False
                attitude_source = None
                # 通常の自動飛行では SERIAL_ENABLED=False。IM920で届く実機の
                # Roll/Pitch を優先してADIへ渡す。旧シリアル経路はフォールバック。
                tel = link.state() if link is not None else {}
                if link is not None and link.telemetry_ok() and "roll" in tel and "pitch" in tel:
                    roll_deg = float(tel["roll"])
                    pitch_deg = float(tel["pitch"])
                    imu_available = True
                    attitude_source = "IM920"
                elif alt_sensor is not None:
                    roll_deg, pitch_deg = accel_to_angles(alt_sensor.get_accel())
                    imu_available = True
                    attitude_source = "serial accel"

                velocity_start = time.perf_counter()
                vel_img = velocity_view.get_image(
                    P, roll_deg, pitch_deg, imu_available, attitude_source)
                cv2.imshow("Velocity", vel_img)
                velocity_ms = (time.perf_counter() - velocity_start) * 1000.0
                link_view.render_and_show(link)

                if O1 is not None:
                    target_alt = controller.get_target()
                    graph_start = time.perf_counter()
                    dashboard.render_and_show(P, current_z, target_alt)
                    graph_ms = (time.perf_counter() - graph_start) * 1000.0

            display_values = {
                "Display_ms": (time.perf_counter() - display_loop_start) * 1000.0,
                "Velocity_ms": velocity_ms,
                "Graph_ms": graph_ms,
            }
            display_perf_log.write("display", display_values)
            if time.time() - display_perf_time >= 2.0:
                if mission is not None and mission.phase is not MissionPhase.IDLE:
                    print(f"[Mission] {mission.status_line()}")
                print("[PERF] display "
                      f"total={display_values['Display_ms']:.1f}ms "
                      f"velocity={velocity_ms:.1f}ms "
                      f"graph={graph_ms:.1f}ms")
                display_perf_time = time.time()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            shared["quit"] = True
        elif key == ord('b'):
            shared["do_bg_reset"] = True
        elif key == ord('t'):
            def ask_target():
                try:
                    val = input("\n>>> 新しい目標高度を入力 (m): ")
                    controller.set_target(float(val))
                    print(f">>> 目標高度を {float(val):.1f}m に設定しました。")
                except ValueError:
                    print(">>> [エラー] 数値を入力してください。")
            threading.Thread(target=ask_target, daemon=True).start()

    # ── 終了処理 ──────────────────────────────────────────
    #  ★ リンクを閉じる前に必ずミッションを止める。close() が ABORT を
    #    送るので機体は自動着陸へ落ちるが、飛行中に Q を押した場合は
    #    それでも「降りてくるまで見ていること」。
    #  ★ 2026-09-16: 機体が既にディスアーム済み/地上なら待たない。以前は
    #    着陸後に Q を押しても「着陸完了」が来ない (ディスアーム済みなので
    #    landed が立たない) まま 20 秒待ち、その間 ACK を延々と表示していた。
    in_air = (link is not None and link.ok and link.n_data() > 0
              and link.flag("armed") and link.flag("airborne"))
    if (mission is not None and in_air
            and mission.phase not in (MissionPhase.IDLE, MissionPhase.DONE)):
        print("[Mission] 終了要求 → 自動着陸を指示します。着地を見届けてください")
        mission.abort("プログラム終了")
        for _ in range(200):                 # 最大 20 秒だけ着陸に付き合う
            mission.update()
            if mission.phase is MissionPhase.DONE or not link.flag("armed"):
                break
            time.sleep(0.1)
    if link is not None:
        link.close()
    if mission_log is not None:
        mission_log.close()
        print(f"[Mission] ミッションログを保存しました: {mission_log.path}")
    if ble_tap is not None:
        ble_tap.stop()

    if alt_sensor is not None:
        alt_sensor.stop()
    dashboard.close()
    velocity_view.close()
    link_view.close()
    display_perf_log.close()
    cam_thread.join(timeout=2.0)   # カメラを解放する前に追跡スレッドを止める
    cv2.destroyAllWindows()
    for _ in range(10):
        cv2.waitKey(1)
