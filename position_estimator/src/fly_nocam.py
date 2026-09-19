"""
fly_nocam.py  -  カメラを使わずに、本番プログラム (core/program.py) だけを走らせる。

==========================================================================
これは何か
==========================================================================
main.py から「カメラによる自己位置推定」を丸ごと外したもの。地上局の仕事を
**「機体に離陸を指示して、プログラムの段階を順に送る」だけ** に絞ってある。

    このスクリプト ──BLE──> log_recorder (XIAO ESP32C3) ──UART──> 機体 (drone_s5)

やること:
  1. 地上局リンク (BLE) を開く。ログ (LOGnnnn.BIN) はリンクが自動で保存する
  2. core/program.py の Step 列を組み立てて表示する
  3. **アームの立ち上がり**で待機に入り、REQ_HOLD を送り続ける
     (これが無いと機体は GUIDED に入れない。quad/Guided.h の「初回エンゲージ」)
  4. パイロットが SW_HOVER を up にした瞬間に機体が GUIDED に入り、離陸する
  5. あとは段階を順に送る。締切が来たら着陸へ倒す

==========================================================================
カメラが無いと何が変わるか
==========================================================================
自己位置が出ないので `pos_valid=False` / `yaw_valid=False` を渡す。すると:

  * **GOTO (進入 / 定位置へ戻る / 帰投)** … core/mission.py の _phase_cruise が
    「ヘディングが分からない状態で速度を出すと 90 度ずれた方向へ飛ぶ」ため
    **REQ_HOLD だけを送ってその場でホバー**し、段階の制限時間が来たら次へ進む。
  * **定型機動 (水平旋回 / 上昇旋回 / 8の字)** … 機体単独で完結する
    (quad/Maneuver.h)。位置もヘディングも要らないので **そのまま回る**。
  * **離陸 / 着陸** … 高度ベースなので動く。着陸はその場に降りる。

つまり「移動して位置を決める」部分だけがホバーに置き換わり、
順序・タイマー・締切・機動・着陸はそのまま実行される。

==========================================================================
使い方
==========================================================================
    cd position_estimator/src
    python fly_nocam.py

  キー:
    G … ★離陸 (2回押し)。READY のときだけ効く
    S … 本番スケール ⇄ 1/10 スケール (飛行前のみ)
    C … 上昇旋回 ON / OFF            (飛行前のみ)
    M … 待機に入る (MISSION_AUTOSTART=False のとき用。通常は自動)
    X … 中断 → その場から着陸
    Q … 終了

  手順:
    1. 操縦者 THR_CUT 解除 → アーム    (ここで待機 ARMING に入る)
    2. 操縦者 スロットルを 15% 以上へ
    3. 操縦者 SW_HOVER を up (GUIDED)  → 画面 READY。**まだ上がらない**
    4. 「離陸」とコール
    5. PC係  [G] を 2 回               (ここで競技時計が 0 から走る)

★ main.py とは同時に起動できない (同じ機体へ BLE 接続するため)。
★ ble_monitor.py とも同時に起動できない (同上)。
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.ble_link import open_ground_link
from core.mission import MissionRunner, Phase as MissionPhase
from core.program import (build_competition_program, build_waypoint_program,
                          program_summary, check_geometry)
from core.keyreader import KeyReader
from core.s5_protocol import ALT_STATE_NAME
from utils import config
from utils.config import (COMP_ENABLED, GROUND_LINK_BACKEND, GROUND_LINK_PORT,
                          BLE_LOG_NAME, LOG_DIR, MISSION_AUTOSTART,
                          MISSION_TAKEOFF_ALT_M, YAW_INITIAL_ALIGN_DEG)

LOOP_HZ = 50.0


class NoCamFlight:
    CONFIRM_S = 3.0        # [G] の 2 回押しを受け付ける間隔 (ble_monitor.py と同じ)

    def __init__(self):
        self.link = None
        self.mission = None
        self.program = None
        self._prev_armed = False
        self._go_pending = None           # [G] 1 回目を押した時刻 (2 回押し確認)
        self._quit = False

    # ------------------------------------------------------------ 表示
    def _print_program(self):
        print()
        print("  ---- 飛行プログラム "
              f"({'本番' if COMP_ENABLED else '段階確認 (COMP_ENABLED=False)'}"
              f" / {'1/10 スケール' if config.COMP_SCALE < 0.999 else '本番スケール'}"
              f" / 上昇旋回 {'ON' if config.COMP_ENABLE_CLIMB else 'OFF'}) ----")
        for line in program_summary(self.program, config):
            print(line)
        ok, lines = check_geometry(self.program, config, YAW_INITIAL_ALIGN_DEG)
        print()
        for line in lines:
            print(line)
        if not ok:
            print("  ★★ このまま飛ばすと機動が崩れます。設定を直してください ★★")
        print()

    def _build(self):
        self.program = (build_competition_program(config) if COMP_ENABLED
                        else build_waypoint_program(config))

    def _rebuild(self, what):
        self._build()
        if self.mission is not None and not self.mission.set_program(self.program):
            print(f"[KEY] {what}: 飛行中なので変更しませんでした")
            return
        self._print_program()

    # ------------------------------------------------------------ キー
    def _on_key(self, ch):
        ch = (ch or "").lower()
        if ch == "q":
            print("[KEY] Q → 終了")
            self._quit = True
        elif ch == "s":
            config.COMP_SCALE = 0.1 if config.COMP_SCALE > 0.999 else 1.0
            name = "本番スケール" if config.COMP_SCALE > 0.999 else "1/10 スケール (通し練習)"
            print(f"[KEY] S → {name}")
            self._rebuild("スケール切替")
        elif ch == "c":
            config.COMP_ENABLE_CLIMB = not config.COMP_ENABLE_CLIMB
            print("[KEY] C → 上昇旋回 "
                  + ("ON" if config.COMP_ENABLE_CLIMB else "OFF (飛ばない)"))
            self._rebuild("上昇旋回 ON/OFF")
        elif ch == "m":
            print("[KEY] M → 待機開始")
            self.mission.start()
        elif ch == "g":
            # ★ 離陸は押し間違いで始まってはいけないので 2 回押し
            #   (CONFIRM_S 秒以内にもう一度)。
            now = time.time()
            if self._go_pending is not None and now - self._go_pending <= self.CONFIRM_S:
                self._go_pending = None
                print("[KEY] G → ★離陸")
                self.mission.go()
            else:
                self._go_pending = now
                print(f"[KEY] G → 離陸しますか? {self.CONFIRM_S:.0f}秒以内にもう一度 [G]")
        elif ch == "x":
            print("[KEY] X → 中断 (その場から着陸)")
            self.mission.abort("キー操作")

    # ------------------------------------------------------------ 起動
    def setup(self):
        self._build()
        print("=" * 60)
        print("  fly_nocam.py  -  カメラ無し。地上局はプログラムの送信だけを行う")
        print("=" * 60)
        self._print_program()

        print(f"[INIT] 地上局リンク = {GROUND_LINK_BACKEND} へ接続中...")
        self.link = open_ground_link(GROUND_LINK_BACKEND, port=GROUND_LINK_PORT,
                                     ble_name=BLE_LOG_NAME, log_dir=LOG_DIR)
        if not self.link.ok:
            print("  [NG] 地上局に接続できません。機体とロガーの電源を確認してください")
            return False
        print("  [OK] 接続しました")

        # ★ use_camera=False: 自己位置が最初から無いので、
        #   「位置を見失った -> 着陸」の安全判定を切り、GOTO は
        #   開ループ進入 (COMP_ENTRY_DR_S) かその場ホバーにする。
        self.mission = MissionRunner(self.link, self.program, use_camera=False)
        self.mission.TAKEOFF_ALT_M = MISSION_TAKEOFF_ALT_M
        if MISSION_AUTOSTART:
            print("  [OK] アームすると待機に入ります。SW_HOVER を GUIDED にすると READY、"
                  "★離陸は [G] 2回")
        else:
            print("  [OK] [M] で待機に入ります (離陸は READY になってから [G] 2回)")
        print()
        print("  キー:  ★G=離陸(2回押し)   S=スケール切替   C=上昇旋回ON/OFF   "
              "X=中断して着陸   Q=終了")
        print("  ※ カメラを使わないので GOTO はその場ホバー、機動は機体単独で回ります")
        print()
        return True

    # ------------------------------------------------------------ ループ
    def _status_line(self):
        m, st = self.mission, self.link.state()
        s = m.step
        t = m.elapsed()
        mode = st.get("mode")
        # ★ 2026-09-19 LOG0101: ここは st.get("h_agl") を読んでいたが、リンクの state に
        #   そんなキーは無く (h_agl は core/mission.py の Telemetry のフィールド名)、
        #   高度が常に None 表示だった。機体が離陸を拒否して 0m のまま 17.8 秒座っていても
        #   画面からは分からず、原因の切り分けに丸一便かかった。正しいキーは range_h。
        h = st.get("range_h")
        # ★ 同上: alt_state を出す。離陸しないときの理由がここに出ている
        #   (STANDBY = スロットルが ALT_ENABLE_THR 15% 未満 / NO_RANGE = 測距を掴めていない)。
        alt_state = st.get("alt_state")
        alt_s = (ALT_STATE_NAME.get(int(alt_state), str(alt_state))
                 if alt_state is not None else "--")
        line = (f"\r  T+{t:6.1f}s  {m.phase.value:<8} "
                f"[{m.step_idx + 1}/{len(m.program)}] {(s.label if s else '-'):<16} "
                f"mode={mode} h={h if h is None else round(float(h), 2)} "
                f"alt={alt_s:<12} "
                f"armed={int(bool(self.link.flag('armed')))} "
                f"guided={int(bool(self.link.flag('guided')))} "
                f"tx={m.n_tx()}   ")
        return line

    def run(self):
        dt = 1.0 / LOOP_HZ
        last_print = 0.0
        # KeyReader は端末モードを戻すためにコンテキストマネージャで使う
        with KeyReader() as keys:
          try:
            while not self._quit:
                ch = keys.get()
                if ch:
                    self._on_key(ch)

                # アームの立ち上がりで待機に入る (main.py と同じ条件)
                armed_now = bool(self.link.flag("armed"))
                if (MISSION_AUTOSTART and armed_now and not self._prev_armed
                        and self.mission.phase in (MissionPhase.IDLE,
                                                   MissionPhase.DONE,
                                                   MissionPhase.ABORT)):
                    print("\n[Mission] アーム検出 -> 待機開始 "
                          "(SW_HOVER を GUIDED にすると待機。離陸は [G] 2回)")
                    self.mission.start()
                self._prev_armed = armed_now

                # ★ カメラが無いので位置もヘディングも無効を渡す。
                #   mission.py はこれを見て GOTO で速度を出さない (その場ホバー)。
                self.mission.update(pos=None, yaw_rad=None,
                                    pos_valid=False, yaw_valid=False,
                                    yaw_src="fixed")

                ev = self.mission.take_event()
                if ev:
                    print(f"\n[Mission] {ev}")

                now = time.time()
                if now - last_print > 0.2:
                    last_print = now
                    sys.stdout.write(self._status_line())
                    sys.stdout.flush()
                time.sleep(dt)
          except KeyboardInterrupt:
            print("\n[INIT] Ctrl-C")
          finally:
            print()
            try:
                self.mission.abort("終了")
            except Exception:
                pass
            try:
                self.link.close()
            except Exception:
                pass
            print("[INIT] 終了しました")


def main():
    app = NoCamFlight()
    if not app.setup():
        return 1
    app.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
