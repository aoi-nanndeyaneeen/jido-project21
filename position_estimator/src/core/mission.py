"""
core/mission.py
「ボタン一つで、事前に決めた経路を飛んで、戻ってきて降りる」の本体。

==========================================================================
役割分担 (ここを間違えると必ず落ちる)
==========================================================================

    カメラ (tracker)  ->  フィールド座標の位置 P [m]
    yaw_estimator     ->  機首方位 psi [rad]
         |
         v
    このファイル: 位置誤差 -> 「機体座標の目標速度」 [m/s]        ← 外側ループ (5Hz)
         |  IM920 上り 5Hz
         v
    機体 (drone_s5 / PosHold): 目標速度 -> 速度PID -> 目標リーン角  ← 内側ループ (100Hz)
    機体 (Mixer/姿勢PID):      目標リーン角 -> モーター            ← 最内 (1000Hz)

★ PC は速度までしか指令しない。姿勢は絶対に送らない。
  IM920sL は半二重 19200bps・実効 15Hz・往復 100〜200ms。この遅れを姿勢
  ループに入れると位相余裕を食って発振する。詳しくは S5Cmd.h の先頭。

★ PC が黙れば機体は勝手に「その場ホールド -> 自動着陸」に落ちる。
  だから「指令を出し続けること」自体が生存確認になっている。
  逆に言うと、このクラスは異常を見つけたら **黙るのではなく LAND を出す**。
  黙って落ちても最終的には降りてくるが、降り始めが 4 秒遅れる。

==========================================================================
自動離陸・自動着陸をどこでやるか
==========================================================================
機体側 (AltHold::commandTarget) にやらせる。PC は「目標高度 1.0m」と
言うだけで、そこへ何 m/s で寄せるかは機体側のスルーレート。
理由: 離陸中にリンクが切れても、機体は最後の目標高度でホバーし続けられる。
PC が高度を刻んで送る方式だと、リンクが切れた瞬間に上昇が止まるのか
続くのかが「最後に届いたパケット次第」になり、挙動が再現しなくなる。
"""

import math
import time
from enum import Enum

from utils.config import FIELD_W, FIELD_D, MISSION_TAKEOFF_ALT_M
from core.s5_link import (REQ_ABORT, REQ_GUIDED, REQ_HOLD, REQ_LAND,
                          REQ_TAKEOFF, CF_ARMED_OK, CF_POS_VALID, CF_YAW_VALID,
                          CF_ALT_ABS)


class Phase(Enum):
    IDLE      = "IDLE"       # 待機。何も送らない
    ARMING    = "ARMING"     # 機体が GUIDED に入るのを待っている
    TAKEOFF   = "TAKEOFF"    # 離陸高度まで上昇中
    CRUISE    = "CRUISE"     # ウェイポイントへ移動中
    DWELL     = "DWELL"      # ウェイポイント上で静止保持中
    LAND      = "LAND"       # 自動着陸中
    DONE      = "DONE"       # 着陸完了
    ABORT     = "ABORT"      # 中断 (異常検知 / 手動)


def body_frame_errors(dx, dy, yaw_rad):
    """
    フィールド座標の誤差 (dx, dy) を機体座標の (前方, 右方) へ変換する。

        n = (sin psi, cos psi)   機首方向 (psi=0 が +y = フィールド奥)
        r = (ny, -nx)            機体右方向

    ★ core/autopilot.py と同じ定義。片方だけ直すと符号が食い違って
      位置ループが正帰還になる (= 機体が飛んでいく)。
    """
    nx, ny = math.sin(yaw_rad), math.cos(yaw_rad)
    rx, ry = ny, -nx
    return dx * nx + dy * ny, dx * rx + dy * ry


class WaypointMission:
    """
    ウェイポイント飛行のシーケンサ。

    使い方 (main_loop から):
        mission = WaypointMission(link, waypoints)
        mission.start()                      # ← ボタン1つ
        ...毎フレーム...
        mission.update(pos=P, yaw_rad=yaw, pos_valid=ok, yaw_valid=yv)

    waypoints は [(x, y, z), ...] のフィールド座標 [m]。
    最後まで行ったら離陸地点へ戻って着陸する (RTL)。
    """

    # ---- チューニング ------------------------------------------------
    # 位置誤差 [m] -> 目標速度 [m/s] の比例ゲイン。
    #  ★ 小さく始めること。これは「機体のPID」ではなく「PCが出す目標速度」。
    #    0.5 なら 1m ずれで 0.5m/s を指令する。
    KP_POS = 0.5
    # 指令してよい最大速度 [m/s]。機体側 GUIDED_MAX_VEL でもクランプされる。
    MAX_VEL = 0.4
    # 目標速度がこれ未満なら 0 として送る (機体側は不感帯で位置ホールドへ落ちる)
    MIN_VEL = 0.04

    # 到達判定
    #  ★ MISSION_SQUARE_M (WP 間隔) より十分小さくすること。同程度だと
    #    離陸した瞬間に全WPが到達済みになり、一度も移動しないまま帰投する。
    ARRIVE_R_M   = 0.20    # 水平 [m]
    ARRIVE_Z_M   = 0.20    # 高度 [m]
    DWELL_S      = 2.0     # 到達後ここで静止する時間 [s]

    # 離陸
    # ★ 2026-09-12: ここが config.MISSION_TAKEOFF_ALT_M と独立に 1.00 で
    #   ハードコードされていたため、機体側 ALT_TARGET_M(0.50m) とずれていた。
    #   WAYPOINT_BRINGUP.md §4-5 の警告どおり、ずれたままだと GUIDED を
    #   抜けた瞬間(スイッチ操作・リンク断・スティック介入)にその差だけ
    #   勝手に昇降する。config 側と必ず一致させること。
    TAKEOFF_ALT_M   = MISSION_TAKEOFF_ALT_M  # 離陸後にいったん保持する高度 [m]
    TAKEOFF_TOL_M   = 0.15  # この差まで来たら離陸完了
    TAKEOFF_TIMEOUT_S = 20.0

    # 指令レート [Hz]。XIAO が 5Hz に間引くので、それ以上送っても無駄。
    SEND_HZ = 5.0

    # ---- 安全 ---------------------------------------------------------
    # 自己位置をこの秒数見失ったら着陸する
    POS_LOST_LAND_S = 1.5
    # ミッション全体の上限時間 [s]
    MISSION_TIMEOUT_S = 180.0
    # ジオフェンス [m] (フィールド座標の絶対値)。超えたら即着陸。
    #  ★ フィールド寸法から作る。ベタ値にしておくと FIELD_PROFILE を
    #    small <-> large で切り替えたときに必ず食い違う
    #    (1.4m のフィールドに 1.5m のフェンス = 事実上フェンス無し)。
    FENCE_X = FIELD_W / 2.0 + 0.2
    FENCE_Y = FIELD_D / 2.0 + 0.2
    FENCE_Z = 2.5

    def __init__(self, link, waypoints, home=None, verbose=True):
        self.link = link
        self.waypoints = [tuple(float(v) for v in wp) for wp in waypoints]
        self.home = tuple(home) if home is not None else None
        self.verbose = verbose

        self.phase = Phase.IDLE
        self.wp_idx = 0
        self.reason = ""

        self._t_phase = 0.0
        self._t_start = 0.0
        self._t_send = 0.0
        self._t_pos_ok = 0.0
        self._returning = False     # 最後の WP を終えて home へ戻っている

    # ---------------------------------------------------------------- 制御
    def start(self):
        """ボタン1つで呼ぶ入口。ここから先は update() が全部やる。"""
        if self.phase not in (Phase.IDLE, Phase.DONE, Phase.ABORT):
            self._say(f"すでに実行中です ({self.phase.value})")
            return False
        if not self.link.ok:
            self._say("地上局リンクが無いので開始できません")
            return False
        now = time.time()
        self.phase = Phase.ARMING
        self.wp_idx = 0
        self.reason = ""
        self._returning = False
        self._t_phase = now
        self._t_start = now
        self._t_pos_ok = now
        self._say(f"ミッション開始: WP {len(self.waypoints)} 点 "
                  f"-> 離陸地点へ戻って自動着陸")
        return True

    def abort(self, reason="手動中断"):
        """即座に着陸させる。緊急停止ではない (それはプロポの仕事)。"""
        if self.phase in (Phase.IDLE, Phase.DONE):
            return
        self.reason = reason
        self._goto(Phase.LAND)
        self._say(f"中断 -> 自動着陸: {reason}")

    def _goto(self, phase):
        self.phase = phase
        self._t_phase = time.time()

    def _say(self, msg):
        if self.verbose:
            print(f"[Mission] {msg}")

    # ---------------------------------------------------------------- 本体
    def update(self, pos=None, yaw_rad=None, pos_valid=False, yaw_valid=False):
        """
        毎フレーム呼ぶ。実際に送信するのは SEND_HZ に間引いた回だけ。

        Args:
            pos       : フィールド座標 [x, y, z] (m)。None なら未検出
            yaw_rad   : 機首方位 [rad]。None なら未確定
            pos_valid : 自己位置が信用できるか (ダミー飛行中は False を渡すこと)
            yaw_valid : ヘディング推定が収束しているか
        """
        if self.phase in (Phase.IDLE, Phase.DONE, Phase.ABORT):
            return

        now = time.time()
        if pos_valid and pos is not None:
            self._t_pos_ok = now

        # ---- 1) 安全側の打ち切り判定 (フェーズより先に見る) -------------
        if self.phase is not Phase.LAND:
            why = self._safety_check(now, pos, pos_valid)
            if why:
                self.reason = why
                self._goto(Phase.LAND)
                self._say(f"異常検知 -> 自動着陸: {why}")

        # ---- 2) 送信レートに間引く ------------------------------------
        if now - self._t_send < 1.0 / self.SEND_HZ:
            return
        self._t_send = now

        flags = CF_ARMED_OK | CF_ALT_ABS
        if pos_valid:
            flags |= CF_POS_VALID
        if yaw_valid:
            flags |= CF_YAW_VALID

        st = self.link.state()
        h_agl = float(st.get("range_h", 0.0))    # 機体の測距による対地高度 [m]

        # ---- 3) フェーズごとの指令 ------------------------------------
        if self.phase is Phase.ARMING:
            # 機体が GUIDED に入るまで HOLD を送り続ける。
            #  入れない理由 (SW_AUTO が下 / フローが死んでいる / 未アーム) は
            #  機体側のシリアル画面に出る。ここでは待つだけ。
            self.link.send_command(REQ_HOLD, flags=flags)
            if self.link.flag("guided"):
                self._say("機体が GUIDED に入りました -> 離陸")
                self._goto(Phase.TAKEOFF)
            elif now - self._t_phase > 15.0:
                self._say("機体が GUIDED に入りません。"
                          "SW_AUTO / SW_HOVER が上か、アーム済みか、"
                          "スロットルが 15% 以上かを確認してください")
                self._t_phase = now      # 15 秒ごとに出し直す
            return

        if self.phase is Phase.TAKEOFF:
            self.link.send_command(REQ_TAKEOFF, alt_m=self.TAKEOFF_ALT_M,
                                   flags=flags)
            if abs(h_agl - self.TAKEOFF_ALT_M) < self.TAKEOFF_TOL_M:
                # 離陸地点を覚える (RTL の戻り先)。カメラが見えていれば実測。
                if self.home is None and pos is not None:
                    self.home = (float(pos[0]), float(pos[1]), self.TAKEOFF_ALT_M)
                    self._say(f"離陸地点を記録: "
                              f"({self.home[0]:+.2f}, {self.home[1]:+.2f})")
                self._say(f"離陸完了 (対地 {h_agl:.2f} m) -> WP0 へ")
                self._goto(Phase.CRUISE)
            elif now - self._t_phase > self.TAKEOFF_TIMEOUT_S:
                self.reason = "離陸がタイムアウト"
                self._goto(Phase.LAND)
                self._say(f"離陸が {self.TAKEOFF_TIMEOUT_S:.0f} 秒で完了しません "
                          f"(対地 {h_agl:.2f} m) -> 着陸")
            return

        if self.phase in (Phase.CRUISE, Phase.DWELL):
            target = self._target()
            # ヘディングが分からないと「前」がどっちか分からない。
            #  この状態で速度を出すと 90 度ずれた方向へ飛ぶので、必ず止める。
            if not yaw_valid or yaw_rad is None or not pos_valid or pos is None:
                self.link.send_command(REQ_HOLD, alt_m=target[2], flags=flags)
                return

            if self.phase is Phase.DWELL:
                self.link.send_command(REQ_HOLD, alt_m=target[2], flags=flags)
                if now - self._t_phase >= self.DWELL_S:
                    self._advance()
                return

            vx, vy = self._velocity_command(pos, target, yaw_rad)
            self.link.send_command(REQ_GUIDED, vx_mps=vx, vy_mps=vy,
                                   alt_m=target[2], flags=flags)

            dh = math.hypot(target[0] - float(pos[0]), target[1] - float(pos[1]))
            dz = abs(target[2] - h_agl)
            if dh < self.ARRIVE_R_M and dz < self.ARRIVE_Z_M:
                label = "HOME" if self._returning else f"WP{self.wp_idx}"
                self._say(f"{label} 到達 (残り {dh:.2f} m) -> {self.DWELL_S:.0f} 秒保持")
                self._goto(Phase.DWELL)
            return

        if self.phase is Phase.LAND:
            self.link.send_command(REQ_LAND, flags=flags)
            if self.link.flag("landed"):
                self._goto(Phase.DONE)
                self._say("着陸完了。THR_CUT でディスアームしてください"
                          + (f"  (理由: {self.reason})" if self.reason else ""))
            elif now - self._t_phase > 40.0:
                self._goto(Phase.DONE)
                self._say("着陸完了の通知が来ませんでした。"
                          "機体の状態を目視で確認し、手動で回収してください")
            return

    # ------------------------------------------------------------ 内部
    def _target(self):
        """今向かうべき点 (x, y, z)。全 WP を消化したら home へ。"""
        if self._returning or self.wp_idx >= len(self.waypoints):
            if self.home is not None:
                return (self.home[0], self.home[1], self.TAKEOFF_ALT_M)
            return (0.0, 0.0, self.TAKEOFF_ALT_M)
        return self.waypoints[self.wp_idx]

    def _advance(self):
        """保持が終わったので次へ進む。最後まで行ったら帰投 -> 着陸。"""
        if self._returning:
            self._say("帰投完了 -> 自動着陸")
            self._goto(Phase.LAND)
            return
        self.wp_idx += 1
        if self.wp_idx >= len(self.waypoints):
            self._returning = True
            self._say("全ウェイポイント消化 -> 離陸地点へ帰投")
        else:
            wp = self.waypoints[self.wp_idx]
            self._say(f"次は WP{self.wp_idx} "
                      f"({wp[0]:+.2f}, {wp[1]:+.2f}, {wp[2]:.2f})")
        self._goto(Phase.CRUISE)

    def _velocity_command(self, pos, target, yaw_rad):
        """
        位置誤差 -> 機体座標の目標速度 [m/s]。

        ★ ここは比例だけ。積分は入れない。
          外側ループに積分を入れると、リンクの遅れと機体側の速度I項と
          二重になって低周波の振動 (数秒周期で行ったり来たり) が出る。
          定常偏差は機体側の位置ホールドが吸収する。
        """
        dx = target[0] - float(pos[0])
        dy = target[1] - float(pos[1])
        fwd, right = body_frame_errors(dx, dy, yaw_rad)

        vx = self.KP_POS * fwd
        vy = self.KP_POS * right

        # 大きさでクランプする (成分ごとに切ると方向が曲がる)
        mag = math.hypot(vx, vy)
        if mag > self.MAX_VEL:
            k = self.MAX_VEL / mag
            vx, vy = vx * k, vy * k
        elif mag < self.MIN_VEL:
            vx, vy = 0.0, 0.0
        return vx, vy

    def _safety_check(self, now, pos, pos_valid):
        """着陸させるべき理由があれば文字列で返す。無ければ None。"""
        if not self.link.telemetry_ok(max_age_s=2.0):
            return "機体からのテレメトリが途絶"

        if now - self._t_start > self.MISSION_TIMEOUT_S:
            return f"ミッション時間 {self.MISSION_TIMEOUT_S:.0f} 秒を超過"

        # 離陸前はまだカメラが機体を捉えていなくてよい
        if self.phase in (Phase.CRUISE, Phase.DWELL):
            if now - self._t_pos_ok > self.POS_LOST_LAND_S:
                return f"自己位置を {self.POS_LOST_LAND_S:.1f} 秒見失った"

        if pos_valid and pos is not None:
            if abs(float(pos[0])) > self.FENCE_X or \
               abs(float(pos[1])) > self.FENCE_Y or \
               float(pos[2]) > self.FENCE_Z:
                return (f"ジオフェンス逸脱 "
                        f"({float(pos[0]):+.2f}, {float(pos[1]):+.2f}, "
                        f"{float(pos[2]):.2f})")

        # 機体が自分で GUIDED を降りた (パイロットがスイッチを戻した等)。
        #  この場合 機体は POSHOLD でその場に留まっているので、こちらも
        #  指令をやめて手動へ譲る。着陸させない (パイロットが操縦中かもしれない)。
        if self.phase in (Phase.CRUISE, Phase.DWELL, Phase.TAKEOFF):
            if self.link.n_data() > 0 and not self.link.flag("guided"):
                self.reason = "機体が GUIDED を抜けた (手動介入)"
                self._goto(Phase.ABORT)
                self._say("機体が GUIDED を抜けました。"
                          "指令を停止して手動へ譲ります")
                self.link.send_command(REQ_ABORT)
                return None
        return None

    # ------------------------------------------------------------ 表示
    def status_line(self):
        st = self.link.state()
        tgt = self._target() if self.phase not in (Phase.IDLE, Phase.DONE) else None
        s = f"{self.phase.value}"
        if self.phase in (Phase.CRUISE, Phase.DWELL):
            label = "HOME" if self._returning else f"WP{self.wp_idx}"
            s += f" -> {label}"
        if tgt is not None:
            s += f" ({tgt[0]:+.2f}, {tgt[1]:+.2f}, {tgt[2]:.2f})"
        s += f"  h={st.get('range_h', 0.0):.2f}m"
        s += f"  guided={int(bool(st.get('guided', 0)))}"
        s += f"  link={'OK' if self.link.telemetry_ok() else 'LOST'}"
        return s
