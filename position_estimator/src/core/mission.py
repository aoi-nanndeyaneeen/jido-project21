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
自己位置の補正 (2026-09-14〜)
==========================================================================
機体はオプティカルフローの積分だけで自分の水平位置 (pos_n/pos_e) を
持っている。床の模様が薄いフィールドではこれが流れる。カメラは絶対位置
(cm オーダー) を持っているので、POS_CORR_PERIOD_S 秒に1回だけ
「本当はここにいるはず」を機体へ送って pos_n/pos_e を上書きする。

★ これは姿勢・速度のクローズドループとは別物。IM920sL の往復遅延
  (100〜200ms) を含んだ位置を毎ループ使うと発振するため、数秒に1回の
  「たまに書き換えるだけ」に留める。効くのは静止保持中だけで、GUIDED
  巡航中は機体側の hold が pos に毎ループ追従するため実質無効になる
  (flight_controller/include/quad/PosHold.h の correctPosition() 参照)。
  巡航中に効かないのは意図した挙動: 行き先はカメラ由来の速度指令
  そのもので決まっており、位置を書き換えても無駄にレイテンシを
  持ち込むだけだから。

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

from utils.config import (MISSION_TAKEOFF_ALT_M, YAW_INITIAL_ALIGN_DEG,
                          MISSION_FENCE_X, MISSION_FENCE_Y, MISSION_FENCE_Z,
                          MISSION_FENCE_GRACE_S, MISSION_YAW_PROBE_VEL,
                          POS_CORR_ENABLED, POS_CORR_PERIOD_S,
                          POS_CORR_MAX_STEP_M,
                          POS_CORR_TRACK_TOL_M, POS_CORR_CONFIRM_S)
from core.s5_link import (REQ_ABORT, REQ_GUIDED, REQ_HOLD, REQ_LAND,
                          REQ_TAKEOFF, REQ_NAME,
                          CF_ARMED_OK, CF_POS_VALID, CF_YAW_VALID,
                          CF_ALT_ABS, CF_POS_CORR)


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
    # ★ 2026-09-14: 「1秒保持できたら成功」という定義に合わせて 2.0 -> 1.0 に短縮。
    #   問題が出るようなら伸ばす (WAYPOINT_BRINGUP.md 参照)。
    DWELL_S      = 1.0     # 到達後ここで静止する時間 [s] = ミッション成功の定義

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
    # 飛び始めてからの上限時間 [s]
    #  ★ start() からではなく「離陸フェーズに入ってから」数える。
    #    自動開始だと地上での待ち時間が読めず、start() 起点だと
    #    離陸する前にタイムアウトして着陸指令が出てしまう。
    MISSION_TIMEOUT_S = 180.0
    # ジオフェンス [m]。config で 3Dゲートと一緒に定義している
    # (両者の関係が壊れると誤検知1発でミッションが死ぬ。config 側の注記参照)。
    FENCE_X = MISSION_FENCE_X
    FENCE_Y = MISSION_FENCE_Y
    FENCE_Z = MISSION_FENCE_Z
    FENCE_GRACE_S = MISSION_FENCE_GRACE_S

    def __init__(self, link, waypoints, home=None, verbose=True):
        self.link = link
        self.waypoints = [tuple(float(v) for v in wp) for wp in waypoints]
        self.home = tuple(home) if home is not None else None
        self.verbose = verbose

        self.phase = Phase.IDLE
        self.wp_idx = 0
        self.reason = ""
        # 実際に巡回する経路。start() のたびに組み直す (TAKEOFF_ALT_M が
        # コンストラクタの後に上書きされることがあるため)。index 0 は必ず
        # フィールド中心 (0,0)。理由は start() のコメント参照。
        self._path = [(0.0, 0.0, self.TAKEOFF_ALT_M)] + self.waypoints

        self._t_phase = 0.0
        self._t_start = 0.0
        self._t_send = 0.0
        self._t_pos_ok = 0.0
        self._returning = False     # 最後の WP を終えて home へ戻っている

        # ---- 安全判定の状態 --------------------------------------------
        self._t_fly = None      # 離陸フェーズに入った時刻 (タイムアウトの起点)
        self._t_fence = None    # ジオフェンスの外に出始めた時刻
        self._airborne = False  # 機体が一度でも「浮いた」と言ったか

        # ---- 自己位置補正の状態 ------------------------------------------
        # align: (フィールドx - フロー e, フィールドy - フロー n)。
        #   機体のフロー原点はフィールド原点ではないので、両方が同時に
        #   信用できる最初の瞬間に1回だけ取る (機首がフィールド奥を向いて
        #   いる前提 = YAW_INITIAL_ALIGN_DEG=0 と同じ前提。core/mission.py
        #   の body_frame_errors と揃えてある: n<->y(奥), e<->x(右))。
        self._align = None
        self._t_corr = 0.0
        self._pending_corr = None   # 次の _send() 1回にだけ乗せる (n_m, e_m)
        # ログ用: (diff_x, diff_y, diff_norm, drone_x, drone_y) or None
        self._diff = None
        # 持続性チェック: 「直近の推定から大きく変わらない値が一定時間
        # 続いたか」を見る。誤検知1点だけを信じて補正しないための蓋。
        self._corr_ref = None       # (diff_x, diff_y) 追跡中の基準値
        self._t_corr_start = 0.0    # その基準値が始まった時刻

        # ヨーの出所。"camera"(実測収束済み) か "fixed"(決め打ち)。
        # status_line() と GUIDED 進入時の警告で使う。
        self._yaw_src = "fixed"

        # ---- ログ用 ----------------------------------------------------
        # ★ 「何を送ったか」は送った本人しか知らない。update() の中で
        #   分岐した結果を外から再現しようとすると必ずズレるので、
        #   送信と同時にここへ控える。
        self._tx = {"req": "", "vx": 0.0, "vy": 0.0, "alt": 0.0, "flags": 0,
                    "phase": Phase.IDLE.value}
        self._n_tx = 0              # 送信回数。ログはこれが増えた時だけ書く
        self._dist_h = None
        # _say() したメッセージ。ログが1回読み出したら消える (イベント列)。
        self._pending_event = ""

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
        # ★ 2026-09-14: 経路の先頭に必ずフィールド中心 (0,0) を挟む。
        #   人が PosHold で任意の場所へ飛ばしてから GUIDED に切り替えても、
        #   機体はまず中心へ戻ってから設定した経路 (self.waypoints) を
        #   巡回する。「今どこにいるか分からないまま経路の1点目へ直行する」
        #   より安全 (中心なら三方の壁から距離が最大になる)。
        self._path = [(0.0, 0.0, self.TAKEOFF_ALT_M)] + self.waypoints
        self._returning = False
        self._t_phase = now
        self._t_start = now
        self._t_pos_ok = now
        self._t_fly = None
        self._t_fence = None
        self._airborne = False
        # 機体側の pos_n/pos_e はアーム/リセットのたびに0へ戻る
        # (PositionHold::reset())。原点合わせもフライトごとに取り直す。
        self._align = None
        self._t_corr = 0.0
        self._pending_corr = None
        self._diff = None
        self._corr_ref = None
        self._t_corr_start = 0.0
        self._say(f"待機開始: 機体が GUIDED に入ったら自動で離陸します "
                  f"(WP {len(self.waypoints)} 点 -> 離陸地点へ戻って自動着陸)")
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
        self._pending_event = msg
        if self.verbose:
            print(f"[Mission] {msg}")

    def _send(self, req, vx_mps=0.0, vy_mps=0.0, alt_m=0.0, flags=0,
              yaw_rad=None):
        """送信と記録を必ずセットで行う。link.send_command() を直接呼ばないこと。"""
        # 保留中の位置補正があれば、この送信1回にだけ乗せて消費する。
        # どのフェーズの _send() 呼び出しに乗るかは問わない
        # (補正はどのみち GUIDED 巡航中は効かないので、req が何でもよい)。
        corr = self._pending_corr
        self._pending_corr = None
        corr_n = corr_e = None
        if corr is not None:
            flags |= CF_POS_CORR
            corr_n, corr_e = corr

        # ★ phase は「送った時点」のものを控える。update() は送信後に
        #   フェーズを進めることがあるので、あとから self.phase を読むと
        #   「HOLD を送った行が TAKEOFF になっている」というズレが出る。
        self._tx = {"req": REQ_NAME.get(req, str(req)),
                    "vx": vx_mps, "vy": vy_mps, "alt": alt_m, "flags": flags,
                    "phase": self.phase.value, "corr": corr}
        self._n_tx += 1
        self.link.send_command(req, vx_mps=vx_mps, vy_mps=vy_mps,
                               alt_m=alt_m, flags=flags,
                               corr_n_m=corr_n, corr_e_m=corr_e,
                               yaw_abs_deg=(math.degrees(yaw_rad)
                                            if yaw_rad is not None and (flags & CF_YAW_VALID)
                                            else None))

    def n_tx(self):
        return self._n_tx

    def take_event(self):
        """未読のイベント文字列を1回だけ返す。無ければ空文字。"""
        ev, self._pending_event = self._pending_event, ""
        return ev

    def snapshot(self):
        """ログ1行分のミッション内部状態。"""
        tgt = self._target() if self.phase not in (Phase.IDLE, Phase.DONE) else None
        return {"wp_idx": self.wp_idx,
                "returning": self._returning, "tgt": tgt,
                "dist_h": self._dist_h,
                # phase = 送信時のフェーズ / phase_next = 送信後の今のフェーズ。
                # 2つが違う行がフェーズ遷移そのもの。
                "phase_next": self.phase.value,
                "aligned": self._align is not None,
                "diff": self._diff,   # (diff_x, diff_y, diff_norm, drone_x, drone_y) or None
                **self._tx}

    # ---------------------------------------------------------------- 本体
    def update(self, pos=None, yaw_rad=None, pos_valid=False, yaw_valid=False,
              yaw_src="fixed"):
        """
        毎フレーム呼ぶ。実際に送信するのは SEND_HZ に間引いた回だけ。

        Args:
            pos       : フィールド座標 [x, y, z] (m)。None なら未検出
            yaw_rad   : 機首方位 [rad]。None なら未確定
            pos_valid : 自己位置が信用できるか (ダミー飛行中は False を渡すこと)
            yaw_valid : ヘディング推定が収束しているか
            yaw_src   : "camera" (実測収束済み) か "fixed" (初期アラインメント
                       決め打ち)。"fixed" の間は機首が本当にフィールド奥を
                       向いているかを機体側では確認できない
                       (2026-09-14: 複数回リトライする間に手動操作で機首が
                       30度以上ズレたまま気づかなかった事故を参照)。
        """
        if self.phase in (Phase.IDLE, Phase.DONE, Phase.ABORT):
            return

        now = time.time()
        self._yaw_src = yaw_src
        if pos_valid and pos is not None:
            self._t_pos_ok = now

        # 機体が一度でも「浮いた」と言ったら覚えておく。ジオフェンスは
        # これが立ってからしか効かせない (着陸中に落ちても戻さない。
        # LAND フェーズではそもそも安全判定を通らない)。
        if self.link.flag("airborne"):
            self._airborne = True

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
        # ★ CF_YAW_VALID は「カメラで実測した絶対ヨー」のときだけ立てる。
        #   機体はこのフラグ付きの値で g_yaw_est を上書きするので、"fixed"
        #   (前提値) で立てると機体が正しく知っている自分の回転を消してしまう
        #   (2026-09-15: -17.8deg が GUIDED 最初の指令で +1.2deg に飛んだ)。
        if yaw_valid and yaw_src == "camera":
            flags |= CF_YAW_VALID

        st = self.link.state()
        h_agl = float(st.get("range_h", 0.0))    # 機体の測距による対地高度 [m]

        self._maybe_correct_position(now, pos, pos_valid, st)

        # 今の目標までの水平距離。到達判定とログの両方がこれを見る
        # (別々に計算すると、ログと判定が食い違って原因追跡が狂う)。
        tgt_now = self._target()
        self._dist_h = (math.hypot(tgt_now[0] - float(pos[0]),
                                   tgt_now[1] - float(pos[1]))
                        if pos is not None else None)

        # ---- 3) フェーズごとの指令 ------------------------------------
        if self.phase is Phase.ARMING:
            # 機体が GUIDED に入るまで HOLD を送り続ける。
            #  入れない理由 (SW_AUTO が下 / フローが死んでいる / 未アーム) は
            #  機体側のシリアル画面に出る。ここでは待つだけ。
            self._send(REQ_HOLD, flags=flags, yaw_rad=yaw_rad)
            if self.link.flag("guided"):
                dev_yaw = st.get("yaw")
                if self._yaw_src != "camera":
                    # ★ リトライのたびに出す (プログラム起動中1回だけだと、
                    #   2026-09-14 のように手動操作の合間に機首がズレていく
                    #   のを見逃す)。機体自身の yaw (アーム基準の相対方位)
                    #   も一緒に出す。ここが 0 から離れていたら、機首は
                    #   もうフィールド奥を向いていない。
                    dev_str = f"{dev_yaw:+.1f}deg" if dev_yaw is not None else "不明"
                    self._say(f"機体が GUIDED に入りました -> 離陸 "
                              f"(機首方位は{YAW_INITIAL_ALIGN_DEG:+.1f}deg決め打ち。"
                              f"機体自身のyaw={dev_str}。0から離れていたら"
                              f"機首はもうフィールド奥を向いていません)")
                else:
                    self._say("機体が GUIDED に入りました -> 離陸")
                self._t_fly = now        # 飛行時間はここから数える
                self._goto(Phase.TAKEOFF)
            elif self.link.flag("armed") and now - self._t_phase > 15.0:
                # ★ 未アームのうちは出さない。自動開始だと離陸まで数分
                #   待つことがあり、その間ずっと鳴っていると本当の
                #   メッセージが流れて見えなくなる。
                self._say("アーム済みですが GUIDED に入りません。"
                          "SW_HOVER が上か、フロー/測距が生きているか、"
                          "スロットルが 15% 以上かを確認してください")
                self._t_phase = now      # 15 秒ごとに出し直す
            return

        if self.phase is Phase.TAKEOFF:
            self._send(REQ_TAKEOFF, alt_m=self.TAKEOFF_ALT_M, flags=flags,
                       yaw_rad=yaw_rad)
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
                self._send(REQ_HOLD, alt_m=target[2], flags=flags, yaw_rad=yaw_rad)
                return

            if self.phase is Phase.DWELL:
                self._send(REQ_HOLD, alt_m=target[2], flags=flags, yaw_rad=yaw_rad)
                if now - self._t_phase >= self.DWELL_S:
                    self._advance()
                return

            vx, vy = self._velocity_command(pos, target, yaw_rad)
            self._send(REQ_GUIDED, vx_mps=vx, vy_mps=vy,
                       alt_m=target[2], flags=flags, yaw_rad=yaw_rad)

            dh = self._dist_h
            dz = abs(target[2] - h_agl)
            if dh < self.ARRIVE_R_M and dz < self.ARRIVE_Z_M:
                self._say(f"{self._label()} 到達 (残り {dh:.2f} m) -> "
                          f"{self.DWELL_S:.0f} 秒保持")
                self._goto(Phase.DWELL)
            return

        if self.phase is Phase.LAND:
            self._send(REQ_LAND, flags=flags, yaw_rad=yaw_rad)
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
    def _label(self):
        """今の目標を表す短い文字列 ("CENTER" / "WP0" / "HOME")。ログと音声(print)で共有する。"""
        if self._returning:
            return "HOME"
        if self.wp_idx == 0:
            return "CENTER"      # self._path[0] = (0,0) 固定
        return f"WP{self.wp_idx - 1}"   # self.waypoints[0] が WP0

    def _target(self):
        """今向かうべき点 (x, y, z)。全 WP (中心含む) を消化したら home へ。"""
        if self._returning or self.wp_idx >= len(self._path):
            if self.home is not None:
                return (self.home[0], self.home[1], self.TAKEOFF_ALT_M)
            return (0.0, 0.0, self.TAKEOFF_ALT_M)
        return self._path[self.wp_idx]

    def _advance(self):
        """保持が終わったので次へ進む。最後まで行ったら帰投 -> 着陸。"""
        if self._returning:
            self._say("帰投完了 -> 自動着陸")
            self._goto(Phase.LAND)
            return
        self.wp_idx += 1
        if self.wp_idx >= len(self._path):
            self._returning = True
            self._say("全ウェイポイント消化 -> 離陸地点へ帰投")
        else:
            wp = self._path[self.wp_idx]
            self._say(f"次は {self._label()} "
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

        # ★ カメラのヨー推定 (YawEstimator) がまだ収束していない
        #   (yaw_src != "camera") あいだは、YAW_INITIAL_ALIGN_DEG の
        #   決め打ちがズレていた場合に備えて速度を絞る。ここで稼いだ
        #   低速の移動そのものが YawEstimator の収束用Δvにもなるので、
        #   ただ待つだけの安全策ではなく収束を進める側にも効く。
        max_vel = self.MAX_VEL if self._yaw_src == "camera" else MISSION_YAW_PROBE_VEL

        # 大きさでクランプする (成分ごとに切ると方向が曲がる)
        mag = math.hypot(vx, vy)
        if mag > max_vel:
            k = max_vel / mag
            vx, vy = vx * k, vy * k
        elif mag < self.MIN_VEL:
            vx, vy = 0.0, 0.0
        return vx, vy

    def _maybe_correct_position(self, now, pos, pos_valid, st):
        """
        数秒に1回、カメラの絶対位置で機体のフロー積分位置を上書きする。

        ★ 効くのは静止保持中だけ。GUIDED 巡航中は PositionHold 側で
          実質無効化される (correctPosition() のコメント参照)。ここでは
          「いつ・どれだけ送るか」だけを決め、実際に効くかは機体任せ。

        ★ 着陸中 (Phase.LAND) は送らない。降下中に横移動を誘発したくない。

        ★ 持続性チェック: 2026-09-14 の飛行で、align直後 0.02m だったズレが
          次のチェック (2秒後) には 1.22m に見えたことがあった。窓の反射
          などゲートを1回だけすり抜けた誤検知1点が原因。MAX_STEP_M で
          動く量はクランプしていたが、「信じてよいズレか」自体は見て
          いなかった。ここでは直近の推定 (self._corr_ref) から
          POS_CORR_TRACK_TOL_M 以上外れたら「まだ様子見」として基準を
          そこから取り直し、同じ値が POS_CORR_CONFIRM_S 秒続いて初めて
          信用する。誤検知1フレームでは持続時間が足りず弾かれる。
        """
        if not POS_CORR_ENABLED or self.phase is Phase.LAND:
            return
        if not pos_valid or pos is None:
            return
        fh_n, fh_e = st.get("fh_posn"), st.get("fh_pose")
        if fh_n is None or fh_e is None or not st.get("flow_ok"):
            return

        cam_x, cam_y = float(pos[0]), float(pos[1])

        if self._align is None:
            # 両方が同時に信用できる最初の瞬間。定義よりズレは0なので
            # 送る意味がない。原点だけ覚えて次回以降に備える。
            self._align = (cam_x - fh_e, cam_y - fh_n)
            self._diff = (0.0, 0.0, 0.0, cam_x, cam_y)
            self._corr_ref = None
            return

        drone_x = fh_e + self._align[0]
        drone_y = fh_n + self._align[1]
        diff_x = cam_x - drone_x
        diff_y = cam_y - drone_y
        diff_norm = math.hypot(diff_x, diff_y)
        self._diff = (diff_x, diff_y, diff_norm, drone_x, drone_y)

        # ---- 持続性チェック --------------------------------------------
        if (self._corr_ref is None or
                math.hypot(diff_x - self._corr_ref[0],
                          diff_y - self._corr_ref[1]) > POS_CORR_TRACK_TOL_M):
            # 新しい系列の始まり (初回、または直前の推定から大きく外れた)。
            # まだ確信できないので、この基準で数える所からやり直す。
            self._corr_ref = (diff_x, diff_y)
            self._t_corr_start = now
            return
        if now - self._t_corr_start < POS_CORR_CONFIRM_S:
            return   # 一致はしているが確認時間にまだ達していない

        # ---- ここまで来たら「信用してよいズレ」。送信レートを間引く ----
        if now - self._t_corr < POS_CORR_PERIOD_S:
            return
        self._t_corr = now
        if diff_norm < 0.02:      # ノイズだけなら送らない (無線帯域の節約)
            return

        # 1回の補正量に上限を設ける。機体側クランプ (FLOW_POS_VEL_LIM) は
        # 「反応の速さ」を絞るだけなので、飛んでくる値そのものを絞る蓋を
        # PC側にも置いておく。
        step = min(diff_norm, POS_CORR_MAX_STEP_M) / diff_norm
        corr_n = fh_n + diff_y * step
        corr_e = fh_e + diff_x * step
        self._pending_corr = (corr_n, corr_e)
        clipped = "" if step >= 0.999 else f" (上限{POS_CORR_MAX_STEP_M:.1f}mで制限)"
        self._say(f"自己位置を補正: ずれ{diff_norm:.2f}m{clipped}")

    def _safety_check(self, now, pos, pos_valid):
        """
        着陸させるべき理由があれば文字列で返す。無ければ None。

        ★ ARMING (まだ飛んでいない) では一切判定しない。
          地上に置いてある機体を「着陸させる」ことに意味は無く、
          むしろ 2026-09-14 のように、離陸前の誤検知1発でミッションが
          再起不能 (LAND -> DONE) になる事故を生む。自動開始にすると
          地上での待ち時間が数分になるので、時間切れ判定も同じ理由で外す。
          地上にいるあいだの安全はパイロットのプロポが受け持つ。
        """
        if self.phase is Phase.ARMING:
            return None

        if not self.link.telemetry_ok(max_age_s=2.0):
            return "機体からのテレメトリが途絶"

        if self._t_fly is not None and now - self._t_fly > self.MISSION_TIMEOUT_S:
            return f"飛行時間 {self.MISSION_TIMEOUT_S:.0f} 秒を超過"

        # 離陸前はまだカメラが機体を捉えていなくてよい
        if self.phase in (Phase.CRUISE, Phase.DWELL):
            if now - self._t_pos_ok > self.POS_LOST_LAND_S:
                return f"自己位置を {self.POS_LOST_LAND_S:.1f} 秒見失った"

        # ---- ジオフェンス ------------------------------------------------
        #  ★ 「浮いてから」「逸脱が続いたら」の2条件。単発の誤検知では落とさない。
        #    カメラは窓や反射を掴むことがあり、1フレームだけフィールド外へ
        #    跳ぶことが実際にある。そのたびに着陸していては飛べない。
        if self._airborne and pos_valid and pos is not None:
            outside = (abs(float(pos[0])) > self.FENCE_X or
                       abs(float(pos[1])) > self.FENCE_Y or
                       float(pos[2]) > self.FENCE_Z)
            if not outside:
                self._t_fence = None
            else:
                if self._t_fence is None:
                    self._t_fence = now
                elif now - self._t_fence >= self.FENCE_GRACE_S:
                    return (f"ジオフェンス逸脱が {self.FENCE_GRACE_S:.1f} 秒継続 "
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
                self._send(REQ_ABORT)
                return None
        return None

    # ------------------------------------------------------------ 表示
    MODE_NAME = {0: "RATE", 1: "ANGLE", 2: "GUIDED", 3: "POSHOLD", 4: "ALTHOLD"}

    def status_line(self):
        """
        「今どういう状態か」を1行で。2秒ごとにコンソールへ出る想定。

        ★ 機体自身の yaw (アーム基準の相対方位) を必ず出す。yaw_src が
          "fixed" のときはこれが唯一の手掛かり。0 から離れているほど、
          PCが仮定している「機首はフィールド奥」という前提が崩れている
          (2026-09-14 に複数回リトライの間に機首が30度以上ズレて
          フィールド外まで飛んだ事故を参照)。
        """
        st = self.link.state()
        tgt = self._target() if self.phase not in (Phase.IDLE, Phase.DONE) else None
        mode = st.get("mode")
        mode_name = self.MODE_NAME.get(int(mode), "?") if mode is not None else "?"

        s = f"{self.phase.value}"
        if self.phase in (Phase.CRUISE, Phase.DWELL):
            s += f" -> {self._label()}"
        if tgt is not None:
            s += f" ({tgt[0]:+.2f}, {tgt[1]:+.2f}, {tgt[2]:.2f})"
        if self._dist_h is not None:
            s += f" 残{self._dist_h:.2f}m"
        s += f"  mode={mode_name}"
        s += f"  armed={int(bool(st.get('armed', 0)))}"
        s += f"  guided={int(bool(st.get('guided', 0)))}"
        s += f"  h={st.get('range_h', 0.0):.2f}m"

        dev_yaw = st.get("yaw")
        if self._yaw_src == "camera":
            s += "  yaw=camera(実測)"
        elif dev_yaw is not None:
            flag = " ★機首ズレ疑い" if abs(dev_yaw) > 15.0 else ""
            s += (f"  yaw=fixed(前提{YAW_INITIAL_ALIGN_DEG:+.0f}) "
                  f"機体実測={dev_yaw:+.1f}deg{flag}")
        else:
            s += "  yaw=fixed(機体側yaw不明)"

        if self._diff is not None:
            s += f"  自己位置ズレ={self._diff[2]:.2f}m"
        s += f"  link={'OK' if self.link.telemetry_ok() else 'LOST'}"
        return s
