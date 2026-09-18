"""
core/mission.py
「プログラムに書いた順番どおりに、時間内に飛んで、必ず降りる」の本体。

==========================================================================
役割分担 (ここを間違えると必ず落ちる)
==========================================================================

    カメラ (tracker)  ->  フィールド座標の位置 P [m]
    yaw_estimator     ->  機首方位 psi [rad]
         |
         v
    このファイル: 位置誤差 -> 「機体座標の目標速度」 [m/s]        <- 外側ループ (SEND_HZ=10Hz 上限)
         |  USB -> 地上局 (蓋 125ms) -> IM920 上り 最大 8Hz
         v
    機体 (drone_s5 / PosHold): 目標速度 -> 速度PID -> 目標リーン角  <- 内側ループ (25Hz, フロー窓)
    機体 (Mixer/姿勢PID):      目標リーン角 -> モーター            <- 最内 (1000Hz)

★ PC は速度までしか指令しない。姿勢は絶対に送らない。
  IM920sL は半二重 19200bps・実効 15Hz・往復 100〜200ms。この遅れを姿勢
  ループに入れると位相余裕を食って発振する。詳しくは protocol/S5Cmd.h の先頭。

★ PC が黙れば機体は勝手に「その場ホールド -> 自動着陸」に落ちる。
  だから「指令を出し続けること」自体が生存確認になっている。
  逆に言うと、このクラスは異常を見つけたら **黙るのではなく LAND を出す**。

==========================================================================
何を飛ぶかは core/program.py (Step の列)
==========================================================================
本番は 離陸 -> 水平旋回 -> 上昇旋回 -> 8の字 -> 着陸 で、機動どうしの間に
必ず定位置へ戻る。各段階には制限時間 (Step.budget_s) があり、超えたら
**打ち切って次へ進む**。粘ると後続のミッションと着陸を巻き添えにするため。

時間の守り方は3段:
  1. Step.budget_s        … その段階を打ち切る
  2. COMP_LAND_BY_S       … 何をしていても帰投・着陸へ入る
  3. COMP_FORCE_LAND_BY_S … 帰投を諦めて **その場で** 降りる
     (離着陸エリア外の着陸は 800->300点。飛んだまま 3分55秒 を迎えると 0点)

==========================================================================
自己位置の補正
==========================================================================
機体はオプティカルフローの積分だけで自分の水平位置 (pos_n/pos_e) を持って
いる。床の模様が薄いとこれが流れる。カメラは絶対位置 (cm オーダー) を
持っているので、POS_CORR_PERIOD_S 秒に1回だけ「本当はここにいるはず」を
送って上書きする。★ 姿勢・速度のクローズドループとは別物。無線の往復遅延を
含んだ位置を毎ループ使うと発振するので「たまに書き換えるだけ」に留める。

==========================================================================
自動離陸・自動着陸をどこでやるか
==========================================================================
機体側 (AltHold::commandTarget) にやらせる。PC は「目標高度 1.0m」と言うだけで、
そこへ何 m/s で寄せるかは機体側のスルーレート。離陸中にリンクが切れても
機体は最後の目標高度でホバーし続けられる。
"""

import math
import time
from dataclasses import dataclass
from enum import Enum

from utils.config import (MISSION_TAKEOFF_ALT_M, YAW_INITIAL_ALIGN_DEG,
                          MISSION_FENCE_X, MISSION_FENCE_Y, MISSION_FENCE_Z,
                          MISSION_FENCE_GRACE_S, MISSION_YAW_PROBE_VEL,
                          POS_CORR_ENABLED, POS_CORR_PERIOD_S,
                          POS_CORR_MAX_STEP_M,
                          POS_CORR_TRACK_TOL_M, POS_CORR_CONFIRM_S,
                          COMP_FIELD_CENTER, COMP_CRUISE_ALT_M,
                          COMP_LAND_BY_S, COMP_FORCE_LAND_BY_S,
                          COMP_RULE_DEADLINE_S, COMP_LAND_STILL_S,
                          COMP_LAND_STILL_TOL_M, COMP_TARGET_S)
from core.maneuver import ManeuverRunner
from core.program import StepKind, entry_point
from core.s5_link import (REQ_ABORT, REQ_GUIDED, REQ_HOLD, REQ_LAND,
                          REQ_TAKEOFF, REQ_NAME, MODE_NAME,
                          CF_ARMED_OK, CF_POS_VALID, CF_YAW_VALID,
                          CF_ALT_ABS, CF_POS_CORR, CF_POS_SHIFT)


class Phase(Enum):
    IDLE      = "IDLE"       # 待機。何も送らない
    ARMING    = "ARMING"     # 機体が GUIDED に入るのを待って REQ_HOLD を送り続ける
    TAKEOFF   = "TAKEOFF"    # 離陸高度まで上昇中
    CRUISE    = "CRUISE"     # 目標地点へ移動中
    DWELL     = "DWELL"      # 目標地点で静止保持中 (機動の直前に落ち着かせる)
    MANEUVER  = "MANEUVER"   # 機体単独の定型機動 (旋回/8の字/上昇旋回) を実行中
    LAND      = "LAND"       # 自動着陸中
    STILL     = "STILL"      # 接地後、静止判定 (ルール: 5秒以上) の確認中
    DONE      = "DONE"       # 完了
    ABORT     = "ABORT"      # 中断 (手動 / 機体が GUIDED を抜けた)


@dataclass
class _Tick:
    """update() の 1 回 (SEND_HZ に間引かれた回) の材料。_phase_* に渡す。"""
    now: float
    pos: object            # フィールド座標 [x, y, z] or None
    pos_valid: bool
    yaw_rad: object        # float or None
    yaw_valid: bool
    flags: int             # CF_* の論理和 (この回の送信に付ける)
    st: dict               # link.state() のスナップショット
    h_agl: float           # 機体の測距による対地高度 [m]


def body_frame_errors(dx, dy, yaw_rad):
    """
    フィールド座標の誤差 (dx, dy) を機体座標の (前方, 右方) へ変換する。

        n = (sin psi, cos psi)   機首方向 (psi=0 が +y = フィールド奥)
        r = (ny, -nx)            機体右方向

    ★ ヨーの定義は core/yaw_estimator.py (yaw = atan2(nx, ny)) および
      core/program.py の right_unit() と同じ。符号を間違えると位置ループが
      正帰還になる (= 機体が飛んでいく)。
    """
    nx, ny = math.sin(yaw_rad), math.cos(yaw_rad)
    rx, ry = ny, -nx
    return dx * nx + dy * ny, dx * rx + dy * ry


class MissionRunner:
    """
    core/program.py の Step 列を順に実行する。

    使い方 (main_loop から):
        mission = MissionRunner(link, build_competition_program(config))
        mission.start()                      # アームのエッジ or [M]
        ...毎フレーム...
        mission.update(pos=P, yaw_rad=yaw, pos_valid=ok, yaw_valid=yv)
    """

    # ---- チューニング ------------------------------------------------
    # 位置誤差 [m] -> 目標速度 [m/s] の比例ゲイン。
    #  ★ これは「機体のPID」ではなく「PCが出す目標速度」。
    #  ★ 2026-09-15: 0.5 -> 0.3 + KD_VEL 0.4。中心保持試験で周期7秒・平均0.30m の
    #    揺れが続いた。指令→実速度の遅れが1.4〜1.6秒あり、機体の速度PI (KP4/KI2) が
    #    同じ帯域で共振して実速度が指令の約2倍に振れていた。
    #  ★ 2026-09-16 00:11: 0.45/0.45 を試して即戻した。離陸直後の中心へ向かう区間で
    #    円運動になり、20秒間一度も 0.2m 以内に入れずカメラを見失って自動着陸した。
    #    0.3/0.4 で固定。速さは遅れの短縮で稼ぐ話で、このゲインでは出ない。
    KP_POS = 0.3
    # カメラ実測速度 [m/s] -> 目標速度 [m/s] のブレーキ (減衰) ゲイン。
    KD_VEL = 0.4
    # カメラ速度の平滑化 (送信ごとの差分に掛ける1次LPF係数)。
    VEL_ALPHA = 0.5
    # 指令してよい最大速度 [m/s]。機体側 GUIDED_MAX_VEL でもクランプされる。
    #  ★ 2026-09-17: 0.4 -> 0.6。KP_POS はそのままなので、この上限に当たるのは
    #    誤差 2m 以上の遠方だけ (0.3 * 2.0 = 0.6)。揺れが出るのは目標の近くで
    #    ループが閉じるところなので、遠方の上限を上げても振動には効かず、
    #    本番の 4〜5m の移動だけが速くなる。
    MAX_VEL = 0.6
    # 目標速度がこれ未満なら 0 として送る (機体側は不感帯で位置ホールドへ落ちる)
    MIN_VEL = 0.04

    # 到達判定
    ARRIVE_R_M   = 0.20    # 水平 [m]
    ARRIVE_Z_M   = 0.20    # 高度 [m]
    # 機動の開始地点は、多少ずれていても円の位置がずれるだけで危険はない。
    # 厳しくすると指数的な追い込みで時間を食うので、緩めの半径で先へ進む。
    ARRIVE_R_ENTRY_M = 0.35

    # 離陸
    TAKEOFF_ALT_M   = MISSION_TAKEOFF_ALT_M  # 離陸後にいったん保持する高度 [m]
    TAKEOFF_TOL_M   = 0.15  # この差まで来たら離陸完了

    # 指令レート [Hz]。上限であって実効レートではない (カメラのフレームが
    # 来たときにしか update() が呼ばれないので、15fps のカメラなら 7.5Hz)。
    SEND_HZ = 10.0
    # カメラ速度 (_vcam) を計算するときの最小の時間差 [s]。
    VEL_DT_MIN_S = 0.15

    # yaw_src が "fixed" のとき、機体自身の実測yaw(アーム基準)がこれ未満なら
    # フル速度で飛ばしてよいとみなす閾値 [deg]。
    YAW_FIXED_RISK_DEG = 15.0

    # ---- 安全 ---------------------------------------------------------
    # 自己位置をこの秒数見失ったら着陸する (移動中のみ。機動中は機体単独で飛べる)
    POS_LOST_LAND_S = 1.5
    # 飛び始めてからの上限時間 [s]。プログラムの締切 (COMP_FORCE_LAND_BY_S) を
    # 超えても何も起きなかったときの最後の保険。
    MISSION_TIMEOUT_S = COMP_FORCE_LAND_BY_S + 40.0
    # ジオフェンス [m]
    FENCE_X = MISSION_FENCE_X
    FENCE_Y = MISSION_FENCE_Y
    FENCE_Z = MISSION_FENCE_Z
    FENCE_GRACE_S = MISSION_FENCE_GRACE_S

    def __init__(self, link, program, home=None, verbose=True, use_camera=True):
        self.link = link
        self.program = list(program)
        # カメラ (自己位置) を当てにしてよいか。False = fly_nocam.py から。
        #  ★ False のとき変わるのは 2 つだけ:
        #    - 「自己位置を見失った -> 着陸」の安全判定を行わない
        #      (行うと離陸直後に必ず着陸してしまう。位置は最初から無い)
        #    - GOTO は開ループ進入 (Step.dr_s) か、その場ホバー
        #  高度・機動・着陸・締切は位置を使っていないのでそのまま動く。
        self.use_camera = bool(use_camera)
        self.home = tuple(home) if home is not None else None
        # 呼び出し側が戻り先を固定したか。None なら毎フライト離陸地点から決め直す。
        self._home_fixed = self.home
        # アーム中に地上 (airborne=0) で最後に見えた位置 (x, y)。ディスアームで消す。
        self._ground_pos = None
        self.verbose = verbose

        # 着陸へ倒すときの飛び先。プログラム末尾の「帰投 GOTO」と「LAND」。
        self._idx_land = next((i for i, s in enumerate(self.program)
                               if s.kind is StepKind.LAND), len(self.program) - 1)
        self._idx_home = next((i for i, s in enumerate(self.program)
                               if s.kind is StepKind.GOTO and s.target is None
                               and s.entry_for is None), self._idx_land)

        self.phase = Phase.IDLE
        self.step_idx = 0
        self.reason = ""
        self._runner = None          # 実行中の ManeuverRunner
        self._target_xyz = None      # 今の GOTO の目標 (段階の頭で1回だけ決める)
        self._step_note = ""         # 画面に出す補足 (打ち切りの理由など)

        self._t_phase = 0.0
        self._t_step = 0.0
        self._t_mission = None       # GUIDED に入った時刻 = 競技時計の 0 秒
        self._t_send = 0.0
        self._t_pos_ok = 0.0
        self._dr_said = None         # 開ループ進入の告知を段階ごとに1回だけ出す
        self._t_still = None         # 静止判定を始めた時刻
        self._still_ref = None       # 静止判定の基準位置
        # カメラ実測速度 (フィールド座標 m/s) と、その差分用の直前位置 (x, y, t)
        self._vcam = None
        self._prev_cam = None
        # 保持試験モード (Step.hold_forever) の出入り記録
        self._hold_inside = False
        self._t_hold_in = None
        self._t_hold_first = None

        # ---- 安全判定の状態 --------------------------------------------
        self._t_fly = None      # 離陸フェーズに入った時刻 (タイムアウトの起点)
        self._t_fence = None    # ジオフェンスの外に出始めた時刻
        self._airborne = False  # 機体が一度でも「浮いた」と言ったか

        # ---- 自己位置補正の状態 ------------------------------------------
        self._align = None
        self._t_corr = 0.0
        self._pending_corr = None   # 次の _send() 1回にだけ乗せる (n_m, e_m)
        self._diff = None           # (diff_x, diff_y, diff_norm, drone_x, drone_y) or None
        self._corr_ref = None       # 持続性チェックで追跡中の基準値
        self._t_corr_start = 0.0

        # ヨーの出所。"camera"(実測収束済み) か "fixed"(決め打ち)。
        self._yaw_src = "fixed"
        self._yaw_dev_deg = None    # 機体自身の実測yaw [deg] (アーム基準)
        self._yaw_rad = math.radians(YAW_INITIAL_ALIGN_DEG)  # 直近の機首方位

        # ---- ログ用 ----------------------------------------------------
        self._tx = {"req": "", "vx": 0.0, "vy": 0.0, "alt": 0.0, "flags": 0,
                    "phase": Phase.IDLE.value}
        self._n_tx = 0              # 送信回数。ログはこれが増えた時だけ書く
        self._dist_h = None
        self._pending_event = ""    # _say() したメッセージ (ログが1回読むと消える)

    # ---------------------------------------------------------------- 制御
    def set_program(self, program):
        """プログラムを差し替える (飛行前だけ)。[S]/[C] の切り替えから呼ぶ。

        戻り値 False = 実行中なので差し替えなかった。
        """
        if self.phase not in (Phase.IDLE, Phase.DONE, Phase.ABORT):
            return False
        self.program = list(program)
        self._idx_land = next((i for i, s in enumerate(self.program)
                               if s.kind is StepKind.LAND), len(self.program) - 1)
        self._idx_home = next((i for i, s in enumerate(self.program)
                               if s.kind is StepKind.GOTO and s.target is None
                               and s.entry_for is None), self._idx_land)
        self.step_idx = 0
        return True

    def start(self):
        """アームの立ち上がり (自動) か [M] キーで呼ぶ入口。以降は update() が全部やる。"""
        if self.phase not in (Phase.IDLE, Phase.DONE, Phase.ABORT):
            self._say(f"すでに実行中です ({self.phase.value})")
            return False
        if not self.link.ok:
            self._say("地上局リンクが無いので開始できません")
            return False
        now = time.time()
        self.phase = Phase.ARMING
        self.step_idx = 0
        self.reason = ""
        self._step_note = ""
        self._runner = None
        self._target_xyz = None
        # 前のフライトの戻り先を持ち越さない
        self.home = self._home_fixed
        self._vcam = None
        self._prev_cam = None
        self._hold_inside = False
        self._t_hold_in = None
        self._t_hold_first = None
        self._t_phase = now
        self._t_step = now
        self._t_mission = None      # GUIDED に入った時点で 0 にする
        self._t_pos_ok = now
        self._t_fly = None
        self._t_fence = None
        self._t_still = None
        self._still_ref = None
        self._airborne = False
        # 機体側の pos_n/pos_e はアーム/リセットのたびに0へ戻る。原点合わせも取り直す。
        self._align = None
        self._t_corr = 0.0
        self._pending_corr = None
        self._diff = None
        self._corr_ref = None
        self._t_corr_start = 0.0
        self._say(f"待機開始: 機体が GUIDED に入ったら自動で離陸します "
                  f"({len(self.program)} 段階 / 想定 {COMP_TARGET_S:.0f} 秒以内)")
        return True

    def abort(self, reason="手動中断"):
        """即座に着陸させる。緊急停止ではない (それはプロポの仕事)。"""
        if self.phase in (Phase.IDLE, Phase.DONE):
            return
        self.reason = reason
        self._jump_to(self._idx_land, Phase.LAND)
        self._say(f"中断 -> 自動着陸: {reason}")

    def elapsed(self):
        """競技時計 [s]。GUIDED に入ってからの経過。未離陸なら 0。"""
        if self._t_mission is None:
            return 0.0
        return time.time() - self._t_mission

    def remaining(self):
        """ルール上の締切 (3分55秒) までの残り [s]。"""
        return COMP_RULE_DEADLINE_S - self.elapsed()

    @property
    def step(self):
        if 0 <= self.step_idx < len(self.program):
            return self.program[self.step_idx]
        return None

    def _goto(self, phase):
        self.phase = phase
        self._t_phase = time.time()

    def _say(self, msg):
        # ログが読み出す前に次の _say が来ても消さない
        self._pending_event = (f"{self._pending_event} / {msg}"
                               if self._pending_event else msg)
        if self.verbose:
            print(f"[Mission] {msg}")

    def _send(self, req, vx_mps=0.0, vy_mps=0.0, alt_m=0.0, flags=0,
              yaw_rad=None, yaw_rate_dps=0.0, laps=0):
        """送信と記録を必ずセットで行う。link.send_command() を直接呼ばないこと。"""
        # 保留中の位置補正があれば、この送信1回にだけ乗せて消費する。
        corr = self._pending_corr
        self._pending_corr = None
        corr_n = corr_e = None
        if corr is not None:
            flags |= CF_POS_CORR
            corr_n, corr_e = corr[0], corr[1]
            if len(corr) > 2 and corr[2]:
                flags |= CF_POS_SHIFT      # 原点合わせ (機体は動かない)

        # ★ phase は「送った時点」のものを控える (update() は送信後にフェーズを
        #   進めることがあるので、あとから self.phase を読むとズレる)。
        self._tx = {"req": REQ_NAME.get(req, str(req)),
                    "vx": vx_mps, "vy": vy_mps, "alt": alt_m, "flags": flags,
                    "phase": self.phase.value, "corr": corr}
        self._n_tx += 1
        self.link.send_command(req, vx_mps=vx_mps, vy_mps=vy_mps,
                               alt_m=alt_m, yaw_rate_dps=yaw_rate_dps, laps=laps,
                               flags=flags, corr_n_m=corr_n, corr_e_m=corr_e,
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
        s = self.step
        return {"wp_idx": self.step_idx,
                "step": s.name if s else "",
                "returning": self.step_idx >= self._idx_home,
                "tgt": self._target_xyz,
                "dist_h": self._dist_h,
                "elapsed": self.elapsed(),
                # phase = 送信時のフェーズ / phase_next = 送信後の今のフェーズ。
                "phase_next": self.phase.value,
                "aligned": self._align is not None,
                "diff": self._diff,
                **self._tx}

    # ------------------------------------------------------- 段階の出入り
    def _begin_step(self, now):
        """今の step_idx の段階を開始する (目標の確定・フェーズの設定)。"""
        s = self.step
        self._t_step = now
        self._step_note = ""
        self._runner = None
        self._target_xyz = None
        self._hold_inside = False
        self._t_hold_in = None
        self._t_hold_first = None
        if s is None:
            self._goto(Phase.DONE)
            return

        if s.kind is StepKind.TAKEOFF:
            self._goto(Phase.TAKEOFF)
        elif s.kind is StepKind.GOTO:
            self._target_xyz = self._resolve_goto_target(s)
            self._goto(Phase.CRUISE)
        elif s.kind is StepKind.MANEUVER:
            self._runner = ManeuverRunner(s.maneuver, self.link)
            self._runner.start(now)
            self._goto(Phase.MANEUVER)
        elif s.kind is StepKind.LAND:
            self._goto(Phase.LAND)

        if s.kind is not StepKind.TAKEOFF:
            tgt = self._target_xyz
            where = f" -> ({tgt[0]:+.2f}, {tgt[1]:+.2f}, {tgt[2]:.2f})" if tgt else ""
            self._say(f"[{self.step_idx + 1}/{len(self.program)}] {s.label}"
                      f"{where}  制限 {s.budget_s:.0f}s / 経過 {self.elapsed():.0f}s")
        self._say_call_cue()

    def _say_call_cue(self):
        """次のミッションの「コール」を操縦者に知らせる (ルール 6.4)。

        ★ 各ミッションは **開始直前に操縦者がミッション名をコール** しないと
          無効になる。機体は勝手に次へ進むので、PC 側から「今コールする」と
          言ってやらないと間に合わない。移動・滞空の段階に入った時点で、
          その次に来るミッション名を出す。
        """
        nxt = None
        for s in self.program[self.step_idx:]:
            if s.score:
                nxt = s
                break
        cur = self.step
        if nxt is None or cur is None:
            return
        if cur is nxt:
            self._say(f"★★ いまコール中のミッション:「{nxt.name}」を実行しています")
        else:
            self._say(f"★★ 次のミッションは「{nxt.name}」です。"
                      "始まる前に審判へコールしてください")

    def _resolve_goto_target(self, s):
        """GOTO の目標を決める。段階の頭で1回だけ計算し、以後は動かさない。

        ★ 機動の開始地点は「今の機首方位」から逆算する (core/program.entry_point)。
          こうすると機首がどちらを向いていても、円の中心が必ずフィールド中心に来る。
          毎ループ計算し直すと目標が動いて追いかけっこになるので、ここで固定する。
        """
        if s.entry_for is not None:
            # ★ 高度は Step.entry_alt (機動を **始める** 高度)。機動の alt_m を
            #   使ってはいけない: ClimbTurn の alt_m は到達高度 (2.2m) なので、
            #   先にそこまで上がってしまい「低高度で2周 -> 上昇」が成立しない。
            alt = s.entry_alt if s.entry_alt is not None else COMP_CRUISE_ALT_M
            return entry_point(COMP_FIELD_CENTER, self._yaw_rad, s.entry_for, alt)
        if s.target is not None:
            return tuple(s.target)
        # 帰投先: 離陸完了時に確定した home -> アーム中に地上で見えていた位置 ->
        # フィールド中心、の順。
        #  ★ 2番目の逃げ道が要る: 締切や異常で **離陸が終わる前に** 着陸へ倒すと
        #    home はまだ None で、中心へ飛んでから降りることになる。地上で見えて
        #    いた位置 (= 離着陸エリア) が分かっているなら、そちらへ戻るほうが
        #    着陸点が高い (自動着陸滑走路 800点 / 場外 300点)。
        if self.home is not None:
            return (self.home[0], self.home[1], COMP_CRUISE_ALT_M)
        if self._ground_pos is not None:
            return (self._ground_pos[0], self._ground_pos[1], COMP_CRUISE_ALT_M)
        return (COMP_FIELD_CENTER[0], COMP_FIELD_CENTER[1], COMP_CRUISE_ALT_M)

    def _stop_maneuver(self, alt_m=None):
        """機体が回っている途中なら止める。

        ★ 機体側 (quad/Guided.h) は GP_MANEUVER 中、HOLD / ABORT / LAND しか
          受け付けない。REQ_GUIDED を送っても無視されるので、次の段階へ進む前に
          必ずここを通すこと。通さないと「PC は移動しているつもり、機体はまだ
          回っている」という食い違いが起きる。
        """
        if self._runner is None:
            return
        self._runner = None
        self._send(REQ_HOLD, flags=CF_ARMED_OK | CF_ALT_ABS,
                   alt_m=alt_m if alt_m is not None else COMP_CRUISE_ALT_M)

    def _advance_step(self, now, note=""):
        """次の段階へ。末尾まで行ったら DONE。"""
        if note:
            self._step_note = note
        self._stop_maneuver()
        self.step_idx += 1
        if self.step_idx >= len(self.program):
            self._goto(Phase.DONE)
            self._say(f"プログラム完了 (経過 {self.elapsed():.0f}s)")
            return
        self._begin_step(now)

    def _jump_to(self, idx, phase=None):
        """締切や中断で、プログラムの途中へ飛ぶ (帰投 or 着陸)。"""
        self._stop_maneuver()
        self.step_idx = max(0, min(idx, len(self.program) - 1))
        self._begin_step(time.time())
        if phase is not None:
            self._goto(phase)

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
            yaw_src   : "camera" (実測収束済み) か "fixed" (初期アラインメント決め打ち)
        """
        # ★ 離陸地点 (帰投先) は、ミッション開始前から見ておく必要がある。
        #   アーム中に地上にいる間の最後の位置を覚える。
        st_ground = self.link.state()
        if not st_ground.get("armed"):
            self._ground_pos = None
        elif not st_ground.get("airborne") and pos_valid and pos is not None:
            self._ground_pos = (float(pos[0]), float(pos[1]))

        if self.phase in (Phase.IDLE, Phase.DONE, Phase.ABORT):
            return

        now = time.time()
        self._yaw_src = yaw_src
        if yaw_rad is not None:
            self._yaw_rad = yaw_rad
        if pos_valid and pos is not None:
            self._t_pos_ok = now

        if self.link.flag("airborne"):
            self._airborne = True

        # ---- 1) 安全側の打ち切り判定 (フェーズより先に見る) -------------
        if self.phase not in (Phase.LAND, Phase.STILL):
            why = self._safety_check(now, pos, pos_valid)
            if why:
                self.reason = why
                self._jump_to(self._idx_land, Phase.LAND)
                self._say(f"異常検知 -> 自動着陸: {why}")

        # ---- 2) 送信レートに間引く ------------------------------------
        if now - self._t_send < 1.0 / self.SEND_HZ:
            return
        self._t_send = now
        self._update_camera_velocity(now, pos, pos_valid)

        flags = CF_ARMED_OK | CF_ALT_ABS
        if pos_valid:
            flags |= CF_POS_VALID
        # ★ CF_YAW_VALID は「カメラで実測した絶対ヨー」のときだけ立てる。
        #   "fixed" (前提値) で立てると、機体が正しく知っている自分の回転を消す。
        if yaw_valid and yaw_src == "camera":
            flags |= CF_YAW_VALID

        st = self.link.state()
        h_agl = float(st.get("range_h", 0.0))    # 機体の測距による対地高度 [m]
        self._yaw_dev_deg = st.get("yaw")

        self._maybe_correct_position(now, pos, pos_valid, st)

        # 今の目標までの水平距離。到達判定とログの両方がこれを見る。
        tgt_now = self._target_xyz
        self._dist_h = (math.hypot(tgt_now[0] - float(pos[0]),
                                   tgt_now[1] - float(pos[1]))
                        if (tgt_now is not None and pos is not None) else None)

        # ---- 3) 競技時計の締切 (段階の中身より優先) ---------------------
        if self._t_mission is not None and self.phase not in (Phase.LAND, Phase.STILL):
            el = now - self._t_mission
            if el >= COMP_FORCE_LAND_BY_S and self.step_idx < self._idx_land:
                self.reason = f"締切 {COMP_FORCE_LAND_BY_S:.0f}s: その場で着陸"
                self._say(f"★ 経過 {el:.0f}s: 何があっても **その場で** 着陸します "
                          f"(規定の締切 {COMP_RULE_DEADLINE_S:.0f}s まで残り {self.remaining():.0f}s)"
                          "  ★ 審判へ「着陸」とコールしてください")
                self._jump_to(self._idx_land, Phase.LAND)
            elif el >= COMP_LAND_BY_S and self.step_idx < self._idx_home:
                # 帰投 GOTO があればそこへ、無ければ着陸そのものへ跳ぶ
                # (_idx_home は帰投が無いとき _idx_land に落ちる)。
                self.reason = f"締切 {COMP_LAND_BY_S:.0f}s: 着陸へ"
                self._say(f"★ 経過 {el:.0f}s: 残りを打ち切って着陸へ入ります"
                          "  ★ 審判へ「着陸」とコールしてください")
                self._jump_to(self._idx_home)

        # ---- 4) 段階ごとの制限時間 ------------------------------------
        s = self.step
        if (s is not None and self.phase not in (Phase.ARMING, Phase.DONE, Phase.ABORT)
                and now - self._t_step > s.budget_s):
            self._say(f"★ {s.name} が制限 {s.budget_s:.0f}s を超えました -> 打ち切って次へ")
            self._advance_step(now, note="時間切れ")
            s = self.step

        # ---- 5) 段階ごとの指令 ----------------------------------------
        tick = _Tick(now=now, pos=pos, pos_valid=pos_valid, yaw_rad=yaw_rad,
                     yaw_valid=yaw_valid, flags=flags, st=st, h_agl=h_agl)
        handler = self._PHASE_HANDLERS.get(self.phase)
        if handler is not None:
            handler(self, tick)

    # ---------------------------------------------------------- フェーズ
    def _phase_arming(self, t):
        """機体が GUIDED に入るまで REQ_HOLD を送り続ける。

        ★ ここで送り続けることが「離陸の号令」そのもの。機体は
          「新鮮な上りコマンドがある」+「SW_HOVER=up」+「スロットル15%以上」+
          「フロー/測距が健全」で初めて GUIDED に入る (quad/Guided.h)。
          入れない理由は機体側のシリアル画面に出る。ここでは待つだけ。
        """
        now = t.now
        self._send(REQ_HOLD, flags=t.flags, yaw_rad=t.yaw_rad)
        if self.link.flag("guided"):
            self._t_mission = now        # ここが競技時計の 0 秒
            dev_yaw = t.st.get("yaw")
            if self._yaw_src != "camera":
                dev_str = f"{dev_yaw:+.1f}deg" if dev_yaw is not None else "不明"
                self._say(f"機体が GUIDED に入りました -> 競技時計スタート "
                          f"(機首方位は{YAW_INITIAL_ALIGN_DEG:+.0f}deg決め打ち。"
                          f"機体自身のyaw={dev_str}。0から離れていたら機首がズレています)")
            else:
                self._say("機体が GUIDED に入りました -> 競技時計スタート")
            self._t_fly = now
            self.step_idx = 0
            self._begin_step(now)
        elif self.link.flag("landed") and now - self._t_phase > 2.0:
            self._say("機体が前回の着陸状態のままです。THR_CUT でディスアーム -> "
                      "再アームすると次の便に入れます")
            self._t_phase = now
        elif self.link.flag("armed") and now - self._t_phase > 15.0:
            self._say("アーム済みですが GUIDED に入りません。"
                      "SW_HOVER が上か、フロー/測距が生きているか、"
                      "スロットルが 15% 以上かを確認してください")
            self._t_phase = now      # 15 秒ごとに出し直す

    def _phase_takeoff(self, t):
        """滑走路内離陸 (ルール 6.7)。目標高度まで上げるだけ。

        ★ 上げ方 (スルーレート) は機体側 (AltHold::commandTarget)。PC は
          目標高度を言うだけなので、途中でリンクが切れてもホバーし続ける。
        """
        self._send(REQ_TAKEOFF, alt_m=self.TAKEOFF_ALT_M, flags=t.flags, yaw_rad=t.yaw_rad)
        if abs(t.h_agl - self.TAKEOFF_ALT_M) < self.TAKEOFF_TOL_M:
            # 離陸地点を覚える (帰投先)。地上にいた位置を優先し、
            # 見えていなければ今の位置で代用する。
            if self._home_fixed is None:
                if self._ground_pos is not None:
                    self.home = (self._ground_pos[0], self._ground_pos[1], self.TAKEOFF_ALT_M)
                    note = "地上の位置"
                elif t.pos is not None:
                    self.home = (float(t.pos[0]), float(t.pos[1]), self.TAKEOFF_ALT_M)
                    note = "地上で見えていなかったため今の位置"
                else:
                    note = "不明 (フィールド中心へ帰投します)"
                if self.home is not None:
                    self._say(f"離陸完了 (対地 {t.h_agl:.2f} m)  "
                              f"帰投先=({self.home[0]:+.2f}, {self.home[1]:+.2f}) [{note}]")
                else:
                    self._say(f"離陸完了 (対地 {t.h_agl:.2f} m)  帰投先={note}")
            else:
                self._say(f"離陸完了 (対地 {t.h_agl:.2f} m)")
            self._advance_step(t.now)

    def _phase_cruise(self, t):
        """定位置へ移動する。到達したら DWELL で落ち着かせてから次へ。"""
        target = self._target_xyz
        s = self.step
        if target is None:
            self._advance_step(t.now)
            return
        # ヘディングが分からないと「前」がどっちか分からない。
        #  この状態で **フィールド座標の** 速度を出すと 90 度ずれた方向へ飛ぶので、
        #  位置ループは必ず止める。
        if not t.yaw_valid or t.yaw_rad is None or not t.pos_valid or t.pos is None:
            if self._dead_reckon(t, target):
                return
            self._send(REQ_HOLD, alt_m=target[2], flags=t.flags, yaw_rad=t.yaw_rad)
            return

        vx, vy = self._velocity_command(t.pos, target, t.yaw_rad)
        self._send(REQ_GUIDED, vx_mps=vx, vy_mps=vy,
                   alt_m=target[2], flags=t.flags, yaw_rad=t.yaw_rad)

        dh = self._dist_h
        dz = abs(target[2] - t.h_agl)
        # 機動の開始地点は緩め、着陸地点と試験用WPは厳しめ
        r = self.ARRIVE_R_ENTRY_M if (s and s.entry_for is not None) else self.ARRIVE_R_M
        inside = dh is not None and dh < r and dz < self.ARRIVE_Z_M
        if s is not None and getattr(s, "hold_forever", False):
            self._note_hold(t.now, bool(inside), dh if dh is not None else 0.0)
            return
        if inside:
            self._say(f"{s.name} 到達 (残り {dh:.2f} m)")
            self._goto(Phase.DWELL)

    def _dead_reckon(self, t, target):
        """自己位置が無いときの開ループ進入。処理したら True。

        ★ なぜ要るか: ルール 6.7 の離陸は「離着陸エリア②から **ミッションエリアに
          進入すること**」まで含む。カメラが無い/落ちた便でその場ホバーしていると、
          離陸 120点 と、そのあとの倍率チェーンが丸ごとずれる。
        ★ なぜ安全か: 送るのは **機体座標の前進速度** (vx = 前, vy = 右) で、
          フィールド座標ではない。ヨーが分からなくても「機首の方向」は機体が
          知っているので、90度ずれた方向へ飛ぶ心配がない。機首は置いたときの
          向き (+x) のままなので、進む先も分かっている。
        ★ 止まるのは時間だけ。位置を見ていないので距離は速度 x 秒の見積り。
        """
        s = self.step
        if s is None or s.dr_s <= 0.0 or s.dr_speed <= 0.0:
            return False
        el = t.now - self._t_phase
        if el < s.dr_s:
            if self._dr_said is not s:
                self._dr_said = s
                self._say(f"自己位置が無いので開ループで進入します "
                          f"(機首へ {s.dr_speed:.2f}m/s x {s.dr_s:.0f}s "
                          f"= 約 {s.dr_speed * s.dr_s:.1f}m)")
            self._send(REQ_GUIDED, vx_mps=s.dr_speed, vy_mps=0.0,
                       alt_m=target[2], flags=t.flags, yaw_rad=t.yaw_rad)
            return True
        # 進みきった。止めて落ち着かせてから次の段階へ。
        self._send(REQ_HOLD, alt_m=target[2], flags=t.flags, yaw_rad=t.yaw_rad)
        self._say(f"開ループ進入 完了 (約 {s.dr_speed * s.dr_s:.1f}m)")
        self._goto(Phase.DWELL)
        return True

    def _phase_dwell(self, t):
        """到達点で静止して落ち着かせる (機動は静止から始めたい)。"""
        target = self._target_xyz
        self._send(REQ_HOLD, alt_m=target[2] if target else COMP_CRUISE_ALT_M,
                   flags=t.flags, yaw_rad=t.yaw_rad)
        s = self.step
        settle = s.settle_s if s else 0.0
        if t.now - self._t_phase >= settle:
            self._advance_step(t.now)

    def _phase_maneuver(self, t):
        """機体が自分で回っている。PC は開始要求 (バースト) のあと IDLE でリンクだけ生かす。

        位置・ヨーの有無は問わない (機体は使わない)。
        ★ HOLD/GUIDED を送ると機体側で機動が中断されるので送らないこと。
        """
        s = self.step
        req, vx, yr, alt, laps = self._runner.request()
        self._send(req, vx_mps=vx, yaw_rate_dps=yr, alt_m=alt, flags=t.flags,
                   yaw_rad=t.yaw_rad, laps=laps)
        st_m = self._runner.update(t.now)
        if st_m == "done":
            # 機体は自分で HOLD へ戻っている (Maneuver 完了時)。止め直す必要はない。
            self._say(f"{s.name} 完了 ({self._runner.elapsed_s(t.now):.1f}s / "
                      f"経過 {self.elapsed():.0f}s)")
            self._runner = None
            self._advance_step(t.now)
        elif st_m == "failed":
            # ★ 機動が始まらない/終わらないだけで着陸まで捨てる必要はない。
            #   打ち切って次の段階へ進む (後続のミッションと着陸を守る)。
            #   _advance_step -> _stop_maneuver が HOLD を送って機体を止める。
            self._say(f"{s.name} 失敗: {self._runner.reason} -> 打ち切って次へ")
            self._advance_step(t.now, note="失敗")

    def _phase_land(self, t):
        """自動着陸 (ルール 6.13)。接地を確認したら静止判定へ。"""
        self._send(REQ_LAND, flags=t.flags, yaw_rad=t.yaw_rad)
        if self.link.n_data() > 0 and not self.link.flag("armed"):
            # 既にディスアーム済み (地上) なら降ろすものが無い。
            self._begin_still(t.now, "機体はディスアーム済み")
            return
        if self.link.flag("landed"):
            self._begin_still(t.now, "接地を検知")
        elif t.now - self._t_phase > 40.0:
            self._goto(Phase.DONE)
            self._say("着陸完了の通知が来ませんでした。"
                      "機体の状態を目視で確認し、手動で回収してください")

    def _begin_still(self, now, why):
        self._t_still = now
        self._still_ref = None
        self._goto(Phase.STILL)
        self._say(f"着陸 ({why}) -> 静止判定 {COMP_LAND_STILL_S:.0f}s を開始 "
                  f"(経過 {self.elapsed():.0f}s / 規定締切まで {self.remaining():.0f}s)")

    def _phase_still(self, t):
        """ルール 6.13: 5秒間以上の静止を求められる。カメラで動いていないことを見る。

        ★ 機体は既に出力を切っている (GP_LANDED はディスアームまで解けない) ので、
          ここでやることは「動いていないと確認して記録に残す」だけ。
        """
        self._send(REQ_LAND, flags=t.flags, yaw_rad=t.yaw_rad)
        if t.pos_valid and t.pos is not None:
            p = (float(t.pos[0]), float(t.pos[1]))
            if self._still_ref is None:
                self._still_ref = p
                self._t_still = t.now
            elif math.hypot(p[0] - self._still_ref[0], p[1] - self._still_ref[1]) > COMP_LAND_STILL_TOL_M:
                # まだ動いている (接地の跳ね返り等)。基準を取り直して数え直す。
                self._still_ref = p
                self._t_still = t.now
        if t.now - (self._t_still or t.now) >= COMP_LAND_STILL_S:
            self._goto(Phase.DONE)
            self._say(f"静止 {COMP_LAND_STILL_S:.0f}s を確認 -> 着陸成立 "
                      f"(経過 {self.elapsed():.0f}s / 規定締切まで {self.remaining():.0f}s)。"
                      "THR_CUT でディスアームしてください"
                      + (f"  (理由: {self.reason})" if self.reason else ""))

    _PHASE_HANDLERS = {
        Phase.ARMING:   _phase_arming,
        Phase.TAKEOFF:  _phase_takeoff,
        Phase.CRUISE:   _phase_cruise,
        Phase.DWELL:    _phase_dwell,
        Phase.MANEUVER: _phase_maneuver,
        Phase.LAND:     _phase_land,
        Phase.STILL:    _phase_still,
    }

    # ------------------------------------------------------------ 内部

    def _update_camera_velocity(self, now, pos, pos_valid):
        """カメラ位置の差分から、フィールド座標の速度を平滑化して持つ。

        ★ 送信のたび (SEND_HZ) に呼ばれるが、微分に使う時間差は
          VEL_DT_MIN_S 以上が溜まるまで基準点を進めない。指令を速くしても
          微分の刻みが細かくならないようにするため (Camera1 は 15〜30fps
          しか無いので、0.1 秒だと同じサンプルを引いて速度 0 になる)。
        """
        if not pos_valid or pos is None:
            self._prev_cam = None
            self._vcam = None
            return
        x, y = float(pos[0]), float(pos[1])
        prev = self._prev_cam
        if prev is None:
            self._prev_cam = (x, y, now)
            return
        dt = now - prev[2]
        if dt < self.VEL_DT_MIN_S:
            # まだ差分を取るには近すぎる。基準点は据え置き、前回の速度を保つ。
            return
        self._prev_cam = (x, y, now)
        if dt > 0.6:
            self._vcam = None
            return
        vx, vy = (x - prev[0]) / dt, (y - prev[1]) / dt
        # 見失い明けの飛びや、1回だけゲートをすり抜けた誤検知は速度に入れない
        if math.hypot(vx, vy) > 2.0:
            self._vcam = None
            return
        if self._vcam is None:
            self._vcam = (vx, vy)
        else:
            a = self.VEL_ALPHA
            self._vcam = (self._vcam[0] + a * (vx - self._vcam[0]),
                          self._vcam[1] + a * (vy - self._vcam[1]))

    def _step_name(self):
        s = self.step
        return s.name if s is not None else "-"

    def _note_hold(self, now, inside, dh):
        """保持試験モードで、到達半径の出入りと滞在時間をイベントに残す。"""
        if inside == self._hold_inside:
            return
        self._hold_inside = inside
        if inside:
            self._t_hold_in = now
            if self._t_hold_first is None:
                self._t_hold_first = now
                self._say(f"{self._step_name()} 到達 (残り {dh:.2f} m) -> そのまま留まり続けます")
            else:
                self._say(f"{self._step_name()} 範囲内へ復帰 (残り {dh:.2f} m)")
        else:
            self._say(f"{self._step_name()} 範囲外へ (残り {dh:.2f} m)。"
                      f"連続 {now - self._t_hold_in:.1f} 秒留まった / "
                      f"初到達から {now - self._t_hold_first:.1f} 秒")

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
        if self._vcam is not None and self.KD_VEL > 0.0:
            # 速度ベクトルも誤差と同じ回転で機体座標へ (body_frame_errors は回転そのもの)
            v_fwd, v_right = body_frame_errors(self._vcam[0], self._vcam[1], yaw_rad)
            vx -= self.KD_VEL * v_fwd
            vy -= self.KD_VEL * v_right

        # ★ 速度クランプは「yaw_src==camera か」ではなく、機体自身の実測yaw
        #   (アーム基準の相対値) の大きさそのもので判断する。
        #   2026-09-15(3): YawEstimator (camera) はほぼ収束しないため、
        #   yaw_src だけで絞ると実害の無い日(実測yawのズレが小さい)でも
        #   ずっと低速(MISSION_YAW_PROBE_VEL)に貼り付いたままになり、
        #   中心到達に14秒かかった。実測yawが小さければ、それは
        #   YAW_INITIAL_ALIGN_DEG の前提が壊れていないという直接の証拠
        #   なので、camera実測(yaw_src=="camera")と同じくフル速度でよい。
        #   実測yawが取れない/大きい ときだけ絞る (安全側)。
        if self._yaw_src == "camera":
            max_vel = self.MAX_VEL
        elif self._yaw_dev_deg is not None and abs(self._yaw_dev_deg) < self.YAW_FIXED_RISK_DEG:
            max_vel = self.MAX_VEL
        else:
            max_vel = MISSION_YAW_PROBE_VEL

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
        if not POS_CORR_ENABLED or self.phase in (Phase.LAND, Phase.MANEUVER):
            return
        if not pos_valid or pos is None:
            return
        fh_n, fh_e = st.get("fh_posn"), st.get("fh_pose")
        if fh_n is None or fh_e is None or not st.get("flow_ok"):
            return

        cam_x, cam_y = float(pos[0]), float(pos[1])

        # ★ 2026-09-16: 原点合わせは「機体の座標系をフィールド座標に付け替える」
        #   (CF_POS_SHIFT)。以前は PC 側だけが _align を覚えていたが、それだと
        #   機体は自分がフィールドのどこにいるか知らず、機体側フェンスが
        #   効かせられない。shift 後は fh_posn/fh_pose がそのままフィールド
        #   座標 (n=y, e=x) なので _align は (0,0) になる。
        #   ★ 離陸前 (PosHold が active でない間) は機体側が pos を毎ループ 0 に
        #     戻すので、shift は airborne になってから送る。モード切替でも
        #     消える (frame_ok が落ちる) ので、そのときは取り直す。
        if self._align is not None and not st.get("frame_ok"):
            self._align = None
        if self._align is None:
            if not st.get("airborne"):
                return
            self._pending_corr = (cam_y, cam_x, True)
            self._align = (0.0, 0.0)
            self._diff = (0.0, 0.0, 0.0, cam_x, cam_y)
            self._corr_ref = None
            self._say(f"機体の座標系をフィールド座標へ合わせました "
                      f"({cam_x:+.2f}, {cam_y:+.2f}) -> 機体側フェンス有効")
            return

        drone_x = fh_e + self._align[0]
        drone_y = fh_n + self._align[1]
        diff_x = cam_x - drone_x
        diff_y = cam_y - drone_y
        diff_norm = math.hypot(diff_x, diff_y)
        self._diff = (diff_x, diff_y, diff_norm, drone_x, drone_y)

        # ★ PC がカメラ位置で速度指令を出している間 (CRUISE) は送らない。
        #   指令が不感帯を割ると機体はフロー位置保持へ落ち、そこへ 0.4m の
        #   書き換えが入ると機体自身が偽のズレを埋めに動き、PC のループと
        #   喧嘩する。中心保持試験 (20:30) ではフロー推定が 1.2〜1.6m 流れ、
        #   保持中ずっと 0.4m ずつ書き換えていた。記録 (_diff) は続ける。
        if self.phase is Phase.CRUISE:
            self._corr_ref = None
            return

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
        #  ★ カメラを使わない便 (fly_nocam.py) では見ない。位置は最初から無いので、
        #    ここを通すと離陸直後に必ず着陸する。
        if self.use_camera and self.phase in (Phase.CRUISE, Phase.DWELL):
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
        if self.phase in (Phase.CRUISE, Phase.DWELL, Phase.TAKEOFF, Phase.MANEUVER):
            if self.link.n_data() > 0 and not self.link.flag("guided"):
                self.reason = "機体が GUIDED を抜けた (手動介入)"
                self._goto(Phase.ABORT)
                self._say("機体が GUIDED を抜けました。"
                          "指令を停止して手動へ譲ります")
                self._send(REQ_ABORT)
                return None
        return None

    # ------------------------------------------------------------ 表示
    def status_line(self):
        """「今どの段階を、競技時計の何秒で飛んでいるか」を1行で。

        ★ 機体自身の yaw (アーム基準の相対方位) を必ず出す。yaw_src が "fixed" の
          ときはこれが唯一の手掛かりで、0 から離れているほど「機首は
          YAW_INITIAL_ALIGN_DEG を向いている」という前提が崩れている。
        """
        st = self.link.state()
        s = self.step
        mode = st.get("mode")
        mode_name = MODE_NAME.get(int(mode), "?") if mode is not None else "?"

        out = f"{self.phase.value}"
        if s is not None and self.phase not in (Phase.IDLE, Phase.DONE, Phase.ABORT):
            out += (f" [{self.step_idx + 1}/{len(self.program)}] {s.name}"
                    f" {time.time() - self._t_step:.0f}/{s.budget_s:.0f}s")
            if self._step_note:
                out += f" ({self._step_note})"
        if self._t_mission is not None:
            out += f"  T+{self.elapsed():.0f}s 残{self.remaining():.0f}s"
        # 次に来る得点ミッション (コール担当が見る)
        nxt = next((x for x in self.program[self.step_idx:] if x.score), None)
        if nxt is not None and nxt is not s:
            out += f"  次コール「{nxt.name}」"
        if self._target_xyz is not None:
            t = self._target_xyz
            out += f" ({t[0]:+.2f}, {t[1]:+.2f}, {t[2]:.2f})"
        if self._dist_h is not None:
            out += f" 残{self._dist_h:.2f}m"
        out += f"  mode={mode_name}"
        out += f"  armed={int(bool(st.get('armed', 0)))}"
        out += f"  guided={int(bool(st.get('guided', 0)))}"
        out += f"  h={st.get('range_h', 0.0):.2f}m"

        dev_yaw = st.get("yaw")
        if self._yaw_src == "camera":
            out += "  yaw=camera(実測)"
        elif dev_yaw is not None:
            flag = " ★機首ズレ疑い" if abs(dev_yaw) > self.YAW_FIXED_RISK_DEG else ""
            out += (f"  yaw=fixed(前提{YAW_INITIAL_ALIGN_DEG:+.0f}) "
                    f"機体実測={dev_yaw:+.1f}deg{flag}")
        else:
            out += "  yaw=fixed(機体側yaw不明)"

        if self._diff is not None:
            out += f"  自己位置ズレ={self._diff[2]:.2f}m"
        out += f"  link={'OK' if self.link.telemetry_ok() else 'LOST'}"
        return out

