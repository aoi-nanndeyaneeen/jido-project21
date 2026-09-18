"""
core/program.py
本番 (4分間の飛行競技) で飛ぶ順番と、それぞれに許す時間を「プログラム」として書き下す。

==========================================================================
なぜ「プログラム」という形にするか
==========================================================================
2026-09-16 までの core/mission.py は「ウェイポイントのリスト + 最後に1回だけ
定型機動」という形だった。本番は

    離陸 → 水平旋回 → 上昇旋回 → 8の字 → 着陸

と **機動が3つ並び、その合間に必ず定位置へ戻る**。しかも各ミッションには
制限時間があり、間に合わなければ打ち切って次へ行かないと後続が全部落ちる。
「WPリスト + 機動1回」ではこれを表現できないので、
    Step (種類・名前・制限時間・目標) の列
として持ち、MissionRunner がこの列を順に実行する形にした。

==========================================================================
得点と順番 (ルールブック 6.7〜6.13)
==========================================================================
連続成功で倍率が 1.5 倍ずつ乗る。だから順番は固定で、**着陸が最も価値が高い**。

| # | ミッション | 素点 | 倍率 | 実得点 |
|---|---|---|---|---|
| 1 | 滑走路内離陸 | 100  | 1.2       |  120 |
| 2 | 水平旋回 (連続2周) | 1000 | 1.2x1.5   | 1800 |
| 3 | 上昇旋回 | 1200 | 1.2x1.5^2 | 3240 |
| 4 | 8の字飛行 | 1400 | 1.2x1.5^3 | 5670 |
| 5 | 着陸 (自動着陸滑走路) | 800  | 1.2x1.5^4 | 4860 |

★ 着陸は 5 秒以上の静止を求められ、**3分55秒以降の着陸は失敗**になる。
  つまり「何があっても時間内に降ろす」が最優先。途中の機動を1つ捨てても
  着陸を守るほうが得点は高い (後続の倍率ごと失うため)。
  → MissionRunner は LAND_BY_S / FORCE_LAND_BY_S の 2 段で必ず着陸へ倒す。

==========================================================================
幾何 (6m x 9m フィールド / 最小旋回半径 1.5m)
==========================================================================
機動は機体座標で「前進 v + ヨーレート ω」を出すだけなので、地面に描かれる
円の中心は **開始地点から見て旋回方向側へ半径 R だけ横** になる
(R = v / ω)。したがって:

    開始地点 = 円の中心にしたい点 - s * R * (機体右方向の単位ベクトル)
                                     s = +1 (右旋回) / -1 (左旋回)

・水平旋回 / 上昇旋回 … 円1つ。直径 2R = 3.0m。6x9m のどこにでも収まる。
・8の字        … 円を2つ「並べて」描く。並ぶ向きは機首に直交する向きで、
                 端から端まで 4R = 6.0m 要る (円2つが接する = 直径2つぶん)。

  ルールブックは「機首の左右に」とは言っていない。要は 4R=6.0m の長い辺を、
  フィールドの **長い方 (9m) に沿って縦に** 8の字を並べればよい。
  6m(x) x 9m(y) で、機首を +y (奥) に向けると 8の字は x 方向 (幅6m) に並んで
  ちょうど詰まる (ドリフトぶんはみ出す)。機首を +x (右) に向けると 8の字は
  y 方向 (奥行9m) に **縦に** 並ぶので余裕を持って収まる。
  → 既定の YAW_INITIAL_ALIGN_DEG は 90 (= 機首を +x = フィールド右へ向けて置く)。

  「起動時に実寸で検算」とは: check_geometry() が、設定した半径・周回数・機首の
  向きから **各機動が地面に描く軌跡の外接矩形** を計算し、機体側フェンス
  (x±2.7, y±4.2m) に収まるかを起動時に一覧で出す、ということ。設定を間違えたまま
  飛ばして「機動の途中でフェンスに押し返されて円が崩れる」のを、飛ばす前の
  画面で気づけるようにする安全確認。
"""

import math
from dataclasses import dataclass
from enum import Enum

from core.maneuver import Maneuver, Circle, FigureEight, ClimbTurn
from core.s5_protocol import REQ_FIGURE8


class StepKind(Enum):
    TAKEOFF  = "TAKEOFF"    # 地上 -> 巡航高度。滑走路内離陸 (ルール 6.7)
    GOTO     = "GOTO"       # 定位置へ移動して落ち着く (ミッションの前後に必ず挟む)
    MANEUVER = "MANEUVER"   # 機体単独の定型機動 (旋回 / 上昇旋回 / 8の字)
    LAND     = "LAND"       # 帰投済みの地点で自動着陸 (ルール 6.13)


@dataclass(frozen=True)
class Step:
    """プログラムの1段階。

    budget_s : この段階に許す時間 [s]。超えたら **打ち切って次へ進む**。
               (機動が決まらないまま粘ると、後続のミッションと着陸を巻き添えにする)
    target   : GOTO の目標。None なら「機動の開始地点を機首から計算する」意味。
    """
    kind: StepKind
    name: str                       # 画面・ログに出す名前
    budget_s: float
    maneuver: Maneuver = None       # MANEUVER のとき
    target: tuple = None            # GOTO のとき (x, y, z)。None = 機動の開始地点
    settle_s: float = 0.0           # GOTO 到達後にこの秒数だけ静止してから次へ
    score: int = 0                  # ルールブックの素点 (表示用)
    entry_for: Maneuver = None      # GOTO: この機動の開始地点へ行く、という意味
    dr_s: float = 0.0               # GOTO: 自己位置が無いときの開ループ進入 [s]。
                                    # ★ 機首方向へ dr_speed で dr_s 秒だけ前進して
                                    #   から次へ進む。カメラが無い/落ちたときに
                                    #   「ミッションエリアへ進入する」(ルール 6.7)
                                    #   を成立させるための最後の手段。
                                    #   位置が出ているときは使わない (普通に飛ぶ)。
    dr_speed: float = 0.0           # 上の前進速度 [m/s]
    entry_alt: float = None         # GOTO: そこへ行くときの高度 [m]。
                                    # ★ 機動の alt_m とは別物。ClimbTurn の alt_m は
                                    #   「到達高度 (2.2m)」なので、これを開始高度に使うと
                                    #   先に 2.2m まで上がってしまい、ルールの
                                    #   「低高度で2周 -> 上昇 -> 高高度で2周」が成立しない。
    hold_forever: bool = False      # GOTO: 到達しても次へ進まずその場に留まり続ける
                                    #       (保持性能の測定 / 本番の「滞空」段階)
    loiter: bool = False            # この段階は「時間を使うための滞空」。
                                    # ★ 想定所要 (expected_seconds) は 0 として扱う。
                                    #   締切 (COMP_LAND_BY_S) が来るまで居座るのが
                                    #   仕事なので、想定合計に入れると意味が壊れる。

    @property
    def label(self) -> str:
        return f"{self.name}" + (f" ({self.score}点)" if self.score else "")


# ==========================================================================
#  機動の開始地点 (機首方向から計算する)
# ==========================================================================
def right_unit(yaw_rad: float):
    """機体右方向の水平単位ベクトル (フィールド座標)。

    core/mission.py の body_frame_errors と同じ定義:
        n = (sin psi, cos psi)   機首方向
        r = (ny, -nx) = (cos psi, -sin psi)   機体右方向
    """
    return math.cos(yaw_rad), -math.sin(yaw_rad)


def entry_point(center_xy, yaw_rad: float, maneuver: Maneuver, alt_m: float):
    """`center_xy` を円の中心にするための開始地点 (x, y, z)。

    旋回方向 s は yaw_rate の符号。開始地点は中心から s*R だけ機体左へ戻った点。
    ★ 8の字は「右の円 → 左の円」なので、1つ目の円の中心が center_xy に来る。
      全体の中心を center_xy にしたいなら center_xy をそのまま開始地点にする
      (下の footprint() が両方を見て検算する)。
    """
    rx, ry = right_unit(yaw_rad)
    s = 1.0 if maneuver.yaw_rate_dps >= 0 else -1.0
    r = maneuver.radius_m
    if maneuver.req == REQ_FIGURE8:
        # 8の字は開始地点の左右に円ができる。開始地点 = 全体の中心。
        return (center_xy[0], center_xy[1], alt_m)
    return (center_xy[0] - s * r * rx, center_xy[1] - s * r * ry, alt_m)


def footprint(start_xy, yaw_rad: float, maneuver: Maneuver):
    """開始地点と機首から、機動が地面に描く軌跡の外接矩形 (x0, x1, y0, y1)。"""
    rx, ry = right_unit(yaw_rad)
    s = 1.0 if maneuver.yaw_rate_dps >= 0 else -1.0
    r = maneuver.radius_m
    centers = [(start_xy[0] + s * r * rx, start_xy[1] + s * r * ry)]
    if maneuver.req == REQ_FIGURE8:
        centers.append((start_xy[0] - s * r * rx, start_xy[1] - s * r * ry))
    x0 = min(c[0] for c in centers) - r
    x1 = max(c[0] for c in centers) + r
    y0 = min(c[1] for c in centers) - r
    y1 = max(c[1] for c in centers) + r
    return x0, x1, y0, y1


# ==========================================================================
#  プログラムの組み立て
# ==========================================================================
def build_competition_program(cfg) -> list:
    """utils.config の COMP_* から本番のプログラムを組み立てる。

    cfg は utils.config モジュールをそのまま渡す (テストから差し替えられるよう
    module ではなく引数にしてある)。
    """
    # スケール: 半径と速度を同じ率で縮めると omega = v/R が変わらないので、
    # 1周の時間も時間配分もそのままで場所だけ小さくなる (通し練習用)。
    scale  = float(getattr(cfg, "COMP_SCALE", 1.0))
    small  = (scale < 0.999)
    v      = cfg.COMP_MANEUVER_SPEED * scale
    r      = cfg.COMP_TURN_RADIUS_M * scale
    alt    = cfg.COMP_SMALL_CRUISE_ALT_M if small else cfg.COMP_CRUISE_ALT_M
    hi_alt = cfg.COMP_SMALL_CLIMB_ALT_M  if small else cfg.COMP_CLIMB_ALT_M
    dir_s  = 1.0 if cfg.COMP_TURN_RIGHT else -1.0
    # 半径 R = v / omega  ->  omega[deg/s] = v / R * 180/pi
    omega = dir_s * math.degrees(v / r)

    circle = Circle(v, omega, alt, cfg.COMP_CIRCLE_LAPS)
    # ★ ClimbTurn の alt_m は「到達高度」。開始高度は機動が始まった時点の実高度
    #   (機体側 Guided.h が range_h から取る) なので、ここへは巡航高度で入る。
    climb  = ClimbTurn(v, omega, hi_alt, cfg.COMP_CLIMB_LAPS)
    fig8   = FigureEight(v, omega, alt)

    b = cfg.COMP_BUDGET_S      # 段階名 -> 制限時間 [s]
    center = cfg.COMP_FIELD_CENTER
    with_climb = bool(getattr(cfg, "COMP_ENABLE_CLIMB", True))

    # 進入だけは「自己位置が無くても必ずミッションエリアへ入る」必要がある
    # (ルール 6.7)。カメラが落ちていたら機首方向へ開ループで前進する。
    entry_dr = float(getattr(cfg, "COMP_ENTRY_DR_S", 0.0)) * (scale if small else 1.0)

    steps = [
        Step(StepKind.TAKEOFF, "滑走路内離陸", b["takeoff"], score=100),
        Step(StepKind.GOTO, "ミッションエリアへ進入", b["goto_first"],
             entry_for=circle, entry_alt=alt, settle_s=cfg.COMP_SETTLE_S,
             dr_s=entry_dr, dr_speed=v),
        Step(StepKind.MANEUVER, "水平旋回", b["circle"], maneuver=circle, score=1000),
    ]
    if with_climb:
        steps += [
            Step(StepKind.GOTO, "定位置へ戻る", b["goto"],
                 entry_for=climb, entry_alt=alt, settle_s=cfg.COMP_SETTLE_S),
            Step(StepKind.MANEUVER, "上昇旋回", b["climb"], maneuver=climb, score=1200),
        ]
    steps += [
        Step(StepKind.GOTO, "定位置へ戻る", b["goto"],
             entry_for=fig8, entry_alt=alt, settle_s=cfg.COMP_SETTLE_S),
        Step(StepKind.MANEUVER, "8の字飛行", b["figure8"], maneuver=fig8, score=1400),
    ]

    # ---- 滞空 (飛行継続ボーナス) ----------------------------------------
    # 飛行継続ボーナス = 飛行時間[s] x 2 x 成功ミッション数。5ミッションなら
    # 10点/秒 で、これは倍率の対象外。つまり **早く降りた秒数はそのまま失点**。
    # 機動が予定より早く終わっても、締切 (COMP_LAND_BY_S) までここで粘る。
    #  ★ 目標は「8の字の開始地点」= 直前の機動が終わった場所そのもの。新しい所へ
    #    移動させない (移動はドリフトと時間の無駄でしかない)。
    #  ★ hold_forever なので自分からは終わらない。終わらせるのは締切だけ。
    steps.append(Step(StepKind.GOTO, "滞空 (飛行継続ボーナス)", b["loiter"],
                      entry_for=fig8, entry_alt=alt,
                      hold_forever=True, loiter=True))

    # ---- 着陸 ------------------------------------------------------------
    # COMP_RETURN_HOME=False (既定) なら帰投せず、いまいる場所で降りる。
    # 素点は 飛行競技エリア内 300 / 離着陸エリア内 400 (COMPETITION_OPEN_ISSUES G5)。
    if bool(getattr(cfg, "COMP_RETURN_HOME", False)):
        steps.append(Step(StepKind.GOTO, "離着陸エリアへ帰投", b["goto_home"],
                          target=None, settle_s=cfg.COMP_LAND_SETTLE_S))
        steps.append(Step(StepKind.LAND, "着陸", b["land"], score=400))
    else:
        steps.append(Step(StepKind.LAND, "着陸 (その場)", b["land"], score=300))
    return steps


def build_waypoint_program(cfg) -> list:
    """段階確認用 (COMP_ENABLED=False)。MISSION_WAYPOINTS を巡回して帰投・着陸するだけ。

    Phase 4〜6 の立ち上げ手順 (WAYPOINT_BRINGUP.md) に戻したいときに使う。
    本番のプログラムと同じ Step の列なので、実行側 (MissionRunner) は共通。
    """
    b = cfg.COMP_BUDGET_S
    steps = [Step(StepKind.TAKEOFF, "離陸", b["takeoff"])]
    for i, wp in enumerate(cfg.MISSION_WAYPOINTS):
        steps.append(Step(StepKind.GOTO, f"WP{i}", b["goto_first"],
                          target=tuple(wp), settle_s=cfg.COMP_SETTLE_S,
                          hold_forever=(i == 0 and cfg.MISSION_HOLD_AT_FIRST_WP)))
    steps.append(Step(StepKind.GOTO, "帰投", b["goto_home"],
                      settle_s=cfg.COMP_LAND_SETTLE_S))
    steps.append(Step(StepKind.LAND, "着陸", b["land"]))
    return steps


def program_summary(program, cfg, yaw_rad=0.0) -> list:
    """起動時に出す「何を何秒で飛ぶつもりか」の一覧。戻り値は行のリスト。"""
    lines = []
    t = 0.0
    total_expected = 0.0
    lines.append("  # 段階                    想定    制限   累計(制限)  備考")
    for i, s in enumerate(program):
        exp = expected_seconds(s, cfg)
        t += s.budget_s
        total_expected += exp
        note = ""
        if s.kind is StepKind.MANEUVER:
            m = s.maneuver
            note = (f"r={m.radius_m:.2f}m {m.n_legs}周 "
                    f"v={m.fwd_mps:.2f} w={m.yaw_rate_dps:+.1f}deg/s")
        elif s.loiter:
            note = f"締切 {cfg.COMP_LAND_BY_S:.0f}s まで居座る (継続ボーナス)"
        elif s.kind is StepKind.GOTO and s.entry_for is not None:
            note = "機動の開始地点 (機首から計算)"
            if s.dr_s > 0.0:
                note += f" / 位置が無ければ機首へ {s.dr_speed * s.dr_s:.1f}m 開ループ進入"
        elif s.kind is StepKind.GOTO:
            note = "離陸地点 (地上で見えていた位置)"
        lines.append(f"  {i + 1} {s.label:<22} {exp:5.0f}s {s.budget_s:5.0f}s "
                     f"{t:7.0f}s   {note}")
    lines.append(f"  想定合計 {total_expected:.0f}s (滞空を除く)  "
                 f"(目標 {cfg.COMP_TARGET_S}s / 着陸開始の締切 {cfg.COMP_LAND_BY_S}s / "
                 f"その場着陸 {cfg.COMP_FORCE_LAND_BY_S}s)")
    lines.append(f"  着陸は {'離陸地点へ帰投 (400点)' if getattr(cfg, 'COMP_RETURN_HOME', False) else 'その場 (300点)'}"
                 f" / 機動が早く終わったぶんは滞空で使い切る")
    return lines


def expected_seconds(step: Step, cfg) -> float:
    """その段階の想定所要時間 [s] (制限時間ではなく、うまくいったときの値)。"""
    # 滞空は「余った時間を使い切る」段階なので、想定所要は 0 として数える。
    # (合計に入れると「想定合計 < 目標」という検算の意味が無くなる)
    if step.loiter:
        return 0.0
    if step.kind is StepKind.MANEUVER:
        return step.maneuver.expected_duration_s + cfg.COMP_MANEUVER_START_S
    if step.kind is StepKind.TAKEOFF:
        return cfg.COMP_EXPECT_TAKEOFF_S
    if step.kind is StepKind.LAND:
        return cfg.COMP_EXPECT_LAND_S + cfg.COMP_LAND_STILL_S
    # GOTO: 距離が飛行時に決まるので、設定の想定値を使う
    return cfg.COMP_EXPECT_GOTO_S + step.settle_s


def check_geometry(program, cfg, yaw_deg: float) -> list:
    """機動の軌跡が機体側ジオフェンスの内側に収まるかを実寸で検算する。

    戻り値は (ok, 行のリスト)。ok=False なら、その設定で飛ばすと境界で
    押し返されて円が崩れる (= 機動が判定されない)。
    """
    yaw = math.radians(yaw_deg)
    lim_x, lim_y = cfg.COMP_FENCE_X, cfg.COMP_FENCE_Y
    lines = [f"  機首 {yaw_deg:+.0f}deg のとき、各機動が使う範囲 "
             f"(フェンス x±{lim_x:.1f} y±{lim_y:.1f} m):"]
    ok = True
    for s in program:
        if s.kind is not StepKind.MANEUVER:
            continue
        start = entry_point(cfg.COMP_FIELD_CENTER, yaw, s.maneuver, 0.0)
        x0, x1, y0, y1 = footprint(start, yaw, s.maneuver)
        fits = (-lim_x <= x0 and x1 <= lim_x and -lim_y <= y0 and y1 <= lim_y)
        ok = ok and fits
        lines.append(f"    {s.name:<12} x[{x0:+.2f},{x1:+.2f}] y[{y0:+.2f},{y1:+.2f}]"
                     f"  {'OK' if fits else '★はみ出す'}")
    if not ok:
        lines.append("    ★ 機首の向き (YAW_INITIAL_ALIGN_DEG) か旋回半径 "
                     "(COMP_TURN_RADIUS_M) を見直してください。")
        lines.append("      8の字は 4R=6.0m の長さを、フィールドの長辺 (9m) に沿って")
        lines.append("      縦に並べれば収まります (= 機首をフィールド右 +x へ)。")
    return ok, lines
