"""
tests/test_mission_sim.py
本番プログラム (core/program.py) を模擬機体で通し、
「順番・制限時間・締切・着陸の静止判定」が意図どおりに働くかを確認する。

    cd position_estimator
    python -m unittest tests.test_mission_sim -v

時計は偽物 (FakeTime) なので数秒で終わる。制御の良し悪し (ゲイン) は見ない。
模擬機体は「指令した速度どおりに動き、指令した高度へスルーレートで寄り、
機動は所要時間ぶん maneuver フラグを立てる」だけの単純化。
"""

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import utils.config as config                # noqa: E402
import core.mission as mission_mod           # noqa: E402
import core.maneuver as maneuver_mod         # noqa: E402
import core.program as program_mod           # noqa: E402
from core.mission import MissionRunner, Phase   # noqa: E402
from core.program import StepKind, build_competition_program, check_geometry  # noqa: E402
from core.s5_protocol import (REQ_HOLD, REQ_TAKEOFF, REQ_GUIDED, REQ_LAND,   # noqa: E402
                              REQ_IDLE, REQ_NAME, REQ_CIRCLE, REQ_FIGURE8,
                              REQ_CLIMB_TURN, MODE_GUIDED, MODE_POSHOLD,
                              CF_YAW_VALID)

MANEUVER_REQS = (REQ_CIRCLE, REQ_FIGURE8, REQ_CLIMB_TURN)


class FakeTime:
    """core.mission / core.maneuver の time.time() を差し替える。"""
    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class FakeAircraft:
    """S5Link のふりをする模擬機体。send_command() を受けて自分の状態を進める。

    ・TAKEOFF/GUIDED/LAND の目標高度へ 0.3 m/s (着陸 0.2 m/s) のスルーで寄る
    ・GUIDED の目標速度どおりに水平位置が動く
    ・機動 (CIRCLE/FIGURE8/CLIMB) は「1周 = 360/|yaw_rate| 秒」だけ maneuver=1
      を立て、終わると自然に落ちる。HOLD/LAND が来たら即中断する (実機と同じ)
    ・着陸目標に近づいたら landed=1
    """

    def __init__(self, clock, guided_after_s=0.5, home=(0.0, -3.5)):
        self.clock = clock
        self.x, self.y, self.h = home[0], home[1], 0.0
        self.mode = MODE_POSHOLD
        self.armed = True
        self.guided = False
        self.landed = False
        self.airborne = False
        self.maneuver_until = None
        self.maneuver_req = None
        self.alt_target = 0.0
        self.vx = self.vy = 0.0
        self.yaw_deg = 0.0        # アーム基準の相対ヨー (ずれていない前提)
        self.t_first_cmd = None
        self.guided_after_s = guided_after_s
        self.sent = []            # (t, req, vx, vy, alt, flags)
        self.ok = True
        self._last_t = clock.time()
        self._n = 0

    # ---- S5Link 互換 ----
    def send_command(self, req, vx_mps=0.0, vy_mps=0.0, alt_m=0.0, yaw_rate_dps=0.0,
                     flags=0, corr_n_m=None, corr_e_m=None, yaw_abs_deg=None, laps=0):
        now = self.clock.time()
        self.sent.append((now, req, vx_mps, vy_mps, alt_m, flags))
        if self.t_first_cmd is None:
            self.t_first_cmd = now

        if self.maneuver_until is not None:
            # 実機と同じ: 機動中は HOLD / ABORT / LAND しか受け付けない
            if req in (REQ_HOLD, REQ_LAND):
                self.maneuver_until = None
                self.maneuver_req = None
            else:
                return

        if req == REQ_TAKEOFF:
            self.alt_target, self.vx, self.vy = alt_m, 0.0, 0.0
        elif req == REQ_GUIDED:
            self.alt_target, self.vx, self.vy = alt_m, vx_mps, vy_mps
        elif req == REQ_LAND:
            self.alt_target, self.vx, self.vy = 0.05, 0.0, 0.0
        elif req in MANEUVER_REQS:
            legs = {REQ_CIRCLE: max(1, laps), REQ_FIGURE8: 2,
                    REQ_CLIMB_TURN: 2 * max(1, laps) + 1}[req]
            self.maneuver_until = now + 360.0 * legs / abs(yaw_rate_dps or 15.0)
            self.maneuver_req = req
            self.vx = self.vy = 0.0
            if alt_m > 0:
                self.alt_target = alt_m
        elif req in (REQ_HOLD, REQ_IDLE):
            self.vx = self.vy = 0.0

    def step(self):
        """時計が進んだぶん機体を動かす (テスト側が clock.advance() の後に呼ぶ)。"""
        now = self.clock.time()
        dt = now - self._last_t
        self._last_t = now
        if (self.t_first_cmd is not None and not self.guided and not self.landed
                and now - self.t_first_cmd >= self.guided_after_s):
            self.guided = True
            self.mode = MODE_GUIDED
        if not self.guided:
            self._n += 1
            return
        slew = 0.2 if self.alt_target <= 0.05 else 0.3
        d = self.alt_target - self.h
        self.h += max(-slew * dt, min(slew * dt, d))
        if self.h > 0.10:
            self.airborne = True
        # 機体座標 (前+, 右+) -> フィールド座標。yaw=YAW_INITIAL_ALIGN_DEG 前提。
        psi = math.radians(config.YAW_INITIAL_ALIGN_DEG + self.yaw_deg)
        nx, ny = math.sin(psi), math.cos(psi)
        self.x += (self.vx * nx + self.vy * ny) * dt
        self.y += (self.vx * ny - self.vy * nx) * dt
        if self.alt_target <= 0.05 and self.h < 0.12 and self.airborne:
            self.landed = True
        if self.maneuver_until is not None and now >= self.maneuver_until:
            self.maneuver_until = None
            self.maneuver_req = None
        self._n += 1

    def state(self):
        return {"armed": 1.0 if self.armed else 0.0,
                "guided": 1.0 if self.guided else 0.0,
                "landed": 1.0 if self.landed else 0.0,
                "airborne": 1.0 if self.airborne else 0.0,
                "maneuver": 1.0 if self.maneuver_until is not None else 0.0,
                "mode": float(self.mode), "range_h": self.h, "yaw": self.yaw_deg,
                "flow_ok": 1.0, "frame_ok": 0.0,
                "fh_posn": self.y, "fh_pose": self.x, "alt_hold": self.alt_target}

    def flag(self, name):
        return bool(self.state().get(name, 0))

    def n_data(self):
        return self._n

    def telemetry_ok(self, max_age_s=1.0):
        return True

    def age(self):
        return 0.05

    def pos(self):
        return [self.x, self.y, self.h]


class _Base(unittest.TestCase):
    def setUp(self):
        self.clock = FakeTime()
        mission_mod.time = self.clock
        maneuver_mod.time = self.clock
        mission_mod.POS_CORR_ENABLED = False     # 原点合わせは別テストの範囲
        self.ac = FakeAircraft(self.clock)
        self.program = build_competition_program(config)

    def tearDown(self):
        import time
        mission_mod.time = time
        maneuver_mod.time = time

    def _fly_until(self, mission, cond, max_s=60.0, dt=0.033):
        """cond() が真になるまで模擬機体を進める。"""
        t_end = self.clock.time() + max_s
        while not cond() and self.clock.time() < t_end:
            self.clock.advance(dt)
            self.ac.step()
            mission.update(pos=self.ac.pos(),
                           yaw_rad=math.radians(config.YAW_INITIAL_ALIGN_DEG),
                           pos_valid=True, yaw_valid=True, yaw_src="fixed")
        self.assertTrue(cond(), msg=f"条件を満たさないまま {max_s}s 経過 "
                                    f"(phase={mission.phase.value})")

    def _run(self, mission, max_s=400.0, dt=0.033, on_tick=None):
        """DONE/ABORT になるまで回して、通ったフェーズと段階名を返す。"""
        phases = [mission.phase]
        steps = []
        t_end = self.clock.time() + max_s
        while mission.phase not in (Phase.DONE, Phase.ABORT) and self.clock.time() < t_end:
            self.clock.advance(dt)
            self.ac.step()
            mission.update(pos=self.ac.pos(),
                           yaw_rad=math.radians(config.YAW_INITIAL_ALIGN_DEG),
                           pos_valid=True, yaw_valid=True, yaw_src="fixed")
            if mission.phase is not phases[-1]:
                phases.append(mission.phase)
            s = mission.step
            if s is not None and (not steps or steps[-1] != s.name):
                steps.append(s.name)
            if on_tick is not None:
                on_tick(mission)
        return phases, steps


class ProgramGeometryTest(_Base):
    def test_all_maneuvers_fit_inside_the_fence(self):
        """既定の設定で、各機動の軌跡が機体側フェンスに収まること。

        ここが落ちたら本番でフェンスに押し返されて円が崩れる = ミッション不成立。
        """
        ok, lines = check_geometry(self.program, config, config.YAW_INITIAL_ALIGN_DEG)
        self.assertTrue(ok, msg="\n".join(lines))

    def test_turn_radius_meets_the_rulebook(self):
        """ルールブック: 旋回半径は概ね 1.5m 以上。"""
        for s in self.program:
            if s.kind is StepKind.MANEUVER:
                self.assertGreaterEqual(s.maneuver.radius_m, 1.5 - 1e-6,
                                        msg=f"{s.name} r={s.maneuver.radius_m:.2f}m")

    def test_expected_total_fits_the_target(self):
        """想定所要時間の合計が目標 (3分) に収まること。"""
        total = sum(program_mod.expected_seconds(s, config) for s in self.program)
        self.assertLess(total, config.COMP_TARGET_S,
                        msg=f"想定合計 {total:.0f}s > 目標 {config.COMP_TARGET_S:.0f}s")

    def test_deadlines_are_ordered_and_inside_the_rule(self):
        self.assertLess(config.COMP_LAND_BY_S, config.COMP_FORCE_LAND_BY_S)
        self.assertLess(config.COMP_FORCE_LAND_BY_S, config.COMP_RULE_DEADLINE_S)

    def test_climb_turn_is_entered_at_the_low_altitude(self):
        """上昇旋回は「低高度で2周 -> 上昇 -> 高高度で2周」。

        ★ その前の GOTO が到達高度 (2.2m) へ上がってしまうと、低高度の2周が
          成立せずミッションが判定されない。進入高度は巡航高度であること。
        """
        for i, s in enumerate(self.program):
            if s.kind is StepKind.MANEUVER and s.maneuver.req == REQ_CLIMB_TURN:
                entry = self.program[i - 1]
                self.assertIs(entry.kind, StepKind.GOTO)
                self.assertAlmostEqual(entry.entry_alt, config.COMP_CRUISE_ALT_M,
                                       msg="上昇旋回の進入高度が巡航高度でない")
                self.assertGreater(s.maneuver.alt_m, config.COMP_CRUISE_ALT_M + 1.0,
                                   msg="到達高度が低すぎる (ポール以上にならない)")
                self.assertLess(s.maneuver.alt_m, config.MISSION_FENCE_Z,
                                msg="到達高度がジオフェンスの天井以上 (途中で着陸に落ちる)")
                break
        else:
            self.fail("上昇旋回がプログラムに無い")

    def test_order_matches_the_rulebook(self):
        """離陸 -> 水平旋回 -> 上昇旋回 -> 8の字 -> 着陸 の順で、間に必ず GOTO が入る。"""
        kinds = [s.kind for s in self.program]
        self.assertIs(kinds[0], StepKind.TAKEOFF)
        self.assertIs(kinds[-1], StepKind.LAND)
        man = [s for s in self.program if s.kind is StepKind.MANEUVER]
        self.assertEqual([s.maneuver.req for s in man],
                         [REQ_CIRCLE, REQ_CLIMB_TURN, REQ_FIGURE8])
        # どの機動の直前も GOTO であること (定位置へ戻ってから始める)
        for i, s in enumerate(self.program):
            if s.kind is StepKind.MANEUVER:
                self.assertIs(self.program[i - 1].kind, StepKind.GOTO,
                              msg=f"{s.name} の前が GOTO でない")


class FullFlightTest(_Base):
    def test_full_program_completes_in_time(self):
        m = MissionRunner(self.ac, self.program, verbose=False)
        self.assertTrue(m.start())
        phases, steps = self._run(m)
        self.assertEqual(phases[-1], Phase.DONE,
                         msg=f"phases={[p.value for p in phases]} reason={m.reason}")
        # 全段階を順に通っていること
        self.assertEqual(steps, [s.name for s in self.program], msg=steps)
        # 静止判定まで到達している
        self.assertIn(Phase.STILL, phases)
        # 競技時計がルールの締切内
        self.assertLess(m.elapsed(), config.COMP_RULE_DEADLINE_S,
                        msg=f"T+{m.elapsed():.0f}s")
        # 3つの機動がすべて機体へ届いている
        reqs = {s[1] for s in self.ac.sent}
        for r in MANEUVER_REQS:
            self.assertIn(r, reqs, msg=f"{REQ_NAME[r]} を送っていない")
        # "fixed" のあいだ CF_YAW_VALID を立てない (機体のヨーを踏みつぶさない)
        self.assertTrue(all(not (s[5] & CF_YAW_VALID) for s in self.ac.sent))
        # 離陸地点へ帰って降りている
        self.assertLess(math.hypot(self.ac.x - 0.0, self.ac.y - (-3.5)), 0.6,
                        msg=f"着陸位置 ({self.ac.x:.2f}, {self.ac.y:.2f})")
        self.assertTrue(self.ac.landed)

    def test_mission_clock_starts_at_guided(self):
        """競技時計は「GUIDED に入った瞬間」から。待機で待たされた分は入らない。"""
        m = MissionRunner(self.ac, self.program, verbose=False)
        self.ac.guided_after_s = 8.0      # 8秒アームしたまま待たされる
        m.start()
        for _ in range(400):              # 13秒ぶん回す
            self.clock.advance(0.033)
            self.ac.step()
            m.update(pos=self.ac.pos(),
                     yaw_rad=math.radians(config.YAW_INITIAL_ALIGN_DEG),
                     pos_valid=True, yaw_valid=True, yaw_src="fixed")
        self.assertTrue(self.ac.guided)
        self.assertLess(m.elapsed(), 6.0,
                        msg=f"待機の8秒が競技時計に入っている: T+{m.elapsed():.1f}s")

    def test_arming_sends_hold_so_the_aircraft_can_enter_guided(self):
        """待機中に REQ_HOLD を送り続けること。

        ★ ここが送られないと機体は GUIDED に入れない (新鮮な上りコマンドが無いため)。
          2026-09-16 まではこれが原因で毎回 [M] を押す必要があった。
        """
        self.ac.guided_after_s = 1e9      # 永遠に GUIDED に入らない
        m = MissionRunner(self.ac, self.program, verbose=False)
        m.start()
        for _ in range(100):
            self.clock.advance(0.033)
            self.ac.step()
            m.update(pos=self.ac.pos(), yaw_rad=0.0, pos_valid=True, yaw_valid=True)
        self.assertEqual(m.phase, Phase.ARMING)
        self.assertGreater(len(self.ac.sent), 5)
        self.assertTrue(all(s[1] == REQ_HOLD for s in self.ac.sent),
                        msg=f"待機中に HOLD 以外を送った: "
                            f"{ {REQ_NAME[s[1]] for s in self.ac.sent} }")


class DeadlineTest(_Base):
    def test_step_budget_aborts_a_stuck_maneuver(self):
        """機動が終わらなくても、制限時間で打ち切って次へ進むこと。"""
        m = MissionRunner(self.ac, self.program, verbose=False)
        m.start()
        # 機動を絶対に終わらせない模擬機体にする
        orig = self.ac.send_command

        def never_finish(req, **kw):
            orig(req, **kw)
            if req in MANEUVER_REQS and self.ac.maneuver_until is not None:
                self.ac.maneuver_until = self.clock.time() + 1e9
        self.ac.send_command = never_finish

        phases, steps = self._run(m, max_s=600.0)
        self.assertEqual(phases[-1], Phase.DONE, msg=f"reason={m.reason}")
        # 打ち切られても最後まで進み、着陸している
        self.assertIn("着陸", steps)
        self.assertTrue(self.ac.landed)

    def test_land_by_deadline_skips_remaining_missions(self):
        """COMP_LAND_BY_S を過ぎたら、残りのミッションを捨てて帰投・着陸する。"""
        m = MissionRunner(self.ac, self.program, verbose=False)
        m.start()
        self._fly_until(m, lambda: m.phase is Phase.CRUISE)   # 離陸を終えて巡航へ
        # 競技時計だけを締切の向こう側へ飛ばす
        m._t_mission = self.clock.time() - (config.COMP_LAND_BY_S + 1.0)
        phases, steps = self._run(m, max_s=120.0)
        self.assertEqual(phases[-1], Phase.DONE, msg=f"reason={m.reason}")
        self.assertIn("締切", m.reason)
        self.assertTrue(self.ac.landed)
        # 帰投してから降りている (その場着陸ではない)
        self.assertLess(math.hypot(self.ac.x, self.ac.y - (-3.5)), 0.6)

    def test_force_land_lands_in_place(self):
        """COMP_FORCE_LAND_BY_S を過ぎたら帰投せず、その場で降りる。"""
        m = MissionRunner(self.ac, self.program, verbose=False)
        m.start()
        # 離陸してミッションエリアへ移動し、離着陸エリアから十分離れた状態にする
        self._fly_until(m, lambda: m.phase is Phase.MANEUVER, max_s=120.0)
        away = (self.ac.x, self.ac.y)
        self.assertGreater(math.hypot(away[0], away[1] - (-3.5)), 1.5)
        m._t_mission = self.clock.time() - (config.COMP_FORCE_LAND_BY_S + 1.0)
        phases, steps = self._run(m, max_s=120.0)
        self.assertEqual(phases[-1], Phase.DONE, msg=f"reason={m.reason}")
        self.assertTrue(self.ac.landed)
        # 帰投していない = 締切を踏んだ場所の近くで降りている
        self.assertLess(math.hypot(self.ac.x - away[0], self.ac.y - away[1]), 1.0,
                        msg=f"その場着陸のはずが移動した: {away} -> "
                            f"({self.ac.x:.2f}, {self.ac.y:.2f})")


class SafetyTest(_Base):
    def test_abort_lands(self):
        m = MissionRunner(self.ac, self.program, verbose=False)
        m.start()
        for _ in range(200):
            self.clock.advance(0.033)
            self.ac.step()
            m.update(pos=self.ac.pos(), yaw_rad=math.radians(config.YAW_INITIAL_ALIGN_DEG),
                     pos_valid=True, yaw_valid=True)
        self.assertNotIn(m.phase, (Phase.IDLE, Phase.DONE))
        m.abort("テスト")
        self.assertEqual(m.phase, Phase.LAND)
        phases, _ = self._run(m)
        self.assertEqual(phases[-1], Phase.DONE)
        self.assertTrue(self.ac.landed)

    def test_lost_position_lands_while_cruising(self):
        """移動中にカメラを見失ったら着陸する (機動中は機体単独で飛べるので別扱い)。"""
        m = MissionRunner(self.ac, self.program, verbose=False)
        m.start()
        while m.phase is not Phase.CRUISE:
            self.clock.advance(0.033)
            self.ac.step()
            m.update(pos=self.ac.pos(), yaw_rad=math.radians(config.YAW_INITIAL_ALIGN_DEG),
                     pos_valid=True, yaw_valid=True)
        for _ in range(int(m.POS_LOST_LAND_S / 0.033) + 10):
            self.clock.advance(0.033)
            self.ac.step()
            m.update(pos=None, yaw_rad=math.radians(config.YAW_INITIAL_ALIGN_DEG),
                     pos_valid=False, yaw_valid=True)
        self.assertIn(m.phase, (Phase.LAND, Phase.STILL, Phase.DONE))
        self.assertIn("見失", m.reason)

    def test_still_requires_five_seconds(self):
        """静止判定は 5 秒。途中で動いたら数え直す。"""
        m = MissionRunner(self.ac, self.program, verbose=False)
        m.start()
        self._fly_until(m, lambda: m.phase is Phase.CRUISE)   # 浮かせる
        m.abort("静止判定の確認")
        self._fly_until(m, lambda: m.phase in (Phase.STILL, Phase.DONE), max_s=120.0)
        self.assertEqual(m.phase, Phase.STILL)
        t0 = self.clock.time()
        # 2秒たったところで大きく動かす -> 数え直しになる
        moved = False
        while m.phase is Phase.STILL:
            self.clock.advance(0.033)
            self.ac.step()
            if not moved and self.clock.time() - t0 > 2.0:
                moved = True
                self.ac.x += 1.0
            m.update(pos=self.ac.pos(), yaw_rad=math.radians(config.YAW_INITIAL_ALIGN_DEG),
                     pos_valid=True, yaw_valid=True)
        self.assertEqual(m.phase, Phase.DONE)
        self.assertGreaterEqual(self.clock.time() - t0,
                                2.0 + config.COMP_LAND_STILL_S - 0.5)


if __name__ == "__main__":
    unittest.main()
