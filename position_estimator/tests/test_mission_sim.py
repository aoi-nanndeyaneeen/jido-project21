"""
tests/test_mission_sim.py
ミッション状態機械 (core/mission.py) を模擬機体で ARMING → TAKEOFF → WP → 帰投 → 着陸
まで通す。実機を触らずに「フェーズが正しい順で進むか」「送っている REQ が正しいか」だけ
を確認する。時計は偽物 (FakeTime) で進めるので数秒で終わる。

    cd position_estimator
    python -m unittest tests.test_mission_sim -v

★ 制御の良し悪し (ゲイン) は見ない。位置ループの応答は模擬機体側で
  「指令速度どおりに動く」と単純化してある。
"""

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import core.mission as mission_mod          # noqa: E402
import core.maneuver as maneuver_mod        # noqa: E402
from core.mission import WaypointMission, Phase   # noqa: E402
from core.s5_protocol import (REQ_HOLD, REQ_TAKEOFF, REQ_GUIDED, REQ_LAND,   # noqa: E402
                              REQ_CIRCLE, REQ_IDLE, REQ_NAME,
                              MODE_GUIDED, MODE_POSHOLD, CF_YAW_VALID)


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
    ・GUIDED の目標速度どおりに水平位置が動く (yaw=0 前提: 前+ = +y, 右+ = +x)
    ・REQ_CIRCLE を受けたら maneuver フラグを立て、想定時間で自然に落ちる
    ・着陸目標 (alt<=0.05) に近づいたら landed=1
    """
    def __init__(self, clock, guided_after_s=0.5):
        self.clock = clock
        self.x, self.y, self.h = 0.0, -1.0, 0.0
        self.mode = MODE_POSHOLD
        self.armed = True
        self.guided = False
        self.landed = False
        self.airborne = False
        self.maneuver_until = None
        self.alt_target = 0.0
        self.vx = self.vy = 0.0
        self.t_first_cmd = None
        self.guided_after_s = guided_after_s
        self.sent = []          # (t, req, vx, vy, alt, flags)
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
        if req == REQ_TAKEOFF:
            self.alt_target, self.vx, self.vy = alt_m, 0.0, 0.0
        elif req == REQ_GUIDED:
            self.alt_target, self.vx, self.vy = alt_m, vx_mps, vy_mps
        elif req == REQ_LAND:
            self.alt_target, self.vx, self.vy = 0.05, 0.0, 0.0
        elif req == REQ_CIRCLE:
            if self.maneuver_until is None:
                self.maneuver_until = now + 360.0 * max(1, laps) / abs(yaw_rate_dps or 15.0)
            self.vx = self.vy = 0.0
        elif req in (REQ_HOLD, REQ_IDLE):
            self.vx = self.vy = 0.0

    def step(self):
        """時計が進んだぶん機体を動かす (テスト側が clock.advance() の後に呼ぶ)。"""
        now = self.clock.time()
        dt = now - self._last_t
        self._last_t = now
        if self.t_first_cmd is not None and now - self.t_first_cmd >= self.guided_after_s:
            if not self.guided and not self.landed:
                self.guided = True
                self.mode = MODE_GUIDED
        if self.guided:
            slew = 0.2 if self.alt_target <= 0.05 else 0.3
            d = self.alt_target - self.h
            self.h += max(-slew * dt, min(slew * dt, d))
            if self.h > 0.10:
                self.airborne = True
            self.y += self.vx * dt          # yaw=0: 前+ = +y
            self.x += self.vy * dt          #        右+ = +x
            if self.alt_target <= 0.05 and self.h < 0.12 and self.airborne:
                self.landed = True
                self.guided = False
                self.mode = MODE_POSHOLD
            if self.maneuver_until is not None and now >= self.maneuver_until:
                self.maneuver_until = None
        self._n += 1

    def state(self):
        return {"armed": 1.0 if self.armed else 0.0,
                "guided": 1.0 if self.guided else 0.0,
                "landed": 1.0 if self.landed else 0.0,
                "airborne": 1.0 if self.airborne else 0.0,
                "maneuver": 1.0 if self.maneuver_until is not None else 0.0,
                "mode": float(self.mode), "range_h": self.h, "yaw": 0.0,
                "flow_ok": 1.0, "frame_ok": 0.0,
                "fh_posn": self.y + 1.0, "fh_pose": self.x, "alt_hold": self.alt_target}
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


class MissionSimTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeTime()
        mission_mod.time = self.clock
        maneuver_mod.time = self.clock
        mission_mod.POS_CORR_ENABLED = False     # 原点合わせは別テストの範囲
        self.ac = FakeAircraft(self.clock)

    def tearDown(self):
        import time
        mission_mod.time = time
        maneuver_mod.time = time

    def _run(self, mission, max_s=300.0, dt=0.033):   # 0.1 の約数にしない (浮動小数の丸めで送信が 1 周期飛ぶ)
        phases = [mission.phase]
        t_end = self.clock.time() + max_s
        while mission.phase not in (Phase.DONE, Phase.ABORT) and self.clock.time() < t_end:
            self.clock.advance(dt)
            self.ac.step()
            mission.update(pos=self.ac.pos(), yaw_rad=0.0, pos_valid=True,
                           yaw_valid=True, yaw_src="fixed")
            if mission.phase is not phases[-1]:
                phases.append(mission.phase)
        return phases

    def test_full_flight_without_maneuver(self):
        m = WaypointMission(self.ac, [(0.5, -0.5, 0.5), (0.5, 0.5, 0.5)], maneuver=None, verbose=False)
        m.TAKEOFF_ALT_M = 0.5
        self.assertTrue(m.start())
        phases = self._run(m)
        self.assertEqual(phases[0], Phase.ARMING)
        self.assertEqual(phases[-1], Phase.DONE, msg=f"phases={[p.value for p in phases]} reason={m.reason}")
        # CENTER, WP0, WP1, HOME = 4 回の CRUISE→DWELL
        self.assertEqual(phases.count(Phase.DWELL), 4, msg=[p.value for p in phases])
        reqs = [REQ_NAME[s[1]] for s in self.ac.sent]
        for r in ("HOLD", "TAKEOFF", "GUIDED", "LAND"):
            self.assertIn(r, reqs)
        # 送信レートは SEND_HZ (10Hz) を上限に、呼び出し周期 (=カメラ fps) で量子化される。
        #  ここは dt=0.033 で回しているので 0.132s ごと = 7.6Hz になる。実機でも Camera1 が
        #  15fps なら 7.5Hz。「SEND_HZ は上限であって実効レートではない」ことの確認。
        span = self.ac.sent[-1][0] - self.ac.sent[0][0]
        rate = len(self.ac.sent) / span
        self.assertLessEqual(rate, m.SEND_HZ * 1.05)
        self.assertGreaterEqual(rate, m.SEND_HZ * 0.5)
        # "fixed" のあいだ CF_YAW_VALID を立てていない (機体のヨーを踏みつぶさない)
        self.assertTrue(all(not (s[5] & CF_YAW_VALID) for s in self.ac.sent))
        # 着陸地点 = 離陸地点 (地上で見えていた位置) の近く
        self.assertLess(math.hypot(self.ac.x - 0.0, self.ac.y - (-1.0)), m.ARRIVE_R_M + 0.1)

    def test_maneuver_at_last_waypoint(self):
        m = WaypointMission(self.ac, [(0.3, 0.0, 0.5)], maneuver="default", verbose=False)
        m.TAKEOFF_ALT_M = 0.5
        if m.maneuver is None:
            self.skipTest("config の MISSION_MANEUVER が None")
        m.start()
        phases = self._run(m, max_s=400.0)
        self.assertEqual(phases[-1], Phase.DONE, msg=f"{[p.value for p in phases]} {m.reason}")
        self.assertIn(Phase.MANEUVER, phases)
        reqs = [s[1] for s in self.ac.sent]
        self.assertIn(m.maneuver.req, reqs)
        # 機動要求のバースト後は IDLE でリンクだけ生かしている
        i_first = reqs.index(m.maneuver.req)
        tail = reqs[i_first:]
        self.assertIn(REQ_IDLE, tail)

    def test_abort_lands(self):
        m = WaypointMission(self.ac, [(0.5, 0.5, 0.5)], maneuver=None, verbose=False)
        m.start()
        for _ in range(60):          # 3 秒回して離陸させる
            self.clock.advance(0.05)
            self.ac.step()
            m.update(pos=self.ac.pos(), yaw_rad=0.0, pos_valid=True, yaw_valid=True)
        self.assertIn(m.phase, (Phase.TAKEOFF, Phase.CRUISE))
        m.abort("テスト")
        self.assertEqual(m.phase, Phase.LAND)
        phases = self._run(m)
        self.assertEqual(phases[-1], Phase.DONE)
        self.assertTrue(self.ac.landed)

    def test_lost_position_lands(self):
        m = WaypointMission(self.ac, [(0.5, 0.5, 0.5)], maneuver=None, verbose=False)
        m.start()
        # 離陸完了まで進める
        while m.phase in (Phase.ARMING, Phase.TAKEOFF):
            self.clock.advance(0.05)
            self.ac.step()
            m.update(pos=self.ac.pos(), yaw_rad=0.0, pos_valid=True, yaw_valid=True)
        self.assertEqual(m.phase, Phase.CRUISE)
        # カメラを見失う
        for _ in range(int(m.POS_LOST_LAND_S / 0.05) + 5):
            self.clock.advance(0.05)
            self.ac.step()
            m.update(pos=None, yaw_rad=0.0, pos_valid=False, yaw_valid=True)
        self.assertEqual(m.phase, Phase.LAND)
        self.assertIn("見失", m.reason)


if __name__ == "__main__":
    unittest.main()
