"""Fake mailbox tests only: these tests never connect to hardware."""
import contextlib
import copy
import io
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/live_tuning"))
import continuous_random_session as cr


class Log:
    def __init__(self):
        self.rows = []

    def emit(self, event, **data):
        self.rows.append((event, data))


def held(pose=(1., 2., math.radians(137))):
    x, y, yaw = pose
    return dict(command=7, phase=4, error=0, request_seq=3, ack_seq=3, sample_seq=100,
                sample=dict(phase=4, error=0, enabled=1, state_1=1, state_2=1, state_3=1,
                            target_1=0., target_2=0., target_3=0., odom_x_m=x, odom_y_m=y,
                            odom_yaw_rad=yaw, yaw_deg=math.degrees(yaw), odom_valid=1,
                            odom_age_ms=2., imu_age_ms=1., tick_ms=1000.,
                            end_x_m=x, end_y_m=y, end_yaw_rad=yaw, end_valid=1, end_stopped=1,
                            end_elapsed_ms=1000., end_progress_m=.1, end_target_distance_m=.1))


def read_fake(session):
    session.client.state["sample_seq"] += 2
    return copy.deepcopy(session.client.state)


class FakeSession(cr.RandomSession):
    """Real Session.execute/guard/finally with fake publish/completion transport."""
    def __init__(self, clock, fail_leg=None, bad_final_hold=False, final_yaw_error_deg=0.):
        super().__init__(SimpleNamespace(usable=True, state=held()), {}, Log())
        self._firmware_verified = True
        self.calls, self.clock = [], clock
        self.fail_leg, self.bad_final_hold = fail_leg, bad_final_hold
        self.final_yaw_error_deg = final_yaw_error_deg
        self.motion_count = 0

    def publish(self, command, args):
        self.stopping = command == 1
        self.guard()
        self.calls.append((command, args))
        self.command, self.args = command, args
        self.heartbeat = 12
        if command == 7:
            self.motion_count += 1
        return len(self.calls)

    def wait_complete(self, sequence, timeout, stop=False, expected_command=None):
        if self.command == 7:
            if self.motion_count == self.fail_leg:
                raise cr.swd.SessionError("Firmware fault 9")
            self.clock[0] += 3.
            pose = list(self.args[:3])
            if self.motion_count == 11:
                pose[2] = cr.wrap(pose[2] + math.radians(self.final_yaw_error_deg))
            self.client.state = held(pose)
            return self.snapshot()
        self.client.state["command"] = 1
        self.client.state["sample"]["enabled"] = 0
        return self.snapshot()

    def snapshot(self):
        if self.bad_final_hold and self.motion_count == 11 and self.heartbeat is None:
            self.client.state["sample"]["target_2"] = .01
        return super().snapshot()


class ContinuousRandomTests(unittest.TestCase):
    def test_seed_route_is_reproducible_bounded_and_has_large_round_trips(self):
        route = cr.make_route()
        self.assertEqual(route, cr.make_route())
        metrics = cr.validate_plan(route)
        self.assertEqual(len(route), 11)
        self.assertTrue(4 <= metrics["path_m"] <= 6)
        self.assertLessEqual(metrics["max_leg_m"], .95)
        self.assertGreater(max(p[0] for p in route), .8)
        self.assertTrue(any(route[i + 1][0] < route[i][0] - .4 for i in range(10)))
        self.assertTrue(any(abs(route[i + 1][1] - route[i][1]) > .3 for i in range(10)))
        self.assertTrue(all(45 <= abs(math.degrees(p[2])) <= 60 for p in route[:-1]))
        self.assertLess(metrics["angle_rad"], 6 * math.pi)
        self.assertTrue(45 < metrics["estimated_seconds"] < 70)

    def test_invalid_route_region_leg_and_total_budget_are_rejected(self):
        for route in ([[[1.06, 0, 0], [0., 0., 0.]]],
                      [[[.1, .26, 0], [0., 0., 0.]]],
                      [[[.96, 0, 0], [0., 0., 0.]]],
                      [[.9, 0., 0.], [0., 0., 0.]] * 5):
            # First three inputs are wrapped to keep this table readable.
            if len(route) == 1:
                route = route[0]
            with self.subTest(route=route), self.assertRaises(cr.swd.SessionError):
                cr.validate_plan(route)

    def test_long_profile_extends_moves_without_expanding_floor_fence(self):
        route = cr.make_route(profile="long")
        self.assertEqual(route, cr.make_route(profile="long"))
        self.assertNotEqual(route, cr.make_route(seed=2, profile="long"))
        metrics = cr.validate_plan(route, "long")
        self.assertEqual(len(route), 9)
        self.assertTrue(all(.8 <= leg <= .95 for leg in metrics["legs_m"][:-1]))
        self.assertTrue(all(abs(point[2]) == math.pi / 2 for point in route[:-1]))
        self.assertLess(metrics["path_m"], 9)
        self.assertAlmostEqual(metrics["angle_rad"], 8 * math.pi)
        self.assertTrue(60 < metrics["estimated_seconds"] < 80)
        with self.assertRaises(cr.swd.SessionError):
            cr.validate_plan(route)  # Default profile retains the original limits.

    def test_long_profile_executes_once_and_keeps_the_same_hold_protocol(self):
        clock = [0.]
        session = FakeSession(clock)
        with patch.object(cr.swd.Session, "snapshot", read_fake), \
             patch.object(cr.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(cr.time, "sleep", side_effect=lambda dt: clock.__setitem__(0, clock[0] + dt)):
            report = cr.run_mission(session, cr.make_route(profile="long"), profile="long")
        self.assertEqual([command for command, _ in session.calls], [7] * 9)
        self.assertEqual(session.deadline, 120.)
        self.assertEqual(report["profile"], "long")
        self.assertTrue(report["motors_enabled"])
        self.assertLess(report["position_error_m"], 1e-12)

    def test_rotated_reference_and_pi_wrap_preserve_body_coordinates(self):
        origin = [2., -3., math.radians(179)]
        local = [.8, -.2, math.radians(60)]
        world = cr.world_goal(origin, local)
        self.assertTrue(-math.pi <= world[2] <= math.pi)
        for actual, expected in zip(cr.local_pose(origin, world), local):
            self.assertAlmostEqual(actual, expected)

    def test_entire_mission_uses_only_cmd7_and_keeps_enabled(self):
        clock = [0.]
        session = FakeSession(clock)
        with patch.object(cr.swd.Session, "snapshot", read_fake), \
             patch.object(cr.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(cr.time, "sleep", side_effect=lambda dt: clock.__setitem__(0, clock[0] + dt)):
            report = cr.run_mission(session, cr.make_route())
        self.assertEqual([command for command, _ in session.calls], [7] * 11)
        self.assertTrue(report["motors_enabled"])
        self.assertFalse(report["real_ground_return_verified"])
        self.assertLess(report["position_error_m"], 1e-12)
        self.assertIsNone(session.heartbeat)
        self.assertEqual(session.client.state["sample"]["enabled"], 1)
        self.assertGreaterEqual(report["elapsed_seconds"], 33.5)
        self.assertLess(report["requested_path_m"], cr.LINEAR_BUDGET)

    def test_fault_and_bad_final_hold_stop_without_motion_retry(self):
        for kwargs, count in ((dict(fail_leg=3), 3), (dict(bad_final_hold=True), 11)):
            clock = [0.]
            session = FakeSession(clock, **kwargs)
            with self.subTest(kwargs=kwargs), patch.object(cr.swd.Session, "snapshot", read_fake), \
                 patch.object(cr.time, "monotonic", side_effect=lambda: clock[0]), \
                 self.assertRaises(cr.swd.SessionError):
                cr.run_mission(session, cr.make_route())
            commands = [command for command, _ in session.calls]
            self.assertEqual(commands.count(7), count)
            self.assertEqual(commands[-1], 1)
            self.assertEqual(session.client.state["sample"]["enabled"], 0)

    def test_final_home_accepts_six_degree_error_but_pose_consistency_remains_five(self):
        clock = [0.]
        session = FakeSession(clock, final_yaw_error_deg=6.)
        with patch.object(cr.swd.Session, "snapshot", read_fake), \
             patch.object(cr.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(cr.time, "sleep", side_effect=lambda dt: clock.__setitem__(0, clock[0] + dt)):
            report = cr.run_mission(session, cr.make_route())
        self.assertAlmostEqual(report["yaw_error_deg"], 6.)
        self.assertEqual([command for command, _ in session.calls], [7] * 11)
        inconsistent = held()
        inconsistent["sample"]["yaw_deg"] += 6.
        with self.assertRaisesRegex(cr.swd.SessionError, "inconsistent"):
            cr.fresh_pose(inconsistent)

    def test_global_fence_uses_initial_heading_and_latches_stop_context(self):
        session = FakeSession([0.])
        session.origin = [1., 2., math.radians(137)]
        session.client.state = held(cr.world_goal(session.origin, [.4, .401, 0.]))
        with patch.object(cr.swd.Session, "snapshot", read_fake):
            with self.assertRaisesRegex(cr.swd.SessionError, "fence"):
                session.snapshot()
            self.assertTrue(session.stopping)
            # Cleanup read/STOP must not be blocked by the same fence.
            session.snapshot()
            session.execute(1, [0.] * 8, 2.5)
        self.assertEqual(session.calls[-1][0], 1)

    def test_deadline_stop_file_and_stale_pose_allow_cleanup(self):
        with tempfile.TemporaryDirectory() as folder:
            stop_file = Path(folder) / "STOP"
            for mode in ("deadline", "stop_file", "stale"):
                session = FakeSession([0.])
                session.origin = [1., 2., math.radians(137)]
                if mode == "deadline":
                    session.deadline = -1.
                elif mode == "stop_file":
                    stop_file.touch()
                    session.stop_file = stop_file
                else:
                    session.client.state["sample"]["imu_age_ms"] = 21
                with self.subTest(mode=mode), patch.object(cr.swd.Session, "snapshot", read_fake):
                    with self.assertRaises(cr.swd.SessionError):
                        session.snapshot()
                    self.assertTrue(session.stopping)
                    session.execute(1, [0.] * 8, 2.5)
                self.assertEqual(session.calls[-1][0], 1)

    def test_default_describe_never_connects(self):
        with patch.object(cr.swd, "TclClient") as client, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(cr.main([]), 0)
        client.assert_not_called()

    def test_initial_must_be_fresh_enabled_hold(self):
        for key, value in (("enabled", 0), ("odom_valid", 0), ("target_1", .1), ("yaw_deg", 0.)):
            state = held()
            state["sample"][key] = value
            with self.subTest(key=key), self.assertRaises(cr.swd.SessionError):
                cr.hold_pose(state)


if __name__ == "__main__":
    unittest.main()
