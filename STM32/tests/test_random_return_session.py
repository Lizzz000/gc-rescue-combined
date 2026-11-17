"""Fake-only random mission/guard tests: no probe, serial port or network."""
import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

FOLDER = Path(__file__).resolve().parents[1] / "tools/live_tuning"
sys.path.insert(0, str(FOLDER))
import random_return_session as rr


class Log:
    def __init__(self):
        self.records = []

    def emit(self, event, **data):
        self.records.append((event, data))


def state(x=0., y=0., yaw=0., running=False):
    sample = dict(phase=2 if running else 4, error=0, enabled=int(running),
                  odom_x_m=x, odom_y_m=y, yaw_deg=math.degrees(yaw), imu_age_ms=1,
                  odom_yaw_rad=yaw, odom_vx_mps=.1, odom_vy_mps=0.,
                  odom_wheel_wz_rad_s=0., odom_age_ms=1., odom_valid=int(running),
                  state_1=int(running), state_2=int(running), state_3=int(running))
    return dict(phase=sample["phase"], error=0, command=4, sample_seq=2, sample=sample)


class FakeSession:
    def __init__(self, clock, yaw=0., fail_on=None, gain=.9, direction_offset=0.):
        self.clock, self.initial_yaw, self.gain, self.fail_on = clock, yaw, gain, fail_on
        self.x = self.y = 0.
        self.yaw = yaw
        self.direction_offset = direction_offset
        self.calls, self.moves = [], 0
        self.origin = self.deadline = None
        self.log = Log()
        self._firmware_verified = self._initial_validated = True
        self.motion_samples = 1

    def abort(self, reason):
        raise rr.swd.SessionError(reason)

    check_fence = rr.GuardedSession.check_fence

    def execute(self, command, args, timeout):
        self.calls.append((command, args))
        if command == 4:
            self.moves += 1
            if self.moves == self.fail_on:
                raise rr.swd.SessionError("injected firmware fault")
            duration = args[3] / 1000
            self.x += self.gain * args[0] * duration * math.cos(self.yaw + self.direction_offset)
            self.y += self.gain * args[0] * duration * math.sin(self.yaw + self.direction_offset)
            self.yaw = rr.wrap(self.yaw + self.gain * args[2] * duration)
            self.clock[0] += duration + .3
        elif command == 5:
            self.x = self.y = 0.
            self.clock[0] += .8
        else:
            self.clock[0] += .2
        return state(self.x, self.y, self.yaw)


class RandomReturnTests(unittest.TestCase):
    def test_default_describe_never_connects(self):
        output = io.StringIO()
        with patch.object(rr.swd, "TclClient") as client, contextlib.redirect_stdout(output):
            self.assertEqual(rr.main([]), 0)
        client.assert_not_called()
        self.assertEqual(json.loads(output.getvalue())["profile"]["name"], "low")

    def test_plan_repeatability_bounds_and_first_two_forward(self):
        pulses, predicted = rr.departure_plan()
        self.assertEqual((pulses, predicted), rr.departure_plan())
        self.assertEqual(sum(p.vx != 0 for p in pulses), 8)
        self.assertEqual(sum(p.wz != 0 for p in pulses), 2)
        self.assertTrue(any(p.vx < 0 for p in pulses))
        self.assertTrue(all(p.vx > 0 for p in pulses[:2]))
        self.assertLessEqual(sum(abs(p.vx) * p.duration_ms / 1000 for p in pulses), .5)
        self.assertTrue(all(0 <= x <= .5 and abs(y) <= .15 for x, y, _ in predicted))
        for pulse in pulses:
            pulse.validate()

    def test_return_uses_reverse_to_avoid_half_turn_and_wraps_heading(self):
        pulse = rr.next_return_pulse((.2, 0, 0), (0, 0, 0))
        self.assertLess(pulse.vx, 0)
        self.assertEqual(pulse.wz, 0)
        self.assertIsNone(rr.next_return_pulse((0, 0, math.radians(-179)), (0, 0, math.radians(179))))
        yaw = math.radians(137)
        pulse = rr.next_return_pulse((.2 * math.cos(yaw), .2 * math.sin(yaw), yaw), (0, 0, yaw))
        self.assertLess(pulse.vx, 0)
        self.assertEqual(pulse.wz, 0)

    def test_complete_fake_mission_from_nonzero_initial_yaw(self):
        clock = [0.]
        session = FakeSession(clock, yaw=math.radians(137))
        with patch.object(rr.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(rr.time, "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)):
            report = rr.Mission(session).run()
        self.assertLessEqual(report["estimated_position_error_m"], .02)
        self.assertLessEqual(abs(report["estimated_yaw_error_deg"]), 5)
        self.assertLessEqual(report["actions"], 24)
        self.assertFalse(report["real_ground_return_verified"])
        self.assertEqual(sum(command == 5 for command, _ in session.calls), 1)
        self.assertNotIn(6, [command for command, _ in session.calls])
        self.assertEqual(session.calls[-1][0], 1)

    def test_firmware_fault_no_displacement_and_reverse_abort_without_retry(self):
        for fail_on, gain, offset in ((2, .9, 0.), (None, 0., 0.), (None, -1., 0.), (None, 1., math.pi / 2)):
            clock = [0.]
            session = FakeSession(clock, fail_on=fail_on, gain=gain, direction_offset=offset)
            with self.subTest(fail_on=fail_on, gain=gain), \
                 patch.object(rr.time, "monotonic", side_effect=lambda: clock[0]), \
                 patch.object(rr.time, "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)), \
                 self.assertRaises(rr.swd.SessionError):
                rr.Mission(session).run()
            self.assertEqual(session.moves, fail_on or 1)
            self.assertEqual(session.calls[-1][0], 1)

    def test_final_stop_failure_preserves_original_firmware_failure(self):
        clock = [0.]
        session = FakeSession(clock, fail_on=1)
        real_execute = session.execute

        def fail_stop(command, args, timeout):
            if command == 1:
                raise rr.swd.SessionError("injected final STOP failure")
            return real_execute(command, args, timeout)

        session.execute = fail_stop
        with patch.object(rr.time, "monotonic", side_effect=lambda: clock[0]), \
             self.assertRaisesRegex(rr.swd.SessionError, "injected firmware fault"):
            rr.Mission(session).run()
        self.assertIn("mission_final_stop_unconfirmed", [event for event, _ in session.log.records])

    def test_tcp_failure_enters_stop_context_before_reconnect_preflight(self):
        symbols = {"tuning_" + name: index * 4 for index, name in enumerate(
                   ("request_seq", "heartbeat", "args", "command"))}

        class Client:
            usable = True
            fail = True
            writes = []

            def read_words(self, address):
                return [0]

            def write_words(self, address, words):
                if address == symbols["tuning_args"] and self.fail:
                    self.fail = False
                    self.usable = False
                    raise OSError("injected TCP failure")
                self.writes.append((address, words))

            def reconnect(self):
                self.usable = True

        clock = [0.]
        client = Client()
        guard = rr.GuardedSession(client, symbols, Log())
        guard._firmware_verified = True
        guard.deadline = 60

        def reverify(elf):
            clock[0] = 100  # The mission expired while rechecking Flash.
            self.assertTrue(guard.stopping)
            guard._firmware_verified = True
            guard._initial_validated = False

        with patch.object(rr.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(rr.swd.Session, "snapshot", return_value=state()), \
             patch.object(guard, "verify_firmware", side_effect=reverify), \
             patch.object(guard, "wait_complete", return_value=state()), \
             self.assertRaisesRegex(OSError, "injected TCP failure"):
            guard.execute(4, rr.Pulse(vx=.1).arguments(), 3)
        self.assertIn((symbols["tuning_command"], [1]), client.writes)
        self.assertIn("final_stop_confirmed", [event for event, _ in guard.log.records])

    def test_guard_latches_failure_but_allows_stop_and_blocks_new_motion(self):
        guard = rr.GuardedSession(None, {}, Log())
        guard.origin = (0, 0, math.pi / 2)
        with self.assertRaisesRegex(rr.swd.SessionError, "geofence"):
            guard.inspect(state(0, .7, math.pi / 2, True))
        self.assertTrue(guard.stopping)
        guard.inspect(state(0, .7, math.pi / 2, True))  # shutdown reads remain possible
        with patch.object(rr.swd.Session, "execute", return_value={}) as execute:
            guard.execute(1, [0.] * 8, 2.5)
            with self.assertRaises(rr.swd.SessionError):
                guard.execute(4, rr.Pulse(vx=.1).arguments(), 3)
        self.assertEqual(execute.call_count, 1)

    def test_invalid_run_feedback_and_budget_expiry_latch_guard(self):
        for bad in ("odom_valid", "odom_x_m", "odom_age_ms"):
            guard = rr.GuardedSession(None, {}, Log())
            current = state(running=True)
            current["sample"][bad] = {"odom_valid": 0, "odom_x_m": float("nan"), "odom_age_ms": 60}[bad]
            with self.subTest(bad=bad), self.assertRaises(rr.swd.SessionError):
                guard.inspect(current)
            self.assertTrue(guard.stopping)
        guard = rr.GuardedSession(None, {}, Log())
        guard.deadline = 10
        with patch.object(rr.time, "monotonic", return_value=10), self.assertRaisesRegex(rr.swd.SessionError, "budget"):
            guard.inspect(state(running=True))

    def test_operator_stop_file_aborts_but_stop_remains_possible(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "STOP"
            guard = rr.GuardedSession(None, {}, Log())
            guard.stop_file = path
            guard.check_deadline()
            path.write_text("operator stop", encoding="utf-8")
            with self.assertRaisesRegex(rr.swd.SessionError, "Operator stop"):
                guard.check_deadline()
            self.assertTrue(guard.stopping)
            guard.check_deadline()
            self.assertTrue(path.exists())
            with patch.object(rr.swd, "TclClient") as client, contextlib.redirect_stderr(io.StringIO()), \
                 self.assertRaises(SystemExit):
                rr.main(["--execute", "--elf", "unused.elf", "--log", str(Path(folder) / "run.jsonl"),
                         "--stop-file", str(path)])
            client.assert_not_called()

    def test_braking_enabled_requires_valid_odometry_but_disabled_does_not(self):
        current = state(running=True)
        current["phase"] = current["sample"]["phase"] = 3
        current["sample"]["odom_valid"] = 0
        guard = rr.GuardedSession(None, {}, Log())
        with self.assertRaisesRegex(rr.swd.SessionError, "odometry"):
            guard.inspect(current)
        guard = rr.GuardedSession(None, {}, Log())
        current["sample"]["enabled"] = 0
        guard.inspect(current)

    def test_insufficient_time_and_integral_caps_do_not_publish_motion(self):
        for field, value, pulse in (("deadline", 1, rr.Pulse(vx=.1, duration_ms=400)),
                                    ("linear_integral", 1.19, rr.Pulse(vx=.1, duration_ms=400)),
                                    ("angular_integral", math.pi - .01, rr.Pulse(wz=.2, duration_ms=400)),
                                    ("actions", 24, rr.Pulse(vx=.1, duration_ms=400))):
            session = FakeSession([0.])
            mission = rr.Mission(session)
            mission.current = mission.origin = (0, 0, 0)
            mission.deadline = 60
            setattr(mission, field, value)
            with self.subTest(field=field), patch.object(rr.time, "monotonic", return_value=0), self.assertRaises(rr.swd.SessionError):
                mission.pulse(pulse, "test")
            self.assertEqual(session.calls, [])

    def test_accelerated_plan_reaches_further_with_bounded_large_turns(self):
        pulses, predicted = rr.departure_plan(profile="accelerated")
        self.assertEqual((pulses, predicted), rr.departure_plan(profile="accelerated"))
        self.assertEqual(sum(p.vx != 0 for p in pulses), 8)
        self.assertEqual(sum(p.wz != 0 for p in pulses), 2)
        self.assertTrue(all(p.vx > 0 for p in pulses[:2]))
        self.assertTrue(any(p.vx < 0 for p in pulses))
        self.assertTrue(all(0 <= x <= 1.1 and abs(y) <= .25 for x, y, _ in predicted))
        self.assertGreaterEqual(max(x for x, _, _ in predicted), .5)
        self.assertLessEqual(sum(abs(p.vx) * p.duration_ms / 1000 for p in pulses), 1.6)
        for pulse in pulses:
            pulse.validate("accelerated")
            if pulse.wz:
                angle = math.degrees(abs(pulse.wz) * pulse.duration_ms / 1000)
                self.assertGreaterEqual(angle, 25)
                self.assertLessEqual(angle, 35)

    def test_profile_limits_do_not_raise_default_limits_or_firmware_duration(self):
        rr.Pulse(vx=.2, duration_ms=1500).validate("accelerated")
        rr.Pulse(wz=.5, duration_ms=1500).validate("accelerated")
        for pulse, profile in ((rr.Pulse(vx=.10001), "low"), (rr.Pulse(wz=.25001), "low"),
                               (rr.Pulse(vx=.20001), "accelerated"), (rr.Pulse(wz=.50001), "accelerated"),
                               (rr.Pulse(vx=.2, duration_ms=1501), "accelerated"),
                               (rr.Pulse(vx=.2, wz=.5), "accelerated")):
            with self.subTest(pulse=pulse, profile=profile), self.assertRaises(rr.swd.SessionError):
                pulse.validate(profile)

    def test_accelerated_return_uses_fast_cruise_and_fine_near_target(self):
        for distance, expected_speed in ((.7, .2), (.2, .15), (.05, .06)):
            pulse = rr.next_return_pulse((distance, 0, 0), (0, 0, 0), "accelerated")
            self.assertEqual(pulse.vx, -expected_speed)
            pulse.validate("accelerated")
        far_angle = rr.next_return_pulse((0, 0, math.radians(45)), (0, 0, 0), "accelerated")
        near_angle = rr.next_return_pulse((0, 0, math.radians(7)), (0, 0, 0), "accelerated")
        self.assertEqual(far_angle.wz, -.5)
        self.assertEqual(near_angle.wz, -.25)

    def test_accelerated_fake_mission_keeps_limits_and_returns_across_yaw_wrap(self):
        clock = [0.]
        session = FakeSession(clock, yaw=math.radians(179))
        with patch.object(rr.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(rr.time, "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)):
            report = rr.Mission(session, profile="accelerated").run()
        self.assertEqual(report["profile"], "accelerated")
        self.assertLessEqual(report["estimated_position_error_m"], .02)
        self.assertLessEqual(abs(report["estimated_yaw_error_deg"]), 5)
        self.assertLessEqual(report["actions"], 24)
        self.assertLessEqual(report["elapsed_s"], 60)
        self.assertLessEqual(report["linear_command_integral_m"], 3)
        self.assertLessEqual(report["angular_command_integral_rad"], 2 * math.pi)
        self.assertEqual(session.calls[0][1][3], 1000)

    def test_accelerated_fence_is_in_initial_heading_coordinates(self):
        yaw = math.radians(137)
        guard = rr.GuardedSession(None, {}, Log(), profile="accelerated")
        guard.origin = (0, 0, yaw)
        x, y = 1.2 * math.cos(yaw) - .3 * math.sin(yaw), 1.2 * math.sin(yaw) + .3 * math.cos(yaw)
        guard.check_fence((x, y, yaw))
        with self.assertRaisesRegex(rr.swd.SessionError, "geofence"):
            guard.check_fence((1.31 * math.cos(yaw), 1.31 * math.sin(yaw), yaw))


if __name__ == "__main__":
    unittest.main()
