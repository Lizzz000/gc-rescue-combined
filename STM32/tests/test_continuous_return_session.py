"""Offline checkpoint/frame/hold tests; never connects to hardware."""
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/live_tuning"))
import continuous_return_session as cr


class Log:
    def __init__(self):
        self.records = []

    def emit(self, event, **data):
        self.records.append((event, data))


def disabled(x=.4, y=0., yaw=0., fresh_yaw=0., tick=1000.):
    return dict(command=1, phase=4, error=0, request_seq=0, ack_seq=0, sample={
        "phase": 4, "error": 0, "enabled": 0, "state_1": 0, "state_2": 0, "state_3": 0,
        "target_1": 0., "target_2": 0., "target_3": 0., "odom_x_m": x, "odom_y_m": y,
        "odom_yaw_rad": yaw, "yaw_deg": math.degrees(fresh_yaw), "imu_age_ms": 2., "tick_ms": tick})


def logs(folder):
    mission = Path(folder) / "mission.jsonl"
    checkpoint = Path(folder) / "checkpoint.jsonl"
    rows = [dict(event="firmware_verified", elf_sha256="old"),
            dict(event="mission_origin", pose=[0., 0., .02]),
            dict(event="complete", **disabled()), dict(event="mission_final_stop_confirmed")]
    mission.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    rows = [dict(event="firmware_verified", elf_sha256="old"),
            dict(event="sample", **disabled(fresh_yaw=math.radians(13), tick=300000))]
    checkpoint.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    return mission, checkpoint


class FakeSession:
    def __init__(self, bad_hold=False):
        self.log, self.calls = Log(), []
        self._firmware_verified = self._initial_validated = True
        self.heartbeat, self.bad_hold = 123, bad_hold
        self.initial = disabled(x=0., fresh_yaw=math.pi / 2)
        self.initial.update(phase=0, command=0)
        self.initial["sample"]["phase"] = 0

    def validate_initial(self):
        return self.initial

    def execute(self, command, args, timeout):
        self.calls.append((command, args))
        if command == 7:
            self.result = dict(command=7, phase=4, error=0, sample={
                "phase": 4, "error": 0, "enabled": 1, "state_1": 1, "state_2": 1, "state_3": 1,
                "target_1": 0, "target_2": 0, "target_3": 0, "odom_valid": 1,
                "end_x_m": args[0], "end_y_m": args[1], "end_yaw_rad": args[2],
                "end_valid": 1, "end_stopped": 1, "end_elapsed_ms": 2000,
                "end_progress_m": .4, "end_target_distance_m": .4})
            return self.result
        return disabled()

    def snapshot(self):
        if self.bad_hold:
            self.result["sample"]["target_2"] = .1
        return self.result


class ContinuousReturnTests(unittest.TestCase):
    def test_continuous_mission_failure_uses_saved_original_origin_and_same_hash(self):
        with tempfile.TemporaryDirectory() as folder:
            mission, checkpoint = logs(folder)
            rows = [json.loads(line) for line in mission.read_text().splitlines()]
            rows[1].update(event="continuous_random_origin", pose=[-.35, .04, -.15])
            rows[-1]["event"] = "mission_failure_stop_confirmed"
            mission.write_text("\n".join(json.dumps(row) for row in rows), encoding="utf-8")
            saved = cr.load_checkpoint(mission, checkpoint)
            self.assertEqual(saved["old_origin"], [-.35, .04, -.15])
            self.assertEqual(saved["old_current"], [.4, 0., 0.])
            self.assertAlmostEqual(cr.map_goal(saved, [0., 0., 0.])["body_delta_m"][0], -.75)
            bad = [json.loads(line) for line in checkpoint.read_text().splitlines()]
            bad[0]["elf_sha256"] = "different"
            checkpoint.write_text("\n".join(json.dumps(row) for row in bad), encoding="utf-8")
            with self.assertRaisesRegex(cr.swd.SessionError, "provenance"):
                cr.load_checkpoint(mission, checkpoint)

    def test_checkpoint_uses_frozen_yaw_not_idle_imu_drift(self):
        with tempfile.TemporaryDirectory() as folder:
            mission, checkpoint = logs(folder)
            saved = cr.load_checkpoint(mission, checkpoint)
        self.assertEqual(saved["old_current"], [.4, 0., 0.])
        self.assertAlmostEqual(saved["idle_imu_drift_deg"], 13.)
        mapped = cr.map_goal(saved, [1., 2., math.pi / 2])
        self.assertAlmostEqual(mapped["new_world_goal"][0], 1.)
        self.assertAlmostEqual(mapped["new_world_goal"][1], 1.6)
        self.assertAlmostEqual(mapped["new_world_goal"][2], math.pi / 2 + .02)
        self.assertTrue(saved["stationary_across_reflash_assumed"])

    def test_changed_checkpoint_pose_or_bad_disabled_state_is_rejected(self):
        for key, value in (("odom_x_m", .41), ("odom_yaw_rad", .1), ("state_1", 1),
                           ("target_3", .01), ("imu_age_ms", 100)):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as folder:
                mission, checkpoint = logs(folder)
                rows = [json.loads(line) for line in checkpoint.read_text().splitlines()]
                rows[-1]["sample"][key] = value
                checkpoint.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
                with self.assertRaises(cr.swd.SessionError):
                    cr.load_checkpoint(mission, checkpoint)

    def test_mapping_handles_nonzero_old_frame_and_angle_wrap(self):
        yaw = math.radians(137)
        checkpoint = dict(old_origin=[0, 0, math.radians(-179)],
                          old_current=[.4 * math.cos(yaw), .4 * math.sin(yaw), yaw])
        mapped = cr.map_goal(checkpoint, [0, 0, math.radians(179)])
        self.assertAlmostEqual(mapped["body_delta_m"][0], -.4)
        self.assertAlmostEqual(mapped["body_delta_m"][1], 0.)
        self.assertGreaterEqual(mapped["new_world_goal"][2], -math.pi)
        self.assertLessEqual(mapped["new_world_goal"][2], math.pi)
        with self.assertRaises(cr.swd.SessionError):
            cr.map_goal(dict(old_origin=[0, 0, 0], old_current=[1.01, 0, 0]), [0, 0, 0])

    def test_default_describe_is_offline(self):
        with tempfile.TemporaryDirectory() as folder:
            mission, checkpoint = logs(folder)
            with patch.object(cr.swd, "TclClient") as client, contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cr.main(["--mission-log", str(mission), "--checkpoint-log", str(checkpoint)]), 0)
            client.assert_not_called()

    def test_single_return_keeps_enable_and_does_not_replay_or_reset(self):
        session, clock = FakeSession(), [0.]
        checkpoint = dict(old_origin=[0, 0, 0], old_current=[.4, 0, 0])
        with patch.object(cr.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(cr.time, "sleep", side_effect=lambda dt: clock.__setitem__(0, clock[0] + dt)):
            result = cr.run_return(session, checkpoint)
        self.assertEqual([command for command, _ in session.calls], [7])
        self.assertTrue(result["motors_enabled"])
        self.assertFalse(result["real_ground_return_verified"])
        self.assertIsNone(session.heartbeat)
        self.assertAlmostEqual(session.calls[0][1][0], 0.)
        self.assertAlmostEqual(session.calls[0][1][1], -.4)

    def test_hold_failure_disables_but_never_retries_return(self):
        session = FakeSession(bad_hold=True)
        with self.assertRaises(cr.swd.SessionError):
            cr.run_return(session, dict(old_origin=[0, 0, 0], old_current=[.4, 0, 0]))
        self.assertEqual([command for command, _ in session.calls], [7, 1])

    def test_previously_operated_new_frame_cannot_invent_an_origin(self):
        session = FakeSession()
        session.initial["request_seq"] = 1
        with self.assertRaisesRegex(cr.swd.SessionError, "fresh boot"):
            cr.run_return(session, dict(old_origin=[0, 0, 0], old_current=[.4, 0, 0]))
        self.assertEqual(session.calls, [])


if __name__ == "__main__":
    unittest.main()
