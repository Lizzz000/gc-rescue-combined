"""Return once to a saved physical-origin estimate after reflashing, then hold enable.

Default is offline description. The frozen old odometry pose is mapped through
the current body frame into the new firmware's IMU frame. This assumes the
vehicle was not physically moved between the checkpoint and the new boot.
It deliberately does not replay the random route or reset a new home position.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import swd_session as swd


def wrap(value):
    return math.remainder(value, 2 * math.pi)


def finite(values):
    return all(isinstance(value, (int, float)) and math.isfinite(value) for value in values)


def read_log(path):
    data = Path(path).read_bytes()
    records = [json.loads(line) for line in data.decode("utf-8-sig").splitlines() if line.strip()]
    return records, hashlib.sha256(data).hexdigest()


def disabled_pose(state):
    sample = state.get("sample", {})
    values = [sample.get(key) for key in ("odom_x_m", "odom_y_m", "odom_yaw_rad", "yaw_deg", "imu_age_ms", "tick_ms")]
    if (not finite(values) or not 0 <= sample["imu_age_ms"] <= 20 or
        state.get("phase") not in (0, 4) or state.get("error") != 0 or
        sample.get("enabled") != 0 or any(sample.get(f"state_{i}") != 0 for i in (1, 2, 3)) or
        any(sample.get(f"target_{i}") != 0 for i in (1, 2, 3))):
        raise swd.SessionError("Checkpoint/new boot lacks disabled zero-target pose and fresh IMU evidence")
    return values[:3]


def load_checkpoint(mission_path, checkpoint_path):
    mission, mission_sha = read_log(mission_path)
    checkpoint, checkpoint_sha = read_log(checkpoint_path)
    origins = [r["pose"] for r in mission if r.get("event") in ("mission_origin", "continuous_random_origin")]
    if len(origins) != 1 or len(origins[0]) != 3 or not finite(origins[0]):
        raise swd.SessionError("Mission log must contain exactly one finite original pose")
    if not any(r.get("event") in ("mission_final_stop_confirmed", "mission_failure_stop_confirmed") for r in mission):
        raise swd.SessionError("Original mission lacks final STOP confirmation")
    stopped = [r for r in mission if r.get("event") == "complete" and r.get("command") == 1]
    samples = [r for r in checkpoint if r.get("event") == "sample"]
    old_hashes = [r["elf_sha256"] for r in mission if r.get("event") == "firmware_verified"]
    checkpoint_hashes = [r["elf_sha256"] for r in checkpoint if r.get("event") == "firmware_verified"]
    if not stopped or not samples or not old_hashes or not checkpoint_hashes or old_hashes[-1] != checkpoint_hashes[-1]:
        raise swd.SessionError("Missing/mismatched old firmware and checkpoint provenance")
    old_pose, saved_pose = disabled_pose(stopped[-1]), disabled_pose(samples[-1])
    if math.hypot(old_pose[0] - saved_pose[0], old_pose[1] - saved_pose[1]) > 1e-6 or abs(wrap(old_pose[2] - saved_pose[2])) > 1e-6:
        raise swd.SessionError("Checkpoint frozen pose changed since final STOP; cannot reuse the original frame")
    sample = samples[-1]["sample"]
    return dict(old_origin=origins[0], old_current=saved_pose,
                stationary_across_reflash_assumed=True, real_ground_return_verified=False,
                old_fresh_imu_yaw_rad=math.radians(sample["yaw_deg"]),
                idle_imu_drift_deg=math.degrees(wrap(math.radians(sample["yaw_deg"]) - saved_pose[2])),
                checkpoint_tick_ms=sample["tick_ms"], old_elf_sha256=old_hashes[-1],
                mission_log=str(Path(mission_path).resolve()), mission_log_sha256=mission_sha,
                checkpoint_log=str(Path(checkpoint_path).resolve()), checkpoint_log_sha256=checkpoint_sha)


def map_goal(checkpoint, new_pose):
    if len(new_pose) != 3 or not finite(new_pose):
        raise swd.SessionError("New pose must contain three finite values")
    origin, current = checkpoint["old_origin"], checkpoint["old_current"]
    dx, dy = origin[0] - current[0], origin[1] - current[1]
    c, s = math.cos(current[2]), math.sin(current[2])
    bx, by = c * dx + s * dy, -s * dx + c * dy
    if math.hypot(bx, by) > 1.0:
        raise swd.SessionError("Saved return displacement exceeds the command 7 one-metre limit")
    c, s = math.cos(new_pose[2]), math.sin(new_pose[2])
    goal = [new_pose[0] + c * bx - s * by, new_pose[1] + s * bx + c * by,
            wrap(new_pose[2] + wrap(origin[2] - current[2]))]
    return dict(body_delta_m=[bx, by], relative_yaw_rad=wrap(origin[2] - current[2]),
                new_start_pose=list(new_pose), new_world_goal=goal)


class ContinuousSession(swd.Session):
    def __init__(self, *args, stop_file=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.stop_file, self.start_pose = stop_file, None
        self.stopping, self.guard_failure = False, None

    def guard(self):
        if self.stopping:
            return
        if self.stop_file is not None and self.stop_file.exists():
            self.guard_failure = "Operator stop file detected"
        if self.guard_failure:
            self.stopping = True
            raise swd.SessionError(self.guard_failure)

    def snapshot(self):
        try:
            self.guard()
            state = super().snapshot()
            if not self.stopping:
                if state["error"] or state["phase"] == 5:
                    raise swd.SessionError(f"Firmware fault {state['error']}")
                sample = state["sample"]
                if (state["command"] == 7 and sample["phase"] == state["phase"] and
                    state["phase"] in (2, 3, 4) and sample["enabled"] == 1):
                    values = [sample.get(key) for key in ("odom_x_m", "odom_y_m", "odom_yaw_rad", "odom_age_ms")]
                    if not finite(values) or sample.get("odom_valid") != 1 or not 0 <= values[3] <= 50:
                        raise swd.SessionError("Continuous return odometry is invalid/stale")
                    if self.start_pose and math.hypot(values[0] - self.start_pose[0], values[1] - self.start_pose[1]) > 1.3:
                        raise swd.SessionError("Continuous return exceeded the 1.3 metre estimated fence")
            return state
        except (OSError, swd.SessionError) as exc:
            self.guard_failure, self.stopping = str(exc), True
            raise

    def beat(self, force=False):
        self.guard()
        try:
            return super().beat(force)
        except (OSError, swd.SessionError):
            self.stopping = True
            raise

    def publish(self, command, args):
        self.stopping = command == 1
        self.guard()
        try:
            return super().publish(command, args)
        except (OSError, swd.SessionError):
            self.stopping = True
            raise

    def execute(self, command, args, timeout):
        self.stopping = command == 1
        self.guard()
        return super().execute(command, args, timeout)


def run_return(session, checkpoint, linear_limit=.2, angular_limit=.5, duration_ms=20000):
    initial = session.validate_initial()
    old_new_pose = disabled_pose(initial)
    # Reusing an already-operated new firmware frame could map a moved vehicle
    # back to an invented home. This entry is specifically for a fresh reflash.
    if initial.get("request_seq") != 0 or initial.get("ack_seq") != 0 or initial.get("command") != 0:
        raise swd.SessionError("Expected a fresh boot with no previous motor command")
    new_pose = [old_new_pose[0], old_new_pose[1], wrap(math.radians(initial["sample"]["yaw_deg"]))]
    mapped = map_goal(checkpoint, new_pose)
    args = [*mapped["new_world_goal"], linear_limit, angular_limit, float(duration_ms), 0., 0.]
    swd.validate_request(7, args)
    session.start_pose = new_pose
    session.log.emit("reflash_frame_mapping", **checkpoint, **mapped,
                     new_tick_ms=initial["sample"]["tick_ms"])
    completed = False
    try:
        result = session.execute(7, args, duration_ms / 1000 + 3)
        # HOLD intentionally needs no heartbeat. Check that zero target/enable
        # survives a 0.3 second read-only observation before normal process exit.
        session.heartbeat = None
        end = time.monotonic() + .3
        while True:
            result = session.snapshot()
            swd.validate_endpoint(7, args, result)
            session.log.emit("enabled_hold_observation", **result)
            if time.monotonic() >= end:
                break
            time.sleep(.05)
        report = dict(estimated_only=True, real_ground_return_verified=False,
                      stationary_across_reflash_assumed=True, motors_enabled=True, zero_targets=True,
                      position_error_m=math.hypot(result["sample"]["end_x_m"] - args[0], result["sample"]["end_y_m"] - args[1]),
                      yaw_error_deg=math.degrees(wrap(result["sample"]["end_yaw_rad"] - args[2])),
                      mapping=mapped)
        session.log.emit("continuous_return_complete_enabled", **report)
        completed = True
        return report
    finally:
        if not completed and session._firmware_verified and session._initial_validated:
            # execute already stops its own failures; this also covers failures
            # during the post-success hold observation, with no movement retry.
            failure = sys.exc_info()[1]
            try:
                session.execute(1, [0.] * 8, 2.5)
            except BaseException as exc:
                session.log.emit("continuous_cleanup_stop_unconfirmed", message=str(exc))
                if failure is None:
                    raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--describe", action="store_true")
    action.add_argument("--execute", action="store_true")
    parser.add_argument("--mission-log", type=Path, required=True)
    parser.add_argument("--checkpoint-log", type=Path, required=True)
    parser.add_argument("--new-pose", type=float, nargs=3, help="Offline assumed new x/y/yaw only")
    parser.add_argument("--elf", type=Path)
    parser.add_argument("--nm")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--stop-file", type=Path)
    parser.add_argument("--linear-limit", type=float, default=.2)
    parser.add_argument("--angular-limit", type=float, default=.5)
    parser.add_argument("--duration-ms", type=int, default=20000)
    args = parser.parse_args(argv)
    checkpoint = load_checkpoint(args.mission_log, args.checkpoint_log)
    if not args.execute:
        print(json.dumps(dict(offline=True, **checkpoint, **map_goal(checkpoint, args.new_pose or [0., 0., 0.])), indent=2))
        return 0
    if not args.elf or not args.log or args.new_pose is not None:
        parser.error("Execution requires exact --elf and new --log; --new-pose is offline only")
    if args.stop_file and args.stop_file.exists():
        parser.error("Stop file already exists")
    client = log = session = None
    try:
        symbols = swd.load_symbols(args.elf, args.nm)
        new_sha = hashlib.sha256(args.elf.read_bytes()).hexdigest()
        if new_sha == checkpoint["old_elf_sha256"]:
            raise swd.SessionError("Expected the new command 7 firmware, not the pre-reflash ELF")
        with swd.writer_lock("127.0.0.1", 6666):
            log = swd.JsonLog(args.log)
            client = swd.TclClient("127.0.0.1", 6666)
            if client.evaluate("stm32f4x.cpu curstate") != "running":
                raise swd.SessionError("MCU is not running")
            session = ContinuousSession(client, symbols, log, stop_file=args.stop_file)
            session.verify_firmware(args.elf)
            report = run_return(session, checkpoint, args.linear_limit, args.angular_limit, args.duration_ms)
            print(json.dumps(report, indent=2))
            return 0
    except (OSError, swd.SessionError, KeyboardInterrupt) as exc:
        if log:
            log.emit("session_error", message=str(exc), kind=type(exc).__name__)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if client:
            client.close()
        if log:
            log.close()


if __name__ == "__main__":
    raise SystemExit(main())
