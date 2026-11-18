"""Bounded live random-motion/estimated-return session. Defaults to offline describe.

Uses existing firmware commands 5 once (static origin), then 4 (short pure
translation/rotation pulses). Every pulse must stop and disable successfully.
Estimated odometry fences and return do not establish real ground position.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import random
import sys
import time

import swd_session as swd

MAX_ACTIONS = 24
MAX_RETURN_ACTIONS = 12
MAX_SECONDS = 60.0
MAX_LINEAR_INTEGRAL = 1.2
MAX_ANGULAR_INTEGRAL = math.pi
POSITION_TOLERANCE = .02
YAW_TOLERANCE = math.radians(5)


@dataclass(frozen=True)
class Profile:
    name: str
    max_vx: float
    max_wz: float
    planned_x_max: float
    planned_abs_y: float
    departure_linear_budget: float
    minimum_reach: float
    fence_x_max: float
    fence_abs_y: float
    linear_budget: float
    angular_budget: float


PROFILES = {
    "low": Profile("low", .10, .25, .5, .15, .5, 0., .65, .30, 1.2, math.pi),
    "accelerated": Profile("accelerated", .20, .50, 1.1, .25, 1.6, .5, 1.3, .35, 3., 2 * math.pi),
}


def select_profile(profile):
    return PROFILES[profile] if isinstance(profile, str) else profile


def wrap(angle):
    return math.remainder(angle, 2 * math.pi)


def relative_xy(x, y, origin):
    dx, dy = x - origin[0], y - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return c * dx + s * dy, -s * dx + c * dy


@dataclass(frozen=True)
class Pulse:
    vx: float = 0.0
    wz: float = 0.0
    duration_ms: int = 400

    def validate(self, profile="low"):
        profile = select_profile(profile)
        if (not all(math.isfinite(x) for x in (self.vx, self.wz, self.duration_ms)) or
            not 100 <= self.duration_ms <= 1500 or int(self.duration_ms) != self.duration_ms or
            abs(self.vx) > profile.max_vx or abs(self.wz) > profile.max_wz or
            ((self.vx != 0) == (self.wz != 0))):
            raise swd.SessionError("Invalid bounded random-session pulse")
        return self

    def arguments(self, profile="low"):
        self.validate(profile)
        return [self.vx, 0., self.wz, float(self.duration_ms), 0., 0., 0., 0.]


def departure_plan(seed=20261117, profile="low"):
    profile = select_profile(profile)
    accelerated = profile.name == "accelerated"
    rng = random.Random(seed)
    for _ in range(1000):
        pulses, predicted = [], []
        x = y = yaw = linear = 0.
        first_turn_sign = 0
        for index in range(8):
            if index in (2, 5):
                if accelerated:
                    sign = rng.choice((-1, 1)) if index == 2 else -first_turn_sign
                    first_turn_sign = sign if index == 2 else first_turn_sign
                    turn = Pulse(wz=sign * .5, duration_ms=rng.randrange(900, 1201, 100))
                else:
                    turn = Pulse(wz=rng.choice((-.2, .2)), duration_ms=rng.randrange(400, 901, 100))
                pulses.append(turn)
                yaw = wrap(yaw + turn.wz * turn.duration_ms / 1000)
                predicted.append([x, y, yaw])
            speed = rng.choice((.15, .18, .20) if accelerated else (.06, .08, .10))
            sign = 1 if index < 2 or rng.random() < .65 else -1
            pulse = Pulse(vx=sign * speed, duration_ms=rng.randrange(900, 1501, 100) if accelerated else
                          rng.randrange(400, 901, 100))
            delta = pulse.vx * pulse.duration_ms / 1000
            x, y = x + delta * math.cos(yaw), y + delta * math.sin(yaw)
            linear += abs(delta)
            if not 0 <= x <= profile.planned_x_max or abs(y) > profile.planned_abs_y or linear > profile.departure_linear_budget:
                break
            pulses.append(pulse)
            predicted.append([x, y, yaw])
        else:
            if any(p.vx < 0 for p in pulses) and max(p[0] for p in predicted) >= profile.minimum_reach:
                return pulses, predicted
    raise swd.SessionError("Could not generate a bounded departure plan")


def pose(state):
    sample = state["sample"]
    fields = ("odom_x_m", "odom_y_m", "yaw_deg", "imu_age_ms")
    if any(not isinstance(sample.get(key), (int, float)) or not math.isfinite(sample[key]) for key in fields):
        raise swd.SessionError("Missing finite odometry/IMU pose")
    if not 0 <= sample["imu_age_ms"] <= 20:
        raise swd.SessionError("IMU pose is stale")
    return sample["odom_x_m"], sample["odom_y_m"], wrap(math.radians(sample["yaw_deg"]))


class GuardedSession(swd.Session):
    def __init__(self, *args, profile="low", **kwargs):
        super().__init__(*args, **kwargs)
        self.profile = select_profile(profile)
        self.origin = self.deadline = None
        self.guard_failure = None
        self.stopping = False
        self.stop_file = None
        self.motion_samples = 0
        self.last_motion_sample_seq = None

    def abort(self, reason):
        if self.guard_failure is None:
            self.guard_failure = reason
            self.log.emit("guard_fault", reason=reason)
        self.stopping = True
        raise swd.SessionError(self.guard_failure)

    def check_deadline(self):
        if not self.stopping:
            if self.stop_file is not None and self.stop_file.exists():
                self.abort("Operator stop file detected")
            if self.guard_failure is not None:
                self.abort(self.guard_failure)
            if self.deadline is not None and time.monotonic() >= self.deadline:
                self.abort("Global 60 second mission budget expired")

    def check_fence(self, current):
        if self.origin is not None:
            x, y = relative_xy(*current[:2], self.origin)
            if not -.10 <= x <= self.profile.fence_x_max or abs(y) > self.profile.fence_abs_y:
                self.abort(f"Estimated geofence exceeded: x={x:.4f}, y={y:.4f} m")

    def inspect(self, state):
        if self.stopping:
            return state
        self.check_deadline()
        if state["phase"] == 5 or state["error"]:
            self.abort(f"Firmware fault {state['error']}; mission cannot continue")
        sample = state["sample"]
        observed_motion = (state["command"] == 4 and sample.get("phase") == state["phase"] and
                           (state["phase"] == 2 or (state["phase"] == 3 and sample.get("enabled") == 1)))
        if observed_motion:
            fields = ("odom_x_m", "odom_y_m", "odom_yaw_rad", "odom_vx_mps", "odom_vy_mps",
                      "odom_wheel_wz_rad_s", "odom_age_ms")
            if (sample.get("odom_valid") != 1 or
                any(not isinstance(sample.get(key), (int, float)) or not math.isfinite(sample[key]) for key in fields) or
                not 0 <= sample["odom_age_ms"] <= 50):
                self.abort("Invalid or stale odometry during motion")
            try:
                current = pose(state)
            except swd.SessionError as exc:
                self.abort(str(exc))
            self.check_fence(current)
            if state["phase"] == 2 and state["sample_seq"] != self.last_motion_sample_seq:
                self.motion_samples += 1
                self.last_motion_sample_seq = state["sample_seq"]
        return state

    def snapshot(self):
        try:
            return self.inspect(super().snapshot())
        except (OSError, swd.SessionError):
            self.stopping = True
            raise

    def beat(self, force=False):
        self.check_deadline()
        try:
            return super().beat(force)
        except (OSError, swd.SessionError):
            self.stopping = True
            raise

    def publish(self, command, args):
        self.stopping = command == 1
        self.check_deadline()
        try:
            return super().publish(command, args)
        except (OSError, swd.SessionError):
            self.stopping = True
            raise

    def execute(self, command, args, timeout):
        # STOP must bypass the latched mission guard even during validate_initial.
        self.stopping = command == 1
        self.check_deadline()
        if command == 4:
            Pulse(vx=args[0], wz=args[2], duration_ms=args[3]).validate(self.profile)
            self.motion_samples = 0
            self.last_motion_sample_seq = None
        return super().execute(command, args, timeout)


def next_return_pulse(current, origin, profile="low"):
    profile = select_profile(profile)
    accelerated = profile.name == "accelerated"

    def turn(error):
        # Reduce angular speed near the 5 degree arrival boundary.
        rate = (.5 if abs(error) > math.radians(15) else .25) if accelerated else .2
        maximum = 1500 if accelerated else 900
        return Pulse(wz=math.copysign(rate, error),
                     duration_ms=max(200, min(maximum, round(abs(error) * .7 / rate * 1000))))

    dx, dy = origin[0] - current[0], origin[1] - current[1]
    distance = math.hypot(dx, dy)
    if distance <= POSITION_TOLERANCE:
        error = wrap(origin[2] - current[2])
        if abs(error) <= YAW_TOLERANCE:
            return None
        return turn(error)
    bearing = math.atan2(dy, dx)
    options = [(wrap(bearing - current[2]), 1), (wrap(bearing + math.pi - current[2]), -1)]
    error, sign = min(options, key=lambda item: abs(item[0]))
    if abs(error) > YAW_TOLERANCE:
        return turn(error)
    speed = (.06 if distance < .08 else .15 if distance < .30 else .20) if accelerated else (.06 if distance < .06 else .08)
    duration = max(200, min(1500 if accelerated else 1000, round(max(0., distance - .012) * .7 / speed * 1000)))
    return Pulse(vx=sign * speed, duration_ms=duration)


class Mission:
    def __init__(self, session, seed=20261117, profile="low"):
        self.session, self.seed = session, seed
        self.profile = self.session.profile = select_profile(profile)
        self.origin = self.current = None
        self.started = self.deadline = None
        self.actions = 0
        self.linear_integral = self.angular_integral = 0.

    def check_complete(self, result):
        sample = result["sample"]
        if (result["phase"] != 4 or result["error"] != 0 or sample.get("phase") != 4 or
            sample.get("error") != 0 or sample.get("enabled") != 0 or
            any(sample.get(f"state_{i}") != 0 for i in (1, 2, 3))):
            raise swd.SessionError("Pulse lacks successful stop and three-motor disable evidence")
        current = pose(result)
        self.session.check_fence(current)
        return current

    def pulse(self, pulse, stage):
        pulse.validate(self.profile)
        seconds = pulse.duration_ms / 1000
        if self.actions >= MAX_ACTIONS:
            raise swd.SessionError("Global action limit reached")
        if time.monotonic() + seconds + 2 > self.deadline:
            raise swd.SessionError("Insufficient mission budget for pulse plus stop")
        linear = self.linear_integral + abs(pulse.vx) * seconds
        angular = self.angular_integral + abs(pulse.wz) * seconds
        if linear > self.profile.linear_budget + 1e-12 or angular > self.profile.angular_budget + 1e-12:
            raise swd.SessionError("Cumulative linear/angular command budget exceeded")
        before = self.current
        self.actions += 1
        self.linear_integral, self.angular_integral = linear, angular
        self.session.log.emit("mission_pulse", stage=stage, index=self.actions, pulse=asdict(pulse),
                              pose_before=before, linear_integral_m=linear, angular_integral_rad=angular)
        result = self.session.execute(4, pulse.arguments(self.profile), seconds + 2)
        self.current = self.check_complete(result)
        if self.session.motion_samples == 0:
            raise swd.SessionError("No new valid RUN odometry sample observed for this pulse")
        if pulse.vx:
            dx, dy = self.current[0] - before[0], self.current[1] - before[1]
            progress = math.copysign(1, pulse.vx) * (dx * math.cos(before[2]) + dy * math.sin(before[2]))
            if progress < .002:
                raise swd.SessionError(f"Translation feedback has insufficient/reversed displacement: {progress:.5f} m")
        self.session.log.emit("mission_pulse_complete", stage=stage, index=self.actions,
                              pose=self.current, stop_disabled_confirmed=True,
                              valid_motion_samples=self.session.motion_samples)

    def run(self):
        failure = None
        try:
            result = self.session.execute(5, [0., 0., 0., 1000., 0., 0., 0., 0.], 4.)
            self.current = self.origin = self.check_complete(result)
            self.session.origin = self.origin
            self.started = time.monotonic()
            self.deadline = self.session.deadline = self.started + MAX_SECONDS
            self.actions = 1  # The one static/reset action also consumes the cap.
            self.session.log.emit("mission_origin", pose=self.origin, seed=self.seed,
                                  deadline_seconds=MAX_SECONDS, profile=asdict(self.profile))
            pulses, predicted = departure_plan(self.seed, self.profile)
            self.session.log.emit("departure_plan", pulses=[asdict(p) for p in pulses],
                                  predicted_poses=predicted, prediction_only=True)
            for pulse in pulses:
                self.pulse(pulse, "departure")
                # Already disabled; this is a transport dwell, not an active hold.
                time.sleep(.2)
            for _ in range(MAX_RETURN_ACTIONS):
                pulse = next_return_pulse(self.current, self.origin, self.profile)
                if pulse is None:
                    break
                self.pulse(pulse, "return")
            else:
                if next_return_pulse(self.current, self.origin, self.profile) is not None:
                    raise swd.SessionError("Return pulse limit reached before estimated arrival")
            if time.monotonic() > self.deadline:
                raise swd.SessionError("Mission deadline exceeded")
            report = dict(estimated_only=True, real_ground_return_verified=False,
                          profile=self.profile.name,
                          estimated_position_error_m=math.hypot(self.current[0] - self.origin[0], self.current[1] - self.origin[1]),
                          estimated_yaw_error_deg=math.degrees(wrap(self.current[2] - self.origin[2])),
                          elapsed_s=time.monotonic() - self.started, actions=self.actions,
                          linear_command_integral_m=self.linear_integral, angular_command_integral_rad=self.angular_integral,
                          initial_pose=self.origin, final_pose=self.current)
            self.session.log.emit("estimated_return_complete", **report)
            return report
        except BaseException as exc:
            failure = exc
            self.session.log.emit("mission_aborted", message=str(exc), kind=type(exc).__name__)
            raise
        finally:
            # Never write against an unverified ELF or invalid initial mailbox.
            if self.session._firmware_verified and self.session._initial_validated:
                try:
                    self.session.execute(1, [0.] * 8, 2.5)
                    self.session.log.emit("mission_final_stop_confirmed")
                except BaseException as exc:
                    self.session.log.emit("mission_final_stop_unconfirmed", message=str(exc), kind=type(exc).__name__)
                    if failure is None:
                        raise swd.SessionError("Mission final STOP not confirmed: " + str(exc)) from exc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--describe", action="store_true", help="Offline plan only (default)")
    action.add_argument("--execute", action="store_true", help="Explicitly execute the bounded hardware session")
    parser.add_argument("--seed", type=int, default=20261117)
    parser.add_argument("--profile", choices=tuple(PROFILES), default="low")
    parser.add_argument("--elf", type=Path)
    parser.add_argument("--nm")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--stop-file", type=Path, help="Creating this file requests STOP; it is never deleted")
    args = parser.parse_args(argv)
    profile = PROFILES[args.profile]
    pulses, predicted = departure_plan(args.seed, profile)
    if not args.execute:
        print(json.dumps(dict(offline=True, seed=args.seed, profile=asdict(profile), static_origin_ms=1000,
                              pulses=[asdict(p) for p in pulses],
                              predicted_poses=predicted, predicted_only=True, max_actions=MAX_ACTIONS,
                              max_return_actions=MAX_RETURN_ACTIONS, mission_seconds=MAX_SECONDS,
                              estimated_geofence=dict(x_min_m=-.10, x_max_m=profile.fence_x_max, abs_y_m=profile.fence_abs_y),
                              linear_budget_m=profile.linear_budget, angular_budget_rad=profile.angular_budget,
                              position_tolerance_m=POSITION_TOLERANCE, yaw_tolerance_deg=5), indent=2))
        return 0
    if args.elf is None or args.log is None:
        parser.error("--execute requires the exact --elf and a new --log path")
    if args.stop_file is not None and args.stop_file.exists():
        parser.error("Stop file already exists; refusing execution")
    client = log = None
    try:
        symbols = swd.load_symbols(args.elf, args.nm)
        with swd.writer_lock("127.0.0.1", 6666):
            log = swd.JsonLog(args.log)
            client = swd.TclClient("127.0.0.1", 6666)
            if client.evaluate("stm32f4x.cpu curstate") != "running":
                raise swd.SessionError("MCU must already be running; no reset/halt/resume is performed")
            session = GuardedSession(client, symbols, log, profile=profile)
            session.stop_file = args.stop_file
            session.verify_firmware(args.elf)
            report = Mission(session, args.seed, profile).run()
            print(json.dumps(report, indent=2, allow_nan=False))
            return 0
    except (swd.SessionError, OSError, KeyboardInterrupt) as exc:
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
