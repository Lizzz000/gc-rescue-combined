"""Seeded continuous waypoint mission; default is offline description.

Requires the existing command 7 enabled, zero-target HOLD. Every waypoint uses
the same live odometry frame. No reset, reflash or intermediate disable occurs.
The reported return is an odometry estimate, not a ground-truth measurement.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import sys
import time

from continuous_return_session import ContinuousSession, finite, wrap
import swd_session as swd

MAX_SECONDS = 90.
MAX_ACTIONS = 12
LINEAR_BUDGET = 7.
ANGULAR_BUDGET = 6 * math.pi
LINEAR_SPEED = .20
ANGULAR_SPEED = .50
PROFILES = {
    "standard": dict(max_seconds=MAX_SECONDS, linear_budget=LINEAR_BUDGET,
                     angular_budget=ANGULAR_BUDGET, max_yaw=math.pi / 3),
    "long": dict(max_seconds=120., linear_budget=9.,
                 angular_budget=9 * math.pi, max_yaw=math.pi / 2),
}


def plan_metrics(route):
    previous = [0., 0., 0.]
    distances, angles = [], []
    for point in route:
        if len(point) != 3 or not finite(point):
            raise swd.SessionError("Route contains an invalid waypoint")
        distances.append(math.hypot(point[0] - previous[0], point[1] - previous[1]))
        angles.append(abs(wrap(point[2] - previous[2])))
        previous = point
    ideal = sum(max(d / LINEAR_SPEED, a / ANGULAR_SPEED) for d, a in zip(distances, angles))
    return dict(path_m=sum(distances), angle_rad=sum(angles), max_leg_m=max(distances, default=0),
                ideal_motion_seconds=ideal, estimated_seconds=ideal + len(route) * 1.5 + .5,
                legs_m=distances)


def validate_plan(route, profile="standard"):
    if profile not in PROFILES:
        raise swd.SessionError("Unknown continuous motion profile")
    limits = PROFILES[profile]
    metrics = plan_metrics(route)
    if (not 1 <= len(route) <= MAX_ACTIONS or route[-1] != [0., 0., 0.] or
        any(not (0 <= p[0] <= 1.05 and abs(p[1]) <= .25 and abs(p[2]) <= limits["max_yaw"]) for p in route) or
        metrics["max_leg_m"] > .95 or metrics["path_m"] > limits["linear_budget"] or
        metrics["angle_rad"] > limits["angular_budget"]):
        raise swd.SessionError("Route exceeds waypoint, region, leg or total budget limits")
    return metrics


def make_route(seed=20261117, profile="standard"):
    if profile not in PROFILES:
        raise swd.SessionError("Unknown continuous motion profile")
    rng = random.Random(seed)
    if profile == "long":
        # Longer compound moves fit the same floor area: near-0.9 m traverses
        # with alternating +/-90 degree goals, hence roughly 180 degree turns.
        for _ in range(1000):
            route = [[rng.uniform(.89, .93) if i % 2 == 0 else rng.uniform(.06, .10),
                      (1 if i % 2 == 0 else -1) * rng.uniform(.14, .19),
                      (1 if i % 2 == 0 else -1) * math.pi / 2]
                     for i in range(8)] + [[0., 0., 0.]]
            metrics = plan_metrics(route)
            if metrics["max_leg_m"] <= .95 and min(metrics["legs_m"][:-1]) >= .8:
                validate_plan(route, profile)
                return route
        raise swd.SessionError("Could not generate the long-motion route")
    signs = [1, -1, 1, -1, -1, 1, -1, 1, 1, -1]
    for _ in range(1000):
        route = [[rng.uniform(.76, .91) if i % 2 == 0 else rng.uniform(.20, .33),
                  rng.uniform(-.22, .22), signs[i] * math.radians(rng.uniform(45, 60))]
                 for i in range(10)] + [[0., 0., 0.]]
        metrics = plan_metrics(route)
        if (4 <= metrics["path_m"] <= 6 and .75 <= metrics["max_leg_m"] <= .95 and
            metrics["angle_rad"] <= ANGULAR_BUDGET):
            validate_plan(route)
            return route
    raise swd.SessionError("Could not generate a route inside the fixed limits")


def world_goal(origin, point):
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return [origin[0] + c * point[0] - s * point[1],
            origin[1] + s * point[0] + c * point[1], wrap(origin[2] + point[2])]


def local_pose(origin, pose):
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return [c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - origin[2])]


def fresh_pose(state):
    sample = state.get("sample", {})
    values = [sample.get(key) for key in ("odom_x_m", "odom_y_m", "odom_yaw_rad", "odom_age_ms", "yaw_deg", "imu_age_ms", "tick_ms")]
    if (not finite(values) or sample.get("odom_valid") != 1 or
        not 0 <= values[3] <= 50 or not 0 <= values[5] <= 20 or
        abs(wrap(values[2] - math.radians(values[4]))) > math.radians(5)):
        raise swd.SessionError("Odometry/IMU pose is invalid, stale or inconsistent")
    return values[:3]


def hold_pose(state):
    sample = state.get("sample", {})
    if (state.get("command") != 7 or state.get("phase") != 4 or state.get("error") != 0 or
        sample.get("phase") != 4 or sample.get("error") != 0 or sample.get("enabled") != 1 or
        any(sample.get(f"state_{i}") != 1 for i in (1, 2, 3)) or
        any(sample.get(f"target_{i}") != 0 for i in (1, 2, 3))):
        raise swd.SessionError("Expected command 7 enabled zero-target HOLD; no reset is permitted")
    return fresh_pose(state)


class RandomSession(ContinuousSession):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Keep the parent start_pose unset: this mission uses one global frame
        # and its own 1.35 m radius, rather than the old return's 1.3 m fence.
        self.origin = None
        self.deadline = None
        self.max_seconds = MAX_SECONDS
        self.last_sample_seq = None
        self.last_sample_time = None

    def guard(self):
        super().guard()
        if not self.stopping and self.deadline is not None and time.monotonic() >= self.deadline:
            self.guard_failure, self.stopping = f"Mission exceeded the {self.max_seconds:g} second budget", True
            raise swd.SessionError(self.guard_failure)

    def snapshot(self):
        try:
            state = super().snapshot()
            if not self.stopping and self.origin is not None:
                sample = state["sample"]
                if state["command"] != 7 or sample.get("enabled") != 1:
                    raise swd.SessionError("Continuous mission lost command 7 or motor enable")
                pose = fresh_pose(state)
                x, y, _ = local_pose(self.origin, pose)
                if not (-.10 <= x <= 1.30 and abs(y) <= .40 and math.hypot(x, y) <= 1.35):
                    raise swd.SessionError("Continuous mission exceeded the global estimated fence")
                now = time.monotonic()
                if state["sample_seq"] != self.last_sample_seq:
                    self.last_sample_seq, self.last_sample_time = state["sample_seq"], now
                elif self.last_sample_time is not None and now - self.last_sample_time > .5:
                    raise swd.SessionError("Continuous mission telemetry publication stopped")
            return state
        except (OSError, swd.SessionError) as exc:
            self.guard_failure, self.stopping = str(exc), True
            raise


def run_mission(session, route, seed=20261117, profile="standard"):
    metrics = validate_plan(route, profile)
    limits = PROFILES[profile]
    completed = False
    try:
        initial = session.validate_initial()
        origin = hold_pose(initial)
        session.origin = origin
        started = time.monotonic()
        session.max_seconds = limits["max_seconds"]
        session.deadline = started + session.max_seconds
        session.log.emit("continuous_random_origin", pose=origin, seed=seed,
                         profile=profile, limits=limits,
                         tick_ms=initial["sample"]["tick_ms"], route=route, **metrics)
        linear_used = angular_used = 0.
        for index, point in enumerate(route, 1):
            session.guard()
            current = hold_pose(session.snapshot())
            goal = world_goal(origin, point)
            distance = math.hypot(goal[0] - current[0], goal[1] - current[1])
            angle = abs(wrap(goal[2] - current[2]))
            # Planned legs are <= .95 m; the 1 m runtime hard limit leaves
            # room for the previous waypoint's allowed 2 cm endpoint error.
            if distance > 1.0:
                raise swd.SessionError("Actual start-to-goal leg exceeds firmware's one metre limit")
            linear_used += distance
            angular_used += angle
            if (index > MAX_ACTIONS or linear_used > limits["linear_budget"] or
                    angular_used > limits["angular_budget"]):
                raise swd.SessionError("Actual requested distance/angle/action budget exceeded")
            remaining = session.deadline - time.monotonic()
            # Keep an explicit host/STOP margin while respecting the global
            # deadline. There is no retry or extra corrective waypoint.
            duration_ms = min(20000, int((remaining - 3.) * 1000))
            minimum_seconds = max(distance / LINEAR_SPEED, angle / ANGULAR_SPEED) + 1.5
            if duration_ms < 100 or duration_ms / 1000 < minimum_seconds:
                raise swd.SessionError("Insufficient global time for the next waypoint")
            args = [*goal, LINEAR_SPEED, ANGULAR_SPEED, float(duration_ms), 0., 0.]
            session.log.emit("waypoint_start", index=index, local_goal=point, world_goal=goal,
                             requested_path_m=linear_used, requested_angle_rad=angular_used)
            result = session.execute(7, args, duration_ms / 1000 + 3.)
            swd.validate_endpoint(7, args, result)
            hold_pose(result)
            session.log.emit("waypoint_complete_enabled", index=index, **result)
        # Firmware HOLD deliberately does not require a heartbeat. Observe it
        # read-only before exiting, including current rather than saved endpoint.
        session.heartbeat = None
        observe_until = time.monotonic() + .5
        while True:
            result = session.snapshot()
            swd.validate_endpoint(7, args, result)
            final_pose = hold_pose(result)
            relative = local_pose(origin, final_pose)
            if math.hypot(*relative[:2]) > .02 + 1e-6 or abs(relative[2]) > math.radians(10) + 1e-6:
                raise swd.SessionError("Current estimated home pose is outside 2 cm / 10 degree tolerance")
            session.log.emit("final_enabled_hold_observation", **result)
            if time.monotonic() >= observe_until:
                break
            time.sleep(.05)
        report = dict(estimated_only=True, real_ground_return_verified=False, motors_enabled=True,
                      zero_targets=True, waypoints_completed=len(route), origin=origin, final_pose=final_pose,
                      position_error_m=math.hypot(*relative[:2]), yaw_error_deg=math.degrees(relative[2]),
                      requested_path_m=linear_used, requested_angle_rad=angular_used,
                      elapsed_seconds=time.monotonic() - started, seed=seed, profile=profile)
        session.log.emit("continuous_random_complete_enabled", **report)
        completed = True
        return report
    finally:
        if not completed and session._firmware_verified and session._initial_validated:
            failure = sys.exc_info()[1]
            session.stopping = True
            try:
                session.execute(1, [0.] * 8, 2.5)
                session.log.emit("mission_failure_stop_confirmed")
            except BaseException as exc:
                session.log.emit("mission_failure_stop_unconfirmed", message=str(exc))
                if failure is None:
                    raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--describe", action="store_true")
    action.add_argument("--execute", action="store_true")
    parser.add_argument("--seed", type=int, default=20261117)
    parser.add_argument("--profile", choices=PROFILES, default="standard")
    parser.add_argument("--elf", type=Path)
    parser.add_argument("--nm")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--stop-file", type=Path)
    args = parser.parse_args(argv)
    route = make_route(args.seed, args.profile)
    limits = PROFILES[args.profile]
    if not args.execute:
        print(json.dumps(dict(offline=True, estimated_only=True, seed=args.seed, profile=args.profile, route=route,
                              **validate_plan(route, args.profile), linear_speed_mps=LINEAR_SPEED, angular_speed_rad_s=ANGULAR_SPEED,
                              max_seconds=limits["max_seconds"], max_actions=MAX_ACTIONS, linear_budget_m=limits["linear_budget"],
                              angular_budget_rad=limits["angular_budget"], fence_initial_body=[[-.10, 1.30], [-.40, .40]],
                              fence_radius_m=1.35, initial_requires="existing command 7 enabled zero-target HOLD"), indent=2))
        return 0
    if not args.elf or not args.log:
        parser.error("Execution requires exact --elf and a new --log")
    if args.stop_file and args.stop_file.exists():
        parser.error("Stop file already exists")
    client = log = None
    try:
        symbols = swd.load_symbols(args.elf, args.nm)
        with swd.writer_lock("127.0.0.1", 6666):
            log = swd.JsonLog(args.log)
            client = swd.TclClient("127.0.0.1", 6666)
            if client.evaluate("stm32f4x.cpu curstate") != "running":
                raise swd.SessionError("MCU is not running")
            session = RandomSession(client, symbols, log, stop_file=args.stop_file)
            session.verify_firmware(args.elf)
            report = run_mission(session, route, args.seed, args.profile)
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
