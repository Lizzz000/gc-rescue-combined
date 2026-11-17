"""Offline random transport/return study using the repository's real C odometry.

No serial, SWD, CAN, or hardware command interface is included. The navigator
and ground plant are deliberately simplified models, not the chassis firmware.
All perturbation values are assumptions, not fitted vehicle calibration.
"""
from __future__ import annotations

import argparse
import csv
import ctypes
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
ALGORITHM = ROOT / "project/Core/Algorithm/Chassis"
SOURCES = (ALGORITHM / "chassis_odometry.c", ALGORITHM / "motor_chassis.c",
           ALGORITHM / "chassis_odometry.h", ALGORITHM / "motor_chassis.h")
DT = 0.01
RADIUS, OFFSET = 0.035, 0.108
WRAPPER = r'''
#include "chassis_odometry.h"
#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif
static Odometry_State_t state;
EXPORT void simulation_reset(void) { Odometry_Init(&state); }
EXPORT int simulation_update(float p1, float p2, float p3,
                             float a, float b, float c, float yaw, unsigned int us)
{
    const float positions[3] = {p1,p2,p3};
    const float wheels[3] = {a,b,c};
    return Odometry_Update_Position(&state,positions,wheels,yaw,us);
}
EXPORT void simulation_read(double *out)
{
    out[0]=state.data.x_m; out[1]=state.data.y_m; out[2]=state.data.yaw_rad;
    out[3]=state.data.vx_mps; out[4]=state.data.vy_mps;
    out[5]=state.data.wheel_wz_rad_s; out[6]=state.data.valid;
}
'''


def wrap(angle):
    return math.remainder(angle, 2 * math.pi)


def body_vector(dx, dy, yaw):
    """World displacement expressed at the current body heading."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def inverse(vx, vy, wz):
    # ID1 rear, ID2 right front, ID3 left front. Project the rigid-body
    # contact velocity onto each CCW tangent, independently of the C formula.
    contacts = ((-OFFSET, 0.), (OFFSET / 2, -math.sqrt(3) * OFFSET / 2),
                (OFFSET / 2, math.sqrt(3) * OFFSET / 2))
    return [((-y / OFFSET) * (vx - wz * y) + (x / OFFSET) * (vy + wz * x)) / RADIUS
            for x, y in contacts]


def forward(wheels):
    a, b, c = [value * RADIUS for value in wheels]
    return (b - c) / math.sqrt(3), (-2 * a + b + c) / 3, (a + b + c) / (3 * OFFSET)


def compiler_path(explicit=None):
    fallback = Path("C:/Program Files/JetBrains/CLion 2026.2.3.1/bin/mingw/bin/gcc.exe")
    cc = explicit or shutil.which("gcc") or (str(fallback) if fallback.exists() else None)
    if not cc or "arm-none-eabi" in str(cc):
        raise RuntimeError("需要主机 GCC；使用 --cc 指定，不能使用 ARM 交叉编译器")
    return str(Path(cc).resolve())


class NativeOdometry:
    """Compile the unchanged production C files; no duplicated Python estimator."""
    def __init__(self, build, cc=None):
        build = Path(build)
        build.mkdir(parents=True, exist_ok=True)
        cc = compiler_path(cc)
        self.provenance = {
            "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCES},
            "wrapper_sha256": hashlib.sha256(WRAPPER.encode()).hexdigest(),
            "compiler": cc,
        }
        wrapper = build / "odometry_wrapper.c"
        wrapper.write_text(WRAPPER, encoding="utf-8")
        library = build / ("odometry.dll" if os.name == "nt" else "odometry.so")
        env = dict(os.environ, PATH=str(Path(cc).parent) + os.pathsep + os.environ.get("PATH", ""))
        command = [cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", "-shared",
                   "-fPIC", "-static-libgcc", "-I" + str(ALGORITHM), str(wrapper),
                   str(SOURCES[0]), str(SOURCES[1]), "-lm", "-o", str(library)]
        subprocess.run(command, check=True, capture_output=True, text=True, env=env, timeout=30)
        self.provenance["compiler_version"] = subprocess.check_output(
            [cc, "--version"], text=True, env=env, timeout=10).splitlines()[0]
        self.lib = ctypes.CDLL(str(library.resolve()))
        self.lib.simulation_reset.argtypes = []
        self.lib.simulation_reset.restype = None
        self.lib.simulation_update.argtypes = [ctypes.c_float] * 7 + [ctypes.c_uint32]
        self.lib.simulation_update.restype = ctypes.c_int
        self.lib.simulation_read.argtypes = [ctypes.POINTER(ctypes.c_double)]
        self.lib.simulation_read.restype = None

    def reset(self):
        self.lib.simulation_reset()

    def update(self, positions, wheels, yaw, time_s):
        if not self.lib.simulation_update(*positions, *wheels, yaw, round(time_s * 1e6) & 0xFFFFFFFF):
            raise RuntimeError("真实 C 里程计算法拒绝模拟反馈")
        values = (ctypes.c_double * 7)()
        self.lib.simulation_read(values)
        return list(values)

    def close(self):
        if self.lib is not None:
            # Release the Windows DLL before a caller removes its build directory.
            import _ctypes
            (_ctypes.FreeLibrary if os.name == "nt" else _ctypes.dlclose)(self.lib._handle)
            self.lib = None


@dataclass(frozen=True)
class Model:
    name: str
    wheel_gain: tuple = (1.0, 1.0, 1.0)
    wheel_bias_rad_s: tuple = (0.0, 0.0, 0.0)
    wheel_noise_rad_s: float = 0.0
    loaded_slip_xy: tuple = (0.0, 0.0)
    imu_drift_deg_s: float = 0.0
    imu_noise_deg: float = 0.0
    wheel_response_s: float = 0.06
    wheel_accel_rad_s2: float = 20.0


IDEAL = Model("ideal")
PERTURBED = Model("assumed_sensor_error_and_slip", (1.02, .98, 1.01), (.008, -.006, .004),
                  .002, (.08, .04), .05, .005)


def make_route(seed=20261117, box_m=.5, segments=8, speed_mps=.1):
    if not 0.2 <= box_m <= .5 or not 1 <= segments <= 8 or not .02 <= speed_mps <= .1:
        raise ValueError("范围要求：正方形边长0.2..0.5m，搬运段1..8，速度0.02..0.10m/s")
    rng = random.Random(seed)
    directions = [("forward", 1, 0), ("backward", -1, 0), ("left", 0, 1), ("right", 0, -1)]
    margin = min(.05, box_m / 5)
    bound = box_m / 2 - margin
    for _ in range(1000):
        ordered = (directions * 2)[:segments]
        rng.shuffle(ordered)
        x = y = yaw = 0.0
        actions = []
        for index, (name, ux, uy) in enumerate(ordered):
            if index in (2, 5):
                yaw = wrap(yaw + math.radians(rng.choice((-45, -30, 30, 45))))
                actions.append(dict(kind="turn", x_m=x, y_m=y, yaw_rad=yaw, loaded=index >= 4))
            distance = rng.uniform(.03, .08)
            x += distance * (ux * math.cos(yaw) - uy * math.sin(yaw))
            y += distance * (ux * math.sin(yaw) + uy * math.cos(yaw))
            if abs(x) > bound or abs(y) > bound:
                break
            actions.append(dict(kind="transport", direction=name, distance_m=distance,
                                x_m=x, y_m=y, yaw_rad=yaw, loaded=index >= 4))
            actions.append(dict(kind="dwell", duration_s=.8 if index in (3, 7) else .4,
                                purpose="pickup" if index == 3 else "place" if index == 7 else "settle",
                                x_m=x, y_m=y, yaw_rad=yaw, loaded=index >= 3 and index < 7))
        else:
            return dict(model_only=True, seed=seed, box_side_m=box_m, nominal_margin_m=margin,
                        bounds_apply_to="chassis centre only; footprint, obstacles and gripper are not modeled",
                        speed_limit_mps=speed_mps, turn_limit_rad_s=.3, transport_segments=segments,
                        actions=actions, return_policy="estimate-only translation to (0,0), then yaw to 0")
    raise RuntimeError("无法在给定边界内生成路线")


class Plant:
    def __init__(self, model, seed):
        self.model, self.rng = model, random.Random(seed)
        self.x = self.y = self.yaw = self.time = 0.0
        self.wheels = [0.0] * 3
        self.positions = [0.0] * 3

    def step(self, command, loaded):
        targets = inverse(*command)
        # 1 ms encoder/ground integration, separate from the estimator's 10 ms SE(2) step.
        for _ in range(10):
            for i in range(3):
                derivative = (targets[i] - self.wheels[i]) / self.model.wheel_response_s
                derivative = max(-self.model.wheel_accel_rad_s2, min(self.model.wheel_accel_rad_s2, derivative))
                self.wheels[i] += derivative * .001
                self.positions[i] += self.wheels[i] * .001
            vx, vy, wz = forward(self.wheels)
            sx, sy = self.model.loaded_slip_xy if loaded else (0, 0)
            vx, vy = vx * (1 - sx), vy * (1 - sy)
            heading = self.yaw + wz * .0005
            self.x += (vx * math.cos(heading) - vy * math.sin(heading)) * .001
            self.y += (vx * math.sin(heading) + vy * math.cos(heading)) * .001
            self.yaw = wrap(self.yaw + wz * .001)
            self.time += .001

    def feedback(self):
        wheels = [value * gain + bias + self.rng.gauss(0, self.model.wheel_noise_rad_s)
                  for value, gain, bias in zip(self.wheels, self.model.wheel_gain, self.model.wheel_bias_rad_s)]
        yaw = wrap(self.yaw + math.radians(self.time * self.model.imu_drift_deg_s +
                                         self.rng.gauss(0, self.model.imu_noise_deg)))
        # Position comes from shaft motion, independently of reported velocity bias/noise.
        positions = [math.remainder(value * gain, 25.)
                     for value, gain in zip(self.positions, self.model.wheel_gain)]
        return positions, wheels, yaw


def run_scenario(route, model, odometry, position_tolerance_m=.02, yaw_tolerance_deg=5.):
    if not math.isfinite(position_tolerance_m) or not math.isfinite(yaw_tolerance_deg) or \
       position_tolerance_m <= 0 or not 0 < yaw_tolerance_deg <= 180:
        raise ValueError("验收容差必须为有限正数，航向容差不得超过180度")
    plant = Plant(model, route["seed"] + 17)
    odometry.reset()
    estimate = odometry.update(*plant.feedback(), plant.time)
    trace, outcomes = [], []
    status, max_position_error, ground_distance = "completed", 0.0, 0.0
    return_vector = None
    actions = list(route["actions"])
    actions.extend([dict(kind="return", x_m=0., y_m=0., yaw_rad=actions[-1]["yaw_rad"], loaded=False),
                    dict(kind="restore_heading", x_m=0., y_m=0., yaw_rad=0., loaded=False)])
    for stage, action in enumerate(actions):
        started, settled = plant.time, 0
        if action["kind"] == "return":
            bx, by = body_vector(-estimate[0], -estimate[1], estimate[2])
            return_vector = dict(body_dx_m=bx, body_dy_m=by, distance_m=math.hypot(bx, by),
                                 bearing_body_rad=math.atan2(by, bx), final_yaw_change_rad=wrap(-estimate[2]))
        distance = math.hypot(action["x_m"] - estimate[0], action["y_m"] - estimate[1])
        timeout = max(5., 4 * distance / route["speed_limit_mps"] +
                      4 * abs(wrap(action["yaw_rad"] - estimate[2])) / route["turn_limit_rad_s"] + 3)
        while True:
            dx, dy = action["x_m"] - estimate[0], action["y_m"] - estimate[1]
            error = math.hypot(dx, dy)
            yaw_error = wrap(action["yaw_rad"] - estimate[2])
            moving = action["kind"] in ("transport", "return")
            vx, vy = body_vector(dx, dy, estimate[2]) if moving and error > .002 else (0., 0.)
            scale = min(2.0, route["speed_limit_mps"] / max(math.hypot(vx, vy), 1e-12))
            vx, vy = vx * scale, vy * scale
            wz = max(-route["turn_limit_rad_s"], min(route["turn_limit_rad_s"], 2 * yaw_error))
            if abs(yaw_error) <= math.radians(.2):
                wz = 0.
            if action["kind"] == "dwell":
                vx = vy = wz = 0.
            before = plant.x, plant.y
            plant.step((vx, vy, wz), action["loaded"])
            positions, feedback, imu_yaw = plant.feedback()
            estimate = odometry.update(positions, feedback, imu_yaw, plant.time)
            ground_distance += math.hypot(plant.x - before[0], plant.y - before[1])
            pose_error = math.hypot(estimate[0] - plant.x, estimate[1] - plant.y)
            max_position_error = max(max_position_error, pose_error)
            trace.append(dict(time_s=round(plant.time, 6), stage=stage, kind=action["kind"],
                              loaded=int(action["loaded"]), true_x_m=plant.x, true_y_m=plant.y,
                              true_yaw_rad=plant.yaw, estimated_x_m=estimate[0], estimated_y_m=estimate[1],
                              estimated_yaw_rad=estimate[2], cmd_vx_mps=vx, cmd_vy_mps=vy, cmd_wz_rad_s=wz,
                              actual_wheel_1=plant.wheels[0], actual_wheel_2=plant.wheels[1],
                              actual_wheel_3=plant.wheels[2],
                              encoder_position_1=positions[0], encoder_position_2=positions[1],
                              encoder_position_3=positions[2],
                              feedback_wheel_1=feedback[0], feedback_wheel_2=feedback[1],
                              feedback_wheel_3=feedback[2], position_error_m=pose_error))
            # Truth is used only for evaluation and a model boundary abort,
            # never to compute navigation commands or claim arrival.
            if max(abs(plant.x), abs(plant.y)) > route["box_side_m"] / 2:
                status = "ground_box_exceeded"
                break
            if plant.time - started >= timeout:
                status = "segment_timeout"
                break
            if action["kind"] == "dwell":
                if plant.time - started >= action["duration_s"]:
                    break
            else:
                position_ok = not moving or math.hypot(action["x_m"] - estimate[0], action["y_m"] - estimate[1]) <= .003
                stopped = math.hypot(estimate[3], estimate[4]) <= .005 and abs(estimate[5]) <= .02
                heading_ok = abs(wrap(action["yaw_rad"] - estimate[2])) <= math.radians(.3)
                settled = settled + 1 if position_ok and stopped and heading_ok else 0
                if settled >= 3:
                    break
        outcomes.append(dict(stage=stage, kind=action["kind"], status=status,
                             elapsed_s=plant.time - started))
        if status != "completed":
            break
    summary = dict(model_only=True, scope="production C odometry + simplified plant/navigation, no hardware",
                   assumptions=asdict(model), status=status, duration_s=plant.time,
                   estimated_origin_error_m=math.hypot(estimate[0], estimate[1]),
                   true_origin_error_m=math.hypot(plant.x, plant.y),
                   estimated_final_yaw_deg=math.degrees(estimate[2]), true_final_yaw_deg=math.degrees(plant.yaw),
                   max_estimate_ground_disagreement_m=max_position_error, true_path_length_m=ground_distance,
                   max_abs_true_coordinate_m=max(max(abs(r["true_x_m"]), abs(r["true_y_m"])) for r in trace),
                   return_plan_from_estimate=return_vector, stages=outcomes)
    summary["return_tolerance"] = dict(position_m=position_tolerance_m, yaw_deg=yaw_tolerance_deg,
                                       purpose="coarse positioning; visual correction is not simulated")
    summary["estimated_return_within_tolerance"] = (status == "completed" and
        summary["estimated_origin_error_m"] <= position_tolerance_m and
        abs(summary["estimated_final_yaw_deg"]) <= yaw_tolerance_deg)
    summary["true_return_within_tolerance"] = (status == "completed" and
        summary["true_origin_error_m"] <= position_tolerance_m and
        abs(summary["true_final_yaw_deg"]) <= yaw_tolerance_deg)
    return summary, trace


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=20261117)
    parser.add_argument("--box-m", type=float, default=.5, help="Square side length, centred at origin")
    parser.add_argument("--segments", type=int, default=8)
    parser.add_argument("--speed-mps", type=float, default=.1)
    parser.add_argument("--position-tolerance-m", type=float, default=.02)
    parser.add_argument("--yaw-tolerance-deg", type=float, default=5.)
    parser.add_argument("--cc", help="Host GCC executable")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/random_return_20261117")
    args = parser.parse_args(argv)
    route = make_route(args.seed, args.box_m, args.segments, args.speed_mps)
    args.output.mkdir(parents=True, exist_ok=True)
    # Preserve previous runs; select a new output directory rather than overwrite.
    if any((args.output / name).exists() for name in ("route.json", "summary.json")):
        parser.error("输出已有路线或结果，请指定新的 --output 目录")
    odometry = NativeOdometry(args.output / "native", args.cc)
    try:
        summaries = []
        for model in (IDEAL, PERTURBED):
            summary, trace = run_scenario(route, model, odometry,
                                          args.position_tolerance_m, args.yaw_tolerance_deg)
            summaries.append(summary)
            with (args.output / f"{model.name}.csv").open("x", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(trace[0]))
                writer.writeheader()
                writer.writerows(trace)
            print(f"{model.name}: {summary['status']}，估计回零误差 "
                  f"{summary['estimated_origin_error_m']*1000:.2f} mm，模拟真实回零误差 "
                  f"{summary['true_origin_error_m']*1000:.2f} mm，真实末航向 "
                  f"{summary['true_final_yaw_deg']:.2f}°；估计/真值粗定位判定 "
                  f"{summary['estimated_return_within_tolerance']}/{summary['true_return_within_tolerance']}")
        report = dict(model_only=True, note="误差参数均为假设；不代表车辆标定、真实搬运或实车返航已通过。",
                      bounds_apply_to=route["bounds_apply_to"],
                      ground_geometry=dict(wheel_radius_m=RADIUS, centre_offset_m=OFFSET),
                      provenance=odometry.provenance, scenarios=summaries)
        for name, value in (("route.json", route), ("summary.json", report)):
            with (args.output / name).open("x", encoding="utf-8") as stream:
                json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write("\n")
    finally:
        odometry.close()
    print(f"仅离线准备，未连接硬件。输出：{args.output.resolve()}")
    return 0 if all(item["status"] == "completed" for item in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
