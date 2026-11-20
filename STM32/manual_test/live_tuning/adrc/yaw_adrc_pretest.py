"""偏航 LADRC 初测。100 Hz 合成闭环和既有实车数据的只读回放。"""
import argparse
from collections import deque
from dataclasses import asdict, dataclass
from itertools import product
import json
import math
from pathlib import Path
from random import Random
from statistics import mean, median

DT = 0.01
ANGLE_DEADBAND = 0.001


def clamp(value, low, high):
    return min(high, max(low, value))


def bounded_yaw(profile, desired, applied, accel, dt):
    # 复现底盘反馈输出的同向、参考上限和发送斜率约束。
    if profile == 0.0:
        return 0.0
    sign = math.copysign(1.0, profile)
    limit = abs(profile)
    previous = sign * applied
    step = accel * dt
    next_rate = previous + clamp(sign * desired - previous, -step, step)
    return sign * clamp(next_rate, 0.0, limit)


@dataclass(frozen=True)
class ObserverTuning:
    b0: float
    control_bandwidth_rad_s: float
    observer_bandwidth_rad_s: float


@dataclass(frozen=True)
class ProportionalTuning:
    gain: float
    filter_s: float
    deadband: float


class YawESO:
    def __init__(self, tuning, max_interval_s=0.05):
        self.tuning = tuning
        self.max_interval_s = max_interval_s
        self.rate = None
        self.disturbance = 0.0

    def step(self, gyro, applied_last, dt):
        if not (math.isfinite(gyro) and math.isfinite(applied_last) and 0.0 < dt <= self.max_interval_s):
            self.rate = None
            self.disturbance = 0.0
            return None
        if self.rate is None:
            self.rate = gyro
            return None
        predicted = self.rate + dt * (self.tuning.b0 * applied_last + self.disturbance)
        innovation = gyro - predicted
        q = -math.expm1(-self.tuning.observer_bandwidth_rad_s * dt)
        self.rate = predicted + q * (2.0 - q) * innovation
        self.disturbance += q * q / dt * innovation
        return innovation


@dataclass(frozen=True)
class Scenario:
    name: str
    speed: float
    sign: int
    gain: float
    lag: float
    delay: int
    deadband: float
    disturbance: float
    noise: float
    seed: int


def profile_step(ref, ramp, demand, accel, smoothing):
    step = accel * DT
    ramp += clamp(demand - ramp, -step, step)
    if ramp == demand and abs(ref - demand) <= min(ANGLE_DEADBAND, step):
        return demand, ramp
    return ref + DT / (smoothing + DT) * (ramp - ref), ramp


def compare_run(case, cfg, tuning=None, proportional=None):
    # 被控模型为内环速度控制近似。增益、纯滞后、阻力和测量噪声分开注入。
    delay = deque([0.0] * case.delay)
    rng = Random(case.seed)
    ref = ramp = applied = speed = 0.0
    rate_filter = None
    observer = YawESO(tuning) if tuning else None
    p = proportional or ProportionalTuning(cfg['CHASSIS_YAW_RATE_FEEDBACK_KP'],
                                          cfg['CHASSIS_YAW_RATE_FILTER_S'],
                                          cfg['CHASSIS_YAW_RATE_DEADBAND_RAD_S'])
    speed_errors, overspeeds = [], []
    peak = slew = plateau_variation = 0.0
    settle_s = None
    for sample in range(451):
        now = sample * DT
        # 启动、稳速、反向、停车。扰动均按时间给定，两种控制器使用同一序列。
        direction = 1 if now < 2.05 else -1
        demand = 0.0 if now < 0.25 or now >= 3.35 else direction * case.sign * case.speed
        ref, ramp = profile_step(ref, ramp, demand, cfg["CHASSIS_VECTOR_ACCEL_RAD_S2"],
                                 cfg["CHASSIS_VECTOR_SMOOTHING_S"])
        gyro = speed + rng.gauss(0.0, case.noise) + 0.75 * case.noise * math.sin(now * 147.0)
        if observer:
            observer.step(gyro, applied, DT)

        magnitude = abs(ref)
        sign = math.copysign(1.0, ref)
        cap = min(cfg["CHASSIS_YAW_RATE_MAX_REDUCTION_RAD_S"],
                  cfg["CHASSIS_YAW_RATE_MAX_REDUCTION_RATIO"] * magnitude)
        if observer:
            raw = (tuning.control_bandwidth_rad_s * (ref - observer.rate) - observer.disturbance) / tuning.b0
            desired = sign * clamp(sign * raw, magnitude - cap, magnitude)
        elif ref != 0.0:
            if rate_filter is None:
                rate_filter = gyro
            else:
                rate_filter += DT / (p.filter_s + DT) * (gyro - rate_filter)
            excess = sign * rate_filter - magnitude - p.deadband
            reduction = min(cap, p.gain * max(0.0, excess))
            desired = sign * (magnitude - reduction)
        else:
            rate_filter = None
            desired = 0.0
        previous = applied
        applied = bounded_yaw(ref, desired, applied, cfg["CHASSIS_VECTOR_ACCEL_RAD_S2"], DT)
        assert math.isfinite(applied) and applied * ref >= 0.0 and abs(applied) <= abs(ref) + 1e-9
        assert ref != 0.0 or applied == 0.0
        slew = max(slew, abs(applied - previous) / DT)
        assert abs(applied - previous) <= DT * cfg["CHASSIS_VECTOR_ACCEL_RAD_S2"] + 1e-7

        # 只在两个方向的稳定指令段评价跟踪。停车计时包含斜坡及对象响应。
        if 0.70 <= now < 2.05 or 2.85 <= now < 3.35:
            speed_errors.append((speed - demand) ** 2)
            overspeeds.append(max(0.0, speed * math.copysign(1.0, demand) - abs(demand)))
            peak = max(peak, abs(speed))
            plateau_variation += abs(applied - previous)
        if now >= 3.35:
            if abs(speed) <= 0.02:
                settle_s = now - 3.35 if settle_s is None else settle_s
            else:
                settle_s = None

        delay.append(applied)
        delayed = delay.popleft()
        drive = math.copysign(max(0.0, abs(delayed) - case.deadband), delayed)
        force = case.disturbance * case.sign if 0.95 <= now < 1.19 else 0.0
        if 2.92 <= now < 3.12:
            force = -0.7 * case.disturbance * case.sign
        alpha = -math.expm1(-DT / case.lag)
        speed += alpha * (case.gain * drive + case.lag * force - speed)
    assert applied == 0.0 and settle_s is not None
    return {"rmse": math.sqrt(mean(speed_errors)), "max_overspeed": max(overspeeds),
            "peak_rate": peak, "settle_s": settle_s, "max_slew": slew,
            "plateau_variation": plateau_variation, "samples": overspeeds}


def load_config(repo):
    header = (repo / "STM32/project/Core/Control/Chassis/chassis.h").read_text(encoding="utf-8")
    cfg = {}
    for line in header.splitlines():
        words = line.split()
        if len(words) >= 3 and words[0] == "#define" and words[1].startswith(
                ("CHASSIS_VECTOR_", "CHASSIS_YAW_RATE_")):
            cfg[words[1]] = float(words[2].removesuffix("f"))
    return cfg


def expand_cases(specs):
    return [Scenario(name, speed, sign, *model, seed=index * 19 + 42)
            for index, ((name, model), speed, sign) in enumerate(product(specs, (0.32, 0.40), (1, -1)))]


def training_cases():
    # delay 的单位为 10 ms 控制拍。disturbance 的单位为 rad/s²。
    return expand_cases([
        ("train_nominal", (1.12, .080, 2, .012, 0.0, .008)),
        ("train_assist", (1.52, .090, 3, .014, 2.0, .012)),
        ("train_load", (0.95, .120, 3, .025, -1.2, .012)),
    ])


def holdout_cases():
    holdout = expand_cases([
        ("fast", (1.66, .060, 2, .008, 2.8, .014)),
        ("lagged", (1.41, .135, 9, .022, 0.0, .014)),
        ("loaded", (0.88, .160, 4, .045, -1.8, .014)),
        ("noisy", (1.28, .085, 3, .018, 1.0, .035)),
        ("impulse", (1.30, .115, 5, .020, 4.0, .010)),
    ])
    rng = Random(281119)
    for i in range(96):
        holdout.append(Scenario("spread", (.32, .40)[i % 2], (-1, 1)[i // 2 % 2],
                                rng.uniform(.80, 1.85), rng.uniform(.05, .17), rng.randrange(1, 10),
                                rng.uniform(.005, .055), rng.uniform(-3.5, 4.0), rng.uniform(.006, .036), 29100 + i))
    return holdout


def evaluate(cfg, chosen, chosen_p):
    """用给定参数复算训练和留出指标。不搜索，也不写文件。"""
    observer_sanity(chosen)
    training = training_cases()
    holdout = holdout_cases()
    training_adrc = stats([compare_run(case, cfg, chosen) for case in training])
    training_p = stats([compare_run(case, cfg, proportional=chosen_p) for case in training])
    baseline = [compare_run(case, cfg) for case in holdout]
    experimental = [compare_run(case, cfg, chosen) for case in holdout]
    tuned_proportional = [compare_run(case, cfg, proportional=chosen_p) for case in holdout]
    groups = {}
    for name in sorted({case.name for case in holdout}):
        indices = [i for i, case in enumerate(holdout) if case.name == name]
        groups[name] = {"count": len(indices), "baseline": stats([baseline[i] for i in indices]),
                        "adrc": stats([experimental[i] for i in indices]),
                        "retuned_proportional": stats([tuned_proportional[i] for i in indices])}
    return {
        "training_count": len(training),
        "training_adrc": training_adrc,
        "training_retuned_proportional": training_p,
        "holdout_count": len(holdout),
        "holdout_baseline": stats(baseline),
        "holdout_adrc": stats(experimental),
        "holdout_retuned_proportional": stats(tuned_proportional),
        "groups": groups,
        "cases_with_higher_rmse": sum(a["rmse"] > b["rmse"] + 1e-6 for a, b in zip(experimental, baseline)),
        "worst_rmse_increase": max(a["rmse"] - b["rmse"] for a, b in zip(experimental, baseline)),
        "cases_with_higher_rmse_than_retuned_p": sum(a["rmse"] > p["rmse"] + 1e-6
                                                    for a, p in zip(experimental, tuned_proportional)),
        "baseline": baseline,
        "experimental": experimental,
        "tuned_proportional": tuned_proportional,
        "holdout": holdout,
    }


def metric_mismatches(actual, recorded, path="report"):
    if isinstance(actual, dict) and isinstance(recorded, dict):
        for key in sorted(actual.keys() ^ recorded.keys()):
            yield f"{path}.{key}: missing from {'record' if key in actual else 'computed report'}"
        for key in sorted(actual.keys() & recorded.keys()):
            yield from metric_mismatches(actual[key], recorded[key], f"{path}.{key}")
    elif isinstance(actual, list) and isinstance(recorded, list):
        if len(actual) != len(recorded):
            yield f"{path}: length {len(actual)} != {len(recorded)}"
        for index, (value, reference) in enumerate(zip(actual, recorded)):
            yield from metric_mismatches(value, reference, f"{path}[{index}]")
    elif type(actual) in (int, float) and type(recorded) in (int, float):
        if not abs(actual - recorded) <= 1e-12:
            yield f"{path}: {actual} != {recorded}"
    elif type(actual) is not type(recorded) or actual != recorded:
        yield f"{path}: {actual!r} != {recorded!r}"


def stats(rows):
    overspeeds = sorted(v for row in rows for v in row["samples"])
    p95 = overspeeds[math.ceil(0.95 * len(overspeeds)) - 1]
    return {"mean_rmse": mean(row["rmse"] for row in rows),
            "p95_overspeed": p95,
            "max_overspeed": max(overspeeds),
            "peak_rate": max(row["peak_rate"] for row in rows),
            "mean_settle_s": mean(row["settle_s"] for row in rows),
            "max_settle_s": max(row["settle_s"] for row in rows),
            "max_slew": max(row["max_slew"] for row in rows),
            "mean_plateau_command_variation": mean(row["plateau_variation"] for row in rows)}


def shadow_replay(legs, tuning):
    results = []
    for leg, samples in sorted(legs.items()):
        observer = YawESO(tuning, max_interval_s=0.12)
        es_errors, held_errors, intervals = [], [], []
        previous = None
        for row in samples:
            tick, gyro = row["tick"], row["gyro"]
            if previous is None:
                observer.step(gyro, 0.0, DT)
            else:
                dt = ((tick - previous["tick"]) & 0xffffffff) * 1e-6
                if dt == 0.0:
                    continue
                innovation = observer.step(gyro, previous["applied_yaw"], dt)
                if innovation is not None:
                    es_errors.append(innovation ** 2)
                    held_errors.append((gyro - previous["gyro"]) ** 2)
                    intervals.append(dt)
            previous = row
        results.append({"leg": leg, "samples": len(intervals), "median_dt_ms": 1000.0 * median(intervals),
                        "eso_next_rmse": math.sqrt(mean(es_errors)), "hold_last_rmse": math.sqrt(mean(held_errors))})
    return results


def observer_sanity(tuning):
    observer = YawESO(tuning)
    hidden_disturbance = 0.7
    true_rate = -0.14
    observer.step(true_rate, 0.0, DT)
    for count in range(450):
        applied = 0.07 * math.sin(count * 0.06)
        true_rate += DT * (tuning.b0 * applied + hidden_disturbance)
        observer.step(true_rate, applied, DT)
    assert abs(observer.rate - true_rate) < 1e-8 and abs(observer.disturbance - hidden_disturbance) < 1e-8
    assert observer.step(float("nan"), 0.0, DT) is None
    assert observer.rate is None and observer.disturbance == 0.0
    observer.step(-0.2, 0.0, DT)
    assert observer.step(-0.2, 0.0, 0.30) is None and observer.rate is None
    assert bounded_yaw(0.0, 0.37, 0.3, 1.6, DT) == 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repo", type=Path)
    parser.add_argument("report", type=Path, nargs="?")
    parser.add_argument("--verify", action="store_true",
                        help="用 results.json 中的参数复算，不重新搜索，也不改写结果")
    args = parser.parse_args()
    if not args.verify and args.report is None:
        parser.error("完整搜索需要提供输出路径")
    cfg = load_config(args.repo)
    recorded_path = Path(__file__).with_name("results.json")
    inputs = json.loads(Path(__file__).with_name("replay_inputs.json").read_text(encoding="utf-8"))
    if args.verify:
        recorded = json.loads(recorded_path.read_text(encoding="utf-8"))
        chosen = ObserverTuning(**recorded["tuning"])
        chosen_p = ProportionalTuning(**recorded["retuned_proportional_tuning"])
        evaluated = evaluate(cfg, chosen, chosen_p)
        report = {"sample_rate_hz": 100, "tuning": asdict(chosen),
                  "training_count": evaluated["training_count"],
                  "training_adrc": evaluated["training_adrc"],
                  "holdout_count": evaluated["holdout_count"],
                  "holdout_baseline": evaluated["holdout_baseline"],
                  "holdout_adrc": evaluated["holdout_adrc"],
                  "retuned_proportional_tuning": asdict(chosen_p),
                  "training_retuned_proportional": evaluated["training_retuned_proportional"],
                  "holdout_retuned_proportional": evaluated["holdout_retuned_proportional"],
                  "groups": evaluated["groups"],
                  "cases_with_higher_rmse": evaluated["cases_with_higher_rmse"],
                  "worst_rmse_increase": evaluated["worst_rmse_increase"],
                  "cases_with_higher_rmse_than_retuned_p": evaluated["cases_with_higher_rmse_than_retuned_p"],
                  "observer_shadow_replay": shadow_replay(inputs["20hz"], chosen),
                  "observer_shadow_replay_current_kd": shadow_replay(inputs["10hz"], chosen)}
        mismatches = list(metric_mismatches(report, recorded))
        if mismatches:
            raise SystemExit("ADRC 复算与 results.json 不一致:\n" + "\n".join(mismatches))
        print("PASS: recorded ADRC metrics reproduced without retuning")
        return
    training = training_cases()
    best = None
    for candidate in map(lambda values: ObserverTuning(*values),
                         product((8.0, 12.0, 18.0, 24.0), (5.0, 8.0, 12.0, 16.0), (9.0, 15.0, 22.0, 30.0))):
        scores = stats([compare_run(case, cfg, candidate) for case in training])
        objective = scores["mean_rmse"] + 0.25 * scores["p95_overspeed"]
        if best is None or objective < best[0]:
            best = (objective, candidate, scores)
    _, chosen, fitted = best
    best_p = None
    for values in product((0.6, 0.9, 1.4, 2.0, 3.0, 5.0), (0.005, 0.01, 0.025, 0.05),
                          (0.0, 0.0125, 0.025)):
        candidate = ProportionalTuning(*values)
        scores = stats([compare_run(case, cfg, proportional=candidate) for case in training])
        objective = scores["mean_rmse"] + 0.25 * scores["p95_overspeed"]
        if best_p is None or objective < best_p[0]:
            best_p = (objective, candidate, scores)
    _, chosen_p, fitted_p = best_p
    evaluated = evaluate(cfg, chosen, chosen_p)
    report = {"sample_rate_hz": 100, "tuning": asdict(chosen), "training_count": evaluated["training_count"],
              "training_adrc": evaluated["training_adrc"], "holdout_count": evaluated["holdout_count"],
              "holdout_baseline": evaluated["holdout_baseline"], "holdout_adrc": evaluated["holdout_adrc"],
              "retuned_proportional_tuning": asdict(chosen_p),
              "training_retuned_proportional": evaluated["training_retuned_proportional"],
              "holdout_retuned_proportional": evaluated["holdout_retuned_proportional"],
              "groups": evaluated["groups"],
              "cases_with_higher_rmse": evaluated["cases_with_higher_rmse"],
              "worst_rmse_increase": evaluated["worst_rmse_increase"],
              "cases_with_higher_rmse_than_retuned_p": evaluated["cases_with_higher_rmse_than_retuned_p"],
              "observer_shadow_replay": shadow_replay(inputs["20hz"], chosen),
              "observer_shadow_replay_current_kd": shadow_replay(inputs["10hz"], chosen)}
    if abs(evaluated["training_adrc"]["mean_rmse"] - fitted["mean_rmse"]) > 1e-15:
        raise SystemExit("training ADRC stats drifted during evaluation")
    if abs(evaluated["training_retuned_proportional"]["mean_rmse"] - fitted_p["mean_rmse"]) > 1e-15:
        raise SystemExit("training proportional stats drifted during evaluation")
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
