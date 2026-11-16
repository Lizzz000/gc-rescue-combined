"""Offline comparisons of swd_session.py JSONL; never connects to a device.

Example: python analyze_runs.py STM32/outputs/live_tuning_20261116/*move*.jsonl
Use --format csv for a compact table. The complete JSON summary is always saved.
Motor feedback and encoder-derived body speeds are wheel-based estimates, not
independent measurements of chassis motion on the ground.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
from pathlib import Path
import statistics
import sys

RADIUS_M = 0.035
CENTER_M = 0.108
SQRT3 = 1.7320508
TICK_PERIOD_MS = 2**32
ENCODER_PERIOD_RAD = 25.0
PHASE_RUN, PHASE_BRAKE, PHASE_DONE = 2, 3, 4


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def stats(values):
    values = [float(value) for value in values if finite(value)]
    return {"n": len(values), "mean": statistics.fmean(values) if values else None,
            "std": statistics.pstdev(values) if len(values) > 1 else (0.0 if values else None),
            "min": min(values) if values else None, "max": max(values) if values else None}


def elapsed_ms(start, end):
    if not finite(start) or not finite(end):
        return None
    delta = (end - start) % TICK_PERIOD_MS
    return float(delta) if delta < TICK_PERIOD_MS / 2 else None


def wrapped_delta(start, end, period):
    return (end - start + period / 2) % period - period / 2


def sample_of(record):
    return record.get("sample") if isinstance(record.get("sample"), dict) else {}


def field(record, name):
    return sample_of(record).get(name)


def matches_request(record, sequence):
    return record.get("ack_seq") == sequence and record.get("request_seq", sequence) == sequence


def phase_is(record, phase):
    # Scalars and the coherent sample may straddle a phase transition. Requiring
    # both prevents an old RUN snapshot from becoming part of a STOP command.
    return record.get("phase") == phase and field(record, "phase") == phase


def deduplicate(records):
    unique, seen = [], set()
    duplicates = 0
    for record in records:
        if not sample_of(record):
            continue
        sequence = record.get("sample_seq")
        if sequence is None or not isinstance(sequence, int) or sequence % 2:
            continue
        if sequence in seen:
            duplicates += 1
            continue
        seen.add(sequence)
        unique.append(record)
    return unique, duplicates


def intervals(records):
    device, host = [], []
    for first, second in zip(records, records[1:]):
        dt = elapsed_ms(field(first, "tick_ms"), field(second, "tick_ms"))
        if dt is not None:
            device.append(dt)
        if finite(first.get("host_s")) and finite(second.get("host_s")):
            host.append((second["host_s"] - first["host_s"]) * 1000)
    return {"device_ms": stats(device), "host_ms": stats(host)}


def motor_targets(vx, vy, wz):
    rotate = CENTER_M * wz
    # Geometry: front/left rear/right rear; all three motor directions are -1.
    return [-(vy + rotate) / RADIUS_M,
            -(0.8660254 * vx - 0.5 * vy + rotate) / RADIUS_M,
            -(-0.8660254 * vx - 0.5 * vy + rotate) / RADIUS_M]


def target_translation(targets):
    """Project motor targets to body vx/vy; a common yaw term cancels exactly."""
    front, left, right = targets
    return ((right - left) * RADIUS_M / SQRT3,
            (-2 * front + left + right) * RADIUS_M / 3)


def steady_samples(run, expected, mode, dwell_ms, tolerance, step_tolerance):
    selected, segments, current = set(), [], []
    translation = mode == "translation"
    expected_body = target_translation(expected) if translation else None
    # Convert the existing wheel rad/s thresholds to tangential m/s thresholds.
    # Heading correction is allowed to change all three wheel targets together;
    # it must not split a constant body-translation segment.
    body_tolerance = tolerance * RADIUS_M
    body_step_tolerance = step_tolerance * RADIUS_M

    def finish():
        if len(current) > 1:
            span = elapsed_ms(field(run[current[0]], "tick_ms"), field(run[current[-1]], "tick_ms"))
            if span is not None and span >= dwell_ms:
                selected.update(current)
                segments.append({"first_run_sample": current[0], "last_run_sample": current[-1],
                                 "duration_ms": span, "n": len(current)})
        current.clear()

    for index, record in enumerate(run):
        targets = [field(record, f"target_{i + 1}") for i in range(3)]
        near = all(finite(value) for value in targets)
        if near:
            if translation:
                body = target_translation(targets)
                near = math.hypot(body[0] - expected_body[0], body[1] - expected_body[1]) <= body_tolerance
            else:
                near = all(abs(targets[i] - expected[i]) <= tolerance for i in range(3))
        if not near:
            finish()
            continue
        if current:
            previous = run[current[-1]]
            dt = elapsed_ms(field(previous, "tick_ms"), field(record, "tick_ms"))
            previous_targets = [field(previous, f"target_{i + 1}") for i in range(3)]
            if translation:
                previous_body = target_translation(previous_targets)
                small_step = math.hypot(body[0] - previous_body[0], body[1] - previous_body[1]) <= body_step_tolerance
            else:
                small_step = all(abs(targets[i] - previous_targets[i]) <= step_tolerance for i in range(3))
            stable = dt is not None and 0 < dt <= 150 and small_step
            if not stable:
                finish()
        current.append(index)
    finish()
    return selected, segments


def encoder_rates(run, steady_indices):
    rates, steady = [], []
    rejected = 0
    for index, (first, second) in enumerate(zip(run, run[1:])):
        dt_ms = elapsed_ms(field(first, "tick_ms"), field(second, "tick_ms"))
        positions = [(field(first, f"pos_{i}"), field(second, f"pos_{i}")) for i in (1, 2, 3)]
        # At the configured 40 rad/s motor limit, 25 rad wrapping becomes
        # ambiguous beyond 312.5 ms. Use a stricter 250 ms limit for inference.
        if dt_ms is None or not 0 < dt_ms <= 250 or not all(finite(a) and finite(b) for a, b in positions):
            rejected += 1
            continue
        wheel = [-wrapped_delta(a, b, ENCODER_PERIOD_RAD) / (dt_ms / 1000) for a, b in positions]
        if any(abs(value) > 40.5 for value in wheel):
            rejected += 1
            continue
        vx = (wheel[1] - wheel[2]) * RADIUS_M / SQRT3
        wz = sum(wheel) * RADIUS_M / (3 * CENTER_M)
        item = {"vx": vx, "wz": wz, "dt_ms": dt_ms}
        rates.append(item)
        if dt_ms <= 150 and index in steady_indices and index + 1 in steady_indices:
            steady.append(item)
    return {"run": {name: stats(row[name] for row in rates) for name in ("vx", "wz")},
            "steady": {name: stats(row[name] for row in steady) for name in ("vx", "wz")},
            "valid_interval_ms": stats(row["dt_ms"] for row in rates),
            "rejected_intervals": rejected, "position_period_rad": ENCODER_PERIOD_RAD}


def yaw_change(records):
    values = [field(row, "yaw_deg") for row in records]
    if len(values) < 2 or not all(finite(value) for value in values):
        return None
    return sum(wrapped_delta(a, b, 360) for a, b in zip(values, values[1:]))


def rms_errors(run):
    result = []
    for motor in (1, 2, 3):
        errors = [field(row, f"vel_{motor}") - field(row, f"target_{motor}") for row in run
                  if finite(field(row, f"vel_{motor}")) and finite(field(row, f"target_{motor}"))]
        result.append({"motor_id": motor, "n": len(errors),
                       "rmse_rad_s": math.sqrt(statistics.fmean(e * e for e in errors)) if errors else None,
                       "mean_error_rad_s": statistics.fmean(errors) if errors else None})
    return result


def braking_metrics(owned, run, events):
    brake = next((row for row in owned if phase_is(row, PHASE_BRAKE)), None)
    done = next((row for row in owned if phase_is(row, PHASE_DONE) and row.get("error") == 0), None)
    quiet = next((row for row in owned if phase_is(row, PHASE_BRAKE) and
                  all(finite(field(row, f"vel_{i}")) and abs(field(row, f"vel_{i}")) <= 0.20
                      and field(row, f"state_{i}") == 1 for i in (1, 2, 3))), None)
    tick = lambda row: field(row, "tick_ms") if row else None
    first_brake, first_done = tick(brake), tick(done)
    last_run = tick(run[-1]) if run else None
    elapsed = elapsed_ms(first_brake, first_done)
    # Completion can be observed between two 20ms telemetry publications. Keep
    # the scalar phase observation's host timing separately instead of assigning
    # its older sample tick to the completion event.
    host_brake = next((row.get("host_s") for row in events if row.get("phase") == PHASE_BRAKE), None)
    host_done = next((row.get("host_s") for row in events if row.get("phase") == PHASE_DONE and
                      row.get("error") == 0), None)
    host_elapsed = ((host_done - host_brake) * 1000 if finite(host_brake) and finite(host_done)
                    and host_done >= host_brake else None)
    return {"first_brake_tick_ms": first_brake,
            "first_done_tick_ms": first_done,
            "first_quiet_observation_after_brake_ms": elapsed_ms(first_brake, tick(quiet)),
            "brake_to_stop_and_disable_confirm_observed_ms": elapsed,
            "last_run_to_done_observed_ms": elapsed_ms(last_run, first_done),
            "onset_observation_gap_ms": elapsed_ms(last_run, first_brake),
            "host_brake_to_done_observed_ms": host_elapsed,
            "note": "BRAKING includes brake and disable acknowledgement; a single quiet sample is not the firmware's three-feedback stop confirmation."}


def analyze_move(path, request, records, options, run_index):
    sequence = request.get("sequence")
    args = request.get("args", [])
    args = list(args) if isinstance(args, list) else []
    vx, vy, wz, duration = (args + [None] * 4)[:4]
    mode = "translation" if finite(vx) and vx != 0 and wz == 0 else (
        "rotation" if finite(wz) and wz != 0 and vx == 0 else "unknown")
    if not all(finite(value) for value in (vx, vy, wz)):
        mode = "unknown"
    command_value = vx if mode == "translation" else wz if mode == "rotation" else None
    sign = math.copysign(1, command_value) if finite(command_value) and command_value else None
    unique, duplicates = deduplicate(records)
    owned = [row for row in unique if matches_request(row, sequence) and row.get("command", 4) == 4]
    run = [row for row in owned if phase_is(row, PHASE_RUN)]
    expected = motor_targets(vx, vy, wz) if all(finite(v) for v in (vx, vy, wz)) else [None] * 3
    selected, segments = steady_samples(run, expected, mode, options.steady_ms,
                                         options.target_tolerance, options.target_step_tolerance) if mode != "unknown" else (set(), [])
    steady = [row for index, row in enumerate(run) if index in selected]
    primary = "body_vx" if mode == "translation" else "wz_imu"
    direction_values = [sign * field(row, primary) for row in steady
                        if sign is not None and finite(field(row, primary))]
    config_fields = ("kd_1", "kd_2", "kd_3", "ff_gain", "friction_1", "friction_2", "friction_3")
    configs = sorted({tuple(field(row, key) for key in config_fields) for row in run}, key=repr)
    config = dict(zip(config_fields, configs[0])) if configs else None
    completed = any(row.get("event") == "complete" and matches_request(row, sequence) and
                    row.get("phase") == PHASE_DONE and row.get("error") == 0 for row in records)
    result = {
        "file": str(path.resolve()), "run_index": run_index, "request_seq": sequence,
        "command": {"id": 4, "args": args, "mode": mode, "vx_mps": vx, "vy_mps": vy,
                    "wz_rad_s": wz, "duration_ms": duration, "direction_sign": sign},
        "config": config, "config_changed_during_run": len(configs) > 1,
        "samples": {"unique_all_phases": len(unique), "duplicates_removed": duplicates,
                    "command_owned": len(owned), "run": len(run), "steady": len(steady)},
        "intervals": {"all_phases": intervals(unique), "run": intervals(run)},
        "run_motor_tracking": rms_errors(run),
        "steady_selection": {"expected_motor_rad_s": expected, "minimum_contiguous_ms": options.steady_ms,
                             "basis": "projected_body_translation" if mode == "translation" else "individual_motor_targets",
                             "body_translation_tolerance_mps": options.target_tolerance * RADIUS_M if mode == "translation" else None,
                             "maximum_adjacent_body_translation_change_mps": options.target_step_tolerance * RADIUS_M if mode == "translation" else None,
                             "target_tolerance_rad_s": options.target_tolerance,
                             "maximum_adjacent_target_change_rad_s": options.target_step_tolerance,
                             "segments": segments},
        "steady": {"body_vx_wheel_estimate_mps": stats(field(row, "body_vx") for row in steady),
                   "imu_wz_rad_s": stats(field(row, "wz_imu") for row in steady),
                   "along_request": stats(direction_values), "along_request_source": primary,
                   "along_request_mean_error": statistics.fmean(direction_values) - abs(command_value)
                   if direction_values else None},
        "run_imu_wz_rad_s": stats(field(row, "wz_imu") for row in run),
        "run_yaw_change_deg": yaw_change(run), "steady_yaw_change_deg": yaw_change(steady),
        "encoder_wheel_estimate": encoder_rates(run, selected),
        "braking": braking_metrics(owned, run, [row for row in records if matches_request(row, sequence)]),
        "move_complete": completed,
        "final_stop_confirmed": any(row.get("event") == "final_stop_confirmed" for row in records),
        "failures": [row.get("message") for row in records if row.get("event") in
                     ("failure", "session_error", "final_stop_unconfirmed")],
    }
    if not steady:
        result["steady_selection"]["note"] = "No sufficiently long contiguous target-steady segment; steady statistics remain empty."
    return result


def load_log(path):
    records = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{line_number}: invalid JSON: {exc.msg}") from exc
        if not isinstance(record, dict):
            raise ValueError(f"{path}:{line_number}: expected a JSON object")
        records.append(record)
    return records


def analyze_file(path, options):
    records = load_log(path)
    moves = [index for index, row in enumerate(records) if row.get("event") == "request" and row.get("command") == 4]
    results = []
    for run_index, start in enumerate(moves, 1):
        stop = moves[run_index] if run_index < len(moves) else len(records)
        # A new session can restart sample_seq, so it ends the previous run.
        stop = next((i for i in range(start + 1, stop) if records[i].get("event") == "session"), stop)
        results.append(analyze_move(path, records[start], records[start + 1:stop], options, run_index))
    return results


def compact_row(run):
    command, config = run["command"], run["config"] or {}
    row = {"file": Path(run["file"]).name, "request_seq": run["request_seq"], "mode": command["mode"],
           "vx": command["vx_mps"], "wz": command["wz_rad_s"], "duration_ms": command["duration_ms"],
           **config, "run_n": run["samples"]["run"], "steady_n": run["samples"]["steady"],
           "sample_dt_mean_ms": run["intervals"]["run"]["device_ms"]["mean"],
           "sample_dt_max_ms": run["intervals"]["run"]["device_ms"]["max"]}
    row.update({f"rmse_motor_{item['motor_id']}": item["rmse_rad_s"] for item in run["run_motor_tracking"]})
    for name, values in (("steady_vx", run["steady"]["body_vx_wheel_estimate_mps"]),
                         ("steady_imu_wz", run["steady"]["imu_wz_rad_s"]),
                         ("encoder_vx", run["encoder_wheel_estimate"]["steady"]["vx"]),
                         ("encoder_wz", run["encoder_wheel_estimate"]["steady"]["wz"])):
        row[name + "_mean"], row[name + "_std"] = values["mean"], values["std"]
    row.update(along_request_mean_error=run["steady"]["along_request_mean_error"],
               yaw_change_deg=run["run_yaw_change_deg"],
               brake_confirm_ms=run["braking"]["brake_to_stop_and_disable_confirm_observed_ms"],
               host_brake_confirm_ms=run["braking"]["host_brake_to_done_observed_ms"],
               move_complete=run["move_complete"], final_stop_confirmed=run["final_stop_confirmed"])
    return row


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", help="JSONL paths, directories, or wildcard patterns")
    parser.add_argument("--output", type=Path, help="Saved complete JSON (default: first input directory/run_summary.json)")
    parser.add_argument("--format", choices=("json", "csv"), default="json", help="Compact stdout format")
    parser.add_argument("--steady-ms", type=float, default=150.0)
    parser.add_argument("--target-tolerance", type=float, default=0.20,
                        help="rad/s target tolerance; translation uses this times wheel radius in m/s")
    parser.add_argument("--target-step-tolerance", type=float, default=0.10,
                        help="adjacent target change in rad/s; translation converts to body m/s by wheel radius")
    options = parser.parse_args(argv)
    if not all(math.isfinite(v) and v > 0 for v in (options.steady_ms, options.target_tolerance, options.target_step_tolerance)):
        parser.error("Steady selection thresholds must be finite and positive")
    paths = []
    for value in options.logs:
        matches = sorted(glob.glob(value))
        if not matches:
            parser.error(f"No input matches: {value}")
        for match in matches:
            path = Path(match)
            paths.extend(sorted(path.glob("*.jsonl")) if path.is_dir() else [path])
    paths = list(dict.fromkeys(path.resolve() for path in paths))
    if not paths:
        parser.error("No JSONL files found")
    runs, without_moves = [], []
    try:
        for path in paths:
            analyzed = analyze_file(path, options)
            runs.extend(analyzed)
            if not analyzed:
                without_moves.append(str(path))
        output = (options.output or paths[0].parent / "run_summary.json").resolve()
        if output in paths:
            parser.error("Summary output must not overwrite an input log")
        output.parent.mkdir(parents=True, exist_ok=True)
        summary = {"schema_version": 1, "runs": runs, "files_without_move_requests": without_moves,
                   "notes": ["All means/stds are unweighted sample statistics; actual sample intervals are reported.",
                             "Body speed and encoder-derived speeds are wheel-based estimates, not independent ground truth.",
                             "Compare matched direction, floor, load, duration and command; differences alone do not prove improved stability.",
                             "RUN metrics exclude mismatched requests, phase-transition samples and STOP tails."]}
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (OSError, ValueError) as exc:
        parser.exit(1, str(exc) + "\n")
    rows = [compact_row(run) for run in runs]
    if options.format == "csv" and rows:
        columns = list(dict.fromkeys(key for row in rows for key in row))
        writer = csv.DictWriter(sys.stdout, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    else:
        print(json.dumps({"summary_file": str(output), "runs": rows, "files_without_moves": len(without_moves)},
                         ensure_ascii=False, allow_nan=False))
    print(f"Saved summary: {output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
