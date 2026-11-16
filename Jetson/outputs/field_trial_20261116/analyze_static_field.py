#!/usr/bin/env python3
"""Offline static-scene analysis of vision_probe JSON; stdlib only, no device access.

Geometry must come from FieldObservation, in original-image pixels. Stability is
not accuracy: without labelled image boundaries this report cannot measure bias,
false positives, or metric/world-position error. No acceptance thresholds applied.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics


GEOMETRY = {"corners": 4, "left_region": 4, "right_region": 4, "split_line": 2}
DIAGNOSTICS = ("score", "red_ratio", "split_confidence", "split_x", "split_measured",
               "jump_ratio", "measured_edges")


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * fraction
    lo, hi = math.floor(position), math.ceil(position)
    return values[lo] + (values[hi] - values[lo]) * (position - lo)


def stats(values):
    values = [v for v in values if finite(v)]
    if not values:
        return {"n": 0, "mean": None, "median": None, "stddev": None,
                "min": None, "p05": None, "p95": None, "max": None}
    return {"n": len(values), "mean": statistics.fmean(values),
            "median": statistics.median(values), "stddev": statistics.pstdev(values),
            "min": min(values), "p05": percentile(values, .05),
            "p95": percentile(values, .95), "max": max(values)}


def points(row, key):
    raw = row.get(key)
    if not isinstance(raw, list) or len(raw) != GEOMETRY[key]:
        return None
    result = []
    for point in raw:
        if isinstance(point, dict):
            point = [point.get(axis) for axis in ("x", "y", "z")]
        if not isinstance(point, (list, tuple)) or len(point) != 3:
            return None
        if not all(finite(v) for v in point) or point[2] != 0:
            return None
        result.append(tuple(point[:2]))
    return result


def area(quad):
    return abs(sum(a[0] * b[1] - a[1] * b[0]
                   for a, b in zip(quad, quad[1:] + quad[:1]))) * .5


def center(quad):
    # Same diagonal-intersection definition as interface_pkg/app/core/projective_center.hpp.
    ax, ay = (quad[2][i] - quad[0][i] for i in (0, 1))
    bx, by = (quad[3][i] - quad[1][i] for i in (0, 1))
    denominator = ax * by - ay * bx
    scale = math.hypot(ax, ay) * math.hypot(bx, by)
    if not finite(scale) or scale == 0 or abs(denominator) <= scale * 1e-9:
        return None
    dx, dy = (quad[1][i] - quad[0][i] for i in (0, 1))
    t, u = (dx * by - dy * bx) / denominator, (dx * ay - dy * ax) / denominator
    if not all(finite(v) and 0 <= v <= 1 for v in (t, u)):
        return None
    return quad[0][0] + t * ax, quad[0][1] + t * ay


def drift(series, adjacent_pairs):
    """series contains (original unique-row index, points), never candidate/HOLD geometry."""
    if not series:
        return {"frames": 0, "points": [], "adjacent_frame_rms_px": stats([])}
    output = []
    for index in range(len(series[0][1])):
        xy = [entry[1][index] for entry in series]
        median = [statistics.median(point[axis] for point in xy) for axis in (0, 1)]
        output.append({"median_xy_px": median,
                       "x_px": stats([p[0] for p in xy]), "y_px": stats([p[1] for p in xy]),
                       "distance_from_median_px": stats([math.dist(p, median) for p in xy])})
    adjacent = [math.sqrt(statistics.fmean(math.dist(p, q) ** 2 for p, q in zip(a[1], b[1])))
                for a, b in zip(series, series[1:]) if (a[0], b[0]) in adjacent_pairs]
    return {"frames": len(series), "points": output, "adjacent_frame_rms_px": stats(adjacent)}


def runs(rows, wanted):
    result, current = [], []
    for row in rows:
        accepted = row.get("valid") is True and row.get("committed") is True
        if accepted == wanted:
            current.append(row)
        elif current:
            result.append(current)
            current = []
    if current:
        result.append(current)
    if not result:
        return {"runs": 0, "longest_frames": 0, "longest_observed_source_span_s": 0.0,
                "longest_observed_receive_span_s": 0.0}
    def span(group, key, divisor):
        first, last = group[0].get(key), group[-1].get(key)
        return max(0., (last - first) / divisor) if finite(first) and finite(last) else 0.
    return {"runs": len(result), "longest_frames": max(map(len, result)),
            "longest_observed_source_span_s": max(span(g, "source_stamp_ns", 1e9) for g in result),
            "longest_observed_receive_span_s": max(span(g, "receive_monotonic_s", 1.) for g in result)}


def analyze(report, scene_notes=(), frame_size=None):
    rows = report.get("raw", {}).get("field", [])
    if not isinstance(rows, list) or any(not isinstance(row, dict) for row in rows):
        raise ValueError("expected raw.field list of observation objects")
    if not scene_notes:
        source_notes = report.get("scene_notes", [])
        scene_notes = [source_notes] if isinstance(source_notes, str) else source_notes
    if frame_size is None:
        dimensions = [report.get("frame_width"), report.get("frame_height")]
        if all(type(value) is int and value > 0 for value in dimensions):
            frame_size = dimensions
    unique, seen, duplicates = [], set(), 0
    invalid_stamps = 0
    for row in rows:
        stamp = row.get("source_stamp_ns")
        if type(stamp) is not int or stamp <= 0:
            invalid_stamps += 1
            continue
        key = row.get("frame_id", ""), stamp
        if key in seen:
            duplicates += 1
            continue
        seen.add(key)
        unique.append(row)
    accepted = [row for row in unique if row.get("valid") is True and row.get("committed") is True]
    duration = report.get("summary", {}).get("sample_duration_s")
    source_intervals = [(b["source_stamp_ns"] - a["source_stamp_ns"]) / 1e6
                        for a, b in zip(unique, unique[1:])
                        if a.get("frame_id", "") == b.get("frame_id", "")]
    arrival_intervals = [(b.get("receive_monotonic_s") - a.get("receive_monotonic_s")) * 1000
                         for a, b in zip(rows, rows[1:])
                         if finite(a.get("receive_monotonic_s")) and finite(b.get("receive_monotonic_s"))]
    adjacent_pairs = {(index, index + 1) for index, (a, b) in enumerate(zip(unique, unique[1:]))
                      if a.get("frame_id", "") == b.get("frame_id", "")
                      and b["source_stamp_ns"] > a["source_stamp_ns"]}
    output = {
        "schema": 1, "analysis": "static_field_observations", "scene_notes": list(scene_notes),
        "stationarity_verified_by_analyzer": False,
        "frame_size_px": frame_size, "source_probe_ok": report.get("ok"),
        "source_probe_errors": report.get("errors", []), "sample_duration_s": duration,
        "counts": {"received": len(rows), "unique_source_frames": len(unique),
                   "duplicate_source_frames": duplicates, "invalid_source_stamps": invalid_stamps,
                   "valid": sum(row.get("valid") is True for row in unique),
                   "valid_and_committed": len(accepted),
                   "invalid_or_uncommitted": len(unique) - len(accepted),
                   "valid_and_committed_fraction": len(accepted) / len(unique) if unique else None,
                   "unique_received_fps": len(unique) / duration if finite(duration) and duration > 0 else None,
                   "switched": sum(row.get("switched") is True for row in unique)
                   if any("switched" in row for row in unique) else None,
                   "split_suspect": sum(row.get("split_suspect") is True for row in unique)
                   if any("split_suspect" in row for row in unique) else None},
        "field_coverage_unique_frames": {key: sum(key in row for row in unique)
                                          for key in (*GEOMETRY, *DIAGNOSTICS, "frame_id", "edge_states",
                                                      "switched", "split_suspect", "bev_width", "bev_height")},
        "source_interval_ms": stats(source_intervals), "receive_interval_ms": stats(arrival_intervals),
        "nonincreasing_unique_source_intervals": sum(value <= 0 for value in source_intervals),
        "valid_runs": runs(unique, True), "invalid_or_uncommitted_runs": runs(unique, False),
        "labels": {key: dict(Counter(str(row.get(key, "<missing>")) for row in unique))
                   for key in ("state", "quality", "reason", "hold_reason", "measured_edges")},
        "diagnostics_all_unique": {key: stats([row.get(key) for row in unique]) for key in DIAGNOSTICS},
        "diagnostics_valid_committed": {key: stats([row.get(key) for row in accepted]) for key in DIAGNOSTICS},
        "geometry_by_frame_id": {},
        "limitations": [
            "Stability assumes a stationary camera, robot and scene throughout this interval; scene_notes are operator context, not inferred from observations.",
            "No ground truth: low pixel jitter and high valid rate do not prove correct field boundaries, correct partition, or metric/world-position accuracy.",
            "Geometry uses only valid=true AND committed=true observations with finite original-image [x,y,z=0] points; candidate/HOLD geometry is excluded.",
            "Frames are deduplicated by (frame_id, source_stamp_ns), preserving arrival order. There is no source sequence in FieldObservation, so transport loss cannot be counted exactly.",
            "Source stamps describe backend delivery, not exposure. Source and monotonic clocks are never subtracted; these metrics do not measure end-to-end latency.",
            "Adjacent geometry deltas require consecutive observed source frames with usable geometry. Source gaps may contain unobserved frames; no missing-frame geometry is interpolated.",
            "Longest run spans cover observed first-to-last messages and are lower bounds, not exact onset/recovery durations. Singleton spans are zero.",
            "No automatic acceptance thresholds or pass/fail verdict are introduced."
        ],
    }
    groups = defaultdict(list)
    for index, row in enumerate(unique):
        if row.get("valid") is True and row.get("committed") is True:
            groups[row.get("frame_id", "<missing>")].append((index, row))
    for frame_id, entries in groups.items():
        group = {"accepted_frames": len(entries), "geometry": {}, "geometry_issues": {},
                 "split_fraction_of_bev_width": stats([
                     row["split_x"] / (row["bev_width"] - 1) for _, row in entries
                     if finite(row.get("split_x")) and finite(row.get("bev_width")) and row["bev_width"] > 1])}
        for key, count in GEOMETRY.items():
            series = [(index, points(row, key)) for index, row in entries]
            usable = [(index, value) for index, value in series if value is not None]
            result = drift(usable, adjacent_pairs)
            if frame_size is not None:
                width, height = frame_size
                outside = [sum(not (0 <= p[0] <= width - 1 and 0 <= p[1] <= height - 1)
                               for p in value) for _, value in usable]
                result["outside_image"] = {"frames": sum(count > 0 for count in outside),
                                           "vertices": sum(outside),
                                           "coordinate_bounds_px": [0, 0, width - 1, height - 1]}
            group["geometry_issues"][key] = {
                "missing_field": sum(key not in row for _, row in entries),
                "missing_or_malformed_frames": len(entries) - len(usable)}
            if count == 4:
                result["area_px2"] = stats([area(value) for _, value in usable])
                centers = [(index, center(value)) for index, value in usable]
                result["projective_center"] = drift(
                    [(index, [value]) for index, value in centers if value is not None], adjacent_pairs)
                result["degenerate_projective_centers"] = sum(value is None for _, value in centers)
            group["geometry"][key] = result
        group["edge_state_counts"] = dict(Counter(
            ",".join(map(str, row.get("edge_states", []))) for _, row in entries))
        output["geometry_by_frame_id"][frame_id] = group
    for key in ("detect", "stream", "detect_counters", "stream_counters", "max_receive_gap_s"):
        output["source_" + key] = report.get("summary", {}).get(key)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-note", action="append", default=[])
    parser.add_argument("--frame-size", nargs=2, type=int, metavar=("WIDTH", "HEIGHT"))
    args = parser.parse_args()
    if args.frame_size and any(value <= 0 for value in args.frame_size):
        parser.error("frame dimensions must be positive")
    if args.input.resolve() == args.output.resolve():
        parser.error("input and output must be different files")
    raw = args.input.read_bytes()
    report = json.loads(raw.decode("utf-8-sig"))
    result = analyze(report, args.scene_note, args.frame_size)
    result["input"] = {"path": str(args.input.resolve()), "sha256": hashlib.sha256(raw).hexdigest()}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "counts": result["counts"],
                      "geometry_frame_ids": list(result["geometry_by_frame_id"])}, ensure_ascii=True))


if __name__ == "__main__":
    main()
