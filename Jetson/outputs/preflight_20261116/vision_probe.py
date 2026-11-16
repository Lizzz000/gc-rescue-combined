#!/usr/bin/env python3
"""Sample ROS vision diagnostics after selecting seg/field; does not control motion.

Run in the new Jetson ROS overlay. Status FPS values are server diagnostics,
not independently measured image delivery rates or end-to-end latency.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import sys
import time


DETECT_METRICS = (
    "capture_fps", "seg_fps", "field_fps", "raw_fps", "seg_inference_ms",
    "field_inference_ms", "seg_gpu_ms", "seg_post_ms", "field_gpu_ms", "field_post_ms",
)
DETECT_COUNTERS = (
    "inference_replaced", "raw_replaced", "camera_reopens", "seg_skipped", "field_skipped",
)
STREAM_METRICS = ("image_fps", "compose_fps", "compose_ms", "stream_fps", "viewers", "instances")
STREAM_COUNTERS = ("replaced_frames",)


def stats(values):
    values = [value for value in values if value is not None and math.isfinite(value)]
    if not values:
        return {"n": 0, "mean": None, "min": None, "max": None, "last": None}
    return {"n": len(values), "mean": statistics.fmean(values),
            "min": min(values), "max": max(values), "last": values[-1]}


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def safe_scalar(value):
    return None if isinstance(value, float) and not math.isfinite(value) else value


def counter_deltas(rows, baseline, fields):
    result = {}
    for field in fields:
        series = ([baseline] if baseline else []) + rows
        if len(series) < 2:
            result[field] = {"delta": None, "observed_span_s": 0.0, "reset_detected": False}
            continue
        reset = any(b[field] < a[field] for a, b in zip(series, series[1:]))
        result[field] = {
            "delta": None if reset else series[-1][field] - series[0][field],
            "observed_span_s": series[-1]["receive_monotonic_s"] - series[0]["receive_monotonic_s"],
            "first": series[0][field], "last": series[-1][field], "reset_detected": reset,
        }
    return result


def arrival_gaps(rows, start, end):
    arrivals = [start] + [row["receive_monotonic_s"] for row in rows] + [end]
    return max((b - a for a, b in zip(arrivals, arrivals[1:])), default=0.0)


class Collector:
    def __init__(self, mode):
        self.mode = mode
        self.rows = {"detect": [], "stream": [], "field": []}
        self.latest = {}
        self.baselines = {}
        self.stage = None
        self.stage_events = []
        self.start = None
        self.end = None

    def stage_callback(self, message):
        self.stage = {"task": message.task, "epoch": int(message.epoch),
                      "source_stamp_ns": stamp_ns(message), "receive_monotonic_s": time.monotonic()}
        # Stage is periodically republished; retain changes rather than identical announcements.
        key = (message.task, int(message.epoch), stamp_ns(message))
        if not self.stage_events or self.stage_events[-1]["key"] != key:
            self.stage_events.append({"key": key, **self.stage})

    def status_callback(self, message, kind):
        fields = (DETECT_METRICS + DETECT_COUNTERS + ("raw_subscribers", "seg_active", "field_active", "serial_inference")
                  if kind == "detect" else STREAM_METRICS + STREAM_COUNTERS)
        row = {field: safe_scalar(getattr(message, field)) for field in fields}
        row.update(source_stamp_ns=stamp_ns(message), receive_monotonic_s=time.monotonic())
        self.latest[kind] = row
        self.append(kind, row)

    def field_callback(self, message):
        row = {field: getattr(message, field) for field in
               ("valid", "committed", "state", "quality", "reason", "hold_reason", "measured_edges")}
        row.update(source_stamp_ns=stamp_ns(message), receive_monotonic_s=time.monotonic())
        self.append("field", row)

    def append(self, kind, row):
        now = row["receive_monotonic_s"]
        if self.start is not None and self.start <= now < self.end:
            self.rows[kind].append(row)

    def begin(self, duration):
        self.start = time.monotonic()
        self.end = self.start + duration
        self.baselines = dict(self.latest)

    def summarize(self, finished):
        if self.start is None:
            return {"sample_duration_s": 0.0, "sample_counts": {key: 0 for key in self.rows}}
        end = min(self.end, finished)
        duration = max(0.0, end - self.start)
        detect, stream, field = (self.rows[key] for key in ("detect", "stream", "field"))
        result = {"sample_duration_s": duration, "sample_counts": {key: len(value) for key, value in self.rows.items()},
                  "detect": {name: stats([row[name] for row in detect]) for name in DETECT_METRICS},
                  "stream": {name: stats([row[name] for row in stream]) for name in STREAM_METRICS},
                  "detect_counters": counter_deltas(detect, self.baselines.get("detect"), DETECT_COUNTERS),
                  "stream_counters": counter_deltas(stream, self.baselines.get("stream"), STREAM_COUNTERS),
                  "max_receive_gap_s": {key: arrival_gaps(rows, self.start, end) for key, rows in self.rows.items()},
                  "mode_mismatch_status_count": sum(not row[self.mode + "_active"] or row[("field" if self.mode == "seg" else "seg") + "_active"] for row in detect)}
        # Received field messages are measured separately from server-reported FPS.
        unique = {row["source_stamp_ns"] for row in field}
        result["field"] = {
            "received": len(field), "unique_source_stamps": len(unique),
            "duplicate_source_stamps": len(field) - len(unique),
            "received_messages_per_s": len(field) / duration if duration else None,
            "valid": sum(bool(row["valid"]) for row in field),
            "valid_and_committed": sum(bool(row["valid"] and row["committed"]) for row in field),
            "invalid": sum(not row["valid"] for row in field),
            "states": dict(Counter(row["state"] for row in field)),
            "qualities": dict(Counter(row["quality"] for row in field)),
            "reasons": dict(Counter(row["reason"] for row in field)),
        }
        return result


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return number


def run(args, collector, report):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from interface_pkg.msg import DetectorStatus, StreamStatus, FieldObservation, TaskState
    from std_srvs.srv import SetBool

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node("preflight_vision_probe")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        stage_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(DetectorStatus, "/detect_node/status", lambda msg: collector.status_callback(msg, "detect"), qos)
        node.create_subscription(StreamStatus, "/stream_node/status", lambda msg: collector.status_callback(msg, "stream"), qos)
        node.create_subscription(FieldObservation, "/detect_node/field/results", collector.field_callback, qos)
        node.create_subscription(TaskState, "/detect_node/active_task", collector.stage_callback, stage_qos)
        client = node.create_client(SetBool, "/detect_node/set_field")
        if not client.wait_for_service(timeout_sec=args.timeout):
            raise TimeoutError("/detect_node/set_field service unavailable")
        request = SetBool.Request()
        request.data = args.mode == "field"
        future = client.call_async(request)
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if not future.done():
            raise TimeoutError("set_field response timeout")
        response = future.result()
        report["service_response"] = {"success": response.success, "message": response.message}
        if not response.success:
            raise RuntimeError("set_field rejected: " + response.message)
        warm_end = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < warm_end:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, warm_end - time.monotonic())))
        if not collector.stage or collector.stage["task"] != args.mode:
            raise RuntimeError("requested active_task was not observed during 3 s warmup")
        report["stage_at_sample_start"] = dict(collector.stage)
        collector.begin(args.duration)
        while rclpy.ok() and time.monotonic() < collector.end:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, collector.end - time.monotonic())))
        if not rclpy.ok():
            raise RuntimeError("ROS context stopped before sampling completed")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("seg", "field"), required=True)
    parser.add_argument("--duration", type=positive, default=15.0, help="sampling seconds, after fixed 3 s warmup")
    parser.add_argument("--timeout", type=positive, default=5.0, help="service wait/response timeout seconds")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    report = {"schema": 1, "probe": "vision", "mode": args.mode,
              "started_utc": datetime.now(timezone.utc).isoformat(),
              "requested_duration_s": args.duration, "warmup_s": 3.0, "errors": [],
              "notes": ["Status FPS/latencies are server diagnostics, not independently measured delivery FPS or end-to-end latency.",
                        "Counter deltas use the last warmup status through the last sampled status; actual observed span is reported.",
                        "Field valid/committed counts describe this camera scene, not overall recognition accuracy."]}
    collector = Collector(args.mode)
    try:
        run(args, collector, report)
    except KeyboardInterrupt:
        report["errors"].append("interrupted")
    except Exception as error:
        report["errors"].append(type(error).__name__ + ": " + str(error))
    summary = collector.summarize(time.monotonic())
    report.update(summary=summary, raw=collector.rows, counter_baselines=collector.baselines,
                  stage_events=collector.stage_events, stage_at_end=collector.stage)
    if collector.start is not None:
        for kind in ("detect", "stream"):
            if not collector.rows[kind]:
                report["errors"].append("no " + kind + " status samples")
            elif summary["max_receive_gap_s"][kind] > 3.5:
                report["errors"].append(kind + " status gap exceeded 3.5 s")
        for group, metric in (("detect", "capture_fps"), ("detect", args.mode + "_fps"), ("stream", "stream_fps")):
            value = summary[group][metric]
            if value["n"] == 0 or value["max"] <= 0 or value["last"] <= 0:
                report["errors"].append(group + "." + metric + " absent or zero at sample end")
        if summary["mode_mismatch_status_count"] or not collector.stage or collector.stage["task"] != args.mode:
            report["errors"].append("active mode changed or detector status did not match requested mode")
        if args.mode == "field" and (not collector.rows["field"] or summary["max_receive_gap_s"]["field"] > 3.5):
            report["errors"].append("field results absent or gap exceeded 3.5 s")
        if any(value["reset_detected"] for key in ("detect_counters", "stream_counters") for value in summary[key].values()):
            report["errors"].append("diagnostic counter reset detected; interval is not continuous")
    report["ok"] = not report["errors"]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    compact = {"ok": report["ok"], "mode": args.mode, "output": str(args.output),
               "sample_counts": summary["sample_counts"], "errors": report["errors"]}
    if collector.start is not None:
        compact.update(capture_fps=summary["detect"]["capture_fps"],
                       recognition_fps=summary["detect"][args.mode + "_fps"],
                       stream_server_fps=summary["stream"]["stream_fps"],
                       camera_reopens=summary["detect_counters"]["camera_reopens"], field=summary["field"])
    print(json.dumps(compact, ensure_ascii=False, allow_nan=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
