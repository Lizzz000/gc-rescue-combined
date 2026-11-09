#!/usr/bin/env python3
"""串行真实 GPU A/B；封存既有引擎、强制 builder guard、保留失败及原始张量。"""
from __future__ import annotations

import argparse
import array
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def require(value, message):
    if not value:
        raise RuntimeError(message)


def dump(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def identity(path):
    path = Path(path).resolve(strict=True)
    stat = path.stat()
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"path": str(path), "sha256": digest.hexdigest(), "size": stat.st_size,
            "mtime_ns": stat.st_mtime_ns}


def quantile(values, fraction):
    values = sorted(values)
    require(bool(values), "Empty performance window")
    position = (len(values) - 1) * fraction
    lo, hi = math.floor(position), math.ceil(position)
    return values[lo] + (values[hi] - values[lo]) * (position - lo)


def summarize(directory):
    with (directory / "frames.csv").open() as stream:
        frames = list(csv.DictReader(stream))
    with (directory / "windows.csv").open() as stream:
        windows = list(csv.DictReader(stream))
    require(len(windows) == 3, "Expected three completed windows")
    result = []
    for index, window in enumerate(windows):
        require(int(window["window"]) == index, "Window order changed")
        selected = [row for row in frames if int(row["window"]) == index]
        require(len(selected) == int(window["completed"]), "Missing per-frame timings")
        require([int(row["sequence"]) for row in selected] == list(range(len(selected))),
                "Non-contiguous completion sequence")
        item = {"window": index, "completed": len(selected),
                "wall_seconds": float(window["wall_seconds"]),
                "completed_per_second": float(window["completed_per_second"])}
        for key in ("raw_ready_ms", "completed_ms"):
            values = [float(row[key]) for row in selected]
            require(all(math.isfinite(x) and x >= 0 for x in values), "Invalid timing")
            item[key] = {"median": quantile(values, .5), "p95": quantile(values, .95)}
        result.append(item)
    return result


def compare(left, right, image_count, atol):
    lm = json.loads((left / "model.json").read_text())
    rm = json.loads((right / "model.json").read_text())
    require(lm == rm, "Engine tensor identities changed between backends")
    manifests = []
    for directory in (left, right):
        with (directory / "captures.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        require(len(rows) == image_count * len(lm["tensors"]), "Missing captured tensors")
        manifests.append(rows)
    require(manifests[0] == manifests[1], "Input/output frame pairing differs")
    records = []
    for row in manifests[0]:
        index = int(row["tensor_index"])
        tensor = lm["tensors"][index]
        values = []
        for directory in (left, right):
            raw = (directory / row["path"]).read_bytes()
            require(len(raw) == tensor["count"] * 4, "Truncated float32 tensor")
            data = array.array("f")
            data.frombytes(raw)
            require(data.itemsize == 4 and sys.byteorder == "little", "Unsupported float host")
            require(all(math.isfinite(x) for x in data), "NaN/Inf in model tensors")
            values.append(data)
        tolerance = 0.0 if tensor["name"] == "__input__" else atol
        maximum = max((abs(a - b) for a, b in zip(*values)), default=0.0)
        count = sum(abs(a - b) > tolerance for a, b in zip(*values))
        records.append({"frame": int(row["frame"]), "tensor": tensor["name"],
                        "elements": tensor["count"], "absolute_tolerance": tolerance,
                        "max_abs_difference": maximum, "different_elements": count,
                        "left": identity(left / row["path"]),
                        "right": identity(right / row["path"])})
    return {"passed": all(row["different_elements"] == 0 for row in records),
            "records": records, "scope": "ordered fixed images; full input and both output tensors",
            "postprocessing_executed": False, "ros_pipeline_executed": False}


STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


class StopRequested(RuntimeError):
    pass


class StopSignals:
    """首个停止信号退出正常工作；清理期间的重复信号只留证，不再打断回收。"""
    def __init__(self):
        self.received = None
        self.cleaning = False
        for signum in STOP_SIGNALS:
            signal.signal(signum, self.handle)

    def handle(self, signum, _frame):
        if self.received is None:
            self.received = signum
        if not self.cleaning:
            self.cleaning = True
            raise StopRequested(f"Received {signal.Signals(signum).name}")


def stop_process_group(process):
    """只回收 launch 创建的独立 session；TERM 最多 10 秒，KILL 后再等最多 5 秒。"""
    receipt = {"pid": process.pid, "pgid": process.pid, "term_sent": False, "kill_sent": False}

    def exists():
        try:
            os.killpg(process.pid, 0)
            return True
        except ProcessLookupError:
            return False

    def send(signum, key):
        try:
            os.killpg(process.pid, signum)
            receipt[key] = True
        except ProcessLookupError:
            pass

    def wait_until_empty(seconds):
        deadline = time.monotonic() + seconds
        while True:
            process.poll()  # 回收直接子进程，避免僵尸 leader 使组存在检查一直为真。
            if not exists():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(.05)

    send(signal.SIGTERM, "term_sent")
    empty = wait_until_empty(10)
    if not empty:
        send(signal.SIGKILL, "kill_sent")
        empty = wait_until_empty(5)
    receipt["returncode"] = process.poll()
    receipt["leader_reaped"] = receipt["returncode"] is not None
    receipt["group_empty"] = empty
    return receipt


def launch(command, env, output, timeout, stop_signals, pass_fds=()):
    begin = time.time_ns()
    process = None
    receipt = {"command": command, "start_ns": begin}
    with output.open("xb") as log:
        try:
            # Popen 返回并保存句柄前不交付父停止信号，防止子进程已启动却丢失回收句柄。
            old_mask = signal.pthread_sigmask(signal.SIG_BLOCK, STOP_SIGNALS)
            try:
                def reset_signal():
                    for signum in STOP_SIGNALS:
                        signal.signal(signum, signal.SIG_DFL)
                    signal.pthread_sigmask(signal.SIG_SETMASK, old_mask)
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           env=env, pass_fds=pass_fds, start_new_session=True,
                                           preexec_fn=reset_signal)
                receipt["pid"] = process.pid
            finally:
                signal.pthread_sigmask(signal.SIG_SETMASK, old_mask)
            code = process.wait(timeout=timeout)
        except BaseException as error:
            stop_signals.cleaning = True
            if process is not None:
                try:
                    receipt["cleanup"] = stop_process_group(process)
                except BaseException as cleanup_error:
                    receipt["cleanup_error"] = f"{type(cleanup_error).__name__}: {cleanup_error}"
            receipt["error"] = f"{type(error).__name__}: {error}"
            receipt["finish_ns"] = time.time_ns()
            error.launch_receipt = receipt
            raise
    receipt.update(returncode=code, finish_ns=time.time_ns())
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True, help="Fingerprint only; never supplied to nvinfer")
    parser.add_argument("--labels", type=Path, required=True, help="Fingerprint only; no DS parser")
    parser.add_argument("--parameters", type=Path, required=True, help="Frozen effective configuration")
    parser.add_argument("--images", type=Path, nargs="+", required=True, help="Explicit ordered original inputs")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--warmup-frames", type=int, default=120)
    parser.add_argument("--window-seconds", type=float, default=30)
    parser.add_argument("--atol", type=float, choices=(0.0, 1e-6), default=0.0)
    args = parser.parse_args()
    import fcntl
    require(args.engine.name == "seg.769056489e90ef35.trt", "Unexpected production engine identity")
    require(args.engine.resolve().parent == args.model.resolve().parent,
            "Engine and ONNX must be in the same existing model directory")
    require(0 < args.warmup_frames <= 10000 and math.isfinite(args.window_seconds)
            and 0 < args.window_seconds <= 300, "Invalid bounded workload")
    output = args.output.resolve()
    output.mkdir(parents=False, exist_ok=False)
    paths = [args.engine, args.model, args.labels, args.parameters, *args.images]
    before = [identity(path) for path in paths]
    executable = (args.build / "ds_compare").resolve(strict=True)
    guard = (args.build / "libno_engine_build.so").resolve(strict=True)
    build_identity = (args.build / "build-identity.txt").resolve(strict=True)
    dump(output / "inputs.json", {"assets": before, "runner": identity(executable),
         "guard": identity(guard), "build_identity": identity(build_identity),
         "warmup_frames_per_backend": args.warmup_frames, "windows": 3,
         "window_seconds": args.window_seconds, "atol": args.atol,
         "postprocess_note": "No confidence/NMS/labels/mask algorithm is applied; parameters are evidence only",
         "tool_sources": [identity(p) for p in sorted(Path(__file__).parent.iterdir()) if p.is_file()]})
    (output / "build-identity.txt").write_bytes(build_identity.read_bytes())
    fd = None
    status = {"status": "running", "engine_builds": 0, "steps": []}
    exit_code = 2
    stop_signals = StopSignals()
    try:
        require(not os.environ.get("LD_PRELOAD"), "Use a clean environment without other preloaded libraries")
        env = os.environ.copy()
        env["LD_PRELOAD"] = str(guard)
        env["DS_COMPARE_GUARD_PATH"] = str(guard)
        env["DS_COMPARE_GUARD_LOG"] = str(output / "guard-selftest.jsonl")
        selftest = launch([str(executable), "--guard-selftest"], env,
                          output / "guard-selftest.log", timeout=30, stop_signals=stop_signals)
        status["steps"].append(selftest)
        events = [json.loads(line) for line in (output / "guard-selftest.jsonl").read_text().splitlines()]
        require(selftest["returncode"] == 86 and
                sum(e["event"] == "builder_blocked" and e["attempts"] == 1 for e in events) == 1,
                "Mandatory builder interception selftest failed; GPU comparison will not start")
        require(any(e["event"] == "guard_loaded" for e in events), "Preload not observed")
        # 封存副本仅复用原始 plan 字节，不转换引擎；原目录绝不交给 nvinfer 写入。
        fd = os.memfd_create("readonly-existing-seg-plan", os.MFD_ALLOW_SEALING)
        with args.engine.open("rb") as source:
            for chunk in iter(lambda: source.read(1024 * 1024), b""):
                view = memoryview(chunk)
                while view:
                    view = view[os.write(fd, view):]
        os.lseek(fd, 0, os.SEEK_SET)
        seals = fcntl.F_SEAL_SEAL | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_GROW | fcntl.F_SEAL_WRITE
        fcntl.fcntl(fd, fcntl.F_ADD_SEALS, seals)
        require(fcntl.fcntl(fd, fcntl.F_GET_SEALS) == seals, "Could not seal engine bytes")
        with os.fdopen(os.dup(fd), "rb") as sealed:
            digest = hashlib.file_digest(sealed, "sha256").hexdigest()
        require(digest == before[0]["sha256"], "Sealed engine differs from original")
        for backend in ("direct", "deepstream"):
            env["DS_COMPARE_GUARD_LOG"] = str(output / f"guard-{backend}.jsonl")
            command = [str(executable), backend, f"/proc/self/fd/{fd}", str(output / backend),
                       str(args.warmup_frames), str(args.window_seconds),
                       *(str(p.resolve()) for p in args.images)]
            step = launch(command, env, output / f"{backend}.log",
                          timeout=180 + args.warmup_frames * 2 + 3 * args.window_seconds,
                          stop_signals=stop_signals, pass_fds=(fd,))
            status["steps"].append(step)
            guard_events = [json.loads(line) for line in
                            (output / f"guard-{backend}.jsonl").read_text().splitlines()]
            require(not any(e["event"] == "builder_blocked" for e in guard_events),
                    f"{backend} requested engine rebuilding; guard stopped it")
            require(step["returncode"] == 0, f"{backend} failed; preserved log contains the cause")
            require(json.loads((output / backend / "completed.json").read_text())["status"] == "completed",
                    "Missing backend completion receipt")
            if backend == "deepstream":
                effective = json.loads((output / backend / "effective-nvinfer.json").read_text())
                require(effective == {"input_tensor_meta": True, "batch_size": 1, "interval": 0,
                                      "preprocessing": "external_project_cuda"},
                        "nvinfer did not verify the required effective tensor-input settings")
        comparison = compare(output / "direct", output / "deepstream", len(args.images), args.atol)
        dump(output / "comparison.json", comparison)
        performance = {name: summarize(output / name) for name in ("direct", "deepstream")}
        performance["model_only_gpu_ms"] = None
        performance["model_only_gpu_ms_reason"] = "Unavailable via gst-nvinfer public callback; separate profiler required"
        performance["scope"] = "BGR host pixels to host raw tensor/callback and completed pipeline; no camera/ROS/JPEG"
        dump(output / "performance.json", performance)
        require(comparison["passed"], "Raw tensor comparison failed; no equivalent-performance conclusion")
        status["status"] = "completed"
        exit_code = 0
    except BaseException as error:
        status["status"] = "failed"
        status["error"] = f"{type(error).__name__}: {error}"
        if hasattr(error, "launch_receipt"):
            status["steps"].append(error.launch_receipt)
        print(status["error"], file=sys.stderr)
    finally:
        stop_signals.cleaning = True
        if fd is not None:
            os.close(fd)
        after = []
        for path in paths:
            try:
                after.append(identity(path))
            except Exception as error:
                after.append({"path": str(path), "error": f"{type(error).__name__}: {error}"})
        status["assets_unchanged"] = before == after
        dump(output / "assets-after.json", after)
        if not status["assets_unchanged"]:
            status["status"] = "failed"
            status["asset_error"] = "Original asset sha256/size/mtime changed"
            exit_code = 2
        if stop_signals.received is not None:
            status["status"] = "failed"
            status["stop_signal"] = signal.Signals(stop_signals.received).name
            exit_code = 2
        status["finish_ns"] = time.time_ns()
        dump(output / "status.json", status)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
