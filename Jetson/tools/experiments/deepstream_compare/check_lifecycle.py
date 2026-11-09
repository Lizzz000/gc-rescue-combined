#!/usr/bin/env python3
"""板端纯 CPU 合同：真实 run.py 的父信号、Popen 窗口、超时及进程组回收。"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

sys.dont_write_bytecode = True


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def write_json(path, data):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def load_runtime(path):
    spec = importlib.util.spec_from_file_location("actual_deepstream_run", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def group_exists(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def start_ticks(pid):
    try:
        # comm 可以有空格或括号，从最后一个右括号之后数第 22 个 stat 字段。
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
    except FileNotFoundError:
        return None


def worker(runtime_path, case, output):
    runtime = load_runtime(runtime_path)
    stops = runtime.StopSignals()
    original_popen = runtime.subprocess.Popen
    original_cleanup = runtime.stop_process_group

    def observed_cleanup(process):
        write_json(output / "cleanup-started.json", {"pid": process.pid})
        return original_cleanup(process)

    def observed_popen(*args, **kwargs):
        process = original_popen(*args, **kwargs)
        write_json(output / "child-created.json",
                   {"pid": process.pid, "start_ticks": start_ticks(process.pid)})
        if case == "spawn_window":
            blocked = signal.pthread_sigmask(signal.SIG_BLOCK, [])
            require(all(signum in blocked for signum in runtime.STOP_SIGNALS),
                    "Popen handoff window is not protected by the real launch()")
            # 注入在真实 Popen 已返回、launch 尚未拿到句柄的确定位置。
            # 此时信号必须保持 pending；恢复 mask 后才应进入真实异常清理。
            os.kill(os.getpid(), signal.SIGTERM)
            require(signal.SIGTERM in signal.sigpending(), "Injected TERM was not deferred")
            write_json(output / "spawn-window.json",
                       {"parent_term_pending": True, "all_stop_signals_blocked": True})
        return process

    runtime.subprocess.Popen = observed_popen
    runtime.stop_process_group = observed_cleanup
    ready = str(output / "child-ready.json")
    # 无设备、模型或插件加载。ready 在安装明确的子信号策略之后产生。
    child_source = (
        "import os,signal,time,json,sys\n"
        f"case={case!r}\n"
        "if case=='normal_exit': sys.exit(42)\n"
        "if case=='parent_term_repeat': signal.signal(signal.SIGTERM,signal.SIG_IGN)\n"
        f"with open({ready!r},'x') as f: json.dump({{'pid':os.getpid()}},f)\n"
        "while True: signal.pause()\n"
    )
    result = {"case": case}
    try:
        result["launch"] = runtime.launch(
            [sys.executable, "-c", child_source], os.environ.copy(),
            output / "child.log", timeout=0 if case == "timeout" else 30,
            stop_signals=stops)
        result["kind"] = "returned"
    except BaseException as error:
        result.update(kind="raised", exception=type(error).__name__, message=str(error),
                      launch=getattr(error, "launch_receipt", None))
    finally:
        stops.cleaning = True
        result["received_signal"] = (
            signal.Signals(stops.received).name if stops.received is not None else None)
        write_json(output / "worker-result.json", result)
    return 0


def wait_for_file(path, process, timeout=20):
    deadline = time.monotonic() + timeout
    while not path.is_file():
        require(process.poll() is None, f"Worker exited before {path.name}")
        require(time.monotonic() < deadline, f"Timed out waiting for {path.name}")
        time.sleep(.01)


def emergency_cleanup(process, directory):
    """仅合同自身失败时回收本测试创建的 CPU fixture；不触及外部进程。"""
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
        try:
            process.wait(timeout=17)
        except subprocess.TimeoutExpired:
            pass
    # 即使 worker 因待测函数的回归提前死亡，也回收有本合同启动时间证据的 fixture。
    created = directory / "child-created.json"
    if created.is_file():
        child = json.loads(created.read_text())
        if child["start_ticks"] is not None and start_ticks(child["pid"]) == child["start_ticks"]:
            try:
                os.killpg(child["pid"], signal.SIGKILL)
            except ProcessLookupError:
                pass
    if process.poll() is None:
        process.kill()
        process.wait(timeout=5)


def check_case(script, runtime_path, case, directory):
    directory.mkdir()
    with (directory / "worker.log").open("xb") as log:
        process = subprocess.Popen(
            [sys.executable, str(script), "--worker", str(runtime_path), case, str(directory)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            if case == "parent_term_repeat":
                wait_for_file(directory / "child-ready.json", process)
                process.send_signal(signal.SIGTERM)
                wait_for_file(directory / "cleanup-started.json", process)
                # 子进程明确忽略 TERM，真实回收器还在有界等待；重复信号不得跳过 KILL/reap。
                for signum in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP, signal.SIGTERM):
                    process.send_signal(signum)
            require(process.wait(timeout=25) == 0, f"{case}: worker failed")
            result = json.loads((directory / "worker-result.json").read_text())
            created = json.loads((directory / "child-created.json").read_text())
            require(not group_exists(created["pid"]), f"{case}: child process group remains")
            receipt = result["launch"]
            require(receipt and receipt["pid"] == created["pid"], f"{case}: missing child identity")
            if case == "normal_exit":
                require(result["kind"] == "returned" and receipt["returncode"] == 42,
                        "Normal child exit status was lost")
                require(result["received_signal"] is None, "Unexpected normal-run signal")
            else:
                expected = "TimeoutExpired" if case == "timeout" else "StopRequested"
                require(result["kind"] == "raised" and result["exception"] == expected,
                        f"{case}: wrong exception path")
                cleanup = receipt["cleanup"]
                require(cleanup["leader_reaped"] and cleanup["group_empty"],
                        f"{case}: cleanup did not reap/empty its group")
                require(cleanup["term_sent"], f"{case}: TERM was not sent")
                if case == "parent_term_repeat":
                    require(cleanup["kill_sent"] and cleanup["returncode"] == -signal.SIGKILL,
                            "TERM-ignoring child did not exercise KILL escalation")
                else:
                    require(not cleanup["kill_sent"] and cleanup["returncode"] == -signal.SIGTERM,
                            f"{case}: expected graceful group termination")
                if case == "spawn_window":
                    require(json.loads((directory / "spawn-window.json").read_text()) ==
                            {"parent_term_pending": True, "all_stop_signals_blocked": True},
                            "Missing deterministic spawn-window injection")
                require(result["received_signal"] == (None if case == "timeout" else "SIGTERM"),
                        f"{case}: first stop signal was not preserved")
            return result
        finally:
            emergency_cleanup(process, directory)


def main():
    script = Path(__file__).resolve()
    if len(sys.argv) == 5 and sys.argv[1] == "--worker":
        return worker(Path(sys.argv[2]), sys.argv[3], Path(sys.argv[4]))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="New directory under the system temporary directory")
    args = parser.parse_args()
    temp_root = Path(tempfile.gettempdir()).resolve()
    if args.output is None:
        output = Path(tempfile.mkdtemp(prefix="deepstream-lifecycle-", dir=temp_root))
    else:
        output = args.output.resolve()
        require(output != temp_root and output.is_relative_to(temp_root), "Output must be an isolated tmp directory")
        output.mkdir(parents=False, exist_ok=False)
    runtime_path = script.with_name("run.py")
    report = {"status": "running", "gpu_used": False, "camera_used": False,
              "run_py_sha256": hashlib.sha256(runtime_path.read_bytes()).hexdigest(),
              "check_sha256": hashlib.sha256(script.read_bytes()).hexdigest(), "cases": []}
    controller_stops = load_runtime(runtime_path).StopSignals()
    print(output, flush=True)
    try:
        for case in ("normal_exit", "parent_term_repeat", "spawn_window", "timeout"):
            report["cases"].append(check_case(script, runtime_path, case, output / case))
        report["status"] = "passed"
        return 0
    except BaseException as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}")
        print(report["error"], file=sys.stderr)
        return 1
    finally:
        controller_stops.cleaning = True
        write_json(output / "result.json", report)


if __name__ == "__main__":
    raise SystemExit(main())
