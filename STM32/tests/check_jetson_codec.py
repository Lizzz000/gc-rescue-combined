"""只读加载 Jetson 现行 codec，离线核对本库真实 C 解码；不连接硬件或网络。"""

import argparse
import copy
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

# Jetson 是只读业务基准，动态加载其模块也不能写入 __pycache__。
sys.dont_write_bytecode = True


def load_codec(jetson_root):
    """读取指定 Jetson 的真实 codec；缺失或导入失败时直接终止。"""
    tools = jetson_root.resolve(strict=True) / "tools"
    source = tools / "source" / "can_codec.py"
    if not source.is_file():
        legacy_source = tools / "can_codec.py"
        if not legacy_source.is_file():
            raise FileNotFoundError(f"未找到 Jetson codec: {source} 或 {legacy_source}")
        source = legacy_source
    spec = importlib.util.spec_from_file_location("jetson_reference_codec", source)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 Jetson codec: {source}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def initial_state():
    """声明接收快照可见字段，初始值均为零。"""
    fields = {
        "velocity": "valid vx vy wz received_ms sequence invalid_sequence",
        "target": "valid x y received_ms",
        "safety": "valid x y received_ms",
        "arrival_ack": "valid accepted received_ms",
        "roi_reply": "valid passed destination received_ms",
        "command_reply": "valid accepted command token received_ms",
        "turn_to_safety": "valid destination token received_ms",
    }
    return {kind: dict.fromkeys(names.split(), 0) for kind, names in fields.items()}


def decoded_fields(codec, wire):
    """以 Jetson 的解码语义为期望值，只映射 C 快照字段及定点单位。"""
    value = codec.decode(wire)
    kind = value["kind"]
    if kind in ("velocity", "target", "safety"):
        names = ("vx", "vy", "wz") if kind == "velocity" else ("x", "y")
        source_names = ("vx", "vy", "yaw_rate") if kind == "velocity" else names
        scale = 1000 if kind == "velocity" else 10000
        fields = {"valid": int(value["valid"])}
        fields.update({name: round(value[source] * scale) if value["valid"] else 0
                       for name, source in zip(names, source_names)})
        return kind, fields
    if kind == "arrival_ack":
        return kind, {"valid": 1, "accepted": int(value["accepted"])}
    if kind == "command_reply":
        return kind, {"valid": 1, "accepted": int(value["accepted"]),
                      "command": codec.COMMANDS[value["command"]], "token": value["token"]}
    destination = {"none": 0, "left": 1, "right": 2}[value["destination"]]
    if kind == "roi_reply":
        return kind, {"valid": int(value["valid"]), "passed": int(value["passed"]),
                      "destination": destination}
    if kind == "turn_to_safety":
        return kind, {"valid": 1, "destination": destination, "token": value["token"]}
    raise AssertionError(f"本工具只互验七种下行报文: {kind}")


def scenarios(codec):
    """真实 encode 生成正常帧，再注入错误以核对撤效与独立快照。"""
    cases = []
    state = initial_state()

    def append(wire, kind=None, fields=None, label=""):
        if kind is not None:
            previous = state[kind]
            current = dict.fromkeys(previous, 0)
            current.update(fields or {})
            current["received_ms"] = len(cases) + 1
            if kind == "velocity":
                current["sequence"] = previous["sequence"] + 1
                current["invalid_sequence"] = previous["invalid_sequence"] + (not current["valid"])
            state[kind] = current
        cases.append((wire, copy.deepcopy(state), label or wire))

    def encoded(kind, values=(), invalid=False):
        wire = codec.encode(kind, [str(value) for value in values], invalid=invalid)
        decoded_kind, fields = decoded_fields(codec, wire)
        append(wire, decoded_kind, fields)
        return wire

    for values in ((0, 0, 0), (-0.125, 0.25, -0.8), (-32.768, 32.767, -0.001)):
        encoded("velocity", values)
    encoded("velocity", (1, -1, 1), invalid=True)
    encoded("velocity", (100, 0, 0))  # Jetson 自行把定点溢出编码为无效整帧。
    encoded("velocity", (0, 0, 0))  # 恢复有效不清除本批撤权序号。
    for kind in ("target", "safety"):
        for values in ((0.5, 0.75), (-0.25, 1.2), (-3.2768, 3.2767)):
            encoded(kind, values)
        encoded(kind, (0.5, 0.75), invalid=True)
    encoded("arrival_ack")
    encoded("arrival_ack", invalid=True)
    for destination in ("left", "right", "rejected"):
        encoded("roi_reply", (destination,))
    encoded("roi_reply", ("left",), invalid=True)
    for command in codec.COMMANDS:
        tokens = (0,) if command == "start_search" else (0, 1, 0x89ABCDEF, 0xFFFFFFFF)
        for token in tokens:
            encoded("command_reply", (command, token))
            encoded("command_reply", (command, token), invalid=True)
    for destination in ("left", "right"):
        for token in (0, 0x89ABCDEF, 0xFFFFFFFF):
            encoded("turn_to_safety", (destination, token))

    seeds = {
        "velocity": (-0.125, 0.25, -0.8), "target": (-0.25, 1.2),
        "safety": (0.5, 0.75), "arrival_ack": (), "roi_reply": ("right",),
        "command_reply": ("delivery_completed", 0xFFFFFFFF),
        "turn_to_safety": ("left", 0),
    }
    for kind, values in seeds.items():
        seed = encoded(kind, values)
        identifier, payload = seed.split("#")
        # 每条错误前先恢复有效状态，证明旧内容确实被撤销。
        for malformed in (f"{identifier}#{payload[:-2]}",
                          f"{identifier}#{payload}00",
                          f"{identifier}#02{payload[2:]}"):
            encoded(kind, values)
            try:
                codec.decode(malformed)
            except ValueError:
                pass
            else:
                raise AssertionError(f"Jetson 未拒绝预期错误帧: {malformed}")
            append(malformed, kind, label=f"错误帧撤销 {kind}: {malformed}")

    for kind, malformed in (("roi_reply", "191#010100"),
                            ("command_reply", "192#010000000000"),
                            ("command_reply", "192#010301000000"),
                            ("turn_to_safety", "193#010000000000"),
                            ("turn_to_safety", "193#000100000000")):
        encoded(kind, seeds[kind])
        try:
            codec.decode(malformed)
        except ValueError:
            pass
        else:
            raise AssertionError(f"Jetson 未拒绝预期矛盾字段: {malformed}")
        append(malformed, kind, label=f"矛盾字段撤销 {kind}: {malformed}")
    append("555#0102030405060708", label="未知 ID 不改变任何快照")
    append("201#01", label="废弃运输 ID 不改变任何快照")
    return cases


def main():
    """编译临时 C 探针，逐帧比较完整快照，结束时自动清理编译产物。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jetson-root", type=Path,
                        default=Path("C:/Users/Lenovo/OneDrive/Desktop/Jetson"))
    parser.add_argument("--cc", help="主机 GCC 路径，不能使用 arm-none-eabi-gcc")
    parser.add_argument("--temp-dir", type=Path, help="临时构建目录的父目录")
    args = parser.parse_args()
    fallback = Path("C:/Program Files/JetBrains/CLion 2026.2.3.1/bin/mingw/bin/gcc.exe")
    compiler = args.cc or shutil.which("gcc") or (str(fallback) if fallback.exists() else None)
    if compiler is None:
        parser.error("未找到主机 GCC，请使用 --cc 指定")
    root = Path(__file__).resolve().parents[1]
    codec = load_codec(args.jetson_root)
    cases = scenarios(codec)
    with tempfile.TemporaryDirectory(prefix="cboard_jetson_codec_", dir=args.temp_dir) as directory:
        executable = Path(directory) / "jetson_codec_probe.exe"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O0",
                   "-I" + str(root / "tests/native/stubs"),
                   "-I" + str(root / "project/Core/Communication/Jetson"),
                   str(root / "tests/native/jetson_codec_probe.c"),
                   str(root / "project/Core/Communication/Jetson/jetson_can.c"),
                   "-o", str(executable)]
        subprocess.run(command, check=True, timeout=30)
        result = subprocess.run([str(executable)], input="\n".join(case[0] for case in cases) + "\n",
                                text=True, capture_output=True, check=True, timeout=10)
        lines = result.stdout.splitlines()
        if len(lines) != len(cases):
            raise AssertionError(f"快照数量不符: {len(lines)} != {len(cases)}")
        for index, (line, (_, expected, label)) in enumerate(zip(lines, cases), 1):
            actual = json.loads(line)
            if actual != expected:
                raise AssertionError(f"第 {index} 帧互验失败 ({label})\n"
                                     f"期望: {expected}\n实际: {actual}")
    print(f"PASS: Jetson 现行 codec 与真实 C 解码互验，7 种下行、{len(cases)} 帧；无硬件操作")


if __name__ == "__main__":
    main()
