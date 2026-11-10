"""只读加载 Jetson 现行 codec 与手动 0x17F 发送脚本，离线核对本库真实 C 解码；不连接硬件或网络。"""

import argparse
import copy
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

MANUAL_ID = 0x17F
HEARTBEAT_ID = 0x180
VELOCITY_DLC = 7
# C 端 Jetson 模块的已知下行 ID；每帧（含畸形）推进 rx_sequence。
JETSON_DOWNLINK_IDS = {0x180, 0x181, 0x182, 0x190, 0x191, 0x192, 0x193}

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


def load_manual_sender(jetson_root):
    """读取 Jetson 端独立手动脚本，只调用其 USB-CAN 封包函数，不打开设备。"""
    source = jetson_root.resolve(strict=True) / "src" / "teleop_pkg" / "move_timed.py"
    spec = importlib.util.spec_from_file_location("jetson_manual_sender", source)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载手动发送脚本: {source}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def manual_wire(sender, values, valid):
    """把达妙 USB-CAN 串口包还原为 ID#HEX：ID 在 13..16 字节，DLC 在 18，载荷从 21 起。"""
    packet = sender.make_packet(values, valid)
    identifier = int.from_bytes(packet[13:17], "little")
    length = packet[18]
    if identifier != MANUAL_ID or length != VELOCITY_DLC:
        raise AssertionError(f"手动脚本封包 ID/DLC 不符: 0x{identifier:X}/{length}")
    return f"{identifier:03X}#{bytes(packet[21:21 + length]).hex().upper()}"


def initial_state():
    """声明接收快照可见字段，初始值均为零。"""
    fields = {
        "manual": "valid vx vy wz received_ms sequence invalid_sequence zero_sequence",
        "heartbeat": "seen received_ms mission_loss_sequence",
        "target": "valid x y received_ms source_frame",
        "safety": "valid x y received_ms source_frame",
        "arrival_ack": "valid accepted received_ms",
        "roi_reply": "valid passed destination received_ms",
        "command_reply": "valid accepted command token received_ms",
        "turn_to_safety": "valid destination token received_ms",
    }
    state = {kind: dict.fromkeys(names.split(), 0) for kind, names in fields.items()}
    state["rx_sequence"] = 0
    return state


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
        if kind != "velocity":
            fields["source_frame"] = value["source_frame"] if value["valid"] else 0
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


def scenarios(codec, sender):
    """真实 encode 生成正常帧，再注入错误以核对撤效与独立快照。"""
    cases = []
    state = initial_state()

    def append(wire, kind=None, fields=None, label=""):
        received_ms = len(cases) + 1
        identifier = int(wire.split("#")[0], 16)
        if identifier in JETSON_DOWNLINK_IDS:
            state["rx_sequence"] += 1
        if kind == "heartbeat":
            heartbeat = state["heartbeat"]
            if fields["well_formed"]:
                heartbeat.update(seen=1, received_ms=received_ms)
            elif fields["empty"]:
                heartbeat.update(seen=0, mission_loss_sequence=heartbeat["mission_loss_sequence"] + 1)
        elif kind is not None:
            previous = state[kind]
            current = dict.fromkeys(previous, 0)
            current.update(fields or {})
            current["received_ms"] = received_ms
            if kind == "manual":
                current["sequence"] = previous["sequence"] + 1
                current["invalid_sequence"] = previous["invalid_sequence"] + (not current["valid"])
                zero = current["valid"] and not (current["vx"] or current["vy"] or current["wz"])
                current["zero_sequence"] = previous["zero_sequence"] + bool(zero)
            state[kind] = current
        cases.append((wire, copy.deepcopy(state), label or wire))

    def heartbeat(wire, label=""):
        """0x180 只作心跳：Jetson codec 能解出即合法格式（含 valid=0），空载荷为失联标志。"""
        payload = wire.split("#")[1]
        try:
            codec.decode(wire)
            well_formed = True
        except ValueError:
            well_formed = False
        append(wire, "heartbeat", {"well_formed": well_formed, "empty": not payload}, label)

    def manual(wire, label=""):
        """0x17F 与 0x180 同格式：以 Jetson 的 0x180 解码语义为期望，坏帧整帧撤销。"""
        payload = wire.split("#")[1]
        try:
            _, fields = decoded_fields(codec, f"{HEARTBEAT_ID:03X}#{payload}")
        except ValueError:
            fields = {"valid": 0}
        append(wire, "manual", fields, label)

    def encoded(kind, values=(), invalid=False):
        wire = codec.encode(kind, [str(value) for value in values], invalid=invalid)
        if kind == "velocity":
            heartbeat(wire)
            return wire
        decoded_kind, fields = decoded_fields(codec, wire)
        append(wire, decoded_kind, fields)
        return wire

    # 0x180 心跳：bridge 现行为 valid=0 全零（DLC 7）及空载荷失联；有效帧格式仍算合法心跳。
    for values in ((0, 0, 0), (-0.125, 0.25, -0.8)):
        encoded("velocity", values)
    encoded("velocity", (0, 0, 0), invalid=True)
    heartbeat(f"{HEARTBEAT_ID:03X}#", label="空 0x180 失联标志")
    encoded("velocity", (0, 0, 0), invalid=True)
    for malformed in ("180#000000000000", "180#02000000000000", "180#0000000000000000"):
        heartbeat(malformed, label=f"畸形 0x180 不保活: {malformed}")

    # 0x17F：由 Jetson 端 move_timed.py 的真实封包函数生成，只进入手动快照。
    for values in ((0.0, 0.0, 0.0), (0.08, 0.0, 0.0), (-0.125, 0.25, -0.8), (-32.768, 32.767, -0.001)):
        manual(manual_wire(sender, values, True))
    manual(manual_wire(sender, (0.0, 0.0, 0.0), False), label="手动 valid=0 停车撤权")
    manual(manual_wire(sender, (0.0, 0.0, 0.0), True))
    seed = manual_wire(sender, (-0.125, 0.25, -0.8), True)
    identifier, payload = seed.split("#")
    for malformed in (f"{identifier}#", f"{identifier}#{payload[:-2]}",
                      f"{identifier}#02{payload[2:]}", f"{identifier}#{payload}00"):
        manual(seed)
        manual(malformed, label=f"错误帧撤销手动速度: {malformed}")
    encoded("velocity", (0, 0, 0), invalid=True)
    manual(seed, label="0x17F 不刷新心跳、不推进 Jetson 接收序号")

    for kind in ("target", "safety"):
        for values in ((0.5, 0.75), (-0.25, 1.2), (-3.2768, 3.2767)):
            encoded(kind, values)
        encoded(kind, (0.5, 0.75, 0xABCDEF))
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
        "target": (-0.25, 1.2),
        "safety": (0.5, 0.75), "arrival_ack": (), "roi_reply": ("right",),
        "command_reply": ("delivery_completed", 0xFFFFFFFF),
        "turn_to_safety": ("left", 0),
    }
    for kind, values in seeds.items():
        seed = encoded(kind, values)
        identifier, payload = seed.split("#")
        # 每条错误前先恢复有效状态，证明旧内容确实被撤销。
        malformed_frames = [f"{identifier}#{payload[:-2]}", f"{identifier}#02{payload[2:]}"]
        # 标准 CAN 的 DLC 最大为 8，满长帧不能再构造 9 字节的合法报文。
        if len(payload) < 16:
            malformed_frames.append(f"{identifier}#{payload}00")
        for malformed in malformed_frames:
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
    cases = scenarios(codec, load_manual_sender(args.jetson_root))
    with tempfile.TemporaryDirectory(prefix="cboard_jetson_codec_", dir=args.temp_dir) as directory:
        executable = Path(directory) / "jetson_codec_probe.exe"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O0",
                   "-I" + str(root / "tests/native/stubs"),
                   "-I" + str(root / "project/Core/Communication/Jetson"),
                   "-I" + str(root / "project/Core/Communication/Manual"),
                   "-I" + str(root / "project/Core/Control/Jetson"),
                   "-I" + str(root / "project/Core/Algorithm/Chassis"),
                   str(root / "tests/native/jetson_codec_probe.c"),
                   str(root / "project/Core/Communication/Jetson/jetson_can.c"),
                   str(root / "project/Core/Communication/Manual/manual_can.c"),
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
    print(f"PASS: Jetson 现行 codec/手动脚本与真实 C 解码互验，7 种下行 + 0x17F、{len(cases)} 帧；无硬件操作")


if __name__ == "__main__":
    main()
