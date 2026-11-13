"""离线核对本库 C 解码与上行编码是否符合类型号帧，并与 Jetson codec 对照。不连接硬件。"""

import argparse
import copy
import importlib.util
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile

# Jetson→STM32：1 心跳，2 手动，3 目标，4 安全区，5 到达回执，6 ROI，7 命令回执，8 转向请求。
# STM32→Jetson：16 到达，17 转向完成，18 安全区到达，19 开始搜索，20 投送，21 到点。
HEARTBEAT = 1
MANUAL = 2
TARGET = 3
SAFETY = 4
ARRIVAL_ACK = 5
ROI = 6
COMMAND_REPLY = 7
TURN = 8
ARRIVAL = 16
TURN_COMPLETED = 17
SAFETY_ARRIVED = 18
START_SEARCH = 19
DELIVERY = 20
POINT = 21
DOWNLINK = {HEARTBEAT, TARGET, SAFETY, ARRIVAL_ACK, ROI, COMMAND_REPLY, TURN}
COMMANDS = {
    "turn_completed": 1,
    "safety_arrived": 2,
    "start_search": 3,
    "delivery_completed": 4,
    "arrival_point": 5,
}

sys.dont_write_bytecode = True


def wire(type_number, payload):
    """类型号按十六进制书写，与 C 探针的 ID#HEX 输入一致。"""
    return f"{type_number:03X}#{payload.hex().upper()}"


def load_codec(jetson_root):
    """读取指定 Jetson 的 codec。缺失时直接终止。"""
    tools = jetson_root.resolve(strict=True) / "tools"
    source = tools / "source" / "usb_codec.py"
    if not source.is_file():
        raise FileNotFoundError(f"未找到 Jetson USB codec: {source}")
    spec = importlib.util.spec_from_file_location("jetson_reference_codec", source)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 Jetson codec: {source}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_manual_sender(jetson_root):
    """读取 Jetson 手动封包函数，不打开设备。"""
    source = jetson_root.resolve(strict=True) / "src" / "teleop_pkg" / "manual" / "link.py"
    sys.path.insert(0, str(source.parents[1]))
    sys.path.insert(0, str(source.parent))
    spec = importlib.util.spec_from_file_location("jetson_manual_link", source)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载手动链路模块: {source}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def initial_state():
    """声明接收快照可见字段，初始值均为零。"""
    fields = {
        "manual": "valid vx vy wz received_ms sequence invalid_sequence zero_sequence",
        "heartbeat": "seen received_ms sequence pause mission_loss_sequence",
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


def scenarios():
    """按 STM32 规则生成类型号帧和期望快照。"""
    cases = []
    state = initial_state()

    def append(frame, kind=None, fields=None, label=""):
        received_ms = len(cases) + 1
        type_number = int(frame.split("#")[0], 16)
        if type_number in DOWNLINK:
            state["rx_sequence"] += 1
        if kind == "heartbeat":
            heartbeat = state["heartbeat"]
            if fields["online"]:
                heartbeat.update(seen=1, received_ms=received_ms, sequence=state["rx_sequence"],
                                 pause=fields["pause"])
            elif fields["lost"]:
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
        cases.append((frame, copy.deepcopy(state), label or frame))

    def heartbeat(alive, pause, vx=0, vy=0, wz=0, payload=None, label=""):
        if payload is None:
            payload = struct.pack("<BBhhh", alive, pause, vx, vy, wz)
        online = len(payload) == 8 and payload[0] == 1 and payload[1] in (0, 1)
        lost = False
        if len(payload) == 8 and payload[0] == 0 and payload[1] in (0, 1):
            speeds = struct.unpack_from("<hhh", payload, 2)
            lost = speeds == (0, 0, 0)
        append(wire(HEARTBEAT, payload), "heartbeat",
               {"online": online, "lost": lost and not online, "pause": payload[1] if online else 0},
               label)

    def manual(valid, vx=0, vy=0, wz=0, payload=None, label=""):
        if payload is None:
            payload = struct.pack("<Bhhh", int(valid), vx, vy, wz)
        fields = {"valid": 0, "vx": 0, "vy": 0, "wz": 0}
        if len(payload) == 7 and payload[0] in (0, 1):
            vx_value, vy_value, wz_value = struct.unpack_from("<hhh", payload, 1)
            if payload[0] == 1:
                fields = {"valid": 1, "vx": vx_value, "vy": vy_value, "wz": wz_value}
        append(wire(MANUAL, payload), "manual", fields, label)

    def center(type_number, x, y, source=0, valid=1):
        payload = struct.pack("<Bhh", valid, x if valid else 0, y if valid else 0)
        payload += (source if valid else 0).to_bytes(3, "little")
        kind = "target" if type_number == TARGET else "safety"
        fields = {"valid": int(valid), "x": x if valid else 0, "y": y if valid else 0,
                  "source_frame": source if valid else 0}
        append(wire(type_number, payload), kind, fields)

    heartbeat(1, 0, label="在线 pause=0")
    heartbeat(1, 0, -125, 250, -800, label="在线非零速度仍是心跳")
    heartbeat(1, 1, label="在线 pause=1")
    heartbeat(1, 0)
    heartbeat(1, 2, label="pause 越界不保活")
    heartbeat(0, 0, label="alive=0 且速度为 0 是失联")
    heartbeat(1, 0, label="失联后的心跳恢复保活")
    heartbeat(0, 0, 1, 0, 0, label="alive=0 但速度非 0 不是失联")
    heartbeat(2, 0, label="alive 越界不是心跳")
    heartbeat(1, 0, payload=b"", label="空载荷不是失联")
    heartbeat(1, 0, payload=bytes(7), label="长度 7 不是心跳")

    for milli in ((0, 0, 0), (80, 0, 0), (-125, 250, -800), (-32768, 32767, -1)):
        manual(1, *milli)
    manual(0, 1, 2, 3, label="手动 valid=0 撤权")
    manual(1, 0, 0, 0)
    manual(1, payload=b"", label="空手动帧撤权")
    manual(1, payload=bytes(8), label="长度 8 的手动帧撤权")
    manual(2, 0, 0, 0, label="valid 越界撤权")
    heartbeat(1, 0, label="类型 1 不改手动快照")
    manual(1, -125, 250, -800, label="类型 2 不刷新心跳")

    for type_number, samples in (
        (TARGET, ((5000, 7500, 0), (-2500, 12000, 0xABCDEF), (-32768, 32767, 0))),
        (SAFETY, ((5000, 7500, 1), (-2500, 12000, 0), (32767, -32768, 0xFFFFFF))),
    ):
        for x, y, source in samples:
            center(type_number, x, y, source, 1)
        center(type_number, 1, 2, 3, 0)
        append(wire(type_number, bytes(5)), "target" if type_number == TARGET else "safety",
               label=f"错误长度撤销类型 {type_number}")

    append(wire(ARRIVAL_ACK, b"\x01"), "arrival_ack", {"valid": 1, "accepted": 1})
    append(wire(ARRIVAL_ACK, b"\x00"), "arrival_ack", {"valid": 1, "accepted": 0})
    append(wire(ARRIVAL_ACK, b"\x02"), "arrival_ack", label="到达回执越界")

    for payload, fields in (
        (b"\x01\x00\x00", {"valid": 1, "passed": 0, "destination": 0}),
        (b"\x01\x01\x01", {"valid": 1, "passed": 1, "destination": 1}),
        (b"\x01\x01\x02", {"valid": 1, "passed": 1, "destination": 2}),
        (b"\x00\x00\x00", {}),
        (b"\x01\x01\x00", {}),
        (b"\x01\x00", {}),
    ):
        append(wire(ROI, payload), "roi_reply", fields)

    for command, name in COMMANDS.items():
        tokens = (0,) if name in (3, 5) else (0, 1, 0x89ABCDEF, 0xFFFFFFFF)
        for token in tokens:
            payload = struct.pack("<BBI", 1, name, token)
            append(wire(COMMAND_REPLY, payload), "command_reply",
                   {"valid": 1, "accepted": 1, "command": name, "token": token})
            rejected = struct.pack("<BBI", 0, name, token)
            append(wire(COMMAND_REPLY, rejected), "command_reply",
                   {"valid": 1, "accepted": 0, "command": name, "token": token})
    append(wire(COMMAND_REPLY, struct.pack("<BBI", 1, 3, 1)), "command_reply", label="命令 3 token 非 0")
    append(wire(COMMAND_REPLY, struct.pack("<BBI", 1, 5, 1)), "command_reply", label="命令 5 token 非 0")
    append(wire(COMMAND_REPLY, struct.pack("<BBI", 1, 6, 0)), "command_reply", label="命令号越界")

    for destination in (1, 2):
        for token in (0, 0x89ABCDEF, 0xFFFFFFFF):
            payload = struct.pack("<BBI", 1, destination, token)
            append(wire(TURN, payload), "turn_to_safety",
                   {"valid": 1, "destination": destination, "token": token})
    append(wire(TURN, struct.pack("<BBI", 0, 1, 0)), "turn_to_safety", label="转向 data0 不是 1")
    append(wire(TURN, struct.pack("<BBI", 1, 0, 0)), "turn_to_safety", label="转向目的区非法")
    append(wire(0x55, b"\x01\x02\x03\x04"), label="未知类型不改变快照")
    return cases


def uplink_cases():
    """上行期望与 Jetson_CAN_Encode_Uplink 一致。"""
    def pack(type_number, argument):
        if type_number in (ARRIVAL, START_SEARCH):
            return bytes([1])
        if type_number == POINT:
            if argument not in (1, 2):
                return b""
            return bytes([argument])
        if type_number in (TURN_COMPLETED, SAFETY_ARRIVED, DELIVERY):
            return struct.pack("<BI", 1, argument)
        return b""

    requests = [(f"^{ARRIVAL:03X}:0", pack(ARRIVAL, 0)), (f"^{START_SEARCH:03X}:0", pack(START_SEARCH, 0))]
    for point in (1, 2):
        requests.append((f"^{POINT:03X}:{point}", pack(POINT, point)))
    for type_number in (TURN_COMPLETED, SAFETY_ARRIVED, DELIVERY):
        for token in (0, 1, 0x89ABCDEF, 0xFFFFFFFF):
            requests.append((f"^{type_number:03X}:{token}", pack(type_number, token)))
    for point in (0, 3):
        requests.append((f"^{POINT:03X}:{point}", b""))
    cases = []
    for request, payload in requests:
        type_number = int(request[1:].split(":")[0], 16)
        expected = "" if not payload else wire(type_number, payload)
        cases.append((request, expected))
    return cases


def stm32_probe(compiler, root, cases, uplinks):
    """编译 C 探针，逐帧比较快照和上行编码。"""
    with tempfile.TemporaryDirectory(prefix="cboard_jetson_codec_") as directory:
        executable = Path(directory) / "jetson_codec_probe.exe"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O0",
                   "-I" + str(root / "tests/native/stubs"),
                   "-I" + str(root / "project/Core/Communication/Jetson"),
                   "-I" + str(root / "project/Core/Communication/Manual"),
                   "-I" + str(root / "project/Core/Control/Jetson"),
                   "-I" + str(root / "project/Core/Algorithm/Chassis"),
                   str(root / "tests/native/jetson_codec_probe.c"),
                   str(root / "project/Core/Communication/Jetson/jetson_link.c"),
                   str(root / "project/Core/Communication/Manual/manual_link.c"),
                   "-o", str(executable)]
        subprocess.run(command, check=True, timeout=30)
        frames = [case[0] for case in cases] + [case[0] for case in uplinks]
        result = subprocess.run([str(executable)], input="\n".join(frames) + "\n",
                                text=True, capture_output=True, check=True, timeout=10)
        lines = result.stdout.splitlines()
        if len(lines) != len(frames):
            raise AssertionError(f"输出数量不符: {len(lines)} != {len(frames)}\n{result.stderr}")
        for line, (request, expected) in zip(lines[len(cases):], uplinks):
            actual = json.loads(line)["uplink"]
            if actual != expected:
                raise AssertionError(f"上行 {request} 编码不符\n期望: {expected!r}\n实际: {actual!r}")
        for index, (line, (_, expected, label)) in enumerate(zip(lines, cases), 1):
            actual = json.loads(line)
            if actual != expected:
                raise AssertionError(f"第 {index} 帧不符 ({label})\n期望: {expected}\n实际: {actual}")


def from_jetson_text(text):
    """Jetson 文本是十进制 T01#HEX。探针使用十六进制类型号。"""
    type_text, payload = text.split("#", 1)
    if not type_text.startswith("T") or not type_text[1:].isdigit():
        raise AssertionError(f"Jetson 文本不是类型号: {text}")
    return wire(int(type_text[1:]), bytes.fromhex(payload))


def jetson_cross_check(codec, sender):
    """对照 Jetson 现行编码。类型号或载荷不一致则互验失败。"""
    target = struct.pack("<Bhh", 1, 5000, 7500) + (0xABCDEF).to_bytes(3, "little")
    samples = [
        ("heartbeat", ["0", "0", "0", "1"], False,
         wire(HEARTBEAT, struct.pack("<BBhhh", 1, 1, 0, 0, 0))),
        ("mission_loss", [], False, wire(HEARTBEAT, bytes(8))),
        ("heartbeat", ["-0.125", "0.25", "-0.8", "0"], False,
         wire(HEARTBEAT, struct.pack("<BBhhh", 1, 0, -125, 250, -800))),
        ("manual", ["0.08", "0", "0"], False, wire(MANUAL, struct.pack("<Bhhh", 1, 80, 0, 0))),
        ("target", ["0.5", "0.75", str(0xABCDEF)], False, wire(TARGET, target)),
        ("arrival_ack", [], True, wire(ARRIVAL_ACK, b"\x00")),
        ("roi_reply", ["left"], False, wire(ROI, b"\x01\x01\x01")),
        ("command_reply", ["arrival_point", "0"], False,
         wire(COMMAND_REPLY, struct.pack("<BBI", 1, 5, 0))),
        ("turn_to_safety", ["right", "0"], False, wire(TURN, struct.pack("<BBI", 1, 2, 0))),
        ("arrival_point", ["2"], False, wire(POINT, b"\x02")),
        ("turn_completed", [str(0x89ABCDEF)], False,
         wire(TURN_COMPLETED, struct.pack("<BI", 1, 0x89ABCDEF))),
        ("start_search", [], False, wire(START_SEARCH, b"\x01")),
    ]
    for kind, values, invalid, expected in samples:
        try:
            actual = from_jetson_text(codec.encode(kind, values, invalid))
        except Exception as error:
            raise SystemExit(f"FAIL: Jetson 编码 {kind} 失败，互验未通过：{error}")
        if actual != expected:
            raise SystemExit(
                f"FAIL: Jetson {kind} 与 STM32 不一致，互验未通过。\n"
                f"期望 {expected}\n实际 {actual}"
            )
    packet = sender.manual_packet(1, 0, 0, 0)
    payload = struct.pack("<Bhhh", 1, 0, 0, 0)
    if packet[:4] != bytes((0xA5, 0x01, MANUAL, 7)) or packet[4:11] != payload:
        raise SystemExit(f"FAIL: Jetson 手动封包不是类型 2 长度 7，互验未通过：{packet.hex()}")


def main():
    """先核对 STM32 探针，再对照 Jetson codec。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jetson-root", type=Path,
                        default=Path(__file__).resolve().parents[2] / "Jetson")
    parser.add_argument("--cc", help="主机 GCC 路径，不能使用 arm-none-eabi-gcc")
    parser.add_argument("--temp-dir", type=Path, help="临时构建目录的父目录")
    args = parser.parse_args()
    fallback = Path("C:/Program Files/JetBrains/CLion 2026.2.3.1/bin/mingw/bin/gcc.exe")
    compiler = args.cc or shutil.which("gcc") or (str(fallback) if fallback.exists() else None)
    if compiler is None:
        parser.error("未找到主机 GCC，请使用 --cc 指定")
    root = Path(__file__).resolve().parents[1]
    cases = scenarios()
    uplinks = uplink_cases()
    stm32_probe(compiler, root, cases, uplinks)
    print(f"PASS: STM32 类型号编解码，下行 {len(cases)} 帧，上行 {len(uplinks)} 帧")
    jetson_cross_check(load_codec(args.jetson_root), load_manual_sender(args.jetson_root))
    print("PASS: Jetson codec 与 STM32 类型号互验")


if __name__ == "__main__":
    main()
