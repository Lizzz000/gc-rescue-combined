#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""离线核对 CAN 载荷。只转换文本，不打开 CAN 接口。"""
import argparse
import json
import math
import struct

# 坐标和速度缩放保持原协议；0x180 为 DLC 8：data[0..6] 仍为 valid + vx/vy/wz，data[7] 为 pause（0/1）。
# 0x17F 不在本工具中编码，避免与自动心跳的 DLC 8 共用路径。
# 任务采用显式握手，不接受旧 0x201/0x202。
IDS = {'velocity': 0x180, 'target': 0x181, 'safety': 0x182,
       'arrival_ack': 0x190, 'roi_reply': 0x191, 'command_reply': 0x192,
       'turn_to_safety': 0x193, 'arrival': 0x200,
       'turn_completed': 0x203, 'safety_arrived': 0x204, 'start_search': 0x205,
       'delivery_completed': 0x206, 'arrival_point': 0x208}
LENGTHS = {0x180: 8, 0x181: 8, 0x182: 8, 0x190: 1, 0x191: 3,
           0x192: 6, 0x193: 6, 0x200: 1, 0x203: 5, 0x204: 5, 0x205: 1, 0x206: 5, 0x208: 1}
COMMANDS = {'turn_completed': 1, 'safety_arrived': 2, 'start_search': 3,
            'delivery_completed': 4, 'arrival_point': 5}


def route_token(value: str) -> int:
    token = int(value)
    if not 0 <= token <= 0xffffffff:
        raise ValueError('token must fit in uint32')
    return token


def fixed_point(value: float, scale: float) -> int:
    """与 C++ std::round 一致，半整数向远离零的方向舍入。"""
    if not math.isfinite(value) or not math.isfinite(scale) or scale <= 0:
        raise ValueError('value and scale must be finite; scale must be positive')
    scaled = value * scale
    if not math.isfinite(scaled):
        raise ValueError('fixed-point overflow')
    # 不先加0.5，避免半整数前一个double在相加时被舍入，偏离std::round。
    magnitude = abs(scaled)
    rounded = math.floor(magnitude)
    if magnitude - rounded >= 0.5:
        rounded += 1
    if scaled < 0:
        rounded = -rounded
    if not -32768 <= rounded <= 32767:
        raise ValueError('fixed-point overflow')
    return int(rounded)


def encode(kind: str, values: list[str], invalid: bool = False) -> str:
    if kind not in IDS:
        raise ValueError('unknown frame kind')
    identifier = IDS[kind]
    if kind in ('target', 'safety', 'velocity'):
        count = 3 if kind == 'velocity' else 2
        if kind == 'velocity' and len(values) not in (3, 4):
            raise ValueError('velocity needs vx, vy, yaw_rate and optionally pause')
        if kind != 'velocity' and len(values) != count and not (count == 2 and len(values) == 3):
            raise ValueError(f'{kind} needs {count} values and optionally a source frame for coordinates')
        source_frame = int(values[2]) if count == 2 and len(values) == 3 else 0
        if source_frame < 0 or source_frame > 0xffffff:
            raise ValueError('source frame must fit in 24 bits')
        numbers = [float(value) for value in values[:count]]
        try:
            quantized = [fixed_point(value, 1000 if count == 3 else 10000) for value in numbers]
        except ValueError:
            invalid = True
            quantized = [0] * count
        if kind == 'velocity':
            data = bytes(7) if invalid else struct.pack('<Bhhh', 1, *quantized)
            pause = 0
            if len(values) == 4:
                pause = int(values[3])
                if pause not in (0, 1):
                    raise ValueError('0x180 pause must be 0 or 1')
            data += bytes([pause])
        else:
            data = bytes(LENGTHS[identifier]) if invalid else struct.pack('<B' + 'h' * count, 1, *quantized)
            if not invalid:
                data += source_frame.to_bytes(3, 'little')
    elif kind == 'arrival_point':
        if len(values) != 1 or values[0] not in ('1', '2') or invalid:
            raise ValueError('arrival_point needs 1 (grab) or 2 (place)')
        data = bytes([int(values[0])])
    elif kind in ('arrival', 'start_search'):
        if values or invalid:
            raise ValueError(f'{kind} needs no values and cannot use --invalid')
        data = b'\x01'
    elif kind == 'arrival_ack':
        if values:
            raise ValueError('arrival_ack needs no values')
        data = bytes([0 if invalid else 1])
    elif kind == 'command_reply':
        if len(values) != 2 or values[0] not in COMMANDS:
            raise ValueError('command_reply needs a command name and its uint32 token')
        token = route_token(values[1])
        if values[0] in ('start_search', 'arrival_point') and token != 0:
            raise ValueError(f'{values[0]} reply token must be zero')
        data = struct.pack('<BBI', int(not invalid), COMMANDS[values[0]], token)
    elif kind == 'turn_to_safety':
        if len(values) != 2 or values[0] not in ('left', 'right') or invalid:
            raise ValueError('turn_to_safety needs left or right and a uint32 token; --invalid is forbidden')
        data = struct.pack('<BBI', 1, 1 if values[0] == 'left' else 2, route_token(values[1]))
    elif kind in ('turn_completed', 'safety_arrived', 'delivery_completed'):
        if len(values) != 1 or invalid:
            raise ValueError(f'{kind} needs a uint32 token; --invalid is forbidden')
        data = struct.pack('<BI', 1, route_token(values[0]))
    else:
        if len(values) != 1 or values[0] not in ('left', 'right', 'rejected'):
            raise ValueError('roi_reply needs left, right or rejected')
        destination = {'left': 1, 'right': 2, 'rejected': 0}[values[0]]
        data = bytes(3) if invalid else bytes([1, int(destination != 0), destination])
    return f'{identifier:03X}#{data.hex().upper()}'


def decode(text: str) -> dict:
    try:
        identifier_text, payload = text.strip().split('#')
        identifier = int(identifier_text, 16)
        data = bytes.fromhex(payload)
    except (ValueError, TypeError) as error:
        raise ValueError('expected ID#HEX, for example 181#0188134C1D010000') from error
    if identifier not in LENGTHS or len(data) != LENGTHS[identifier]:
        raise ValueError('unknown standard ID or incorrect DLC')
    result = {'id': f'0x{identifier:03X}', 'dlc': len(data),
              'kind': next(name for name, value in IDS.items() if value == identifier)}
    if identifier == 0x208:
        if data[0] not in (1, 2):
            raise ValueError('0x208 kind must be 1 or 2')
        result['point'] = data[0]
        return result
    if data[0] not in (0, 1):
        raise ValueError('first byte must be 0 or 1')
    if identifier in (0x180, 0x181, 0x182):
        names = ('vx', 'vy', 'yaw_rate') if identifier == 0x180 else ('x', 'y')
        result['valid'] = bool(data[0])
        scale = 1000 if identifier == 0x180 else 10000
        result.update(zip(names, [value / scale for value in struct.unpack('<' + 'h' * len(names), data[1:1 + 2 * len(names)])]))
        if identifier == 0x180:
            if data[7] not in (0, 1):
                raise ValueError('0x180 pause must be 0 or 1')
            result['pause'] = data[7]
            if not data[0] and any(data[1:7]):
                result['warning'] = 'invalid frame has nonzero payload; ignore its values'
        else:
            result['source_frame'] = int.from_bytes(data[5:8], 'little')
            if not data[0] and any(data[1:]):
                result['warning'] = 'invalid frame has nonzero payload; ignore its values'
    elif identifier == 0x190:
        result['accepted'] = bool(data[0])
    elif identifier in (0x200, 0x205):
        if data != b'\x01':
            raise ValueError('command byte must be 1')
        result['request'] = True
    elif identifier == 0x192:
        if data[1] not in COMMANDS.values():
            raise ValueError('unknown mission command')
        result['accepted'] = bool(data[0])
        result['command'] = next(name for name, value in COMMANDS.items() if value == data[1])
        result['token'] = struct.unpack('<I', data[2:])[0]
        if data[1] in (COMMANDS['start_search'], COMMANDS['arrival_point']) and result['token'] != 0:
            raise ValueError('start_search and arrival_point reply token must be zero')
    elif identifier == 0x193:
        if data[0] != 1 or data[1] not in (1, 2):
            raise ValueError('invalid turn instruction')
        result.update(request=True, destination=('none', 'left', 'right')[data[1]],
                      token=struct.unpack('<I', data[2:])[0])
    elif identifier in (0x203, 0x204, 0x206):
        if data[0] != 1:
            raise ValueError('physical command byte must be 1')
        result.update(request=True, token=struct.unpack('<I', data[1:])[0])
    else:
        # 只有无效、有效拒绝、通过左区、通过右区四种组合；通过必须有目的区。
        if tuple(data) not in ((0, 0, 0), (1, 0, 0), (1, 1, 1), (1, 1, 2)):
            raise ValueError('inconsistent ROI validity/destination')
        result.update(valid=bool(data[0]), passed=bool(data[1]),
                      destination=('none', 'left', 'right')[data[2]])
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    read = commands.add_parser('decode', help='decode ID#HEX text')
    read.add_argument('frame')
    write = commands.add_parser('encode', help='print a frame without sending it')
    write.add_argument('kind', choices=IDS)
    write.add_argument('values', nargs='*')
    write.add_argument('--invalid', action='store_true')
    args = parser.parse_args()
    try:
        output = decode(args.frame) if args.command == 'decode' else encode(args.kind, args.values, args.invalid)
    except ValueError as error:
        parser.error(str(error))
    print(json.dumps(output, ensure_ascii=False, indent=2) if isinstance(output, dict) else output)


if __name__ == '__main__':
    main()
