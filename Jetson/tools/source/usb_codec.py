#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""离线核对 USB 报文。文本形如 T01#HEX，数字是类型号，不是电机总线编号。不打开设备。"""
import argparse
import json
import math
import struct

# 类型号与 bridge_pkg/protocol/usb_protocol.hpp 一致。失联是类型 1、alive=0 的 8 字节。
IDS = {'heartbeat': 1, 'manual': 2, 'target': 3, 'safety': 4,
       'arrival_ack': 5, 'roi_reply': 6, 'command_reply': 7,
       'turn_to_safety': 8, 'arrival': 16,
       'turn_completed': 17, 'safety_arrived': 18, 'start_search': 19,
       'delivery_completed': 20, 'arrival_point': 21}
LENGTHS = {1: 8, 2: 7, 3: 8, 4: 8, 5: 1, 6: 3,
           7: 6, 8: 6, 16: 1, 17: 5, 18: 5, 19: 1, 20: 5, 21: 1}
COMMANDS = {'turn_completed': 1, 'safety_arrived': 2, 'start_search': 3,
            'delivery_completed': 4, 'arrival_point': 5}
KINDS = {value: name for name, value in IDS.items()}


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


def frame_text(msg_type: int, data: bytes) -> str:
    return f'T{msg_type:02d}#{data.hex().upper()}'


def encode(kind: str, values: list[str], invalid: bool = False) -> str:
    if kind == 'mission_loss':
        if values or invalid:
            raise ValueError('mission_loss is type 1 with alive=0 and cannot use values or --invalid')
        return frame_text(IDS['heartbeat'], bytes(8))
    if kind not in IDS:
        raise ValueError('unknown frame kind')
    identifier = IDS[kind]
    if kind in ('target', 'safety', 'heartbeat', 'manual'):
        count = 2 if kind in ('target', 'safety') else 3
        if kind == 'heartbeat' and len(values) not in (3, 4):
            raise ValueError('heartbeat needs vx, vy, yaw_rate and optionally pause')
        if kind == 'manual' and len(values) != 3:
            raise ValueError('manual needs vx, vy, yaw_rate')
        if kind in ('target', 'safety') and len(values) != count and not (len(values) == 3):
            raise ValueError(f'{kind} needs {count} values and optionally a source frame')
        source_frame = int(values[2]) if kind in ('target', 'safety') and len(values) == 3 else 0
        if source_frame < 0 or source_frame > 0xffffff:
            raise ValueError('source frame must fit in 24 bits')
        numbers = [float(value) for value in values[:count]]
        try:
            quantized = [fixed_point(value, 1000 if count == 3 else 10000) for value in numbers]
        except ValueError:
            invalid = True
            quantized = [0] * count
        if kind == 'heartbeat':
            pause = 0
            if len(values) == 4:
                pause = int(values[3])
                if pause not in (0, 1):
                    raise ValueError('heartbeat pause must be 0 or 1')
            # alive=0 时速度必须为 0。--invalid 表示失联，不是把 pause 挪到末字节。
            data = bytes([0, pause]) + bytes(6) if invalid else struct.pack('<BBhhh', 1, pause, *quantized)
        elif kind == 'manual':
            data = bytes(LENGTHS[identifier]) if invalid else struct.pack('<Bhhh', 1, *quantized)
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
    return frame_text(identifier, data)


def decode(text: str) -> dict:
    try:
        type_text, payload = text.strip().split('#')
        if not type_text.startswith('T') or not type_text[1:].isdigit():
            raise ValueError
        identifier = int(type_text[1:])
        data = bytes.fromhex(payload)
    except (ValueError, TypeError) as error:
        raise ValueError('expected T01#HEX; the number is a USB type') from error
    if identifier not in LENGTHS or len(data) != LENGTHS[identifier]:
        raise ValueError('unknown USB type or incorrect length')
    result = {'type': identifier, 'length': len(data), 'kind': KINDS[identifier]}
    if identifier == IDS['arrival_point']:
        if data[0] not in (1, 2):
            raise ValueError('arrival point must be 1 or 2')
        result['point'] = data[0]
        return result
    if identifier == IDS['heartbeat']:
        if data[0] not in (0, 1) or data[1] not in (0, 1):
            raise ValueError('heartbeat alive and pause must be 0 or 1')
        vx, vy, wz = struct.unpack('<hhh', data[2:8])
        if data[0] == 0 and (vx or vy or wz):
            raise ValueError('lost heartbeat must have zero speeds')
        result.update(alive=bool(data[0]), lost=data[0] == 0, pause=data[1],
                      vx=vx / 1000, vy=vy / 1000, yaw_rate=wz / 1000)
        return result
    if identifier == IDS['manual']:
        if data[0] not in (0, 1):
            raise ValueError('manual valid byte must be 0 or 1')
        vx, vy, wz = struct.unpack('<hhh', data[1:7])
        result.update(valid=bool(data[0]), vx=vx / 1000, vy=vy / 1000, yaw_rate=wz / 1000)
        if not data[0] and any(data[1:]):
            result['warning'] = 'invalid frame has nonzero payload; ignore its values'
        return result
    if data[0] not in (0, 1):
        raise ValueError('first byte must be 0 or 1')
    if identifier in (IDS['target'], IDS['safety']):
        result['valid'] = bool(data[0])
        result.update(zip(('x', 'y'), [value / 10000 for value in struct.unpack('<hh', data[1:5])]))
        result['source_frame'] = int.from_bytes(data[5:8], 'little')
        if not data[0] and any(data[1:]):
            result['warning'] = 'invalid frame has nonzero payload; ignore its values'
    elif identifier == IDS['arrival_ack']:
        result['accepted'] = bool(data[0])
    elif identifier in (IDS['arrival'], IDS['start_search']):
        if data != b'\x01':
            raise ValueError('command byte must be 1')
        result['request'] = True
    elif identifier == IDS['command_reply']:
        if data[1] not in COMMANDS.values():
            raise ValueError('unknown mission command')
        result['accepted'] = bool(data[0])
        result['command'] = next(name for name, value in COMMANDS.items() if value == data[1])
        result['token'] = struct.unpack('<I', data[2:])[0]
        if data[1] in (COMMANDS['start_search'], COMMANDS['arrival_point']) and result['token'] != 0:
            raise ValueError('start_search and arrival_point reply token must be zero')
    elif identifier == IDS['turn_to_safety']:
        if data[0] != 1 or data[1] not in (1, 2):
            raise ValueError('invalid turn instruction')
        result.update(request=True, destination=('none', 'left', 'right')[data[1]],
                      token=struct.unpack('<I', data[2:])[0])
    elif identifier in (IDS['turn_completed'], IDS['safety_arrived'], IDS['delivery_completed']):
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
    read = commands.add_parser('decode', help='decode T01#HEX text')
    read.add_argument('frame')
    write = commands.add_parser('encode', help='print a frame without sending it')
    write.add_argument('kind', choices=[*IDS, 'mission_loss'])
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
