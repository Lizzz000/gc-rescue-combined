#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""离线核对 USB 报文。文本形如 T03#HEX，数字是类型号，不是电机总线编号。不打开设备。

arrival_point KIND REQUEST_ID（grasp 另加 SOURCE_FRAME）；action_result KIND STATUS REQUEST_ID。
KIND 为 0/startup、1/grasp、2/release、3/reject_open；STATUS 为 0/1；
REQUEST_ID 为非零 uint32。start_search、start_align 同样携带请求 ID，命令 3/5/6 回显各自编号。
"""
import argparse
import json
import math
import struct

# 类型号与 bridge_pkg/protocol/usb_protocol.hpp 一致。
IDS = {'manual': 2, 'target': 3, 'safety': 4,
       'arrival_ack': 5, 'roi_reply': 6, 'command_reply': 7,
       'turn_to_safety': 8, 'action_result': 9, 'arrival': 16,
       'turn_completed': 17, 'safety_arrived': 18, 'start_search': 19,
       'delivery_completed': 20, 'arrival_point': 21, 'start_align': 22}
LENGTHS = {2: 7, 3: 8, 4: 8, 5: 1, 6: 3,
           7: 6, 8: 6, 9: 6, 16: 1, 17: 5, 18: 5, 19: 5, 20: 5, 21: 5, 22: 5}
COMMANDS = {'turn_completed': 1, 'safety_arrived': 2, 'start_search': 3,
            'delivery_completed': 4, 'arrival_point': 5, 'start_align': 6}
KINDS = {value: name for name, value in IDS.items()}
ACTION_KINDS = {'startup': 0, 'grasp': 1, 'release': 2, 'reject_open': 3}


def route_token(value: str) -> int:
    token = int(value)
    if not 0 <= token <= 0xffffffff:
        raise ValueError('token must fit in uint32')
    return token


def action_kind(value) -> int:
    kind = ACTION_KINDS[value] if value in ACTION_KINDS else int(value)
    if kind not in ACTION_KINDS.values():
        raise ValueError('action kind must be 0/startup, 1/grasp, 2/release or 3/reject_open')
    return kind


def action_request_id(value) -> int:
    request_id = route_token(value)
    if request_id == 0:
        raise ValueError('action request_id must be nonzero')
    return request_id


def frame_number(value) -> int:
    number = int(value)
    if not 0 <= number <= 0xffffff:
        raise ValueError('source frame must fit in 24 bits')
    return number


def reply_token(command, value) -> int:
    if command in (COMMANDS['arrival_point'], COMMANDS['start_search'], COMMANDS['start_align']):
        return action_request_id(value)
    return route_token(value)


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
    if kind not in IDS:
        raise ValueError('unknown frame kind')
    identifier = IDS[kind]
    if kind in ('target', 'safety', 'manual'):
        count = 2 if kind in ('target', 'safety') else 3
        if kind == 'manual' and len(values) != 3:
            raise ValueError('manual needs vx, vy, yaw_rate')
        if kind in ('target', 'safety') and len(values) != count and not (len(values) == 3):
            raise ValueError(f'{kind} needs {count} values and optionally a source frame')
        source_frame = frame_number(values[2]) if kind in ('target', 'safety') and len(values) == 3 else 0
        numbers = [float(value) for value in values[:count]]
        try:
            quantized = [fixed_point(value, 1000 if count == 3 else 10000) for value in numbers]
        except ValueError:
            invalid = True
            quantized = [0] * count
        if kind == 'manual':
            data = bytes(LENGTHS[identifier]) if invalid else struct.pack('<Bhhh', 1, *quantized)
        else:
            data = bytes(LENGTHS[identifier]) if invalid else struct.pack('<B' + 'h' * count, 1, *quantized)
            if not invalid:
                data += source_frame.to_bytes(3, 'little')
    elif kind == 'arrival_point':
        if not values or invalid:
            raise ValueError('arrival_point needs kind and request_id; --invalid is forbidden')
        action = action_kind(values[0])
        if len(values) != (3 if action == 1 else 2):
            raise ValueError('arrival_point requires a source frame only for grasp')
        data = struct.pack('<BI', action, action_request_id(values[1]))
        if action == 1:
            data += frame_number(values[2]).to_bytes(3, 'little')
    elif kind == 'action_result':
        if len(values) != 3 or invalid:
            raise ValueError('action_result needs kind, status (0/1) and nonzero uint32 request_id; --invalid is forbidden')
        status = int(values[1])
        if status not in (0, 1):
            raise ValueError('action result status must be 0 or 1')
        data = struct.pack('<BBI', action_kind(values[0]), status, action_request_id(values[2]))
    elif kind in ('start_search', 'start_align'):
        if len(values) != 1 or invalid:
            raise ValueError(f'{kind} needs a nonzero uint32 request_id; --invalid is forbidden')
        data = struct.pack('<BI', 1, action_request_id(values[0]))
    elif kind == 'arrival':
        if values not in ([], ['selected']) or invalid:
            raise ValueError('arrival accepts optional selected and cannot use --invalid')
        data = bytes([2 if values else 1])
    elif kind == 'arrival_ack':
        if values:
            raise ValueError('arrival_ack needs no values')
        data = bytes([0 if invalid else 1])
    elif kind == 'command_reply':
        if len(values) != 2 or values[0] not in COMMANDS:
            raise ValueError('command_reply needs a command name and its uint32 token')
        token = reply_token(COMMANDS[values[0]], values[1])
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
        raise ValueError('expected T03#HEX; the number is a USB type') from error
    expected_length = 8 if identifier == IDS['arrival_point'] and data[:1] == b'\x01' else LENGTHS.get(identifier)
    if len(data) != expected_length:
        raise ValueError('unknown USB type or incorrect length')
    result = {'type': identifier, 'length': len(data), 'kind': KINDS[identifier]}
    if identifier == IDS['arrival_point']:
        result.update(action_kind=action_kind(data[0]),
                      request_id=action_request_id(struct.unpack_from('<I', data, 1)[0]))
        if data[0] == 1:
            result['source_frame'] = int.from_bytes(data[5:8], 'little')
        return result
    if identifier == IDS['action_result']:
        if data[1] not in (0, 1):
            raise ValueError('action result status must be 0 or 1')
        result.update(action_kind=action_kind(data[0]), success=bool(data[1]),
                      request_id=action_request_id(struct.unpack('<I', data[2:])[0]))
        return result
    if identifier == IDS['manual']:
        if data[0] not in (0, 1):
            raise ValueError('manual valid byte must be 0 or 1')
        vx, vy, wz = struct.unpack('<hhh', data[1:7])
        result.update(valid=bool(data[0]), vx=vx / 1000, vy=vy / 1000, yaw_rate=wz / 1000)
        if not data[0] and any(data[1:]):
            result['warning'] = 'invalid frame has nonzero payload; ignore its values'
        return result
    if identifier == IDS['arrival']:
        if data[0] not in (1, 2):
            raise ValueError('arrival request must be 1 (diagnostic) or 2 (selected route)')
        result['request'] = True
        if data[0] == 2:
            result['selected_route'] = True
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
    elif identifier in (IDS['start_search'], IDS['start_align']):
        if data[0] != 1:
            raise ValueError('command byte must be 1')
        result.update(request=True, request_id=action_request_id(struct.unpack('<I', data[1:])[0]))
    elif identifier == IDS['command_reply']:
        if data[1] not in COMMANDS.values():
            raise ValueError('unknown mission command')
        result['accepted'] = bool(data[0])
        result['command'] = next(name for name, value in COMMANDS.items() if value == data[1])
        result['token'] = reply_token(data[1], struct.unpack('<I', data[2:])[0])
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
    read = commands.add_parser('decode', help='decode T03#HEX text')
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
