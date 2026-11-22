#!/usr/bin/env python3
"""独立手动控制程序：独占 STM32 的 USB 设备，以固定频率发送类型 2，网页经 HTTP 下发速度。

舵机转角只走本程序自己的 S288 串口，不经过 CAN，也不参与自动搬运。
不启动 ROS，必须先停止 bridge（两者以 flock + TIOCEXCL 互斥）。STM32 须运行手动固件。
访问：在 PC 上 ssh -L 8090:127.0.0.1:8090 nvidia@<jetson>，浏览器打开 http://127.0.0.1:8090/。

设备路径和夹爪参数来自 --config 中的 link.device 与 gripper.*，不读 bridge.* / mission / vision。
现场低速可加 --max-linear-mps 0.25 --max-angular-radps 0.5 --max-timed-s 1.5；
这些上限由后端执行，网页读取同一组有效值。
"""

import argparse
import math
from pathlib import Path
import signal
import sys
import threading
import time

import yaml

from manual.command import (
    DEFAULT_ARM_ZERO_S, MAX_ARM_ZERO_S, MAX_LEASE_S, MAX_LINEAR_MPS,
    MAX_ANGULAR_RADPS, MAX_TIMED_S, STOP_ZERO_S, ManualCommand)
from manual.link import CdcLink, LinkError, manual_packet
from servo.s288 import (MAX_MOVE_DEGREES, MAX_MOVE_SECONDS, MAX_OPEN_HOLD_NM, MIN_MOVE_DEGREES,
                        MIN_MOVE_SECONDS, ServoSession, check_ramp_fraction)
from web.server import BIND_ADDRESS, ManualServer

DEFAULT_PORT = 8090
DEFAULT_RATE_HZ = 100.0
# 退出前 valid=0 的发送时长，让 STM32 先看到明确撤销再进入超时。
EXIT_INVALID_S = 0.1
_GRIPPER_KEYS = ('device', 'id', 'kp', 'kd', 'closed_deg', 'open_deg', 'move_s', 'ramp_fraction',
                 'open_hold_max_nm', 'output_tolerance_deg', 'open_trim_deg', 'closed_trim_deg')


def _mapping(path, document, label):
    if not isinstance(document, dict):
        raise ValueError(f'{path}: {label}须为映射')
    return document


def _merge_fields(document, values, prefix=''):
    for key, value in document.items():
        name = f'{prefix}.{key}' if prefix else key
        if isinstance(value, dict):
            _merge_fields(value, values, name)
        elif name in values:
            raise ValueError(f'配置字段重复：{name}')
        else:
            values[name] = value


def _read_config_values(path):
    """读取 --config 及其 advanced_config 引用，按叶键合并；只用于提取中性键。"""
    path = Path(path).resolve()
    document = _mapping(path, yaml.safe_load(path.read_bytes()), '配置')
    reference = document.pop('advanced_config', None)
    values = {}
    _merge_fields(document, values)
    if reference is None:
        return values
    if not isinstance(reference, str) or not reference.strip():
        raise ValueError('advanced_config 须为非空路径')
    advanced_path = Path(reference)
    if not advanced_path.is_absolute():
        advanced_path = path.parent / advanced_path
    advanced = _mapping(advanced_path, yaml.safe_load(advanced_path.read_bytes()), '高级配置')
    _merge_fields(advanced, values)
    return values


def load_manual_config(path):
    """只取 link.device 与 gripper.*；其它顶层段忽略。"""
    values = _read_config_values(path)
    link_keys = {key for key in values if key == 'link' or key.startswith('link.')}
    gripper_keys = {key for key in values if key == 'gripper' or key.startswith('gripper.')}
    unknown_link = link_keys - {'link.device'}
    if unknown_link:
        raise ValueError(f'link 含未知键：{sorted(unknown_link)}')
    unknown_gripper = gripper_keys - {f'gripper.{name}' for name in _GRIPPER_KEYS}
    if unknown_gripper:
        raise ValueError(f'gripper 含未知键：{sorted(unknown_gripper)}')
    config = {
        'link_device': values.get('link.device'),
        'gripper_device': values.get('gripper.device') or '',
        'gripper_id': values.get('gripper.id', 0),
        'kp': values.get('gripper.kp'),
        'kd': values.get('gripper.kd'),
        'closed_deg': values.get('gripper.closed_deg'),
        'open_deg': values.get('gripper.open_deg'),
        'move_s': values.get('gripper.move_s'),
        'ramp_fraction': values.get('gripper.ramp_fraction', 0.0),
        'open_hold_max_nm': values.get('gripper.open_hold_max_nm', 0.0),
        'output_tolerance_deg': values.get('gripper.output_tolerance_deg', 1.0),
        'open_trim_deg': values.get('gripper.open_trim_deg', 0.0),
        'closed_trim_deg': values.get('gripper.closed_trim_deg', 0.0),
    }
    if config['link_device'] is not None and not isinstance(config['link_device'], str):
        raise ValueError('link.device 须为字符串')
    if not isinstance(config['gripper_device'], str):
        raise ValueError('gripper.device 须为字符串')
    if type(config['gripper_id']) is bool or not isinstance(config['gripper_id'], int):
        raise ValueError('gripper.id 须为整数')
    if not 0 <= config['gripper_id'] <= 14:
        raise ValueError('gripper.id 须在 0 到 14 之间')
    for name in ('kp', 'kd', 'closed_deg', 'open_deg'):
        value = config[name]
        if value is None:
            continue
        if type(value) is bool or not isinstance(value, (int, float)):
            raise ValueError(f'gripper.{name} 须为数值或留空')
        config[name] = float(value)
    move_s = config['move_s']
    if move_s is not None:
        if (type(move_s) is bool or not isinstance(move_s, (int, float)) or
                not MIN_MOVE_SECONDS <= move_s <= MAX_MOVE_SECONDS):
            raise ValueError(f'gripper.move_s 须在 {MIN_MOVE_SECONDS:g} 到 {MAX_MOVE_SECONDS:g} 秒之间')
        config['move_s'] = float(move_s)
    check_ramp_fraction(config['ramp_fraction'])
    config['ramp_fraction'] = float(config['ramp_fraction'])
    for name in ('open_hold_max_nm', 'output_tolerance_deg', 'open_trim_deg', 'closed_trim_deg'):
        value = config[name]
        if type(value) is bool or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'gripper.{name} 须为有限数值')
        config[name] = float(value)
    if not 0.0 <= config['open_hold_max_nm'] <= MAX_OPEN_HOLD_NM:
        raise ValueError(f'gripper.open_hold_max_nm 须为 0 到 {MAX_OPEN_HOLD_NM:g} 之间的数值')
    if not 0.0 < config['output_tolerance_deg'] < 180.0:
        raise ValueError('gripper.output_tolerance_deg 须大于 0 且小于 180')
    for end in ('open', 'closed'):
        target = config[f'{end}_deg']
        if target is not None and not MIN_MOVE_DEGREES <= target + config[f'{end}_trim_deg'] <= MAX_MOVE_DEGREES:
            raise ValueError(f'gripper.{end}_trim_deg 补偿后的角度超出范围')
    if config['open_hold_max_nm'] > 0.0 and config['kp'] is not None and config['kp'] <= 0.0:
        raise ValueError('张开保位需要正的 gripper.kp')
    if config['gripper_device']:
        missing = [name for name, value in (
            ('kp', config['kp']), ('kd', config['kd']),
            ('closed_deg', config['closed_deg']), ('open_deg', config['open_deg']),
        ) if value is None]
        if missing:
            raise ValueError('配置了 gripper.device 时必须提供 gripper.' + '、gripper.'.join(missing))
    return config


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description='独立手动控制：网页上位机经 USB 发送类型 2。')
    parser.add_argument('--config', required=True,
                        help='rescue.yaml 或等价文件；只读 link.device 与 gripper.*')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT,
                        help=f'HTTP 端口，只绑定 {BIND_ADDRESS}；0 由系统分配')
    parser.add_argument('--rate-hz', type=float, default=DEFAULT_RATE_HZ, help='类型 2 发送频率')
    parser.add_argument('--lease-s', type=float, default=MAX_LEASE_S,
                        help=f'按住指令租约，(0, {MAX_LEASE_S}] 秒')
    parser.add_argument('--arm-zero-s', type=float, default=DEFAULT_ARM_ZERO_S,
                        help=f'从 valid=0 起步前的使能零速时长，(0, {MAX_ARM_ZERO_S}] 秒；需实车确认')
    parser.add_argument('--max-linear-mps', type=float, default=MAX_LINEAR_MPS,
                        help=f'后端平移合速度上限，(0, {MAX_LINEAR_MPS}] m/s')
    parser.add_argument('--max-angular-radps', type=float, default=MAX_ANGULAR_RADPS,
                        help=f'后端转向速度上限，(0, {MAX_ANGULAR_RADPS}] rad/s')
    parser.add_argument('--max-timed-s', type=float, default=MAX_TIMED_S,
                        help=f'定时动作最长时长，(0, {MAX_TIMED_S:g}] 秒')
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error('端口须在 0..65535')
    if not 10.0 <= args.rate_hz <= 1000.0:
        parser.error('发送频率须在 [10, 1000] Hz')
    if not 0.0 < args.lease_s <= MAX_LEASE_S:
        parser.error(f'租约须在 (0, {MAX_LEASE_S}] 秒')
    if not 0.0 < args.arm_zero_s <= MAX_ARM_ZERO_S:
        parser.error(f'使能零速须在 (0, {MAX_ARM_ZERO_S}] 秒')
    try:
        ManualCommand(args.lease_s, args.arm_zero_s, max_linear_mps=args.max_linear_mps,
                      max_angular_radps=args.max_angular_radps, max_timed_s=args.max_timed_s)
    except ValueError as error:
        parser.error(str(error))
    try:
        config = load_manual_config(args.config)
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.error(str(error))
    if not config['link_device']:
        parser.error('--config 必须提供 link.device')
    args.manual_config = config
    return args


class Sender:
    """固定周期把指令状态编码成类型 2 写入链路；单线程独占写。"""

    def __init__(self, link, command, rate_hz):
        self._link = link
        self._command = command
        self._period = 1.0 / rate_hz
        self.frames = 0
        self.last = (False, 0, 0, 0)

    def run(self, stop_event):
        next_send = time.monotonic()
        while not stop_event.is_set():
            now = time.monotonic()
            frame = self._command.frame(now)
            if frame is not None:
                self._send(frame)
            # 固定节拍不累积唤醒延迟；停顿超过一个周期时从当前时刻重新对齐，不补发积压帧。
            next_send += self._period
            if next_send < now:
                next_send = now + self._period
            stop_event.wait(max(0.0, next_send - time.monotonic()))

    def shutdown(self):
        """未接管时静默退出，已接管时发送停车和撤权序列。"""
        if not self.frames and self._command.frame(time.monotonic()) is None:
            return
        sequence = [((False, 0, 0, 0), EXIT_INVALID_S)]
        if self.last[0]:
            sequence.insert(0, ((True, 0, 0, 0), STOP_ZERO_S))
        for frame, seconds in sequence:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                self._send(frame)
                time.sleep(self._period)

    def _send(self, frame):
        self._link.send(manual_packet(*frame))
        self.frames += 1
        self.last = frame


def main(argv=None):
    args = parse_arguments(argv)
    config = args.manual_config
    command = ManualCommand(args.lease_s, args.arm_zero_s, max_linear_mps=args.max_linear_mps,
                            max_angular_radps=args.max_angular_radps, max_timed_s=args.max_timed_s)
    try:
        link = CdcLink(config['link_device'])
    except OSError as error:
        print(f'USB 设备不可用：{error}。请确认 bridge 已停止且 link.device 可访问。', file=sys.stderr)
        return 1
    kp = 0.0 if config['kp'] is None else config['kp']
    kd = 0.0 if config['kd'] is None else config['kd']
    servo = ServoSession(
        config['gripper_device'], config['gripper_id'], kp, kd,
        config['closed_deg'], config['open_deg'], ramp_fraction=config['ramp_fraction'],
        open_hold_max_nm=config['open_hold_max_nm'], output_tolerance_deg=config['output_tolerance_deg'],
        open_trim_deg=config['open_trim_deg'], closed_trim_deg=config['closed_trim_deg'])
    if config['gripper_device'] and not servo.available:
        print(f'舵机未打开：{servo.message}。底盘控制仍可使用。', file=sys.stderr)
    sender = Sender(link, command, args.rate_hz)
    server = ManualServer(args.port, command, lambda: {
        'device': config['link_device'], 'frames': sender.frames, 'last_frame': list(sender.last)},
        servo, servo_move_s=config['move_s'])
    stop_event = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop_event.set())
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'manual control listening http://{BIND_ADDRESS}:{server.server_address[1]}/ '
          f'device={config["link_device"]}', flush=True)
    code = 0
    try:
        sender.run(stop_event)
    except LinkError as error:
        print(f'{error}；请确认 link.device 可访问后再试。', file=sys.stderr)
        code = 1
    finally:
        # 先发停车序列再关 HTTP，避免 serve_forever 退出等待造成发送间隙。
        if code == 0:
            try:
                sender.shutdown()
            except LinkError as error:
                print(f'退出停车发送失败：{error}。', file=sys.stderr)
                code = 1
        # 拔线后关闭串口也可能失败；各资源仍须逐项关闭。
        for label, close in (('USB', link.close), ('舵机', servo.close),
                             ('HTTP 服务', server.shutdown), ('HTTP 套接字', server.server_close)):
            try:
                close()
            except OSError as error:
                print(f'{label}关闭失败：{error}。', file=sys.stderr)
                code = 1
    if code == 0:
        print('已停车并退出。' if sender.frames else '未接管底盘，服务已退出。', flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
