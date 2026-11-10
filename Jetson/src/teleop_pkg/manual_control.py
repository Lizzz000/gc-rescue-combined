#!/usr/bin/env python3
"""独立手动控制程序：独占达妙 USB-CAN，以固定频率发送 0x17F，网页经 HTTP 下发速度。

不启动 ROS，必须先停止 bridge（两者以 flock + TIOCEXCL 互斥）。STM32 须运行手动固件。
访问：在 PC 上 ssh -L 8090:127.0.0.1:8090 nvidia@<jetson>，浏览器打开 http://127.0.0.1:8090/。
"""

import argparse
import signal
import sys
import threading
import time

from manual.command import (
    DEFAULT_ARM_ZERO_S, MAX_ARM_ZERO_S, MAX_LEASE_S, STOP_ZERO_S, ManualCommand)
from manual.link import DamiaoLink, LinkError, manual_packet
from web.server import BIND_ADDRESS, ManualServer

# 与 bringup_pkg/config/rescue.yaml 的 bridge.serial_device 相同，两处须一致。
DEFAULT_DEVICE = '/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00'
DEFAULT_PORT = 8090
DEFAULT_RATE_HZ = 100.0
# 退出前 valid=0 的发送时长，让 STM32 先看到明确撤销再进入超时。
EXIT_INVALID_S = 0.1


def parse_arguments():
    parser = argparse.ArgumentParser(description='独立手动控制：网页上位机 + 达妙 USB-CAN 0x17F。')
    parser.add_argument('--device', default=DEFAULT_DEVICE, help='达妙 USB-CAN 设备路径')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT,
                        help=f'HTTP 端口，只绑定 {BIND_ADDRESS}；0 由系统分配')
    parser.add_argument('--rate-hz', type=float, default=DEFAULT_RATE_HZ, help='0x17F 发送频率')
    parser.add_argument('--lease-s', type=float, default=MAX_LEASE_S,
                        help=f'按住指令租约，(0, {MAX_LEASE_S}] 秒')
    parser.add_argument('--arm-zero-s', type=float, default=DEFAULT_ARM_ZERO_S,
                        help=f'从 valid=0 起步前的使能零速时长，(0, {MAX_ARM_ZERO_S}] 秒；需实车确认')
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error('端口须在 0..65535')
    if not 10.0 <= args.rate_hz <= 1000.0:
        parser.error('发送频率须在 [10, 1000] Hz')
    if not 0.0 < args.lease_s <= MAX_LEASE_S:
        parser.error(f'租约须在 (0, {MAX_LEASE_S}] 秒')
    if not 0.0 < args.arm_zero_s <= MAX_ARM_ZERO_S:
        parser.error(f'使能零速须在 (0, {MAX_ARM_ZERO_S}] 秒')
    return args


class Sender:
    """固定周期把指令状态编码成 0x17F 写入链路；单线程独占写。"""

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
            self._send(self._command.frame(now))
            # 固定节拍不累积唤醒延迟；停顿超过一个周期时从当前时刻重新对齐，不补发积压帧。
            next_send += self._period
            if next_send < now:
                next_send = now + self._period
            stop_event.wait(max(0.0, next_send - time.monotonic()))

    def shutdown(self):
        """退出序列：零速 valid=1 持续 STOP_ZERO_S，再 valid=0 持续 EXIT_INVALID_S。"""
        for frame, seconds in (((True, 0, 0, 0), STOP_ZERO_S), ((False, 0, 0, 0), EXIT_INVALID_S)):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                self._send(frame)
                time.sleep(self._period)

    def _send(self, frame):
        self._link.send(manual_packet(*frame))
        self.frames += 1
        self.last = frame


def main():
    args = parse_arguments()
    command = ManualCommand(args.lease_s, args.arm_zero_s)
    try:
        link = DamiaoLink(args.device)
    except OSError as error:
        print(f'USB-CAN 不可用：{error}。请确认 bridge 已停止且设备可访问。', file=sys.stderr)
        return 1
    sender = Sender(link, command, args.rate_hz)
    server = ManualServer(args.port, command, lambda: {
        'device': args.device, 'frames': sender.frames, 'last_frame': list(sender.last)})
    stop_event = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop_event.set())
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'manual control listening http://{BIND_ADDRESS}:{server.server_address[1]}/ '
          f'device={args.device}', flush=True)
    code = 0
    try:
        sender.run(stop_event)
    except LinkError as error:
        print(f'{error}；请重新插拔适配器后再试。', file=sys.stderr)
        code = 1
    finally:
        # 先发停车序列再关 HTTP，避免 serve_forever 退出等待造成发送间隙。
        if code == 0:
            sender.shutdown()
        link.close()
        server.shutdown()
        server.server_close()
    print('已停车并退出。', flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
