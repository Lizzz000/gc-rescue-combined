#!/usr/bin/env python3
"""独立 USB-CAN 手动定时运动，不启动 ROS 或自动搬运程序。"""

import argparse
import fcntl
import math
import os
import select
import signal
import socket
import subprocess
import struct
import sys
import termios
import time
import tty

DEFAULT_DEVICE = '/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00'
STOP_SOCKET = '/tmp/gc_rescue_motion_stop.sock'
LOCK_FILE = '/tmp/gc_rescue_motion.lock'


class IncompletePacket(OSError):
    pass


def finite_number(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError('数值必须有限')
    return number


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='独立 USB-CAN 手动运动：平移 m/s，车体角速度 rad/s，时间 s。',
        epilog='例：python3 move_timed.py forward 0.08 1.2；停车：python3 move_timed.py stop')
    parser.add_argument('--device', default=DEFAULT_DEVICE, help='达妙 USB-CAN 设备路径')
    commands = parser.add_subparsers(dest='command')
    for direction in ('forward', 'backward', 'left', 'right', 'ccw', 'cw'):
        sub = commands.add_parser(direction, help=f'{direction} 速度 时间')
        sub.add_argument('speed', type=finite_number, help='速度幅值；平移 m/s，转向 rad/s')
        sub.add_argument('seconds', type=finite_number, help='持续时间，秒')
    velocity = commands.add_parser('velocity', help='指定 vx vy wz 时间')
    velocity.add_argument('vx', type=finite_number, help='前进速度，m/s')
    velocity.add_argument('vy', type=finite_number, help='左移速度，m/s')
    velocity.add_argument('wz', type=finite_number, help='逆时针车体角速度，rad/s')
    velocity.add_argument('seconds', type=finite_number, help='持续时间，秒')
    commands.add_parser('stop', help='停止正在执行的手动动作')
    args = parser.parse_args()
    if args.command is None:
        return args.device, 'interactive'
    if args.command == 'stop':
        return args.device, None
    if not 0.0 < args.seconds <= 3600.0:
        parser.error('持续时间必须在 (0, 3600] 秒内')
    if args.command == 'velocity':
        values = [args.vx, args.vy, args.wz, args.seconds]
    else:
        if args.speed <= 0.0:
            parser.error('方向命令的速度幅值必须大于 0')
        directions = {
            'forward': (1.0, 0.0, 0.0), 'backward': (-1.0, 0.0, 0.0),
            'left': (0.0, 1.0, 0.0), 'right': (0.0, -1.0, 0.0),
            'ccw': (0.0, 0.0, 1.0), 'cw': (0.0, 0.0, -1.0),
        }
        x, y, angle = directions[args.command]
        values = [x * args.speed, y * args.speed, angle * args.speed, args.seconds]
    if any(value < -32.768 or value > 32.767 for value in values[:3]):
        parser.error('速度超出 CAN 编码范围')
    return args.device, values


def make_packet(values, valid):
    packet = bytearray(30)
    packet[:5] = bytes([0x55, 0xaa, 30, 1, 1])
    packet[8] = 10
    packet[13:17] = (0x17f).to_bytes(4, 'little')
    packet[18] = 7
    packet[21:28] = struct.pack('<Bhhh', int(valid), *(round(value * 1000) for value in values))
    return packet


def send_for(fd, packet, seconds, stop_socket=None):
    deadline = time.monotonic() + seconds
    next_send = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:
            try:
                os.read(fd, 4096)
            except BlockingIOError:
                pass
            count = os.write(fd, packet)
            if count != len(packet):
                raise IncompletePacket('串口报文写入不完整')
            next_send = now + 0.01
        wait = max(0.0, min(deadline, next_send) - time.monotonic())
        if stop_socket is None:
            time.sleep(wait)
        elif select.select([stop_socket], [], [], wait)[0]:
            stop_socket.recv(32)
            return False
    return True


def numeric_menu(device):
    linear, angular, seconds = 0.08, 0.2, 1.2
    directions = {
        '1': ('前进', 'forward'), '2': ('后退', 'backward'),
        '3': ('左移', 'left'), '4': ('右移', 'right'),
        '5': ('左转', 'ccw'), '6': ('右转', 'cw'),
    }
    print('独立手动控制：运动时按 Ctrl+C 可停车。')
    while True:
        print(f'\n平移 {linear:g} m/s | 转向 {angular:g} rad/s | 时间 {seconds:g} s')
        print('1 前进  2 后退  3 左移  4 右移  5 左转  6 右转')
        print('7 修改速度和时间  8 组合速度  0 停车  q 退出')
        try:
            choice = input('请选择：').strip().lower()
            if choice in ('q', 'quit', 'exit'):
                return 0
            if choice == '7':
                entries = (
                    input(f'平移速度 m/s [{linear:g}]：').strip(),
                    input(f'车体转向速度 rad/s [{angular:g}]：').strip(),
                    input(f'持续时间 s [{seconds:g}]：').strip(),
                )
                numbers = [float(value) if value else default for value, default in
                           zip(entries, (linear, angular, seconds))]
                if not all(math.isfinite(value) and value > 0 for value in numbers) or numbers[2] > 3600:
                    print('请输入正的有限速度，时间为 (0, 3600] 秒。')
                    continue
                linear, angular, seconds = numbers
                continue
            if choice == '8':
                entries = input('输入 vx vy wz 时间（空格分隔）：').split()
                if len(entries) != 4:
                    print('请输入四个数值。')
                    continue
                action = ['velocity', *entries]
            elif choice == '0':
                action = ['stop']
            elif choice in directions:
                label, command = directions[choice]
                speed = angular if choice in ('5', '6') else linear
                print(f'开始{label}；按 Ctrl+C 停车。', flush=True)
                action = [command, str(speed), str(seconds)]
            else:
                print('未识别选项，请重新选择。')
                continue
            subprocess.run([sys.executable, __file__, '--device', device, *action], check=False)
        except (EOFError, KeyboardInterrupt):
            print('\n已退出。')
            return 0
        except ValueError:
            print('请输入有效数字。')


def interactive(device):
    if not sys.stdin.isatty() or not sys.stdout.isatty() or os.environ.get('TERM') in (None, '', 'dumb'):
        return numeric_menu(device)

    linear, angular, seconds = 0.08, 0.2, 1.2
    terminal = sys.stdin.fileno()
    original = termios.tcgetattr(terminal)
    message = '选择方向开始定时运动。'
    keys = {
        b'\x1b[A': ('前进', 'forward'), b'\x1b[B': ('后退', 'backward'),
        b'\x1b[D': ('左移', 'left'), b'\x1b[C': ('右移', 'right'),
        b'q': ('左转', 'ccw'), b'e': ('右转', 'cw'),
    }

    def display(status):
        print('\x1b[2J\x1b[H', end='')
        print('独立手动定时控制')
        print(f'设备：{device}')
        print(f'平移：{linear:g} m/s   转向：{angular:g} rad/s   时间：{seconds:g} s')
        print('方向键：前后左右平移    Q / E：左转 / 右转')
        print('P：设置参数    V：组合速度    X：退出')
        print('运动时 Ctrl+C 停车；也可在另一终端执行 stop。')
        print(f'\n{status}', flush=True)

    try:
        tty.setcbreak(terminal)
        while True:
            display(message)
            key = os.read(terminal, 1).lower()
            if key == b'\x1b':
                while len(key) < 3 and select.select([terminal], [], [], 0.05)[0]:
                    key += os.read(terminal, 1)
            if not key or key == b'x':
                return 0
            if key in (b'p', b'v'):
                termios.tcsetattr(terminal, termios.TCSADRAIN, original)
                try:
                    if key == b'p':
                        values = [
                            input(f'平移速度 m/s [{linear:g}]：').strip(),
                            input(f'转向速度 rad/s [{angular:g}]：').strip(),
                            input(f'持续时间 s [{seconds:g}]：').strip(),
                        ]
                        numbers = [float(value) if value else default for value, default in
                                   zip(values, (linear, angular, seconds))]
                        if (not all(math.isfinite(value) and value > 0 for value in numbers)
                                or any(value > 32.767 for value in numbers[:2]) or numbers[2] > 3600):
                            raise ValueError('速度必须为 (0, 32.767]，时间为 (0, 3600] 秒。')
                        linear, angular, seconds = numbers
                        message = '参数已更新。'
                        continue
                    values = input('输入 vx vy wz 时间（空格分隔）：').split()
                    if len(values) != 4:
                        raise ValueError('请输入四个数值。')
                    numbers = [float(value) for value in values]
                    if (not all(math.isfinite(value) for value in numbers)
                            or any(value < -32.768 or value > 32.767 for value in numbers[:3])
                            or not 0 < numbers[3] <= 3600):
                        raise ValueError('速度超出 CAN 编码范围，或时间不在 (0, 3600] 秒。')
                    label, action, duration = '组合速度', ['velocity', *values], numbers[3]
                except ValueError as error:
                    message = f'输入无效：{error}'
                    continue
                finally:
                    tty.setcbreak(terminal)
            elif key in keys:
                label, direction = keys[key]
                speed = angular if key in (b'q', b'e') else linear
                action, duration = [direction, str(speed), str(seconds)], seconds
            else:
                message = '未识别按键，请使用屏幕上的快捷键。'
                continue

            process = subprocess.Popen([sys.executable, __file__, '--device', device, *action],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                       text=True, start_new_session=True)
            start = time.monotonic()
            interrupted = False
            try:
                while process.poll() is None:
                    elapsed = time.monotonic() - start
                    if elapsed < 0.5:
                        status = f'{label}：发送零速准备中；Ctrl+C 停车。'
                    else:
                        remaining = max(0.0, duration - (elapsed - 0.5))
                        status = f'{label}：预计剩余 {remaining:.1f} s；Ctrl+C 停车。'
                    display(status)
                    if select.select([terminal], [], [], 0.1)[0]:
                        os.read(terminal, 32)  # 动作期间丢弃其他按键。
            except KeyboardInterrupt:
                interrupted = True
                if process.poll() is None:
                    try:
                        process.send_signal(signal.SIGINT)
                    except ProcessLookupError:
                        pass
            output, error = process.communicate()
            if error.strip():
                message = f'发送提示：{error.strip()}'
            elif interrupted:
                message = '已请求停车；未读取电机反馈。'
            elif process.returncode == 0:
                message = '动作已结束，已尝试发送停车报文；未读取电机反馈。'
            else:
                message = f'动作失败：{error.strip() or output.strip() or process.returncode}'
    except (EOFError, KeyboardInterrupt):
        return 0
    finally:
        termios.tcsetattr(terminal, termios.TCSADRAIN, original)
        print('\n已退出。')


def main():
    device, values = parse_arguments()
    if values == 'interactive':
        return interactive(device)
    if values is None:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
            try:
                sender.sendto(b'stop', STOP_SOCKET)
            except (FileNotFoundError, ConnectionRefusedError):
                print('没有正在运行的手动脚本。')
                return 0
        print('已发送停车请求。')
        return 0

    with open(LOCK_FILE, 'w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('已有手动运动脚本在运行，请先停车。', file=sys.stderr)
            return 1
        fd = None
        stop_socket = None
        port_ready = False
        exclusive = False
        incomplete = False
        started = False
        try:
            fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            exclusive = True
            tty.setraw(fd)
            settings = termios.tcgetattr(fd)
            settings[4] = settings[5] = termios.B921600
            termios.tcsetattr(fd, termios.TCSANOW, settings)
            port_ready = True
            stop_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            if os.path.exists(STOP_SOCKET):
                os.unlink(STOP_SOCKET)
            stop_socket.bind(STOP_SOCKET)
            zero = make_packet((0.0, 0.0, 0.0), True)
            movement = make_packet(values[:3], True)
            started = True
            if send_for(fd, zero, 0.5, stop_socket):
                send_for(fd, movement, values[3], stop_socket)
            print('手动命令结束，正在停车。')
            return 0
        except KeyboardInterrupt:
            print('已中断，正在停车。')
            return 130
        except IncompletePacket as error:
            incomplete = True
            print(f'{error}；请重新插拔适配器后再试。', file=sys.stderr)
            return 1
        except OSError as error:
            print(f'USB-CAN 不可用：{error}。请确认桥接程序已停止且设备可访问。', file=sys.stderr)
            return 1
        finally:
            if fd is not None:
                if port_ready and started and not incomplete:
                    try:
                        send_for(fd, make_packet((0.0, 0.0, 0.0), True), 0.3)
                        send_for(fd, make_packet((0.0, 0.0, 0.0), False), 0.1)
                        termios.tcdrain(fd)
                    except OSError as error:
                        print(f'停车报文未能完成发送：{error}', file=sys.stderr)
                if exclusive:
                    try:
                        fcntl.ioctl(fd, termios.TIOCNXCL)
                    except OSError:
                        pass
                os.close(fd)
            if stop_socket is not None:
                stop_socket.close()
                if os.path.exists(STOP_SOCKET):
                    os.unlink(STOP_SOCKET)


if __name__ == '__main__':
    sys.exit(main())
