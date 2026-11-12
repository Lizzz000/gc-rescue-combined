#!/usr/bin/env python3
"""单个 S288 夹爪的串口调试工具；协议走本目录薄转发，不接入自动搬运。"""

import argparse
import math
import sys
import time

from s288_protocol import (
    BAUD, COMMAND_PERIOD_S, FEEDBACK_BYTES, FEEDBACK_MARK, FEEDBACK_WAIT_S,
    MODE_RUN, MODE_STOP, STOP_ATTEMPTS, control_packet, parse_feedback,
)

REPORT_PERIOD_S = 0.5
SERIAL_TIMEOUT_S = 0.05
SERIAL_WRITE_TIMEOUT_S = 0.2
DEFAULT_KP = 1.0
DEFAULT_KD = 0.05


def exchange(port, motor_id, packet):
    """发送一帧并从串口流中查找一帧可校验的反馈。"""
    if port.write(packet) != len(packet):
        raise OSError('串口写入不完整')
    deadline = time.monotonic() + FEEDBACK_WAIT_S
    data = bytearray()
    while time.monotonic() < deadline:
        data.extend(port.read(FEEDBACK_BYTES - len(data)))
        start = data.find(FEEDBACK_MARK)
        if start < 0:
            data[:] = data[-1:]
            continue
        if start:
            del data[:start]
        if len(data) < FEEDBACK_BYTES:
            continue
        try:
            return parse_feedback(bytes(data[:FEEDBACK_BYTES]), motor_id)
        except ValueError:
            del data[0]
    raise TimeoutError('未收到有效的舵机反馈')


def stop(port, motor_id):
    """发送停机命令，读取到 Stop 模式后才视为确认。"""
    packet = control_packet(motor_id, MODE_STOP, 0, 0, 0, 0, 0, 0)
    for _ in range(STOP_ATTEMPTS):
        try:
            feedback = exchange(port, motor_id, packet)
        except TimeoutError:
            continue
        if feedback['mode'] == MODE_STOP:
            return feedback
    raise RuntimeError('已发送停机命令，但未确认 Stop 模式')


def show(feedback):
    print('模式={} 位置={:.5f} rad ({:.2f}°) 速度={:.4f} rad/s 力矩={:.4f} N·m '
          '电压={:.1f} V 温度={}/{}°C 错误=0x{:08x} 警告={}'.format(
              feedback['mode'], feedback['position'], math.degrees(feedback['position']),
              feedback['speed'], feedback['torque'], feedback['voltage'],
              feedback['temperature'], feedback['winding_temperature'],
              feedback['error'], feedback['warning']), flush=True)


def move(port, motor_id, degrees, seconds, kp, kd):
    """按当前反馈建立相对目标，在指定时间内重复发送，结束后停机。"""
    if not all(math.isfinite(value) for value in (degrees, seconds, kp, kd)) or seconds <= 0:
        raise ValueError('角度、时长和增益必须是有限数，时长必须大于零')
    initial = stop(port, motor_id)
    if initial['error'] or initial['timeout']:
        raise RuntimeError('舵机反馈有错误或超时，未开始运动')
    target = initial['position'] + math.radians(degrees)
    command = control_packet(motor_id, MODE_RUN, 1, 0, 0, target, kp, kd)
    print('起始 {:.5f} rad，目标 {:.5f} rad；开始发送，Ctrl+C 请求停机'.format(
        initial['position'], target), flush=True)
    started = time.monotonic()
    next_report = started
    try:
        while time.monotonic() - started < seconds:
            cycle = time.monotonic()
            feedback = exchange(port, motor_id, command)
            if feedback['error'] or feedback['timeout']:
                raise RuntimeError('舵机反馈错误或超时')
            if cycle >= next_report:
                show(feedback)
                next_report = cycle + REPORT_PERIOD_S
            remaining = cycle + COMMAND_PERIOD_S - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
    finally:
        try:
            final = stop(port, motor_id)
            print('已确认 Stop，实际转动 {:.2f}°'.format(
                math.degrees(final['position'] - initial['position'])), flush=True)
            show(final)
        except (OSError, TimeoutError, RuntimeError) as error:
            print('无法确认舵机停机：{}'.format(error), file=sys.stderr, flush=True)
            raise


def main():
    import serial
    parser = argparse.ArgumentParser(description='S288 夹爪串口交互调试')
    parser.add_argument('--device', required=True, help='USB 单总线串口，如 /dev/ttyACM0')
    parser.add_argument('--id', type=int, default=0, help='舵机 ID，默认 0')
    parser.add_argument('--kp', type=float, default=DEFAULT_KP, help='输出位置刚度，默认 1')
    parser.add_argument('--kd', type=float, default=DEFAULT_KD, help='输出速度刚度，默认 0.05')
    args = parser.parse_args()
    control_packet(args.id, MODE_RUN, 1, 0, 0, 0, args.kp, args.kd)
    with serial.Serial(args.device, BAUD, timeout=SERIAL_TIMEOUT_S,
                       write_timeout=SERIAL_WRITE_TIMEOUT_S, exclusive=True) as port:
        print('S288 ID {}，串口 {}；连接后先发送停机命令。'.format(args.id, args.device))
        show(stop(port, args.id))
        print('命令：status 查看反馈并停机；move 角度(°) 时长(s)；'
              'gain Kp Kd；stop 停机；quit 退出')
        while True:
            try:
                parts = input('S288> ').split()
                if not parts:
                    continue
                if parts == ['quit']:
                    break
                if parts in (['stop'], ['status']):
                    show(stop(port, args.id))
                elif parts[0] == 'gain' and len(parts) == 3:
                    kp, kd = float(parts[1]), float(parts[2])
                    control_packet(args.id, MODE_RUN, 1, 0, 0, 0, kp, kd)
                    args.kp, args.kd = kp, kd
                    print('Kp={}，Kd={}'.format(kp, kd))
                elif parts[0] == 'move' and len(parts) == 3:
                    move(port, args.id, float(parts[1]), float(parts[2]), args.kp, args.kd)
                else:
                    print('命令格式不正确')
            except (ValueError, RuntimeError) as error:
                print('操作失败：{}'.format(error), file=sys.stderr)
            except KeyboardInterrupt:
                print('\n已中断；输入 quit 退出。')


if __name__ == '__main__':
    try:
        main()
    except (OSError, TimeoutError, ValueError, RuntimeError, ImportError) as error:
        print('调试程序退出：{}'.format(error), file=sys.stderr)
        sys.exit(1)
