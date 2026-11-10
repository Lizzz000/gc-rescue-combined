#!/usr/bin/env python3
"""独立手动运动：持续发布速度，定时结束或收到停车命令后发送零速。"""

import argparse
import fcntl
import math
import sys
import time


def finite_number(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError('数值必须有限')
    return number


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='手动定时运动：平移 m/s，车体角速度 rad/s，时间 s。',
        epilog='例：python3 move_timed.py forward 0.08 1.2；停车：python3 move_timed.py stop')
    commands = parser.add_subparsers(dest='command', required=True)
    for direction in ('forward', 'backward', 'left', 'right', 'ccw', 'cw'):
        sub = commands.add_parser(direction, help=f'{direction} 速度 时间')
        sub.add_argument('speed', type=finite_number, help='速度幅值；平移 m/s，转向 rad/s')
        sub.add_argument('seconds', type=finite_number, help='持续时间，秒')
    velocity = commands.add_parser('velocity', help='指定 vx vy wz 时间')
    velocity.add_argument('vx', type=finite_number, help='前进速度，m/s')
    velocity.add_argument('vy', type=finite_number, help='左移速度，m/s')
    velocity.add_argument('wz', type=finite_number, help='逆时针车体角速度，rad/s')
    velocity.add_argument('seconds', type=finite_number, help='持续时间，秒')
    commands.add_parser('stop', help='停止本脚本动作并发送零速')
    args = parser.parse_args()
    if args.command == 'stop':
        return None
    if not 0.0 < args.seconds <= 3600.0:
        parser.error('持续时间必须在 (0, 3600] 秒内')
    if args.command == 'velocity':
        return [args.vx, args.vy, args.wz, args.seconds]
    if args.speed <= 0.0:
        parser.error('方向命令的速度幅值必须大于 0')
    directions = {
        'forward': (1.0, 0.0, 0.0), 'backward': (-1.0, 0.0, 0.0),
        'left': (0.0, 1.0, 0.0), 'right': (0.0, -1.0, 0.0),
        'ccw': (0.0, 0.0, 1.0), 'cw': (0.0, 0.0, -1.0),
    }
    x, y, angle = directions[args.command]
    return [x * args.speed, y * args.speed, angle * args.speed, args.seconds]


def main():
    values = parse_arguments()
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import Empty
    except ImportError:
        print('无法加载 ROS 2 Python 包，请先 source /opt/ros/humble/setup.bash。', file=sys.stderr)
        return 1

    lock = None
    if values is not None:
        lock = open('/tmp/gc_rescue_timed_motion.lock', 'w')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('已有定时运动脚本在运行，请先停车。', file=sys.stderr)
            lock.close()
            return 1

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('move_timed_command')
    cancelled = False
    publisher = None

    def stop_callback(_message):
        nonlocal cancelled
        cancelled = True

    try:
        publisher = node.create_publisher(Twist, '/cmd_vel', 1)
        stop_publisher = node.create_publisher(Empty, '/teleop/stop', 1)
        node.create_subscription(Empty, '/teleop/stop', stop_callback, 1)
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            print('没有找到 /cmd_vel 的订阅者，请先启动完整程序。', file=sys.stderr)
            return 1

        if values is None:
            # 短时重复，给运行中的脚本留出话题发现时间。
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                stop_publisher.publish(Empty())
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.02)
            print('已发送停车命令。')
            return 0

        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = values[:3]
        for duration, command in ((0.5, Twist()), (values[3], message)):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline and not cancelled:
                publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.02)
            if cancelled:
                break
        print('定时运动已停止。' if cancelled else '定时命令执行时间已结束。')
        return 0
    except KeyboardInterrupt:
        print('已中断，发送零速。')
        return 130
    finally:
        if publisher is not None and rclpy.ok():
            for _ in range(5):
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()
        if lock is not None:
            lock.close()


if __name__ == '__main__':
    sys.exit(main())
