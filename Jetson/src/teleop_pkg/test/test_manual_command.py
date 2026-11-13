"""ManualCommand 纯状态：限幅、租约、准备与停车序列（注入时间）。"""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manual.command import (  # noqa: E402
    DEFAULT_ARM_ZERO_S, MAX_ARM_ZERO_S, MAX_LEASE_S, STOP_ZERO_S, ManualCommand, quantize)
from manual.link import MANUAL_ID, manual_packet  # noqa: E402

LEASE = MAX_LEASE_S
ARM = DEFAULT_ARM_ZERO_S


class QuantizeTest(unittest.TestCase):
    def test_linear_and_angular_limits(self):
        vx, vy, wz = quantize(1.0, 1.0, -5.0)
        self.assertLessEqual((vx * vx + vy * vy) ** 0.5, 500)
        self.assertEqual(wz, -2000)
        self.assertEqual(quantize(0.08, -0.1, 0.2), (80, -100, 200))

    def test_rejects_non_finite(self):
        for value in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                quantize(value, 0, 0)

    def test_packet_layout(self):
        packet = manual_packet(True, -2, 0, 1000)
        from manual.frame import Parser
        frames = Parser().feed(packet)
        self.assertEqual(frames, [(MANUAL_ID, bytes([1, 0xFE, 0xFF, 0, 0, 0xE8, 0x03]))])


class LeaseTest(unittest.TestCase):
    def test_lease_bounds(self):
        for lease in (0.0, MAX_LEASE_S + 0.01):
            with self.assertRaises(ValueError):
                ManualCommand(lease, ARM)
        for arm in (0.0, MAX_ARM_ZERO_S + 0.01):
            with self.assertRaises(ValueError):
                ManualCommand(LEASE, arm)

    def test_idle_is_invalid(self):
        self.assertEqual(ManualCommand(LEASE, ARM).frame(0.0), (False, 0, 0, 0))

    def test_hold_prepares_then_moves_then_expires(self):
        command = ManualCommand(LEASE, ARM)
        t = 0.0
        while t < ARM - 1e-9:
            command.hold(0.1, 0, 0, t)
            self.assertEqual(command.frame(t), (True, 0, 0, 0))
            t += 0.02
        t = ARM
        command.hold(0.1, 0, 0, t)
        self.assertEqual(command.frame(t), (True, 100, 0, 0))
        # 不再续租：租约内保持速度，过期后先零速 valid=1，STOP_ZERO_S 后 valid=0。
        self.assertEqual(command.frame(t + LEASE - 0.01), (True, 100, 0, 0))
        self.assertEqual(command.frame(t + LEASE), (True, 0, 0, 0))
        self.assertEqual(command.frame(t + LEASE + STOP_ZERO_S - 0.01), (True, 0, 0, 0))
        self.assertEqual(command.frame(t + LEASE + STOP_ZERO_S), (False, 0, 0, 0))

    def test_zero_in_session_skips_arm_but_stop_rearms(self):
        command = ManualCommand(LEASE, ARM)
        command.hold(0, 0, 1.0, 0.0)
        self.assertEqual(command.frame(ARM), (True, 0, 0, 1000))
        # 会话中指令零速再起步：不重发使能零速。
        command.hold(0, 0, 0, ARM + 0.05)
        command.hold(0, 0.2, 0, ARM + 0.1)
        self.assertEqual(command.frame(ARM + 0.1), (True, 0, 200, 0))
        # 停车序列中（尚未到 valid=0）再起步：重新使能。
        command.stop(1.0)
        command.hold(0, 0.2, 0, 1.1)
        self.assertEqual(command.frame(1.1), (True, 0, 0, 0))
        self.assertEqual(command.frame(1.1 + ARM), (True, 0, 200, 0))
        # 停车完成（valid=0）后再起步：同样重新使能。
        command.stop(1.5)
        command.hold(0, 0.2, 0, 1.5 + STOP_ZERO_S)
        self.assertEqual(command.frame(1.5 + STOP_ZERO_S), (True, 0, 0, 0))
        self.assertEqual(command.frame(1.5 + STOP_ZERO_S + ARM), (True, 0, 200, 0))

    def test_timed_duration_excludes_prepare(self):
        command = ManualCommand(LEASE, ARM)
        command.timed(0.2, 0, 0, 1.0, 0.0)
        self.assertEqual(command.frame(ARM + 0.99), (True, 200, 0, 0))
        self.assertEqual(command.frame(ARM + 1.0), (True, 0, 0, 0))
        with self.assertRaises(ValueError):
            command.timed(0.1, 0, 0, 0.0, 0.0)


if __name__ == '__main__':
    unittest.main()
