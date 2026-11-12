import math
import struct
import unittest
from unittest.mock import patch

import s288_debug
import s288_protocol


class FakePort:
    def __init__(self):
        self.mode = 0
        self.position = 0
        self.pending = b''
        self.commands = []

    def write(self, packet):
        self.commands.append(packet)
        mode = self.mode
        position = self.position
        self.mode = (packet[2] >> 4) & 7
        if self.mode == 1:
            self.position = struct.unpack_from('<i', packet, 8)[0]
        body = struct.pack('<BbBBhhiIHBB', mode << 4, 30, 35, 24, 0, 0, position, 0, 0, 0, 0)
        self.pending = s288_protocol.FEEDBACK_MARK + body + struct.pack('<I', s288_protocol.crc32(body))
        return len(packet)

    def read(self, count):
        # 模拟串口半包。
        result = self.pending[:min(count, 7)]
        self.pending = self.pending[len(result):]
        return result


class S288DebugTest(unittest.TestCase):
    def test_move_relative_and_confirm_stop_with_one_frame_delay(self):
        port = FakePort()
        with patch('builtins.print'):
            s288_debug.move(port, 0, 30, 0.05, 0.15, 0.5)
        positions = [struct.unpack_from('<i', command, 8)[0] for command in port.commands
                     if (command[2] >> 4) & 7 == 1]
        self.assertTrue(positions)
        self.assertAlmostEqual(positions[0] * 2 * math.pi / (s288_protocol._POSITION_SCALE * s288_protocol.RATIO),
                               math.pi / 6, places=4)
        self.assertEqual(port.mode, 0)
        self.assertGreaterEqual(sum(1 for command in port.commands[-3:] if command[2] == 0), 2)

    def test_incomplete_write_is_error(self):
        port = FakePort()
        port.write = lambda packet: 0
        with self.assertRaises(OSError):
            s288_debug.exchange(port, 0, bytes(20))

    def test_invalid_motion_is_rejected_before_sending(self):
        port = FakePort()
        with self.assertRaises(ValueError):
            s288_debug.move(port, 0, math.nan, 1, 0.15, 0.5)
        self.assertEqual(port.commands, [])


if __name__ == '__main__':
    unittest.main()
