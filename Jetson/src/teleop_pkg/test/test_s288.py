"""手动 S288 协议和转角会话。不打开真实串口，也不接触自动搬运。"""

import math
from pathlib import Path
import struct
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from servo.s288 import (  # noqa: E402
    FEEDBACK_BYTES, FEEDBACK_MARK, ServoSession, _take_feedback, check_move, control_packet, crc32)


class ProtocolTest(unittest.TestCase):
    def test_vendor_vector(self):
        packet = control_packet(2, 1, 1, 0.1, 0.2, 0.3, 1.0, 0.1)
        self.assertEqual(packet.hex(), 'feee9200590017004ae206000f009a005e72f7ee')

    def test_move_limits(self):
        check_move(30, 2)
        for degrees, seconds in ((0, 1), (361, 1), (10, 0.1), (10, 11), (float('nan'), 1)):
            with self.assertRaises(ValueError):
                check_move(degrees, seconds)
        with self.assertRaisesRegex(ValueError, '角度须在 -360 到 360 度之间，且不能为 0'):
            check_move(361, 1)
        with self.assertRaisesRegex(ValueError, '时长须在 0.2 到 10 秒之间'):
            check_move(10, 0.1)

    def test_take_feedback_one_attempt(self):
        body = struct.pack('<BbBBhhiIHBB', 0x02, 1, 2, 24, 0, 0, 0, 0, 0, 0, 0)
        packet = FEEDBACK_MARK + body + struct.pack('<I', crc32(body))
        pending = bytearray(b'\x00' + packet)
        feedback = _take_feedback(pending, 2)
        self.assertEqual(feedback['id'], 2)
        # 成功帧留在缓冲区里；调用方拿到结果后就不再读这块缓冲。
        self.assertEqual(bytes(pending), packet)
        corrupt = bytearray(packet)
        corrupt[8] ^= 1
        self.assertIsNone(_take_feedback(corrupt, 2))
        self.assertEqual(len(corrupt), FEEDBACK_BYTES - 1)
        noise = bytearray(b'\x11\x22')
        self.assertIsNone(_take_feedback(noise, 2))
        self.assertEqual(bytes(noise), b'\x22')


class FakeLink:
    def __init__(self):
        self.calls = []
        self.closed = False

    def move_degrees(self, degrees, seconds, kp, kd, cancel):
        self.calls.append(('move', degrees, seconds, kp, kd, cancel.is_set()))
        return {'position': 0.5, 'turned_deg': degrees}

    def stop(self):
        self.calls.append(('stop',))
        return {'position': 0.25}

    def close(self):
        self.closed = True


class SessionTest(unittest.TestCase):
    def test_without_device_rejects_motion(self):
        session = ServoSession('', 0, 1.0, 0.05)
        self.assertFalse(session.available)
        with self.assertRaises(RuntimeError):
            session.move(10, 1)
        self.assertIsNone(session.stop())

    def test_move_updates_position_and_does_not_touch_can(self):
        session = ServoSession('', 0, 1.0, 0.05)
        session._link = FakeLink()
        feedback = session.move(-15, 0.2)
        self.assertEqual(feedback['turned_deg'], -15)
        self.assertFalse(session.busy)
        self.assertAlmostEqual(session.snapshot()['position_deg'], math.degrees(0.5))
        self.assertEqual(session._link.calls[0][0], 'move')

    def test_second_move_rejected_while_busy(self):
        session = ServoSession('', 0, 1.0, 0.05)
        started = threading.Event()
        release = threading.Event()

        class BlockingLink(FakeLink):
            def move_degrees(self, degrees, seconds, kp, kd, cancel):
                started.set()
                release.wait(timeout=2)
                return super().move_degrees(degrees, seconds, kp, kd, cancel)

        session._link = BlockingLink()
        worker = threading.Thread(target=session.move, args=(20, 1))
        worker.start()
        self.assertTrue(started.wait(timeout=1))
        with self.assertRaises(RuntimeError):
            session.move(5, 1)
        release.set()
        worker.join(timeout=2)
        self.assertFalse(worker.is_alive())
