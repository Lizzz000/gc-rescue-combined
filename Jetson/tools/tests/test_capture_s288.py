# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""冻结图像整链验收所用的 S288 伪终端回归。"""
import json
import os
from pathlib import Path
import select
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from validation.evidence import ContractFailure
from validation.s288_probe import S288Probe, S288Responder, protocol


def control(motor_id=2, mode=protocol.MODE_STOP, position=0):
    return protocol.control_packet(motor_id, mode, int(mode == protocol.MODE_RUN),
                                   0, 0, position, 0, 0)


class CaptureS288Test(unittest.TestCase):
    def test_split_frames_keep_position_after_stop(self):
        responder = S288Responder(2)
        run = control(mode=protocol.MODE_RUN, position=-0.25)
        self.assertEqual(responder.feed(b'noise' + run[:1]), b'')
        self.assertEqual(responder.feed(run[1:7]), b'')
        replies = responder.feed(run[7:] + control())
        middle = protocol.FEEDBACK_BYTES
        moving = protocol.parse_feedback(replies[:middle], 2)
        stopped = protocol.parse_feedback(replies[middle:], 2)
        self.assertEqual((moving['mode'], stopped['mode']), (protocol.MODE_RUN, protocol.MODE_STOP))
        self.assertAlmostEqual(moving['position'], -0.25, places=5)
        self.assertEqual(stopped['position'], moving['position'])
        self.assertEqual(stopped['error'], 0)
        self.assertFalse(stopped['timeout'])
        self.assertEqual(responder.commands, {protocol.MODE_STOP: 1, protocol.MODE_RUN: 1})

    def test_bad_crc_id_and_mode_fail(self):
        packet = bytearray(control())
        packet[-1] ^= 1
        with self.assertRaisesRegex(ContractFailure, 'control CRC'):
            S288Responder(2).feed(packet)
        with self.assertRaisesRegex(ContractFailure, 'motor ID'):
            S288Responder(2).feed(control(motor_id=3))
        packet = bytearray(control())
        packet[2] |= 2 << 4
        packet[-4:] = protocol.crc32(packet[:-4]).to_bytes(4, 'little')
        with self.assertRaisesRegex(ContractFailure, 'control mode'):
            S288Responder(2).feed(packet)

    @unittest.skipUnless(hasattr(os, 'openpty'), 'POSIX pseudo-terminal required')
    def test_pseudo_terminal_delivers_feedback_and_stops(self):
        with tempfile.TemporaryDirectory() as name:
            probe = S288Probe(Path(name), 2)
            try:
                with open(probe.device, 'r+b', buffering=0) as device:
                    device.write(control())
                    ready, _, _ = select.select([device], [], [], 1.0)
                    self.assertTrue(ready, 'no S288 pseudo-terminal response')
                    packet = device.read(protocol.FEEDBACK_BYTES)
                    self.assertEqual(protocol.parse_feedback(packet, 2)['mode'], protocol.MODE_STOP)
            finally:
                probe.close()
            receipt = json.loads((Path(name) / 's288-pty.json').read_text(encoding='utf-8'))
            self.assertEqual((receipt['stop_commands'], receipt['run_commands']), (1, 0))
            self.assertTrue(receipt['thread_stopped'])
            self.assertIsNone(receipt['error'])

    @unittest.skipUnless(hasattr(os, 'openpty'), 'POSIX pseudo-terminal required')
    def test_pseudo_terminal_failure_survives_cleanup(self):
        with tempfile.TemporaryDirectory() as name:
            probe = S288Probe(Path(name), 2)
            try:
                with open(probe.device, 'wb', buffering=0) as device:
                    device.write(control(motor_id=3))
                probe.thread.join(timeout=1.0)
                self.assertFalse(probe.thread.is_alive())
            finally:
                with self.assertRaisesRegex(ContractFailure, 'responder failed: unexpected S288 control motor ID'):
                    probe.close()
            receipt = json.loads((Path(name) / 's288-pty.json').read_text(encoding='utf-8'))
            self.assertEqual(receipt['error'], 'unexpected S288 control motor ID')
            self.assertTrue(receipt['thread_stopped'])


if __name__ == '__main__':
    unittest.main()
