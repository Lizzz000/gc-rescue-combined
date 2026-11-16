"""真实 main/停车序列/舵机清理回归；串口与 HTTP 边界替身，不连接设备。"""

from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import manual_control  # noqa: E402
from manual.frame import Parser  # noqa: E402
from manual.link import LinkError, MANUAL_ID  # noqa: E402
from servo.s288 import ServoSession  # noqa: E402


class ShutdownTest(unittest.TestCase):
    def run_main(self, *, fail_send=False, fail_close=False, fail_run=False):
        events = []
        packets = []

        def send(packet):
            events.append('send')
            if fail_send:
                raise LinkError('injected shutdown write failure')
            packets.extend(Parser().feed(packet))

        def close_link():
            events.append('link close')
            if fail_close:
                raise OSError('injected close failure after disconnect')

        link = Mock(send=send, close=close_link)
        servo = ServoSession('', 0, 0.0, 0.0)
        servo._link = Mock(stop=lambda: events.append('servo stop'),
                           close=lambda: events.append('servo close'))
        server = Mock(server_address=('127.0.0.1', 0),
                      shutdown=lambda: events.append('http stop'),
                      server_close=lambda: events.append('http close'))
        args = SimpleNamespace(lease_s=0.2, arm_zero_s=0.1, rate_hz=100, port=0,
            max_linear_mps=0.5, max_angular_radps=2.0, max_timed_s=3600.0,
            manual_config=dict(link_device='/fake', gripper_device='', gripper_id=0,
                               kp=0, kd=0, closed_deg=None, open_deg=None))
        clock = [0.0]

        def sleep(seconds):
            clock[0] += seconds

        stdout, stderr = StringIO(), StringIO()
        with patch.object(manual_control, 'parse_arguments', return_value=args), \
             patch.object(manual_control, 'CdcLink', return_value=link), \
             patch.object(manual_control, 'ServoSession', return_value=servo), \
             patch.object(manual_control, 'ManualServer', return_value=server), \
             patch.object(manual_control.Sender, 'run',
                          side_effect=LinkError('injected run failure') if fail_run else None), \
             patch.object(manual_control.signal, 'signal'), \
             patch.object(manual_control.threading, 'Thread'), \
             patch.object(manual_control.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(manual_control.time, 'sleep', side_effect=sleep), \
             redirect_stdout(stdout), redirect_stderr(stderr):
            code = manual_control.main([])

        self.assertEqual(events[-5:], ['link close', 'servo stop', 'servo close',
                                      'http stop', 'http close'])
        self.assertTrue(servo._cancel.is_set())
        self.assertIsNone(servo._link)
        return code, packets, stdout.getvalue(), stderr.getvalue()

    def test_normal_exit_sends_zero_then_invalid_before_cleanup(self):
        code, packets, stdout, stderr = self.run_main()
        self.assertEqual(code, 0)
        self.assertEqual(stderr, '')
        self.assertIn('已停车并退出', stdout)
        self.assertTrue(packets)
        self.assertTrue(all(kind == MANUAL_ID and payload[1:] == bytes(6)
                            for kind, payload in packets))
        valid = [payload[0] for _, payload in packets]
        self.assertIn(1, valid)
        self.assertIn(0, valid)
        first_invalid = valid.index(0)
        self.assertEqual(valid[:first_invalid], [1] * first_invalid)
        self.assertEqual(valid[first_invalid:], [0] * (len(valid) - first_invalid))

    def test_shutdown_write_error_cleans_up_and_returns_failure(self):
        code, _, stdout, stderr = self.run_main(fail_send=True)
        self.assertNotEqual(code, 0)
        self.assertIn('injected shutdown write failure', stderr)
        self.assertNotIn('已停车并退出', stdout)

    def test_link_close_error_still_closes_servo_and_http(self):
        for fail_send in (False, True):
            with self.subTest(fail_send=fail_send):
                code, _, stdout, stderr = self.run_main(fail_send=fail_send, fail_close=True)
                self.assertNotEqual(code, 0)
                if fail_send:
                    self.assertIn('injected shutdown write failure', stderr)
                self.assertIn('injected close failure', stderr)
                self.assertNotIn('已停车并退出', stdout)

    def test_run_error_skips_shutdown_writes_but_cleans_up(self):
        code, packets, stdout, stderr = self.run_main(fail_run=True)
        self.assertNotEqual(code, 0)
        self.assertEqual(packets, [])
        self.assertIn('injected run failure', stderr)
        self.assertNotIn('已停车并退出', stdout)


if __name__ == '__main__':
    unittest.main()
