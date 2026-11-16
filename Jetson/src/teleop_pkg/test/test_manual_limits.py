"""Configurable manual bounds, including direct/backend and HTTP input bypasses."""
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from manual.command import ManualCommand
from web.server import ManualServer


def low_command():
    return ManualCommand(0.2, 0.1, max_linear_mps=0.25, max_angular_radps=0.5, max_timed_s=1.5)


class BackendLimitsTest(unittest.TestCase):
    def test_diagonal_and_rotation_clip_at_instance_limits(self):
        command = low_command()
        command.hold(1, 1, -9, 0)
        valid, vx, vy, wz = command.frame(0.1)
        self.assertTrue(valid)
        self.assertEqual(vx, vy)
        self.assertLessEqual(math.hypot(vx, vy), 250)
        self.assertGreater(math.hypot(vx, vy), 248)
        self.assertEqual(wz, -500)
        self.assertEqual(command.limits, {'linear_mps': 0.25, 'angular_radps': 0.5, 'timed_s': 1.5})
        self.assertEqual(ManualCommand(0.2, 0.1).limits,
                         {'linear_mps': 0.5, 'angular_radps': 2.0, 'timed_s': 3600.0})

    def test_timed_upper_boundary_excludes_prepare_then_stops(self):
        command = low_command()
        command.timed(-0.5, 0, 0, 1.5, 0)
        self.assertEqual(command.frame(0.099), (True, 0, 0, 0))
        self.assertEqual(command.frame(1.599), (True, -250, 0, 0))
        self.assertEqual(command.frame(1.6), (True, 0, 0, 0))

    def test_invalid_limits_rejected_without_relaxing_protocol(self):
        for key, above in (('max_linear_mps', 0.5001), ('max_angular_radps', 2.001), ('max_timed_s', 3601)):
            for value in (0, -1, above, float('nan'), float('inf'), True, False):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    ManualCommand(0.2, 0.1, **{key: value})

    def test_invalid_motion_values_do_not_change_pending_command(self):
        command = low_command()
        command.timed(0.1, 0, 0, 1, 0)
        initial = command.snapshot(0)
        for value in (float('nan'), float('inf'), -float('inf'), True, False, 10 ** 400):
            for axis in range(3):
                velocity = [0, 0, 0]
                velocity[axis] = value
                with self.subTest(value=value, axis=axis), self.assertRaises(ValueError):
                    command.hold(*velocity, 0)
                self.assertEqual(command.snapshot(0), initial)
        for seconds in (0, -1, 1.5001, float('nan'), float('inf'), True, False):
            with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                command.timed(0.25, 0, 0, seconds, 0)
            self.assertEqual(command.snapshot(0), initial)

class CliLimitsTest(unittest.TestCase):
    def test_cli_accepts_low_limits_and_rejects_nonfinite_or_above_protocol(self):
        import manual_control
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / 'manual.yaml'
            config.write_text('link:\n  device: /dev/unused-test-device\n', encoding='utf-8')
            base = ['--config', str(config)]
            args = manual_control.parse_arguments(base + ['--max-linear-mps', '0.25',
                '--max-angular-radps', '0.5', '--max-timed-s', '1.5'])
            self.assertEqual((args.max_linear_mps, args.max_angular_radps, args.max_timed_s), (0.25, 0.5, 1.5))
            for flag, above in (('--max-linear-mps', '0.501'), ('--max-angular-radps', '2.01'),
                                ('--max-timed-s', '3601')):
                for value in ('nan', 'inf', 'true', '0', above):
                    with self.subTest(flag=flag, value=value), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                        manual_control.parse_arguments(base + [flag, value])


class HttpLimitsTest(unittest.TestCase):
    def setUp(self):
        self.command = low_command()
        servo = SimpleNamespace(snapshot=lambda: {})
        self.server = ManualServer(0, self.command, lambda: {'device': 'test'}, servo)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def request(self, path, body=None):
        request = urllib.request.Request(self.url + path, data=json.dumps(body).encode() if body is not None else None,
                                         headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=2) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read())

    def test_api_reports_and_enforces_low_limits(self):
        status, state = self.request('/api/state')
        self.assertEqual(status, 200)
        self.assertEqual(state['limits'], {'linear_mps': 0.25, 'angular_radps': 0.5, 'timed_s': 1.5})
        status, state = self.request('/api/hold', {'vx': 1, 'vy': -1, 'wz': 4})
        self.assertEqual(status, 200)
        vx, vy, wz = state['velocity_milli']
        self.assertLessEqual(math.hypot(vx, vy), 250)
        self.assertEqual(wz, 500)
        status, _ = self.request('/api/timed', {'vx': 0.25, 'vy': 0, 'wz': 0, 'seconds': 1.5001})
        self.assertEqual(status, 400)

    def test_invalid_json_numbers_never_start_motion(self):
        for value in (float('nan'), float('inf'), True, False, 10 ** 400):
            for path, field in (('/api/hold', 'vx'), ('/api/timed', 'seconds')):
                body = {'vx': 0.25, 'vy': 0, 'wz': 0, 'seconds': 0.8}
                body[field] = value
                with self.subTest(path=path, value=value):
                    status, _ = self.request(path, body)
                    self.assertEqual(status, 400)
                    self.assertEqual(self.command.frame(0), (False, 0, 0, 0))


if __name__ == '__main__':
    unittest.main()
