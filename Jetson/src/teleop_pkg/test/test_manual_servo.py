"""手动舵机 HTTP、--config 与底盘停机互不打断。不打开真实串口。"""

import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manual.command import ManualCommand  # noqa: E402
from servo.s288 import ServoSession  # noqa: E402
from web.server import BIND_ADDRESS, ManualServer  # noqa: E402

import manual_control  # noqa: E402


class FakeLink:
    def __init__(self):
        self.calls = []
        self.started = threading.Event()
        self.release = threading.Event()
        self.block = False

    def move_degrees(self, degrees, seconds, kp, kd, cancel):
        self.calls.append(('move', degrees, seconds, cancel.is_set()))
        if self.block:
            self.started.set()
            self.release.wait(timeout=2)
        return {'position': math.radians(degrees), 'turned_deg': degrees}

    def goto_degrees(self, target_deg, seconds, kp, kd, cancel, zero_rad, *, ramp_fraction=0.0):
        self.calls.append(('goto', target_deg, seconds, cancel.is_set(), zero_rad))
        if self.block:
            self.started.set()
            self.release.wait(timeout=2)
        return {'position': zero_rad + math.radians(target_deg), 'turned_deg': target_deg,
                'target_deg': target_deg}

    def stop(self):
        self.calls.append(('stop',))
        return {'position': 0.0}

    def close(self):
        self.calls.append(('close',))


def session_with_link(**kwargs):
    session = ServoSession('', 0, kwargs.get('kp', 1.0), kwargs.get('kd', 0.05),
                           kwargs.get('closed_deg'), kwargs.get('open_deg'))
    session._link = FakeLink()
    session._zero_rad = 0.0
    session.message = '已连接，上电零点已记录'
    return session


class ManualConfigTest(unittest.TestCase):
    def test_dotted_path_matches_nested_and_rejects_conflicts(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'manual.yaml'
            path.write_text('link.device: /dev/first\n', encoding='utf-8')
            self.assertEqual(manual_control.load_manual_config(path)['link_device'], '/dev/first')
            path.write_text('link.device: /dev/first\nlink:\n  device: /dev/second\n', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, r'link\.device'):
                manual_control.load_manual_config(path)

    def test_load_split_files_and_ignore_other_sections(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            (folder / 'advanced.yaml').write_text(
                'vision:\n  detect:\n    conf_threshold_seg: 0.5\n'
                'gripper:\n  kp: 1.0\n  kd: 0.05\n  closed_deg: -40\n  open_deg: 40\n  move_s:\n')
            basic = folder / 'rescue.yaml'
            basic.write_text(
                'advanced_config: advanced.yaml\n'
                'bridge:\n  transport: cdc\n'
                'link:\n  device: /dev/serial/by-id/link\n'
                'gripper:\n  device: /dev/serial/by-id/servo\n  id: 2\n')
            config = manual_control.load_manual_config(basic)
        self.assertEqual(config['link_device'], '/dev/serial/by-id/link')
        self.assertEqual(config['gripper_device'], '/dev/serial/by-id/servo')
        self.assertEqual(config['gripper_id'], 2)
        self.assertEqual(config['kp'], 1.0)
        self.assertEqual(config['kd'], 0.05)
        self.assertEqual(config['closed_deg'], -40)
        self.assertEqual(config['open_deg'], 40)
        self.assertIsNone(config['move_s'])
        self.assertEqual(config['ramp_fraction'], 0.0)
        self.assertEqual(config['open_hold_max_nm'], 0.0)
        self.assertEqual(config['output_tolerance_deg'], 1.0)
        self.assertEqual(config['open_trim_deg'], 0.0)
        self.assertEqual(config['closed_trim_deg'], 0.0)

    def test_official_yaml_loads_gripper_positions(self):
        path = Path(__file__).resolve().parents[2] / 'bringup_pkg/config/rescue.yaml'
        config = manual_control.load_manual_config(path)
        self.assertTrue(config['gripper_device'].startswith('/dev/'))
        self.assertEqual(config['closed_deg'], -50.0)
        self.assertEqual(config['open_deg'], 0.0)
        self.assertEqual(config['move_s'], 1.0)
        self.assertEqual(config['ramp_fraction'], 0.8)

    def test_rejects_invalid_move_time_and_ramp(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'bad.yaml'
            for key, value in (('move_s', 'true'), ('move_s', '0.1'), ('move_s', '.inf'),
                               ('ramp_fraction', 'true'), ('ramp_fraction', '1.1'),
                               ('ramp_fraction', '-0.1'), ('ramp_fraction', '.nan'),
                               ('open_hold_max_nm', '-0.1'), ('open_hold_max_nm', '0.61'),
                               ('output_tolerance_deg', '0'), ('output_tolerance_deg', '180'),
                               ('open_trim_deg', '.nan'), ('closed_trim_deg', 'true')):
                with self.subTest(key=key, value=value):
                    path.write_text(f'gripper.{key}: {value}\n', encoding='utf-8')
                    with self.assertRaisesRegex(ValueError, f'gripper\\.{key}'):
                        manual_control.load_manual_config(path)
            path.write_text('gripper.open_deg: 359\ngripper.open_trim_deg: 2\n', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, r'gripper\.open_trim_deg'):
                manual_control.load_manual_config(path)

    def test_load_output_control_parameters(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'hold.yaml'
            path.write_text('gripper:\n  open_hold_max_nm: 0.25\n  output_tolerance_deg: 1.0\n'
                            '  open_trim_deg: 5.0\n  closed_trim_deg: 0.35\n', encoding='utf-8')
            config = manual_control.load_manual_config(path)
        self.assertEqual(config['open_hold_max_nm'], 0.25)
        self.assertEqual(config['output_tolerance_deg'], 1.0)
        self.assertEqual(config['open_trim_deg'], 5.0)
        self.assertEqual(config['closed_trim_deg'], 0.35)

    def test_config_required_and_old_flags_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'only.yaml'
            path.write_text('link:\n  device: /dev/serial/by-id/link\n')
            args = manual_control.parse_arguments(['--config', str(path)])
        self.assertEqual(args.manual_config['link_device'], '/dev/serial/by-id/link')
        with self.assertRaises(SystemExit):
            manual_control.parse_arguments(['--port', '0'])
        with self.assertRaises(SystemExit):
            manual_control.parse_arguments(['--device', '/tmp/pty'])
        with self.assertRaises(SystemExit):
            manual_control.parse_arguments(['--config', str(path), '--device', '/tmp/pty'])
        with self.assertRaises(SystemExit):
            manual_control.parse_arguments(['--servo-device', '/dev/ttyUSB0', '--config', str(path)])

    def test_gripper_device_requires_angles_at_load(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'bad.yaml'
            path.write_text(
                'link:\n  device: /dev/serial/by-id/link\n'
                'gripper:\n  device: /dev/serial/by-id/servo\n  id: 0\n  kp: 1.0\n  kd: 0.05\n')
            with self.assertRaises(ValueError):
                manual_control.load_manual_config(path)


class ManualServoApiTest(unittest.TestCase):
    preset_seconds = 1.0

    def setUp(self):
        self.command = ManualCommand(0.2, 0.1)
        self.session = session_with_link(open_deg=40, closed_deg=-40)
        self.server = ManualServer(0, self.command, lambda: {'device': 'test', 'frames': 0,
                                                            'last_frame': [0, 0, 0, 0]}, self.session,
                                   servo_move_s=self.preset_seconds)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f'http://{BIND_ADDRESS}:{self.server.server_address[1]}'

    def tearDown(self):
        self.session._link.release.set()
        self.session.close()
        self.server.shutdown()
        self.server.server_close()

    def request(self, path, body=None, method='POST'):
        data = json.dumps(body or {}).encode() if method == 'POST' else None
        request = urllib.request.Request(self.url + path, data=data,
                                         method=method, headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=2) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read())

    def test_page_uses_configured_open_close_time(self):
        with urllib.request.urlopen(self.url, timeout=2) as response:
            page = response.read()
        self.assertIn(b'id="servo_s" type="number" step="0.1" value="2"', page)
        self.assertIn(b'id="servo_preset_s" type="number" step="0.1" value="1.0"', page)

    def test_move_returns_202_and_busy_is_409(self):
        self.session._link.block = True
        started = time.monotonic()
        status, payload = self.request('/api/servo', {'degrees': 10, 'seconds': 0.2})
        self.assertEqual(status, 202)
        self.assertLess(time.monotonic() - started, 0.05)
        self.assertTrue(self.session._link.started.wait(timeout=1))
        status, payload = self.request('/api/servo', {'degrees': 5, 'seconds': 0.2})
        self.assertEqual(status, 409)
        self.assertIn('正在转动', payload['error'])
        hold_started = time.monotonic()
        status, payload = self.request('/api/hold', {'vx': 0.1, 'vy': 0, 'wz': 0})
        self.assertEqual(status, 200)
        self.assertLess(time.monotonic() - hold_started, 0.05)
        self.session._link.release.set()
        self.session.wait(timeout=2)

    def test_open_close_and_chassis_stop_do_not_cancel(self):
        self.session._link.block = True
        status, payload = self.request('/api/servo/open', {'seconds': 0.2})
        self.assertEqual(status, 202)
        self.assertTrue(self.session._link.started.wait(timeout=1))
        status, payload = self.request('/api/stop')
        self.assertEqual(status, 200)
        self.assertFalse(self.session._cancel.is_set())
        self.assertTrue(self.session.busy)
        self.session._link.release.set()
        self.session.wait(timeout=2)
        self.assertEqual(self.session._result['target_deg'], 40)
        self.session._link.block = False
        status, payload = self.request('/api/servo/close', {'seconds': 0.2})
        self.assertEqual(status, 202)
        self.session.wait(timeout=2)
        self.assertEqual(self.session._result['target_deg'], -40)
        self.assertEqual(self.session._link.calls[-1][2], 0.2)

    def test_open_close_empty_body_is_400(self):
        # 空请求体 {} 须返回 400，不能断连接。
        status, payload = self.request('/api/servo/open', {})
        self.assertEqual(status, 400)
        self.assertIn('seconds', payload['error'])
        status, payload = self.request('/api/servo/close', {})
        self.assertEqual(status, 400)
        self.assertIn('seconds', payload['error'])


class ManualServoLegacyPageTest(unittest.TestCase):
    def test_null_config_preserves_old_open_close_default(self):
        command = ManualCommand(0.2, 0.1)
        session = session_with_link()
        with ManualServer(0, command, lambda: {}, session) as server:
            self.assertIn(b'id="servo_preset_s" type="number" step="0.1" value="2.0"', server.page)


if __name__ == '__main__':
    unittest.main()
