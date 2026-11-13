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

    def goto_degrees(self, target_deg, seconds, kp, kd, cancel, zero_rad):
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

    def test_official_yaml_empty_move_s_still_loads(self):
        # 官方 yaml 的 move_s 为空只约束自动路径；手动入口忽略该键。
        path = Path(__file__).resolve().parents[2] / 'bringup_pkg/config/rescue.yaml'
        config = manual_control.load_manual_config(path)
        self.assertTrue(config['gripper_device'].startswith('/dev/'))

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
    def setUp(self):
        self.command = ManualCommand(0.2, 0.1)
        self.session = session_with_link(open_deg=40, closed_deg=-40)
        self.server = ManualServer(0, self.command, lambda: {'device': 'test', 'frames': 0,
                                                            'last_frame': [0, 0, 0, 0]}, self.session)
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
    def test_open_close_empty_body_is_400(self):
        # 空请求体 {} 须返回 400，不能断连接。
        status, payload = self.request('/api/servo/open', {})
        self.assertEqual(status, 400)
        self.assertIn('seconds', payload['error'])
        status, payload = self.request('/api/servo/close', {})
        self.assertEqual(status, 400)
        self.assertIn('seconds', payload['error'])


if __name__ == '__main__':
    unittest.main()
