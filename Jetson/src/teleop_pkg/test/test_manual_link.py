"""手动链路验收（pty 代替达妙串口，不需要硬件）：

- 0x17F 封装与边界、限幅与 STM32 头文件一致；
- 100 Hz 周期：中位 10±0.5 ms，p99 ≤ 15 ms；
- 租约 ≤ 0.2 s，过期先零速再 valid=0；
- 进程被 SIGKILL 后，把线上实际帧回放进 STM32 真实 manual_control.c，判据为在
  “超时 + 一个任务周期”内进入 BRAKING（进入制动，不是停稳）；常量从 STM32 源码读取；
- HTTP 接口的错误输入全部 4xx 且不产生运动帧；
- 第二个打开者失败且不写任何字节。
"""

import importlib.util
import json
import math
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import wire  # noqa: E402
from manual import command as manual_command  # noqa: E402
from manual import link as manual_link  # noqa: E402

PROGRAM = wire.TELEOP / 'manual_control.py'

# 判据（单位 ms / s）；频率取自 manual_control.DEFAULT_RATE_HZ
PERIOD_MEDIAN_TOL_MS = 0.5
PERIOD_P99_MAX_MS = 15.0
RATE_SAMPLE_S = 3.0
HOLD_REPOST_S = 0.05          # 网页按住时的续租间隔
CAPTURE_LATENCY_S = 0.02      # pty 抓包时间戳相对写入的最大滞后
LEASE_LIMIT_S = 0.2
SIGKILL_HOLD_S = 0.6
SETTLE_S = 0.6
LEASE_OBSERVE_S = 1.0       # 单次 hold 后观察：使能零速 + 租约 + 停车零速 + 若干 valid=0
HTTP_TIMEOUT_S = 2.0
OVERSIZE_BODY = 4096


def _manual_control_module():
    spec = importlib.util.spec_from_file_location('manual_control_entry', PROGRAM)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ENTRY = _manual_control_module()
PERIOD_MS = 1000.0 / ENTRY.DEFAULT_RATE_HZ


def frames_of(capture):
    return capture.frames(manual_link.MANUAL_ID, manual_link.MANUAL_DLC)


class ConstantsTest(unittest.TestCase):
    """上位机常量与 STM32 手动固件一致。"""

    @unittest.skipUnless(wire.stm32_available(), 'STM32 源码不在仓库中')
    def test_limits_and_id_match_stm32(self):
        c = wire.stm32_constants()
        self.assertEqual(manual_link.MANUAL_ID, c['manual_id'])
        self.assertAlmostEqual(manual_command.MAX_LINEAR_MPS, c['max_linear_mps'])
        self.assertAlmostEqual(manual_command.MAX_ANGULAR_RADPS, c['max_angular_radps'])
        self.assertLessEqual(manual_command.MAX_LEASE_S, LEASE_LIMIT_S)
        self.assertLess(manual_command.MAX_LEASE_S * 1000.0, c['timeout_ms'])

    def test_packet_int16_boundaries(self):
        for values in ((1, 32767, -32768, 0), (0, 0, 0, 0), (1, -1, 1, -32768)):
            packet = manual_link.manual_packet(*values)
            self.assertEqual(wire.decode_packet(packet, manual_link.MANUAL_ID, manual_link.MANUAL_DLC), values)
        with self.assertRaises(struct.error):
            manual_link.manual_packet(1, 32768, 0, 0)

    def test_quantized_limits_never_trip_stm32(self):
        """量化值按 STM32 的 /1000 还原后不超过阈值（STM32 超阈值即制动）。"""
        lin, ang = manual_command.MAX_LINEAR_MPS, manual_command.MAX_ANGULAR_RADPS
        cases = [(lin, 0, 0), (lin * 1.000001, 0, 0), (lin, lin, 0), (-lin, 0, ang), (0, 0, -ang * 1.5),
                 (0.3, -0.4, 0), (1e9, 1e9, 1e9)]
        for vx, vy, wz in cases:
            qx, qy, qz = manual_command.quantize(vx, vy, wz)
            self.assertLessEqual(math.hypot(qx / 1000.0, qy / 1000.0), lin + 1e-9, (vx, vy, wz))
            self.assertLessEqual(abs(qz) / 1000.0, ang + 1e-9, (vx, vy, wz))
        self.assertEqual(manual_command.quantize(lin, 0, ang), (round(lin * 1000), 0, round(ang * 1000)))


class LinkExclusiveTest(unittest.TestCase):
    def test_second_opener_fails_without_write(self):
        capture = wire.PtyCapture()
        try:
            first = manual_link.DamiaoLink(capture.device)
            # TIOCEXCL 使后打开者在 open() 即 EBUSY；flock 冲突为 LinkError，两者都是 OSError
            with self.assertRaises(OSError):
                manual_link.DamiaoLink(capture.device)
            time.sleep(0.1)
            self.assertEqual(capture.packets, [])
            first.send(manual_link.manual_packet(0, 0, 0, 0))
            time.sleep(0.1)
            self.assertEqual(len(capture.packets), 1)
            first.close()
        finally:
            capture.close()


class ProcessTestBase(unittest.TestCase):
    """启动 manual_control.py，pty 抓线上帧，HTTP 走 127.0.0.1 随机端口。"""

    def setUp(self):
        self.capture = wire.PtyCapture()
        self.temp = tempfile.TemporaryDirectory()
        config = Path(self.temp.name) / 'manual.yaml'
        config.write_text(f'damiao:\n  device: {self.capture.device}\n')
        self.process = subprocess.Popen(
            [sys.executable, str(PROGRAM), '--config', str(config), '--port', '0'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        line = self.process.stdout.readline()
        self.assertIn('listening http://127.0.0.1:', line, self.process.stderr.read() if not line else line)
        self.url = line.split()[3].rstrip('/')

    def tearDown(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=5)
        self.process.stdout.close()
        self.process.stderr.close()
        self.temp.cleanup()
        self.capture.close()

    def request(self, path, body=None, raw=None, method='POST'):
        data = raw if raw is not None else (json.dumps(body or {}).encode() if method == 'POST' else None)
        req = urllib.request.Request(self.url + path, data, {'Content-Type': 'application/json'}, method=method)
        try:
            with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()

    def hold_for(self, seconds, velocity):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.request('/api/hold', dict(zip(('vx', 'vy', 'wz'), velocity)))
            time.sleep(HOLD_REPOST_S)

    def stop_cleanly(self):
        self.process.send_signal(signal.SIGINT)
        self.assertEqual(self.process.wait(timeout=5), 0, self.process.stderr.read())
        time.sleep(0.1)
        return frames_of(self.capture)


class RateTest(ProcessTestBase):
    def test_period_median_and_p99(self):
        # 用定时动作而不是 50 ms 续租：测试进程少发请求，抓包线程的时间戳更接近真实写入
        self.request('/api/timed', {'vx': 0.1, 'vy': 0, 'wz': 0, 'seconds': RATE_SAMPLE_S})
        time.sleep(RATE_SAMPLE_S)
        frames = self.stop_cleanly()
        median, p99, worst, n = wire.period_stats(frames)
        print(f'[周期] 样本 {n}，中位 {median:.2f} ms，p99 {p99:.2f} ms，最大 {worst:.2f} ms（目标 {PERIOD_MS:.1f} ms）')
        self.assertLessEqual(abs(median - PERIOD_MS), PERIOD_MEDIAN_TOL_MS)
        self.assertLessEqual(p99, PERIOD_P99_MAX_MS)


class LeaseTest(ProcessTestBase):
    def test_single_hold_expires_within_lease_zero_then_invalid(self):
        time.sleep(0.2)
        sent = time.monotonic()
        self.request('/api/hold', {'vx': 0.1, 'vy': 0, 'wz': 0})
        time.sleep(LEASE_OBSERVE_S)
        stopping = time.monotonic()
        # 只看进程退出序列之前的帧：退出序列本身会再发零速 valid=1
        frames = [f for f in self.stop_cleanly() if sent <= f.t < stopping]
        moving = [f for f in frames if f.valid and any(f.velocity)]
        self.assertTrue(moving, '租约内没有运动帧（使能零速长于租约？）')
        stop = next(f for f in frames if f.t > moving[-1].t)
        lease_s = stop.t - sent
        print(f'[租约] 单次 hold 到首个停车帧 {lease_s * 1000:.1f} ms（上限 {manual_command.MAX_LEASE_S * 1000:.0f} ms + 2 周期）')
        self.assertLessEqual(lease_s, manual_command.MAX_LEASE_S + 2 * PERIOD_MS / 1000.0 + CAPTURE_LATENCY_S)
        self.assertEqual((stop.valid, *stop.velocity), (1, 0, 0, 0))
        after = [f for f in frames if f.t >= stop.t]
        first_invalid = next(i for i, f in enumerate(after) if not f.valid)
        self.assertTrue(all(f.valid and not any(f.velocity) for f in after[:first_invalid]))
        self.assertTrue(all(not f.valid for f in after[first_invalid:]))
        zero_s = after[first_invalid].t - stop.t
        self.assertGreaterEqual(zero_s + PERIOD_MS / 1000.0, manual_command.STOP_ZERO_S)


class SigkillTest(ProcessTestBase):
    @unittest.skipUnless(wire.stm32_available(), 'STM32 源码不在仓库中')
    def test_sigkill_enters_braking_within_timeout(self):
        replay = wire.build_replay()
        if replay is None:
            self.skipTest('没有主机 gcc，无法编译 STM32 回放')
        constants = wire.stm32_constants()
        origin = time.monotonic()
        self.hold_for(SIGKILL_HOLD_S, (0.1, 0.0, 0.0))
        self.process.send_signal(signal.SIGKILL)
        killed = time.monotonic()
        self.process.wait(timeout=5)
        time.sleep(SETTLE_S)
        frames = frames_of(self.capture)
        self.assertTrue(all(f.t <= killed + CAPTURE_LATENCY_S for f in frames), '进程被杀后线上仍有帧')
        end = killed - origin + SETTLE_S
        states = wire.replay(replay, frames, origin, end, constants['task_period_ms'])
        self.assertIn('RUNNING', [s for _, s in states], states)
        braking = next((t for t, s in states if s == 'BRAKING' and t >= frames[-1].t - origin), None)
        self.assertIsNotNone(braking, states)
        latency_ms = (braking - (killed - origin)) * 1000.0
        limit_ms = constants['timeout_ms'] + constants['task_period_ms']
        print(f'[SIGKILL] 末帧→杀进程 {(killed - frames[-1].t) * 1000:.1f} ms，杀进程→STM32 进入 BRAKING '
              f'{latency_ms:.1f} ms（上限 超时 {constants["timeout_ms"]:.0f} + 周期 {constants["task_period_ms"]:.0f} ms）')
        self.assertLessEqual(latency_ms, limit_ms)


class HttpTest(ProcessTestBase):
    def test_bad_requests_rejected_without_motion(self):
        bad = [
            ('/api/hold', None, b'not json'),
            ('/api/hold', None, b'[1, 2, 3]'),
            ('/api/hold', {'vx': True, 'vy': 0, 'wz': 0}, None),
            ('/api/hold', {'vx': 'fast', 'vy': 0, 'wz': 0}, None),
            ('/api/hold', {'vx': 0.1, 'vy': 0}, None),
            ('/api/hold', None, b'{"vx": NaN, "vy": 0, "wz": 0}'),
            ('/api/hold', None, b'{"vx": Infinity, "vy": 0, "wz": 0}'),
            ('/api/timed', {'vx': 0.1, 'vy': 0, 'wz': 0, 'seconds': 0}, None),
            ('/api/timed', {'vx': 0.1, 'vy': 0, 'wz': 0, 'seconds': manual_command.MAX_TIMED_S + 1}, None),
            ('/api/hold', None, b'{' + b' ' * OVERSIZE_BODY + b'}'),
        ]
        for path, body, raw in bad:
            status, _ = self.request(path, body, raw)
            self.assertEqual(status, 400, (path, body, raw[:40] if raw else None))
        self.assertEqual(self.request('/api/nothing')[0], 404)
        self.assertEqual(self.request('/api/nothing', method='GET')[0], 404)
        status, page = self.request('/', method='GET')
        self.assertEqual(status, 200)
        self.assertIn(b'<html', page.lower())
        status, state = self.request('/api/state', method='GET')
        limits = json.loads(state)['limits']
        self.assertEqual(limits, {'linear_mps': manual_command.MAX_LINEAR_MPS,
                                  'angular_radps': manual_command.MAX_ANGULAR_RADPS})
        stopping = time.monotonic()
        frames = self.stop_cleanly()
        before = [f for f in frames if f.t < stopping]
        self.assertTrue(before)
        # 退出序列本身会发零速 valid=1，只检查退出前全为 valid=0、全程无非零速度
        self.assertTrue(all(not f.valid for f in before), '错误请求产生了 valid=1 帧')
        self.assertTrue(all(not any(f.velocity) for f in frames), '错误请求产生了运动帧')


if __name__ == '__main__':
    unittest.main()
