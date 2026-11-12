"""manual_control.py 端到端：pty 代替达妙串口，HTTP 走 127.0.0.1 随机端口。"""

import fcntl
import json
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

PROGRAM = Path(__file__).resolve().parents[1] / 'manual_control.py'
PACKET = 30


def write_config(folder, device):
    path = Path(folder) / 'manual.yaml'
    path.write_text(f'damiao:\n  device: {device}\n')
    return path



def decode(wire):
    """切成 30 字节封装，返回 (单调时间, valid, vx, vy, wz) 列表；帧头、ID 或 DLC 不符抛 ValueError。"""
    frames = []
    for stamp, chunk in wire:
        for i in range(0, len(chunk), PACKET):
            packet = chunk[i:i + PACKET]
            if (packet[:5] != bytes([0x55, 0xAA, 30, 1, 1]) or packet[18] != 7
                    or int.from_bytes(packet[13:17], 'little') != 0x17F):
                raise ValueError(f'unexpected packet {packet.hex()}')
            frames.append((stamp, *struct.unpack('<Bhhh', packet[21:28])))
    return frames


class ManualControlTest(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.device = os.ttyname(self.slave)
        self.wire, self.buffer = [], bytearray()
        self.reading = True
        self.temp = tempfile.TemporaryDirectory()
        self.config = write_config(self.temp.name, self.device)

    def tearDown(self):
        self.reading = False
        time.sleep(0.05)
        os.close(self.slave)
        os.close(self.master)
        self.temp.cleanup()

    def start(self):
        self.process = subprocess.Popen(
            [sys.executable, str(PROGRAM), '--config', str(self.config), '--port', '0'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        threading.Thread(target=self.read, daemon=True).start()
        line = self.process.stdout.readline()
        self.assertIn('listening http://127.0.0.1:', line)
        self.url = line.split()[3].rstrip('/')

    def read(self):
        while self.reading:
            if select.select([self.master], [], [], 0.02)[0]:
                try:
                    self.buffer.extend(os.read(self.master, 8192))
                except OSError:
                    return
                whole = len(self.buffer) // PACKET * PACKET
                if whole:
                    self.wire.append((time.monotonic(), bytes(self.buffer[:whole])))
                    del self.buffer[:whole]

    def post(self, path, body=None):
        request = urllib.request.Request(self.url + path, json.dumps(body or {}).encode(),
                                         {'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.loads(response.read())

    def finish(self):
        self.process.send_signal(signal.SIGINT)
        self.assertEqual(self.process.wait(timeout=5), 0, self.process.stderr.read())
        time.sleep(0.1)
        self.process.stdout.close()
        self.process.stderr.close()
        return decode(self.wire)

    def test_hold_lease_expiry_sends_zero_then_invalid(self):
        self.start()
        time.sleep(0.2)
        started = time.monotonic()
        while time.monotonic() - started < 1.0:
            self.post('/api/hold', {'vx': 0.08, 'vy': 0, 'wz': 0})
            time.sleep(0.05)
        released = time.monotonic()
        time.sleep(0.8)
        frames = self.finish()
        moving = [f for f in frames if f[1] == 1 and f[2] == 80]
        self.assertGreater(len(moving), 30)
        last_move = moving[-1][0]
        self.assertLess(last_move - released, 0.25)
        after = [f for f in frames if f[0] > last_move + 0.02]
        zero = [f for f in after if f[1:] == (1, 0, 0, 0)]
        invalid = [f for f in after if f[1] == 0]
        self.assertGreater(len(zero), 20)
        self.assertGreater(len(invalid), 20)
        self.assertLess(zero[19][0], invalid[0][0])
        self.assertEqual(frames[-1][1], 0)

    def test_rate_limits_and_api(self):
        self.start()
        state = self.post('/api/hold', {'vx': 2.0, 'vy': 0, 'wz': -9})
        self.assertEqual(state['phase'], 'prepare')
        self.assertEqual(state['limits'], {'linear_mps': 0.5, 'angular_radps': 2.0})
        with self.assertRaises(urllib.error.HTTPError):
            self.post('/api/hold', {'vx': 'fast', 'vy': 0, 'wz': 0})
        self.post('/api/timed', {'vx': 0, 'vy': 0.1, 'wz': 0, 'seconds': 0.5})
        time.sleep(1.5)
        self.assertEqual(self.post('/api/stop')['phase'], 'idle')
        frames = self.finish()
        span = frames[-1][0] - frames[0][0]
        self.assertGreater(len(frames) / span, 80)
        self.assertTrue(any(f[1:] == (1, 0, 100, 0) for f in frames))
        for _, _, vx, vy, wz in frames:
            self.assertLessEqual((vx * vx + vy * vy) ** 0.5, 500)
            self.assertLessEqual(abs(wz), 2000)

    def test_busy_device_is_not_written(self):
        fcntl.flock(self.slave, fcntl.LOCK_EX | fcntl.LOCK_NB)
        process = subprocess.run([sys.executable, str(PROGRAM), '--config', str(self.config), '--port', '0'],
                                 capture_output=True, text=True, timeout=5)
        self.assertNotEqual(process.returncode, 0)
        self.assertIn('USB-CAN', process.stderr)
        self.assertFalse(select.select([self.master], [], [], 0.2)[0])


if __name__ == '__main__':
    unittest.main()
