import fcntl
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import time
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'move_timed.py'
STOP_SOCKET = '/tmp/gc_rescue_motion_stop.sock'


class TimedMotionTest(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.device = os.ttyname(self.slave)

    def tearDown(self):
        os.close(self.slave)
        os.close(self.master)

    def start(self, *args, input_text=None):
        process = subprocess.Popen(
            [sys.executable, str(SCRIPT), '--device', self.device, *args],
            stdin=subprocess.PIPE if input_text is not None else subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if input_text is not None:
            process.stdin.write(input_text)
            process.stdin.close()
        return process

    def collect(self, process):
        wire = bytearray()
        deadline = time.monotonic() + 10
        while process.poll() is None and time.monotonic() < deadline:
            if select.select([self.master], [], [], 0.1)[0]:
                wire.extend(os.read(self.master, 8192))
        if process.poll() is None:
            process.kill()
            self.fail('脚本未按时结束')
        while select.select([self.master], [], [], 0)[0]:
            wire.extend(os.read(self.master, 8192))
        stdout = process.stdout.read()
        stderr = process.stderr.read()
        process.stdout.close()
        process.stderr.close()
        self.assertEqual(process.returncode, 0, stderr)
        self.assertEqual(len(wire) % 30, 0)
        packets = [wire[i:i + 30] for i in range(0, len(wire), 30)]
        for packet in packets:
            self.assertEqual(packet[:4], b'\x55\xaa\x1e\x01')
            self.assertEqual(int.from_bytes(packet[13:17], 'little'), 0x17f)
        return packets, stdout

    def test_menu_executes_and_stops(self):
        process = self.start(input_text='7\n0.08\n0.2\n0.3\n1\nq\n')
        packets, output = self.collect(process)
        movement = [frame for frame in packets if frame[21] == 1 and frame[22:24] == b'\x50\x00']
        self.assertGreaterEqual(len(movement), 20)
        self.assertLessEqual(len(movement), 35)
        self.assertEqual(packets[-1][21], 0)
        self.assertIn('前进', output)

    def test_stop_interrupts_motion(self):
        process = self.start('forward', '0.08', '8')
        deadline = time.monotonic() + 3
        while not os.path.exists(STOP_SOCKET) and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(os.path.exists(STOP_SOCKET))
        time.sleep(0.8)
        stop = subprocess.run([sys.executable, str(SCRIPT), 'stop'], capture_output=True, timeout=5)
        self.assertEqual(stop.returncode, 0, stop.stderr)
        packets, _ = self.collect(process)
        movement = [frame for frame in packets if frame[21] == 1 and frame[22:24] == b'\x50\x00']
        self.assertGreater(len(movement), 0)
        self.assertLess(len(movement), 300)
        self.assertEqual(packets[-1][21], 0)

    def test_busy_device_is_not_written(self):
        fcntl.flock(self.slave, fcntl.LOCK_EX | fcntl.LOCK_NB)
        process = self.start('forward', '0.08', '1')
        stdout, stderr = process.communicate(timeout=5)
        self.assertNotEqual(process.returncode, 0, stdout)
        self.assertIn('USB-CAN', stderr)
        self.assertFalse(select.select([self.master], [], [], 0.2)[0])


if __name__ == '__main__':
    unittest.main()
