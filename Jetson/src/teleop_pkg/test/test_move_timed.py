import fcntl
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import termios
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

    def test_key_interface_and_terminal_restore(self):
        screen, terminal = pty.openpty()
        original = termios.tcgetattr(terminal)
        env = dict(os.environ, TERM='xterm')
        process = subprocess.Popen([sys.executable, str(SCRIPT), '--device', self.device],
                                   stdin=terminal, stdout=terminal, stderr=terminal, env=env)
        output = bytearray()

        def wait_for(text, timeout=5):
            target = text.encode()
            deadline = time.monotonic() + timeout
            while target not in output and time.monotonic() < deadline:
                if select.select([screen], [], [], 0.1)[0]:
                    output.extend(os.read(screen, 8192))
            self.assertIn(target, output)
            output.clear()

        try:
            wait_for('选择方向开始')
            os.write(screen, b'p')
            wait_for('平移速度')
            os.write(screen, b'0.08\n0.2\n0.3\n')
            wait_for('参数已更新')
            os.write(screen, b'\x1b[A')
            wait_for('预计剩余')
            wait_for('已尝试发送停车报文')
            wire = bytearray()
            while select.select([self.master], [], [], 0)[0]:
                wire.extend(os.read(self.master, 8192))
            self.assertTrue(any(wire[i + 21] == 1 and wire[i + 22:i + 24] == b'\x50\x00'
                                for i in range(0, len(wire) - 29, 30)))
            os.write(screen, b'q')
            wait_for('左转：')
            wait_for('已尝试发送停车报文')
            while select.select([self.master], [], [], 0)[0]:
                os.read(self.master, 8192)
            movements = [
                (b'\x1b[B', (-80, 0, 0)), (b'\x1b[D', (0, 80, 0)),
                (b'\x1b[C', (0, -80, 0)), (b'e', (0, 0, -200)),
            ]
            for key, expected in movements:
                os.write(screen, key)
                wait_for('预计剩余')
                wait_for('已尝试发送停车报文')
                wire = bytearray()
                while select.select([self.master], [], [], 0)[0]:
                    wire.extend(os.read(self.master, 8192))
                self.assertTrue(any(frame[21] == 1 and struct.unpack('<hhh', frame[22:28]) == expected
                                    for frame in (wire[i:i + 30] for i in range(0, len(wire) - 29, 30))))
            os.write(screen, b'v')
            wait_for('输入 vx vy wz')
            os.write(screen, b'nan 0 0 1\n')
            wait_for('输入无效')
            os.write(screen, b'x')
            process.wait(timeout=5)
            self.assertEqual(process.returncode, 0)
            self.assertEqual(termios.tcgetattr(terminal), original)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            os.close(screen)
            os.close(terminal)

    def test_key_interface_interrupt_stops(self):
        screen, terminal = pty.openpty()
        original = termios.tcgetattr(terminal)
        process = subprocess.Popen([sys.executable, str(SCRIPT), '--device', self.device],
                                   stdin=terminal, stdout=terminal, stderr=terminal,
                                   env=dict(os.environ, TERM='xterm'))
        try:
            time.sleep(0.2)
            os.write(screen, b'\x1b[C')
            time.sleep(0.8)
            process.send_signal(signal.SIGINT)
            output = bytearray()
            deadline = time.monotonic() + 5
            while '已请求停车'.encode() not in output and time.monotonic() < deadline:
                if select.select([screen], [], [], 0.1)[0]:
                    output.extend(os.read(screen, 8192))
            self.assertIn('已请求停车'.encode(), output)
            os.write(screen, b'x')
            process.wait(timeout=5)
            self.assertEqual(process.returncode, 0)
            self.assertEqual(termios.tcgetattr(terminal), original)
            wire = bytearray()
            while select.select([self.master], [], [], 0)[0]:
                wire.extend(os.read(self.master, 8192))
            self.assertGreater(len(wire), 0)
            self.assertEqual(wire[-9], 0)  # 最后一帧 valid=0。
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            os.close(screen)
            os.close(terminal)


if __name__ == '__main__':
    unittest.main()
