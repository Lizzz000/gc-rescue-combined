"""回归 USART6 网页定时动作、按键遥控、返航限速、断流及本机 API 校验。"""
from pathlib import Path
import http.client
import struct
import sys
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import uart6_protocol as wire
import uart6_web as web


class SerialStub:
    def __init__(self):
        self.writes = []
        self.incoming = b""

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, length):
        chunk, self.incoming = self.incoming[:length], self.incoming[length:]
        return chunk

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        pass

    def status(self, state, command, number=0, result=wire.DONE, flags=2):
        self.incoming += wire.frame(
            wire.STATUS, struct.pack("<BBHBBB", state, flags, number, command, result, 0)
        )


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.serial = SerialStub()
        self.control = web.Controller(
            lambda device: self.serial, lambda: [{"device": "COM9", "description": "DAP-Link"}]
        )
        self.clock = patch.object(web.time, "monotonic", return_value=0.0).start()
        self.addCleanup(patch.stopall)
        self.control.action("connect", {"port": "COM9"})
        self.serial.status(wire.DISARMED, wire.STOP)
        self.step(0.12)
        self.control.action("arm", {})
        self.serial.status(wire.READY, wire.ARM, self.control.sequence, wire.ACCEPTED)
        self.step(0.14)
        self.assertTrue(self.control.active)

    def step(self, now, browser=True):
        self.clock.return_value = now
        if browser:
            self.control.get_state()
        self.control.step(now)

    def start_run(self):
        self.control.action("run", {"vx": 0.12, "vy": 0, "wz": 0, "seconds": 0.2})
        return wire.frame(wire.VELOCITY, struct.pack("<Bhhh", 1, 120, 0, 0))

    def test_timer_sends_zero_and_allows_new_run_before_ready_report(self):
        moving = self.start_run()
        self.assertEqual(self.serial.writes[-1], moving)
        self.serial.status(wire.RUNNING, wire.ARM)
        self.step(0.31)
        self.assertEqual(self.serial.writes[-1], moving)
        self.step(0.35)
        self.assertEqual(self.serial.writes[-1], web.ZERO)
        self.assertEqual(self.control.mode, "idle")
        self.start_run()
        self.assertEqual(self.serial.writes[-2:], [web.ZERO, moving])
        self.assertTrue(self.control.active)

    def test_expired_page_cannot_resume_by_polling_again(self):
        self.start_run()
        self.clock.return_value = 0.60
        snapshot = self.control.get_state()
        self.assertFalse(snapshot["armed"])
        self.assertEqual(self.serial.writes[-1], web.STOP_FRAME)
        with self.assertRaises(ValueError):
            self.control.action("run", {"vx": 0.08, "vy": 0, "wz": 0, "seconds": 0.5})

    def test_missing_uart_status_stops_before_cached_motion(self):
        self.start_run()
        self.step(0.41)
        self.assertFalse(self.control.active)
        self.assertEqual(self.serial.writes[-1], web.STOP_FRAME)

    def test_key_hold_release_and_timeout(self):
        moving = wire.frame(wire.VELOCITY, struct.pack("<Bhhh", 1, 100, 0, 0))
        self.control.action("drive", {"keys": ["w"], "linear": 0.10, "angular": 0.50})
        self.assertEqual(self.serial.writes[-1], moving)
        self.assertEqual(self.control.mode, "keys")
        self.serial.status(wire.RUNNING, wire.ARM)
        self.step(0.20)
        self.assertEqual(self.serial.writes[-1], moving)
        self.control.action("drive", {"keys": ["W", "a"], "linear": 0.10, "angular": 0.50})
        diagonal = wire.frame(wire.VELOCITY, struct.pack("<Bhhh", 1, 70, 70, 0))
        self.assertEqual(self.serial.writes[-1], diagonal)
        self.control.action("drive", {"keys": [], "linear": 0.10, "angular": 0.50})
        self.assertEqual(self.serial.writes[-1], web.ZERO)
        self.assertEqual(self.control.mode, "idle")
        self.assertTrue(self.control.active)
        self.clock.return_value = 0.32
        self.control.action("drive", {"keys": ["q"], "linear": 0.10, "angular": 0.40})
        turning = wire.frame(wire.VELOCITY, struct.pack("<Bhhh", 1, 0, 0, 400))
        self.assertEqual(self.serial.writes[-1], turning)
        self.serial.status(wire.RUNNING, wire.ARM)
        self.step(0.40)
        self.assertEqual(self.control.mode, "keys")
        self.step(0.58)
        self.assertEqual(self.serial.writes[-1], web.ZERO)
        self.assertEqual(self.control.mode, "idle")
        self.assertTrue(self.control.active)
        with self.assertRaises(ValueError):
            self.control.action("drive", {"keys": ["x"], "linear": 0.10, "angular": 0.50})

    def test_return_speed_is_sent_and_limited(self):
        self.serial.status(wire.READY, wire.ARM, flags=3)
        self.step(0.20)
        for invalid in (0.51, 0.0005, 10 ** 350):
            with self.assertRaises(ValueError):
                self.control.action("return", {"linear": invalid, "angular": 0.50})
        self.control.action("return", {"linear": 0.20, "angular": 0.80})
        payload = struct.pack("<HHH", self.control.sequence, 200, 800)
        self.assertEqual(self.serial.writes[-1], wire.frame(wire.RETURN, payload))
        self.assertEqual(self.control.mode, "return")

    def test_invalid_timing_and_speed(self):
        for invalid in ({"vx": 0.51}, {"seconds": 0.01}, {"seconds": float("inf")}, {"vx": 0}):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    web.timed_velocity({"vx": 0.1, "vy": 0, "wz": 0, "seconds": 1, **invalid})

    def test_passive_connect_takeover_and_clean_disconnect(self):
        self.control.action("stop", {})
        self.control.action("disconnect", {})
        self.serial.writes.clear()
        self.control.action("connect", {"port": "COM9"})
        self.serial.status(wire.READY, 0, flags=wire.AVAILABLE | wire.RUN_PROFILE | 2)
        self.step(0.23)
        self.assertFalse(self.serial.writes)
        self.assertTrue(self.control.get_state()["can_arm"])
        self.control.action("arm", {})
        self.serial.status(wire.READY, wire.ARM, self.control.sequence, wire.ACCEPTED, flags=10)
        self.step(0.25)
        self.control.action("run", {"vx": 0.4, "vy": 0, "wz": 3, "seconds": .10})
        self.assertEqual(self.serial.writes[-1], wire.frame(wire.VELOCITY, struct.pack("<Bhhh", 1, 400, 0, 3000)))
        self.serial.status(wire.RUNNING, wire.ARM, flags=10)
        self.step(.30)
        self.assertFalse(self.control.get_state()["can_release"])
        self.step(.36)
        self.assertEqual(self.control.get_state()["state"], wire.RUNNING)
        self.assertEqual(self.serial.writes[-1], web.ZERO)
        self.control.action("disconnect", {})
        seq = self.control.sequence
        self.assertEqual(self.serial.writes[-1], wire.frame(wire.RELEASE, struct.pack("<H", seq)))
        self.control.action("disconnect", {})
        self.serial.status(wire.READY, wire.RELEASE, seq-1, wire.DONE, flags=14)
        self.step(.37)
        self.assertTrue(self.control.active)
        self.serial.status(wire.READY, wire.RELEASE, seq, wire.DONE, flags=14)
        self.step(.38)
        self.assertIsNone(self.control.serial)
        self.assertNotIn(web.STOP_FRAME, self.serial.writes)

    def test_legacy_firmware_limits(self):
        with self.assertRaises(ValueError):
            self.control.action("run", {"vx": 0.4, "vy": 0, "wz": 3, "seconds": .1})
        self.assertFalse(self.control.get_state()["can_release"])


class LocalHttpTests(unittest.TestCase):
    def test_token_and_origin_are_required_for_commands(self):
        control = web.Controller(lambda _port: None, lambda: [])
        server = web.LocalServer(0, control)
        runner = threading.Thread(target=server.serve_forever)
        runner.start()
        self.addCleanup(runner.join)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        self.assertEqual(server.server_address[0], "127.0.0.1")

        with HttpClient(server.server_port) as client:
            self.assertEqual(client.post({"Origin": server.url}), 403)
            self.assertEqual(client.post({"X-Control-Token": server.token, "Origin": "http://other.local"}), 403)
            self.assertEqual(client.post({"Host": "other.local", "X-Control-Token": server.token}), 403)
            self.assertEqual(client.post({"X-Control-Token": server.token, "Origin": server.url}), 200)


class HttpClient(http.client.HTTPConnection):
    def __init__(self, port):
        super().__init__("127.0.0.1", port, timeout=2)

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()

    def post(self, headers):
        self.request("POST", "/api/disconnect", "{}", {"Content-Type": "application/json", **headers})
        response = self.getresponse()
        response.read()
        return response.status


if __name__ == "__main__":
    unittest.main()
