"""HTTP requests can finish out of order; an older movement must not undo stop."""
import http.client
import json
from pathlib import Path
import socket
import sys
import threading
from types import SimpleNamespace
import unittest
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from manual.command import ManualCommand
from web.server import ManualServer


class OrderingTest(unittest.TestCase):
    def setUp(self):
        self.command = ManualCommand(0.2, 0.1)
        self.server = ManualServer(0, self.command, lambda: {}, SimpleNamespace(snapshot=lambda: {}))

    def tearDown(self):
        self.server.server_close()

    def hold(self, body, now):
        self.command.hold(body.get('vx', 0.25), 0, 0, now)

    def ordered(self, seq, now, action=None, client='page', **extra):
        return self.server.apply_motion(action or self.hold,
            {'client_id': client, 'sequence': seq, **extra}, now)

    def test_stop_dominates_late_hold_and_timed(self):
        self.ordered(1, 0)
        self.ordered(4, 0.12, lambda body, now: self.command.stop(now))
        self.ordered(2, 0.15)
        self.ordered(3, 0.16, lambda body, now: self.command.timed(0.25, 0, 0, 1, now))
        self.assertEqual(self.command.snapshot(0.17)['phase'], 'stopping')
        self.assertEqual(self.command.frame(0.5), (False, 0, 0, 0))
        self.ordered(5, 0.6)
        self.assertEqual(self.command.frame(0.71), (True, 250, 0, 0))

    def test_duplicate_does_not_renew_lease(self):
        self.ordered(1, 0)
        self.ordered(1, 0.19)
        self.assertEqual(self.command.snapshot(0.21)['phase'], 'stopping')

    def test_late_stop_cannot_replace_newer_intent(self):
        self.ordered(2, 0)
        self.ordered(1, 0.05, lambda body, now: self.command.stop(now))
        self.assertEqual(self.command.frame(0.11), (True, 250, 0, 0))

    def test_sequence_check_and_action_are_atomic(self):
        entered, release, second_started, stopped = (threading.Event() for _ in range(4))

        def delayed_hold(body, now):
            entered.set()
            if not release.wait(2):
                raise TimeoutError('test action was not released')
            self.hold(body, now)

        def stop():
            second_started.set()
            self.ordered(2, 0.12, lambda body, now: self.command.stop(now))
            stopped.set()

        first = threading.Thread(target=lambda: self.ordered(1, 0, delayed_hold))
        second = threading.Thread(target=stop)
        first.start()
        try:
            self.assertTrue(entered.wait(2))
            second.start()
            self.assertTrue(second_started.wait(2))
            self.assertFalse(stopped.wait(0.02))
        finally:
            release.set()
            first.join(timeout=2)
            if second.ident is not None:
                second.join(timeout=2)
        self.assertTrue(stopped.is_set())
        self.assertEqual(self.command.snapshot(0.13)['phase'], 'stopping')

    def test_invalid_action_does_not_consume_sequence(self):
        with self.assertRaises(ValueError):
            self.ordered(2, 0, vx=float('nan'))
        self.ordered(1, 0.1)
        self.assertEqual(self.command.frame(0.21), (True, 250, 0, 0))

    def test_metadata_requires_valid_pair(self):
        for body in ({'client_id': 'a'}, {'sequence': 1}, {'client_id': 'a', 'sequence': True},
                     {'client_id': 'a', 'sequence': 0}, {'client_id': '', 'sequence': 1},
                     {'client_id': 'a', 'sequence': 2 ** 53}):
            with self.subTest(body=body), self.assertRaises(ValueError):
                self.server.apply_motion(self.hold, body, 0)
        self.server.apply_motion(self.hold, {}, 0)
        self.assertEqual(self.command.frame(0.11), (True, 250, 0, 0))

    def test_client_table_is_bounded_without_losing_stop(self):
        from unittest.mock import patch
        with patch('web.server._MAX_COMMAND_CLIENTS', 1):
            self.ordered(3, 0)
            with self.assertRaises(ValueError):
                self.ordered(1, 0.1, client='second')
            self.server.apply_motion(lambda body, now: self.command.stop(now),
                {'client_id': 'second', 'sequence': 2}, 0.11, stopping=True)
            self.ordered(2, 0.12)
            self.assertEqual(self.command.snapshot(0.13)['phase'], 'stopping')

    def test_http_slow_old_body_cannot_restart_after_stop(self):
        thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        thread.start()
        address = self.server.server_address
        body = json.dumps({'client_id': 'browser', 'sequence': 1, 'vx': 0.25, 'vy': 0, 'wz': 0}).encode()
        try:
            with socket.create_connection(address, timeout=2) as slow:
                slow.sendall(f'POST /api/hold HTTP/1.0\r\nContent-Length: {len(body)}\r\n\r\n'.encode())
                request = urllib.request.Request(f'http://{address[0]}:{address[1]}/api/stop',
                    data=json.dumps({'client_id': 'browser', 'sequence': 2}).encode())
                with urllib.request.urlopen(request, timeout=2) as response:
                    self.assertEqual(response.status, 200)
                slow.sendall(body)
                response = http.client.HTTPResponse(slow)
                response.begin()
                self.assertEqual(response.status, 200)
                self.assertEqual(json.loads(response.read())['phase'], 'idle')
        finally:
            self.server.shutdown()
            thread.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
