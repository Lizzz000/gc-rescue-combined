"""手动控制 HTTP 接口：只绑定 127.0.0.1，经 SSH 端口转发访问。

POST /api/hold  {"vx","vy","wz"}            按住中的速度，租约内须续发
POST /api/timed {"vx","vy","wz","seconds"}  定时动作
POST /api/release                           正常松键，减速并保持使能
POST /api/stop                              停车并撤权
POST /api/servo {"degrees","seconds"}       手动舵机相对转角，立即 202；忙时 409
POST /api/servo/open {"seconds"}            转到已标定的张开角；缺 seconds 或非法时 400，忙时 409，受理后 202
POST /api/servo/close {"seconds"}           转到已标定的闭合角；缺 seconds 或非法时 400，忙时 409，受理后 202
POST /api/servo/stop                        停止当前舵机转角
GET  /api/state                             阶段、速度、链路计数、限幅与舵机状态
GET  /                                       控制页面
"""

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time

from servo.s288 import ServoBusy

BIND_ADDRESS = '127.0.0.1'
_PAGE = (Path(__file__).resolve().parent / 'index.html').read_bytes()
_MAX_BODY = 1024
_MAX_COMMAND_CLIENTS = 256


class ManualServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, port, command, status, servo, *, servo_move_s=None):
        """command 为 ManualCommand；status() 返回链路诊断；servo 只处理手动转角。"""
        super().__init__((BIND_ADDRESS, port), _Handler)
        self.command = command
        self.status = status
        self.servo = servo
        preset_seconds = 2.0 if servo_move_s is None else servo_move_s
        self.page = _PAGE.replace(b'__SERVO_PRESET_SECONDS__', str(preset_seconds).encode('ascii'))
        self._motion_lock = threading.Lock()
        self._command_sequences = {}

    def apply_motion(self, action, body, now, *, stopping=False):
        """同一页面的停车、续租和定时动作按发起序号执行，迟到或重复请求不续租。

        无元数据的直接 API 调用保持兼容，但不具有跨 HTTP 连接的乱序保护。
        客户端表不淘汰旧序号，避免被淘汰页面的迟到运动重新生效。
        """
        client, sequence = body.get('client_id'), body.get('sequence')
        ordered = 'client_id' in body or 'sequence' in body
        if ordered:
            if (not isinstance(client, str) or not 1 <= len(client) <= 64
                    or not client.isascii() or not all(c.isalnum() or c in '-_' for c in client)
                    or type(sequence) is not int or not 1 <= sequence <= 2 ** 53 - 1):
                raise ValueError('client_id 和 sequence 须成对提供有效的页面标识及正整数序号')
        with self._motion_lock:
            if ordered:
                previous = self._command_sequences.get(client)
                if previous is not None and sequence <= previous:
                    return
                if previous is None and len(self._command_sequences) >= _MAX_COMMAND_CLIENTS:
                    if stopping:
                        return action(body, now)
                    raise ValueError('控制页面会话数已达上限，请重启手动控制服务')
            result = action(body, now)
            if ordered:
                self._command_sequences[client] = sequence
            return result


def _numbers(body, names):
    values = []
    for name in names:
        value = body.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} 须为数值')
        try:
            value = float(value)
        except OverflowError as error:
            raise ValueError(f'{name} 须为有限数值') from error
        if not math.isfinite(value):
            raise ValueError(f'{name} 须为有限数值')
        values.append(value)
    return values


class _Handler(BaseHTTPRequestHandler):
    server_version = 'gc-rescue-manual'

    def log_message(self, format, *args):
        """每 50 ms 一次的续租请求不写访问日志。"""

    def do_GET(self):
        if self.path == '/':
            self._reply(HTTPStatus.OK, self.server.page, 'text/html; charset=utf-8')
        elif self.path == '/api/state':
            self._json(HTTPStatus.OK, self._state())
        else:
            self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})

    def do_POST(self):
        actions = {
            '/api/hold': self._hold, '/api/timed': self._timed, '/api/stop': self._stop,
            '/api/release': self._release,
            '/api/servo': self._servo, '/api/servo/open': self._servo_open,
            '/api/servo/close': self._servo_close, '/api/servo/stop': self._servo_stop,
        }
        action = actions.get(self.path)
        if action is None:
            self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})
            return
        try:
            body = self._body()
            if self.path in ('/api/hold', '/api/timed', '/api/stop', '/api/release'):
                status = self.server.apply_motion(action, body, time.monotonic(),
                                                  stopping=self.path == '/api/stop')
            else:
                status = action(body, time.monotonic())
        except ServoBusy as error:
            self._json(HTTPStatus.CONFLICT, {'error': str(error), **self._state()})
            return
        except (ValueError, TimeoutError, OSError, RuntimeError) as error:
            self._json(HTTPStatus.BAD_REQUEST, {'error': str(error)})
            return
        self._json(status or HTTPStatus.OK, self._state())

    def _hold(self, body, now):
        self.server.command.hold(*_numbers(body, ('vx', 'vy', 'wz')), now)

    def _timed(self, body, now):
        self.server.command.timed(*_numbers(body, ('vx', 'vy', 'wz', 'seconds')), now)

    def _stop(self, body, now):
        self.server.command.stop(now)

    def _release(self, body, now):
        self.server.command.release(now)

    def _servo(self, body, now):
        degrees, seconds = _numbers(body, ('degrees', 'seconds'))
        self.server.servo.start_move(degrees, seconds)
        return HTTPStatus.ACCEPTED

    def _servo_open(self, body, now):
        seconds, = _numbers(body, ('seconds',))
        self.server.servo.start_open(seconds)
        return HTTPStatus.ACCEPTED

    def _servo_close(self, body, now):
        seconds, = _numbers(body, ('seconds',))
        self.server.servo.start_close(seconds)
        return HTTPStatus.ACCEPTED

    def _servo_stop(self, body, now):
        self.server.servo.stop()

    def _body(self):
        length = int(self.headers.get('Content-Length') or 0)
        if not 0 <= length <= _MAX_BODY:
            raise ValueError('请求体过大')
        raw = self.rfile.read(length) if length else b'{}'
        try:
            body = json.loads(raw)
        except json.JSONDecodeError as error:
            raise ValueError('请求体须为 JSON') from error
        if not isinstance(body, dict):
            raise ValueError('请求体须为 JSON 对象')
        return body

    def _state(self):
        return {**self.server.command.snapshot(time.monotonic()), **self.server.status(),
                'limits': self.server.command.limits,
                'servo': self.server.servo.snapshot()}

    def _json(self, status, payload):
        self._reply(status, json.dumps(payload, ensure_ascii=False).encode(), 'application/json')

    def _reply(self, status, data, content_type):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)
