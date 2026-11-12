"""手动控制 HTTP 接口：只绑定 127.0.0.1，经 SSH 端口转发访问。

POST /api/hold  {"vx","vy","wz"}            按住中的速度，租约内须续发
POST /api/timed {"vx","vy","wz","seconds"}  定时动作
POST /api/stop                              停车
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
from pathlib import Path
import time

from manual.command import MAX_ANGULAR_RADPS, MAX_LINEAR_MPS
from servo.s288 import ServoBusy

BIND_ADDRESS = '127.0.0.1'
_PAGE = (Path(__file__).resolve().parent / 'index.html').read_bytes()
_MAX_BODY = 1024


class ManualServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, port, command, status, servo):
        """command 为 ManualCommand；status() 返回链路诊断；servo 只处理手动转角。"""
        super().__init__((BIND_ADDRESS, port), _Handler)
        self.command = command
        self.status = status
        self.servo = servo


def _numbers(body, names):
    values = []
    for name in names:
        value = body.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} 须为数值')
        values.append(float(value))
    return values


class _Handler(BaseHTTPRequestHandler):
    server_version = 'gc-rescue-manual'

    def log_message(self, format, *args):
        """每 50 ms 一次的续租请求不写访问日志。"""

    def do_GET(self):
        if self.path == '/':
            self._reply(HTTPStatus.OK, _PAGE, 'text/html; charset=utf-8')
        elif self.path == '/api/state':
            self._json(HTTPStatus.OK, self._state())
        else:
            self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})

    def do_POST(self):
        actions = {
            '/api/hold': self._hold, '/api/timed': self._timed, '/api/stop': self._stop,
            '/api/servo': self._servo, '/api/servo/open': self._servo_open,
            '/api/servo/close': self._servo_close, '/api/servo/stop': self._servo_stop,
        }
        action = actions.get(self.path)
        if action is None:
            self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})
            return
        try:
            status = action(self._body(), time.monotonic())
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
                'limits': {'linear_mps': MAX_LINEAR_MPS, 'angular_radps': MAX_ANGULAR_RADPS},
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
