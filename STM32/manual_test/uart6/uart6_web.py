"""在电脑本机提供 USART6 遥控网页。"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import struct
import threading
import time
import webbrowser

from uart6_protocol import (
    ANGULAR_LIMIT, AVAILABLE, LINEAR_LIMIT, RELEASE, RUN_PROFILE, angular_limit, linear_limit,
    ACCEPTED, ARM, BRAKING, DETAIL_NAMES, DISARMED, DISARM_CONFIRMED, DONE, FAULT, KEEPALIVE, ORIGIN,
    POSE, READY, REJECTED, RETURN, RUNNING, SETTLING, STATUS, STOP,
    VELOCITY, FrameReader, frame,
)

PAGES = Path(__file__).with_name("uart6_web")
TICK_S = 0.05
LINK_TIMEOUT_S = 0.25
PAGE_TIMEOUT_S = 0.35
REQUEST_TIMEOUT_S = 1.0
KEY_HOLD_S = 0.25
DRIVE_KEYS = ("w", "a", "s", "d", "q", "e")
ZERO = frame(VELOCITY, struct.pack("<Bhhh", 1, 0, 0, 0))
HOLD = frame(KEEPALIVE)
STOP_FRAME = frame(STOP)


@dataclass
class Request:
    command: int
    number: int
    packet: bytes
    started: float
    sent: float


class Controller:
    """串口操作由一把锁串行化；定时发送与截止判断独立于网页请求。"""

    def __init__(self, open_serial, ports):
        self.open_serial = open_serial
        self.ports = ports
        self.lock = threading.Lock()
        self.serial = None
        self.reader = FrameReader()
        self.state = None
        self.flags = 0
        self.pose = None
        self.port = None
        self.last_status = float("-inf")
        self.last_page = float("-inf")
        self.last_stop = float("-inf")
        self.stop_confirmed = False
        self.active = False
        self.pending = None
        self.sequence = secrets.randbelow(65535)
        self.return_id = None
        self.close_after_release = False
        self.mode = "idle"
        self.end_time = None
        self.velocity = ZERO
        self.last_drive = 0.0
        self.next_tx = 0.0
        self.message = "选择串口后连接"

    def reset_motion(self):
        self.close_after_release = False
        self.active = False
        self.pending = None
        self.return_id = None
        self.mode = "idle"
        self.end_time = None
        self.velocity = ZERO

    def close_serial(self, message):
        port, self.serial = self.serial, None
        self.reset_motion()
        self.state = self.port = self.pose = None
        self.flags = 0
        self.last_status = float("-inf")
        self.stop_confirmed = False
        self.message = message
        if port is not None:
            try:
                port.close()
            except OSError:
                pass

    def send(self, packet):
        if self.serial.write(packet) != len(packet):
            raise OSError("串口写入不完整")

    def stop(self, message, now):
        self.reset_motion()
        self.stop_confirmed = False
        self.last_stop = now
        self.message = message
        self.send(STOP_FRAME)

    def disconnect(self):
        with self.lock:
            if self.serial is not None:
                try:
                    if self.active:
                        self.stop("串口已关闭", time.monotonic())
                except OSError:
                    pass
                finally:
                    self.close_serial("串口已关闭")

    def snapshot(self, now):
        fresh = self.serial is not None and now - self.last_status < LINK_TIMEOUT_S
        return {
            "connected": self.serial is not None, "port": self.port, "fresh": fresh,
            "state": self.state, "origin": fresh and bool(self.flags & 1),
            "odom": fresh and bool(self.flags & 2), "pose": self.pose if fresh else None,
            "disarm_confirmed": fresh and self.state == DISARMED and bool(self.flags & DISARM_CONFIRMED),
            "armed": self.active, "pending": self.pending is not None, "mode": self.mode,
            "can_arm": fresh and not self.active and self.can_arm(),
            "can_release": fresh and self.can_release(),
            "linear_limit": linear_limit(self.flags) if fresh else None,
            "angular_limit": angular_limit(self.flags) if fresh else None,
            "remaining": max(0.0, self.end_time - now) if self.end_time is not None else None,
            "message": self.message,
        }

    def get_state(self):
        with self.lock:
            now = time.monotonic()
            if self.active and now - self.last_page >= PAGE_TIMEOUT_S:
                try:
                    self.stop("网页连接超时，已停车", now)
                except OSError as error:
                    self.close_serial(f"串口中断，底盘按断流保护停车（{error}）")
            self.last_page = now
            return self.snapshot(now)

    def begin_request(self, command, now, extra=b""):
        self.sequence = self.sequence % 65535 + 1
        packet = frame(command, struct.pack("<H", self.sequence) + extra)
        self.send(packet)
        self.pending = Request(command, self.sequence, packet, now, now)
        self.next_tx = now

    def finish_motion(self, message, now):
        self.send(ZERO)
        self.velocity = ZERO
        self.mode = "idle"
        self.end_time = None
        self.next_tx = now + TICK_S
        self.message = message

    def receive(self, kind, payload, now):
        if kind == POSE and len(payload) == 6 and self.flags & 3 == 3:
            x, y, yaw = struct.unpack("<hhh", payload)
            self.pose = {"x": x / 1000, "y": y / 1000, "yaw": yaw / 100}
            return
        if kind != STATUS or len(payload) != 7:
            return
        state, flags, number, command, result, detail = struct.unpack("<BBHBBB", payload)
        if state > SETTLING:
            return
        self.state, self.flags = state, flags
        self.last_status = now
        if flags & 3 != 3:
            self.pose = None
        if not self.active and state == DISARMED and (self.last_stop == float("-inf") or
                (command == STOP and result == DONE and now - self.last_stop >= 0.10)):
            self.stop_confirmed = True

        reason = DETAIL_NAMES[detail] if detail < len(DETAIL_NAMES) else "底盘拒绝请求"
        if self.pending and (self.pending.command, self.pending.number) == (command, number):
            if command == ARM and result == ACCEPTED and state in (READY, RUNNING):
                self.pending = None
                self.message = "已使能，底盘就绪"
            elif command == ORIGIN and result == DONE:
                self.pending = None
                self.message = "已建立原点"
            elif command == RELEASE and result == DONE and state == READY and flags & AVAILABLE:
                close = self.close_after_release
                self.reset_motion()
                self.message = "已归还控制，底盘保持使能"
                if close:
                    self.close_serial(self.message)
            elif command == RETURN and result in (ACCEPTED, DONE):
                self.pending = None
                self.message = "正在返航"
            elif result in (REJECTED, FAULT):
                self.pending = None
                self.close_after_release = False
                self.message = reason or "指令未完成"
                if command == ARM:
                    self.reset_motion()
                elif command == RETURN:
                    self.return_id = None
                    self.mode = "idle"

        if command == RETURN and number == self.return_id and result in (DONE, REJECTED, FAULT):
            self.return_id = None
            self.mode = "idle"
            self.message = "已回到原点" if result == DONE else (reason or "返航未完成")
        arming = self.pending is not None and self.pending.command == ARM
        if self.active and (state == BRAKING or (state == DISARMED and not arming)):
            self.reset_motion()
            self.message = reason or "底盘已停车"

    def step(self, now):
        if self.serial is None:
            return
        # 先检查上一次会话是否过期，避免旧缓冲帧恢复已停止的动作。
        if self.active and now - self.last_page >= PAGE_TIMEOUT_S:
            self.stop("网页失去连接，已停车", now)
        elif self.active and now - self.last_status >= LINK_TIMEOUT_S:
            self.stop("串口状态超时，已请求停车", now)
        chunk = self.serial.read(min(self.serial.in_waiting, 4096))
        for kind, payload in self.reader.feed(chunk):
            self.receive(kind, payload, now)
            if self.serial is None:
                return
        if not self.active:
            return
        if self.pending:
            request = self.pending
            if now - request.started >= REQUEST_TIMEOUT_S:
                self.stop("指令应答超时，已请求停车", now)
                return
            if now - request.sent >= 0.10:
                self.send(request.packet)
                request.sent = now
        if self.mode == "keys" and now - self.last_drive >= KEY_HOLD_S:
            self.finish_motion("按键中断，已发零速", now)
        if self.mode == "timed" and now >= self.end_time:
            self.finish_motion("定时运行结束，已发零速", now)
        if now >= self.next_tx:
            if self.mode == "return":
                packet = HOLD
            elif self.mode in ("timed", "keys"):
                packet = self.velocity
            else:
                packet = ZERO
            self.send(packet)
            self.next_tx = now + TICK_S

    def worker(self, finished):
        while not finished.wait(0.01):
            with self.lock:
                try:
                    self.step(time.monotonic())
                except OSError as error:
                    self.close_serial(f"串口中断，底盘按断流保护停车（{error}）")

    def action(self, name, data):
        with self.lock:
            now = time.monotonic()
            was_live = now - self.last_page < PAGE_TIMEOUT_S
            self.last_page = now
            try:
                if self.active and not was_live:
                    self.stop("网页连接超时，已停车", now)
                self.apply(name, data, now)
            except OSError as error:
                self.close_serial(f"串口异常（{error}）")
                raise ValueError(self.message) from error
            return self.snapshot(now)

    def apply(self, name, data, now):
        if name == "connect":
            if self.serial is not None:
                raise ValueError("请先断开当前串口")
            port = data.get("port")
            if not isinstance(port, str) or port not in {item["device"] for item in self.ports()}:
                raise ValueError("请选择当前可用的串口")
            self.serial = self.open_serial(port)
            self.port = port
            self.reader = FrameReader()
            self.last_stop = float("-inf")
            self.message = "串口已连接，等待读取底盘状态"
            return
        if name == "disconnect":
            if self.serial is not None:
                if self.pending and self.pending.command == RELEASE:
                    self.close_after_release = True
                    return
                if now - self.last_status < LINK_TIMEOUT_S and self.can_release():
                    self.close_after_release = True
                    self.begin_request(RELEASE, now)
                    return
                try:
                    if self.active:
                        self.stop("串口已关闭", now)
                finally:
                    self.close_serial("串口已关闭")
            return
        if self.serial is None:
            raise ValueError("请先连接串口")
        if name == "stop":
            self.stop("已请求停车并失能", now)
            return
        if now - self.last_status >= LINK_TIMEOUT_S:
            raise ValueError("等待最新底盘状态")
        if name == "arm":
            if self.active or not self.can_arm():
                raise ValueError("等待底盘就绪或确认失能")
            self.begin_request(ARM, now)
            self.active = True
            self.message = "等待底盘使能就绪"
            return
        if name == "release":
            if not self.can_release():
                raise ValueError("松开方向键，等待底盘零速就绪后归还控制")
            self.begin_request(RELEASE, now)
            return
        if name == "drive":
            if not self.active or self.pending or self.mode not in ("idle", "keys"):
                raise ValueError("请等待底盘就绪")
            if self.state not in (READY, RUNNING):
                raise ValueError("请等待底盘就绪")
            self.velocity = stick_velocity(data, linear_limit(self.flags), angular_limit(self.flags))
            self.last_drive = now
            if self.velocity == ZERO:
                self.finish_motion("已松开按键，已发零速", now)
            else:
                self.send(self.velocity)
                self.next_tx = now + TICK_S
                self.mode = "keys"
                self.message = "按键遥控中"
            return
        if not self.active or self.pending or self.mode != "idle":
            raise ValueError("请等待底盘就绪")
        if name == "run" and self.state in (READY, RUNNING):
            self.velocity, duration = timed_velocity(data, linear_limit(self.flags), angular_limit(self.flags))
            self.send(self.velocity)
            self.mode = "timed"
            self.end_time = now + duration
            self.next_tx = now + TICK_S
            self.message = "正在按设定速度运行"
            return
        if name in ("origin", "return"):
            if self.state != READY:
                raise ValueError("等待底盘确认零速指令")
            if not self.flags & 2:
                raise ValueError("里程计尚未就绪")
            if name == "origin":
                self.begin_request(ORIGIN, now)
                return
            if not self.flags & 1:
                raise ValueError("请先建立原点")
            linear = milli_speed(data.get("linear"), linear_limit(self.flags), "返航平移速度")
            angular = milli_speed(data.get("angular"), angular_limit(self.flags), "返航转向速度")
            self.begin_request(RETURN, now, struct.pack("<HH", linear, angular))
            self.return_id = self.sequence
            self.mode = "return"
            self.message = "正在返航"
            return
        raise ValueError("请等待底盘就绪")

    def can_arm(self):
        if self.state == DISARMED:
            return self.stop_confirmed
        return self.state == READY and bool(self.flags & AVAILABLE) and (
            self.last_stop == float("-inf") or self.stop_confirmed)

    def can_release(self):
        return (self.active and self.pending is None and self.mode == "idle" and
                self.velocity == ZERO and self.state in (READY, RUNNING) and bool(self.flags & RUN_PROFILE))


def milli_speed(value, limit, label):
    if type(value) not in (int, float) or not 0.001 <= value <= limit:
        raise ValueError(f"{label}须在 0.001 至 {limit:.2f} 之间")
    return math.trunc(value * 1000)


def stick_velocity(data, speed_limit=LINEAR_LIMIT, yaw_limit=ANGULAR_LIMIT):
    """按住的方向合成一帧速度。斜向平移按合速度缩回设定值，松开则为零速。"""
    keys = data.get("keys")
    if not isinstance(keys, list) or any(type(key) is not str or key.lower() not in DRIVE_KEYS for key in keys):
        raise ValueError("按键无效")
    held = {key.lower() for key in keys}
    forward = int("w" in held) - int("s" in held)
    left = int("a" in held) - int("d" in held)
    turn = int("q" in held) - int("e" in held)
    if forward == 0 and left == 0 and turn == 0:
        return ZERO
    linear = milli_speed(data.get("linear"), speed_limit, "平移速度") if forward or left else 0
    angular = milli_speed(data.get("angular"), yaw_limit, "转向速度") if turn else 0
    if forward or left:
        scale = math.floor(linear / max(1.0, math.hypot(forward, left)))
        vx, vy = forward * scale, left * scale
    else:
        vx = vy = 0
    wz = turn * angular
    return frame(VELOCITY, struct.pack("<Bhhh", 1, vx, vy, wz))


def timed_velocity(data, speed_limit=LINEAR_LIMIT, yaw_limit=ANGULAR_LIMIT):
    values = [data.get(key) for key in ("vx", "vy", "wz", "seconds")]
    if any(type(value) not in (int, float) or abs(value) > 1e100 or not math.isfinite(value)
           for value in values):
        raise ValueError("请填写有效的速度和运行时间")
    vx, vy, wz, duration = values
    if math.hypot(vx, vy) > speed_limit or abs(wz) > yaw_limit or duration < 0.10:
        raise ValueError("速度超限，或运行时间小于 0.1 秒")
    # 朝零量化，确保斜向速度也在板端支持的合速度范围内。
    fixed = tuple(math.trunc(value * 1000) for value in (vx, vy, wz))
    if not any(fixed):
        raise ValueError("请选择方向并填写非零速度")
    return frame(VELOCITY, struct.pack("<Bhhh", 1, *fixed)), duration


class LocalServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, port, control):
        super().__init__(("127.0.0.1", port), Handler)
        self.control = control
        self.token = secrets.token_urlsafe(32)
        self.host = f"127.0.0.1:{self.server_port}"
        self.url = f"http://{self.host}"


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def respond(self, status, body, content_type):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; base-uri 'none'; frame-ancestors 'none'")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def send_json(self, status, data):
        self.respond(status, json.dumps(data, ensure_ascii=False).encode(), "application/json; charset=utf-8")

    def allowed(self):
        return self.headers.get("Host") == self.server.host and self.headers.get("Origin") in (None, self.server.url)

    def authorized(self):
        return self.allowed() and secrets.compare_digest(
            self.headers.get("X-Control-Token", "").encode(), self.server.token.encode()
        )

    def do_GET(self):
        if not self.allowed():
            self.send_json(403, {"error": "请从本机地址打开控制台"})
            return
        assets = {"/": ("index.html", "text/html"), "/app.js": ("app.js", "text/javascript"),
                  "/style.css": ("style.css", "text/css")}
        if self.path in assets:
            name, mime = assets[self.path]
            content = (PAGES / name).read_bytes()
            if name == "index.html":
                content = content.replace(b"__CONTROL_TOKEN__", self.server.token.encode())
            self.respond(200, content, f"{mime}; charset=utf-8")
            return
        if self.path in ("/api/state", "/api/ports"):
            if not self.authorized():
                self.send_json(403, {"error": "请刷新控制页"})
                return
            control = self.server.control
            self.send_json(200, {"ports": control.ports()} if self.path.endswith("ports") else control.get_state())
            return
        self.send_json(404, {"error": "路径不存在"})

    def do_POST(self):
        if not self.authorized():
            self.send_json(403, {"error": "请求来源无效，请刷新控制页"})
            return
        allowed_actions = {"connect", "disconnect", "arm", "release", "stop", "origin", "return", "run", "drive"}
        action = self.path.removeprefix("/api/") if self.path.startswith("/api/") else ""
        if action not in allowed_actions:
            self.send_json(404, {"error": "命令不存在"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 1 <= length <= 1024 or self.headers.get_content_type() != "application/json":
                raise ValueError
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError
        except (ValueError, UnicodeError):
            self.send_json(400, {"error": "请求参数无效"})
            return
        try:
            state = self.server.control.action(action, body)
        except ValueError as error:
            self.send_json(409, {"error": str(error)})
        else:
            self.send_json(200, state)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--http-port", type=int, default=8765, help="本机网页端口")
    cli.add_argument("--no-browser", action="store_true", help="只启动服务")
    args = cli.parse_args()
    try:
        import serial
        from serial.tools.list_ports import comports
    except ImportError:
        cli.error("缺少 pyserial，请运行 python -m pip install pyserial")

    def ports():
        return [{"device": item.device, "description": item.description} for item in comports()]

    def open_serial(device):
        port = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=0.04)
        port.dtr = port.rts = False
        port.port = device
        port.open()
        try:
            port.reset_input_buffer()
        except OSError:
            port.close()
            raise
        return port

    control = Controller(open_serial, ports)
    finished = threading.Event()
    try:
        server = LocalServer(args.http_port, control)
    except OSError as error:
        cli.exit(1, f"无法启动本地网页：{error}\n")
    runner = threading.Thread(target=control.worker, args=(finished,), daemon=True)
    runner.start()
    print(f"USART6 网页控制台 {server.url}", flush=True)
    if not args.no_browser:
        webbrowser.open(server.url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        finished.set()
        runner.join()
        control.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
