"""通过 STM32 USART6 遥控底盘、建立里程计原点并返航。"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import secrets
import struct
import time

from uart6_protocol import (
    AVAILABLE, ANGULAR_LIMIT, DEFAULT_ANGULAR, DEFAULT_LINEAR, LINEAR_LIMIT,
    RELEASE, RUN_PROFILE, angular_limit, linear_limit,
    ACCEPTED, ARM, BRAKING, DETAIL_NAMES, DISARMED, DONE, FAULT, KEEPALIVE,
    ORIGIN, POSE, PREPARING, READY, REJECTED, RETURN, RUNNING, STATE_NAMES,
    STATUS, STOP, VELOCITY, FrameReader, frame,
)


DIRECTION_KEYS = frozenset("wsadqe")
CONTROL_KEYS = DIRECTION_KEYS | {"return", "space", "o", "r", "escape"}
STATUS_TIMEOUT_S = 0.25


@dataclass
class Request:
    kind: int
    number: int
    started: float
    sent: float


class ManualPanel:
    def __init__(self, root, port, speed, yaw_rate):
        from tkinter import StringVar, ttk

        self.root = root
        self.port = port
        self.speed = speed
        self.yaw_rate = yaw_rate
        self.reader = FrameReader()
        self.state = None
        self.flags = 0
        self.last_status = 0.0
        self.active = False
        self.arm_deadline = None
        self.pending = None
        self.return_id = None
        self.close_after_release = False
        self.request_number = secrets.randbelow(65535)
        self.held = set()
        self.motion = set()
        self.pose = None

        root.title("GC Rescue · USART6 底盘遥控")
        root.protocol("WM_DELETE_WINDOW", self.close)
        panel = ttk.Frame(root, padding=18)
        panel.grid()
        self.label = StringVar(value="等待固件状态")
        self.pose_label = StringVar(value="原点未建立")
        self.note = StringVar()
        ttk.Label(panel, text=f"{port.port} · 115200 8N1").grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(panel, textvariable=self.label).grid(row=1, column=0, columnspan=4, sticky="w", pady=(8, 0))
        ttk.Label(panel, textvariable=self.pose_label).grid(row=2, column=0, columnspan=4, sticky="w")
        ttk.Label(panel, textvariable=self.note, width=44).grid(row=3, column=0, columnspan=4, sticky="w", pady=(0, 10))
        ttk.Button(panel, text="Enter 接管 / 归还", command=self.toggle_control).grid(row=4, column=0)
        ttk.Button(panel, text="O 建原点", command=self.save_origin).grid(row=4, column=1)
        ttk.Button(panel, text="R 返航", command=self.return_home).grid(row=4, column=2)
        ttk.Button(panel, text="空格 停车", command=self.stop).grid(row=4, column=3)
        ttk.Label(panel, text="W/S 前后  A/D 左右  Q/E 转向  松键停止 • Esc 退出").grid(
            row=5, column=0, columnspan=4, sticky="w", pady=(10, 0))
        # 即使焦点在按钮上，停车键也必须先于按钮的默认快捷键。
        root.bind_class("UartManualKeys", "<KeyPress>", self.key_down)
        root.bind_class("UartManualKeys", "<KeyRelease>", self.key_up)
        for widget in (root, panel, *panel.winfo_children()):
            widget.bindtags(("UartManualKeys", *widget.bindtags()))
        root.bind("<FocusOut>", self.focus_out)
        self.note.set("等待底盘状态，按 Enter 接管")
        root.after(0, self.tick)

    def send(self, kind, payload=b""):
        if self.port is None:
            return False
        try:
            data = frame(kind, payload)
            if self.port.write(data) != len(data):
                raise OSError("串口写入不完整")
            return True
        except OSError as error:
            self.link_error(error)
            return False

    def link_error(self, error):
        port, self.port = self.port, None
        self.active = False
        self.arm_deadline = None
        self.pending = self.return_id = None
        self.motion.clear()
        if port is not None:
            try:
                port.close()
            except OSError:
                pass
        self.label.set("串口已关闭")
        self.note.set(f"串口异常，底盘按超时停车：{error}")

    def request(self, kind):
        if self.port is None or self.pending is not None or self.return_id is not None:
            return
        if kind == RELEASE and not self.send(VELOCITY, struct.pack("<Bhhh", 1, 0, 0, 0)):
            return
        self.request_number = self.request_number % 65535 + 1
        now = time.monotonic()
        self.pending = Request(kind, self.request_number, now, now)
        if kind == RETURN:
            self.return_id = self.request_number
        self.send_request(self.pending)

    def send_request(self, request):
        payload = struct.pack("<H", request.number)
        if request.kind == RETURN:
            payload += struct.pack("<HH", round(min(self.speed, linear_limit(self.flags)) * 1000),
                                   round(min(self.yaw_rate, angular_limit(self.flags)) * 1000))
        self.send(request.kind, payload)

    def arm(self):
        available = self.state == READY and self.flags & AVAILABLE
        if self.active or (self.state != DISARMED and not available) or time.monotonic() - self.last_status >= STATUS_TIMEOUT_S:
            self.note.set("等待底盘就绪或确认失能后接管")
            return
        self.active = True
        self.arm_deadline = time.monotonic() + 1.0
        self.motion.clear()
        self.note.set("等待底盘使能就绪")
        self.request(ARM)

    def at_rest(self):
        return self.active and self.state == READY and not self.motion and self.pending is None

    def can_release(self):
        return (self.active and self.pending is None and self.return_id is None and not self.motion and
                self.state in (READY, RUNNING) and bool(self.flags & RUN_PROFILE) and
                time.monotonic() - self.last_status < STATUS_TIMEOUT_S)

    def toggle_control(self):
        if not self.active:
            self.arm()
        elif self.can_release():
            self.request(RELEASE)

    def save_origin(self):
        if not self.at_rest() or not (self.flags & 2):
            self.note.set("停稳且里程计有效后再建立原点")
            return
        self.request(ORIGIN)

    def return_home(self):
        if not self.at_rest() or (self.flags & 3) != 3:
            self.note.set("请先建立原点并松开方向键，等底盘就绪后返航")
            return
        self.request(RETURN)

    def stop(self, message="已请求停车并失能"):
        self.active = False
        self.arm_deadline = None
        self.pending = self.return_id = None
        self.motion.clear()
        if self.send(STOP):
            self.note.set(message)

    def focus_out(self, _event):
        # 窗口内按钮切换时仍由同一操作员控制。
        self.root.after_idle(self.check_focus)

    def check_focus(self):
        if self.root.focus_displayof() is None:
            self.held.clear()
            if self.pending and self.pending.kind == RELEASE:
                return
            if self.can_release():
                self.request(RELEASE)
            elif self.active:
                self.stop("窗口失焦，已请求停车")

    def key_down(self, event):
        key = event.keysym.lower()
        if key not in CONTROL_KEYS:
            return None
        if key in self.held:
            return "break"
        self.held.add(key)
        if key in DIRECTION_KEYS:
            # 真正的新按键才允许启动，已按住的键不会跨使能、制动或返航生效。
            if self.active and self.state in (READY, RUNNING) and self.pending is None and self.return_id is None:
                self.motion.add(key)
        elif key == "return":
            self.toggle_control()
        elif key == "space":
            self.stop()
        elif key == "o":
            self.save_origin()
        elif key == "r":
            self.return_home()
        else:
            self.close()
        return "break"

    def key_up(self, event):
        key = event.keysym.lower()
        self.held.discard(key)
        self.motion.discard(key)
        return "break" if key in CONTROL_KEYS else None

    def velocity(self):
        x = int("w" in self.motion) - int("s" in self.motion)
        y = int("a" in self.motion) - int("d" in self.motion)
        turn = int("q" in self.motion) - int("e" in self.motion)
        speed = min(self.speed, linear_limit(self.flags))
        scale = math.floor(speed * 1000.0 / max(1.0, math.hypot(x, y)))
        yaw = min(self.yaw_rate, angular_limit(self.flags))
        return struct.pack("<Bhhh", 1, x * scale, y * scale, round(turn * yaw * 1000))

    def receive(self, kind, payload, now):
        if kind == POSE and len(payload) == 6 and (self.flags & 3) == 3:
            self.pose = struct.unpack("<hhh", payload)
        if kind != STATUS or len(payload) != 7:
            return
        state, flags, number, command, result, detail = struct.unpack("<BBHBBB", payload)
        if state >= len(STATE_NAMES):
            return
        previous_state = self.state
        self.state, self.flags = state, flags
        self.last_status = now
        if state in (READY, RUNNING):
            self.arm_deadline = None
        elif state == PREPARING and previous_state == RUNNING:
            self.motion.clear()
        if (self.flags & 3) != 3:
            self.pose = None

        pending_match = self.pending and (command, number) == (self.pending.kind, self.pending.number)
        if pending_match and result in (ACCEPTED, REJECTED, DONE, FAULT):
            self.pending = None
            reason = DETAIL_NAMES[detail] if detail < len(DETAIL_NAMES) else str(detail)
            if result in (REJECTED, FAULT):
                self.note.set(f"请求未完成：{reason}")
                self.close_after_release = False
                if command == ARM:
                    self.active = False
                    self.arm_deadline = None
            elif command == ORIGIN:
                self.note.set("已建立原点")
            elif command == RELEASE and result == DONE and state == READY and flags & AVAILABLE:
                self.active = False
                self.note.set("已归还控制，底盘保持使能")
                if self.close_after_release:
                    self.root.after_idle(self.close)
            elif command == RETURN and result == ACCEPTED:
                self.note.set("正在返航")
        if command == RETURN and number == self.return_id and result in (DONE, REJECTED, FAULT):
            reason = DETAIL_NAMES[detail] if detail < len(DETAIL_NAMES) else str(detail)
            self.note.set("返航完成" if result == DONE else f"返航未完成：{reason}")
            self.return_id = None
            self.motion.clear()
        if state == BRAKING or (state == DISARMED and self.arm_deadline is None):
            if self.active and result == FAULT and command != RETURN:
                reason = DETAIL_NAMES[detail] if detail < len(DETAIL_NAMES) else str(detail)
                self.note.set(f"底盘已停车：{reason}")
            self.active = False
            self.arm_deadline = None
            self.pending = self.return_id = None
            self.motion.clear()

    def tick(self):
        now = time.monotonic()
        # 窗口调度可能停顿；先处理已到期的会话，缓冲状态只更新显示。
        if self.active and now - self.last_status >= STATUS_TIMEOUT_S:
            self.stop("状态超时，已请求停车")
        if self.port is not None:
            try:
                chunk = self.port.read(min(self.port.in_waiting, 4096))
                for kind, payload in self.reader.feed(chunk):
                    self.receive(kind, payload, now)
            except OSError as error:
                self.link_error(error)

        if self.port is not None:
            status_fresh = self.state is not None and now - self.last_status < STATUS_TIMEOUT_S
            if not status_fresh:
                self.label.set("等待有效串口状态")
                self.pose = None
            else:
                self.label.set(STATE_NAMES[self.state])
            if self.active and self.arm_deadline is not None and now >= self.arm_deadline:
                self.stop("使能等待超时，已请求停车")
            if self.active and self.pending is not None:
                request = self.pending
                if now - request.started >= 1.0:
                    self.stop("请求应答超时，已请求停车")
                elif now - request.sent >= 0.10:
                    request.sent = now
                    self.send_request(request)

            if self.active:
                if self.return_id is not None:
                    self.send(KEEPALIVE)
                else:
                    self.send(VELOCITY, self.velocity())

        if self.pose is not None:
            x, y, yaw = self.pose
            self.pose_label.set(f"相对原点  X {x / 1000:+.3f} m  Y {y / 1000:+.3f} m  航向 {yaw / 100:+.2f}°")
        elif (self.flags & 3) == 3:
            self.pose_label.set("等待原点相对位姿")
        else:
            self.pose_label.set("原点未就绪")
        self.root.after(50, self.tick)

    def close(self):
        if self.pending and self.pending.kind == RELEASE:
            self.close_after_release = True
            return
        if self.can_release():
            self.close_after_release = True
            self.request(RELEASE)
            return
        if self.active:
            self.stop()
        if self.port is not None:
            try:
                self.port.close()
            except OSError:
                pass
            self.port = None
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="接 USART6 的电脑串口，例如 COM7")
    parser.add_argument("--speed", type=float, default=DEFAULT_LINEAR, help="手动平移速度，m/s")
    parser.add_argument("--yaw-rate", type=float, default=DEFAULT_ANGULAR, help="手动转向速度，rad/s")
    args = parser.parse_args()
    if not 0.001 <= args.speed <= LINEAR_LIMIT or not 0.001 <= args.yaw_rate <= ANGULAR_LIMIT:
        parser.error(f"平移速度须为 0.001–{LINEAR_LIMIT:.2f} m/s，转向速度须为 0.001–{ANGULAR_LIMIT:.2f} rad/s")
    try:
        import serial
    except ImportError:
        parser.error("缺少 pyserial，请运行 python -m pip install pyserial")
    try:
        import tkinter as tk
    except ImportError:
        parser.error("缺少 tkinter，请先安装 Python 的 Tk 支持")
    try:
        root = tk.Tk()
    except tk.TclError as error:
        parser.exit(1, f"无法打开控制窗口：{error}\n")
    try:
        port = serial.Serial(port=None, baudrate=115200, bytesize=8, parity="N", stopbits=1,
                             timeout=0, write_timeout=0.04)
        port.dtr = False
        port.rts = False
        port.port = args.port
        port.open()
        port.reset_input_buffer()
    except OSError as error:
        root.destroy()
        parser.exit(1, f"无法打开 {args.port}：{error}\n")
    try:
        panel = ManualPanel(root, port, args.speed, args.yaw_rate)
        try:
            root.mainloop()
        except KeyboardInterrupt:
            panel.close()
    finally:
        if port.is_open:
            port.close()


if __name__ == "__main__":
    main()
