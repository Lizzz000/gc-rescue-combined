# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""验收专用 S288 反馈。只响应本进程创建的伪终端。"""
import importlib.util
import os
from pathlib import Path
import select
import struct
import threading

from .evidence import ContractFailure, require, write_json

_codec_path = Path(__file__).resolve().parents[2] / 'src/teleop_pkg/servo/s288.py'
_spec = importlib.util.spec_from_file_location('validation_s288_codec', _codec_path)
protocol = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(protocol)

_CONTROL_BYTES = len(protocol.control_packet(0, protocol.MODE_STOP, 0, 0, 0, 0, 0, 0))


class S288Responder:
    """同一台测试舵机的字节流，运行回显目标位置，停止保留最后位置。"""

    def __init__(self, motor_id):
        require(type(motor_id) is int and 0 <= motor_id <= 14, 'invalid S288 test motor ID')
        self.motor_id = motor_id
        self.position = 0
        self.pending = bytearray()
        self.commands = {protocol.MODE_STOP: 0, protocol.MODE_RUN: 0}

    def feed(self, payload):
        self.pending.extend(payload)
        replies = bytearray()
        while self.pending:
            start = self.pending.find(protocol.CONTROL_MARK)
            if start < 0:
                # 分段读取可能只收到下一个帧头的第一个字节。
                tail = self.pending[-1:]
                self.pending[:] = tail if tail == protocol.CONTROL_MARK[:1] else b''
                break
            del self.pending[:start]
            if len(self.pending) < _CONTROL_BYTES:
                break
            packet = self.pending[:_CONTROL_BYTES]
            del self.pending[:_CONTROL_BYTES]
            require(protocol.crc32(packet[:-4]) == int.from_bytes(packet[-4:], 'little'),
                    'invalid S288 control CRC')
            flags = packet[2]
            mode = (flags >> 4) & 7
            require((flags & 15) == self.motor_id, 'unexpected S288 control motor ID')
            require(mode in self.commands, 'unexpected S288 control mode')
            if mode == protocol.MODE_RUN:
                self.position = struct.unpack_from('<i', packet, 8)[0]
            self.commands[mode] += 1
            body = struct.pack('<BbBBhhiIHBB', self.motor_id | (mode << 4), 0, 0, 0,
                               0, 0, self.position, 0, 0, 0, 0)
            replies.extend(protocol.FEEDBACK_MARK + body + struct.pack('<I', protocol.crc32(body)))
        return bytes(replies)


class S288Probe:
    """在 Bridge 打开串口前启动反馈线程，退出时留下伪终端证据。"""

    def __init__(self, directory, motor_id):
        import tty

        self.responder = S288Responder(motor_id)
        self.report = Path(directory) / 's288-pty.json'
        self.stop = threading.Event()
        self.error = None
        self.closed = False
        self.master, self.slave = os.openpty()
        try:
            tty.setraw(self.slave)
            os.set_blocking(self.master, False)
            self.device = os.ttyname(self.slave)
            self.thread = threading.Thread(target=self._serve, name='s288-pty', daemon=True)
            self.thread.start()
        except BaseException:
            self.stop.set()
            os.close(self.master)
            os.close(self.slave)
            raise

    def _serve(self):
        pending = bytearray()
        try:
            while not self.stop.is_set():
                readable, writable, _ = select.select(
                    [] if pending else [self.master], [self.master] if pending else [], [], 0.05)
                if self.stop.is_set():
                    break
                try:
                    if readable:
                        data = os.read(self.master, 4096)
                        require(data, 'S288 pseudo-terminal closed during capture')
                        pending.extend(self.responder.feed(data))
                    if writable:
                        del pending[:os.write(self.master, pending)]
                except BlockingIOError:
                    continue
        except Exception as error:
            self.error = error

    def check(self):
        if self.error is not None:
            raise ContractFailure(f'S288 pseudo-terminal responder failed: {self.error}') from self.error

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        self.thread.join(timeout=1.0)
        os.close(self.master)
        os.close(self.slave)
        stopped = not self.thread.is_alive()
        write_json(self.report, {'device': self.device, 'motor_id': self.responder.motor_id,
                                'stop_commands': self.responder.commands[protocol.MODE_STOP],
                                'run_commands': self.responder.commands[protocol.MODE_RUN],
                                'thread_stopped': stopped,
                                'error': None if self.error is None else str(self.error)})
        require(stopped, 'S288 pseudo-terminal responder did not stop')
        self.check()
