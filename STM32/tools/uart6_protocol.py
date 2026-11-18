"""USART6 遥控串口帧与状态定义，供桌面和网页工具共用。"""
import struct


VELOCITY = 0x02
ARM, STOP, ORIGIN, RETURN, KEEPALIVE = range(0x40, 0x45)
STATUS, POSE = 0x80, 0x81

DISARMED, PREPARING, READY, RUNNING, BRAKING, RETURNING, SETTLING = range(7)
ACCEPTED, REJECTED, DONE, FAULT = range(1, 5)
STATE_NAMES = ("失能", "准备中", "就绪", "遥控中", "制动中", "返航中", "返航停稳中")
DETAIL_NAMES = ("", "状态不允许", "里程计失效", "尚未设原点", "超出行程限位",
                "动作超时", "串口断流", "串口接收错误", "底盘故障")


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def frame(kind, payload=b""):
    body = bytes((1, kind, len(payload))) + payload
    return b"\xa5" + body + struct.pack("<H", crc16(body))


class FrameReader:
    """从串口读到的拆包或粘包中取出 CRC 正确的帧。"""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        while self.buffer:
            start = self.buffer.find(b"\xa5")
            if start == -1:
                self.buffer.clear()
                return
            del self.buffer[:start]
            if len(self.buffer) < 4:
                return
            version, kind, length = self.buffer[1:4]
            if version != 1 or length > 8:
                del self.buffer[0]
                continue
            size = 6 + length
            if len(self.buffer) < size:
                return
            received = int.from_bytes(self.buffer[size - 2:size], "little")
            if received != crc16(self.buffer[1:size - 2]):
                del self.buffer[0]
                continue
            payload = bytes(self.buffer[4:size - 2])
            del self.buffer[:size]
            yield kind, payload
