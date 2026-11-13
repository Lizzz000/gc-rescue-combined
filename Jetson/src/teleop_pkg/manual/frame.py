"""板间 USB 分帧。与 STM32 link_frame.c、bridge transport/frame/link_frame.hpp 同一布局。"""

SOF = 0xA5
VERSION = 0x01
MAX_PAYLOAD = 8


def crc16(data: bytes) -> int:
    """CRC-16/IBM，初值 0xFFFF，多项式反射后为 0xA001。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def encode(identifier: int, payload: bytes) -> bytes:
    """组一帧。DLC 0 合法，用来表示任务失联。"""
    if not 0 <= identifier <= 0x7FF:
        raise ValueError('standard CAN id required')
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload longer than 8 bytes')
    body = bytes((VERSION, identifier & 0xFF, identifier >> 8, len(payload))) + payload
    crc = crc16(body)
    return bytes((SOF,)) + body + bytes((crc & 0xFF, crc >> 8))


class Parser:
    """逐字节同步。CRC 错时丢掉一个字节，不交出半包。"""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes):
        frames = []
        for byte in data:
            self._buffer.append(byte)
            frames.extend(self._pull())
        return frames

    def _pull(self):
        frames = []
        while len(self._buffer) >= 5:
            if self._buffer[0] != SOF:
                del self._buffer[0]
                continue
            if self._buffer[1] != VERSION or self._buffer[4] > MAX_PAYLOAD:
                del self._buffer[0]
                continue
            total = 7 + self._buffer[4]
            if len(self._buffer) < total:
                return frames
            body = bytes(self._buffer[1:5 + self._buffer[4]])
            actual = self._buffer[5 + self._buffer[4]] | (self._buffer[6 + self._buffer[4]] << 8)
            if crc16(body) != actual:
                del self._buffer[0]
                continue
            payload_len = self._buffer[4]
            identifier = self._buffer[2] | (self._buffer[3] << 8)
            payload = bytes(self._buffer[5:5 + payload_len])
            del self._buffer[:total]
            frames.append((identifier, payload))
        return frames
