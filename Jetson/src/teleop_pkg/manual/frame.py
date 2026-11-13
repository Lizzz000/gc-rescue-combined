"""板间 USB 分帧。与 bridge transport/frame/link_frame.hpp 同一布局。"""

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


def encode(msg_type: int, payload: bytes) -> bytes:
    """组一帧。类型是 uint8 报文类型，不是电机总线编号。"""
    if not 0 <= msg_type <= 0xFF:
        raise ValueError('USB type must fit in uint8')
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload longer than 8 bytes')
    body = bytes((VERSION, msg_type, len(payload))) + payload
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
        while len(self._buffer) >= 4:
            if self._buffer[0] != SOF:
                del self._buffer[0]
                continue
            if self._buffer[1] != VERSION or self._buffer[3] > MAX_PAYLOAD:
                del self._buffer[0]
                continue
            total = 6 + self._buffer[3]
            if len(self._buffer) < total:
                return frames
            body = bytes(self._buffer[1:4 + self._buffer[3]])
            actual = self._buffer[4 + self._buffer[3]] | (self._buffer[5 + self._buffer[3]] << 8)
            if crc16(body) != actual:
                del self._buffer[0]
                continue
            payload_len = self._buffer[3]
            msg_type = self._buffer[2]
            payload = bytes(self._buffer[4:4 + payload_len])
            del self._buffer[:total]
            frames.append((msg_type, payload))
        return frames
