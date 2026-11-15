"""板间 USB 分帧。与 bridge transport/frame/link_frame.hpp 同一布局。"""

SOF = 0xA5
VERSION = 0x01
MAX_PAYLOAD = 8


_CRC_NIBBLE = (
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400,
)


def crc16(data: bytes) -> int:
    """CRC-16/IBM，初值0xFFFF；与两端C/C++相同的4位递推。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        crc = (crc >> 4) ^ _CRC_NIBBLE[crc & 0x0F]
        crc = (crc >> 4) ^ _CRC_NIBBLE[crc & 0x0F]
    return crc


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
    """字节流同步。CRC 错时继续寻找帧头，不交出半包。"""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes):
        """一次扫描输入块，保留末尾半包；无需逐字节创建临时列表或搬移前缀。"""
        self._buffer.extend(data)
        frames = []
        cursor = 0
        while cursor < len(self._buffer):
            start = self._buffer.find(b'\xa5', cursor)
            if start < 0:
                cursor = len(self._buffer)
                break
            cursor = start
            if len(self._buffer) - start < 4:
                break
            payload_len = self._buffer[start + 3]
            if self._buffer[start + 1] != VERSION or payload_len > MAX_PAYLOAD:
                cursor += 1
                continue
            total = 6 + payload_len
            if len(self._buffer) - start < total:
                break
            body = self._buffer[start + 1:start + 4 + payload_len]
            actual = self._buffer[start + 4 + payload_len] | (self._buffer[start + 5 + payload_len] << 8)
            if crc16(body) != actual:
                cursor += 1
                continue
            payload = bytes(self._buffer[start + 4:start + 4 + payload_len])
            frames.append((self._buffer[start + 2], payload))
            cursor += total
        if cursor:
            del self._buffer[:cursor]
        return frames
