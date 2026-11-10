"""达妙 USB-CAN 串口链路：独占打开设备，封装并发送 0x17F 手动速度帧。"""

import fcntl
import os
import struct
import termios
import tty

# 0x17F 与 0x180 同格式：DLC 7，valid(u8) + vx/vy/wz(int16 小端，×1000)。
MANUAL_ID = 0x17F
MANUAL_DLC = 7
# 达妙 30 字节发送封装：帧头、长度、命令、发送次数；8 号字节为发送间隔，13..16 为 CAN ID。
_HEADER = bytes([0x55, 0xAA, 30, 1, 1])
_PACKET_SIZE = 30
_INTERVAL_OFFSET, _INTERVAL = 8, 10
_ID_OFFSET, _DLC_OFFSET, _DATA_OFFSET = 13, 18, 21
_BAUD = termios.B921600


class LinkError(OSError):
    """设备不可用、被占用或写入不完整。"""


def manual_packet(valid, vx_milli, vy_milli, wz_milli):
    """返回一帧 0x17F 的达妙封装；速度为已量化的 ×1000 整数。"""
    packet = bytearray(_PACKET_SIZE)
    packet[:len(_HEADER)] = _HEADER
    packet[_INTERVAL_OFFSET] = _INTERVAL
    packet[_ID_OFFSET:_ID_OFFSET + 4] = MANUAL_ID.to_bytes(4, 'little')
    packet[_DLC_OFFSET] = MANUAL_DLC
    packet[_DATA_OFFSET:_DATA_OFFSET + MANUAL_DLC] = struct.pack(
        '<Bhhh', int(valid), vx_milli, vy_milli, wz_milli)
    return bytes(packet)


class DamiaoLink:
    """flock + TIOCEXCL 独占设备，与 bridge 的达妙传输同一约定；后打开者失败且不写任何字节。"""

    def __init__(self, device):
        self._fd = None
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            tty.setraw(fd)
            settings = termios.tcgetattr(fd)
            settings[4] = settings[5] = _BAUD
            termios.tcsetattr(fd, termios.TCSANOW, settings)
        except OSError as error:
            os.close(fd)
            raise LinkError(f'{device} 被占用或不可配置：{error}') from error
        self._fd = fd

    def send(self, packet):
        """丢弃适配器回传字节后写一帧；写入不完整视为链路故障。"""
        try:
            os.read(self._fd, 4096)
        except BlockingIOError:
            pass
        try:
            written = os.write(self._fd, packet)
        except OSError as error:
            raise LinkError(f'串口写入失败：{error}') from error
        if written != len(packet):
            raise LinkError('串口报文写入不完整')

    def close(self):
        if self._fd is None:
            return
        try:
            termios.tcdrain(self._fd)
            fcntl.ioctl(self._fd, termios.TIOCNXCL)
        finally:
            os.close(self._fd)
            self._fd = None
