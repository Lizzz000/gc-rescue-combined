"""板间 USB 全速 CDC：独占打开 link.device，只发送类型 2。"""

import os
import select
import struct
import time

from manual.frame import encode

# 类型 2，长度 7：valid(u8) + vx/vy/wz(int16 小端，×1000)。
# 不带 pause。自动桥的类型 1 是另一条编码，这里不能复用。
MANUAL_ID = 2
MANUAL_DLC = 7
WRITE_TIMEOUT_S = 0.05


class LinkError(OSError):
    """设备不可用、被占用或写入不完整。"""


def manual_packet(valid, vx_milli, vy_milli, wz_milli):
    """返回一帧类型 2；速度为已量化的 ×1000 整数。"""
    payload = struct.pack('<Bhhh', int(valid), vx_milli, vy_milli, wz_milli)
    return encode(MANUAL_ID, payload)


class CdcLink:
    """独占 STM32 的 USB CDC。短写会继续写完。"""

    def __init__(self, device):
        import fcntl
        import termios
        import tty
        self._fd = None
        self._write_failed = False
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            tty.setraw(fd)
            settings = termios.tcgetattr(fd)
            settings[2] |= termios.CLOCAL | termios.CREAD
            settings[6][termios.VMIN] = 0
            settings[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, settings)
        except OSError as error:
            os.close(fd)
            raise LinkError(f'{device} 被占用或不可配置：{error}') from error
        self._fd = fd

    def send(self, packet):
        """限时写完整帧；持续背压时退出，由下位机断流处理。"""
        sent = 0
        deadline = time.monotonic() + WRITE_TIMEOUT_S
        while sent < len(packet):
            try:
                wrote = os.write(self._fd, packet[sent:])
            except BlockingIOError:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not select.select([], [self._fd], [], remaining)[1]:
                    self._write_failed = True
                    raise LinkError('串口持续不可写，已停止发送')
                continue
            except OSError as error:
                self._write_failed = True
                raise LinkError(f'串口写入失败：{error}') from error
            if wrote <= 0:
                self._write_failed = True
                raise LinkError('串口报文写入不完整')
            sent += wrote

    def close(self):
        import fcntl
        import termios
        if self._fd is None:
            return
        try:
            if self._write_failed:
                termios.tcflush(self._fd, termios.TCOFLUSH)
            fcntl.ioctl(self._fd, termios.TIOCNXCL)
        finally:
            os.close(self._fd)
            self._fd = None
