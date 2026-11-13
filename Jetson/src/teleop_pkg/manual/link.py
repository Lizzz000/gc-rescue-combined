"""板间 USB CDC：独占打开 STM32 串口，发送 0x17F 手动速度帧。"""

import os
import struct

try:
    from manual.frame import encode
except ImportError:
    from frame import encode

# 0x17F 保持 DLC 7：valid(u8) + vx/vy/wz(int16 小端，×1000)。
# 自动 0x180 是 DLC 8，pause 在 data[7]。这里不能复用那条编码。
MANUAL_ID = 0x17F
MANUAL_DLC = 7


class LinkError(OSError):
    """设备不可用、被占用或写入不完整。"""


def manual_packet(valid, vx_milli, vy_milli, wz_milli):
    """返回一帧 0x17F 的 USB 分帧；速度为已量化的 ×1000 整数。"""
    payload = struct.pack('<Bhhh', int(valid), vx_milli, vy_milli, wz_milli)
    return encode(MANUAL_ID, payload)


class DamiaoLink:
    """名字沿用调用点。现在独占的是 STM32 CDC，不再是达妙 30 字节外壳。"""

    def __init__(self, device):
        import fcntl
        import termios
        import tty
        self._fd = None
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
        """写完整一帧。短写继续写完；设备错误才失败。"""
        sent = 0
        while sent < len(packet):
            try:
                wrote = os.write(self._fd, packet[sent:])
            except BlockingIOError:
                continue
            except OSError as error:
                raise LinkError(f'串口写入失败：{error}') from error
            if wrote <= 0:
                raise LinkError('串口报文写入不完整')
            sent += wrote

    def close(self):
        import fcntl
        import termios
        if self._fd is None:
            return
        try:
            termios.tcdrain(self._fd)
            fcntl.ioctl(self._fd, termios.TIOCNXCL)
        finally:
            os.close(self._fd)
            self._fd = None
