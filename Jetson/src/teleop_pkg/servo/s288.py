"""宇树 S288 单总线协议；这是仓库里唯一的协议实现。

手动网页与 bridge_pkg/gripper 薄转发都使用本模块。
不导入 bridge，不发送 CAN。角度、速度和力矩都是输出端单位。
6 Mbps 用 termios2，数值在 Orin aarch64 上核对过。
"""

import math
import os
import select
import struct
import threading
import time

RATIO = 70070.0 / 243.0
BAUD = 6000000
MIN_MOVE_DEGREES = -360.0
MAX_MOVE_DEGREES = 360.0
MIN_MOVE_SECONDS = 0.2
MAX_MOVE_SECONDS = 10.0
# 反馈等待、停机重试和位置指令间隔。间隔只让出 CPU，不表示已经到位。
FEEDBACK_WAIT_S = 0.2
STOP_ATTEMPTS = 3
COMMAND_PERIOD_S = 0.02
CONTROL_MARK = b'\xfe\xee'
FEEDBACK_MARK = b'\xfc\xee'
FEEDBACK_BYTES = 26
READ_BYTES = 64
MODE_STOP = 0
MODE_RUN = 1
# aarch64：TCGETS2/TCSETS2 与 44 字节 termios2，BOTHER=4096，CBAUD=4111。
_TCGETS2 = 0x802C542A
_TCSETS2 = 0x402C542B
_BOTHER = 4096
_CBAUD = 4111
_TERMIOS2_BYTES = 44
_CFLAG_OFFSET = 8
_SPEED_OFFSET = 36
# 手册里的定点缩放：力矩、速度、位置、位置刚度和速度刚度。
_TORQUE_SCALE = 256000
_SPEED_SCALE = 2.56
_POSITION_SCALE = 32768
_KP_SCALE = 1280000
_KD_SCALE = 128000000


def check_move(degrees, seconds):
    """拒绝看不懂或幅度过大的手动转角。"""
    if not math.isfinite(degrees) or not math.isfinite(seconds):
        raise ValueError('角度和时长必须是有限数')
    if degrees == 0 or not MIN_MOVE_DEGREES <= degrees <= MAX_MOVE_DEGREES:
        raise ValueError(
            f'角度须在 {MIN_MOVE_DEGREES:g} 到 {MAX_MOVE_DEGREES:g} 度之间，且不能为 0')
    if not MIN_MOVE_SECONDS <= seconds <= MAX_MOVE_SECONDS:
        raise ValueError(f'时长须在 {MIN_MOVE_SECONDS:g} 到 {MAX_MOVE_SECONDS:g} 秒之间')


def crc32(data):
    """每四字节按小端字序计算手册中的 CRC32。"""
    if len(data) % 4:
        raise ValueError('CRC 数据长度必须是 4 的倍数')
    crc = 0xFFFFFFFF
    for offset in range(0, len(data), 4):
        word = int.from_bytes(data[offset:offset + 4], 'little')
        crc ^= word
        for _ in range(32):
            crc = ((crc << 1) ^ (0x04C11DB7 if crc & 0x80000000 else 0)) & 0xFFFFFFFF
    return crc


def control_packet(motor_id, mode, timeout, torque, speed, position, kp, kd):
    """生成 20 字节控制包；拒绝无法编码的数值。"""
    if not 0 <= motor_id <= 14 or mode not in (0, 1) or timeout not in (0, 1):
        raise ValueError('舵机 ID、模式或超时位不合法')
    if not all(math.isfinite(value) for value in (torque, speed, position, kp, kd)) or kp < 0 or kd < 0:
        raise ValueError('控制量必须有限，增益不能为负')
    raw = (
        round(torque / RATIO * _TORQUE_SCALE),
        round(speed * RATIO * _SPEED_SCALE / (2 * math.pi)),
        round(position * RATIO * _POSITION_SCALE / (2 * math.pi)),
        round(kp / (RATIO * RATIO) * _KP_SCALE),
        round(kd / (RATIO * RATIO) * _KD_SCALE),
    )
    try:
        payload = struct.pack('<2sBBhhihh', CONTROL_MARK, motor_id | mode << 4 | timeout << 7, 0, *raw)
    except struct.error as error:
        raise ValueError('控制量超出 S288 协议范围') from error
    return payload + struct.pack('<I', crc32(payload))


def parse_feedback(packet, expected_id):
    """校验并解析 26 字节反馈，返回输出端物理量。"""
    if len(packet) != FEEDBACK_BYTES or packet[:2] != FEEDBACK_MARK:
        raise ValueError('反馈长度或帧头不正确')
    if crc32(packet[2:22]) != struct.unpack_from('<I', packet, 22)[0]:
        raise ValueError('反馈 CRC 不正确')
    mode, temperature, winding, voltage, torque, speed, position, error, output = struct.unpack_from(
        '<BbBBhhiIH', packet, 2)
    if mode & 15 != expected_id:
        raise ValueError('反馈舵机 ID 不匹配')
    return {
        'id': mode & 15, 'mode': (mode >> 4) & 7, 'timeout': bool(mode & 128),
        'temperature': temperature, 'winding_temperature': winding, 'voltage': voltage / 2.0,
        'torque': torque * RATIO / _TORQUE_SCALE, 'speed': speed * (2 * math.pi) / (_SPEED_SCALE * RATIO),
        'position': position * (2 * math.pi) / (_POSITION_SCALE * RATIO),
        'output_encoder': output & 0x1FFF, 'warning': output >> 13, 'error': error,
    }


def _take_feedback(data, motor_id):
    """看缓冲区开头有没有一帧反馈。不完整或校验失败时改写 data 并返回 None。"""
    start = data.find(FEEDBACK_MARK)
    if start < 0:
        data[:] = data[-1:]
        return None
    if start:
        del data[:start]
    if len(data) < FEEDBACK_BYTES:
        return None
    try:
        return parse_feedback(bytes(data[:FEEDBACK_BYTES]), motor_id)
    except ValueError:
        del data[0]
        return None


def _set_baud(fd):
    """把已打开的串口设成 8N1、6 Mbps。标准 termios 没有这个波特率。"""
    import fcntl
    import termios
    buf = bytearray(_TERMIOS2_BYTES)
    fcntl.ioctl(fd, _TCGETS2, buf, True)
    cflag = struct.unpack_from('<I', buf, _CFLAG_OFFSET)[0]
    cflag = (cflag & ~_CBAUD & ~termios.CSIZE) | _BOTHER | termios.CS8 | termios.CLOCAL | termios.CREAD
    struct.pack_into('<IIII', buf, 0, 0, 0, cflag, 0)
    struct.pack_into('<II', buf, _SPEED_OFFSET, BAUD, BAUD)
    fcntl.ioctl(fd, _TCSETS2, buf)


class ServoLink:
    """独占 S288 串口。调用方保证同一时刻只有一个线程在收发。"""

    def __init__(self, device, motor_id):
        import fcntl
        import termios
        import tty
        self.motor_id = motor_id
        self._fd = None
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            tty.setraw(fd)
            _set_baud(fd)
        except OSError as error:
            os.close(fd)
            raise OSError(f'{device} 被占用或不可配置：{error}') from error
        self._fd = fd
        self.device = device

    def exchange(self, packet):
        """发送一帧，并在反馈期限内找到一帧 CRC 正确的反馈。"""
        if os.write(self._fd, packet) != len(packet):
            raise OSError('串口写入不完整')
        deadline = time.monotonic() + FEEDBACK_WAIT_S
        data = bytearray()
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if not select.select([self._fd], [], [], remaining)[0]:
                continue
            try:
                data.extend(os.read(self._fd, READ_BYTES))
            except BlockingIOError:
                continue
            feedback = _take_feedback(data, self.motor_id)
            if feedback is not None:
                return feedback
        raise TimeoutError('未收到有效的舵机反馈')

    def stop(self):
        """发送停机命令，读到 Stop 模式后才算确认。"""
        packet = control_packet(self.motor_id, MODE_STOP, 0, 0, 0, 0, 0, 0)
        for _ in range(STOP_ATTEMPTS):
            try:
                feedback = self.exchange(packet)
            except TimeoutError:
                continue
            if feedback['mode'] == MODE_STOP:
                return feedback
        raise RuntimeError('已发送停机命令，但未确认 Stop 模式')

    def move_degrees(self, degrees, seconds, kp, kd, cancel):
        """从当前反馈位置相对转动，到时或取消后停机。返回停机反馈。"""
        check_move(degrees, seconds)
        # 先确认增益能编码，再停机读位置。非法增益不会先把舵机停掉。
        control_packet(self.motor_id, MODE_RUN, 1, 0, 0, 0, kp, kd)
        initial = self.stop()
        if initial['error'] or initial['timeout']:
            raise RuntimeError('舵机反馈有错误或超时，未开始转动')
        target = initial['position'] + math.radians(degrees)
        command = control_packet(self.motor_id, MODE_RUN, 1, 0, 0, target, kp, kd)
        started = time.monotonic()
        try:
            while time.monotonic() - started < seconds:
                if cancel.is_set():
                    break
                feedback = self.exchange(command)
                if feedback['error'] or feedback['timeout']:
                    raise RuntimeError('舵机反馈错误或超时')
                time.sleep(COMMAND_PERIOD_S)
        finally:
            final = self.stop()
        final['turned_deg'] = math.degrees(final['position'] - initial['position'])
        return final

    def close(self):
        if self._fd is None:
            return
        import fcntl
        import termios
        try:
            fcntl.ioctl(self._fd, termios.TIOCNXCL)
        finally:
            os.close(self._fd)
            self._fd = None


class ServoSession:
    """网页上的一次转角。底盘停车不会取消它；只有舵机停机或进程退出会。"""

    def __init__(self, device, motor_id, kp, kd):
        self._lock = threading.Lock()
        # 转角、停机和关闭都走这把锁，避免两个线程同时写同一串口。
        self._motion = threading.Lock()
        self._cancel = threading.Event()
        self._link = None
        self.busy = False
        self.position_deg = None
        self.message = '未配置舵机串口'
        self.device = device
        self.kp = kp
        self.kd = kd
        if not device:
            return
        try:
            control_packet(motor_id, MODE_RUN, 1, 0, 0, 0, kp, kd)
            self._link = ServoLink(device, motor_id)
        except (OSError, ValueError) as error:
            self.message = str(error)
            return
        self.message = '已连接，尚未转动'

    @property
    def available(self):
        return self._link is not None

    def snapshot(self):
        with self._lock:
            return {
                'available': self.available, 'busy': self.busy, 'device': self.device,
                'position_deg': self.position_deg, 'message': self.message,
            }

    def move(self, degrees, seconds):
        check_move(degrees, seconds)
        with self._lock:
            if self._link is None:
                raise RuntimeError(self.message)
            if self.busy:
                raise RuntimeError('舵机正在转动')
            self.busy = True
            self._cancel.clear()
            self.message = f'正在转动 {degrees:g}°'
        try:
            with self._motion:
                feedback = self._link.move_degrees(degrees, seconds, self.kp, self.kd, self._cancel)
        except Exception as error:
            with self._lock:
                self.busy = False
                self.message = str(error)
            raise
        with self._lock:
            self.busy = False
            self.position_deg = math.degrees(feedback['position'])
            self.message = f"已停机，实际转动 {feedback['turned_deg']:.2f}°"
        return feedback

    def stop(self):
        """请求正在进行的转角停机；空闲时再发一次停机命令。"""
        self._cancel.set()
        with self._lock:
            if self.busy or self._link is None:
                return None
            link = self._link
        with self._motion:
            feedback = link.stop()
        with self._lock:
            self.position_deg = math.degrees(feedback['position'])
            self.message = '已停机'
        return feedback

    def close(self):
        self._cancel.set()
        with self._motion:
            with self._lock:
                link = self._link
                self._link = None
                self.busy = False
        if link is None:
            return
        try:
            link.stop()
        except (OSError, TimeoutError, RuntimeError):
            pass
        link.close()
