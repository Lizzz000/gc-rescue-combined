"""手动控制使用的宇树 S288 单总线协议，与自动搬运的 C++ 实现共用线协议和开爪判据。

角度、速度和力矩使用输出端单位。6 Mbps 串口使用 Orin aarch64 的 termios2。
"""

from collections.abc import Callable
from dataclasses import dataclass
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
OUTPUT_COUNTS_PER_REV = 8192
MAX_OPEN_HOLD_NM = 0.6
HOLD_CAPTURE_S = 0.5
HOLD_PROGRESS_DEG = 0.5
HOLD_MAX_EXTENSION_S = 2.5
HOLD_MAX_STEP_S = 0.04
HOLD_CORRECTION_GAIN = 6.0
HOLD_MAX_CORRECTION_RADPS = 0.45
HOLD_STABLE_SPEED_RADPS = 0.12
HOLD_STABLE_SAMPLES = 3
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
    """拒绝看不懂或幅度过大的手动相对转角。"""
    if not math.isfinite(degrees) or not math.isfinite(seconds):
        raise ValueError('角度和时长必须是有限数')
    if degrees == 0 or not MIN_MOVE_DEGREES <= degrees <= MAX_MOVE_DEGREES:
        raise ValueError(
            f'角度须在 {MIN_MOVE_DEGREES:g} 到 {MAX_MOVE_DEGREES:g} 度之间，且不能为 0')
    if not MIN_MOVE_SECONDS <= seconds <= MAX_MOVE_SECONDS:
        raise ValueError(f'时长须在 {MIN_MOVE_SECONDS:g} 到 {MAX_MOVE_SECONDS:g} 秒之间')


def check_goto(target_deg, seconds):
    """拒绝看不懂或超出范围的绝对目标角。目标可以是 0（回到上电零点）。"""
    if not math.isfinite(target_deg) or not math.isfinite(seconds):
        raise ValueError('角度和时长必须是有限数')
    if not MIN_MOVE_DEGREES <= target_deg <= MAX_MOVE_DEGREES:
        raise ValueError(
            f'目标角须在 {MIN_MOVE_DEGREES:g} 到 {MAX_MOVE_DEGREES:g} 度之间')
    if not MIN_MOVE_SECONDS <= seconds <= MAX_MOVE_SECONDS:
        raise ValueError(f'时长须在 {MIN_MOVE_SECONDS:g} 到 {MAX_MOVE_SECONDS:g} 秒之间')


def check_ramp_fraction(fraction):
    if type(fraction) is bool or not isinstance(fraction, (int, float)) or not 0 <= fraction <= 1:
        raise ValueError('gripper.ramp_fraction 须为 0 到 1 之间的数值')


def output_degrees(encoder, zero_encoder):
    half = OUTPUT_COUNTS_PER_REV // 2
    delta = (encoder - zero_encoder + half) % OUTPUT_COUNTS_PER_REV - half
    return delta * 360.0 / OUTPUT_COUNTS_PER_REV


def output_error_degrees(target_deg, encoder, zero_encoder):
    error = target_deg - output_degrees(encoder, zero_encoder)
    return (error + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class OpenHold:
    zero_encoder: int
    target_deg: float
    max_nm: float
    tolerance_deg: float
    report: Callable[[dict], None]


def check_servo_feedback(feedback):
    if feedback['error'] or feedback['timeout'] or feedback['warning']:
        raise RuntimeError(f"舵机反馈异常（错误 {feedback['error']}，"
                           f"超时 {int(feedback['timeout'])}，告警 {feedback['warning']}）")


def check_run_feedback(feedback, first_run):
    check_servo_feedback(feedback)
    if feedback['mode'] == MODE_RUN:
        return True
    if first_run and feedback['mode'] == MODE_STOP:
        return False
    raise RuntimeError('未收到舵机运行模式反馈')


class ServoBusy(RuntimeError):
    """舵机正在执行动作，新的转动或开合不能抢占。"""


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
        initial = self._prepare_motion(kp, kd)
        return self._run_to(initial['position'] + math.radians(degrees), seconds, kp, kd, cancel, initial)

    def goto_degrees(self, target_deg, seconds, kp, kd, cancel, zero_rad, *, ramp_fraction=0.0,
                     trim_deg=0.0, open_hold=None, check_warning=False):
        """转到相对上电零点的目标角，到时或取消后停机。返回停机反馈。"""
        check_goto(target_deg, seconds)
        check_goto(target_deg + trim_deg, seconds)
        check_ramp_fraction(ramp_fraction)
        if not math.isfinite(zero_rad):
            raise RuntimeError('尚未记录上电零点')
        if open_hold is not None:
            if not 0 < open_hold.max_nm <= MAX_OPEN_HOLD_NM or not 0 < open_hold.tolerance_deg < 180:
                raise ValueError('张开保位的力矩或角度容差超出范围')
            if kp <= 0:
                raise ValueError('张开保位需要正的 gripper.kp')
        initial = self._prepare_motion(kp, kd)
        if open_hold is not None or check_warning:
            check_servo_feedback(initial)
        final = self._run_to(zero_rad + math.radians(target_deg + trim_deg), seconds, kp, kd,
                             cancel, initial, ramp_fraction, open_hold=open_hold,
                             check_warning=check_warning)
        final['target_deg'] = target_deg
        return final

    def _prepare_motion(self, kp, kd):
        # 先确认增益能编码，再停机读位置。非法增益不会先把舵机停掉。
        control_packet(self.motor_id, MODE_RUN, 1, 0, 0, 0, kp, kd)
        initial = self.stop()
        if initial['error'] or initial['timeout']:
            raise RuntimeError('舵机反馈有错误或超时，未开始转动')
        return initial

    def _run_to(self, target, seconds, kp, kd, cancel, initial, ramp_fraction=0.0, *,
                open_hold=None, check_warning=False):
        target_packet = control_packet(self.motor_id, MODE_RUN, 1, 0, 0, target, kp, kd)
        initial_position = initial['position']
        ramp_seconds = seconds * ramp_fraction
        check_output = open_hold is not None or check_warning
        started = time.monotonic()
        deadline = started + seconds
        motion_end = deadline
        if open_hold is not None:
            # 阶跃配置也先留出运动段，之后再交给独立输出轴闭环。
            motion_end = started + (ramp_seconds if ramp_seconds > 0 else 0.8 * seconds)
            initial_error = output_error_degrees(open_hold.target_deg, initial['output_encoder'], open_hold.zero_encoder)
            if abs(initial_error) <= open_hold.tolerance_deg:
                motion_end = started
        # 给最后一个周期留出发送目标角的时间，避免比例为 1 时只能在截止后到达目标。
        final_window = deadline - COMMAND_PERIOD_S
        tick = started
        previous_tick = started
        position = initial_position
        feedback = initial
        sent_target = False
        first_run = True
        received_run = False
        try:
            while not cancel.is_set() and tick < motion_end:
                position = target
                if ramp_seconds > 0 and tick < final_window:
                    progress = min((tick - started) / ramp_seconds, 1.0)
                    if progress < 1.0:
                        weight = progress * progress * (3.0 - 2.0 * progress)
                        position = initial_position + (target - initial_position) * weight
                command = target_packet if position == target else control_packet(
                    self.motor_id, MODE_RUN, 1, 0, 0, position, kp, kd)
                if cancel.is_set() or time.monotonic() >= motion_end:
                    break
                feedback = self.exchange(command)
                if check_output:
                    running = check_run_feedback(feedback, first_run)
                    received_run = received_run or running
                elif feedback['error'] or feedback['timeout']:
                    raise RuntimeError('舵机反馈错误或超时')
                first_run = False
                sent_target = sent_target or position == target
                if ramp_seconds == 0 and open_hold is None:
                    time.sleep(COMMAND_PERIOD_S)
                else:
                    next_boundary = deadline if sent_target else final_window
                    next_boundary = min(next_boundary, motion_end) if open_hold is not None else next_boundary
                    # 避免浮点舍入让整拍落在截止点前一个可表示值，额外发一帧。
                    next_period = math.nextafter(tick + COMMAND_PERIOD_S, math.inf)
                    wake = min(next_period, next_boundary)
                    cancel.wait(max(0.0, wake - time.monotonic()))
                previous_tick = tick
                tick = time.monotonic()
            if open_hold is not None and not cancel.is_set():
                self._hold_open(open_hold, position, feedback, previous_tick, started, deadline,
                                kp, kd, cancel, first_run)
        finally:
            final = self.stop()
        if check_output:
            check_servo_feedback(final)
        if check_output and open_hold is None and not received_run and not cancel.is_set():
            raise RuntimeError('未收到舵机运行模式反馈')
        if open_hold is None and ramp_seconds > 0 and not sent_target and not cancel.is_set():
            raise RuntimeError('舵机动作超时，未发送最终目标角')
        final['turned_deg'] = math.degrees(final['position'] - initial['position'])
        return final

    def _hold_open(self, hold, anchor, feedback, previous_tick, started, deadline,
                   kp, kd, cancel, first_run):
        # 梯度用独立输出轴误差。限位以电机反馈为中心，只限制位置刚度对应的力矩。
        max_position_error = hold.max_nm / kp
        stable_samples = 0
        holding = False
        outside_since = None
        capture_end = deadline + HOLD_CAPTURE_S
        hard_end = min(capture_end + HOLD_MAX_EXTENSION_S, started + MAX_MOVE_SECONDS + HOLD_CAPTURE_S)
        last_sample = time.monotonic()
        error = output_error_degrees(hold.target_deg, feedback['output_encoder'], hold.zero_encoder)
        best_error = abs(error)
        progress_candidate = None
        while not cancel.is_set():
            tick = time.monotonic()
            if (not holding and deadline <= tick <= hard_end and stable_samples >= HOLD_STABLE_SAMPLES and
                    tick - last_sample <= FEEDBACK_WAIT_S):
                holding = True
                hold.report({**feedback, 'target_deg': hold.target_deg, 'output_error_deg': error})
            if not holding and tick >= capture_end:
                raise RuntimeError('张开保位捕获超时')
            dt = min(tick - previous_tick, HOLD_MAX_STEP_S)
            if abs(error) > hold.tolerance_deg:
                speed = max(-HOLD_MAX_CORRECTION_RADPS, min(
                    HOLD_MAX_CORRECTION_RADPS, HOLD_CORRECTION_GAIN * math.radians(error)))
                anchor += speed * dt
            motor = feedback['position']
            anchor = max(motor - max_position_error, min(motor + max_position_error, anchor))
            packet = control_packet(self.motor_id, MODE_RUN, 1, 0, 0, anchor, kp, kd)
            if cancel.is_set():
                return
            if not holding and time.monotonic() >= capture_end:
                raise RuntimeError('张开保位捕获超时')
            feedback = self.exchange(packet)
            running = check_run_feedback(feedback, first_run)
            first_run = False

            now = time.monotonic()
            previous_error = error
            error = output_error_degrees(hold.target_deg, feedback['output_encoder'], hold.zero_encoder)
            within = abs(error) <= hold.tolerance_deg
            if not holding:
                if now > capture_end:
                    raise RuntimeError('张开保位捕获超时')
                if running and best_error - abs(error) >= HOLD_PROGRESS_DEG:
                    if progress_candidate is None:
                        progress_candidate = abs(error)
                    else:
                        best_error = max(progress_candidate, abs(error))
                        capture_end = min(hard_end, max(capture_end, now + HOLD_CAPTURE_S))
                        progress_candidate = None
                else:
                    progress_candidate = None
                # 电机估速与独立输出轴都要稳定。允许一个编码器计数的量化误差。
                travel = abs(math.remainder(error - previous_error, 360.0))
                quiet_travel = (math.degrees(HOLD_STABLE_SPEED_RADPS) * (now - last_sample)
                                + 360.0 / OUTPUT_COUNTS_PER_REV)
                stable = (running and within and travel <= quiet_travel
                          and abs(feedback['speed']) <= HOLD_STABLE_SPEED_RADPS)
                stable_samples = stable_samples + 1 if stable else 0
                holding = now >= deadline and stable_samples >= HOLD_STABLE_SAMPLES
            elif within:
                outside_since = None
            elif outside_since is None:
                outside_since = now
            elif now - outside_since >= HOLD_CAPTURE_S:
                raise RuntimeError(f'张开保位持续超差（{error:+.2f}°）')
            if holding and not cancel.is_set():
                hold.report({**feedback, 'target_deg': hold.target_deg, 'output_error_deg': error})
            last_sample = now
            previous_tick = tick
            wake = tick + COMMAND_PERIOD_S
            if not holding:
                boundary = deadline if now < deadline and stable_samples >= HOLD_STABLE_SAMPLES else capture_end
                wake = min(wake, boundary)
            cancel.wait(max(0.0, wake - time.monotonic()))

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
    """网页或自动夹爪的一次舵机会话。底盘停车和租约到期不会取消它；只有舵机停机或进程退出会。"""

    def __init__(self, device, motor_id, kp, kd, closed_deg=None, open_deg=None, *,
                 ramp_fraction=0.0, open_hold_max_nm=0.0, output_tolerance_deg=1.0,
                 open_trim_deg=0.0, closed_trim_deg=0.0):
        self._lock = threading.Lock()
        # 网页请求在接管串口前交接工作线程。线程只拿状态锁，退出时不依赖请求锁。
        self._requests = threading.Lock()
        self._cancel = threading.Event()
        self._finished = threading.Event()
        self._finished.set()
        self._link = None
        self._worker = None
        self._result = None
        self._error = None
        self._zero_rad = None
        self._output_zero = None
        self.busy = False
        self.holding = False
        self.position_deg = None
        self.output_position_deg = None
        self.output_error_deg = None
        self.message = '未配置舵机串口'
        self.device = device
        self.kp = kp
        self.kd = kd
        self.closed_deg = closed_deg
        self.open_deg = open_deg
        self.ramp_fraction = ramp_fraction
        self.open_hold_max_nm = open_hold_max_nm
        self.output_tolerance_deg = output_tolerance_deg
        self.open_trim_deg = open_trim_deg
        self.closed_trim_deg = closed_trim_deg
        if not device:
            return
        try:
            control_packet(motor_id, MODE_RUN, 1, 0, 0, 0, kp, kd)
            self._link = ServoLink(device, motor_id)
        except (OSError, ValueError) as error:
            self.message = str(error)
            return
        try:
            feedback = self._link.stop()
            if feedback['error'] or feedback['timeout']:
                raise RuntimeError('舵机反馈有错误或超时')
            if self.open_hold_max_nm > 0:
                check_servo_feedback(feedback)
        except (OSError, TimeoutError, RuntimeError) as error:
            self.message = f'已打开串口，但未能记录上电零点：{error}'
            return
        self._zero_rad = feedback['position']
        self._output_zero = feedback.get('output_encoder')
        self._record_feedback(feedback)
        self.message = '已连接，上电零点已记录'

    @property
    def available(self):
        return self._link is not None

    def snapshot(self):
        with self._lock:
            return {
                'available': self.available, 'busy': self.busy, 'device': self.device,
                'position_deg': self.position_deg, 'output_position_deg': self.output_position_deg,
                'output_error_deg': self.output_error_deg, 'holding': self.holding,
                'message': self.message,
            }

    def start_move(self, degrees, seconds):
        check_move(degrees, seconds)
        self._submit(f'正在转动 {degrees:g}°',
                     lambda cancel: self._link.move_degrees(degrees, seconds, self.kp, self.kd, cancel))

    def start_goto(self, target_deg, seconds, *, ramp_fraction=0.0):
        check_goto(target_deg, seconds)
        self._submit(f'正在转到 {target_deg:g}°',
                     lambda cancel: self._goto(target_deg, seconds, cancel, ramp_fraction))

    def start_open(self, seconds):
        self._start_preset(self.open_deg, self.open_trim_deg, seconds, self.open_hold_max_nm > 0)

    def start_close(self, seconds):
        self._start_preset(self.closed_deg, self.closed_trim_deg, seconds, False)

    def _start_preset(self, target_deg, trim_deg, seconds, hold_open):
        check_goto(target_deg, seconds)
        check_goto(target_deg + trim_deg, seconds)
        self._submit(f'正在转到 {target_deg:g}°',
                     lambda cancel: self._run_preset(target_deg, trim_deg, seconds, hold_open, cancel))

    def _run_preset(self, target_deg, trim_deg, seconds, hold_open, cancel):
        options = {}
        if trim_deg != 0:
            options['trim_deg'] = trim_deg
        if self.open_hold_max_nm > 0 or trim_deg != 0:
            options['check_warning'] = True
        if hold_open:
            if self._output_zero is None:
                raise RuntimeError('尚未记录独立输出轴零位')
            options['open_hold'] = OpenHold(self._output_zero, target_deg, self.open_hold_max_nm,
                                            self.output_tolerance_deg, self._report_hold)
        feedback = self._goto(target_deg, seconds, cancel, self.ramp_fraction, **options)
        if self._output_zero is not None:
            feedback['output_error_deg'] = output_error_degrees(target_deg, feedback['output_encoder'],
                                                                 self._output_zero)
        return feedback

    def move(self, degrees, seconds):
        """同步相对转动，供单元测试等待完成；网页接口走 start_move。"""
        self.start_move(degrees, seconds)
        return self.wait()

    def wait(self, timeout=None):
        with self._lock:
            finished = self._finished
        if not finished.wait(timeout):
            raise TimeoutError('等待舵机动作结束超时')
        with self._lock:
            if self._error is not None:
                raise self._error
            return self._result

    def _goto(self, target_deg, seconds, cancel, ramp_fraction, **options):
        if self._zero_rad is None:
            raise RuntimeError('尚未记录上电零点')
        return self._link.goto_degrees(target_deg, seconds, self.kp, self.kd, cancel,
                                       self._zero_rad, ramp_fraction=ramp_fraction, **options)

    def _submit(self, message, work):
        with self._requests:
            with self._lock:
                if self._link is None:
                    raise RuntimeError(self.message)
                if self.busy:
                    raise ServoBusy('舵机正在转动')
                worker = self._worker
                interrupting_hold = self.holding
                if interrupting_hold:
                    self._cancel.set()
            if worker is not None:
                worker.join()
            with self._lock:
                if interrupting_hold and self._error is not None:
                    raise RuntimeError(self.message)
                self.busy = True
                self._cancel = threading.Event()
                self._finished = threading.Event()
                self._result = None
                self._error = None
                self.output_error_deg = None
                self.message = message
                self._worker = threading.Thread(target=self._run, args=(work,), daemon=True)
                self._worker.start()

    def _run(self, work):
        try:
            feedback = work(self._cancel)
        except Exception as error:
            with self._lock:
                self.busy = False
                self.holding = False
                self._error = error
                self.message = str(error)
                self._finished.set()
            return
        with self._lock:
            self.busy = False
            self.holding = False
            self._result = feedback
            self._record_feedback(feedback)
            if 'target_deg' in feedback:
                self.message = f'已停机，位置 {self.position_deg:.2f}°'
            else:
                self.message = f"已停机，实际转动 {feedback['turned_deg']:.2f}°"
            self._finished.set()

    def _record_feedback(self, feedback):
        self.position_deg = math.degrees(feedback['position'])
        if self._output_zero is not None:
            self.output_position_deg = output_degrees(feedback['output_encoder'], self._output_zero)
        if 'output_error_deg' in feedback:
            self.output_error_deg = feedback['output_error_deg']

    def _report_hold(self, feedback):
        with self._lock:
            if self._cancel.is_set():
                return
            self._record_feedback(feedback)
            if not self.holding:
                self.holding = True
                self.busy = False
                self._result = feedback
                self.message = '张开保位中'
                self._finished.set()

    def stop(self):
        """请求正在进行的转角停机；空闲时再发一次停机命令。"""
        with self._requests:
            self._cancel.set()
            with self._lock:
                if self.busy or self.holding or self._link is None:
                    return None
            if self._worker is not None:
                self._worker.join()
            feedback = self._link.stop()
            with self._lock:
                self._record_feedback(feedback)
                self.message = '已停机'
            return feedback

    def close(self):
        with self._requests:
            self._cancel.set()
            worker = self._worker
            if worker is not None:
                worker.join()
            with self._lock:
                link = self._link
                self._link = None
                self.busy = False
                self.holding = False
            if link is not None:
                try:
                    link.stop()
                except (OSError, TimeoutError, RuntimeError):
                    pass
                link.close()
