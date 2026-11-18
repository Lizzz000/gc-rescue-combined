"""手动速度指令：限幅、租约与停车序列。纯状态，时间由调用方传入。"""

import enum
import math
import threading

# 与 STM32 手动控制的运行限幅一致，平移按合速度计算。
MAX_LINEAR_MPS = 0.50
MAX_ANGULAR_RADPS = 4.0
# 租约上限须小于 STM32 手动帧 250 ms 超时。
MAX_LEASE_S = 0.2
# 首次接管或显式停车后，先发送一段有效零速。持续使能期间的新方向可直接接续。
DEFAULT_ARM_ZERO_S = 0.1
MAX_ARM_ZERO_S = 1.0
# 停车：先发零速 valid=1，再转入 valid=0。
STOP_ZERO_S = 0.3
MAX_TIMED_S = 3600.0
_SCALE = 1000


class Phase(enum.Enum):
    WAITING = 'waiting'    # 启动后等待首次接管，不给底盘发送撤权或运动指令
    IDLE = 'idle'          # valid=0
    PREPARE = 'prepare'    # 使能零速 valid=1，之后进入 ACTIVE
    ACTIVE = 'active'      # 指令速度 valid=1
    HOLDING = 'holding'    # 正常松键或定时结束，继续发送有效零速
    STOPPING = 'stopping'  # 零速 valid=1，之后进入 IDLE


def _finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label}须为有限数值')
    try:
        value = float(value)
    except OverflowError as error:
        raise ValueError(f'{label}须为有限数值') from error
    if not math.isfinite(value):
        raise ValueError(f'{label}须为有限数值')
    return value


def _positive_limit(value, maximum, label):
    value = _finite_number(value, label)
    if not 0.0 < value <= maximum:
        raise ValueError(f'{label}须在 (0, {maximum:g}] 内')
    return value


def quantize(vx, vy, wz, *, max_linear_mps=MAX_LINEAR_MPS, max_angular_radps=MAX_ANGULAR_RADPS):
    """限幅后向零截断到 ×1000 整数，保证量化值仍不超过 STM32 阈值。"""
    vx, vy, wz = (_finite_number(value, '速度') for value in (vx, vy, wz))
    max_linear_mps = _positive_limit(max_linear_mps, MAX_LINEAR_MPS, '平移上限')
    max_angular_radps = _positive_limit(max_angular_radps, MAX_ANGULAR_RADPS, '转向上限')
    linear = math.hypot(vx, vy)
    if linear > max_linear_mps:
        vx, vy = vx * max_linear_mps / linear, vy * max_linear_mps / linear
    wz = max(-max_angular_radps, min(max_angular_radps, wz))
    return tuple(math.trunc(value * _SCALE) for value in (vx, vy, wz))


class ManualCommand:
    """按键续租，定时动作到期和正常松键都保持零速。急停和按键租约超时撤权。"""

    def __init__(self, lease_s, arm_zero_s, *, max_linear_mps=MAX_LINEAR_MPS,
                 max_angular_radps=MAX_ANGULAR_RADPS, max_timed_s=MAX_TIMED_S):
        self._lease_s = _positive_limit(lease_s, MAX_LEASE_S, 'lease_s')
        self._arm_zero_s = _positive_limit(arm_zero_s, MAX_ARM_ZERO_S, 'arm_zero_s')
        self._max_linear_mps = _positive_limit(max_linear_mps, MAX_LINEAR_MPS, '平移上限')
        self._max_angular_radps = _positive_limit(max_angular_radps, MAX_ANGULAR_RADPS, '转向上限')
        self._max_timed_s = _positive_limit(max_timed_s, MAX_TIMED_S, '定时上限')
        self._lock = threading.Lock()
        self._phase = Phase.WAITING
        self._timed = False
        self._phase_until = 0.0
        self._velocity = (0, 0, 0)
        self._deadline = 0.0

    @property
    def limits(self):
        return {'linear_mps': self._max_linear_mps, 'angular_radps': self._max_angular_radps,
                'timed_s': self._max_timed_s}

    def _quantize(self, vx, vy, wz):
        return quantize(vx, vy, wz, max_linear_mps=self._max_linear_mps,
                        max_angular_radps=self._max_angular_radps)

    def hold(self, vx, vy, wz, now):
        """按住中的速度；租约 lease_s 内未续期即停车。"""
        self._start(self._quantize(vx, vy, wz), self._lease_s, now)

    def timed(self, vx, vy, wz, seconds, now):
        """定时动作：持续 seconds 秒，不需要续期。"""
        seconds = _positive_limit(seconds, self._max_timed_s, 'seconds')
        self._start(self._quantize(vx, vy, wz), seconds, now, timed=True)

    def stop(self, now):
        with self._lock:
            self._enter_stop(now)

    def release(self, now):
        """只结束当前运动。丢失租约或已显式撤权的会话不会被迟到的松键恢复。"""
        with self._lock:
            self._advance(now)
            if self._phase in (Phase.PREPARE, Phase.ACTIVE):
                self._phase = Phase.HOLDING
                self._velocity = (0, 0, 0)

    def frame(self, now):
        """推进阶段并返回本拍速度帧 (valid, vx, vy, wz)。尚未接管时返回 None。"""
        with self._lock:
            self._advance(now)
            if self._phase is Phase.WAITING:
                return None
            if self._phase is Phase.IDLE:
                return (False, 0, 0, 0)
            if self._phase is Phase.ACTIVE:
                return (True, *self._velocity)
            return (True, 0, 0, 0)

    def snapshot(self, now):
        with self._lock:
            self._advance(now)
            return {'phase': self._phase.value, 'velocity_milli': list(self._velocity),
                    'remaining_s': max(0.0, self._deadline - now)
                    if self._phase in (Phase.PREPARE, Phase.ACTIVE) else 0.0}

    def _start(self, velocity, duration, now, *, timed=False):
        with self._lock:
            self._advance(now)
            if self._phase in (Phase.WAITING, Phase.IDLE, Phase.STOPPING):
                self._phase, self._phase_until = Phase.PREPARE, now + self._arm_zero_s
            elif self._phase is Phase.HOLDING:
                self._phase = Phase.PREPARE if now < self._phase_until else Phase.ACTIVE
            self._velocity = velocity
            self._timed = timed
            # 定时动作只补当前剩余的使能时间；hold 租约仍从收到请求时起算。
            start = self._phase_until if timed and self._phase is Phase.PREPARE else now
            self._deadline = start + duration

    def _enter_stop(self, now):
        if self._phase is Phase.WAITING:
            self._phase = Phase.IDLE
        elif self._phase in (Phase.PREPARE, Phase.ACTIVE, Phase.HOLDING):
            self._phase, self._phase_until = Phase.STOPPING, now + STOP_ZERO_S
        self._velocity = (0, 0, 0)

    def _advance(self, now):
        if self._phase in (Phase.PREPARE, Phase.ACTIVE) and now >= self._deadline:
            if self._timed:
                self._phase = Phase.HOLDING
                self._velocity = (0, 0, 0)
            else:
                self._enter_stop(now)
        if self._phase is Phase.PREPARE and now >= self._phase_until:
            self._phase = Phase.ACTIVE
        if self._phase is Phase.STOPPING and now >= self._phase_until:
            self._phase = Phase.IDLE
