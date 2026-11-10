"""手动速度指令：限幅、租约与停车序列。纯状态，时间由调用方传入。"""

import enum
import math
import threading

# 与 STM32 jetson_control.c 的超限制动阈值一致：平移按 hypot(vx, vy)，角速度按 |wz|。
MAX_LINEAR_MPS = 0.5
MAX_ANGULAR_RADPS = 2.0
# 租约上限须小于 STM32 手动帧 250 ms 超时。
MAX_LEASE_S = 0.2
# STM32 使能握手要求先收到零速；从空闲开始运动前发零速的时长（沿用原 move_timed）。
PREPARE_S = 0.5
# 停车：先发零速 valid=1，再转入 valid=0。
STOP_ZERO_S = 0.3
MAX_TIMED_S = 3600.0
_SCALE = 1000


class Phase(enum.Enum):
    IDLE = 'idle'          # valid=0
    PREPARE = 'prepare'    # 零速 valid=1，之后进入 ACTIVE
    ACTIVE = 'active'      # 指令速度 valid=1
    STOPPING = 'stopping'  # 零速 valid=1，之后进入 IDLE


def quantize(vx, vy, wz):
    """限幅后向零截断到 ×1000 整数，保证量化值仍不超过 STM32 阈值。"""
    for value in (vx, vy, wz):
        if not math.isfinite(value):
            raise ValueError('速度必须是有限数')
    linear = math.hypot(vx, vy)
    if linear > MAX_LINEAR_MPS:
        vx, vy = vx * MAX_LINEAR_MPS / linear, vy * MAX_LINEAR_MPS / linear
    wz = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, wz))
    return tuple(math.trunc(value * _SCALE) for value in (vx, vy, wz))


class ManualCommand:
    """线程安全的指令状态。hold 续租约；timed 定时；stop 或过期进入停车序列。"""

    def __init__(self, lease_s):
        if not 0.0 < lease_s <= MAX_LEASE_S:
            raise ValueError(f'lease_s 须在 (0, {MAX_LEASE_S}] 秒内')
        self._lease_s = lease_s
        self._lock = threading.Lock()
        self._phase = Phase.IDLE
        self._phase_until = 0.0
        self._velocity = (0, 0, 0)
        self._deadline = 0.0

    def hold(self, vx, vy, wz, now):
        """按住中的速度；租约 lease_s 内未续期即停车。"""
        self._start(quantize(vx, vy, wz), now + self._lease_s, now)

    def timed(self, vx, vy, wz, seconds, now):
        """定时动作：持续 seconds 秒，不需要续期。"""
        if not 0.0 < seconds <= MAX_TIMED_S:
            raise ValueError(f'seconds 须在 (0, {MAX_TIMED_S:g}] 秒内')
        velocity = quantize(vx, vy, wz)
        with self._lock:
            # 定时从运动开始计时，准备阶段不占用时长。
            prepare = PREPARE_S if self._phase in (Phase.IDLE, Phase.PREPARE) else 0.0
        self._start(velocity, now + prepare + seconds, now)

    def stop(self, now):
        with self._lock:
            self._enter_stop(now)

    def frame(self, now):
        """返回本拍应发送的 (valid, vx, vy, wz)，并推进阶段。"""
        with self._lock:
            self._advance(now)
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

    def _start(self, velocity, deadline, now):
        with self._lock:
            self._advance(now)
            if self._phase is Phase.IDLE:
                self._phase, self._phase_until = Phase.PREPARE, now + PREPARE_S
            elif self._phase is Phase.STOPPING:
                self._phase = Phase.ACTIVE
            self._velocity = velocity
            self._deadline = deadline

    def _enter_stop(self, now):
        if self._phase in (Phase.PREPARE, Phase.ACTIVE):
            self._phase, self._phase_until = Phase.STOPPING, now + STOP_ZERO_S
        self._velocity = (0, 0, 0)

    def _advance(self, now):
        if self._phase in (Phase.PREPARE, Phase.ACTIVE) and now >= self._deadline:
            self._enter_stop(now)
        if self._phase is Phase.PREPARE and now >= self._phase_until:
            self._phase = Phase.ACTIVE
        if self._phase is Phase.STOPPING and now >= self._phase_until:
            self._phase = Phase.IDLE
