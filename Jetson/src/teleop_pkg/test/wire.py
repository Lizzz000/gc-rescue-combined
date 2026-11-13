"""手动链路测试的公共工具：pty 抓包、USB 分帧解码、周期统计、STM32 常量读取与回放。

不依赖 ROS；STM32 常量从仓库里的头文件/源文件解析，不在测试里写死。
"""

import os
from pathlib import Path
import pty
import re
import select
import shutil
import statistics
import struct
import subprocess
import tempfile
import threading
import time

from manual.frame import Parser

TELEOP = Path(__file__).resolve().parents[1]
REPO = TELEOP.parents[2]
STM32_CORE = REPO / 'STM32/project/Core'
STM32_STUBS = REPO / 'STM32/tests/native/stubs'
REPLAY_SOURCE = Path(__file__).resolve().parent / 'stm32_manual_replay.c'




class Frame(tuple):
    """(t, valid, vx, vy, wz)：t 为抓到该帧的单调时间（秒）。"""

    __slots__ = ()

    def __new__(cls, t, valid, vx, vy, wz):
        return super().__new__(cls, (t, valid, vx, vy, wz))

    t = property(lambda self: self[0])
    valid = property(lambda self: self[1])
    velocity = property(lambda self: self[2:])


def decode_packet(packet, identifier, length):
    """解出一帧 (valid, vx, vy, wz)；分帧、类型或长度不符抛 ValueError。"""
    frames = Parser().feed(packet)
    if len(frames) != 1 or frames[0][0] != identifier or len(frames[0][1]) != length:
        raise ValueError(f'分帧不符：{packet.hex()}')
    return struct.unpack('<Bhhh', frames[0][1])


class PtyCapture:
    """打开一对 pty；后台线程读主端，每个完整 USB 分帧记一个时间戳。"""

    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.device = os.ttyname(self.slave)
        self.packets = []           # (t, id, payload)
        self._parser = Parser()
        self._running = True
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        while self._running:
            if not select.select([self.master], [], [], 0.005)[0]:
                continue
            try:
                data = os.read(self.master, 8192)
            except OSError:
                return
            now = time.monotonic()
            for identifier, payload in self._parser.feed(data):
                self.packets.append((now, identifier, payload))

    def frames(self, identifier, length):
        return [Frame(t, *struct.unpack('<Bhhh', payload))
                for t, got, payload in list(self.packets)
                if got == identifier and len(payload) == length]

    def close(self):
        self._running = False
        self._thread.join(timeout=1)
        os.close(self.slave)
        os.close(self.master)


def periods_ms(frames):
    return [(b.t - a.t) * 1000.0 for a, b in zip(frames, frames[1:])]


def percentile(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def period_stats(frames):
    """返回 (中位 ms, p99 ms, 最大 ms, 样本数)。"""
    p = periods_ms(frames)
    return statistics.median(p), percentile(p, 0.99), max(p), len(p)


# ---------------- STM32 常量 ----------------

def _define(path, name):
    text = path.read_text(encoding='utf-8')
    match = re.search(rf'#define\s+{name}\s+\(?\s*([0-9.]+)[uUfF]*', text)
    if not match:
        raise LookupError(f'{path} 中没有 {name}')
    return float(match.group(1))


def stm32_available():
    return (STM32_CORE / 'Control/Manual/manual_control.h').exists()


def stm32_constants():
    """手动固件的超时、限速与任务周期；任务周期取 chassis_task.c 的 vTaskDelayUntil 参数。"""
    task = (STM32_CORE / 'Task/Chassis/chassis_task.c').read_text(encoding='utf-8')
    period = re.search(r'vTaskDelayUntil\([^,]+,\s*pdMS_TO_TICKS\((\d+)U?\)\)', task)
    return {
        'timeout_ms': _define(STM32_CORE / 'Control/Manual/manual_control.h', 'MANUAL_VELOCITY_TIMEOUT_MS'),
        'max_linear_mps': _define(STM32_CORE / 'Control/Chassis/chassis.h', 'CHASSIS_STRAIGHT_MAX_SPEED_MPS'),
        'max_angular_radps': _define(STM32_CORE / 'Control/Chassis/chassis.h', 'CHASSIS_VECTOR_MAX_WZ_RAD_S'),
        'task_period_ms': float(period.group(1)) if period else None,
    }


# ---------------- STM32 回放 ----------------

def build_replay():
    """用主机 gcc 编译回放程序；没有 gcc 或 STM32 源码时返回 None。"""
    cc = shutil.which('gcc') or shutil.which('cc')
    if not cc or not stm32_available():
        return None
    out = Path(tempfile.mkdtemp(prefix='gc_manual_replay_')) / 'replay'
    includes = [STM32_STUBS, STM32_CORE / 'Communication/Jetson', STM32_CORE / 'Communication/Manual',
                STM32_CORE / 'Control/Manual', STM32_CORE / 'Control/Chassis']
    sources = [REPLAY_SOURCE, STM32_CORE / 'Communication/Manual/manual_link.c',
               STM32_CORE / 'Communication/Jetson/jetson_link.c', STM32_CORE / 'Control/Manual/manual_control.c']
    command = [cc, '-std=c11', '-O1', *[f'-I{p}' for p in includes], *map(str, sources), '-lm', '-o', str(out)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    return out


def replay(binary, frames, origin, end_s, task_period_ms):
    """按相对 origin 的毫秒回放；返回 [(秒, 状态名)]（相对 origin）。"""
    lines = [f'{max(1, round((f.t - origin) * 1000))} {f.valid} {f[2]} {f[3]} {f[4]}' for f in frames]
    lines.append(f'end {round(end_s * 1000)}')
    result = subprocess.run([str(binary), str(int(task_period_ms))], input='\n'.join(lines) + '\n',
                            capture_output=True, text=True, check=True)
    states = []
    for row in result.stdout.split('\n'):
        if row.strip():
            ms, name = row.split()
            states.append((int(ms) / 1000.0, name))
    return states
