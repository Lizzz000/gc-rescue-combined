# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""视觉进程取流前，向 USB 摄像头写入本场参数并回读。"""
import subprocess
import sys

from .core import ConfigurationError


CONTROLS_PREFIX = 'vision.detect.camera_controls.'
_MODE_CONTROLS = ('auto_exposure', 'white_balance_automatic', 'focus_automatic_continuous')


def apply_camera_controls(snapshot):
    camera = snapshot['nodes'].get('detect_node')
    if (not sys.platform.startswith('linux') or camera is None
            or camera['camera_backend'] != 'gstreamer' or camera['camera_pipeline']):
        return {}

    controls = {}
    if camera['camera_brightness'] != -1:
        controls['brightness'] = camera['camera_brightness']
    for key, value in snapshot['values'].items():
        if key.startswith(CONTROLS_PREFIX):
            controls[key[len(CONTROLS_PREFIX):]] = value
    if not controls:
        return {}

    device = camera['camera_device']
    command = ['v4l2-ctl', '-d', device]
    # 手动曝光模式生效后才能写曝光时间。以两次驱动调用明确先后顺序。
    modes = {name: controls[name] for name in _MODE_CONTROLS if name in controls}
    settings = {name: value for name, value in controls.items() if name not in modes}
    try:
        for group in (modes, settings):
            if group:
                values = ','.join(f'{name}={value}' for name, value in group.items())
                subprocess.run(command + ['--set-ctrl=' + values], check=True,
                               capture_output=True, text=True, timeout=5)
        result = subprocess.run(command + ['--get-ctrl=' + ','.join(controls)], check=True,
                                capture_output=True, text=True, timeout=5)
    except subprocess.CalledProcessError as error:
        raise ConfigurationError(f'{device} 相机参数设置或回读失败：'
                                 f'{error.stderr.strip() or error.stdout.strip() or error}') from error
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ConfigurationError(f'{device} 无法执行 v4l2-ctl：{error}') from error

    observed = {}
    for line in result.stdout.splitlines():
        # 菜单型控件可能在整数后附带模式名称，例如 1 (Manual Mode)。
        name, _, value = line.strip().partition(':')
        words = value.split()
        if name in controls and words and words[0].lstrip('-').isdigit():
            observed[name] = int(words[0])
    for name, wanted in controls.items():
        if observed.get(name) != wanted:
            raise ConfigurationError(f'{device} {name} 期望 {wanted}，回读 {observed.get(name)}')
    return observed
