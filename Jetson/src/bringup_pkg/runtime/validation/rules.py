# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""配置校验：数组形状、发布开关依赖与跨节点业务约束（与节点重复的范围校验交由节点）。"""
from __future__ import annotations

from pathlib import PurePosixPath
import sys

from ..core import ConfigurationError


# ---- 值规整：数组端点形状；类型与数值范围由节点 declareRequired/checked* 负责 ----

def _number(value):
    # 整数写法规整为浮点，保证 ROS 参数文件按 double 声明；其余值原样交给节点判定。
    return float(value) if type(value) is int else value


def _typed(name, value, kind):
    if kind == 'numbers':
        # ARRAY_EXPANSION 按 [lo,hi] 下标映射节点标量键，形状必须在此确认。
        if not isinstance(value, list) or len(value) != 2:
            raise ConfigurationError(f'{name}: expected [lo, hi], got {value!r}')
        return [_number(item) for item in value]
    if kind == 'number':
        return _number(value)
    return value


# ---- 接口验证：选定部署内的发布开关依赖 ----

# 只验证当前选定角色组合必须打开的发布开关；话题名本身可配置，不改写也不暗中打开开关。
def _validate_connections(values, run):
    if run not in ('all', 'vision'):
        return
    task = values['vision.detect.task']
    if (run == 'all' and task in ('both', 'field')
            and not values['vision.detect.publish_field_results']):
        raise ConfigurationError('mission.safety.input_topic requires vision.detect.publish_field_results')
    output = any(values['vision.stream.' + key] for key in
                 ('stream_enabled', 'publish_annotated', 'publish_compressed'))
    if not output:
        return
    image = values['vision.stream.image_topic']
    publish = ('publish_processed' if image.endswith('/image_processed') else
               'publish_raw' if image.endswith('/image_raw') else None)
    if publish and not values['vision.detect.' + publish]:
        raise ConfigurationError(f'vision.stream.image_topic requires vision.detect.{publish}')
    if (task in ('both', 'seg') and values['vision.stream.draw_masks']
            and not values['vision.detect.publish_masks']):
        raise ConfigurationError('vision.stream.draw_masks requires vision.detect.publish_masks')
    if (task in ('both', 'field') and values['vision.stream.draw_field']
            and not values['vision.detect.publish_field_results']):
        raise ConfigurationError('vision.stream.draw_field requires vision.detect.publish_field_results')


# ---- 业务验证：超时与通信资源 ----

def _validate_timeouts(values):
    for name, value in values.items():
        key = name.rsplit('.', 1)[-1]
        if key.endswith('_timeout_s'):
            limit = sys.float_info.max / 1e9 if key == 'reset_timeout_s' else 3600
            if not 0 < value <= limit:
                raise ConfigurationError(f'{name}: expected (0, {limit}], got {value}')
    # 三层等待独立计时，外层更长以留出下游正常失败应答；不可合并成一个期限。
    roi = values['mission.target.roi_check_timeout_s']
    mission = values['mission.controller.request_timeout_s']
    bridge = values['bridge.request_timeout_s']
    if not roi < mission < bridge:
        raise ConfigurationError('timeouts must satisfy Target ROI < Mission request < Bridge request')


def _validate_bridge(values):
    """校验 USB 设备路径；配置检查不打开设备。"""
    # Linux 设备名在 Windows 离线检查时也按 POSIX 解析，保留 by-id 符号链接原文。
    _validate_device_path('link.device', values['link.device'], allow_empty=False)

def _validate_device_path(name, device, allow_empty):
    if allow_empty and device == '':
        return
    parts = PurePosixPath(device).parts
    if (not isinstance(device, str) or len(parts) < 3 or parts[:2] != ('/', 'dev') or '..' in parts or
            any(not 0x21 <= ord(char) <= 0x7e for char in device)):
        raise ConfigurationError(f'{name}: expected printable ASCII /dev/ device path without whitespace or ..')


def _optional_number(name, value, minimum=None):
    if value is None:
        return
    if type(value) is bool or not isinstance(value, (int, float)) or value != value or value in (float('inf'), float('-inf')):
        raise ConfigurationError(f'{name}: expected a finite number or empty, got {value!r}')
    if minimum is not None and value <= minimum:
        raise ConfigurationError(f'{name}: expected > {minimum}, got {value}')


# 单次开合时长范围（秒），须与 bridge_pkg/gripper/s288_servo.hpp 及 teleop_pkg/servo/s288.py 一致。
GRIPPER_MOVE_S_MIN = 0.2
GRIPPER_MOVE_S_MAX = 10.0


def _validate_gripper(values, run):
    _validate_device_path('gripper.device', values['gripper.device'], allow_empty=True)
    ident = values['gripper.id']
    if type(ident) is bool or not isinstance(ident, int) or not 0 <= ident <= 14:
        raise ConfigurationError(f'gripper.id: expected integer in [0, 14], got {ident!r}')
    for name in ('gripper.kp', 'gripper.kd'):
        value = values[name]
        if type(value) is bool or not isinstance(value, (int, float)) or value != value or value < 0:
            raise ConfigurationError(f'{name}: expected a finite number ≥ 0, got {value!r}')
    for name in ('gripper.closed_deg', 'gripper.open_deg'):
        value = values[name]
        if type(value) is bool or not isinstance(value, (int, float)) or value != value or value in (float('inf'), float('-inf')):
            raise ConfigurationError(f'{name}: expected a finite number, got {value!r}')
    move_s = values['gripper.move_s']
    _optional_number('gripper.move_s', move_s)
    if move_s is not None and not GRIPPER_MOVE_S_MIN <= move_s <= GRIPPER_MOVE_S_MAX:
        raise ConfigurationError(
            f'gripper.move_s: expected [{GRIPPER_MOVE_S_MIN:g}, {GRIPPER_MOVE_S_MAX:g}], got {move_s}')
    # 自动路径在 bridge 角色启用时拒绝空键；手动 teleop 不走本入口。
    if run in ('all', 'bridge'):
        missing = [name for name, filled in (
            ('gripper.device', bool(values['gripper.device'])),
            ('gripper.move_s', values['gripper.move_s'] is not None),
        ) if not filled]
        if missing:
            raise ConfigurationError(f"{'、'.join(missing)}: 待实车标定")


def _validate(values, run):
    # 唯一业务校验入口，由 load_configuration 调用一次；校验顺序决定首个报错。
    _validate_timeouts(values)
    _validate_bridge(values)
    _validate_gripper(values, run)
    _validate_connections(values, run)
