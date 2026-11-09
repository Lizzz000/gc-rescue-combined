# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""schema 常量、角色表与配置错误类型，供各子模块共用。"""
from __future__ import annotations

import json
from pathlib import Path

# schema 登记类型、路径、数组端点与范围；生产取值由基础和高级 YAML 互补提供。
_SCHEMA = json.loads((Path(__file__).parent / 'schema.json').read_text(encoding='utf-8'))
NODE_PARAMETERS = _SCHEMA['parameters']
FIELDS = _SCHEMA['schema']
ARRAY_EXPANSION = _SCHEMA['array_expansion']
# 阅读：ROLES 是唯一角色表。完整部署按 FULL_ROLES 拉起七个进程。
# vision 出检测和场地，track 出轨迹，select 选一个目标，
# mission 编排阶段，target 检查 ROI，safety 给出目的区，bridge 发给 STM32。
# 下一份：ros/launch.py。各进程再打开对应包的 app 入口。
# 角色映射到包、业务程序和节点；vision 在同一进程装配 Detect/Stream。
ROLES = {
    'vision': ('vision_pkg', 'vision_runtime', ('detect_node', 'stream_node')),
    'detect': ('vision_pkg', 'vision_runtime', ('detect_node',)),
    'stream': ('vision_pkg', 'vision_runtime', ('stream_node',)),
    'track': ('track_pkg', 'tracking_runtime', ('track_node',)),
    'select': ('track_pkg', 'tracking_runtime', ('select_node',)),
    'mission': ('mission_pkg', 'mission_runtime', ('mission_node',)),
    'target': ('mission_pkg', 'mission_runtime', ('target_node',)),
    'safety': ('mission_pkg', 'mission_runtime', ('safety_node',)),
    'bridge': ('bridge_pkg', 'bridge_runtime', ('bridge_node',)),
}
# 完整部署是七个业务进程：vision 一个进程含 Detect/Stream，其余角色各一个进程。
FULL_ROLES = ('vision', 'track', 'select', 'mission', 'target', 'safety', 'bridge')


class ConfigurationError(ValueError):
    """启动前可定位到具体字段的配置错误。"""
