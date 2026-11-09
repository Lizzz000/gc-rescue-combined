# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""ROS 文件发现入口；配置、校验与角色装配统一实现在 runtime 包（ros/launch.py）。

阅读：这是整车入口。角色表在 runtime/core.py 的 ROLES。
下一份是 runtime/ros/launch.py，它按角色启动四个 runtime 进程。
"""
import importlib
import importlib.util
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory

# 从同一安装包按包加载唯一实现，避免入口与配置模块各自维护校验或角色列表。
_root = Path(get_package_share_directory('bringup_pkg')) / 'runtime'
_spec = importlib.util.spec_from_file_location('bringup_runtime', _root / '__init__.py',
                                               submodule_search_locations=[str(_root)])
_runtime = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _runtime
_spec.loader.exec_module(_runtime)
generate_launch_description = importlib.import_module('bringup_runtime.ros.launch').generate_launch_description
