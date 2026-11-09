# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""ROS 装配：参数物化（参数文件与有效配置）及正式启动入口。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile

import yaml

from ..core import ROLES
from ..yaml_io.reader import _package_directory


def _parameter_value(values, mapping):
    """ROS仍接收原标量键；仅在装配边界按数组索引取值。"""
    value = values[mapping['path']]
    return value[mapping['index']] if 'index' in mapping else value


# ---- 物化：将已装配快照写为唯一参数文件与有效配置记录 ----

class _RosParameterDumper(yaml.SafeDumper):
    # ROS参数解析器不接受YAML锚点；共享类别数组仍须按节点独立写出。
    def ignore_aliases(self, _value):
        return True


def dump_ros_parameters(nodes):
    return yaml.dump({node: {'ros__parameters': values} for node, values in nodes.items()},
                     Dumper=_RosParameterDumper, sort_keys=False, allow_unicode=True)


def materialize(snapshot, directory):
    """记录完整有效值并生成唯一ROS参数文件，不再由子launch叠加配置。"""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    # 全部节点只消费这份装配后的参数文件，快照同时记录解析路径和实际后端需求。
    parameters = directory / 'ros-parameters.yaml'
    parameters.write_text(dump_ros_parameters(snapshot['nodes']), encoding='utf-8')
    (directory / 'effective-configuration.json').write_text(
        json.dumps(snapshot, indent=2, ensure_ascii=False), encoding='utf-8')
    return parameters


# ---- 正式启动入口：与配置实现合并，launch 文件只绑定 ROS 发现入口 ----

def _start(context):
    """经 load_configuration 验证配置一次，再生成快照并装配角色；失败时不创建业务资源。

    context 为 ROS LaunchContext，读取 config/run。返回 launch action 列表；
    配置错误直接抛出，参数文件和节点均在完整校验成功后创建。
    """
    from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    # 延迟导入：configuration 依赖本模块的 _parameter_value，顶层导入会循环。
    from ..configuration import load_configuration

    snapshot = load_configuration(LaunchConfiguration('config').perform(context),
                                  LaunchConfiguration('run').perform(context))
    directory = os.environ.get('RESCUE_RUN_DIR') or tempfile.mkdtemp(prefix='rescue-run-')
    parameter_file = materialize(snapshot, directory)
    actions = [LogInfo(msg='Effective configuration: ' + str(Path(directory) / 'effective-configuration.json'))]
    for role in snapshot['roles']:
        package, executable, _ = ROLES[role]
        node = Node(
            package=package, executable=executable, arguments=['--role', role],
            parameters=[str(parameter_file)], output='screen',
            additional_env={'RESCUE_EFFECTIVE_SHA256': snapshot['effective_sha256']})
        actions.append(node)
        if role == 'bridge':
            # bridge 独占 CAN 串口；它退出（含串口被占用）后其余节点无法控制底盘，整体关闭 launch。
            reason = 'bridge process exited; shutting down rescue launch'
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=node,
                on_exit=[LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))])))
    return actions


def generate_launch_description():
    """声明唯一的 config/run 启动入口；实际校验由 _start 在资源创建前完成。"""
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    default = str(Path(_package_directory('bringup_pkg')) / 'config/rescue.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default,
                              description='基础配置路径（通过advanced_config引用高级文件）或完整生效快照'),
        DeclareLaunchArgument('run', default_value='all',
                              description='all、vision，或detect/stream/track/select/mission/target/safety/bridge'),
        OpaqueFunction(function=_start),
    ])
