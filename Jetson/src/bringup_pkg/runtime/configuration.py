# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""统一配置装配：读取互补文件、验证完整值、映射节点参数，返回运行快照。

read_configuration 供编辑工具读取原值；load_configuration 返回已验证的运行快照；
物化与 launch 装配均见 ros/launch.py。
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Callable

# ROLES、FULL_ROLES、ConfigurationError 由本模块再导出，工具通过 cfg.ROLES 等访问，勿当作未用导入删除。
from .core import FIELDS, FULL_ROLES, NODE_PARAMETERS, ROLES, ConfigurationError  # noqa: F401
from .ros.launch import _parameter_value
from .validation.rules import _typed, _validate
from .yaml_io.reader import _flatten, _package_directory, _read_yaml, resolve_path


def _read_configuration(config_file, package_resolver):
    """只接受一层显式引用；两文件提供互补字段，绝不按读取顺序覆盖。"""
    path = Path(config_file).resolve()
    document, digest = _read_yaml(path)
    sources = [{'role': 'basic', 'path': str(path), 'sha256': digest}]
    # 元数据不传给节点，也不计入有效参数身份；完整归档快照可以不引用高级文件。
    if 'advanced_config' in document:
        reference = document.pop('advanced_config')
        if not isinstance(reference, str) or not reference.strip():
            raise ConfigurationError('advanced_config: expected a non-empty path string')
        advanced_path = Path(resolve_path(reference, path, package_resolver))
        advanced, advanced_digest = _read_yaml(advanced_path)
        advanced_values = _flatten(advanced)
        sources.append({'role': 'advanced', 'path': str(advanced_path), 'sha256': advanced_digest})
    else:
        advanced_path, advanced_values = None, {}
    basic_values = _flatten(document)
    repeated = basic_values.keys() & advanced_values.keys()
    if repeated:
        raise ConfigurationError(f'configuration fields defined in both files: {sorted(repeated)}')
    values = {**basic_values, **advanced_values}
    unknown, missing = set(values) - set(FIELDS), set(FIELDS) - set(values)
    if unknown or missing:
        raise ConfigurationError(f'configuration fields: unknown={sorted(unknown)}, missing={sorted(missing)}')
    origins = {name: str(path) for name in basic_values}
    origins.update({name: str(advanced_path) for name in advanced_values})
    return values, origins, sources


def read_configuration(config_file, package_resolver: Callable | None = None):
    """给文档/编辑工具读取完整原始值；运行、校验和导出须用 load_configuration。

    不展开资源路径，不创建 ROS 或设备资源。需要另存可运行快照时，应使用
    load_configuration() 返回快照的 values 中已按各自来源文件解析的绝对路径。
    """
    values, _, _ = _read_configuration(config_file, package_resolver or _package_directory)
    document = {}
    for name, value in values.items():
        cursor = document
        parts = name.split('.')
        for part in parts[:-1]:
            cursor = cursor.setdefault(part, {})
        cursor[parts[-1]] = value
    return document


# ---- 运行装配：读取 → 验证 → 路径/后端检查 → 节点映射 → 快照 ----

def load_configuration(config_file, run='all', package_resolver: Callable | None = None,
                       require_assets=True):
    # 读取：先限定角色，再合并完整字段并保留每个字段的来源文件。
    if run != 'all' and run not in ROLES:
        raise ConfigurationError(f'run: expected all or one of {tuple(ROLES)}, got {run!r}')
    path = Path(config_file).resolve()
    package_resolver = package_resolver or _package_directory
    values, origins, sources = _read_configuration(path, package_resolver)
    # 验证：唯一校验入口；单角色同样执行全部业务校验，不从节点默认值补缺项。
    values = {name: _typed(name, value, FIELDS[name]) for name, value in values.items()}
    _validate(values, run)
    # 资源：按字段来源解析路径；文件存在性由 detect_node 打开时确认。
    # require_assets 仅为调用方兼容保留，已不再触发任何检查。
    for name in ('vision.detect.seg_model_path', 'vision.detect.seg_labels_path'):
        values[name] = resolve_path(values[name], origins[name], package_resolver)
    if values['vision.detect.camera_backend'] == 'file':
        name = 'vision.detect.camera_device'
        values[name] = resolve_path(values[name], origins[name], package_resolver)
    roles = FULL_ROLES if run == 'all' else (run,)
    selected = tuple(node for role in roles for node in ROLES[role][2])
    # GPU 需求由实际选定角色与可用算法决定；不会根据环境缺能力自动切 CPU 或构建引擎。
    gpu = 'detect_node' in selected and values['vision.detect.task'] in ('seg', 'both')
    # JPEG 硬件能力独立于 TensorRT；关闭两种 JPEG 输出时不要求编码设备。
    hardware_jpeg = ('stream_node' in selected and values['vision.stream.jpeg_backend'] == 'nvjpeg'
                     and (values['vision.stream.stream_enabled'] or values['vision.stream.publish_compressed']))
    if gpu and Path(values['vision.detect.seg_model_path']).suffix.lower() not in ('.trt', '.engine'):
        raise ConfigurationError('vision.detect.seg_model_path: existing TRT engine required; building is disabled')
    # 映射：只为选定节点生成参数，数组端点在此转换为节点既有标量键。
    nodes = {node: {name: _parameter_value(values, entry) for name, entry in NODE_PARAMETERS[node].items()}
             for node in selected}
    snapshot = {'configuration': str(path), 'configuration_sha256': sources[0]['sha256'],
                'configuration_sources': sources,
                'run': run, 'deployment': 'vision_fast' if run == 'all' else 'development',
                'roles': list(roles), 'nodes': nodes, 'values': values,
                'required_backends': ['cpu'] + (['tensorrt_gpu'] if gpu else [])
                                     + (['jetson_jpeg'] if hardware_jpeg else [])}
    # 两份文件分别留指纹；有效身份只由解析后的值和节点参数决定（H04 注入 RESCUE_EFFECTIVE_SHA256）。
    identity = {'nodes': nodes, 'values': values}
    snapshot['effective_sha256'] = hashlib.sha256(json.dumps(
        identity, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()).hexdigest()
    return snapshot
