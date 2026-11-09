# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""YAML 读取：防重复键、字段展平、读文件与 sha256、资源路径解析。"""
from __future__ import annotations

import hashlib
from pathlib import Path

import yaml

from ..core import ConfigurationError


class _UniqueLoader(yaml.SafeLoader):
    pass


# 在构造映射时拒绝重复键，避免 YAML 后项静默覆盖前项而造成生效参数不明确。
def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str) or key in result:
            raise ConfigurationError(f'configuration key must be unique text: {key!r}')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


# 展平只生成规范路径，不建立旧字段别名；字段齐全与否由未知/缺失字段检查确认。
def _flatten(document, prefix=''):
    result = {}
    for key, value in document.items():
        name = f'{prefix}.{key}' if prefix else key
        result.update(_flatten(value, name) if isinstance(value, dict) else {name: value})
    return result


# ---- 文件读取：资源路径、基础/高级合并与字段来源 ----

# 相对路径以字段所属配置文件的目录为基准；package:// 经包索引解析。
# 相机设备名称只在文件输入模式按资源路径处理。
def resolve_path(value, config_file, package_resolver):
    if not value:
        return value
    if value.startswith('package://'):
        resource = value[len('package://'):]
        package, separator, suffix = resource.partition('/')
        if not package or not separator:
            raise ConfigurationError(f'invalid package URI: {value}')
        path = Path(package_resolver(package)) / suffix
    else:
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = Path(config_file).parent / path
    return str(path.resolve())


def _package_directory(name):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(name)


def _read_yaml(path):
    try:
        raw = path.read_bytes()
        document = yaml.load(raw.decode('utf-8'), Loader=_UniqueLoader)
    except (OSError, UnicodeError, yaml.YAMLError, ConfigurationError) as error:
        raise ConfigurationError(f'cannot read configuration {path}: {error}') from error
    if not isinstance(document, dict) or not document:
        raise ConfigurationError(f'{path}: configuration must be a non-empty mapping')
    return document, hashlib.sha256(raw).hexdigest()

