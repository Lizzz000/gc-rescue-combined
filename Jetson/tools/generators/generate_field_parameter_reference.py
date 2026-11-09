#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""生成 Field 参考及全量参数清单；只读源码配置，不加载 ROS、模型或设备。"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

PREFIX = 'vision.field.'


def load_json(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'duplicate JSON key: {key}')
            result[key] = value
        return result
    return json.loads(path.read_text(encoding='utf-8'), object_pairs_hook=unique)


def read_configuration(root):
    # 与启动入口共用互补配置读取器，避免文档另行实现覆盖顺序或漏掉高级字段。
    # 这里只合并源码值；package:// 模型路径保持原文，不加载 ROS 索引或业务资源。
    runtime = root / 'src/bringup_pkg/runtime'
    spec = importlib.util.spec_from_file_location('rescue_field_documentation_runtime', runtime / '__init__.py',
                                                  submodule_search_locations=[str(runtime)])
    package = importlib.util.module_from_spec(spec)
    sys.modules['rescue_field_documentation_runtime'] = package
    spec.loader.exec_module(package)
    configuration = importlib.import_module('rescue_field_documentation_runtime.configuration')
    # 复用读取器的原值与来源；不在文档工具重做展平、引用合并或资源路径展开。
    values, origins, _ = configuration._read_configuration(
        root / 'src/bringup_pkg/config/rescue.yaml', lambda name: root / 'src' / name)
    return values, {name: Path(path).relative_to(root.resolve()).as_posix()
                    for name, path in origins.items()}


def cell(value):
    return str(value).replace('|', '&#124;').replace('\n', ' ')


def render(values, schema, notes):
    """默认值只取传入 YAML，类型/消费者只取 schema；文字元数据不允许保存默认值。"""
    paths = {path for path in schema['schema'] if path.startswith(PREFIX)}
    if paths != {path for path in values if path.startswith(PREFIX)}:
        raise ValueError('Field YAML/schema fields differ')
    if set(notes) != {'groups', 'parameters'}:
        raise ValueError('unexpected metadata keys')
    if paths != {PREFIX + name for name in notes['parameters']}:
        raise ValueError('Field descriptions do not cover exactly the schema fields')
    consumers = {path: [] for path in paths}
    for node, parameters in schema['parameters'].items():
        for name, mapping in parameters.items():
            path = mapping['path']
            if path in paths:
                index = f"[{mapping['index']}]" if 'index' in mapping else ''
                consumers[path].append(f'{node}.{name} ← {path.removeprefix(PREFIX)}{index}')
    for name, description in notes['parameters'].items():
        path = PREFIX + name
        if set(description) != {'group', 'level', 'unit', 'purpose'}:
            raise ValueError(f'{name}: metadata contains missing/unknown keys (defaults belong in YAML)')
        if description['group'] not in notes['groups'] or description['level'] not in ('常用', '高级'):
            raise ValueError(f'{name}: unknown group/level')
        if not all(isinstance(v, str) and v.strip() for v in description.values()) or not consumers[path]:
            raise ValueError(f'{name}: empty description or missing consumer')
        value, kind = values[path], schema['schema'][path]
        valid = ((kind == 'integer' and type(value) is int) or
                 (kind == 'number' and type(value) in (float, int) and math.isfinite(value)) or
                 (kind == 'boolean' and type(value) is bool) or
                 (kind == 'numbers' and isinstance(value, list) and len(value) == 2 and
                  all(type(v) in (float, int) and math.isfinite(v) for v in value)))
        if not valid:
            raise ValueError(f'{path}: value does not match {kind}')
    lines = [
        '# Field 参数参考（自动生成）', '',
        '字段前缀均为 `vision.field.`。默认值来自基础配置 [rescue.yaml](../../src/bringup_pkg/config/rescue.yaml)'
        '与高级配置 [rescue_advanced.yaml](../../src/bringup_pkg/config/rescue_advanced.yaml)，'
        '类型、数组端点及消费者来自 [schema.json](../../src/bringup_pkg/runtime/schema.json)；'
        '分组、用途和单位来自 [说明元数据](../../tools/generators/field_parameter_notes.json)。', '',
        '通过 `python3 tools/generators/generate_field_parameter_reference.py` 更新；`--check` 只核对文件是否过期。'
        '请修改来源后生成，不手工修改本表。范围与组合合法性仍由统一装配和 `validateFieldConfig` 决定，'
        '此工具不新增生产校验，也不保存实际运行值。', '',
        '“常用”指现场优先检查的输入/几何/颜色和主要准入项；“高级”指需要固定序列证据后再修改的机制参数。'
        '基础与高级文件共同构成一份完整配置，字段互补，禁止重复或覆盖；全部现有配置能力保留。'
        '调参顺序与证据判读见[Field 调参指南](../guides/field-tuning.md)。', '',
    ]
    for group, title in notes['groups'].items():
        lines += [f'## {title}', '', '| 字段 | 层级 | 类型 / 单位 | 生产配置默认值 | 用途 |',
                  '| --- | --- | --- | --- | --- |']
        for name, description in notes['parameters'].items():
            if description['group'] != group:
                continue
            path = PREFIX + name
            default = json.dumps(values[path], ensure_ascii=False, allow_nan=False)
            lines.append(f"| `{name}` | {description['level']} | {schema['schema'][path]} / "
                         f"{cell(description['unit'])} | `{default}` | {cell(description['purpose'])} |")
        lines.append('')
    lines += ['## 实际消费者与数组展开', '',
              '下面只列装配映射；数组端点在装配时展开，纯值 `FieldConfig` 仍接收原标量字段。', '',
              '| 字段 | ROS 参数 ← 配置值 |', '| --- | --- |']
    for name in notes['parameters']:
        lines.append(f'| `{name}` | ' + '<br>'.join(f'`{v}`' for v in consumers[PREFIX + name]) + ' |')
    return '\n'.join(lines) + '\n'


def catalog_from_sources(values, origins, schema, catalog):
    """JSON 保留人工单位/说明及历史元信息；默认值、类型、来源、消费者只由源码生成。"""
    entries = catalog['parameters']
    paths = [entry['path'] for entry in entries]
    if len(paths) != len(set(paths)) or set(paths) != set(values) or set(paths) != set(schema['schema']):
        raise ValueError('parameter catalog must describe every configuration field exactly once')
    consumers = {path: [] for path in paths}
    for node, parameters in schema['parameters'].items():
        for name, mapping in parameters.items():
            consumers[mapping['path']].append(f'{node}.{name}')
    consumers['version'] = ['configuration.py']
    generated = []
    for entry in entries:
        path = entry['path']
        if entry.get('actual_value') is not None or entry.get('status') != 'source_default_only':
            raise ValueError(f'{path}: source catalog cannot contain runtime evidence')
        if not consumers[path] or not entry.get('unit') or not entry.get('validation'):
            raise ValueError(f'{path}: missing consumer or description')
        generated.append(dict(entry, type=schema['schema'][path], planned_default=values[path],
                              source=origins[path], consumers=consumers[path]))
    return dict(catalog, parameters=generated)


def render_catalog(catalog):
    lines = ['# 源码默认参数清单（自动生成）', '',
             '默认值与来源取自基础/高级配置，类型与消费者取自 '
             '[schema.json](../../src/bringup_pkg/runtime/schema.json)。'
             '单位与校验说明在[机器清单](parameter-catalog.json)维护；'
             '范围和组合合法性以配置装配器及真实核心为准。', '',
             '通过 `python3 tools/generators/generate_field_parameter_reference.py` 同步本表、机器清单与 '
             '[Field 参数参考](field-parameters.md)；`--check` 检查三者是否过期。'
             '修改配置后重新生成，不在文档中另存一套默认值。', '',
             f"共 {len(catalog['parameters'])} 个有效配置字段。配置复制、路径解析和启动方式见"
             '[配置指南](../guides/configuration.md)。实际运行值与资源路径以对应运行的 '
             '`effective-configuration.json` 为准，历史结果见[历史索引](../history/README.md)。', '',
             '`baseline` 仅记录统一重建的原始基线；`planned_default` 是当前源码默认值，'
             '`actual_value=null` 表示不保存板端运行值。源码默认值不代表新版本板端验收通过。', '',
             '| 字段 | 配置文件 | 类型 / 单位 | 源码默认值 | 消费者 | 校验 |',
             '| --- | --- | --- | --- | --- | --- |']
    for entry in catalog['parameters']:
        default = json.dumps(entry['planned_default'], ensure_ascii=False, allow_nan=False)
        source = entry['source']
        lines.append(f"| `{entry['path']}` | [{Path(source).name}](../../{source}) | "
                     f"{entry['type']} / {cell(entry['unit'])} | `{cell(default)}` | "
                     f"{cell('、'.join(entry['consumers']))} | {cell(entry['validation'])} |")
    return '\n'.join(lines) + '\n'


def generated_documents(root):
    values, origins = read_configuration(root)
    schema = load_json(root / 'src/bringup_pkg/runtime/schema.json')
    catalog = catalog_from_sources(values, origins, schema,
                                   load_json(root / 'docs/reference/parameter-catalog.json'))
    return {
        'docs/reference/field-parameters.md': render(
            values, schema, load_json(root / 'tools/generators/field_parameter_notes.json')),
        'docs/reference/parameter-catalog.json': json.dumps(catalog, indent=2, ensure_ascii=False) + '\n',
        'docs/reference/parameter-catalog.md': render_catalog(catalog),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='只核对已生成文档，不写文件')
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[2]
    try:
        documents = generated_documents(root)
        stale = [name for name, content in documents.items() if not (root / name).is_file() or
                 (root / name).read_text(encoding='utf-8') != content]
        if args.check and stale:
            raise ValueError('parameter references are stale; regenerate: ' + ', '.join(stale))
        if not args.check:
            for name in stale:
                (root / name).write_text(documents[name], encoding='utf-8', newline='\n')
                print(root / name)
        print('Field and full parameter references match source configuration (documentation check only)')
    except (ValueError, KeyError, OSError, ImportError) as error:
        print(f'Parameter documentation: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
