#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""静态源码合同：核对布局、入口约束、语法与参数映射，不代表构建或板端测试通过。

只检查自有源码，不导入测试主体，不修改第三方代码。
"""
import ast
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


def check(root):
    errors = []
    def require(condition, message):
        if not condition:
            errors.append(message)
    # 自动栈六包及两个独立调试工具；自动程序和配置入口仍只检查六包。
    packages = {path.parent.name for path in (root / 'src').glob('*/package.xml')}
    automatic = {'interface_pkg', 'vision_pkg', 'track_pkg', 'mission_pkg', 'bridge_pkg', 'bringup_pkg'}
    require(packages == automatic | {'teleop_pkg', 'record_pkg'},
            'expected six automatic packages plus teleop_pkg and record_pkg')
    programs = []
    for path in (root / 'src').glob('*/CMakeLists.txt'):
        text = path.read_text(encoding='utf-8')
        if path.parent.name not in automatic:
            continue
        programs += re.findall(r'add_executable\((\w+_runtime)(?:\s|\))', text)
        require('entry/' not in text, str(path) + ': old entry directory')
        require(not re.search(r'install\([^)]*\b(?:config|launch)\b', text, re.S) or path.parent.name == 'bringup_pkg',
                str(path) + ': package config/launch overlay remains')
    require(set(programs) == {'vision_runtime', 'tracking_runtime', 'mission_runtime', 'bridge_runtime'} and len(programs) == 4,
            'expected four runtime targets')
    # 只读检查自有源码；解析 Python 语法不会执行测试或创建业务资源。
    for path in (root / 'src').rglob('*'):
        if not path.is_file() or 'third_party' in path.parts or 'vendor' in path.parts:
            continue
        if path.suffix in ('.cpp', '.hpp', '.h'):
            text = path.read_text(encoding='utf-8')
            require(not re.search(r'^\s*#\s*include\s*[<"][^>"\n]+\.cpp[>"]', text, re.M),
                    str(path) + ': includes implementation cpp')
            require('RESCUE_NODE_' not in text, str(path) + ': obsolete entry macro')
            require('test/stubs' not in text, str(path) + ': fake GPU dependency')
        elif path.suffix == '.py':
            try:
                tree = ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path))
            except SyntaxError as error:
                errors.append(str(error))
                continue
            if 'checks' in path.parts or 'test' in path.parts:
                require(not any(isinstance(node, ast.Assert) for node in ast.walk(tree)),
                        str(path) + ': acceptance must survive python -O')
        elif path.name == 'package.xml':
            try:
                ET.parse(path)
            except ET.ParseError as error:
                errors.append(str(error))
    # 所有节点参数须映射到规范字段。
    schema = json.loads((root / 'src/bringup_pkg/runtime/schema.json').read_text(encoding='utf-8'))
    for node, parameters in schema['parameters'].items():
        require(bool(parameters), node + ': empty parameter mapping')
        for name, mapping in parameters.items():
            require(mapping['path'] in schema['schema'], node + '/' + name + ': missing canonical field')
    return errors


def main():
    root = Path(__file__).resolve().parents[2]
    errors = check(root)
    print(json.dumps({'root': str(root), 'errors': errors}, indent=2, ensure_ascii=False))
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
