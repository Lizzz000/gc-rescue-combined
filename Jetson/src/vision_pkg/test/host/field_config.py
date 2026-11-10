#!/usr/bin/env python3
"""把统一配置里 detect_node 的 field.* 参数展开成 SET(name, value) 行，供 host 测试使用生产参数。"""
import importlib
import importlib.util
import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[4]
runtime = root / 'src/bringup_pkg/runtime'
spec = importlib.util.spec_from_file_location('field_test_runtime', runtime / '__init__.py',
                                              submodule_search_locations=[str(runtime)])
package = importlib.util.module_from_spec(spec)
sys.modules['field_test_runtime'] = package
spec.loader.exec_module(package)
configuration = importlib.import_module('field_test_runtime.configuration')
values, _, _ = configuration._read_configuration(root / 'src/bringup_pkg/config/rescue.yaml',
                                                 lambda name: root / 'src' / name)
schema = json.loads((runtime / 'schema.json').read_text(encoding='utf-8'))
lines = []
for name, mapping in schema['parameters']['detect_node'].items():
    if not name.startswith('field.'):
        continue
    value = values[mapping['path']]
    if 'index' in mapping:
        value = value[mapping['index']]
    text = ('true' if value else 'false') if isinstance(value, bool) else repr(value)
    lines.append(f'SET({name[len("field."):]}, {text})')
Path(sys.argv[1]).write_text('\n'.join(lines) + '\n', encoding='utf-8')
