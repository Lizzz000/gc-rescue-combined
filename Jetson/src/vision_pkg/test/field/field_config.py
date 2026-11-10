#!/usr/bin/env python3
"""从统一配置（rescue.yaml + rescue_advanced.yaml）生成测试用参数头。

输出两类宏行，测试与算法读同一份 yaml，不在测试里写死：
  SET(name, value)     detect_node 的 field.* 参数，展开进 FieldConfig
  DETECT(name, value)  测试另需的 detect_node 参数（如 field_max_fps）
用法：field_config.py <输出 .inc 路径>
"""
import importlib
import importlib.util
import json
from pathlib import Path
import sys

# 测试需要的非 field.* 参数
DETECT_KEYS = ('field_max_fps',)


def load_runtime(root):
    runtime = root / 'src/bringup_pkg/runtime'
    spec = importlib.util.spec_from_file_location('field_test_runtime', runtime / '__init__.py',
                                                  submodule_search_locations=[str(runtime)])
    package = importlib.util.module_from_spec(spec)
    sys.modules['field_test_runtime'] = package
    spec.loader.exec_module(package)
    return runtime, importlib.import_module('field_test_runtime.configuration')


def read_values(root):
    runtime, configuration = load_runtime(root)
    values, _, _ = configuration._read_configuration(root / 'src/bringup_pkg/config/rescue.yaml',
                                                     lambda name: root / 'src' / name)
    schema = json.loads((runtime / 'schema.json').read_text(encoding='utf-8'))
    return values, schema['parameters']['detect_node']


def literal(value):
    return ('true' if value else 'false') if isinstance(value, bool) else repr(value)


def lookup(values, mapping):
    value = values[mapping['path']]
    return value[mapping['index']] if 'index' in mapping else value


def render_lines(values, detect):
    lines = []
    for name, mapping in detect.items():
        if name.startswith('field.'):
            lines.append(f'SET({name[len("field."):]}, {literal(lookup(values, mapping))})')
    for name in DETECT_KEYS:
        lines.append(f'DETECT({name}, {literal(lookup(values, detect[name]))})')
    return lines


def main(argv):
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    root = Path(__file__).resolve().parents[4]
    values, detect = read_values(root)
    Path(argv[1]).write_text('\n'.join(render_lines(values, detect)) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
