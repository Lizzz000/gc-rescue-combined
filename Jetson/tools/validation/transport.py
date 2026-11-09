# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""同一探针链接两个真实运输/ROI规则；仅由统一板端验收显式执行。"""
from __future__ import annotations

import json

from .evidence import fingerprint, require, write_json

# 冻结基线仍按原拆分文件核验；候选身份按实际合并后的编译源记录。
CORE_FILES = {
    'mission/mission.cpp': 'src/mission_pkg/node/mission_node.cpp',
    'mission/mission.hpp': 'src/mission_pkg/node/mission_node.hpp',
    'interface_pkg/app/core.hpp': 'src/interface_pkg/app/core.hpp',
}
OPTIONAL_CORE_FILES = {}


# ---- 探针执行：冻结依赖身份、链接实际核心并保存本轮观察 ----

def core_identity(source):
    files = {**CORE_FILES, **{key: relative for key, relative in OPTIONAL_CORE_FILES.items()
                             if (source / relative).is_file()}}
    return {key: fingerprint(source / relative)['sha256']
            for key, relative in files.items() if (source / relative).is_file()}


def run_probe(args):
    from .workflow import ROOT, command
    source = args.source.resolve()
    before = core_identity(source)
    tool_root = ROOT / 'src/mission_pkg/checks'
    tool_files = {name: fingerprint(tool_root / name) for name in (
        'transport_sequence_probe.cpp', 'transport_sequence/CMakeLists.txt',
        'transport_sequence/transport_identity.hpp.in')}
    probe = args.probe
    if probe is None:
        build = args.build / ('transport-' + args.kind)
        command(['cmake', '-S', tool_root / 'transport_sequence', '-B', build,
                 '-DMISSION_SOURCE_ROOT=' + str(source / 'src/mission_pkg'),
                 '-DCMAKE_BUILD_TYPE=RelWithDebInfo', *args.cmake_arg], args.output / 'transport-configure.log')
        command(['cmake', '--build', build, '-j2'], args.output / 'transport-build.log')
        probe = build / 'mission_transport_sequence_probe'
    require(probe.is_file(), 'actual production transport probe is missing')
    executable = fingerprint(probe)
    observation = args.output / 'transport-observation'
    command([probe, '--output', observation, '--source-id', args.business_version],
            args.output / 'transport-probe.log')
    identity = json.loads((observation / 'identity.json').read_text(encoding='utf-8'))
    require(identity.get('schema') == 'rescue.transport-observation.v1' and identity.get('complete') is True,
            'transport probe did not finish')
    require(identity['case_count'] > 0, 'transport probe observed no inputs')
    require(identity.get('roi_case_count') == 1944, 'ROI probe did not observe all count/policy combinations')
    require(identity['core_files'] == before == core_identity(source),
            'actual compiled production dependency SHA differs, or sources changed during probe')
    require(identity['tool_sha256'] == tool_files['transport_sequence_probe.cpp']['sha256'],
            'probe was not built from the common new probe source')
    require(executable == fingerprint(probe), 'probe executable changed during execution')
    require(all(fingerprint(tool_root / name) == record for name, record in tool_files.items()),
            'common transport probe tools changed during execution')
    write_json(observation / 'execution.json', {'schema': 'rescue.transport-execution.v1',
               'complete': True, 'kind': args.kind, 'source_root': str(source), 'core_files': before,
               'executable': executable, 'tool_files': tool_files,
               'identity': fingerprint(observation / 'identity.json'),
               'results': fingerprint(observation / 'results.jsonl'),
               'roi_results': fingerprint(observation / 'roi-results.jsonl')})


# ---- 结果比较：先复核执行身份，再比较运输与 ROI 规则输出 ----

def read_identity(directory):
    identity = json.loads((directory / 'identity.json').read_text(encoding='utf-8'))
    execution = json.loads((directory / 'execution.json').read_text(encoding='utf-8'))
    require(identity.get('schema') == 'rescue.transport-observation.v1' and identity.get('complete') is True,
            'transport observation is incomplete')
    require(execution.get('schema') == 'rescue.transport-execution.v1' and execution.get('complete') is True,
            'transport production dependency/executable evidence is incomplete')
    require(identity['core_files'] == execution['core_files'], 'transport core identity disagrees with execution')
    artifacts = [('identity', 'identity.json'), ('results', 'results.jsonl')]
    require(('roi_case_count' in identity) == ('roi_results' in execution),
            'ROI observation and execution evidence must both be present')
    if 'roi_case_count' in identity:
        require(identity['roi_case_count'] == 1944, 'ROI observation case count is incomplete')
        artifacts.append(('roi_results', 'roi-results.jsonl'))
    for key, name in artifacts:
        actual = fingerprint(directory / name)
        require(all(actual[item] == execution[key][item] for item in ('sha256', 'bytes', 'mtime_ns')),
                'transport artifact changed after execution: ' + name)
    return identity, execution


def compared(args):
    require(args.baseline and args.candidate, 'transport-compare requires two actual observation directories')
    left, left_execution = read_identity(args.baseline)
    right, right_execution = read_identity(args.candidate)
    require(left_execution['kind'] == 'baseline' and right_execution['kind'] == 'candidate',
            'transport comparison requires distinct baseline/candidate executions')
    require(left['case_count'] == right['case_count'] and left['tool_sha256'] == right['tool_sha256'],
            'transport input generator or observed case count differs')
    require(left.get('roi_case_count') == right.get('roi_case_count'),
            'ROI input generator or observed case count differs')
    require({key: value['sha256'] for key, value in left_execution['tool_files'].items()} ==
            {key: value['sha256'] for key, value in right_execution['tool_files'].items()},
            'transport build/probe tools differ')
    differences = []
    count = 0
    from itertools import zip_longest
    with (args.baseline / 'results.jsonl').open(encoding='utf-8') as baseline, \
            (args.candidate / 'results.jsonl').open(encoding='utf-8') as candidate:
        for index, lines in enumerate(zip_longest(baseline, candidate)):
            require(all(line is not None for line in lines), 'transport result lengths differ')
            before, after = (json.loads(line) for line in lines)
            require(before['index'] == after['index'] == index, 'transport input order is not contiguous')
            require(all(row['input_unchanged'] is True for row in (before, after)), 'production input was modified')
            require(all(before[key] == after[key] for key in ('input', 'transport', 'policy')),
                    'transport runs used different inputs')
            if before['decision'] != after['decision']:
                differences.append({'index': index, 'input': before['input'], 'policy': before['policy'],
                                    'transport': before['transport'], 'baseline': before['decision'],
                                    'candidate': after['decision']})
            count += 1
    require(count == left['case_count'] and count > 0, 'transport result count differs from complete identity')
    roi_differences = []
    roi_count = 0
    with (args.baseline / 'roi-results.jsonl').open(encoding='utf-8') as baseline, \
            (args.candidate / 'roi-results.jsonl').open(encoding='utf-8') as candidate:
        for index, lines in enumerate(zip_longest(baseline, candidate)):
            require(all(line is not None for line in lines), 'ROI result lengths differ')
            before, after = (json.loads(line) for line in lines)
            require(before['index'] == after['index'] == index, 'ROI input order is not contiguous')
            require(all(row['input_unchanged'] is True for row in (before, after)),
                    'production ROI input was modified')
            require(all(before[key] == after[key] for key in ('input', 'policy')),
                    'ROI runs used different inputs')
            if before['result'] != after['result']:
                roi_differences.append({'index': index, 'input': before['input'],
                                        'policy': before['policy'], 'baseline': before['result'],
                                        'candidate': after['result']})
            roi_count += 1
    require(roi_count == left['roi_case_count'], 'ROI result count differs from complete identity')
    report = args.output / 'transport-comparison.json'
    require(not report.exists(), 'transport comparison report exists; preserve prior evidence')
    write_json(report, {'schema': 'rescue.transport-comparison.v1',
               'status': 'failed' if differences or roi_differences else 'executed', 'case_count': count,
               'baseline_core_files': left['core_files'], 'candidate_core_files': right['core_files'],
               'baseline_source_id': left['declared_source_id'], 'candidate_source_id': right['declared_source_id'],
               'numeric_policy': 'exact; the rule copies normalized ROI values and changes no floating-point geometry',
               'differences': differences, 'roi_case_count': roi_count, 'roi_differences': roi_differences})
    require(not differences and not roi_differences, 'actual production transport or ROI rule differences remain')
