#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""板端独立夹具：只启动短命 Python 进程，验证 profiler 的创建期信号与 HTTP 窗口。"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from unittest.mock import patch


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def load_profile(path):
    spec = importlib.util.spec_from_file_location('profile_under_check', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fingerprint(path):
    data = path.read_bytes()
    return {'path': str(path.resolve()), 'bytes': len(data),
            'sha256': hashlib.sha256(data).hexdigest()}


CHILD = r'''
import json, os, pathlib, signal, sys
# 有限寿命兜底；即使夹具本身异常退出，也不会留下永久后台进程。
signal.signal(signal.SIGALRM, signal.SIG_DFL)
signal.alarm(12)
blocked = signal.pthread_sigmask(signal.SIG_BLOCK, [])
signal.signal(signal.SIGINT, signal.SIG_DFL)
signal.signal(signal.SIGTERM, signal.SIG_DFL)
pathlib.Path(sys.argv[1]).write_text(json.dumps({
    'pid': os.getpid(), 'pgid': os.getpgrp(),
    'blocked': [int(item) for item in blocked]
}) + '\n')
while True:
    signal.pause()
'''


def wait_ready(path, process):
    deadline = time.monotonic() + 4
    while not path.exists():
        require(process.poll() is None, 'fixture child exited before ready')
        require(time.monotonic() < deadline, 'fixture child readiness timeout')
        time.sleep(.01)
    # 生产者只写一次小文件；等待完整 JSON，避免把写入中间状态当作测试失败。
    while True:
        try:
            return json.loads(path.read_text())
        except json.JSONDecodeError:
            require(time.monotonic() < deadline, 'fixture child readiness file incomplete')
            time.sleep(.01)


def worker_case(args):
    """信号明确注入在 spawn 完成、工厂返回与外层变量赋值之前。"""
    profile = load_profile(args.profile)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, profile.cancel_profile)
    sys.path.insert(0, str(args.source / 'tools'))
    from validation.evidence import OwnedProcess, group_members
    signum = int(args.signal)
    report = {'case': args.child_case, 'signal': signal.Signals(signum).name,
              'status': 'failed', 'gpu_started': False}
    ready_path = args.output / 'child-ready.json'
    log_path = args.output / 'child.log'
    handle = None
    created = []
    stream = None
    failure = None
    try:
        def factory():
            nonlocal stream
            argv = [sys.executable, '-u', '-c', CHILD, str(ready_path)]
            if args.child_case == 'owner':
                result = OwnedProcess(argv, log_path)
                process = result.process
            else:
                stream = log_path.open('xb')
                result = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
                                          start_new_session=True)
                process = result
            created.append(result)
            report['child'] = wait_ready(ready_path, process)
            require(handle is None, 'injection did not precede outer assignment')
            require(report['child']['pid'] == process.pid and
                    report['child']['pgid'] == process.pid,
                    'fixture child is not its own verified process group')
            require(not {int(signal.SIGINT), int(signal.SIGTERM)}.intersection(report['child']['blocked']),
                    'child inherited blocked INT/TERM')
            report['injected_before_assignment'] = True
            os.kill(os.getpid(), signum)
            # 延迟处理器必须允许工厂返回。
            report['factory_returned_after_signal'] = True
            return result

        try:
            with profile.defer_creation_cancel():
                handle = factory()
            raise RuntimeError('pending cancellation was not delivered on context exit')
        except profile.ProfileCancelled as error:
            report['delivered_signal'] = error.signum
            report['handle_saved_before_exception'] = handle is not None
            require(handle is not None and error.signum == signum,
                    'cancellation escaped before the cleanup handle was saved')
        # 已进入保护性清理后，重复/另一种停止信号不得再打断。
        for repeated in (signal.SIGINT, signal.SIGTERM):
            os.kill(os.getpid(), repeated)
        report['repeated_signals_did_not_interrupt_cleanup'] = True
    except BaseException as error:
        failure = f'{type(error).__name__}: {error}'
    finally:
        profile.protect_cleanup()
        # 即使正在验证的代码回退，夹具仍保留工厂内私有句柄用于清理自身资源。
        owned = handle or (created[-1] if created else None)
        if owned is not None:
            process = owned.process if args.child_case == 'owner' else owned
            try:
                if args.child_case == 'owner':
                    report['shutdown'] = owned.stop((), grace=2)
                    require(not report['shutdown']['forced'], 'fixture owner required forced cleanup')
                    require(process.returncode == -int(signal.SIGINT),
                            'fixture owner child did not receive unblocked SIGINT')
                else:
                    process.send_signal(signum)
                    process.wait(timeout=3)
                    require(process.returncode == -signum, 'fixture child did not receive requested signal')
                report['returncode'] = process.returncode
                report['remaining'] = group_members(process.pid)
                require(not report['remaining'], 'fixture process group is not empty')
            except BaseException as error:
                failure = (failure + '; ' if failure else '') + f'cleanup: {type(error).__name__}: {error}'
                if process.poll() is None or group_members(process.pid):
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=3)
        if stream is not None:
            stream.close()
        report['status'] = 'passed' if failure is None else 'failed'
        if failure:
            report['error'] = failure
        save(args.output / 'result.json', report)
    return 0 if failure is None else 1


def check_http_boundary(profile):
    """显式时间覆盖暖机、长于 history 容量的窗口及统计结束后的迟来帧。"""
    now = [1.0]
    counter = profile.HttpWindowCounter()
    history = profile.CountingHttpHistory(counter, maxlen=3)
    with patch.object(profile.time, 'monotonic', side_effect=lambda: now[0]):
        history.append({'steady': now[0]})  # 窗口外的暖机帧。
        now[0] = 10.0
        started = counter.begin()
        for index in range(6):
            now[0] = 10.1 + index * .1
            history.append({'steady': now[0]})
        now[0] = 11.0
        first = counter.finish()
        for index in range(5):
            now[0] = 11.1 + index * .1
            history.append({'steady': now[0]})  # 模拟分位数计算期间 HTTP 线程仍工作。
        require(first['steady_start'] == started == 10.0 and first['steady_end'] == 11.0,
                'HTTP boundaries differ from measurement boundaries')
        require(first['complete_parts'] == 6 and len(history) == 3,
                'HTTP count includes outside-window parts or is truncated by history')
        now[0] = 20.0
        counter.begin()
        now[0] = 20.1
        history.append({'steady': now[0]})
        counter.cancel()
        now[0] = 21.0
        counter.begin()
        now[0] = 22.0
        second = counter.finish()
        require(second['complete_parts'] == 0, 'cancelled/previous window leaked into the next window')
    return {'status': 'passed', 'first': first, 'second': second,
            'outside_window_parts_excluded': True, 'history_capacity_did_not_limit_count': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=Path(__file__).with_name('pipeline_profile.py'))
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[2],
                        help='源码根目录；从冻结工具副本运行时显式指定实际 validation/evidence.py 来源')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--child-case', choices=('worker', 'owner'), help=argparse.SUPPRESS)
    parser.add_argument('--signal', type=int, choices=(int(signal.SIGINT), int(signal.SIGTERM)),
                        help=argparse.SUPPRESS)
    args = parser.parse_args()
    args.profile = args.profile.resolve(strict=True)
    args.source = args.source.resolve(strict=True)
    require((args.source / 'tools/validation/evidence.py').is_file(), 'invalid --source tools directory')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    if args.child_case:
        require(args.signal is not None, 'child fixture requires --signal')
        return worker_case(args)
    report = {'status': 'failed', 'profile': fingerprint(args.profile),
              'fixture': fingerprint(Path(__file__).resolve()),
              'process_helpers': [fingerprint(args.source / 'tools/validation' / name)
                                  for name in ('evidence.py', 'exec_clean.py')],
              'gpu_started': False, 'cases': []}
    try:
        for mode in ('worker', 'owner'):
            for signum in (signal.SIGINT, signal.SIGTERM):
                output = args.output / f'{mode}-{signal.Signals(signum).name}'
                argv = [sys.executable, str(Path(__file__).resolve()),
                        '--profile', str(args.profile), '--source', str(args.source),
                        '--output', str(output),
                        '--child-case', mode, '--signal', str(int(signum))]
                completed = subprocess.run(argv, capture_output=True, text=True, timeout=25)
                (args.output / f'{output.name}.log').write_text(
                    completed.stdout + completed.stderr, encoding='utf-8')
                row = json.loads((output / 'result.json').read_text())
                report['cases'].append(row)
                require(completed.returncode == 0 and row['status'] == 'passed',
                        'signal fixture failed: ' + output.name)
        report['http_boundary'] = check_http_boundary(load_profile(args.profile))
        report['status'] = 'passed'
    except BaseException as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        save(args.output / 'result.json', report)
    print(json.dumps({'status': report['status'], 'output': str(args.output)}, ensure_ascii=False))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
