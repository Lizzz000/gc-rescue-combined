# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""板端统一验收：准备与构建、合同、场景采集及比较、摄像头与性能观察。

各阶段保留实际后端、原始证据与退出检查；源码检查或历史记录不能代替本次板端执行。
"""
from __future__ import annotations

import argparse
import copy
from contextlib import contextmanager
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tarfile
import tempfile
import io
import time
import uuid

from .evidence import (CapabilityMissing, ContractFailure, OwnedProcess, asset_snapshot,
                       fingerprint, group_members, isolated_vcan, port_open, require, write_json,
                       HostFileUseWitness, host_file_use)

from .transport import run_probe as transport_probe, compared as transport_compare
from .provenance import PACKAGES, begin_build, finish_build, verify_install

ROOT = Path(__file__).resolve().parents[2]
BASELINE = 'c1f83e9ed987d6dd24bc59641e22f6b2a44f6717'
FULL_ROLES = ('vision', 'track', 'select', 'mission', 'target', 'safety', 'bridge')
MISSION_CONTRACT = 'stm32-handshake-v3'
CAPTURE_SCENARIOS = (('detect', (0, 1, 2)), ('clock', (0,)), ('chain', (0, 1, 2)))


# ---- 准备与构建：统一配置、命令留证、资产快照和实际安装身份 ----

def configuration():
    name = 'rescue_validation_runtime'
    runtime = ROOT / 'src/bringup_pkg/runtime'
    spec = importlib.util.spec_from_file_location(name, runtime / '__init__.py',
                                                  submodule_search_locations=[str(runtime)])
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return importlib.import_module(name + '.configuration')


def command(command_line, log, env=None):
    """每条外部命令单独保留输出；不通过shell拼接路径或参数。"""
    log = Path(log)
    log.parent.mkdir(parents=True, exist_ok=True)
    metadata = {'argv': list(map(str, command_line))}
    write_json(log.with_suffix(log.suffix + '.command.json'), metadata)
    started = time.monotonic()
    with log.open('w', encoding='utf-8') as stream:
        result = subprocess.run(list(map(str, command_line)), stdout=stream, stderr=subprocess.STDOUT, env=env)
    metadata.update(exit_code=result.returncode, elapsed_seconds=time.monotonic() - started)
    write_json(log.with_suffix(log.suffix + '.command.json'), metadata)
    require(result.returncode == 0, f'command exited {result.returncode}; see {log}')


def flat_to_document(values):
    result = {}
    for key, value in values.items():
        cursor = result
        parts = key.split('.')
        for part in parts[:-1]:
            cursor = cursor.setdefault(part, {})
        cursor[parts[-1]] = value
    return result


def prepare(args):
    cfg = configuration()
    snapshot = cfg.load_configuration(args.config, 'all')
    args.output.mkdir(parents=True, exist_ok=True)
    importlib.import_module('rescue_validation_runtime.ros.launch').materialize(snapshot, args.output)
    # 保留生产参数及合同读取所需分组；软件场景在 contracts 阶段另存显式覆盖。
    parameters = copy.deepcopy(snapshot)
    parameters.update(field={key[6:]: value for key, value in snapshot['nodes']['detect_node'].items()
                             if key.startswith('field.')},
                      detect={key: value for key, value in snapshot['nodes']['detect_node'].items()
                              if not key.startswith('field.')}, stream=snapshot['nodes']['stream_node'])
    write_json(args.output / 'contract-parameters.json', parameters)
    assets = asset_snapshot(snapshot)
    previous_assets = args.output / 'assets-before.json'
    if previous_assets.exists():
        require(json.loads(previous_assets.read_text(encoding='utf-8')) == assets,
                'protected assets differ from the first stage snapshot in this output directory')
    else:
        write_json(previous_assets, assets)
    env = {key: os.environ.get(key) for key in ('ROS_DISTRO', 'AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH',
            'CUDA_HOME', 'OpenCV_DIR', 'cv_bridge_DIR', 'RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID')}
    platform = os.uname()
    env['platform'] = {name: getattr(platform, name)
                       for name in ('sysname', 'nodename', 'release', 'version', 'machine')}
    env['python'] = sys.version
    env['source_commit'] = subprocess.check_output(['git', '-C', ROOT, 'rev-parse', 'HEAD'], text=True).strip()
    env['source_status'] = subprocess.check_output(['git', '-C', ROOT, 'status', '--porcelain'], text=True)
    for tool in ('nvcc', 'nvidia-smi', 'colcon', 'clangd', 'chromium', 'ip', 'unshare'):
        env[tool] = shutil.which(tool)
    write_json(args.output / 'environment.json', env)
    return snapshot


def build(args):
    snapshot = prepare(args)
    gpu = 'tensorrt_gpu' in snapshot['required_backends']
    if gpu:
        require(shutil.which('nvcc'), 'actual TensorRT build requires CUDA nvcc')
        command(['nvcc', '--version'], args.output / 'cuda-version.log')
    # 一份安装：CPU业务合同始终执行；只有模型执行选择真实GPU，绝不运行两套全包矩阵。
    argv = ['colcon', '--log-base', args.output / 'colcon-log', 'build', '--base-paths', args.source / 'src',
             '--build-base', args.build, '--install-base', args.install,
             '--packages-select', *PACKAGES,
             '--cmake-args', '-DBUILD_TESTING=' + ('OFF' if args.kind == 'baseline' else 'ON'), '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON',
             '-DVISION_ENABLE_TENSORRT=' + ('ON' if gpu else 'OFF'), '-DCMAKE_CUDA_ARCHITECTURES=87',
             *args.cmake_arg]
    identity = begin_build(args, ROOT, BASELINE, argv)
    command(argv, args.output / 'build.log')
    finish_build(args, identity, ROOT, BASELINE)


# ---- 合同执行：核对安装、读取真实登记、运行并检查本轮结果 ----

def contracts(args):
    from .provenance import IDENTITY, beneath
    require(args.kind == 'candidate', 'the rebuilt contract suite only runs against a candidate build')
    snapshot = prepare(args)
    identity = json.loads((args.install.resolve() / IDENTITY).read_text(encoding='utf-8'))
    require(identity['build_root'] == str(args.build.resolve()),
            '--build does not match the verified installation')
    cfg = configuration()
    selected = {role: executable(args.install, package, program)
                for role, (package, program, _) in cfg.ROLES.items()}
    from ament_index_python.packages import get_package_prefix
    for role, selected_path in selected.items():
        package, program, _ = cfg.ROLES[role]
        active = Path(get_package_prefix(package)) / 'lib' / package / program
        require(active.resolve() == selected_path, 'sourced ROS overlay differs from --install: ' + role)
    verify_install(args, ROOT, BASELINE, selected, args.output)
    assets = asset_snapshot(snapshot)
    # 软件合同只使用隔离 vcan 或显式不存在的接口；不能继承生产串口后端。
    # 保留 prepare 的生产快照，另存本次覆盖值及其实际装配来源。
    import yaml
    overrides = {'bridge.transport': 'socketcan', 'bridge.can_interface': 'vcan0',
                 'bridge.serial_device': '', 'bridge.can_bitrate': 0}
    values = dict(snapshot['values'])
    values.update(overrides)
    software_config = args.output / 'software-contract-config.yaml'
    software_config.write_text(yaml.safe_dump(flat_to_document(values), sort_keys=False), encoding='utf-8')
    parameters = json.loads((args.output / 'contract-parameters.json').read_text(encoding='utf-8'))
    parameters.update(cfg.load_configuration(software_config, 'all'))
    parameters.update(production_configuration=snapshot, explicit_execution_overrides=overrides)
    software_parameters = args.output / 'software-contract-parameters.json'
    write_json(software_parameters, parameters)
    env = os.environ.copy()
    env['RESCUE_CONTRACT_PARAMETERS'] = str(software_parameters.resolve())
    env['RESCUE_TEST_ARTIFACTS'] = str((args.output / 'contract-artifacts').resolve())
    # 从实际 CTest 登记确定应生成的结果，登记数量本身不表示执行通过。
    registrations = []
    expected = set()
    for package in PACKAGES:
        package_build = args.build.resolve() / package
        require((package_build / 'CTestTestfile.cmake').is_file(), 'missing actual CTest registration: ' + package)
        argv = ['ctest', '--show-only=json-v1', '--test-dir', str(package_build)]
        observed = subprocess.run(argv, text=True, capture_output=True, env=env)
        write_json(args.output / 'ctest-registration' / (package + '.command.json'),
                   {'argv': argv, 'exit_code': observed.returncode, 'stderr': observed.stderr})
        (args.output / 'ctest-registration' / (package + '.json')).write_text(observed.stdout, encoding='utf-8')
        require(observed.returncode == 0, 'cannot list real CTest registrations: ' + package)
        listing = json.loads(observed.stdout)
        require(listing.get('tests'), 'package registered no rebuilt contracts: ' + package)
        for test in listing['tests']:
            properties = {item['name']: item['value'] for item in test.get('properties', [])}
            require(not properties.get('DISABLED'), 'registered contract is disabled: ' + test['name'])
            test_argv = test['command']
            runners = [index for index, part in enumerate(test_argv) if Path(part).name == 'run_test.py']
            require(len(runners) == 1 and runners[0] + 1 < len(test_argv),
                    'unrecognized ament result registration: ' + test['name'])
            result = Path(test_argv[runners[0] + 1])
            result = (package_build / result).resolve() if not result.is_absolute() else result.resolve()
            require(beneath(result, package_build / 'test_results'), 'CTest result outside this package: ' + test['name'])
            require(result not in expected, 'multiple CTest registrations reuse one result XML: ' + str(result))
            expected.add(result)
            registrations.append({'package': package, 'name': test['name'], 'result_file': str(result)})
    write_json(args.output / 'contract-registration.json', registrations)
    gpu_inputs = []
    gpu_input_identity = []
    if any(row['package'] == 'vision_pkg' and row['name'] == 'vision_contract_yolo_transfer'
           for row in registrations):
        # 先检查本次真实登记；旧基线没有此合同，不额外要求图片或 GPU 环境变量。
        if 'tensorrt_gpu' not in snapshot['required_backends']:
            raise CapabilityMissing('vision_contract_yolo_transfer is registered but the configuration '
                                    'does not enable TensorRT; use the matching build/configuration')
        if not args.inputs or not args.inputs.is_dir():
            raise CapabilityMissing('vision_contract_yolo_transfer requires --inputs with the three '
                                    'frozen real GPU images before contracts can run')
        # 与 capture 使用同一顺序：文件名排序的 jpg，然后文件名排序的 png。
        gpu_inputs = [path.resolve() for path in (
            sorted(args.inputs.glob('*.jpg')) + sorted(args.inputs.glob('*.png'))) if path.is_file()]
        if len(gpu_inputs) != 3 or len(set(gpu_inputs)) != 3:
            raise CapabilityMissing('vision_contract_yolo_transfer requires exactly three distinct '
                                    '*.jpg/*.png files in --inputs')
        require(all('\n' not in str(path) and '\r' not in str(path) for path in gpu_inputs),
                'GPU input paths cannot contain line breaks')
        engine = Path(snapshot['values']['vision.detect.seg_model_path']).resolve()
        labels = Path(snapshot['values']['vision.detect.seg_labels_path']).resolve()
        if engine.suffix.lower() not in ('.trt', '.engine') or not engine.is_file() or not labels.is_file():
            raise CapabilityMissing('vision_contract_yolo_transfer requires the configured existing '
                                    'TensorRT engine and class table; engine building is disabled')
        image_list = args.output / 'gpu-contract-inputs.txt'
        image_list.write_text(''.join(str(path) + '\n' for path in gpu_inputs), encoding='utf-8')
        gpu_input_identity = [fingerprint(path) for path in gpu_inputs]
        env.update(RESCUE_GPU_ENGINE=str(engine), RESCUE_GPU_LABELS=str(labels),
                   RESCUE_GPU_IMAGE_LIST=str(image_list.resolve()))
        write_json(args.output / 'gpu-contract-inputs-before.json', {
            'test': 'vision_contract_yolo_transfer',
            'engine': assets[str(engine)], 'labels': assets[str(labels)],
            'ordered_images': gpu_input_identity, 'image_list': fingerprint(image_list),
            'environment': {key: env[key] for key in
                            ('RESCUE_GPU_ENGINE', 'RESCUE_GPU_LABELS', 'RESCUE_GPU_IMAGE_LIST')}})
    # 保存运行前身份；旧 XML、跳过项、缺失项与命令失败都不能计为本轮通过。
    prior = {str(path): fingerprint(path) for path in expected if path.is_file()}
    commands_failed = []
    for argv, log in (
            # 限定本轮源码，避免从SSH默认home递归发现历史工作区或其他项目。
            (['colcon', '--log-base', args.output / 'test-log', 'test', '--base-paths', args.source / 'src',
              '--build-base', args.build, '--packages-select', *PACKAGES,
              '--install-base', args.install, '--event-handlers', 'console_direct+'], 'contracts.log'),
            (['colcon', 'test-result', '--test-result-base', args.build, '--verbose'], 'contract-results.log')):
        try:
            command(argv, args.output / log, env)
        except ContractFailure as error:
            commands_failed.append(str(error))
    import xml.etree.ElementTree as ET
    cases = []
    stale, malformed, missing = [], [], []
    for path in sorted(expected):
        if not path.is_file():
            missing.append(str(path))
            continue
        current = fingerprint(path)
        if prior.get(str(path)) == current:
            stale.append(str(path))
            continue
        saved = args.output / 'result-xml' / path.relative_to(args.build.resolve())
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, saved)
        try:
            tree = ET.parse(path)
            found = list(tree.iter('testcase'))
            require(found, 'registered result XML contains no testcase: ' + str(path))
        except (ET.ParseError, ContractFailure) as error:
            malformed.append({'path': str(path), 'error': str(error)})
            continue
        for case in found:
            status = 'failed' if case.find('failure') is not None or case.find('error') is not None else (
                'not_executed' if case.find('skipped') is not None or case.get('status') == 'notrun' else 'executed')
            cases.append({'source': str(path), 'name': case.get('name'), 'status': status})
    unexpected = [str(path) for path in args.build.resolve().glob('*/test_results/**/*.xml') if path.resolve() not in expected]
    write_json(args.output / 'contract-status.json', {'cases': cases, 'stale_not_counted': stale,
               'missing_registered_results': missing, 'unexpected_not_counted': unexpected,
               'malformed': malformed, 'command_errors': commands_failed})
    after = asset_snapshot(snapshot)
    write_json(args.output / 'assets-after.json', after)
    require(after == assets, 'protected model/engine/cache changed during contracts')
    if gpu_inputs:
        gpu_inputs_after = [fingerprint(path) for path in gpu_inputs]
        write_json(args.output / 'gpu-contract-inputs-after.json', gpu_inputs_after)
        require(gpu_inputs_after == gpu_input_identity, 'fixed GPU images changed during contracts')
    require(cases, 'no freshly executed contract result XML found')
    require(not commands_failed and not malformed and not stale and not missing and not unexpected,
            'registered contracts incomplete, command failure or unregistered XML; inspect contract-status.json')
    require(all(case['status'] == 'executed' for case in cases), 'failed or unexecuted contracts remain')

# ---- 场景基础：程序定位、同参启动与场景快照 ----

def executable(install, package, name):
    for root in (install / package, install):
        candidate = root / 'lib' / package / name
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    raise CapabilityMissing(f'missing executable {package}/{name} in {install}')


def launch_case(args, snapshot, directory, run='all', clock=False):
    """同一有效参数启动基线/候选。"""
    import yaml
    cfg = configuration()
    directory.mkdir(parents=True, exist_ok=True)
    roles = FULL_ROLES if run == 'all' else (run,)
    role_executables = {}
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    for role in roles:
        package, name, _ = cfg.ROLES[role]
        selected = executable(args.install, package, name)
        active = Path(get_package_prefix(package)) / 'lib' / package / name
        require(active.resolve() == selected, 'sourced ROS overlay differs from --install: ' + role)
        role_executables[role] = selected
    verify_install(args, ROOT, BASELINE, role_executables, directory)
    if not clock:
        identity = {}
        runtime_files = sorted(str(path.relative_to(ROOT / 'src/bringup_pkg'))
                               for path in (ROOT / 'src/bringup_pkg/runtime').rglob('*.py')
                               if '__pycache__' not in path.parts)
        for relative in (*runtime_files, 'runtime/schema.json', 'launch/rescue.launch.py'):
            installed = Path(get_package_share_directory('bringup_pkg')) / relative
            source = ROOT / 'src/bringup_pkg' / relative
            row = {'source': fingerprint(source), 'installed': fingerprint(installed)}
            require(row['source']['sha256'] == row['installed']['sha256'],
                    'installed production launch/configuration differs from validated source: ' + relative)
            identity[relative] = row
        write_json(directory / 'launch-configuration-identity.json', identity)
        env = os.environ.copy()
        env['RESCUE_RUN_DIR'] = str((directory / 'launch').resolve())
        return OwnedProcess(['ros2', 'launch', 'bringup_pkg', 'rescue.launch.py',
                             'config:=' + snapshot['configuration'], 'run:=' + run],
                            directory / 'launcher.log', env, role_executables)
    nodes = copy.deepcopy(snapshot['nodes'])
    param_file = directory / 'parameters.yaml'
    param_file.write_text(importlib.import_module('rescue_validation_runtime.ros.launch').dump_ros_parameters(nodes), encoding='utf-8')
    actions = []
    for role in roles:
        package, name, _ = cfg.ROLES[role]
        arguments = ['--role', role]
        executable(args.install, package, name)
        actions.append(f'Node(package={package!r}, executable={name!r}, arguments={arguments!r}, '
                       f'parameters=[{str(param_file.resolve())!r}], output="screen")')
    launch = directory / 'observation.launch.py'
    launch.write_text('from launch import LaunchDescription\nfrom launch_ros.actions import Node\n'
                      'def generate_launch_description():\n    return LaunchDescription([' + ','.join(actions) + '])\n', encoding='utf-8')
    write_json(directory / 'effective-configuration.json', snapshot)
    return OwnedProcess(['ros2', 'launch', str(launch.resolve())], directory / 'launcher.log',
                        role_executables=role_executables)


def case_snapshot(args, image=None, clock=False, directory=None):
    """将场景覆盖值写入独立验收配置，再经生产装配器 load_configuration 读取；配置期能拦住哪些非法组合以 bringup 当前校验为准，其余在节点启动时暴露。"""
    import yaml
    cfg = configuration()
    original = cfg.load_configuration(args.config, 'all')
    values = dict(original['values'])
    values.update({'bridge.transport': 'socketcan', 'bridge.serial_device': '', 'bridge.can_bitrate': 0,
                   'bridge.can_interface': 'vcan0', 'vision.detect.active_task': 'seg',
                   'vision.detect.publish_masks': True, 'vision.detect.publish_timing': True,
                   'vision.stream.draw_masks': True, 'vision.stream.overlay_stats': False,
                   'vision.stream.stream_address': '127.0.0.1'})
    with socket.socket() as channel:
        channel.bind(('127.0.0.1', 0))
        values['vision.stream.stream_port'] = channel.getsockname()[1]
    if image:
        import cv2
        decoded = cv2.imread(str(image))
        require(decoded is not None, 'fixed input cannot be decoded: ' + str(image))
        values.update({'vision.detect.camera_backend': 'file', 'vision.detect.camera_device': str(image.resolve()),
                       'vision.detect.camera_fps': (original['values']['vision.detect.camera_fps']
                                                  if args.stage == 'performance' else 10),
                       'shared.image.width': decoded.shape[1], 'shared.image.height': decoded.shape[0]})
    if clock:
        values.update({'vision.detect.camera_device': '/dev/rescue_validation_absent',
                       'vision.detect.camera_backend': 'v4l2', 'vision.stream.stream_enabled': False})
    path = (directory or args.output) / ('clock-config.yaml' if clock else 'case-config.yaml')
    path.write_text(yaml.safe_dump(flat_to_document(values), sort_keys=False), encoding='utf-8')
    snapshot = cfg.load_configuration(path, 'all')
    if clock:
        # ROS通用use_sim_time不是生产配置键，仅时钟合同显式启用。
        for node in ('detect_node', 'stream_node'):
            snapshot['nodes'][node]['use_sim_time'] = True
    return snapshot


# ---- 场景观察：源时钟、资源采样、清理与运输握手 ----

def clock_contract(observer, owner):
    # /clock 驱动源时间；等待预算仍使用单调秒，避免源时间回退延长验收期限。
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import SetBool
    publisher = observer.node.create_publisher(Clock, '/clock', 10)
    def drive(second, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            message = Clock()
            message.clock.sec = second
            publisher.publish(message)
            observer.spin(min(0.03, max(0, end - time.monotonic())), owner)
    def drive_until(second, predicate, reason):
        # 一秒墙钟刷新有任意相位；持续驱动源时钟，按真实重复通知完成，六秒内仍不足则失败。
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            drive(second, min(0.1, max(0, deadline - time.monotonic())))
            if predicate():
                return
        require(False, reason)
    try:
        observer.wait(lambda: observer.direct_switch.service_is_ready() and publisher.get_subscription_count() > 0,
                      reason='clock subscriptions', owner=owner)
        drive(0, 1.2)
        require(not observer.states, 'zero source time produced stage')
        drive_until(10, lambda: len(observer.states) >= 2, 'stage repetition missing after six seconds')
        initial = observer.states[-1]
        require(initial['source'][0] == 10_000_000_000 and all(
                item['task'] == 'seg' and item['epoch'] == initial['epoch'] and
                item['source'] == initial['source'] for item in observer.states), 'initial stage identity changed')
        drive(4, 0.2)
        future = observer.direct_switch.call_async(SetBool.Request(data=True))
        end = time.monotonic() + 5
        while not future.done() and time.monotonic() < end:
            drive(4, 0.05)
        require(future.done() and future.result().success, 'clock rollback switch rejected')
        observer.states.clear()
        drive(4, 1.2)
        require(not observer.states, 'rollback emitted future stage')
        drive_until(10, lambda: len(observer.states) >= 2, 'recovered stage repetition missing after six seconds')
        require(all(item['task'] == 'field' and item['epoch'] == initial['epoch'] + 1
                and item['source'] == initial['source'] for item in observer.states), 'recovery changed stage identity')
        return {'zero_quiet': True, 'rollback_quiet': True, 'recovered_identity': True}
    finally:
        observer.node.destroy_publisher(publisher)


def resource_sample(owner):
    # 只采本轮业务进程；内存以 KiB、温度以摄氏度、观察时间以单调秒记录。
    sample = {'steady': time.monotonic(), 'processes': [], 'temperatures_c': {}, 'temperature_read_errors': []}
    for process in owner.sample():
        if not process['role']:
            continue
        path = Path('/proc') / str(process['pid'])
        memory = {}
        for line in (path / 'smaps_rollup').read_text().splitlines():
            if line.startswith(('Pss:', 'Rss:')):
                name, value, _ = line.split()
                memory[name.rstrip(':') + '_kib'] = int(value)
        sample['processes'].append(dict(process, **memory, fd_count=len(list((path / 'fd').iterdir())),
                                        threads=len(list((path / 'task').iterdir()))))
    for zone in Path('/sys/class/thermal').glob('thermal_zone*'):
        try:
            sample['temperatures_c'][(zone / 'type').read_text().strip()] = float((zone / 'temp').read_text()) / 1000
        except (OSError, ValueError, TypeError) as error:
            sample['temperature_read_errors'].append(
                {'path': str(zone / 'temp'), 'type': type(error).__name__, 'error': str(error)})
    return sample


def cleanup_case(record, observer, owner, http, can, snapshot, before, expected, port):
    """某一步清理失败也继续关闭其余自有资源，并在原始报告保留每个错误。"""
    errors = []
    for name, action in (
            ('http', lambda: http.stop() if http else None),
            ('launcher', lambda: record.update(shutdown=owner.stop(expected, port)) if owner else None),
            ('can', lambda: can.close() if can else None),
            ('observer', lambda: observer.close() if observer else None),
            ('assets', lambda: record.update(assets_unchanged=asset_snapshot(snapshot) == before))):
        try:
            action()
        except BaseException as error:
            errors.append({'step': name, 'error': str(error)})
    if http:
        record['http_transport'] = http.evidence()
    record['cleanup_errors'] = errors


def mission_command(observer, can, owner, command, phase):
    """分别等待真实 CAN 指令受理和任务终态；不伪造实际识别模式。"""
    old_generation = (observer.records['mission'][-1]['value']['cycle_generation']
                      if observer.records['mission'] else 0)
    roi_id = observer.records['mission'][-1]['value']['roi_request_id'] if command in (1, 2, 4) else None
    if roi_id is not None:
        require(type(roi_id) is int and roi_id > 0, 'physical completion requires current ROI identity')
    since = can.command({1: 0x203, 2: 0x204, 3: 0x205, 4: 0x206}[command],
                        None if roi_id is None else roi_id & 0xffffffff)
    reply = bytes([1, command]) + (0 if roi_id is None else roi_id & 0xffffffff).to_bytes(4, 'little')
    observer.wait(lambda: can.has_reply(0x192, reply.hex(), since),
                  owner=owner, reason='mission command accepted')

    def completed():
        if not observer.records['mission']:
            return False
        state = observer.records['mission'][-1]['value']
        if command == 4:
            return (state['cycle_phase'] == phase and state['roi_request_id'] == roi_id and
                    state['transport_phase'] == 1 and state['cycle_generation'] == old_generation)
        return state['cycle_phase'] == phase and (command == 2 or
                state['cycle_generation'] > old_generation)

    observer.wait(completed, owner=owner, reason='mission command phase confirmation')
    if command in (1, 3):
        mode = 'field' if command == 1 else 'seg'
        observer.wait(lambda: observer.states and observer.states[-1]['task'] == mode,
                      owner=owner, reason='observer actual mode after mission command')
    result = dict(command=command, phase=phase,
                  cycle_generation=observer.records['mission'][-1]['value']['cycle_generation'])
    if roi_id is not None:
        result['roi_request_id'] = roi_id
    return result


def chain_arrival(observer, can, owner, result):
    """失败固定输入只覆盖失败恢复，不用直接切 Field 冒充成功运输。"""
    from interface_pkg.srv import CheckRoi
    from .observations import value_record
    client = observer.node.create_client(CheckRoi, '/target_node/check_roi')
    try:
        observer.wait(client.service_is_ready, owner=owner, reason='direct Target ROI')
        future = client.call_async(CheckRoi.Request())
        observer.wait(future.done, owner=owner, reason='direct Target ROI response')
        require(future.exception() is None, 'direct Target ROI future failed')
        result['/target_node/check_roi'] = value_record(future.result().result)
    finally:
        observer.node.destroy_client(client)
    before = len(observer.records['roi'])
    since = can.command(0x200)
    observer.wait(lambda: can.has_reply(0x190, '01', since),
                  owner=owner, reason='arrival/stop acknowledgment')
    observer.wait(lambda: len(observer.records['roi']) > before and observer.records['mission'] and
                  observer.records['mission'][-1]['value']['cycle_phase'] in (2, 7),
                  owner=owner, reason='committed arrival ROI outcome')
    roi = observer.records['roi'][-1]['value']
    # 这里实际观察的是诊断消息，不能把它冒称为本工具未订阅的服务应答。
    result['/mission_node/roi_result'] = roi
    state = observer.records['mission'][-1]['value']
    can.require_paused(observer, owner)
    result['handshake'] = {'phase_after_roi': state['cycle_phase'], 'coordinates_paused': True}
    if state['cycle_phase'] == 7:
        expected = '010000' if roi['valid'] else '000000'
        observer.wait(lambda: can.has_reply(0x191, expected, since),
                      owner=owner, reason='explicit ROI failure instruction')
        result['mission_roi_wire'] = expected
        result['handshake']['field_executed'] = False
        result['handshake']['reason'] = 'fixed input rejected by unchanged ROI/transport rules'
        result['handshake']['restart'] = mission_command(observer, can, owner, 3, 0)
        return None
    destination = state['destination']
    require(destination in (1, 2), 'committed ROI has no destination')
    expected = bytes([1, 1, destination]).hex()
    observer.wait(lambda: can.has_reply(0x191, expected, since),
                  owner=owner, reason='ROI result on actual CAN')
    result['mission_roi_wire'] = expected
    from source.can_codec import encode
    turn = encode('turn_to_safety', ['left' if destination == 1 else 'right',
                                    str(state['roi_request_id'] & 0xffffffff)]).split('#')[1].lower()
    observer.wait(lambda: can.has_reply(0x193, turn, since),
                  owner=owner, reason='turn instruction after committed Mission state')
    result['handshake']['turn'] = mission_command(observer, can, owner, 1, 4)
    observer.wait(lambda: observer.states and observer.states[-1]['task'] == 'field',
                  owner=owner, reason='actual field after turn completion')
    result['handshake']['field_executed'] = True
    return observer.states[-1]


# ---- 固定输入采集：检测、时钟、完整链路及退出后的证据收口 ----

def capture_scenarios(inputs):
    """采集与比较共用三个冻结输入的七场景顺序，clock 固定使用首个输入身份。"""
    require(isinstance(inputs, list) and len(inputs) == 3,
            'capture requires exactly three frozen inputs')
    return [(scenario, inputs[index]) for scenario, indices in CAPTURE_SCENARIOS for index in indices]


def capture_inputs(inputs):
    capture_scenarios(inputs)
    identities = {}
    for item in inputs:
        sha = item.get('sha256') if isinstance(item, dict) else None
        require(isinstance(sha, str) and len(sha) == 64 and
                all(char in '0123456789abcdef' for char in sha), 'invalid frozen input sha256')
        require(sha not in identities, 'duplicate frozen input')
        identities[sha] = item
    return identities


def evidence_source(header):
    require(isinstance(header, dict) and isinstance(header.get('frame_id'), str) and
            isinstance(header.get('stamp'), dict), 'source header evidence missing')
    stamp = header['stamp']
    require(type(stamp.get('sec')) is int and type(stamp.get('nanosec')) is int and
            0 <= stamp['nanosec'] < 1_000_000_000, 'source time evidence malformed')
    return [stamp['sec'] * 1_000_000_000 + stamp['nanosec'], header['frame_id']]


def paired_evidence(rows, names):
    """重查磁盘中的五帧源配对；字段丢失或重复同一帧不能充当采集样本。"""
    require(isinstance(rows, list) and len(rows) >= 5, 'paired_count evidence requires five frames')
    sources = set()
    for row in rows:
        require(isinstance(row, dict) and isinstance(row.get('source'), list) and
                len(row['source']) == 2 and type(row['source'][0]) is int and row['source'][0] > 0 and
                isinstance(row['source'][1], str), 'paired source evidence missing')
        source = tuple(row['source'])
        require(source not in sources, 'duplicate paired source evidence')
        sources.add(source)
        for name in names:
            value = row.get(name)
            require(isinstance(value, dict) and evidence_source(value.get('header')) == row['source'],
                    'paired header/source evidence differs: ' + name)
            if name in ('detections', 'tracks', 'selected', 'target'):
                require(isinstance(value.get('detections'), list), 'detection array evidence missing: ' + name)
                require(all(isinstance(item, dict) and item.get('header') == value['header']
                            for item in value['detections']), 'detection header evidence differs: ' + name)
            elif name in ('processed', 'masks'):
                require({'width', 'height', 'step', 'encoding', 'is_bigendian', 'sha256'} <= value.keys(),
                        'image evidence missing: ' + name)
            elif name == 'field':
                require({'valid', 'committed', 'corners', 'left_region', 'right_region', 'split_line', 'split_x'} <= value.keys()
                        and type(value['valid']) is bool and type(value['committed']) is bool and
                        all(isinstance(value[key], list) for key in ('corners', 'left_region', 'right_region', 'split_line')),
                        'field geometry evidence missing')
            elif name == 'safety':
                require({'valid', 'side', 'x', 'y'} <= value.keys() and type(value['valid']) is bool,
                        'safety target evidence missing')
        for parent, child in (('tracks', 'selected'), ('selected', 'target')):
            if parent in names and child in names:
                require(all(item in row[parent]['detections'] for item in row[child]['detections']),
                        'paired selection evidence differs: ' + child)


def result_evidence(case):
    """只校验现有三种采集结果及握手分支，不为业务消息另建通用 schema。"""
    result = case['result']
    if case['scenario'] == 'clock':
        require(all(result.get(name) is True for name in
                    ('zero_quiet', 'rollback_quiet', 'recovered_identity')), 'clock evidence incomplete or failed')
        return
    http = case.get('http')
    require(isinstance(http, dict) and type(http.get('frames')) is int and http['frames'] >= 3 and
            isinstance(http.get('snapshot'), dict), 'HTTP evidence missing or incomplete')
    snapshot = http['snapshot']
    require(all(type(snapshot.get(name)) is int and snapshot[name] > 0 for name in ('bytes', 'width', 'height')) and
            isinstance(snapshot.get('sha256'), str) and len(snapshot['sha256']) == 64 and
            all(char in '0123456789abcdef' for char in snapshot['sha256']), 'HTTP evidence snapshot malformed')
    require({'seg', 'task_states', 'mission_states', 'roi_diagnostics'} <= result.keys(),
            'scene evidence missing')
    require(isinstance(result['task_states'], list) and result['task_states'] and
            all(isinstance(result[name], list) and all(isinstance(row, dict) for row in result[name])
                for name in ('mission_states', 'roi_diagnostics')), 'state history evidence missing')
    for state in result['task_states']:
        require(isinstance(state, dict) and {'task', 'epoch', 'source', 'header'} <= state.keys() and
                type(state['epoch']) is int and evidence_source(state['header']) == state['source'],
                'task state evidence missing or inconsistent')
    names = ['detections', 'processed', 'masks']
    chain = case['scenario'] == 'chain'
    if chain:
        names += ['tracks', 'selected', 'target']
    paired_evidence(result['seg'], names)
    field_executed = True
    if chain:
        require({'handshake', '/target_node/check_roi', '/mission_node/roi_result',
                 'mission_roi_wire', 'seg_can'} <= result.keys(), 'chain evidence missing')
        handshake = result['handshake']
        require(isinstance(handshake, dict) and type(handshake.get('field_executed')) is bool and
                handshake.get('coordinates_paused') is True, 'handshake evidence incomplete')
        field_executed = handshake['field_executed']
        require(type(handshake.get('phase_after_roi')) is int and
                handshake['phase_after_roi'] == (2 if field_executed else 7), 'ROI phase evidence differs')
        commands = (('turn', 1, 4), ('safety_arrival', 2, 5), ('delivery', 4, 5), ('restart', 3, 0)) if field_executed else (('restart', 3, 0),)
        for name, command_id, phase in commands:
            command = handshake.get(name)
            require(isinstance(command, dict) and type(command.get('command')) is int and
                    command['command'] == command_id and type(command.get('phase')) is int and
                    command['phase'] == phase and type(command.get('cycle_generation')) is int and
                    command['cycle_generation'] > 0, 'handshake command evidence missing or inconsistent: ' + name)
        if field_executed:
            generation = handshake['turn']['cycle_generation']
            require(handshake['safety_arrival']['cycle_generation'] == generation and
                    handshake['delivery']['cycle_generation'] == generation and
                    handshake['restart']['cycle_generation'] > generation, 'handshake generation evidence differs')
            for name in ('turn', 'safety_arrival', 'delivery'):
                roi_id = handshake[name].get('roi_request_id')
                require(type(roi_id) is int and roi_id > 0 and
                        roi_id == result['/mission_node/roi_result'].get('request_id'),
                        name + ' identity evidence differs')
        else:
            require(isinstance(handshake.get('reason'), str) and handshake['reason'] and
                    not {'turn', 'safety_arrival', 'delivery'} & handshake.keys() and
                    not {'field', 'field_phase', 'field_safety_pairs', 'field_can'} & result.keys(),
                    'failed ROI branch evidence differs')
        roi_fields = {'header', 'request_id', 'valid', 'passed', 'reason', 'roi_x', 'roi_y', 'roi_width',
                      'roi_height', 'green_count', 'blue_count', 'black_count', 'orange_count', 'other_count'}
        for name in ('/target_node/check_roi', '/mission_node/roi_result'):
            roi = result[name]
            require(isinstance(roi, dict) and roi_fields <= roi.keys() and type(roi['request_id']) is int and
                    type(roi['valid']) is bool and type(roi['passed']) is bool, 'ROI result evidence missing')
        require(isinstance(result['mission_roi_wire'], str) and result['mission_states'] and
                result['/mission_node/roi_result'] in result['roi_diagnostics'], 'Mission/ROI evidence missing')
        require(all({'roi_request_id', 'cycle_phase', 'cycle_generation'} <= row.keys()
                    for row in result['mission_states']), 'Mission state evidence missing')
        final = result['mission_states'][-1]
        require(final['cycle_phase'] == 0 and
                final['cycle_generation'] == handshake['restart']['cycle_generation'] and
                final.get('transport_phase') == int(field_executed), 'restart state evidence differs')
        for name in (('seg_can', 'field_can') if field_executed else ('seg_can',)):
            can = result.get(name)
            require(isinstance(can, dict) and {'180', '181', '182'} <= can.keys() and
                    all(isinstance(can[key], dict) and
                        {'valid_bytes', 'ros_expected_bytes', 'invalid_only'} <= can[key].keys() and
                        isinstance(can[key]['valid_bytes'], list) and isinstance(can[key]['ros_expected_bytes'], list) and
                        type(can[key]['invalid_only']) is bool for key in ('180', '181', '182')), 'CAN evidence missing')
    if field_executed:
        phase = result.get('field_phase')
        require(isinstance(phase, dict) and {'task', 'epoch', 'stage_header', 'source_frames',
                'valid_geometry_frames', 'first_source', 'last_source'} <= phase.keys() and
                phase['task'] == 'field' and type(phase['source_frames']) is int and phase['source_frames'] > 0 and
                type(phase['valid_geometry_frames']) is int and 0 <= phase['valid_geometry_frames'] <= phase['source_frames'],
                'field phase evidence missing or failed')
        paired_evidence(result.get('field'), ['field'])
        if chain:
            paired_evidence(result.get('field_safety_pairs'), ['field', 'safety'])


def capture_cases(capture):
    """先拒绝不完整证据，再进行业务值投影；两边同样缺项也不能通过。"""
    require(isinstance(capture, dict) and capture.get('status') == 'executed',
            'cannot compare failed captures')
    require(capture.get('mission_contract') == MISSION_CONTRACT, 'capture mission protocol differs')
    inputs = capture_inputs(capture.get('inputs'))
    expected = {(scenario, item['sha256']) for scenario, item in capture_scenarios(capture['inputs'])}
    cases = capture.get('cases')
    require(isinstance(cases, list) and len(cases) == len(expected), 'capture case count is incomplete')
    indexed = {}
    for case in cases:
        require(isinstance(case, dict), 'malformed capture case')
        identity = case.get('input')
        require(isinstance(identity, dict) and isinstance(identity.get('sha256'), str) and
                isinstance(case.get('scenario'), str), 'capture input/scenario identity missing')
        key = (case['scenario'], identity['sha256'])
        require(key in expected and identity == inputs.get(key[1]), 'capture input/scenario identity differs')
        require(key not in indexed, 'duplicate capture input/scenario')
        require(case.get('status') == 'executed' and case.get('assets_unchanged') is True and
                case.get('input_unchanged') is True and isinstance(case.get('shutdown'), dict) and
                case['shutdown'].get('clean') is True and not case.get('cleanup_errors'),
                'case evidence is incomplete or failed')
        require(all(isinstance(case.get(name), dict) and case[name]
                    for name in ('parameters', 'assets', 'result')), 'case evidence is empty or missing')
        result_evidence(case)
        indexed[key] = case
    require(indexed.keys() == expected, 'capture scenarios are incomplete')
    return indexed


def capture(args):
    from .observations import Observer, MjpegClient, CanProbe, value_record
    import rclpy
    isolated_vcan()
    args.output.mkdir(parents=True, exist_ok=True)
    require(args.inputs and args.inputs.is_dir(), '--inputs frozen directory is required')
    inputs = sorted(args.inputs.glob('*.jpg')) + sorted(args.inputs.glob('*.png'))
    require(len(inputs) == 3, 'GPU comparison requires exactly three frozen images')
    identities = [fingerprint(path) for path in inputs]
    capture_inputs(identities)
    scenarios = capture_scenarios(inputs)
    rclpy.init()
    output = {'business_version': args.business_version, 'kind': args.kind,
              'mission_contract': MISSION_CONTRACT, 'cases': [], 'inputs': identities}
    try:
        for scenario, image in scenarios:
            directory = args.output / f'{scenario}-{image.stem}'
            directory.mkdir(parents=True, exist_ok=True)
            snapshot = case_snapshot(args, image if scenario != 'clock' else None,
                                     scenario == 'clock', directory)
            before = asset_snapshot(snapshot)
            observer = Observer(snapshot, directory)
            owner = None
            http = None
            can = None
            record = {'scenario': scenario, 'input': fingerprint(image),
                      'parameters': snapshot['values'], 'assets': before, 'status': 'failed'}
            output['cases'].append(record)
            try:
                if scenario == 'chain':
                    can = CanProbe(directory)
                    observer.pollers.append(can.poll)
                owner = launch_case(args, snapshot, directory, 'all' if scenario == 'chain' else
                                    ('detect' if scenario == 'clock' else 'vision'),
                                    args.kind == 'baseline', scenario == 'clock')
                if scenario == 'clock':
                    record['result'] = clock_contract(observer, owner)
                else:
                    names = ['detections', 'processed', 'masks']
                    if scenario == 'chain':
                        names += ['tracks', 'selected', 'target']
                    def enough():
                        return observer.has_pairs(names, 5)
                    observer.wait(enough, owner=owner, reason='five source-paired GPU/chain frames')
                    record['result'] = {'seg': observer.paired(names, 5)}
                    if can:
                        settled = time.monotonic()
                        observer.spin(1.5, owner)
                        record['result']['seg_can'] = can.steady_periodic(observer, 'seg', settled)
                    if scenario == 'chain':
                        state = chain_arrival(observer, can, owner, record['result'])
                    else:
                        state = observer.switch('field', owner, direct=True)
                    if state is not None:
                        observer.wait(lambda: observer.records['field'] and len(observer.records['field']) >= 15,
                                      owner=owner, reason='field state history')
                        record['result']['field_phase'] = observer.phase_evidence('field', state)
                        record['result']['field'] = [dict(source=row['source'], field=row['value'])
                                                    for row in list(observer.records['field'])[-5:]]
                        if scenario == 'chain':
                            observer.wait(lambda: observer.has_pairs(['field', 'safety'], 5),
                                          owner=owner, reason='field/safety source pairs')
                            record['result']['field_safety_pairs'] = observer.paired(['field', 'safety'], 5)
                    http = MjpegClient('http://127.0.0.1:' + str(snapshot['values']['vision.stream.stream_port']))
                    http.start()
                    observer.wait(lambda: http.frames >= 3, owner=owner, http=http, reason='complete multipart JPEG')
                    record['http'] = {'frames': http.frames, 'snapshot': http.snapshot()}
                    if can and state is not None:
                        settled = time.monotonic()
                        observer.spin(1.5, owner, http)
                        record['result']['field_can'] = can.steady_periodic(observer, 'field', settled)
                        record['result']['handshake']['safety_arrival'] = mission_command(observer, can, owner, 2, 5)
                        can.require_paused(observer, owner)
                        # 隔离 vcan 夹具模拟物理送达事件；仅验证软件握手，不声称物资实车入区。
                        record['result']['handshake']['delivery'] = mission_command(observer, can, owner, 4, 5)
                        record['result']['handshake']['restart'] = mission_command(observer, can, owner, 3, 0)
                    record['result']['mission_states'] = [row['value'] for row in observer.records['mission']]
                    record['result']['roi_diagnostics'] = [row['value'] for row in observer.records['roi']]
                    record['result']['task_states'] = copy.deepcopy(observer.states)
                # 观察完成仍待清理核验；进程、输入和资产均满足条件后才记为 executed。
                record['status'] = 'observed_pending_shutdown'
            except BaseException as error:
                record['error'] = str(error)
                raise
            finally:
                expected = FULL_ROLES if scenario == 'chain' else ('detect',) if scenario == 'clock' else ('vision',)
                cleanup_case(record, observer, owner, http, can, snapshot, before, expected,
                             None if scenario == 'clock' else snapshot['values']['vision.stream.stream_port'])
                record['input_unchanged'] = fingerprint(image) == record['input']
                if (record['status'] == 'observed_pending_shutdown' and not record['cleanup_errors'] and
                        record.get('shutdown', {}).get('clean') and record.get('assets_unchanged') and record['input_unchanged']):
                    record['status'] = 'executed'
                write_json(directory / 'result.json', record)
            require(record['status'] == 'executed', 'capture cleanup/input/asset checks failed')
    except BaseException as error:
        output['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        try:
            rclpy.shutdown()
        except BaseException as error:
            output['error'] = f'{type(error).__name__}: {error}'
            raise
        finally:
            output['status'] = 'executed' if 'error' not in output and len(output['cases']) == len(scenarios) and all(
                item['status'] == 'executed' for item in output['cases']) else 'failed'
            write_json(args.output / 'capture.json', output)


# ---- 结果比较：先复核原始关联，再投影并比较同输入、同参数的业务值 ----

def semantic_result(case):
    """先核对原始关联；比较投影省略观测计数、折叠重复状态，仅比较 Mission 稳定终态。

    原始日志完整保留；中间动作顺序须由独立真实节点合同验证，不能由此投影推断。
    """
    result = copy.deepcopy(case['result'])
    states = result.get('task_states', [])
    sessions = []
    for state in states:
        require(state['source'][0] > 0, 'TaskState has invalid source time')
        identity = (state['task'], state['epoch'], tuple(state['source']))
        if sessions and identity == sessions[-1]:
            continue
        if sessions:
            require(identity[2][0] >= sessions[-1][2][0], 'TaskState source time regressed')
            require(identity[1] != sessions[-1][1], 'changed task reused epoch')
        sessions.append(identity)
    if states:
        expected = (['seg', 'field', 'seg'] if result['handshake']['field_executed'] else ['seg']) if 'handshake' in result else ['seg', 'field']
        require([item[0] for item in sessions] == expected, 'unexpected captured task session sequence')
    direct = result.get('/target_node/check_roi')
    mission = result.get('/mission_node/roi_result')
    if direct is not None and mission is not None:
        require(direct['request_id'] > 0 and mission['request_id'] > direct['request_id'],
                'Target and Mission ROI requests did not consume distinct ordered Target requests')
        from source.can_codec import decode
        wire = decode('191#' + result['mission_roi_wire'])
        require(wire['valid'] == mission['valid'], 'CAN ROI validity differs from observed diagnostic')
        if result['handshake']['field_executed']:
            require(wire['passed'] and mission['passed'], 'turn path lacks accepted ROI evidence')
        else:
            require(not wire['passed'], 'failed ROI path emitted a passing CAN result')
        statuses = result.get('mission_states', [])
        require(any(row['roi_request_id'] == mission['request_id'] for row in statuses),
                'Mission status did not preserve ROI response request_id')
        require(all(row['roi_request_id'] in (0, mission['request_id']) for row in statuses),
                'unexpected Mission/ROI request identity')
    # 磁盘记录保留原始源头和计数；只规范化当前比较视图。
    tracks, requests, epochs = {}, {}, {}
    def numbered(mapping, identity):
        if identity not in mapping:
            mapping[identity] = len(mapping) + 1
        return mapping[identity]
    def project(value, key=None):
        if key == 'header' or key == 'stage_header':
            require(isinstance(value, dict) and 'stamp' in value and 'frame_id' in value, 'malformed raw header')
            return {'frame_id': value['frame_id']}
        if key in ('request_id', 'roi_request_id'):
            return numbered(requests, value) if value else 0
        if key == 'epoch':
            return numbered(epochs, value)
        if key == 'id' and isinstance(value, str):
            return str(numbered(tracks, value)) if value else ''
        if isinstance(value, dict):
            # 时序与绝对配对键已在单次运行中核对，不作为跨运行算法值比较。
            return {name: project(item, name) for name, item in value.items()
                    if name not in ('source', 'steady', 'source_frames', 'valid_geometry_frames',
                                    'first_source', 'last_source')}
        if isinstance(value, list):
            return [project(item) for item in value]
        return value
    # 整条链路共用编号映射，保持轨迹、请求与阶段身份之间的对应关系。
    projected = project(result)
    for key in ('mission_states', 'roi_diagnostics', 'task_states'):
        if key in projected:
            collapsed = []
            for row in projected[key]:
                if not collapsed or row != collapsed[-1]:
                    collapsed.append(row)
            projected[key] = collapsed
    if projected.get('mission_states'):
        # 两种自动切换确认到达顺序都合法。跨运行比较稳定业务终态；中间发布顺序由真实节点合同验证。
        projected['mission_states'] = projected['mission_states'][-1:]
    return projected


def compare(args):
    report_path = args.output / 'comparison.json'
    require(report_path.resolve() not in (args.baseline.resolve(), args.candidate.resolve()),
            'comparison output would overwrite an input capture')
    differences = []
    report = {'differences': differences, 'numeric_abs_tol': 1e-6, 'numeric_rel_tol': 1e-6,
              'parameter_exclusions': ['vision.stream.stream_port'], 'status': 'failed'}
    def load_capture(path):
        identity = fingerprint(path)
        return json.loads(path.read_bytes()), identity

    def compare_parameters(left, right):
        normalize = lambda values: {key: value for key, value in values.items()
                                    if key != 'vision.stream.stream_port'}
        before, after = normalize(left['parameters']), normalize(right['parameters'])
        require(before == after, 'effective parameters differ')

    def walk(left, right, path):
        if type(left) is bool or type(right) is bool:
            if type(left) is not type(right) or left != right:
                differences.append({'path': path, 'baseline': left, 'candidate': right})
        elif isinstance(left, (int, float)) and isinstance(right, (int, float)):
            equal = left == right if type(left) is int and type(right) is int else math.isclose(
                left, right, rel_tol=1e-6, abs_tol=1e-6)
            if not equal:
                differences.append({'path': path, 'baseline': left, 'candidate': right})
        elif isinstance(left, dict) and isinstance(right, dict) and left.keys() == right.keys():
            for key in left:
                walk(left[key], right[key], path + '.' + key)
        elif isinstance(left, list) and isinstance(right, list) and len(left) == len(right):
            for index, (a, b) in enumerate(zip(left, right)):
                walk(a, b, path + f'[{index}]')
        elif left != right:
            differences.append({'path': path, 'baseline': left, 'candidate': right})
    try:
        baseline, report['baseline_capture'] = load_capture(args.baseline)
        candidate, report['candidate_capture'] = load_capture(args.candidate)
        left_cases, right_cases = capture_cases(baseline), capture_cases(candidate)
        require(left_cases.keys() == right_cases.keys(), 'capture input/scenario identity differs')
        for key, left in left_cases.items():
            right = right_cases[key]
            compare_parameters(left, right)
            require(left['assets'] == right['assets'], 'engine/model/cache identity differs')
            walk(semantic_result(left), semantic_result(right), left['scenario'] + ':' + left['input']['sha256'])
        require(not differences, 'GPU/chain output differences remain')
        report['status'] = 'executed'
    except (ContractFailure, OSError, ValueError, KeyError, TypeError) as error:
        report['error'] = str(error)
        raise
    finally:
        write_json(report_path, report)


# ---- 摄像头与性能：真实资源准入、预热、观察窗口和释放证明 ----

def camera_or_performance(args):
    # 性能使用固定文件输入，摄像头使用真实设备；直接切阶段不验证 Mission 运输握手。
    import rclpy
    from .observations import Observer, MjpegClient
    isolated_vcan()
    require(args.stage != 'performance' or (args.input and args.input.is_file()),
            'performance requires one frozen --input file')
    snapshot = case_snapshot(args, args.input if args.stage == 'performance' else None)
    before = asset_snapshot(snapshot)
    if args.stage == 'camera':
        device = Path(snapshot['values']['vision.detect.camera_device'])
        require(snapshot['values']['vision.detect.camera_backend'] != 'file' and device.exists(), 'real camera required')
        ownership_before = host_file_use(device, 'before')
        write_json(args.output / 'camera-ownership-before.json', ownership_before)
        require(ownership_before['free'],
                'camera ownership not proven free; never stop another project')
    port = snapshot['values']['vision.stream.stream_port']
    require(not port_open(port), 'HTTP port already owned')
    args.output.mkdir(parents=True, exist_ok=True)
    report = {'stage': args.stage, 'configuration': snapshot, 'assets_before': before,
              'stage_control': 'detector-development-only', 'mission_handshake_tested': False,
              'phases': [], 'resources': [], 'windows': [], 'status': 'failed'}
    if args.stage == 'camera':
        report['camera_ownership_before'] = ownership_before
    if args.stage == 'performance':
        nvpmodel = shutil.which('nvpmodel')
        if nvpmodel is None:
            raise CapabilityMissing('nvpmodel is required to record fixed Jetson power mode')
        power = subprocess.run([nvpmodel, '-q'], capture_output=True, text=True, timeout=10)
        require(power.returncode == 0, 'cannot read actual Jetson power mode: ' + power.stderr)
        report['execution_conditions'] = {'nvpmodel': power.stdout.strip(),
            'input': fingerprint(args.input), 'ROS_DISTRO': os.environ.get('ROS_DISTRO'),
            'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION')}
    rclpy.init()
    observer = Observer(snapshot, args.output)
    owner = http = None
    try:
        owner = launch_case(args, snapshot, args.output)
        observer.wait(lambda: observer.states and observer.records['detections'], owner=owner, reason='initial real seg source')
        # 预热起点是首个真实源帧，不是进程创建时刻。
        warmup_start = time.monotonic()
        observer.spin(120, owner)
        report['actual_warmup_s'] = time.monotonic() - warmup_start
        http = MjpegClient('http://127.0.0.1:' + str(port))
        http.start()
        observer.wait(lambda: http.frames > 0, owner=owner, http=http, reason='initial MJPEG')
        # 两类观察共用预热和清理，分别记录固定性能窗口或摄像头持续切换过程。
        if args.stage == 'performance':
            for task in ('seg', 'field'):
                observer.switch(task, owner, direct=True, http=http)
                observer.spin(10, owner, http)
                for index in range(3):
                    window = observer.performance(30, owner, task, http)
                    window['round'] = index + 1
                    report['windows'].append(window)
        else:
            state = observer.states[-1]
            require(state['task'] == 'seg', 'camera measurement must start in seg')
            start = time.monotonic()
            next_switch, next_snapshot, next_reconnect, next_aggregate = 5, 17, 37, 7
            switches = reconnects = 0
            while True:
                elapsed = time.monotonic() - start
                if elapsed >= 600:
                    break
                observer.spin(min(0.05, 600 - elapsed), owner, http)
                elapsed = time.monotonic() - start
                if elapsed >= 600:
                    break  # 终点不发送第120次请求。
                if elapsed >= next_switch and next_switch < 600:
                    phase = observer.phase_evidence(state['task'], state)
                    phase['mjpeg'] = http.phase_evidence(state, snapshot['nodes']['stream_node']['input_timeout_s'])
                    phase['end_s'] = elapsed
                    report['phases'].append(phase)
                    state = observer.switch('field' if state['task'] == 'seg' else 'seg', owner, direct=True, http=http)
                    switches += 1
                    next_switch += 5
                elapsed = time.monotonic() - start
                if elapsed >= 600:
                    break
                if elapsed >= next_snapshot:
                    report.setdefault('snapshots', []).append({'elapsed_s': elapsed, 'task': state['task'], **http.snapshot(not_after=start + 600)})
                    next_snapshot += 17
                elapsed = time.monotonic() - start
                if elapsed >= 600:
                    break
                if elapsed >= next_reconnect:
                    http.reconnect()
                    reconnects += 1
                    next_reconnect += 37
                elapsed = time.monotonic() - start
                if elapsed >= 600:
                    break
                if elapsed >= next_aggregate:
                    report['resources'].append(resource_sample(owner))
                    next_aggregate += 7
            report['phases'].append(observer.phase_evidence(state['task'], state))
            report['phases'][-1]['mjpeg'] = http.phase_evidence(state, snapshot['nodes']['stream_node']['input_timeout_s'])
            report.update(measured_s=time.monotonic() - start, switches=switches,
                          active_reconnects=reconnects, mjpeg_frames=http.frames)
            require(switches == 119 and len(report['phases']) == 120 and reconnects == 16,
                    'camera schedule incomplete (119/120/16 required)')
            require({item['task'] for item in report['snapshots']} == {'seg', 'field'}, 'two-stage snapshots missing')
            require(http.frames > 0 and set(owner.roles) == set(FULL_ROLES), 'HTTP frames or seven role identities missing')
            require(report['resources'] and all(sample['temperatures_c'] for sample in report['resources']),
                    'temperature evidence missing')
        report['status'] = 'observed_pending_shutdown'
    finally:
        cleanup_case(report, observer, owner, http, None, snapshot, before, FULL_ROLES, port)
        try:
            rclpy.shutdown()
        except BaseException as error:
            report['cleanup_errors'].append({'step': 'rclpy.shutdown', 'error': str(error)})
        if owner:
            write_json(args.output / 'process-identities.json', owner.roles)
        if args.stage == 'camera':
            try:
                released = host_file_use(device, 'after')
                report['camera_ownership_after'] = released
                report['camera_released'] = released['free']
            except BaseException as error:
                report['cleanup_errors'].append({'step': 'camera-release', 'error': str(error)})
                report['camera_released'] = False
        if args.stage == 'performance':
            # 固定输入与模型一样在运行后复核；中途替换或删除不能留下成功收据。
            report['input_unchanged'] = False
            try:
                report['input_unchanged'] = fingerprint(args.input) == report['execution_conditions']['input']
            except BaseException as error:
                report['cleanup_errors'].append({'step': 'input-fingerprint', 'error': str(error)})
        report['status'] = 'executed' if (report['status'] == 'observed_pending_shutdown' and
            not report['cleanup_errors'] and report.get('shutdown', {}).get('clean') and
            report.get('assets_unchanged') and report.get('camera_released', True) and
            report.get('input_unchanged', True)) else 'failed'
        write_json(args.output / 'result.json', report)
    require(report['status'] == 'executed', 'shutdown, camera or asset checks failed')


# ---- 性能比较：同输入、功耗模式、参数与资产下比较已完成窗口 ----

def performance_compare(args):
    from .observations import percentile
    report_path = args.output / 'performance-comparison.json'
    require(report_path.resolve() not in (args.baseline.resolve(), args.candidate.resolve()),
            'performance comparison output would overwrite an input')
    left = json.loads(args.baseline.read_text(encoding='utf-8'))
    right = json.loads(args.candidate.read_text(encoding='utf-8'))
    def finite(value):
        return type(value) in (int, float) and math.isfinite(value)

    # 先独立复核每份磁盘证据；相同的缺项、失败或重复轮次不能被中位数掩盖。
    for record in (left, right):
        require(isinstance(record, dict) and record.get('stage') == 'performance' and
                isinstance(record.get('shutdown'), dict) and record['shutdown'].get('clean') is True and
                record.get('cleanup_errors') == [] and record.get('assets_unchanged') is True and
                record.get('input_unchanged') is True and finite(record.get('actual_warmup_s')) and
                record['actual_warmup_s'] >= 120, 'performance evidence incomplete or failed')
        windows = record.get('windows')
        require(isinstance(windows, list) and len(windows) == 6 and
                all(isinstance(window, dict) and window.get('task') in ('seg', 'field') and
                    type(window.get('round')) is int for window in windows) and
                {(window['task'], window['round']) for window in windows} ==
                {(task, index) for task in ('seg', 'field') for index in (1, 2, 3)},
                'performance window evidence requires three distinct rounds per task')
    require(left.get('status') == right.get('status') == 'executed', 'cannot compare failed performance runs')
    normal = lambda values: {key: value for key, value in values.items() if key != 'vision.stream.stream_port'}
    require(normal(left['configuration']['values']) == normal(right['configuration']['values']),
            'performance effective parameters differ')
    require(left['assets_before'] == right['assets_before'], 'performance model/cache identities differ')
    require(left.get('execution_conditions') == right.get('execution_conditions') and
            left.get('execution_conditions'), 'performance input/power/ROS conditions differ or are missing')
    regressions, rows = [], []
    for task in ('seg', 'field'):
        topic = 'detections' if task == 'seg' else 'field'
        aw = [w for w in left['windows'] if w['task'] == task]
        bw = [w for w in right['windows'] if w['task'] == task]
        require(all(finite(w.get('seconds')) and w['seconds'] >= 30 and
                    all(finite(w.get(metric)) and w[metric] >= 0 for metric in ('inference_p95_ms', 'publish_p95_ms')) and
                    all(isinstance(w.get(name), dict) and finite(w[name].get(topic)) and
                        (w[name][topic] > 0 if name == 'throughput' else w[name][topic] >= 0)
                        for name in ('throughput', 'source_to_observer_p95_ms')) for w in aw + bw),
                'performance window evidence has missing, invalid or insufficient measurements')
        for metric in ('inference_p95_ms', 'publish_p95_ms'):
            a = percentile([w[metric] for w in left['windows'] if w['task'] == task], .5)
            b = percentile([w[metric] for w in right['windows'] if w['task'] == task], .5)
            row = {'task': task, 'metric': metric, 'baseline': a, 'candidate': b,
                   'ratio': b / a if a else None}
            rows.append(row)
            if b > a * 1.05:
                regressions.append(row)
        for name, better_lower in (('throughput', False), ('source_to_observer_p95_ms', True)):
            a = percentile([w[name][topic] for w in aw], .5)
            b = percentile([w[name][topic] for w in bw], .5)
            row = {'task': task, 'metric': name, 'baseline': a, 'candidate': b}
            rows.append(row)
            if (better_lower and b > a * 1.05) or (not better_lower and b < a * .95):
                regressions.append(row)
    write_json(report_path, {'metrics': rows, 'needs_repeat': regressions})
    require(not regressions, 'performance change >5%; repeat and explain/fix before acceptance')


# ---- 历史冻结与 Field 序列：绑定源码/探针身份，分别记录执行和比较 ----

def freeze(args):
    """冻结生产实现与传递依赖，不运行或归档旧测试套件。"""
    listing = subprocess.check_output(['git', '-C', ROOT, 'ls-tree', '-rz', BASELINE])
    files = {}
    for record in listing.split(b'\0'):
        if record:
            header, name = record.split(b'\t', 1)
            files[name.decode('utf-8')] = header.split()[0].decode('ascii')
    selected = [name for name in files if name.startswith('src/') and
                ('/test/' not in name or '/third_party/' in name)]
    records = []
    with tarfile.open(args.output / 'baseline-production.tar.gz', 'w:gz') as archive:
        for name in selected:
            data = subprocess.check_output(['git', '-C', ROOT, 'show', BASELINE + ':' + name])
            info = tarfile.TarInfo(name)
            if files[name] == '120000':
                info.type = tarfile.SYMTYPE
                info.linkname = data.decode('utf-8')
                archive.addfile(info)
            else:
                info.size = len(data)
                info.mode = 0o755 if files[name] == '100755' else 0o644
                archive.addfile(info, io.BytesIO(data))
            records.append({'path': name, 'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)})
    snapshot = prepare(args)
    write_json(args.output / 'baseline-identity.json', {'business_version': BASELINE,
               'tool_version': subprocess.check_output(['git', '-C', ROOT, 'rev-parse', 'HEAD'], text=True).strip(),
               'production_files': records, 'archive': fingerprint(args.output / 'baseline-production.tar.gz'),
               'effective_parameters': snapshot, 'status': 'frozen_not_executed'})


def field_sequence(args):
    require(args.inputs and (args.inputs / 'sequence.json').is_file(), '--inputs/sequence.json is required')
    parameters = args.inputs / 'effective.json'
    require(parameters.is_file(), 'frozen full Field parameters are missing')
    from .provenance import source_files, check_baseline
    source = args.source.resolve()
    production = source_files(source)
    if args.kind == 'baseline':
        check_baseline(source, production, ROOT, BASELINE)
    snapshot = prepare(args)
    frozen = json.loads(parameters.read_text(encoding='utf-8'))
    field = {key[6:]: value for key, value in snapshot['nodes']['detect_node'].items() if key.startswith('field.')}
    require(frozen.get('field') == field, 'Field frozen input configuration differs from actual unified configuration')
    # 同一新探针分别链接被测实际核心；不编译旧测试，也不使用不受支持的 --help。
    probe = args.probe
    if probe is None:
        standalone = args.build / ('field-' + args.kind)
        command(['cmake', '-S', ROOT / 'src/vision_pkg/checks/field_sequence', '-B', standalone,
                 '-DFIELD_SOURCE_ROOT=' + str((args.source / 'src/vision_pkg').resolve()),
                 '-DCMAKE_BUILD_TYPE=RelWithDebInfo', *args.cmake_arg], args.output / 'field-configure.log')
        command(['cmake', '--build', standalone, '-j2'], args.output / 'field-build.log')
        probe = standalone / 'vision_field_sequence_probe'
    require(probe.is_file() and os.access(probe, os.X_OK), 'Field sequence executable is missing')
    before = asset_snapshot(snapshot)
    compiled = fingerprint(probe)
    write_json(args.output / 'field-command-identity.json', {'probe': fingerprint(probe),
               'manifest': fingerprint(args.inputs / 'sequence.json'), 'parameters': fingerprint(parameters),
               'declared_source_id': args.business_version, 'source_root': str(args.source.resolve())})
    command([probe, '--manifest', (args.inputs / 'sequence.json').resolve(), '--parameters', parameters.resolve(),
             '--source-id', args.business_version, '--output', args.output / 'field-observation'],
            args.output / 'field-probe.log')
    identity = json.loads((args.output / 'field-observation/identity.json').read_text(encoding='utf-8'))
    require(identity.get('complete') is True and identity.get('frame_count', 0) > 0, 'Field probe incomplete')
    require(identity.get('compiled_identity_verified') is True, 'Field probe lacks compiled source identity')
    require(asset_snapshot(snapshot) == before, 'model/engine/cache changed during Field verification')
    expected = {name: fingerprint(source / 'src/vision_pkg' / name)['sha256'] for name in (
        'node/detect_node.cpp', 'node/detect_node.hpp')}
    require(identity['core_files'] == expected, 'Field probe was compiled against another source root')
    require(source_files(source) == production, 'production sources changed during Field verification')
    require(fingerprint(probe) == compiled, 'Field probe changed during execution')
    write_json(args.output / 'field-observation/execution.json', {
        'schema': 'rescue.field-execution.v1', 'complete': True, 'kind': args.kind,
        'verified_baseline': BASELINE if args.kind == 'baseline' else None,
        'source_root': str(source), 'probe': compiled, 'core_files': expected,
        'identity': fingerprint(args.output / 'field-observation/identity.json'),
        'results': fingerprint(args.output / 'field-observation/results.jsonl')})


def field_compare(args):
    require(args.baseline and args.candidate, 'field-compare requires observation directories')
    require(args.abs_tol is not None and args.rel_tol is not None,
            'field-compare requires the existing explicit --abs-tol and --rel-tol')
    for kind, folder in (('baseline', args.baseline), ('candidate', args.candidate)):
        record = json.loads((folder / 'execution.json').read_text(encoding='utf-8'))
        require(record.get('schema') == 'rescue.field-execution.v1' and
                record.get('complete') is True and record['kind'] == kind,
                'Field comparison requires verified baseline and candidate executions')
        if kind == 'baseline':
            require(record['verified_baseline'] == BASELINE, 'Field execution uses another baseline')
        for key, name in (('identity', 'identity.json'), ('results', 'results.jsonl')):
            require(fingerprint(folder / name) == record[key], 'Field artifact changed after execution: ' + name)
    command([sys.executable, ROOT / 'src/vision_pkg/checks/compare_field_sequence.py', 'compare',
             '--baseline', args.baseline, '--candidate', args.candidate, '--abs-tol', args.abs_tol,
             '--rel-tol', args.rel_tol, '--report', args.output / 'field-comparison.json'],
            args.output / 'field-comparison.log')



# ---- 编辑器合同：从真实构建导出数据库，再检查 clangd 行为 ----

def clangd_contract(args):
    database = ROOT / 'build/unified-clangd'
    command([sys.executable, ROOT / 'tools/clangd/clangd_database.py', '--workspace', ROOT,
             '--build-base', args.build, '--output', database], args.output / 'database-export.log')
    command([sys.executable, ROOT / 'tools/validation/clangd_contract.py', '--workspace', ROOT,
             '--database', database, '--output', args.output / 'clangd-observation'],
            args.output / 'clangd-contract.log')


# ---- CLI：解析阶段、建立隔离环境、分发并记录执行/阻塞/失败 ----

STATUS_EXIT_CODES = {'executed': 0, 'failed': 1, 'blocked': 77}


def write_status(args, status, reason=None):
    """同目录替换收据，读者只会看见完整 running 或终态，不会采信半写入文件。"""
    record = {'stage': args.stage, 'run_id': args.run_id, 'status': status,
              'exit_code': STATUS_EXIT_CODES.get(status)}
    if reason is not None:
        record['reason'] = reason
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=args.output,
                                         prefix='.status-', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(record, stream, indent=2, ensure_ascii=False, allow_nan=False)
        os.replace(temporary, args.output / 'status.json')
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def worker_receipt(args, returncode):
    # 收据须绑定本次 stage/run_id，并与实际退出码一致；旧收据或 running 状态均拒绝。
    try:
        record = json.loads((args.output / 'status.json').read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise ContractFailure('namespace worker has no valid status receipt') from error
    require(isinstance(record, dict) and record.get('stage') == args.stage and
            record.get('run_id') == args.run_id, 'namespace worker status stage/run_id differs')
    status = record.get('status')
    require(isinstance(status, str) and status in STATUS_EXIT_CODES,
            'namespace worker has no terminal status receipt')
    require(type(record.get('exit_code')) is int and
            record['exit_code'] == returncode == STATUS_EXIT_CODES[status],
            'namespace worker exit code/status differs')
    return returncode


@contextmanager
def defer_creation_interrupt():
    """沿用 profiler 的窄创建期策略：保存句柄后再抛取消，不让子进程继承信号屏蔽。"""
    pending = False
    def postpone(_signum, _frame):
        nonlocal pending
        pending = True
    previous = signal.signal(signal.SIGINT, postpone)
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, previous)
        if pending:
            raise KeyboardInterrupt()


def isolated_stage(args, forwarded):
    require(shutil.which('unshare') and shutil.which('ip'), 'unshare and ip are required')
    env = os.environ.copy()
    env['RESCUE_TEST_HOST_NETNS'] = os.readlink('/proc/self/ns/net')
    env['RESCUE_VALIDATION_RUN_ID'] = args.run_id
    witness = worker = None
    interrupted_handler = None
    try:
        if args.stage == 'camera':
            snapshot = configuration().load_configuration(args.config, 'all')
            with defer_creation_interrupt():
                witness = HostFileUseWitness(snapshot['values']['vision.detect.camera_device'],
                                            args.output / 'host-camera-witness.json')
            env['RESCUE_CAMERA_WITNESS_FD'] = str(witness.worker_fd)
        with defer_creation_interrupt():
            worker = subprocess.Popen(['unshare', '--user', '--map-root-user', '--net',
                                       sys.executable, str(ROOT / 'tools/validate.py'), *forwarded],
                                      env=env, pass_fds=(witness.worker_fd,) if witness else (),
                                      start_new_session=True)
        returncode = worker.wait()
    except KeyboardInterrupt:
        # 先取消本轮 worker 进程组，再等 worker 回收独立业务会话，最后关闭宿主见证。
        # worker 退出后仍残留组员时强制回收，并将本轮判为失败。
        interrupted_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        if worker is not None:
            try:
                os.killpg(worker.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            worker.wait()
            deadline = time.monotonic() + 3
            while group_members(worker.pid) and time.monotonic() < deadline:
                time.sleep(.05)
            if group_members(worker.pid):
                try:
                    os.killpg(worker.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                deadline = time.monotonic() + 3
                while group_members(worker.pid) and time.monotonic() < deadline:
                    time.sleep(.05)
                require(not group_members(worker.pid), 'namespace worker group cleanup failed')
                raise ContractFailure('namespace worker left descendants; forced group cleanup')
        raise
    finally:
        try:
            if witness:
                witness.close()
        finally:
            if interrupted_handler is not None:
                signal.signal(signal.SIGINT, interrupted_handler)
    return worker_receipt(args, returncode)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('freeze', 'prepare', 'build', 'contracts', 'capture', 'compare',
                                         'field', 'field-compare', 'transport', 'transport-compare', 'clangd', 'performance', 'performance-compare', 'camera'))
    parser.add_argument('--config', type=Path, default=ROOT / 'src/bringup_pkg/config/rescue.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--build', type=Path, default=ROOT / 'build/unified-runtime')
    parser.add_argument('--install', type=Path, default=ROOT / 'install/unified-runtime')
    parser.add_argument('--source', type=Path, default=ROOT)
    parser.add_argument('--probe', type=Path)
    parser.add_argument('--kind', choices=('baseline', 'candidate'), default='candidate')
    parser.add_argument('--business-version', default='working-tree')
    parser.add_argument('--inputs', type=Path)
    parser.add_argument('--input', type=Path)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--candidate', type=Path)
    parser.add_argument('--abs-tol', type=float)
    parser.add_argument('--rel-tol', type=float)
    parser.add_argument('--cmake-arg', action='append', default=[])
    parser.add_argument('--isolated', action='store_true', help='create a private network namespace and vcan0')
    args = parser.parse_args(argv)
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    inherited_run = os.environ.get('RESCUE_VALIDATION_RUN_ID') if (
        not args.isolated and os.environ.get('RESCUE_TEST_HOST_NETNS')) else None
    args.run_id = inherited_run or uuid.uuid4().hex
    handlers = {'freeze': freeze, 'prepare': prepare, 'build': build, 'contracts': contracts, 'capture': capture,
                'field': field_sequence, 'field-compare': field_compare,
                'transport': transport_probe, 'transport-compare': transport_compare, 'clangd': clangd_contract,
                'compare': compare, 'performance': camera_or_performance, 'camera': camera_or_performance,
                'performance-compare': performance_compare}
    try:
        write_status(args, 'running')
        # 统一入口仍面向板端；隔离 vcan 的证据不能代替物理 CAN 验收。
        require(sys.platform == 'linux', 'all project builds/tests/hardware checks run on the board')
        if args.isolated:
            forwarded = list(sys.argv[1:] if argv is None else argv)
            forwarded.remove('--isolated')
            return isolated_stage(args, forwarded)
        if os.environ.get('RESCUE_TEST_HOST_NETNS'):
            require(os.readlink('/proc/self/ns/net') != os.environ['RESCUE_TEST_HOST_NETNS'],
                    'namespace isolation did not happen')
            if subprocess.run(['ip', 'link', 'show', 'vcan0'], capture_output=True).returncode != 0:
                try:
                    command(['ip', 'link', 'add', 'vcan0', 'type', 'vcan'], args.output / 'vcan-create.log')
                    command(['ip', 'link', 'set', 'vcan0', 'up'], args.output / 'vcan-up.log')
                except ContractFailure as error:
                    raise CapabilityMissing('isolated vcan unavailable: ' + str(error)) from error
            command(['ip', 'link', 'set', 'lo', 'up'], args.output / 'loopback-up.log')
        handlers[args.stage](args)
        write_status(args, 'executed')
        return 0
    except (Exception, KeyboardInterrupt) as error:
        status = 'blocked' if isinstance(error, CapabilityMissing) else 'failed'
        reason = f'{type(error).__name__}: {error}'
        write_status(args, status, reason)
        print(reason, file=sys.stderr)
        return STATUS_EXIT_CODES[status]
