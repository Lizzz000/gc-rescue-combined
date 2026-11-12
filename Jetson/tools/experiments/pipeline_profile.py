#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""按生产开关测识别吞吐与源帧延迟；不主动开启掩膜，也不订阅整幅图像。"""
from __future__ import annotations

import argparse
from collections import deque
from contextlib import contextmanager
import copy
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time


class ProfileCancelled(BaseException):
    """信号先进入受控清理；不直接结束持有独立业务会话的验收 worker。"""
    def __init__(self, signum):
        super().__init__('profile cancelled by ' + signal.Signals(signum).name)
        self.signum = signum


def protect_cleanup():
    # 第二次 Ctrl-C/TERM 不截断 HTTP、launcher、相机及 witness 的有序收尾。
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)


_creation_depth = 0
_pending_signal = None


def cancel_profile(signum, _frame):
    global _pending_signal
    if _creation_depth:
        # 仅推迟 Python 异常，不屏蔽内核信号；子进程不会继承 blocked INT/TERM。
        if _pending_signal is None:
            _pending_signal = signum
        return
    protect_cleanup()
    raise ProfileCancelled(signum)


@contextmanager
def defer_creation_cancel():
    """创建自有资源并保存回收句柄后，再交付期间收到的首个停止信号。"""
    global _creation_depth, _pending_signal
    _creation_depth += 1
    try:
        yield
    finally:
        _creation_depth -= 1
        if _creation_depth == 0 and _pending_signal is not None:
            signum, _pending_signal = _pending_signal, None
            protect_cleanup()
            raise ProfileCancelled(signum)


class HttpWindowCounter:
    """用同一把短锁划分开始、完整帧入账和结束，统计不延伸到分位数计算期。"""
    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.started = None
        self.frames = 0
        self.first = self.last = None

    def begin(self):
        with self.lock:
            if self.active:
                raise RuntimeError('HTTP measurement window already active')
            self.started = time.monotonic()
            self.frames = 0
            self.first = self.last = None
            self.active = True
            return self.started

    def complete(self):
        with self.lock:
            if self.active:
                at = time.monotonic()
                self.frames += 1
                if self.first is None:
                    self.first = at
                self.last = at

    def finish(self):
        with self.lock:
            if not self.active:
                raise RuntimeError('HTTP measurement window is not active')
            ended = time.monotonic()
            self.active = False
            return {'steady_start': self.started, 'steady_end': ended,
                    'complete_parts': self.frames,
                    'first_complete_steady': self.first, 'last_complete_steady': self.last}

    def cancel(self):
        with self.lock:
            self.active = False


class CountingHttpHistory(deque):
    """保留原 MJPEG 历史容量；完整解码后的 append 另向当前测量窗口入账。"""
    def __init__(self, counter, maxlen):
        super().__init__(maxlen=maxlen)
        self.counter = counter

    def append(self, item):
        super().append(item)
        self.counter.complete()


def window_counts(rows, started, ended, task):
    """按观察者回调起始时刻分桶；尾部不足一秒不参与最慢完整秒。"""
    seconds = math.floor(ended - started)
    buckets = [{'index': index, 'steady_start': started + index,
                'steady_end': started + index + 1.0, 'output_frames': 0,
                'field_valid_frames': 0, 'field_committed_frames': 0,
                'field_valid_and_committed_frames': 0, 'detection_instances': 0}
               for index in range(seconds)]
    totals = {'field_frames': 0, 'field_valid_frames': 0, 'field_committed_frames': 0,
              'field_valid_and_committed_frames': 0, 'detection_frames': 0,
              'detection_nonempty_frames': 0, 'detection_instances': 0}
    topic = 'field' if task == 'field' else 'detections'
    for name, at, valid, committed, instances in rows:
        if not started <= at < ended:
            continue
        if name == 'field':
            totals['field_frames'] += 1
            totals['field_valid_frames'] += int(valid)
            totals['field_committed_frames'] += int(committed)
            totals['field_valid_and_committed_frames'] += int(valid and committed)
        else:
            totals['detection_frames'] += 1
            totals['detection_nonempty_frames'] += int(instances > 0)
            totals['detection_instances'] += instances
        index = int(at - started)
        if index >= seconds:
            continue
        bucket = buckets[index]
        bucket['output_frames'] += int(name == topic)
        if name == 'field':
            bucket['field_valid_frames'] += int(valid)
            bucket['field_committed_frames'] += int(committed)
            bucket['field_valid_and_committed_frames'] += int(valid and committed)
        else:
            bucket['detection_instances'] += instances
    return {
        'output_topic': topic, 'counts': totals, 'complete_seconds': seconds,
        'partial_tail_seconds': ended - started - seconds,
        'per_complete_second': buckets,
        'minimum_complete_second_output_frames':
            min(row['output_frames'] for row in buckets) if buckets else None,
        'count_semantics': 'observed ROS messages, including empty detections and invalid field results',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--install', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--input', type=Path, help='仅替换采集源，仍按生产尺寸和帧率交付')
    parser.add_argument('--warmup', type=float, default=120)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--tasks', nargs='+', choices=('seg', 'field'), default=['seg', 'field'])
    parser.add_argument('--isolated', action='store_true')
    args = parser.parse_args()
    for key in ('source', 'build', 'install', 'config'):
        setattr(args, key, getattr(args, key).resolve(strict=True))
    args.output = args.output.resolve()
    sys.path.insert(0, str(args.source / 'tools'))
    from validation import workflow
    from validation.evidence import (HostFileUseWitness, asset_snapshot, fingerprint,
                                     host_file_use, isolated_vcan, port_open, require,
                                     write_json)

    if args.isolated:
        # 仅父进程创建一次；连空的既有目录也拒绝，避免意外覆盖旧证据。
        args.output.mkdir(parents=True, exist_ok=False)
        env = os.environ.copy()
        env['RESCUE_TEST_HOST_NETNS'] = os.readlink('/proc/self/ns/net')
        env['RESCUE_PROFILE_OUTPUT'] = str(args.output)
        witness = None
        worker = None
        try:
            if args.input is None:
                snapshot = workflow.configuration().load_configuration(args.config, 'all')
                with defer_creation_cancel():
                    witness = HostFileUseWitness(
                        snapshot['values']['vision.detect.camera_device'],
                        args.output / 'host-camera-witness.json')
                env['RESCUE_CAMERA_WITNESS_FD'] = str(witness.worker_fd)
            forwarded = sys.argv[1:]
            forwarded.remove('--isolated')
            with defer_creation_cancel():
                worker = subprocess.Popen(
                    ['unshare', '--user', '--map-root-user', '--net',
                     sys.executable, str(Path(__file__).resolve()), *forwarded], env=env,
                    pass_fds=(witness.worker_fd,) if witness else (), start_new_session=True)
            return worker.wait()
        except BaseException:
            protect_cleanup()
            if worker is not None:
                if worker.poll() is None:
                    try:
                        worker.send_signal(signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                # witness 必须活到 worker 核对相机释放以后。不要先 kill worker，
                # 否则它在独立会话启动的 launcher/业务进程将失去清理者。
                worker.wait()
            raise
        finally:
            protect_cleanup()
            if witness:
                witness.close()

    require(os.environ.get('RESCUE_PROFILE_OUTPUT') == str(args.output) and
            args.output.is_dir() and not any(args.output.iterdir()),
            'worker 只能使用本次 --isolated 父进程新建的空输出目录')
    require(os.environ.get('RESCUE_TEST_HOST_NETNS') and
            os.readlink('/proc/self/ns/net') != os.environ['RESCUE_TEST_HOST_NETNS'],
            '必须使用独立网络命名空间，不操作物理 CAN')
    for command in (['ip', 'link', 'add', 'vcan0', 'type', 'vcan'],
                    ['ip', 'link', 'set', 'vcan0', 'up'],
                    ['ip', 'link', 'set', 'lo', 'up']):
        subprocess.run(command, check=True, capture_output=True)
    isolated_vcan()

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    import yaml
    from validation.observations import MjpegClient, Observer, window_statistics

    class ProfileMjpegClient(MjpegClient):
        def __init__(self, url):
            super().__init__(url)
            self.window_counter = HttpWindowCounter()
            self.history = CountingHttpHistory(self.window_counter, self.history.maxlen)

    class ProfileObserver(Observer):
        """沿用原观察/日志逻辑，仅在窗口内追加小型计数记录。"""
        def __init__(self, snapshot, directory):
            self.window_rows = None
            super().__init__(snapshot, directory)

        def on_message(self, name, message):
            super().on_message(name, message)
            if self.window_rows is None or name not in ('field', 'detections'):
                return
            record = self.records[name][-1]
            value = record['value']
            self.window_rows.append(
                (name, record['steady'], bool(value.get('valid', False)),
                 bool(value.get('committed', False)), len(value.get('detections', ()))))

        def performance(self, seconds, owner, task, http=None):
            # 与 Observer.performance 保持同一采样/清空/分位数口径；这里显式暴露
            # 原窗口单调时钟边界，使秒桶和吞吐分母来自同一次 spin。
            before = self.counts.copy()
            self.latency.clear()
            self.timings.clear()
            rows = []
            self.window_rows = rows
            counter = http.window_counter if http else None
            started = counter.begin() if counter else time.monotonic()
            http_window = None
            try:
                self.spin(seconds, owner, http)
                if counter:
                    http_window = counter.finish()
                    ended = http_window['steady_end']
                else:
                    ended = time.monotonic()
            except BaseException:
                if counter:
                    counter.cancel()
                raise
            finally:
                self.window_rows = None
            elapsed = ended - started
            result = window_statistics(task, elapsed, before, self.counts, self.timings, self.latency)
            result.update(steady_start=started, steady_end=ended, http_window=http_window,
                          debug_mjpeg_fps=http_window['complete_parts'] / elapsed if http_window else None,
                          recognition=window_counts(rows, started, ended, task))
            return result

    args.kind = 'candidate'  # 本工具双方均为有来源记录的独立当前协议安装。
    args.stage = 'pipeline-profile'
    cfg = workflow.configuration()
    original = cfg.load_configuration(args.config, 'all')
    values = copy.deepcopy(original['values'])
    require(values['vision.detect.publish_timing'], '生产配置未开启耗时消息，不能伪造分段数据')
    overrides = {'bridge.transport': 'socketcan', 'damiao.device': '', 'bridge.can_bitrate': 0,
                 'bridge.can_interface': 'vcan0',
                 'vision.detect.active_task': 'seg',
                 'vision.stream.stream_address': '127.0.0.1'}
    with socket.socket() as channel:
        channel.bind(('127.0.0.1', 0))
        overrides['vision.stream.stream_port'] = channel.getsockname()[1]
    if args.input:
        args.input = args.input.resolve(strict=True)
        overrides.update({'vision.detect.camera_backend': 'file',
                          'vision.detect.camera_device': str(args.input)})
    values.update(overrides)
    case_config = args.output / 'case-config.yaml'
    case_config.write_text(yaml.safe_dump(workflow.flat_to_document(values),
                                         sort_keys=False), encoding='utf-8')
    snapshot = cfg.load_configuration(case_config, 'all')
    port = values['vision.stream.stream_port']
    require(not port_open(port), '本轮 HTTP 端口已被占用')
    require(values['vision.stream.stream_enabled'], '此对照要求保留生产图传服务与单个调试客户端')
    before = asset_snapshot(snapshot)
    report = {'schema': 'pipeline-profile-v3', 'status': 'failed', 'configuration': snapshot,
              'production_configuration': original, 'explicit_execution_overrides': overrides,
              'assets_before': before, 'windows': [], 'resources': [],
              'stage_control': 'detector-development-only', 'mission_handshake_tested': False,
              'measurement': 'new ROS results and source-header-to-observer latency',
              'observer_images_subscribed': False,
              'warmup_policy': {
                  'initial_task': 'seg', 'requested_seconds': args.warmup,
                  'http_client_enabled': False,
                  'per_task_settle_seconds': 10, 'per_task_http_client_enabled': True},
              'task_settle': [],
              'http_measurement': {
                  'role': 'secondary debugging observation',
                  'client_count': 1, 'client_runs_on_jetson': True,
                  'per_frame_work': 'multipart parsing, complete JPEG check, CPU imdecode and SHA-256',
                  'debug_mjpeg_fps': 'complete HTTP parts decoded by this local client per window second',
                  'window_boundary': 'locked complete-part admission over the same steady interval as ROS spin',
                  'not_server_submit_fps': True},
              'latency_semantics': 'source header to ROS observer; not sensor exposure to display',
              'input_before': fingerprint(args.input) if args.input else None,
              'tool': fingerprint(Path(__file__).resolve())}
    power = subprocess.run(['nvpmodel', '-q'], capture_output=True, text=True, check=True)
    report['power_mode'] = power.stdout.strip()
    if args.input is None:
        ownership = host_file_use(values['vision.detect.camera_device'], 'before')
        report['camera_ownership_before'] = ownership
        require(ownership['free'], '相机被其他进程使用，保留其他项目')
    write_json(args.output / 'before.json', report)

    observer = owner = http = None
    context_started = False
    try:
        # 保留本工具的信号收尾，不让 rclpy 先关闭上下文或接管 SIGINT/SIGTERM。
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        context_started = True
        observer = ProfileObserver(snapshot, args.output)
        # 整图哈希/跨进程图像订阅属于效果回归；性能观察器避免额外强迫生产者复制大图。
        for subscription in list(observer.subscriptions):
            if subscription.topic_name in ('/detect_node/image_processed', '/detect_node/seg/masks'):
                observer.node.destroy_subscription(subscription)
                observer.subscriptions.remove(subscription)
        with defer_creation_cancel():
            owner = workflow.launch_case(args, snapshot, args.output)
        observer.wait(lambda: observer.states and observer.records['detections'],
                      owner=owner, reason='initial real seg source')
        started = time.monotonic()
        observer.spin(args.warmup, owner)
        report['actual_warmup_s'] = time.monotonic() - started
        report['warmup_policy']['steady_start'] = started
        report['warmup_policy']['steady_end'] = started + report['actual_warmup_s']
        http = ProfileMjpegClient('http://127.0.0.1:' + str(port))
        http.start()
        observer.wait(lambda: http.frames > 0, owner=owner, http=http, reason='debug MJPEG client')
        for task in args.tasks:
            observer.switch(task, owner, direct=True, http=http)
            settle_started = time.monotonic()
            observer.spin(10, owner, http)
            settle_ended = time.monotonic()
            report['task_settle'].append({
                'task': task, 'steady_start': settle_started, 'steady_end': settle_ended,
                'actual_seconds': settle_ended - settle_started, 'http_client_enabled': True})
            for index in range(args.rounds):
                window = observer.performance(args.seconds, owner, task, http)
                window.update(round=index + 1)
                report['windows'].append(window)
                report['resources'].append(workflow.resource_sample(owner))
                write_json(args.output / 'progress.json', report)
        report['status'] = 'observed_pending_shutdown'
    except BaseException as error:
        report['error'] = str(error)
        report['error_type'] = type(error).__name__
        if isinstance(error, ProfileCancelled):
            report['cancelled_signal'] = error.signum
        raise
    finally:
        protect_cleanup()
        workflow.cleanup_case(report, observer, owner, http, None, snapshot, before,
                              workflow.FULL_ROLES, port)
        if context_started:
            try:
                rclpy.shutdown()
            except BaseException as error:
                report['cleanup_errors'].append({'step': 'rclpy.shutdown', 'error': str(error)})
        if args.input:
            try:
                report['input_unchanged'] = fingerprint(args.input) == report['input_before']
            except BaseException as error:
                report['cleanup_errors'].append({'step': 'input-fingerprint', 'error': str(error)})
        else:
            try:
                ownership = host_file_use(values['vision.detect.camera_device'], 'after')
                report['camera_ownership_after'] = ownership
                report['camera_released'] = ownership['free']
            except BaseException as error:
                report['cleanup_errors'].append({'step': 'camera-release', 'error': str(error)})
        report['status'] = 'executed' if (
            report['status'] == 'observed_pending_shutdown' and not report['cleanup_errors']
            and report.get('shutdown', {}).get('clean') and report.get('assets_unchanged')
            and report.get('input_unchanged', True) and report.get('camera_released', True)) else 'failed'
        write_json(args.output / 'result.json', report)
    require(report['status'] == 'executed', '测量或资源清理检查失败')
    return 0


if __name__ == '__main__':
    signal.signal(signal.SIGINT, cancel_profile)
    signal.signal(signal.SIGTERM, cancel_profile)
    try:
        raise SystemExit(main())
    except ProfileCancelled as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(128 + error.signum)
