# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""真实 ROS/HTTP 观察者。仅在板端显式调用，不替代生产模型执行。"""
from __future__ import annotations

from collections import defaultdict, deque
import hashlib
import json
from pathlib import Path
import queue
import socket
import struct
import threading
import time
import urllib.request
import urllib.error

from .evidence import ContractFailure, isolated_vcan, require


# ---- 观测值基础：源帧身份、统计、原始消息与 JPEG 完整性 ----

def frame_key(header):
    return (header.stamp.sec * 1_000_000_000 + header.stamp.nanosec, header.frame_id)


def percentile(values, fraction=0.95):
    # fraction 由调用方保证在 [0,1] 内；线性插值结果沿用样本单位。
    require(bool(values), 'no samples for percentile')
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    low = int(index)
    return ordered[low] + (ordered[min(low + 1, len(ordered) - 1)] - ordered[low]) * (index - low)


def window_statistics(task, elapsed, before, counts, timings, latency):
    """窗口关闭后计算纯统计；采样、清空和 HTTP 边界仍由各观察器负责。"""
    # elapsed 为单调秒，before/counts 为窗口前后累计计数；timings/latency 仅含窗口内毫秒样本。
    # 当前阶段 timing 少于 10 条则失败；单话题 latency 少于 10 条仅省略该指标。
    rows = timings.get(task + '_timing', ())
    require(len(rows) >= 10, 'insufficient inference timing samples')
    return {'task': task, 'seconds': elapsed,
            'throughput': {name: (count - before.get(name, 0)) / elapsed for name, count in counts.items()},
            'inference_p95_ms': percentile([row['inference_ms'] for row in rows]),
            'publish_p95_ms': percentile([row['publish_ms'] for row in rows]),
            'source_to_observer_p95_ms': {name: percentile(values) for name, values in latency.items()
                                        if len(values) >= 10}}


def value_record(message):
    """完整保留原始头、业务编号和业务字段；跨运行归一化只在比较阶段进行。"""
    from rosidl_runtime_py.convert import message_to_ordereddict
    value = dict(message_to_ordereddict(message))
    return value


def jpeg_check(data):
    import cv2
    import numpy as np
    require(data.startswith(b'\xff\xd8') and data.endswith(b'\xff\xd9'), 'incomplete JPEG')
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    require(image is not None, 'JPEG cannot be decoded')
    return {'bytes': len(data), 'width': image.shape[1], 'height': image.shape[0],
            'sha256': hashlib.sha256(data).hexdigest()}


# ---- HTTP 观察：完整 multipart、阶段切换窗口及后台错误回传 ----

class MjpegDisconnect(ConnectionError):
    """已建立multipart连接在边界、头或载荷中结束；协议错误使用ContractFailure。"""


class MjpegClient:
    """只在主线程声明的切阶段窗口恢复服务端reset，其他后台错误交还主线程。"""
    def __init__(self, base_url):
        self.url = base_url
        self.stop_event = threading.Event()
        self.errors = queue.Queue()
        self.frames = 0
        self.connections = 0
        self.active_reconnects = 0
        self.automatic_reconnect_attempts = 0
        self.automatic_reconnections = 0
        self.latest = None
        self.response = None
        self.thread = None
        self.history = deque(maxlen=3000)
        self.transport_lock = threading.Lock()
        self.transition = None
        self.transport_events = []

    def _event(self, **values):
        with self.transport_lock:
            self.transport_events.append(dict(steady=time.monotonic(), **values))

    def allow_stage_transition(self, task):
        # 只有发送阶段请求的主线程开窗；窗口外EOF、断线和所有超时仍为失败。
        now = time.monotonic()
        with self.transport_lock:
            identifier = (self.transition['id'] + 1) if self.transition else 1
            self.transition = {'id': identifier, 'task': task, 'started': now, 'deadline': now + 2.0}
            self.transport_events.append(dict(event='stage-transition-window', **self.transition))

    def evidence(self):
        with self.transport_lock:
            events = list(self.transport_events)
        return {'connections': self.connections, 'active_reconnects': self.active_reconnects,
                'automatic_reconnect_attempts': self.automatic_reconnect_attempts,
                'automatic_reconnections': self.automatic_reconnections, 'events': events}

    def start(self):
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._receive, name='validation-mjpeg', daemon=True)
        self.thread.start()

    def _receive(self):
        automatic = False
        while not self.stop_event.is_set():
            established = False
            progress = {'phase': 'http-response', 'expected_bytes': None, 'received_bytes': 0}
            try:
                with urllib.request.urlopen(self.url + '/stream.mjpg', timeout=4) as response:
                    self.response = response
                    require('multipart/x-mixed-replace' in response.headers.get('Content-Type', ''),
                            'MJPEG content type differs')
                    boundary = response.headers.get_param('boundary')
                    require(boundary and len(boundary) < 200, 'multipart boundary parameter missing')
                    delimiter = b'--' + boundary.encode('ascii')
                    established = True
                    self.connections += 1
                    if automatic:
                        self.automatic_reconnections += 1
                    self._event(event='connected', connection=self.connections,
                                origin='automatic-stage-recovery' if automatic else 'initial-or-active')
                    while not self.stop_event.is_set():
                        progress = {'phase': 'multipart-boundary', 'expected_bytes': None, 'received_bytes': 0}
                        line = response.readline(4096)
                        if not line:
                            raise MjpegDisconnect('MJPEG ended before multipart boundary')
                        if not line.endswith(b'\n'):
                            require(len(line) < 4096, 'multipart boundary line exceeds limit')
                            progress['received_bytes'] = len(line)
                            raise MjpegDisconnect('MJPEG ended within multipart boundary line')
                        if not line.strip():
                            continue
                        require(line.strip() == delimiter, 'MJPEG boundary differs from HTTP declaration')
                        progress['phase'] = 'multipart-headers'
                        headers = {}
                        while True:
                            line = response.readline(4096)
                            if not line:
                                raise MjpegDisconnect('MJPEG ended within multipart headers')
                            progress['received_bytes'] += len(line)
                            if not line.endswith(b'\n'):
                                require(len(line) < 4096, 'multipart header line exceeds limit')
                                raise MjpegDisconnect('MJPEG ended within multipart header line')
                            if line in (b'\r\n', b'\n'):
                                break
                            name, value = line.decode('ascii').split(':', 1)
                            headers[name.lower()] = value.strip()
                        length = int(headers.get('content-length', '0'))
                        require(headers.get('content-type') == 'image/jpeg', 'multipart payload is not JPEG')
                        require(0 < length < 50_000_000, 'invalid MJPEG Content-Length')
                        progress = {'phase': 'jpeg-payload', 'expected_bytes': length, 'received_bytes': 0}
                        chunks, left = [], length
                        while left:
                            part = response.read(left)
                            if not part:
                                raise MjpegDisconnect('MJPEG ended within JPEG payload')
                            chunks.append(part)
                            left -= len(part)
                            progress['received_bytes'] += len(part)
                        self.latest = dict(jpeg_check(b''.join(chunks)), steady=time.monotonic(),
                                           headers=headers, connection=self.connections)
                        self.history.append(self.latest)
                        self.frames += 1
            except Exception as error:
                if self.stop_event.is_set():
                    break
                now = time.monotonic()
                with self.transport_lock:
                    window = dict(self.transition) if self.transition else None
                recoverable = (established and isinstance(error, (MjpegDisconnect, ConnectionResetError,
                               ConnectionAbortedError, BrokenPipeError)) and window and now <= window['deadline'])
                self._event(event='disconnect', connection=self.connections, reason=str(error),
                            error_type=type(error).__name__, partial=progress, transition=window,
                            automatic_recovery_allowed=bool(recoverable))
                if not recoverable:
                    self.errors.put(error)
                    break
                self.automatic_reconnect_attempts += 1
                automatic = True
                self.stop_event.wait(0.03)
            finally:
                self.response = None

    def check(self):
        if not self.errors.empty():
            raise ContractFailure('MJPEG worker: ' + str(self.errors.get()))

    def stop(self):
        self.stop_event.set()
        if self.response:
            self.response.close()
        if self.thread:
            self.thread.join(timeout=5)
            require(not self.thread.is_alive(), 'MJPEG worker did not stop')
        self.check()

    def reconnect(self):
        self.stop()
        self.active_reconnects += 1
        self._event(event='active-reconnect', request=self.active_reconnects)
        self.start()

    def snapshot(self, not_after=None):
        while True:
            require(not_after is None or time.monotonic() < not_after,
                    'measurement ended before snapshot request; no new request sent')
            try:
                with urllib.request.urlopen(self.url + '/snapshot.jpg', timeout=4) as response:
                    require(response.headers.get_content_type() == 'image/jpeg', 'snapshot content type differs')
                    return dict(jpeg_check(response.read()), steady=time.monotonic())
            except urllib.error.HTTPError as error:
                now = time.monotonic()
                with self.transport_lock:
                    window = dict(self.transition) if self.transition else None
                retry = (error.code in (404, 503) and window and now < window['deadline'] and
                         (not_after is None or now < not_after))
                self._event(event='snapshot-unavailable', http_status=error.code, transition=window,
                            retry_allowed=bool(retry), reason=str(error))
                error.close()
                if not retry:
                    raise
                remaining = window['deadline'] - now
                if not_after is not None:
                    remaining = min(remaining, not_after - now)
                self.stop_event.wait(min(0.03, max(0, remaining)))

    def phase_evidence(self, state, settle_seconds):
        # HTTP 协议没有源 header；只证明阶段稳定窗口收到完整新 part，不冒称逐帧 ROS/HTTP 配对。
        frames = [item for item in list(self.history) if item['steady'] >= state['steady'] + settle_seconds]
        require(frames, 'no complete MJPEG part in settled phase: ' + state['task'])
        return {'complete_parts': len(frames), 'first_sequence': frames[0]['headers'].get('x-frame-sequence'),
                'last_sequence': frames[-1]['headers'].get('x-frame-sequence'),
                'source_pairing_available': False}


# ---- 隔离 CAN 观察：握手、停发窗口与同窗口 ROS 载荷核对 ----

class CanProbe:
    def __init__(self, directory):
        self.proof = isolated_vcan()  # 先证明隔离，再绑定；不能意外打开生产CAN。
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.socket.bind(('vcan0',))
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            raise
        self.records = []
        self.raw = (Path(directory) / 'can.jsonl').open('w', encoding='utf-8')

    def poll(self):
        for _ in range(256):
            try:
                payload = self.socket.recv(16)
            except BlockingIOError:
                break
            require(len(payload) == 16, 'unexpected classic CAN frame size')
            identifier, length, data = struct.unpack('=IB3x8s', payload)
            require(not identifier & 0xE0000000, 'extended/RTR/error frame in isolated validation')
            record = {'steady': time.monotonic(), 'id': identifier, 'dlc': length, 'data': data[:length].hex()}
            self.records.append(record)
            self.raw.write(json.dumps(record) + '\n')

    def command(self, identifier, token=None):
        """只允许新握手事件；接口已证明隔离，不向真实 can0 发送。"""
        from source.can_codec import encode
        names = {0x200: 'arrival', 0x203: 'turn_completed', 0x204: 'safety_arrived',
                 0x205: 'start_search', 0x206: 'delivery_completed'}
        require(identifier in names and (identifier in (0x203, 0x204, 0x206)) == (token is not None),
                'unsupported mission event or missing/unexpected trip token')
        # 共用字节编解码，采集工具不能保留另一份旧协议或自行截断非法 token。
        data = bytes.fromhex(encode(names[identifier], [] if token is None else [str(token)]).split('#')[1])
        began = time.monotonic()
        payload = struct.pack('=IB3x8s', identifier, len(data), data)
        require(self.socket.send(payload) == len(payload), 'incomplete classic CAN command write')
        self.raw.write(json.dumps({'direction': 'tx', 'steady': began, 'id': identifier,
                                   'dlc': len(data), 'data': data.hex()}) + '\n')
        return began

    def has_reply(self, identifier, payload, since):
        self.poll()
        return any(row['id'] == identifier and row['data'] == payload and
                   row['steady'] >= since for row in self.records)

    def require_paused(self, observer, owner):
        # 排空已交给内核的历史帧后观察停发窗口，任务等待期间不生成新坐标。
        observer.spin(0.2, owner)
        since = time.monotonic()
        observer.spin(0.2, owner)
        self.poll()
        require(not any(row['id'] in (0x181, 0x182) and row['steady'] >= since and row['data'].startswith('01')
                        for row in self.records), 'coordinates emitted while mission waits for STM32')

    def steady_periodic(self, observer, task, since):
        """只比较收敛窗口，并将有效载荷与同窗口真实 ROS 输出逐项核对。"""
        self.poll()
        result = {}
        invalid = {0x180: bytes(8).hex(), 0x181: bytes(8).hex(), 0x182: bytes(8).hex()}
        expected = {0x180: set(), 0x181: set(), 0x182: set()}
        mission = observer.records['mission'][-1]['value'] if observer.records['mission'] else None
        idle = mission is not None and mission['operation'] == 0
        values = observer.snapshot['values']
        def quantize(value):
            # 共用文本工具的舍入，避免半整数邻域先加0.5造成二次舍入。
            from source.can_codec import fixed_point
            try:
                return fixed_point(value, 10000)
            except ValueError:
                return None
        if task == 'seg' and idle and mission.get('cycle_phase') == 0:
            for row in observer.records['target']:
                if row['steady'] < since:
                    continue
                detections = row['value']['detections']
                if not detections or not detections[0]['results']:
                    continue
                detection = detections[0]
                class_id = detection['results'][0]['hypothesis']['class_id']
                allowed = [str(item) for item in observer.snapshot['nodes']['bridge_node']['target_class_ids']]
                first = str(observer.snapshot['nodes']['bridge_node']['first_target_class_id'])
                if class_id not in allowed or (mission is None or mission['transport_phase'] == 0) and class_id != first:
                    continue
                center = detection['bbox']['center']['position']
                x = quantize(center['x'] / values['shared.image.width'])
                y = quantize(center['y'] / values['shared.image.height'])
                if x is not None and y is not None and detection['id']:
                    expected[0x181].add((struct.pack('<Bhh', 1, x, y) + ((row['source'][0] // 1000000) & 0xffffff).to_bytes(3, 'little')).hex())
        if task == 'field' and idle and mission.get('cycle_phase') == 4 and mission['outcome'] == 2 and mission['destination'] in (1, 2):
            for row in observer.records['safety']:
                safety = row['value']
                if row['steady'] < since or not safety['valid'] or safety['side'] != mission['destination']:
                    continue
                x, y = quantize(safety['x']), quantize(safety['y'])
                if x is not None and y is not None:
                    expected[0x182].add((struct.pack('<Bhh', 1, x, y) + ((row['source'][0] // 1000000) & 0xffffff).to_bytes(3, 'little')).hex())
        for identifier, length in ((0x180, 8), (0x181, 8), (0x182, 8)):
            rows = [row for row in self.records if row['id'] == identifier and row['steady'] >= since]
            require(rows and all(row['dlc'] == length for row in rows), f'missing/wrong CAN {identifier:03x}')
            require(all(row['data'][:2] in ('00', '01') for row in rows), 'CAN valid byte outside 0/1')
            if identifier == 0x180:
                require(all(row['data'][14:] in ('00', '01') for row in rows), '0x180 pause must be 0 or 1')
                require(all(row['data'][:14] == '00' * 7 for row in rows if row['data'].startswith('00')),
                        'invalid 0x180 frame has nonzero speed bytes')
            else:
                require(all(row['data'] == invalid[identifier] for row in rows if row['data'].startswith('00')),
                        'invalid CAN frame contains nonzero coordinates')
            actual = {row['data'] for row in rows if row['data'].startswith('01')}
            require(actual <= expected[identifier], f'CAN {identifier:03x} disagrees with ROS coordinates/gates')
            require(not expected[identifier] or actual, f'no valid CAN {identifier:03x} for valid ROS output')
            result[f'{identifier:03x}'] = {'valid_bytes': sorted(actual), 'ros_expected_bytes': sorted(expected[identifier]),
                                          'invalid_only': not actual}
        return result

    def close(self):
        self.socket.close()
        self.raw.close()


# ---- ROS 观察：原始消息、源帧配对、阶段证据与单调时间等待 ----

class Observer:
    def __init__(self, snapshot, directory):
        import rclpy
        from interface_pkg.msg import (TaskState, FieldObservation, InferenceTiming,
                                      MissionStatus, RoiCheck, SafetyTarget)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import Image
        from std_srvs.srv import SetBool
        from vision_msgs.msg import Detection2DArray
        self.rclpy = rclpy
        self.snapshot = snapshot
        self.node = rclpy.create_node('unified_validation_observer')
        self.raw = (Path(directory) / 'observations.jsonl').open('w', encoding='utf-8')
        self.records = defaultdict(lambda: deque(maxlen=250))
        self.counts = defaultdict(int)
        self.latency = defaultdict(list)
        self.timings = defaultdict(list)
        self.states = []
        self.subscriptions = []
        qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
        topics = {
            'detections': (Detection2DArray, '/detect_node/seg/detections'),
            'tracks': (Detection2DArray, '/track_node/seg/tracks'),
            'selected': (Detection2DArray, '/select_node/target'),
            'target': (Detection2DArray, '/target_node/target'),
            'field': (FieldObservation, '/detect_node/field/results'),
            'safety': (SafetyTarget, '/safety_node/target'),
            'mission': (MissionStatus, '/mission_node/status'),
            'roi': (RoiCheck, '/mission_node/roi_result'),
            'processed': (Image, '/detect_node/image_processed'),
            'masks': (Image, '/detect_node/seg/masks'),
            'seg_timing': (InferenceTiming, '/detect_node/seg/timing'),
            'field_timing': (InferenceTiming, '/detect_node/field/timing'),
        }
        for name, (message_type, topic) in topics.items():
            self.subscriptions.append(self.node.create_subscription(
                message_type, topic, lambda message, key=name: self.on_message(key, message),
                QoSProfile(depth=30, durability=DurabilityPolicy.TRANSIENT_LOCAL) if name == 'mission' else qos))
        self.subscriptions.append(self.node.create_subscription(
            TaskState, '/detect_node/active_task', self.on_state,
            QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.direct_switch = self.node.create_client(SetBool, '/detect_node/set_field')
        self.phase_started = 0
        self.pollers = []

    def on_state(self, message):
        record = {'task': message.task, 'epoch': message.epoch, 'header': value_record(message)['header'],
                  'source': frame_key(message.header), 'steady': time.monotonic()}
        self.states.append(record)
        self.raw.write(json.dumps(dict(record, topic='active_task'), ensure_ascii=False, allow_nan=False) + '\n')

    def on_message(self, name, message):
        at = time.monotonic()
        source = frame_key(message.header)
        if message.__class__.__name__ == 'Image':
            value = {'width': message.width, 'height': message.height, 'step': message.step,
                     'encoding': message.encoding, 'is_bigendian': message.is_bigendian,
                     'header': value_record(message.header),
                     'sha256': hashlib.sha256(bytes(message.data)).hexdigest()}
        else:
            value = value_record(message)
        record = {'topic': name, 'source': source, 'steady': at, 'value': value}
        self.records[name].append(record)
        self.counts[name] += 1
        if source[0] > 0:
            delay = (self.node.get_clock().now().nanoseconds - source[0]) / 1e6
            if delay >= 0:
                self.latency[name].append(delay)
        if name.endswith('_timing'):
            self.timings[name].append(value)
        self.raw.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')

    def spin(self, seconds, owner=None, http=None):
        deadline = time.monotonic() + max(0, seconds)
        while time.monotonic() < deadline:
            if owner:
                require(owner.process.poll() is None, 'launcher exited during observation')
                # 身份四次/秒足够；消息到达频率不应放大/proc扫描与验收器内存。
                owner.sample(maximum_age=0.25)
            if http:
                http.check()
            # Humble 在 timeout kwargs 改变时重建就绪生成器；短 wait 若每次传剩余时间，
            # 高频前列订阅会饿死末尾状态和服务应答。固定量子保持公平，截止仍由外层检查。
            self.rclpy.spin_once(self.node, timeout_sec=0.03)
            for poll in self.pollers:
                poll()

    def wait(self, predicate, timeout=30, reason='observation', owner=None, http=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.03, owner, http)
            if predicate():
                return
        raise ContractFailure('timeout: ' + reason)

    def switch(self, task, owner, direct=True, http=None):
        from std_srvs.srv import SetBool
        # 仅供视觉和生命周期开发回归；真实运输走 CAN 握手。
        require(direct, 'business mode switches require the STM32 mission handshake')
        client = self.direct_switch
        self.wait(client.service_is_ready, owner=owner, reason='stage service')
        before = len(self.states)
        previous = self.states[-1] if self.states else None
        if http and (previous is None or previous['task'] != task):
            http.allow_stage_transition(task)
        future = client.call_async(SetBool.Request(data=task == 'field'))
        self.wait(future.done, owner=owner, reason='stage reply')
        require(future.exception() is None and future.result().success, 'stage request rejected')
        def matching(item):
            return item['task'] == task and (previous is None or previous['task'] == task or
                    (item['epoch'], item['source']) != (previous['epoch'], previous['source']))
        self.wait(lambda: any(matching(item) for item in self.states[before:]),
                  owner=owner, reason='actual ' + task + ' confirmation')
        self.phase_started = time.monotonic()
        return next(item for item in reversed(self.states[before:]) if matching(item))

    def phase_evidence(self, task, state, minimum=1):
        key = 'field' if task == 'field' else 'detections'
        frames = [item for item in self.records[key] if item['source'][0] >= state['source'][0]
                  and item['steady'] >= state['steady']]
        require(len(frames) >= minimum, 'stage has no fresh source frame: ' + task)
        return {'task': task, 'epoch': state['epoch'], 'source_frames': len(frames),
                'stage_header': state['header'], 'valid_geometry_frames': sum(
                    bool(item['value'].get('valid') and item['value'].get('committed')) for item in frames)
                    if task == 'field' else None,
                'first_source': frames[0]['source'], 'last_source': frames[-1]['source']}

    def paired(self, names, minimum=3):
        mappings = {name: {tuple(row['source']): row['value'] for row in self.records[name]} for name in names}
        common = set.intersection(*(set(mapping) for mapping in mappings.values()))
        require(len(common) >= minimum, 'missing same-source pairing: ' + ','.join(names))
        pairs = [dict(source=key, **{name: mappings[name][key] for name in names}) for key in sorted(common)[-minimum:]]
        for pair in pairs:
            for name in names:
                value = pair[name]
                header = value['header']
                require((header['stamp']['sec'] * 1_000_000_000 + header['stamp']['nanosec'], header['frame_id']) ==
                        tuple(pair['source']), 'outer header/source mismatch: ' + name)
                for detection in value.get('detections', []):
                    require(detection['header'] == header, 'detection header differs from source: ' + name)
            if 'tracks' in pair and 'selected' in pair:
                require(all(detection in pair['tracks']['detections'] for detection in pair['selected']['detections']),
                        'selected ID/geometry/results are not from same-source tracks')
            if 'selected' in pair and 'target' in pair:
                require(all(detection in pair['selected']['detections'] for detection in pair['target']['detections']),
                        'target ID/geometry/results are not from same-source selection')
        return pairs

    def has_pairs(self, names, minimum=3):
        sources = [set(tuple(row['source']) for row in self.records[name]) for name in names]
        return bool(sources) and len(set.intersection(*sources)) >= minimum

    def performance(self, seconds, owner, task, http=None):
        before = self.counts.copy()
        self.latency.clear()
        self.timings.clear()
        start = time.monotonic()
        self.spin(seconds, owner, http)
        elapsed = time.monotonic() - start
        return window_statistics(task, elapsed, before, self.counts, self.timings, self.latency)

    def close(self):
        self.raw.close()
        self.node.destroy_node()


# ---- 观察器自身合同：真实 ROS 调度回归 ----

def run_spin_contract(output, domain_id=191):
    """显式真实 ROS 回归：高频前列订阅不能饿死末尾状态和服务应答。"""
    import inspect
    import os

    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)

    # 本入口在独立进程内使用隔离域及唯一命名，只创建本轮两个测试节点。
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import UInt64
    from std_srvs.srv import SetBool

    require(not rclpy.ok(), 'spin contract requires its own uninitialized ROS process')
    result = {'status': 'failed',
              'candidate_spin_sha256': hashlib.sha256(inspect.getsource(Observer.spin).encode()).hexdigest(),
              'rclpy_executors': str(Path(inspect.getfile(SingleThreadedExecutor)).resolve()),
              'domain_id': domain_id, 'rounds': [], 'context_shutdown': False,
              'fixture': {'busy_callback_s': 0.04, 'publish_interval_s': 0.002,
                          'deadline_s': 1.2, 'rounds_per_implementation': 3}}
    executor_file = Path(result['rclpy_executors'])
    result['rclpy_executors_sha256'] = hashlib.sha256(executor_file.read_bytes()).hexdigest()

    def run(label, spin):
        prefix = '/observer_spin_contract_' + str(os.getpid()) + '/' + label
        observer = rclpy.create_node('observer_' + label)
        peer = rclpy.create_node('peer_' + label)
        executor = SingleThreadedExecutor()
        executor.add_node(peer)
        errors = queue.Queue()
        stop = threading.Event()
        enabled = threading.Event()
        counts = {'early': 0, 'late': 0, 'service_calls': 0}
        early_pub = peer.create_publisher(UInt64, prefix + '/early', 256)
        late_pub = peer.create_publisher(UInt64, prefix + '/late', 256)

        def early(_):
            counts['early'] += 1
            # 明确模拟耗时订阅，保持首位队列就绪；不是依靠随机压力命中业务边界。
            time.sleep(0.04)

        def late(_):
            counts['late'] += 1

        subscriptions = [
            observer.create_subscription(UInt64, prefix + '/early', early, 256),
            observer.create_subscription(UInt64, prefix + '/late', late, 256)]

        def serve(request, response):
            counts['service_calls'] += 1
            response.success = request.data
            response.message = 'real response'
            return response

        service = peer.create_service(SetBool, prefix + '/set', serve)
        client = observer.create_client(SetBool, prefix + '/set')

        def publish():
            if enabled.is_set():
                early_pub.publish(UInt64(data=1))
                late_pub.publish(UInt64(data=1))

        timer = peer.create_timer(0.002, publish)

        def background():
            try:
                while not stop.is_set():
                    executor.spin_once(timeout_sec=0.01)
            except BaseException as error:
                errors.put(error)

        thread = threading.Thread(target=background, name='spin-contract-' + label)
        thread.start()
        record = {'label': label, 'own_peer_thread_joined': False, 'nodes_destroyed': False}
        try:
            deadline = time.monotonic() + 8
            while not (client.service_is_ready() and early_pub.get_subscription_count() == 1 and
                       late_pub.get_subscription_count() == 1):
                if not errors.empty():
                    raise errors.get()
                require(time.monotonic() < deadline, 'spin fixture discovery timeout')
                rclpy.spin_once(observer, timeout_sec=0.03)
            enabled.set()
            time.sleep(0.1)
            future = client.call_async(SetBool.Request(data=True))
            adapter = Observer.__new__(Observer)
            adapter.rclpy, adapter.node, adapter.pollers = rclpy, observer, []
            start = time.monotonic()
            while time.monotonic() - start < 1.2:
                if not errors.empty():
                    raise errors.get()
                spin(adapter, 0.03)
                if future.done() and counts['late'] > 0:
                    break
            record.update(elapsed_s=time.monotonic() - start, counts=dict(counts),
                          future_done=future.done(),
                          response_success=future.result().success if future.done() else None)
        finally:
            stop.set()
            executor.wake()
            thread.join(timeout=3)
            record['own_peer_thread_joined'] = not thread.is_alive()
            require(record['own_peer_thread_joined'], 'own spin fixture peer thread did not stop')
            executor.shutdown(timeout_sec=2)
            peer.destroy_node()
            observer.destroy_node()
            record['nodes_destroyed'] = True
            if not errors.empty():
                raise errors.get()
        return record

    rclpy.init()
    try:
        for index in range(3):
            row = run('positive' + str(index), Observer.spin)
            result['rounds'].append(row)
            print(json.dumps(row), flush=True)
            require(row['future_done'] and row['response_success'] and row['counts']['late'] > 0,
                    'current spin starved the late message or real service response')
        result['status'] = 'passed'
    except BaseException as error:
        result['reason'] = str(error)
        raise
    finally:
        rclpy.shutdown()
        result['context_shutdown'] = not rclpy.ok()
        (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n',
                                          encoding='utf-8')
    return result


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='显式运行真实 ROS 观察器调度回归，不启动生产节点或 GPU。')
    parser.add_argument('--spin-contract', action='store_true', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, choices=range(1, 233), default=191)
    options = parser.parse_args()
    run_spin_contract(options.output, options.domain_id)
