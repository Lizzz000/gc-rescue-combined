# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""验收证据和自有进程组；不按全局程序名查杀其他项目。"""
from __future__ import annotations

from collections import deque
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time


# ---- 失败分类与文件证据：区分合同失败、能力不足及资产身份 ----

class ContractFailure(RuntimeError):
    pass


class CapabilityMissing(ContractFailure):
    pass


def require(condition, message):
    if not condition:
        raise ContractFailure(message)


def write_json(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')


def fingerprint(path):
    path = Path(path).resolve()
    stat = path.stat()
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    after = path.stat()
    require((stat.st_size, stat.st_mtime_ns, stat.st_ino) ==
            (after.st_size, after.st_mtime_ns, after.st_ino), 'file changed while hashing: ' + str(path))
    return {'path': str(path), 'bytes': stat.st_size, 'mtime_ns': stat.st_mtime_ns,
            'sha256': digest.hexdigest()}


def asset_snapshot(snapshot):
    model = Path(snapshot['values']['vision.detect.seg_model_path'])
    labels = Path(snapshot['values']['vision.detect.seg_labels_path'])
    # 不存在的伴随缓存也有身份；运行中意外新建缓存同样必须被发现。
    paths = {model, labels, model.parent / 'seg.onnx', model.with_suffix('.timing.cache')}
    for pattern in ('*.onnx', '*.trt', '*.engine', '*.cache', '*.calib'):
        paths.update(model.parent.glob(pattern))
    if 'tensorrt_gpu' in snapshot.get('required_backends', []):
        require(model.is_file() and labels.is_file(), 'configured model/labels are missing')
    return {str(path.resolve()): fingerprint(path) if path.is_file() else {'exists': False}
            for path in sorted(paths)}


# ---- 运行身份与隔离：进程组、端口和 vcan 准入 ----

def proc_info(pid):
    try:
        root = Path('/proc') / str(pid)
        rest = (root / 'stat').read_text().rsplit(')', 1)[1].split()
        argv = [part.decode(errors='replace') for part in (root / 'cmdline').read_bytes().split(b'\0') if part]
        role = argv[argv.index('--role') + 1] if '--role' in argv else None
        return {'pid': int(pid), 'pgid': int(rest[2]), 'start_ticks': int(rest[19]),
                'executable': str((root / 'exe').resolve()), 'argv': argv, 'role': role,
                'state': rest[0]}
    except (OSError, ValueError, IndexError):
        return None


def group_members(pgid):
    members = []
    for path in Path('/proc').iterdir():
        if path.name.isdigit():
            record = proc_info(path.name)
            if record and record['pgid'] == pgid and record['state'] != 'Z':
                members.append(record)
    return members


def port_open(port):
    with socket.socket() as connection:
        connection.settimeout(0.2)
        return connection.connect_ex(('127.0.0.1', port)) == 0


def isolated_vcan(interface='vcan0', parent_namespace=None):
    """绑定/发送前证明命名空间隔离与虚拟接口身份，不操作物理CAN。"""
    require(interface == 'vcan0', 'validation only binds vcan0')
    current = os.readlink('/proc/self/ns/net')
    if parent_namespace is None:
        parent_namespace = os.environ.get('RESCUE_TEST_HOST_NETNS')
    require(parent_namespace and current != parent_namespace,
            'isolated namespace proof missing; use the validation namespace worker')
    result = subprocess.run(['ip', '-details', '-json', 'link', 'show', 'dev', interface],
                            text=True, capture_output=True, check=True)
    links = json.loads(result.stdout)
    require(len(links) == 1 and links[0].get('linkinfo', {}).get('info_kind') == 'vcan' and
            'UP' in links[0].get('flags', []), 'isolated vcan0 is not UP/vcan')
    return {'current': current, 'host': parent_namespace, 'interface': links[0]}


# ---- 自有进程生命周期：启动、身份取样、限时停止与退出证明 ----

class OwnedProcess:
    def __init__(self, command, log, env=None, role_executables=None):
        self.log_path = Path(log)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.log_path.open('w', encoding='utf-8')
        helper = Path(__file__).with_name('exec_clean.py')
        try:
            self.process = subprocess.Popen([sys.executable, str(helper), *map(str, command)],
                                            stdin=subprocess.DEVNULL,
                                            stdout=self.stream, stderr=subprocess.STDOUT,
                                            env=env, start_new_session=True)
        except BaseException:
            self.stream.close()
            raise
        self.role_executables = {key: str(Path(value).resolve())
                                 for key, value in (role_executables or {}).items()}
        self.roles = {}
        self.samples = deque(maxlen=6000)
        self.last_sample_at = 0.0
        self.current_members = []
        self.stopped = False

    def sample(self, maximum_age=0):
        now = time.monotonic()
        if maximum_age > 0 and now - self.last_sample_at < maximum_age:
            return self.current_members
        members = group_members(self.process.pid)
        for member in members:
            if member['role']:
                key = member['role']
                require(key in self.role_executables and
                        member['executable'] == self.role_executables[key],
                        'role executable differs from selected installation: ' + key)
                prior = self.roles.get(key)
                require(not prior or (prior['pid'], prior['start_ticks']) ==
                        (member['pid'], member['start_ticks']), f'role restarted unexpectedly: {key}')
                self.roles[key] = member
        self.last_sample_at = now
        self.current_members = members
        self.samples.append({'steady': now, 'members': members})
        return members

    def stop(self, expected_roles, http_port=None, grace=10):
        if self.stopped:
            return self.shutdown
        # 取样异常先留证、继续回收进程组；即使进程最终退出，clean 也为 False。
        sampling_error = None
        try:
            self.sample()
        except BaseException as error:
            sampling_error = f'{type(error).__name__}: {error}'
        if self.process.poll() is None:
            try:
                # 本工具启动的是非交互launch；由launcher统一通知子进程。
                # 先向全组发SIGINT会与launch的转发重叠，第二次信号可能击中已恢复默认处理器的析构窗口。
                os.kill(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline and (self.process.poll() is None or group_members(self.process.pid)):
            time.sleep(0.05)
        forced = self.process.poll() is None or bool(group_members(self.process.pid))
        if forced:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and (self.process.poll() is None or group_members(self.process.pid)):
                time.sleep(0.05)
            # launcher 已退出时仍可能有子进程；不能只看 launcher.wait() 决定是否升级清理。
            if self.process.poll() is None or group_members(self.process.pid):
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                sampling_error = (sampling_error or '') + '; launcher not reaped after SIGKILL'
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and group_members(self.process.pid):
                time.sleep(0.05)
        else:
            self.process.wait()
        self.stream.flush()
        self.stream.close()
        text = self.log_path.read_text(encoding='utf-8', errors='replace')
        exits = {}
        for line in text.splitlines():
            clean = re.search(r'process has finished cleanly \[pid (\d+)\]', line)
            failed = re.search(r'process has died \[pid (\d+), exit code (-?\d+)', line)
            if clean:
                exits[int(clean.group(1))] = 0
            if failed:
                exits[int(failed.group(1))] = int(failed.group(2))
        role_exits = {role: {'pid': self.roles.get(role, {}).get('pid'),
                            'exit_code': exits.get(self.roles.get(role, {}).get('pid'))}
                      for role in expected_roles}
        remaining = group_members(self.process.pid)
        closed = http_port is None or not port_open(http_port)
        engine_builds = re.findall(r'\[engine\] building [^\n]+ engine from ', text)
        clean = (not engine_builds and not sampling_error and not forced and self.process.returncode == 0 and not remaining and closed and
                 set(self.roles) == set(expected_roles) and
                 all(record['exit_code'] == 0 for record in role_exits.values()))
        self.shutdown = {'forced': forced, 'launcher_exit': self.process.returncode,
                         'role_exits': role_exits, 'remaining': remaining,
                         'http_closed': closed, 'sampling_error': sampling_error,
                         'production_engine_builds': len(engine_builds), 'clean': clean}
        self.stopped = True
        return self.shutdown


# ---- 宿主设备占用见证：隔离 worker 经固定通道查询父宿主 ----

def _witness_namespace():
    return {'pid': os.getpid(), 'uid': os.geteuid(),
            **{key: os.readlink('/proc/self/ns/' + key) for key in ('user', 'net')}}


def _witness_file(device):
    path = Path(device).resolve(strict=True)
    value = path.stat()
    return {'path': str(path), **{key: getattr(value, key) for key in ('st_dev', 'st_ino', 'st_mode', 'st_rdev')}}


def _witness_receive(channel):
    data = bytearray()
    while not data.endswith(b'\n'):
        part = channel.recv(4096)
        require(part, 'host ownership witness closed before a complete record')
        data.extend(part)
        require(len(data) <= 8 * 1024 * 1024, 'ownership witness record exceeds limit')
    return json.loads(data)


class HostFileUseWitness:
    """父宿主仅为固定设备运行 fuser；继承 socketpair 不提供任意命令或路径代理。"""
    def __init__(self, device, log):
        import queue
        import threading
        self.identity, self.host = _witness_file(device), _witness_namespace()
        self.log = Path(log)
        self.parent, self.child = socket.socketpair()
        self.stop, self.errors = threading.Event(), queue.Queue()
        self.records = []
        self.thread = threading.Thread(target=self._serve, name='host-file-use-witness')
        self.thread.start()

    @property
    def worker_fd(self):
        return self.child.fileno()

    def _serve(self):
        try:
            for phase in ('before', 'after'):
                try:
                    request = _witness_receive(self.parent)
                except (OSError, ContractFailure):
                    if self.stop.is_set():
                        return
                    raise
                require(set(request) == {'phase', 'identity', 'namespace'} and request['phase'] == phase and
                        request['identity'] == self.identity, 'ownership witness request differs from fixed device/order')
                worker = request['namespace']
                require(worker['user'] != self.host['user'] and worker['net'] != self.host['net'],
                        'ownership witness worker is not in the isolated namespaces')
                require(_witness_file(self.identity['path']) == self.identity and
                        _witness_namespace() == self.host, 'ownership witness identity changed')
                start = time.monotonic()
                argv = ['fuser', self.identity['path']]
                checked = subprocess.run(argv, text=True, capture_output=True, timeout=5)
                require(_witness_file(self.identity['path']) == self.identity, 'device changed during host fuser')
                result = {'phase': phase, 'identity': self.identity, 'host': self.host, 'worker': worker,
                          'argv': argv, 'exit_code': checked.returncode, 'stdout': checked.stdout,
                          'stderr': checked.stderr, 'elapsed_s': time.monotonic() - start,
                          'free': checked.returncode == 1 and not checked.stdout and not checked.stderr}
                self.records.append(result)
                write_json(self.log, {'host': self.host, 'identity': self.identity, 'records': self.records,
                                      'closed': False})
                self.parent.sendall(json.dumps(result).encode() + b'\n')
        except BaseException as error:
            self.errors.put(error)
            try:
                self.parent.sendall(json.dumps({'error': str(error)}).encode() + b'\n')
            except OSError:
                pass

    def close(self):
        self.stop.set()
        try:
            self.parent.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.parent.close()
        self.child.close()
        self.thread.join(timeout=7)
        errors = []
        while not self.errors.empty():
            errors.append(str(self.errors.get()))
        write_json(self.log, {'host': self.host, 'identity': self.identity, 'records': self.records,
                              'closed': not self.thread.is_alive(), 'errors': errors})
        require(not self.thread.is_alive(), 'own host witness thread did not stop')
        require(not errors, 'host ownership witness failed: ' + '; '.join(errors))


def host_file_use(device, phase):
    """隔离 worker 只向父宿主查询；任何 stderr、错误或身份变化都不能当作空闲。"""
    descriptor = os.environ.get('RESCUE_CAMERA_WITNESS_FD')
    require(descriptor is not None, 'camera requires the isolated parent host ownership witness')
    identity, namespace = _witness_file(device), _witness_namespace()
    # fromfd 复制描述符；关闭一次查询不能关闭后续释放核对所用的继承通道。
    with socket.fromfd(int(descriptor), socket.AF_UNIX, socket.SOCK_STREAM) as channel:
        channel.settimeout(10)
        channel.sendall(json.dumps({'phase': phase, 'identity': identity, 'namespace': namespace}).encode() + b'\n')
        result = _witness_receive(channel)
    require('error' not in result, 'host ownership witness: ' + result.get('error', ''))
    require(result['identity'] == identity == _witness_file(device) and result['phase'] == phase and
            result['worker'] == namespace and result['host']['user'] != namespace['user'],
            'host ownership witness response identity mismatch')
    require(result['free'] == (result['exit_code'] == 1 and not result['stdout'] and not result['stderr']),
            'host ownership witness has inconsistent free result')
    return result


# ---- 见证通道合同：临时普通文件验证 busy/free，不访问实际摄像头 ----

def run_witness_contract(output):
    """仅持有本轮临时普通文件，证明隔离 worker 得到真实宿主的 busy/free 证据。"""
    import tempfile
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'failed', 'cases': []}
    try:
        with tempfile.TemporaryDirectory(prefix='rescue-host-witness-') as folder:
            device = Path(folder) / 'own-file'
            device.write_text('only this test owns the descriptor', encoding='utf-8')
            for busy in (True, False):
                held = device.open('rb') if busy else None
                witness = HostFileUseWitness(device, output / ('busy.json' if busy else 'free.json'))
                try:
                    env = dict(os.environ, RESCUE_CAMERA_WITNESS_FD=str(witness.worker_fd))
                    argv = ['unshare', '--user', '--map-root-user', '--net', sys.executable, __file__,
                            '--witness-worker', '--device', str(device), '--expected', 'busy' if busy else 'free']
                    child = subprocess.run(argv, env=env, pass_fds=(witness.worker_fd,),
                                           text=True, capture_output=True, timeout=20)
                    row = {'expected': 'busy' if busy else 'free', 'argv': argv,
                           'exit_code': child.returncode, 'stdout': child.stdout, 'stderr': child.stderr}
                    report['cases'].append(row)
                    require(child.returncode == 0 and not child.stderr, 'isolated witness contract worker failed')
                finally:
                    if held:
                        held.close()
                    witness.close()
                require(not witness.thread.is_alive() and witness.parent.fileno() == witness.child.fileno() == -1,
                        'host witness resources remain open')
                row['own_thread_and_sockets_closed'] = True
        report['temporary_file_removed'] = not device.exists()
        report['status'] = 'passed'
    except BaseException as error:
        report['reason'] = str(error)
        raise
    finally:
        write_json(output / 'result.json', report)
    return report


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='仅显式运行宿主文件占用见证回归；不打开相机。')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--witness-contract', action='store_true')
    mode.add_argument('--witness-worker', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--device', type=Path)
    parser.add_argument('--expected', choices=('busy', 'free'))
    options = parser.parse_args()
    if options.witness_contract:
        require(options.output is not None, '--output is required')
        print(json.dumps(run_witness_contract(options.output)))
    else:
        require(options.device is not None and options.expected is not None, 'worker needs --device/--expected')
        before = host_file_use(options.device, 'before')
        if options.expected == 'busy':
            require(before['exit_code'] == 0 and str(before['host']['pid']) in before['stdout'].split(),
                    'host fuser did not identify the test-owned open descriptor')
        else:
            require(before['free'] and host_file_use(options.device, 'after')['free'],
                    'released test file was not proven free before/after')
        print(json.dumps(before))
