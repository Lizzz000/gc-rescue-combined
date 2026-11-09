#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""真实 clangd 语法检查及 JSON-RPC 悬浮、定义跳转、补全合同。

只启动本工作区的 tools/clangd/clangd.sh；补全仅改 LSP 文档覆盖层，不写源码。
本模块不调用项目构建或测试命令，也不证明板端业务或硬件通过。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import threading
import time
import traceback
from urllib.parse import unquote, urlparse


# ---- 证据与协议基础：文件身份、UTF-16 位置及自有进程组回收 ----

# 生产入口已并入节点实现，clangd 合同直接检查真实入口文件。
NONVISION_ENTRYPOINTS = (
    'src/track_pkg/node/track_node.cpp',
    'src/mission_pkg/node/mission_node.cpp',
    'src/bridge_pkg/node/bridge_node.cpp',
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def unchanged(snapshot):
    try:
        return bool(snapshot) and all(sha256(Path(path)) == digest for path, digest in snapshot.items())
    except OSError:
        return False


def position(source, offset):
    # LSP 行号从 0 开始，character 按 UTF-16 code units，而非 UTF-8 字节或 Python 字符计数。
    prefix = source[:offset]
    return {'line': prefix.count('\n'), 'character': len(prefix.rsplit('\n', 1)[-1].encode('utf-16-le')) // 2}


def contains(span, point):
    coordinate = lambda item: (item['line'], item['character'])
    return coordinate(span['start']) <= coordinate(point) <= coordinate(span['end'])


def group_exists(identifier):
    try:
        os.killpg(identifier, 0)
        return True
    except ProcessLookupError:
        return False


def reap_owned(process, timeout=10):
    forced = []
    try:
        code = process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        forced.append('SIGTERM')
        os.killpg(process.pid, signal.SIGTERM)
        try:
            code = process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            forced.append('SIGKILL')
            os.killpg(process.pid, signal.SIGKILL)
            code = process.wait(timeout=5)
    if group_exists(process.pid):
        forced.append('surviving-own-group-SIGKILL')
        os.killpg(process.pid, signal.SIGKILL)
    return {'pid': process.pid, 'own_process_group': process.pid, 'exit_code': code,
            'forced_cleanup': forced, 'process_group_empty': not group_exists(process.pid)}


# ---- JSON-RPC 会话：后台读取、请求/通知配对与限时关闭 ----

class RpcClient:
    def __init__(self, command, workspace, directory, timeout):
        self.timeout = timeout
        self.events = queue.Queue()
        self.responses = {}
        self.diagnostics = []
        self.next_id = 0
        self.stopping = False
        self.raw_lock = threading.Lock()
        self.raw = (directory / 'rpc.jsonl').open('w', encoding='utf-8')
        self.client_wire = (directory / 'rpc-client.bin').open('wb')
        self.server_wire = (directory / 'rpc-server.bin').open('wb')
        self.stderr = (directory / 'clangd.stderr.log').open('wb')
        self.process = None
        self.threads = []
        self.exit_result = None
        try:
            self.process = subprocess.Popen(command, cwd=workspace, stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                            start_new_session=True)
            self.threads = [threading.Thread(target=self.read_stdout, name='clangd-rpc-reader'),
                            threading.Thread(target=self.read_stderr, name='clangd-stderr-reader')]
            for worker in self.threads:
                worker.start()
        except BaseException:
            if self.process is not None:
                self.exit_result = reap_owned(self.process, timeout=0.1)
            for worker in self.threads:
                if worker.ident is not None:
                    worker.join(timeout=5)
            self.raw.close()
            self.client_wire.close()
            self.server_wire.close()
            self.stderr.close()
            raise

    def log(self, direction, message):
        with self.raw_lock:
            self.raw.write(json.dumps({'time_monotonic': time.monotonic(), 'direction': direction,
                                     'message': message}, ensure_ascii=False) + '\n')
            self.raw.flush()

    def read_stdout(self):
        try:
            while True:
                headers = {}
                while True:
                    line = self.process.stdout.readline(65537)
                    self.server_wire.write(line)
                    if not line:
                        self.events.put(('eof', None))
                        return
                    if len(line) > 65536:
                        raise RuntimeError('Oversized JSON-RPC header')
                    if line in (b'\r\n', b'\n'):
                        break
                    name, value = line.decode('ascii').split(':', 1)
                    headers[name.lower()] = value.strip()
                length = int(headers['content-length'])
                require(0 < length <= 64 * 1024 * 1024, 'Invalid JSON-RPC Content-Length')
                body = bytearray()
                while len(body) < length:
                    block = self.process.stdout.read(length - len(body))
                    require(bool(block), 'EOF inside a JSON-RPC body')
                    body.extend(block)
                    self.server_wire.write(block)
                self.server_wire.flush()
                message = json.loads(body.decode('utf-8'))
                self.log('server', message)
                self.events.put(('message', message))
        except Exception:
            self.events.put(('background_error', traceback.format_exc()))

    def read_stderr(self):
        try:
            while True:
                data = self.process.stderr.read1(65536)
                if not data:
                    return
                self.stderr.write(data)
                self.stderr.flush()
        except Exception:
            self.events.put(('background_error', traceback.format_exc()))

    def send(self, message):
        self.log('client', message)
        body = json.dumps(message, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
        wire = f'Content-Length: {len(body)}\r\n\r\n'.encode('ascii') + body
        self.client_wire.write(wire)
        self.client_wire.flush()
        self.process.stdin.write(wire)
        self.process.stdin.flush()

    def notify(self, method, params=None):
        message = {'jsonrpc': '2.0', 'method': method}
        if params is not None:
            message['params'] = params
        self.send(message)

    def pump(self, deadline):
        remaining = deadline - time.monotonic()
        require(remaining > 0, 'Timed out waiting for clangd JSON-RPC')
        try:
            event, payload = self.events.get(timeout=min(0.2, remaining))
        except queue.Empty:
            require(self.process.poll() is None or self.stopping, 'clangd exited before the protocol completed')
            return
        if event == 'background_error':
            raise RuntimeError('clangd background reader failed:\n' + payload)
        if event == 'eof':
            require(self.stopping, 'Unexpected EOF from clangd')
            return
        require(isinstance(payload, dict) and payload.get('jsonrpc') == '2.0', 'Invalid JSON-RPC message')
        if 'method' in payload:
            if 'id' in payload:
                method = payload['method']
                if method == 'workspace/configuration':
                    result = [{} for _ in payload.get('params', {}).get('items', [])]
                elif method == 'workspace/applyEdit':
                    result = {'applied': False, 'failureReason': 'Read-only validation never applies workspace edits'}
                else:
                    result = None
                self.send({'jsonrpc': '2.0', 'id': payload['id'], 'result': result})
            elif payload['method'] == 'textDocument/publishDiagnostics':
                self.diagnostics.append(payload['params'])
        elif 'id' in payload:
            self.responses[payload['id']] = payload

    def request(self, method, params=None, timeout=None):
        self.next_id += 1
        identifier = self.next_id
        self.send({'jsonrpc': '2.0', 'id': identifier, 'method': method, 'params': params})
        deadline = time.monotonic() + (timeout or self.timeout)
        while identifier not in self.responses:
            self.pump(deadline)
        response = self.responses.pop(identifier)
        require('error' not in response, f'{method} failed: {response.get("error")}')
        return response.get('result')

    def wait_diagnostics(self, uri, version, after=0):
        # 必须收到 URI 和版本匹配的真实通知；只将 severity=1 判为失败，警告仍原样留证。
        deadline = time.monotonic() + self.timeout
        while True:
            for message in self.diagnostics[after:]:
                if message.get('uri') == uri and message.get('version') == version:
                    errors = [item for item in message.get('diagnostics', []) if item.get('severity') == 1]
                    require(not errors, f'Real-file diagnostics for {uri}: {errors}')
                    return message
            self.pump(deadline)

    def close(self):
        cleanup_error = None
        if self.process.poll() is None:
            try:
                self.request('shutdown', timeout=10)
                self.stopping = True
                self.notify('exit')
            except Exception as error:
                cleanup_error = str(error)
            finally:
                self.stopping = True
                self.process.stdin.close()
        self.exit_result = reap_owned(self.process)
        alive = []
        for worker in self.threads:
            worker.join(timeout=5)
            if worker.is_alive():
                alive.append(worker.name)
        background = []
        while not self.events.empty():
            kind, payload = self.events.get_nowait()
            if kind == 'background_error':
                background.append(payload)
        self.raw.close()
        self.client_wire.close()
        self.server_wire.close()
        self.stderr.close()
        require(not alive, 'clangd background readers did not finish: ' + ', '.join(alive))
        require(not background, 'Background errors after shutdown: ' + '\n'.join(background))
        require(cleanup_error is None, 'LSP shutdown failed: ' + str(cleanup_error))
        require(self.exit_result['exit_code'] == 0 and not self.exit_result['forced_cleanup'] and
                self.exit_result['process_group_empty'], f'clangd did not exit cleanly: {self.exit_result}')


# ---- 实际检查：编译数据库下的语法诊断与编辑器协议行为 ----

def syntax_check(command, source, workspace, output, timeout):
    stdout = output.with_suffix('.stdout.log')
    stderr = output.with_suffix('.stderr.log')
    with stdout.open('wb') as out, stderr.open('wb') as err:
        process = subprocess.Popen(command + ['--check=' + str(source)], cwd=workspace,
                                   stdout=out, stderr=err, start_new_session=True)
        try:
            result = reap_owned(process, timeout)
        except BaseException:
            result = reap_owned(process, timeout=0.1)
            save(output.with_suffix('.result.json'), dict(result, source=str(source), interrupted=True))
            raise
    text = stderr.read_text(encoding='utf-8', errors='replace') + stdout.read_text(encoding='utf-8', errors='replace')
    summary = re.search(r'All checks completed,\s+(\d+) errors?', text)
    result = dict(result, source=str(source), command=command + ['--check=' + str(source)],
                  errors=int(summary.group(1)) if summary else None)
    save(output.with_suffix('.result.json'), result)
    require(result['exit_code'] == 0 and not result['forced_cleanup'] and result['process_group_empty'],
            f'clangd --check failed for {source}: {result}')
    require(summary is not None and int(summary.group(1)) == 0, f'No zero-error --check summary for {source}')
    require('generic fallback command' not in text.lower(), f'clangd used a fallback command for {source}')
    return result


def definition_locations(response):
    if response is None:
        return []
    values = response if isinstance(response, list) else [response]
    return [(item.get('targetUri', item.get('uri')),
             item.get('targetSelectionRange', item.get('range'))) for item in values]


def file_from_uri(uri):
    parsed = urlparse(uri)
    require(parsed.scheme == 'file' and parsed.netloc in ('', 'localhost'), 'Definition is not a local source URI')
    return Path(unquote(parsed.path)).resolve(strict=True)


def protocol(command, workspace, database, output, timeout, records):
    source_path = workspace / 'src/mission_pkg/mission/mission.cpp'
    node_path = workspace / 'src/mission_pkg/node/mission_node.cpp'
    source = source_path.read_text(encoding='utf-8')
    node = node_path.read_text(encoding='utf-8')
    expected_offset = source.index('checkTimeouts', source.index('MissionController::checkTimeouts'))
    expected_line = position(source, expected_offset)['line']
    call_start = node.index('controller_.checkTimeouts')
    method_offset = call_start + len('controller_.')
    call_position = position(node, method_offset + 2)
    client = RpcClient(command, workspace, output, timeout)
    try:
        initialized = client.request('initialize', {
            'processId': os.getpid(), 'rootUri': workspace.as_uri(),
            'workspaceFolders': [{'uri': workspace.as_uri(), 'name': workspace.name}],
            'capabilities': {'general': {'positionEncodings': ['utf-16']}, 'offsetEncoding': ['utf-16'],
                             'textDocument': {'completion': {'completionItem': {'snippetSupport': False}},
                                              'definition': {'linkSupport': True},
                                              'hover': {'contentFormat': ['plaintext', 'markdown']}}},
            'initializationOptions': {'compilationDatabasePath': str(database), 'clangdFileStatus': True}})
        # 采用标准 positionEncoding；缺失时采用 LSP 默认 UTF-16。
        selected = initialized.get('capabilities', {}).get('positionEncoding', 'utf-16')
        require(selected == 'utf-16', f'Unsupported negotiated offset encoding: {selected}')
        records['server_info'] = initialized.get('serverInfo')
        records['position_encoding'] = selected
        client.notify('initialized', {})
        for path, contents in ((source_path, source), (node_path, node)):
            client.notify('textDocument/didOpen', {'textDocument': {'uri': path.as_uri(), 'languageId': 'cpp',
                          'version': 1, 'text': contents}})
            client.wait_diagnostics(path.as_uri(), 1)
        location = {'textDocument': {'uri': node_path.as_uri()}, 'position': call_position}
        hover = client.request('textDocument/hover', location)
        require(isinstance(hover, dict), 'Real member hover returned no information')
        description = json.dumps(hover.get('contents'), ensure_ascii=False)
        require('checkTimeouts' in description and 'MissionActions' in description, 'Hover did not describe the actual method')
        require('range' in hover and contains(hover['range'], call_position), 'Hover range does not cover the real call site')
        definitions = client.request('textDocument/definition', location)
        matched = []
        for uri, span in definition_locations(definitions):
            if file_from_uri(uri) == source_path and span and span['start']['line'] <= expected_line <= span['end']['line']:
                matched.append({'uri': uri, 'range': span})
        require(bool(matched), 'Definition did not resolve to real MissionController::checkTimeouts implementation')
        records['hover'] = {'source': str(node_path), 'position': call_position, 'response': hover}
        records['definition'] = {'expected_source': str(source_path), 'expected_line': expected_line,
                                 'response': definitions, 'matched': matched}
        # 只移除调用处的方法名；光标位于真实成员 controller_. 的点号后，不改磁盘文件。
        overlay = node[:method_offset] + node[method_offset + len('checkTimeouts'):]
        completion_position = position(overlay, method_offset)
        client.notify('textDocument/didChange', {'textDocument': {'uri': node_path.as_uri(), 'version': 2},
                      'contentChanges': [{'text': overlay}], 'wantDiagnostics': False})
        completed = client.request('textDocument/completion', {
            'textDocument': {'uri': node_path.as_uri()}, 'position': completion_position,
            'context': {'triggerKind': 2, 'triggerCharacter': '.'}})
        items = completed if isinstance(completed, list) else (completed or {}).get('items', [])
        matches = [item for item in items if any(re.search(r'\bcheckTimeouts\b', str(item.get(key, '')))
                   for key in ('label', 'filterText', 'insertText'))]
        require(bool(matches), 'Real controller_. completion omitted checkTimeouts')
        records['completion'] = {'position': completion_position, 'item_count': len(items), 'matches': matches,
                                 'overlay_only': True}
        marker = len(client.diagnostics)
        client.notify('textDocument/didChange', {'textDocument': {'uri': node_path.as_uri(), 'version': 3},
                      'contentChanges': [{'text': node}], 'wantDiagnostics': True})
        # LLVM20会复用与v1相同的AST并省略重复诊断；重新打开后必须收到明确v4的新诊断，
        # didClose产生的无version清空通知不能满足验收，也不把等待超时视为通过。
        client.notify('textDocument/didClose', {'textDocument': {'uri': node_path.as_uri()}})
        client.notify('textDocument/didOpen', {'textDocument': {'uri': node_path.as_uri(), 'languageId': 'cpp',
                      'version': 4, 'text': node}})
        restored_diagnostics = client.wait_diagnostics(node_path.as_uri(), 4, marker)
        restored_hover = client.request('textDocument/hover', location)
        restored_description = json.dumps((restored_hover or {}).get('contents'), ensure_ascii=False)
        require(isinstance(restored_hover, dict) and 'checkTimeouts' in restored_description and
                'MissionActions' in restored_description and 'range' in restored_hover and
                contains(restored_hover['range'], call_position), 'Restored real member hover did not match')
        records['reopened_for_fresh_diagnostics'] = {'version': 4, 'original_text_restored': True,
            'diagnostics': restored_diagnostics, 'hover_barrier': restored_hover,
            'reason': 'LLVM20 omits diagnostics when the restored contents reuse a previously reported AST'}
        for path in (node_path, source_path):
            client.notify('textDocument/didClose', {'textDocument': {'uri': path.as_uri()}})
    finally:
        try:
            client.close()
        finally:
            records['lsp_exit'] = client.exit_result
            stderr = (output / 'clangd.stderr.log').read_text(encoding='utf-8', errors='replace')
            require('generic fallback command' not in stderr.lower(), 'LSP session used a generic fallback compile command')


# ---- CLI：固定工作区与数据库，执行检查并复核磁盘文件未变 ----

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--database', type=Path, required=True, help='This run compile_commands.json or its directory')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--request-timeout', type=float, default=90)
    parser.add_argument('--check-timeout', type=float, default=120)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    database_file = args.database.resolve()
    if database_file.is_dir():
        database_file = database_file / 'compile_commands.json'
    output = args.output.resolve()
    require(not output.exists(), 'Output directory already exists; preserve previous evidence')
    output.mkdir(parents=True)
    result = {'schema': 'rescue.clangd-contract.v1', 'status': 'failed', 'checks': [], 'lsp': {},
              'limitations': ['Only explicitly opened sources are tested; not a full repository index guarantee.',
                              'No automatic refactoring or code-action edits are executed.']}
    watched = [database_file, workspace / 'tools/clangd/clangd.sh',
               workspace / 'src/mission_pkg/node/mission_node.cpp',
               workspace / 'src/interface_pkg/app/core.hpp']
    checks = [workspace / 'src/interface_pkg/app/core.hpp']
    checks += [workspace / 'src/vision_pkg/node/detect_node.cpp']
    checks += [workspace / path for path in NONVISION_ENTRYPOINTS]
    watched += checks
    original = {}
    def interrupted(number, _):
        raise InterruptedError(f'clangd validation received signal {number}')
    old_handlers = {number: signal.signal(number, interrupted) for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        require(workspace.is_dir(), 'Workspace directory does not exist')
        require(database_file.name == 'compile_commands.json' and database_file.is_file(), 'Expected actual compile_commands.json')
        original = {str(path): sha256(path) for path in dict.fromkeys(watched)}
        result['input_sha256'] = original
        rows = json.loads(database_file.read_text(encoding='utf-8'))
        actual_files = {(Path(row['directory']) / row['file']).resolve() for row in rows}
        for source in checks + watched[2:4]:
            require(source in actual_files, 'No exact database entry for actual source/header: ' + str(source))
        command = ['bash', str(workspace / 'tools/clangd/clangd.sh'), '--compile-commands-dir=' + str(database_file.parent),
                   '--enable-config=false', '--background-index=false', '--clang-tidy=false', '--log=verbose']
        result['command'] = command
        result['database_entries'] = len(rows)
        for index, source in enumerate(checks):
            result['checks'].append(syntax_check(command, source, workspace, output / f'check-{index}', args.check_timeout))
        protocol(command, workspace, database_file.parent, output, args.request_timeout, result['lsp'])
        require(unchanged(original), 'Source/database changed during clangd validation')
        result['source_files_unchanged'] = True
        result['status'] = 'passed'
        return_code = 0
    except BaseException as error:
        result['error'] = str(error)
        result['traceback'] = traceback.format_exc()
        result['source_files_unchanged'] = unchanged(original)
        return_code = 1
    finally:
        for number, handler in old_handlers.items():
            signal.signal(number, handler)
    save(output / 'result.json', result)
    print(json.dumps({'status': result['status'], 'report': str(output / 'result.json')}, ensure_ascii=False))
    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
