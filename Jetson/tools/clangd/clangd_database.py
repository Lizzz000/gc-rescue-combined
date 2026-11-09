#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""从本机 CMake/colcon 构建导出 clangd 数据库；不执行编译器、不改原数据库。"""
import argparse
import collections
import hashlib
import json
import os
from pathlib import Path, PureWindowsPath
import re
import shlex
import shutil
import subprocess
import sys

CPP = {'.cpp', '.cc', '.cxx'}
HEADERS = {'.h', '.hpp', '.hh', '.hxx'}


def inside(path, parent):
    return path == parent or parent in path.parents


def preference(row):
    # 同一源文件先选非测试目标；头文件消费者再优先业务实现，其次 app 入口。
    # 前两项相同时优先未显式禁用 TensorRT 的命令；不凭历史数据库条数判断完整性。
    text = ' '.join(row['arguments'])
    output = str(object_file(row) or '')
    normalized = row['file'].replace('\\', '/')
    # 合并后的同一节点源同时用于生产入口和无入口测试库，优先真实生产编译命令。
    library_only = bool(re.search(r'(?:^|\s)(?:-D|/D)[A-Z_]+_LIBRARY_ONLY(?:[=\s]|$)', text))
    return ('/test/' in normalized or '/checks/' in normalized or 'test_' in output,
            library_only, '/app/' in normalized,
            bool(re.search(r'VISION_ENABLE_TENSORRT=0(?:\s|$)', text)),
            row['file'], output, tuple(row['arguments']), row['directory'], row['database'])



def object_file(row):
    args = row.get('original_arguments', row['arguments'])
    output = row.get('output')
    if not output and '-o' in args:
        output = args[args.index('-o') + 1]
    if not output:
        output = next((value[2:] for value in args[1:] if value.startswith('-o') and len(value) > 2), None)
    return (Path(row['directory']) / native_path(output)).resolve() if output else None


def native_path(value):
    """只解释当前宿主的路径，不能把另一个系统的绝对路径变成本地相对路径。"""
    path = Path(value)
    if os.name != 'nt' and PureWindowsPath(str(value)).drive:
        raise ValueError('expected a native host path: ' + str(value))
    if os.name == 'nt' and path.root and not path.drive:
        raise ValueError('expected a native host path with a drive: ' + str(value))
    if os.name == 'nt' and path.drive and not path.is_absolute():
        raise ValueError('drive-relative paths are ambiguous: ' + str(value))
    return path


def split_command(value):
    if not isinstance(value, str):
        raise ValueError('compiler command must be a string')
    if not value.strip() or '\0' in value:
        raise ValueError('compiler command must be nonempty and contain no NUL')
    if os.name != 'nt':
        return shlex.split(value)
    # Windows 的反斜杠是路径分隔符；仅在双引号前才参与 CRT 参数转义。
    import ctypes
    from ctypes import wintypes
    shell = ctypes.WinDLL('shell32', use_last_error=True)
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    shell.CommandLineToArgvW.argtypes = [wintypes.LPCWSTR, ctypes.POINTER(ctypes.c_int)]
    shell.CommandLineToArgvW.restype = ctypes.POINTER(wintypes.LPWSTR)
    kernel.LocalFree.argtypes = [wintypes.HLOCAL]
    kernel.LocalFree.restype = wintypes.HLOCAL
    count = ctypes.c_int()
    arguments = shell.CommandLineToArgvW(value.lstrip(), ctypes.byref(count))
    if not arguments:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        return [arguments[index] for index in range(count.value)]
    finally:
        kernel.LocalFree(ctypes.cast(arguments, wintypes.HLOCAL))


def append_flag(args, *flags):
    index = args.index('--') if '--' in args else len(args)
    args[index:index] = flags


def cuda_arguments(args):
    """把 NVCC 的实际语言/架构参数转换为 clang 语法，不降低设备架构。"""
    compiler = Path(shutil.which(args[0]) or args[0]).resolve()
    toolkit = compiler.parent.parent
    if not (toolkit / 'include/cuda_runtime.h').is_file():
        raise ValueError('CUDA toolkit cannot be resolved from nvcc: ' + str(compiler))
    result, architectures, index = ['clang++'], set(), 1
    while index < len(args):
        value = args[index]
        if value in ('-forward-unknown-to-host-compiler', '--forward-unknown-to-host-compiler'):
            index += 1
            continue
        if value in ('--generate-code', '-gencode', '--gpu-architecture', '-arch'):
            if index + 1 >= len(args):
                raise ValueError('missing CUDA architecture value')
            value += '=' + args[index + 1]
            index += 1
        if value.startswith(('--generate-code=', '-gencode=', '--gpu-architecture=', '-arch=')):
            architectures.update(re.findall(r'sm_([0-9]+)', value))
            if not architectures:
                architectures.update(re.findall(r'compute_([0-9]+)', value))
        elif value in ('-Xcompiler', '--compiler-options') or value.startswith(('-Xcompiler=', '--compiler-options=')):
            if '=' in value:
                host = value.split('=', 1)[1]
            else:
                index += 1
                if index >= len(args):
                    raise ValueError('missing NVCC host compiler options')
                host = args[index]
            result.extend(split_command(host.replace(',', ' ')))
        elif value.startswith('-isystem='):
            result.extend(['-isystem', value.split('=', 1)[1]])
        elif value == '-x':
            index += 1
            if index >= len(args):
                raise ValueError('missing CUDA language')
            result.extend(['-x', 'cuda' if args[index] == 'cu' else args[index]])
        elif value in ('-xcu', '-x=cu'):
            result.extend(['-x', 'cuda'])
        else:
            result.append(value)
        index += 1
    if len(architectures) != 1:
        raise ValueError('clangd CUDA requires one explicit actual architecture; got ' + repr(sorted(architectures)))
    append_flag(result, '--cuda-path=' + str(toolkit), '--cuda-gpu-arch=sm_' + architectures.pop())
    return result


def input_record(path, data):
    return {'path': str(path), 'sha256': hashlib.sha256(data).hexdigest()}


def load_rows(workspace, builds):
    grouped, rejected, inputs = collections.defaultdict(list), [], []
    databases = {}
    for build in builds:
        # 仅直接 CMake 数据库及一层 colcon 包；不递归搜索历史构建目录。
        found = ([build / 'compile_commands.json'] if (build / 'compile_commands.json').is_file() else [])
        found += sorted(build.glob('*/compile_commands.json'))
        if not found:
            raise ValueError('no input compile_commands.json in selected build: ' + str(build))
        for path in found:
            database = path.resolve(strict=True)
            if not inside(database, build):
                raise ValueError('input database is outside selected build: ' + str(database))
            databases.setdefault(database, build)
    for database, build in sorted(databases.items()):
        data = database.read_bytes()
        records = json.loads(data)
        if not isinstance(records, list):
            raise ValueError('database must be a JSON array: ' + str(database))
        inputs.append(input_record(database, data))
        for raw in records:
            try:
                directory = native_path(raw['directory'])
                if not directory.is_absolute():
                    raise ValueError('working directory must be an absolute native host path')
                directory = directory.resolve(strict=True)
                source = (directory / native_path(raw['file'])).resolve(strict=True)
                if not directory.is_dir() or not inside(directory, build):
                    raise ValueError('working directory is outside selected build')
                if not source.is_file() or not any(inside(source, parent) for parent in
                    (workspace / 'src', build)):
                    raise ValueError('source is outside owned source or selected build')
                args = raw['arguments'] if 'arguments' in raw else split_command(raw['command'])
                if (not isinstance(args, list) or not args or not args[0] or
                    not all(isinstance(value, str) and '\0' not in value for value in args)):
                    raise ValueError('compiler arguments must be a nonempty string array')
                args = list(args)
                row = {'directory': str(directory), 'file': str(source), 'arguments': args,
                       'database': str(database), 'build_base': str(build)}
                if raw.get('output'):
                    row['output'] = raw['output']
                # 原命令用于依赖文件定位与替代目标审计；只改 clangd 的副本。
                row['original_arguments'] = list(args)
                if source.suffix == '.cu':
                    row['arguments'] = cuda_arguments(args)
                elif source.suffix in CPP and not any(value.startswith(('-std=', '--std='))
                    or value in ('-std', '--std') for value in args):
                    append_flag(row['arguments'], '-std=c++17')
                object_file(row)  # 有 -o 时须有参数，不能把坏命令留给目标选择或依赖关联。
                dependency_file(row)
                grouped[str(source)].append(row)
            except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
                rejected.append({'database': str(database), 'file': raw.get('file') if isinstance(raw, dict) else None,
                                 'reason': str(error)})
    return grouped, rejected, inputs


def dependency_file(row):
    args = row['original_arguments']
    dependency = None
    for index, value in enumerate(args):
        if value == '-MF':
            dependency = args[index + 1]
        elif value.startswith('-MF'):
            dependency = value[3:].removeprefix('=')
        if value.startswith('-MF') and not dependency:
            raise ValueError('missing dependency file after -MF')
    if dependency is not None:
        return (Path(row['directory']) / native_path(dependency)).resolve()
    obj = object_file(row)
    return Path(str(obj) + '.d').resolve() if obj else None


def make_dependencies(text):
    """读取编译器 Make depfile 的规则 RHS，保留盘符和普通 Windows 反斜杠。"""
    text = re.sub(r'\\\r?\n', ' ', text)
    dependencies = []
    for line in text.splitlines():
        words, word, rhs, index = [], [], False, 0
        while index < len(line):
            value = line[index]
            following = line[index + 1] if index + 1 < len(line) else ''
            if value == '\\' and following and following in ' \t\\:#':
                word.append(following)
                index += 2
                continue
            if value == '$' and following == '$':
                word.append('$')
                index += 2
                continue
            if value == '#':
                break
            drive = len(word) == 1 and word[0].isalpha() and following in ('/', '\\')
            if value == ':' and not rhs and not drive:
                word, words, rhs = [], [], True
            elif value.isspace():
                if word:
                    words.append(''.join(word))
                    word = []
            else:
                word.append(value)
            index += 1
        if word:
            words.append(''.join(word))
        if rhs:
            dependencies.extend(words)
        elif words:
            raise ValueError('dependency file contains a line without a Make rule')
    return dependencies


def dependency_paths(row):
    """优先读取保留的 depfile；Ninja 收走它时只查询真实依赖库，不重新编译。"""
    dependency = dependency_file(row)
    if dependency is not None and dependency.is_file():
        data = dependency.read_bytes()
        return make_dependencies(data.decode('utf-8')), [input_record(dependency, data)]
    directory = Path(row['directory'])
    files = [directory / name for name in ('build.ninja', 'CMakeCache.txt', '.ninja_deps')]
    obj = object_file(row)
    if obj is None or not inside(obj, directory) or not all(path.is_file() for path in files):
        return None, []
    data = [path.read_bytes() for path in files]
    executable = next((line.partition('=')[2] for line in data[1].decode('utf-8').splitlines()
                       if line.startswith('CMAKE_MAKE_PROGRAM:FILEPATH=')), None)
    if not executable:
        return None, []
    result = subprocess.run([executable, '-C', str(directory), '-t', 'deps',
                             obj.relative_to(directory).as_posix()],
                            check=True, capture_output=True, text=True, encoding='utf-8', timeout=30)
    lines = result.stdout.splitlines()
    inputs = [input_record(path, body) for path, body in zip(files, data)]
    if not lines or not lines[0].endswith('(VALID)'):
        return None, inputs
    return [line[4:] for line in lines[1:] if line.startswith('    ')], inputs


def source_header(path, workspace, build):
    resolved = path.resolve()
    if resolved.suffix not in HEADERS or not resolved.is_file():
        return None
    if inside(resolved, workspace / 'src') or inside(resolved, build):
        return resolved
    # install 的公共 app 头可能是副本；仅字节一致时关联到本轮真实源头。
    if inside(resolved, workspace / 'install') and 'app' in resolved.parts:
        index = resolved.parts.index('app')
        candidate = workspace / 'src/interface_pkg/app' / Path(*resolved.parts[index + 1:])
        if candidate.is_file() and candidate.read_bytes() == resolved.read_bytes():
            return candidate.resolve()
    if inside(resolved, workspace / 'install'):
        for kind in ('msg', 'srv'):
            if kind in resolved.parts:
                candidate = build / 'interface_pkg/rosidl_generator_cpp/interface_pkg' / Path(
                    *resolved.parts[resolved.parts.index(kind):])
                if candidate.is_file() and candidate.read_bytes() == resolved.read_bytes():
                    return candidate.resolve()
    return None


def header_rows(candidates, workspace):
    # 只从非测试 C++ 目标的真实依赖建立关联，不猜同名 .cpp，也不借用 CUDA 命令。
    consumers, cache, missing, inputs = collections.defaultdict(list), {}, 0, {}
    for row in candidates:
        if Path(row['file']).suffix not in CPP or preference(row)[0]:
            continue
        build = Path(row['build_base'])
        paths, records = dependency_paths(row)
        for item in records:
            inputs[Path(item['path'])] = item
        if paths is None:
            missing += 1
            continue
        for value in paths:
            path = (Path(row['directory']) / native_path(value)).resolve()
            if not any(inside(path, parent) for parent in (workspace / 'src', workspace / 'install', build)):
                continue
            key = (str(path), str(build))
            if key not in cache:
                cache[key] = source_header(path, workspace, build)
            header = cache[key]
            if header is not None:
                consumers[str(header)].append(row)
    headers = {}
    for header, options in consumers.items():
        consumer = min(options, key=preference)
        args, replaced = list(consumer['arguments']), False
        for index, value in enumerate(args[1:], 1):
            if not value.startswith('-') and (Path(consumer['directory']) / value).resolve() == Path(consumer['file']):
                args[index], replaced = header, True
        if not replaced:
            raise ValueError('consumer command lacks its source argument: ' + consumer['file'])
        headers[header] = {**consumer, 'file': header, 'arguments': args,
                           'clangd_header_from': consumer['file']}
    return headers, missing, [item for _, item in sorted(inputs.items())]


def command(row):
    return {key: row[key] for key in ('directory', 'file', 'arguments', 'output') if key in row}


def create_database(workspace, build, output):
    workspace = native_path(workspace).resolve(strict=True)
    raw_builds = [build] if isinstance(build, (str, os.PathLike)) else list(build)
    if not raw_builds:
        raise ValueError('at least one build base is required')
    builds = sorted({(workspace / native_path(value)).resolve(strict=True) for value in raw_builds})
    output = (workspace / native_path(output)).resolve()
    if not workspace.is_dir() or any(not value.is_dir() or not inside(value, workspace) for value in builds):
        raise ValueError('build must be a directory inside workspace')
    if not inside(output, workspace / 'build'):
        raise ValueError('output must be inside workspace/build')
    grouped, rejected, inputs = load_rows(workspace, builds)
    if not grouped:
        raise ValueError('no valid commands; existing clangd database preserved')
    if output in builds:
        raise ValueError('clangd output must not overwrite an input build database')
    selected, choices, candidates = {}, [], []
    for file, rows in sorted(grouped.items()):
        unique = {}
        for row in sorted(rows, key=preference):
            # 相同相对参数在不同工作目录中代表不同构建；不能按 argv 单独去重。
            key = (row['directory'], tuple(row['original_arguments']), str(object_file(row) or ''))
            unique.setdefault(key, row)
        ordered = list(unique.values())
        candidates.extend(ordered)
        selected[file] = ordered[0]
        choices.append({'file': file, 'selected_from': command(ordered[0]),
                        'original_arguments': ordered[0]['original_arguments'],
                        'selected_database': ordered[0]['database'],
                        'alternatives': [command(row) for row in ordered[1:]]})
    headers, missing, dependency_inputs = header_rows(candidates, workspace)
    for file, row in sorted(headers.items()):
        if file not in selected:
            selected[file] = row
            choices.append({'file': file, 'header_from': row['clangd_header_from'], 'selected_from': command(row),
                            'selected_database': row['database'],
                            'alternatives': []})
    build_bases = [str(value) for value in builds]
    manifest = {'format_version': 1, 'build_base': build_bases[0], 'build_bases': build_bases,
                'inputs': inputs, 'dependency_inputs': dependency_inputs, 'selected': choices,
                'rejected': rejected, 'missing_dependency_files': missing}
    # 普通输出、临时输出及已有硬链接/符号链接都不得覆盖任何读入的数据库或 depfile。
    targets = [output / name for name in ('compile_commands.json', 'selection.json',
                                         'compile_commands.json.tmp', 'selection.json.tmp')]
    for item in inputs + dependency_inputs:
        origin = Path(item['path'])
        if any(target.resolve() == origin or (target.exists() and target.samefile(origin)) for target in targets):
            raise ValueError('clangd output must not overwrite an input: ' + str(origin))
    output.mkdir(parents=True, exist_ok=True)
    for name, value in [('compile_commands.json', [command(row) for _, row in sorted(selected.items())]),
                        ('selection.json', manifest)]:
        temporary = output / (name + '.tmp')
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(output / name)
    return {'build_base': build_bases[0], 'build_bases': build_bases,
            'output': str(output), 'sources': len(grouped), 'headers': len(headers),
            'rejected': len(rejected), 'missing_dependency_files': missing}



def configure_vscode(workspace, output):
    """显式生成本机工作空间的 clangd 设置，保留其他用户字段。"""
    path = workspace / '.vscode/settings.json'
    if not inside(path.resolve(), workspace):
        raise ValueError('VS Code settings must stay inside this workspace')
    settings = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
    arguments = settings.setdefault('clangd.arguments', ['--background-index', '-j=2'])
    if os.name == 'nt':
        executable = shutil.which('clangd')
        if not executable:
            raise ValueError('add the installed clangd.exe directory to PATH before configuring VS Code')
        settings.setdefault('clangd.path', executable)
        if not any(value.startswith('--query-driver') for value in arguments):
            rows = json.loads((output / 'compile_commands.json').read_text(encoding='utf-8'))
            drivers = sorted({Path(row['arguments'][0]).as_posix() for row in rows
                              if Path(row['arguments'][0]).is_file()})
            if not drivers:
                raise ValueError('no actual compiler paths for clangd query-driver')
            arguments.append('--query-driver=' + ','.join(drivers))
    else:
        settings['clangd.path'] = str(workspace / 'tools/clangd/clangd.sh')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(settings, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)
    return str(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--build-base', type=Path, action='append', required=True,
                        help='native build directory; repeat for CMake/colcon builds in this workspace')
    parser.add_argument('--output', type=Path, default=Path('build/unified-clangd'))
    parser.add_argument('--configure-vscode', action='store_true',
                        help='set this workspace clangd.path, preserving other JSON settings')
    args = parser.parse_args()
    try:
        result = create_database(args.workspace, args.build_base, args.output)
        if args.configure_vscode:
            result['vscode_settings'] = configure_vscode(args.workspace.resolve(strict=True), Path(result['output']))
        print(json.dumps(result, ensure_ascii=False))
    except (OSError, ValueError, TypeError, KeyError, subprocess.SubprocessError) as error:
        print('clangd database: ' + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
