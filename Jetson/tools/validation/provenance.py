# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""绑定实际生产源码、编译依赖、CMake配置与安装产物；版本标签不作为来源证明。"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time

from .evidence import fingerprint, require, write_json

SCHEMA = 'rescue.build-identity.v2'
IDENTITY = '.rescue-build-identity.json'
PACKAGES = ('interface_pkg', 'vision_pkg', 'track_pkg', 'mission_pkg', 'bridge_pkg', 'bringup_pkg')
EXCLUDED = {'.git', '.vscode', '__pycache__', '.cache', 'test', 'tests', 'checks', 'models',
            'build', 'install', 'log', 'logs', 'CMakeFiles', 'test_results', 'node_modules'}
SOURCE_SUFFIXES = {'.cpp', '.cc', '.cxx', '.c', '.h', '.hpp', '.hxx', '.cu', '.cuh', '.in',
                   '.cmake', '.py', '.sh', '.msg', '.srv', '.idl', '.xml', '.yaml', '.yml',
                   '.json', '.cfg', '.ini', '.toml', '.em'}


# ---- 来源清单：生产源码、验收文件、基线与工具身份分别留证 ----

def production_path(relative):
    parts = Path(relative).parts
    return (len(parts) >= 3 and parts[0] == 'src' and parts[1] in PACKAGES and
            not any(part in EXCLUDED for part in parts[2:-1]) and
            (Path(relative).suffix in SOURCE_SUFFIXES or parts[-1] in ('CMakeLists.txt', 'COLCON_IGNORE')))


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def source_files(source):
    source = Path(source).resolve(strict=True)
    result = {}
    for package in PACKAGES:
        require((source / 'src' / package / 'CMakeLists.txt').is_file(), 'missing production package: ' + package)
    for folder, dirs, files in os.walk(source / 'src', followlinks=False):
        dirs[:] = sorted(name for name in dirs if name not in EXCLUDED)
        for name in dirs:
            require(not (Path(folder) / name).is_symlink(), 'source directory symlink needs explicit provenance: ' + name)
        for name in sorted(files):
            path = Path(folder) / name
            relative = path.relative_to(source).as_posix()
            if not production_path(relative):
                continue
            # 不把链接悄悄解引用成另一棵源码；Git blob 对应链接文本，其目标另存身份。
            if path.is_symlink():
                data = os.readlink(path).encode()
                target = fingerprint(path)
            else:
                before = path.stat()
                data = path.read_bytes()
                after = path.stat()
                require((before.st_size, before.st_mtime_ns, before.st_ino) ==
                        (after.st_size, after.st_mtime_ns, after.st_ino), 'source changed while reading: ' + str(path))
                target = None
            blob = b'blob ' + str(len(data)).encode('ascii') + b'\0' + data
            result[relative] = {'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data),
                                'git_blob_oid': hashlib.sha1(blob).hexdigest()}
            if target:
                result[relative]['symlink_target'] = os.readlink(path)
                result[relative]['target'] = target
    require(result, 'production source inventory is empty')
    return result



def contract_files(source, root):
    """绑定候选的全部测试/工具文件（含MD），与生产清单分开；不表示每项都会编译。"""
    paths = set()
    for package in PACKAGES:
        for name in ('test', 'checks'):
            folder = source / 'src' / package / name
            if folder.is_dir():
                paths.update(path for path in folder.rglob('*') if path.is_file() and
                             not any(part in ('__pycache__', 'CMakeFiles', 'build') for part in path.relative_to(folder).parts))
    for folder in (root / 'checks', root / 'tools'):
        paths.update(path for path in folder.rglob('*') if path.is_file() and
                     not any(part in ('__pycache__', 'CMakeFiles', 'build') for part in path.relative_to(folder).parts))
    return {str(path.resolve()): {'sha256': fingerprint(path)['sha256'], 'bytes': path.stat().st_size}
            for path in sorted(paths)}


def contract_binaries(build):
    """收集各包构建根的可执行普通文件（含测试和业务程序），保存其实际身份。"""
    paths = [path for package in PACKAGES for path in (build / package).glob('*')
             if path.is_file() and os.access(path, os.X_OK)]
    require(paths, 'candidate build has no executable contract artifacts')
    return {str(path.resolve()): fingerprint(path) for path in sorted(paths)}


def installed_libraries(install):
    """组件经 dlopen 装载，不能只靠业务程序的 ldd 记录它们。"""
    paths = set(path.resolve() for pattern in ('lib/*.so*', '*/lib/*.so*')
                for path in install.glob(pattern) if path.is_file())
    require(paths, 'installation contains no shared-library artifacts')
    return {str(path): fingerprint(path) for path in sorted(paths)}

def check_baseline(source, files, tool_root, baseline):
    require(source.resolve() != tool_root.resolve() and
            (source / 'src').resolve() != (tool_root / 'src').resolve(),
            'baseline must use a separately frozen source tree, never the current candidate tree')
    listing = subprocess.check_output(['git', '-C', str(tool_root), 'ls-tree', '-rz', baseline, '--', 'src'])
    expected = {}
    for record in listing.split(b'\0'):
        if not record:
            continue
        header, path = record.split(b'\t', 1)
        relative = path.decode('utf-8')
        if production_path(relative):
            _, kind, oid = header.split()
            require(kind == b'blob', 'unsupported baseline source object: ' + relative)
            expected[relative] = oid.decode('ascii')
    require(expected and set(files) == set(expected), 'baseline production file inventory differs from Git snapshot')
    require(all(files[path]['git_blob_oid'] == oid for path, oid in expected.items()),
            'baseline production bytes differ from Git ' + baseline)


def tool_files(root):
    paths = [root / 'tools/validate.py', *sorted((root / 'tools/validation').glob('*.py'))]
    return {str(path.relative_to(root)): fingerprint(path)['sha256'] for path in paths}


# ---- 构建依赖：实际 CMake 缓存、编译依赖文件与动态库 ----

def cache_files(build, source, install):
    result = {}
    for package in PACKAGES:
        path = build / package / 'CMakeCache.txt'
        require(path.is_file(), 'missing actual CMake cache: ' + str(path))
        values = {}
        for line in path.read_text(encoding='utf-8', errors='strict').splitlines():
            if line and not line.startswith(('#', '//')) and '=' in line and ':' in line.split('=', 1)[0]:
                key, value = line.split('=', 1)
                values[key.split(':', 1)[0]] = value
        require(Path(values['CMAKE_HOME_DIRECTORY']).resolve() == source / 'src' / package,
                'CMake cache belongs to a different source tree: ' + package)
        require(Path(values['CMAKE_INSTALL_PREFIX']).resolve() in (install, install / package),
                'CMake cache installs to another prefix: ' + package)
        result[package] = {'file': fingerprint(path), 'effective': values}
    return result


def beneath(path, root):
    return path == root or root in path.parents


def compiler_dependencies(build, source, install, started_ns):
    """读取实际生产编译命令及其 .o.d；排除本项目生成物，保留外部实际头文件依赖。"""
    dependencies, commands, databases = {}, {}, {}
    for database in sorted(build.glob('*/compile_commands.json')):
        databases[str(database)] = fingerprint(database)
        for entry in json.loads(database.read_text(encoding='utf-8')):
            directory = Path(entry['directory']).resolve()
            file = (directory / entry['file']).resolve()
            if not beneath(file, source):
                continue  # ROS 生成源码不冒充手写源码；对应接口源定义在生产树身份中。
            if not production_path(file.relative_to(source).as_posix()):
                continue
            argv = entry.get('arguments') or shlex.split(entry['command'])
            output = entry.get('output')
            if output is None:
                require('-o' in argv, 'production compile command has no output identity')
                output = argv[argv.index('-o') + 1]
            dependency_file = Path(str((directory / output).resolve()) + '.d')
            require(dependency_file.is_file(), 'actual compiler dependency file missing: ' + str(dependency_file))
            text = dependency_file.read_text(encoding='utf-8').replace('\\\n', '').replace('\\\r\n', '')
            rule = text.splitlines()[0]
            require(':' in rule, 'malformed compiler dependency file')
            commands[str(dependency_file)] = {'argv': argv, 'directory': str(directory),
                                               'file': str(file), 'dependency_file': fingerprint(dependency_file)}
            for name in shlex.split(rule.split(':', 1)[1]):
                path = (directory / name).resolve()
                if beneath(path, build) or beneath(path, install):
                    continue
                # .deps 是工作空间内显式隔离的第三方依赖，不冒充自有生产源码。
                # 路径已 resolve：经 .deps 链接回测试目录仍按生产边界拒绝；依赖照常核对时间并留完整指纹。
                if beneath(path, source) and not beneath(path, source / '.deps'):
                    relative = path.relative_to(source).as_posix()
                    require(production_path(relative), 'production compile depends on excluded test/generated/asset: ' + relative)
                else:
                    require(path.stat().st_mtime_ns <= started_ns,
                            'external compiler dependency changed during build: ' + str(path))
                dependencies[str(path)] = fingerprint(path)
    require(commands and dependencies, 'no actual production compiler dependency evidence')
    return {'commands': commands, 'files': dependencies, 'databases': databases}


def binaries(install, kind):
    from .workflow import configuration, executable
    programs = {(package, program) for package, program, _ in configuration().ROLES.values()}
    return {package + '/' + program: fingerprint(executable(install, package, program))
            for package, program in sorted(programs)}


def runtime_dependencies(programs, install=None):
    env = os.environ.copy()
    if install is not None:
        # 构建完成时新 install 尚未 source；只对 ldd 提供该安装的包库路径。
        folders = [install / 'lib', *sorted(install.glob('*/lib'))]
        env['LD_LIBRARY_PATH'] = ':'.join(str(path) for path in folders if path.is_dir()) + ':' + env.get('LD_LIBRARY_PATH', '')
    result = {}
    for program in programs.values():
        completed = subprocess.run(['ldd', program['path']], text=True, capture_output=True, env=env, check=True)
        require('not found' not in completed.stdout, 'runtime shared library is unresolved: ' + completed.stdout)
        for line in completed.stdout.splitlines():
            match = re.search(r'(?:=>\s+)?(/\S+)\s+\(', line)
            if match:
                path = Path(match.group(1)).resolve()
                result[str(path)] = fingerprint(path)
    require(result, 'runtime shared library inventory is empty')
    return result


# ---- 身份刷新：仅处理明确允许且未参与编译的解释执行工具 ----

def noncompiled_tool_refresh(previous, current, root, build):
    """仅明确列出的解释执行验收模块可刷新身份；C++、测试和生成输入仍必须一致。"""
    if previous == current:
        return None
    require(previous is not None and current is not None and set(previous) == set(current),
            'contract inventory changed: use a fresh independent --build directory')
    changed = [Path(path).resolve() for path in current if current[path] != previous[path]]
    allowed = {root.resolve() / 'tools/validation' / name
               for name in ('clangd_contract.py', 'provenance.py', 'evidence.py', 'observations.py', 'workflow.py')}
    require(changed and set(changed) <= allowed,
            'compiled/test inputs changed: use a fresh independent --build directory')
    # 解释模块一旦被 CMake 生成规则引用，就不能仅刷新身份后复用构建。
    graph_files = sorted(set((root / 'src').rglob('CMakeLists.txt')) |
                         set((root / 'src').rglob('*.cmake')) |
                         set(build.rglob('*.cmake')) | set(build.rglob('*.make')))
    checked = {}
    for path in graph_files:
        if not path.is_file():
            continue
        contents = path.read_text(encoding='utf-8', errors='strict')
        require(all(item.name not in contents and item.stem not in contents for item in allowed),
                'validation module participates in CMake generation: ' + str(path))
        checked[str(path)] = fingerprint(path)
    # 按实际编译命令逐一核对 depfile；任一缺失即拒绝刷新，不能据此判定模块未参与编译。
    dependency_files = set()
    for database in sorted(build.glob('*/compile_commands.json')):
        for entry in json.loads(database.read_text(encoding='utf-8')):
            argv = entry.get('arguments') or shlex.split(entry['command'])
            output = entry.get('output')
            if output is None:
                require('-o' in argv, 'compile command lacks output during tool-only refresh')
                output = argv[argv.index('-o') + 1]
            path = Path(str((Path(entry['directory']) / output).resolve()) + '.d')
            require(path.is_file(), 'missing recorded compiler dependency graph: ' + str(path))
            dependency_files.add(path)
    dependency_files = sorted(dependency_files)
    require(dependency_files, 'missing actual C++ dependency graph for tool-only refresh')
    for path in dependency_files:
        contents = path.read_text(encoding='utf-8').replace('\\\n', '').replace('\\\r\n', '')
        rule = contents.splitlines()[0]
        require(':' in rule, 'malformed dependency file during tool-only refresh')
        names = shlex.split(rule.split(':', 1)[1])
        require(all(item.name not in {Path(name).name for name in names} for item in allowed),
                'validation module is a compiler dependency: ' + str(path))
        checked[str(path)] = fingerprint(path)
    return {'kind': 'noncompiled_validation_modules',
            'changed': [str(path) for path in sorted(changed)],
            'unchanged_cpp_and_profile_required': True, 'generation_and_dependency_graph': checked}


# ---- 构建生命周期：开始时撤销旧完成标记，成功后绑定实际产物 ----

def begin_build(args, root, baseline, argv):
    source, build, install = args.source.resolve(), args.build.resolve(), args.install.resolve()
    require(not beneath(build, source / 'src') and not beneath(install, source / 'src'),
            'build/install must be outside the production source tree')
    files = source_files(source)
    test_files = contract_files(source, root) if args.kind == 'candidate' else None
    if args.kind == 'baseline':
        check_baseline(source, files, root, baseline)
    marker = install / IDENTITY
    previous = json.loads(marker.read_text(encoding='utf-8')) if marker.exists() else None
    cmake_args = list(map(str, argv))[list(map(str, argv)).index('--cmake-args') + 1:]
    warm = bool(list(build.glob('*/CMakeCache.txt')))
    if not warm:
        require(not build.exists() or (build.is_dir() and not any(build.iterdir())),
                'first proven build requires an empty independent --build directory; old files are preserved')
    refresh = None
    if warm:
        require(previous and previous.get('schema') == SCHEMA and previous.get('status') == 'complete',
                'unproven/failed incremental objects: use a fresh independent --build directory; existing outputs are preserved')
        require(previous['source_root'] == str(source) and previous['build_root'] == str(build) and
                previous['kind'] == args.kind and previous['source_files'] == files and
                previous['cmake_args'] == cmake_args,
                'source/profile changed: use a fresh independent --build directory to prove actual compilation')
        # 刷新解释模块也必须保留原实际配置、数据库、depfile和构建/安装产物身份。
        recorded = [value['file'] for value in previous['cmake'].values()]
        recorded += list(previous['compiler_dependencies']['databases'].values())
        recorded += [value['dependency_file'] for value in previous['compiler_dependencies']['commands'].values()]
        recorded += list(previous['binaries'].values()) + list(previous['build_binaries'].values())
        recorded += list(previous['install_manifests'].values())
        require(all(fingerprint(item['path']) == item for item in recorded),
                'recorded C++ build/cache/installation changed; use a fresh independent --build directory')
        refresh = noncompiled_tool_refresh(previous['contract_files'], test_files, root, build)
        for group in (previous['compiler_dependencies']['files'], previous['runtime_dependencies'],
                      previous['installed_libraries'], previous.get('contract_binaries') or {}):
            require(all(fingerprint(path) == record for path, record in group.items()),
                    'previous build dependencies changed; use a fresh independent --build directory')
    identity = {'schema': SCHEMA, 'status': 'building', 'kind': args.kind,
                'source_root': str(source), 'build_root': str(build), 'install_root': str(install),
                'source_files': files, 'source_sha256': digest(files), 'contract_files': test_files, 'build_argv': list(map(str, argv)),
                'declared_business_version': args.business_version,
                'verified_baseline_commit': baseline if args.kind == 'baseline' else None,
                'tools': tool_files(root), 'started_ns': time.time_ns(), 'cmake_args': cmake_args,
                'trusted_unchanged_incremental': warm, 'noncompiled_tool_refresh': refresh}
    checkout = subprocess.run(['git', '-C', str(source), 'rev-parse', '--show-toplevel'], text=True, capture_output=True)
    actual_checkout = checkout.returncode == 0 and Path(checkout.stdout.strip()).resolve() == source
    commit = subprocess.run(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True, capture_output=True) if actual_checkout else None
    identity['source_git_commit'] = baseline if args.kind == 'baseline' else (
        commit.stdout.strip() if commit and commit.returncode == 0 else None)
    if marker.exists():
        write_json(args.output / 'previous-build-identity.json', previous)
    write_json(args.output / 'build-input-identity.json', identity)
    write_json(marker, identity)  # 构建失败时旧的“完成”身份不能继续冒充本次构建。
    return identity


def finish_build(args, identity, root, baseline):
    source, build, install = Path(identity['source_root']), Path(identity['build_root']), Path(identity['install_root'])
    after = source_files(source)
    require(after == identity['source_files'], 'production sources changed while building; rebuild before acceptance')
    if args.kind == 'baseline':
        check_baseline(source, after, root, baseline)
    require(tool_files(root) == identity['tools'], 'validation tools changed during build')
    identity['cmake'] = cache_files(build, source, install)
    identity['compiler_dependencies'] = compiler_dependencies(build, source, install, identity['started_ns'])
    identity['binaries'] = binaries(install, args.kind)
    identity['build_binaries'] = {}
    identity['install_manifests'] = {}
    for key, installed in identity['binaries'].items():
        package, program = key.split('/', 1)
        built = fingerprint(build / package / program)
        require(identity['trusted_unchanged_incremental'] or built['mtime_ns'] >= identity['started_ns'],
                'fresh build reused an older executable; actual compilation was not proven: ' + key)
        # CMake 安装可改 ELF RPATH/执行 strip，build 与 install 的全文件 SHA 可合理不同。
        # 两份文件分别留证；本次实际安装清单与时间提供关联，不冒称二进制逐字节等价证明。
        require(identity['trusted_unchanged_incremental'] or installed['mtime_ns'] >= identity['started_ns'],
                'fresh build did not refresh the selected installed executable: ' + key)
        manifest = build / package / 'install_manifest.txt'
        record = fingerprint(manifest)
        require(identity['trusted_unchanged_incremental'] or record['mtime_ns'] >= identity['started_ns'],
                'CMake install manifest predates this fresh build: ' + package)
        destinations = {str(Path(line).resolve()) for line in manifest.read_text(encoding='utf-8').splitlines() if line}
        require(str(Path(installed['path']).resolve()) in destinations,
                'selected executable absent from actual CMake install manifest: ' + key)
        identity['install_manifests'][package] = record
        identity['build_binaries'][key] = built
    require(identity['contract_files'] == (contract_files(source, root) if args.kind == 'candidate' else None),
            'contract inputs changed while building')
    identity['contract_binaries'] = contract_binaries(build) if args.kind == 'candidate' else None
    identity['installed_libraries'] = installed_libraries(install)
    identity['runtime_dependencies'] = runtime_dependencies(identity['binaries'], install)
    if identity.get('noncompiled_tool_refresh'):
        previous = json.loads((args.output / 'previous-build-identity.json').read_text(encoding='utf-8'))
        for group in ('binaries', 'build_binaries', 'installed_libraries', 'contract_binaries'):
            require(identity[group] == previous[group],
                    'tool-only refresh changed compiled artifacts: ' + group)
        identity['noncompiled_tool_refresh']['cpp_artifacts_unchanged_after_build'] = True
        identity['noncompiled_tool_refresh']['production_sha256'] = previous['source_sha256']
    identity['status'] = 'complete'
    identity['finished_ns'] = time.time_ns()
    write_json(install / IDENTITY, identity)
    write_json(args.output / 'build-identity.json', identity)


# ---- 运行准入：复核所选安装与已记录的源码、依赖和产物身份 ----

def verify_install(args, root, baseline, role_executables, directory):
    marker = args.install.resolve() / IDENTITY
    require(marker.is_file(), 'installation has no build provenance; run the unified build stage first')
    identity = json.loads(marker.read_text(encoding='utf-8'))
    require(identity.get('schema') == SCHEMA and identity.get('status') == 'complete', 'build identity is incomplete')
    require(identity['kind'] == args.kind and identity['install_root'] == str(args.install.resolve()),
            'selected install belongs to another build kind/prefix')
    require(identity['source_root'] == str(args.source.resolve()), '--source differs from the actual compiled source root')
    source = Path(identity['source_root'])
    current = source_files(source)
    require(current == identity['source_files'] and digest(current) == identity['source_sha256'],
            'production source changed since the installation was built')
    if args.kind == 'baseline':
        require(identity['verified_baseline_commit'] == baseline, 'build identity references another baseline')
        check_baseline(source, current, root, baseline)
    require(tool_files(root) == identity['tools'], 'validation tools changed since the recorded build')
    require(identity['contract_files'] == (contract_files(source, root) if args.kind == 'candidate' else None),
            'contract inputs changed since the recorded build')
    require(identity['contract_binaries'] == (contract_binaries(Path(identity['build_root']))
            if args.kind == 'candidate' else None), 'contract executables changed since the recorded build')
    require(installed_libraries(args.install.resolve()) == identity['installed_libraries'],
            'installed component/shared-library artifacts changed since the recorded build')
    for package, cache in identity['cmake'].items():
        require(fingerprint(cache['file']['path']) == cache['file'], 'CMake cache changed after build: ' + package)
    for path, expected in identity['compiler_dependencies']['databases'].items():
        require(fingerprint(path) == expected, 'actual compile command database changed after build: ' + path)
    for path, expected in identity['compiler_dependencies']['files'].items():
        require(fingerprint(path) == expected, 'actual compiler dependency changed after build: ' + path)
    for package, expected in identity['install_manifests'].items():
        require(fingerprint(expected['path']) == expected, 'CMake install manifest changed after build: ' + package)
    for key, expected in identity['build_binaries'].items():
        require(fingerprint(expected['path']) == expected, 'actual build output changed after installation: ' + key)
    actual = binaries(args.install.resolve(), args.kind)
    require(actual == identity['binaries'], 'installed executable bytes/mtime differ from recorded build')
    known = {str(Path(item['path']).resolve()) for item in actual.values()}
    require(all(str(Path(path).resolve()) in known for path in role_executables.values()),
            'launcher selected a binary outside the verified build')
    libraries = runtime_dependencies(actual)
    require(libraries == identity['runtime_dependencies'], 'runtime library resolution/bytes differ from build identity')
    write_json(directory / 'verified-build-identity.json', {'identity': fingerprint(marker),
               'source_sha256': identity['source_sha256'], 'source_git_commit': identity['source_git_commit'],
               'verified_baseline_commit': identity['verified_baseline_commit'], 'kind': args.kind,
               'selected_roles': {role: fingerprint(path) for role, path in role_executables.items()},
               'runtime_dependencies_sha256': digest(libraries), 'status': 'verified_not_started'})
