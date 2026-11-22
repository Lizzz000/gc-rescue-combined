# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""验证验收脚本的角色接线与 CTest 登记检查，无需启动 ROS 或连接设备。"""
import importlib
import json
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from validation import workflow
from validation.evidence import ContractFailure


class ContractRegistrationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.build = self.root / 'build'
        self.output = self.root / 'output'
        self.packages = {'vision_pkg', 'track_pkg', 'mission_pkg', 'bridge_pkg', 'bringup_pkg'}
        self.listings = {}
        for package in self.packages:
            package_build = self.build / package
            package_build.mkdir(parents=True)
            (package_build / 'CTestTestfile.cmake').touch()
            self.listings[package] = [{
                'name': name,
                'command': ['python', 'run_test.py', f'test_results/{package}/{name}.xml'],
            } for name in sorted(workflow.REQUIRED_CONTRACTS[package])]

    def list_ctest(self, argv, **_kwargs):
        self.assertEqual(argv[:3], ['ctest', '--show-only=json-v1', '--test-dir'])
        return SimpleNamespace(
            returncode=0, stderr='', stdout=json.dumps({'tests': self.listings[Path(argv[3]).name]}))

    def test_interface_is_built_but_not_enumerated(self):
        self.assertIn('interface_pkg', workflow.PACKAGES)
        with patch.object(workflow.subprocess, 'run', side_effect=self.list_ctest) as ctest:
            rows, expected = workflow.contract_registrations(self.build, self.output, {})
        self.assertEqual({Path(call.args[0][3]).name for call in ctest.call_args_list}, self.packages)
        self.assertEqual(len(ctest.call_args_list), len(self.packages))
        self.assertEqual({row['package'] for row in rows}, self.packages)
        self.assertEqual({Path(row['result_file']) for row in rows}, expected)
        self.assertTrue(expected)

    def test_missing_required_registration_fails_for_nonempty_package(self):
        removed = self.listings['bridge_pkg'].pop()
        self.assertTrue(self.listings['bridge_pkg'])
        with patch.object(workflow.subprocess, 'run', side_effect=self.list_ctest):
            with self.assertRaises(ContractFailure) as error:
                workflow.contract_registrations(self.build, self.output, {})
        self.assertIn(removed['name'], str(error.exception))

    def test_required_contracts_cannot_share_one_result(self):
        first, second = self.listings['bridge_pkg'][:2]
        second['command'][-1] = first['command'][-1]
        with patch.object(workflow.subprocess, 'run', side_effect=self.list_ctest):
            with self.assertRaisesRegex(ContractFailure, 'reuse one result XML'):
                workflow.contract_registrations(self.build, self.output, {})


class Node(SimpleNamespace):
    pass


class Shutdown(SimpleNamespace):
    pass


class LaunchConfiguration:
    def __init__(self, key):
        self.key = key

    def perform(self, context):
        return context[self.key]


def mock_module(name, **attributes):
    module = ModuleType(name)
    vars(module).update(attributes)
    return module


class LaunchTopologyTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.install = self.root / 'install'
        self.bringup = workflow.ROOT / 'src/bringup_pkg'
        self.cfg = workflow.configuration()
        modules = {
            'ament_index_python': mock_module('ament_index_python'),
            'ament_index_python.packages': mock_module(
                'ament_index_python.packages',
                get_package_share_directory=lambda name: str(workflow.ROOT / 'src' / name),
                get_package_prefix=lambda name: str(self.install / name)),
            'launch': mock_module('launch'),
            'launch.actions': mock_module(
                'launch.actions', EmitEvent=SimpleNamespace, LogInfo=SimpleNamespace,
                RegisterEventHandler=lambda handler: SimpleNamespace(handler=handler)),
            'launch.event_handlers': mock_module('launch.event_handlers', OnProcessExit=SimpleNamespace),
            'launch.events': mock_module('launch.events', Shutdown=Shutdown),
            'launch.substitutions': mock_module(
                'launch.substitutions', LaunchConfiguration=LaunchConfiguration),
            'launch_ros': mock_module('launch_ros'),
            'launch_ros.actions': mock_module('launch_ros.actions', Node=Node),
        }
        patched_modules = patch.dict(sys.modules, modules)
        patched_modules.start()
        self.addCleanup(patched_modules.stop)
        document = self.cfg.read_configuration(self.bringup / 'config/rescue.yaml')
        # 只用于测试拓扑，临时填充标定占位值，不启动任何硬件。
        document['gripper']['move_s'] = 1.0
        self.config = self.root / 'config.yaml'
        self.config.write_text(yaml.safe_dump(document), encoding='utf-8')

    def test_production_topology_and_bridge_exit_action(self):
        launch = importlib.import_module('rescue_validation_runtime.ros.launch')
        for run, processes, nodes in (('all', 7, 8), ('vision', 1, 2)):
            with self.subTest(run=run):
                output = self.root / run
                with patch.dict(os.environ, {'RESCUE_RUN_DIR': str(output)}):
                    actions = launch._start({'config': str(self.config), 'run': run})
                snapshot = json.loads((output / 'effective-configuration.json').read_text(encoding='utf-8'))
                launched = [action for action in actions if isinstance(action, Node)]
                self.assertEqual([action.arguments[1] for action in launched], snapshot['roles'])
                self.assertEqual(len(launched), processes)
                self.assertEqual(len(snapshot['nodes']), nodes)
                handlers = [action.handler for action in actions if hasattr(action, 'handler')]
                if run == 'all':
                    handler, = handlers
                    bridge, = [action for action in launched if action.arguments == ['--role', 'bridge']]
                    self.assertIs(handler.target_action, bridge)
                    self.assertTrue(any(isinstance(getattr(action, 'event', None), Shutdown)
                                        for action in handler.on_exit))
                else:
                    self.assertEqual(set(snapshot['nodes']), {'detect_node', 'stream_node'})
                    self.assertFalse(handlers)

    def test_validation_uses_selected_snapshot_for_launch(self):
        args = SimpleNamespace(
            config=self.bringup / 'config/rescue.yaml', install=self.install, stage='capture')
        for run, clock in (('vision', False), ('detect', True)):
            with self.subTest(run=run):
                directory = self.root / run
                directory.mkdir()
                snapshot = workflow.case_snapshot(args, clock=clock, directory=directory, run=run)
                self.assertEqual(snapshot['roles'], [run])
                if clock:
                    self.assertEqual(set(snapshot['nodes']), {'detect_node'})
                    self.assertIs(snapshot['nodes']['detect_node']['use_sim_time'], True)
                    self.assertEqual(snapshot['values']['vision.detect.camera_backend'], 'gstreamer')
                    self.assertEqual(snapshot['values']['vision.detect.camera_pipeline'], '')
                    self.assertEqual(snapshot['values']['vision.detect.camera_device'],
                                     '/dev/rescue_validation_absent')
                with (
                    patch.object(workflow, 'executable', side_effect=lambda install, package, name:
                                 install / package / 'lib' / package / name),
                    patch.object(workflow, 'verify_install'),
                    patch.object(workflow, 'OwnedProcess') as launcher,
                ):
                    workflow.launch_case(args, snapshot, directory, clock=clock)
                argv = launcher.call_args.args[0]
                if clock:
                    clock_launch = (directory / 'observation.launch.py').resolve()
                    self.assertEqual(argv, ['ros2', 'launch', str(clock_launch)])
                    generated = clock_launch.read_text(encoding='utf-8')
                    self.assertIn("arguments=['--role', 'detect']", generated)
                    self.assertEqual(generated.count('Node('), 1)
                    self.assertEqual(set(launcher.call_args.kwargs['role_executables']), {'detect'})
                else:
                    self.assertIn('run:=vision', argv)
                    self.assertEqual(set(launcher.call_args.args[3]), {'vision'})


if __name__ == '__main__':
    unittest.main()
