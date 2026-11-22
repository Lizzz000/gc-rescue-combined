# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""冻结图回放的配置和启动回归。ROS 进程和图像解码使用主机替身。"""
import json
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from validation import workflow
from validation.evidence import ContractFailure


REPLAY_ENV = 'RESCUE_VALIDATION_NVMM_REPLAY'


class CaptureReplayTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.config = self.root / 'case-source.yaml'
        self.input = self.root / "冻结 图像 ' & replay.png"
        self.input.write_bytes(b'replay fixture')
        self.install = self.root / 'install'
        self.directory = self.root / 'artifacts'
        self.directory.mkdir()
        self.args = SimpleNamespace(stage='capture', config=self.config, install=self.install,
                                    output=self.directory)

        packages = ModuleType('ament_index_python.packages')
        packages.get_package_share_directory = lambda name: str(workflow.ROOT / 'src' / name)
        packages.get_package_prefix = lambda name: str(self.install / name)
        cv2 = ModuleType('cv2')
        cv2.IMREAD_COLOR = 1
        cv2.imread = Mock(return_value=SimpleNamespace(shape=(7, 9, 3)))
        self.imread = cv2.imread
        modules = patch.dict(sys.modules, {
            'ament_index_python': ModuleType('ament_index_python'),
            'ament_index_python.packages': packages,
            'cv2': cv2,
        })
        modules.start()
        self.addCleanup(modules.stop)

        config = workflow.configuration().read_configuration(
            workflow.ROOT / 'src/bringup_pkg/config/rescue.yaml')
        config['gripper']['move_s'] = 1.0
        self.config.write_text(yaml.safe_dump(config, allow_unicode=True), encoding='utf-8')

    def test_file_case_keeps_frozen_dimensions_and_uses_owned_pty_paths(self):
        snapshot = workflow.case_snapshot(
            self.args, self.input, directory=self.directory, link_device='/dev/pts/11',
            run='all', gripper_device='/dev/pts/12')
        config = snapshot['values']
        self.assertEqual(config['vision.detect.camera_backend'], 'file')
        self.assertEqual(config['vision.detect.camera_device'], str(self.input.resolve()))
        self.assertEqual(config['vision.detect.camera_brightness'], -1)
        self.assertEqual(config['vision.detect.camera_pipeline'], '')
        self.assertEqual((config['shared.image.width'], config['shared.image.height']), (9, 7))
        self.assertEqual((config['link.device'], config['gripper.device']),
                         ('/dev/pts/11', '/dev/pts/12'))
        self.assertEqual(len(snapshot['roles']), 7)
        self.assertIn('tensorrt_gpu', snapshot['required_backends'])

    def test_file_case_rejects_either_dimension_smaller_than_nvmm_letterbox_input(self):
        for shape in ((7, 1, 3), (1, 9, 3)):
            self.imread.return_value = SimpleNamespace(shape=shape)
            with self.subTest(shape=shape), self.assertRaises(ContractFailure):
                workflow.case_snapshot(self.args, self.input, directory=self.directory, run='vision')

    def test_replay_path_is_only_passed_through_the_selected_child_environment(self):
        snapshot = workflow.case_snapshot(
            self.args, self.input, directory=self.directory, run='vision')
        with (
            patch.dict(os.environ, {REPLAY_ENV: 'inherited'}),
            patch.object(workflow, 'executable', side_effect=lambda install, package, name:
                         install / package / 'lib' / package / name),
            patch.object(workflow, 'verify_install'),
            patch.object(workflow, 'OwnedProcess') as launcher,
        ):
            workflow.launch_case(self.args, snapshot, self.directory, replay=self.input)
            replay_argv, _, replay_env, _ = launcher.call_args.args
            self.assertEqual(replay_env[REPLAY_ENV], str(self.input.resolve()))
            self.assertNotIn(str(self.input.resolve()), replay_argv)
            self.assertEqual(os.environ[REPLAY_ENV], 'inherited')

            workflow.launch_case(self.args, snapshot, self.directory)
            self.assertNotIn(REPLAY_ENV, launcher.call_args.args[2])

            clock = workflow.case_snapshot(
                self.args, clock=True, directory=self.directory, run='detect')
            workflow.launch_case(self.args, clock, self.directory, clock=True)
            clock_call = launcher.call_args
            clock_env = clock_call.kwargs['env'] if 'env' in clock_call.kwargs else clock_call.args[2]
            self.assertNotIn(REPLAY_ENV, clock_env)

    def test_replay_rejects_a_different_path_or_non_file_backend(self):
        file_case = workflow.case_snapshot(
            self.args, self.input, directory=self.directory, run='vision')
        live_directory = self.root / 'live'
        live_directory.mkdir()
        live_case = workflow.case_snapshot(self.args, directory=live_directory, run='vision')
        other = self.root / 'other.png'
        other.write_bytes(b'other fixture')
        with (
            patch.object(workflow, 'executable', side_effect=lambda install, package, name:
                         install / package / 'lib' / package / name),
            patch.object(workflow, 'verify_install'),
            patch.object(workflow, 'OwnedProcess') as launcher,
        ):
            for snapshot, replay in ((file_case, other), (live_case, self.input)):
                with self.subTest(replay=replay), self.assertRaises(ContractFailure):
                    workflow.launch_case(self.args, snapshot, self.directory, replay=replay)
            launcher.assert_not_called()

    def test_prelaunch_failure_closes_any_opened_pty(self):
        from validation import observations, s288_probe

        (self.root / 'second.jpg').write_bytes(b'second fixture')
        (self.root / 'third.png').write_bytes(b'third fixture')
        self.args.inputs = self.root
        self.args.business_version = 'host-test'
        self.args.kind = 'candidate'
        for fail_gripper in (False, True):
            with self.subTest(fail_gripper=fail_gripper):
                can = SimpleNamespace(device='/dev/pts/11', close=Mock())
                gripper = SimpleNamespace(device='/dev/pts/12', close=Mock())
                ros = ModuleType('rclpy')
                ros.init, ros.shutdown = Mock(), Mock()
                error = ContractFailure('fixture cannot prepare replay')
                with (
                    patch.dict(sys.modules, {'rclpy': ros}),
                    patch.object(workflow, 'capture_scenarios', return_value=[('chain', self.input)]),
                    patch.object(observations, 'UsbProbe', return_value=can),
                    patch.object(s288_probe, 'S288Probe', return_value=gripper,
                                 side_effect=error if fail_gripper else None),
                    patch.object(workflow, 'case_snapshot', side_effect=error),
                    patch.object(workflow, 'launch_case') as launcher,
                ):
                    with self.assertRaisesRegex(ContractFailure, 'cannot prepare replay'):
                        workflow.capture(self.args)
                    launcher.assert_not_called()
                can.close.assert_called_once()
                if fail_gripper:
                    gripper.close.assert_not_called()
                else:
                    gripper.close.assert_called_once()
                ros.shutdown.assert_called_once()
                report = json.loads((self.args.output / 'capture.json').read_text(encoding='utf-8'))
                self.assertEqual(report['status'], 'failed')
                self.assertEqual(report['cases'][0]['cleanup_errors'], [])


if __name__ == '__main__':
    unittest.main()
