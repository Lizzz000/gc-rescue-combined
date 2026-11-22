"""验证正式启动配置、相机预设和夹爪参数。"""
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from runtime.camera_controls import apply_camera_controls
from runtime.configuration import ConfigurationError, load_configuration, read_configuration
from runtime.ros import launch as rescue_launch

OFFICIAL = Path(__file__).resolve().parents[1] / 'config' / 'rescue.yaml'


def _resolver(package):
    return str(Path(__file__).resolve().parents[2] / package)


class CameraControlsTest(unittest.TestCase):
    def test_field_preset_sets_modes_then_manual_values_and_reads_device(self):
        snapshot = load_configuration(OFFICIAL, run='vision', package_resolver=_resolver)
        calibrated = {
            'brightness': 175, 'contrast': 68, 'saturation': 90, 'hue': 200,
            'gamma': 138, 'gain': 48, 'auto_exposure': 1, 'exposure_time_absolute': 150,
            'white_balance_automatic': 1, 'focus_automatic_continuous': 1,
            'exposure_dynamic_framerate': 0, 'sharpness': 2000, 'power_line_frequency': 1,
            'backlight_compensation': 0, 'zoom_absolute': 100,
        }
        # 模拟驱动回读，与下发顺序无关。
        output = ''.join(f'{name}: {value}\n' for name, value in reversed(calibrated.items()))
        output = output.replace('auto_exposure: 1\n', 'auto_exposure: 1 (Manual Mode)\n')
        with patch.object(sys, 'platform', 'linux'), patch('subprocess.run') as run:
            run.return_value.stdout = output
            actual = apply_camera_controls(snapshot)
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(len(commands), 3)
        for argv in commands:
            self.assertEqual(argv[:-1], ['v4l2-ctl', '-d', '/dev/video0'])
        modes = {'auto_exposure', 'white_balance_automatic', 'focus_automatic_continuous'}
        for argv, names in zip(commands[:2], (modes, calibrated.keys() - modes)):
            option, assignments = argv[-1].split('=', 1)
            self.assertEqual(option, '--set-ctrl')
            self.assertEqual(set(assignments.split(',')), {f'{k}={calibrated[k]}' for k in names})
        option, names = commands[2][-1].split('=', 1)
        self.assertEqual(option, '--get-ctrl')
        self.assertEqual(set(names.split(',')), calibrated.keys())
        self.assertEqual(actual, calibrated)

    def test_missing_control_or_stale_brightness_fails_readback(self):
        snapshot = load_configuration(OFFICIAL, run='detect', package_resolver=_resolver)
        for name in list(snapshot['values']):
            if name.startswith('vision.detect.camera_controls.'):
                del snapshot['values'][name]
        with patch.object(sys, 'platform', 'linux'), patch('subprocess.run') as run:
            for output in ('brightness: 150\n', ''):
                with self.subTest(output=output):
                    run.return_value.stdout = output
                    with self.assertRaisesRegex(ConfigurationError, 'brightness'):
                        apply_camera_controls(snapshot)

    def test_unrelated_roles_and_replay_do_not_access_camera(self):
        vision = load_configuration(OFFICIAL, run='vision', package_resolver=_resolver)
        stream = load_configuration(OFFICIAL, run='stream', package_resolver=_resolver)
        with patch('subprocess.run') as run:
            for snap, backend, pipeline, platform in (
                    (stream, 'gstreamer', '', 'linux'), (vision, 'file', '', 'linux'),
                    (vision, 'gstreamer', 'videotestsrc ! appsink', 'linux'),
                    (vision, 'gstreamer', '', 'win32')):
                with self.subTest(roles=snap['roles'], backend=backend, platform=platform):
                    if 'detect_node' in snap['nodes']:
                        snap['nodes']['detect_node'].update(camera_backend=backend, camera_pipeline=pipeline)
                    with patch.object(sys, 'platform', platform):
                        self.assertEqual(apply_camera_controls(snap), {})
            run.assert_not_called()

    def test_driver_write_error_prevents_launching_any_node(self):
        snapshot = load_configuration(OFFICIAL, run='all', package_resolver=_resolver)
        # 在无 ROS 的主机上走正式 launch 装配路径，保留真实预写与失败传播。
        modules = {name: MagicMock() for name in (
            'launch', 'launch.actions', 'launch.event_handlers', 'launch.events',
            'launch.substitutions', 'launch_ros', 'launch_ros.actions')}
        failure = subprocess.CalledProcessError(1, ['v4l2-ctl'], stderr='device busy')
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.dict(sys.modules, modules),
            patch.dict(os.environ, RESCUE_RUN_DIR=directory),
            patch.object(sys, 'platform', 'linux'),
            patch.object(rescue_launch, 'load_configuration', return_value=snapshot),
            patch('subprocess.run', side_effect=failure) as run,
        ):
            with self.assertRaises(ConfigurationError):
                rescue_launch._start(None)
            self.assertEqual(run.call_count, 1)
            modules['launch_ros.actions'].Node.assert_not_called()


class AutoGripperConfigTest(unittest.TestCase):
    def test_dotted_and_nested_fields_have_one_identity(self):
        config = read_configuration(OFFICIAL, package_resolver=_resolver)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.yaml'
            for mapping in ({**config, 'gripper.kp': 2.0}, {'gripper.kp': 2.0, **config}):
                path.write_text(yaml.safe_dump(mapping, sort_keys=False), encoding='utf-8')
                with self.assertRaisesRegex(ConfigurationError, r'field defined twice: gripper\.kp'):
                    load_configuration(path, run='vision', package_resolver=_resolver)
            config['gripper.kp'] = config['gripper'].pop('kp')
            path.write_text(yaml.safe_dump(config), encoding='utf-8')
            snapshot = load_configuration(path, run='vision', package_resolver=_resolver)
            self.assertEqual(snapshot['values']['gripper.kp'], config['gripper.kp'])

    def test_invalid_device_types_and_infinite_gains_report_field(self):
        config = read_configuration(OFFICIAL, package_resolver=_resolver)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.yaml'
            for section, key, value in (('link', 'device', 17), ('gripper', 'device', None),
                                        ('gripper', 'kp', float('inf')), ('gripper', 'kd', float('inf'))):
                with self.subTest(section=section, key=key):
                    original = config[section][key]
                    config[section][key] = value
                    path.write_text(yaml.safe_dump(config), encoding='utf-8')
                    with self.assertRaises(ConfigurationError) as raised:
                        load_configuration(path, run='vision', package_resolver=_resolver)
                    self.assertIn(f'{section}.{key}', str(raised.exception))
                    config[section][key] = original

    def test_official_black_band_width_reaches_detect_node(self):
        snapshot = load_configuration(OFFICIAL, run='vision', package_resolver=_resolver,
                                      require_assets=False)
        self.assertEqual(snapshot['values']['vision.field.black_width_m'], 0.020)
        self.assertEqual(snapshot['nodes']['detect_node']['field.black_width_m'], 0.020)

    def test_official_yaml_passes_gripper_preset_to_bridge(self):
        for role in ('all', 'bridge'):
            with self.subTest(role=role):
                snapshot = load_configuration(OFFICIAL, run=role, package_resolver=_resolver,
                                              require_assets=False)
                bridge = snapshot['nodes']['bridge_node']
                self.assertEqual(bridge['gripper_closed_deg'], -50.0)
                self.assertEqual(bridge['gripper_open_deg'], 0.0)
                self.assertEqual(bridge['gripper_move_s'], 1.0)

    def test_empty_move_s_rejected_when_bridge_enabled(self):
        config = read_configuration(OFFICIAL, package_resolver=_resolver)
        config['gripper']['move_s'] = None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.yaml'
            path.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
            for role in ('all', 'bridge'):
                with self.subTest(role=role):
                    with self.assertRaises(ConfigurationError) as raised:
                        load_configuration(path, run=role, package_resolver=_resolver, require_assets=False)
                    self.assertIn('gripper.move_s', str(raised.exception))
                    self.assertIn('待实车标定', str(raised.exception))
            snapshot = load_configuration(path, run='vision', package_resolver=_resolver,
                                          require_assets=False)
            self.assertIsNone(snapshot['values']['gripper.move_s'])

    def test_official_yaml_loads_when_bridge_is_not_selected(self):
        snapshot = load_configuration(OFFICIAL, run='vision', package_resolver=_resolver,
                                      require_assets=False)
        self.assertNotIn('bridge_node', snapshot['nodes'])


if __name__ == '__main__':
    unittest.main()
