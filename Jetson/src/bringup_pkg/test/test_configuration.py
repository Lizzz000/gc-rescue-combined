"""验证正式启动配置、相机预设和夹爪参数。"""
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from runtime.camera_controls import apply_camera_controls
from runtime.configuration import ConfigurationError, load_configuration, read_configuration
from runtime.ros import launch as rescue_launch

OFFICIAL = Path(__file__).resolve().parents[1] / 'config' / 'rescue.yaml'


def _resolver(package):
    return str(Path(__file__).resolve().parents[2] / package)


class StreamConfigTest(unittest.TestCase):
    def test_stream_is_opt_in_without_changing_detection(self):
        for role in ('vision', 'all'):
            with self.subTest(role=role):
                default = load_configuration(OFFICIAL, role, _resolver)
                enabled = load_configuration(OFFICIAL, role, _resolver, stream_enabled=True)
                self.assertFalse(default['nodes']['stream_node']['stream_enabled'])
                self.assertTrue(enabled['nodes']['stream_node']['stream_enabled'])
                self.assertNotIn('jetson_jpeg', default['required_backends'])
                self.assertIn('jetson_jpeg', enabled['required_backends'])
                self.assertEqual(default['nodes']['detect_node'], enabled['nodes']['detect_node'])
                self.assertNotEqual(default['effective_sha256'], enabled['effective_sha256'])

    def test_explicit_off_overrides_a_config_that_enables_streaming(self):
        config = read_configuration(OFFICIAL, package_resolver=_resolver)
        config['vision']['stream']['stream_enabled'] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.yaml'
            path.write_text(yaml.safe_dump(config), encoding='utf-8')
            self.assertTrue(load_configuration(path, 'vision', _resolver)['nodes']['stream_node']['stream_enabled'])
            off = load_configuration(path, 'vision', _resolver, stream_enabled=False)
            self.assertFalse(off['nodes']['stream_node']['stream_enabled'])


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
        with tempfile.TemporaryDirectory() as directory:
            ros_parameters = yaml.safe_load(rescue_launch.materialize(
                snapshot, directory).read_text(encoding='utf-8'))['detect_node']['ros__parameters']
        self.assertEqual(ros_parameters['camera_brightness'], calibrated['brightness'])
        self.assertEqual({name.removeprefix('camera_controls.'): value
                          for name, value in ros_parameters.items() if name.startswith('camera_controls.')},
                         {name: value for name, value in calibrated.items() if name != 'brightness'})
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


class LaunchFailureTest(unittest.TestCase):
    def test_every_role_has_an_exit_handler_before_launching(self):
        snapshot = load_configuration(OFFICIAL, run='all', package_resolver=_resolver)
        modules = {name: MagicMock() for name in (
            'launch', 'launch.actions', 'launch.event_handlers', 'launch.substitutions',
            'launch_ros', 'launch_ros.actions')}
        roles = tuple(snapshot['roles'])
        nodes = [object() for _ in roles]
        handlers = [object() for _ in roles]
        modules['launch_ros.actions'].Node.side_effect = nodes
        modules['launch.actions'].RegisterEventHandler.side_effect = handlers
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.dict(sys.modules, modules),
            patch.dict(os.environ, RESCUE_RUN_DIR=directory),
            patch.object(rescue_launch, 'load_configuration', return_value=snapshot),
            patch.object(rescue_launch, 'apply_camera_controls', return_value={}),
        ):
            actions = rescue_launch._start(None)

        self.assertEqual(actions[1:], handlers + nodes)
        registrations = modules['launch.event_handlers'].OnProcessExit.call_args_list
        self.assertEqual(len(registrations), len(roles))
        for role, node, registration in zip(roles, nodes, registrations):
            with self.subTest(role=role):
                self.assertIs(registration.kwargs['target_action'], node)
                callback = registration.kwargs['on_exit']
                for code in (0, 7):
                    event = SimpleNamespace(returncode=code)
                    with self.assertRaisesRegex(RuntimeError, f'{role} .* code {code}'):
                        callback(event, SimpleNamespace(is_shutdown=False))
                    self.assertIsNone(callback(event, SimpleNamespace(is_shutdown=True)))


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

    def test_field_exposure_reference_is_site_specific(self):
        site = OFFICIAL.with_name('field_20261122.yaml')
        for config, expected in ((OFFICIAL, 0), (site, 132)):
            snapshot = load_configuration(config, 'vision', _resolver, require_assets=False)
            self.assertEqual(snapshot['nodes']['detect_node']['field.neutral_reference_luma'], expected)
        legacy = read_configuration(site, package_resolver=_resolver)
        del legacy['vision']['field']['neutral_reference_luma']
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'legacy.yaml'
            path.write_text(yaml.safe_dump(legacy), encoding='utf-8')
            snapshot = load_configuration(path, 'vision', _resolver, require_assets=False)
            self.assertEqual(snapshot['nodes']['detect_node']['field.neutral_reference_luma'], 0)

    def test_official_yaml_passes_gripper_preset_to_bridge(self):
        for role in ('all', 'bridge'):
            with self.subTest(role=role):
                snapshot = load_configuration(OFFICIAL, run=role, package_resolver=_resolver,
                                              require_assets=False)
                bridge = snapshot['nodes']['bridge_node']
                self.assertEqual(bridge['gripper_closed_deg'], -50.0)
                self.assertEqual(bridge['gripper_open_deg'], 0.0)
                self.assertEqual(bridge['gripper_move_s'], 1.0)
                self.assertEqual(bridge['gripper_ramp_fraction'], 0.8)
                self.assertEqual(bridge['gripper_open_hold_max_nm'], 0.14)
                self.assertEqual(bridge['gripper_output_tolerance_deg'], 2.0)
                self.assertEqual(bridge['gripper_closed_trim_deg'], 0.35)
                self.assertEqual(bridge['gripper_open_trim_deg'], 5.0)

    def test_legacy_gripper_step_and_invalid_ramp(self):
        config = read_configuration(OFFICIAL, package_resolver=_resolver)
        for key in ('ramp_fraction', 'closed_trim_deg', 'open_trim_deg', 'open_hold_max_nm', 'output_tolerance_deg'):
            del config['gripper'][key]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.yaml'
            path.write_text(yaml.safe_dump(config), encoding='utf-8')
            snap = load_configuration(path, 'bridge', _resolver)
            self.assertEqual(snap['nodes']['bridge_node']['gripper_ramp_fraction'], 0.0)
            self.assertEqual(snap['nodes']['bridge_node']['gripper_open_hold_max_nm'], 0.0)
            for bad in (-0.1, 1.01, float('nan'), True):
                with self.subTest(bad=bad):
                    config['gripper']['ramp_fraction'] = bad
                    path.write_text(yaml.safe_dump(config), encoding='utf-8')
                    with self.assertRaisesRegex(ConfigurationError, r'gripper\.ramp_fraction'):
                        load_configuration(path, 'bridge', _resolver)

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
