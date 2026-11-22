"""验证正式启动配置的夹爪参数及缺失时的启动检查。"""
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from runtime.configuration import ConfigurationError, load_configuration, read_configuration

OFFICIAL = Path(__file__).resolve().parents[1] / 'config' / 'rescue.yaml'


def _resolver(package):
    return str(Path(__file__).resolve().parents[2] / package)


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
