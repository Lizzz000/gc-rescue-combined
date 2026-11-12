"""配置入口：自动路径在 bridge 角色下拒绝未标定的夹爪键；其它角色不受影响。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from runtime.configuration import ConfigurationError, load_configuration

OFFICIAL = Path(__file__).resolve().parents[1] / 'config' / 'rescue.yaml'


def _resolver(package):
    return str(Path(__file__).resolve().parents[2] / package)


class AutoGripperConfigTest(unittest.TestCase):
    def test_official_yaml_rejects_empty_move_s_when_bridge_enabled(self):
        with self.assertRaises(ConfigurationError) as raised:
            load_configuration(OFFICIAL, run='all', package_resolver=_resolver, require_assets=False)
        self.assertIn('gripper.move_s', str(raised.exception))
        self.assertIn('待实车标定', str(raised.exception))
        with self.assertRaises(ConfigurationError) as raised_bridge:
            load_configuration(OFFICIAL, run='bridge', package_resolver=_resolver, require_assets=False)
        self.assertIn('gripper.move_s', str(raised_bridge.exception))

    def test_official_yaml_loads_when_bridge_is_not_selected(self):
        snapshot = load_configuration(OFFICIAL, run='vision', package_resolver=_resolver,
                                      require_assets=False)
        self.assertIsNone(snapshot['values']['gripper.move_s'])
        self.assertNotIn('bridge_node', snapshot['nodes'])


if __name__ == '__main__':
    unittest.main()
