"""桌面遥控调度与串口状态时效回归；不打开窗口或串口。"""
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import uart6_manual as manual
import uart6_protocol as wire


class StatusDeadlineTests(unittest.TestCase):
    def driving_panel(self):
        panel = manual.ManualPanel.__new__(manual.ManualPanel)
        panel.root, panel.label, panel.note, panel.pose_label = (Mock() for _ in range(4))
        panel.arm_deadline = panel.pending = panel.return_id = panel.pose = None
        panel.speed, panel.yaw_rate = 0.10, 0.50
        panel.active, panel.state, panel.flags = True, wire.RUNNING, 2
        panel.last_status = 1.0
        panel.motion = {"w"}
        panel.reader = wire.FrameReader()
        panel.port = Mock()
        status = wire.frame(wire.STATUS, struct.pack("<BBHBBB", wire.RUNNING, 2, 1, wire.ARM, wire.ACCEPTED, 0))
        panel.port.in_waiting = len(status)
        panel.port.read.return_value = status
        panel.port.write.side_effect = len
        return panel

    def test_buffered_status_cannot_renew_expired_control(self):
        for gap in (manual.STATUS_TIMEOUT_S, manual.STATUS_TIMEOUT_S + 0.05):
            with self.subTest(gap=gap):
                panel = self.driving_panel()
                with patch.object(manual.time, "monotonic", return_value=panel.last_status + gap):
                    panel.tick()
                self.assertFalse(panel.active)
                self.assertFalse(panel.motion)
                panel.port.write.assert_called_once_with(wire.frame(wire.STOP))

    def test_timely_status_keeps_current_motion(self):
        panel = self.driving_panel()
        with patch.object(manual.time, "monotonic", return_value=panel.last_status + manual.STATUS_TIMEOUT_S - 0.001):
            panel.tick()
        self.assertTrue(panel.active)
        panel.port.write.assert_called_once_with(wire.frame(wire.VELOCITY, struct.pack("<Bhhh", 1, 100, 0, 0)))


if __name__ == "__main__":
    unittest.main()
