"""手动 S288 协议和转角会话。不打开真实串口，也不接触自动搬运。"""

import math
import struct
import threading
import unittest
from unittest.mock import patch

from servo.s288 import (
    FEEDBACK_BYTES, FEEDBACK_MARK, MODE_RUN, MODE_STOP, OpenHold, RATIO,
    ServoBusy, ServoLink, ServoSession, _take_feedback,
    check_goto, check_move, control_packet, crc32, output_degrees)


class ProtocolTest(unittest.TestCase):
    def test_vendor_vector(self):
        packet = control_packet(2, 1, 1, 0.1, 0.2, 0.3, 1.0, 0.1)
        self.assertEqual(packet.hex(), 'feee9200590017004ae206000f009a005e72f7ee')

    def test_move_limits(self):
        check_move(30, 2)
        for degrees, seconds in ((0, 1), (361, 1), (10, 0.1), (10, 11), (float('nan'), 1)):
            with self.assertRaises(ValueError):
                check_move(degrees, seconds)
        with self.assertRaisesRegex(ValueError, '角度须在 -360 到 360 度之间，且不能为 0'):
            check_move(361, 1)
        with self.assertRaisesRegex(ValueError, '时长须在 0.2 到 10 秒之间'):
            check_move(10, 0.1)

    def test_take_feedback_one_attempt(self):
        body = struct.pack('<BbBBhhiIHBB', 0x02, 1, 2, 24, 0, 0, 0, 0, 0, 0, 0)
        packet = FEEDBACK_MARK + body + struct.pack('<I', crc32(body))
        pending = bytearray(b'\x00' + packet)
        feedback = _take_feedback(pending, 2)
        self.assertEqual(feedback['id'], 2)
        # 成功帧留在缓冲区里；调用方拿到结果后就不再读这块缓冲。
        self.assertEqual(bytes(pending), packet)
        corrupt = bytearray(packet)
        corrupt[8] ^= 1
        self.assertIsNone(_take_feedback(corrupt, 2))
        self.assertEqual(len(corrupt), FEEDBACK_BYTES - 1)
        noise = bytearray(b'\x11\x22')
        self.assertIsNone(_take_feedback(noise, 2))
        self.assertEqual(bytes(noise), b'\x22')


class FakeLink:
    def __init__(self):
        self.calls = []
        self.closed = False

    def move_degrees(self, degrees, seconds, kp, kd, cancel):
        self.calls.append(('move', degrees, seconds, kp, kd, cancel.is_set()))
        return {'position': 0.5, 'turned_deg': degrees}

    def goto_degrees(self, target_deg, seconds, kp, kd, cancel, zero_rad, *, ramp_fraction=0.0):
        self.calls.append(('goto', target_deg, seconds, kp, kd, cancel.is_set(), zero_rad, ramp_fraction))
        return {'position': zero_rad + math.radians(target_deg), 'turned_deg': target_deg,
                'target_deg': target_deg}

    def stop(self):
        self.calls.append(('stop',))
        return {'position': 0.25}

    def close(self):
        self.closed = True


class SessionTest(unittest.TestCase):
    def test_faulted_startup_stop_cannot_establish_absolute_zero(self):
        for error, timeout in ((1, False), (0, True)):
            with self.subTest(error=error, timeout=timeout), patch('servo.s288.ServoLink') as link:
                link.return_value.stop.return_value = {
                    'position': 0.25, 'error': error, 'timeout': timeout}
                session = ServoSession('/dev/mock', 0, 1.0, 0.05, open_deg=40)
                self.assertIsNone(session.position_deg)
                session.start_open(0.2)
                with self.assertRaisesRegex(RuntimeError, '尚未记录上电零点'):
                    session.wait(timeout=2)
                link.return_value.goto_degrees.assert_not_called()
                session.close()

    def test_without_device_rejects_motion(self):
        session = ServoSession('', 0, 1.0, 0.05)
        self.assertFalse(session.available)
        with self.assertRaises(RuntimeError):
            session.move(10, 1)
        self.assertIsNone(session.stop())

    def test_move_updates_position_and_does_not_touch_can(self):
        session = ServoSession('', 0, 1.0, 0.05)
        session._link = FakeLink()
        feedback = session.move(-15, 0.2)
        self.assertEqual(feedback['turned_deg'], -15)
        self.assertFalse(session.busy)
        self.assertAlmostEqual(session.snapshot()['position_deg'], math.degrees(0.5))
        self.assertEqual(session._link.calls[0][0], 'move')

    def test_second_move_rejected_while_busy(self):
        session = ServoSession('', 0, 1.0, 0.05)
        started = threading.Event()
        release = threading.Event()

        class BlockingLink(FakeLink):
            def move_degrees(self, degrees, seconds, kp, kd, cancel):
                started.set()
                release.wait(timeout=2)
                return super().move_degrees(degrees, seconds, kp, kd, cancel)

        session._link = BlockingLink()
        worker = threading.Thread(target=session.move, args=(20, 1))
        worker.start()
        self.assertTrue(started.wait(timeout=1))
        with self.assertRaises(RuntimeError):
            session.move(5, 1)
        release.set()
        worker.join(timeout=2)
        self.assertFalse(worker.is_alive())

    def test_goto_uses_power_on_zero(self):
        session = ServoSession('', 0, 1.0, 0.05)
        session._link = FakeLink()
        session._zero_rad = 0.1
        session.start_goto(-15, 0.2)
        feedback = session.wait(timeout=2)
        self.assertAlmostEqual(feedback['target_deg'], -15)
        self.assertAlmostEqual(feedback['position'], 0.1 + math.radians(-15))
        self.assertEqual(session._link.calls[0][0], 'goto')
        self.assertAlmostEqual(session._link.calls[0][6], 0.1)

    def test_start_move_is_async_and_rejects_when_busy(self):
        session = ServoSession('', 0, 1.0, 0.05)
        started = threading.Event()
        release = threading.Event()

        class BlockingLink(FakeLink):
            def move_degrees(self, degrees, seconds, kp, kd, cancel):
                started.set()
                release.wait(timeout=2)
                return super().move_degrees(degrees, seconds, kp, kd, cancel)

        session._link = BlockingLink()
        session.start_move(20, 1)
        self.assertTrue(started.wait(timeout=1))
        self.assertTrue(session.busy)
        with self.assertRaises(ServoBusy):
            session.start_move(5, 1)
        release.set()
        session.wait(timeout=2)
        self.assertFalse(session.busy)

    def test_open_close_use_required_seconds(self):
        session = ServoSession('', 0, 1.0, 0.05, closed_deg=-40, open_deg=40, ramp_fraction=0.8)
        session._link = FakeLink()
        session._zero_rad = 0.0
        session.start_open(0.2)
        feedback = session.wait(timeout=2)
        self.assertEqual(feedback['target_deg'], 40)
        session.start_close(0.2)
        feedback = session.wait(timeout=2)
        self.assertEqual(feedback['target_deg'], -40)
        self.assertEqual([call[-1] for call in session._link.calls], [0.8, 0.8])
        session.start_goto(10, 0.2)
        session.wait(timeout=2)
        self.assertEqual(session._link.calls[-1][-1], 0.0)

    def test_holding_releases_the_serial_worker_before_new_motion(self):
        class HoldLink(FakeLink):
            def goto_degrees(self, target_deg, seconds, kp, kd, cancel, zero_rad, *, open_hold, **options):
                self.calls.append(('hold',))
                feedback = {'position': 0.1, 'output_encoder': 4, 'target_deg': 0.0,
                            'output_error_deg': -0.4}
                open_hold.report(feedback)
                if not cancel.wait(2):
                    raise RuntimeError('保位未取消')
                self.calls.append(('hold_stop',))
                return feedback

            def move_degrees(self, degrees, seconds, kp, kd, cancel):
                result = super().move_degrees(degrees, seconds, kp, kd, cancel)
                return {**result, 'output_encoder': 5}

            def stop(self):
                return {**super().stop(), 'output_encoder': 4}

        session = ServoSession('', 0, 1.0, 0.05, closed_deg=-50, open_deg=0,
                               open_hold_max_nm=0.25)
        self.addCleanup(session.close)
        session._link = HoldLink()
        session._zero_rad = 0.1
        session._output_zero = 8188
        session.start_open(0.2)
        session.wait(timeout=1)
        self.assertFalse(session.busy)
        self.assertTrue(session.snapshot()['holding'])
        self.assertAlmostEqual(session.snapshot()['output_position_deg'], 8 * 360 / 8192)
        session.start_move(2, 0.2)
        session.wait(timeout=1)
        self.assertEqual([call[0] for call in session._link.calls], ['hold', 'hold_stop', 'move'])
        session.start_open(0.2)
        session.wait(timeout=1)
        session.stop()
        session._worker.join(timeout=1)
        self.assertFalse(session._worker.is_alive())
        self.assertFalse(session.snapshot()['holding'])
        session.start_open(0.2)
        session.wait(timeout=1)
        link = session._link
        session.close()
        self.assertTrue(link.closed)
        self.assertEqual([call[0] for call in link.calls[-3:]], ['hold', 'hold_stop', 'stop'])


class LinkTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.cancel_at = math.inf
        self.exchange_delay = 0.0
        self.follow_motor = True
        self.encoder = lambda at: 8190
        self.speed = lambda at: 0.0
        self.warning_at = math.inf
        self.timeout_at = math.inf
        self.previous_stop_frames = 0
        self.position = 0.4
        self.commands = []
        self.link = ServoLink.__new__(ServoLink)
        self.link.motor_id = 0
        self.link.exchange = self.exchange
        clock = patch('servo.s288.time.monotonic', lambda: self.now)
        legacy_sleep = patch('servo.s288.time.sleep', self.wait)
        clock.start()
        legacy_sleep.start()
        self.addCleanup(clock.stop)
        self.addCleanup(legacy_sleep.stop)

    def is_set(self):
        return self.now >= self.cancel_at

    def wait(self, seconds):
        self.now = min(self.now + seconds, self.cancel_at)

    def exchange(self, packet):
        mode = (packet[2] >> 4) & 7
        position = self.position
        if mode == MODE_RUN:
            self.assertEqual(struct.unpack_from('<hh', packet, 4), (0, 0))
            raw_position, = struct.unpack_from('<i', packet, 8)
            position = raw_position * (2 * math.pi) / (RATIO * 32768)
            if self.follow_motor:
                self.position = position
        self.commands.append((self.now, mode, position))
        if mode == MODE_RUN:
            self.now += self.exchange_delay
            if self.now >= self.timeout_at:
                raise TimeoutError('未收到有效的舵机反馈')
        feedback_mode = mode
        if mode == MODE_RUN and self.previous_stop_frames > 0:
            feedback_mode = MODE_STOP
            self.previous_stop_frames -= 1
        return {'mode': feedback_mode, 'position': self.position, 'error': 0, 'timeout': False,
                'output_encoder': self.encoder(self.now), 'speed': self.speed(self.now),
                'warning': int(mode == MODE_RUN and self.now >= self.warning_at)}

    def test_ramp_uses_current_position_and_holds_exact_target(self):
        self.exchange_delay = 0.003
        target = 0.1 - math.radians(50)
        self.link.goto_degrees(-50, 0.2, 1.0, 0.05, self, 0.1, ramp_fraction=0.8)
        runs = [(at, position) for at, mode, position in self.commands if mode == MODE_RUN]
        self.assertEqual(self.commands[0], (0.0, MODE_STOP, 0.4))
        self.assertAlmostEqual(runs[0][1], 0.4, delta=4e-7)
        self.assertAlmostEqual(runs[4][1], (0.4 + target) / 2.0, delta=4e-7)
        self.assertTrue(all(left[1] >= right[1] for left, right in zip(runs, runs[1:])))
        for at, position in runs:
            if at >= 0.16:
                self.assertAlmostEqual(position, target, delta=4e-7)
        self.assertEqual(self.commands[-1][:2], (0.2, MODE_STOP))

    def test_shortest_full_ramp_sends_final_target_before_stop(self):
        feedback = self.link.goto_degrees(0, 0.2, 1.0, 0.05, self, 0.1, ramp_fraction=1.0)
        at, mode, position = self.commands[-2]
        self.assertLess(at, 0.2)
        self.assertEqual(mode, MODE_RUN)
        self.assertAlmostEqual(position, 0.1, delta=4e-7)
        self.assertEqual(self.commands[-1][:2], (0.2, MODE_STOP))
        self.assertAlmostEqual(feedback['position'], 0.1, delta=4e-7)

    def test_cancel_stops_without_sending_target(self):
        self.cancel_at = 0.175
        self.link.goto_degrees(0, 0.2, 1.0, 0.05, self, 0.1, ramp_fraction=1.0)
        self.assertEqual(self.commands[-1][:2], (self.cancel_at, MODE_STOP))
        self.assertTrue(all(position > 0.1 for _, _, position in self.commands))

    def test_slow_exchange_missing_final_frame_stops_and_fails(self):
        self.exchange_delay = 0.21
        with self.assertRaisesRegex(RuntimeError, '未发送最终目标角'):
            self.link.goto_degrees(0, 0.2, 1.0, 0.05, self, 0.1, ramp_fraction=1.0)
        self.assertEqual([mode for _, mode, _ in self.commands], [MODE_STOP, MODE_RUN, MODE_STOP])
        self.assertAlmostEqual(self.commands[-1][2], 0.4, delta=4e-7)

    def test_jog_and_old_goto_send_target_in_first_run_packet(self):
        self.link.move_degrees(20, 0.2, 1.0, 0.05, self)
        self.assertEqual(self.commands[1][1], MODE_RUN)
        self.assertAlmostEqual(self.commands[1][2], 0.4 + math.radians(20), delta=4e-7)
        self.commands.clear()
        self.link.goto_degrees(0, 0.2, 1.0, 0.05, self, 0.1)
        self.assertEqual(self.commands[1][1], MODE_RUN)
        self.assertAlmostEqual(self.commands[1][2], 0.1, delta=4e-7)

    def test_open_holds_after_three_fresh_stable_samples_and_wraps_encoder(self):
        self.encoder = lambda at: 2
        self.speed = lambda at: 0.13 if at < 1.0 else 0.0
        self.cancel_at = 1.13
        accepted = []
        hold = OpenHold(8190, 0.0, 0.25, 1.0, lambda feedback: accepted.append((self.now, feedback)))
        self.link.goto_degrees(0, 1, 1.0, 0.05, self, 0.1, ramp_fraction=0.8,
                               trim_deg=5.0, open_hold=hold)
        self.assertTrue(accepted)
        self.assertGreaterEqual(accepted[0][0], 1.02)
        self.assertAlmostEqual(accepted[0][1]['output_error_deg'], -4 * 360 / 8192)
        self.assertAlmostEqual(output_degrees(8190, 2), -4 * 360 / 8192)
        self.assertEqual([mode for _, mode, _ in self.commands].count(MODE_STOP), 2)
        self.assertEqual(self.commands[-1][:2], (self.cancel_at, MODE_STOP))

    def test_slow_open_extends_only_while_output_approaches_target(self):
        # 独立轴越过零点，在原来的 1.5 s 截止后才到位。
        self.encoder = lambda at: round(8190 - max(0.0, 30.0 - 16.0 * at) * 8192 / 360) % 8192
        self.cancel_at = 2.1
        accepted = []
        hold = OpenHold(8190, 0.0, 0.14, 2.0, lambda _: accepted.append(self.now))
        self.link.goto_degrees(0, 1, 2.0, 0.1, self, 0.1, ramp_fraction=0.8, open_hold=hold)
        self.assertTrue(accepted)
        self.assertGreater(accepted[0], 1.5)
        self.assertEqual(self.commands[-1][:2], (self.cancel_at, MODE_STOP))

    def test_moving_output_inside_window_cannot_confirm_open(self):
        self.encoder = lambda at: (8190 + (18 if round(at / 0.02) % 2 else -18)) % 8192
        hold = OpenHold(8190, 0.0, 0.14, 2.0, lambda _: self.fail('输出轴往返时误报张开'))
        with self.assertRaisesRegex(RuntimeError, '捕获超时'):
            self.link.goto_degrees(0, 1, 2.0, 0.1, self, 0.1, ramp_fraction=0.8, open_hold=hold)
        self.assertEqual(self.commands[-1][:2], (1.5, MODE_STOP))

    def test_isolated_improvement_does_not_extend_a_stalled_open(self):
        self.encoder = lambda at: 8150 if 1.39 <= at < 1.41 else 8000
        hold = OpenHold(8190, 0.0, 0.14, 1.0, lambda _: self.fail('夹爪卡住时误报张开'))
        with self.assertRaisesRegex(RuntimeError, '捕获超时'):
            self.link.goto_degrees(0, 1, 2.0, 0.1, self, 0.1, ramp_fraction=0.8, open_hold=hold)
        self.assertEqual(self.commands[-1][:2], (1.5, MODE_STOP))

    def test_already_open_holds_starting_motor_angle(self):
        self.cancel_at = 0.25
        accepted = []
        hold = OpenHold(8190, 0.0, 0.14, 1.0, accepted.append)
        self.link.goto_degrees(0, 0.2, 2.0, 0.1, self, 0.1,
                               ramp_fraction=0.8, trim_deg=5, open_hold=hold)
        self.assertTrue(accepted)
        for _, mode, position in self.commands:
            if mode == MODE_RUN:
                self.assertAlmostEqual(position, 0.4, delta=4e-7)

    def test_capture_timeout_stops_and_respects_position_torque_limit(self):
        self.follow_motor = False
        self.encoder = lambda at: 8000
        hold = OpenHold(8190, 0.0, 0.02, 1.0, lambda _: self.fail('未到位即报告保位'))
        with self.assertRaisesRegex(RuntimeError, '捕获超时'):
            self.link.goto_degrees(0, 1, 1.0, 0.05, self, 0.1, ramp_fraction=0.8,
                                   trim_deg=5.0, open_hold=hold)
        held_positions = [position for at, mode, position in self.commands if at >= 0.8 and mode == MODE_RUN]
        self.assertTrue(held_positions)
        self.assertTrue(all(abs(position - 0.4) <= 0.0200004 for position in held_positions))
        self.assertEqual(self.commands[-1][:2], (1.5, MODE_STOP))

    def test_maximum_full_ramp_uses_the_capture_grace(self):
        self.cancel_at = 10.10
        self.encoder = lambda at: 8100 if at < 10.0 else 8190
        accepted = []
        hold = OpenHold(8190, 0.0, 0.25, 1.0, lambda _: accepted.append(self.now))
        self.link.goto_degrees(0, 10, 1.0, 0.05, self, 0.1, ramp_fraction=1.0, open_hold=hold)
        self.assertGreater(accepted[0], 10.0)
        self.assertLess(accepted[0], self.cancel_at)
        self.assertEqual(self.commands[-1][:2], (10.10, MODE_STOP))

        self.now = 0.0
        self.commands.clear()
        self.cancel_at = math.inf
        accepted.clear()
        self.encoder = lambda at: 8100
        with self.assertRaisesRegex(RuntimeError, '捕获超时'):
            self.link.goto_degrees(0, 10, 1.0, 0.05, self, 0.1, ramp_fraction=0.8, open_hold=hold)
        self.assertEqual(accepted, [])
        self.assertEqual(self.commands[-1][:2], (10.5, MODE_STOP))

    def test_warning_stops_active_hold(self):
        self.warning_at = 1.1
        accepted = []
        hold = OpenHold(8190, 0.0, 0.25, 1.0, accepted.append)
        with self.assertRaisesRegex(RuntimeError, '告警 1'):
            self.link.goto_degrees(0, 1, 1.0, 0.05, self, 0.1, ramp_fraction=0.8, open_hold=hold)
        self.assertTrue(accepted)
        self.assertEqual(self.commands[-1][1], MODE_STOP)

    def test_only_the_first_run_can_return_previous_stop(self):
        self.previous_stop_frames = 1
        self.link.goto_degrees(-50, 0.2, 1.0, 0.05, self, 0.1, check_warning=True)
        self.previous_stop_frames = 2
        self.commands.clear()
        with self.assertRaisesRegex(RuntimeError, '未收到舵机运行模式反馈'):
            self.link.goto_degrees(-50, 0.2, 1.0, 0.05, self, 0.1, check_warning=True)
        self.assertEqual([mode for _, mode, _ in self.commands],
                         [MODE_STOP, MODE_RUN, MODE_RUN, MODE_STOP])
        self.previous_stop_frames = 1
        self.exchange_delay = 0.21
        with self.assertRaisesRegex(RuntimeError, '未收到舵机运行模式反馈'):
            self.link.goto_degrees(-50, 0.2, 1.0, 0.05, self, 0.1, check_warning=True)

    def test_missing_serial_feedback_stops_active_hold(self):
        self.timeout_at = 1.10
        hold = OpenHold(8190, 0.0, 0.25, 1.0, lambda _: None)
        with self.assertRaisesRegex(TimeoutError, '未收到有效的舵机反馈'):
            self.link.goto_degrees(0, 1, 1.0, 0.05, self, 0.1, ramp_fraction=0.8, open_hold=hold)
        self.assertEqual(self.commands[-1][1], MODE_STOP)

    def test_persistent_output_drift_stops_active_hold(self):
        self.encoder = lambda at: 8190 if at < 1.10 else 8100
        accepted = []
        hold = OpenHold(8190, 0.0, 0.25, 1.0, accepted.append)
        with self.assertRaisesRegex(RuntimeError, '持续超差'):
            self.link.goto_degrees(0, 1, 1.0, 0.05, self, 0.1, ramp_fraction=0.8, open_hold=hold)
        self.assertTrue(accepted)
        self.assertEqual(self.commands[-1][1], MODE_STOP)
        self.assertGreaterEqual(self.commands[-1][0], 1.60)
        self.assertLess(self.commands[-1][0], 1.63)

    def test_close_trim_stops_on_time_without_requiring_empty_grip_angle(self):
        self.encoder = lambda at: 8170
        final = self.link.goto_degrees(-50, 1, 1.0, 0.05, self, 0.1, ramp_fraction=0.8,
                                       trim_deg=0.35, check_warning=True)
        self.assertEqual(final['target_deg'], -50)
        self.assertEqual(self.commands[-1][:2], (1.0, MODE_STOP))
        self.assertAlmostEqual(self.commands[-2][2], 0.1 + math.radians(-49.65), delta=4e-7)
        self.assertEqual(final['output_encoder'], 8170)


class GotoCheckTest(unittest.TestCase):
    def test_goto_allows_zero_and_rejects_invalid(self):
        check_goto(0, 1)
        check_goto(-40, 0.2)
        for target, seconds in ((361, 1), (10, 0.1), (float('nan'), 1)):
            with self.assertRaises(ValueError):
                check_goto(target, seconds)


if __name__ == '__main__':
    unittest.main()
