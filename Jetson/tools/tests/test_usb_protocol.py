# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""Offline wire contract; no serial device, ROS installation or Jetson required."""
import copy
import io
import importlib.util
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from source.usb_codec import decode, encode
from validation.evidence import ContractFailure
from validation.observations import UsbProbe
from validation import workflow


class CodecContract(unittest.TestCase):
    def test_removed_type_is_unknown(self):
        for frame in ('T01#' + '00' * 8, 'T01#0100' + '00' * 6):
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                decode(frame)
        for name in ('heartbeat', 'mission_loss'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                encode(name, [])

    def test_action_requests_exact_bytes(self):
        for number, name in enumerate(('startup', 'grasp', 'release', 'reject_open')):
            expected = 'T21#' + struct.pack('<BI', number, 0xa1b2c3d4).hex().upper()
            for kind in (str(number), name):
                with self.subTest(kind=kind):
                    self.assertEqual(encode('arrival_point', [kind, '2712847316']), expected)
            decoded = decode(expected)
            self.assertEqual(decoded['action_kind'], number)
            self.assertEqual(decoded['request_id'], 0xa1b2c3d4)

    def test_action_results_exact_bytes(self):
        for kind in range(4):
            for status in range(2):
                for request_id in (1, 0xffffffff):
                    with self.subTest(kind=kind, status=status, request_id=request_id):
                        expected = 'T09#' + struct.pack('<BBI', kind, status, request_id).hex().upper()
                        self.assertEqual(encode('action_result', list(map(str, (kind, status, request_id)))), expected)
                        decoded = decode(expected)
                        self.assertEqual(decoded['action_kind'], kind)
                        self.assertEqual(decoded['success'], bool(status))
                        self.assertEqual(decoded['request_id'], request_id)

    def test_invalid_action_requests_rejected(self):
        cases = (['1'], ['4', '1'], ['-1', '1'], ['grab', '1'],
                 ['1', '0'], ['1', '-1'], ['1', '4294967296'])
        for values in cases:
            with self.subTest(values=values), self.assertRaises(ValueError):
                encode('arrival_point', values)
        with self.assertRaises(ValueError):
            encode('arrival_point', ['1', '1'], invalid=True)
        for payload in ('01', '0401000000', '0100000000'):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                decode('T21#' + payload)

    def test_invalid_action_results_rejected(self):
        for values in (['1', '1'], ['4', '1', '1'], ['1', '2', '1'],
                       ['1', '-1', '1'], ['1', '1', '0'], ['1', '1', '4294967296']):
            with self.subTest(values=values), self.assertRaises(ValueError):
                encode('action_result', values)
        with self.assertRaises(ValueError):
            encode('action_result', ['1', '0', '1'], invalid=True)
        for payload in ('0101010000', '040101000000', '010201000000', '010100000000'):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                decode('T09#' + payload)

    def test_command_five_echoes_nonzero_request_id(self):
        for invalid in (False, True):
            expected = ('T07#0005' if invalid else 'T07#0105') + 'D4C3B2A1'
            self.assertEqual(encode('command_reply', ['arrival_point', '2712847316'], invalid), expected)
            self.assertEqual(decode(expected)['token'], 0xa1b2c3d4)
        with self.assertRaises(ValueError):
            encode('command_reply', ['arrival_point', '0'])
        with self.assertRaises(ValueError):
            decode('T07#010500000000')

    def test_start_search_request_and_reply_echo_identity(self):
        for request_id in (1, 0xa1b2c3d4, 0xffffffff):
            token = struct.pack('<I', request_id).hex().upper()
            self.assertEqual(encode('start_search', [str(request_id)]), 'T19#01' + token)
            self.assertEqual(decode('T19#01' + token)['request_id'], request_id)
            for accepted in (0, 1):
                reply = 'T07#' + f'{accepted:02X}' + '03' + token
                self.assertEqual(encode('command_reply', ['start_search', str(request_id)], not accepted), reply)
                self.assertEqual(decode(reply)['token'], request_id)
        for frame in ('T19#01', 'T19#0100000000', 'T19#0001000000', 'T07#010300000000'):
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                decode(frame)
        for values in ([], ['0'], ['4294967296']):
            with self.subTest(values=values), self.assertRaises(ValueError):
                encode('start_search', values)

    def test_manual_and_coordinate_frames_are_preserved(self):
        self.assertEqual(encode('manual', ['1', '-1', '0.5']), 'T02#01E80318FCF401')
        self.assertEqual(encode('manual', ['1', '2', '3'], invalid=True), 'T02#' + '00' * 7)
        self.assertEqual(encode('target', ['0.5', '0.25', '1193046']), 'T03#018813C409563412')
        self.assertEqual(decode('T04#0000000000000000')['valid'], False)


class ProbeContract(unittest.TestCase):
    def test_search_retries_keep_identity_and_next_search_gets_new_id(self):
        probe = UsbProbe.__new__(UsbProbe)
        probe.action_sequence = 0
        sent = []
        probe._send = lambda kind, values: sent.append((kind, values)) or 10.0
        first, _ = probe.begin_search()
        probe.command(19, first)
        second, _ = probe.begin_search()
        self.assertEqual((first, second), (1, 2))
        self.assertEqual(sent, [('start_search', ['1']), ('start_search', ['1']), ('start_search', ['2'])])

    def test_action_request_uses_real_stream_framing_and_unique_ids(self):
        path = Path(__file__).resolve().parents[2] / 'src/teleop_pkg/manual/frame.py'
        spec = importlib.util.spec_from_file_location('test_usb_frame', path)
        frame = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(frame)
        probe = UsbProbe.__new__(UsbProbe)
        probe.frame, probe.master, probe.raw = frame, -1, io.StringIO()
        probe.action_sequence = 0
        packets = []
        with patch('validation.observations.os.write', side_effect=lambda _, data: packets.append(data) or len(data)):
            first, _ = probe.begin_action('grasp')
            probe.action_request('grasp', first)
            second, _ = probe.begin_action('release')
        self.assertEqual((first, second), (1, 2))
        self.assertEqual(frame.Parser().feed(b''.join(packets)), [
            (21, bytes.fromhex('0101000000')), (21, bytes.fromhex('0101000000')),
            (21, bytes.fromhex('0202000000'))])

    def test_periodic_evidence_requires_only_coordinate_types(self):
        probe = UsbProbe.__new__(UsbProbe)
        probe.poll = lambda: None
        probe.records = [{'steady': 1, 'id': kind, 'dlc': 8, 'data': '00' * 8} for kind in (3, 4)]
        observer = SimpleNamespace(records={'mission': [], 'target': [], 'safety': []},
                                   snapshot={'values': {}})
        self.assertEqual(set(probe.steady_periodic(observer, 'seg', 0)), {'03', '04'})
        probe.records.append({'steady': 1, 'id': 1, 'dlc': 8, 'data': '00' * 8})
        with self.assertRaises(ContractFailure):
            probe.steady_periodic(observer, 'seg', 0)


class ActionWaitContract(unittest.TestCase):
    def run_action(self, replies):
        """Drive the real wait predicate without ROS, time delays or a device."""
        class Probe:
            def __init__(self):
                self.current, self.sent, self.clock, self.calls = set(), [], 10.0, []

            def begin_action(self, kind):
                self.action_request(kind, 23)
                return 23, self.clock

            def action_request(self, kind, request_id):
                self.sent.append((kind, request_id))
                return self.clock

            def has_reply(self, identifier, payload, since):
                self.calls.append((identifier, payload, since))
                return (identifier, payload) in self.current

        probe = Probe()
        observed = []

        def wait(predicate, **_):
            for messages in replies:
                probe.clock += 0.21
                probe.current.update(messages)
                completed = predicate()
                observed.append(completed)
                if completed:
                    return
            raise ContractFailure('timeout: no terminal action result')

        with patch('validation.workflow.time.monotonic', side_effect=lambda: probe.clock):
            result = workflow.gripper_action(SimpleNamespace(wait=wait), probe, None, 'grasp')
        return result, probe, observed

    def test_ack_and_stale_terminal_do_not_complete_action(self):
        ack = (7, '010517000000')
        stale = (9, '010116000000')
        wrong_kind = (9, '020117000000')
        terminal = (9, '010117000000')
        result, probe, observed = self.run_action([[ack], [stale, wrong_kind], [terminal]])
        self.assertEqual(observed, [False, False, True])
        self.assertEqual(result, {'action_kind': 1, 'request_id': 23, 'accepted': True, 'success': True})
        self.assertGreaterEqual(len(probe.sent), 2)
        self.assertEqual(set(probe.sent), {(1, 23)})
        self.assertTrue(all(since == 10.0 for _, _, since in probe.calls))

    def test_ack_only_times_out(self):
        with self.assertRaisesRegex(ContractFailure, 'terminal'):
            self.run_action([[(7, '010517000000')]] * 3)

    def test_failed_terminal_stops_the_chain(self):
        with self.assertRaisesRegex(ContractFailure, 'failed'):
            self.run_action([[(7, '010517000000'), (9, '010017000000')]])

    def test_request_rejection_stops_the_chain(self):
        with self.assertRaisesRegex(ContractFailure, 'rejected'):
            self.run_action([[(7, '000517000000')]])


class CaptureEvidenceContract(unittest.TestCase):
    def rejected_case(self):
        def header(second):
            return {'frame_id': 'camera', 'stamp': {'sec': second, 'nanosec': 0}}

        pairs = []
        for second in range(1, 6):
            row = {'source': [second * 1000000000, 'camera']}
            for name in ('detections', 'tracks', 'selected', 'target'):
                row[name] = {'header': header(second), 'detections': []}
            for name in ('processed', 'masks'):
                row[name] = {'header': header(second), 'width': 1, 'height': 1, 'step': 3,
                             'encoding': 'bgr8', 'is_bigendian': 0, 'sha256': '0' * 64}
            pairs.append(row)
        roi = dict(header=header(5), request_id=55, valid=False, passed=False, reason=1,
                   roi_x=0, roi_y=0, roi_width=0, roi_height=0, green_count=0,
                   blue_count=0, black_count=0, orange_count=0, other_count=0)
        direct = dict(roi, request_id=54)
        return {'scenario': 'chain', 'http': {'frames': 3, 'snapshot': {
            'bytes': 3, 'width': 1, 'height': 1, 'sha256': '0' * 64}}, 'result': {
            'seg': pairs, 'task_states': [{'task': 'seg', 'epoch': 1,
                                          'source': [1000000000, 'camera'], 'header': header(1)}],
            'mission_states': [{'roi_request_id': 55, 'cycle_phase': 0, 'cycle_generation': 2,
                                'transport_phase': 0}], 'roi_diagnostics': [roi],
            'startup_action': {'action_kind': 0, 'request_id': 1, 'accepted': True, 'success': True},
            '/target_node/check_roi': direct, '/mission_node/roi_result': roi,
            'mission_roi_wire': '000000',
            'seg_can': {key: {'valid_bytes': [], 'ros_expected_bytes': [], 'invalid_only': True}
                        for key in ('03', '04')},
            'handshake': {'field_executed': False, 'coordinates_paused': True,
                          'phase_after_roi': 7, 'reason': 'ROI rejected',
                          'grasp': {'action_kind': 1, 'request_id': 2, 'accepted': True, 'success': True},
                          'reject_open': {'action_kind': 3, 'request_id': 3, 'accepted': True, 'success': True},
                          'restart': {'command': 3, 'phase': 0, 'cycle_generation': 2}}}}

    def test_current_capture_requires_terminal_action_evidence(self):
        case = self.rejected_case()
        workflow.result_evidence(case)
        for label in ('startup_missing', 'grasp_missing', 'reopen_missing', 'ack_only', 'reused_id', 'old_periodic'):
            changed = copy.deepcopy(case)
            result = changed['result']
            if label == 'startup_missing':
                del result['startup_action']
            elif label == 'grasp_missing':
                del result['handshake']['grasp']
            elif label == 'reopen_missing':
                del result['handshake']['reject_open']
            elif label == 'ack_only':
                del result['handshake']['reject_open']['success']
            elif label == 'reused_id':
                result['handshake']['reject_open']['request_id'] = 1
            else:
                result['seg_can']['01'] = copy.deepcopy(result['seg_can']['03'])
            with self.subTest(label=label), self.assertRaises(ContractFailure):
                workflow.result_evidence(changed)

    def test_action_ids_are_not_roi_ids_when_comparing_captures(self):
        first = self.rejected_case()
        second = copy.deepcopy(first)
        second['result']['startup_action']['request_id'] = 54
        second['result']['handshake']['grasp']['request_id'] = 56
        second['result']['handshake']['reject_open']['request_id'] = 55
        workflow.result_evidence(second)
        self.assertEqual(workflow.semantic_result(first), workflow.semantic_result(second))

    def test_successful_capture_checks_post_delivery_localization(self):
        case = self.rejected_case()
        result = case['result']
        roi = result['/mission_node/roi_result']
        roi.update(valid=True, passed=True, green_count=1)
        result['mission_roi_wire'] = '010101'
        result['mission_states'][-1].update(cycle_generation=3, transport_phase=1)
        handshake = result['handshake']
        del handshake['reject_open']
        handshake.update(
            field_executed=True, phase_after_roi=2,
            turn=dict(command=1, phase=4, cycle_generation=2, roi_request_id=55),
            safety_arrival=dict(command=2, phase=5, cycle_generation=2, roi_request_id=55),
            delivery=dict(command=4, phase=9, cycle_generation=2, roi_request_id=55),
            restart=dict(command=3, phase=0, cycle_generation=3),
            release=dict(action_kind=2, request_id=3, accepted=True, success=True),
        )
        result['field_can'] = copy.deepcopy(result['seg_can'])
        field_frames = []
        for row in result['seg']:
            header = row['processed']['header']
            field = dict(header=header, valid=True, committed=True, corners=[], left_region=[],
                         right_region=[], split_line=[], split_x=0)
            safety = dict(header=header, valid=True, side=1, x=0.5, y=0.5)
            field_frames.append(dict(source=row['source'], field=field, safety=safety))
        result.update(field=field_frames, field_safety_pairs=field_frames, field_phase=dict(
            task='field', epoch=2, stage_header=field_frames[0]['field']['header'],
            source_frames=5, valid_geometry_frames=5, first_source=field_frames[0]['source'],
            last_source=field_frames[-1]['source']))
        workflow.result_evidence(case)
        handshake['delivery']['phase'] = 5
        with self.assertRaisesRegex(ContractFailure, 'delivery'):
            workflow.result_evidence(case)


class ChainSequenceContract(unittest.TestCase):
    def test_grasp_finishes_before_arrival_in_both_roi_branches(self):
        for passed in (False, True):
            with self.subTest(passed=passed):
                events = []
                roi = {'valid': True, 'passed': passed}
                state = {'cycle_phase': 2 if passed else 7, 'destination': 1 if passed else 0,
                         'roi_request_id': 77}
                future = SimpleNamespace(done=lambda: True, exception=lambda: None,
                                         result=lambda: SimpleNamespace(result=roi))
                client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda _: future)
                observer = SimpleNamespace(
                    records={'roi': [], 'mission': [{'value': state}]}, states=[{'task': 'field'}],
                    node=SimpleNamespace(create_client=lambda *_: client, destroy_client=lambda _: None))

                def wait(predicate, **_):
                    self.assertTrue(predicate())

                def command(identifier):
                    events.append(('command', identifier))
                    observer.records['roi'].append({'value': roi})
                    return 10.0

                def has_reply(identifier, *_):
                    events.append(('reply', identifier))
                    return True

                def action(_observer, _can, _owner, kind):
                    events.append(('action', kind))
                    return {'action_kind': {'grasp': 1, 'reject_open': 3}[kind],
                            'request_id': len(events), 'accepted': True, 'success': True}

                observer.wait = wait
                can = SimpleNamespace(command=command, has_reply=has_reply, require_paused=lambda *_: None)
                service = SimpleNamespace(CheckRoi=SimpleNamespace(Request=lambda: object()))
                result = {}
                with patch.dict(sys.modules, {'interface_pkg.srv': service}), \
                        patch('validation.observations.value_record', side_effect=lambda value: value), \
                        patch('validation.workflow.gripper_action', side_effect=action), \
                        patch('validation.workflow.mission_command', return_value={}):
                    workflow.chain_arrival(observer, can, None, result)
                self.assertLess(events.index(('action', 'grasp')), events.index(('command', 16)))
                self.assertEqual(result['handshake']['grasp']['action_kind'], 1)
                if not passed:
                    self.assertLess(events.index(('reply', 6)), events.index(('action', 'reject_open')))


if __name__ == '__main__':
    unittest.main()
