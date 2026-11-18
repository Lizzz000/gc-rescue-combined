"""Host-only safety/transport tests; never connects to a debug probe."""
import contextlib
import importlib.util
import io
import math
from pathlib import Path
import socket
import struct
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace


SCRIPT = Path(__file__).resolve().parents[1] / "tools/live_tuning/swd_session.py"
SPEC = importlib.util.spec_from_file_location("swd_session", SCRIPT)
swd = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(swd)
SYMBOLS = {name: 0x20001000 + index * 256 for index, name in enumerate(swd.SYMBOLS)}
BURST_SYMBOLS = {"tuning_" + name: 0x200074D4 + offset for name, offset in (
    ("request_seq", 0), ("command", 4), ("heartbeat", 8), ("ack_seq", 12),
    ("phase", 16), ("error", 20), ("sample_seq", 24), ("args", 28), ("sample", 60))}


def valid_state():
    return {"phase": 0, "error": 0, "sample": {"phase": 0.0, "error": 0.0, "enabled": 0.0, "yaw_deg": None}}


def fixture_elf():
    segments = [(0x08000000, 0x08000000, bytes(i % 256 for i in range(1103))),
                (0x08002000, 0x20000000, b"RAM-load-data")]
    ident = b"\x7fELF\x01\x01\x01" + bytes(9)
    header = struct.pack("<16sHHIIIIIHHHHHH", ident, 2, 40, 1, 0x08000001,
                         52, 0, 0, 52, 32, len(segments), 0, 0, 0)
    image = bytearray(header + bytes(256 - len(header)))
    for i, (physical, virtual, data) in enumerate(segments):
        offset = len(image)
        struct.pack_into("<IIIIIIII", image, 52 + i * 32, 1, offset, virtual,
                         physical, len(data), len(data), 5, 4)
        image.extend(data)
    return bytes(image), segments


class FlashClient:
    def __init__(self, segments):
        self.memory = {physical + i: value for physical, _, data in segments for i, value in enumerate(data)}
        self.reads, self.writes = [], []
        self.timeout, self.usable = 0.3, True

    def read_words(self, address, count=1):
        self.reads.append((address, count))
        data = bytes(self.memory.get(address + i, 0xFF) for i in range(count * 4))
        return list(struct.unpack("<" + "I" * count, data))

    def write_words(self, address, values):
        self.writes.append((address, values))

    def set_timeout(self, timeout):
        self.timeout = timeout

    def evaluate(self, command):
        return "running"

    def close(self):
        self.usable = False


class Log:
    def __init__(self):
        self.records = []

    def emit(self, event, **data):
        self.records.append((event, data))


class MemoryClient:
    def __init__(self):
        self.memory = {address: 0 for address in SYMBOLS.values()}
        self.writes = []

    def read_words(self, address, count=1):
        return [self.memory[address]]

    def write_words(self, address, words):
        self.writes.append((address, list(words)))
        self.memory[address] = words[0]


class FakeSocket:
    def __init__(self, chunks):
        self.chunks = iter(chunks)
        self.sent = []
        self.closed = False

    def settimeout(self, timeout):
        pass

    def setsockopt(self, *args):
        pass

    def sendall(self, data):
        self.sent.append(data)

    def recv(self, size):
        value = next(self.chunks)
        if isinstance(value, BaseException):
            raise value
        return value

    def close(self):
        self.closed = True


class LiveTuningTests(unittest.TestCase):
    def parse(self, *args):
        return swd.arguments(["--elf", "unused.elf", *args])

    @staticmethod
    def hold_result():
        return {"command": 7, "phase": 4, "error": 0, "sample": {
            "phase": 4, "error": 0, "enabled": 1, "state_1": 1, "state_2": 1, "state_3": 1,
            "target_1": 0, "target_2": 0, "target_3": 0, "odom_valid": 1,
            "end_x_m": -.3, "end_y_m": .02, "end_yaw_rad": .1, "end_valid": 1,
            "end_stopped": 1, "end_elapsed_ms": 2000, "end_progress_m": .3,
            "end_target_distance_m": .3}}

    def test_continuous_goal_arguments_and_limits(self):
        base = ("--command", "7", "--goal-x-m", "-.3", "--goal-y-m", ".02",
                "--goal-yaw-rad", ".1", "--duration-ms", "20000", "--timeout", "23")
        self.assertEqual(swd.command_values(self.parse(*base)), [-.3, .02, .1, .2, .5, 20000, 0, 0])
        for extra in (("--linear-limit", ".201"), ("--angular-limit", ".501"),
                      ("--linear-limit", ".019"), ("--angular-limit", ".099"),
                      ("--goal-yaw-rad", "3.142"), ("--goal-x-m", "nan"),
                      ("--duration-ms", "20001"), ("--timeout", "22")):
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.parse(*base, *extra)

    def test_successful_continuous_goal_keeps_enabled_without_publishing_stop(self):
        session = swd.Session(MemoryClient(), SYMBOLS, Log())
        session._firmware_verified = True
        with patch.object(session, "snapshot", return_value=valid_state()), \
             patch.object(session, "publish", return_value=10) as publish, \
             patch.object(session, "wait_complete", return_value=self.hold_result()) as complete:
            result = session.execute(7, [-.3, .02, .1, .2, .5, 20000, 0, 0], 23)
        self.assertEqual(result["sample"]["enabled"], 1)
        self.assertEqual([call.args[0] for call in publish.call_args_list], [7])
        self.assertEqual(complete.call_args.kwargs, {"expected_command": 7})
        self.assertIn("enabled_hold_preserved", [event for event, _ in session.log.records])

    def test_continuous_goal_bad_hold_or_endpoint_still_publishes_stop(self):
        for field, value in (("enabled", 0), ("state_2", 0), ("target_3", .1),
                             ("end_valid", 0), ("end_stopped", 0), ("odom_valid", 0),
                             ("end_x_m", -.25), ("end_yaw_rad", .3)):
            session = swd.Session(MemoryClient(), SYMBOLS, Log())
            session._firmware_verified = True
            result = self.hold_result()
            result["sample"][field] = value
            with self.subTest(field=field), patch.object(session, "snapshot", return_value=valid_state()), \
                 patch.object(session, "publish", side_effect=[10, 11]) as publish, \
                 patch.object(session, "wait_complete", side_effect=[result, {}]), self.assertRaises(swd.SessionError):
                session.execute(7, [-.3, .02, .1, .2, .5, 20000, 0, 0], 23)
            self.assertEqual([call.args[0] for call in publish.call_args_list], [7, 1])

    def test_continuous_goal_accepts_coarse_ten_degree_yaw_only(self):
        args = [-.3, .02, .1, .2, .5, 20000, 0, 0]
        for degrees in (-10, -5.005, 6, 10):
            result = self.hold_result()
            result["sample"]["end_yaw_rad"] = args[2] + math.radians(degrees)
            with self.subTest(degrees=degrees):
                swd.validate_endpoint(7, args, result)
        for degrees in (-10.001, 10.001):
            result = self.hold_result()
            result["sample"]["end_yaw_rad"] = args[2] + math.radians(degrees)
            with self.subTest(degrees=degrees), self.assertRaises(swd.SessionError):
                swd.validate_endpoint(7, args, result)

    def test_visual_window_tolerance_does_not_relax_odometry_return(self):
        args = [-.3, .02, .1, .2, .5, 20000, 0, 0]
        result = self.hold_result()
        result['sample']['end_x_m'] = args[0] + .02027
        with self.assertRaises(swd.SessionError):
            swd.validate_endpoint(7, args, result)
        result['command'] = 8
        swd.validate_endpoint(8, args, result)
        result['sample']['end_x_m'] = args[0] + .036
        with self.assertRaises(swd.SessionError):
            swd.validate_endpoint(8, args, result)

    def test_continuous_goal_fault_stops_without_retry(self):
        session = swd.Session(MemoryClient(), SYMBOLS, Log())
        session._firmware_verified = True
        with patch.object(session, "snapshot", return_value=valid_state()), \
             patch.object(session, "publish", side_effect=[10, 11]) as publish, \
             patch.object(session, "wait_complete", side_effect=[swd.SessionError("firmware fault"), {}]), \
             self.assertRaisesRegex(swd.SessionError, "firmware fault"):
            session.execute(7, [-.3, .02, .1, .2, .5, 20000, 0, 0], 23)
        self.assertEqual([call.args[0] for call in publish.call_args_list], [7, 1])

    def test_terminal_enabled_one_only_accepted_for_expected_command_seven(self):
        session = swd.Session(MemoryClient(), SYMBOLS, Log())
        first = {**self.hold_result(), "ack_seq": 7, "sample_seq": 12}
        second = {**self.hold_result(), "ack_seq": 7, "sample_seq": 14}
        with patch.object(session, "beat"), patch.object(session, "snapshot", side_effect=[first, second]):
            self.assertEqual(session.wait_complete(7, 1, expected_command=7)["sample"]["enabled"], 1)
        wrong = {**first, "command": 4}
        with patch.object(session, "beat"), patch.object(session, "snapshot", return_value=wrong), \
             self.assertRaisesRegex(swd.SessionError, "replaced"):
            session.wait_complete(7, 1, expected_command=7)

    def test_motion_limits_and_nonfinite_rejected(self):
        valid = self.parse("--command", "4", "--vx", "0.25")
        self.assertEqual(swd.command_values(valid)[:4], [0.25, 0.0, 0.0, 800.0])
        for extra in (("--vx", "0.251"), ("--vx", "nan"), ("--vy", "0.001"),
                      ("--wz", "0.501"), ("--duration-ms", "1501"),
                      ("--duration-ms", "99"), ("--wz", "0.1"),
                      ("--kd", "0.049", "0.2", "0.2"), ("--kd", "0.2", "0.501", "0.2"),
                      ("--friction", "0", "0.031", "0")):
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.parse("--command", "4", "--vx", "0.1", *extra)

    def test_arguments_publish_before_sequence_and_wrap(self):
        client = MemoryClient()
        client.memory[SYMBOLS["tuning_request_seq"]] = 0xFFFFFFFF
        session = swd.Session(client, SYMBOLS, Log())
        session._firmware_verified = session._initial_validated = True
        sequence = session.publish(4, [0.25, 0, 0, 800, 0, 0, 0, 0])
        self.assertEqual(sequence, 0)
        self.assertEqual([entry[0] for entry in client.writes],
                         [SYMBOLS["tuning_heartbeat"], SYMBOLS["tuning_args"],
                          SYMBOLS["tuning_command"], SYMBOLS["tuning_request_seq"]])
        self.assertEqual(client.writes[-2][1], [4])
        self.assertEqual(client.writes[-1][1], [0])
        self.assertAlmostEqual(struct.unpack("<f", struct.pack("<I", client.writes[1][1][0]))[0], 0.25)

    def test_move_failure_always_stops_without_retrying_move(self):
        session = swd.Session(MemoryClient(), SYMBOLS, Log())
        session._firmware_verified = True
        with patch.object(session, "snapshot", return_value=valid_state()), \
             patch.object(session, "publish", side_effect=[10, 11]) as publish, \
             patch.object(session, "wait_complete", side_effect=[swd.SessionError("lost ack"), {}]):
            with self.assertRaisesRegex(swd.SessionError, "lost ack"):
                session.execute(4, [0.1, 0, 0, 800, 0, 0, 0, 0], 1)
        self.assertEqual([call.args[0] for call in publish.call_args_list], [4, 1])
        self.assertIn("final_stop_confirmed", [record[0] for record in session.log.records])

    def test_explicit_stop_is_sent_once_after_confirmation(self):
        session = swd.Session(MemoryClient(), SYMBOLS, Log())
        session._firmware_verified = True
        with patch.object(session, "snapshot", return_value=valid_state()), \
             patch.object(session, "publish", return_value=10) as publish, \
             patch.object(session, "wait_complete", return_value=valid_state()) as complete:
            session.execute(1, [0.] * 8, 2.5)
        publish.assert_called_once_with(1, [0.] * 8)
        complete.assert_called_once_with(10, 2.5, stop=True)
        self.assertIn("final_stop_confirmed", [event for event, _ in session.log.records])

    def test_unconfirmed_explicit_stop_still_retries_cleanup(self):
        session = swd.Session(MemoryClient(), SYMBOLS, Log())
        session._firmware_verified = True
        with patch.object(session, "snapshot", return_value=valid_state()), \
             patch.object(session, "publish", side_effect=[10, 11]) as publish, \
             patch.object(session, "wait_complete", side_effect=[swd.SessionError("lost ack"), {}]), \
             self.assertRaisesRegex(swd.SessionError, "lost ack"):
            session.execute(1, [0.] * 8, 2.5)
        self.assertEqual([call.args[0] for call in publish.call_args_list], [1, 1])
        self.assertIn("final_stop_confirmed", [event for event, _ in session.log.records])

    def test_unconfirmed_final_stop_fails_successful_session(self):
        session = swd.Session(MemoryClient(), SYMBOLS, Log())
        session._firmware_verified = True
        with patch.object(session, "snapshot", return_value=valid_state()), \
             patch.object(session, "publish", side_effect=[10, 11]), \
             patch.object(session, "wait_complete", side_effect=[{}, swd.SessionError("timeout")]):
            with self.assertRaisesRegex(swd.SessionError, "Final STOP not confirmed"):
                session.execute(3, [0.2, 0.25, 0.2, 0, 0, 0, 0, 0], 1)

    def test_snapshot_rejects_torn_sample_then_accepts_even_sequence(self):
        client = MemoryClient()
        client.parse_words = swd.TclClient.parse_words
        values = [0, 0, 4, 0, 7, 4, 2] + [swd.float_word(float(i)) for i in range(len(swd.SAMPLE_NAMES))] + [2]
        torn = list(values)
        torn[-1] = 4
        client.evaluate = unittest.mock.Mock(side_effect=[" ".join(map(str, torn)), " ".join(map(str, values))])
        state = swd.Session(client, SYMBOLS, Log()).snapshot()
        self.assertEqual(state["sample_seq"], 2)
        self.assertEqual(state["sample"]["motion_elapsed_ms"], 39.0)
        self.assertEqual(state["sample"]["odom_x_m"], 40.0)
        self.assertEqual(state["sample"]["end_target_distance_m"], 55.0)
        self.assertEqual(client.evaluate.call_count, 2)

    def test_contiguous_mailbox_uses_two_reads_and_retries_renew_heartbeat(self):
        client = MemoryClient()
        client.parse_words = swd.TclClient.parse_words
        raw = [11, 4, 17, 11, 2, 0, 8] + [0] * 8
        raw += [swd.float_word(float(i)) for i in range(len(swd.SAMPLE_NAMES))] + [8]
        torn = list(raw)
        torn[-1] = 10
        client.evaluate = unittest.mock.Mock(side_effect=[" ".join(map(str, torn)), " ".join(map(str, raw))])
        session = swd.Session(client, BURST_SYMBOLS, Log())
        session.heartbeat = 17
        with patch.object(session, "beat") as beat:
            state = session.snapshot()
        self.assertEqual(beat.call_count, 2)
        self.assertEqual(state["request_seq"], 11)
        self.assertEqual(state["heartbeat"], 17)
        self.assertEqual(state["phase"], 2)
        self.assertEqual(state["sample_seq"], 8)
        self.assertEqual(state["sample"]["motion_elapsed_ms"], 39.0)
        self.assertEqual(state["sample"]["odom_age_ms"], 47.0)
        self.assertEqual(state["sample"]["end_progress_m"], 54.0)
        for call in client.evaluate.call_args_list:
            self.assertEqual(call.args[0].count("read_memory"), 2)
            self.assertIn(f"read_memory 0x200074d4 32 {len(swd.SAMPLE_NAMES) + 15}", call.args[0])
            self.assertIn("read_memory 0x200074ec 32 1", call.args[0])

    def test_read_only_burst_retry_never_starts_heartbeat(self):
        client = MemoryClient()
        client.parse_words = swd.TclClient.parse_words
        raw = [0] * (len(swd.SAMPLE_NAMES) + 16)
        raw[6] = 2
        raw[-1] = 2
        torn = list(raw)
        torn[-1] = 4
        client.evaluate = unittest.mock.Mock(side_effect=[" ".join(map(str, torn)), " ".join(map(str, raw))])
        session = swd.Session(client, BURST_SYMBOLS, Log())
        with patch.object(session, "beat") as beat:
            session.snapshot()
        beat.assert_not_called()
        self.assertEqual(client.writes, [])

    def test_tcl_fragmented_framing_and_error(self):
        sock = FakeSocket([b"0:0x1234 ", b"5\x1a", b"1:read failed\x1a"])
        with patch.object(swd.socket, "create_connection", return_value=sock):
            client = swd.TclClient("127.0.0.1", 6666)
        self.assertEqual(client.read_words(0x20000000, 2), [0x1234, 5])
        self.assertTrue(sock.sent[0].endswith(b"\x1a"))
        with self.assertRaisesRegex(swd.SessionError, "read failed"):
            client.read_words(0x20000000)

    def test_timeout_poisoning_prevents_late_reply_reuse(self):
        sock = FakeSocket([socket.timeout("late reply")])
        with patch.object(swd.socket, "create_connection", return_value=sock):
            client = swd.TclClient("127.0.0.1", 6666)
        with self.assertRaises(socket.timeout):
            client.read_words(0x20000000)
        self.assertFalse(client.usable)
        self.assertTrue(sock.closed)
        with self.assertRaisesRegex(swd.SessionError, "cannot be reused"):
            client.write_words(0x20000000, [1])
        self.assertEqual(len(sock.sent), 1)

    def test_heartbeat_runs_at_100ms_between_25hz_samples(self):
        clock = [0.0]
        beats = []
        client = MemoryClient()
        real_write = client.write_words

        def write(address, words):
            if address == SYMBOLS["tuning_heartbeat"]:
                beats.append(clock[0])
            real_write(address, words)

        client.write_words = write
        session = swd.Session(client, SYMBOLS, Log(), 25.0)
        session._firmware_verified = session._initial_validated = True
        with patch.object(swd.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(swd.time, "sleep", side_effect=lambda delay: clock.__setitem__(0, clock[0] + delay)), \
             patch.object(session, "snapshot", side_effect=lambda: {
                 "ack_seq": 7, "phase": 4 if clock[0] >= .21 else 2, "error": 0,
                 "sample_seq": round(clock[0] * 100),
                 "sample": {"phase": 4 if clock[0] >= .21 else 2, "error": 0, "enabled": 0}}):
            session.wait_complete(7, 0.4)
        self.assertEqual(len(beats), 3)
        for actual, expected in zip(beats, (0.0, 0.1, 0.2)):
            self.assertAlmostEqual(actual, expected)

    def test_parameter_reply_keeps_bits_and_decodes_float_registers(self):
        client = unittest.mock.Mock()
        client.read_words.return_value = [1, swd.float_word(0.1), 400, 1,
                                          1, swd.float_word(0.2), 410, 1,
                                          1, swd.float_word(0.3), 420, 1]
        session = swd.Session(client, {**SYMBOLS, "dm_param_reply": 0x2000F000}, Log())
        replies = session.parameter_replies()
        self.assertEqual([reply["motor_id"] for reply in replies], [1, 2, 3])
        self.assertEqual(replies[0]["value_bits"], swd.float_word(0.1))
        self.assertAlmostEqual(replies[0]["value_float"], 0.1)

    def test_read_mode_never_writes_heartbeat_or_stop(self):
        state = valid_state()
        client = unittest.mock.Mock()
        client.evaluate.return_value = "running"
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(swd, "load_symbols", return_value=SYMBOLS), \
             patch.object(swd, "TclClient", return_value=client), \
             patch.object(swd.Session, "verify_firmware"), \
             patch.object(swd.Session, "snapshot", return_value=state), \
             contextlib.redirect_stdout(io.StringIO()):
            code = swd.main(["--elf", "unused.elf", "--read", "--log", str(Path(folder) / "read.jsonl")])
        self.assertEqual(code, 0)
        client.write_words.assert_not_called()

    def test_flash_verification_includes_ram_load_segment_and_partial_words(self):
        image, segments = fixture_elf()
        client = FlashClient(segments)
        checked = swd.verify_flash(client, image)
        self.assertEqual(checked, [{"address": physical, "bytes": len(data)} for physical, _, data in segments])
        self.assertTrue(all(count <= 256 for _, count in client.reads))
        self.assertEqual(client.writes, [])
        client.memory[0x08002004] ^= 1
        with self.assertRaisesRegex(swd.SessionError, "0x08002004"):
            swd.verify_flash(client, image)
        self.assertEqual(client.writes, [])

    def test_flash_mismatch_blocks_read_and_every_command_without_stop(self):
        image, segments = fixture_elf()
        with tempfile.TemporaryDirectory() as folder:
            elf = Path(folder) / "test.elf"
            elf.write_bytes(image)
            for index, action in enumerate((["--read"], ["--command", "1"], ["--command", "4", "--vx", "0.25"])):
                client = FlashClient(segments)
                client.memory[0x08000004] ^= 1
                with self.subTest(action=action), \
                     patch.object(swd, "load_symbols", return_value=SYMBOLS), \
                     patch.object(swd, "TclClient", return_value=client), \
                     patch.object(swd.Session, "snapshot") as snapshot, \
                     contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    code = swd.main(["--elf", str(elf), "--log", str(Path(folder) / f"case{index}.jsonl"), *action])
                self.assertEqual(code, 1)
                self.assertEqual(client.writes, [])
                snapshot.assert_not_called()

    def test_invalid_initial_mailbox_never_publishes_or_stops(self):
        for field, value in (("phase", 3593822552), ("error", 11),
                             ("sample.phase", 6), ("sample.error", -1),
                             ("sample.enabled", 2), ("sample.enabled", None)):
            client = MemoryClient()
            session = swd.Session(client, SYMBOLS, Log())
            session._firmware_verified = True
            state = valid_state()
            if field.startswith("sample."):
                state["sample"][field.split(".")[1]] = value
            else:
                state[field] = value
            with self.subTest(field=field, value=value), \
                 patch.object(session, "snapshot", return_value=state), \
                 patch.object(session, "publish") as publish, \
                 self.assertRaisesRegex(swd.SessionError, "Invalid mailbox"):
                session.execute(4, [0.25, 0, 0, 800, 0, 0, 0, 0], 1)
            self.assertEqual(client.writes, [])
            publish.assert_not_called()

    def test_no_ram_write_before_preflight_even_for_stop(self):
        client = MemoryClient()
        session = swd.Session(client, SYMBOLS, Log())
        with self.assertRaisesRegex(swd.SessionError, "preflight"):
            session.publish(1, [0.0] * 8)
        self.assertEqual(client.writes, [])
        with patch.object(session, "publish") as publish, self.assertRaisesRegex(swd.SessionError, "preflight"):
            session.execute(1, [0.0] * 8, 1)
        publish.assert_not_called()

    def test_bad_elf_is_rejected_before_any_memory_read(self):
        client = FlashClient([])
        for image in (b"", b"not an ELF" + bytes(100)):
            with self.assertRaises(swd.SessionError):
                swd.verify_flash(client, image)
        self.assertEqual(client.reads, [])
        self.assertEqual(client.writes, [])

    def test_odometry_commands_encode_bounded_explicit_parameters(self):
        static = self.parse("--command", "5", "--duration-ms", "1500")
        self.assertEqual(swd.command_values(static), [0, 0, 0, 1500, 0, 0, 0, 0])
        for direction in (1, 2):
            distance = self.parse("--command", "6", "--direction", str(direction),
                                  "--distance-m", "0.05", "--speed-mps", "0.05",
                                  "--duration-ms", "5000", "--timeout", "8")
            self.assertEqual(swd.command_values(distance), [direction, .05, .05, 5000, 0, 0, 0, 0])
        base = ("--command", "6", "--direction", "1", "--distance-m", "0.05", "--speed-mps", "0.05")
        for extra in (("--distance-m", "0.009"), ("--distance-m", "1.001"),
                      ("--distance-m", "nan"), ("--speed-mps", "0.019"),
                      ("--speed-mps", "0.101"), ("--speed-mps", "inf"),
                      ("--direction", "3"), ("--duration-ms", "20001"),
                      ("--duration-ms", "4000", "--timeout", "6"), ("--vx", "0.1")):
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.parse(*base, *extra)
        for command in (("--command", "6"), ("--command", "5", "--duration-ms", "1501"),
                        ("--command", "5", "--direction", "1"), ("--command", "5", "--vx", "0.1")):
            with self.subTest(command=command), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.parse(*command)

    def test_odometry_failures_stop_without_replaying_the_action(self):
        for command, args in ((5, [0, 0, 0, 800, 0, 0, 0, 0]),
                              (6, [1, .05, .05, 1000, 0, 0, 0, 0])):
            session = swd.Session(MemoryClient(), SYMBOLS, Log())
            session._firmware_verified = True
            with self.subTest(command=command), \
                 patch.object(session, "snapshot", return_value=valid_state()), \
                 patch.object(session, "publish", side_effect=[10, 11]) as publish, \
                 patch.object(session, "wait_complete", side_effect=[swd.SessionError("deadline"), {}]), \
                 self.assertRaisesRegex(swd.SessionError, "deadline"):
                session.execute(command, args, 5)
            self.assertEqual([call.args[0] for call in publish.call_args_list], [command, 1])

    def test_one_metre_distance_limits_do_not_extend_timed_motion(self):
        base = ("--command", "6", "--direction", "1", "--distance-m", "1",
                "--speed-mps", "0.1", "--duration-ms", "20000")
        for timeout in (23, 25):
            args = self.parse(*base, "--timeout", str(timeout))
            self.assertEqual(swd.command_values(args), [1, 1, .1, 20000, 0, 0, 0, 0])
            self.assertEqual(args.timeout, timeout)
        for extra in (("--timeout", "22"), ("--timeout", "25.001"),
                      ("--timeout", "23", "--duration-ms", "20001"),
                      ("--timeout", "23", "--distance-m", "1.000001"),
                      ("--timeout", "23", "--speed-mps", "0.100001")):
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.parse(*base, *extra)
        for command in (("--command", "4", "--vx", "0.1", "--duration-ms", "1501"),
                        ("--command", "4", "--vx", "0.1", "--timeout", "15.001"),
                        ("--command", "5", "--duration-ms", "1501"),
                        ("--command", "5", "--timeout", "15.001")):
            with self.subTest(command=command), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.parse(*command)

    def test_direct_execute_rejects_excessive_timeout_without_writes(self):
        for command, args, timeout in ((6, [1, 1, .1, 20000, 0, 0, 0, 0], 25.001),
                                       (4, [.1, 0, 0, 800, 0, 0, 0, 0], 15.001),
                                       (6, [1, 1, .1, 20000, 0, 0, 0, 0], float("inf"))):
            session = swd.Session(MemoryClient(), SYMBOLS, Log())
            session._firmware_verified = True
            with self.subTest(command=command, timeout=timeout), \
                 patch.object(session, "snapshot", return_value=valid_state()), \
                 self.assertRaisesRegex(swd.SessionError, "Command timeout"):
                session.execute(command, args, timeout)
            self.assertEqual(session.client.writes, [])

    def test_distance_timeout_rejected_before_ram_write(self):
        session = swd.Session(MemoryClient(), SYMBOLS, Log())
        session._firmware_verified = True
        with patch.object(session, "snapshot", return_value=valid_state()), \
             self.assertRaisesRegex(swd.SessionError, "duration plus 2 seconds"):
            session.execute(6, [1, .05, .05, 1000, 0, 0, 0, 0], 3)
        self.assertEqual(session.client.writes, [])

    def test_symbol_sizes_reject_stale_abi_duplicates_and_out_of_bounds(self):
        rows = [f"{address:08x} {swd.SYMBOL_SIZES[name]:08x} B {name}" for name, address in SYMBOLS.items()]
        with patch.object(swd.subprocess, "run", return_value=SimpleNamespace(stdout="\n".join(rows))) as run:
            self.assertEqual(swd.load_symbols("test.elf", "nm"), SYMBOLS)
        self.assertIn("-S", run.call_args.args[0])
        old_sample = rows[:-1] + [f"{SYMBOLS['tuning_sample']:08x} 000000a0 B tuning_sample"]
        corruptions = (old_sample, rows + [rows[0]],
                       rows[:-1] + ["2001fff0 000000e0 B tuning_sample"], rows[:-1])
        for corrupted in corruptions:
            with self.subTest(rows=corrupted), \
                 patch.object(swd.subprocess, "run", return_value=SimpleNamespace(stdout="\n".join(corrupted))), \
                 self.assertRaises(swd.SessionError):
                swd.load_symbols("test.elf", "nm")

    def test_extra_firmware_symbols_are_required_and_unique_in_one_nm_pass(self):
        rows = [f"{address:08x} {swd.SYMBOL_SIZES[name]:08x} B {name}" for name, address in SYMBOLS.items()]
        extra = {"tuning_spin_sample": 32}
        row = "20003000 00000020 B tuning_spin_sample"
        with patch.object(swd.subprocess, "run", return_value=SimpleNamespace(stdout="\n".join(rows + [row]))) as nm:
            self.assertEqual(swd.load_symbols("test.elf", "nm", extra_sizes=extra),
                             {**SYMBOLS, "tuning_spin_sample": 0x20003000})
            nm.assert_called_once()
        for extra_rows in ([], [row, row], ["20003000 00000004 B tuning_spin_sample"]):
            with self.subTest(extra_rows=extra_rows), \
                 patch.object(swd.subprocess, "run", return_value=SimpleNamespace(stdout="\n".join(rows + extra_rows))), \
                 self.assertRaises(swd.SessionError):
                swd.load_symbols("test.elf", "nm", extra_sizes=extra)

    def test_new_fault_codes_are_valid_mailbox_values(self):
        for code in (9, 10):
            state = valid_state()
            state["phase"] = state["sample"]["phase"] = 5
            state["error"] = state["sample"]["error"] = code
            swd.Session.validate_snapshot(state)

    def test_completion_waits_for_fresh_terminal_endpoint_sample(self):
        session = swd.Session(MemoryClient(), SYMBOLS, Log())
        states = [
            {"ack_seq": 7, "phase": 4, "error": 0, "sample_seq": 12,
             "sample": {"phase": 4, "error": 0, "enabled": 0, "end_progress_m": .01}},
            {"ack_seq": 7, "phase": 4, "error": 0, "sample_seq": 12,
             "sample": {"phase": 4, "error": 0, "enabled": 0, "end_progress_m": .01}},
            {"ack_seq": 7, "phase": 4, "error": 0, "sample_seq": 14,
             "sample": {"phase": 4, "error": 0, "enabled": 0, "end_progress_m": .05}},
        ]
        with patch.object(session, "beat"), patch.object(session, "snapshot", side_effect=states):
            result = session.wait_complete(7, 1)
        self.assertEqual(result["sample"]["end_progress_m"], .05)

    def test_odometry_done_requires_complete_endpoint_evidence_and_still_stops(self):
        good = {"end_x_m": .05, "end_y_m": 0, "end_yaw_rad": 0,
                "end_elapsed_ms": 800, "end_progress_m": .05,
                "end_target_distance_m": .05, "end_valid": 1, "end_stopped": 1}
        cases = [(5, "end_valid", 0), (5, "end_stopped", 0),
                 (5, "end_x_m", None), (5, "end_y_m", float("nan")),
                 (5, "end_yaw_rad", float("inf")), (5, "end_elapsed_ms", -1),
                 (6, "end_valid", 0), (6, "end_stopped", 0),
                 (6, "end_progress_m", .0469), (6, "end_progress_m", .0531),
                 (6, "end_target_distance_m", .06)]
        for command, field, value in cases:
            session = swd.Session(MemoryClient(), SYMBOLS, Log())
            session._firmware_verified = True
            args = [0, 0, 0, 800, 0, 0, 0, 0] if command == 5 else [1, .05, .05, 1000, 0, 0, 0, 0]
            invalid = {**good, field: value}
            with self.subTest(command=command, field=field, value=value), \
                 patch.object(session, "snapshot", return_value=valid_state()), \
                 patch.object(session, "publish", side_effect=[10, 11]) as publish, \
                 patch.object(session, "wait_complete", side_effect=[{"sample": invalid}, {}]), \
                 self.assertRaisesRegex(swd.SessionError, "Odometry endpoint"):
                session.execute(command, args, 5)
            self.assertEqual([call.args[0] for call in publish.call_args_list], [command, 1])
            self.assertIn("final_stop_confirmed", [event for event, _ in session.log.records])

    def test_odometry_endpoint_accepts_float32_target_and_arrival_tolerance(self):
        target = swd.word_float(swd.float_word(.05))
        for command, progress in ((5, 0), (6, target - .003), (6, target), (6, target + .003)):
            session = swd.Session(MemoryClient(), SYMBOLS, Log())
            session._firmware_verified = True
            args = [0, 0, 0, 800, 0, 0, 0, 0] if command == 5 else [1, .05, .05, 1000, 0, 0, 0, 0]
            result = {"command": command, "sample": {"end_x_m": progress, "end_y_m": 0, "end_yaw_rad": 0,
                                 "end_elapsed_ms": 800, "end_progress_m": progress,
                                 "end_target_distance_m": target, "end_valid": 1, "end_stopped": 1}}
            with self.subTest(command=command, progress=progress), \
                 patch.object(session, "snapshot", return_value=valid_state()), \
                 patch.object(session, "publish", side_effect=[10, 11]), \
                 patch.object(session, "wait_complete", side_effect=[result, {}]):
                self.assertEqual(session.execute(command, args, 5), result)
            self.assertIn("endpoint_verified", [event for event, _ in session.log.records])
            verified = next(data for event, data in session.log.records if event == "endpoint_verified")
            self.assertEqual(verified["command"], command)
            self.assertEqual(verified["sample"], result["sample"])


if __name__ == "__main__":
    unittest.main()
