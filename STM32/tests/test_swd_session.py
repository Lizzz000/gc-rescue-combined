"""Host-only safety/transport tests; never connects to a debug probe."""
import contextlib
import importlib.util
import io
from pathlib import Path
import socket
import struct
import tempfile
import unittest
from unittest.mock import patch


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
        values = [0, 0, 4, 0, 7, 4, 2] + [swd.float_word(float(i)) for i in range(40)] + [2]
        torn = list(values)
        torn[-1] = 4
        client.evaluate = unittest.mock.Mock(side_effect=[" ".join(map(str, torn)), " ".join(map(str, values))])
        state = swd.Session(client, SYMBOLS, Log()).snapshot()
        self.assertEqual(state["sample_seq"], 2)
        self.assertEqual(state["sample"]["motion_elapsed_ms"], 39.0)
        self.assertEqual(client.evaluate.call_count, 2)

    def test_contiguous_mailbox_uses_two_reads_and_retries_renew_heartbeat(self):
        client = MemoryClient()
        client.parse_words = swd.TclClient.parse_words
        raw = [11, 4, 17, 11, 2, 0, 8] + [0] * 8
        raw += [swd.float_word(float(i)) for i in range(40)] + [8]
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
        for call in client.evaluate.call_args_list:
            self.assertEqual(call.args[0].count("read_memory"), 2)
            self.assertIn("read_memory 0x200074d4 32 55", call.args[0])
            self.assertIn("read_memory 0x200074ec 32 1", call.args[0])

    def test_read_only_burst_retry_never_starts_heartbeat(self):
        client = MemoryClient()
        client.parse_words = swd.TclClient.parse_words
        raw = [0] * 56
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
             patch.object(session, "snapshot", side_effect=lambda: {"ack_seq": 7, "phase": 4 if clock[0] >= .25 else 2, "error": 0}):
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
        for field, value in (("phase", 3593822552), ("error", 9),
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


if __name__ == "__main__":
    unittest.main()
