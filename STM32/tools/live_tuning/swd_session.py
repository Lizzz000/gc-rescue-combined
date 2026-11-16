"""Bounded live-tuning sessions over OpenOCD Tcl; never halts or resets the MCU.

Use the exact ELF flashed on the board. --describe is offline; all connected
sessions compare every ELF Flash load byte before accessing the mailbox.
Mutating sessions own one local writer lock, publish arguments before the
request sequence, renew the heartbeat, and request STOP in finally.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time


SYMBOLS = (
    "tuning_request_seq", "tuning_command", "tuning_heartbeat", "tuning_args",
    "tuning_ack_seq", "tuning_phase", "tuning_error", "tuning_sample_seq",
    "tuning_sample",
)
OPTIONAL_SYMBOLS = ("dm_param_reply",)
SCALARS = ("request_seq", "ack_seq", "phase", "error", "heartbeat", "command")
SAMPLE_NAMES = (
    "tick_ms", "phase", "error", "enabled", "yaw_deg", "wz_imu",
    "body_vx", "body_vy", "body_wz", "pos_1", "pos_2", "pos_3",
    "vel_1", "vel_2", "vel_3", "torque_1", "torque_2", "torque_3",
    "state_1", "state_2", "state_3", "mos_temp_1", "mos_temp_2", "mos_temp_3",
    "target_1", "target_2", "target_3", "ff_1", "ff_2", "ff_3",
    "kd_1", "kd_2", "kd_3", "ff_gain", "friction_1", "friction_2",
    "friction_3", "battery", "imu_age_ms", "motion_elapsed_ms",
)
ERRORS = {
    1: "invalid command/argument", 2: "busy", 3: "prepare/IMU failure",
    4: "motion/controller failure", 5: "heartbeat expired", 6: "parameter timeout",
    7: "parameter transmit failure", 8: "stop timeout",
}


class SessionError(RuntimeError):
    pass


def float_word(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def word_float(value):
    return struct.unpack("<f", struct.pack("<I", value))[0]


def load_symbols(elf, nm=None):
    nm = nm or shutil.which("arm-none-eabi-nm") or shutil.which("arm-none-eabi-nm.exe")
    if not nm:
        raise SessionError("arm-none-eabi-nm not found; pass --nm")
    result = subprocess.run([nm, "--defined-only", "--numeric-sort", str(elf)],
                            check=True, capture_output=True, text=True, timeout=15)
    symbols = {}
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[2] in (*SYMBOLS, *OPTIONAL_SYMBOLS):
            address = int(fields[0], 16)
            if not 0x20000000 <= address < 0x20020000 or address % 4:
                raise SessionError(f"{fields[2]} is not aligned F407 SRAM: {address:#x}")
            symbols[fields[2]] = address
    missing = set(SYMBOLS) - symbols.keys()
    if missing:
        raise SessionError("ELF missing tuning symbols: " + ", ".join(sorted(missing)))
    return symbols


def flash_segments(image):
    """Return ELF32 little-endian ARM PT_LOAD bytes at their physical address.

    The .data segment has a RAM virtual address but a Flash physical/load
    address, so it must be compared too. BSS has no file bytes to compare.
    """
    if len(image) < 52:
        raise SessionError("ELF header is truncated")
    header = struct.unpack_from("<16sHHIIIIIHHHHHH", image)
    ident, kind, machine, version = header[:4]
    phoff, ehsize, phsize, phcount = header[5], header[8], header[9], header[10]
    if ident[:7] != b"\x7fELF\x01\x01\x01" or kind != 2 or machine != 40 or version != 1:
        raise SessionError("Expected a little-endian ELF32 ARM executable")
    if ehsize != 52 or phsize < 32 or not phcount or phoff + phsize * phcount > len(image):
        raise SessionError("Invalid ELF program-header table")
    segments = []
    for index in range(phcount):
        ptype, offset, vaddr, paddr, filesz, memsz, _, _ = struct.unpack_from(
            "<IIIIIIII", image, phoff + index * phsize)
        if ptype != 1:
            continue
        if filesz > memsz or offset + filesz > len(image):
            raise SessionError("Invalid or truncated ELF PT_LOAD segment")
        if not filesz:
            continue
        if 0x08000000 <= paddr < 0x08100000:
            if paddr + filesz > 0x08100000:
                raise SessionError("ELF load segment exceeds F407 Flash")
            segments.append((paddr, image[offset:offset + filesz]))
        elif 0x08000000 <= vaddr < 0x08100000:
            raise SessionError("Flash PT_LOAD has an unexpected physical address")
    segments.sort(key=lambda entry: entry[0])
    if not segments:
        raise SessionError("ELF has no Flash PT_LOAD bytes to verify")
    for (address, data), (next_address, _) in zip(segments, segments[1:]):
        if address + len(data) > next_address:
            raise SessionError("ELF Flash load segments overlap")
    return segments


def verify_flash(client, image):
    """Read only, in <=256-word bursts; require every load byte to match."""
    segments = flash_segments(image)
    for address, expected in segments:
        start, end = address & ~3, (address + len(expected) + 3) & ~3
        for current in range(start, end, 256 * 4):
            count = min(256, (end - current) // 4)
            words = client.read_words(current, count)
            if len(words) != count:
                raise SessionError("Incomplete Flash read during preflight")
            actual = struct.pack("<" + "I" * count, *words)
            lo, hi = max(address, current), min(address + len(expected), current + len(actual))
            wanted = expected[lo - address:hi - address]
            received = actual[lo - current:hi - current]
            if received != wanted:
                offset = next(i for i, (got, want) in enumerate(zip(received, wanted)) if got != want)
                raise SessionError(f"Flash/ELF mismatch at {lo + offset:#010x}: "
                                   f"board={received[offset]:02x}, ELF={wanted[offset]:02x}; no RAM writes allowed")
    return [{"address": address, "bytes": len(data)} for address, data in segments]


class TclClient:
    def __init__(self, host, port, timeout=0.3):
        self.endpoint, self.timeout = (host, port), timeout
        self.sock = None
        self.reconnect()

    def reconnect(self):
        if self.sock:
            self.sock.close()
        self.sock = socket.create_connection(self.endpoint, self.timeout)
        self.sock.settimeout(self.timeout)
        self.buffer = b""
        self.usable = True

    def close(self):
        self.sock.close()
        self.usable = False

    def set_timeout(self, timeout):
        self.sock.settimeout(timeout)
        self.timeout = timeout

    def evaluate(self, command):
        if not self.usable:
            raise SessionError("Tcl connection has failed; its responses cannot be reused")
        # catch distinguishes OpenOCD errors from successful empty responses.
        wrapped = ("set _gc_swd_status [catch {" + command +
                   "} _gc_swd_result]; format {%d:%s} $_gc_swd_status $_gc_swd_result")
        try:
            self.sock.sendall(wrapped.encode("ascii") + b"\x1a")
            while b"\x1a" not in self.buffer:
                chunk = self.sock.recv(65536)
                if not chunk:
                    raise SessionError("OpenOCD closed the Tcl connection")
                self.buffer += chunk
                if len(self.buffer) > 1024 * 1024:
                    raise SessionError("OpenOCD response exceeded 1 MiB")
        except (OSError, SessionError):
            self.close()
            raise
        response, self.buffer = self.buffer.split(b"\x1a", 1)
        status, separator, body = response.decode("utf-8", "replace").partition(":")
        if not separator or status != "0":
            raise SessionError("OpenOCD: " + response.decode("utf-8", "replace"))
        return body.strip()

    def read_words(self, address, count=1):
        return self.parse_words(self.evaluate(f"read_memory {address:#x} 32 {count}"), count)

    @staticmethod
    def parse_words(text, count):
        try:
            words = [int(item, 0) for item in text.split()]
        except ValueError as exc:
            raise SessionError(f"Invalid memory response: {text!r}") from exc
        if len(words) != count or any(not 0 <= value <= 0xFFFFFFFF for value in words):
            raise SessionError(f"Expected {count} uint32 words, received {text!r}")
        return words

    def write_words(self, address, words):
        if any(not 0 <= word <= 0xFFFFFFFF for word in words):
            raise ValueError("write requires uint32 words")
        values = " ".join(f"0x{word:08x}" for word in words)
        self.evaluate(f"write_memory {address:#x} 32 {{{values}}}")


class JsonLog:
    def __init__(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("x", encoding="utf-8")
        self.started = time.monotonic()

    def emit(self, event, **data):
        record = {"event": event, "host_s": time.monotonic() - self.started, **data}
        self.file.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self.file.flush()

    def close(self):
        self.file.close()


@contextmanager
def writer_lock(host, port):
    # Locks persist as files; deleting a locked file would allow split ownership.
    canonical_host = socket.gethostbyname(host)
    identity = hashlib.sha256(f"{canonical_host}:{port}".encode()).hexdigest()[:20]
    path = Path(tempfile.gettempdir()) / f"gc_swd_writer_{identity}.lock"
    stream = path.open("a+b")
    acquired = False
    try:
        if stream.tell() == 0:
            stream.write(b"0")
            stream.flush()
        stream.seek(0)
        try:
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            raise SessionError("Another live-tuning writer owns this OpenOCD endpoint") from exc
        acquired = True
        yield
    finally:
        if acquired:
            stream.seek(0)
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        stream.close()


class Session:
    def __init__(self, client, symbols, log, rate=25.0):
        self.client, self.symbols, self.log = client, symbols, log
        self.period = 1.0 / rate
        self.next_heartbeat = 0.0
        self.heartbeat = None
        self._firmware_verified = False
        self._initial_validated = False
        self.elf_path = None

    def verify_firmware(self, elf):
        self._firmware_verified = self._initial_validated = False
        self.heartbeat = None
        self.elf_path = Path(elf)
        image = self.elf_path.read_bytes()
        previous_timeout = self.client.timeout
        self.client.set_timeout(2.0)
        try:
            segments = verify_flash(self.client, image)
        finally:
            if getattr(self.client, "usable", True):
                self.client.set_timeout(previous_timeout)
        self._firmware_verified = True
        self.log.emit("firmware_verified", elf_sha256=hashlib.sha256(image).hexdigest(), segments=segments)

    @staticmethod
    def validate_snapshot(state):
        sample = state.get("sample", {})
        checks = (("phase", state.get("phase"), range(6)),
                  ("error", state.get("error"), range(9)),
                  ("sample.phase", sample.get("phase"), range(6)),
                  ("sample.error", sample.get("error"), range(9)),
                  ("sample.enabled", sample.get("enabled"), (0, 1)))
        for name, value, allowed in checks:
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value not in allowed:
                raise SessionError(f"Invalid mailbox {name}={value!r}; no initial RAM writes allowed")

    def validate_initial(self):
        if not self._firmware_verified:
            raise SessionError("Flash/ELF preflight has not passed; no RAM writes allowed")
        state = self.snapshot()
        self.validate_snapshot(state)
        self._initial_validated = True
        return state

    def read(self, name):
        return self.client.read_words(self.symbols["tuning_" + name])[0]

    def write(self, name, values):
        if not self._firmware_verified or not self._initial_validated:
            raise SessionError("Flash/ELF and initial mailbox preflight required before any RAM write")
        self.client.write_words(self.symbols["tuning_" + name], values)

    def beat(self, force=False):
        now = time.monotonic()
        if force or now >= self.next_heartbeat:
            if self.heartbeat is None:
                self.heartbeat = self.read("heartbeat")
            self.heartbeat = (self.heartbeat + 1) & 0xFFFFFFFF
            self.write("heartbeat", [self.heartbeat])
            self.next_heartbeat = time.monotonic() + 0.1

    def snapshot(self):
        names = (*SCALARS, "sample_seq", "sample", "sample_seq")
        # The firmware's 55-word mailbox is contiguous in the live ELF. A
        # single burst lets OpenOCD batch DAP reads, instead of setting up nine
        # separate reads. Its embedded sequence must precede the sample array.
        ranges = sorted((self.symbols[name], 40 if name == "tuning_sample" else
                         8 if name == "tuning_args" else 1) for name in SYMBOLS)
        contiguous = all(address + count * 4 == next_address
                         for (address, count), (next_address, _) in zip(ranges, ranges[1:]))
        burst = contiguous and self.symbols["tuning_sample_seq"] < self.symbols["tuning_sample"]
        if burst:
            base = ranges[0][0]
            count = sum(length for _, length in ranges)
            offsets = {name: (address - base) // 4 for name, address in self.symbols.items()
                       if name in SYMBOLS}
            command = (f"concat [read_memory {base:#x} 32 {count}] "
                       f"[read_memory {self.symbols['tuning_sample_seq']:#x} 32 1]")
        else:
            commands = [f"[read_memory {self.symbols['tuning_' + name]:#x} 32 "
                        f"{40 if name == 'sample' else 1}]" for name in names]
            command = "concat " + " ".join(commands)
        for _ in range(5):
            # Retrying a torn sample must not starve a moving session's lease.
            # A read-only session never starts a heartbeat and never writes RAM.
            if self.heartbeat is not None:
                self.beat()
            raw = self.client.parse_words(self.client.evaluate(command), count + 1 if burst else 48)
            if burst:
                sample_offset = offsets["tuning_sample"]
                words = [raw[offsets["tuning_" + name]] for name in SCALARS]
                words += [raw[offsets["tuning_sample_seq"]]]
                words += raw[sample_offset:sample_offset + 40] + [raw[-1]]
            else:
                words = raw
            if words[6] == words[-1] and words[6] % 2 == 0:
                sample = [word_float(value) for value in words[7:47]]
                invalid = [SAMPLE_NAMES[i] for i, value in enumerate(sample) if not math.isfinite(value)]
                result = dict(zip(SCALARS, words[:6]))
                result.update(sample_seq=words[6], sample=dict(zip(SAMPLE_NAMES,
                              [value if math.isfinite(value) else None for value in sample])),
                              nonfinite_fields=invalid)
                return result
        raise SessionError("Could not obtain a complete sample (seqlock changed)")

    def publish(self, command, args):
        sequence = (self.read("request_seq") + 1) & 0xFFFFFFFF
        self.beat(force=True)
        self.write("args", [float_word(value) for value in args])
        self.write("command", [command])
        self.write("request_seq", [sequence])
        self.log.emit("request", command=command, sequence=sequence, args=args)
        return sequence

    def wait_complete(self, sequence, timeout, stop=False):
        deadline = time.monotonic() + timeout
        acknowledged = False
        next_sample = 0.0
        while time.monotonic() < deadline:
            self.beat()
            if time.monotonic() >= next_sample:
                next_sample = time.monotonic() + self.period
                state = self.snapshot()
                self.log.emit("sample", **state)
                if state["ack_seq"] == sequence:
                    acknowledged = True
                    if state["phase"] == 5:
                        code = state["error"]
                        raise SessionError(f"Firmware fault {code}: {ERRORS.get(code, 'unknown')}")
                    if state["phase"] == 4 or (stop and state["phase"] == 0):
                        if state["error"]:
                            raise SessionError(f"Firmware completed with error {state['error']}")
                        self.log.emit("complete", sequence=sequence, **state)
                        return state
            wake = min(deadline, next_sample, self.next_heartbeat)
            time.sleep(max(0.0, wake - time.monotonic()))
        raise SessionError("Command timeout after " + ("acknowledgement" if acknowledged else "no acknowledgement"))

    def parameter_replies(self):
        if "dm_param_reply" not in self.symbols:
            return None
        words = self.client.read_words(self.symbols["dm_param_reply"], 12)
        replies = []
        for index in range(3):
            reg, bits, received_ms, sequence = words[index * 4:index * 4 + 4]
            reply = dict(motor_id=index + 1, register=reg, value_bits=bits,
                         received_ms=received_ms, sequence=sequence)
            if reg in (1, 21, 22, 23):
                value = word_float(bits)
                reply["value_float"] = value if math.isfinite(value) else None
            replies.append(reply)
        return replies

    def execute(self, command, args, timeout):
        # These checks precede try/finally: an unknown program or corrupt
        # mailbox must never receive even a speculative STOP or heartbeat.
        state = self.validate_initial()
        validate_request(command, args)
        self.log.emit("initial", **state)
        if command != 1 and state["phase"] in (1, 2, 3):
            raise SessionError("Firmware is already preparing/moving/braking")
        failure = None
        try:
            # Only a validated, actually attempted publish enables final STOP.
            sequence = self.publish(command, args)
            result = self.wait_complete(sequence, timeout, stop=command == 1)
            if command == 2:
                result["parameter_replies"] = self.parameter_replies()
                self.log.emit("parameter_replies", replies=result["parameter_replies"])
            return result
        except BaseException as exc:
            failure = exc
            self.log.emit("failure", message=str(exc), kind=type(exc).__name__)
            raise
        finally:
            try:
                # A timeout leaves the old Tcl reply indeterminate. Reconnect
                # only for STOP; never retry a motion command.
                if not getattr(self.client, "usable", True):
                    self.client.reconnect()
                    self.verify_firmware(self.elf_path)
                    self.validate_initial()
                sequence = self.publish(1, [0.0] * 8)
                self.wait_complete(sequence, 2.5, stop=True)
                self.log.emit("final_stop_confirmed")
            except BaseException as exc:
                self.log.emit("final_stop_unconfirmed", message=str(exc))
                if failure is None:
                    raise SessionError("Final STOP not confirmed: " + str(exc)) from exc


def validate_request(command, args):
    if len(args) != 8 or not all(math.isfinite(value) for value in args):
        raise SessionError("Command requires eight finite arguments")
    valid = command == 1 and all(value == 0 for value in args)
    if command == 2:
        valid = 1 <= args[0] <= 35 and args[0] == int(args[0])
    elif command == 3:
        valid = (all(0.05 <= value <= 0.5 for value in args[:3]) and
                 0 <= args[3] <= 1 and all(0 <= value <= 0.03 for value in args[4:7]))
    elif command == 4:
        valid = (abs(args[0]) <= 0.25 and args[1] == 0 and abs(args[2]) <= 0.5 and
                 ((args[0] != 0) != (args[2] != 0)) and
                 100 <= args[3] <= 1500 and args[3] == int(args[3]))
    if not valid:
        raise SessionError("Command/arguments are outside the firmware tuning limits")


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, required=True, help="Exact ELF flashed on board")
    parser.add_argument("--nm", help="Path to arm-none-eabi-nm")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6666)
    parser.add_argument("--target", default="stm32f4x.cpu")
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--describe", action="store_true", help="Resolve symbols offline")
    action.add_argument("--read", action="store_true", help="Read RAM only; never writes STOP/heartbeat")
    action.add_argument("--command", type=int, choices=(1, 2, 3, 4), help="1 stop, 2 read parameters, 3 configure, 4 move")
    parser.add_argument("--read-seconds", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=25.0)
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument("--reg", type=int, default=1)
    parser.add_argument("--kd", type=float, nargs=3, default=[0.2, 0.25, 0.2])
    parser.add_argument("--ff-gain", type=float, default=0.0)
    parser.add_argument("--friction", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--duration-ms", type=int, default=800)
    parser.add_argument("--log", type=Path)
    args = parser.parse_args(argv)
    numeric = [args.rate, args.timeout, args.read_seconds, args.vx, args.vy, args.wz,
               args.ff_gain, *args.kd, *args.friction]
    if not all(math.isfinite(value) for value in numeric):
        parser.error("All numeric values must be finite")
    if not 20 <= args.rate <= 25 or not 0 < args.timeout <= 15 or not 0 <= args.read_seconds <= 300:
        parser.error("rate must be 20..25 Hz, timeout (0,15] s, read-seconds 0..300")
    if not re.fullmatch(r"[A-Za-z0-9_.:-]+", args.target):
        parser.error("Invalid OpenOCD target name")
    if not 1 <= args.port <= 65535 or not 1 <= args.reg <= 35:
        parser.error("Invalid port or parameter register")
    if abs(args.vx) > 0.25 or args.vy != 0 or abs(args.wz) > 0.5 or not 100 <= args.duration_ms <= 1500:
        parser.error("Motion requires |vx|<=0.25, vy=0, |wz|<=0.5, duration 100..1500 ms")
    if args.vx != 0 and args.wz != 0:
        parser.error("Use pure translation or pure rotation, not both in one test")
    if any(not 0.05 <= value <= 0.5 for value in args.kd) or not 0 <= args.ff_gain <= 1:
        parser.error("Kd must be 0.05..0.5, ff-gain 0..1")
    if any(not 0 <= value <= 0.03 for value in args.friction):
        parser.error("Friction torque must be 0..0.03 N m per wheel")
    if args.command == 4 and args.vx == 0 and args.wz == 0:
        parser.error("Move requires a nonzero vx or wz; use command 1 to stop")
    return args


def command_values(args):
    if args.command == 2:
        return [float(args.reg)] + [0.0] * 7
    if args.command == 3:
        return [*args.kd, args.ff_gain, *args.friction, 0.0]
    if args.command == 4:
        return [args.vx, args.vy, args.wz, float(args.duration_ms), 0.0, 0.0, 0.0, 0.0]
    return [0.0] * 8


def main(argv=None):
    args = arguments(argv)
    client = log = None
    try:
        symbols = load_symbols(args.elf, args.nm)
        if args.describe:
            print(json.dumps({key: f"0x{value:08x}" for key, value in symbols.items()}, indent=2))
            return 0
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        path = args.log or Path.cwd() / "outputs/live_tuning" / f"swd_{stamp}.jsonl"
        log = JsonLog(path)
        log.emit("session", elf=str(args.elf.resolve()), symbols=symbols,
                 host=args.host, port=args.port, command=args.command,
                 utc=datetime.now(timezone.utc).isoformat())
        client = TclClient(args.host, args.port)
        if client.evaluate(args.target + " curstate") != "running":
            raise SessionError("MCU is not running; this tool will not halt/reset/resume it")
        session = Session(client, symbols, log, args.rate)
        print(f"JSONL: {path.resolve()}", flush=True)
        if args.read:
            session.verify_firmware(args.elf)
            deadline = time.monotonic() + args.read_seconds
            while True:
                state = session.snapshot()
                session.validate_snapshot(state)
                log.emit("sample", **state)
                if time.monotonic() >= deadline:
                    print(json.dumps(state, ensure_ascii=False, allow_nan=False))
                    break
                time.sleep(session.period)
        else:
            with writer_lock(args.host, args.port):
                session.verify_firmware(args.elf)
                state = session.execute(args.command, command_values(args), args.timeout)
                print(json.dumps(state, ensure_ascii=False, allow_nan=False))
        return 0
    except (SessionError, OSError, subprocess.SubprocessError, KeyboardInterrupt) as exc:
        if log:
            log.emit("session_error", message=str(exc), kind=type(exc).__name__)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if client:
            client.close()
        if log:
            log.close()


if __name__ == "__main__":
    raise SystemExit(main())
