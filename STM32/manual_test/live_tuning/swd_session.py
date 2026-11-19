"""Bounded live-tuning sessions over OpenOCD Tcl; never halts or resets the MCU.

Use the exact ELF flashed on the board. --describe is offline; all connected
sessions compare every ELF Flash load byte before accessing the mailbox.
Mutating sessions own one local writer lock, publish arguments before the
request sequence, renew the heartbeat, and confirm STOP or the requested HOLD.
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
SCALARS = ("request_seq", "ack_seq", "phase", "error", "heartbeat", "command")
SAMPLE_NAMES = (
    "tick_ms", "phase", "error", "enabled", "yaw_deg", "wz_imu",
    "body_vx", "body_vy", "body_wz", "pos_1", "pos_2", "pos_3",
    "vel_1", "vel_2", "vel_3", "torque_1", "torque_2", "torque_3",
    "state_1", "state_2", "state_3", "mos_temp_1", "mos_temp_2", "mos_temp_3",
    "target_1", "target_2", "target_3", "ff_1", "ff_2", "ff_3",
    "kd_1", "kd_2", "kd_3", "ff_gain", "friction_1", "friction_2",
    "friction_3", "battery", "imu_age_ms", "motion_elapsed_ms",
    "odom_x_m", "odom_y_m", "odom_yaw_rad", "odom_vx_mps", "odom_vy_mps",
    "odom_wheel_wz_rad_s", "odom_valid", "odom_age_ms",
    "end_x_m", "end_y_m", "end_yaw_rad", "end_valid", "end_stopped",
    "end_elapsed_ms", "end_progress_m", "end_target_distance_m",
)
SYMBOL_SIZES = {name: 4 for name in SYMBOLS}
SYMBOL_SIZES.update(tuning_args=32, tuning_sample=4 * len(SAMPLE_NAMES), dm_param_reply=48)
ERRORS = {
    1: "invalid command/argument", 2: "busy", 3: "prepare/IMU failure",
    4: "motion/controller failure", 5: "heartbeat/visual source expired", 6: "parameter timeout",
    7: "parameter transmit failure", 8: "stop timeout",
    9: "odometry/endpoint mismatch", 10: "motion deadline expired",
}


class SessionError(RuntimeError):
    pass


def float_word(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def word_float(value):
    return struct.unpack("<f", struct.pack("<I", value))[0]


def load_symbols(elf, nm=None, *, extra_sizes=None):
    nm = nm or shutil.which("arm-none-eabi-nm") or shutil.which("arm-none-eabi-nm.exe")
    if not nm:
        raise SessionError("arm-none-eabi-nm not found; pass --nm")
    result = subprocess.run([nm, "-S", "--defined-only", "--numeric-sort", str(elf)],
                            check=True, capture_output=True, text=True, timeout=15)
    extra_sizes = extra_sizes or {}
    sizes = {**SYMBOL_SIZES, **extra_sizes}
    symbols = {}
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) == 4 and fields[3] in sizes:
            name, size = fields[3], int(fields[1], 16)
            address = int(fields[0], 16)
            if name in symbols or size != sizes[name]:
                raise SessionError(f"ELF tuning ABI size/uniqueness mismatch: {name}, size={size}")
            if not 0x20000000 <= address < address + size <= 0x20020000 or address % 4:
                raise SessionError(f"{name} is not aligned F407 SRAM: {address:#x}")
            symbols[name] = address
    missing = (set(SYMBOLS) | extra_sizes.keys()) - symbols.keys()
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
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
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
    def __init__(self, client, symbols, log, rate=25.0, snapshot_reader=None):
        self.client, self.symbols, self.log = client, symbols, log
        self.period = 1.0 / rate
        self.next_heartbeat = 0.0
        self.heartbeat = None
        self._firmware_verified = False
        self._initial_validated = False
        self.elf_path = None
        self.snapshot_reader = snapshot_reader

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
                  ("error", state.get("error"), range(11)),
                  ("sample.phase", sample.get("phase"), range(6)),
                  ("sample.error", sample.get("error"), range(11)),
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
        if self.snapshot_reader is not None:
            return self.snapshot_reader(self)
        names = (*SCALARS, "sample_seq", "sample", "sample_seq")
        sample_count = len(SAMPLE_NAMES)
        # When the firmware's mailbox is contiguous in the live ELF, a
        # single burst lets OpenOCD batch DAP reads, instead of setting up nine
        # separate reads. Its embedded sequence must precede the sample array.
        ranges = sorted((self.symbols[name], sample_count if name == "tuning_sample" else
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
                        f"{sample_count if name == 'sample' else 1}]" for name in names]
            command = "concat " + " ".join(commands)
        for _ in range(5):
            # Retrying a torn sample must not starve a moving session's lease.
            # A read-only session never starts a heartbeat and never writes RAM.
            if self.heartbeat is not None:
                self.beat()
            raw = self.client.parse_words(self.client.evaluate(command), count + 1 if burst else sample_count + 8)
            if burst:
                sample_offset = offsets["tuning_sample"]
                words = [raw[offsets["tuning_" + name]] for name in SCALARS]
                words += [raw[offsets["tuning_sample_seq"]]]
                words += raw[sample_offset:sample_offset + sample_count] + [raw[-1]]
            else:
                words = raw
            if words[6] == words[-1] and words[6] % 2 == 0:
                sample = [word_float(value) for value in words[7:7 + sample_count]]
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

    def wait_complete(self, sequence, timeout, stop=False, expected_command=None):
        deadline = time.monotonic() + timeout
        acknowledged = False
        next_sample = 0.0
        terminal_sample_seq = None
        while time.monotonic() < deadline:
            self.beat()
            if time.monotonic() >= next_sample:
                next_sample = time.monotonic() + self.period
                state = self.snapshot()
                self.log.emit("sample", **state)
                if state["ack_seq"] == sequence:
                    acknowledged = True
                    if expected_command == 7 and state.get("command") != 7:
                        raise SessionError("Continuous goal command was replaced")
                    if state["phase"] == 5:
                        code = state["error"]
                        raise SessionError(f"Firmware fault {code}: {ERRORS.get(code, 'unknown')}")
                    if state["phase"] == 4 or (stop and state["phase"] == 0):
                        if state["error"]:
                            raise SessionError(f"Firmware completed with error {state['error']}")
                        # Scalars can change between 10 Hz publications. Wait
                        # for a new terminal sample, including endpoint evidence,
                        # rather than returning a previous command's DONE frame.
                        identity = state.get("telemetry_tick_ms", state["sample_seq"])
                        if terminal_sample_seq is None:
                            terminal_sample_seq = identity
                        sample = state["sample"]
                        if (identity != terminal_sample_seq and
                            sample.get("phase") == state["phase"] and
                            sample.get("error") == 0 and sample.get("enabled") == (1 if expected_command == 7 else 0)):
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
        timeout_limit = 25 if command in (6, 7) else 15
        if not math.isfinite(timeout) or not 0 < timeout <= timeout_limit:
            raise SessionError(f"Command timeout must be finite and in (0,{timeout_limit}] seconds")
        if command == 6 and not timeout > args[3] / 1000.0 + 2.0:
            raise SessionError("Distance timeout must exceed firmware duration plus 2 seconds")
        if command == 7 and not timeout > args[5] / 1000.0 + 2.0:
            raise SessionError("Continuous goal timeout must exceed firmware duration plus 2 seconds")
        self.log.emit("initial", **state)
        if command != 1 and state["phase"] in (1, 2, 3):
            raise SessionError("Firmware is already preparing/moving/braking")
        failure = None
        try:
            # Only a validated, actually attempted publish enables final STOP.
            sequence = self.publish(command, args)
            result = (self.wait_complete(sequence, timeout, expected_command=7) if command == 7 else
                      self.wait_complete(sequence, timeout, stop=command == 1))
            if command in (5, 6, 7):
                validate_endpoint(command, args, result)
                self.log.emit("endpoint_verified", **{**result, "command": command})
                if command == 7:
                    self.log.emit("enabled_hold_preserved")
            if command == 1:
                self.log.emit("final_stop_confirmed")
            if command == 2:
                result["parameter_replies"] = self.parameter_replies()
                self.log.emit("parameter_replies", replies=result["parameter_replies"])
            return result
        except BaseException as exc:
            failure = exc
            self.log.emit("failure", message=str(exc), kind=type(exc).__name__)
            raise
        finally:
            # Commands 1 and 7 already verified their requested STOP or HOLD.
            if failure is not None or command not in (1, 7):
                try:
                    # Reconnect only for STOP; never retry a motion command.
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


def validate_endpoint(command, args, result):
    """DONE alone is insufficient evidence for an odometry acceptance test."""
    sample = result.get("sample", {})
    names = ("end_x_m", "end_y_m", "end_yaw_rad", "end_elapsed_ms",
             "end_progress_m", "end_target_distance_m")
    if sample.get("end_valid") != 1 or sample.get("end_stopped") != 1:
        raise SessionError("Odometry endpoint lacks valid pose/stopped evidence")
    if any(not isinstance(sample.get(name), (int, float)) or not math.isfinite(sample[name])
           for name in names) or sample["end_elapsed_ms"] < 0:
        raise SessionError("Odometry endpoint contains invalid numeric evidence")
    if command == 6:
        target = args[1]
        # The mailbox contains float32 metres; 1 um is sufficient numerical
        # margin here, separate from the controller's 3 mm arrival tolerance.
        if (not math.isclose(sample["end_target_distance_m"], target, rel_tol=1e-6, abs_tol=1e-6) or
            abs(sample["end_progress_m"] - target) > 0.003 + 1e-6):
            raise SessionError("Odometry endpoint does not match requested distance")
    elif command in (7, 8):
        if (result.get("command") != command or result.get("phase") != 4 or result.get("error") != 0 or
            sample.get("phase") != 4 or sample.get("error") != 0 or sample.get("enabled") != 1 or
            sample.get("odom_valid") != 1 or any(sample.get(f"state_{i}") != 1 for i in (1, 2, 3)) or
            any(sample.get(f"target_{i}") != 0 for i in (1, 2, 3))):
            raise SessionError("Continuous goal lacks enabled zero-target HOLD evidence")
        tolerance = .035 if command == 8 else .02
        if (math.hypot(sample["end_x_m"] - args[0], sample["end_y_m"] - args[1]) > tolerance + 1e-6 or
            abs(math.remainder(sample["end_yaw_rad"] - args[2], 2 * math.pi)) > math.radians(10) + 1e-6):
            raise SessionError("Continuous goal endpoint is outside position/heading tolerance")


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
    elif command == 5:
        valid = (all(value == 0 for value in (*args[:3], *args[4:])) and
                 100 <= args[3] <= 1500 and args[3] == int(args[3]))
    elif command == 6:
        valid = (args[0] in (1, 2) and 0.01 <= args[1] <= 1.0 and
                 0.02 <= args[2] <= 0.10 and 100 <= args[3] <= 20000 and
                 args[3] == int(args[3]) and all(value == 0 for value in args[4:]))
    elif command == 7:
        valid = (-math.pi <= args[2] <= math.pi and .02 <= args[3] <= .20 and
                 .10 <= args[4] <= .50 and 100 <= args[5] <= 20000 and
                 args[5] == int(args[5]) and args[6] == 0 and args[7] == 0)
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
    action.add_argument("--command", type=int, choices=(1, 2, 3, 4, 5, 6, 7),
                        help="1 stop, 2 read parameters, 3 configure, 4 timed move, 5 static odometry, 6 distance, 7 world goal/hold enabled")
    parser.add_argument("--read-seconds", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=25.0)
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument("--reg", type=int, default=1)
    parser.add_argument("--kd", type=float, nargs=3, default=[0.22, 0.27, 0.22])
    parser.add_argument("--ff-gain", type=float, default=0.0)
    parser.add_argument("--friction", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--duration-ms", type=int, default=800)
    parser.add_argument("--direction", type=int, choices=(1, 2), help="Distance direction: 1 forward, 2 backward")
    parser.add_argument("--distance-m", type=float, help="Distance target in metres (0.01..1.0)")
    parser.add_argument("--speed-mps", type=float, help="Distance speed limit in m/s (0.02..0.10)")
    parser.add_argument("--goal-x-m", type=float)
    parser.add_argument("--goal-y-m", type=float)
    parser.add_argument("--goal-yaw-rad", type=float)
    parser.add_argument("--linear-limit", type=float, default=.20)
    parser.add_argument("--angular-limit", type=float, default=.50)
    parser.add_argument("--log", type=Path)
    args = parser.parse_args(argv)
    numeric = [args.rate, args.timeout, args.read_seconds, args.vx, args.vy, args.wz,
               args.ff_gain, *args.kd, *args.friction]
    numeric += [value for value in (args.distance_m, args.speed_mps, args.goal_x_m, args.goal_y_m,
                                    args.goal_yaw_rad, args.linear_limit, args.angular_limit) if value is not None]
    if not all(math.isfinite(value) for value in numeric):
        parser.error("All numeric values must be finite")
    timeout_limit = 25 if args.command in (6, 7) else 15
    if not 20 <= args.rate <= 25 or not 0 < args.timeout <= timeout_limit or not 0 <= args.read_seconds <= 300:
        parser.error(f"rate must be 20..25 Hz, timeout (0,{timeout_limit}] s, read-seconds 0..300")
    if not re.fullmatch(r"[A-Za-z0-9_.:-]+", args.target):
        parser.error("Invalid OpenOCD target name")
    if not 1 <= args.port <= 65535 or not 1 <= args.reg <= 35:
        parser.error("Invalid port or parameter register")
    duration_limit = 20000 if args.command in (6, 7) else 1500
    if abs(args.vx) > 0.25 or args.vy != 0 or abs(args.wz) > 0.5 or not 100 <= args.duration_ms <= duration_limit:
        parser.error(f"Motion requires |vx|<=0.25, vy=0, |wz|<=0.5, duration 100..{duration_limit} ms")
    if args.vx != 0 and args.wz != 0:
        parser.error("Use pure translation or pure rotation, not both in one test")
    if any(not 0.05 <= value <= 0.5 for value in args.kd) or not 0 <= args.ff_gain <= 1:
        parser.error("Kd must be 0.05..0.5, ff-gain 0..1")
    if any(not 0 <= value <= 0.03 for value in args.friction):
        parser.error("Friction torque must be 0..0.03 N m per wheel")
    if args.command == 4 and args.vx == 0 and args.wz == 0:
        parser.error("Move requires a nonzero vx or wz; use command 1 to stop")
    if args.command in (5, 6, 7) and (args.vx != 0 or args.vy != 0 or args.wz != 0):
        parser.error("Odometry commands do not accept vx/vy/wz")
    if args.command == 6:
        if args.direction is None or args.distance_m is None or args.speed_mps is None:
            parser.error("Distance requires --direction, --distance-m and --speed-mps")
        if not args.timeout > args.duration_ms / 1000.0 + 2.0:
            parser.error("Distance --timeout must exceed --duration-ms / 1000 + 2 seconds")
    elif any(value is not None for value in (args.direction, args.distance_m, args.speed_mps)):
        parser.error("Direction/distance/speed options require command 6")
    if args.command == 7:
        if any(value is None for value in (args.goal_x_m, args.goal_y_m, args.goal_yaw_rad)):
            parser.error("Continuous goal requires --goal-x-m, --goal-y-m and --goal-yaw-rad")
        if not args.timeout > args.duration_ms / 1000.0 + 2.0:
            parser.error("Continuous goal --timeout must exceed duration plus 2 seconds")
    elif any(value is not None for value in (args.goal_x_m, args.goal_y_m, args.goal_yaw_rad)):
        parser.error("World goal options require command 7")
    if args.command is not None:
        try:
            validate_request(args.command, command_values(args))
        except SessionError as exc:
            parser.error(str(exc))
    return args


def command_values(args):
    if args.command == 2:
        return [float(args.reg)] + [0.0] * 7
    if args.command == 3:
        return [*args.kd, args.ff_gain, *args.friction, 0.0]
    if args.command == 4:
        return [args.vx, args.vy, args.wz, float(args.duration_ms), 0.0, 0.0, 0.0, 0.0]
    if args.command == 5:
        return [0.0, 0.0, 0.0, float(args.duration_ms), 0.0, 0.0, 0.0, 0.0]
    if args.command == 6:
        return [float(args.direction), args.distance_m, args.speed_mps, float(args.duration_ms),
                0.0, 0.0, 0.0, 0.0]
    if args.command == 7:
        return [args.goal_x_m, args.goal_y_m, args.goal_yaw_rad, args.linear_limit,
                args.angular_limit, float(args.duration_ms), 0., 0.]
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
