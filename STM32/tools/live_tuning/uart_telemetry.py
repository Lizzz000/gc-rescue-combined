"""Read the live-tuning USART6 JustFloat stream; never sends serial commands."""
import math
import struct
import time
from pathlib import Path
import swd_session as swd


class FrameDecoder:
    TAIL = b"\x00\x00\x80\x7f"
    SIZE = 4 * len(swd.SAMPLE_NAMES)

    def __init__(self, names=swd.SAMPLE_NAMES):
        self.buffer = bytearray()
        self.names = tuple(names)
        self.SIZE = 4 * len(self.names)

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            end = self.buffer.find(self.TAIL)
            if end < 0:
                # Keep one complete candidate and a partial marker for resync.
                if len(self.buffer) > self.SIZE + 3:
                    del self.buffer[:-(self.SIZE + 3)]
                return frames
            if end >= self.SIZE:
                values = struct.unpack("<" + "f" * len(self.names),
                                       self.buffer[end-self.SIZE:end])
                sample = dict(zip(self.names, values))
                if (all(math.isfinite(x) for x in values) and
                    sample["phase"] in range(6) and sample["error"] in range(11) and
                    sample["enabled"] in (0, 1) and sample["odom_valid"] in (0, 1) and
                    0 <= sample["tick_ms"] <= 0xffffffff):
                    frames.append(sample)
            del self.buffer[:end + len(self.TAIL)]


class UartTelemetry:
    def __init__(self, port, capture_path, log):
        import serial  # Optional dependency, required only for connected visual tests.
        self.port = serial.Serial(port=None, baudrate=115200, timeout=.01)
        self.port.dtr = self.port.rts = False
        self.port.port = port
        self.capture = Path(capture_path).open("xb")
        self.decoder, self.log = FrameDecoder(), log
        try:
            self.port.open()
            self.discard()
        except BaseException:
            self.close()
            raise

    def discard(self):
        self.port.reset_input_buffer()
        self.decoder = FrameDecoder(self.decoder.names)

    def set_names(self, names):
        self.decoder = FrameDecoder(names)
        self.discard()

    def latest(self, timeout=.18):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            data = self.port.read(max(1, self.port.in_waiting))
            if not data:
                continue
            self.capture.write(data)
            self.capture.flush()
            frames = self.decoder.feed(data)
            for sample in frames:
                self.log.emit("uart_sample", sample=sample)
            if frames:
                return frames[-1]
        raise swd.SessionError("USART6 telemetry expired; no fresh image source")

    def close(self):
        self.port.close()
        self.capture.close()


def visual_snapshot(session, telemetry):
    """A complete serial frame plus a short SWD command/lease read.

    A 56-channel SWD read takes longer than the 40 ms publication period on
    some DAP probes. UART keeps the source fresh without relaxing the watchdog.
    The serial tick identifies its frame; sample_seq is explicitly unavailable.
    """
    session.guard()
    fields = sorted((session.symbols["tuning_" + key], key) for key in swd.SCALARS)
    base = fields[0][0]
    if any(address != base + i * 4 for i, (address, _) in enumerate(fields)):
        raise swd.SessionError("Visual UART mode requires a contiguous scalar mailbox")
    # Some USB-UART bridges retain a few old frames across reopening even
    # after the OS receive buffer is purged. Compare against the live board
    # publication tick before using any received pose as an image source.
    deadline = time.monotonic() + .18
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise swd.SessionError("USART6 frames do not match the live board clock")
        sample = telemetry.latest(timeout=remaining)
        # Waiting for the 100 ms return frame must not consume most of the
        # 300 ms lease before a potentially delayed SWD diagnostic read.
        # Read-only sessions still never start a heartbeat or write RAM.
        if session.heartbeat is not None:
            session.beat()
        words = session.client.parse_words(session.client.evaluate(
            f"concat [read_memory {base:#x} 32 {len(fields)}] "
            f"[read_memory {session.symbols['tuning_sample']:#x} 32 1]"), len(fields)+1)
        board_tick = swd.word_float(words[-1])
        if not math.isfinite(board_tick) or not 0 <= board_tick <= 0xffffffff:
            raise swd.SessionError("Invalid live board publication clock")
        age = (int(board_tick) - int(sample["tick_ms"])) & 0xffffffff
        if age <= 120:
            break
        session.log.emit("uart_stale_discarded", source_ms=sample["tick_ms"], board_ms=board_tick)
    state = {key: value for (_, key), value in zip(fields, words)}
    state.update(sample=sample, sample_seq=None, telemetry_tick_ms=int(sample["tick_ms"]),
                 nonfinite_fields=[], transport="USART6")
    session.validate_snapshot(state)
    return state
