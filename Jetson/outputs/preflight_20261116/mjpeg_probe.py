#!/usr/bin/env python3
"""Read-only multipart MJPEG probe; all reported receive times use the client clock."""
import argparse
import datetime
import email.message
import http.client
import io
import json
import math
from pathlib import Path
import queue
import socket
import ssl
import statistics
import threading
import time
import urllib.parse


METRICS = ('X-Frame-Age-At-Encode-Ms', 'X-Frame-Age-At-Publish-Ms', 'X-Jpeg-Encode-Ms')
MAX_JPEG_BYTES = 32 * 1024 * 1024
MAX_HEADER_BYTES = 64 * 1024


class DurationComplete(Exception):
    pass


class FrameTimeout(Exception):
    pass


class StreamEnded(Exception):
    pass


class InvalidMultipart(Exception):
    pass


class Budget:
    def __init__(self, duration, timeout):
        self.started = time.monotonic()
        self.deadline = self.started + duration
        self.timeout = timeout
        self.frame_deadline = self.started + timeout

    def remaining(self):
        now = time.monotonic()
        if now >= self.deadline:
            raise DurationComplete()
        if now >= self.frame_deadline:
            raise FrameTimeout('No complete JPEG within the frame timeout')
        return min(self.deadline, self.frame_deadline) - now

    def timed_out(self):
        if time.monotonic() >= self.deadline - 0.001:
            raise DurationComplete()
        raise FrameTimeout('Connection/read timed out before another complete JPEG')

    def complete_frame(self, now):
        self.frame_deadline = now + self.timeout


class BoundedSocketReader(io.RawIOBase):
    def __init__(self, sock, budget):
        super().__init__()
        self.sock, self.budget = sock, budget
        self.received = 0

    def readable(self):
        return True

    def readinto(self, buffer):
        self.sock.settimeout(self.budget.remaining())
        try:
            count = self.sock.recv_into(buffer)
        except (TimeoutError, socket.timeout):
            self.budget.timed_out()
        self.received += count
        return count


class SocketFacade:
    def __init__(self, raw):
        self.raw = raw

    def makefile(self, mode):
        if mode != 'rb':
            raise ValueError('Only binary response reads are supported')
        return io.BufferedReader(self.raw, buffer_size=65536)


def connect(parsed, budget):
    """Bound DNS waiting, each connect attempt, and TLS negotiation by the same deadline."""
    addresses = queue.Queue(maxsize=1)

    def resolve():
        try:
            addresses.put((socket.getaddrinfo(parsed.hostname, parsed.port or
                           (443 if parsed.scheme == 'https' else 80), type=socket.SOCK_STREAM), None))
        except Exception as error:
            addresses.put((None, error))

    threading.Thread(target=resolve, daemon=True).start()
    try:
        choices, error = addresses.get(timeout=budget.remaining())
    except queue.Empty:
        budget.timed_out()
    if error is not None:
        raise error
    last_error = None
    for family, kind, protocol, _, address in choices:
        sock = socket.socket(family, kind, protocol)
        try:
            sock.settimeout(budget.remaining())
            sock.connect(address)
            if parsed.scheme == 'https':
                sock.settimeout(budget.remaining())
                sock = ssl.create_default_context().wrap_socket(sock, server_hostname=parsed.hostname)
            return sock
        except (TimeoutError, socket.timeout):
            sock.close()
            budget.timed_out()
        except Exception as error:
            sock.close()
            last_error = error
    raise last_error or OSError('No address resolved')


class MultipartInput:
    def __init__(self, response, budget):
        self.response, self.budget = response, budget
        self.buffer = bytearray()

    def fill(self):
        self.budget.remaining()
        chunk = self.response.read1(65536)
        if not chunk:
            raise StreamEnded('HTTP EOF before the requested duration ended')
        self.buffer.extend(chunk)

    def line(self, maximum=MAX_HEADER_BYTES):
        while True:
            position = self.buffer.find(b'\n')
            if position >= 0:
                if position + 1 > maximum:
                    raise InvalidMultipart('Multipart header line too large')
                result = bytes(self.buffer[:position + 1])
                del self.buffer[:position + 1]
                return result.rstrip(b'\r\n')
            if len(self.buffer) >= maximum:
                raise InvalidMultipart('Multipart header line too large')
            self.fill()

    def exact(self, length):
        while len(self.buffer) < length:
            self.fill()
        result = bytes(self.buffer[:length])
        del self.buffer[:length]
        return result


def read_part(source, boundary):
    boundary_line = b'--' + boundary
    preamble_bytes = 0
    while True:
        line = source.line()
        if line == boundary_line:
            break
        if line == boundary_line + b'--':
            raise StreamEnded('Multipart closing boundary before the requested duration ended')
        preamble_bytes += len(line) + 2
        if preamble_bytes > MAX_HEADER_BYTES:
            raise InvalidMultipart('Multipart boundary not found within 64 KiB')
    headers = {}
    header_bytes = 0
    while True:
        line = source.line()
        header_bytes += len(line) + 2
        if header_bytes > MAX_HEADER_BYTES:
            raise InvalidMultipart('Multipart headers exceed 64 KiB')
        if not line:
            break
        if b':' not in line:
            raise InvalidMultipart('Malformed multipart header')
        key, value = line.split(b':', 1)
        key = key.decode('ascii').strip().lower()
        if key in headers:
            raise InvalidMultipart('Duplicate multipart header: ' + key)
        headers[key] = value.decode('latin-1').strip()
    try:
        length = int(headers['content-length'])
    except (KeyError, ValueError) as error:
        raise InvalidMultipart('A valid Content-Length is required for each JPEG') from error
    if not 4 <= length <= MAX_JPEG_BYTES:
        raise InvalidMultipart('JPEG Content-Length outside 4..32 MiB')
    if headers.get('content-type', '').lower().split(';')[0] != 'image/jpeg':
        raise InvalidMultipart('Multipart part is not image/jpeg')
    jpeg = source.exact(length)
    if not (jpeg.startswith(b'\xff\xd8') and jpeg.endswith(b'\xff\xd9')):
        raise InvalidMultipart('Content-Length payload lacks complete JPEG SOI/EOI markers')
    return headers, len(jpeg)


def finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def aggregate(values):
    values = sorted(value for value in values if value is not None)
    if not values:
        return {'count': 0, 'min': None, 'mean': None, 'p50': None, 'p95': None, 'max': None}

    def percentile(q):
        location = (len(values) - 1) * q
        low = int(location)
        high = min(low + 1, len(values) - 1)
        return values[low] + (values[high] - values[low]) * (location - low)

    return {'count': len(values), 'min': values[0], 'mean': statistics.fmean(values),
            'p50': percentile(0.50), 'p95': percentile(0.95), 'max': values[-1]}


def probe(url, duration, timeout):
    budget = Budget(duration, timeout)
    result = {'schema_version': 1, 'url': url,
              'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'requested_duration_s': duration, 'frame_timeout_s': timeout,
              'status': 'error', 'exit_reason': None, 'error': None,
              'http_status': None, 'http_content_type': None, 'boundary': None, 'frames': []}
    sock = response = raw = None
    highest_sequence = None
    seen_sequences = set()
    sequence_stats = {'valid_frames': 0, 'missing_or_invalid_header_frames': 0,
                      'forward_gap_events': 0, 'forward_gap_sequence_slots': 0,
                      'out_of_order_frames': 0, 'duplicate_frames': 0}
    try:
        parsed = urllib.parse.urlsplit(url)
        if parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.username is not None:
            raise ValueError('URL must be HTTP(S), with a host and without embedded credentials')
        sock = connect(parsed, budget)
        host = ('[' + parsed.hostname + ']') if ':' in parsed.hostname else parsed.hostname
        if parsed.port is not None:
            host += ':' + str(parsed.port)
        target = (parsed.path or '/') + (('?' + parsed.query) if parsed.query else '')
        if any(character in target for character in '\r\n'):
            raise ValueError('Invalid request target')
        request = ('GET ' + target + ' HTTP/1.1\r\nHost: ' + host +
                   '\r\nAccept: multipart/x-mixed-replace\r\nConnection: close\r\n\r\n').encode('ascii')
        sock.settimeout(budget.remaining())
        sock.sendall(request)
        raw = BoundedSocketReader(sock, budget)
        response = http.client.HTTPResponse(SocketFacade(raw))
        response.begin()
        result['http_status'] = response.status
        result['http_content_type'] = response.getheader('Content-Type', '')
        if response.status != 200:
            result['exit_reason'] = 'http_error'
            raise InvalidMultipart('Expected HTTP 200, got ' + str(response.status))
        content_type = email.message.Message()
        content_type['Content-Type'] = result['http_content_type']
        if content_type.get_content_type().lower() != 'multipart/x-mixed-replace':
            raise InvalidMultipart('Response is not multipart/x-mixed-replace')
        boundary = content_type.get_param('boundary')
        if not isinstance(boundary, str) or not 1 <= len(boundary) <= 200 or '\r' in boundary or '\n' in boundary:
            raise InvalidMultipart('Missing or invalid multipart boundary')
        result['boundary'] = boundary
        source = MultipartInput(response, budget)
        encoded_boundary = boundary.encode('ascii')
        while True:
            budget.remaining()
            headers, jpeg_bytes = read_part(source, encoded_boundary)
            now = time.monotonic()
            budget.complete_frame(now)
            sequence = None
            try:
                sequence = int(headers.get('x-frame-sequence', ''))
                if sequence < 0:
                    sequence = None
            except ValueError:
                pass
            event = None
            if sequence is None:
                sequence_stats['missing_or_invalid_header_frames'] += 1
            else:
                sequence_stats['valid_frames'] += 1
                if sequence in seen_sequences:
                    sequence_stats['duplicate_frames'] += 1
                    event = 'duplicate'
                elif highest_sequence is not None and sequence < highest_sequence:
                    sequence_stats['out_of_order_frames'] += 1
                    event = 'out_of_order'
                elif highest_sequence is not None and sequence > highest_sequence + 1:
                    sequence_stats['forward_gap_events'] += 1
                    sequence_stats['forward_gap_sequence_slots'] += sequence - highest_sequence - 1
                    event = 'forward_gap'
                seen_sequences.add(sequence)
                highest_sequence = sequence if highest_sequence is None else max(highest_sequence, sequence)
            result['frames'].append({'index': len(result['frames']), 'receive_complete_elapsed_s': now - budget.started,
                'sequence': sequence, 'sequence_event': event, 'jpeg_bytes': jpeg_bytes,
                'server_metric_raw': {name: headers.get(name.lower()) for name in METRICS},
                'server_metric_ms': {name: finite_float(headers.get(name.lower())) for name in METRICS},
                'source_stamp_ns_raw': headers.get('x-source-stamp-ns')})
    except DurationComplete:
        result['exit_reason'] = 'duration_elapsed'
        result['status'] = 'ok' if result['frames'] else 'error'
        if not result['frames']:
            result['error'] = 'No complete JPEG received during the measurement window'
    except FrameTimeout as error:
        result['exit_reason'], result['error'] = 'frame_timeout', str(error)
    except StreamEnded as error:
        result['exit_reason'], result['error'] = 'eof', str(error)
    except (TimeoutError, socket.timeout) as error:
        if time.monotonic() >= budget.deadline - 0.001:
            result['exit_reason'] = 'duration_elapsed'
            result['status'] = 'ok' if result['frames'] else 'error'
            result['error'] = None if result['frames'] else 'No complete JPEG received'
        else:
            result['exit_reason'], result['error'] = 'frame_timeout', str(error)
    except InvalidMultipart as error:
        result['exit_reason'] = result['exit_reason'] or 'invalid_multipart'
        result['error'] = str(error)
    except Exception as error:
        result['exit_reason'], result['error'] = 'connection_or_read_error', type(error).__name__ + ': ' + str(error)
    finally:
        if response is not None:
            response.close()
        if sock is not None:
            sock.close()
    elapsed = time.monotonic() - budget.started
    frames = result['frames']
    times = [frame['receive_complete_elapsed_s'] for frame in frames]
    intervals = [(right - left) * 1000 for left, right in zip(times, times[1:])]
    span = times[-1] - times[0] if len(times) >= 2 else None
    result.update({'elapsed_total_window_s': elapsed, 'complete_frames': len(frames),
        'first_complete_elapsed_s': times[0] if times else None,
        'last_complete_elapsed_s': times[-1] if times else None,
        'fps_first_to_last': (len(frames) - 1) / span if span is not None and span > 0 else None,
        'fps_total_window': len(frames) / elapsed if elapsed > 0 else None,
        'receive_interval_ms': aggregate(intervals),
        'jpeg_bytes': {**aggregate([frame['jpeg_bytes'] for frame in frames]),
                       'total': sum(frame['jpeg_bytes'] for frame in frames)},
        'socket_received_bytes_including_http_headers': raw.received if raw is not None else 0,
        'sequence': sequence_stats,
        'server_metric_ms': {name: aggregate([frame['server_metric_ms'][name] for frame in frames]) for name in METRICS},
        'definitions': {
            'fps_first_to_last': '(complete_frames - 1) / seconds between first and last complete JPEG',
            'fps_total_window': 'complete_frames / client elapsed time including DNS/connect/HTTP and final wait',
            'receive_interval_ms': 'Client parser completion intervals; buffered/burst delivery may shorten intervals',
            'server_metric_ms': 'Server-reported frame age/encode time, NOT end-to-end or network latency',
            'forward_gap_sequence_slots': 'Missing slots at each new high-water sequence jump; not a proven network drop count',
            'frame_timeout_s': 'Maximum wait for a complete JPEG, including initial connection/headers; slow partial data does not reset it'}})
    return result


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('Must be finite and positive')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True)
    parser.add_argument('--duration', type=positive, default=15.0)
    parser.add_argument('--timeout', type=positive, default=3.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    result = probe(args.url, args.duration, args.timeout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({key: result[key] for key in ('status', 'exit_reason', 'error', 'complete_frames',
        'elapsed_total_window_s', 'fps_first_to_last', 'fps_total_window')}, ensure_ascii=False))
    return 0 if result['status'] == 'ok' else 1


if __name__ == '__main__':
    raise SystemExit(main())
