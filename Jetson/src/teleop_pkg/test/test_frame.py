"""与逐位参考和线上黄金帧对照，不依赖串口或电机。"""
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from manual.frame import Parser, crc16, encode


def reference_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


class FrameAlgorithmTest(unittest.TestCase):
    def test_crc_exhaustive_two_bytes(self):
        self.assertEqual(crc16(b'123456789'), 0x4B37)
        for value in range(65536):
            data = value.to_bytes(2, 'little')
            self.assertEqual(crc16(data), reference_crc(data))

    def test_all_payload_lengths_and_types(self):
        for length in range(9):
            for msg_type in range(256):
                data = bytes((msg_type + i * 37) & 255 for i in range(length))
                packet = encode(msg_type, data)
                self.assertEqual(int.from_bytes(packet[-2:], 'little'), reference_crc(packet[1:-2]))
                self.assertEqual(Parser().feed(packet), [(msg_type, data)])

    def test_arbitrary_chunk_boundaries(self):
        expected = [(3, b'\xa5\x01\x05\x00'), (4, bytes(range(8))), (21, b'')]
        stream = b'\x00\xff\xa5\x02' + encode(*expected[0]) + b'\x11' * 100
        damaged = bytearray(encode(9, b'\x03\x03'))
        damaged[-1] ^= 1
        stream += damaged + encode(*expected[1]) + encode(*expected[2])
        for split in range(len(stream) + 1):
            parser = Parser()
            self.assertEqual(parser.feed(stream[:split]) + parser.feed(stream[split:]), expected)
        rng = random.Random(917)
        for _ in range(30):
            parser = Parser()
            received = []
            position = 0
            while position < len(stream):
                size = rng.randrange(1, 18)
                received.extend(parser.feed(stream[position:position + size]))
                position += size
            self.assertEqual(received, expected)
            self.assertEqual(parser.feed(b''), [])

    def test_each_single_bit_damage_then_recover(self):
        payload = bytes([1, 0xA5, 0xFF, 0, 3, 8, 127, 128])
        packet = encode(7, payload)
        for bit in range(len(packet) * 8):
            damaged = bytearray(packet)
            damaged[bit // 8] ^= 1 << (bit % 8)
            parser = Parser()
            self.assertEqual(parser.feed(damaged), [])
            self.assertEqual(parser.feed(packet), [(7, payload)])


if __name__ == '__main__':
    unittest.main()
