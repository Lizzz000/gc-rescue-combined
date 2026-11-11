import math
import struct
import unittest

import s288_protocol as protocol


class S288ProtocolTest(unittest.TestCase):
    def test_vendor_control_vector(self):
        packet = protocol.control_packet(2, 1, 1, 0.1, 0.2, 0.3, 1.0, 0.1)
        self.assertEqual(packet.hex(), 'feee9200590017004ae206000f009a005e72f7ee')

    def test_negative_position_is_signed(self):
        packet = protocol.control_packet(0, 1, 1, 0, 0, -0.3, 1, 0.1)
        self.assertLess(struct.unpack_from('<i', packet, 8)[0], 0)
        self.assertEqual(packet.hex(), 'feee900000000000b61df9ff0f009a00800c6ba0')

    def test_feedback_and_corruption(self):
        body = struct.pack('<BbBBhhiIHBB', 0x12, -5, 200, 25, -100, 30, -1234, 4, 0xA123, 0, 0)
        packet = b'\xfc\xee' + body + struct.pack('<I', protocol.crc32(body))
        feedback = protocol.parse_feedback(packet, 2)
        self.assertEqual(feedback['winding_temperature'], 200)
        self.assertEqual(feedback['voltage'], 12.5)
        self.assertEqual(feedback['error'], 4)
        self.assertAlmostEqual(feedback['position'], -1234 * 2 * math.pi / (32768 * protocol.RATIO))
        with self.assertRaises(ValueError):
            protocol.parse_feedback(packet, 3)
        corrupt = bytearray(packet)
        corrupt[8] ^= 1
        with self.assertRaises(ValueError):
            protocol.parse_feedback(corrupt, 2)

    def test_reject_bad_inputs(self):
        for values in ((15, 1, 1, 0, 0, 0, 0, 0), (0, 1, 1, 0, 0, math.nan, 0, 0),
                       (0, 1, 1, 1e9, 0, 0, 0, 0)):
            with self.assertRaises(ValueError):
                protocol.control_packet(*values)


if __name__ == '__main__':
    unittest.main()
