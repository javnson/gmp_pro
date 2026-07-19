"""Compatibility tests for the GMP peripheral router wire format."""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gmp_router.protocol import MessageType, Packet, Peripheral, cobs_decode, cobs_encode, crc16


class ProtocolTests(unittest.TestCase):
    def test_crc_standard_vector(self) -> None:
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_packet_golden_vector(self) -> None:
        packet = Packet(MessageType.REQUEST, 0x12345678, Peripheral.GPIO, 2,
                        endpoint=3, channel=25, payload=b"\x01")
        self.assertEqual(packet.encode().hex(),
                         "52470101785634120102030019000000010001cc7a")
        self.assertEqual(Packet.decode(packet.encode()), packet)

    def test_cobs_round_trip(self) -> None:
        source = bytes(range(256)) + b"\x00tail"
        self.assertEqual(cobs_decode(cobs_encode(source)), source)

    def test_bad_crc_is_rejected(self) -> None:
        encoded = bytearray(Packet(1, 1, 0, 3).encode())
        encoded[-1] ^= 1
        with self.assertRaises(ValueError):
            Packet.decode(encoded)


if __name__ == "__main__":
    unittest.main()
