"""CAN wire ABI tests independent of physical controller availability."""

import pathlib
import struct
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gmp_router.can import CanCapabilities, CanFrame, CanRxEvent, CanState, CanTxEvent


class CanCodecTests(unittest.TestCase):
    def test_fd_frame_round_trip(self) -> None:
        frame = CanFrame(0x18FF1234, 0x0D, bytes(range(64)))
        token, decoded = CanFrame.decode(frame.encode(17))
        self.assertEqual(token, 17)
        self.assertEqual(decoded, frame)

    def test_capabilities_layout_matches_c_structure(self) -> None:
        payload = struct.pack("<IHHHHHHII", 3, 32, 2, 28, 64, 32, 0,
                              1_000_000, 8_000_000)
        capabilities = CanCapabilities.decode(payload)
        self.assertEqual(capabilities.hardware_tx_slots, 32)
        self.assertEqual(capabilities.data_bitrate_max, 8_000_000)

    def test_state_layout(self) -> None:
        payload = struct.pack("<HHH2xIIII", 4, 255, 10, 1, 2, 3, 4)
        self.assertEqual(CanState.decode(payload).bus_off_count, 3)

    def test_async_events(self) -> None:
        rx = struct.pack("<IHHIHH", 99, 2, 1, 0x321, 0, 2) + b"\xaa\x55"
        self.assertEqual(CanRxEvent.decode(rx).frame.data, b"\xaa\x55")
        tx = struct.pack("<Ih2xI", 17, -3, 100)
        self.assertEqual(CanTxEvent.decode(tx).status, -3)


if __name__ == "__main__":
    unittest.main()
