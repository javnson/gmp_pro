"""Host API tests using an in-memory router transport."""

import pathlib
import queue
import struct
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gmp_router.client import RouterClient
from gmp_router.device import RouterDevice
from gmp_router.protocol import MessageType, Packet, cobs_decode, encode_wire


class FakeTransport:
    def __init__(self) -> None:
        self.responses: queue.Queue[bytes] = queue.Queue()
        self.closed = False

    def write(self, data: bytes) -> int:
        request = Packet.decode(cobs_decode(data[:-1]))
        if request.peripheral == 0 and request.operation == 1:
            payload = struct.pack("<BBBBBBB", 1, 30, 2, 2, 2, 0, 4) + b"TEST"
        elif request.peripheral == 1 and request.operation == 3:
            payload = b"\x01"
        else:
            payload = b""
        self.responses.put(encode_wire(Packet(MessageType.RESPONSE, request.sequence,
                                               request.peripheral, request.operation,
                                               request.endpoint, request.channel, 0, payload)))
        return len(data)

    def read_until(self, delimiter: bytes = b"\x00") -> bytes:
        try:
            return self.responses.get(timeout=0.05)
        except queue.Empty:
            return b""

    def close(self) -> None:
        self.closed = True


class DeviceTests(unittest.TestCase):
    def test_typed_client_and_sequence_correlation(self) -> None:
        transport = FakeTransport()
        device = RouterDevice(transport)
        try:
            client = RouterClient(device)
            self.assertEqual(client.hello()["uid"], "TEST")
            self.assertTrue(client.gpio_read(25))
        finally:
            device.close()
        self.assertTrue(transport.closed)


if __name__ == "__main__":
    unittest.main()
