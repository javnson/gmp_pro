"""Focused unit tests for the public headless GMP Data Link API."""

from __future__ import annotations

import struct
import tempfile
import unittest
from pathlib import Path

from apis import (
    GmpDatalinkClient,
    ScopeConfiguration,
    ScopeLayout,
    ScopeResource,
    ScopeSampleType,
    ScopeTriggerMode,
    crc16_ccitt,
    encode_frame,
)
from apis.protocol import (
    encode_scope_configuration,
    parse_memory_descriptor,
    parse_tunable_descriptor,
)


class FakeTargetTransport:
    """In-memory target implementing the three documented service contracts."""

    def __init__(self) -> None:
        self.tunable_values = {0: 50.0, 1: 1.0, 2: 0.0}
        self.tunable_types = {0: "<f", 1: "<f", 2: "<f"}
        self.tunable_names = ("Frequency", "Gain", "Offset")
        self.memory_address = 0x2000
        self.memory = bytearray(range(64))
        self.scope_generation = 0
        self.scope_divider = 0
        self.scope_raw = struct.pack(
            "<16f",
            *(float(value) for value in range(8)),
            *(float(-value) for value in range(8)),
        )

    def transact(self, command: int, payload: bytes = b"") -> bytes:
        if command == 0x31 and len(payload) == 1:
            item_id = payload[0]
            name = self.tunable_names[item_id].encode("ascii")
            return struct.pack("<BBBBBBB", 2, 0, 3, item_id, 4, 1, len(name)) + name
        if command == 0x30:
            response = bytearray((payload[0],))
            for item_id in payload[1:]:
                response.append(item_id)
                response.extend(struct.pack(self.tunable_types[item_id], self.tunable_values[item_id]))
            return bytes(response)
        if command == 0x31:
            count = payload[0]
            index = 1
            for _ in range(count):
                item_id = payload[index]
                index += 1
                self.tunable_values[item_id] = struct.unpack_from("<f", payload, index)[0]
                index += 4
            return b"\x00"
        if command == 0x51 and len(payload) == 1:
            name = b"Scratch"
            return struct.pack(
                "<BBBBIIBB", 2, 0, 1, 0, self.memory_address, len(self.memory), 1, len(name)
            ) + name
        if command == 0x50:
            address, item_size, count = struct.unpack("<IBH", payload)
            offset = address - self.memory_address
            length = item_size * count
            return b"\x00" + bytes(self.memory[offset:offset + length])
        if command == 0x51:
            address, item_size, count = struct.unpack_from("<IBH", payload)
            offset = address - self.memory_address
            length = item_size * count
            self.memory[offset:offset + length] = payload[7:7 + length]
            return b"\x00"
        if command == 0x60 and payload[:1] == b"\x00":
            name = b"Control Scope"
            return struct.pack(
                "<BBBBBBBHIIIB", 0, 0, 2, 1, 0, 7, 1, 2, 8, 1000, len(self.scope_raw), len(name)
            ) + name
        if command == 0x60 and payload[:1] == b"\x01":
            self.scope_divider = struct.unpack_from("<H", payload, 14)[0]
            return b"\x01\x00\x00"
        if command == 0x60 and payload == b"\x02\x00":
            self.scope_generation += 1
            return b"\x02\x00\x00"
        if command == 0x60 and payload == b"\x03\x00":
            return struct.pack("<BBBBI", 3, 0, 0, 2, self.scope_generation)
        if command == 0x60 and payload[:1] == b"\x04":
            _operation, resource_id, offset, length = struct.unpack("<BBIH", payload)
            return struct.pack("<BBBIH", 4, 0, resource_id, offset, length) + self.scope_raw[offset:offset + length]
        raise AssertionError(f"Unhandled request 0x{command:02X}: {payload.hex(' ')}")


class PublicApiTests(unittest.TestCase):
    """Validate high-level service behavior without serial hardware."""

    def setUp(self) -> None:
        self.target = FakeTargetTransport()
        self.client = GmpDatalinkClient(transport=self.target)

    def test_frame_codec_crc_and_header_escaping(self) -> None:
        payload = b"GMP"
        frame = encode_frame(0x7B, 0x7D, payload)
        self.assertEqual(frame[0], 0x7B)
        self.assertIn(b"%[", frame)
        self.assertIn(b"%]", frame)
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_empty_target_names_receive_stable_fallbacks(self) -> None:
        memory_payload = struct.pack("<BBBBIIBB", 2, 0, 1, 3, 0x4000, 16, 0, 0)
        tunable_payload = struct.pack("<BBBBBBB", 2, 0, 1, 4, 0, 1, 0)
        _total, _item_id, memory = parse_memory_descriptor(memory_payload)
        _total, _item_id, tunable = parse_tunable_descriptor(tunable_payload)
        self.assertEqual(memory.name, "Memory Region 3")
        self.assertEqual(tunable.name, "Parameter 4")

        legacy_scope = ScopeResource(
            resource_id=0,
            protocol_version=1,
            sample_type=ScopeSampleType.F32,
            layout=ScopeLayout.STRUCTURE_OF_ARRAYS,
            channels=1,
            depth=8,
            sample_rate_hz=1000,
            byte_length=32,
            name="Legacy Scope",
        )
        with self.assertRaises(ValueError):
            encode_scope_configuration(
                legacy_scope, ScopeConfiguration(sample_divider=1)
            )

    def test_tunable_discovery_read_and_write_by_name(self) -> None:
        parameters = self.client.tunables.discover()
        self.assertEqual([item.name for item in parameters], list(self.target.tunable_names))
        self.assertEqual(self.client.tunables.read("Frequency"), 50.0)
        self.client.tunables.write_many({"Frequency": 25.0, "Gain": 0.5})
        self.assertEqual(self.client.tunables.read_all()["Frequency"], 25.0)
        self.assertEqual(self.client.tunables.read_all()["Gain"], 0.5)

    def test_memory_discovery_chunked_access_and_region_selection(self) -> None:
        region = self.client.memory.discover()[0]
        self.assertEqual(region.name, "Scratch")
        self.assertEqual(self.client.memory.read_region("Scratch", offset=4, byte_length=4), b"\x04\x05\x06\x07")
        self.client.memory.write_region(region, b"\xAA\xBB", offset=6)
        self.assertEqual(self.client.memory.read(region.address + 6, 2), b"\xAA\xBB")
        with self.assertRaises(ValueError):
            self.client.memory.read(region.address + region.byte_length, 1)

    def test_scope_capture_divider_decode_and_continuous_generation(self) -> None:
        resource = self.client.scope.discover()[0]
        configuration = ScopeConfiguration(
            mode=ScopeTriggerMode.IMMEDIATE,
            sample_divider=9,
        )
        frame = self.client.scope.capture(resource, configuration, poll_interval=0.0)
        self.assertEqual(frame.sample_rate_hz, 100.0)
        self.assertEqual(frame.channels[0], tuple(float(value) for value in range(8)))
        self.assertEqual(frame.channels[1][4], -4.0)
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "frame.csv"
            frame.save_csv(csv_path)
            csv_text = csv_path.read_text(encoding="utf-8")
            self.assertIn("generation,time_seconds,channel_0,channel_1", csv_text)
        frames = list(
            self.client.scope.iter_captures(
                resource, configuration, count=3, poll_interval=0.0
            )
        )
        self.assertEqual([item.generation for item in frames], [2, 3, 4])


if __name__ == "__main__":
    unittest.main()
