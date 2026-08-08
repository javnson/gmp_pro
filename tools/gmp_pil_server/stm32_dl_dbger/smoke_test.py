"""Exercise the standalone GMP u8 Data Link firmware over the ST-Link VCP."""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports

SOF = 0x7B
EOF = 0x7D
ESC = 0x25
XOR = 0x20


def crc16_ccitt(data: bytes) -> int:
    """Return CRC16-CCITT with the GMP initial value and polynomial."""
    crc = 0xFFFF
    for octet in data:
        crc ^= octet << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(sequence: int, command: int, payload: bytes = b"") -> bytes:
    """Encode one GMP Data Link frame."""
    header = struct.pack("<BBH", sequence, command, len(payload))
    header += struct.pack("<H", crc16_ccitt(header))
    wire = bytearray([SOF])
    for octet in header:
        if octet in (SOF, EOF, ESC):
            wire.extend((ESC, octet ^ XOR))
        else:
            wire.append(octet)
    wire.append(EOF)
    if payload:
        wire.extend(payload)
        wire.extend(struct.pack("<H", crc16_ccitt(payload)))
    return bytes(wire)


@dataclass(frozen=True)
class Frame:
    """Decoded GMP Data Link frame."""

    sequence: int
    command: int
    payload: bytes


def read_exact(port: serial.Serial, size: int, deadline: float) -> bytes:
    """Read exactly *size* bytes before the shared frame deadline."""
    result = bytearray()
    while len(result) < size and time.monotonic() < deadline:
        result.extend(port.read(size - len(result)))
    return bytes(result)


def read_frame(port: serial.Serial, timeout: float = 1.0) -> Frame:
    """Read and validate one frame while ignoring unrelated raw bytes."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if port.read(1) != bytes([SOF]):
            continue
        header = bytearray()
        escaped = False
        while time.monotonic() < deadline:
            raw = port.read(1)
            if not raw:
                continue
            octet = raw[0]
            if escaped:
                header.append(octet ^ XOR)
                escaped = False
            elif octet == ESC:
                escaped = True
            elif octet == EOF:
                break
            elif octet == SOF:
                header.clear()
            else:
                header.append(octet)
        if len(header) != 6 or crc16_ccitt(header[:4]) != struct.unpack("<H", header[4:])[0]:
            continue
        sequence, command, payload_length = struct.unpack("<BBH", header[:4])
        payload_with_crc = read_exact(port, payload_length + 2, deadline) if payload_length else b""
        if payload_length:
            if len(payload_with_crc) != payload_length + 2:
                continue
            payload = payload_with_crc[:-2]
            if crc16_ccitt(payload) != struct.unpack("<H", payload_with_crc[-2:])[0]:
                continue
        else:
            payload = b""
        return Frame(sequence, command, payload)
    raise TimeoutError("No valid GMP Data Link frame was received")


def transact(port: serial.Serial, sequence: int, command: int, payload: bytes = b"") -> bytes:
    """Send one request and return its matching response payload."""
    port.write(encode_frame(sequence, command, payload))
    port.flush()
    while True:
        frame = read_frame(port)
        if frame.sequence == sequence and frame.command == command:
            return frame.payload
        if frame.sequence == sequence and frame.command == 0x01:
            raise RuntimeError(f"Target returned NACK: {frame.payload.hex(' ')}")


def discover_port() -> str:
    """Find the NUCLEO-C092RC ST-Link virtual COM port."""
    matches = [port.device for port in list_ports.comports() if port.vid == 0x0483 and port.pid == 0x3752]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one NUCLEO-C092RC VCP, found: {matches}")
    return matches[0]


def run_smoke_test(port_name: str, baudrate: int) -> None:
    """Validate DMA transport, discovery, read/write services, and Scope capture."""
    with serial.Serial(port_name, baudrate, timeout=0.05, write_timeout=1.0) as port:
        port.reset_input_buffer()
        sequence = 1

        info = transact(port, sequence, 0x02)
        if len(info) != 13 or info[:7] != bytes((2, 1, 1, 8, 0x30, 0x50, 0x60)):
            raise AssertionError(f"Unexpected target info: {info.hex(' ')}")
        memory_address, memory_length = struct.unpack_from("<IH", info, 7)

        sequence += 1
        echo_payload = b"GMP-u8-{%}-\x00"
        if transact(port, sequence, 0x00, echo_payload) != echo_payload:
            raise AssertionError("ECHO payload mismatch")
        sequence += 1
        dma_stress_payload = bytes(range(256))
        if transact(port, sequence, 0x00, dma_stress_payload) != dma_stress_payload:
            raise AssertionError("DMA stress ECHO payload mismatch")

        tunable_names = []
        for item_id in range(4):
            sequence += 1
            descriptor = transact(port, sequence, 0x31, bytes((item_id,)))
            if len(descriptor) < 8 or descriptor[:4] != bytes((1, 0, 4, item_id)):
                raise AssertionError(f"Invalid Tunable descriptor {item_id}: {descriptor.hex(' ')}")
            name_length, unit_length = descriptor[6:8]
            if len(descriptor) != 8 + name_length + unit_length:
                raise AssertionError("Tunable descriptor text length mismatch")
            tunable_names.append(descriptor[8:8 + name_length].decode("ascii"))
        if tunable_names[-1] != "Pi Estimate":
            raise AssertionError(f"Unexpected Tunable discovery names: {tunable_names}")

        sequence += 1
        tunable = transact(port, sequence, 0x30, bytes((4, 0, 1, 2, 3)))
        if len(tunable) != 17 or tunable[0] != 4:
            raise AssertionError(f"Unexpected Tunable response: {tunable.hex(' ')}")
        original_u16 = struct.unpack_from("<H", tunable, 2)[0]
        replacement_u16 = original_u16 ^ 0xFFFF
        sequence += 1
        if transact(port, sequence, 0x31, struct.pack("<BBH", 1, 0, replacement_u16)) != b"\x00":
            raise AssertionError("Tunable write failed")
        sequence += 1
        changed = transact(port, sequence, 0x30, bytes((1, 0)))
        if struct.unpack_from("<H", changed, 2)[0] != replacement_u16:
            raise AssertionError("Tunable readback mismatch")
        sequence += 1
        transact(port, sequence, 0x31, struct.pack("<BBH", 1, 0, original_u16))

        sequence += 1
        memory_descriptor = transact(port, sequence, 0x51, b"\x00")
        if len(memory_descriptor) < 25 or memory_descriptor[:4] != bytes((1, 0, 1, 0)):
            raise AssertionError(f"Invalid Memory descriptor: {memory_descriptor.hex(' ')}")
        discovered_address, discovered_length = struct.unpack_from("<II", memory_descriptor, 4)
        name_length = memory_descriptor[24]
        memory_name = memory_descriptor[25:25 + name_length].decode("ascii")
        if (discovered_address, discovered_length, memory_name) != (
            memory_address, memory_length, "Scratch Memory"
        ):
            raise AssertionError("Memory discovery does not match target info")

        sequence += 1
        memory = transact(port, sequence, 0x50, struct.pack("<IBH", memory_address, 1, 16))
        if memory != b"\x00" + bytes(range(16)):
            raise AssertionError(f"Unexpected memory data: {memory.hex(' ')}")
        original_octet = memory[8]
        sequence += 1
        if transact(port, sequence, 0x51, struct.pack("<IBHB", memory_address + 7, 1, 1, 0xA5)) != b"\x00":
            raise AssertionError("Memory write failed")
        sequence += 1
        changed = transact(port, sequence, 0x50, struct.pack("<IBH", memory_address + 7, 1, 1))
        if changed != b"\x00\xA5":
            raise AssertionError("Memory readback mismatch")
        sequence += 1
        transact(port, sequence, 0x51, struct.pack("<IBHB", memory_address + 7, 1, 1, original_octet))

        sequence += 1
        scope_descriptor = transact(port, sequence, 0x60, bytes((0, 0)))
        descriptor_format = struct.Struct("<BBBBBBBHIIIB")
        if len(scope_descriptor) < descriptor_format.size:
            raise AssertionError(f"Scope descriptor is truncated: {scope_descriptor.hex(' ')}")
        fields = descriptor_format.unpack_from(scope_descriptor)
        if fields[:7] != (0, 0, 1, 1, 0, 7, 1):
            raise AssertionError(f"Invalid Scope discovery header: {fields}")
        channels, depth, sample_rate, scope_bytes, scope_name_length = fields[7:]
        scope_name = scope_descriptor[descriptor_format.size:descriptor_format.size + scope_name_length].decode("ascii")
        if (channels, depth, sample_rate, scope_bytes, scope_name) != (
            2, 400, 1000, 3200, "Sine and Cosine Scope"
        ):
            raise AssertionError("Unexpected Scope resource metadata")

        sequence += 1
        config = struct.pack("<BBBBHfI", 1, 0, 1, 0, 250, 0.0, 1000)
        if transact(port, sequence, 0x60, config) != bytes((1, 0, 0)):
            raise AssertionError("Scope configuration failed")
        sequence += 1
        if transact(port, sequence, 0x60, bytes((2, 0))) != bytes((2, 0, 0)):
            raise AssertionError("Scope arm failed")

        generation = 0
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            sequence += 1
            status = transact(port, sequence, 0x60, bytes((3, 0)))
            if len(status) != 8 or status[:3] != bytes((3, 0, 0)):
                raise AssertionError(f"Invalid Scope status: {status.hex(' ')}")
            state, generation = status[3], struct.unpack_from("<I", status, 4)[0]
            if state == 2:
                break
            time.sleep(0.05)
        else:
            raise AssertionError("Scope capture did not complete")

        scope_raw = bytearray(scope_bytes)
        offset = 0
        read_header = struct.Struct("<BBBIH")
        while offset < scope_bytes:
            length = min(180, scope_bytes - offset)
            sequence += 1
            response = transact(port, sequence, 0x60, struct.pack("<BBIH", 4, 0, offset, length))
            if len(response) != read_header.size + length:
                raise AssertionError("Scope read response length mismatch")
            operation, status, resource_id, returned_offset, returned_length = read_header.unpack_from(response)
            if (operation, status, resource_id, returned_offset, returned_length) != (4, 0, 0, offset, length):
                raise AssertionError("Scope read response header mismatch")
            scope_raw[offset:offset + length] = response[read_header.size:]
            offset += length

        values = struct.unpack(f"<{channels * depth}f", scope_raw)
        sine = values[:depth]
        cosine = values[depth:]
        sine_rms = math.sqrt(sum(value * value for value in sine) / depth)
        cosine_rms = math.sqrt(sum(value * value for value in cosine) / depth)
        quadrature_error = abs(sum(a * b for a, b in zip(sine, cosine)) / depth)
        periodic_error = max(abs(sine[index] - sine[index + 20]) for index in range(depth - 20))
        if not (0.69 < sine_rms < 0.72 and 0.69 < cosine_rms < 0.72):
            raise AssertionError("Scope waveform RMS is outside the expected range")
        if quadrature_error > 0.01 or periodic_error > 0.001:
            raise AssertionError("Scope sine/cosine waveform validation failed")
        if not (sine[99] < 0.0 and abs(sine[100]) < 0.001):
            raise AssertionError("Scope pre-trigger position does not match the configured 25 percent")

    print(f"PASS: u8 Data Link validated on {port_name} at {baudrate} baud")
    print(f"      Memory discovery: {memory_name}, 0x{memory_address:08X}, {memory_length} bytes")
    print(f"      Tunable discovery: {len(tunable_names)} named parameters")
    print(f"      Scope: {scope_name}, generation {generation}, {depth} x {channels} float32")


def main() -> None:
    """Parse command-line options and run the hardware smoke test."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None, help="Serial port; auto-detected when omitted")
    parser.add_argument("--baudrate", type=int, default=921600)
    args = parser.parse_args()
    run_smoke_test(args.port or discover_port(), args.baudrate)


if __name__ == "__main__":
    main()
