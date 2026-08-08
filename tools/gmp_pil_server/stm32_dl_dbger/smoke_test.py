"""Exercise the GMP u8 Data Link firmware through the ST-Link virtual COM port."""

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
    """Encode one GMP DL frame."""
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
    """Decoded GMP DL frame."""

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
    """Read and validate one frame, ignoring unrelated raw bytes."""
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
        payload_with_crc = (
            read_exact(port, payload_length + 2, deadline) if payload_length else b""
        )
        if payload_length:
            if len(payload_with_crc) != payload_length + 2:
                continue
            payload = payload_with_crc[:-2]
            if crc16_ccitt(payload) != struct.unpack("<H", payload_with_crc[-2:])[0]:
                continue
        else:
            payload = b""
        return Frame(sequence, command, payload)
    raise TimeoutError("No valid GMP DL frame was received")


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
    """Validate Data Link services, DMA load, and the DSA waveform snapshot."""
    with serial.Serial(port_name, baudrate, timeout=0.05, write_timeout=1.0) as port:
        port.reset_input_buffer()
        sequence = 1

        info = transact(port, sequence, 0x02)
        if len(info) != 12 or info[:6] != bytes((1, 1, 1, 8, 0x30, 0x50)):
            raise AssertionError(f"Unexpected target info: {info.hex(' ')}")
        memory_address, memory_length = struct.unpack_from("<IH", info, 6)

        sequence += 1
        echo_payload = b"GMP-u8-{%}-\x00"
        if transact(port, sequence, 0x00, echo_payload) != echo_payload:
            raise AssertionError("ECHO payload mismatch")

        sequence += 1
        dma_stress_payload = bytes(range(256))
        if transact(port, sequence, 0x00, dma_stress_payload) != dma_stress_payload:
            raise AssertionError("DMA stress ECHO payload mismatch")

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
        memory = transact(port, sequence, 0x50, struct.pack("<IBH", memory_address, 1, 16))
        if memory != b"\x00" + bytes(range(16)):
            raise AssertionError(f"Unexpected memory data: {memory.hex(' ')}")
        original_octet = memory[8]

        sequence += 1
        if transact(port, sequence, 0x51,
                    struct.pack("<IBHB", memory_address + 7, 1, 1, 0xA5)) != b"\x00":
            raise AssertionError("Memory write failed")
        sequence += 1
        changed = transact(port, sequence, 0x50, struct.pack("<IBH", memory_address + 7, 1, 1))
        if changed != b"\x00\xA5":
            raise AssertionError("Memory readback mismatch")
        sequence += 1
        transact(port, sequence, 0x51,
                 struct.pack("<IBHB", memory_address + 7, 1, 1, original_octet))

        sequence += 1
        if transact(port, sequence, 0x04) != b"\x00":
            raise AssertionError("DSA arm command failed")

        dsa_info = b""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            sequence += 1
            dsa_info = transact(port, sequence, 0x03)
            if len(dsa_info) != 20:
                raise AssertionError(f"Unexpected DSA info: {dsa_info.hex(' ')}")
            if dsa_info[1] == 2:
                break
            time.sleep(0.05)
        else:
            raise AssertionError("DSA capture did not complete")

        (version, state, sample_format, channels, depth, sample_rate,
         dsa_address, dsa_bytes, generation) = struct.unpack("<BBBBHIIHI", dsa_info)
        if (version, state, sample_format, channels, depth, sample_rate, dsa_bytes) != (
                1, 2, 1, 2, 400, 1000, 3200):
            raise AssertionError(f"Invalid DSA metadata: {dsa_info.hex(' ')}")

        dsa_raw = bytearray()
        while len(dsa_raw) < dsa_bytes:
            item_count = min(60, (dsa_bytes - len(dsa_raw)) // 4)
            sequence += 1
            response = transact(
                port, sequence, 0x50,
                struct.pack("<IBH", dsa_address + len(dsa_raw), 4, item_count),
            )
            if not response or response[0] != 0:
                raise AssertionError("DSA Memory Perspective read failed")
            dsa_raw.extend(response[1:])

        values = struct.unpack(f"<{channels * depth}f", dsa_raw)
        sine = values[:depth]
        cosine = values[depth:]
        sine_rms = math.sqrt(sum(value * value for value in sine) / depth)
        cosine_rms = math.sqrt(sum(value * value for value in cosine) / depth)
        quadrature_error = abs(sum(a * b for a, b in zip(sine, cosine)) / depth)
        periodic_error = max(abs(sine[index] - sine[index + 20]) for index in range(depth - 20))
        if not (0.69 < sine_rms < 0.72 and 0.69 < cosine_rms < 0.72):
            raise AssertionError("DSA waveform RMS is outside the expected range")
        if quadrature_error > 0.01 or periodic_error > 0.001:
            raise AssertionError("DSA sine/cosine waveform validation failed")

        sequence += 1
        read_only_status = transact(
            port, sequence, 0x51,
            struct.pack("<IBHf", dsa_address, 4, 1, 0.0),
        )
        if read_only_status == b"\x00":
            raise AssertionError("DSA buffer was not protected as read-only")

    print(f"PASS: u8 Data Link validated on {port_name} at {baudrate} baud")
    print(f"      Memory sandbox: 0x{memory_address:08X}, {memory_length} bytes")
    print(f"      DSA snapshot: generation {generation}, {depth} x {channels} float32")


def main() -> None:
    """Parse command-line options and run the hardware smoke test."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None, help="Serial port; auto-detected when omitted")
    parser.add_argument("--baudrate", type=int, default=921600)
    args = parser.parse_args()
    run_smoke_test(args.port or discover_port(), args.baudrate)


if __name__ == "__main__":
    main()
