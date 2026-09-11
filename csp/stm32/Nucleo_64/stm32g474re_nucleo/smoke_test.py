"""Exercise the NUCLEO-G474RE GMP Data Link firmware over the ST-Link VCP."""

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
    """Find the NUCLEO-G474RE ST-Link virtual COM port."""
    matches = [port.device for port in list_ports.comports() if port.vid == 0x0483 and port.pid == 0x374E]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one NUCLEO-G474RE VCP, found: {matches}")
    return matches[0]


def run_smoke_test(port_name: str, baudrate: int) -> None:
    """Validate DMA transport, discovery, read/write services, and Scope capture."""
    with serial.Serial(port_name, baudrate, timeout=0.05, write_timeout=1.0) as port:
        port.reset_input_buffer()
        sequence = 1

        info = transact(port, sequence, 0x02)
        expected_info = bytes(
            (3, 1, 1, 8, 4, 4, 0x10, 5, 1, 0x30, 2,
             2, 0x50, 2, 3, 0x60, 1)
        )
        if info != expected_info:
            raise AssertionError(f"Unexpected target info: {info.hex(' ')}")

        sequence += 1
        pil_tx_mask = 0x00010001
        pil_rx_mask = 0x01000001
        pil_masks = transact(port, sequence, 0x11, struct.pack("<II", pil_tx_mask, pil_rx_mask))
        if pil_masks != struct.pack("<II", pil_tx_mask, pil_rx_mask):
            raise AssertionError(f"PIL mask synchronization failed: {pil_masks.hex(' ')}")
        sequence += 1
        pil_step = transact(
            port, sequence, 0x12,
            struct.pack("<IIHf", 1234, 0xA55A5AA5, 3210, 1.25),
        )
        if len(pil_step) != 10:
            raise AssertionError(f"Unexpected PIL STEP length: {len(pil_step)}")
        pil_digital, pil_pwm, pil_monitor = struct.unpack("<IHf", pil_step)
        if (pil_digital, pil_pwm) != (0xA55A5AA5, 3210) or abs(pil_monitor - 1.25) > 1.0e-6:
            raise AssertionError("PIL STEP did not preserve the selected inputs")

        sequence += 1
        echo_payload = b"GMP-u8-{%}-\x00"
        if transact(port, sequence, 0x00, echo_payload) != echo_payload:
            raise AssertionError("ECHO payload mismatch")
        sequence += 1
        dma_stress_payload = bytes(range(256))
        if transact(port, sequence, 0x00, dma_stress_payload) != dma_stress_payload:
            raise AssertionError("DMA stress ECHO payload mismatch")

        sequence += 1
        bad_crc_frame = bytearray(encode_frame(sequence, 0x00, b"bad-crc"))
        bad_crc_frame[-1] ^= 0x80
        port.write(bad_crc_frame)
        port.flush()
        try:
            read_frame(port, timeout=0.2)
        except TimeoutError:
            pass
        else:
            raise AssertionError("Target responded to a payload with an invalid CRC")
        sequence += 1
        if transact(port, sequence, 0x00, b"crc-recovery") != b"crc-recovery":
            raise AssertionError("Data Link did not recover after an invalid CRC")

        tunable_names = []
        for item_id in range(3):
            sequence += 1
            descriptor = transact(port, sequence, 0x31, bytes((item_id,)))
            if len(descriptor) < 7 or descriptor[:4] != bytes((2, 0, 3, item_id)):
                raise AssertionError(f"Invalid Tunable descriptor {item_id}: {descriptor.hex(' ')}")
            name_length = descriptor[6]
            if len(descriptor) != 7 + name_length:
                raise AssertionError("Tunable descriptor text length mismatch")
            tunable_names.append(descriptor[7:7 + name_length].decode("ascii"))
        if tunable_names != ["Signal Frequency (Hz)", "Signal Gain (x)", "Signal DC Offset (V)"]:
            raise AssertionError(f"Unexpected Tunable discovery names: {tunable_names}")

        sequence += 1
        tunable = transact(port, sequence, 0x30, bytes((3, 0, 1, 2)))
        if len(tunable) != 16 or tunable[0] != 3:
            raise AssertionError(f"Unexpected Tunable response: {tunable.hex(' ')}")
        defaults = (
            struct.unpack_from("<f", tunable, 2)[0],
            struct.unpack_from("<f", tunable, 7)[0],
            struct.unpack_from("<f", tunable, 12)[0],
        )
        test_settings = (25.0, 0.5, 0.25)
        sequence += 1
        write_payload = struct.pack(
            "<BBfBfBf", 3, 0, test_settings[0], 1, test_settings[1], 2, test_settings[2]
        )
        if transact(port, sequence, 0x31, write_payload) != b"\x00":
            raise AssertionError("Tunable write failed")
        sequence += 1
        changed = transact(port, sequence, 0x30, bytes((3, 0, 1, 2)))
        changed_values = (
            struct.unpack_from("<f", changed, 2)[0],
            struct.unpack_from("<f", changed, 7)[0],
            struct.unpack_from("<f", changed, 12)[0],
        )
        if any(abs(actual - expected) > 1.0e-6 for actual, expected in zip(changed_values, test_settings)):
            raise AssertionError("Tunable readback mismatch")

        sequence += 1
        memory_descriptor = transact(port, sequence, 0x51, b"\x00")
        memory_header = struct.Struct("<BBBBIIBB")
        if len(memory_descriptor) < memory_header.size or memory_descriptor[:4] != bytes((2, 0, 5, 0)):
            raise AssertionError(f"Invalid Memory descriptor: {memory_descriptor.hex(' ')}")
        memory_fields = memory_header.unpack_from(memory_descriptor)
        discovered_address, discovered_length = memory_fields[4:6]
        name_length = memory_fields[-1]
        name_start = memory_header.size
        memory_name = memory_descriptor[name_start:name_start + name_length].decode("ascii")
        if discovered_length != 128 or memory_name != "Scratch Memory":
            raise AssertionError("Unexpected Memory resource metadata")
        memory_address, memory_length = discovered_address, discovered_length

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

        expected_regions = (
            ("ADC Feedback Raw", 24),
            ("QEP Count", 4),
            ("PWM Compare (outputs remain disabled)", 12),
            ("Platform Diagnostics", 28),
        )
        discovered_regions = {}
        for resource_id, (expected_name, expected_length) in enumerate(
            expected_regions, start=1
        ):
            sequence += 1
            descriptor = transact(port, sequence, 0x51, bytes((resource_id,)))
            if len(descriptor) < memory_header.size or descriptor[:4] != bytes(
                (2, 0, 5, resource_id)
            ):
                raise AssertionError(
                    f"Invalid Memory descriptor {resource_id}: {descriptor.hex(' ')}"
                )
            fields = memory_header.unpack_from(descriptor)
            address, byte_length = fields[4:6]
            name_length = fields[-1]
            name = descriptor[
                memory_header.size:memory_header.size + name_length
            ].decode("ascii")
            if (name, byte_length) != (expected_name, expected_length):
                raise AssertionError(
                    f"Unexpected Memory resource {resource_id}: {name}, {byte_length}"
                )
            discovered_regions[name] = (address, byte_length)

        adc_address, adc_length = discovered_regions["ADC Feedback Raw"]
        sequence += 1
        adc_payload = transact(
            port, sequence, 0x50, struct.pack("<IBH", adc_address, 1, adc_length)
        )
        adc_values = struct.unpack("<6I", adc_payload[1:])
        if adc_payload[:1] != b"\x00" or any(value > 4095 for value in adc_values):
            raise AssertionError(f"Invalid ADC feedback values: {adc_values}")
        sequence += 1
        readonly_result = transact(
            port, sequence, 0x51, struct.pack("<IBHB", adc_address, 1, 1, 0)
        )
        if readonly_result == b"\x00":
            raise AssertionError("Read-only ADC region accepted a write")

        pwm_address, pwm_length = discovered_regions[
            "PWM Compare (outputs remain disabled)"
        ]
        sequence += 1
        pwm_original = transact(
            port, sequence, 0x50, struct.pack("<IBH", pwm_address, 1, pwm_length)
        )[1:]
        pwm_test = struct.pack("<III", 100, 200, 300)
        sequence += 1
        if transact(
            port, sequence, 0x51,
            struct.pack("<IBH", pwm_address, 1, pwm_length) + pwm_test,
        ) != b"\x00":
            raise AssertionError("PWM compare memory write failed")
        sequence += 1
        pwm_readback = transact(
            port, sequence, 0x50, struct.pack("<IBH", pwm_address, 1, pwm_length)
        )
        if pwm_readback != b"\x00" + pwm_test:
            raise AssertionError("PWM compare readback mismatch")
        sequence += 1
        transact(
            port, sequence, 0x51,
            struct.pack("<IBH", pwm_address, 1, pwm_length) + pwm_original,
        )

        diag_address, diag_length = discovered_regions["Platform Diagnostics"]
        sequence += 1
        diag_first = transact(
            port, sequence, 0x50, struct.pack("<IBH", diag_address, 1, diag_length)
        )
        diag_start_time = time.monotonic()
        for stress_index in range(8):
            sequence += 1
            stress_payload = bytes(
                (value + stress_index) & 0xFF for value in range(256)
            )
            if transact(port, sequence, 0x00, stress_payload) != stress_payload:
                raise AssertionError(
                    f"Repeated DMA stress payload {stress_index} mismatch"
                )
        time.sleep(0.55)
        sequence += 1
        diag_second = transact(
            port, sequence, 0x50, struct.pack("<IBH", diag_address, 1, diag_length)
        )
        diag_elapsed = time.monotonic() - diag_start_time
        diagnostics_first = struct.unpack("<7I", diag_first[1:])
        diagnostics = struct.unpack("<7I", diag_second[1:])
        if diag_first[:1] != b"\x00" or diag_second[:1] != b"\x00":
            raise AssertionError("Platform diagnostics read failed")
        control_isr_rate = (
            diagnostics[0] - diagnostics_first[0]
        ) / diag_elapsed
        if not 18000.0 <= control_isr_rate <= 22000.0:
            raise AssertionError(
                f"TIM/ADC control ISR rate is {control_isr_rate:.1f} Hz"
            )
        if diagnostics[1] != 0:
            raise AssertionError(f"PWM outputs unexpectedly enabled: 0x{diagnostics[1]:08X}")
        if diagnostics[2] == 0 or diagnostics[3] == 0:
            raise AssertionError("UART DMA RX/TX callbacks were not observed")
        if diagnostics[4] != diagnostics_first[4]:
            raise AssertionError(
                "UART error callback count advanced during the test window: "
                f"{diagnostics_first[4]} -> {diagnostics[4]}"
            )
        if diagnostics[6] <= diagnostics_first[6]:
            raise AssertionError("Tick-driven user LED heartbeat is not advancing")

        sequence += 1
        scope_descriptor = transact(port, sequence, 0x60, bytes((0, 0)))
        descriptor_format = struct.Struct("<BBBBBBBHIIIB")
        if len(scope_descriptor) < descriptor_format.size:
            raise AssertionError(f"Scope descriptor is truncated: {scope_descriptor.hex(' ')}")
        fields = descriptor_format.unpack_from(scope_descriptor)
        if fields[:7] != (0, 0, 2, 1, 0, 7, 1):
            raise AssertionError(f"Invalid Scope discovery header: {fields}")
        channels, depth, sample_rate, scope_bytes, scope_name_length = fields[7:]
        scope_name = scope_descriptor[descriptor_format.size:descriptor_format.size + scope_name_length].decode("ascii")
        if (channels, depth, sample_rate, scope_bytes, scope_name) != (
            2, 400, 1000, 3200, "Sine and Cosine Scope"
        ):
            raise AssertionError("Unexpected Scope resource metadata")

        sequence += 1
        config = struct.pack("<BBBBHfIH", 1, 0, 1, 0, 250, test_settings[2], 1000, 0)
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

        sequence += 1
        restore_payload = struct.pack(
            "<BBfBfBf", 3, 0, defaults[0], 1, defaults[1], 2, defaults[2]
        )
        if transact(port, sequence, 0x31, restore_payload) != b"\x00":
            raise AssertionError("Failed to restore default signal parameters")

        values = struct.unpack(f"<{channels * depth}f", scope_raw)
        sine = values[:depth]
        cosine = values[depth:]
        dc_offset = test_settings[2]
        centered_sine = [value - dc_offset for value in sine]
        centered_cosine = [value - dc_offset for value in cosine]
        sine_rms = math.sqrt(sum(value * value for value in centered_sine) / depth)
        cosine_rms = math.sqrt(sum(value * value for value in centered_cosine) / depth)
        quadrature_error = abs(sum(a * b for a, b in zip(centered_sine, centered_cosine)) / depth)
        periodic_error = max(abs(sine[index] - sine[index + 40]) for index in range(depth - 40))
        mean_error = abs(sum(sine) / depth - dc_offset)
        if not (0.34 < sine_rms < 0.36 and 0.34 < cosine_rms < 0.36):
            raise AssertionError("Scope waveform RMS is outside the expected range")
        if quadrature_error > 0.005 or periodic_error > 0.001 or mean_error > 0.001:
            raise AssertionError(
                "Scope waveform validation failed: "
                f"quadrature={quadrature_error:.6g}, periodic={periodic_error:.6g}, "
                f"mean={mean_error:.6g}"
            )
        if not (sine[99] < dc_offset <= sine[100]):
            raise AssertionError("Scope pre-trigger position does not match the configured 25 percent")

    print(f"PASS: u8 Data Link validated on {port_name} at {baudrate} baud")
    print(f"      Memory discovery: {memory_name}, 0x{memory_address:08X}, {memory_length} bytes")
    print(f"      PIL: mask synchronization and STEP loopback validated")
    print(
        f"      Platform: ISR={diagnostics[0]} ({control_isr_rate:.1f} Hz), "
        f"RX={diagnostics[2]}, "
        f"TX={diagnostics[3]}, UART error baseline={diagnostics[4]}, "
        f"LED toggles={diagnostics[6]}, PWM output state=0"
    )
    print(f"      Tunable discovery: {len(tunable_names)} physical signal parameters")
    print(
        f"      Scope: {scope_name}, generation {generation}, {depth} x {channels} float32, "
        f"validated at {test_settings[0]:.0f} Hz / {test_settings[1]:.2f}x / {test_settings[2]:.2f} V"
    )


def main() -> None:
    """Parse command-line options and run the hardware smoke test."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None, help="Serial port; auto-detected when omitted")
    parser.add_argument("--baudrate", type=int, default=115200)
    args = parser.parse_args()
    run_smoke_test(args.port or discover_port(), args.baudrate)


if __name__ == "__main__":
    main()
