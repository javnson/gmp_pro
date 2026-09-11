"""Prove that the CM4 USART3 and CM7 Ethernet Data Links run concurrently."""

from __future__ import annotations

import argparse
import struct
import threading
import time

import serial

from smoke_test import (
    DEFAULT_TARGET_IP,
    DEFAULT_TCP_DL_PORT,
    DEFAULT_UDP_DL_PORT,
    SocketByteTransport,
    discover_port,
    transact,
)


def read_memory_resource(channel, sequence: int, resource_id: int, length: int) -> bytes:
    """Discover and read one memory perspective resource."""
    descriptor = transact(channel, sequence, 0x51, bytes((resource_id,)))
    fields = struct.Struct("<BBBBIIBB").unpack_from(descriptor)
    if fields[:4] != (2, 0, 6, resource_id) or fields[5] != length:
        raise AssertionError(f"Memory resource {resource_id} descriptor is invalid")
    response = transact(
        channel, sequence + 1, 0x50, struct.pack("<IBH", fields[4], 1, fields[5])
    )
    if response[:1] != b"\x00" or len(response) != length + 1:
        raise AssertionError(f"Memory resource {resource_id} response is invalid")
    return response[1:]


def read_diagnostics(channel, sequence: int) -> tuple[int, ...]:
    return struct.unpack("<18I", read_memory_resource(channel, sequence, 4, 72))


def read_dual_status(channel, sequence: int) -> tuple[int, ...]:
    return struct.unpack("<4I", read_memory_resource(channel, sequence, 5, 16))


def exercise_tunable(channel, sequence: int, value: float) -> None:
    """Write, read back, and restore one endpoint's private Tunable value."""
    original = transact(channel, sequence, 0x30, bytes((1, 0)))
    if len(original) != 6:
        raise AssertionError("Tunable read failed")
    original_value = struct.unpack_from("<f", original, 2)[0]
    try:
        result = transact(channel, sequence + 1, 0x31, struct.pack("<BBf", 1, 0, value))
        if result != b"\x00":
            raise AssertionError("Tunable write failed")
        readback = transact(channel, sequence + 2, 0x30, bytes((1, 0)))
        if len(readback) != 6 or abs(struct.unpack_from("<f", readback, 2)[0] - value) > 1.0e-6:
            raise AssertionError("Tunable readback failed")
    finally:
        restored = transact(
            channel, sequence + 3, 0x31, struct.pack("<BBf", 1, 0, original_value)
        )
        if restored != b"\x00":
            raise AssertionError("Failed to restore the Tunable value")


def run_parallel_echo(name: str, channel, barrier: threading.Barrier) -> None:
    """Exchange synchronized frames whose sizes exercise burst reception."""
    for index in range(48):
        length = (0, 1, 32, 256)[index % 4]
        seed = 0x31 if name == "UART" else 0xA7
        payload = bytes((seed + index + offset) & 0xFF for offset in range(length))
        barrier.wait(timeout=2.0)
        response = transact(channel, index + 20, 0x00, payload)
        if response != payload:
            raise AssertionError(f"{name} ECHO mismatch at iteration {index}")


def run_test(
    serial_port: str,
    baudrate: int,
    network_transport: str,
    target_ip: str,
    network_port: int,
) -> None:
    """Run independent CM4 and CM7 clients against the same dual-core firmware."""
    errors: list[BaseException] = []
    barrier = threading.Barrier(2)

    with serial.Serial(
        serial_port, baudrate, timeout=0.05, write_timeout=1.0
    ) as uart, SocketByteTransport(
        network_transport,
        target_ip,
        network_port,
        timeout=0.05,
        connect_timeout=2.0,
    ) as ethernet:
        uart.reset_input_buffer()
        ethernet.reset_input_buffer()
        uart_baseline = read_diagnostics(uart, 1)
        ethernet_baseline = read_diagnostics(ethernet, 1)
        dual_baseline = read_dual_status(uart, 3)

        def worker(name: str, channel) -> None:
            try:
                run_parallel_echo(name, channel, barrier)
            except BaseException as error:
                errors.append(error)
                try:
                    barrier.abort()
                except threading.BrokenBarrierError:
                    pass

        workers = [
            threading.Thread(target=worker, args=("UART", uart), daemon=True),
            threading.Thread(
                target=worker, args=(network_transport.upper(), ethernet), daemon=True
            ),
        ]
        for worker_thread in workers:
            worker_thread.start()
        for worker_thread in workers:
            worker_thread.join(timeout=15.0)
        if any(worker_thread.is_alive() for worker_thread in workers):
            raise TimeoutError("Concurrent Data Link test did not finish")
        if errors:
            raise errors[0]

        uart_info = transact(uart, 90, 0x02)
        ethernet_info = transact(ethernet, 90, 0x02)
        if uart_info != ethernet_info or not uart_info.startswith(bytes((3,))):
            raise AssertionError("The two Data Link endpoints report different facilities")

        exercise_tunable(uart, 91, 37.0)
        exercise_tunable(ethernet, 91, 43.0)
        time.sleep(1.1)

        uart_diagnostics = read_diagnostics(uart, 100)
        ethernet_diagnostics = read_diagnostics(ethernet, 100)
        dual_status = read_dual_status(ethernet, 102)

        if uart_diagnostics[2] <= uart_baseline[2] or uart_diagnostics[3] <= uart_baseline[3]:
            raise AssertionError("CM4 UART RX/TX counters did not advance")
        if uart_diagnostics[4] != uart_baseline[4]:
            raise AssertionError("A CM4 UART platform error occurred")
        if ethernet_diagnostics[12] <= ethernet_baseline[12] or ethernet_diagnostics[13] <= ethernet_baseline[13]:
            raise AssertionError("CM7 Ethernet DL RX/TX counters did not advance")
        if ethernet_diagnostics[14] <= ethernet_baseline[14] or ethernet_diagnostics[15] <= ethernet_baseline[15]:
            raise AssertionError("CM7 Ethernet DL byte counters did not advance")
        if ethernet_diagnostics[11] != ethernet_baseline[11] or ethernet_diagnostics[17] != ethernet_baseline[17]:
            raise AssertionError("A CM7 Ethernet platform error occurred")
        if uart_diagnostics[1] != 0 or ethernet_diagnostics[1] != 0:
            raise AssertionError("PWM outputs unexpectedly became enabled")
        if dual_status[0] != 0x47373535:
            raise AssertionError(f"Invalid shared dual-core magic: 0x{dual_status[0]:08X}")
        if any(dual_status[index] <= dual_baseline[index] for index in (1, 2, 3)):
            raise AssertionError(
                "CM7 scheduler, CM4 scheduler, or CM7 control heartbeat did not advance: "
                f"{dual_baseline} -> {dual_status}"
            )

    print(
        f"PASS: simultaneous CM4 UART {serial_port} and CM7 "
        f"{network_transport.upper()} {target_ip}:{network_port} Data Link"
    )
    print("      48 synchronized ECHO transactions per link; 256-byte bursts passed")
    print("      Facility tables match; independent Tunable write/read/restore passed")
    print(
        f"      UART RX/TX={uart_diagnostics[2]}/{uart_diagnostics[3]}, "
        f"Ethernet RX/TX={ethernet_diagnostics[12]}/{ethernet_diagnostics[13]}, errors=0"
    )
    print(
        f"      Shared status: CM7 scheduler={dual_status[1]}, "
        f"CM4 scheduler={dual_status[2]}, CM7 control={dual_status[3]}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--network-transport", choices=("tcp", "udp"), default="tcp")
    parser.add_argument("--target-ip", default=DEFAULT_TARGET_IP)
    parser.add_argument("--network-port", type=int, default=None)
    args = parser.parse_args()
    network_port = args.network_port
    if network_port is None:
        network_port = (
            DEFAULT_TCP_DL_PORT
            if args.network_transport == "tcp"
            else DEFAULT_UDP_DL_PORT
        )
    run_test(
        args.port or discover_port(),
        args.baudrate,
        args.network_transport,
        args.target_ip,
        network_port,
    )


if __name__ == "__main__":
    main()
