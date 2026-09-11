"""Prove that USART3 and Ethernet GMP Data Link remain live concurrently."""

from __future__ import annotations

import argparse
import struct
import threading

import serial

from smoke_test import (
    DEFAULT_TARGET_IP,
    DEFAULT_TCP_DL_PORT,
    DEFAULT_UDP_DL_PORT,
    SocketByteTransport,
    discover_port,
    transact,
)


def read_diagnostics(channel, sequence: int) -> tuple[int, ...]:
    """Discover and read the platform diagnostics region."""
    descriptor = transact(channel, sequence, 0x51, b"\x04")
    fields = struct.Struct("<BBBBIIBB").unpack_from(descriptor)
    response = transact(
        channel, sequence + 1, 0x50, struct.pack("<IBH", fields[4], 1, fields[5])
    )
    if response[:1] != b"\x00" or len(response) != 73:
        raise AssertionError("Platform diagnostics response is invalid")
    return struct.unpack("<18I", response[1:])


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
    """Run two independent clients against the same firmware."""
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
        baseline = read_diagnostics(uart, 1)

        def worker(name: str, channel) -> None:
            try:
                run_parallel_echo(name, channel, barrier)
            except BaseException as error:  # preserve worker traceback context
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

        original = transact(uart, 91, 0x30, bytes((1, 0)))
        if len(original) != 6:
            raise AssertionError("UART Tunable read failed")
        original_frequency = struct.unpack_from("<f", original, 2)[0]
        new_frequency = 37.0
        try:
            if transact(uart, 92, 0x31, struct.pack("<BBf", 1, 0, new_frequency)) != b"\x00":
                raise AssertionError("UART Tunable write failed")
            readback = transact(ethernet, 91, 0x30, bytes((1, 0)))
            if len(readback) != 6 or abs(struct.unpack_from("<f", readback, 2)[0] - new_frequency) > 1.0e-6:
                raise AssertionError("Ethernet did not observe the UART Tunable write")
        finally:
            restored = transact(
                ethernet, 92, 0x31, struct.pack("<BBf", 1, 0, original_frequency)
            )
            if restored != b"\x00":
                raise AssertionError("Failed to restore the signal frequency")

        diagnostics = read_diagnostics(uart, 94)
        if diagnostics[2] <= baseline[2] or diagnostics[3] <= baseline[3]:
            raise AssertionError("UART RX/TX counters did not advance")
        if diagnostics[12] <= baseline[12] or diagnostics[13] <= baseline[13]:
            raise AssertionError("Ethernet DL RX/TX counters did not advance")
        if diagnostics[4] != baseline[4] or diagnostics[11] != baseline[11]:
            raise AssertionError("A UART or Ethernet platform error occurred")
        if diagnostics[17] != baseline[17]:
            raise AssertionError("The Ethernet Data Link error counter advanced")
        if diagnostics[1] != 0:
            raise AssertionError("PWM outputs unexpectedly became enabled")

    print(
        f"PASS: simultaneous UART {serial_port} and "
        f"{network_transport.upper()} {target_ip}:{network_port} Data Link"
    )
    print("      48 synchronized ECHO transactions per link; 256-byte bursts passed")
    print("      Facility tables match and shared Tunable write/read/restore passed")
    print(
        f"      UART RX/TX={diagnostics[2]}/{diagnostics[3]}, "
        f"Ethernet RX/TX={diagnostics[12]}/{diagnostics[13]}, errors=0, PWM=disabled"
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
