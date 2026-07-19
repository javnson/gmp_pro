"""Byte transports used by the GMP peripheral router host library."""

from __future__ import annotations

from typing import Protocol


class Transport(Protocol):
    """Minimal blocking transport contract used by :class:`RouterDevice`."""

    def read_until(self, delimiter: bytes = b"\x00") -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def close(self) -> None: ...


class SerialTransport:
    """PySerial adapter with lazy dependency loading."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("Install pyserial to use a physical router") from exc
        self._serial = serial.Serial(port, baudrate=baudrate, timeout=timeout,
                                     write_timeout=timeout)

    def read_until(self, delimiter: bytes = b"\x00") -> bytes:
        return self._serial.read_until(delimiter)

    def write(self, data: bytes) -> int:
        return self._serial.write(data)

    def close(self) -> None:
        self._serial.close()
