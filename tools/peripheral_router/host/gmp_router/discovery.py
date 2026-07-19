"""Discovery helpers for multiple simultaneously connected router boards."""

from __future__ import annotations

from .client import RouterClient
from .device import RouterDevice
from .transport import SerialTransport


def serial_ports() -> list[str]:
    """List candidate serial ports without opening them."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [item.device for item in list_ports.comports()]


def probe(port: str, timeout: float = 0.4) -> dict[str, object]:
    """Open a port, perform HELLO, and return device metadata."""
    device = RouterDevice(SerialTransport(port, timeout=timeout))
    try:
        result = RouterClient(device).hello()
        result["port"] = port
        return result
    finally:
        device.close()
