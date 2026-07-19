"""Local JSON-line service that owns and multiplexes physical router boards."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import socketserver
import threading
from typing import Any

from .client import RouterClient
from .device import RouterDevice
from .discovery import serial_ports
from .transport import SerialTransport


class DeviceManager:
    """Maintain named connections so several boards can be used concurrently."""

    def __init__(self) -> None:
        self._devices: dict[str, RouterDevice] = {}
        self._lock = threading.Lock()

    def execute(self, request: dict[str, Any]) -> Any:
        command = request.get("command")
        if command == "list_ports":
            return serial_ports()
        if command == "connect":
            name, port = str(request["device"]), str(request["port"])
            with self._lock:
                if name in self._devices:
                    raise ValueError(f"device {name!r} is already connected")
                self._devices[name] = RouterDevice(SerialTransport(port))
            return RouterClient(self._devices[name]).hello()
        if command == "disconnect":
            with self._lock:
                device = self._devices.pop(str(request["device"]))
            device.close()
            return True
        device = self._devices[str(request["device"])]
        client = RouterClient(device, int(request.get("endpoint", 0)))
        channel = int(request.get("channel", 0))
        if command == "hello":
            return client.hello()
        if command == "gpio_configure":
            client.gpio_configure(channel, bool(request["output"]), int(request.get("pull", 0)))
            return True
        if command == "gpio_write":
            client.gpio_write(channel, bool(request["value"]))
            return True
        if command == "gpio_read":
            return client.gpio_read(channel)
        if command == "spi_transfer":
            return client.spi_transfer(channel, bytes.fromhex(request.get("hex", ""))).hex()
        if command == "uart_write":
            client.uart_write(channel, bytes.fromhex(request.get("hex", "")))
            return True
        if command == "uart_configure":
            client.uart_configure(channel, int(request["baudrate"]),
                                  int(request["tx_pin"]), int(request["rx_pin"]))
            return True
        if command == "uart_read":
            return client.uart_read(channel, int(request["length"])).hex()
        if command == "i2c_write":
            client.i2c_write(channel, int(request["address"]), bytes.fromhex(request.get("hex", "")))
            return True
        if command == "i2c_configure":
            client.i2c_configure(channel, int(request["frequency"]),
                                 int(request["sda_pin"]), int(request["scl_pin"]))
            return True
        if command == "i2c_read":
            return client.i2c_read(channel, int(request["address"]), int(request["length"])).hex()
        if command == "can_capabilities":
            return asdict(client.can_capabilities(channel))
        if command == "spi_configure":
            client.spi_configure(channel, int(request["frequency"]), int(request["sck_pin"]),
                                 int(request["tx_pin"]), int(request["rx_pin"]),
                                 int(request.get("mode", 0)))
            return True
        raise ValueError(f"unknown command {command!r}")

    def close(self) -> None:
        with self._lock:
            devices, self._devices = self._devices, {}
        for device in devices.values():
            device.close()


class RequestHandler(socketserver.StreamRequestHandler):
    """Handle newline-delimited UTF-8 JSON requests on localhost."""

    def handle(self) -> None:
        for line in self.rfile:
            try:
                result = self.server.manager.execute(json.loads(line))  # type: ignore[attr-defined]
                response = {"ok": True, "result": result}
            except Exception as exc:
                response = {"ok": False, "error": str(exc)}
            self.wfile.write(json.dumps(response, separators=(",", ":")).encode() + b"\n")


class RouterService(socketserver.ThreadingTCPServer):
    """Loopback-only server used by Windows CSP clients and the debug UI."""

    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], manager: DeviceManager):
        self.manager = manager
        super().__init__(address, RequestHandler)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47620)
    args = parser.parse_args()
    manager = DeviceManager()
    server = RouterService((args.host, args.port), manager)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        manager.close()


if __name__ == "__main__":
    main()
