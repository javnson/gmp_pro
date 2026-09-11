"""Socket transports exposing the serial-like byte contract used by Data Link."""

from __future__ import annotations

import socket
import threading


class SocketByteTransport:
    """Buffered TCP or connected-UDP byte transport for GMP Data Link."""

    def __init__(
        self,
        protocol: str,
        host: str,
        port: int,
        *,
        timeout: float = 0.1,
        connect_timeout: float = 2.0,
    ) -> None:
        selected = protocol.lower()
        if selected not in ("tcp", "udp"):
            raise ValueError("Network protocol must be 'tcp' or 'udp'.")
        if not host:
            raise ValueError("Network host cannot be empty.")
        if not 1 <= int(port) <= 65535:
            raise ValueError("Network port must be between 1 and 65535.")
        if timeout <= 0.0 or connect_timeout <= 0.0:
            raise ValueError("Socket timeouts must be positive.")
        self.protocol = selected
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self.connect_timeout = float(connect_timeout)
        self._socket: socket.socket | None = None
        self._rx = bytearray()
        self._rx_lock = threading.Lock()
        self._tx_lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._socket is not None

    @property
    def in_waiting(self) -> int:
        with self._rx_lock:
            return len(self._rx)

    @property
    def endpoint(self) -> str:
        return f"{self.protocol}://{self.host}:{self.port}"

    def open(self) -> None:
        if self._socket is not None:
            return
        socket_type = socket.SOCK_STREAM if self.protocol == "tcp" else socket.SOCK_DGRAM
        channel = socket.socket(socket.AF_INET, socket_type)
        try:
            channel.settimeout(self.connect_timeout)
            channel.connect((self.host, self.port))
            channel.settimeout(self.timeout)
        except OSError:
            channel.close()
            raise
        self._socket = channel

    def reset_input_buffer(self) -> None:
        with self._rx_lock:
            self._rx.clear()

    def write(self, data: bytes) -> int:
        channel = self._socket
        if channel is None:
            raise OSError("network transport is closed")
        payload = bytes(data)
        with self._tx_lock:
            if self.protocol == "tcp":
                channel.sendall(payload)
                return len(payload)
            written = channel.send(payload)
            if written != len(payload):
                raise OSError(f"UDP socket accepted {written} of {len(payload)} bytes")
            return written

    def flush(self) -> None:
        """Match the serial transport contract; socket writes are already submitted."""

    def read(self, size: int = 1) -> bytes:
        requested = max(int(size), 1)
        with self._rx_lock:
            if self._rx:
                return self._take_buffered(requested)
        channel = self._socket
        if channel is None:
            return b""
        try:
            received = channel.recv(65535 if self.protocol == "udp" else 4096)
        except socket.timeout:
            return b""
        if not received:
            if self.protocol == "tcp":
                raise OSError("TCP peer closed the connection")
            return b""
        with self._rx_lock:
            self._rx.extend(received)
            return self._take_buffered(requested)

    def _take_buffered(self, requested: int) -> bytes:
        count = min(requested, len(self._rx))
        result = bytes(self._rx[:count])
        del self._rx[:count]
        return result

    def close(self) -> None:
        channel, self._socket = self._socket, None
        if channel is not None:
            if self.protocol == "tcp":
                try:
                    channel.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
            channel.close()
        self.reset_input_buffer()

    def __enter__(self) -> "SocketByteTransport":
        self.open()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()
