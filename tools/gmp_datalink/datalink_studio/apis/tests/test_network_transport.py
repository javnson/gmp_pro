"""Loopback tests for TCP and UDP Data Link transports."""

from __future__ import annotations

import socket
import threading
import unittest

from apis import TcpDataLinkTransport, UdpDataLinkTransport


class NetworkTransportTests(unittest.TestCase):
    def _run_tcp_echo(self) -> None:
        ready = threading.Event()
        endpoint: list[int] = []

        def server() -> None:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
                listener.bind(("127.0.0.1", 0))
                listener.listen(1)
                endpoint.append(listener.getsockname()[1])
                ready.set()
                connection, _peer = listener.accept()
                with connection:
                    connection.sendall(connection.recv(4096))

        worker = threading.Thread(target=server, daemon=True)
        worker.start()
        self.assertTrue(ready.wait(1.0))
        with TcpDataLinkTransport("127.0.0.1", endpoint[0], timeout=0.5) as transport:
            self.assertEqual(transport.transact(0x00, b"tcp-loopback"), b"tcp-loopback")
        worker.join(1.0)

    def _run_udp_echo(self) -> None:
        ready = threading.Event()
        endpoint: list[int] = []

        def server() -> None:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as channel:
                channel.bind(("127.0.0.1", 0))
                endpoint.append(channel.getsockname()[1])
                ready.set()
                request, peer = channel.recvfrom(4096)
                channel.sendto(request, peer)

        worker = threading.Thread(target=server, daemon=True)
        worker.start()
        self.assertTrue(ready.wait(1.0))
        with UdpDataLinkTransport("127.0.0.1", endpoint[0], timeout=0.5) as transport:
            self.assertEqual(transport.transact(0x00, b"udp-loopback"), b"udp-loopback")
        worker.join(1.0)

    def test_tcp_frame_transaction(self) -> None:
        self._run_tcp_echo()

    def test_udp_frame_transaction(self) -> None:
        self._run_udp_echo()


if __name__ == "__main__":
    unittest.main()
