"""Thread-safe request and asynchronous event handling for one router board."""

from __future__ import annotations

from collections import defaultdict
import queue
import threading
from typing import Callable

from .protocol import MessageType, Packet, Status, cobs_decode, encode_wire
from .transport import Transport


class RouterError(RuntimeError):
    """Raised when a router returns an error or misses a deadline."""


class RouterDevice:
    """Own one serial connection and correlate protocol sequence numbers."""

    def __init__(self, transport: Transport):
        self.transport = transport
        self._sequence = 0
        self._pending: dict[int, queue.Queue[Packet]] = {}
        self._event_handlers: dict[tuple[int, int], list[Callable[[Packet], None]]] = defaultdict(list)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True,
                                        name="gmp-router-reader")
        self._reader.start()

    def request(self, peripheral: int, operation: int, *, endpoint: int = 0,
                channel: int = 0, payload: bytes = b"", timeout: float = 1.0) -> bytes:
        with self._lock:
            self._sequence = (self._sequence + 1) & 0xFFFFFFFF or 1
            sequence = self._sequence
            result: queue.Queue[Packet] = queue.Queue(maxsize=1)
            self._pending[sequence] = result
            self.transport.write(encode_wire(Packet(MessageType.REQUEST, sequence,
                                                     peripheral, operation, endpoint,
                                                     channel, 0, payload)))
        try:
            response = result.get(timeout=timeout)
        except queue.Empty as exc:
            raise RouterError(f"router request {sequence} timed out") from exc
        finally:
            with self._lock:
                self._pending.pop(sequence, None)
        if response.status != Status.OK:
            raise RouterError(f"router status {response.status}")
        return response.payload

    def on_event(self, peripheral: int, operation: int,
                 handler: Callable[[Packet], None]) -> None:
        self._event_handlers[(peripheral, operation)].append(handler)

    def close(self) -> None:
        self._stop.set()
        self.transport.close()
        self._reader.join(timeout=0.5)

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            wire = self.transport.read_until(b"\x00")
            if not wire or wire[-1:] != b"\x00":
                continue
            try:
                packet = Packet.decode(cobs_decode(wire[:-1]))
            except ValueError:
                continue
            if packet.message_type == MessageType.RESPONSE:
                result = self._pending.get(packet.sequence)
                if result is not None:
                    try:
                        result.put_nowait(packet)
                    except queue.Full:
                        pass
            elif packet.message_type == MessageType.EVENT:
                for handler in tuple(self._event_handlers[(packet.peripheral, packet.operation)]):
                    handler(packet)
