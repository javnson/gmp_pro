"""Portable CAN types and codecs for current and future router boards."""

from __future__ import annotations

from dataclasses import dataclass
import struct


@dataclass(slots=True)
class CanFrame:
    """Normalized Classic CAN or CAN FD frame."""

    identifier: int
    flags: int
    data: bytes = b""

    def encode(self, token: int = 0) -> bytes:
        if len(self.data) > 64:
            raise ValueError("CAN payload exceeds 64 bytes")
        return struct.pack("<IIHH", token, self.identifier, self.flags, len(self.data)) + self.data

    @classmethod
    def decode(cls, payload: bytes, offset: int = 0) -> tuple[int, "CanFrame"]:
        token, identifier, flags, length = struct.unpack_from("<IIHH", payload, offset)
        start = offset + 12
        if length > 64 or len(payload) < start + length:
            raise ValueError("invalid CAN frame payload")
        return token, cls(identifier, flags, payload[start:start + length])


@dataclass(slots=True)
class CanCapabilities:
    """Hardware properties reported with the same fields as GMP core CAN."""

    feature_flags: int
    hardware_tx_slots: int
    hardware_rx_fifos: int
    hardware_filter_slots: int
    max_data_length: int
    timestamp_width: int
    nominal_bitrate_max: int
    data_bitrate_max: int

    @classmethod
    def decode(cls, payload: bytes) -> "CanCapabilities":
        if len(payload) != 24:
            raise ValueError("invalid CAN capability payload")
        flags, tx, rx, filters, maximum, timestamp, _reserved, nominal, data = struct.unpack("<IHHHHHHII", payload)
        return cls(flags, tx, rx, filters, maximum, timestamp, nominal, data)


@dataclass(slots=True)
class CanState:
    """CAN bus state and controller diagnostic counters."""

    bus_state: int
    tx_error_count: int
    rx_error_count: int
    rx_overflow_count: int
    tx_drop_count: int
    bus_off_count: int
    last_error: int

    @classmethod
    def decode(cls, payload: bytes) -> "CanState":
        if len(payload) != 24:
            raise ValueError("invalid CAN state payload")
        return cls(*struct.unpack("<HHH2xIIII", payload))


@dataclass(slots=True)
class CanRxEvent:
    """Asynchronous receive event with hardware metadata."""

    timestamp: int
    filter_index: int
    metadata_flags: int
    frame: CanFrame

    @classmethod
    def decode(cls, payload: bytes) -> "CanRxEvent":
        timestamp, filter_index, metadata_flags, identifier, flags, length = struct.unpack_from("<IHHIHH", payload)
        if length > 64 or len(payload) != 16 + length:
            raise ValueError("invalid CAN RX event")
        return cls(timestamp, filter_index, metadata_flags,
                   CanFrame(identifier, flags, payload[16:]))


@dataclass(slots=True)
class CanTxEvent:
    """Asynchronous transmission completion event."""

    token: int
    status: int
    timestamp: int

    @classmethod
    def decode(cls, payload: bytes) -> "CanTxEvent":
        if len(payload) != 12:
            raise ValueError("invalid CAN TX event")
        return cls(*struct.unpack("<Ih2xI", payload))


@dataclass(slots=True)
class CanStateEvent:
    """Asynchronous controller state transition event."""

    state: CanState
    timestamp: int

    @classmethod
    def decode(cls, payload: bytes) -> "CanStateEvent":
        if len(payload) != 28:
            raise ValueError("invalid CAN state event")
        return cls(CanState.decode(payload[:24]), struct.unpack_from("<I", payload, 24)[0])
