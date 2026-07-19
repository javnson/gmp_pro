"""Wire protocol shared by the host service and Pico router firmware."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct

MAGIC = 0x4752
VERSION = 1
MAX_PAYLOAD = 512
HEADER = struct.Struct("<HBBIBBHHhH")


class MessageType(IntEnum):
    REQUEST = 1
    RESPONSE = 2
    EVENT = 3


class Peripheral(IntEnum):
    SYSTEM = 0
    GPIO = 1
    UART = 2
    I2C = 3
    SPI = 4
    CAN = 5


class Status(IntEnum):
    OK = 0
    INVALID = -1
    UNSUPPORTED = -2
    BUSY = -3
    TIMEOUT = -4
    IO = -5
    NOT_FOUND = -6


class CanOperation(IntEnum):
    GET_CAPABILITIES = 1
    CONFIGURE = 2
    START = 3
    STOP = 4
    TRANSMIT = 5
    SET_FILTER = 6
    GET_STATE = 7
    RECOVER = 8
    RX_EVENT = 0x81
    TX_EVENT = 0x82
    STATE_EVENT = 0x83


@dataclass(slots=True)
class Packet:
    message_type: int
    sequence: int
    peripheral: int
    operation: int
    endpoint: int = 0
    channel: int = 0
    status: int = 0
    payload: bytes = b""

    def encode(self) -> bytes:
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError("payload exceeds protocol limit")
        body = HEADER.pack(MAGIC, VERSION, self.message_type, self.sequence,
                           self.peripheral, self.operation, self.endpoint,
                           self.channel, self.status, len(self.payload)) + self.payload
        return body + struct.pack("<H", crc16(body))

    @classmethod
    def decode(cls, data: bytes) -> "Packet":
        if len(data) < HEADER.size + 2:
            raise ValueError("packet is too short")
        fields = HEADER.unpack_from(data)
        magic, version, message_type, sequence, peripheral, operation, endpoint, channel, status, length = fields
        if magic != MAGIC or version != VERSION or length > MAX_PAYLOAD:
            raise ValueError("invalid protocol header")
        if len(data) != HEADER.size + length + 2:
            raise ValueError("invalid packet length")
        if crc16(data[:-2]) != struct.unpack_from("<H", data, len(data) - 2)[0]:
            raise ValueError("CRC mismatch")
        return cls(message_type, sequence, peripheral, operation, endpoint, channel,
                   status, data[HEADER.size:-2])


def crc16(data: bytes) -> int:
    """Return CRC-16/CCITT-FALSE for *data*."""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """COBS-encode bytes without appending the wire delimiter."""
    output = bytearray(b"\x00")
    code_index = 0
    code = 1
    for value in data:
        if value == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(value)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    """Decode one COBS frame that does not include its delimiter."""
    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0 or index + code > len(data) + 1:
            raise ValueError("invalid COBS frame")
        index += 1
        end = index + code - 1
        if end > len(data):
            raise ValueError("truncated COBS frame")
        output.extend(data[index:end])
        index = end
        if code != 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


def encode_wire(packet: Packet) -> bytes:
    """Encode one packet for a zero-delimited serial stream."""
    return cobs_encode(packet.encode()) + b"\x00"
