"""Pure-Python protocol models and codecs for GMP Data Link services."""

from __future__ import annotations

import csv
import struct
from dataclasses import dataclass
from enum import IntEnum
from os import PathLike
from typing import Sequence


class AccessPermission(IntEnum):
    """Target-reported resource access permission."""

    READ_ONLY = 0
    READ_WRITE = 1


class TunableDataType(IntEnum):
    """Wire types supported by the GMP Tunable service."""

    U16 = 0
    I16 = 1
    U32 = 2
    I32 = 3
    F32 = 4

    @property
    def struct_format(self) -> str:
        """Return the little-endian ``struct`` format for this type."""
        return {
            self.U16: "<H",
            self.I16: "<h",
            self.U32: "<I",
            self.I32: "<i",
            self.F32: "<f",
        }[self]

    @property
    def byte_size(self) -> int:
        """Return the encoded value size in protocol bytes."""
        return struct.calcsize(self.struct_format)


class ScopeSampleType(IntEnum):
    """Sample encodings reported by the Scope service."""

    RAW = 0
    U8 = 1
    I8 = 2
    U16 = 3
    I16 = 4
    U32 = 5
    I32 = 6
    F32 = 7
    F64 = 8

    @property
    def struct_code(self) -> str:
        """Return the scalar ``struct`` code for this sample type."""
        return {
            self.RAW: "B",
            self.U8: "B",
            self.I8: "b",
            self.U16: "H",
            self.I16: "h",
            self.U32: "I",
            self.I32: "i",
            self.F32: "f",
            self.F64: "d",
        }[self]

    @property
    def byte_size(self) -> int:
        """Return the encoded sample size in protocol bytes."""
        return struct.calcsize("<" + self.struct_code)


class ScopeLayout(IntEnum):
    """Channel layouts supported by the Scope service."""

    LINEAR = 0
    STRUCTURE_OF_ARRAYS = 1
    INTERLEAVED = 2


class ScopeCaptureState(IntEnum):
    """Target Scope acquisition states."""

    WAITING = 0
    CAPTURING = 1
    READY = 2


class ScopeTriggerMode(IntEnum):
    """Standard trigger modes used by the CTL DSA Scope backend."""

    IMMEDIATE = 0
    RISING_EDGE = 1
    FALLING_EDGE = 2
    RISING_EDGE_AUTO = 3
    FALLING_EDGE_AUTO = 4


@dataclass(frozen=True)
class TunableParameter:
    """One target-registered Tunable parameter."""

    item_id: int
    data_type: TunableDataType
    permission: AccessPermission
    name: str
    unit: str = ""


@dataclass(frozen=True)
class MemoryRegion:
    """One target-registered Memory Perspective whitelist region."""

    region_id: int
    address: int
    byte_length: int
    permission: AccessPermission
    name: str

    def contains(self, address: int, byte_length: int) -> bool:
        """Return whether an access is fully contained in this region."""
        return (
            byte_length >= 0
            and address >= self.address
            and address + byte_length <= self.address + self.byte_length
        )


@dataclass(frozen=True)
class ScopeResource:
    """One target-registered Data Link Scope resource."""

    resource_id: int
    protocol_version: int
    sample_type: ScopeSampleType
    layout: ScopeLayout
    channels: int
    depth: int
    sample_rate_hz: int
    byte_length: int
    name: str


@dataclass(frozen=True)
class ScopeConfiguration:
    """Editable trigger and sampling configuration for one Scope capture."""

    mode: ScopeTriggerMode = ScopeTriggerMode.IMMEDIATE
    trigger_channel: int = 0
    trigger_level: float = 0.0
    trigger_position_percent: float = 50.0
    auto_timeout_ms: int = 1000
    sample_divider: int = 0


@dataclass(frozen=True)
class ScopeStatus:
    """Current target Scope state and completed-frame generation."""

    state: ScopeCaptureState
    generation: int


@dataclass(frozen=True)
class ScopeFrame:
    """One decoded multi-channel Scope snapshot."""

    resource: ScopeResource
    generation: int
    sample_rate_hz: float
    time_seconds: tuple[float, ...]
    channels: tuple[tuple[int | float, ...], ...]

    def save_csv(self, path: str | PathLike[str]) -> None:
        """Write this frame to a UTF-8 CSV file."""
        with open(path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                ["generation", "time_seconds"]
                + [f"channel_{index}" for index in range(len(self.channels))]
            )
            for sample_index, time_value in enumerate(self.time_seconds):
                writer.writerow(
                    [self.generation, time_value]
                    + [channel[sample_index] for channel in self.channels]
                )


MEMORY_DESCRIPTOR_V1 = struct.Struct("<BBBBIIBBBHHIBB")
MEMORY_DESCRIPTOR_V2 = struct.Struct("<BBBBIIBB")
TUNABLE_DESCRIPTOR_V1 = struct.Struct("<BBBBBBBB")
TUNABLE_DESCRIPTOR_V2 = struct.Struct("<BBBBBBB")
SCOPE_DESCRIPTOR = struct.Struct("<BBBBBBBHIIIB")


def parse_memory_descriptor(payload: bytes) -> tuple[int, int, MemoryRegion | None]:
    """Parse a v1 or v2 Memory Perspective descriptor response."""
    if len(payload) < 4:
        raise ValueError("Memory discovery response is too short.")
    version, status, total, region_id = payload[:4]
    if version not in (1, 2):
        raise ValueError(f"Unsupported memory discovery version {version}.")
    if status != 0:
        return total, region_id, None
    header = MEMORY_DESCRIPTOR_V2 if version == 2 else MEMORY_DESCRIPTOR_V1
    if len(payload) < header.size:
        raise ValueError("Memory descriptor is truncated.")
    fields = header.unpack_from(payload)
    name_length = fields[-1]
    name_end = header.size + name_length
    if name_end > len(payload):
        raise ValueError("Memory descriptor name is truncated.")
    name = payload[header.size:name_end].decode("utf-8", errors="replace")
    return total, region_id, MemoryRegion(
        region_id=fields[3],
        address=fields[4],
        byte_length=fields[5],
        permission=AccessPermission(fields[6]),
        name=name or f"Memory Region {fields[3]}",
    )


def parse_tunable_descriptor(payload: bytes) -> tuple[int, int, TunableParameter | None]:
    """Parse a v1 or v2 Tunable descriptor response."""
    if len(payload) < 4:
        raise ValueError("Tunable discovery response is too short.")
    version, status, total, item_id = payload[:4]
    if version not in (1, 2):
        raise ValueError(f"Unsupported tunable discovery version {version}.")
    if status != 0:
        return total, item_id, None
    header = TUNABLE_DESCRIPTOR_V2 if version == 2 else TUNABLE_DESCRIPTOR_V1
    if len(payload) < header.size:
        raise ValueError("Tunable descriptor is truncated.")
    fields = header.unpack_from(payload)
    name_length = fields[6]
    unit_length = fields[7] if version == 1 else 0
    name_end = header.size + name_length
    unit_end = name_end + unit_length
    if unit_end > len(payload):
        raise ValueError("Tunable descriptor text is truncated.")
    name = payload[header.size:name_end].decode("utf-8", errors="replace")
    unit = payload[name_end:unit_end].decode("utf-8", errors="replace")
    return total, item_id, TunableParameter(
        item_id=fields[3],
        data_type=TunableDataType(fields[4]),
        permission=AccessPermission(fields[5]),
        name=name or f"Parameter {fields[3]}",
        unit=unit,
    )


def parse_scope_descriptor(payload: bytes) -> tuple[int, int, ScopeResource | None]:
    """Parse a v1 or v2 Scope descriptor response."""
    if len(payload) < 5:
        raise ValueError("Scope discovery response is too short.")
    operation, status, version, total, resource_id = payload[:5]
    if operation != 0 or version not in (1, 2):
        raise ValueError("Unsupported Scope discovery response.")
    if status != 0:
        return total, resource_id, None
    if len(payload) < SCOPE_DESCRIPTOR.size:
        raise ValueError("Scope descriptor is truncated.")
    fields = SCOPE_DESCRIPTOR.unpack_from(payload)
    name_length = fields[11]
    name_end = SCOPE_DESCRIPTOR.size + name_length
    if name_end > len(payload):
        raise ValueError("Scope descriptor name is truncated.")
    name = payload[SCOPE_DESCRIPTOR.size:name_end].decode("utf-8", errors="replace")
    return total, resource_id, ScopeResource(
        resource_id=fields[4],
        protocol_version=fields[2],
        sample_type=ScopeSampleType(fields[5]),
        layout=ScopeLayout(fields[6]),
        channels=fields[7],
        depth=fields[8],
        sample_rate_hz=fields[9],
        byte_length=fields[10],
        name=name or f"Scope {fields[4]}",
    )


def encode_scope_configuration(
    resource: ScopeResource, configuration: ScopeConfiguration
) -> bytes:
    """Encode a version-compatible Scope Configure request."""
    if not 0 <= configuration.trigger_channel < resource.channels:
        raise ValueError("The trigger channel is outside the Scope resource.")
    if not 0.0 <= configuration.trigger_position_percent <= 100.0:
        raise ValueError("The trigger position must be between 0 and 100 percent.")
    if not 0 <= configuration.auto_timeout_ms <= 0xFFFFFFFF:
        raise ValueError("The automatic trigger timeout is outside uint32 range.")
    if not 0 <= configuration.sample_divider <= 0xFFFF:
        raise ValueError("The sampling divider is outside uint16 range.")
    fields = (
        1,
        resource.resource_id,
        int(configuration.mode),
        configuration.trigger_channel,
        int(round(configuration.trigger_position_percent * 10.0)),
        float(configuration.trigger_level),
        configuration.auto_timeout_ms,
    )
    if resource.protocol_version >= 2:
        return struct.pack("<BBBBHfIH", *fields, configuration.sample_divider)
    if configuration.sample_divider != 0:
        raise ValueError("Scope protocol version 1 does not support a sampling divider.")
    return struct.pack("<BBBBHfI", *fields)


def decode_scope_channels(
    resource: ScopeResource, payload: bytes
) -> tuple[tuple[int | float, ...], ...]:
    """Decode one complete Scope payload into channel-major samples."""
    expected_samples = resource.channels * resource.depth
    expected_bytes = expected_samples * resource.sample_type.byte_size
    if len(payload) != expected_bytes:
        raise ValueError(
            f"Scope payload has {len(payload)} bytes; expected {expected_bytes}."
        )
    values: Sequence[int | float] = struct.unpack(
        f"<{expected_samples}{resource.sample_type.struct_code}", payload
    )
    if resource.layout == ScopeLayout.INTERLEAVED:
        return tuple(
            tuple(values[index * resource.channels + channel] for index in range(resource.depth))
            for channel in range(resource.channels)
        )
    return tuple(
        tuple(values[channel * resource.depth:(channel + 1) * resource.depth])
        for channel in range(resource.channels)
    )
