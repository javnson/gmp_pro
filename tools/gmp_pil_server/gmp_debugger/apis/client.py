"""Synchronous, headless client API for GMP Data Link debugging services."""

from __future__ import annotations

import struct
import threading
import time
from collections.abc import Generator, Iterable, Mapping
from typing import Protocol, TypeAlias

import serial

from .protocol import (
    AccessPermission,
    MemoryRegion,
    ScopeCaptureState,
    ScopeConfiguration,
    ScopeFrame,
    ScopeResource,
    ScopeStatus,
    TunableParameter,
    decode_scope_channels,
    encode_scope_configuration,
    parse_memory_descriptor,
    parse_scope_descriptor,
    parse_tunable_descriptor,
)

SOF = 0x7B
EOF = 0x7D
ESC = 0x25
XOR = 0x20
MAX_PAYLOAD = 256

ResourceSelector: TypeAlias = int | str | TunableParameter | MemoryRegion | ScopeResource


class GmpDlError(RuntimeError):
    """Base exception for the public GMP Data Link API."""


class GmpDlTimeout(GmpDlError):
    """Raised when a transport transaction or Scope capture times out."""


class GmpDlProtocolError(GmpDlError):
    """Raised when a target response violates the service protocol."""


class GmpDlTargetError(GmpDlError):
    """Raised when the target explicitly rejects an operation."""


def crc16_ccitt(data: bytes) -> int:
    """Return the GMP CRC16-CCITT value for ``data``."""
    crc = 0xFFFF
    for octet in data:
        crc ^= octet << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(sequence: int, command: int, payload: bytes = b"") -> bytes:
    """Encode one GMP Data Link frame for serial transmission."""
    if not 0 <= sequence <= 0xFF or not 0 <= command <= 0xFF:
        raise ValueError("Sequence and command identifiers must fit in uint8.")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"A Data Link payload cannot exceed {MAX_PAYLOAD} bytes.")
    header = struct.pack("<BBH", sequence, command, len(payload))
    header += struct.pack("<H", crc16_ccitt(header))
    encoded = bytearray((SOF,))
    for octet in header:
        if octet in (SOF, EOF, ESC):
            encoded.extend((ESC, octet ^ XOR))
        else:
            encoded.append(octet)
    encoded.append(EOF)
    if payload:
        encoded.extend(payload)
        encoded.extend(struct.pack("<H", crc16_ccitt(payload)))
    return bytes(encoded)


class TransactionTransport(Protocol):
    """Minimal transport contract accepted by :class:`GmpDatalinkClient`."""

    def transact(self, command: int, payload: bytes = b"") -> bytes:
        """Send one request and return its matching response payload."""


class SerialDataLinkTransport:
    """Reliable synchronous GMP Data Link transport over a serial port."""

    def __init__(
        self,
        port: str,
        baudrate: int = 921600,
        *,
        timeout: float = 0.6,
        retries: int = 2,
    ) -> None:
        if timeout <= 0.0:
            raise ValueError("The transaction timeout must be positive.")
        if retries < 0:
            raise ValueError("The retry count cannot be negative.")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.retries = retries
        self.serial = serial.Serial()
        self._sequence = 0
        self._lock = threading.RLock()

    @property
    def is_open(self) -> bool:
        """Return whether the serial device is open."""
        return self.serial.is_open

    def open(self) -> None:
        """Open the configured serial port and discard stale input."""
        if self.serial.is_open:
            return
        self.serial.port = self.port
        self.serial.baudrate = self.baudrate
        self.serial.bytesize = serial.EIGHTBITS
        self.serial.parity = serial.PARITY_NONE
        self.serial.stopbits = serial.STOPBITS_ONE
        self.serial.timeout = min(0.05, self.timeout)
        self.serial.write_timeout = self.timeout
        try:
            self.serial.open()
            self.serial.reset_input_buffer()
        except serial.SerialException as error:
            raise GmpDlError(
                f"Cannot open Data Link serial port {self.port!r}: {error}"
            ) from error

    def close(self) -> None:
        """Close the serial port."""
        if self.serial.is_open:
            self.serial.close()

    def __enter__(self) -> "SerialDataLinkTransport":
        self.open()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFF
        return self._sequence

    def _read_exact(self, length: int, deadline: float) -> bytes:
        result = bytearray()
        while len(result) < length and time.monotonic() < deadline:
            result.extend(self.serial.read(length - len(result)))
        return bytes(result)

    def _read_frame(self, deadline: float) -> tuple[int, int, bytes]:
        while time.monotonic() < deadline:
            if self.serial.read(1) != bytes((SOF,)):
                continue
            header = bytearray()
            escaped = False
            while time.monotonic() < deadline:
                raw = self.serial.read(1)
                if not raw:
                    continue
                octet = raw[0]
                if escaped:
                    header.append(octet ^ XOR)
                    escaped = False
                elif octet == ESC:
                    escaped = True
                elif octet == EOF:
                    break
                elif octet == SOF:
                    header.clear()
                elif len(header) < 6:
                    header.append(octet)
                else:
                    header.clear()
                    break
            if len(header) != 6:
                continue
            if crc16_ccitt(header[:4]) != struct.unpack_from("<H", header, 4)[0]:
                continue
            sequence, command, payload_length = struct.unpack_from("<BBH", header)
            if payload_length > MAX_PAYLOAD:
                continue
            if payload_length == 0:
                return sequence, command, b""
            payload_crc = self._read_exact(payload_length + 2, deadline)
            if len(payload_crc) != payload_length + 2:
                continue
            payload = payload_crc[:-2]
            if crc16_ccitt(payload) != struct.unpack_from("<H", payload_crc, payload_length)[0]:
                continue
            return sequence, command, payload
        raise GmpDlTimeout("No valid Data Link response was received before the deadline.")

    def transact(self, command: int, payload: bytes = b"") -> bytes:
        """Send one request with bounded retries and return its response payload."""
        with self._lock:
            if not self.serial.is_open:
                raise GmpDlError("The serial Data Link transport is not open.")
            sequence = self._next_sequence()
            frame = encode_frame(sequence, command, payload)
            try:
                for attempt in range(self.retries + 1):
                    written = self.serial.write(frame)
                    if written != len(frame):
                        raise GmpDlError(
                            f"Serial write accepted {written} of {len(frame)} Data Link bytes."
                        )
                    deadline = time.monotonic() + self.timeout
                    try:
                        while time.monotonic() < deadline:
                            response_sequence, response_command, response_payload = (
                                self._read_frame(deadline)
                            )
                            if response_sequence != sequence:
                                continue
                            if response_command == command:
                                return response_payload
                            if response_command == 0x01:
                                raise GmpDlTargetError(
                                    f"Target returned NACK for command 0x{command:02X}: "
                                    f"{response_payload.hex(' ')}"
                                )
                    except GmpDlTimeout:
                        if attempt >= self.retries:
                            raise
            except serial.SerialException as error:
                raise GmpDlError(
                    f"Serial communication failed on {self.port!r}: {error}"
                ) from error
            raise GmpDlTimeout(f"Command 0x{command:02X} timed out.")


def _resolve_by_id_or_name(items: Iterable[object], selector: ResourceSelector, id_name: str) -> object:
    """Resolve a descriptor by object, integer ID, or exact display name."""
    item_list = list(items)
    if selector in item_list:
        return selector
    if isinstance(selector, int):
        matches = [item for item in item_list if getattr(item, id_name) == selector]
    elif isinstance(selector, str):
        matches = [item for item in item_list if getattr(item, "name") == selector]
    else:
        matches = []
    if len(matches) != 1:
        raise KeyError(f"Resource selector {selector!r} matched {len(matches)} entries.")
    return matches[0]


class TunableApi:
    """Discover, read, and write target Tunable parameters."""

    def __init__(self, transport: TransactionTransport, base_command: int = 0x30) -> None:
        self.transport = transport
        self.base_command = base_command
        self.parameters: list[TunableParameter] = []

    def discover(self) -> list[TunableParameter]:
        """Discover and cache the complete target Tunable table."""
        parameters: list[TunableParameter] = []
        next_id = 0
        while True:
            payload = self.transport.transact(self.base_command + 1, bytes((next_id,)))
            try:
                total, item_id, parameter = parse_tunable_descriptor(payload)
            except ValueError as error:
                raise GmpDlProtocolError(str(error)) from error
            if total == 0 and parameter is None:
                break
            if item_id != next_id or parameter is None:
                raise GmpDlTargetError(f"Target rejected Tunable descriptor {next_id}.")
            parameters.append(parameter)
            next_id += 1
            if next_id >= total:
                break
        self.parameters = parameters
        return list(parameters)

    def resolve(self, selector: ResourceSelector) -> TunableParameter:
        """Resolve one parameter by descriptor, ID, or exact name."""
        if not self.parameters:
            self.discover()
        return _resolve_by_id_or_name(self.parameters, selector, "item_id")  # type: ignore[return-value]

    def read_many(self, selectors: Iterable[ResourceSelector]) -> dict[int, int | float]:
        """Read selected parameters and return values keyed by parameter ID."""
        parameters = [self.resolve(selector) for selector in selectors]
        result: dict[int, int | float] = {}
        for start in range(0, len(parameters), 40):
            batch = parameters[start:start + 40]
            response = self.transport.transact(
                self.base_command, bytes((len(batch), *(item.item_id for item in batch)))
            )
            if not response:
                raise GmpDlProtocolError("Tunable read response is empty.")
            count = response[0]
            index = 1
            for _ in range(count):
                if index >= len(response):
                    raise GmpDlProtocolError("Tunable read response is truncated.")
                item_id = response[index]
                index += 1
                parameter = self.resolve(item_id)
                value_size = parameter.data_type.byte_size
                if index + value_size > len(response):
                    raise GmpDlProtocolError("Tunable value is truncated.")
                result[item_id] = struct.unpack_from(
                    parameter.data_type.struct_format, response, index
                )[0]
                index += value_size
            if count != len(batch):
                raise GmpDlTargetError(
                    f"Target returned {count} of {len(batch)} requested Tunable values."
                )
        return result

    def read(self, selector: ResourceSelector) -> int | float:
        """Read one parameter selected by descriptor, ID, or name."""
        parameter = self.resolve(selector)
        return self.read_many((parameter,))[parameter.item_id]

    def read_all(self) -> dict[str, int | float]:
        """Read the complete table and return values keyed by display name."""
        if not self.parameters:
            self.discover()
        values = self.read_many(self.parameters)
        return {parameter.name: values[parameter.item_id] for parameter in self.parameters}

    def write_many(self, values: Mapping[ResourceSelector, int | float]) -> None:
        """Write one or more read-write parameters in bounded batches."""
        entries = [(self.resolve(selector), value) for selector, value in values.items()]
        for parameter, _value in entries:
            if parameter.permission != AccessPermission.READ_WRITE:
                raise PermissionError(f"Tunable parameter {parameter.name!r} is read-only.")
        for start in range(0, len(entries), 40):
            batch = entries[start:start + 40]
            request = bytearray((len(batch),))
            for parameter, value in batch:
                request.append(parameter.item_id)
                request.extend(struct.pack(parameter.data_type.struct_format, value))
            response = self.transport.transact(self.base_command + 1, bytes(request))
            if response != b"\x00":
                raise GmpDlTargetError(
                    f"Target rejected a Tunable write: {response.hex(' ')}"
                )

    def write(self, selector: ResourceSelector, value: int | float) -> None:
        """Write one read-write parameter."""
        self.write_many({selector: value})


class MemoryApi:
    """Discover and access target Memory Perspective whitelist regions."""

    def __init__(self, transport: TransactionTransport, base_command: int = 0x50) -> None:
        self.transport = transport
        self.base_command = base_command
        self.regions: list[MemoryRegion] = []

    def discover(self) -> list[MemoryRegion]:
        """Discover and cache the complete target memory whitelist."""
        regions: list[MemoryRegion] = []
        next_id = 0
        while True:
            payload = self.transport.transact(self.base_command + 1, bytes((next_id,)))
            try:
                total, region_id, region = parse_memory_descriptor(payload)
            except ValueError as error:
                raise GmpDlProtocolError(str(error)) from error
            if total == 0 and region is None:
                break
            if region_id != next_id or region is None:
                raise GmpDlTargetError(f"Target rejected memory descriptor {next_id}.")
            regions.append(region)
            next_id += 1
            if next_id >= total:
                break
        self.regions = regions
        return list(regions)

    def resolve(self, selector: ResourceSelector) -> MemoryRegion:
        """Resolve one memory region by descriptor, ID, or exact name."""
        if not self.regions:
            self.discover()
        return _resolve_by_id_or_name(self.regions, selector, "region_id")  # type: ignore[return-value]

    def _validate_access(self, address: int, byte_length: int, write: bool) -> None:
        if address < 0 or byte_length < 0:
            raise ValueError("Memory address and length must be non-negative.")
        if not self.regions:
            return
        matches = [region for region in self.regions if region.contains(address, byte_length)]
        if not matches:
            raise ValueError("Memory access is outside every discovered target region.")
        if write and matches[0].permission != AccessPermission.READ_WRITE:
            raise PermissionError(f"Memory region {matches[0].name!r} is read-only.")

    def read(self, address: int, byte_length: int, *, chunk_size: int = 240) -> bytes:
        """Read raw protocol bytes from one whitelisted target address range."""
        if not 1 <= chunk_size <= 255:
            raise ValueError("Memory read chunk size must be between 1 and 255 bytes.")
        self._validate_access(address, byte_length, write=False)
        result = bytearray()
        while len(result) < byte_length:
            length = min(chunk_size, byte_length - len(result))
            response = self.transport.transact(
                self.base_command, struct.pack("<IBH", address + len(result), 1, length)
            )
            if not response or response[0] != 0:
                raise GmpDlTargetError("Target rejected a Memory Perspective read.")
            data = response[1:]
            if len(data) != length:
                raise GmpDlProtocolError("Memory read response length is inconsistent.")
            result.extend(data)
        return bytes(result)

    def write(self, address: int, data: bytes, *, chunk_size: int = 240) -> None:
        """Write raw protocol bytes to one read-write target address range."""
        if not 1 <= chunk_size <= 249:
            raise ValueError("Memory write chunk size must be between 1 and 249 bytes.")
        self._validate_access(address, len(data), write=True)
        offset = 0
        while offset < len(data):
            chunk = data[offset:offset + chunk_size]
            request = struct.pack("<IBH", address + offset, 1, len(chunk)) + chunk
            response = self.transport.transact(self.base_command + 1, request)
            if response != b"\x00":
                raise GmpDlTargetError(
                    f"Target rejected a Memory Perspective write: {response.hex(' ')}"
                )
            offset += len(chunk)

    def read_region(
        self,
        selector: ResourceSelector,
        *,
        offset: int = 0,
        byte_length: int | None = None,
    ) -> bytes:
        """Read all or part of a discovered memory region."""
        region = self.resolve(selector)
        length = region.byte_length - offset if byte_length is None else byte_length
        return self.read(region.address + offset, length)

    def write_region(self, selector: ResourceSelector, data: bytes, *, offset: int = 0) -> None:
        """Write bytes at an offset inside a discovered memory region."""
        region = self.resolve(selector)
        self.write(region.address + offset, data)


class ScopeApi:
    """Discover, configure, capture, and decode target Scope resources."""

    OP_DISCOVER = 0
    OP_CONFIGURE = 1
    OP_ARM = 2
    OP_STATUS = 3
    OP_READ = 4

    def __init__(self, transport: TransactionTransport, base_command: int = 0x60) -> None:
        self.transport = transport
        self.base_command = base_command
        self.resources: list[ScopeResource] = []
        self.configurations: dict[int, ScopeConfiguration] = {}

    def discover(self) -> list[ScopeResource]:
        """Discover and cache all target Scope resources."""
        resources: list[ScopeResource] = []
        next_id = 0
        while True:
            payload = self.transport.transact(
                self.base_command, bytes((self.OP_DISCOVER, next_id))
            )
            try:
                total, resource_id, resource = parse_scope_descriptor(payload)
            except ValueError as error:
                raise GmpDlProtocolError(str(error)) from error
            if total == 0 and resource is None:
                break
            if resource_id != next_id or resource is None:
                raise GmpDlTargetError(f"Target rejected Scope descriptor {next_id}.")
            resources.append(resource)
            next_id += 1
            if next_id >= total:
                break
        self.resources = resources
        return list(resources)

    def resolve(self, selector: ResourceSelector) -> ScopeResource:
        """Resolve one Scope resource by descriptor, ID, or exact name."""
        if not self.resources:
            self.discover()
        return _resolve_by_id_or_name(self.resources, selector, "resource_id")  # type: ignore[return-value]

    def configure(
        self, selector: ResourceSelector, configuration: ScopeConfiguration
    ) -> ScopeResource:
        """Apply trigger and sampling configuration to one Scope resource."""
        resource = self.resolve(selector)
        request = encode_scope_configuration(resource, configuration)
        response = self.transport.transact(self.base_command, request)
        expected = bytes((self.OP_CONFIGURE, 0, resource.resource_id))
        if response != expected:
            raise GmpDlTargetError(
                f"Target rejected Scope configuration: {response.hex(' ')}"
            )
        self.configurations[resource.resource_id] = configuration
        return resource

    def arm(self, selector: ResourceSelector) -> None:
        """Arm one Scope resource and clear its previous acquisition state."""
        resource = self.resolve(selector)
        response = self.transport.transact(
            self.base_command, bytes((self.OP_ARM, resource.resource_id))
        )
        if response != bytes((self.OP_ARM, 0, resource.resource_id)):
            raise GmpDlTargetError(f"Target rejected Scope arm: {response.hex(' ')}")

    def status(self, selector: ResourceSelector) -> ScopeStatus:
        """Read one Scope resource's acquisition state and generation."""
        resource = self.resolve(selector)
        response = self.transport.transact(
            self.base_command, bytes((self.OP_STATUS, resource.resource_id))
        )
        if len(response) != 8:
            raise GmpDlProtocolError("Scope status response has an invalid length.")
        operation, status, resource_id, state, generation = struct.unpack("<BBBBI", response)
        if (operation, status, resource_id) != (self.OP_STATUS, 0, resource.resource_id):
            raise GmpDlTargetError(f"Target rejected Scope status: {response.hex(' ')}")
        return ScopeStatus(ScopeCaptureState(state), generation)

    def read_snapshot(self, selector: ResourceSelector, generation: int | None = None) -> ScopeFrame:
        """Download and decode the ready snapshot of one Scope resource."""
        resource = self.resolve(selector)
        expected_bytes = (
            resource.channels * resource.depth * resource.sample_type.byte_size
        )
        if expected_bytes > resource.byte_length:
            raise GmpDlProtocolError("Scope metadata exceeds its registered buffer capacity.")
        raw = bytearray(expected_bytes)
        read_header = struct.Struct("<BBBIH")
        offset = 0
        while offset < expected_bytes:
            length = min(180, expected_bytes - offset)
            request = struct.pack(
                "<BBIH", self.OP_READ, resource.resource_id, offset, length
            )
            response = self.transport.transact(self.base_command, request)
            if len(response) != read_header.size + length:
                raise GmpDlProtocolError("Scope read response has an invalid length.")
            operation, status, resource_id, returned_offset, returned_length = (
                read_header.unpack_from(response)
            )
            if (
                operation,
                status,
                resource_id,
                returned_offset,
                returned_length,
            ) != (self.OP_READ, 0, resource.resource_id, offset, length):
                raise GmpDlTargetError(f"Target rejected Scope read: {response.hex(' ')}")
            raw[offset:offset + length] = response[read_header.size:]
            offset += length
        configuration = self.configurations.get(resource.resource_id, ScopeConfiguration())
        effective_rate = resource.sample_rate_hz / float(configuration.sample_divider + 1)
        channels = decode_scope_channels(resource, bytes(raw))
        times = tuple(index / effective_rate for index in range(resource.depth))
        return ScopeFrame(
            resource=resource,
            generation=0 if generation is None else generation,
            sample_rate_hz=effective_rate,
            time_seconds=times,
            channels=channels,
        )

    def _wait_until_ready(
        self,
        resource: ScopeResource,
        *,
        timeout: float,
        poll_interval: float,
    ) -> int:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.status(resource)
            if status.state == ScopeCaptureState.READY:
                return status.generation
            time.sleep(poll_interval)
        raise GmpDlTimeout(
            f"Scope {resource.name!r} did not become ready within {timeout:g} seconds."
        )

    def capture(
        self,
        selector: ResourceSelector = 0,
        configuration: ScopeConfiguration | None = None,
        *,
        timeout: float = 5.0,
        poll_interval: float = 0.05,
    ) -> ScopeFrame:
        """Configure, arm, wait for, download, and decode one Scope frame."""
        resource = self.resolve(selector)
        selected_configuration = configuration or ScopeConfiguration()
        self.configure(resource, selected_configuration)
        self.arm(resource)
        generation = self._wait_until_ready(
            resource, timeout=timeout, poll_interval=poll_interval
        )
        return self.read_snapshot(resource, generation)

    def iter_captures(
        self,
        selector: ResourceSelector = 0,
        configuration: ScopeConfiguration | None = None,
        *,
        count: int | None = None,
        timeout: float = 5.0,
        poll_interval: float = 0.05,
    ) -> Generator[ScopeFrame, None, None]:
        """Yield repeatedly armed Scope frames, optionally for a bounded count."""
        if count is not None and count < 0:
            raise ValueError("Continuous capture count cannot be negative.")
        resource = self.resolve(selector)
        self.configure(resource, configuration or ScopeConfiguration())
        completed = 0
        while count is None or completed < count:
            self.arm(resource)
            generation = self._wait_until_ready(
                resource, timeout=timeout, poll_interval=poll_interval
            )
            yield self.read_snapshot(resource, generation)
            completed += 1


class GmpDatalinkClient:
    """Facade exposing Tunable, Memory, and Scope APIs over one transport."""

    def __init__(
        self,
        port: str | None = None,
        baudrate: int = 921600,
        *,
        timeout: float = 0.6,
        retries: int = 2,
        transport: TransactionTransport | None = None,
        tunable_command: int = 0x30,
        memory_command: int = 0x50,
        scope_command: int = 0x60,
    ) -> None:
        if transport is None:
            if port is None:
                raise ValueError("A serial port or custom transaction transport is required.")
            transport = SerialDataLinkTransport(
                port, baudrate, timeout=timeout, retries=retries
            )
        self.transport = transport
        self.tunables = TunableApi(transport, tunable_command)
        self.memory = MemoryApi(transport, memory_command)
        self.scope = ScopeApi(transport, scope_command)

    def open(self) -> None:
        """Open the transport when it provides an ``open`` method."""
        open_method = getattr(self.transport, "open", None)
        if open_method is not None:
            open_method()

    def close(self) -> None:
        """Close the transport when it provides a ``close`` method."""
        close_method = getattr(self.transport, "close", None)
        if close_method is not None:
            close_method()

    def transact(self, command: int, payload: bytes = b"") -> bytes:
        """Expose a raw synchronous transaction for custom DL submodules."""
        return self.transport.transact(command, payload)

    def __enter__(self) -> "GmpDatalinkClient":
        self.open()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()
