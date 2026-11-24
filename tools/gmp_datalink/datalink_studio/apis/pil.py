"""Headless GMP Processor-in-the-Loop API and Simulink UDP bridge."""

from __future__ import annotations

import csv
import json
import socket
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

from .client import GmpDlProtocolError, TransactionTransport

MATLAB_INPUT_STRUCT = struct.Struct("<d24I16d8i")
MATLAB_OUTPUT_STRUCT = struct.Struct("<d8I8I16d")

# Match tools/gmp_sil/sil_helper/gmp_sil_protocol.hpp. Only the frame envelope
# is big-endian; the existing packed Simulink samples remain little-endian.
SIL_HEADER = struct.Struct("!4sHHIQI")
SIL_SESSION = struct.Struct("!HHII16s")
SIL_STATE = struct.Struct("!HHIQ")
SIL_MAGIC = b"GMPT"
SIL_VERSION = 1
SIL_DATA_REQUEST, SIL_DATA_RESPONSE = 1, 2
SIL_SIMULATION_STATE = 8
SIL_HELLO, SIL_HELLO_RESPONSE = 10, 11


def _read_json(path: Path) -> dict[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _macro_values(document: dict[str, object]) -> dict[str, object]:
    values: dict[str, object] = {}
    for item in document.get("requirements", []):
        binding = item.get("binding", {})
        for kind in ("number", "float", "parameter", "literal", "string"):
            if kind in binding:
                raw = binding[kind]
                if kind == "string":
                    values[item["macro"]] = raw
                else:
                    try:
                        values[item["macro"]] = float(raw)
                    except (TypeError, ValueError):
                        pass
                break
    return values


def _integer(values: dict[str, object], macro: str) -> int:
    try:
        return int(values[macro])
    except KeyError as error:
        raise ValueError(f"Required SDPE macro {macro} is missing.") from error


def _semantic_indices(values: dict[str, object], prefix: str) -> dict[str, int]:
    """Collect optional suite-specific channel names from SDPE macros."""
    indices: dict[str, int] = {}
    for macro, value in values.items():
        if macro.startswith(prefix) and macro.endswith("_INDEX"):
            name = macro[len(prefix) : -len("_INDEX")].lower()
            indices[name] = int(value)
    return indices


@dataclass(frozen=True)
class PilConfiguration:
    """Resolved target-local SDPE contract for one PIL bridge."""

    requirement_path: Path
    enabled: bool
    build_level: int
    controller_frequency_hz: float
    serial_baudrate: int
    base_command: int
    udp_host: str
    bridge_listen_port: int
    matlab_listen_port: int
    command_tx_port: int
    command_rx_port: int
    rx_mask: int
    tx_mask: int
    mcu_timeout_s: float
    matlab_timeout_s: float
    encoder_udp_index: int
    adc_indices: dict[str, int]
    pwm_indices: dict[str, int]
    monitor_indices: dict[str, int]

    @property
    def sample_time_s(self) -> float:
        """Return the controller sample interval represented by one PIL step."""
        return 1.0 / self.controller_frequency_hz

    @classmethod
    def from_sdpe(
        cls,
        target_requirement: str | Path,
        common_requirement: str | Path | None = None,
    ) -> "PilConfiguration":
        """Load and validate the target and common SDPE requirement sources."""
        target_path = Path(target_requirement).resolve()
        target = _read_json(target_path)
        if common_requirement is None:
            references = target.get("common_requirements", [])
            if isinstance(references, str):
                references = [references]
            common_paths = [
                (target_path.parent / str(reference)).resolve()
                for reference in references
            ]
            if not common_paths:
                common_paths = [target_path.parents[3] / "sdpe_general" / "sdpe_requirement.json"]
        else:
            common_paths = [Path(common_requirement).resolve()]
        commons = [_read_json(path) for path in common_paths]
        common_values: dict[str, object] = {}
        for common in commons:
            common_values.update(_macro_values(common))
        target_values = _macro_values(target)
        values = {**common_values, **target_values}

        features = {
            item["macro"]: bool(item.get("enabled"))
            for document in (*commons, target)
            for item in document.get("feature_macros", [])
        }
        enabled = any(
            features.get(macro, False)
            for macro in ("ENABLE_GMP_DL_PIL_SIM", "ENABLE_GMP_DL_PIL_SERVER")
        )
        levels = {
            item["macro"]: item.get("value", "")
            for document in (*commons, target)
            for item in document.get("option_macros", [])
        }
        build_level = int(str(levels.get("BUILD_LEVEL", "0")).strip("() "))

        digital_index_macro = (
            "GMP_PIL_UDP_DIGITAL_INDEX"
            if "GMP_PIL_UDP_DIGITAL_INDEX" in values
            else "GMP_PIL_UDP_ENCODER_INDEX"
        )

        config = cls(
            requirement_path=target_path,
            enabled=enabled,
            build_level=build_level,
            controller_frequency_hz=float(values["CONTROLLER_FREQUENCY"]),
            serial_baudrate=_integer(values, "GMP_DL_UART_BAUDRATE"),
            base_command=_integer(values, "GMP_PIL_DL_BASE_COMMAND"),
            udp_host=str(values["GMP_PIL_UDP_HOST"]),
            bridge_listen_port=_integer(values, "GMP_PIL_BRIDGE_UDP_LISTEN_PORT"),
            matlab_listen_port=_integer(values, "GMP_PIL_MATLAB_UDP_LISTEN_PORT"),
            command_tx_port=_integer(values, "GMP_PIL_MATLAB_COMMAND_TX_PORT"),
            command_rx_port=_integer(values, "GMP_PIL_MATLAB_COMMAND_RX_PORT"),
            rx_mask=_integer(values, "GMP_PIL_RX_MASK"),
            tx_mask=_integer(values, "GMP_PIL_TX_MASK"),
            mcu_timeout_s=_integer(values, "GMP_PIL_MCU_TIMEOUT_MS") / 1000.0,
            matlab_timeout_s=_integer(values, "GMP_PIL_MATLAB_TIMEOUT_MS") / 1000.0,
            encoder_udp_index=_integer(values, digital_index_macro),
            adc_indices=_semantic_indices(values, "GMP_PIL_RX_ADC_"),
            pwm_indices=_semantic_indices(values, "GMP_PIL_TX_PWM_"),
            monitor_indices=_semantic_indices(values, "GMP_PIL_TX_MONITOR_"),
        )
        config.validate()
        return config

    def validate(self) -> None:
        """Reject an unsafe or internally inconsistent PIL mapping."""
        if not self.enabled:
            raise ValueError(
                "Neither ENABLE_GMP_DL_PIL_SIM nor ENABLE_GMP_DL_PIL_SERVER "
                "is enabled in the target SDPE requirement."
            )
        if self.build_level < 1:
            raise ValueError("BUILD_LEVEL must be positive.")
        if self.controller_frequency_hz <= 0.0:
            raise ValueError("CONTROLLER_FREQUENCY must be positive.")
        if not 0 <= self.base_command <= 251:
            raise ValueError("The PIL base command must be in the range [0, 251].")
        if not 0 <= self.encoder_udp_index < 8:
            raise ValueError("The UDP digital-input index must be in the range [0, 7].")
        for index in self.adc_indices.values():
            if not 0 <= index < 24 or not self.rx_mask & (1 << index):
                raise ValueError(f"ADC index {index} is outside or disabled by GMP_PIL_RX_MASK.")
        for index in self.pwm_indices.values():
            if not 0 <= index < 8 or not self.tx_mask & (1 << index):
                raise ValueError(f"PWM index {index} is outside or disabled by GMP_PIL_TX_MASK.")
        for index in self.monitor_indices.values():
            if not 0 <= index < 16 or not self.tx_mask & (1 << (16 + index)):
                raise ValueError(f"Monitor index {index} is outside or disabled by GMP_PIL_TX_MASK.")


@dataclass(frozen=True)
class PilInput:
    """One standard Simulink-to-target PIL sample."""

    simulation_time_s: float
    adc: tuple[int, ...]
    panel: tuple[float, ...]
    digital: tuple[int, ...]

    @classmethod
    def from_matlab_datagram(cls, payload: bytes) -> "PilInput":
        """Decode the standard 264-byte GMP Simulink datagram."""
        if len(payload) != MATLAB_INPUT_STRUCT.size:
            raise GmpDlProtocolError(
                f"Expected {MATLAB_INPUT_STRUCT.size} MATLAB input bytes, received {len(payload)}."
            )
        fields = MATLAB_INPUT_STRUCT.unpack(payload)
        return cls(
            simulation_time_s=fields[0],
            adc=tuple(int(value) & 0xFFFF for value in fields[1:25]),
            panel=tuple(float(value) for value in fields[25:41]),
            digital=tuple(int(value) for value in fields[41:49]),
        )

    def to_datalink_payload(self, config: PilConfiguration) -> bytes:
        """Encode the channels enabled by the target RX mask."""
        payload = bytearray(
            struct.pack(
                "<II",
                round(self.simulation_time_s * config.controller_frequency_hz),
                self.digital[config.encoder_udp_index] & 0xFFFFFFFF,
            )
        )
        for index, value in enumerate(self.adc):
            if config.rx_mask & (1 << index):
                payload.extend(struct.pack("<H", value))
        for index, value in enumerate(self.panel[:8]):
            if config.rx_mask & (1 << (24 + index)):
                payload.extend(struct.pack("<f", value))
        return bytes(payload)


@dataclass(frozen=True)
class PilOutput:
    """One target-to-Simulink PIL result."""

    digital: int
    pwm: tuple[int, ...]
    dac: tuple[int, ...]
    monitor: tuple[float, ...]

    @classmethod
    def from_datalink_payload(cls, payload: bytes, tx_mask: int) -> "PilOutput":
        """Decode one exact masked target response."""
        expected = 4 + 2 * (tx_mask & 0xFFFF).bit_count() + 4 * ((tx_mask >> 16) & 0xFFFF).bit_count()
        if len(payload) != expected:
            raise GmpDlProtocolError(f"Expected {expected} PIL output bytes, received {len(payload)}.")
        digital = struct.unpack_from("<I", payload)[0]
        offset = 4
        pwm = [0] * 8
        dac = [0] * 8
        monitor = [0.0] * 16
        for index in range(8):
            if tx_mask & (1 << index):
                pwm[index] = struct.unpack_from("<H", payload, offset)[0]
                offset += 2
        for index in range(8):
            if tx_mask & (1 << (8 + index)):
                dac[index] = struct.unpack_from("<H", payload, offset)[0]
                offset += 2
        for index in range(16):
            if tx_mask & (1 << (16 + index)):
                monitor[index] = struct.unpack_from("<f", payload, offset)[0]
                offset += 4
        return cls(digital, tuple(pwm), tuple(dac), tuple(monitor))

    def to_matlab_datagram(self) -> bytes:
        """Encode the standard 200-byte GMP target-to-Simulink datagram."""
        return MATLAB_OUTPUT_STRUCT.pack(float(self.digital), *self.pwm, *self.dac, *self.monitor)


class PilApi:
    """Synchronous Data Link API for the GMP PIL submodule."""

    def __init__(self, transport: TransactionTransport, config: PilConfiguration) -> None:
        self.transport = transport
        self.config = config

    def synchronize_masks(self) -> tuple[int, int]:
        """Apply the SDPE masks and verify the target accepted them unchanged."""
        request = struct.pack("<II", self.config.tx_mask, self.config.rx_mask)
        response = self.transport.transact(self.config.base_command + 1, request)
        if len(response) != 8:
            raise GmpDlProtocolError("The target returned an invalid PIL mask response.")
        masks = struct.unpack("<II", response)
        if masks != (self.config.tx_mask, self.config.rx_mask):
            raise GmpDlProtocolError(f"The target applied unexpected PIL masks {masks!r}.")
        return masks

    def step(self, sample: PilInput) -> PilOutput:
        """Execute exactly one target controller step."""
        response = self.transport.transact(
            self.config.base_command + 2,
            sample.to_datalink_payload(self.config),
        )
        return PilOutput.from_datalink_payload(response, self.config.tx_mask)


class PilTraceWriter:
    """Write portable, channel-complete CSV evidence for a PIL run."""

    def __init__(self, stream: TextIO, config: PilConfiguration) -> None:
        self.stream = stream
        self.config = config
        self.writer = csv.DictWriter(
            stream,
            fieldnames=(
                [
                    "sequence",
                    "simulation_time_s",
                    "target_roundtrip_ms",
                    "encoder_digital_input",
                    "digital_output",
                ]
                + [f"adc_{index}" for index in range(24)]
                + [f"pwm_{index}" for index in range(8)]
                + [f"monitor_{index}" for index in range(16)]
            ),
        )
        self.writer.writeheader()

    def write(self, sequence: int, sample: PilInput, result: PilOutput, elapsed_s: float) -> None:
        """Append one bridge transaction and flush it for crash-safe evidence."""
        row: dict[str, int | float] = {
            "sequence": sequence,
            "simulation_time_s": sample.simulation_time_s,
            "target_roundtrip_ms": elapsed_s * 1000.0,
            "encoder_digital_input": sample.digital[self.config.encoder_udp_index],
            "digital_output": result.digital,
        }
        row.update({f"adc_{index}": value for index, value in enumerate(sample.adc)})
        row.update({f"pwm_{index}": value for index, value in enumerate(result.pwm)})
        row.update({f"monitor_{index}": value for index, value in enumerate(result.monitor)})
        self.writer.writerow(row)
        self.stream.flush()


class PilBridge:
    """Blocking Simulink UDP to synchronous Data Link PIL bridge."""

    def __init__(
        self,
        api: PilApi,
        *,
        trace: PilTraceWriter | None = None,
        stop_event: threading.Event | None = None,
    ) -> None:
        self.api = api
        self.config = api.config
        self.trace = trace
        self.stop_event = stop_event or threading.Event()

    def run(self, *, maximum_steps: int | None = None, framed_udp: bool = False) -> int:
        """Serve until stopped, timed out, or the optional step limit is met."""
        completed = 0
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as channel:
            channel.bind(("0.0.0.0", self.config.bridge_listen_port))
            channel.settimeout(min(0.25, self.config.matlab_timeout_s))
            destination = (self.config.udp_host, self.config.matlab_listen_port)
            self.api.synchronize_masks()
            if framed_udp:
                return self._run_framed_udp(channel, destination, maximum_steps)
            initial_output = bytes(MATLAB_OUTPUT_STRUCT.size)
            channel.sendto(initial_output, destination)
            last_input = time.monotonic()
            while not self.stop_event.is_set():
                if maximum_steps is not None and completed >= maximum_steps:
                    break
                try:
                    payload, _source = channel.recvfrom(MATLAB_INPUT_STRUCT.size + 1)
                except ConnectionResetError:
                    channel.sendto(initial_output, destination)
                    continue
                except socket.timeout:
                    if completed == 0:
                        channel.sendto(initial_output, destination)
                    if completed > 0 and time.monotonic() - last_input >= self.config.matlab_timeout_s:
                        break
                    continue
                last_input = time.monotonic()
                completed += 1
                sample, result, elapsed = self._step(payload)
                channel.sendto(result.to_matlab_datagram(), destination)
                if self.trace is not None:
                    self.trace.write(completed, sample, result, elapsed)
        return completed

    def _step(self, payload: bytes) -> tuple[PilInput, PilOutput, float]:
        sample = PilInput.from_matlab_datagram(payload)
        started = time.perf_counter()
        result = self.api.step(sample)
        return sample, result, time.perf_counter() - started

    def _run_framed_udp(
        self, channel: socket.socket, destination: tuple[str, int], maximum_steps: int | None
    ) -> int:
        """Serve the session header used by the current GMP SIL MEX."""
        peer = (socket.gethostbyname(destination[0]), destination[1])
        completed = 0
        last_sequence = None
        deadline = None
        max_datagram = SIL_HEADER.size + MATLAB_INPUT_STRUCT.size
        while not self.stop_event.is_set() and (maximum_steps is None or completed < maximum_steps):
            if deadline is not None and time.monotonic() >= deadline:
                break
            try:
                datagram, source = channel.recvfrom(max_datagram + 1)
            except socket.timeout:
                continue
            if source != peer:
                continue
            if not SIL_HEADER.size <= len(datagram) <= max_datagram:
                raise GmpDlProtocolError("Invalid GMP SIL datagram length.")
            magic, version, kind, flags, sequence, size = SIL_HEADER.unpack_from(datagram)
            payload = datagram[SIL_HEADER.size:]
            if (magic, version, flags, size) != (SIL_MAGIC, SIL_VERSION, 0, len(payload)):
                raise GmpDlProtocolError("Invalid GMP SIL frame header.")

            if last_sequence is None:
                if kind != SIL_HELLO or size != SIL_SESSION.size:
                    raise GmpDlProtocolError("Expected a GMP SIL session hello.")
                abi = SIL_SESSION.unpack(payload)[:4]
                if abi != (SIL_VERSION, 0, MATLAB_INPUT_STRUCT.size, MATLAB_OUTPUT_STRUCT.size):
                    raise GmpDlProtocolError("GMP SIL session does not match the PIL packed ABI.")
                response_kind, response = SIL_HELLO_RESPONSE, payload
                last_sequence = sequence
            elif kind == SIL_SIMULATION_STATE:
                if size != SIL_STATE.size:
                    raise GmpDlProtocolError("Invalid GMP SIL simulation state length.")
                state_version, state, _, _ = SIL_STATE.unpack(payload)
                if state_version != SIL_VERSION or state not in range(1, 6):
                    raise GmpDlProtocolError("Invalid GMP SIL simulation state.")
                if state >= 3:
                    break
                continue  # UDP simulation state is a one-way notification.
            elif kind == SIL_DATA_REQUEST and sequence > last_sequence:
                completed += 1
                last_sequence = sequence
                sample, result, elapsed = self._step(payload)
                response_kind, response = SIL_DATA_RESPONSE, result.to_matlab_datagram()
            else:
                raise GmpDlProtocolError("Unexpected or repeated GMP SIL request.")

            header = SIL_HEADER.pack(SIL_MAGIC, SIL_VERSION, response_kind, 0, sequence, len(response))
            channel.sendto(header + response, peer)
            deadline = time.monotonic() + self.config.matlab_timeout_s
            if kind == SIL_DATA_REQUEST and self.trace is not None:
                self.trace.write(completed, sample, result, elapsed)
        return completed
