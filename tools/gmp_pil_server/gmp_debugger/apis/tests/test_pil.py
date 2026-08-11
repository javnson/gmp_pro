"""Unit tests for the headless GMP PIL API."""

from __future__ import annotations

import io
import json
import struct
import tempfile
import unittest
from pathlib import Path

from apis.pil import (
    MATLAB_INPUT_STRUCT,
    PilApi,
    PilConfiguration,
    PilInput,
    PilOutput,
    PilTraceWriter,
)


def _requirement(macro: str, value: int | str) -> dict[str, object]:
    kind = "string" if isinstance(value, str) else "number"
    return {"macro": macro, "binding": {kind: str(value)}}


class FakeTransport:
    """Capture PIL requests and return deterministic responses."""

    def __init__(self, config: PilConfiguration) -> None:
        self.config = config
        self.requests: list[tuple[int, bytes]] = []

    def transact(self, command: int, payload: bytes = b"") -> bytes:
        self.requests.append((command, payload))
        if command == self.config.base_command + 1:
            return struct.pack("<II", self.config.tx_mask, self.config.rx_mask)
        return struct.pack("<I3H6f", 1, 100, 200, 300, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


class PilApiTests(unittest.TestCase):
    """Verify SDPE loading and the standard PIL wire layouts."""

    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        target_values = {
            "GMP_DL_UART_BAUDRATE": 256000,
            "GMP_PIL_DL_BASE_COMMAND": 16,
            "GMP_PIL_UDP_HOST": "127.0.0.1",
            "GMP_PIL_BRIDGE_UDP_LISTEN_PORT": 12501,
            "GMP_PIL_MATLAB_UDP_LISTEN_PORT": 12500,
            "GMP_PIL_MATLAB_COMMAND_TX_PORT": 12502,
            "GMP_PIL_MATLAB_COMMAND_RX_PORT": 12503,
            "GMP_PIL_RX_MASK": 0x7F,
            "GMP_PIL_TX_MASK": 0x003F0007,
            "GMP_PIL_MCU_TIMEOUT_MS": 200,
            "GMP_PIL_MATLAB_TIMEOUT_MS": 5000,
            "GMP_PIL_UDP_ENCODER_INDEX": 0,
            "GMP_PIL_RX_ADC_UDC_INDEX": 0,
            "GMP_PIL_RX_ADC_UU_INDEX": 1,
            "GMP_PIL_RX_ADC_UV_INDEX": 2,
            "GMP_PIL_RX_ADC_UW_INDEX": 3,
            "GMP_PIL_RX_ADC_IU_INDEX": 4,
            "GMP_PIL_RX_ADC_IV_INDEX": 5,
            "GMP_PIL_RX_ADC_IW_INDEX": 6,
            "GMP_PIL_TX_PWM_U_INDEX": 0,
            "GMP_PIL_TX_PWM_V_INDEX": 1,
            "GMP_PIL_TX_PWM_W_INDEX": 2,
            "GMP_PIL_TX_MONITOR_IU_INDEX": 0,
            "GMP_PIL_TX_MONITOR_IV_INDEX": 1,
            "GMP_PIL_TX_MONITOR_ID_INDEX": 2,
            "GMP_PIL_TX_MONITOR_IQ_INDEX": 3,
            "GMP_PIL_TX_MONITOR_POSITION_INDEX": 4,
            "GMP_PIL_TX_MONITOR_SPEED_INDEX": 5,
        }
        target = {
            "feature_macros": [{"macro": "ENABLE_GMP_DL_PIL_SIM", "enabled": True}],
            "option_macros": [{"macro": "BUILD_LEVEL", "value": "(2)"}],
            "requirements": [_requirement(macro, value) for macro, value in target_values.items()],
        }
        common = {"requirements": [{"macro": "CONTROLLER_FREQUENCY", "binding": {"float": "20e3"}}]}
        self.target_path = root / "target.json"
        self.common_path = root / "common.json"
        self.target_path.write_text(json.dumps(target), encoding="utf-8")
        self.common_path.write_text(json.dumps(common), encoding="utf-8")
        self.config = PilConfiguration.from_sdpe(self.target_path, self.common_path)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_sdpe_configuration_resolves_sample_time(self) -> None:
        self.assertEqual(self.config.build_level, 2)
        self.assertEqual(self.config.serial_baudrate, 256000)
        self.assertAlmostEqual(self.config.sample_time_s, 50e-6)

    def test_input_pack_uses_masks_and_encoder_mapping(self) -> None:
        datagram = MATLAB_INPUT_STRUCT.pack(
            0.001,
            *range(24),
            *([0.0] * 16),
            1234,
            *([0] * 7),
        )
        sample = PilInput.from_matlab_datagram(datagram)
        payload = sample.to_datalink_payload(self.config)
        self.assertEqual(len(payload), 22)
        self.assertEqual(struct.unpack_from("<II", payload), (20, 1234))
        self.assertEqual(struct.unpack_from("<7H", payload, 8), tuple(range(7)))

    def test_api_synchronizes_masks_and_steps(self) -> None:
        transport = FakeTransport(self.config)
        api = PilApi(transport, self.config)
        self.assertEqual(api.synchronize_masks(), (0x003F0007, 0x7F))
        sample = PilInput(0.0, tuple(range(24)), (0.0,) * 16, (0,) * 8)
        result = api.step(sample)
        self.assertEqual(result.pwm[:3], (100, 200, 300))
        self.assertEqual(result.monitor[:6], (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
        self.assertEqual(len(result.to_matlab_datagram()), 200)

    def test_output_decoder_rejects_truncated_payload(self) -> None:
        with self.assertRaisesRegex(Exception, "Expected 34 PIL output bytes"):
            PilOutput.from_datalink_payload(b"\x00" * 10, self.config.tx_mask)

    def test_trace_records_selected_encoder_input(self) -> None:
        stream = io.StringIO()
        trace = PilTraceWriter(stream, self.config)
        sample = PilInput(0.001, tuple(range(24)), (0.0,) * 16, (1234,) + (0,) * 7)
        result = PilOutput(1, (100,) * 8, (0,) * 8, (0.0,) * 16)
        trace.write(1, sample, result, 0.03)
        header, row = stream.getvalue().splitlines()
        self.assertIn("encoder_digital_input", header)
        self.assertEqual(row.split(",")[3], "1234")


if __name__ == "__main__":
    unittest.main()
