"""Fixed-point MNA effect tests; the _fp suffix is the public test contract."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


SOLVER_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOLVER_DIR))

import circuit_data as data  # noqa: E402
import cpp_codegen as codegen  # noqa: E402
import fixed_point_scaling as fp  # noqa: E402
import mna_solver as mna  # noqa: E402
import switched_solver as switched  # noqa: E402


BUCK_NETLIST = SOLVER_DIR / "tb" / "buck" / "buck.CIR"
INT32_MIN = -(2**31)
INT32_MAX = 2**31 - 1


def _saturate32(value: int) -> int:
    return min(max(value, INT32_MIN), INT32_MAX)


def _quantize(value: np.ndarray, fractional_bits: int) -> np.ndarray:
    raw = np.rint(np.asarray(value, dtype=float) * (1 << fractional_bits))
    return np.clip(raw, INT32_MIN, INT32_MAX).astype(np.int64)


def _rounded_shift(value: int, fractional_bits: int) -> int:
    half = 1 << (fractional_bits - 1)
    return (value + half) >> fractional_bits if value >= 0 else -((-value + half) >> fractional_bits)


def _mixed_dot(
    coefficients: np.ndarray,
    values: np.ndarray,
    coefficient_bits: int,
) -> int:
    accumulator = sum(
        int(coefficient) * int(value)
        for coefficient, value in zip(coefficients, values)
    )
    return _saturate32(_rounded_shift(accumulator, coefficient_bits))


class FixedPointEffectFpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        circuit = mna.parse_netlist(BUCK_NETLIST)
        cls.document = data.build_circuit_data(
            switched.build_piecewise_model(circuit),
            source_path=BUCK_NETLIST,
            normal_step_s=100e-9,
            short_step_s=1e-9,
        )
        cls.plan = codegen.build_matrix_dedup_plan(cls.document)
        cls.scaling = fp.automatic_per_unit_scaling(
            cls.document,
            cls.plan,
            requested_fractional_bits=24,
            horizon_steps=128,
            input_full_scales={"VS1": 8.0},
            signal_full_scale=16.0,
        )
        cls.pools = fp.transformed_pools(cls.plan, cls.scaling)

    def test_generated_module_uses_fp_suffix_and_mixed_q_arithmetic(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "buck.json"
            data.write_circuit_data(data_path, self.document)
            files = codegen.generate_cpp_project(
                data_path,
                Path(directory) / "generated",
                class_name="BuckCircuitFp",
                backend="fixed_point",
                fixed_point_input_ranges={"VS1": 8.0},
                fixed_point_signal_range=16.0,
                output_stem="buckcircuit_fp",
            )
            header = files["header"].read_text(encoding="utf-8")

        self.assertEqual(files["header"].name, "buckcircuit_fp.hpp")
        self.assertIn("class BuckCircuitFp", header)
        self.assertIn("cctl::mixed_affine_transform", header)
        self.assertIn("cctl::mixed_dot", header)
        self.assertIn("using SignalCoefficient = cctl::fixed_point32<", header)

    def test_quantized_affine_maps_track_float_reference(self) -> None:
        rng = np.random.default_rng(0xC071)
        value_bits = self.scaling.fractional_bits
        coefficient_bits = self.scaling.coefficient_fractional_bits
        maximum_state_error = 0.0
        maximum_signal_error = 0.0

        for state in self.plan.calculation_states:
            state_matrix = self.pools["StateMatrix"][state[0]]
            input_matrix = self.pools["InputMatrix"][state[1]]
            state_bias = self.pools["StateVector"][state[2]]
            signal_matrix = self.pools["SignalMatrix"][state[6]]
            signal_input_matrix = self.pools["SignalInputMatrix"][state[7]]
            signal_bias = self.pools["SignalVector"][state[8]]

            raw_state_matrix = _quantize(
                state_matrix, coefficient_bits["StateMatrix"]
            )
            raw_input_matrix = _quantize(
                input_matrix, coefficient_bits["InputMatrix"]
            )
            raw_state_bias = _quantize(state_bias, value_bits)
            raw_signal_matrix = _quantize(
                signal_matrix, coefficient_bits["SignalMatrix"]
            )
            raw_signal_input = _quantize(
                signal_input_matrix,
                coefficient_bits["SignalInputMatrix"],
            )
            raw_signal_bias = _quantize(signal_bias, value_bits)

            for _ in range(32):
                state_value = rng.uniform(-0.25, 0.25, state_matrix.shape[1])
                input_value = rng.uniform(-0.75, 0.75, input_matrix.shape[1])
                raw_state = _quantize(state_value, value_bits)
                raw_input = _quantize(input_value, value_bits)

                reference_state = (
                    state_matrix @ state_value
                    + input_matrix @ input_value
                    + state_bias
                )
                fixed_state_raw = np.asarray(
                    [
                        _saturate32(
                            _mixed_dot(
                                raw_state_matrix[row],
                                raw_state,
                                coefficient_bits["StateMatrix"],
                            )
                            + _mixed_dot(
                                raw_input_matrix[row],
                                raw_input,
                                coefficient_bits["InputMatrix"],
                            )
                            + int(raw_state_bias[row])
                        )
                        for row in range(state_matrix.shape[0])
                    ],
                    dtype=np.int64,
                )
                fixed_state = fixed_state_raw / float(1 << value_bits)
                maximum_state_error = max(
                    maximum_state_error,
                    float(
                        np.max(
                            np.abs(fixed_state - reference_state)
                            * self.scaling.state_scales
                        )
                    ),
                )

                reference_signal = (
                    signal_matrix @ state_value
                    + signal_input_matrix @ input_value
                    + signal_bias
                )
                fixed_signal_raw = np.asarray(
                    [
                        _saturate32(
                            _mixed_dot(
                                raw_signal_matrix[row],
                                raw_state,
                                coefficient_bits["SignalMatrix"],
                            )
                            + _mixed_dot(
                                raw_signal_input[row],
                                raw_input,
                                coefficient_bits["SignalInputMatrix"],
                            )
                            + int(raw_signal_bias[row])
                        )
                        for row in range(signal_matrix.shape[0])
                    ],
                    dtype=np.int64,
                )
                fixed_signal = fixed_signal_raw / float(1 << value_bits)
                maximum_signal_error = max(
                    maximum_signal_error,
                    float(
                        np.max(np.abs(fixed_signal - reference_signal))
                        * self.scaling.signal_scale
                    ),
                )

        self.assertLess(maximum_state_error, 2.0e-4)
        self.assertLess(maximum_signal_error, 2.0e-4)


if __name__ == "__main__":
    unittest.main()
