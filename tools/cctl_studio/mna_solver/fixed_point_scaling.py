"""Automatic per-unit scaling and coefficient quantization for MNA codegen."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


MATRIX_LAYOUT = (
    ("StateMatrix", "normal_A"),
    ("InputMatrix", "normal_B"),
    ("StateVector", "normal_bias"),
    ("StateMatrix", "short_A"),
    ("InputMatrix", "short_B"),
    ("StateVector", "short_bias"),
    ("SignalMatrix", "C"),
    ("SignalInputMatrix", "D"),
    ("SignalVector", "output_bias"),
)


@dataclass(frozen=True)
class FixedPointScaling:
    fractional_bits: int
    requested_fractional_bits: int
    coefficient_fractional_bits: Mapping[str, int]
    state_scales: np.ndarray
    input_scales: np.ndarray
    signal_scale: float
    horizon_steps: int
    maximum_scaled_coefficient: float
    maximum_quantization_error: float

    @property
    def raw_scale(self) -> int:
        return 1 << self.fractional_bits

    def quantize(
        self,
        values: np.ndarray | Sequence[float] | float,
        matrix_type: str | None = None,
    ) -> np.ndarray:
        array = np.asarray(values, dtype=float)
        fractional_bits = self.coefficient_fractional_bits.get(
            matrix_type,
            self.fractional_bits,
        )
        raw = np.rint(array * (1 << fractional_bits))
        if not np.all(np.isfinite(raw)) or np.any(raw < -(2**31)) or np.any(raw > 2**31 - 1):
            raise ValueError("fixed-point coefficient exceeds signed 32-bit storage")
        return raw.astype(np.int64)


def _next_power_of_two(value: float) -> float:
    if not math.isfinite(value) or value <= 0.0:
        return 1.0
    return math.ldexp(1.0, math.ceil(math.log2(value)))


def _matrix(plan, state: tuple[int, ...], position: int) -> np.ndarray:
    type_name = MATRIX_LAYOUT[position][0]
    return np.asarray(plan.pools[type_name][state[position]], dtype=float)


def _transform_matrix(
    type_name: str,
    matrix: np.ndarray,
    state_scales: np.ndarray,
    input_scales: np.ndarray,
    signal_scale: float,
) -> np.ndarray:
    value = np.asarray(matrix, dtype=float)
    if type_name == "StateMatrix":
        return value * state_scales[np.newaxis, :] / state_scales[:, np.newaxis]
    if type_name == "InputMatrix":
        return value * input_scales[np.newaxis, :] / state_scales[:, np.newaxis]
    if type_name == "StateVector":
        return value / state_scales
    if type_name == "SignalMatrix":
        return value * state_scales[np.newaxis, :] / signal_scale
    if type_name == "SignalInputMatrix":
        return value * input_scales[np.newaxis, :] / signal_scale
    if type_name == "SignalVector":
        return value / signal_scale
    raise ValueError(f"unsupported fixed-point matrix type {type_name!r}")


def transformed_pools(plan, scaling: FixedPointScaling) -> dict[str, list[np.ndarray]]:
    return {
        type_name: [
            _transform_matrix(
                type_name,
                matrix,
                scaling.state_scales,
                scaling.input_scales,
                scaling.signal_scale,
            )
            for matrix in matrices
        ]
        for type_name, matrices in plan.pools.items()
    }


def automatic_per_unit_scaling(
    document: Mapping,
    plan,
    requested_fractional_bits: int = 24,
    horizon_steps: int = 256,
    headroom: float = 2.0,
    input_full_scales: Mapping[str, float] | None = None,
    signal_full_scale: float | None = None,
) -> FixedPointScaling:
    """Estimate power-of-two bases from inputs and representative trajectories.

    The deterministic estimator samples fixed-topology transients and finite
    equilibria for both step profiles, then exercises transitions through all
    calculation states. It is an engineering range estimate, not a formal
    overflow proof. Explicit input full scales should therefore be supplied
    for production operating envelopes; netlist defaults are only the fallback.
    """
    if not 8 <= requested_fractional_bits <= 30:
        raise ValueError("fixed-point fractional bits must be in the range 8..30")
    if horizon_steps <= 0:
        raise ValueError("fixed-point scaling horizon must be positive")
    if not math.isfinite(headroom) or headroom < 1.0:
        raise ValueError("fixed-point scaling headroom must be at least one")
    if signal_full_scale is not None and (
        not math.isfinite(signal_full_scale) or signal_full_scale <= 0.0
    ):
        raise ValueError("fixed-point signal full scale must be finite and positive")

    analog_ports = [
        port for port in document["ports"]["inputs"] if port["data_type"] == "double"
    ]
    overrides = {str(name).upper(): float(value) for name, value in (input_full_scales or {}).items()}
    known_inputs = {str(port["name"]).upper() for port in analog_ports}
    unknown = set(overrides) - known_inputs
    if unknown:
        raise ValueError(
            "unknown fixed-point input full-scale names: " + ", ".join(sorted(unknown))
        )
    if any(not math.isfinite(value) or value <= 0.0 for value in overrides.values()):
        raise ValueError("fixed-point input full scales must be finite and positive")
    input_scales = np.asarray(
        [
            _next_power_of_two(
                overrides.get(
                    str(port["name"]).upper(),
                    max(abs(float(port.get("default", 0.0))) * headroom, 1.0),
                )
            )
            for port in analog_ports
        ],
        dtype=float,
    )
    input_reference = max(
        float(np.max(input_scales)) if input_scales.size else 0.0,
        1.0,
    )
    state_count = len(document["state"]["names"])
    bound = np.zeros(state_count, dtype=float)
    signal_bound = 0.0
    states = plan.calculation_states
    sample_count = min(len(states), 64)
    sample_indices = np.linspace(0, len(states) - 1, sample_count, dtype=int)
    sampled_states = [states[int(index)] for index in sample_indices]
    local_steps = max(8, min(64, horizon_steps // 4))

    def observe(
        value: np.ndarray,
        state: tuple[int, ...],
        input_value: np.ndarray,
        include_signal: bool = True,
    ) -> None:
        nonlocal bound, signal_bound
        if not np.all(np.isfinite(value)) or np.any(np.abs(value) > 1.0e30):
            raise ValueError(
                "automatic per-unit state trajectory diverged; provide bounded operating ranges"
            )
        bound = np.maximum(bound, np.abs(value))
        if not include_signal:
            return
        signals = (
            _matrix(plan, state, 6) @ value
            + _matrix(plan, state, 7) @ input_value
            + _matrix(plan, state, 8)
        )
        if not np.all(np.isfinite(signals)):
            raise ValueError("automatic per-unit signal trajectory is non-finite")
        if signals.size:
            signal_bound = max(signal_bound, float(np.max(np.abs(signals))))

    # Exercise representative fixed topologies with both input polarities.
    for state in sampled_states:
        for offset in (0, 3):
            state_matrix = _matrix(plan, state, offset)
            input_matrix = _matrix(plan, state, offset + 1)
            bias = _matrix(plan, state, offset + 2)
            for polarity in (1.0, -1.0):
                value = np.zeros(state_count, dtype=float)
                input_value = polarity * input_scales
                forcing = input_matrix @ input_value + bias
                for _ in range(local_steps):
                    value = state_matrix @ value + forcing
                    observe(value, state, input_value)
            # Include finite constant-topology equilibria where they exist.
            try:
                lhs = np.eye(state_count) - state_matrix
                if np.linalg.cond(lhs) < 1.0e10:
                    equilibrium_limit = np.maximum(
                        bound * 8.0,
                        input_reference * 8.0,
                    )
                    for input_value in (input_scales, -input_scales):
                        equilibrium = np.linalg.solve(
                            lhs,
                            input_matrix @ input_value + bias,
                        )
                        if np.all(np.abs(equilibrium) <= equilibrium_limit):
                            observe(equilibrium, state, input_value)
            except np.linalg.LinAlgError:
                pass

    # Exercise transitions across all calculation states; this catches state
    # coordinate ratios that a constant-topology trajectory does not visit.
    value = np.zeros(state_count, dtype=float)
    transition_limit = np.maximum(
        bound * 8.0,
        input_reference * 8.0,
    )
    for index in range(horizon_steps):
        state = states[index % len(states)]
        offset = 0 if index % 5 else 3
        input_value = input_scales if index % 2 else -input_scales
        value = (
            _matrix(plan, state, offset) @ value
            + _matrix(plan, state, offset + 1)
            @ input_value
            + _matrix(plan, state, offset + 2)
        )
        # Keep only transitions close to the fixed-topology operating envelope.
        # The artificial radix-order sweep can otherwise concatenate mutually
        # incompatible modes into a state that no switching command can reach.
        if np.all(np.abs(value) <= transition_limit):
            observe(value, state, input_value, include_signal=False)
        else:
            value.fill(0.0)

    state_scales = np.asarray(
        [_next_power_of_two(max(value * headroom, 1.0e-18)) for value in bound],
        dtype=float,
    )
    signal_scale = _next_power_of_two(
        signal_full_scale
        if signal_full_scale is not None
        else max(signal_bound * headroom, 1.0e-18)
    )

    provisional = FixedPointScaling(
        requested_fractional_bits,
        requested_fractional_bits,
        {},
        state_scales,
        input_scales,
        signal_scale,
        horizon_steps,
        0.0,
        0.0,
    )
    pools = transformed_pools(plan, provisional)
    maximum_by_type = {
        type_name: max(
            (float(np.max(np.abs(matrix))) for matrix in values if matrix.size),
            default=0.0,
        )
        for type_name, values in pools.items()
    }
    maximum = max(
        (float(np.max(np.abs(matrix))) for values in pools.values() for matrix in values if matrix.size),
        default=0.0,
    )
    vector_maximum = max(
        maximum_by_type.get("StateVector", 0.0),
        maximum_by_type.get("SignalVector", 0.0),
    )
    integer_bits = max(0, math.ceil(math.log2(max(vector_maximum, 1.0))))
    fractional_bits = min(requested_fractional_bits, 30 - integer_bits)
    if fractional_bits < 8:
        raise ValueError(
            f"scaled affine-bias magnitude {vector_maximum:.6g} leaves fewer than 8 fractional bits"
        )
    coefficient_fractional_bits: dict[str, int] = {}
    for type_name in (
        "StateMatrix",
        "InputMatrix",
        "SignalMatrix",
        "SignalInputMatrix",
    ):
        value = maximum_by_type.get(type_name, 0.0)
        coefficient_integer_bits = max(0, math.ceil(math.log2(max(value, 1.0))))
        bits = min(requested_fractional_bits, 30 - coefficient_integer_bits)
        if bits < 1:
            raise ValueError(
                f"scaled {type_name} coefficient magnitude {value:.6g} exceeds signed 32-bit storage"
            )
        coefficient_fractional_bits[type_name] = bits

    quantization_bits = {
        **coefficient_fractional_bits,
        "StateVector": fractional_bits,
        "SignalVector": fractional_bits,
    }
    maximum_error = max(
        (
            float(
                np.max(
                    np.abs(
                        np.rint(matrix * (1 << quantization_bits[type_name]))
                        / (1 << quantization_bits[type_name])
                        - matrix
                    )
                )
            )
            for type_name, values in pools.items()
            for matrix in values
            if matrix.size
        ),
        default=0.0,
    )
    return FixedPointScaling(
        fractional_bits,
        requested_fractional_bits,
        coefficient_fractional_bits,
        state_scales,
        input_scales,
        signal_scale,
        horizon_steps,
        maximum,
        maximum_error,
    )
