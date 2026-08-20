"""Generate a fixed-size Eigen C++ circuit class from circuit-data JSON."""

from __future__ import annotations

import argparse
import itertools
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping, Sequence

import numpy as np

from circuit_data import DEFAULT_MATRIX_TOLERANCE, load_circuit_data
from console_progress import TimedProgressBar


_MATRIX_SPECS = (
    ("normal_A", "StateMatrix", "normal", "A"),
    ("normal_B", "InputMatrix", "normal", "B"),
    ("normal_bias", "StateVector", "normal", "bias"),
    ("short_A", "StateMatrix", "short", "A"),
    ("short_B", "InputMatrix", "short", "B"),
    ("short_bias", "StateVector", "short", "bias"),
    ("C", "SignalMatrix", "normal", "C"),
    ("D", "SignalInputMatrix", "normal", "D"),
    ("output_bias", "SignalVector", "normal", "output_bias"),
)

_POOL_NAMES = {
    "StateMatrix": "state_matrices",
    "InputMatrix": "input_matrices",
    "StateVector": "state_vectors",
    "SignalMatrix": "signal_matrices",
    "SignalInputMatrix": "signal_input_matrices",
    "SignalVector": "signal_vectors",
}
_POOL_COUNT_NAMES = {
    type_name: function_name.replace("matrices", "matrix_count").replace("vectors", "vector_count")
    for type_name, function_name in _POOL_NAMES.items()
}


def _identifier(value: str, default: str = "Circuit") -> str:
    result = re.sub(r"[^A-Za-z0-9_]", "_", value).strip("_") or default
    if result[0].isdigit():
        result = "Circuit_" + result
    return result


def _default_class_name(data_path: str | Path, document: Mapping) -> str:
    source_file = document.get("circuit", {}).get("source", {}).get("file")
    stem = Path(source_file).stem if source_file else Path(data_path).stem
    words = re.findall(r"[A-Za-z0-9]+", stem)
    base = "".join(word[:1].upper() + word[1:].lower() for word in words) or "Circuit"
    return _identifier(base + "Circuit")


def _number(value: float) -> str:
    text = format(float(value), ".17g")
    return text if any(char in text for char in ".eE") else text + ".0"


def _cpp_string(value: str) -> str:
    return json.dumps(value, ensure_ascii=True)


def _matrix_expression(type_name: str, values: Sequence[Sequence[float]] | Sequence[float]) -> str:
    flattened: list[float] = []
    for item in values:
        if isinstance(item, (list, tuple)):
            flattened.extend(float(value) for value in item)
        else:
            flattened.append(float(item))
    assignments = ", ".join(_number(value) for value in flattened)
    return f"([] {{ {type_name} value; value << {assignments}; return value; }}())"


def _topology_matrices(topology: Mapping) -> tuple[np.ndarray, ...]:
    return tuple(
        np.asarray(topology["discrete"][profile][field], dtype=float)
        for _, _, profile, field in _MATRIX_SPECS
    )


def _equivalent(lhs: Sequence[np.ndarray], rhs: Sequence[np.ndarray], tolerance: float) -> bool:
    return all(
        left.shape == right.shape
        and bool(np.all(np.abs(left - right) <= tolerance))
        for left, right in zip(lhs, rhs)
    )


def _quick_equivalent(lhs: np.ndarray, rhs: np.ndarray, tolerance: float) -> bool:
    """Reject a candidate cheaply before doing a full elementwise comparison."""

    if lhs.shape != rhs.shape:
        return False
    left = lhs.ravel()
    right = rhs.ravel()
    if not left.size:
        return True
    positions = {0, left.size // 3, (2 * left.size) // 3, left.size - 1}
    return all(abs(float(left[index]) - float(right[index])) <= tolerance for index in positions)


def _exact_matrix_key(matrix: np.ndarray) -> tuple[tuple[int, ...], int]:
    """Return a compact process-local accelerator key; callers confirm equality."""

    return matrix.shape, hash(matrix.tobytes(order="C"))


def _invariant(matrix: np.ndarray) -> float:
    if matrix.ndim == 2 and matrix.shape[0] == matrix.shape[1] and matrix.shape[0] > 0:
        value = float(np.linalg.det(matrix))
    else:
        value = float(np.sum(matrix))
    return value if math.isfinite(value) else 0.0


def _bucket_width(matrices: Sequence[np.ndarray], tolerance: float) -> float:
    sample = matrices[0]
    if sample.ndim == 2 and sample.shape[0] == sample.shape[1] and sample.shape[0] > 0:
        order = sample.shape[0]
        maximum_norm = max(float(np.linalg.norm(matrix, ord=np.inf)) for matrix in matrices)
        try:
            width = tolerance * order * max(1.0, maximum_norm + order * tolerance) ** (order - 1)
        except OverflowError:
            width = float("inf")
    else:
        width = tolerance * max(int(sample.size), 1)
    if not math.isfinite(width):
        return float("inf")
    return max(width, np.finfo(float).eps)


def _bucket(value: float, width: float) -> int:
    return 0 if not math.isfinite(width) else math.floor(value / width)


def _neighbor_keys(key: tuple[int, ...]):
    return itertools.product(*(range(value - 1, value + 2) for value in key))


@dataclass
class MatrixDedupPlan:
    tolerance: float
    topology_to_calculation_state: list[int]
    calculation_states: list[tuple[int, ...]]
    pools: dict[str, list[np.ndarray]]
    logical_state_count: int
    unique_state_count: int
    coefficients_before: int
    coefficients_after: int
    pool_references: dict[str, int]

    @property
    def deduplicated_state_count(self) -> int:
        return self.logical_state_count - self.unique_state_count

    @property
    def shared_matrix_copy_count(self) -> int:
        return sum(self.pool_references.values()) - sum(len(pool) for pool in self.pools.values())


def build_matrix_dedup_plan(
    document: Mapping,
    tolerance: float = DEFAULT_MATRIX_TOLERANCE,
    progress: Callable[[int, int], None] | None = None,
) -> MatrixDedupPlan:
    """Deduplicate equivalent runtime states, then intern each fixed-size matrix."""

    if tolerance <= 0.0 or not math.isfinite(tolerance):
        raise ValueError("matrix tolerance must be a finite positive number")
    topology_matrices = [_topology_matrices(item) for item in document["topologies"]]
    logical_count = len(topology_matrices)
    if not topology_matrices:
        raise ValueError("circuit data contains no topology states")

    by_type: dict[str, list[np.ndarray]] = {}
    for matrices in topology_matrices:
        for (_, type_name, _, _), matrix in zip(_MATRIX_SPECS, matrices):
            by_type.setdefault(type_name, []).append(matrix)
    bucket_widths = {
        type_name: _bucket_width(matrices, tolerance)
        for type_name, matrices in by_type.items()
    }

    pools = {type_name: [] for type_name in _POOL_NAMES}
    pool_buckets: dict[str, dict[int, list[int]]] = {
        type_name: {} for type_name in _POOL_NAMES
    }
    pool_exact: dict[str, dict[tuple[tuple[int, ...], int], list[int]]] = {
        type_name: {} for type_name in _POOL_NAMES
    }

    def intern(type_name: str, matrix: np.ndarray) -> int:
        exact_key = _exact_matrix_key(matrix)
        for exact_index in pool_exact[type_name].get(exact_key, ()):
            if _equivalent((matrix,), (pools[type_name][exact_index],), tolerance):
                return exact_index
        key = _bucket(_invariant(matrix), bucket_widths[type_name])
        candidates: list[int] = []
        for neighbor in range(key - 1, key + 2):
            candidates.extend(pool_buckets[type_name].get(neighbor, ()))
        for index in candidates:
            representative = pools[type_name][index]
            if _quick_equivalent(matrix, representative, tolerance) and _equivalent(
                (matrix,), (representative,), tolerance
            ):
                pool_exact[type_name].setdefault(exact_key, []).append(index)
                return index
        index = len(pools[type_name])
        pools[type_name].append(matrix)
        pool_buckets[type_name].setdefault(key, []).append(index)
        pool_exact[type_name].setdefault(exact_key, []).append(index)
        return index

    state_width = bucket_widths["StateMatrix"]
    state_buckets: dict[tuple[int, int], list[int]] = {}
    unique_matrices: list[tuple[np.ndarray, ...]] = []
    calculation_states: list[tuple[int, ...]] = []
    topology_to_state: list[int] = []
    state_exact: dict[tuple[tuple[tuple[int, ...], int], ...], list[int]] = {}
    for logical_index, matrices in enumerate(topology_matrices, start=1):
        exact_state_key = tuple(_exact_matrix_key(matrix) for matrix in matrices)
        exact_state_index = next(
            (
                index
                for index in state_exact.get(exact_state_key, ())
                if _equivalent(matrices, unique_matrices[index], tolerance)
            ),
            None,
        )
        state_key = (
            _bucket(_invariant(matrices[0]), state_width),
            _bucket(_invariant(matrices[3]), state_width),
        )
        candidates: list[int] = []
        for neighbor in _neighbor_keys(state_key):
            candidates.extend(state_buckets.get(tuple(neighbor), ()))
        calculation_index = exact_state_index
        if calculation_index is None:
            calculation_index = next(
                (
                    index
                    for index in candidates
                    if all(
                        _quick_equivalent(matrix, representative, tolerance)
                        for matrix, representative in zip(matrices, unique_matrices[index])
                    )
                    and _equivalent(matrices, unique_matrices[index], tolerance)
                ),
                None,
            )
        if calculation_index is None:
            calculation_index = len(unique_matrices)
            unique_matrices.append(matrices)
            state_buckets.setdefault(state_key, []).append(calculation_index)
            calculation_states.append(
                tuple(
                    intern(type_name, matrix)
                    for (_, type_name, _, _), matrix in zip(_MATRIX_SPECS, matrices)
                )
            )
        state_exact.setdefault(exact_state_key, []).append(calculation_index)
        topology_to_state.append(calculation_index)
        if progress is not None:
            progress(logical_index, logical_count)

    coefficients_before = sum(matrix.size for matrices in topology_matrices for matrix in matrices)
    coefficients_after = sum(matrix.size for matrices in pools.values() for matrix in matrices)
    pool_references = {
        type_name: sum(spec_type == type_name for _, spec_type, _, _ in _MATRIX_SPECS)
        * len(calculation_states)
        for type_name in pools
    }
    return MatrixDedupPlan(
        tolerance,
        topology_to_state,
        calculation_states,
        pools,
        logical_count,
        len(calculation_states),
        coefficients_before,
        coefficients_after,
        pool_references,
    )


def render_header(
    document: Mapping,
    class_name: str,
    matrix_tolerance: float | None = None,
    progress: Callable[[int, int], None] | None = None,
    plan: MatrixDedupPlan | None = None,
) -> str:
    selected_tolerance = (
        float(document.get("solver", {}).get("matrix_tolerance", DEFAULT_MATRIX_TOLERANCE))
        if matrix_tolerance is None
        else float(matrix_tolerance)
    )
    storage = plan or build_matrix_dedup_plan(document, selected_tolerance, progress)
    state_count = len(document["state"]["names"])
    signal_count = len(document["signals"]["names"])
    pwm_ports = [port for port in document["ports"]["inputs"] if port["data_type"] == "uint32_t"]
    input_ports = [port for port in document["ports"]["inputs"] if port["data_type"] == "double"]
    outputs = document["ports"]["outputs"]
    input_count = len(input_ports)
    topology_count = storage.logical_state_count
    switching = document["switching"]
    pwm_fields = [_identifier(port["name"], "PWM") for port in pwm_ports]
    input_fields = [_identifier(port["name"], "input") for port in input_ports]
    output_fields = [_identifier(port["field"], "output") for port in outputs]
    pool_count_constants = "\n".join(
        f"    static constexpr std::size_t {_POOL_COUNT_NAMES[type_name]} = {len(storage.pools[type_name])};"
        for type_name in _POOL_NAMES
    )
    pool_functions = []
    for type_name, function_name in _POOL_NAMES.items():
        values = ",\n            ".join(
            _matrix_expression(type_name, matrix.tolist())
            for matrix in storage.pools[type_name]
        )
        count_name = _POOL_COUNT_NAMES[type_name]
        pool_functions.append(
            f'''    static const std::array<{type_name}, {count_name}>& {function_name}() {{
        static const std::array<{type_name}, {count_name}> pool{{{{
            {values}
        }}}};
        return pool;
    }}'''
        )
    pool_function_text = "\n\n".join(pool_functions)
    calculation_state_fields = "\n".join(
        f"        std::size_t {field};" for field, _, _, _ in _MATRIX_SPECS
    )
    calculation_state_values = ",\n            ".join(
        "CalculationState{" + ", ".join(f"{index}U" for index in indices) + "}"
        for indices in storage.calculation_states
    )
    topology_mapping_values = ", ".join(
        f"{index}U" for index in storage.topology_to_calculation_state
    )
    input_vector = ", ".join(f"inputs.{name}" for name in input_fields)
    function_parameters = ", ".join(
        [*(f"std::uint32_t {name}" for name in pwm_fields), *(f"double {name}" for name in input_fields)]
    )
    function_arguments = ", ".join([*pwm_fields, *input_fields])
    output_updates = "\n".join(
        f"        output.{field} = signals_({port['signal_index']});"
        for field, port in zip(output_fields, outputs)
    )
    output_members = "\n".join(f"        double {field}{{0.0}};" for field in output_fields)
    output_lookup = "\n".join(
        f"            if (name == {_cpp_string(port['name'])} || name == {_cpp_string(field)}) return {field};"
        for field, port in zip(output_fields, outputs)
    )
    input_members = "\n".join(
        f"        double {field}{{{_number(port['default'])}}};"
        for field, port in zip(input_fields, input_ports)
    )
    pwm_members = "\n".join(f"        std::uint32_t {field}{{0U}};" for field in pwm_fields)
    if switching.get("kind") == "multi_diode_switch":
        pwm_field_by_name = {
            str(port["name"]).upper(): field for port, field in zip(pwm_ports, pwm_fields)
        }
        selection_lines: list[str] = []
        for index, diode in enumerate(switching["diodes"]):
            selection_lines.append(
                f'''        const double diode_voltage_{index} = signals_({diode['anode_signal_index']}) - signals_({diode['cathode_signal_index']});
        constexpr double diode_threshold_{index} = {_number(diode['forward_threshold_V'])};
        diode_on_[{index}] = diode_voltage_{index} >= diode_threshold_{index} + (diode_on_[{index}] ? -hysteresis : hysteresis);
        topology_index = topology_index * 2U + (diode_on_[{index}] ? 1U : 0U);'''
            )
        for switch in switching["switches"]:
            field = pwm_field_by_name[str(switch["command_port"]).upper()]
            selection_lines.append(
                f"        topology_index = topology_index * 2U + (inputs.{field} != 0U ? 1U : 0U);"
            )
        reset_switch_state = "        diode_on_.fill(false);"
        selection_body = "\n".join(
            [
                f"        constexpr double hysteresis = {_number(switching['voltage_hysteresis_V'])};",
                "        std::size_t topology_index = 0U;",
                *selection_lines,
                "        return topology_index;",
            ]
        )
        switch_state_members = (
            f"    std::array<bool, {len(switching['diodes'])}> diode_on_{{}};"
        )
    elif switching.get("kind") in {"multi_mosfet", "multi_mosfet_diode"}:
        switch_lines: list[str] = []
        pwm_field_by_name = {
            str(port["name"]).upper(): field for port, field in zip(pwm_ports, pwm_fields)
        }
        for index, switch in enumerate(switching["switches"]):
            field = pwm_field_by_name[str(switch["pwm_port"]).upper()]
            switch_lines.append(
                f'''        topology_index *= 3U;
        if (inputs.{field} != 0U) {{
            body_on_[{index}] = false;
            topology_index += 1U;
        }} else {{
            const double reverse_voltage_{index} = signals_({switch['source_signal_index']}) - signals_({switch['drain_signal_index']});
            constexpr double body_threshold_{index} = {_number(switch['body_forward_threshold_V'])};
            body_on_[{index}] = reverse_voltage_{index} >= body_threshold_{index} + (body_on_[{index}] ? -hysteresis : hysteresis);
            topology_index += body_on_[{index}] ? 2U : 0U;
        }}'''
            )
        mixed_mosfet_diode = switching.get("kind") == "multi_mosfet_diode"
        if mixed_mosfet_diode:
            for index, diode in enumerate(switching["diodes"]):
                switch_lines.append(
                    f'''        const double diode_voltage_{index} = signals_({diode['anode_signal_index']}) - signals_({diode['cathode_signal_index']});
        constexpr double diode_threshold_{index} = {_number(diode['forward_threshold_V'])};
        diode_on_[{index}] = diode_voltage_{index} >= diode_threshold_{index} + (diode_on_[{index}] ? -hysteresis : hysteresis);
        topology_index = topology_index * 2U + (diode_on_[{index}] ? 1U : 0U);'''
                )
        reset_switch_state = "        body_on_.fill(false);"
        if mixed_mosfet_diode:
            reset_switch_state += "\n        diode_on_.fill(false);"
        selection_body = "\n".join(
            [
                f"        constexpr double hysteresis = {_number(switching['voltage_hysteresis_V'])};",
                "        std::size_t topology_index = 0U;",
                *switch_lines,
                "        return topology_index;",
            ]
        )
        switch_state_members = f"    std::array<bool, {len(switching['switches'])}> body_on_{{}};"
        if mixed_mosfet_diode:
            switch_state_members += (
                f"\n    std::array<bool, {len(switching['diodes'])}> diode_on_{{}};"
            )
    else:
        terminal = document["signals"]["switch_terminal_indices"]
        pwm_field = pwm_fields[0]
        reset_switch_state = "        diode_on_ = false;\n        body_on_ = false;"
        selection_body = f'''        const double diode_voltage = signals_({terminal['diode_anode']}) - signals_({terminal['diode_cathode']});
        const double reverse_mosfet_voltage = signals_({terminal['mosfet_source']}) - signals_({terminal['mosfet_drain']});
        constexpr double hysteresis = {_number(switching['voltage_hysteresis_V'])};
        constexpr double diode_threshold = {_number(switching['diode_forward_threshold_V'])};
        constexpr double body_threshold = {_number(switching['body_forward_threshold_V'])};
        diode_on_ = diode_voltage >= diode_threshold + (diode_on_ ? -hysteresis : hysteresis);
        std::size_t path = 0;
        if (inputs.{pwm_field} != 0U) {{
            body_on_ = false;
            path = 1;
        }} else {{
            body_on_ = reverse_mosfet_voltage >= body_threshold + (body_on_ ? -hysteresis : hysteresis);
            path = body_on_ ? 2 : 0;
        }}
        return (diode_on_ ? 3U : 0U) + path;'''
        switch_state_members = "    bool diode_on_{false};\n    bool body_on_{false};"
    source_name = document["circuit"]["source"].get("file") or "<unknown>"
    source_hash = document["circuit"]["source"].get("sha256") or "<unknown>"
    return f'''#pragma once

// Generated by GMP mna_solver/cpp_codegen.py from {source_name}.
// Source SHA-256: {source_hash}
// Matrix equivalence tolerance: {_number(storage.tolerance)}
// Logical states: {storage.logical_state_count}; unique calculation states: {storage.unique_state_count}.
// Do not hand-edit; regenerate from the circuit-data JSON file.

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

class {class_name} {{
public:
    static constexpr std::size_t state_count = {state_count};
    static constexpr std::size_t signal_count = {signal_count};
    static constexpr std::size_t analog_input_count = {input_count};
    static constexpr std::size_t pwm_input_count = {len(pwm_ports)};
    static constexpr std::size_t topology_count = {topology_count};
    static constexpr std::size_t calculation_state_count = {storage.unique_state_count};
{pool_count_constants}
    static constexpr double normal_step_s = {_number(document['solver']['normal_step_s'])};
    static constexpr double short_step_s = {_number(document['solver']['short_step_s'])};
    static constexpr double matrix_tolerance = {_number(storage.tolerance)};

    struct Inputs {{
{pwm_members}
{input_members}
    }};

    struct Outputs {{
{output_members}

        double operator[](std::string_view name) const {{
{output_lookup}
            throw std::out_of_range("unknown circuit output: " + std::string(name));
        }}
    }};

    Outputs output{{}};

    {class_name}() {{ reset(); }}

    void reset() {{
        state_.setZero();
        signals_.setZero();
{reset_switch_state}
        last_topology_index_ = 0;
        last_calculation_state_index_ = topology_to_calculation_state()[0];
        output = Outputs{{}};
    }}

    const Outputs& step_short(const Inputs& inputs) {{ return step(inputs, true); }}
    const Outputs& step_normal(const Inputs& inputs) {{ return step(inputs, false); }}

    const Outputs& step_short({function_parameters}) {{
        return step_short(Inputs{{{function_arguments}}});
    }}

    const Outputs& step_normal({function_parameters}) {{
        return step_normal(Inputs{{{function_arguments}}});
    }}

    const Outputs& run({function_parameters}) {{
        return step_normal({function_arguments});
    }}

    const Outputs& operator()({function_parameters}) {{
        return run({function_arguments});
    }}

    double operator[](std::string_view name) const {{ return output[name]; }}
    const auto& state() const noexcept {{ return state_; }}
    std::size_t last_topology_index() const noexcept {{ return last_topology_index_; }}
    std::size_t last_calculation_state_index() const noexcept {{ return last_calculation_state_index_; }}

private:
    using StateMatrix = Eigen::Matrix<double, {state_count}, {state_count}>;
    using InputMatrix = Eigen::Matrix<double, {state_count}, {input_count}>;
    using StateVector = Eigen::Matrix<double, {state_count}, 1>;
    using SignalMatrix = Eigen::Matrix<double, {signal_count}, {state_count}>;
    using SignalInputMatrix = Eigen::Matrix<double, {signal_count}, {input_count}>;
    using SignalVector = Eigen::Matrix<double, {signal_count}, 1>;
    using InputVector = Eigen::Matrix<double, {input_count}, 1>;

    struct CalculationState {{
{calculation_state_fields}
    }};

{pool_function_text}

    static const std::array<CalculationState, calculation_state_count>& calculation_states() {{
        static const std::array<CalculationState, calculation_state_count> value{{{{
            {calculation_state_values}
        }}}};
        return value;
    }}

    static const std::array<std::size_t, topology_count>& topology_to_calculation_state() {{
        static const std::array<std::size_t, topology_count> value{{{{
            {topology_mapping_values}
        }}}};
        return value;
    }}

    std::size_t select_topology(const Inputs& inputs) {{
{selection_body}
    }}

    const Outputs& step(const Inputs& inputs, bool use_short_step) {{
        last_topology_index_ = select_topology(inputs);
        last_calculation_state_index_ = topology_to_calculation_state()[last_topology_index_];
        const auto& calculation_state = calculation_states()[last_calculation_state_index_];
        InputVector input_vector;
        input_vector << {input_vector};
        if (use_short_step) {{
            state_ = state_matrices()[calculation_state.short_A] * state_
                + input_matrices()[calculation_state.short_B] * input_vector
                + state_vectors()[calculation_state.short_bias];
        }} else {{
            state_ = state_matrices()[calculation_state.normal_A] * state_
                + input_matrices()[calculation_state.normal_B] * input_vector
                + state_vectors()[calculation_state.normal_bias];
        }}
        signals_ = signal_matrices()[calculation_state.C] * state_
            + signal_input_matrices()[calculation_state.D] * input_vector
            + signal_vectors()[calculation_state.output_bias];
{output_updates}
        return output;
    }}

    StateVector state_{{StateVector::Zero()}};
    SignalVector signals_{{SignalVector::Zero()}};
{switch_state_members}
    std::size_t last_topology_index_{{0}};
    std::size_t last_calculation_state_index_{{0}};
}};
'''


def generate_cpp_project(
    data_path: str | Path,
    output_directory: str | Path,
    class_name: str | None = None,
    matrix_tolerance: float | None = None,
    show_progress: bool = False,
) -> dict[str, Path]:
    document = load_circuit_data(data_path)
    selected_tolerance = (
        float(document.get("solver", {}).get("matrix_tolerance", DEFAULT_MATRIX_TOLERANCE))
        if matrix_tolerance is None
        else float(matrix_tolerance)
    )
    logical_count = len(document["topologies"])
    if show_progress:
        print(f"circuit:             {document['circuit']['name']}")
        print(f"logical states:      {logical_count}")
        print(f"discretization:      {document['solver']['method']}")
        print(f"normal step:         {float(document['solver']['normal_step_s']):.12g} s")
        print(f"short step:          {float(document['solver']['short_step_s']):.12g} s")
        print(f"matrix tolerance:    {selected_tolerance:.12g}")
    bar = TimedProgressBar("Deduplicating states", logical_count) if show_progress else None
    plan = build_matrix_dedup_plan(
        document,
        selected_tolerance,
        None if bar is None else bar.update,
    )
    if bar is not None:
        bar.finish()
        saved = plan.coefficients_before - plan.coefficients_after
        ratio = 100.0 * saved / plan.coefficients_before if plan.coefficients_before else 0.0
        print(f"unique states:       {plan.unique_state_count}")
        print(f"deduplicated states: {plan.deduplicated_state_count}")
        print(f"shared matrix copies: {plan.shared_matrix_copy_count}")
        print(
            f"pooled coefficients: {plan.coefficients_after}/{plan.coefficients_before} "
            f"({ratio:.2f}% removed)"
        )
        for type_name in _POOL_NAMES:
            print(
                f"  {type_name:<23} {len(plan.pools[type_name])}/"
                f"{plan.pool_references[type_name]} unique copies"
            )
    selected_class = _identifier(class_name) if class_name else _default_class_name(data_path, document)
    stem = _identifier(selected_class.lower())
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    header = output / f"{stem}.hpp"
    header.write_text(
        render_header(document, selected_class, selected_tolerance, plan=plan),
        encoding="utf-8",
    )
    return {"header": header}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate an Eigen C++ circuit calculation class")
    parser.add_argument("data")
    parser.add_argument("output_directory")
    parser.add_argument("--class-name")
    parser.add_argument("--matrix-tolerance", type=float)
    parser.add_argument("--no-progress", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        generated = generate_cpp_project(
            args.data,
            args.output_directory,
            args.class_name,
            args.matrix_tolerance,
            not args.no_progress,
        )
        for kind, path in generated.items():
            print(f"{kind}: {path}")
    except (OSError, ValueError, KeyError) as error:
        print(f"error: {error}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
