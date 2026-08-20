"""Generate a fixed-size Eigen C++ circuit class from circuit-data JSON."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Mapping, Sequence

from circuit_data import load_circuit_data


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


def _topology_initializer(topology: Mapping) -> str:
    normal = topology["discrete"]["normal"]
    short = topology["discrete"]["short"]
    values = [
        _matrix_expression("StateMatrix", normal["A"]),
        _matrix_expression("InputMatrix", normal["B"]),
        _matrix_expression("StateVector", normal["bias"]),
        _matrix_expression("StateMatrix", short["A"]),
        _matrix_expression("InputMatrix", short["B"]),
        _matrix_expression("StateVector", short["bias"]),
        _matrix_expression("SignalMatrix", normal["C"]),
        _matrix_expression("SignalInputMatrix", normal["D"]),
        _matrix_expression("SignalVector", normal["output_bias"]),
    ]
    return "Topology{\n                " + ",\n                ".join(values) + "\n            }"


def render_header(document: Mapping, class_name: str) -> str:
    state_count = len(document["state"]["names"])
    signal_count = len(document["signals"]["names"])
    pwm_ports = [port for port in document["ports"]["inputs"] if port["data_type"] == "uint32_t"]
    input_ports = [port for port in document["ports"]["inputs"] if port["data_type"] == "double"]
    outputs = document["ports"]["outputs"]
    input_count = len(input_ports)
    topology_count = len(document["topologies"])
    switching = document["switching"]
    pwm_fields = [_identifier(port["name"], "PWM") for port in pwm_ports]
    input_fields = [_identifier(port["name"], "input") for port in input_ports]
    output_fields = [_identifier(port["field"], "output") for port in outputs]
    topology_values = ",\n            ".join(_topology_initializer(item) for item in document["topologies"])
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
    if switching.get("kind") == "multi_mosfet":
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
        reset_switch_state = "        body_on_.fill(false);"
        selection_body = "\n".join(
            [
                f"        constexpr double hysteresis = {_number(switching['voltage_hysteresis_V'])};",
                "        std::size_t topology_index = 0U;",
                *switch_lines,
                "        return topology_index;",
            ]
        )
        switch_state_members = f"    std::array<bool, {len(switching['switches'])}> body_on_{{}};"
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
    static constexpr double normal_step_s = {_number(document['solver']['normal_step_s'])};
    static constexpr double short_step_s = {_number(document['solver']['short_step_s'])};

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

private:
    using StateMatrix = Eigen::Matrix<double, {state_count}, {state_count}>;
    using InputMatrix = Eigen::Matrix<double, {state_count}, {input_count}>;
    using StateVector = Eigen::Matrix<double, {state_count}, 1>;
    using SignalMatrix = Eigen::Matrix<double, {signal_count}, {state_count}>;
    using SignalInputMatrix = Eigen::Matrix<double, {signal_count}, {input_count}>;
    using SignalVector = Eigen::Matrix<double, {signal_count}, 1>;
    using InputVector = Eigen::Matrix<double, {input_count}, 1>;

    struct Topology {{
        StateMatrix normal_A;
        InputMatrix normal_B;
        StateVector normal_bias;
        StateMatrix short_A;
        InputMatrix short_B;
        StateVector short_bias;
        SignalMatrix C;
        SignalInputMatrix D;
        SignalVector output_bias;
    }};

    static const std::array<Topology, topology_count>& topologies() {{
        static const std::array<Topology, topology_count> value{{{{
            {topology_values}
        }}}};
        return value;
    }}

    std::size_t select_topology(const Inputs& inputs) {{
{selection_body}
    }}

    const Outputs& step(const Inputs& inputs, bool use_short_step) {{
        last_topology_index_ = select_topology(inputs);
        const auto& topology = topologies()[last_topology_index_];
        InputVector input_vector;
        input_vector << {input_vector};
        if (use_short_step) {{
            state_ = topology.short_A * state_ + topology.short_B * input_vector + topology.short_bias;
        }} else {{
            state_ = topology.normal_A * state_ + topology.normal_B * input_vector + topology.normal_bias;
        }}
        signals_ = topology.C * state_ + topology.D * input_vector + topology.output_bias;
{output_updates}
        return output;
    }}

    StateVector state_{{StateVector::Zero()}};
    SignalVector signals_{{SignalVector::Zero()}};
{switch_state_members}
    std::size_t last_topology_index_{{0}};
}};
'''


def generate_cpp_project(data_path: str | Path, output_directory: str | Path, class_name: str | None = None) -> dict[str, Path]:
    document = load_circuit_data(data_path)
    selected_class = _identifier(class_name) if class_name else _default_class_name(data_path, document)
    stem = _identifier(selected_class.lower())
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    header = output / f"{stem}.hpp"
    header.write_text(render_header(document, selected_class), encoding="utf-8")
    return {"header": header}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate an Eigen C++ circuit calculation class")
    parser.add_argument("data")
    parser.add_argument("output_directory")
    parser.add_argument("--class-name")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        generated = generate_cpp_project(args.data, args.output_directory, args.class_name)
        for kind, path in generated.items():
            print(f"{kind}: {path}")
    except (OSError, ValueError, KeyError) as error:
        print(f"error: {error}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
