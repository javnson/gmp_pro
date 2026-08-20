"""Portable circuit-data export and data-driven piecewise simulation.

The JSON document produced here is the stable boundary between netlist/MNA
analysis and downstream simulation or code generation.  It contains no Python
objects or symbolic expressions required at run time.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping, Sequence

import numpy as np

from console_progress import TimedProgressBar
from mna_solver import (
    NetlistError,
    StateSpaceModel,
    discover_pmsm_current_source_ports,
    discretize,
    parse_netlist,
    parse_spice_value,
)
from switched_solver import (
    MultiDiodeSwitchLinearModel,
    MultiMosfetDiodeLinearModel,
    MultiMosfetLinearModel,
    MosfetPath,
    PiecewiseLinearModel,
    TopologyKey,
    build_piecewise_model,
    ideal_source_nodes_requiring_capacitance_suppression,
    piecewise_topology_count,
)


SCHEMA_NAME = "gmp.mna_solver.circuit_data"
SCHEMA_VERSION = 1
DEFAULT_MATRIX_TOLERANCE = 1e-12


def _key(value: str) -> str:
    return value.upper()


def _matrix(value: np.ndarray) -> list[list[float]]:
    return np.asarray(value, dtype=float).tolist()


def _vector(value: np.ndarray) -> list[float]:
    return np.asarray(value, dtype=float).reshape(-1).tolist()


def _field_name(observation: str) -> str:
    match = re.fullmatch(r"[VI]\(([^,)]+)(?:,[^)]+)?\)", observation, re.IGNORECASE)
    raw = match.group(1) if match else observation
    field = re.sub(r"[^A-Za-z0-9_]", "_", raw).strip("_")
    if not field:
        field = "value"
    if field[0].isdigit():
        field = "n_" + field
    return field


def _unique_output_fields(names: Sequence[str]) -> list[str]:
    result: list[str] = []
    used: set[str] = set()
    for name in names:
        base = _field_name(name)
        candidate = base
        suffix = 2
        while candidate.lower() in used:
            candidate = f"{base}_{suffix}"
            suffix += 1
        used.add(candidate.lower())
        result.append(candidate)
    return result


def _affine_model(
    model: StateSpaceModel,
    external_inputs: Sequence[str],
    *,
    discrete_dt: float | None = None,
    method: str = "backward_euler",
) -> dict:
    selected = discretize(model, discrete_dt, method) if discrete_dt is not None else model
    input_indices = {_key(name): index for index, name in enumerate(selected.input_names)}
    external_indices = [input_indices[_key(name)] for name in external_inputs]
    constant_indices = [
        index for index, name in enumerate(selected.input_names) if _key(name) not in {_key(item) for item in external_inputs}
    ]
    defaults = np.asarray(selected.input_defaults, dtype=float)
    B = np.asarray(selected.B, dtype=float)
    D = np.asarray(selected.D, dtype=float)
    bias_x = B[:, constant_indices] @ defaults[constant_indices] if constant_indices else np.zeros(B.shape[0])
    bias_y = D[:, constant_indices] @ defaults[constant_indices] if constant_indices else np.zeros(D.shape[0])
    document = {
        "A": _matrix(selected.A),
        "B": _matrix(B[:, external_indices]),
        "bias": _vector(bias_x),
        "C": _matrix(selected.C),
        "D": _matrix(D[:, external_indices]),
        "output_bias": _vector(bias_y),
    }
    if discrete_dt is not None:
        document.update({"dt_s": float(discrete_dt), "method": selected.method})
    return document


def _model_document(model) -> dict:
    return {
        "name": model.name,
        "kind": model.kind,
        "parameters": {
            name: {"spice": text, "value": model.numeric(name)}
            for name, text in sorted(model.parameters.items())
        },
    }


def _pwm_port_name(source_name: str) -> str:
    return source_name[1:] if _key(source_name).startswith("V") else source_name


def _decorate_pmsm_ports(circuit, input_ports: list[dict], output_ports: list[dict]) -> list[dict]:
    """Annotate complete current-source PMSM interfaces and return JSON metadata."""

    input_lookup = {_key(port["name"]): port for port in input_ports}
    output_lookup = {_key(port["name"]): port for port in output_ports}
    documents = []
    for motor in discover_pmsm_current_source_ports(circuit):
        phases = []
        for phase in "ABC":
            source = motor.source(phase)
            voltage_name = motor.voltage_name(phase)
            if _key(source.name) not in input_lookup or _key(voltage_name) not in output_lookup:
                raise NetlistError(f"{motor.name}: incomplete generated PMSM coupling ports")
            input_port = input_lookup[_key(source.name)]
            input_port.update(
                {
                    "role": "pmsm_phase_current",
                    "motor": motor.name,
                    "phase": phase,
                    "default": 0.0,
                }
            )
            output_port = output_lookup[_key(voltage_name)]
            output_port.update(
                {
                    "role": "pmsm_phase_voltage",
                    "motor": motor.name,
                    "phase": phase,
                }
            )
            phases.append(
                {
                    "phase": phase,
                    "current_input": source.name,
                    "voltage_output": voltage_name,
                    "terminal_node": source.nodes[0],
                    "neutral_node": source.nodes[1],
                }
            )
        documents.append({"name": motor.name, "kind": "pmsm_current_source", "phases": phases})
    return documents


def _build_multi_mosfet_circuit_data(
    model: MultiMosfetLinearModel | MultiMosfetDiodeLinearModel,
    *,
    source_path: str | Path | None,
    normal_step_s: float,
    short_step_s: float,
    method: str,
    voltage_hysteresis: float,
    matrix_tolerance: float,
    progress: Callable[[int, int], None] | None,
) -> dict:
    reference = next(iter(model.topologies.values())).state
    gate_names = {_key(source.name) for source in model.gate_sources}
    source_names = [
        element.name
        for element in model.source_circuit.elements
        if element.kind in {"V", "I"} and _key(element.name) not in gate_names
    ]
    external_inputs = [
        name for name in reference.input_names if _key(name) in {_key(item) for item in source_names}
    ]
    source_elements = {_key(element.name): element for element in model.source_circuit.elements}
    input_ports = sorted([
        {
            "name": _pwm_port_name(source.name),
            "source_element": source.name,
            "data_type": "uint32_t",
            "role": "mosfet_gate_command",
            "switch_index": index,
            "default": 0,
        }
        for index, source in enumerate(model.gate_sources)
    ], key=lambda port: _key(port["name"]))
    input_ports.extend(
        {
            "name": name,
            "source_element": name,
            "data_type": "double",
            "default": float(source_elements[_key(name)].numeric_value({})),
        }
        for name in external_inputs
    )

    public_names = [observation.name for observation in model.source_circuit.observations]
    fields = _unique_output_fields(public_names)
    signal_indices = {_key(name): index for index, name in enumerate(reference.output_names)}
    output_ports = [
        {
            "name": name,
            "field": field,
            "data_type": "double",
            "signal_index": signal_indices[_key(name)],
        }
        for name, field in zip(public_names, fields)
    ]
    pmsm_documents = _decorate_pmsm_ports(model.source_circuit, input_ports, output_ports)

    mixed_model = isinstance(model, MultiMosfetDiodeLinearModel)
    diode_documents = []
    diode_selection = []
    if mixed_model:
        ideal_source_nodes = ideal_source_nodes_requiring_capacitance_suppression(
            model.source_circuit.elements, model.gate_sources
        )
        for index, (diode, item) in enumerate(
            zip(model.diodes, model.diode_parameters)
        ):
            device_model = model.source_circuit.models[_key(diode.model_name or "")]
            capacitance_suppressed = any(
                _key(node) in ideal_source_nodes for node in diode.nodes[:2]
            )
            diode_documents.append(
                {
                    "instance": diode.name,
                    "model": _model_document(device_model),
                    "reduced": {
                        "forward_voltage_V": item.forward_voltage,
                        "on_resistance_ohm": item.on_resistance,
                        "off_resistance_ohm": item.off_resistance,
                        "extracted_junction_capacitance_F": item.junction_capacitance,
                        "implemented_junction_capacitance_F": (
                            0.0 if capacitance_suppressed else item.junction_capacitance
                        ),
                    },
                    "extraction": {
                        "forward_voltage": "VJ",
                        "on_resistance": "RS",
                        "junction_capacitance": "SPICE depletion C(Vbias) from CJO,VJ,M,FC; Vbias=0 by default",
                        "junction_capacitance_suppressed_for_ideal_source": capacitance_suppressed,
                    },
                }
            )
            diode_selection.append(
                {
                    "index": index,
                    "instance": diode.name,
                    "anode_signal_index": signal_indices[_key(f"V({diode.name}.A)")],
                    "cathode_signal_index": signal_indices[_key(f"V({diode.name}.K)")],
                    "forward_threshold_V": item.forward_voltage,
                }
            )

    mosfet_documents = []
    switch_documents = []
    for index, (mosfet, gate_source, item) in enumerate(
        zip(model.mosfets, model.gate_sources, model.parameters)
    ):
        device_model = model.source_circuit.models[_key(mosfet.model_name or "")]
        rd = device_model.numeric("RD", 0.0) or 0.0
        rs = device_model.numeric("RS", 0.0) or 0.0
        mosfet_documents.append(
            {
                "instance": mosfet.name,
                "public_name": item.public_name,
                "model": _model_document(device_model),
                "reduced": {
                    "channel_on_resistance_ohm": item.on_resistance,
                    "off_resistance_ohm": item.off_resistance,
                    "output_capacitance_F": item.output_capacitance,
                    "body_forward_voltage_V": item.body_forward_voltage,
                    "body_on_resistance_ohm": item.body_on_resistance,
                    "gate_drive_voltage_V": item.gate_drive_voltage,
                },
                "extraction": {
                    "channel_on_resistance": "RD+RS+1/(KP*(W/L)*(Vdrive-VTO))",
                    "off_resistance": "RDS",
                    "output_capacitance": "CBD+CGDO",
                    "body_forward_voltage": "PB",
                    "body_on_resistance": "RD+RS",
                    "series_resistance_ohm": rd + rs,
                },
            }
        )
        switch_documents.append(
            {
                "index": index,
                "instance": mosfet.name,
                "public_name": item.public_name,
                "pwm_port": _pwm_port_name(gate_source.name),
                "gate_source_ignored": gate_source.name,
                "drain_signal_index": signal_indices[_key(f"V({item.public_name}.D)")],
                "source_signal_index": signal_indices[_key(f"V({item.public_name}.S)")],
                "body_forward_threshold_V": item.body_forward_voltage,
            }
        )

    topology_documents = []
    topology_order = []
    topology_count = len(model.topologies)
    for topology_index, (key, topology) in enumerate(model.topologies.items(), start=1):
        topology_order.append(key.name)
        continuous = _affine_model(topology.state, external_inputs)
        continuous["reconstruction_x"] = _matrix(topology.state.reconstruction_x)
        continuous["reconstruction_u"] = _matrix(topology.state.reconstruction_u)
        continuous["unknown_names"] = list(topology.state.unknown_names)
        continuous["raw_input_names"] = list(topology.state.input_names)
        continuous["raw_input_defaults"] = _vector(topology.state.input_defaults)
        topology_document = {
            "index": len(topology_documents),
            "name": key.name,
            "mosfet_paths": [path.value for path in key.paths],
            "continuous": continuous,
            "discrete": {
                "normal": _affine_model(
                    topology.state, external_inputs, discrete_dt=normal_step_s, method=method
                ),
                "short": _affine_model(
                    topology.state, external_inputs, discrete_dt=short_step_s, method=method
                ),
            },
        }
        if mixed_model:
            topology_document["diode_states"] = list(key.diode_states)
        topology_documents.append(topology_document)
        if progress is not None:
            progress(topology_index, topology_count)

    source = {"file": None, "sha256": None}
    if source_path is not None:
        path = Path(source_path)
        source = {"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    return {
        "schema": {"name": SCHEMA_NAME, "version": SCHEMA_VERSION},
        "circuit": {"name": model.source_circuit.title, "source": source},
        "ports": {"inputs": input_ports, "outputs": output_ports},
        "state": {"names": list(reference.state_names), "initial": [0.0] * len(reference.state_names)},
        "signals": {"names": list(reference.output_names)},
        "devices": {
            "diodes": diode_documents,
            "mosfets": mosfet_documents,
            "pmsm_current_sources": pmsm_documents,
        },
        "switching": {
            "kind": "multi_mosfet_diode" if mixed_model else "multi_mosfet",
            "switches": switch_documents,
            "diodes": diode_selection,
            "voltage_hysteresis_V": voltage_hysteresis,
            "selection_uses_previous_terminal_voltage": True,
            "topology_order": topology_order,
            "topology_count": len(topology_documents),
        },
        "solver": {
            "method": method,
            "normal_step_s": normal_step_s,
            "short_step_s": short_step_s,
            "matrix_tolerance": matrix_tolerance,
        },
        "topologies": topology_documents,
    }


def _build_multi_diode_switch_circuit_data(
    model: MultiDiodeSwitchLinearModel,
    *,
    source_path: str | Path | None,
    normal_step_s: float,
    short_step_s: float,
    method: str,
    voltage_hysteresis: float,
    matrix_tolerance: float,
    progress: Callable[[int, int], None] | None,
) -> dict:
    reference = next(iter(model.topologies.values())).state
    control_names = {_key(source.name) for source in model.control_sources}
    source_names = [
        element.name
        for element in model.source_circuit.elements
        if element.kind in {"V", "I"} and _key(element.name) not in control_names
    ]
    external_inputs = [
        name for name in reference.input_names if _key(name) in {_key(item) for item in source_names}
    ]
    source_elements = {_key(element.name): element for element in model.source_circuit.elements}
    input_ports = sorted(
        [
            {
                "name": _pwm_port_name(source.name),
                "source_element": source.name,
                "data_type": "uint32_t",
                "role": "controlled_switch_command",
                "switch_index": index,
                "switch_instance": switch.name,
                "default": 0,
            }
            for index, (switch, source) in enumerate(
                zip(model.switches, model.control_sources)
            )
        ],
        key=lambda port: _key(port["name"]),
    )
    input_ports.extend(
        {
            "name": name,
            "source_element": name,
            "data_type": "double",
            "default": float(source_elements[_key(name)].numeric_value({})),
        }
        for name in external_inputs
    )

    public_names = [observation.name for observation in model.source_circuit.observations]
    fields = _unique_output_fields(public_names)
    signal_indices = {_key(name): index for index, name in enumerate(reference.output_names)}
    output_ports = [
        {
            "name": name,
            "field": field,
            "data_type": "double",
            "signal_index": signal_indices[_key(name)],
        }
        for name, field in zip(public_names, fields)
    ]
    pmsm_documents = _decorate_pmsm_ports(model.source_circuit, input_ports, output_ports)

    ideal_source_nodes = ideal_source_nodes_requiring_capacitance_suppression(
        model.source_circuit.elements, model.control_sources
    )
    diode_documents = []
    diode_selection = []
    for index, (diode, item) in enumerate(zip(model.diodes, model.diode_parameters)):
        device_model = model.source_circuit.models[_key(diode.model_name or "")]
        capacitance_suppressed = any(
            _key(node) in ideal_source_nodes for node in diode.nodes[:2]
        )
        diode_documents.append(
            {
                "instance": diode.name,
                "model": _model_document(device_model),
                "reduced": {
                    "forward_voltage_V": item.forward_voltage,
                    "on_resistance_ohm": item.on_resistance,
                    "off_resistance_ohm": item.off_resistance,
                    "extracted_junction_capacitance_F": item.junction_capacitance,
                    "implemented_junction_capacitance_F": (
                        0.0 if capacitance_suppressed else item.junction_capacitance
                    ),
                },
                "extraction": {
                    "forward_voltage": "VJ",
                    "on_resistance": "RS",
                    "junction_capacitance": "SPICE depletion C(Vbias) from CJO,VJ,M,FC; Vbias=0 by default",
                    "junction_capacitance_suppressed_for_ideal_source": capacitance_suppressed,
                },
            }
        )
        diode_selection.append(
            {
                "index": index,
                "instance": diode.name,
                "anode_signal_index": signal_indices[_key(f"V({diode.name}.A)")],
                "cathode_signal_index": signal_indices[_key(f"V({diode.name}.K)")],
                "forward_threshold_V": item.forward_voltage,
            }
        )

    switch_documents = []
    switch_selection = []
    for index, (switch, source, item) in enumerate(
        zip(model.switches, model.control_sources, model.switch_parameters)
    ):
        device_model = model.source_circuit.models[_key(switch.model_name or "")]
        command_port = _pwm_port_name(source.name)
        switch_documents.append(
            {
                "instance": switch.name,
                "model": _model_document(device_model),
                "reduced": {
                    "on_resistance_ohm": item.on_resistance,
                    "off_resistance_ohm": item.off_resistance,
                },
                "extraction": {
                    "on_resistance": "RON, clamped to 1 mOhm for numeric conditioning",
                    "off_resistance": "ROFF",
                },
            }
        )
        switch_selection.append(
            {
                "index": index,
                "instance": switch.name,
                "command_port": command_port,
                "control_source_ignored": source.name,
            }
        )

    topology_documents = []
    topology_order = []
    topology_count = len(model.topologies)
    for topology_index, (key, topology) in enumerate(model.topologies.items(), start=1):
        topology_order.append(key.name)
        continuous = _affine_model(topology.state, external_inputs)
        continuous["reconstruction_x"] = _matrix(topology.state.reconstruction_x)
        continuous["reconstruction_u"] = _matrix(topology.state.reconstruction_u)
        continuous["unknown_names"] = list(topology.state.unknown_names)
        continuous["raw_input_names"] = list(topology.state.input_names)
        continuous["raw_input_defaults"] = _vector(topology.state.input_defaults)
        topology_documents.append(
            {
                "index": len(topology_documents),
                "name": key.name,
                "diode_states": list(key.diode_states),
                "switch_states": list(key.switch_states),
                "continuous": continuous,
                "discrete": {
                    "normal": _affine_model(
                        topology.state,
                        external_inputs,
                        discrete_dt=normal_step_s,
                        method=method,
                    ),
                    "short": _affine_model(
                        topology.state,
                        external_inputs,
                        discrete_dt=short_step_s,
                        method=method,
                    ),
                },
            }
        )
        if progress is not None:
            progress(topology_index, topology_count)

    source = {"file": None, "sha256": None}
    if source_path is not None:
        path = Path(source_path)
        source = {"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    return {
        "schema": {"name": SCHEMA_NAME, "version": SCHEMA_VERSION},
        "circuit": {"name": model.source_circuit.title, "source": source},
        "ports": {"inputs": input_ports, "outputs": output_ports},
        "state": {"names": list(reference.state_names), "initial": [0.0] * len(reference.state_names)},
        "signals": {"names": list(reference.output_names)},
        "devices": {
            "diodes": diode_documents,
            "mosfets": [],
            "controlled_switches": switch_documents,
            "pmsm_current_sources": pmsm_documents,
        },
        "switching": {
            "kind": "multi_diode_switch",
            "diodes": diode_selection,
            "switches": switch_selection,
            "voltage_hysteresis_V": voltage_hysteresis,
            "selection_uses_previous_terminal_voltage": True,
            "topology_order": topology_order,
            "topology_count": len(topology_documents),
        },
        "solver": {
            "method": method,
            "normal_step_s": normal_step_s,
            "short_step_s": short_step_s,
            "matrix_tolerance": matrix_tolerance,
        },
        "topologies": topology_documents,
    }


def build_circuit_data(
    model: PiecewiseLinearModel | MultiMosfetLinearModel | MultiMosfetDiodeLinearModel | MultiDiodeSwitchLinearModel,
    *,
    source_path: str | Path | None = None,
    normal_step_s: float = 100e-9,
    short_step_s: float = 0.5e-9,
    method: str = "backward_euler",
    voltage_hysteresis: float = 1e-6,
    matrix_tolerance: float = DEFAULT_MATRIX_TOLERANCE,
    progress: Callable[[int, int], None] | None = None,
) -> dict:
    """Convert PWL topologies to a self-contained JSON-compatible dict."""

    values = (normal_step_s, short_step_s, matrix_tolerance)
    if any(value <= 0.0 or not math.isfinite(value) for value in values):
        raise ValueError(
            "normal/short simulation steps and matrix tolerance must be finite positive numbers"
        )
    if isinstance(model, MultiDiodeSwitchLinearModel):
        return _build_multi_diode_switch_circuit_data(
            model,
            source_path=source_path,
            normal_step_s=normal_step_s,
            short_step_s=short_step_s,
            method=method,
            voltage_hysteresis=voltage_hysteresis,
            matrix_tolerance=matrix_tolerance,
            progress=progress,
        )
    if isinstance(model, (MultiMosfetLinearModel, MultiMosfetDiodeLinearModel)):
        return _build_multi_mosfet_circuit_data(
            model,
            source_path=source_path,
            normal_step_s=normal_step_s,
            short_step_s=short_step_s,
            method=method,
            voltage_hysteresis=voltage_hysteresis,
            matrix_tolerance=matrix_tolerance,
            progress=progress,
        )
    reference = next(iter(model.topologies.values())).state
    source_names = [
        element.name
        for element in model.source_circuit.elements
        if element.kind in {"V", "I"} and _key(element.name) != _key(model.gate_source_name)
    ]
    external_inputs = [name for name in reference.input_names if _key(name) in {_key(item) for item in source_names}]
    source_elements = {_key(element.name): element for element in model.source_circuit.elements}
    input_ports = [
        {
            "name": name,
            "source_element": name,
            "data_type": "double",
            "default": float(source_elements[_key(name)].numeric_value({})),
        }
        for name in external_inputs
    ]
    input_ports.insert(
        0,
        {
            "name": "PWM",
            "source_element": model.gate_source_name,
            "data_type": "uint32_t",
            "role": "mosfet_gate_command",
            "default": 0,
        },
    )

    public_names = [observation.name for observation in model.source_circuit.observations]
    fields = _unique_output_fields(public_names)
    signal_indices = {_key(name): index for index, name in enumerate(reference.output_names)}
    output_ports = []
    for name, field in zip(public_names, fields):
        if _key(name) not in signal_indices:
            raise NetlistError(f"probe {name!r} is absent from the topology output equations")
        output_ports.append(
            {"name": name, "field": field, "data_type": "double", "signal_index": signal_indices[_key(name)]}
        )
    pmsm_documents = _decorate_pmsm_ports(model.source_circuit, input_ports, output_ports)

    diode_model = model.source_circuit.models[_key(model.diode.model_name or "")]
    mosfet_model = model.source_circuit.models[_key(model.mosfet.model_name or "")]
    p = model.parameters
    diode_vj = diode_model.numeric("VJ", 0.7) or 0.7
    diode_m = diode_model.numeric("M", 0.5) or 0.5
    diode_fc = diode_model.numeric("FC", 0.5) or 0.5
    mosfet_rd = mosfet_model.numeric("RD", 0.0) or 0.0
    mosfet_rs = mosfet_model.numeric("RS", 0.0) or 0.0
    devices = {
        "diode": {
            "instance": model.diode.name,
            "model": _model_document(diode_model),
            "reduced": {
                "forward_voltage_V": p.diode_forward_voltage,
                "on_resistance_ohm": p.diode_on_resistance,
                "off_resistance_ohm": p.diode_off_resistance,
                "junction_capacitance_F": p.diode_junction_capacitance,
            },
            "extraction": {
                "forward_voltage": "VJ",
                "on_resistance": "RS",
                "junction_capacitance": "SPICE depletion C(Vbias) from CJO,VJ,M,FC; Vbias=0 by default",
                "junction_linearization_bias_V": 0.0,
                "junction_potential_V": diode_vj,
                "grading_coefficient": diode_m,
                "forward_coefficient": diode_fc,
            },
        },
        "mosfet": {
            "instance": model.mosfet.name,
            "public_name": p.mosfet_name,
            "model": _model_document(mosfet_model),
            "reduced": {
                "channel_on_resistance_ohm": p.mosfet_on_resistance,
                "off_resistance_ohm": p.mosfet_off_resistance,
                "output_capacitance_F": p.mosfet_output_capacitance,
                "body_forward_voltage_V": p.body_forward_voltage,
                "body_on_resistance_ohm": p.body_on_resistance,
                "gate_drive_voltage_V": p.gate_drive_voltage,
            },
            "extraction": {
                "channel_on_resistance": "RD+RS+1/(KP*(W/L)*(Vdrive-VTO))",
                "off_resistance": "RDS",
                "output_capacitance": "CBD+CGDO",
                "body_forward_voltage": "PB",
                "body_on_resistance": "RD+RS",
                "series_resistance_ohm": mosfet_rd + mosfet_rs,
            },
        },
        "pmsm_current_sources": pmsm_documents,
    }

    topology_documents = []
    topology_order = []
    topology_count = len(model.topologies)
    for topology_index, (key, topology) in enumerate(model.topologies.items(), start=1):
        if topology.state.state_names != reference.state_names or topology.state.output_names != reference.output_names:
            raise NetlistError("topologies do not share state/output coordinates")
        topology_order.append(key.name)
        continuous = _affine_model(topology.state, external_inputs)
        continuous["reconstruction_x"] = _matrix(topology.state.reconstruction_x)
        continuous["reconstruction_u"] = _matrix(topology.state.reconstruction_u)
        continuous["unknown_names"] = list(topology.state.unknown_names)
        continuous["raw_input_names"] = list(topology.state.input_names)
        continuous["raw_input_defaults"] = _vector(topology.state.input_defaults)
        topology_documents.append(
            {
                "index": len(topology_documents),
                "name": key.name,
                "diode_on": key.diode_on,
                "mosfet_path": key.mosfet_path.value,
                "continuous": continuous,
                "discrete": {
                    "normal": _affine_model(topology.state, external_inputs, discrete_dt=normal_step_s, method=method),
                    "short": _affine_model(topology.state, external_inputs, discrete_dt=short_step_s, method=method),
                },
            }
        )
        if progress is not None:
            progress(topology_index, topology_count)

    source = {"file": None, "sha256": None}
    if source_path is not None:
        path = Path(source_path)
        source = {"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    required_signals = {
        "mosfet_drain": signal_indices[_key(f"V({p.mosfet_name}.D)")],
        "mosfet_source": signal_indices[_key(f"V({p.mosfet_name}.S)")],
        "diode_anode": signal_indices[_key(f"V({model.diode.name}.A)")],
        "diode_cathode": signal_indices[_key(f"V({model.diode.name}.K)")],
    }
    return {
        "schema": {"name": SCHEMA_NAME, "version": SCHEMA_VERSION},
        "circuit": {"name": model.source_circuit.title, "source": source},
        "ports": {"inputs": input_ports, "outputs": output_ports},
        "state": {"names": list(reference.state_names), "initial": [0.0] * len(reference.state_names)},
        "signals": {"names": list(reference.output_names), "switch_terminal_indices": required_signals},
        "devices": devices,
        "switching": {
            "gate_source_ignored": model.gate_source_name,
            "diode_forward_threshold_V": p.diode_forward_voltage,
            "body_forward_threshold_V": p.body_forward_voltage,
            "voltage_hysteresis_V": voltage_hysteresis,
            "selection_uses_previous_terminal_voltage": True,
            "topology_order": topology_order,
        },
        "solver": {
            "method": method,
            "normal_step_s": normal_step_s,
            "short_step_s": short_step_s,
            "matrix_tolerance": matrix_tolerance,
        },
        "topologies": topology_documents,
    }


def validate_circuit_data(document: Mapping) -> None:
    schema = document.get("schema", {})
    if schema.get("name") != SCHEMA_NAME or schema.get("version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported circuit-data schema: {schema!r}")
    matrix_tolerance = float(
        document.get("solver", {}).get("matrix_tolerance", DEFAULT_MATRIX_TOLERANCE)
    )
    if matrix_tolerance <= 0.0 or not math.isfinite(matrix_tolerance):
        raise ValueError("matrix tolerance must be a finite positive number")
    topologies = document.get("topologies", [])
    expected_topologies = int(document.get("switching", {}).get("topology_count", 6))
    if len(topologies) != expected_topologies:
        raise ValueError(
            f"switched circuit data must contain {expected_topologies} topologies, got {len(topologies)}"
        )
    switching = document.get("switching", {})
    switches = switching.get("switches", [])
    if switching.get("kind") == "multi_mosfet" and expected_topologies != 3 ** len(switches):
        raise ValueError("multi-MOS topology count must be 3**number_of_switches")
    if switching.get("kind") == "multi_mosfet_diode":
        diode_count = len(switching.get("diodes", []))
        if expected_topologies != 3 ** len(switches) * 2**diode_count:
            raise ValueError(
                "mixed MOS/diode topology count must be 3**number_of_MOSFETs * 2**number_of_diodes"
            )
    if switching.get("kind") == "multi_diode_switch":
        device_count = len(switching.get("diodes", [])) + len(switches)
        if expected_topologies != 2**device_count:
            raise ValueError(
                "multi-diode/switch topology count must be 2**number_of_devices"
            )
    state_count = len(document["state"]["names"])
    signal_count = len(document["signals"]["names"])
    input_count = sum(port["data_type"] == "double" for port in document["ports"]["inputs"])
    for topology in topologies:
        for profile in ("normal", "short"):
            item = topology["discrete"][profile]
            if np.asarray(item["A"]).shape != (state_count, state_count):
                raise ValueError(f"{topology['name']} {profile}: invalid A dimensions")
            if np.asarray(item["B"]).shape != (state_count, input_count):
                raise ValueError(f"{topology['name']} {profile}: invalid B dimensions")
            if np.asarray(item["C"]).shape != (signal_count, state_count):
                raise ValueError(f"{topology['name']} {profile}: invalid C dimensions")


def write_circuit_data(path: str | Path, document: Mapping) -> None:
    validate_circuit_data(document)
    Path(path).write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def load_circuit_data(path: str | Path) -> dict:
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_circuit_data(document)
    return document


@dataclass
class DataSimulationResult:
    time: np.ndarray
    pwm: np.ndarray
    topology: list[str]
    states: np.ndarray
    outputs: np.ndarray
    output_names: list[str]


class CircuitDataSimulator:
    """Run only from exported JSON data; no netlist or MNA rebuild is used."""

    def __init__(self, document: Mapping):
        validate_circuit_data(document)
        self.document = document
        self.switching_kind = document["switching"].get("kind", "single_diode_mosfet")
        self.multi_mosfet_diode = self.switching_kind == "multi_mosfet_diode"
        self.multi_mosfet = self.switching_kind in {"multi_mosfet", "multi_mosfet_diode"}
        self.multi_diode_switch = self.switching_kind == "multi_diode_switch"
        if self.multi_mosfet_diode:
            self.topologies = {
                (tuple(item["mosfet_paths"]), tuple(item["diode_states"])): item
                for item in document["topologies"]
            }
        elif self.multi_mosfet:
            self.topologies = {
                tuple(item["mosfet_paths"]): item for item in document["topologies"]
            }
        elif self.multi_diode_switch:
            self.topologies = {
                (tuple(item["diode_states"]), tuple(item["switch_states"])): item
                for item in document["topologies"]
            }
        else:
            self.topologies = {
                (item["diode_on"], item["mosfet_path"]): item
                for item in document["topologies"]
            }
        self.external_inputs = [port for port in document["ports"]["inputs"] if port["data_type"] == "double"]
        self.pwm_ports = [port for port in document["ports"]["inputs"] if port["data_type"] == "uint32_t"]
        self.x = np.asarray(document["state"]["initial"], dtype=float)
        self.signals = np.zeros(len(document["signals"]["names"]), dtype=float)
        self.diode_on = (
            [False] * len(document["switching"].get("diodes", []))
            if self.multi_diode_switch or self.multi_mosfet_diode
            else False
        )
        self.body_on = [False] * len(document["switching"].get("switches", [])) if self.multi_mosfet else False
        self.last_topology = document["topologies"][0]["name"]

    def reset(self, state: Sequence[float] | None = None) -> None:
        self.x = np.asarray(self.document["state"]["initial"] if state is None else state, dtype=float).copy()
        if self.x.shape != (len(self.document["state"]["names"]),):
            raise ValueError("initial state has the wrong dimension")
        self.signals.fill(0.0)
        self.diode_on = (
            [False] * len(self.document["switching"].get("diodes", []))
            if self.multi_diode_switch or self.multi_mosfet_diode
            else False
        )
        self.body_on = [False] * len(self.document["switching"].get("switches", [])) if self.multi_mosfet else False
        self.last_topology = self.document["topologies"][0]["name"]

    def _select(self, pwm: int | Mapping[str, int]) -> tuple:
        if self.multi_diode_switch:
            supplied = (
                {_key(name): int(value) for name, value in pwm.items()}
                if isinstance(pwm, Mapping)
                else {_key(port["name"]): int(pwm) for port in self.pwm_ports}
            )
            unknown = set(supplied) - {_key(port["name"]) for port in self.pwm_ports}
            if unknown:
                raise ValueError("unknown digital input(s): " + ", ".join(sorted(unknown)))
            hysteresis = self.document["switching"]["voltage_hysteresis_V"]
            assert isinstance(self.diode_on, list)
            for index, diode in enumerate(self.document["switching"]["diodes"]):
                vak = (
                    self.signals[diode["anode_signal_index"]]
                    - self.signals[diode["cathode_signal_index"]]
                )
                threshold = diode["forward_threshold_V"]
                self.diode_on[index] = vak >= threshold + (
                    -hysteresis if self.diode_on[index] else hysteresis
                )
            switch_states = tuple(
                supplied.get(_key(switch["command_port"]), 0) != 0
                for switch in self.document["switching"]["switches"]
            )
            return tuple(self.diode_on), switch_states
        if self.multi_mosfet:
            supplied = (
                {_key(name): int(value) for name, value in pwm.items()}
                if isinstance(pwm, Mapping)
                else {_key(port["name"]): int(pwm) for port in self.pwm_ports}
            )
            unknown = set(supplied) - {_key(port["name"]) for port in self.pwm_ports}
            if unknown:
                raise ValueError("unknown PWM input(s): " + ", ".join(sorted(unknown)))
            paths: list[str] = []
            hysteresis = self.document["switching"]["voltage_hysteresis_V"]
            for index, switch in enumerate(self.document["switching"]["switches"]):
                command = supplied.get(_key(switch["pwm_port"]), 0)
                if command:
                    self.body_on[index] = False
                    paths.append(MosfetPath.CHANNEL.value)
                    continue
                vsd = (
                    self.signals[switch["source_signal_index"]]
                    - self.signals[switch["drain_signal_index"]]
                )
                threshold = switch["body_forward_threshold_V"]
                self.body_on[index] = vsd >= threshold + (
                    -hysteresis if self.body_on[index] else hysteresis
                )
                paths.append(
                    MosfetPath.BODY_DIODE.value if self.body_on[index] else MosfetPath.OFF.value
                )
            if not self.multi_mosfet_diode:
                return tuple(paths)
            assert isinstance(self.diode_on, list)
            for index, diode in enumerate(self.document["switching"]["diodes"]):
                vak = (
                    self.signals[diode["anode_signal_index"]]
                    - self.signals[diode["cathode_signal_index"]]
                )
                threshold = diode["forward_threshold_V"]
                self.diode_on[index] = vak >= threshold + (
                    -hysteresis if self.diode_on[index] else hysteresis
                )
            return tuple(paths), tuple(self.diode_on)
        indices = self.document["signals"]["switch_terminal_indices"]
        vak = self.signals[indices["diode_anode"]] - self.signals[indices["diode_cathode"]]
        vsd = self.signals[indices["mosfet_source"]] - self.signals[indices["mosfet_drain"]]
        switching = self.document["switching"]
        hysteresis = switching["voltage_hysteresis_V"]
        diode_threshold = switching["diode_forward_threshold_V"]
        body_threshold = switching["body_forward_threshold_V"]
        self.diode_on = vak >= diode_threshold + (-hysteresis if self.diode_on else hysteresis)
        if int(pwm) != 0:
            self.body_on = False
            return self.diode_on, MosfetPath.CHANNEL.value
        self.body_on = vsd >= body_threshold + (-hysteresis if self.body_on else hysteresis)
        return self.diode_on, MosfetPath.BODY_DIODE.value if self.body_on else MosfetPath.OFF.value

    def step(
        self,
        pwm: int | Mapping[str, int],
        inputs: Mapping[str, float] | None = None,
        profile: str = "normal",
    ) -> dict[str, float]:
        if profile not in {"normal", "short"}:
            raise ValueError("profile must be 'normal' or 'short'")
        supplied = {_key(name): float(value) for name, value in (inputs or {}).items()}
        known = {_key(port["name"]) for port in self.external_inputs}
        unknown = set(supplied) - known
        if unknown:
            raise ValueError("unknown input(s): " + ", ".join(sorted(unknown)))
        u = np.asarray([supplied.get(_key(port["name"]), port["default"]) for port in self.external_inputs])
        key = self._select(pwm)
        topology = self.topologies[key]
        item = topology["discrete"][profile]
        self.x = np.asarray(item["A"]) @ self.x + np.asarray(item["B"]) @ u + np.asarray(item["bias"])
        self.signals = np.asarray(item["C"]) @ self.x + np.asarray(item["D"]) @ u + np.asarray(item["output_bias"])
        self.last_topology = topology["name"]
        return {
            port["name"]: float(self.signals[port["signal_index"]])
            for port in self.document["ports"]["outputs"]
        }

    def step_short(self, pwm: int | Mapping[str, int], inputs: Mapping[str, float] | None = None) -> dict[str, float]:
        return self.step(pwm, inputs, "short")

    def step_normal(self, pwm: int | Mapping[str, int], inputs: Mapping[str, float] | None = None) -> dict[str, float]:
        return self.step(pwm, inputs, "normal")


def simulate_circuit_data(
    document: Mapping,
    duration_s: float,
    pwm_frequency_hz: float = 10_000.0,
    pwm_duty: float = 0.5,
    inputs: Mapping[str, float | Callable[[float], float]] | None = None,
    pwm_inputs: Mapping[str, int | Callable[[float], int]] | None = None,
    startup_short_steps: int = 0,
) -> DataSimulationResult:
    if duration_s < 0 or pwm_frequency_hz <= 0 or not 0.0 <= pwm_duty <= 1.0 or startup_short_steps < 0:
        raise ValueError("invalid simulation duration, PWM, duty, or startup step count")
    simulator = CircuitDataSimulator(document)
    normal_dt = float(document["solver"]["normal_step_s"])
    short_dt = float(document["solver"]["short_step_s"])
    steps = int(math.ceil(duration_s / normal_dt))
    times = np.arange(steps + 1, dtype=float) * normal_dt
    states = np.zeros((steps + 1, len(simulator.x)))
    output_ports = document["ports"]["outputs"]
    output_names = [port["name"] for port in output_ports]
    outputs = np.zeros((steps + 1, len(output_ports)))
    pwm_count = len(simulator.pwm_ports)
    pwm_values = np.zeros(
        (steps + 1, pwm_count) if pwm_count > 1 else (steps + 1,), dtype=np.uint32
    )
    topology = [simulator.last_topology]
    supplied = inputs or {}

    def gate(time_value: float) -> int | dict[str, int]:
        if pwm_inputs is not None:
            return {
                name: int(value(time_value) if callable(value) else value)
                for name, value in pwm_inputs.items()
            }
        if pwm_duty in {0.0, 1.0}:
            value = int(pwm_duty)
        else:
            value = int((time_value % (1.0 / pwm_frequency_hz)) < pwm_duty / pwm_frequency_hz)
        if pwm_count > 1:
            return {port["name"]: value for port in simulator.pwm_ports}
        return value

    def sampled_inputs(time_value: float) -> dict[str, float]:
        return {name: float(value(time_value) if callable(value) else value) for name, value in supplied.items()}

    elapsed = 0.0
    for _ in range(startup_short_steps):
        simulator.step_short(gate(elapsed), sampled_inputs(elapsed))
        elapsed += short_dt
    for index in range(steps):
        current_pwm = gate(float(times[index]))
        values = simulator.step_normal(current_pwm, sampled_inputs(float(times[index])))
        states[index + 1] = simulator.x
        outputs[index + 1] = [values[name] for name in output_names]
        if pwm_count > 1:
            assert isinstance(current_pwm, Mapping)
            pwm_values[index + 1] = [
                int(current_pwm.get(port["name"], 0)) for port in simulator.pwm_ports
            ]
        else:
            pwm_values[index + 1] = (
                int(current_pwm.get(simulator.pwm_ports[0]["name"], 0))
                if isinstance(current_pwm, Mapping)
                else int(current_pwm)
            )
        topology.append(simulator.last_topology)
    return DataSimulationResult(times, pwm_values, topology, states, outputs, output_names)


def _number(text: str) -> float:
    result = parse_spice_value(text)
    if result is None:
        raise argparse.ArgumentTypeError(f"expected a numeric/SPICE value, got {text!r}")
    return result


def _assignments(items: Sequence[str]) -> dict[str, float]:
    result: dict[str, float] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"expected NAME=VALUE, got {item!r}")
        name, raw = item.split("=", 1)
        value = parse_spice_value(raw)
        if not name or value is None:
            raise ValueError(f"expected numeric NAME=VALUE, got {item!r}")
        result[name] = value
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export and simulate portable GMP circuit data")
    commands = parser.add_subparsers(dest="command", required=True)
    export = commands.add_parser("export")
    export.add_argument("netlist")
    export.add_argument("output")
    export.add_argument("--normal-dt", type=_number, default=100e-9)
    export.add_argument("--short-dt", type=_number, default=0.5e-9)
    export.add_argument("--method", default="backward_euler")
    export.add_argument(
        "--matrix-tolerance", type=_number, default=DEFAULT_MATRIX_TOLERANCE
    )
    export.add_argument("--no-progress", action="store_true")
    export.add_argument("--device-param", action="append", default=[], metavar="NAME=VALUE")
    simulate = commands.add_parser("simulate")
    simulate.add_argument("data")
    simulate.add_argument("--duration", type=_number, required=True)
    simulate.add_argument("--pwm-frequency", type=_number, default=10_000.0)
    simulate.add_argument("--duty", type=float, default=0.5)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "export":
            circuit = parse_netlist(args.netlist)
            topology_count = piecewise_topology_count(circuit)
            print(f"circuit:             {circuit.title}")
            print(f"logical states:      {topology_count}")
            print(f"discretization:      {args.method}")
            print(f"normal step:         {args.normal_dt:.12g} s")
            print(f"short step:          {args.short_dt:.12g} s")
            print(f"matrix tolerance:    {args.matrix_tolerance:.12g}")
            build_bar = None if args.no_progress else TimedProgressBar("Building MNA states", topology_count)
            model = build_piecewise_model(
                circuit,
                _assignments(args.device_param),
                None if build_bar is None else build_bar.update,
            )
            if build_bar is not None:
                build_bar.finish()
            export_bar = None if args.no_progress else TimedProgressBar("Discretizing states", topology_count)
            document = build_circuit_data(
                model,
                source_path=args.netlist,
                normal_step_s=args.normal_dt,
                short_step_s=args.short_dt,
                method=args.method,
                matrix_tolerance=args.matrix_tolerance,
                progress=None if export_bar is None else export_bar.update,
            )
            if export_bar is not None:
                export_bar.finish()
            write_circuit_data(args.output, document)
            print(f"wrote {len(document['topologies'])} logical states to {args.output}")
        else:
            document = load_circuit_data(args.data)
            result = simulate_circuit_data(document, args.duration, args.pwm_frequency, args.duty)
            print(f"simulated {len(result.time) - 1} normal steps")
            for index, name in enumerate(result.output_names):
                print(f"{name}: final={result.outputs[-1, index]:.12g}")
    except (NetlistError, ValueError, OSError, np.linalg.LinAlgError) as error:
        print(f"error: {error}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
