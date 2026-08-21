"""Piecewise-linear diode/MOSFET simulation built on the MNA solver.

The switch network is expanded into linear topologies. Terminal voltages from
the previous sample select diode/body-diode states, while MOSFET channels and
TINA VSWITCH devices are controlled by external Boolean signals.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Mapping, Sequence

import numpy as np

from mna_solver import (
    Circuit,
    DescriptorModel,
    DeviceModel,
    DiscreteModel,
    Element,
    NetlistError,
    Observation,
    StateSpaceModel,
    assemble_mna,
    assemble_outputs,
    discretize,
    discrete_equations,
    parse_netlist,
    parse_spice_value,
    reduce_to_state_space,
)


class MosfetPath(str, Enum):
    OFF = "off"
    CHANNEL = "channel"
    BODY_DIODE = "body_diode"


@dataclass(frozen=True)
class SwitchParameters:
    diode_name: str
    diode_forward_voltage: float
    diode_on_resistance: float
    diode_off_resistance: float
    diode_junction_capacitance: float
    mosfet_name: str
    mosfet_on_resistance: float
    mosfet_off_resistance: float
    mosfet_output_capacitance: float
    body_forward_voltage: float
    body_on_resistance: float
    gate_drive_voltage: float


@dataclass(frozen=True)
class MosfetSwitchParameters:
    instance_name: str
    public_name: str
    on_resistance: float
    off_resistance: float
    output_capacitance: float
    body_forward_voltage: float
    body_on_resistance: float
    gate_drive_voltage: float


@dataclass(frozen=True)
class DiodeSwitchParameters:
    instance_name: str
    forward_voltage: float
    on_resistance: float
    off_resistance: float
    junction_capacitance: float


@dataclass(frozen=True)
class ControlledSwitchParameters:
    instance_name: str
    on_resistance: float
    off_resistance: float


@dataclass(frozen=True)
class TopologyKey:
    diode_on: bool
    mosfet_path: MosfetPath

    @property
    def name(self) -> str:
        diode = "D_ON" if self.diode_on else "D_OFF"
        return f"{diode}__MOS_{self.mosfet_path.value.upper()}"


@dataclass(frozen=True)
class MultiMosfetTopologyKey:
    paths: tuple[MosfetPath, ...]

    @property
    def name(self) -> str:
        return "__".join(
            f"MOS{index + 1}_{path.value.upper()}" for index, path in enumerate(self.paths)
        )


@dataclass(frozen=True)
class MultiMosfetDiodeTopologyKey:
    paths: tuple[MosfetPath, ...]
    diode_states: tuple[bool, ...]
    mosfet_names: tuple[str, ...] = ()
    diode_names: tuple[str, ...] = ()

    @property
    def name(self) -> str:
        mosfet_parts = [
            f"{(self.mosfet_names[index] if self.mosfet_names else f'MOS{index + 1}')}_{path.value.upper()}"
            for index, path in enumerate(self.paths)
        ]
        diode_parts = [
            f"{(self.diode_names[index] if self.diode_names else f'DIODE{index + 1}')}_{'ON' if state else 'OFF'}"
            for index, state in enumerate(self.diode_states)
        ]
        return "__".join([*mosfet_parts, *diode_parts])


@dataclass(frozen=True)
class MultiDiodeSwitchTopologyKey:
    diode_states: tuple[bool, ...]
    switch_states: tuple[bool, ...]
    diode_names: tuple[str, ...] = ()
    switch_names: tuple[str, ...] = ()

    @property
    def name(self) -> str:
        diode_parts = [
            f"{(self.diode_names[index] if self.diode_names else f'DIODE{index + 1}')}_{'ON' if state else 'OFF'}"
            for index, state in enumerate(self.diode_states)
        ]
        switch_parts = [
            f"{(self.switch_names[index] if self.switch_names else f'SWITCH{index + 1}')}_{'ON' if state else 'OFF'}"
            for index, state in enumerate(self.switch_states)
        ]
        return "__".join([*diode_parts, *switch_parts])


@dataclass
class LinearTopology:
    key: TopologyKey | MultiMosfetTopologyKey | MultiMosfetDiodeTopologyKey | MultiDiodeSwitchTopologyKey
    circuit: Circuit
    descriptor: DescriptorModel
    state: StateSpaceModel
    discrete: DiscreteModel | None = None


@dataclass
class PiecewiseLinearModel:
    source_circuit: Circuit
    diode: Element
    mosfet: Element
    gate_source_name: str
    parameters: SwitchParameters
    topologies: dict[TopologyKey, LinearTopology]

    @property
    def primary_topologies(self) -> dict[TopologyKey, LinearTopology]:
        """The requested D on/off x commanded MOS channel on/off four modes."""

        return {
            key: value
            for key, value in self.topologies.items()
            if key.mosfet_path in {MosfetPath.OFF, MosfetPath.CHANNEL}
        }


@dataclass
class MultiMosfetLinearModel:
    source_circuit: Circuit
    mosfets: list[Element]
    gate_sources: list[Element]
    parameters: list[MosfetSwitchParameters]
    topologies: dict[MultiMosfetTopologyKey, LinearTopology]


@dataclass
class MultiMosfetDiodeLinearModel:
    source_circuit: Circuit
    mosfets: list[Element]
    gate_sources: list[Element]
    parameters: list[MosfetSwitchParameters]
    diodes: list[Element]
    diode_parameters: list[DiodeSwitchParameters]
    topologies: dict[MultiMosfetDiodeTopologyKey, LinearTopology]


@dataclass
class MultiDiodeSwitchLinearModel:
    source_circuit: Circuit
    diodes: list[Element]
    switches: list[Element]
    control_sources: list[Element]
    diode_parameters: list[DiodeSwitchParameters]
    switch_parameters: list[ControlledSwitchParameters]
    topologies: dict[MultiDiodeSwitchTopologyKey, LinearTopology]


@dataclass
class SwitchingSimulationResult:
    time: np.ndarray
    gate: np.ndarray
    diode_on: np.ndarray
    body_diode_on: np.ndarray
    topology: list[str]
    states: np.ndarray
    outputs: np.ndarray
    state_names: list[str]
    output_names: list[str]


InputValue = float | Callable[[float], float]
ProgressCallback = Callable[[int, int], None]


def _key(name: str) -> str:
    return name.upper()


def ideal_source_nodes_requiring_capacitance_suppression(
    elements: Sequence[Element], excluded_sources: Sequence[Element] = ()
) -> set[str]:
    """Find ideal-source terminals that cannot accept parasitic C in xdot=Ax+Bu."""

    excluded = set(excluded_sources)
    adjacency: dict[str, set[str]] = {}
    for element in elements:
        if element.kind != "V" or element in excluded:
            continue
        positive, negative = (_key(node) for node in element.nodes[:2])
        adjacency.setdefault(positive, set()).add(negative)
        adjacency.setdefault(negative, set()).add(positive)
    result: set[str] = set()
    visited: set[str] = set()
    for start in adjacency:
        if start in visited:
            continue
        component = {start}
        pending = [start]
        visited.add(start)
        while pending:
            node = pending.pop()
            for neighbor in adjacency.get(node, ()):
                if neighbor not in visited:
                    visited.add(neighbor)
                    component.add(neighbor)
                    pending.append(neighbor)
        grounded = bool(component & {"0", "GND"})
        resistively_anchored = any(
            element.kind == "R"
            and ((_key(element.nodes[0]) in component) != (_key(element.nodes[1]) in component))
            for element in elements
        )
        if grounded or not resistively_anchored:
            result.update(component - {"0", "GND"})
    return result


def _model_for(circuit: Circuit, element: Element) -> DeviceModel:
    if element.model_name is None or _key(element.model_name) not in circuit.models:
        raise NetlistError(f"{element.name}: model {element.model_name!r} was not defined in the netlist")
    return circuit.models[_key(element.model_name)]


def _thermal_forward_voltage(model: DeviceModel, reference_current: float = 1.0) -> float:
    saturation_current = max(model.numeric("IS", 1e-14) or 1e-14, 1e-30)
    emission = model.numeric("N", 1.0) or 1.0
    return max(0.0, emission * 0.025851999786 * math.log(reference_current / saturation_current + 1.0))


def linearized_junction_capacitance(
    cjo: float,
    junction_potential: float,
    grading_coefficient: float,
    forward_coefficient: float,
    bias_voltage: float = 0.0,
) -> float:
    """Return the SPICE depletion-capacitance tangent at a fixed bias.

    The generated PWL model intentionally keeps this value constant.  At the
    default zero-bias operating point this reduces to CJO, while retaining the
    VJ/M/FC continuation formula for an explicitly selected linearization bias.
    """

    if cjo <= 0.0:
        return 0.0
    vj = max(junction_potential, np.finfo(float).tiny)
    m = max(grading_coefficient, 0.0)
    fc = min(max(forward_coefficient, 0.0), 0.95)
    if bias_voltage <= fc * vj:
        return cjo * (1.0 - bias_voltage / vj) ** (-m)
    continuation = 1.0 - fc * (1.0 + m) + m * bias_voltage / vj
    return cjo * continuation / (1.0 - fc) ** (1.0 + m)


def _override(overrides: Mapping[str, float], name: str, default: float) -> float:
    lookup = {_key(item): float(value) for item, value in overrides.items()}
    return lookup.get(_key(name), default)


def derive_switch_parameters(
    circuit: Circuit,
    overrides: Mapping[str, float] | None = None,
) -> tuple[Element, Element, SwitchParameters]:
    """Derive a practical PWL approximation from TINA diode/NMOS models."""

    overrides = overrides or {}
    diodes = [element for element in circuit.elements if element.kind == "D"]
    mosfets = [element for element in circuit.elements if element.kind == "M"]
    if len(diodes) != 1 or len(mosfets) != 1:
        raise NetlistError(
            f"piecewise solver currently requires exactly one diode and one MOSFET; got {len(diodes)} and {len(mosfets)}"
        )
    diode, mosfet = diodes[0], mosfets[0]
    diode_model = _model_for(circuit, diode)
    mosfet_model = _model_for(circuit, mosfet)
    mosfet_label = mosfet.name[1:] if _key(mosfet.name).startswith("MT") else mosfet.name

    rd = mosfet_model.numeric("RD", 0.0) or 0.0
    rs = mosfet_model.numeric("RS", 0.0) or 0.0
    series_resistance = max(rd + rs, 1e-6)
    gate_drive = _override(overrides, f"{mosfet_label}.Vdrive", 10.0)
    diode_vj = max(diode_model.numeric("VJ", 0.7) or 0.7, 0.0)
    diode_cjo = max(diode_model.numeric("CJO", 0.0) or 0.0, 0.0)
    diode_m = diode_model.numeric("M", 0.5) or 0.5
    diode_fc = diode_model.numeric("FC", 0.5) or 0.5
    diode_cj_bias = _override(overrides, f"{diode.name}.CjBias", 0.0)
    diode_cj = linearized_junction_capacitance(
        diode_cjo, diode_vj, diode_m, diode_fc, diode_cj_bias
    )
    vto = mosfet_model.numeric("VTO", 0.0) or 0.0
    kp = mosfet_model.numeric("KP", 0.0) or 0.0
    width = mosfet_model.numeric("W", 1.0) or 1.0
    length = max(mosfet_model.numeric("L", 1.0) or 1.0, np.finfo(float).tiny)
    overdrive = max(gate_drive - vto, np.finfo(float).eps)
    channel_conductance = kp * width / length * overdrive
    extracted_ron = series_resistance + (1.0 / channel_conductance if channel_conductance > 0.0 else 0.0)
    estimated_coss = (mosfet_model.numeric("CBD", 0.0) or 0.0) + (mosfet_model.numeric("CGDO", 0.0) or 0.0)

    parameters = SwitchParameters(
        diode.name,
        _override(overrides, f"{diode.name}.Vf", diode_vj),
        _override(overrides, f"{diode.name}.Ron", max(diode_model.numeric("RS", 1e-3) or 1e-3, 1e-6)),
        _override(overrides, f"{diode.name}.Roff", 1e9),
        _override(overrides, f"{diode.name}.Cj", diode_cj),
        mosfet_label,
        _override(overrides, f"{mosfet_label}.Ron", max(extracted_ron, 1e-6)),
        _override(overrides, f"{mosfet_label}.Roff", max(mosfet_model.numeric("RDS", 1e9) or 1e9, 1.0)),
        _override(overrides, f"{mosfet_label}.Coss", max(estimated_coss, 0.0)),
        _override(overrides, f"{mosfet_label}.BodyVf", max(mosfet_model.numeric("PB", 0.7) or 0.7, 0.0)),
        _override(overrides, f"{mosfet_label}.BodyRon", series_resistance),
        gate_drive,
    )
    return diode, mosfet, parameters


def derive_mosfet_parameters(
    circuit: Circuit,
    mosfet: Element,
    overrides: Mapping[str, float] | None = None,
) -> MosfetSwitchParameters:
    """Derive the three-path PWL model for one NMOS instance."""

    selected = overrides or {}
    model = _model_for(circuit, mosfet)
    public_name = mosfet.name[1:] if _key(mosfet.name).startswith("MT") else mosfet.name
    rd = model.numeric("RD", 0.0) or 0.0
    rs = model.numeric("RS", 0.0) or 0.0
    series_resistance = max(rd + rs, 1e-6)
    gate_drive = _override(selected, f"{public_name}.Vdrive", 10.0)
    vto = model.numeric("VTO", 0.0) or 0.0
    kp = model.numeric("KP", 0.0) or 0.0
    width = model.numeric("W", 1.0) or 1.0
    length = max(model.numeric("L", 1.0) or 1.0, np.finfo(float).tiny)
    overdrive = max(gate_drive - vto, np.finfo(float).eps)
    channel_conductance = kp * width / length * overdrive
    extracted_ron = series_resistance + (
        1.0 / channel_conductance if channel_conductance > 0.0 else 0.0
    )
    estimated_coss = (model.numeric("CBD", 0.0) or 0.0) + (
        model.numeric("CGDO", 0.0) or 0.0
    )
    return MosfetSwitchParameters(
        instance_name=mosfet.name,
        public_name=public_name,
        on_resistance=_override(selected, f"{public_name}.Ron", max(extracted_ron, 1e-6)),
        off_resistance=_override(
            selected, f"{public_name}.Roff", max(model.numeric("RDS", 1e9) or 1e9, 1.0)
        ),
        output_capacitance=_override(selected, f"{public_name}.Coss", max(estimated_coss, 0.0)),
        body_forward_voltage=_override(
            selected, f"{public_name}.BodyVf", max(model.numeric("PB", 0.7) or 0.7, 0.0)
        ),
        body_on_resistance=_override(selected, f"{public_name}.BodyRon", series_resistance),
        gate_drive_voltage=gate_drive,
    )


def derive_diode_parameters(
    circuit: Circuit,
    diode: Element,
    overrides: Mapping[str, float] | None = None,
) -> DiodeSwitchParameters:
    """Derive the two-path PWL model for one independent diode."""

    selected = overrides or {}
    model = _model_for(circuit, diode)
    vj = max(model.numeric("VJ", 0.7) or 0.7, 0.0)
    cjo = max(model.numeric("CJO", 0.0) or 0.0, 0.0)
    grading = model.numeric("M", 0.5) or 0.5
    forward_coefficient = model.numeric("FC", 0.5) or 0.5
    bias = _override(selected, f"{diode.name}.CjBias", 0.0)
    junction_capacitance = linearized_junction_capacitance(
        cjo, vj, grading, forward_coefficient, bias
    )
    return DiodeSwitchParameters(
        instance_name=diode.name,
        forward_voltage=_override(selected, f"{diode.name}.Vf", vj),
        on_resistance=_override(
            selected, f"{diode.name}.Ron", max(model.numeric("RS", 1e-3) or 1e-3, 1e-6)
        ),
        off_resistance=_override(selected, f"{diode.name}.Roff", 1e9),
        junction_capacitance=_override(selected, f"{diode.name}.Cj", junction_capacitance),
    )


def derive_controlled_switch_parameters(
    circuit: Circuit,
    switch: Element,
    overrides: Mapping[str, float] | None = None,
) -> ControlledSwitchParameters:
    """Derive Boolean-commanded RON/ROFF values from a VSWITCH model."""

    selected = overrides or {}
    model = _model_for(circuit, switch)
    if _key(model.kind) != "VSWITCH":
        raise NetlistError(
            f"{switch.name}: expected a VSWITCH model, got {model.kind!r}"
        )
    raw_ron = model.numeric("RON", 0.0)
    raw_roff = model.numeric("ROFF", 1e9)
    return ControlledSwitchParameters(
        instance_name=switch.name,
        on_resistance=_override(
            selected, f"{switch.name}.Ron", max(0.0 if raw_ron is None else raw_ron, 1e-3)
        ),
        off_resistance=_override(
            selected, f"{switch.name}.Roff", max(1e9 if raw_roff is None else raw_roff, 1.0)
        ),
    )


def _nodes(elements: Sequence[Element]) -> list[str]:
    result: dict[str, str] = {}
    for element in elements:
        for node in element.nodes:
            if _key(node) not in {"0", "GND"}:
                result.setdefault(_key(node), node)
    return list(result.values())


def _gate_source(circuit: Circuit, mosfet: Element) -> Element:
    gate_node = mosfet.nodes[1]
    candidates = [
        element
        for element in circuit.elements
        if element.kind == "V"
        and _key(element.nodes[0]) == _key(gate_node)
    ]
    if len(candidates) != 1:
        raise NetlistError(
            f"{mosfet.name}: expected one voltage source whose positive terminal is gate {gate_node}, "
            f"got {len(candidates)}"
        )
    return candidates[0]


def _controlled_switch_source(circuit: Circuit, switch: Element) -> Element:
    control_node = switch.nodes[2]
    candidates = [
        element
        for element in circuit.elements
        if element.kind == "V" and _key(element.nodes[0]) == _key(control_node)
    ]
    if len(candidates) != 1:
        raise NetlistError(
            f"{switch.name}: expected one voltage source whose positive terminal is control node "
            f"{control_node}, got {len(candidates)}"
        )
    return candidates[0]


def _expanded_circuit(
    circuit: Circuit,
    diode: Element,
    mosfet: Element,
    gate_source: Element,
    parameters: SwitchParameters,
    key: TopologyKey,
) -> Circuit:
    elements = [
        element
        for element in circuit.elements
        if element not in {diode, mosfet, gate_source}
    ]

    diode_internal = f"__{diode.name}_series"
    elements.extend(
        [
            Element(f"C__{diode.name}_junction", "C", diode.nodes[:2], f"{parameters.diode_junction_capacitance:.17g}"),
            Element(
                f"V__{diode.name}_drop",
                "V",
                (diode.nodes[0], diode_internal),
                f"{parameters.diode_forward_voltage if key.diode_on else 0.0:.17g}",
            ),
            Element(
                f"R__{diode.name}_path",
                "R",
                (diode_internal, diode.nodes[1]),
                f"{parameters.diode_on_resistance if key.diode_on else parameters.diode_off_resistance:.17g}",
            ),
        ]
    )

    drain, _, source, _ = mosfet.nodes
    mosfet_internal = f"__{parameters.mosfet_name}_series"
    ground_nodes = {"0", "GND"}

    def is_dc_constrained(node: str) -> bool:
        node_key = _key(node)
        return node_key in ground_nodes or any(
            element.kind == "V"
            and (
                (_key(element.nodes[0]) == node_key and _key(element.nodes[1]) in ground_nodes)
                or (_key(element.nodes[1]) == node_key and _key(element.nodes[0]) in ground_nodes)
            )
            for element in elements
        )

    drain_is_dc_constrained = is_dc_constrained(drain)
    source_is_dc_constrained = is_dc_constrained(source)
    # A capacitor tied to an ideal voltage-source node introduces u_dot into
    # the raw descriptor model.  Switching simulation treats the supply as DC,
    # so its small-signal derivative is zero and Coss can use ground as the
    # dynamic reference without changing the drain-voltage trajectory.
    if source_is_dc_constrained and not drain_is_dc_constrained:
        coss_nodes = (drain, "0")
    elif drain_is_dc_constrained and not source_is_dc_constrained:
        coss_nodes = (source, "0")
    else:
        coss_nodes = (drain, source)
    if key.mosfet_path == MosfetPath.CHANNEL:
        mosfet_resistance = parameters.mosfet_on_resistance
        body_drop = 0.0
    elif key.mosfet_path == MosfetPath.BODY_DIODE:
        mosfet_resistance = parameters.body_on_resistance
        body_drop = parameters.body_forward_voltage
    else:
        mosfet_resistance = parameters.mosfet_off_resistance
        body_drop = 0.0
    elements.extend(
        [
            Element(
                f"C__{parameters.mosfet_name}_oss",
                "C",
                coss_nodes,
                f"{parameters.mosfet_output_capacitance:.17g}",
            ),
            # The body diode points from source (anode) to drain (cathode).
            # A zero drop makes the same branch a symmetric channel/Roff path.
            Element(
                f"V__{parameters.mosfet_name}_drop",
                "V",
                (source, mosfet_internal),
                f"{body_drop:.17g}",
            ),
            Element(
                f"R__{parameters.mosfet_name}_path",
                "R",
                (mosfet_internal, drain),
                f"{mosfet_resistance:.17g}",
            ),
        ]
    )

    observations = list(circuit.observations)
    terminal_observations = [
        Observation("V", drain, label=f"V({parameters.mosfet_name}.D)"),
        Observation("V", source, label=f"V({parameters.mosfet_name}.S)"),
        Observation("V", diode.nodes[0], label=f"V({diode.name}.A)"),
        Observation("V", diode.nodes[1], label=f"V({diode.name}.K)"),
    ]
    existing = {_key(item.name) for item in observations}
    observations.extend(item for item in terminal_observations if _key(item.name) not in existing)
    return Circuit(
        title=f"{circuit.title} [{key.name}]",
        elements=elements,
        observations=observations,
        nodes=_nodes(elements),
        models=circuit.models,
        ignored_directives=circuit.ignored_directives,
    )


def _expanded_multi_mosfet_circuit(
    circuit: Circuit,
    mosfets: Sequence[Element],
    gate_sources: Sequence[Element],
    parameters: Sequence[MosfetSwitchParameters],
    key: MultiMosfetTopologyKey,
) -> Circuit:
    """Replace every MOSFET by its selected R/C/V PWL path."""

    removed = set(mosfets) | set(gate_sources)
    elements = [element for element in circuit.elements if element not in removed]
    ground_nodes = {"0", "GND"}

    def is_dc_constrained(node: str) -> bool:
        node_key = _key(node)
        return node_key in ground_nodes or any(
            element.kind == "V"
            and (
                (_key(element.nodes[0]) == node_key and _key(element.nodes[1]) in ground_nodes)
                or (_key(element.nodes[1]) == node_key and _key(element.nodes[0]) in ground_nodes)
            )
            for element in elements
        )

    for mosfet, item, path in zip(mosfets, parameters, key.paths):
        drain, _, source, _ = mosfet.nodes
        internal = f"__{item.public_name}_series"
        drain_constrained = is_dc_constrained(drain)
        source_constrained = is_dc_constrained(source)
        if source_constrained and not drain_constrained:
            coss_nodes = (drain, "0")
        elif drain_constrained and not source_constrained:
            coss_nodes = (source, "0")
        else:
            coss_nodes = (drain, source)
        if path == MosfetPath.CHANNEL:
            resistance = item.on_resistance
            body_drop = 0.0
        elif path == MosfetPath.BODY_DIODE:
            resistance = item.body_on_resistance
            body_drop = item.body_forward_voltage
        else:
            resistance = item.off_resistance
            body_drop = 0.0
        elements.extend(
            [
                Element(
                    f"C__{item.public_name}_oss",
                    "C",
                    coss_nodes,
                    f"{item.output_capacitance:.17g}",
                ),
                Element(
                    f"V__{item.public_name}_drop",
                    "V",
                    (source, internal),
                    f"{body_drop:.17g}",
                ),
                Element(
                    f"R__{item.public_name}_path",
                    "R",
                    (internal, drain),
                    f"{resistance:.17g}",
                ),
            ]
        )

    observations = list(circuit.observations)
    existing = {_key(item.name) for item in observations}
    for mosfet, item in zip(mosfets, parameters):
        drain, _, source, _ = mosfet.nodes
        for observation in (
            Observation("V", drain, label=f"V({item.public_name}.D)"),
            Observation("V", source, label=f"V({item.public_name}.S)"),
        ):
            if _key(observation.name) not in existing:
                observations.append(observation)
                existing.add(_key(observation.name))
    return Circuit(
        title=f"{circuit.title} [{key.name}]",
        elements=elements,
        observations=observations,
        nodes=_nodes(elements),
        models=circuit.models,
        ignored_directives=circuit.ignored_directives,
    )


def build_multi_mosfet_model(
    circuit: Circuit,
    device_overrides: Mapping[str, float] | None = None,
    progress: ProgressCallback | None = None,
) -> MultiMosfetLinearModel:
    """Build all 3**N channel/off/body-diode topologies for an NMOS network."""

    mosfets = [element for element in circuit.elements if element.kind == "M"]
    diodes = [element for element in circuit.elements if element.kind == "D"]
    if not mosfets or diodes:
        raise NetlistError(
            "multi-MOS mode requires one or more MOSFETs and no independent diode devices"
        )
    gate_sources = [_gate_source(circuit, mosfet) for mosfet in mosfets]
    if len({_key(source.name) for source in gate_sources}) != len(gate_sources):
        raise NetlistError("each MOSFET must have a distinct gate voltage source")
    parameters = [
        derive_mosfet_parameters(circuit, mosfet, device_overrides) for mosfet in mosfets
    ]
    topologies: dict[MultiMosfetTopologyKey, LinearTopology] = {}
    choices = (MosfetPath.OFF, MosfetPath.CHANNEL, MosfetPath.BODY_DIODE)
    topology_count = 3 ** len(mosfets)
    for topology_index, paths in enumerate(
        itertools.product(choices, repeat=len(mosfets)), start=1
    ):
        key = MultiMosfetTopologyKey(paths)
        expanded = _expanded_multi_mosfet_circuit(
            circuit, mosfets, gate_sources, parameters, key
        )
        descriptor = assemble_mna(expanded)
        outputs = assemble_outputs(expanded, descriptor)
        try:
            state = reduce_to_state_space(descriptor, outputs)
        except NetlistError as error:
            raise NetlistError(f"{key.name}: {error}") from error
        topologies[key] = LinearTopology(key, expanded, descriptor, state)
        if progress is not None:
            progress(topology_index, topology_count)

    reference = next(iter(topologies.values())).state
    for topology in topologies.values():
        if (
            topology.state.state_names != reference.state_names
            or topology.state.input_names != reference.input_names
            or topology.state.output_names != reference.output_names
        ):
            raise NetlistError("multi-MOS expansion produced incompatible topology coordinates")
    return MultiMosfetLinearModel(
        circuit, mosfets, gate_sources, parameters, topologies
    )


def _expanded_multi_mosfet_diode_circuit(
    circuit: Circuit,
    mosfets: Sequence[Element],
    gate_sources: Sequence[Element],
    mosfet_parameters: Sequence[MosfetSwitchParameters],
    diodes: Sequence[Element],
    diode_parameters: Sequence[DiodeSwitchParameters],
    key: MultiMosfetDiodeTopologyKey,
) -> Circuit:
    """Replace independent diodes, then expand every MOSFET selected path."""

    elements = [element for element in circuit.elements if element not in set(diodes)]
    ideal_source_nodes = ideal_source_nodes_requiring_capacitance_suppression(
        elements, gate_sources
    )
    for diode, item, diode_on in zip(diodes, diode_parameters, key.diode_states):
        internal = f"__{diode.name}_series"
        modeled_capacitance = (
            0.0
            if any(_key(node) in ideal_source_nodes for node in diode.nodes[:2])
            else item.junction_capacitance
        )
        elements.extend(
            [
                Element(
                    f"C__{diode.name}_junction",
                    "C",
                    diode.nodes[:2],
                    f"{modeled_capacitance:.17g}",
                ),
                Element(
                    f"V__{diode.name}_drop",
                    "V",
                    (diode.nodes[0], internal),
                    f"{item.forward_voltage if diode_on else 0.0:.17g}",
                ),
                Element(
                    f"R__{diode.name}_path",
                    "R",
                    (internal, diode.nodes[1]),
                    f"{item.on_resistance if diode_on else item.off_resistance:.17g}",
                ),
            ]
        )

    observations = list(circuit.observations)
    existing = {_key(item.name) for item in observations}
    for diode in diodes:
        for observation in (
            Observation("V", diode.nodes[0], label=f"V({diode.name}.A)"),
            Observation("V", diode.nodes[1], label=f"V({diode.name}.K)"),
        ):
            if _key(observation.name) not in existing:
                observations.append(observation)
                existing.add(_key(observation.name))
    diode_expanded = Circuit(
        title=circuit.title,
        elements=elements,
        observations=observations,
        nodes=_nodes(elements),
        models=circuit.models,
        ignored_directives=circuit.ignored_directives,
    )
    return _expanded_multi_mosfet_circuit(
        diode_expanded,
        mosfets,
        gate_sources,
        mosfet_parameters,
        key,
    )


def build_multi_mosfet_diode_model(
    circuit: Circuit,
    device_overrides: Mapping[str, float] | None = None,
    progress: ProgressCallback | None = None,
) -> MultiMosfetDiodeLinearModel:
    """Build all 3**M MOS paths and 2**D independent-diode states."""

    mosfets = [element for element in circuit.elements if element.kind == "M"]
    diodes = [element for element in circuit.elements if element.kind == "D"]
    switches = [element for element in circuit.elements if element.kind == "S"]
    if not mosfets or not diodes or switches:
        raise NetlistError(
            "mixed MOS/diode mode requires MOSFETs and independent diodes, but no VSWITCH devices"
        )
    gate_sources = [_gate_source(circuit, mosfet) for mosfet in mosfets]
    if len({_key(source.name) for source in gate_sources}) != len(gate_sources):
        raise NetlistError("each MOSFET must have a distinct gate voltage source")
    mosfet_parameters = [
        derive_mosfet_parameters(circuit, mosfet, device_overrides) for mosfet in mosfets
    ]
    diode_parameters = [
        derive_diode_parameters(circuit, diode, device_overrides) for diode in diodes
    ]
    topologies: dict[MultiMosfetDiodeTopologyKey, LinearTopology] = {}
    choices = (MosfetPath.OFF, MosfetPath.CHANNEL, MosfetPath.BODY_DIODE)
    topology_count = 3 ** len(mosfets) * 2 ** len(diodes)
    topology_index = 0
    for paths in itertools.product(choices, repeat=len(mosfets)):
        for diode_states in itertools.product((False, True), repeat=len(diodes)):
            topology_index += 1
            key = MultiMosfetDiodeTopologyKey(
                paths,
                diode_states,
                tuple(mosfet.name for mosfet in mosfets),
                tuple(diode.name for diode in diodes),
            )
            expanded = _expanded_multi_mosfet_diode_circuit(
                circuit,
                mosfets,
                gate_sources,
                mosfet_parameters,
                diodes,
                diode_parameters,
                key,
            )
            descriptor = assemble_mna(expanded)
            outputs = assemble_outputs(expanded, descriptor)
            try:
                state = reduce_to_state_space(descriptor, outputs)
            except NetlistError as error:
                raise NetlistError(f"{key.name}: {error}") from error
            topologies[key] = LinearTopology(key, expanded, descriptor, state)
            if progress is not None:
                progress(topology_index, topology_count)

    reference = next(iter(topologies.values())).state
    for topology in topologies.values():
        if (
            topology.state.state_names != reference.state_names
            or topology.state.input_names != reference.input_names
            or topology.state.output_names != reference.output_names
        ):
            raise NetlistError("mixed MOS/diode expansion produced incompatible topology coordinates")
    return MultiMosfetDiodeLinearModel(
        circuit,
        mosfets,
        gate_sources,
        mosfet_parameters,
        diodes,
        diode_parameters,
        topologies,
    )


def _expanded_multi_diode_switch_circuit(
    circuit: Circuit,
    diodes: Sequence[Element],
    switches: Sequence[Element],
    control_sources: Sequence[Element],
    diode_parameters: Sequence[DiodeSwitchParameters],
    switch_parameters: Sequence[ControlledSwitchParameters],
    key: MultiDiodeSwitchTopologyKey,
) -> Circuit:
    """Replace independent diodes and VSWITCH devices by selected linear paths."""

    removed = set(diodes) | set(switches) | set(control_sources)
    elements = [element for element in circuit.elements if element not in removed]
    ideal_source_nodes = ideal_source_nodes_requiring_capacitance_suppression(elements)
    for diode, item, diode_on in zip(diodes, diode_parameters, key.diode_states):
        internal = f"__{diode.name}_series"
        # A depletion capacitor connected directly to an ideal, time-varying
        # voltage-source terminal introduces u_dot and an index-2 descriptor.
        # The current state-space boundary has no input-derivative port, so omit
        # that parasitic in this exceptional case. The extracted CJO value is
        # still retained in circuit data for a future descriptor backend.
        modeled_capacitance = (
            0.0
            if any(_key(node) in ideal_source_nodes for node in diode.nodes[:2])
            else item.junction_capacitance
        )
        elements.extend(
            [
                Element(
                    f"C__{diode.name}_junction",
                    "C",
                    diode.nodes[:2],
                    f"{modeled_capacitance:.17g}",
                ),
                Element(
                    f"V__{diode.name}_drop",
                    "V",
                    (diode.nodes[0], internal),
                    f"{item.forward_voltage if diode_on else 0.0:.17g}",
                ),
                Element(
                    f"R__{diode.name}_path",
                    "R",
                    (internal, diode.nodes[1]),
                    f"{item.on_resistance if diode_on else item.off_resistance:.17g}",
                ),
            ]
        )
    for switch, item, switch_on in zip(switches, switch_parameters, key.switch_states):
        elements.append(
            Element(
                f"R__{switch.name}_path",
                "R",
                switch.nodes[:2],
                f"{item.on_resistance if switch_on else item.off_resistance:.17g}",
            )
        )

    observations = list(circuit.observations)
    existing = {_key(item.name) for item in observations}
    for diode in diodes:
        for observation in (
            Observation("V", diode.nodes[0], label=f"V({diode.name}.A)"),
            Observation("V", diode.nodes[1], label=f"V({diode.name}.K)"),
        ):
            if _key(observation.name) not in existing:
                observations.append(observation)
                existing.add(_key(observation.name))
    return Circuit(
        title=f"{circuit.title} [{key.name}]",
        elements=elements,
        observations=observations,
        nodes=_nodes(elements),
        models=circuit.models,
        ignored_directives=circuit.ignored_directives,
    )


def build_multi_diode_switch_model(
    circuit: Circuit,
    device_overrides: Mapping[str, float] | None = None,
    progress: ProgressCallback | None = None,
) -> MultiDiodeSwitchLinearModel:
    """Build every on/off combination for independent diodes and VSWITCH devices."""

    diodes = [element for element in circuit.elements if element.kind == "D"]
    switches = [element for element in circuit.elements if element.kind == "S"]
    mosfets = [element for element in circuit.elements if element.kind == "M"]
    if mosfets or not (diodes or switches):
        raise NetlistError(
            "multi-diode/switch mode requires at least one diode or VSWITCH and no MOSFETs"
        )
    control_sources = [_controlled_switch_source(circuit, switch) for switch in switches]
    if len({_key(source.name) for source in control_sources}) != len(control_sources):
        raise NetlistError("each controlled switch must have a distinct control voltage source")
    diode_parameters = [
        derive_diode_parameters(circuit, diode, device_overrides) for diode in diodes
    ]
    switch_parameters = [
        derive_controlled_switch_parameters(circuit, switch, device_overrides)
        for switch in switches
    ]
    topologies: dict[MultiDiodeSwitchTopologyKey, LinearTopology] = {}
    device_count = len(diodes) + len(switches)
    topology_count = 2**device_count
    for topology_index, states in enumerate(
        itertools.product((False, True), repeat=device_count), start=1
    ):
        key = MultiDiodeSwitchTopologyKey(
            tuple(states[: len(diodes)]),
            tuple(states[len(diodes) :]),
            tuple(diode.name for diode in diodes),
            tuple(switch.name for switch in switches),
        )
        expanded = _expanded_multi_diode_switch_circuit(
            circuit,
            diodes,
            switches,
            control_sources,
            diode_parameters,
            switch_parameters,
            key,
        )
        descriptor = assemble_mna(expanded)
        outputs = assemble_outputs(expanded, descriptor)
        state = reduce_to_state_space(descriptor, outputs)
        topologies[key] = LinearTopology(key, expanded, descriptor, state)
        if progress is not None:
            progress(topology_index, topology_count)

    reference = next(iter(topologies.values())).state
    for topology in topologies.values():
        if (
            topology.state.state_names != reference.state_names
            or topology.state.input_names != reference.input_names
            or topology.state.output_names != reference.output_names
        ):
            raise NetlistError(
                "multi-diode/switch expansion produced incompatible topology coordinates"
            )
    return MultiDiodeSwitchLinearModel(
        circuit,
        diodes,
        switches,
        control_sources,
        diode_parameters,
        switch_parameters,
        topologies,
    )


def piecewise_topology_count(circuit: Circuit) -> int:
    """Return the number of logical PWL states before constructing matrices."""

    diode_count = sum(element.kind == "D" for element in circuit.elements)
    mosfet_count = sum(element.kind == "M" for element in circuit.elements)
    switch_count = sum(element.kind == "S" for element in circuit.elements)
    if switch_count and mosfet_count:
        raise NetlistError("mixed MOSFET and VSWITCH expansion is not supported yet")
    if not mosfet_count and (diode_count or switch_count):
        return 2 ** (diode_count + switch_count)
    if not diode_count and mosfet_count:
        return 3**mosfet_count
    if mosfet_count and diode_count and (mosfet_count != 1 or diode_count != 1):
        return 3**mosfet_count * 2**diode_count
    return 6


def build_piecewise_model(
    circuit: Circuit,
    device_overrides: Mapping[str, float] | None = None,
    progress: ProgressCallback | None = None,
) -> PiecewiseLinearModel | MultiMosfetLinearModel | MultiMosfetDiodeLinearModel | MultiDiodeSwitchLinearModel:
    """Build the supported diode, MOSFET, and VSWITCH PWL topology family."""

    diodes = [element for element in circuit.elements if element.kind == "D"]
    mosfets = [element for element in circuit.elements if element.kind == "M"]
    switches = [element for element in circuit.elements if element.kind == "S"]
    if switches and mosfets:
        raise NetlistError("mixed MOSFET and VSWITCH expansion is not supported yet")
    if not mosfets and (diodes or switches):
        return build_multi_diode_switch_model(circuit, device_overrides, progress)
    if not diodes and mosfets:
        return build_multi_mosfet_model(circuit, device_overrides, progress)
    if mosfets and diodes and (len(mosfets) != 1 or len(diodes) != 1):
        return build_multi_mosfet_diode_model(circuit, device_overrides, progress)

    diode, mosfet, parameters = derive_switch_parameters(circuit, device_overrides)
    gate_source = _gate_source(circuit, mosfet)
    topologies: dict[TopologyKey, LinearTopology] = {}
    topology_index = 0
    for diode_on in (False, True):
        for path in (MosfetPath.OFF, MosfetPath.CHANNEL, MosfetPath.BODY_DIODE):
            topology_index += 1
            key = TopologyKey(diode_on, path)
            expanded = _expanded_circuit(circuit, diode, mosfet, gate_source, parameters, key)
            descriptor = assemble_mna(expanded)
            outputs = assemble_outputs(expanded, descriptor)
            state = reduce_to_state_space(descriptor, outputs)
            topologies[key] = LinearTopology(key, expanded, descriptor, state)
            if progress is not None:
                progress(topology_index, 6)

    reference = next(iter(topologies.values())).state
    for topology in topologies.values():
        if topology.state.state_names != reference.state_names or topology.state.input_names != reference.input_names:
            raise NetlistError("switch topology expansion produced incompatible state or input coordinates")
    return PiecewiseLinearModel(circuit, diode, mosfet, gate_source.name, parameters, topologies)


def prepare_discrete_topologies(
    model: PiecewiseLinearModel,
    dt: float,
    method: str = "backward_euler",
) -> None:
    for topology in model.topologies.values():
        topology.discrete = discretize(topology.state, dt, method)


def _sample(value: InputValue, time: float) -> float:
    return float(value(time)) if callable(value) else float(value)


def _input_vector(
    topology: LinearTopology,
    time: float,
    inputs: Mapping[str, InputValue],
    discrete: DiscreteModel | None = None,
) -> np.ndarray:
    selected = discrete or topology.discrete
    assert selected is not None
    lookup = {_key(name): value for name, value in inputs.items()}
    unknown = set(lookup) - {_key(name) for name in selected.input_names}
    if unknown:
        raise ValueError("unknown electrical input(s): " + ", ".join(sorted(unknown)))
    result = np.empty(len(selected.input_names))
    for index, name in enumerate(selected.input_names):
        if _key(name) in lookup:
            result[index] = _sample(lookup[_key(name)], time)
        else:
            default = selected.input_defaults[index]
            if default is None:
                raise ValueError(f"no numeric/default value for input {name}")
            result[index] = float(default)
    return result


def simulate_piecewise(
    model: PiecewiseLinearModel,
    duration: float,
    dt: float,
    pwm_frequency: float = 10_000.0,
    pwm_duty: float = 0.5,
    inputs: Mapping[str, InputValue] | None = None,
    initial_state: Sequence[float] | None = None,
    voltage_hysteresis: float = 1e-6,
    method: str = "backward_euler",
    transition_substep: float = 0.5e-9,
) -> SwitchingSimulationResult:
    """Simulate using previous-sample VAK/VSD to select the next topology."""

    if duration < 0 or dt <= 0 or transition_substep <= 0 or pwm_frequency <= 0 or not 0.0 <= pwm_duty <= 1.0:
        raise ValueError(
            "require duration >= 0, dt > 0, transition_substep > 0, pwm_frequency > 0, and 0 <= duty <= 1"
        )
    prepare_discrete_topologies(model, dt, method)
    fine_dt = min(dt, transition_substep)
    fine_discrete = {
        key: discretize(topology.state, fine_dt, method)
        for key, topology in model.topologies.items()
    }
    reference = next(iter(model.topologies.values())).state
    steps = int(math.ceil(duration / dt))
    times = np.arange(steps + 1, dtype=float) * dt
    X = np.zeros((steps + 1, len(reference.state_names)))
    if initial_state is not None:
        value = np.asarray(initial_state, dtype=float)
        if value.shape != X[0].shape:
            raise ValueError(f"initial state must have shape {X[0].shape}")
        X[0] = value
    Y = np.zeros((steps + 1, len(reference.output_names)))
    gate_values = np.zeros(steps + 1, dtype=bool)
    diode_values = np.zeros(steps + 1, dtype=bool)
    body_values = np.zeros(steps + 1, dtype=bool)
    topology_names: list[str] = []
    output_index = {_key(name): index for index, name in enumerate(reference.output_names)}
    p = model.parameters
    required = [f"V({p.mosfet_name}.D)", f"V({p.mosfet_name}.S)", f"V({model.diode.name}.A)", f"V({model.diode.name}.K)"]
    if any(_key(name) not in output_index for name in required):
        raise NetlistError("switch terminal observations are missing from topology output equations")

    previous_vak = 0.0
    previous_vsd = 0.0
    diode_on = False
    body_on = False
    supplied_inputs = inputs or {}
    period = 1.0 / pwm_frequency

    def gate_at(time_value: float) -> bool:
        if pwm_duty in {0.0, 1.0}:
            return pwm_duty == 1.0
        return (time_value % period) < pwm_duty * period

    def select_mode(
        gate_on: bool,
        vak: float,
        vsd: float,
        old_diode: bool,
        old_body: bool,
    ) -> tuple[bool, bool, MosfetPath]:
        new_diode = (
            vak >= p.diode_forward_voltage - voltage_hysteresis
            if old_diode
            else vak >= p.diode_forward_voltage + voltage_hysteresis
        )
        if gate_on:
            return new_diode, False, MosfetPath.CHANNEL
        new_body = (
            vsd >= p.body_forward_voltage - voltage_hysteresis
            if old_body
            else vsd >= p.body_forward_voltage + voltage_hysteresis
        )
        return new_diode, new_body, MosfetPath.BODY_DIODE if new_body else MosfetPath.OFF

    def terminal_differences(values: np.ndarray) -> tuple[float, float]:
        drain = values[output_index[_key(f"V({p.mosfet_name}.D)")]]
        source = values[output_index[_key(f"V({p.mosfet_name}.S)")]]
        anode = values[output_index[_key(f"V({model.diode.name}.A)")]]
        cathode = values[output_index[_key(f"V({model.diode.name}.K)")]]
        return anode - cathode, source - drain

    for step, time in enumerate(times):
        gate_on = gate_at(float(time))
        diode_on, body_on, path = select_mode(
            gate_on, previous_vak, previous_vsd, diode_on, body_on
        )
        key = TopologyKey(diode_on, path)
        topology = model.topologies[key]
        assert topology.discrete is not None
        u = _input_vector(topology, float(time), supplied_inputs)
        Y[step] = topology.discrete.C @ X[step] + topology.discrete.D @ u
        gate_values[step] = gate_on
        diode_values[step] = diode_on
        body_values[step] = body_on
        topology_names.append(key.name)
        if step < steps:
            if not diode_on and path == MosfetPath.OFF and fine_dt < dt:
                local_x = X[step].copy()
                local_vak, local_vsd = terminal_differences(Y[step])
                local_diode, local_body = diode_on, body_on
                substeps = int(math.ceil(dt / fine_dt))
                for substep in range(substeps):
                    elapsed = substep * fine_dt
                    remaining = dt - elapsed
                    if remaining <= max(dt * 1e-12, 1e-18):
                        break
                    sub_dt = min(fine_dt, remaining)
                    local_time = float(time) + elapsed
                    local_gate = gate_at(local_time)
                    local_diode, local_body, local_path = select_mode(
                        local_gate, local_vak, local_vsd, local_diode, local_body
                    )
                    local_key = TopologyKey(local_diode, local_path)
                    local_topology = model.topologies[local_key]
                    local_discrete = (
                        fine_discrete[local_key]
                        if math.isclose(sub_dt, fine_dt, rel_tol=1e-12, abs_tol=0.0)
                        else discretize(local_topology.state, sub_dt, method)
                    )
                    local_u = _input_vector(local_topology, local_time, supplied_inputs, local_discrete)
                    local_x = local_discrete.A @ local_x + local_discrete.B @ local_u
                    local_y = local_discrete.C @ local_x + local_discrete.D @ local_u
                    local_vak, local_vsd = terminal_differences(local_y)
                X[step + 1] = local_x
                previous_vak, previous_vsd = local_vak, local_vsd
                diode_on, body_on = local_diode, local_body
            else:
                X[step + 1] = topology.discrete.A @ X[step] + topology.discrete.B @ u
                previous_vak, previous_vsd = terminal_differences(Y[step])

    return SwitchingSimulationResult(
        times,
        gate_values,
        diode_values,
        body_values,
        topology_names,
        X,
        Y,
        reference.state_names,
        reference.output_names,
    )


def write_switching_csv(
    path: str | Path,
    result: SwitchingSimulationResult,
    stride: int = 1,
) -> None:
    if stride < 1:
        raise ValueError("output stride must be at least 1")
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["time_s", "gate", "diode_on", "body_diode_on", "topology", *result.state_names, *result.output_names]
        )
        indices = list(range(0, len(result.time), stride))
        if indices[-1] != len(result.time) - 1:
            indices.append(len(result.time) - 1)
        for index in indices:
            time = result.time[index]
            writer.writerow(
                [
                    time,
                    int(result.gate[index]),
                    int(result.diode_on[index]),
                    int(result.body_diode_on[index]),
                    result.topology[index],
                    *result.states[index],
                    *result.outputs[index],
                ]
            )


def _assignment(items: Sequence[str]) -> dict[str, float]:
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


def _number(text: str) -> float:
    value = parse_spice_value(text)
    if value is None:
        raise argparse.ArgumentTypeError(f"expected numeric/SPICE value, got {text!r}")
    return value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Piecewise-linear diode/MOSFET MNA solver")
    commands = parser.add_subparsers(dest="command", required=True)
    analyze = commands.add_parser("analyze")
    analyze.add_argument("netlist")
    analyze.add_argument("--device-param", action="append", default=[], metavar="NAME=VALUE")
    analyze.add_argument("--dt", type=_number)
    analyze.add_argument(
        "--method",
        choices=("forward_euler", "backward_euler", "rk4"),
        default="backward_euler",
    )
    simulate = commands.add_parser("simulate")
    simulate.add_argument("netlist")
    simulate.add_argument("--device-param", action="append", default=[], metavar="NAME=VALUE")
    simulate.add_argument("--input", action="append", default=[], metavar="NAME=VALUE")
    simulate.add_argument("--dt", type=_number, required=True)
    simulate.add_argument("--duration", type=_number, required=True)
    simulate.add_argument("--pwm-frequency", type=_number, default=10_000.0)
    simulate.add_argument("--duty", type=float, default=0.5)
    simulate.add_argument("--transition-substep", type=_number, default=0.5e-9)
    simulate.add_argument(
        "--method",
        choices=("forward_euler", "backward_euler", "rk4"),
        default="backward_euler",
    )
    simulate.add_argument("--output-stride", type=int, default=1)
    simulate.add_argument("--output", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        circuit = parse_netlist(args.netlist)
        model = build_piecewise_model(circuit, _assignment(args.device_param))
        if args.command == "analyze":
            print("Derived switch parameters:")
            if isinstance(model, MultiDiodeSwitchLinearModel):
                for item in model.diode_parameters:
                    print(f"  {item.instance_name}: {item}")
                for item in model.switch_parameters:
                    print(f"  {item.instance_name}: {item}")
            elif isinstance(model, MultiMosfetLinearModel):
                for item in model.parameters:
                    print(f"  {item.instance_name}: {item}")
            else:
                for name, value in vars(model.parameters).items():
                    print(f"  {name} = {value}")
                print(f"\nPrimary topology count: {len(model.primary_topologies)}")
            print(f"Physical topology count: {len(model.topologies)}")
            if args.dt is not None:
                prepare_discrete_topologies(model, args.dt, args.method)
            for key, topology in model.topologies.items():
                print(f"\n[{key.name}]")
                print("A =\n", topology.state.A, sep="")
                print("B =\n", topology.state.B, sep="")
                if topology.discrete is not None:
                    print("\n".join(discrete_equations(topology.discrete)))
        else:
            if not isinstance(model, PiecewiseLinearModel):
                raise NetlistError(
                    "direct switched_solver simulation currently supports the one-diode/one-MOSF "
                    "case; export circuit data for multi-device simulation"
                )
            result = simulate_piecewise(
                model,
                args.duration,
                args.dt,
                pwm_frequency=args.pwm_frequency,
                pwm_duty=args.duty,
                inputs=_assignment(args.input),
                method=args.method,
                transition_substep=args.transition_substep,
            )
            write_switching_csv(args.output, result, args.output_stride)
            print(f"wrote {len(result.time)} samples to {args.output}")
            for output_index, name in enumerate(result.output_names):
                print(f"{name}: min={np.min(result.outputs[:, output_index]):.9g}, max={np.max(result.outputs[:, output_index]):.9g}")
    except (NetlistError, ValueError, np.linalg.LinAlgError) as exc:
        print(f"error: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
