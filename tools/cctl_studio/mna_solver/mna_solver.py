"""Linear circuit analysis using descriptor Modified Nodal Analysis (MNA).

The module uses SymEngine for exact MNA construction and NumPy for numeric
reduction/simulation.  It parses the SPICE/TINA subset used by CCTL Studio,
builds ``E z_dot = A z + B u``, eliminates algebraic unknowns, and exposes
simulation and frequency-response helpers.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Mapping, Sequence

import numpy as np
import symengine as se


_GROUND_NAMES = {"0", "GND"}
_VALUE_RE = re.compile(
    r"^([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?)"
    r"(MEG|[TGMKUNPF]?)(?:[A-Za-z]*)$",
    re.IGNORECASE,
)
_SCALE = {
    "": 1.0,
    "T": 1e12,
    "G": 1e9,
    "MEG": 1e6,
    "K": 1e3,
    "M": 1e-3,  # SPICE uses M for milli and MEG for mega.
    "U": 1e-6,
    "N": 1e-9,
    "P": 1e-12,
    "F": 1e-15,
}


class NetlistError(ValueError):
    """A netlist cannot be parsed or represented by the supported subset."""


class UnresolvedParameterError(NetlistError):
    """Numeric analysis was requested before all element values were supplied."""

    def __init__(self, names: Iterable[str]):
        unique = sorted(set(names), key=str.upper)
        super().__init__("unresolved element parameter(s): " + ", ".join(unique))
        self.names = unique


def parse_spice_value(text: str) -> float | None:
    """Parse a SPICE number, returning ``None`` for a symbolic value."""

    match = _VALUE_RE.fullmatch(text.strip())
    if not match:
        return None
    return float(match.group(1)) * _SCALE[match.group(2).upper()]


def _key(name: str) -> str:
    return name.upper()


def _is_ground(name: str) -> bool:
    return _key(name.strip("[]")) in _GROUND_NAMES


def _clean_node(name: str) -> str:
    result = name.strip().strip("[]")
    return "0" if _is_ground(result) else result


@dataclass(frozen=True)
class Element:
    name: str
    kind: str
    nodes: tuple[str, ...]
    value_text: str | None = None
    control: str | None = None
    model_name: str | None = None
    line_number: int = 0

    @property
    def parameter_name(self) -> str:
        return self.name

    def numeric_value(self, parameters: Mapping[str, float]) -> float:
        lookup = {_key(k): float(v) for k, v in parameters.items()}
        if _key(self.name) in lookup:
            return lookup[_key(self.name)]
        if self.value_text is not None:
            value = parse_spice_value(self.value_text)
            if value is not None:
                return value
        raise UnresolvedParameterError([self.parameter_name])


@dataclass(frozen=True)
class Observation:
    kind: str
    target: str
    negative: str = "0"
    label: str | None = None

    @property
    def name(self) -> str:
        if self.label is not None:
            return self.label
        if self.kind == "V" and not _is_ground(self.negative):
            return f"V({self.target},{self.negative})"
        return f"{self.kind}({self.target})"


@dataclass(frozen=True)
class DeviceModel:
    name: str
    kind: str
    parameters: dict[str, str]

    def numeric(self, name: str, default: float | None = None) -> float | None:
        raw = self.parameters.get(_key(name))
        if raw is None:
            return default
        value = parse_spice_value(raw)
        return default if value is None else value


@dataclass
class Circuit:
    title: str
    elements: list[Element]
    observations: list[Observation]
    nodes: list[str]
    models: dict[str, DeviceModel] = field(default_factory=dict)
    ignored_directives: list[str] = field(default_factory=list)

    def element(self, name: str) -> Element:
        for item in self.elements:
            if _key(item.name) == _key(name):
                return item
        raise NetlistError(f"observation refers to unknown element {name!r}")


def _parse_observations(line: str, line_number: int) -> list[Observation]:
    result: list[Observation] = []
    for match in re.finditer(r"\b([VI])\s*\(([^)]*)\)", line, re.IGNORECASE):
        kind = match.group(1).upper()
        body = match.group(2).strip()
        if kind == "I":
            target = body.strip().strip("[]")
            if not target:
                raise NetlistError(f"line {line_number}: empty current observation")
            result.append(Observation("I", target))
            continue
        pieces = [piece for piece in re.split(r"[\s,]+", body) if piece]
        if not 1 <= len(pieces) <= 2:
            raise NetlistError(f"line {line_number}: invalid voltage observation {body!r}")
        result.append(
            Observation("V", _clean_node(pieces[0]), _clean_node(pieces[1]) if len(pieces) == 2 else "0")
        )
    return result


def parse_netlist(path: str | Path) -> Circuit:
    """Parse a SPICE-like netlist, including TINA probes and current arrows."""

    netlist_path = Path(path)
    text = netlist_path.read_text(encoding="utf-8-sig", errors="replace")
    title = netlist_path.stem
    elements: list[Element] = []
    observations: list[Observation] = []
    ignored: list[str] = []
    models: dict[str, DeviceModel] = {}
    node_names: dict[str, str] = {}
    seen_element = False

    logical_lines: list[tuple[int, str]] = []
    for physical_line_number, physical_line in enumerate(text.splitlines(), 1):
        if physical_line.lstrip().startswith("+") and logical_lines:
            first_line_number, previous = logical_lines[-1]
            logical_lines[-1] = (first_line_number, previous + " " + physical_line.lstrip()[1:].strip())
        else:
            logical_lines.append((physical_line_number, physical_line))

    for line_number, raw_line in logical_lines:
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("*"):
            continue
        if stripped.startswith("."):
            directive = stripped.split(None, 1)[0].upper()
            if directive in {".PROBE", ".PRINT", ".SAVE"}:
                observations.extend(_parse_observations(stripped, line_number))
            elif directive == ".MODEL":
                match = re.match(r"\.MODEL\s+(\S+)\s+(\w+)\s*\((.*)\)\s*$", stripped, re.IGNORECASE)
                if not match:
                    raise NetlistError(f"line {line_number}: malformed .MODEL directive")
                parameters = {
                    _key(item.group(1)): item.group(2)
                    for item in re.finditer(r"([A-Za-z][A-Za-z0-9_]*)\s*=\s*([^\s()]+)", match.group(3))
                }
                model = DeviceModel(match.group(1), match.group(2).upper(), parameters)
                models[_key(model.name)] = model
            elif directive == ".END":
                break
            else:
                # Analysis requests and library/model registration do not affect
                # construction of the linear circuit model.
                ignored.append(stripped)
            continue

        body, separator, comment = raw_line.partition(";")
        tokens = body.split()
        if not tokens:
            continue
        name = tokens[0]
        kind = name[0].upper()
        if kind not in {"R", "L", "C", "V", "I", "O", "E", "G", "F", "H", "D", "M", "X"}:
            if not seen_element:
                title = stripped
                continue
            ignored.append(stripped)
            continue

        minimum_tokens = 6 if kind in {"E", "G", "M"} else 5 if kind in {"F", "H", "X"} else 4
        possible_current_arrow = kind == "V" and len(tokens) == 3 and (
            _key(name).startswith("VAM") or "CURRENT ARROW" in comment.upper()
        )
        if not seen_element and len(tokens) < minimum_tokens and not possible_current_arrow:
            # SPICE's first data line is conventionally a free-form title.  It
            # may itself begin with R/C/L/V (for example, "RC low pass").
            title = stripped
            continue

        seen_element = True

        def require(count: int, syntax: str) -> None:
            if len(tokens) < count:
                raise NetlistError(f"line {line_number}: expected {syntax}; got {stripped!r}")

        if kind in {"R", "L", "C"}:
            require(4, f"{kind}<name> n+ n- value")
            element = Element(name, kind, (_clean_node(tokens[1]), _clean_node(tokens[2])), tokens[3], line_number=line_number)
        elif kind in {"V", "I"}:
            require(3, f"{kind}<name> n+ n- value")
            is_current_arrow = kind == "V" and len(tokens) == 3 and (
                _key(name).startswith("VAM") or "CURRENT ARROW" in comment.upper()
            )
            if is_current_arrow:
                element = Element(name, "AMMETER", (_clean_node(tokens[1]), _clean_node(tokens[2])), "0", line_number=line_number)
            else:
                require(4, f"{kind}<name> n+ n- value")
                value_index = 4 if tokens[3].upper() == "DC" and len(tokens) > 4 else 3
                element = Element(name, kind, (_clean_node(tokens[1]), _clean_node(tokens[2])), tokens[value_index], line_number=line_number)
        elif kind == "O":
            require(4, "O<name> n+ n- nout")
            element = Element(name, kind, tuple(_clean_node(token) for token in tokens[1:4]), line_number=line_number)
        elif kind == "D":
            require(4, "D<name> anode cathode model")
            element = Element(
                name,
                kind,
                (_clean_node(tokens[1]), _clean_node(tokens[2])),
                model_name=tokens[3],
                line_number=line_number,
            )
        elif kind == "M":
            require(6, "M<name> drain gate source bulk model")
            element = Element(
                name,
                kind,
                tuple(_clean_node(token) for token in tokens[1:5]),
                model_name=tokens[5],
                line_number=line_number,
            )
        elif kind == "X":
            require(5, "X<name> n+ n- nout subcircuit")
            if _key(tokens[-1]) != "IDOPAMP" or len(tokens) != 5:
                raise NetlistError(
                    f"line {line_number}: unsupported subcircuit {tokens[-1]!r}; only three-pin IdOpamp is supported"
                )
            element = Element(
                name,
                "O",
                tuple(_clean_node(token) for token in tokens[1:4]),
                model_name=tokens[4],
                line_number=line_number,
            )
        elif kind in {"E", "G"}:
            require(6, f"{kind}<name> n+ n- nc+ nc- gain")
            element = Element(name, kind, tuple(_clean_node(token) for token in tokens[1:5]), tokens[5], line_number=line_number)
        else:
            require(5, f"{kind}<name> n+ n- controlling_voltage_source gain")
            element = Element(
                name,
                kind,
                (_clean_node(tokens[1]), _clean_node(tokens[2])),
                tokens[4],
                control=tokens[3],
                line_number=line_number,
            )

        elements.append(element)
        for node in element.nodes:
            if not _is_ground(node):
                node_names.setdefault(_key(node), node)

    if not elements:
        raise NetlistError(f"{netlist_path}: no supported circuit elements found")
    duplicate_names = [name for name in {_key(e.name) for e in elements} if sum(_key(e.name) == name for e in elements) > 1]
    if duplicate_names:
        raise NetlistError("duplicate element name(s): " + ", ".join(sorted(duplicate_names)))
    return Circuit(title, elements, observations, list(node_names.values()), models, ignored)


@dataclass
class DescriptorModel:
    E: np.ndarray
    A: np.ndarray
    B: np.ndarray
    unknown_names: list[str]
    input_names: list[str]
    input_defaults: np.ndarray
    branch_indices: dict[str, int]
    node_indices: dict[str, int]


@dataclass
class SymbolicDescriptorModel:
    """Exact descriptor matrices whose element names are symbolic parameters."""

    E: se.Matrix
    A: se.Matrix
    B: se.Matrix
    unknowns: se.Matrix
    inputs: se.Matrix
    unknown_names: list[str]
    input_names: list[str]
    parameter_defaults: dict[str, float | None]


def assemble_symbolic_mna(circuit: Circuit) -> SymbolicDescriptorModel:
    """Build an exact SymEngine descriptor model without resolving parameters.

    Passive and controlled-source element names are used as symbols, matching
    SCAM's convention.  Numeric values from the netlist are retained separately
    in ``parameter_defaults`` and can be substituted by callers when desired.
    Independent source names form the input vector and therefore do not appear
    as fixed source values in the matrices.
    """

    nonlinear = [element.name for element in circuit.elements if element.kind in {"D", "M"}]
    if nonlinear:
        raise NetlistError(
            "nonlinear switch element(s) require switched_solver.py: " + ", ".join(nonlinear)
        )
    node_indices = {_key(node): index for index, node in enumerate(circuit.nodes)}
    branch_elements = [e for e in circuit.elements if e.kind in {"V", "AMMETER", "L", "O", "E", "H"}]
    branch_indices = {_key(e.name): len(node_indices) + index for index, e in enumerate(branch_elements)}
    inputs = [e for e in circuit.elements if e.kind in {"V", "I"}]
    input_indices = {_key(e.name): index for index, e in enumerate(inputs)}
    size = len(node_indices) + len(branch_elements)
    E = se.zeros(size, size)
    A = se.zeros(size, size)
    B = se.zeros(size, len(inputs))
    symbols = {_key(e.name): se.Symbol(e.name) for e in circuit.elements}

    def ni(node: str) -> int | None:
        return None if _is_ground(node) else node_indices[_key(node)]

    def incidence(n_plus: str, n_minus: str) -> se.Matrix:
        vector = se.zeros(size, 1)
        plus = ni(n_plus)
        minus = ni(n_minus)
        if plus is not None:
            vector[plus, 0] += 1
        if minus is not None:
            vector[minus, 0] -= 1
        return vector

    for element in circuit.elements:
        symbol = symbols[_key(element.name)]
        if element.kind in {"R", "C"}:
            p = incidence(element.nodes[0], element.nodes[1])
            stamp = p * p.T
            if element.kind == "R":
                A -= stamp / symbol
            else:
                E += symbol * stamp
        elif element.kind in {"V", "AMMETER", "L", "E", "H"}:
            p = incidence(element.nodes[0], element.nodes[1])
            branch = branch_indices[_key(element.name)]
            for row in range(size):
                A[row, branch] -= p[row, 0]
                A[branch, row] += p[row, 0]
            if element.kind == "L":
                E[branch, branch] += symbol
        elif element.kind == "O":
            branch = branch_indices[_key(element.name)]
            output_p = incidence(element.nodes[2], "0")
            control_p = incidence(element.nodes[0], element.nodes[1])
            for row in range(size):
                A[row, branch] -= output_p[row, 0]
                A[branch, row] += control_p[row, 0]

    for element in circuit.elements:
        symbol = symbols[_key(element.name)]
        if element.kind == "V":
            B[branch_indices[_key(element.name)], input_indices[_key(element.name)]] -= 1
        elif element.kind == "I":
            p = incidence(element.nodes[0], element.nodes[1])
            for row in range(size):
                B[row, input_indices[_key(element.name)]] -= p[row, 0]
        elif element.kind == "E":
            row = branch_indices[_key(element.name)]
            p = incidence(element.nodes[2], element.nodes[3])
            for column in range(size):
                A[row, column] -= symbol * p[column, 0]
        elif element.kind == "G":
            A -= symbol * incidence(element.nodes[0], element.nodes[1]) * incidence(element.nodes[2], element.nodes[3]).T
        elif element.kind in {"F", "H"}:
            control_key = _key(element.control or "")
            if control_key not in branch_indices:
                raise NetlistError(f"{element.name}: controlling source {element.control!r} has no MNA branch current")
            control_branch = branch_indices[control_key]
            if element.kind == "F":
                p = incidence(element.nodes[0], element.nodes[1])
                for row in range(size):
                    A[row, control_branch] -= symbol * p[row, 0]
            else:
                A[branch_indices[_key(element.name)], control_branch] -= symbol

    unknown_names = [f"v({node})" for node in circuit.nodes] + [f"i({element.name})" for element in branch_elements]
    unknown_symbols = [se.Symbol(f"z{index}") for index in range(size)]
    input_symbols = [se.Symbol(element.name) for element in inputs]
    defaults = {
        element.name: parse_spice_value(element.value_text or "")
        for element in circuit.elements
        if element.kind not in {"V", "I", "AMMETER", "O"}
    }
    return SymbolicDescriptorModel(
        E,
        A,
        B,
        se.DenseMatrix(size, 1, unknown_symbols),
        se.DenseMatrix(len(input_symbols), 1, input_symbols),
        unknown_names,
        [element.name for element in inputs],
        defaults,
    )


def assemble_mna(circuit: Circuit, parameters: Mapping[str, float] | None = None) -> DescriptorModel:
    """Build the numeric descriptor equation ``E z_dot = A z + B u``."""

    nonlinear = [element.name for element in circuit.elements if element.kind in {"D", "M"}]
    if nonlinear:
        raise NetlistError(
            "nonlinear switch element(s) require switched_solver.py: " + ", ".join(nonlinear)
        )
    parameters = parameters or {}
    node_indices = {_key(node): index for index, node in enumerate(circuit.nodes)}
    branch_elements = [e for e in circuit.elements if e.kind in {"V", "AMMETER", "L", "O", "E", "H"}]
    branch_indices = {_key(e.name): len(node_indices) + index for index, e in enumerate(branch_elements)}
    inputs = [e for e in circuit.elements if e.kind in {"V", "I"}]
    input_indices = {_key(e.name): index for index, e in enumerate(inputs)}
    size = len(node_indices) + len(branch_elements)
    E = np.zeros((size, size), dtype=float)
    A = np.zeros((size, size), dtype=float)
    B = np.zeros((size, len(inputs)), dtype=float)

    def ni(node: str) -> int | None:
        return None if _is_ground(node) else node_indices[_key(node)]

    def incidence(n_plus: str, n_minus: str) -> np.ndarray:
        vector = np.zeros(size)
        if ni(n_plus) is not None:
            vector[ni(n_plus)] += 1.0  # type: ignore[index]
        if ni(n_minus) is not None:
            vector[ni(n_minus)] -= 1.0  # type: ignore[index]
        return vector

    unresolved: list[str] = []

    def value(element: Element) -> float:
        try:
            return element.numeric_value(parameters)
        except UnresolvedParameterError:
            unresolved.append(element.parameter_name)
            return math.nan

    # Passive nodal stamps and branch incidence.
    for element in circuit.elements:
        if element.kind in {"R", "C"}:
            val = value(element)
            p = incidence(element.nodes[0], element.nodes[1])
            stamp = np.outer(p, p)
            if element.kind == "R":
                if val == 0:
                    raise NetlistError(f"{element.name}: zero resistance is not allowed; use a 0 V source")
                A -= stamp / val
            else:
                E += val * stamp
        elif element.kind in {"V", "AMMETER", "L", "E", "H"}:
            p = incidence(element.nodes[0], element.nodes[1])
            branch = branch_indices[_key(element.name)]
            A[:, branch] -= p
            A[branch, :] += p
            if element.kind == "L":
                E[branch, branch] += value(element)
        elif element.kind == "O":
            branch = branch_indices[_key(element.name)]
            output_incidence = incidence(element.nodes[2], "0")
            A[:, branch] -= output_incidence
            A[branch, :] += incidence(element.nodes[0], element.nodes[1])

    # Independent sources and controlled-source constraints.
    for element in circuit.elements:
        if element.kind == "V":
            B[branch_indices[_key(element.name)], input_indices[_key(element.name)]] -= 1.0
        elif element.kind == "I":
            B[:, input_indices[_key(element.name)]] -= incidence(element.nodes[0], element.nodes[1])
        elif element.kind == "E":
            gain = value(element)
            row = branch_indices[_key(element.name)]
            A[row, :] -= gain * incidence(element.nodes[2], element.nodes[3])
        elif element.kind == "G":
            gain = value(element)
            A -= gain * np.outer(
                incidence(element.nodes[0], element.nodes[1]),
                incidence(element.nodes[2], element.nodes[3]),
            )
        elif element.kind in {"F", "H"}:
            gain = value(element)
            control_key = _key(element.control or "")
            if control_key not in branch_indices:
                raise NetlistError(f"{element.name}: controlling source {element.control!r} has no MNA branch current")
            control_branch = branch_indices[control_key]
            if element.kind == "F":
                A[:, control_branch] -= gain * incidence(element.nodes[0], element.nodes[1])
            else:
                A[branch_indices[_key(element.name)], control_branch] -= gain

    if unresolved:
        raise UnresolvedParameterError(unresolved)
    defaults = np.array(
        [parse_spice_value(element.value_text or "") if element.value_text is not None else None for element in inputs],
        dtype=object,
    )
    unknown_names = [f"v({node})" for node in circuit.nodes] + [f"i({element.name})" for element in branch_elements]
    return DescriptorModel(E, A, B, unknown_names, [e.name for e in inputs], defaults, branch_indices, node_indices)


def symbolic_mna_equations(circuit: Circuit) -> list[str]:
    """Return human-readable topology equations without requiring numeric values."""

    terms: dict[str, list[str]] = {_key(node): [] for node in circuit.nodes}
    constraints: list[str] = []

    def voltage(node: str) -> str:
        return "0" if _is_ground(node) else f"v({node})"

    def difference(n_plus: str, n_minus: str) -> str:
        return f"({voltage(n_plus)} - {voltage(n_minus)})"

    def add_current(n_plus: str, n_minus: str, expression: str) -> None:
        if not _is_ground(n_plus):
            terms[_key(n_plus)].append(f"+ {expression}")
        if not _is_ground(n_minus):
            terms[_key(n_minus)].append(f"- {expression}")

    for e in circuit.elements:
        if e.kind == "R":
            add_current(e.nodes[0], e.nodes[1], f"{difference(*e.nodes[:2])}/{e.name}")
        elif e.kind == "C":
            add_current(e.nodes[0], e.nodes[1], f"{e.name}*d{difference(*e.nodes[:2])}/dt")
        elif e.kind == "I":
            add_current(e.nodes[0], e.nodes[1], e.name)
        elif e.kind in {"V", "AMMETER", "L", "E", "H"}:
            add_current(e.nodes[0], e.nodes[1], f"i({e.name})")
        elif e.kind == "O":
            add_current(e.nodes[2], "0", f"i({e.name})")
        elif e.kind == "G":
            add_current(e.nodes[0], e.nodes[1], f"{e.name}*{difference(e.nodes[2], e.nodes[3])}")
        elif e.kind == "F":
            add_current(e.nodes[0], e.nodes[1], f"{e.name}*i({e.control})")

        if e.kind == "V":
            constraints.append(f"{difference(*e.nodes[:2])} = {e.name}")
        elif e.kind == "AMMETER":
            constraints.append(f"{difference(*e.nodes[:2])} = 0")
        elif e.kind == "L":
            constraints.append(f"{e.name}*di({e.name})/dt = {difference(*e.nodes[:2])}")
        elif e.kind == "O":
            constraints.append(f"{difference(e.nodes[0], e.nodes[1])} = 0")
        elif e.kind == "E":
            constraints.append(f"{difference(*e.nodes[:2])} = {e.name}*{difference(e.nodes[2], e.nodes[3])}")
        elif e.kind == "H":
            constraints.append(f"{difference(*e.nodes[:2])} = {e.name}*i({e.control})")

    equations = [f"KCL({node}): {' '.join(terms[_key(node)]) or '0'} = 0" for node in circuit.nodes]
    return equations + constraints


@dataclass
class OutputDescriptor:
    Q: np.ndarray
    L: np.ndarray
    M: np.ndarray
    names: list[str]


def assemble_outputs(
    circuit: Circuit,
    model: DescriptorModel,
    parameters: Mapping[str, float] | None = None,
    observations: Sequence[Observation] | None = None,
) -> OutputDescriptor:
    """Build ``y = Q z_dot + L z + M u`` for requested probes."""

    parameters = parameters or {}
    selected = list(observations if observations is not None else circuit.observations)
    if not selected:
        selected = [Observation("V", node) for node in circuit.nodes]
    Q = np.zeros((len(selected), model.A.shape[0]))
    L = np.zeros_like(Q)
    M = np.zeros((len(selected), model.B.shape[1]))
    input_indices = {_key(name): i for i, name in enumerate(model.input_names)}

    def incidence(n_plus: str, n_minus: str) -> np.ndarray:
        vector = np.zeros(model.A.shape[0])
        if not _is_ground(n_plus):
            if _key(n_plus) not in model.node_indices:
                raise NetlistError(f"observation refers to unknown node {n_plus!r}")
            vector[model.node_indices[_key(n_plus)]] += 1
        if not _is_ground(n_minus):
            if _key(n_minus) not in model.node_indices:
                raise NetlistError(f"observation refers to unknown node {n_minus!r}")
            vector[model.node_indices[_key(n_minus)]] -= 1
        return vector

    for row, observation in enumerate(selected):
        if observation.kind == "V":
            L[row, :] = incidence(observation.target, observation.negative)
            continue
        element = circuit.element(observation.target)
        if _key(element.name) in model.branch_indices:
            L[row, model.branch_indices[_key(element.name)]] = 1.0
        elif element.kind == "I":
            M[row, input_indices[_key(element.name)]] = 1.0
        elif element.kind == "R":
            L[row, :] = incidence(*element.nodes[:2]) / element.numeric_value(parameters)
        elif element.kind == "C":
            Q[row, :] = element.numeric_value(parameters) * incidence(*element.nodes[:2])
        elif element.kind == "G":
            L[row, :] = element.numeric_value(parameters) * incidence(element.nodes[2], element.nodes[3])
        elif element.kind == "F":
            control_key = _key(element.control or "")
            if control_key not in model.branch_indices:
                raise NetlistError(f"{element.name}: unknown controlling branch {element.control!r}")
            L[row, model.branch_indices[control_key]] = element.numeric_value(parameters)
        else:
            raise NetlistError(f"current observation for {element.kind} element {element.name} is unsupported")
    return OutputDescriptor(Q, L, M, [item.name for item in selected])


@dataclass
class StateSpaceModel:
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray
    F: np.ndarray
    state_names: list[str]
    input_names: list[str]
    output_names: list[str]
    input_defaults: np.ndarray
    reconstruction_x: np.ndarray
    reconstruction_u: np.ndarray
    unknown_names: list[str]


def reduce_to_state_space(
    descriptor: DescriptorModel,
    outputs: OutputDescriptor,
    tolerance: float | None = None,
) -> StateSpaceModel:
    """Eliminate algebraic variables from an index-1 descriptor model by SVD."""

    E, A, B = descriptor.E, descriptor.A, descriptor.B
    U, singular_values, Vh = np.linalg.svd(E)
    base = singular_values[0] if singular_values.size else 0.0
    tol = tolerance if tolerance is not None else max(E.shape, default=0) * np.finfo(float).eps * max(base, 1.0)
    rank = int(np.sum(singular_values > tol))
    V = Vh.T
    V1, V2 = V[:, :rank], V[:, rank:]
    U1, U2 = U[:, :rank], U[:, rank:]

    E11 = U1.T @ E @ V1
    A11 = U1.T @ A @ V1
    A12 = U1.T @ A @ V2
    A21 = U2.T @ A @ V1
    A22 = U2.T @ A @ V2
    B1 = U1.T @ B
    B2 = U2.T @ B

    if A22.size:
        if np.linalg.matrix_rank(A22) < A22.shape[0]:
            raise NetlistError(
                "algebraic MNA block is singular; check floating nodes, conflicting ideal sources, "
                "or descriptor systems above index 1"
            )
        Kx = -np.linalg.solve(A22, A21)
        Ku = -np.linalg.solve(A22, B2)
    else:
        Kx = np.zeros((0, rank))
        Ku = np.zeros((0, B.shape[1]))
    Tx = V1 + V2 @ Kx
    Tu = V2 @ Ku
    dynamic_A = A11 + A12 @ Kx
    dynamic_B = B1 + A12 @ Ku
    if rank:
        state_A = np.linalg.solve(E11, dynamic_A)
        state_B = np.linalg.solve(E11, dynamic_B)
    else:
        state_A = np.zeros((0, 0))
        state_B = np.zeros((0, B.shape[1]))

    C = outputs.Q @ Tx @ state_A + outputs.L @ Tx
    D = outputs.Q @ Tx @ state_B + outputs.L @ Tu + outputs.M
    F = outputs.Q @ Tu
    return StateSpaceModel(
        state_A,
        state_B,
        C,
        D,
        F,
        [f"x{index}" for index in range(rank)],
        descriptor.input_names,
        outputs.names,
        descriptor.input_defaults,
        Tx,
        Tu,
        descriptor.unknown_names,
    )


@dataclass
class DiscreteModel:
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray
    F: np.ndarray
    dt: float
    method: str
    state_names: list[str]
    input_names: list[str]
    output_names: list[str]
    input_defaults: np.ndarray


def discretize(model: StateSpaceModel, dt: float, method: str = "forward_euler") -> DiscreteModel:
    """Discretize a state model through a replaceable method interface."""

    if dt <= 0:
        raise ValueError("dt must be positive")
    normalized = method.lower().replace("-", "_")
    if normalized in {"forward_euler", "euler"}:
        Ad = np.eye(model.A.shape[0]) + dt * model.A
        Bd = dt * model.B
        normalized = "forward_euler"
    elif normalized in {"backward_euler", "implicit_euler"}:
        lhs = np.eye(model.A.shape[0]) - dt * model.A
        Ad = np.linalg.solve(lhs, np.eye(model.A.shape[0]))
        Bd = np.linalg.solve(lhs, dt * model.B)
        normalized = "backward_euler"
    else:
        raise ValueError(
            f"unknown discretization method {method!r}; available: forward_euler, backward_euler"
        )
    return DiscreteModel(
        Ad, Bd, model.C.copy(), model.D.copy(), model.F.copy(), dt, normalized,
        model.state_names, model.input_names, model.output_names, model.input_defaults,
    )


def discrete_equations(model: DiscreteModel, precision: int = 12) -> list[str]:
    """Render scalar iterative expressions suitable for reviewing code generation."""

    def expression(coefficients: Sequence[tuple[float, str]]) -> str:
        terms: list[str] = []
        threshold = 10.0 ** (-precision)
        for coefficient, variable in coefficients:
            if abs(coefficient) <= threshold:
                continue
            magnitude = f"{abs(coefficient):.{precision}g}"
            term = f"{magnitude}*{variable}"
            if not terms:
                terms.append(("-" if coefficient < 0 else "") + term)
            else:
                terms.append((" - " if coefficient < 0 else " + ") + term)
        return "".join(terms) or "0"

    equations: list[str] = []
    for row, name in enumerate(model.state_names):
        items = [(model.A[row, column], f"{state}[k]") for column, state in enumerate(model.state_names)]
        items += [(model.B[row, column], f"{input_name}[k]") for column, input_name in enumerate(model.input_names)]
        equations.append(f"{name}[k+1] = {expression(items)}")
    for row, name in enumerate(model.output_names):
        items = [(model.C[row, column], f"{state}[k]") for column, state in enumerate(model.state_names)]
        items += [(model.D[row, column], f"{input_name}[k]") for column, input_name in enumerate(model.input_names)]
        items += [
            (model.F[row, column] / model.dt, f"({input_name}[k]-{input_name}[k-1])")
            for column, input_name in enumerate(model.input_names)
        ]
        equations.append(f"{name}[k] = {expression(items)}")
    return equations


@dataclass
class SimulationResult:
    time: np.ndarray
    states: np.ndarray
    inputs: np.ndarray
    outputs: np.ndarray
    state_names: list[str]
    input_names: list[str]
    output_names: list[str]


InputValue = float | Sequence[float] | np.ndarray | Callable[[float], float]


def _sample_inputs(
    model: DiscreteModel,
    times: np.ndarray,
    values: Mapping[str, InputValue] | None,
) -> np.ndarray:
    supplied = {_key(name): value for name, value in (values or {}).items()}
    unknown = set(supplied) - {_key(name) for name in model.input_names}
    if unknown:
        raise ValueError("unknown input(s): " + ", ".join(sorted(unknown)))
    samples = np.empty((len(times), len(model.input_names)))
    missing: list[str] = []
    for column, name in enumerate(model.input_names):
        source = supplied.get(_key(name), model.input_defaults[column])
        if source is None:
            missing.append(name)
            continue
        if callable(source):
            samples[:, column] = [float(source(float(t))) for t in times]
        elif np.isscalar(source):
            samples[:, column] = float(source)
        else:
            array = np.asarray(source, dtype=float)
            if array.shape != (len(times),):
                raise ValueError(f"input {name} requires {len(times)} samples, got {array.shape}")
            samples[:, column] = array
    if missing:
        raise ValueError("no numeric/default value for input(s): " + ", ".join(missing))
    return samples


def simulate(
    model: DiscreteModel,
    duration: float,
    inputs: Mapping[str, InputValue] | None = None,
    initial_state: Sequence[float] | None = None,
) -> SimulationResult:
    """Run a discrete state iteration and return all sampled signals."""

    if duration < 0:
        raise ValueError("duration must be non-negative")
    steps = int(math.ceil(duration / model.dt))
    times = np.arange(steps + 1, dtype=float) * model.dt
    U = _sample_inputs(model, times, inputs)
    X = np.zeros((steps + 1, model.A.shape[0]))
    if initial_state is not None:
        x0 = np.asarray(initial_state, dtype=float)
        if x0.shape != (model.A.shape[0],):
            raise ValueError(f"initial state must have shape {(model.A.shape[0],)}")
        X[0] = x0
    for step in range(steps):
        X[step + 1] = model.A @ X[step] + model.B @ U[step]
    if len(times) > 1:
        Udot = np.vstack(((U[1] - U[0]) / model.dt, np.diff(U, axis=0) / model.dt))
    else:
        Udot = np.zeros_like(U)
    Y = X @ model.C.T + U @ model.D.T + Udot @ model.F.T
    return SimulationResult(times, X, U, Y, model.state_names, model.input_names, model.output_names)


@dataclass
class FrequencyResponse:
    frequency_hz: np.ndarray
    response: np.ndarray
    input_names: list[str]
    output_names: list[str]

    @property
    def magnitude(self) -> np.ndarray:
        return np.abs(self.response)

    @property
    def magnitude_db(self) -> np.ndarray:
        with np.errstate(divide="ignore"):
            return 20.0 * np.log10(self.magnitude)

    @property
    def phase_deg(self) -> np.ndarray:
        return np.angle(self.response, deg=True)


def frequency_response(model: StateSpaceModel, frequencies_hz: Sequence[float]) -> FrequencyResponse:
    """Evaluate ``C(jwI-A)^-1B + D + jwF``."""

    frequencies = np.asarray(frequencies_hz, dtype=float)
    if frequencies.ndim != 1 or np.any(frequencies < 0):
        raise ValueError("frequencies must be a one-dimensional non-negative sequence")
    response = np.empty((len(frequencies), model.C.shape[0], model.B.shape[1]), dtype=complex)
    identity = np.eye(model.A.shape[0])
    for index, frequency in enumerate(frequencies):
        jw = 2j * np.pi * frequency
        dynamic = model.C @ np.linalg.solve(jw * identity - model.A, model.B) if model.A.size else np.zeros_like(model.D)
        response[index] = dynamic + model.D + jw * model.F
    return FrequencyResponse(frequencies, response, model.input_names, model.output_names)


def _matrix_text(name: str, matrix: np.ndarray) -> str:
    return f"{name} =\n{np.array2string(matrix, precision=9, suppress_small=True)}"


def _parse_assignments(items: Sequence[str]) -> dict[str, float]:
    result: dict[str, float] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"expected NAME=VALUE, got {item!r}")
        name, value = item.split("=", 1)
        parsed = parse_spice_value(value)
        if not name or parsed is None:
            raise ValueError(f"expected numeric NAME=VALUE, got {item!r}")
        result[name] = parsed
    return result


def _cli_spice_value(text: str) -> float:
    value = parse_spice_value(text)
    if value is None:
        raise argparse.ArgumentTypeError(f"expected a numeric/SPICE value, got {text!r}")
    return value


def _load_model(path: str, parameters: Mapping[str, float]) -> tuple[Circuit, DescriptorModel, StateSpaceModel]:
    circuit = parse_netlist(path)
    descriptor = assemble_mna(circuit, parameters)
    outputs = assemble_outputs(circuit, descriptor, parameters)
    return circuit, descriptor, reduce_to_state_space(descriptor, outputs)


def _write_simulation(path: str, result: SimulationResult) -> None:
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", *result.input_names, *result.state_names, *result.output_names])
        for index, time in enumerate(result.time):
            writer.writerow([time, *result.inputs[index], *result.states[index], *result.outputs[index]])


def _write_frequency(path: str, result: FrequencyResponse) -> None:
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        header = ["frequency_hz"]
        for output in result.output_names:
            for input_name in result.input_names:
                header.extend([f"{output}/{input_name}_magnitude", f"{output}/{input_name}_magnitude_db", f"{output}/{input_name}_phase_deg"])
        writer.writerow(header)
        for fi, frequency in enumerate(result.frequency_hz):
            row: list[float] = [frequency]
            for oi in range(len(result.output_names)):
                for ii in range(len(result.input_names)):
                    row.extend([result.magnitude[fi, oi, ii], result.magnitude_db[fi, oi, ii], result.phase_deg[fi, oi, ii]])
            writer.writerow(row)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MNA/state-space circuit solver for CCTL Studio")
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("analyze", "discretize", "simulate", "frequency"):
        command = commands.add_parser(name)
        command.add_argument("netlist")
        command.add_argument("--param", action="append", default=[], metavar="NAME=VALUE")
        if name in {"discretize", "simulate"}:
            command.add_argument("--dt", type=_cli_spice_value, required=True)
            command.add_argument("--method", default="forward_euler")
            if name == "simulate":
                command.add_argument("--duration", type=_cli_spice_value, required=True)
                command.add_argument("--input", action="append", default=[], metavar="NAME=VALUE")
                command.add_argument("--output", required=True, help="output CSV path")
        elif name == "frequency":
            command.add_argument("--start", type=_cli_spice_value, required=True, help="start frequency in Hz")
            command.add_argument("--stop", type=_cli_spice_value, required=True, help="stop frequency in Hz")
            command.add_argument("--points", type=int, default=100)
            command.add_argument("--output", required=True, help="output CSV path")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        parameters = _parse_assignments(args.param)
        circuit = parse_netlist(args.netlist)
        if args.command == "analyze":
            print(f"Circuit: {circuit.title}")
            print("\nTopology equations:")
            print("\n".join(symbolic_mna_equations(circuit)))
            symbolic = assemble_symbolic_mna(circuit)
            print("\nExact symbolic descriptor model: E*z_dot = A*z + B*u")
            print("z =", symbolic.unknown_names)
            print("u =", symbolic.input_names)
            print("E =\n", symbolic.E, sep="")
            print("A =\n", symbolic.A, sep="")
            print("B =\n", symbolic.B, sep="")
            if symbolic.parameter_defaults:
                print("parameter defaults =", symbolic.parameter_defaults)
            try:
                descriptor = assemble_mna(circuit, parameters)
            except UnresolvedParameterError as exc:
                print(f"\nNumeric state-space reduction skipped: {exc}")
                print("Supply values with repeated --param NAME=VALUE options.")
                return 0
            state = reduce_to_state_space(descriptor, assemble_outputs(circuit, descriptor, parameters))
            print("\nNumeric descriptor model: E*z_dot = A*z + B*u")
            print("z =", descriptor.unknown_names)
            print("u =", descriptor.input_names)
            print(_matrix_text("E", descriptor.E))
            print(_matrix_text("A", descriptor.A))
            print(_matrix_text("B", descriptor.B))
            print("\nState/output model: x_dot = A*x + B*u; y = C*x + D*u + F*u_dot")
            print("x =", state.state_names, "u =", state.input_names, "y =", state.output_names)
            for name, matrix in (("A", state.A), ("B", state.B), ("C", state.C), ("D", state.D), ("F", state.F)):
                print(_matrix_text(name, matrix))
        else:
            _, _, state = _load_model(args.netlist, parameters)
            if args.command == "discretize":
                discrete = discretize(state, args.dt, args.method)
                print(f"Discrete model ({discrete.method}, dt={discrete.dt:g} s):")
                print(_matrix_text("Ad", discrete.A))
                print(_matrix_text("Bd", discrete.B))
                print(_matrix_text("C", discrete.C))
                print(_matrix_text("D", discrete.D))
                print("\nScalar iteration equations:")
                print("\n".join(discrete_equations(discrete)))
            elif args.command == "simulate":
                discrete = discretize(state, args.dt, args.method)
                result = simulate(discrete, args.duration, _parse_assignments(args.input))
                _write_simulation(args.output, result)
                print(f"wrote {len(result.time)} samples to {args.output}")
            else:
                if args.start <= 0 or args.stop < args.start or args.points < 1:
                    raise ValueError("frequency requires 0 < start <= stop and points >= 1")
                frequencies = np.geomspace(args.start, args.stop, args.points)
                result = frequency_response(state, frequencies)
                _write_frequency(args.output, result)
                print(f"wrote {len(result.frequency_hz)} frequency points to {args.output}")
    except (NetlistError, ValueError, np.linalg.LinAlgError) as exc:
        print(f"error: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
