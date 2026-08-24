"""Export a CCTL Studio circuit layer to the MNA Solver netlist dialect."""

from __future__ import annotations

from collections import defaultdict
from typing import Any, Mapping

from cctl_studio import StudioError
from component_catalog import MNA_COMPONENTS, MnaComponentSpec
from hierarchy_model import HierarchyDocument


Endpoint = tuple[str, str]


class _DisjointSet:
    def __init__(self, values: list[Endpoint]) -> None:
        self.parent = {value: value for value in values}

    def find(self, value: Endpoint) -> Endpoint:
        parent = self.parent[value]
        if parent != value:
            self.parent[value] = self.find(parent)
        return self.parent[value]

    def union(self, first: Endpoint, second: Endpoint) -> None:
        left, right = self.find(first), self.find(second)
        if left != right:
            self.parent[right] = left


def _parameter(parameters: Mapping[str, Any], key: str, node_id: str) -> str:
    value = parameters.get(key)
    if value is None or not str(value).strip():
        raise StudioError(f"{node_id}: parameter {key!r} is required for MNA export")
    text = str(value).strip()
    if "\n" in text or "\r" in text:
        raise StudioError(f"{node_id}: parameter {key!r} must be one line")
    return text


def _render_element(
    spec: MnaComponentSpec,
    name: str,
    nodes: Mapping[str, str],
    parameters: Mapping[str, Any],
) -> str | None:
    kind = spec.netlist_kind
    if kind is None:
        return None
    ordered_nodes = [nodes[port.port_id] for port in spec.ports]
    if kind in {"R", "L", "C"}:
        key = {"R": "resistance", "L": "inductance", "C": "capacitance"}[kind]
        return f"{name} {' '.join(ordered_nodes)} {_parameter(parameters, key, name)}"
    if kind in {"V", "I"}:
        key = "voltage" if kind == "V" else "current"
        return f"{name} {' '.join(ordered_nodes)} {_parameter(parameters, key, name)}"
    if kind == "O":
        return f"{name} {' '.join(ordered_nodes)}"
    if kind == "X":
        return f"{name} {' '.join(ordered_nodes)} {_parameter(parameters, 'subcircuit', name)}"
    if kind in {"E", "G"}:
        return f"{name} {' '.join(ordered_nodes)} {_parameter(parameters, 'gain', name)}"
    if kind in {"F", "H"}:
        return f"{name} {' '.join(ordered_nodes)} {_parameter(parameters, 'control_source', name)} {_parameter(parameters, 'gain', name)}"
    if kind == "D":
        return f"{name} {' '.join(ordered_nodes)} {_parameter(parameters, 'model', name)}"
    if kind in {"M", "S"}:
        raise StudioError(f"{name}: PWM-driven switch must be expanded by the exporter")
    if kind == "AMMETER":
        return f"{name} {' '.join(ordered_nodes)} ; Current Arrow"
    raise StudioError(f"{name}: unsupported MNA export kind {kind!r}")


def export_mna_netlist(
    hierarchy: HierarchyDocument,
    layer_id: str,
    title: str | None = None,
) -> str:
    layer = hierarchy.layer(layer_id)
    if layer.get("kind") != "circuit":
        raise StudioError("MNA export requires a circuit layer")
    if layer.get("source") == "project.instances":
        raise StudioError(
            "the legacy project.instances layer already uses the schema-v1 generator"
        )

    nodes = list(layer.get("nodes", []))
    endpoints: list[Endpoint] = []
    for node in nodes:
        spec = MNA_COMPONENTS.get(str(node.get("type")))
        if spec is None:
            raise StudioError(f"{node.get('id')}: component is not supported by MNA Solver")
        endpoints.extend((str(node["id"]), port.port_id) for port in spec.ports)
    disjoint = _DisjointSet(endpoints)
    endpoint_set = set(endpoints)
    for wire in layer.get("connections", []):
        source = wire.get("source", {})
        target = wire.get("target", {})
        first = (str(source.get("node")), str(source.get("port")))
        second = (str(target.get("node")), str(target.get("port")))
        if first not in endpoint_set or second not in endpoint_set:
            raise StudioError(f"wire {wire.get('id')}: endpoint does not exist")
        disjoint.union(first, second)

    groups: dict[Endpoint, list[Endpoint]] = defaultdict(list)
    for endpoint in endpoints:
        groups[disjoint.find(endpoint)].append(endpoint)
    ground_roots: set[Endpoint] = set()
    for node in nodes:
        if node.get("type") == "circuit.ground":
            ground_roots.add(disjoint.find((str(node["id"]), "node")))

    node_names: dict[Endpoint, str] = {}
    next_net = 1
    for root, members in groups.items():
        if root in ground_roots:
            net = "0"
        else:
            net = f"N{next_net}"
            next_net += 1
        for endpoint in members:
            node_names[endpoint] = net

    lines = [title or str(layer.get("name", "CCTL Studio circuit"))]
    model_lines: list[str] = []
    pwm_indices = {
        str(node["id"]): index
        for index, node in enumerate(
            (
                node
                for node in nodes
                if MNA_COMPONENTS[str(node["type"])].integrated_pwm_driver
            ),
            start=1,
        )
    }
    for node in nodes:
        spec = MNA_COMPONENTS[str(node["type"])]
        if spec.netlist_kind is None:
            continue
        element_name = str(node["id"])
        if not element_name.upper().startswith(spec.designator.upper()):
            raise StudioError(
                f"{element_name}: name must start with {spec.designator!r} for MNA dispatch"
            )
        port_nodes = {
            port.port_id: node_names[(element_name, port.port_id)] for port in spec.ports
        }
        parameters = node.get("parameters", {})
        if spec.netlist_kind == "M":
            source = port_nodes["source"]
            gate = f"__{element_name}_GATE"
            lines.append(f"VPWM{pwm_indices[element_name]} {gate} {source} 0")
            line = (
                f"{element_name} {port_nodes['drain']} {gate} {source} {source} "
                f"{_parameter(parameters, 'model', element_name)}"
            )
        elif spec.netlist_kind == "S":
            control = f"__{element_name}_CONTROL"
            lines.append(f"VPWM{pwm_indices[element_name]} {control} 0 0")
            line = (
                f"{element_name} {port_nodes['p']} {port_nodes['n']} {control} 0 "
                f"{_parameter(parameters, 'model', element_name)}"
            )
        else:
            line = _render_element(spec, element_name, port_nodes, parameters)
        if line:
            lines.append(line)
        model_line = str(parameters.get("model_line", "")).strip()
        if spec.netlist_kind in {"D", "M", "S"} and not model_line:
            raise StudioError(f"{element_name}: model_line is required for MNA export")
        if model_line and model_line not in model_lines:
            if not model_line.upper().startswith(".MODEL "):
                raise StudioError(f"{element_name}: model_line must start with .MODEL")
            model_tokens = model_line.split()
            if len(model_tokens) < 3 or model_tokens[1].upper() != _parameter(
                parameters, "model", element_name
            ).upper():
                raise StudioError(
                    f"{element_name}: model_line name must match the model parameter"
                )
            model_lines.append(model_line)
    if model_lines:
        lines.extend(("", *model_lines))
    lines.extend((".END", ""))
    return "\n".join(lines)
