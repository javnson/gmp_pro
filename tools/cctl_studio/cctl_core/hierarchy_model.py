"""Hierarchy and editor-only graph model for CCTL Studio.

The canonical simulation data remains in the schema-v1 project.  This module
stores the system diagram and non-electrical child layers below ``editor`` until
the schema-v2 normalized graph replaces that compatibility representation.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from cctl_studio import StudioError
from component_catalog import MNA_COMPONENTS
from editor_model import EditorDocument, default_editor_hierarchy


@dataclass(frozen=True)
class PortSpec:
    port_id: str
    label: str
    direction: str
    domain: str


@dataclass(frozen=True)
class NodeType:
    type_id: str
    display_name: str
    layer_kind: str
    symbol: str
    prefix: str
    ports: tuple[PortSpec, ...]
    child_kind: str | None = None
    defaults: Mapping[str, Any] | None = None


def _port(port_id: str, label: str, direction: str, domain: str) -> PortSpec:
    return PortSpec(port_id, label, direction, domain)


NODE_TYPES: dict[str, NodeType] = {
    "system.electrical_topology": NodeType(
        "system.electrical_topology",
        "Electrical Topology",
        "system",
        "topology",
        "TOP",
        (
            _port("u", "u", "input", "numeric"),
            _port("y", "y", "output", "numeric"),
        ),
        "circuit",
    ),
    "system.digital_module": NodeType(
        "system.digital_module",
        "Digital Module",
        "system",
        "digital_module",
        "DIG",
        (
            _port("u", "u", "input", "numeric"),
            _port("y", "y", "output", "numeric"),
        ),
        "digital",
    ),
    "system.motor": NodeType(
        "system.motor",
        "Motor Model",
        "system",
        "motor",
        "MOTOR",
        (
            _port("voltage", "uabc", "input", "numeric"),
            _port("load", "load", "input", "numeric"),
            _port("current", "iabc", "output", "numeric"),
            _port("speed", "speed", "output", "numeric"),
        ),
    ),
    "system.signal_adapter": NodeType(
        "system.signal_adapter",
        "Signal Adapter",
        "system",
        "adapter",
        "ADP",
        (
            _port("in", "in", "input", "numeric"),
            _port("out", "out", "output", "numeric"),
        ),
        defaults={"gain": 1.0, "offset": 0.0},
    ),
    "digital.input": NodeType(
        "digital.input",
        "Logic Input",
        "digital",
        "input",
        "IN",
        (_port("out", "OUT", "output", "logic"),),
    ),
    "digital.output": NodeType(
        "digital.output",
        "Logic Output",
        "digital",
        "output",
        "OUT",
        (_port("in", "IN", "input", "logic"),),
    ),
    "digital.and": NodeType(
        "digital.and",
        "AND Gate",
        "digital",
        "and",
        "AND",
        (
            _port("a", "A", "input", "logic"),
            _port("b", "B", "input", "logic"),
            _port("out", "OUT", "output", "logic"),
        ),
    ),
    "digital.or": NodeType(
        "digital.or",
        "OR Gate",
        "digital",
        "or",
        "OR",
        (
            _port("a", "A", "input", "logic"),
            _port("b", "B", "input", "logic"),
            _port("out", "OUT", "output", "logic"),
        ),
    ),
    "digital.not": NodeType(
        "digital.not",
        "NOT Gate",
        "digital",
        "not",
        "NOT",
        (
            _port("in", "IN", "input", "logic"),
            _port("out", "OUT", "output", "logic"),
        ),
    ),
    "digital.delay": NodeType(
        "digital.delay",
        "Delay",
        "digital",
        "block",
        "DLY",
        (
            _port("in", "IN", "input", "logic"),
            _port("clock", "CLK", "input", "logic"),
            _port("out", "OUT", "output", "logic"),
        ),
        defaults={"ticks": 1},
    ),
    "digital.one_shot": NodeType(
        "digital.one_shot",
        "One Shot",
        "digital",
        "block",
        "OS",
        (
            _port("in", "IN", "input", "logic"),
            _port("clock", "CLK", "input", "logic"),
            _port("out", "OUT", "output", "logic"),
        ),
        defaults={"duration_ticks": 1},
    ),
}

# The analog palette is owned by the same metadata set as the MNA exporter.
for _spec in MNA_COMPONENTS.values():
    NODE_TYPES[_spec.type_id] = NodeType(
        _spec.type_id,
        _spec.display_name,
        "circuit",
        _spec.symbol,
        _spec.designator,
        tuple(
            _port(port.port_id, port.label, "passive", "electrical")
            for port in _spec.ports
        ),
        defaults=_spec.defaults,
    )


def default_hierarchy(has_circuit: bool = True) -> dict[str, Any]:
    return default_editor_hierarchy(legacy_source=has_circuit)


class HierarchyDocument:
    """Operations for system and editor-only digital/circuit layer graphs."""

    def __init__(self, document: EditorDocument) -> None:
        self.document = document
        hierarchy = document.editor.get("hierarchy")
        if not isinstance(hierarchy, dict):
            document.editor["hierarchy"] = default_hierarchy(True)
        self._normalise()

    @property
    def data(self) -> dict[str, Any]:
        return self.document.editor["hierarchy"]

    @property
    def root_layer_id(self) -> str:
        return str(self.data["root_layer"])

    def layer(self, layer_id: str) -> dict[str, Any]:
        try:
            return self.data["layers"][layer_id]
        except KeyError as exc:
            raise StudioError(f"unknown editor layer {layer_id!r}") from exc

    def layer_kind(self, layer_id: str) -> str:
        return str(self.layer(layer_id)["kind"])

    def node(self, layer_id: str, node_id: str) -> dict[str, Any]:
        for node in self.layer(layer_id).get("nodes", []):
            if node["id"] == node_id:
                return node
        raise StudioError(f"unknown node {node_id!r} in layer {layer_id!r}")

    def nodes(self, layer_id: str) -> list[dict[str, Any]]:
        return self.layer(layer_id).setdefault("nodes", [])

    def node_type(self, node: Mapping[str, Any]) -> NodeType:
        try:
            return NODE_TYPES[str(node["type"])]
        except KeyError as exc:
            raise StudioError(f"unknown editor node type {node.get('type')!r}") from exc

    def _normalise(self) -> None:
        hierarchy = self.data
        if not isinstance(hierarchy.get("layers"), dict):
            raise StudioError("editor hierarchy layers must be an object")
        if hierarchy.get("root_layer") not in hierarchy["layers"]:
            raise StudioError("editor hierarchy root_layer does not exist")
        for layer_id, layer in hierarchy["layers"].items():
            if layer.get("kind") not in {"system", "circuit", "digital"}:
                raise StudioError(f"unsupported editor layer kind in {layer_id!r}")
            if layer.get("source") == "project.instances":
                continue
            nodes = layer.setdefault("nodes", [])
            connections = layer.setdefault("connections", [])
            z_order = layer.setdefault("z_order", [])
            if not all(isinstance(value, list) for value in (nodes, connections, z_order)):
                raise StudioError(f"invalid graph arrays in layer {layer_id!r}")
            ids: set[str] = set()
            ordered_ids: list[str] = []
            for node in nodes:
                node_id = str(node.get("id", ""))
                if not node_id or node_id in ids:
                    raise StudioError(f"duplicate or empty node id in layer {layer_id!r}")
                ids.add(node_id)
                ordered_ids.append(node_id)
                node_type = self.node_type(node)
                if node_type.layer_kind != layer["kind"]:
                    raise StudioError(
                        f"node {node_id} is not valid in {layer['kind']} layer"
                    )
                node.setdefault("name", node_id)
                node.setdefault("position", {"x": 100.0, "y": 100.0})
                node.setdefault("parameters", copy.deepcopy(dict(node_type.defaults or {})))
                node.setdefault("execution_order", len(ids))
                node.setdefault("rotation", 0)
                node.setdefault("mirror_x", False)
            layer["z_order"] = [node_id for node_id in z_order if node_id in ids]
            layer["z_order"].extend(
                node_id for node_id in ordered_ids if node_id not in layer["z_order"]
            )

    def available_types(self, layer_id: str) -> dict[str, NodeType]:
        kind = self.layer_kind(layer_id)
        return {
            type_id: node_type
            for type_id, node_type in NODE_TYPES.items()
            if node_type.layer_kind == kind
        }

    def _next_id(self, layer_id: str, node_type: NodeType) -> str:
        used = {node["id"].upper() for node in self.nodes(layer_id)}
        index = 1
        while f"{node_type.prefix}{index}".upper() in used:
            index += 1
        return f"{node_type.prefix}{index}"

    def add_node(self, layer_id: str, type_id: str, x: float, y: float) -> str:
        node_type = NODE_TYPES.get(type_id)
        if node_type is None or node_type.layer_kind != self.layer_kind(layer_id):
            raise StudioError(f"node type {type_id!r} is not valid in this layer")
        created: list[str] = []

        def operation() -> None:
            node_id = self._next_id(layer_id, node_type)
            node = {
                "id": node_id,
                "name": node_type.display_name,
                "type": type_id,
                "position": {"x": float(x), "y": float(y)},
                "parameters": copy.deepcopy(dict(node_type.defaults or {})),
                "execution_order": max(
                    (int(item.get("execution_order", 0)) for item in self.nodes(layer_id)),
                    default=0,
                )
                + 1,
                "rotation": 0,
                "mirror_x": False,
            }
            if node_type.child_kind:
                child_id = self._unique_layer_id(f"{node_id.lower()}_{node_type.child_kind}")
                node["child_layer"] = child_id
                self.data["layers"][child_id] = {
                    "id": child_id,
                    "name": node["name"],
                    "kind": node_type.child_kind,
                    "nodes": [],
                    "connections": [],
                    "z_order": [],
                }
            self.nodes(layer_id).append(node)
            self.layer(layer_id)["z_order"].append(node_id)
            created.append(node_id)

        self.document.change(operation)
        return created[0]

    def _unique_layer_id(self, base: str) -> str:
        layer_id = base
        index = 2
        while layer_id in self.data["layers"]:
            layer_id = f"{base}_{index}"
            index += 1
        return layer_id

    def set_positions(
        self,
        layer_id: str,
        positions: Mapping[str, tuple[float, float]],
        record: bool = True,
    ) -> None:
        def operation() -> None:
            for node_id, (x, y) in positions.items():
                node = self.node(layer_id, node_id)
                node["position"] = {"x": float(x), "y": float(y)}

        if record:
            self.document.change(operation)
        else:
            operation()

    def delete_nodes(self, layer_id: str, node_ids: Sequence[str]) -> None:
        selected = set(node_ids)
        for node_id in selected:
            node = self.node(layer_id, node_id)
            child = node.get("child_layer")
            if child and self.layer(str(child)).get("source") == "project.instances":
                raise StudioError(
                    "the schema-v1 Main Topology cannot be deleted while it owns project.instances"
                )

        def operation() -> None:
            layer = self.layer(layer_id)
            removed = [node for node in self.nodes(layer_id) if node["id"] in selected]
            layer["nodes"] = [node for node in self.nodes(layer_id) if node["id"] not in selected]
            layer["z_order"] = [node_id for node_id in layer["z_order"] if node_id not in selected]
            layer["connections"] = [
                connection
                for connection in layer["connections"]
                if connection["source"]["node"] not in selected
                and connection["target"]["node"] not in selected
            ]
            for node in removed:
                child = node.get("child_layer")
                if child:
                    self._remove_layer_tree(str(child))

        self.document.change(operation)

    def _remove_layer_tree(self, layer_id: str) -> None:
        layer = self.data["layers"].get(layer_id)
        if not layer:
            return
        for node in layer.get("nodes", []):
            if node.get("child_layer"):
                self._remove_layer_tree(str(node["child_layer"]))
        del self.data["layers"][layer_id]

    def duplicate_nodes(self, layer_id: str, node_ids: Sequence[str]) -> list[str]:
        created: list[str] = []

        def operation() -> None:
            for node_id in node_ids:
                source = self.node(layer_id, node_id)
                node_type = self.node_type(source)
                clone = copy.deepcopy(source)
                clone_id = self._next_id(layer_id, node_type)
                clone["id"] = clone_id
                clone["name"] = f"{source['name']} Copy"
                clone["position"] = {
                    "x": float(source["position"]["x"]) + 30.0,
                    "y": float(source["position"]["y"]) + 30.0,
                }
                clone.pop("child_layer", None)
                if node_type.child_kind:
                    child_id = self._unique_layer_id(f"{clone_id.lower()}_{node_type.child_kind}")
                    clone["child_layer"] = child_id
                    self.data["layers"][child_id] = {
                        "id": child_id,
                        "name": clone["name"],
                        "kind": node_type.child_kind,
                        "nodes": [],
                        "connections": [],
                        "z_order": [],
                    }
                self.nodes(layer_id).append(clone)
                self.layer(layer_id)["z_order"].append(clone_id)
                created.append(clone_id)

        self.document.change(operation)
        return created

    def update_node(
        self,
        layer_id: str,
        node_id: str,
        name: str,
        execution_order: int,
        parameters: Mapping[str, Any],
    ) -> None:
        if not name.strip():
            raise StudioError("node name must not be empty")
        if execution_order < 0:
            raise StudioError("execution order must be non-negative")

        def operation() -> None:
            node = self.node(layer_id, node_id)
            node["name"] = name.strip()
            node["execution_order"] = int(execution_order)
            node["parameters"] = copy.deepcopy(dict(parameters))
            child = node.get("child_layer")
            if child:
                self.layer(str(child))["name"] = node["name"]

        self.document.change(operation)

    def set_transform(
        self,
        layer_id: str,
        node_ids: Sequence[str],
        rotation_delta: int = 0,
        toggle_mirror: bool = False,
    ) -> None:
        def operation() -> None:
            for node_id in node_ids:
                node = self.node(layer_id, node_id)
                node["rotation"] = (int(node.get("rotation", 0)) + rotation_delta) % 360
                if node["rotation"] not in {0, 90, 180, 270}:
                    raise StudioError("component rotation must be a multiple of 90 degrees")
                if toggle_mirror:
                    node["mirror_x"] = not bool(node.get("mirror_x", False))

        self.document.change(operation)

    def port_spec(self, layer_id: str, endpoint: tuple[str, str]) -> PortSpec:
        node_type = self.node_type(self.node(layer_id, endpoint[0]))
        for port in node_type.ports:
            if port.port_id == endpoint[1]:
                return port
        raise StudioError(f"unknown port {endpoint[1]!r}")

    def connect(
        self,
        layer_id: str,
        first: tuple[str, str],
        second: tuple[str, str],
        points: Sequence[tuple[float, float]] = (),
    ) -> str:
        if first == second:
            raise StudioError("cannot connect a port to itself")
        first_port = self.port_spec(layer_id, first)
        second_port = self.port_spec(layer_id, second)
        if first_port.domain != second_port.domain:
            raise StudioError(
                f"cannot connect {first_port.domain} to {second_port.domain}"
            )
        passive_electrical = (
            first_port.domain == "electrical"
            and first_port.direction == "passive"
            and second_port.direction == "passive"
        )
        if first_port.direction == second_port.direction and not passive_electrical:
            raise StudioError("connections require one input and one output")
        source, target = (
            (first, second)
            if passive_electrical or first_port.direction == "output"
            else (second, first)
        )
        layer = self.layer(layer_id)
        for connection in layer["connections"]:
            existing_source = (
                str(connection["source"]["node"]),
                str(connection["source"]["port"]),
            )
            existing_target = (
                str(connection["target"]["node"]),
                str(connection["target"]["port"]),
            )
            same_passive_wire = passive_electrical and {
                existing_source,
                existing_target,
            } == {source, target}
            if same_passive_wire or (
                existing_source == source and existing_target == target
            ):
                return str(connection["id"])
        used_ids = {str(connection.get("id")) for connection in layer["connections"]}
        next_index = 1
        while f"wire_{next_index}" in used_ids:
            next_index += 1
        connection_id = f"wire_{next_index}"

        def operation() -> None:
            if not passive_electrical:
                layer["connections"] = [
                    connection
                    for connection in layer["connections"]
                    if connection["target"]
                    != {"node": target[0], "port": target[1]}
                ]
            layer["connections"].append(
                {
                    "id": connection_id,
                    "domain": first_port.domain,
                    "source": {"node": source[0], "port": source[1]},
                    "target": {"node": target[0], "port": target[1]},
                    "points": [
                        {"x": float(point[0]), "y": float(point[1])} for point in points
                    ],
                }
            )

        self.document.change(operation)
        return connection_id

    def connection(self, layer_id: str, connection_id: str) -> dict[str, Any]:
        for connection in self.layer(layer_id).get("connections", []):
            if connection.get("id") == connection_id:
                return connection
        raise StudioError(f"unknown wire {connection_id!r}")

    def set_wire_points(
        self,
        layer_id: str,
        connection_id: str,
        points: Sequence[tuple[float, float]],
    ) -> None:
        def operation() -> None:
            connection = self.connection(layer_id, connection_id)
            connection["points"] = [
                {"x": float(point[0]), "y": float(point[1])} for point in points
            ]

        self.document.change(operation)

    def disconnect_port(self, layer_id: str, endpoint: tuple[str, str]) -> None:
        def operation() -> None:
            layer = self.layer(layer_id)
            layer["connections"] = [
                connection
                for connection in layer["connections"]
                if connection["source"] != {"node": endpoint[0], "port": endpoint[1]}
                and connection["target"] != {"node": endpoint[0], "port": endpoint[1]}
            ]

        self.document.change(operation)

    def delete_connection(self, layer_id: str, connection_id: str) -> None:
        def operation() -> None:
            layer = self.layer(layer_id)
            layer["connections"] = [
                connection
                for connection in layer.get("connections", [])
                if connection.get("id") != connection_id
            ]

        self.document.change(operation)

    def arrange(self, layer_id: str, node_ids: Sequence[str], mode: str) -> None:
        node_ids = list(dict.fromkeys(node_ids))
        if len(node_ids) < 2:
            return
        points = {
            node_id: (
                float(self.node(layer_id, node_id)["position"]["x"]),
                float(self.node(layer_id, node_id)["position"]["y"]),
            )
            for node_id in node_ids
        }

        def operation() -> None:
            if mode in {"left", "right", "hcenter"}:
                values = [point[0] for point in points.values()]
                target = {"left": min(values), "right": max(values), "hcenter": sum(values) / len(values)}[mode]
                self.set_positions(layer_id, {node: (target, point[1]) for node, point in points.items()}, False)
            elif mode in {"top", "bottom", "vcenter"}:
                values = [point[1] for point in points.values()]
                target = {"top": min(values), "bottom": max(values), "vcenter": sum(values) / len(values)}[mode]
                self.set_positions(layer_id, {node: (point[0], target) for node, point in points.items()}, False)
            elif mode in {"distribute_h", "distribute_v"}:
                axis = 0 if mode == "distribute_h" else 1
                ordered = sorted(node_ids, key=lambda node: points[node][axis])
                first, last = points[ordered[0]][axis], points[ordered[-1]][axis]
                spacing = (last - first) / (len(ordered) - 1)
                values: dict[str, tuple[float, float]] = {}
                for index, node_id in enumerate(ordered):
                    x, y = points[node_id]
                    values[node_id] = (first + index * spacing, y) if axis == 0 else (x, first + index * spacing)
                self.set_positions(layer_id, values, False)
            else:
                raise StudioError(f"unknown arrange mode {mode!r}")

        self.document.change(operation)
