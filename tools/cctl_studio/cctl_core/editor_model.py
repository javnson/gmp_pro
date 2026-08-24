"""Document model for the CCTL Studio graphical editor.

The model deliberately has no Qt dependency.  UI state lives in the optional
``editor`` member of a schema-v1 project, so existing netlist generation keeps
working while the desktop editor gains deterministic layout metadata.
"""

from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from cctl_studio import Component, IDENTIFIER_RE, StudioError, SUPPORTED_SCHEMA_VERSION


DEFAULT_GRID_SIZE = 20
DEFAULT_ANALYSIS = {"type": "tran", "step": "10us", "stop": "10ms", "start": "0"}
DEFAULT_OUTPUT = {"format": "CSV", "file": "waveforms.csv"}


def default_editor_hierarchy(legacy_source: bool = True) -> dict[str, Any]:
    """Compatibility hierarchy: one system block owns project.instances."""
    return {
        "root_layer": "system_root",
        "layers": {
            "system_root": {
                "id": "system_root",
                "name": "System",
                "kind": "system",
                "nodes": [
                    {
                        "id": "TOP1",
                        "name": "Main Topology",
                        "type": "system.electrical_topology",
                        "position": {"x": 180.0, "y": 160.0},
                        "parameters": {},
                        "execution_order": 1,
                        "child_layer": "circuit_main",
                    }
                ],
                "connections": [],
                "z_order": ["TOP1"],
            },
            "circuit_main": (
                {
                    "id": "circuit_main",
                    "name": "Main Topology",
                    "kind": "circuit",
                    "source": "project.instances",
                }
                if legacy_source
                else {
                    "id": "circuit_main",
                    "name": "Main Topology",
                    "kind": "circuit",
                    "nodes": [],
                    "connections": [],
                    "z_order": [],
                }
            ),
        },
    }


class EditorDocument:
    """Mutable project document with snapshot-based undo and redo."""

    def __init__(
        self,
        project: Mapping[str, Any] | None,
        components: Mapping[str, Component],
        path: Path | None = None,
    ) -> None:
        self.components = dict(components)
        self.path = path.resolve() if path else None
        self.project = copy.deepcopy(dict(project or self.new_project()))
        self._undo: list[dict[str, Any]] = []
        self._redo: list[dict[str, Any]] = []
        self._normalise()
        self._saved_serialisation = self._serialise_text()

    @staticmethod
    def new_project(title: str = "Untitled CCTL Studio project") -> dict[str, Any]:
        return {
            "schema_version": SUPPORTED_SCHEMA_VERSION,
            "title": title,
            "analysis": copy.deepcopy(DEFAULT_ANALYSIS),
            "instances": [],
            "probes": [],
            "output": copy.deepcopy(DEFAULT_OUTPUT),
            "editor": {
                "grid_size": DEFAULT_GRID_SIZE,
                "snap_to_grid": True,
                "positions": {},
                "execution_order": {},
                "z_order": [],
                "view": {"zoom": 1.0, "pan_x": 80.0, "pan_y": 60.0},
                "hierarchy": default_editor_hierarchy(False),
            },
        }

    @classmethod
    def load(
        cls, path: Path, components: Mapping[str, Component]
    ) -> "EditorDocument":
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except OSError as exc:
            raise StudioError(f"cannot read {path}: {exc}") from exc
        except json.JSONDecodeError as exc:
            raise StudioError(f"invalid JSON in {path}: {exc}") from exc
        if not isinstance(raw, dict):
            raise StudioError(f"top level of {path} must be a JSON object")
        return cls(raw, components, path)

    @property
    def dirty(self) -> bool:
        return self._serialise_text() != self._saved_serialisation

    @property
    def can_undo(self) -> bool:
        return bool(self._undo)

    @property
    def can_redo(self) -> bool:
        return bool(self._redo)

    @property
    def editor(self) -> dict[str, Any]:
        return self.project["editor"]

    @property
    def instances(self) -> list[dict[str, Any]]:
        return self.project["instances"]

    def _normalise(self) -> None:
        if self.project.get("schema_version") != SUPPORTED_SCHEMA_VERSION:
            raise StudioError(
                f"editor supports schema_version {SUPPORTED_SCHEMA_VERSION}; "
                f"got {self.project.get('schema_version')!r}"
            )
        if not isinstance(self.project.get("instances"), list):
            raise StudioError("project instances must be an array")
        self.project.setdefault("title", "Untitled CCTL Studio project")
        self.project.setdefault("analysis", copy.deepcopy(DEFAULT_ANALYSIS))
        self.project.setdefault("probes", [])
        self.project.setdefault("output", copy.deepcopy(DEFAULT_OUTPUT))

        editor = self.project.setdefault("editor", {})
        if not isinstance(editor, dict):
            raise StudioError("project editor metadata must be an object")
        editor.setdefault("grid_size", DEFAULT_GRID_SIZE)
        editor.setdefault("snap_to_grid", True)
        positions = editor.setdefault("positions", {})
        execution = editor.setdefault("execution_order", {})
        z_order = editor.setdefault("z_order", [])
        editor.setdefault("view", {"zoom": 1.0, "pan_x": 80.0, "pan_y": 60.0})
        hierarchy = editor.setdefault(
            "hierarchy", default_editor_hierarchy(bool(self.project.get("instances")))
        )
        if isinstance(hierarchy, dict) and isinstance(hierarchy.get("layers"), dict):
            for layer in hierarchy["layers"].values():
                if isinstance(layer, dict):
                    layer.setdefault(
                        "view", {"zoom": 1.0, "pan_x": 80.0, "pan_y": 70.0}
                    )
        if not isinstance(positions, dict) or not isinstance(execution, dict):
            raise StudioError("editor positions and execution_order must be objects")
        if not isinstance(z_order, list):
            raise StudioError("editor z_order must be an array")

        names: set[str] = set()
        for index, instance in enumerate(self.instances):
            if not isinstance(instance, dict):
                raise StudioError("each project instance must be an object")
            name = instance.get("name")
            module = instance.get("module")
            if not isinstance(name, str) or not IDENTIFIER_RE.fullmatch(name):
                raise StudioError(f"invalid instance name {name!r}")
            if name.upper() in names:
                raise StudioError(f"duplicate instance name {name!r}")
            names.add(name.upper())
            if module not in self.components:
                raise StudioError(f"unknown component {module!r} used by instance {name}")
            instance.setdefault("ports", {})
            instance.setdefault("parameters", {})
            if name not in positions:
                positions[name] = {
                    "x": 100 + (index % 4) * 200,
                    "y": 100 + (index // 4) * 140,
                }
            execution.setdefault(name, index + 1)

        actual_names = [instance["name"] for instance in self.instances]
        editor["positions"] = {
            name: positions[name] for name in actual_names if name in positions
        }
        editor["execution_order"] = {
            name: execution.get(name, index + 1)
            for index, name in enumerate(actual_names)
        }
        editor["z_order"] = [name for name in z_order if name in actual_names]
        editor["z_order"].extend(
            name for name in actual_names if name not in editor["z_order"]
        )

    def _serialise_text(self) -> str:
        return json.dumps(self.project, ensure_ascii=False, sort_keys=True)

    def snapshot(self) -> dict[str, Any]:
        return copy.deepcopy(self.project)

    def _commit(self, before: dict[str, Any]) -> bool:
        if before == self.project:
            return False
        self._undo.append(before)
        if len(self._undo) > 100:
            del self._undo[0]
        self._redo.clear()
        return True

    def change(self, operation: Callable[[], None]) -> bool:
        before = self.snapshot()
        operation()
        return self._commit(before)

    def commit_preview(self, before: dict[str, Any]) -> bool:
        """Record a change whose live preview already mutated the document."""
        return self._commit(before)

    def undo(self) -> bool:
        if not self._undo:
            return False
        self._redo.append(self.snapshot())
        self.project = self._undo.pop()
        self._normalise()
        return True

    def redo(self) -> bool:
        if not self._redo:
            return False
        self._undo.append(self.snapshot())
        self.project = self._redo.pop()
        self._normalise()
        return True

    def save(self, path: Path | None = None) -> Path:
        target = path.resolve() if path else self.path
        if target is None:
            raise StudioError("no project path selected")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            json.dumps(self.project, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
            newline="\n",
        )
        self.path = target
        self._saved_serialisation = self._serialise_text()
        return target

    def instance(self, name: str) -> dict[str, Any]:
        for instance in self.instances:
            if instance["name"] == name:
                return instance
        raise StudioError(f"unknown instance {name!r}")

    def position(self, name: str) -> tuple[float, float]:
        value = self.editor["positions"][name]
        return float(value["x"]), float(value["y"])

    def _next_instance_name(self, component: Component) -> str:
        used = {instance["name"].upper() for instance in self.instances}
        index = 1
        while f"{component.instance_prefix}{index}".upper() in used:
            index += 1
        return f"{component.instance_prefix}{index}"

    @staticmethod
    def _initial_parameter(value_type: str) -> Any:
        return {
            "boolean": False,
            "integer": 0,
            "number": 0.0,
            "spice_scalar": "1",
            "string": "value",
        }[value_type]

    @staticmethod
    def _unconnected_node(name: str, port: str) -> str:
        return f"NC_{name}_{port}"

    @staticmethod
    def _is_unconnected(node: Any) -> bool:
        return isinstance(node, str) and node.startswith("NC_")

    def add_instance(self, component_id: str, x: float, y: float) -> str:
        component = self.components.get(component_id)
        if component is None:
            raise StudioError(f"unknown component {component_id!r}")
        created: list[str] = []

        def operation() -> None:
            name = self._next_instance_name(component)
            parameters = dict(component.defaults)
            for parameter in component.required_parameters:
                parameters.setdefault(
                    parameter,
                    self._initial_parameter(component.parameter_types[parameter]),
                )
            instance = {
                "name": name,
                "module": component_id,
                "ports": {
                    port: self._unconnected_node(name, port) for port in component.ports
                },
                "parameters": parameters,
            }
            self.instances.append(instance)
            self.editor["positions"][name] = {"x": float(x), "y": float(y)}
            orders = [int(value) for value in self.editor["execution_order"].values()]
            self.editor["execution_order"][name] = max(orders, default=0) + 1
            self.editor["z_order"].append(name)
            created.append(name)

        self.change(operation)
        return created[0]

    def delete_instances(self, names: Sequence[str]) -> None:
        selected = set(names)

        def operation() -> None:
            self.project["instances"] = [
                instance for instance in self.instances if instance["name"] not in selected
            ]
            for key in ("positions", "execution_order"):
                for name in selected:
                    self.editor[key].pop(name, None)
            self.editor["z_order"] = [
                name for name in self.editor["z_order"] if name not in selected
            ]

        self.change(operation)

    def duplicate_instances(self, names: Sequence[str], offset: float = 30.0) -> list[str]:
        created: list[str] = []

        def operation() -> None:
            for source_name in names:
                source = self.instance(source_name)
                component = self.components[source["module"]]
                name = self._next_instance_name(component)
                clone = copy.deepcopy(source)
                clone["name"] = name
                clone["ports"] = {
                    port: self._unconnected_node(name, port) for port in component.ports
                }
                self.instances.append(clone)
                x, y = self.position(source_name)
                self.editor["positions"][name] = {"x": x + offset, "y": y + offset}
                self.editor["execution_order"][name] = (
                    max(
                        (int(value) for value in self.editor["execution_order"].values()),
                        default=0,
                    )
                    + 1
                )
                self.editor["z_order"].append(name)
                created.append(name)

        self.change(operation)
        return created

    def set_positions(
        self, positions: Mapping[str, tuple[float, float]], record: bool = True
    ) -> None:
        def operation() -> None:
            for name, (x, y) in positions.items():
                if name in self.editor["positions"]:
                    self.editor["positions"][name] = {"x": float(x), "y": float(y)}

        if record:
            self.change(operation)
        else:
            operation()

    def rename_instance(self, old_name: str, new_name: str) -> None:
        new_name = new_name.strip()
        instance = self.instance(old_name)
        component = self.components[instance["module"]]
        if not IDENTIFIER_RE.fullmatch(new_name):
            raise StudioError(f"invalid instance name {new_name!r}")
        if not new_name.upper().startswith(component.instance_prefix):
            raise StudioError(
                f"instance name must start with {component.instance_prefix!r}"
            )
        if any(
            item["name"].upper() == new_name.upper() and item["name"] != old_name
            for item in self.instances
        ):
            raise StudioError(f"duplicate instance name {new_name!r}")
        if old_name == new_name:
            return

        def operation() -> None:
            instance["name"] = new_name
            for key in ("positions", "execution_order"):
                self.editor[key][new_name] = self.editor[key].pop(old_name)
            self.editor["z_order"] = [
                new_name if name == old_name else name for name in self.editor["z_order"]
            ]
            for port, node in list(instance["ports"].items()):
                if self._is_unconnected(node):
                    instance["ports"][port] = self._unconnected_node(new_name, port)

        self.change(operation)

    def update_instance(
        self,
        name: str,
        parameters: Mapping[str, Any],
        execution_order: int,
        position: tuple[float, float] | None = None,
    ) -> None:
        instance = self.instance(name)
        component = self.components[instance["module"]]
        unknown = set(parameters) - set(component.parameter_types)
        if unknown:
            raise StudioError(f"unknown parameters: {', '.join(sorted(unknown))}")
        if execution_order < 0:
            raise StudioError("execution order must be non-negative")

        def operation() -> None:
            instance["parameters"] = copy.deepcopy(dict(parameters))
            self.editor["execution_order"][name] = int(execution_order)
            if position is not None:
                self.editor["positions"][name] = {
                    "x": float(position[0]),
                    "y": float(position[1]),
                }

        self.change(operation)

    def _next_net_name(self) -> str:
        nodes = {
            node
            for instance in self.instances
            for node in instance.get("ports", {}).values()
            if isinstance(node, str)
        }
        index = 1
        while f"net_{index}" in nodes:
            index += 1
        return f"net_{index}"

    def connect_ports(
        self, first: tuple[str, str], second: tuple[str, str]
    ) -> str:
        if first == second:
            raise StudioError("cannot connect a port to itself")
        first_instance = self.instance(first[0])
        second_instance = self.instance(second[0])
        try:
            first_node = first_instance["ports"][first[1]]
            second_node = second_instance["ports"][second[1]]
        except KeyError as exc:
            raise StudioError(f"unknown port {exc.args[0]!r}") from exc

        if first_node == second_node and not self._is_unconnected(first_node):
            return str(first_node)
        if self._is_unconnected(first_node) and self._is_unconnected(second_node):
            target = self._next_net_name()
            replaced: set[str] = set()
        elif self._is_unconnected(first_node):
            target = str(second_node)
            replaced = {str(first_node)}
        elif self._is_unconnected(second_node):
            target = str(first_node)
            replaced = {str(second_node)}
        else:
            target = "0" if "0" in {first_node, second_node} else str(first_node)
            replaced = {str(first_node), str(second_node)} - {target}

        def operation() -> None:
            for instance in self.instances:
                for port, node in list(instance["ports"].items()):
                    if node in replaced:
                        instance["ports"][port] = target
            first_instance["ports"][first[1]] = target
            second_instance["ports"][second[1]] = target

        self.change(operation)
        return target

    def disconnect_port(self, endpoint: tuple[str, str]) -> None:
        instance = self.instance(endpoint[0])
        if endpoint[1] not in instance["ports"]:
            raise StudioError(f"unknown port {endpoint[1]!r}")

        def operation() -> None:
            instance["ports"][endpoint[1]] = self._unconnected_node(*endpoint)

        self.change(operation)

    def arrange(self, names: Sequence[str], mode: str) -> None:
        names = [name for name in names if name in self.editor["positions"]]
        if len(names) < 2:
            return
        points = {name: self.position(name) for name in names}

        def operation() -> None:
            if mode in {"left", "right", "hcenter"}:
                values = [point[0] for point in points.values()]
                target = {
                    "left": min(values),
                    "right": max(values),
                    "hcenter": sum(values) / len(values),
                }[mode]
                self.set_positions(
                    {name: (target, point[1]) for name, point in points.items()}, False
                )
            elif mode in {"top", "bottom", "vcenter"}:
                values = [point[1] for point in points.values()]
                target = {
                    "top": min(values),
                    "bottom": max(values),
                    "vcenter": sum(values) / len(values),
                }[mode]
                self.set_positions(
                    {name: (point[0], target) for name, point in points.items()}, False
                )
            elif mode in {"distribute_h", "distribute_v"}:
                axis = 0 if mode == "distribute_h" else 1
                ordered = sorted(names, key=lambda name: points[name][axis])
                first = points[ordered[0]][axis]
                last = points[ordered[-1]][axis]
                spacing = (last - first) / (len(ordered) - 1)
                updated: dict[str, tuple[float, float]] = {}
                for index, name in enumerate(ordered):
                    x, y = points[name]
                    if axis == 0:
                        x = first + index * spacing
                    else:
                        y = first + index * spacing
                    updated[name] = (x, y)
                self.set_positions(updated, False)
            else:
                raise StudioError(f"unknown arrange mode {mode!r}")

        self.change(operation)

    def change_z_order(self, names: Sequence[str], to_front: bool) -> None:
        selected = [name for name in self.editor["z_order"] if name in names]
        if not selected:
            return

        def operation() -> None:
            remaining = [name for name in self.editor["z_order"] if name not in names]
            self.editor["z_order"] = (
                remaining + selected if to_front else selected + remaining
            )

        self.change(operation)

    def connected_networks(self) -> dict[str, list[tuple[str, str]]]:
        networks: dict[str, list[tuple[str, str]]] = {}
        for instance in self.instances:
            for port, node in instance.get("ports", {}).items():
                if isinstance(node, str) and not self._is_unconnected(node):
                    networks.setdefault(node, []).append((instance["name"], port))
        return networks
