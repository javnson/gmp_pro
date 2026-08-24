"""Two-level graphical editor used by GMP CCTL Studio.

The system layer renders numeric-signal modules as blocks.  Child layers select
their own renderer: circuit layers use schematic symbols and electrical nets;
digital layers use logic symbols and directed logic wires.
"""

from __future__ import annotations

import argparse
import math
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any, Callable, Mapping, Sequence

import cctl_studio as backend
from editor_model import DEFAULT_GRID_SIZE, EditorDocument
from hierarchy_model import HierarchyDocument, NODE_TYPES, NodeType, PortSpec


APP_NAME = "GMP CCTL Studio"
BG = "#111827"
GRID = "#1f2937"
GRID_MAJOR = "#374151"
LINE = "#9ca3af"
BLUE = "#0e5f91"
BLUE_LIGHT = "#38bdf8"
TEXT = "#f8fafc"
MUTED = "#94a3b8"
ELECTRIC = "#e5e7eb"
LOGIC = "#60a5fa"
NUMERIC = "#22d3ee"
PORT = "#d1fae5"


@dataclass(frozen=True)
class NodeView:
    node_id: str
    name: str
    type_id: str
    display_name: str
    symbol: str
    order: int
    position: tuple[float, float]
    ports: tuple[PortSpec, ...]
    parameters: Mapping[str, Any]
    child_layer: str | None = None


class LayerAdapter:
    """Uniform editing facade over semantic circuit and editor-only graphs."""

    def __init__(
        self,
        document: EditorDocument,
        hierarchy: HierarchyDocument,
        layer_id: str,
    ) -> None:
        self.document = document
        self.hierarchy = hierarchy
        self.layer_id = layer_id
        self.layer = hierarchy.layer(layer_id)
        self.kind = str(self.layer["kind"])

    @property
    def name(self) -> str:
        return str(self.layer.get("name", self.layer_id))

    @property
    def is_semantic_circuit(self) -> bool:
        return self.kind == "circuit" and self.layer.get("source") == "project.instances"

    def palette(self) -> dict[str, str]:
        if self.is_semantic_circuit:
            return {
                component_id: component.display_name
                for component_id, component in self.document.components.items()
            }
        return {
            type_id: node_type.display_name
            for type_id, node_type in self.hierarchy.available_types(self.layer_id).items()
        }

    def node_ids(self) -> list[str]:
        if self.is_semantic_circuit:
            return list(self.document.editor["z_order"])
        return list(self.layer.get("z_order", []))

    def node(self, node_id: str) -> NodeView:
        if self.is_semantic_circuit:
            instance = self.document.instance(node_id)
            component = self.document.components[instance["module"]]
            ports = tuple(
                PortSpec(port, port, "passive", "electrical") for port in component.ports
            )
            symbol = component.component_id.rsplit(".", 1)[-1]
            return NodeView(
                node_id,
                node_id,
                component.component_id,
                component.display_name,
                symbol,
                int(self.document.editor["execution_order"].get(node_id, 0)),
                self.document.position(node_id),
                ports,
                instance.get("parameters", {}),
            )
        raw = self.hierarchy.node(self.layer_id, node_id)
        node_type = self.hierarchy.node_type(raw)
        position = raw["position"]
        return NodeView(
            node_id,
            str(raw["name"]),
            node_type.type_id,
            node_type.display_name,
            node_type.symbol,
            int(raw.get("execution_order", 0)),
            (float(position["x"]), float(position["y"])),
            node_type.ports,
            raw.get("parameters", {}),
            str(raw["child_layer"]) if raw.get("child_layer") else None,
        )

    def add(self, type_id: str, x: float, y: float) -> str:
        if self.is_semantic_circuit:
            return self.document.add_instance(type_id, x, y)
        return self.hierarchy.add_node(self.layer_id, type_id, x, y)

    def set_positions(
        self,
        values: Mapping[str, tuple[float, float]],
        record: bool = True,
    ) -> None:
        if self.is_semantic_circuit:
            self.document.set_positions(values, record)
        else:
            self.hierarchy.set_positions(self.layer_id, values, record)

    def delete(self, node_ids: Sequence[str]) -> None:
        if self.is_semantic_circuit:
            self.document.delete_instances(node_ids)
        else:
            self.hierarchy.delete_nodes(self.layer_id, node_ids)

    def duplicate(self, node_ids: Sequence[str]) -> list[str]:
        if self.is_semantic_circuit:
            return self.document.duplicate_instances(node_ids)
        return self.hierarchy.duplicate_nodes(self.layer_id, node_ids)

    def connect(self, first: tuple[str, str], second: tuple[str, str]) -> str:
        if self.is_semantic_circuit:
            return self.document.connect_ports(first, second)
        return self.hierarchy.connect(self.layer_id, first, second)

    def disconnect(self, endpoint: tuple[str, str]) -> None:
        if self.is_semantic_circuit:
            self.document.disconnect_port(endpoint)
        else:
            self.hierarchy.disconnect_port(self.layer_id, endpoint)

    def connections(self) -> list[tuple[str, tuple[str, str], tuple[str, str], str]]:
        result: list[tuple[str, tuple[str, str], tuple[str, str], str]] = []
        if self.is_semantic_circuit:
            for net, endpoints in self.document.connected_networks().items():
                if len(endpoints) > 1:
                    for endpoint in endpoints[1:]:
                        result.append((net, endpoints[0], endpoint, "electrical"))
            return result
        for connection in self.layer.get("connections", []):
            source = connection["source"]
            target = connection["target"]
            result.append(
                (
                    str(connection["id"]),
                    (str(source["node"]), str(source["port"])),
                    (str(target["node"]), str(target["port"])),
                    str(connection.get("domain", "numeric")),
                )
            )
        return result

    def port_value(self, endpoint: tuple[str, str]) -> str:
        if self.is_semantic_circuit:
            return str(self.document.instance(endpoint[0])["ports"].get(endpoint[1], ""))
        wires = [
            wire_id
            for wire_id, source, target, _domain in self.connections()
            if endpoint in {source, target}
        ]
        return ", ".join(wires) if wires else "unconnected"

    def update(
        self,
        node_id: str,
        name: str,
        order: int,
        parameters: Mapping[str, Any],
        position: tuple[float, float],
    ) -> str:
        if self.is_semantic_circuit:
            if node_id != name:
                self.document.rename_instance(node_id, name)
                node_id = name
            self.document.update_instance(node_id, parameters, order, position)
            return node_id
        self.hierarchy.update_node(self.layer_id, node_id, name, order, parameters)
        self.hierarchy.set_positions(self.layer_id, {node_id: position})
        return node_id

    def arrange(self, node_ids: Sequence[str], mode: str) -> None:
        if self.is_semantic_circuit:
            self.document.arrange(node_ids, mode)
        else:
            self.hierarchy.arrange(self.layer_id, node_ids, mode)

    def view(self) -> dict[str, float]:
        return self.layer.setdefault(
            "view", {"zoom": 1.0, "pan_x": 80.0, "pan_y": 70.0}
        )


class Palette(ttk.Frame):
    def __init__(
        self,
        parent: tk.Misc,
        on_add: Callable[[str], None],
        on_drop: Callable[[str, int, int], None],
    ) -> None:
        super().__init__(parent, padding=8)
        self.on_add = on_add
        self.on_drop = on_drop
        self.entries: dict[str, str] = {}
        self.search = tk.StringVar()
        self.pressed: str | None = None
        self.dragging = False
        ttk.Label(self, text="Components", style="PanelTitle.TLabel").pack(anchor="w")
        ttk.Entry(self, textvariable=self.search).pack(fill="x", pady=(7, 6))
        self.tree = ttk.Treeview(self, show="tree", selectmode="browse", height=15)
        self.tree.pack(fill="both", expand=True)
        self.tree.bind("<Double-1>", self._double)
        self.tree.bind("<ButtonPress-1>", self._press)
        self.tree.bind("<B1-Motion>", self._motion)
        self.tree.bind("<ButtonRelease-1>", self._release)
        self.search.trace_add("write", lambda *_: self.refresh())
        ttk.Label(
            self,
            text="Drag onto the canvas or double-click",
            style="Hint.TLabel",
            wraplength=210,
        ).pack(anchor="w", pady=(6, 0))

    def set_entries(self, entries: Mapping[str, str], layer_kind: str) -> None:
        self.entries = dict(entries)
        self.layer_kind = layer_kind
        self.refresh()

    def refresh(self) -> None:
        self.tree.delete(*self.tree.get_children())
        query = self.search.get().strip().lower()
        parent = self.tree.insert("", "end", text=self.layer_kind.upper(), open=True)
        for type_id, label in sorted(self.entries.items(), key=lambda item: item[1]):
            if query and query not in f"{type_id} {label}".lower():
                continue
            self.tree.insert(parent, "end", iid=f"type::{type_id}", text=label)

    def _type_at(self, event: tk.Event[Any]) -> str | None:
        item = self.tree.identify_row(event.y)
        return item.removeprefix("type::") if item.startswith("type::") else None

    def _double(self, event: tk.Event[Any]) -> None:
        type_id = self._type_at(event)
        if type_id:
            self.on_add(type_id)

    def _press(self, event: tk.Event[Any]) -> None:
        self.pressed = self._type_at(event)
        self.dragging = False

    def _motion(self, _event: tk.Event[Any]) -> None:
        if self.pressed:
            self.dragging = True
            self.configure(cursor="hand2")

    def _release(self, event: tk.Event[Any]) -> None:
        self.configure(cursor="")
        if self.pressed and self.dragging:
            self.on_drop(self.pressed, event.x_root, event.y_root)
        self.pressed = None
        self.dragging = False


class LayerCanvas(ttk.Frame):
    def __init__(
        self,
        parent: tk.Misc,
        adapter: LayerAdapter,
        on_selection: Callable[[set[str]], None],
        on_change: Callable[[str], None],
        on_open_child: Callable[[str], None],
    ) -> None:
        super().__init__(parent)
        self.adapter = adapter
        self.on_selection = on_selection
        self.on_change = on_change
        self.on_open_child = on_open_child
        view = adapter.view()
        self.zoom = float(view.get("zoom", 1.0))
        self.pan_x = float(view.get("pan_x", 80.0))
        self.pan_y = float(view.get("pan_y", 70.0))
        self.selection: set[str] = set()
        self.items_to_nodes: dict[int, str] = {}
        self.items_to_ports: dict[int, tuple[str, str]] = {}
        self.port_positions: dict[tuple[str, str], tuple[float, float]] = {}
        self.mode: str | None = None
        self.press_screen = (0.0, 0.0)
        self.press_world = (0.0, 0.0)
        self.drag_positions: dict[str, tuple[float, float]] = {}
        self.change_snapshot: dict[str, Any] | None = None
        self.marquee: tuple[float, float, float, float] | None = None
        self.wire_start: tuple[str, str] | None = None
        self.wire_end: tuple[float, float] | None = None
        self.space_down = False
        self.canvas = tk.Canvas(self, bg=BG, highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.canvas.bind("<ButtonPress-1>", self._press)
        self.canvas.bind("<B1-Motion>", self._motion)
        self.canvas.bind("<ButtonRelease-1>", self._release)
        self.canvas.bind("<Double-1>", self._double)
        self.canvas.bind("<ButtonPress-2>", self._pan_press)
        self.canvas.bind("<B2-Motion>", self._pan_motion)
        self.canvas.bind("<ButtonRelease-2>", self._pan_release)
        self.canvas.bind("<MouseWheel>", self._wheel)

    def set_adapter(self, adapter: LayerAdapter) -> None:
        self.adapter = adapter
        view = adapter.view()
        self.zoom = float(view.get("zoom", 1.0))
        self.pan_x = float(view.get("pan_x", 80.0))
        self.pan_y = float(view.get("pan_y", 70.0))
        self.selection.clear()
        self.redraw()

    def world_to_screen(self, x: float, y: float) -> tuple[float, float]:
        return x * self.zoom + self.pan_x, y * self.zoom + self.pan_y

    def screen_to_world(self, x: float, y: float) -> tuple[float, float]:
        return (x - self.pan_x) / self.zoom, (y - self.pan_y) / self.zoom

    def root_to_world(self, x: int, y: int) -> tuple[float, float] | None:
        local_x = x - self.canvas.winfo_rootx()
        local_y = y - self.canvas.winfo_rooty()
        if 0 <= local_x <= self.canvas.winfo_width() and 0 <= local_y <= self.canvas.winfo_height():
            return self.screen_to_world(local_x, local_y)
        return None

    def node_size(self, node: NodeView) -> tuple[float, float]:
        if self.adapter.kind == "system":
            return 190.0, max(96.0, 48.0 + 24.0 * math.ceil(len(node.ports) / 2))
        if self.adapter.kind == "digital":
            return 112.0, max(64.0, 30.0 + 20.0 * max(2, len(node.ports)))
        return 130.0, 76.0

    def bounds(self, node_id: str) -> tuple[float, float, float, float]:
        node = self.adapter.node(node_id)
        width, height = self.node_size(node)
        return node.position[0], node.position[1], node.position[0] + width, node.position[1] + height

    def port_world(self, endpoint: tuple[str, str]) -> tuple[float, float]:
        node = self.adapter.node(endpoint[0])
        width, height = self.node_size(node)
        port = next(port for port in node.ports if port.port_id == endpoint[1])
        if self.adapter.kind == "circuit":
            index = list(node.ports).index(port)
            return (
                node.position[0] if index % 2 == 0 else node.position[0] + width,
                node.position[1] + height / 2,
            )
        same_side = [item for item in node.ports if item.direction == port.direction]
        index = same_side.index(port)
        y = node.position[1] + height * (index + 1) / (len(same_side) + 1)
        x = node.position[0] if port.direction == "input" else node.position[0] + width
        return x, y

    def _grid(self) -> None:
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        spacing = max(10.0, DEFAULT_GRID_SIZE * self.zoom)
        x = self.pan_x % spacing
        while x < width:
            world_index = round((x - self.pan_x) / spacing)
            self.canvas.create_line(x, 0, x, height, fill=GRID_MAJOR if world_index % 5 == 0 else GRID)
            x += spacing
        y = self.pan_y % spacing
        while y < height:
            world_index = round((y - self.pan_y) / spacing)
            self.canvas.create_line(0, y, width, y, fill=GRID_MAJOR if world_index % 5 == 0 else GRID)
            y += spacing

    def _wires(self) -> None:
        junctions: dict[tuple[float, float], int] = {}
        for wire_id, source, target, domain in self.adapter.connections():
            start = self.world_to_screen(*self.port_world(source))
            end = self.world_to_screen(*self.port_world(target))
            mid = (start[0] + end[0]) / 2
            color = ELECTRIC if domain == "electrical" else LOGIC if domain == "logic" else NUMERIC
            arrow = tk.LAST if domain in {"logic", "numeric"} else tk.NONE
            self.canvas.create_line(
                start[0], start[1], mid, start[1], mid, end[1], end[0], end[1],
                fill=color, width=2, joinstyle="round", arrow=arrow, arrowshape=(8, 9, 3),
            )
            junctions[start] = junctions.get(start, 0) + 1
            if domain == "electrical" and wire_id != "0":
                self.canvas.create_text(mid + 5, min(start[1], end[1]) - 5, text=wire_id, fill=MUTED, anchor="sw", font=("Segoe UI", 8))
        for point, count in junctions.items():
            if count > 1:
                self.canvas.create_oval(point[0] - 4, point[1] - 4, point[0] + 4, point[1] + 4, fill=ELECTRIC, outline="")

    def _register(self, items: Sequence[int], node_id: str) -> None:
        for item in items:
            self.items_to_nodes[item] = node_id

    def _ports(self, node: NodeView) -> None:
        for spec in node.ports:
            endpoint = (node.node_id, spec.port_id)
            x, y = self.world_to_screen(*self.port_world(endpoint))
            self.port_positions[endpoint] = (x, y)
            radius = max(4.0, 5.5 * self.zoom)
            dot = self.canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=PORT, outline="#065f46", width=2)
            anchor = "w" if spec.direction in {"input", "passive"} else "e"
            offset = 10 * self.zoom if anchor == "w" else -10 * self.zoom
            label = self.canvas.create_text(x + offset, y, text=spec.label, fill=TEXT, anchor=anchor, font=("Segoe UI", max(7, int(8 * self.zoom))))
            self.items_to_ports[dot] = endpoint
            self.items_to_ports[label] = endpoint
            self.items_to_nodes[label] = node.node_id

    def _system_node(self, node: NodeView) -> None:
        x1, y1 = self.world_to_screen(*node.position)
        width, height = self.node_size(node)
        x2, y2 = self.world_to_screen(node.position[0] + width, node.position[1] + height)
        selected = node.node_id in self.selection
        body = self.canvas.create_rectangle(x1, y1, x2, y2, fill="#172033", outline=BLUE_LIGHT if selected else BLUE, width=3 if selected else 2)
        header = self.canvas.create_rectangle(x1, y1, x2, y1 + 31 * self.zoom, fill=BLUE, outline="")
        title = self.canvas.create_text(x1 + 10 * self.zoom, y1 + 16 * self.zoom, text=node.name, fill=TEXT, anchor="w", font=("Segoe UI Semibold", max(8, int(10 * self.zoom))))
        kind = self.canvas.create_text(x1 + 10 * self.zoom, y1 + 43 * self.zoom, text=node.display_name, fill=MUTED, anchor="nw", font=("Segoe UI", max(7, int(8 * self.zoom))))
        badge = self.canvas.create_oval(x2 - 31 * self.zoom, y1 + 5 * self.zoom, x2 - 7 * self.zoom, y1 + 29 * self.zoom, fill="#0284c7", outline="")
        badge_text = self.canvas.create_text(x2 - 19 * self.zoom, y1 + 17 * self.zoom, text=str(node.order), fill="white", font=("Segoe UI Semibold", max(7, int(8 * self.zoom))))
        items = [body, header, title, kind, badge, badge_text]
        if node.child_layer:
            child = self.canvas.create_text(x2 - 10 * self.zoom, y2 - 9 * self.zoom, text="Double-click to open  ↳", fill=BLUE_LIGHT, anchor="se", font=("Segoe UI", max(7, int(8 * self.zoom))))
            items.append(child)
        self._register(items, node.node_id)

    def _digital_node(self, node: NodeView) -> None:
        x, y = node.position
        width, height = self.node_size(node)
        x1, y1 = self.world_to_screen(x, y)
        x2, y2 = self.world_to_screen(x + width, y + height)
        mid_y = (y1 + y2) / 2
        selected = node.node_id in self.selection
        outline = BLUE_LIGHT if selected else "#1675ae"
        items: list[int] = []
        if node.symbol == "and":
            items.append(self.canvas.create_polygon(x1 + 18, y1, (x1 + x2) / 2, y1, x2 - 18, mid_y, (x1 + x2) / 2, y2, x1 + 18, y2, x1 + 18, y1, fill=BLUE, outline=outline, width=2, smooth=True))
        elif node.symbol == "or":
            items.append(self.canvas.create_polygon(x1 + 12, y1, x2 - 18, y1 + 5, x2, mid_y, x2 - 18, y2 - 5, x1 + 12, y2, x1 + 30, mid_y, fill=BLUE, outline=outline, width=2, smooth=True))
        elif node.symbol == "not":
            items.append(self.canvas.create_polygon(x1 + 18, y1 + 5, x2 - 22, mid_y, x1 + 18, y2 - 5, fill=BLUE, outline=outline, width=2))
            items.append(self.canvas.create_oval(x2 - 25, mid_y - 5, x2 - 15, mid_y + 5, fill=BG, outline=outline, width=2))
        elif node.symbol in {"input", "output"}:
            items.append(self.canvas.create_polygon(x1 + 8, y1 + 10, x2 - 18, y1 + 10, x2 - 3, mid_y, x2 - 18, y2 - 10, x1 + 8, y2 - 10, fill=BLUE, outline=outline, width=2))
        else:
            items.append(self.canvas.create_rectangle(x1 + 12, y1 + 4, x2 - 12, y2 - 4, fill=BLUE, outline=outline, width=2))
        items.append(self.canvas.create_text((x1 + x2) / 2, y2 + 13, text=node.name, fill=TEXT, anchor="n", font=("Segoe UI", max(8, int(9 * self.zoom)))))
        self._register(items, node.node_id)

    def _circuit_node(self, node: NodeView) -> None:
        x, y = node.position
        width, height = self.node_size(node)
        left, cy = self.world_to_screen(x, y + height / 2)
        right, _ = self.world_to_screen(x + width, y + height / 2)
        center = (left + right) / 2
        top = self.world_to_screen(x, y)[1]
        bottom = self.world_to_screen(x, y + height)[1]
        color = BLUE_LIGHT if node.node_id in self.selection else ELECTRIC
        line_width = 3 if node.node_id in self.selection else 2
        items: list[int] = []
        lead = 25 * self.zoom
        items.append(self.canvas.create_line(left, cy, left + lead, cy, fill=color, width=line_width))
        items.append(self.canvas.create_line(right - lead, cy, right, cy, fill=color, width=line_width))
        if node.symbol == "resistor":
            points = [left + lead, cy]
            span = right - left - 2 * lead
            for index in range(1, 9):
                points.extend((left + lead + span * index / 9, cy + (-10 if index % 2 else 10) * self.zoom))
            points.extend((right - lead, cy))
            items.append(self.canvas.create_line(*points, fill=color, width=line_width, joinstyle="miter"))
        elif node.symbol == "capacitor":
            gap = 7 * self.zoom
            items.extend([
                self.canvas.create_line(left + lead, cy, center - gap, cy, fill=color, width=line_width),
                self.canvas.create_line(center - gap, top + 15 * self.zoom, center - gap, bottom - 15 * self.zoom, fill=color, width=line_width),
                self.canvas.create_line(center + gap, top + 15 * self.zoom, center + gap, bottom - 15 * self.zoom, fill=color, width=line_width),
                self.canvas.create_line(center + gap, cy, right - lead, cy, fill=color, width=line_width),
            ])
        elif node.symbol == "inductor":
            start = left + lead
            span = right - left - 2 * lead
            for index in range(4):
                items.append(self.canvas.create_arc(start + index * span / 4, cy - 12 * self.zoom, start + (index + 1) * span / 4, cy + 12 * self.zoom, start=0, extent=180, style=tk.ARC, outline=color, width=line_width))
        elif node.symbol == "voltage_pulse":
            radius = 24 * self.zoom
            items.append(self.canvas.create_oval(center - radius, cy - radius, center + radius, cy + radius, outline=color, width=line_width))
            items.append(self.canvas.create_text(center, cy - 9 * self.zoom, text="+", fill=color, font=("Segoe UI", max(9, int(11 * self.zoom)))))
            items.append(self.canvas.create_text(center, cy + 10 * self.zoom, text="−", fill=color, font=("Segoe UI", max(9, int(11 * self.zoom)))))
        else:
            items.append(self.canvas.create_rectangle(left + lead, top + 12 * self.zoom, right - lead, bottom - 12 * self.zoom, outline=color, width=line_width))
        items.append(self.canvas.create_text(center, bottom + 6, text=f"{node.node_id}  {node.display_name}", fill=TEXT, anchor="n", font=("Segoe UI", max(8, int(9 * self.zoom)))))
        self._register(items, node.node_id)

    def redraw(self) -> None:
        self.canvas.delete("all")
        self.items_to_nodes.clear()
        self.items_to_ports.clear()
        self.port_positions.clear()
        self._grid()
        self._wires()
        for node_id in self.adapter.node_ids():
            node = self.adapter.node(node_id)
            if self.adapter.kind == "system":
                self._system_node(node)
            elif self.adapter.kind == "digital":
                self._digital_node(node)
            else:
                self._circuit_node(node)
            self._ports(node)
        if self.marquee:
            self.canvas.create_rectangle(*self.marquee, outline=BLUE_LIGHT, dash=(5, 3), fill="#082f49", stipple="gray50")
        if self.wire_start and self.wire_end and self.wire_start in self.port_positions:
            start = self.port_positions[self.wire_start]
            self.canvas.create_line(start[0], start[1], self.wire_end[0], self.wire_end[1], fill=BLUE_LIGHT, width=2, dash=(5, 3))

    def _hit(self, x: float, y: float) -> tuple[str, Any] | None:
        items = self.canvas.find_overlapping(x - 6, y - 6, x + 6, y + 6)
        for item in reversed(items):
            if item in self.items_to_ports:
                return "port", self.items_to_ports[item]
        for item in reversed(items):
            if item in self.items_to_nodes:
                return "node", self.items_to_nodes[item]
        return None

    def _press(self, event: tk.Event[Any]) -> None:
        self.canvas.focus_set()
        self.press_screen = (event.x, event.y)
        self.press_world = self.screen_to_world(event.x, event.y)
        if self.space_down:
            self._pan_press(event)
            return
        hit = self._hit(event.x, event.y)
        shift = bool(event.state & 0x0001)
        if hit and hit[0] == "port":
            self.mode = "wire"
            self.wire_start = hit[1]
            self.wire_end = (event.x, event.y)
        elif hit and hit[0] == "node":
            node_id = hit[1]
            if shift:
                self.selection.symmetric_difference_update({node_id})
            elif node_id not in self.selection:
                self.selection = {node_id}
            self.on_selection(set(self.selection))
            self.mode = "drag"
            self.change_snapshot = self.adapter.document.snapshot()
            self.drag_positions = {item: self.adapter.node(item).position for item in self.selection}
        else:
            if not shift:
                self.selection.clear()
                self.on_selection(set())
            self.mode = "marquee"
            self.marquee = (event.x, event.y, event.x, event.y)
        self.redraw()

    def _motion(self, event: tk.Event[Any]) -> None:
        if self.mode == "pan":
            self._pan_motion(event)
        elif self.mode == "wire":
            self.wire_end = (event.x, event.y)
            self.redraw()
        elif self.mode == "drag":
            current = self.screen_to_world(event.x, event.y)
            dx, dy = current[0] - self.press_world[0], current[1] - self.press_world[1]
            self.adapter.set_positions({node: (point[0] + dx, point[1] + dy) for node, point in self.drag_positions.items()}, False)
            self.redraw()
        elif self.mode == "marquee":
            self.marquee = (self.press_screen[0], self.press_screen[1], event.x, event.y)
            self.redraw()

    def _release(self, event: tk.Event[Any]) -> None:
        if self.mode == "pan":
            self._pan_release(event)
            return
        if self.mode == "wire" and self.wire_start:
            hit = self._hit(event.x, event.y)
            if hit and hit[0] == "port" and hit[1] != self.wire_start:
                try:
                    wire = self.adapter.connect(self.wire_start, hit[1])
                    self.on_change(f"Connected {wire}")
                except backend.StudioError as exc:
                    messagebox.showerror(APP_NAME, str(exc), parent=self)
            self.wire_start = None
            self.wire_end = None
        elif self.mode == "drag" and self.change_snapshot is not None:
            grid = DEFAULT_GRID_SIZE
            self.adapter.set_positions({node: (round(self.adapter.node(node).position[0] / grid) * grid, round(self.adapter.node(node).position[1] / grid) * grid) for node in self.selection}, False)
            if self.adapter.document.commit_preview(self.change_snapshot):
                self.on_change(f"Moved {len(self.selection)} item(s)")
        elif self.mode == "marquee" and self.marquee:
            x1, y1, x2, y2 = self.marquee
            left, right = sorted((x1, x2))
            top, bottom = sorted((y1, y2))
            for node_id in self.adapter.node_ids():
                bx1, by1, bx2, by2 = self.bounds(node_id)
                sx1, sy1 = self.world_to_screen(bx1, by1)
                sx2, sy2 = self.world_to_screen(bx2, by2)
                if sx1 <= right and sx2 >= left and sy1 <= bottom and sy2 >= top:
                    self.selection.add(node_id)
            self.on_selection(set(self.selection))
        self.mode = None
        self.marquee = None
        self.change_snapshot = None
        self.redraw()

    def _double(self, event: tk.Event[Any]) -> None:
        hit = self._hit(event.x, event.y)
        if hit and hit[0] == "node" and self.adapter.node(hit[1]).child_layer:
            self.on_open_child(hit[1])

    def _pan_press(self, event: tk.Event[Any]) -> None:
        self.mode = "pan"
        self.press_screen = (event.x, event.y)
        self.pan_start = (self.pan_x, self.pan_y)
        self.canvas.configure(cursor="fleur")

    def _pan_motion(self, event: tk.Event[Any]) -> None:
        self.pan_x = self.pan_start[0] + event.x - self.press_screen[0]
        self.pan_y = self.pan_start[1] + event.y - self.press_screen[1]
        self.redraw()

    def _pan_release(self, _event: tk.Event[Any]) -> None:
        self.mode = None
        self.canvas.configure(cursor="crosshair")
        self._save_view()
        self.on_change("Canvas view moved")

    def _wheel(self, event: tk.Event[Any]) -> None:
        self.zoom_at(event.x, event.y, 1.1 if event.delta > 0 else 1 / 1.1)

    def zoom_at(self, x: float, y: float, factor: float) -> None:
        world = self.screen_to_world(x, y)
        self.zoom = min(3.0, max(0.25, self.zoom * factor))
        self.pan_x, self.pan_y = x - world[0] * self.zoom, y - world[1] * self.zoom
        self._save_view()
        self.redraw()
        self.on_change(f"Zoom {self.zoom * 100:.0f}%")

    def fit(self) -> None:
        if not self.adapter.node_ids():
            return
        boxes = [self.bounds(node) for node in self.adapter.node_ids()]
        left, top = min(box[0] for box in boxes), min(box[1] for box in boxes)
        right, bottom = max(box[2] for box in boxes), max(box[3] for box in boxes)
        width, height = right - left, bottom - top
        self.zoom = min(2.0, max(0.25, min((self.canvas.winfo_width() - 120) / max(1, width), (self.canvas.winfo_height() - 120) / max(1, height))))
        self.pan_x = (self.canvas.winfo_width() - width * self.zoom) / 2 - left * self.zoom
        self.pan_y = (self.canvas.winfo_height() - height * self.zoom) / 2 - top * self.zoom
        self._save_view()
        self.on_change("Diagram fitted")

    def _save_view(self) -> None:
        self.adapter.layer["view"] = {"zoom": round(self.zoom, 4), "pan_x": round(self.pan_x, 2), "pan_y": round(self.pan_y, 2)}


class Inspector(ttk.Frame):
    def __init__(
        self,
        parent: tk.Misc,
        on_apply: Callable[[str, str, int, Mapping[str, Any], tuple[float, float]], None],
        on_disconnect: Callable[[tuple[str, str]], None],
        on_arrange: Callable[[str], None],
    ) -> None:
        super().__init__(parent, padding=10)
        self.on_apply = on_apply
        self.on_disconnect = on_disconnect
        self.on_arrange = on_arrange
        self.adapter: LayerAdapter | None = None
        self.selection: set[str] = set()
        ttk.Label(self, text="Inspector", style="PanelTitle.TLabel").pack(anchor="w")
        self.body = ttk.Frame(self)
        self.body.pack(fill="both", expand=True, pady=(8, 0))

    def show(self, adapter: LayerAdapter, selection: set[str]) -> None:
        self.adapter, self.selection = adapter, set(selection)
        for child in self.body.winfo_children():
            child.destroy()
        if len(selection) == 1:
            self._single(next(iter(selection)))
        elif len(selection) > 1:
            self._multi()
        else:
            ttk.Label(self.body, text=f"{adapter.name}\n{adapter.kind.title()} editor", style="Value.TLabel", justify="left").pack(anchor="w")
            ttk.Label(self.body, text="Select an item to edit it. Drag from one port to another to connect.", style="Hint.TLabel", wraplength=260, justify="left").pack(anchor="w", pady=(12, 0))

    def _field(self, row: int, label: str, variable: tk.Variable) -> None:
        ttk.Label(self.body, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=3)
        ttk.Entry(self.body, textvariable=variable).grid(row=row, column=1, sticky="ew", pady=3)

    def _single(self, node_id: str) -> None:
        assert self.adapter is not None
        node = self.adapter.node(node_id)
        self.body.columnconfigure(1, weight=1)
        name = tk.StringVar(value=node.name)
        order = tk.StringVar(value=str(node.order))
        x = tk.StringVar(value=f"{node.position[0]:g}")
        y = tk.StringVar(value=f"{node.position[1]:g}")
        self._field(0, "Name", name)
        ttk.Label(self.body, text="Type").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Label(self.body, text=node.display_name, style="Value.TLabel").grid(row=1, column=1, sticky="w")
        self._field(2, "Exec order", order)
        self._field(3, "X", x)
        self._field(4, "Y", y)
        row = 5
        parameter_vars: dict[str, tk.StringVar] = {}
        if node.parameters:
            ttk.Separator(self.body).grid(row=row, column=0, columnspan=2, sticky="ew", pady=8)
            row += 1
            ttk.Label(self.body, text="Parameters", style="Section.TLabel").grid(row=row, column=0, columnspan=2, sticky="w")
            row += 1
            for key, value in node.parameters.items():
                parameter_vars[key] = tk.StringVar(value=str(value))
                self._field(row, key, parameter_vars[key])
                row += 1
        ttk.Separator(self.body).grid(row=row, column=0, columnspan=2, sticky="ew", pady=8)
        row += 1
        ttk.Label(self.body, text="Ports", style="Section.TLabel").grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1
        for port in node.ports:
            line = ttk.Frame(self.body)
            line.grid(row=row, column=0, columnspan=2, sticky="ew", pady=2)
            ttk.Label(line, text=port.label, width=8).pack(side="left")
            ttk.Label(line, text=self.adapter.port_value((node_id, port.port_id)), style="Value.TLabel").pack(side="left", fill="x", expand=True)
            ttk.Button(line, text="Disconnect", command=lambda endpoint=(node_id, port.port_id): self.on_disconnect(endpoint)).pack(side="right")
            row += 1
        if node.child_layer:
            ttk.Label(self.body, text=f"Child editor: {self.adapter.hierarchy.layer_kind(node.child_layer)}", style="Hint.TLabel").grid(row=row, column=0, columnspan=2, sticky="w", pady=(8, 0))
            row += 1

        def apply() -> None:
            try:
                values: dict[str, Any] = {}
                for key, variable in parameter_vars.items():
                    old = node.parameters[key]
                    text = variable.get()
                    values[key] = bool(text.lower() in {"true", "1", "yes"}) if isinstance(old, bool) else int(text) if isinstance(old, int) else float(text) if isinstance(old, float) else text
                self.on_apply(node_id, name.get(), int(order.get()), values, (float(x.get()), float(y.get())))
            except ValueError as exc:
                messagebox.showerror(APP_NAME, f"Invalid value: {exc}", parent=self)

        ttk.Button(self.body, text="Apply", style="Accent.TButton", command=apply).grid(row=row, column=0, columnspan=2, sticky="ew", pady=(10, 0))

    def _multi(self) -> None:
        ttk.Label(self.body, text=f"{len(self.selection)} items selected", style="Value.TLabel").pack(anchor="w", pady=(0, 10))
        for group in (
            (("Left", "left"), ("Center", "hcenter"), ("Right", "right")),
            (("Top", "top"), ("Middle", "vcenter"), ("Bottom", "bottom")),
            (("Distribute H", "distribute_h"), ("Distribute V", "distribute_v")),
        ):
            row = ttk.Frame(self.body)
            row.pack(fill="x", pady=2)
            for label, mode in group:
                ttk.Button(row, text=label, command=lambda value=mode: self.on_arrange(value)).pack(side="left", fill="x", expand=True, padx=2)


class LayeredStudioApplication:
    def __init__(self, root: tk.Tk, project: Path | None = None) -> None:
        self.root = root
        self.root.geometry("1440x900")
        self.root.minsize(1050, 680)
        self._style()
        self.components = backend.load_components([backend.BUILTIN_LIBRARY])
        self.document = EditorDocument(None, self.components)
        self.hierarchy = HierarchyDocument(self.document)
        self.layer_stack = [self.hierarchy.root_layer_id]
        self.adapter = LayerAdapter(self.document, self.hierarchy, self.layer_stack[-1])
        self.selection: set[str] = set()
        self.copied: list[str] = []
        self.status = tk.StringVar(value="Ready")
        self._menu()
        self._toolbar()
        self._workspace()
        self._bindings()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        if project:
            self.open_path(project)
        else:
            self.refresh()

    def _style(self) -> None:
        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("PanelTitle.TLabel", font=("Segoe UI Semibold", 12))
        style.configure("Section.TLabel", font=("Segoe UI Semibold", 9))
        style.configure("Hint.TLabel", foreground="#64748b")
        style.configure("Value.TLabel", foreground="#0369a1")
        style.configure("Accent.TButton", font=("Segoe UI Semibold", 9))

    def _menu(self) -> None:
        menu = tk.Menu(self.root)
        file = tk.Menu(menu, tearoff=False)
        for label, command, accelerator in (("New", self.new, "Ctrl+N"), ("Open...", self.open, "Ctrl+O"), ("Save", self.save, "Ctrl+S"), ("Save As...", self.save_as, "Ctrl+Shift+S")):
            file.add_command(label=label, command=command, accelerator=accelerator)
        file.add_separator()
        file.add_command(label="Generate Xyce netlist...", command=self.generate)
        file.add_separator()
        file.add_command(label="Exit", command=self.close)
        menu.add_cascade(label="File", menu=file)
        edit = tk.Menu(menu, tearoff=False)
        for label, command, accelerator in (("Undo", self.undo, "Ctrl+Z"), ("Redo", self.redo, "Ctrl+Y"), ("Copy", self.copy, "Ctrl+C"), ("Paste", self.paste, "Ctrl+V"), ("Duplicate", self.duplicate, "Ctrl+D"), ("Delete", self.delete, "Del"), ("Select All", self.select_all, "Ctrl+A")):
            edit.add_command(label=label, command=command, accelerator=accelerator)
        menu.add_cascade(label="Edit", menu=edit)
        view = tk.Menu(menu, tearoff=False)
        view.add_command(label="Back to parent", command=self.back, accelerator="Alt+Left")
        view.add_command(label="Fit diagram", command=self.fit, accelerator="F6")
        menu.add_cascade(label="View", menu=view)
        self.root.configure(menu=menu)

    def _toolbar(self) -> None:
        outer = ttk.Frame(self.root)
        outer.pack(fill="x")
        actions = ttk.Frame(outer, padding=(8, 5))
        actions.pack(fill="x")
        for label, command in (("← Back", self.back), ("New", self.new), ("Open", self.open), ("Save", self.save), ("Undo", self.undo), ("Redo", self.redo), ("Delete", self.delete), ("Fit", self.fit)):
            ttk.Button(actions, text=label, command=command).pack(side="left", padx=2)
        self.breadcrumb = ttk.Frame(outer, padding=(10, 4))
        self.breadcrumb.pack(fill="x")

    def _workspace(self) -> None:
        panes = ttk.Panedwindow(self.root, orient="horizontal")
        panes.pack(fill="both", expand=True)
        left, center, right = ttk.Frame(panes, width=245), ttk.Frame(panes), ttk.Frame(panes, width=310)
        panes.add(left, weight=0)
        panes.add(center, weight=1)
        panes.add(right, weight=0)
        self.palette = Palette(left, self.add_center, self.drop)
        self.palette.pack(fill="both", expand=True)
        nav_frame = ttk.Frame(left, padding=8)
        nav_frame.pack(fill="both", expand=True)
        ttk.Label(nav_frame, text="Current layer", style="PanelTitle.TLabel").pack(anchor="w")
        self.navigator = ttk.Treeview(nav_frame, show="tree", selectmode="extended", height=10)
        self.navigator.pack(fill="both", expand=True, pady=(6, 0))
        self.navigator.bind("<<TreeviewSelect>>", self.nav_select)
        self.navigator.bind("<Double-1>", lambda _event: self.open_selected_child())
        self.canvas = LayerCanvas(center, self.adapter, self.set_selection, self.changed, self.enter_child)
        self.canvas.pack(fill="both", expand=True)
        self.inspector = Inspector(right, self.apply_node, self.disconnect, self.arrange)
        self.inspector.pack(fill="both", expand=True)
        status = ttk.Frame(self.root, padding=(8, 3))
        status.pack(fill="x")
        ttk.Label(status, textvariable=self.status).pack(side="left")
        self.layer_badge = ttk.Label(status)
        self.layer_badge.pack(side="right")

    def _bindings(self) -> None:
        for sequence, callback in (("<Control-n>", self.new), ("<Control-o>", self.open), ("<Control-s>", self.save), ("<Control-z>", self.undo), ("<Control-y>", self.redo), ("<Control-d>", self.duplicate), ("<F6>", self.fit), ("<Alt-Left>", self.back)):
            self.root.bind(sequence, lambda _event, fn=callback: (fn(), "break")[1])
        self.canvas.canvas.bind("<Delete>", lambda _event: (self.delete(), "break")[1])
        self.canvas.canvas.bind("<Control-a>", lambda _event: (self.select_all(), "break")[1])
        self.canvas.canvas.bind("<Control-c>", lambda _event: (self.copy(), "break")[1])
        self.canvas.canvas.bind("<Control-v>", lambda _event: (self.paste(), "break")[1])
        self.canvas.canvas.bind("<KeyPress-space>", lambda _event: self._space(True))
        self.canvas.canvas.bind("<KeyRelease-space>", lambda _event: self._space(False))
        for sequence, dx, dy in (
            ("<Left>", -1, 0),
            ("<Right>", 1, 0),
            ("<Up>", 0, -1),
            ("<Down>", 0, 1),
        ):
            self.canvas.canvas.bind(
                sequence,
                lambda event, x=dx, y=dy: (
                    self.nudge(x, y, bool(event.state & 0x0001)),
                    "break",
                )[1],
            )

    def _space(self, value: bool) -> str:
        self.canvas.space_down = value
        self.canvas.canvas.configure(cursor="fleur" if value else "crosshair")
        return "break"

    def switch_layer(self, layer_id: str) -> None:
        self.hierarchy = HierarchyDocument(self.document)
        self.adapter = LayerAdapter(self.document, self.hierarchy, layer_id)
        self.selection.clear()
        self.canvas.set_adapter(self.adapter)
        self.refresh()

    def enter_child(self, node_id: str) -> None:
        child = self.adapter.node(node_id).child_layer
        if child:
            self.layer_stack.append(child)
            self.switch_layer(child)
            self.status.set(f"Opened {self.adapter.name} {self.adapter.kind} editor")

    def open_selected_child(self) -> None:
        if len(self.selection) == 1:
            self.enter_child(next(iter(self.selection)))

    def back(self) -> None:
        if len(self.layer_stack) > 1:
            self.layer_stack.pop()
            self.switch_layer(self.layer_stack[-1])
            self.status.set(f"Returned to {self.adapter.name}")

    def refresh(self) -> None:
        self.canvas.selection = set(self.selection)
        self.canvas.redraw()
        self.palette.set_entries(self.adapter.palette(), self.adapter.kind)
        self.navigator.delete(*self.navigator.get_children())
        for node_id in self.adapter.node_ids():
            node = self.adapter.node(node_id)
            marker = " ▸" if node.child_layer else ""
            self.navigator.insert("", "end", iid=f"node::{node_id}", text=f"{node.name} · {node.display_name}{marker}")
        for node_id in self.selection:
            if self.navigator.exists(f"node::{node_id}"):
                self.navigator.selection_add(f"node::{node_id}")
        self.inspector.show(self.adapter, self.selection)
        for child in self.breadcrumb.winfo_children():
            child.destroy()
        for index, layer_id in enumerate(self.layer_stack):
            layer = self.hierarchy.layer(layer_id)
            if index:
                ttk.Label(self.breadcrumb, text="  ›  ", style="Hint.TLabel").pack(side="left")
            ttk.Button(self.breadcrumb, text=f"{layer['name']}  [{layer['kind']}]", command=lambda i=index: self.jump(i)).pack(side="left")
        path = self.document.path.name if self.document.path else "Untitled"
        self.root.title(f"{path}{' *' if self.document.dirty else ''} — {APP_NAME}")
        self.layer_badge.configure(text=f"{self.adapter.kind.upper()} LAYER  ·  {self.canvas.zoom * 100:.0f}%")

    def jump(self, index: int) -> None:
        if 0 <= index < len(self.layer_stack) - 1:
            self.layer_stack = self.layer_stack[: index + 1]
            self.switch_layer(self.layer_stack[-1])

    def nav_select(self, _event: tk.Event[Any]) -> None:
        values = {item.removeprefix("node::") for item in self.navigator.selection() if item.startswith("node::")}
        if values != self.selection:
            self.set_selection(values)

    def set_selection(self, values: set[str]) -> None:
        self.selection = set(values)
        self.refresh()

    def changed(self, message: str) -> None:
        self.status.set(message)
        self.refresh()

    def add_center(self, type_id: str) -> None:
        x, y = self.canvas.screen_to_world(self.canvas.canvas.winfo_width() / 2, self.canvas.canvas.winfo_height() / 2)
        self.add(type_id, x, y)

    def drop(self, type_id: str, root_x: int, root_y: int) -> None:
        point = self.canvas.root_to_world(root_x, root_y)
        if point:
            self.add(type_id, *point)

    def add(self, type_id: str, x: float, y: float) -> None:
        try:
            node_id = self.adapter.add(type_id, round(x / 20) * 20, round(y / 20) * 20)
            self.selection = {node_id}
            self.changed(f"Added {node_id}")
        except backend.StudioError as exc:
            messagebox.showerror(APP_NAME, str(exc), parent=self.root)

    def arrange(self, mode: str) -> None:
        if len(self.selection) > 1:
            self.adapter.arrange(list(self.selection), mode)
            self.changed(f"Arrange: {mode}")

    def nudge(self, dx: int, dy: int, coarse: bool = False) -> None:
        if not self.selection:
            return
        amount = DEFAULT_GRID_SIZE if coarse else 1
        self.adapter.set_positions(
            {
                node_id: (
                    self.adapter.node(node_id).position[0] + dx * amount,
                    self.adapter.node(node_id).position[1] + dy * amount,
                )
                for node_id in self.selection
            }
        )
        self.changed(f"Nudged {len(self.selection)} item(s)")

    def apply_node(self, node_id: str, name: str, order: int, parameters: Mapping[str, Any], position: tuple[float, float]) -> None:
        try:
            new_id = self.adapter.update(node_id, name, order, parameters, position)
            self.selection = {new_id}
            self.changed(f"Updated {name}")
        except backend.StudioError as exc:
            messagebox.showerror(APP_NAME, str(exc), parent=self.root)

    def disconnect(self, endpoint: tuple[str, str]) -> None:
        self.adapter.disconnect(endpoint)
        self.changed(f"Disconnected {endpoint[0]}.{endpoint[1]}")

    def delete(self) -> None:
        if self.selection:
            try:
                count = len(self.selection)
                self.adapter.delete(list(self.selection))
                self.selection.clear()
                self.changed(f"Deleted {count} item(s)")
            except backend.StudioError as exc:
                messagebox.showerror(APP_NAME, str(exc), parent=self.root)

    def duplicate(self) -> None:
        if self.selection:
            self.selection = set(self.adapter.duplicate(list(self.selection)))
            self.changed(f"Duplicated {len(self.selection)} item(s)")

    def copy(self) -> None:
        self.copied = list(self.selection)

    def paste(self) -> None:
        available = set(self.adapter.node_ids())
        source = [node for node in self.copied if node in available]
        if source:
            self.selection = set(self.adapter.duplicate(source))
            self.changed(f"Pasted {len(self.selection)} item(s)")

    def select_all(self) -> None:
        self.set_selection(set(self.adapter.node_ids()))

    def undo(self) -> None:
        if self.document.undo():
            self._recover_layer()
            self.changed("Undo")

    def redo(self) -> None:
        if self.document.redo():
            self._recover_layer()
            self.changed("Redo")

    def _recover_layer(self) -> None:
        self.hierarchy = HierarchyDocument(self.document)
        valid = self.hierarchy.data["layers"]
        self.layer_stack = [layer for layer in self.layer_stack if layer in valid]
        if not self.layer_stack:
            self.layer_stack = [self.hierarchy.root_layer_id]
        self.switch_layer(self.layer_stack[-1])

    def fit(self) -> None:
        self.canvas.fit()
        self.refresh()

    def _confirm(self) -> bool:
        if not self.document.dirty:
            return True
        answer = messagebox.askyesnocancel(APP_NAME, "Save changes first?", parent=self.root)
        return False if answer is None else self.save() if answer else True

    def new(self) -> None:
        if not self._confirm():
            return
        self.components = backend.load_components([backend.BUILTIN_LIBRARY])
        self.document = EditorDocument(None, self.components)
        self.hierarchy = HierarchyDocument(self.document)
        self.layer_stack = [self.hierarchy.root_layer_id]
        self.switch_layer(self.layer_stack[-1])
        self.status.set("New project")

    def open(self) -> None:
        if not self._confirm():
            return
        selected = filedialog.askopenfilename(parent=self.root, filetypes=(("CCTL project", "*.json"), ("All files", "*.*")))
        if selected:
            self.open_path(Path(selected))

    def open_path(self, path: Path) -> None:
        try:
            raw = backend._read_json(path)
            self.components = backend.load_components(backend.project_library_paths(path.resolve(), raw))
            self.document = EditorDocument(raw, self.components, path)
            self.hierarchy = HierarchyDocument(self.document)
        except backend.StudioError as exc:
            messagebox.showerror(APP_NAME, str(exc), parent=self.root)
            return
        self.layer_stack = [self.hierarchy.root_layer_id]
        self.switch_layer(self.layer_stack[-1])
        self.status.set(f"Opened {path}")

    def save(self) -> bool:
        if self.document.path is None:
            return self.save_as()
        self.document.save()
        self.changed(f"Saved {self.document.path}")
        return True

    def save_as(self) -> bool:
        selected = filedialog.asksaveasfilename(parent=self.root, defaultextension=".json", filetypes=(("CCTL project", "*.json"),))
        if not selected:
            return False
        self.document.save(Path(selected))
        self.changed(f"Saved {selected}")
        return True

    def generate(self) -> None:
        if not self.save():
            return
        selected = filedialog.asksaveasfilename(parent=self.root, defaultextension=".cir", filetypes=(("SPICE netlist", "*.cir"),))
        if selected:
            try:
                Path(selected).write_text(backend.generate_netlist(self.document.path), encoding="utf-8", newline="\n")
                self.status.set(f"Generated {selected}")
            except backend.StudioError as exc:
                messagebox.showerror(APP_NAME, str(exc), parent=self.root)

    def close(self) -> None:
        if self._confirm():
            self.root.destroy()


def launch(project: Path | None = None) -> int:
    root = tk.Tk()
    LayeredStudioApplication(root, project)
    root.mainloop()
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Launch the two-level GMP CCTL Studio editor.")
    parser.add_argument("project", nargs="?", type=Path)
    args = parser.parse_args(argv)
    return launch(args.project)


if __name__ == "__main__":
    raise SystemExit(main())
