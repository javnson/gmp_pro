#!/usr/bin/env python3
"""Tk desktop application for the CCTL Studio schematic editor."""

from __future__ import annotations

import argparse
import math
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any, Callable, Mapping, Sequence

import cctl_studio as backend
from editor_model import DEFAULT_GRID_SIZE, EditorDocument


APP_NAME = "GMP CCTL Studio"
CARD_WIDTH = 150.0
CARD_MIN_HEIGHT = 78.0
PORT_SPACING = 24.0
CANVAS_BG = "#0f172a"
GRID_MINOR = "#18233a"
GRID_MAJOR = "#253451"
CARD_BG = "#1e293b"
CARD_HEADER = "#26354d"
CARD_BORDER = "#64748b"
SELECT_COLOR = "#38bdf8"
WIRE_COLOR = "#f59e0b"
PORT_COLOR = "#a7f3d0"
TEXT_PRIMARY = "#f8fafc"
TEXT_MUTED = "#94a3b8"


class ComponentPalette(ttk.Frame):
    def __init__(
        self,
        parent: tk.Misc,
        components: Mapping[str, backend.Component],
        on_drop: Callable[[str, int, int], None],
        on_add: Callable[[str], None],
    ) -> None:
        super().__init__(parent, padding=(8, 8))
        self.components = dict(components)
        self.on_drop = on_drop
        self.on_add = on_add
        self.search_var = tk.StringVar()
        self._pressed_component: str | None = None
        self._dragging = False

        ttk.Label(self, text="Components", style="PanelTitle.TLabel").pack(
            anchor="w", pady=(0, 6)
        )
        search = ttk.Entry(self, textvariable=self.search_var)
        search.pack(fill="x", pady=(0, 6))
        search.insert(0, "")
        self.tree = ttk.Treeview(self, show="tree", selectmode="browse", height=12)
        self.tree.pack(fill="both", expand=True)
        self.tree.bind("<Double-1>", self._double_click)
        self.tree.bind("<ButtonPress-1>", self._press)
        self.tree.bind("<B1-Motion>", self._motion)
        self.tree.bind("<ButtonRelease-1>", self._release)
        self.search_var.trace_add("write", lambda *_: self.refresh())
        ttk.Label(
            self,
            text="Drag to canvas or double-click to add",
            style="Hint.TLabel",
            wraplength=210,
        ).pack(anchor="w", pady=(6, 0))
        self.refresh()

    def set_components(self, components: Mapping[str, backend.Component]) -> None:
        self.components = dict(components)
        self.refresh()

    def refresh(self) -> None:
        query = self.search_var.get().strip().lower()
        self.tree.delete(*self.tree.get_children())
        groups: dict[str, list[backend.Component]] = {}
        for component in self.components.values():
            text = f"{component.display_name} {component.component_id}".lower()
            if query and query not in text:
                continue
            category = component.component_id.split(".", 1)[0].upper()
            groups.setdefault(category, []).append(component)
        for category in sorted(groups):
            parent = self.tree.insert("", "end", text=category, open=True)
            for component in sorted(groups[category], key=lambda item: item.display_name):
                self.tree.insert(
                    parent,
                    "end",
                    iid=f"component::{component.component_id}",
                    text=component.display_name,
                    values=(component.component_id,),
                )

    def _selected_component(self, event: tk.Event[Any] | None = None) -> str | None:
        item = self.tree.identify_row(event.y) if event else self.tree.focus()
        if item.startswith("component::"):
            return item.removeprefix("component::")
        return None

    def _double_click(self, event: tk.Event[Any]) -> None:
        component_id = self._selected_component(event)
        if component_id:
            self.on_add(component_id)

    def _press(self, event: tk.Event[Any]) -> None:
        self._pressed_component = self._selected_component(event)
        self._dragging = False

    def _motion(self, event: tk.Event[Any]) -> None:
        if self._pressed_component:
            self._dragging = True
            self.configure(cursor="hand2")

    def _release(self, event: tk.Event[Any]) -> None:
        self.configure(cursor="")
        if self._pressed_component and self._dragging:
            self.on_drop(self._pressed_component, event.x_root, event.y_root)
        self._pressed_component = None
        self._dragging = False


class SchematicCanvas(ttk.Frame):
    def __init__(
        self,
        parent: tk.Misc,
        document: EditorDocument,
        on_selection: Callable[[set[str]], None],
        on_change: Callable[[str], None],
        on_context: Callable[[tk.Event[Any]], None],
    ) -> None:
        super().__init__(parent)
        self.document = document
        self.on_selection = on_selection
        self.on_change = on_change
        self.on_context = on_context
        self.canvas = tk.Canvas(
            self,
            background=CANVAS_BG,
            highlightthickness=0,
            cursor="crosshair",
        )
        self.canvas.pack(fill="both", expand=True)
        view = self.document.editor.get("view", {})
        self.zoom = float(view.get("zoom", 1.0))
        self.pan_x = float(view.get("pan_x", 80.0))
        self.pan_y = float(view.get("pan_y", 60.0))
        self.selection: set[str] = set()
        self.item_instances: dict[int, str] = {}
        self.item_ports: dict[int, tuple[str, str]] = {}
        self.port_screen_positions: dict[tuple[str, str], tuple[float, float]] = {}
        self._mode: str | None = None
        self._press_xy = (0.0, 0.0)
        self._press_world = (0.0, 0.0)
        self._drag_start_positions: dict[str, tuple[float, float]] = {}
        self._change_snapshot: dict[str, Any] | None = None
        self._marquee: tuple[float, float, float, float] | None = None
        self._wire_start: tuple[str, str] | None = None
        self._wire_end_screen: tuple[float, float] | None = None
        self._space_down = False

        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.canvas.bind("<ButtonPress-1>", self._button_press)
        self.canvas.bind("<B1-Motion>", self._button_motion)
        self.canvas.bind("<ButtonRelease-1>", self._button_release)
        self.canvas.bind("<ButtonPress-2>", self._pan_press)
        self.canvas.bind("<B2-Motion>", self._pan_motion)
        self.canvas.bind("<ButtonRelease-2>", self._pan_release)
        self.canvas.bind("<Button-3>", self.on_context)
        self.canvas.bind("<MouseWheel>", self._mouse_wheel)
        self.canvas.bind("<Button-4>", lambda event: self._zoom_at(event.x, event.y, 1.1))
        self.canvas.bind("<Button-5>", lambda event: self._zoom_at(event.x, event.y, 1 / 1.1))

    def set_document(self, document: EditorDocument) -> None:
        self.document = document
        view = document.editor.get("view", {})
        self.zoom = float(view.get("zoom", 1.0))
        self.pan_x = float(view.get("pan_x", 80.0))
        self.pan_y = float(view.get("pan_y", 60.0))
        self.selection.clear()
        self.redraw()

    def set_space_down(self, value: bool) -> None:
        self._space_down = value
        self.canvas.configure(cursor="fleur" if value else "crosshair")

    def world_to_screen(self, x: float, y: float) -> tuple[float, float]:
        return x * self.zoom + self.pan_x, y * self.zoom + self.pan_y

    def screen_to_world(self, x: float, y: float) -> tuple[float, float]:
        return (x - self.pan_x) / self.zoom, (y - self.pan_y) / self.zoom

    def root_to_world(self, root_x: int, root_y: int) -> tuple[float, float] | None:
        local_x = root_x - self.canvas.winfo_rootx()
        local_y = root_y - self.canvas.winfo_rooty()
        if 0 <= local_x <= self.canvas.winfo_width() and 0 <= local_y <= self.canvas.winfo_height():
            return self.screen_to_world(local_x, local_y)
        return None

    def card_height(self, name: str) -> float:
        component = self.document.components[self.document.instance(name)["module"]]
        side_count = math.ceil(len(component.ports) / 2)
        return max(CARD_MIN_HEIGHT, 48.0 + side_count * PORT_SPACING)

    def card_bounds(self, name: str) -> tuple[float, float, float, float]:
        x, y = self.document.position(name)
        return x, y, x + CARD_WIDTH, y + self.card_height(name)

    def _port_world_position(self, endpoint: tuple[str, str]) -> tuple[float, float]:
        name, port = endpoint
        instance = self.document.instance(name)
        component = self.document.components[instance["module"]]
        index = component.ports.index(port)
        left = index % 2 == 0
        row = index // 2
        x, y = self.document.position(name)
        return (
            x if left else x + CARD_WIDTH,
            y + 50.0 + row * PORT_SPACING,
        )

    def _draw_grid(self) -> None:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        base = max(5, int(self.document.editor.get("grid_size", DEFAULT_GRID_SIZE)))
        spacing = base * self.zoom
        if spacing < 8:
            multiplier = math.ceil(8 / spacing)
            spacing *= multiplier
        start_x = self.pan_x % spacing
        start_y = self.pan_y % spacing
        index = 0
        x = start_x
        while x < width:
            world_index = round((x - self.pan_x) / spacing)
            color = GRID_MAJOR if world_index % 5 == 0 else GRID_MINOR
            self.canvas.create_line(x, 0, x, height, fill=color)
            x += spacing
            index += 1
        y = start_y
        while y < height:
            world_index = round((y - self.pan_y) / spacing)
            color = GRID_MAJOR if world_index % 5 == 0 else GRID_MINOR
            self.canvas.create_line(0, y, width, y, fill=color)
            y += spacing

    def _draw_wires(self) -> None:
        for node, endpoints in sorted(self.document.connected_networks().items()):
            if len(endpoints) < 2:
                continue
            anchor = self.world_to_screen(*self._port_world_position(endpoints[0]))
            for endpoint in endpoints[1:]:
                target = self.world_to_screen(*self._port_world_position(endpoint))
                mid_x = (anchor[0] + target[0]) / 2
                self.canvas.create_line(
                    anchor[0],
                    anchor[1],
                    mid_x,
                    anchor[1],
                    mid_x,
                    target[1],
                    target[0],
                    target[1],
                    fill=WIRE_COLOR,
                    width=max(1, int(2 * self.zoom)),
                    joinstyle="round",
                )
            if node != "0":
                self.canvas.create_text(
                    anchor[0] + 8,
                    anchor[1] - 8,
                    text=node,
                    fill="#fcd34d",
                    anchor="sw",
                    font=("Segoe UI", max(7, int(8 * self.zoom))),
                )

    def _draw_instance(self, name: str) -> None:
        instance = self.document.instance(name)
        component = self.document.components[instance["module"]]
        x1, y1, x2, y2 = self.card_bounds(name)
        sx1, sy1 = self.world_to_screen(x1, y1)
        sx2, sy2 = self.world_to_screen(x2, y2)
        selected = name in self.selection
        border = SELECT_COLOR if selected else CARD_BORDER
        width = 3 if selected else 1

        shadow = self.canvas.create_rectangle(
            sx1 + 4,
            sy1 + 5,
            sx2 + 4,
            sy2 + 5,
            fill="#080d18",
            outline="",
        )
        body = self.canvas.create_rectangle(
            sx1,
            sy1,
            sx2,
            sy2,
            fill=CARD_BG,
            outline=border,
            width=width,
        )
        header = self.canvas.create_rectangle(
            sx1,
            sy1,
            sx2,
            sy1 + 30 * self.zoom,
            fill=CARD_HEADER,
            outline="",
        )
        title = self.canvas.create_text(
            sx1 + 10 * self.zoom,
            sy1 + 15 * self.zoom,
            text=name,
            fill=TEXT_PRIMARY,
            anchor="w",
            font=("Segoe UI Semibold", max(8, int(10 * self.zoom))),
        )
        subtitle = self.canvas.create_text(
            sx1 + 10 * self.zoom,
            sy1 + 36 * self.zoom,
            text=component.display_name,
            fill=TEXT_MUTED,
            anchor="nw",
            font=("Segoe UI", max(7, int(8 * self.zoom))),
        )
        order = self.document.editor["execution_order"].get(name, 0)
        badge = self.canvas.create_oval(
            sx2 - 30 * self.zoom,
            sy1 + 6 * self.zoom,
            sx2 - 8 * self.zoom,
            sy1 + 28 * self.zoom,
            fill="#0369a1",
            outline="",
        )
        badge_text = self.canvas.create_text(
            sx2 - 19 * self.zoom,
            sy1 + 17 * self.zoom,
            text=str(order),
            fill="white",
            font=("Segoe UI Semibold", max(7, int(8 * self.zoom))),
        )
        for item in (shadow, body, header, title, subtitle, badge, badge_text):
            self.item_instances[item] = name

        for port in component.ports:
            endpoint = (name, port)
            px, py = self.world_to_screen(*self._port_world_position(endpoint))
            self.port_screen_positions[endpoint] = (px, py)
            radius = max(4, 6 * self.zoom)
            port_item = self.canvas.create_oval(
                px - radius,
                py - radius,
                px + radius,
                py + radius,
                fill=PORT_COLOR,
                outline="#064e3b",
                width=2,
            )
            port_index = component.ports.index(port)
            left = port_index % 2 == 0
            label = self.canvas.create_text(
                px + (10 if left else -10) * self.zoom,
                py,
                text=port,
                fill=TEXT_PRIMARY,
                anchor="w" if left else "e",
                font=("Segoe UI", max(7, int(8 * self.zoom))),
            )
            self.item_ports[port_item] = endpoint
            self.item_ports[label] = endpoint
            self.item_instances[label] = name

    def redraw(self) -> None:
        self.canvas.delete("all")
        self.item_instances.clear()
        self.item_ports.clear()
        self.port_screen_positions.clear()
        self._draw_grid()
        self._draw_wires()
        for name in self.document.editor["z_order"]:
            self._draw_instance(name)
        if self._marquee:
            self.canvas.create_rectangle(
                *self._marquee,
                outline=SELECT_COLOR,
                dash=(4, 3),
                fill="#082f49",
                stipple="gray50",
            )
        if self._wire_start and self._wire_end_screen:
            start = self.port_screen_positions.get(self._wire_start)
            if start:
                self.canvas.create_line(
                    start[0],
                    start[1],
                    self._wire_end_screen[0],
                    self._wire_end_screen[1],
                    fill=SELECT_COLOR,
                    width=2,
                    dash=(5, 3),
                )

    def _hit(self, x: float, y: float) -> tuple[str, Any] | None:
        items = self.canvas.find_overlapping(x - 5, y - 5, x + 5, y + 5)
        for item in reversed(items):
            if item in self.item_ports:
                return "port", self.item_ports[item]
        for item in reversed(items):
            if item in self.item_instances:
                return "instance", self.item_instances[item]
        return None

    def _button_press(self, event: tk.Event[Any]) -> None:
        self.canvas.focus_set()
        self._press_xy = (event.x, event.y)
        self._press_world = self.screen_to_world(event.x, event.y)
        if self._space_down:
            self._pan_press(event)
            return
        hit = self._hit(event.x, event.y)
        shift = bool(event.state & 0x0001)
        if hit and hit[0] == "port":
            self._mode = "wire"
            self._wire_start = hit[1]
            self._wire_end_screen = (event.x, event.y)
            self.redraw()
            return
        if hit and hit[0] == "instance":
            name = hit[1]
            if shift:
                if name in self.selection:
                    self.selection.remove(name)
                else:
                    self.selection.add(name)
            elif name not in self.selection:
                self.selection = {name}
            self.on_selection(set(self.selection))
            self._mode = "drag"
            self._change_snapshot = self.document.snapshot()
            self._drag_start_positions = {
                selected: self.document.position(selected) for selected in self.selection
            }
            self.redraw()
            return
        if not shift:
            self.selection.clear()
            self.on_selection(set())
        self._mode = "marquee"
        self._marquee = (event.x, event.y, event.x, event.y)
        self.redraw()

    def _button_motion(self, event: tk.Event[Any]) -> None:
        if self._mode == "pan":
            self._pan_motion(event)
        elif self._mode == "wire":
            self._wire_end_screen = (event.x, event.y)
            self.redraw()
        elif self._mode == "drag":
            current = self.screen_to_world(event.x, event.y)
            dx = current[0] - self._press_world[0]
            dy = current[1] - self._press_world[1]
            self.document.set_positions(
                {
                    name: (start[0] + dx, start[1] + dy)
                    for name, start in self._drag_start_positions.items()
                },
                record=False,
            )
            self.redraw()
        elif self._mode == "marquee":
            self._marquee = (
                self._press_xy[0],
                self._press_xy[1],
                event.x,
                event.y,
            )
            self.redraw()

    def _button_release(self, event: tk.Event[Any]) -> None:
        if self._mode == "pan":
            self._pan_release(event)
            return
        if self._mode == "wire" and self._wire_start:
            hit = self._hit(event.x, event.y)
            if hit and hit[0] == "port" and hit[1] != self._wire_start:
                try:
                    node = self.document.connect_ports(self._wire_start, hit[1])
                    self.on_change(f"Connected {self._wire_start} to {hit[1]} as {node}")
                except backend.StudioError as exc:
                    messagebox.showerror(APP_NAME, str(exc), parent=self)
            self._wire_start = None
            self._wire_end_screen = None
        elif self._mode == "drag" and self._change_snapshot is not None:
            if self.document.editor.get("snap_to_grid", True):
                grid = max(1, int(self.document.editor.get("grid_size", DEFAULT_GRID_SIZE)))
                self.document.set_positions(
                    {
                        name: (
                            round(self.document.position(name)[0] / grid) * grid,
                            round(self.document.position(name)[1] / grid) * grid,
                        )
                        for name in self.selection
                    },
                    record=False,
                )
            if self.document.commit_preview(self._change_snapshot):
                self.on_change(f"Moved {len(self.selection)} component(s)")
        elif self._mode == "marquee" and self._marquee:
            x1, y1, x2, y2 = self._marquee
            left, right = sorted((x1, x2))
            top, bottom = sorted((y1, y2))
            for name in self.document.editor["z_order"]:
                wx1, wy1, wx2, wy2 = self.card_bounds(name)
                sx1, sy1 = self.world_to_screen(wx1, wy1)
                sx2, sy2 = self.world_to_screen(wx2, wy2)
                if sx1 <= right and sx2 >= left and sy1 <= bottom and sy2 >= top:
                    self.selection.add(name)
            self.on_selection(set(self.selection))
        self._mode = None
        self._marquee = None
        self._change_snapshot = None
        self.redraw()

    def _pan_press(self, event: tk.Event[Any]) -> None:
        self._mode = "pan"
        self._press_xy = (event.x, event.y)
        self._pan_start = (self.pan_x, self.pan_y)
        self.canvas.configure(cursor="fleur")

    def _pan_motion(self, event: tk.Event[Any]) -> None:
        if self._mode != "pan":
            return
        self.pan_x = self._pan_start[0] + event.x - self._press_xy[0]
        self.pan_y = self._pan_start[1] + event.y - self._press_xy[1]
        self.redraw()

    def _pan_release(self, _event: tk.Event[Any]) -> None:
        if self._mode == "pan":
            self._mode = None
            self.canvas.configure(cursor="fleur" if self._space_down else "crosshair")
            self._save_view()
            self.on_change("Canvas view moved")

    def _mouse_wheel(self, event: tk.Event[Any]) -> None:
        factor = 1.1 if event.delta > 0 else 1 / 1.1
        self._zoom_at(event.x, event.y, factor)

    def _zoom_at(self, x: float, y: float, factor: float) -> None:
        world = self.screen_to_world(x, y)
        new_zoom = min(3.0, max(0.25, self.zoom * factor))
        if math.isclose(new_zoom, self.zoom):
            return
        self.zoom = new_zoom
        self.pan_x = x - world[0] * self.zoom
        self.pan_y = y - world[1] * self.zoom
        self._save_view()
        self.redraw()
        self.on_change(f"Zoom {self.zoom * 100:.0f}%")

    def _save_view(self) -> None:
        self.document.editor["view"] = {
            "zoom": round(self.zoom, 4),
            "pan_x": round(self.pan_x, 2),
            "pan_y": round(self.pan_y, 2),
        }

    def zoom_by(self, factor: float) -> None:
        self._zoom_at(
            self.canvas.winfo_width() / 2,
            self.canvas.winfo_height() / 2,
            factor,
        )

    def zoom_reset(self) -> None:
        self.zoom = 1.0
        self.pan_x = 80.0
        self.pan_y = 60.0
        self._save_view()
        self.redraw()
        self.on_change("Canvas view reset")

    def zoom_to_fit(self) -> None:
        if not self.document.instances:
            self.zoom_reset()
            return
        bounds = [self.card_bounds(instance["name"]) for instance in self.document.instances]
        left = min(item[0] for item in bounds)
        top = min(item[1] for item in bounds)
        right = max(item[2] for item in bounds)
        bottom = max(item[3] for item in bounds)
        width = max(1.0, right - left)
        height = max(1.0, bottom - top)
        canvas_width = max(1, self.canvas.winfo_width())
        canvas_height = max(1, self.canvas.winfo_height())
        self.zoom = min(2.0, max(0.25, min((canvas_width - 100) / width, (canvas_height - 100) / height)))
        self.pan_x = (canvas_width - width * self.zoom) / 2 - left * self.zoom
        self.pan_y = (canvas_height - height * self.zoom) / 2 - top * self.zoom
        self._save_view()
        self.redraw()
        self.on_change("Diagram fitted to canvas")


class Inspector(ttk.Frame):
    def __init__(
        self,
        parent: tk.Misc,
        on_apply_instance: Callable[[str, str, Mapping[str, Any], int, tuple[float, float]], None],
        on_disconnect: Callable[[tuple[str, str]], None],
        on_apply_project: Callable[[str, str, str], None],
        on_arrange: Callable[[str], None],
    ) -> None:
        super().__init__(parent, padding=(10, 8))
        self.on_apply_instance = on_apply_instance
        self.on_disconnect = on_disconnect
        self.on_apply_project = on_apply_project
        self.on_arrange = on_arrange
        self.document: EditorDocument | None = None
        self.selection: set[str] = set()
        ttk.Label(self, text="Inspector", style="PanelTitle.TLabel").pack(anchor="w")
        self.body = ttk.Frame(self)
        self.body.pack(fill="both", expand=True, pady=(8, 0))

    def show(self, document: EditorDocument, selection: set[str]) -> None:
        self.document = document
        self.selection = set(selection)
        for child in self.body.winfo_children():
            child.destroy()
        if len(selection) == 1:
            self._show_instance(next(iter(selection)))
        elif len(selection) > 1:
            self._show_multi()
        else:
            self._show_project()

    def _field(self, row: int, label: str, variable: tk.Variable) -> ttk.Entry:
        ttk.Label(self.body, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=3)
        entry = ttk.Entry(self.body, textvariable=variable)
        entry.grid(row=row, column=1, sticky="ew", pady=3)
        return entry

    def _show_instance(self, name: str) -> None:
        assert self.document is not None
        instance = self.document.instance(name)
        component = self.document.components[instance["module"]]
        x, y = self.document.position(name)
        self.body.columnconfigure(1, weight=1)
        name_var = tk.StringVar(value=name)
        x_var = tk.StringVar(value=f"{x:g}")
        y_var = tk.StringVar(value=f"{y:g}")
        order_var = tk.StringVar(value=str(self.document.editor["execution_order"].get(name, 0)))
        self._field(0, "Name", name_var)
        ttk.Label(self.body, text="Module").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Label(self.body, text=component.display_name, style="Value.TLabel").grid(
            row=1, column=1, sticky="w", pady=3
        )
        self._field(2, "X", x_var)
        self._field(3, "Y", y_var)
        self._field(4, "Exec order", order_var)
        ttk.Separator(self.body).grid(row=5, column=0, columnspan=2, sticky="ew", pady=8)
        ttk.Label(self.body, text="Parameters", style="Section.TLabel").grid(
            row=6, column=0, columnspan=2, sticky="w"
        )
        parameter_vars: dict[str, tk.StringVar] = {}
        row = 7
        for parameter in component.parameter_types:
            value = instance["parameters"].get(parameter, component.defaults.get(parameter, ""))
            variable = tk.StringVar(value=str(value))
            parameter_vars[parameter] = variable
            self._field(row, parameter, variable)
            row += 1
        ttk.Separator(self.body).grid(row=row, column=0, columnspan=2, sticky="ew", pady=8)
        row += 1
        ttk.Label(self.body, text="Ports / networks", style="Section.TLabel").grid(
            row=row, column=0, columnspan=2, sticky="w"
        )
        row += 1
        for port in component.ports:
            line = ttk.Frame(self.body)
            line.grid(row=row, column=0, columnspan=2, sticky="ew", pady=2)
            ttk.Label(line, text=port, width=9).pack(side="left")
            ttk.Label(line, text=str(instance["ports"].get(port, "")), style="Value.TLabel").pack(
                side="left", fill="x", expand=True
            )
            ttk.Button(
                line,
                text="Disconnect",
                command=lambda endpoint=(name, port): self.on_disconnect(endpoint),
            ).pack(side="right")
            row += 1

        def apply() -> None:
            try:
                typed: dict[str, Any] = {}
                for parameter, variable in parameter_vars.items():
                    value_type = component.parameter_types[parameter]
                    text = variable.get()
                    if value_type == "boolean":
                        typed[parameter] = text.strip().lower() in {"1", "true", "yes", "on"}
                    elif value_type == "integer":
                        typed[parameter] = int(text)
                    elif value_type == "number":
                        typed[parameter] = float(text)
                    else:
                        typed[parameter] = text
                self.on_apply_instance(
                    name,
                    name_var.get(),
                    typed,
                    int(order_var.get()),
                    (float(x_var.get()), float(y_var.get())),
                )
            except ValueError as exc:
                messagebox.showerror(APP_NAME, f"Invalid numeric value: {exc}", parent=self)

        ttk.Button(self.body, text="Apply", style="Accent.TButton", command=apply).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(10, 0)
        )

    def _show_multi(self) -> None:
        ttk.Label(
            self.body,
            text=f"{len(self.selection)} components selected",
            style="Value.TLabel",
        ).pack(anchor="w", pady=(0, 10))
        ttk.Label(self.body, text="Align", style="Section.TLabel").pack(anchor="w")
        align = ttk.Frame(self.body)
        align.pack(fill="x", pady=(4, 10))
        for label, mode in (("Left", "left"), ("Center", "hcenter"), ("Right", "right")):
            ttk.Button(align, text=label, command=lambda value=mode: self.on_arrange(value)).pack(
                side="left", fill="x", expand=True, padx=2
            )
        align2 = ttk.Frame(self.body)
        align2.pack(fill="x", pady=(0, 10))
        for label, mode in (("Top", "top"), ("Middle", "vcenter"), ("Bottom", "bottom")):
            ttk.Button(align2, text=label, command=lambda value=mode: self.on_arrange(value)).pack(
                side="left", fill="x", expand=True, padx=2
            )
        ttk.Label(self.body, text="Distribute", style="Section.TLabel").pack(anchor="w")
        distribute = ttk.Frame(self.body)
        distribute.pack(fill="x", pady=4)
        ttk.Button(distribute, text="Horizontal", command=lambda: self.on_arrange("distribute_h")).pack(
            side="left", fill="x", expand=True, padx=2
        )
        ttk.Button(distribute, text="Vertical", command=lambda: self.on_arrange("distribute_v")).pack(
            side="left", fill="x", expand=True, padx=2
        )

    def _show_project(self) -> None:
        assert self.document is not None
        self.body.columnconfigure(1, weight=1)
        title_var = tk.StringVar(value=str(self.document.project.get("title", "")))
        analysis = self.document.project.get("analysis", {})
        step_var = tk.StringVar(value=str(analysis.get("step", "10us")))
        stop_var = tk.StringVar(value=str(analysis.get("stop", "10ms")))
        self._field(0, "Project", title_var)
        self._field(1, "Time step", step_var)
        self._field(2, "Stop time", stop_var)
        ttk.Button(
            self.body,
            text="Apply project settings",
            style="Accent.TButton",
            command=lambda: self.on_apply_project(title_var.get(), step_var.get(), stop_var.get()),
        ).grid(row=3, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        ttk.Separator(self.body).grid(row=4, column=0, columnspan=2, sticky="ew", pady=12)
        ttk.Label(
            self.body,
            text="Select a component to edit parameters. Drag between port dots to create a network.",
            style="Hint.TLabel",
            wraplength=260,
            justify="left",
        ).grid(row=5, column=0, columnspan=2, sticky="w")


class StudioApplication:
    def __init__(self, root: tk.Tk, project_path: Path | None = None) -> None:
        self.root = root
        self.root.title(APP_NAME)
        self.root.geometry("1440x900")
        self.root.minsize(1000, 650)
        self._configure_style()
        self.components = backend.load_components([backend.BUILTIN_LIBRARY])
        self.document = EditorDocument(None, self.components)
        self.selection: set[str] = set()
        self._copied_names: list[str] = []
        self.status_var = tk.StringVar(value="Ready")

        self._build_menu()
        self._build_toolbar()
        self._build_workspace()
        self._bind_shortcuts()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        if project_path:
            self.open_path(project_path)
        else:
            self._refresh_all()

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("PanelTitle.TLabel", font=("Segoe UI Semibold", 12))
        style.configure("Section.TLabel", font=("Segoe UI Semibold", 9))
        style.configure("Hint.TLabel", foreground="#64748b")
        style.configure("Value.TLabel", foreground="#0369a1")
        style.configure("Accent.TButton", font=("Segoe UI Semibold", 9))

    def _build_menu(self) -> None:
        menu = tk.Menu(self.root)
        file_menu = tk.Menu(menu, tearoff=False)
        file_menu.add_command(label="New", accelerator="Ctrl+N", command=self.new)
        file_menu.add_command(label="Open...", accelerator="Ctrl+O", command=self.open)
        file_menu.add_separator()
        file_menu.add_command(label="Save", accelerator="Ctrl+S", command=self.save)
        file_menu.add_command(label="Save As...", accelerator="Ctrl+Shift+S", command=self.save_as)
        file_menu.add_separator()
        file_menu.add_command(label="Generate Xyce netlist...", command=self.generate_netlist)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.close)
        menu.add_cascade(label="File", menu=file_menu)

        edit_menu = tk.Menu(menu, tearoff=False)
        edit_menu.add_command(label="Undo", accelerator="Ctrl+Z", command=self.undo)
        edit_menu.add_command(label="Redo", accelerator="Ctrl+Y", command=self.redo)
        edit_menu.add_separator()
        edit_menu.add_command(label="Copy", accelerator="Ctrl+C", command=self.copy)
        edit_menu.add_command(label="Paste", accelerator="Ctrl+V", command=self.paste)
        edit_menu.add_command(label="Duplicate", accelerator="Ctrl+D", command=self.duplicate)
        edit_menu.add_command(label="Delete", accelerator="Del", command=self.delete)
        edit_menu.add_command(label="Select All", accelerator="Ctrl+A", command=self.select_all)
        menu.add_cascade(label="Edit", menu=edit_menu)

        view_menu = tk.Menu(menu, tearoff=False)
        view_menu.add_command(label="Zoom in", accelerator="Ctrl++", command=lambda: self.canvas.zoom_by(1.2))
        view_menu.add_command(label="Zoom out", accelerator="Ctrl+-", command=lambda: self.canvas.zoom_by(1 / 1.2))
        view_menu.add_command(label="Fit diagram", accelerator="F6", command=self.canvas_fit)
        view_menu.add_command(label="Reset view", command=lambda: self.canvas.zoom_reset())
        view_menu.add_separator()
        self.snap_var = tk.BooleanVar(value=True)
        view_menu.add_checkbutton(label="Snap to grid", variable=self.snap_var, command=self.toggle_snap)
        menu.add_cascade(label="View", menu=view_menu)

        arrange_menu = tk.Menu(menu, tearoff=False)
        for label, mode in (
            ("Align left", "left"),
            ("Align horizontal center", "hcenter"),
            ("Align right", "right"),
            ("Align top", "top"),
            ("Align vertical center", "vcenter"),
            ("Align bottom", "bottom"),
            ("Distribute horizontally", "distribute_h"),
            ("Distribute vertically", "distribute_v"),
        ):
            arrange_menu.add_command(label=label, command=lambda value=mode: self.arrange(value))
        arrange_menu.add_separator()
        arrange_menu.add_command(label="Bring to front", command=lambda: self.z_order(True))
        arrange_menu.add_command(label="Send to back", command=lambda: self.z_order(False))
        menu.add_cascade(label="Arrange", menu=arrange_menu)
        self.root.configure(menu=menu)

    def _build_toolbar(self) -> None:
        bar = ttk.Frame(self.root, padding=(8, 5))
        bar.pack(fill="x")
        for label, command in (
            ("New", self.new),
            ("Open", self.open),
            ("Save", self.save),
            ("Undo", self.undo),
            ("Redo", self.redo),
            ("Delete", self.delete),
            ("Fit", self.canvas_fit),
        ):
            ttk.Button(bar, text=label, command=command).pack(side="left", padx=2)
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Label(
            bar,
            text="Drag components • Drag ports to connect • Middle-drag or Space-drag to pan",
            style="Hint.TLabel",
        ).pack(side="left")

    def _build_workspace(self) -> None:
        paned = ttk.Panedwindow(self.root, orient="horizontal")
        paned.pack(fill="both", expand=True)
        left = ttk.Frame(paned, width=250)
        center = ttk.Frame(paned)
        right = ttk.Frame(paned, width=310)
        paned.add(left, weight=0)
        paned.add(center, weight=1)
        paned.add(right, weight=0)

        self.palette = ComponentPalette(left, self.components, self.drop_component, self.add_component_center)
        self.palette.pack(fill="both", expand=True)
        ttk.Separator(left).pack(fill="x")
        navigator_frame = ttk.Frame(left, padding=8)
        navigator_frame.pack(fill="both", expand=True)
        ttk.Label(navigator_frame, text="Design", style="PanelTitle.TLabel").pack(anchor="w", pady=(0, 6))
        self.navigator = ttk.Treeview(navigator_frame, show="tree", selectmode="extended", height=9)
        self.navigator.pack(fill="both", expand=True)
        self.navigator.bind("<<TreeviewSelect>>", self._navigator_select)

        self.canvas = SchematicCanvas(
            center,
            self.document,
            self.set_selection,
            self.document_changed,
            self.show_context_menu,
        )
        self.canvas.pack(fill="both", expand=True)
        self.inspector = Inspector(
            right,
            self.apply_instance,
            self.disconnect_port,
            self.apply_project,
            self.arrange,
        )
        self.inspector.pack(fill="both", expand=True)

        status = ttk.Frame(self.root, padding=(8, 3))
        status.pack(fill="x")
        ttk.Label(status, textvariable=self.status_var).pack(side="left")
        self.zoom_label = ttk.Label(status, text="100%")
        self.zoom_label.pack(side="right")

    def _bind_shortcuts(self) -> None:
        bindings = {
            "<Control-n>": self.new,
            "<Control-o>": self.open,
            "<Control-s>": self.save,
            "<Control-Shift-S>": self.save_as,
            "<Control-z>": self.undo,
            "<Control-y>": self.redo,
            "<Control-c>": self.copy,
            "<Control-v>": self.paste,
            "<Control-d>": self.duplicate,
            "<Control-a>": self.select_all,
            "<F6>": self.canvas_fit,
        }
        for sequence, callback in bindings.items():
            self.root.bind(sequence, lambda event, fn=callback: self._invoke_shortcut(event, fn))
        self.canvas.canvas.bind("<Delete>", lambda _event: (self.delete(), "break")[1])
        self.canvas.canvas.bind("<BackSpace>", lambda _event: (self.delete(), "break")[1])
        self.canvas.canvas.bind(
            "<KeyPress-space>",
            lambda _event: (self.canvas.set_space_down(True), "break")[1],
        )
        self.canvas.canvas.bind(
            "<KeyRelease-space>",
            lambda _event: (self.canvas.set_space_down(False), "break")[1],
        )
        self.root.bind("<Control-plus>", lambda _event: (self.canvas.zoom_by(1.2), "break")[1])
        self.root.bind("<Control-equal>", lambda _event: (self.canvas.zoom_by(1.2), "break")[1])
        self.root.bind("<Control-minus>", lambda _event: (self.canvas.zoom_by(1 / 1.2), "break")[1])
        for sequence, dx, dy in (
            ("<Left>", -1, 0),
            ("<Right>", 1, 0),
            ("<Up>", 0, -1),
            ("<Down>", 0, 1),
        ):
            self.canvas.canvas.bind(
                sequence,
                lambda event, x=dx, y=dy: (self.nudge(x, y, bool(event.state & 0x0001)), "break")[1],
            )

    def _invoke_shortcut(self, event: tk.Event[Any], callback: Callable[[], Any]) -> str | None:
        text_widgets = {"Entry", "TEntry", "Text", "TSpinbox", "Spinbox"}
        if event.widget.winfo_class() in text_widgets and callback in {
            self.copy,
            self.paste,
            self.select_all,
        }:
            return None
        callback()
        return "break"

    def _refresh_all(self) -> None:
        self.canvas.selection = set(self.selection)
        self.canvas.redraw()
        self._refresh_navigator()
        self.inspector.show(self.document, self.selection)
        self.snap_var.set(bool(self.document.editor.get("snap_to_grid", True)))
        self.zoom_label.configure(text=f"{self.canvas.zoom * 100:.0f}%")
        path = self.document.path.name if self.document.path else "Untitled"
        marker = " *" if self.document.dirty else ""
        self.root.title(f"{path}{marker} — {APP_NAME}")

    def _refresh_navigator(self) -> None:
        self.navigator.delete(*self.navigator.get_children())
        for name in self.document.editor["z_order"]:
            instance = self.document.instance(name)
            component = self.document.components[instance["module"]]
            self.navigator.insert("", "end", iid=f"nav::{name}", text=f"{name}  ·  {component.display_name}")
        for name in self.selection:
            iid = f"nav::{name}"
            if self.navigator.exists(iid):
                self.navigator.selection_add(iid)

    def _navigator_select(self, _event: tk.Event[Any]) -> None:
        selected = {iid.removeprefix("nav::") for iid in self.navigator.selection() if iid.startswith("nav::")}
        if selected != self.selection:
            self.set_selection(selected)

    def set_selection(self, selection: set[str]) -> None:
        self.selection = set(selection)
        self.canvas.selection = set(selection)
        self._refresh_all()

    def document_changed(self, status: str = "Modified") -> None:
        self.status_var.set(status)
        self._refresh_all()

    def add_component_center(self, component_id: str) -> None:
        world = self.canvas.screen_to_world(
            self.canvas.canvas.winfo_width() / 2,
            self.canvas.canvas.winfo_height() / 2,
        )
        self._add_component(component_id, *world)

    def drop_component(self, component_id: str, root_x: int, root_y: int) -> None:
        world = self.canvas.root_to_world(root_x, root_y)
        if world:
            self._add_component(component_id, *world)

    def _add_component(self, component_id: str, x: float, y: float) -> None:
        if self.document.editor.get("snap_to_grid", True):
            grid = int(self.document.editor.get("grid_size", DEFAULT_GRID_SIZE))
            x, y = round(x / grid) * grid, round(y / grid) * grid
        try:
            name = self.document.add_instance(component_id, x, y)
            self.selection = {name}
            self.document_changed(f"Added {name}")
        except backend.StudioError as exc:
            messagebox.showerror(APP_NAME, str(exc), parent=self.root)

    def _confirm_discard(self) -> bool:
        if not self.document.dirty:
            return True
        answer = messagebox.askyesnocancel(
            APP_NAME,
            "Save changes before closing this project?",
            parent=self.root,
        )
        if answer is None:
            return False
        if answer:
            return self.save()
        return True

    def new(self) -> None:
        if not self._confirm_discard():
            return
        self.components = backend.load_components([backend.BUILTIN_LIBRARY])
        self.document = EditorDocument(None, self.components)
        self.palette.set_components(self.components)
        self.canvas.set_document(self.document)
        self.selection.clear()
        self.status_var.set("New project")
        self._refresh_all()

    def open(self) -> None:
        if not self._confirm_discard():
            return
        selected = filedialog.askopenfilename(
            parent=self.root,
            title="Open CCTL Studio project",
            filetypes=(("CCTL project", "*.json"), ("All files", "*.*")),
        )
        if selected:
            self.open_path(Path(selected))

    def open_path(self, path: Path) -> None:
        try:
            raw = backend._read_json(path)
            libraries = backend.project_library_paths(path.resolve(), raw)
            self.components = backend.load_components(libraries)
            self.document = EditorDocument(raw, self.components, path)
        except backend.StudioError as exc:
            messagebox.showerror(APP_NAME, f"Cannot open project:\n{exc}", parent=self.root)
            return
        self.palette.set_components(self.components)
        self.canvas.set_document(self.document)
        self.selection.clear()
        self.status_var.set(f"Opened {path}")
        self._refresh_all()

    def save(self) -> bool:
        if self.document.path is None:
            return self.save_as()
        try:
            target = self.document.save()
        except backend.StudioError as exc:
            messagebox.showerror(APP_NAME, str(exc), parent=self.root)
            return False
        self.status_var.set(f"Saved {target}")
        self._refresh_all()
        return True

    def save_as(self) -> bool:
        selected = filedialog.asksaveasfilename(
            parent=self.root,
            title="Save CCTL Studio project",
            defaultextension=".json",
            filetypes=(("CCTL project", "*.json"), ("All files", "*.*")),
        )
        if not selected:
            return False
        try:
            target = self.document.save(Path(selected))
        except backend.StudioError as exc:
            messagebox.showerror(APP_NAME, str(exc), parent=self.root)
            return False
        self.status_var.set(f"Saved {target}")
        self._refresh_all()
        return True

    def undo(self) -> None:
        if self.document.undo():
            self.selection &= {instance["name"] for instance in self.document.instances}
            self.document_changed("Undo")

    def redo(self) -> None:
        if self.document.redo():
            self.selection &= {instance["name"] for instance in self.document.instances}
            self.document_changed("Redo")

    def copy(self) -> None:
        self._copied_names = list(self.selection)
        if self._copied_names:
            self.status_var.set(f"Copied {len(self._copied_names)} component(s)")

    def paste(self) -> None:
        available = {instance["name"] for instance in self.document.instances}
        sources = [name for name in self._copied_names if name in available]
        if sources:
            self.selection = set(self.document.duplicate_instances(sources))
            self.document_changed(f"Pasted {len(self.selection)} component(s)")

    def duplicate(self) -> None:
        if self.selection:
            self.selection = set(self.document.duplicate_instances(list(self.selection)))
            self.document_changed(f"Duplicated {len(self.selection)} component(s)")

    def delete(self) -> None:
        if self.selection:
            count = len(self.selection)
            self.document.delete_instances(list(self.selection))
            self.selection.clear()
            self.document_changed(f"Deleted {count} component(s)")

    def select_all(self) -> None:
        self.set_selection({instance["name"] for instance in self.document.instances})

    def arrange(self, mode: str) -> None:
        if len(self.selection) >= 2:
            self.document.arrange(list(self.selection), mode)
            self.document_changed(f"Arrange: {mode}")

    def z_order(self, front: bool) -> None:
        if self.selection:
            self.document.change_z_order(list(self.selection), front)
            self.document_changed("Changed drawing order")

    def nudge(self, dx: float, dy: float, coarse: bool = False) -> None:
        if not self.selection:
            return
        amount = int(self.document.editor.get("grid_size", DEFAULT_GRID_SIZE)) if coarse else 1
        self.document.set_positions(
            {
                name: (
                    self.document.position(name)[0] + dx * amount,
                    self.document.position(name)[1] + dy * amount,
                )
                for name in self.selection
            }
        )
        self.document_changed(f"Nudged {len(self.selection)} component(s)")

    def toggle_snap(self) -> None:
        before = self.document.snapshot()
        self.document.editor["snap_to_grid"] = bool(self.snap_var.get())
        self.document.commit_preview(before)
        self.document_changed("Snap to grid updated")

    def apply_instance(
        self,
        old_name: str,
        new_name: str,
        parameters: Mapping[str, Any],
        execution_order: int,
        position: tuple[float, float],
    ) -> None:
        try:
            if old_name != new_name:
                self.document.rename_instance(old_name, new_name)
                self.selection = {new_name}
            self.document.update_instance(new_name, parameters, execution_order, position)
            self.document_changed(f"Updated {new_name}")
        except backend.StudioError as exc:
            messagebox.showerror(APP_NAME, str(exc), parent=self.root)

    def disconnect_port(self, endpoint: tuple[str, str]) -> None:
        try:
            self.document.disconnect_port(endpoint)
            self.document_changed(f"Disconnected {endpoint[0]}.{endpoint[1]}")
        except backend.StudioError as exc:
            messagebox.showerror(APP_NAME, str(exc), parent=self.root)

    def apply_project(self, title: str, step: str, stop: str) -> None:
        before = self.document.snapshot()
        self.document.project["title"] = title.strip() or "Untitled CCTL Studio project"
        self.document.project.setdefault("analysis", {})["type"] = "tran"
        self.document.project["analysis"]["step"] = step.strip()
        self.document.project["analysis"]["stop"] = stop.strip()
        self.document.commit_preview(before)
        self.document_changed("Updated project settings")

    def show_context_menu(self, event: tk.Event[Any]) -> None:
        hit = self.canvas._hit(event.x, event.y)
        if hit and hit[0] == "instance" and hit[1] not in self.selection:
            self.set_selection({hit[1]})
        menu = tk.Menu(self.root, tearoff=False)
        menu.add_command(label="Duplicate", command=self.duplicate, state="normal" if self.selection else "disabled")
        menu.add_command(label="Delete", command=self.delete, state="normal" if self.selection else "disabled")
        menu.add_separator()
        menu.add_command(label="Bring to front", command=lambda: self.z_order(True), state="normal" if self.selection else "disabled")
        menu.add_command(label="Send to back", command=lambda: self.z_order(False), state="normal" if self.selection else "disabled")
        menu.tk_popup(event.x_root, event.y_root)

    def canvas_fit(self) -> None:
        self.canvas.zoom_to_fit()
        self._refresh_all()

    def generate_netlist(self) -> None:
        if not self.save():
            return
        target = filedialog.asksaveasfilename(
            parent=self.root,
            title="Generate Xyce netlist",
            defaultextension=".cir",
            filetypes=(("SPICE netlist", "*.cir"), ("All files", "*.*")),
        )
        if not target:
            return
        try:
            netlist = backend.generate_netlist(self.document.path or Path("project.json"))
            Path(target).write_text(netlist, encoding="utf-8", newline="\n")
            self.status_var.set(f"Generated {target}")
        except backend.StudioError as exc:
            messagebox.showerror(APP_NAME, f"Project is not ready for generation:\n{exc}", parent=self.root)

    def close(self) -> None:
        if self._confirm_discard():
            self.root.destroy()


def launch(project: Path | None = None) -> int:
    root = tk.Tk()
    StudioApplication(root, project)
    root.mainloop()
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Launch the GMP CCTL Studio graphical editor.")
    parser.add_argument("project", nargs="?", type=Path, help="optional project JSON to open")
    args = parser.parse_args(argv)
    return launch(args.project)


if __name__ == "__main__":
    raise SystemExit(main())
