#!/usr/bin/env python3
"""Qt implementation of the two-level GMP CCTL Studio editor."""

from __future__ import annotations

import argparse
import copy
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

from PyQt5 import QtCore, QtGui, QtWidgets

import cctl_studio as backend
from component_catalog import MNA_COMPONENTS, MnaComponentSpec, ParameterSpec
from editor_model import DEFAULT_GRID_SIZE, EditorDocument
from hierarchy_model import HierarchyDocument, NODE_TYPES, NodeType, PortSpec
from mna_export import export_mna_netlist


APP_NAME = "GMP CCTL Studio"
MIME_COMPONENT = "application/x-gmp-cctl-component"
COLOR_BG = QtGui.QColor("#10151d")
COLOR_GRID = QtGui.QColor("#1d2733")
COLOR_GRID_MAJOR = QtGui.QColor("#334155")
COLOR_TEXT = QtGui.QColor("#e5e7eb")
COLOR_MUTED = QtGui.QColor("#94a3b8")
COLOR_BLUE = QtGui.QColor("#1475a8")
COLOR_SELECT = QtGui.QColor("#38bdf8")
COLOR_WIRE = QtGui.QColor("#d1d5db")
COLOR_LOGIC = QtGui.QColor("#60a5fa")
COLOR_NUMERIC = QtGui.QColor("#22d3ee")
COLOR_PORT = QtGui.QColor("#6ee7b7")


@dataclass(frozen=True)
class NodeData:
    node_id: str
    name: str
    type_id: str
    display_name: str
    symbol: str
    ports: tuple[PortSpec, ...]
    parameters: Mapping[str, Any]
    position: QtCore.QPointF
    rotation: int
    mirror_x: bool
    execution_order: int
    child_layer: str | None = None


@dataclass(frozen=True)
class WireData:
    wire_id: str
    source: tuple[str, str]
    target: tuple[str, str]
    domain: str
    points: tuple[QtCore.QPointF, ...]
    editable: bool = True


class LayerAdapter:
    """Uniform Qt-facing facade over hierarchy and legacy circuit data."""

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
    def legacy_circuit(self) -> bool:
        return self.kind == "circuit" and self.layer.get("source") == "project.instances"

    def palette(self) -> dict[str, str]:
        if self.kind == "circuit" and not self.legacy_circuit:
            return {type_id: spec.display_name for type_id, spec in MNA_COMPONENTS.items()}
        if self.legacy_circuit:
            return {
                component_id: component.display_name
                for component_id, component in self.document.components.items()
            }
        return {
            type_id: node_type.display_name
            for type_id, node_type in self.hierarchy.available_types(self.layer_id).items()
        }

    def node_ids(self) -> list[str]:
        if self.legacy_circuit:
            return list(self.document.editor["z_order"])
        return list(self.layer.get("z_order", []))

    def _legacy_symbol(self, component_id: str) -> str:
        return {
            "spice.resistor": "resistor",
            "spice.capacitor": "capacitor",
            "spice.inductor": "inductor",
            "spice.voltage_pulse": "voltage_source",
        }.get(component_id, "block")

    def node(self, node_id: str) -> NodeData:
        if self.legacy_circuit:
            instance = self.document.instance(node_id)
            component = self.document.components[instance["module"]]
            transform = self.layer.setdefault("transforms", {}).setdefault(
                node_id, {"rotation": 0, "mirror_x": False}
            )
            return NodeData(
                node_id,
                node_id,
                component.component_id,
                component.display_name,
                self._legacy_symbol(component.component_id),
                tuple(
                    PortSpec(port, port, "passive", "electrical")
                    for port in component.ports
                ),
                instance.get("parameters", {}),
                QtCore.QPointF(*self.document.position(node_id)),
                int(transform.get("rotation", 0)),
                bool(transform.get("mirror_x", False)),
                int(self.document.editor["execution_order"].get(node_id, 0)),
            )
        raw = self.hierarchy.node(self.layer_id, node_id)
        node_type = self.hierarchy.node_type(raw)
        position = raw["position"]
        return NodeData(
            node_id,
            str(raw.get("name", node_id)),
            node_type.type_id,
            node_type.display_name,
            node_type.symbol,
            node_type.ports,
            raw.get("parameters", {}),
            QtCore.QPointF(float(position["x"]), float(position["y"])),
            int(raw.get("rotation", 0)),
            bool(raw.get("mirror_x", False)),
            int(raw.get("execution_order", 0)),
            str(raw["child_layer"]) if raw.get("child_layer") else None,
        )

    def parameter_specs(self, node_id: str) -> tuple[ParameterSpec, ...]:
        node = self.node(node_id)
        if node.type_id in MNA_COMPONENTS:
            return MNA_COMPONENTS[node.type_id].parameters
        if self.legacy_circuit:
            component = self.document.components[node.type_id]
            return tuple(
                ParameterSpec(name, name.replace("_", " ").title(), value)
                for name, value in node.parameters.items()
            )
        return tuple(
            ParameterSpec(name, name.replace("_", " ").title(), value)
            for name, value in node.parameters.items()
        )

    def add_node(self, type_id: str, position: QtCore.QPointF) -> str:
        if self.legacy_circuit:
            return self.document.add_instance(type_id, position.x(), position.y())
        return self.hierarchy.add_node(
            self.layer_id, type_id, position.x(), position.y()
        )

    def set_position(
        self, node_id: str, position: QtCore.QPointF, record: bool = True
    ) -> None:
        values = {node_id: (position.x(), position.y())}
        if self.legacy_circuit:
            self.document.set_positions(values, record)
        else:
            self.hierarchy.set_positions(self.layer_id, values, record)

    def update_node(
        self, node_id: str, name: str, parameters: Mapping[str, Any], order: int
    ) -> str:
        if self.legacy_circuit:
            if name != node_id:
                self.document.rename_instance(node_id, name)
                node_id = name
            self.document.update_instance(node_id, parameters, order)
            return node_id
        self.hierarchy.update_node(self.layer_id, node_id, name, order, parameters)
        return node_id

    def delete_nodes(self, node_ids: Sequence[str]) -> None:
        if self.legacy_circuit:
            self.document.delete_instances(node_ids)
        else:
            self.hierarchy.delete_nodes(self.layer_id, node_ids)

    def duplicate_nodes(self, node_ids: Sequence[str]) -> list[str]:
        if self.legacy_circuit:
            return self.document.duplicate_instances(node_ids)
        return self.hierarchy.duplicate_nodes(self.layer_id, node_ids)

    def arrange_nodes(self, node_ids: Sequence[str], mode: str) -> None:
        if self.legacy_circuit:
            self.document.arrange(node_ids, mode)
        else:
            self.hierarchy.arrange(self.layer_id, node_ids, mode)

    def transform_nodes(
        self, node_ids: Sequence[str], rotation_delta: int = 0, mirror: bool = False
    ) -> None:
        if self.legacy_circuit:
            def operation() -> None:
                transforms = self.layer.setdefault("transforms", {})
                for node_id in node_ids:
                    value = transforms.setdefault(
                        node_id, {"rotation": 0, "mirror_x": False}
                    )
                    value["rotation"] = (
                        int(value.get("rotation", 0)) + rotation_delta
                    ) % 360
                    if mirror:
                        value["mirror_x"] = not bool(value.get("mirror_x", False))

            self.document.change(operation)
        else:
            self.hierarchy.set_transform(
                self.layer_id, node_ids, rotation_delta, mirror
            )

    def connect(
        self,
        source: tuple[str, str],
        target: tuple[str, str],
        points: Sequence[QtCore.QPointF],
    ) -> str:
        if self.legacy_circuit:
            return self.document.connect_ports(source, target)
        return self.hierarchy.connect(
            self.layer_id,
            source,
            target,
            [(point.x(), point.y()) for point in points],
        )

    def wires(self) -> list[WireData]:
        if self.legacy_circuit:
            result: list[WireData] = []
            for net, endpoints in self.document.connected_networks().items():
                if len(endpoints) > 1:
                    for index, endpoint in enumerate(endpoints[1:], start=1):
                        result.append(
                            WireData(
                                f"legacy:{net}:{index}",
                                endpoints[0],
                                endpoint,
                                "electrical",
                                (),
                                False,
                            )
                        )
            return result
        result = []
        for connection in self.layer.get("connections", []):
            source, target = connection["source"], connection["target"]
            result.append(
                WireData(
                    str(connection["id"]),
                    (str(source["node"]), str(source["port"])),
                    (str(target["node"]), str(target["port"])),
                    str(connection.get("domain", "electrical")),
                    tuple(
                        QtCore.QPointF(float(point["x"]), float(point["y"]))
                        for point in connection.get("points", [])
                    ),
                    True,
                )
            )
        return result

    def set_wire_points(
        self, wire_id: str, points: Sequence[QtCore.QPointF]
    ) -> None:
        if not self.legacy_circuit:
            self.hierarchy.set_wire_points(
                self.layer_id,
                wire_id,
                [(point.x(), point.y()) for point in points],
            )

    def delete_wire(self, wire_id: str) -> None:
        if not self.legacy_circuit:
            self.hierarchy.delete_connection(self.layer_id, wire_id)

    def port_value(self, endpoint: tuple[str, str]) -> str:
        if self.legacy_circuit:
            return str(
                self.document.instance(endpoint[0])["ports"].get(endpoint[1], "")
            )
        wire_ids = [
            wire.wire_id
            for wire in self.wires()
            if endpoint in {wire.source, wire.target}
        ]
        return ", ".join(wire_ids) if wire_ids else "Unconnected"


class ComponentList(QtWidgets.QListWidget):
    def startDrag(self, supported_actions: QtCore.Qt.DropActions) -> None:
        item = self.currentItem()
        if item is None:
            return
        mime = QtCore.QMimeData()
        mime.setData(MIME_COMPONENT, str(item.data(QtCore.Qt.UserRole)).encode("utf-8"))
        drag = QtGui.QDrag(self)
        drag.setMimeData(mime)
        drag.exec_(QtCore.Qt.CopyAction)


class VertexHandle(QtWidgets.QGraphicsEllipseItem):
    def __init__(self, wire: "WireItem", index: int, point: QtCore.QPointF) -> None:
        super().__init__(-5, -5, 10, 10, wire)
        self.wire = wire
        self.index = index
        self.setPos(point)
        self.setBrush(QtGui.QBrush(COLOR_SELECT))
        self.setPen(QtGui.QPen(QtGui.QColor("#082f49"), 1))
        self.setFlag(self.ItemIsMovable)
        self.setFlag(self.ItemSendsGeometryChanges)
        self.setCursor(QtCore.Qt.SizeAllCursor)

    def itemChange(self, change: "QtWidgets.QGraphicsItem.GraphicsItemChange", value: Any) -> Any:
        if change == self.ItemPositionChange and self.scene() is not None:
            point = self.wire.scene_ref.snap_point(value)
            self.wire.preview_vertex(self.index, point)
            return point
        return super().itemChange(change, value)

    def mouseReleaseEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:
        super().mouseReleaseEvent(event)
        self.wire.commit_vertices()


class WireItem(QtWidgets.QGraphicsPathItem):
    def __init__(self, scene: "SchematicScene", data: WireData) -> None:
        super().__init__()
        self.scene_ref = scene
        self.data = data
        self.points = list(data.points)
        self.handles: list[VertexHandle] = []
        self.setFlag(self.ItemIsSelectable, data.editable)
        self.setZValue(-2)
        self.refresh()

    def shape(self) -> QtGui.QPainterPath:
        stroker = QtGui.QPainterPathStroker()
        stroker.setWidth(12)
        return stroker.createStroke(self.path())

    def color(self) -> QtGui.QColor:
        if self.isSelected():
            return COLOR_SELECT
        if self.data.domain == "logic":
            return COLOR_LOGIC
        if self.data.domain == "numeric":
            return COLOR_NUMERIC
        return COLOR_WIRE

    def full_points(self) -> list[QtCore.QPointF]:
        source = self.scene_ref.port_scene_position(self.data.source)
        target = self.scene_ref.port_scene_position(self.data.target)
        result = [source]
        for waypoint in [*self.points, target]:
            last = result[-1]
            if not math.isclose(last.x(), waypoint.x()) and not math.isclose(
                last.y(), waypoint.y()
            ):
                result.append(QtCore.QPointF(waypoint.x(), last.y()))
            if result[-1] != waypoint:
                result.append(waypoint)
        return result

    def refresh(self) -> None:
        points = self.full_points()
        path = QtGui.QPainterPath(points[0])
        for point in points[1:]:
            path.lineTo(point)
        self.setPath(path)
        self.setPen(
            QtGui.QPen(
                self.color(),
                2.2,
                QtCore.Qt.SolidLine,
                QtCore.Qt.SquareCap,
                QtCore.Qt.MiterJoin,
            )
        )
        self._sync_handles()

    def _sync_handles(self) -> None:
        for handle in self.handles:
            handle.setParentItem(None)
            if handle.scene():
                handle.scene().removeItem(handle)
        self.handles = []
        if self.isSelected() and self.data.editable:
            for index, point in enumerate(self.points):
                self.handles.append(VertexHandle(self, index, point))

    def itemChange(self, change: "QtWidgets.QGraphicsItem.GraphicsItemChange", value: Any) -> Any:
        if change == self.ItemSelectedHasChanged:
            QtCore.QTimer.singleShot(0, self.refresh)
        return super().itemChange(change, value)

    def preview_vertex(self, index: int, point: QtCore.QPointF) -> None:
        if 0 <= index < len(self.points):
            self.points[index] = point
            points = self.full_points()
            path = QtGui.QPainterPath(points[0])
            for value in points[1:]:
                path.lineTo(value)
            self.setPath(path)

    def commit_vertices(self) -> None:
        self.scene_ref.adapter.set_wire_points(self.data.wire_id, self.points)
        self.scene_ref.changed("Wire route updated")

    def mouseDoubleClickEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:
        if not self.data.editable:
            return super().mouseDoubleClickEvent(event)
        point = self.scene_ref.snap_point(event.scenePos())
        all_points = self.full_points()
        insert_at = 0
        best_distance = float("inf")
        for index, (first, second) in enumerate(zip(all_points, all_points[1:])):
            distance = _distance_to_segment(point, first, second)
            if distance < best_distance:
                best_distance = distance
                insert_at = index
        self.points.insert(insert_at, point)
        self.commit_vertices()
        self.refresh()
        event.accept()


def _distance_to_segment(
    point: QtCore.QPointF, first: QtCore.QPointF, second: QtCore.QPointF
) -> float:
    dx, dy = second.x() - first.x(), second.y() - first.y()
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return math.hypot(point.x() - first.x(), point.y() - first.y())
    ratio = max(
        0.0,
        min(
            1.0,
            ((point.x() - first.x()) * dx + (point.y() - first.y()) * dy)
            / length_squared,
        ),
    )
    projection = QtCore.QPointF(first.x() + ratio * dx, first.y() + ratio * dy)
    return math.hypot(point.x() - projection.x(), point.y() - projection.y())


class NodeItem(QtWidgets.QGraphicsObject):
    def __init__(self, scene: "SchematicScene", data: NodeData) -> None:
        super().__init__()
        self.scene_ref = scene
        self.data = data
        self.press_position = QtCore.QPointF(data.position)
        self.setPos(data.position)
        self.setRotation(data.rotation)
        if data.mirror_x:
            self.setTransform(QtGui.QTransform().scale(-1, 1))
        self.setFlag(self.ItemIsMovable)
        self.setFlag(self.ItemIsSelectable)
        self.setFlag(self.ItemSendsGeometryChanges)
        self.setCacheMode(self.DeviceCoordinateCache)
        self.setZValue(2)

    def boundingRect(self) -> QtCore.QRectF:
        if self.scene_ref.adapter.kind == "system":
            return QtCore.QRectF(-90, -52, 180, 112)
        return QtCore.QRectF(-76, -55, 152, 120)

    def port_local_position(self, port_id: str) -> QtCore.QPointF:
        ports = list(self.data.ports)
        port = next(port for port in ports if port.port_id == port_id)
        kind = self.scene_ref.adapter.kind
        if kind == "system" or kind == "digital":
            inputs = [value for value in ports if value.direction == "input"]
            outputs = [value for value in ports if value.direction == "output"]
            if port.direction == "input":
                index = inputs.index(port)
                return QtCore.QPointF(-70, -25 + 50 * (index + 1) / (len(inputs) + 1))
            index = outputs.index(port)
            return QtCore.QPointF(70, -25 + 50 * (index + 1) / (len(outputs) + 1))
        if self.data.symbol == "ground":
            return QtCore.QPointF(0, -38)
        if self.data.symbol == "opamp":
            positions = {
                "plus": QtCore.QPointF(-62, -18),
                "minus": QtCore.QPointF(-62, 18),
                "out": QtCore.QPointF(62, 0),
            }
            return positions[port_id]
        if self.data.symbol == "mosfet":
            positions = {
                "drain": QtCore.QPointF(18, -45),
                "source": QtCore.QPointF(18, 45),
            }
            return positions[port_id]
        if len(ports) == 1:
            return QtCore.QPointF(0, -38)
        if len(ports) == 2:
            return QtCore.QPointF(-62 if ports.index(port) == 0 else 62, 0)
        if len(ports) == 3:
            return (
                QtCore.QPointF(-58, -18)
                if ports.index(port) == 0
                else QtCore.QPointF(-58, 18)
                if ports.index(port) == 1
                else QtCore.QPointF(62, 0)
            )
        positions = (
            QtCore.QPointF(-62, -20),
            QtCore.QPointF(62, -20),
            QtCore.QPointF(-62, 20),
            QtCore.QPointF(62, 20),
        )
        return positions[ports.index(port) % 4]

    def port_scene_position(self, port_id: str) -> QtCore.QPointF:
        return self.mapToScene(self.port_local_position(port_id))

    def itemChange(self, change: "QtWidgets.QGraphicsItem.GraphicsItemChange", value: Any) -> Any:
        if change == self.ItemPositionChange and self.scene() is not None:
            value = self.scene_ref.snap_point(value)
        if change == self.ItemPositionHasChanged and self.scene() is not None:
            self.scene_ref.refresh_wires()
        if change == self.ItemSelectedHasChanged and self.scene() is not None:
            QtCore.QTimer.singleShot(0, self.scene_ref.selection_changed)
        return super().itemChange(change, value)

    def mousePressEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:
        self.press_position = self.pos()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:
        super().mouseReleaseEvent(event)
        if self.pos() != self.press_position:
            self.scene_ref.adapter.set_position(self.data.node_id, self.pos())
            self.scene_ref.changed(f"Moved {self.data.name}")

    def mouseDoubleClickEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:
        if self.data.child_layer:
            self.scene_ref.open_child(self.data.node_id)
            event.accept()
        else:
            super().mouseDoubleClickEvent(event)

    def contextMenuEvent(self, event: QtWidgets.QGraphicsSceneContextMenuEvent) -> None:
        if not self.isSelected():
            self.scene_ref.clearSelection()
            self.setSelected(True)
        menu = QtWidgets.QMenu()
        rotate_cw = menu.addAction("Rotate clockwise")
        rotate_ccw = menu.addAction("Rotate counter-clockwise")
        mirror = menu.addAction("Mirror horizontally")
        menu.addSeparator()
        delete = menu.addAction("Delete")
        selected = menu.exec_(event.screenPos())
        if selected == rotate_cw:
            self.scene_ref.transform_selected(90, False)
        elif selected == rotate_ccw:
            self.scene_ref.transform_selected(-90, False)
        elif selected == mirror:
            self.scene_ref.transform_selected(0, True)
        elif selected == delete:
            self.scene_ref.delete_selected()

    def paint(
        self,
        painter: QtGui.QPainter,
        option: QtWidgets.QStyleOptionGraphicsItem,
        widget: QtWidgets.QWidget | None = None,
    ) -> None:
        del widget
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        selected = bool(option.state & QtWidgets.QStyle.State_Selected)
        color = COLOR_SELECT if selected else COLOR_WIRE
        pen = QtGui.QPen(color, 2.4 if selected else 1.8)
        pen.setCosmetic(True)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        if self.scene_ref.adapter.kind == "system":
            self._paint_system(painter, selected)
        elif self.scene_ref.adapter.kind == "digital":
            self._paint_digital(painter, color)
        else:
            self._paint_circuit(painter, color)
        self._paint_ports(painter)

    def _paint_system(self, painter: QtGui.QPainter, selected: bool) -> None:
        painter.setBrush(QtGui.QColor("#172033"))
        painter.setPen(QtGui.QPen(COLOR_SELECT if selected else COLOR_BLUE, 2))
        painter.drawRoundedRect(QtCore.QRectF(-78, -42, 156, 84), 4, 4)
        painter.fillRect(QtCore.QRectF(-78, -42, 156, 27), COLOR_BLUE)
        painter.setPen(COLOR_TEXT)
        painter.drawText(QtCore.QRectF(-68, -39, 115, 22), QtCore.Qt.AlignVCenter, self.data.name)
        painter.setPen(COLOR_MUTED)
        painter.drawText(QtCore.QRectF(-68, -10, 136, 30), QtCore.Qt.AlignLeft, self.data.display_name)
        painter.setBrush(QtGui.QColor("#0284c7"))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(QtCore.QPointF(61, -28), 11, 11)
        painter.setPen(QtCore.Qt.white)
        painter.drawText(QtCore.QRectF(50, -39, 22, 22), QtCore.Qt.AlignCenter, str(self.data.execution_order))
        if self.data.child_layer:
            painter.setPen(COLOR_SELECT)
            painter.drawText(QtCore.QRectF(-70, 21, 140, 17), QtCore.Qt.AlignRight, "Double-click to open  ↳")

    def _paint_digital(self, painter: QtGui.QPainter, color: QtGui.QColor) -> None:
        painter.setPen(QtGui.QPen(color, 2))
        painter.setBrush(COLOR_BLUE)
        symbol = self.data.symbol
        path = QtGui.QPainterPath()
        if symbol == "and":
            path.moveTo(-42, -30)
            path.lineTo(-5, -30)
            path.cubicTo(45, -30, 45, 30, -5, 30)
            path.lineTo(-42, 30)
            path.closeSubpath()
            painter.drawPath(path)
        elif symbol == "or":
            path.moveTo(-42, -30)
            path.cubicTo(-5, -25, 25, -22, 43, 0)
            path.cubicTo(25, 22, -5, 25, -42, 30)
            path.cubicTo(-22, 12, -22, -12, -42, -30)
            path.closeSubpath()
            painter.drawPath(path)
        elif symbol == "not":
            painter.drawPolygon(
                QtGui.QPolygonF(
                    [QtCore.QPointF(-40, -28), QtCore.QPointF(30, 0), QtCore.QPointF(-40, 28)]
                )
            )
            painter.setBrush(COLOR_BG)
            painter.drawEllipse(QtCore.QPointF(37, 0), 7, 7)
        elif symbol in {"input", "output"}:
            painter.drawPolygon(
                QtGui.QPolygonF(
                    [
                        QtCore.QPointF(-45, -22),
                        QtCore.QPointF(25, -22),
                        QtCore.QPointF(45, 0),
                        QtCore.QPointF(25, 22),
                        QtCore.QPointF(-45, 22),
                    ]
                )
            )
        else:
            painter.drawRoundedRect(QtCore.QRectF(-45, -30, 90, 60), 4, 4)
        painter.setPen(COLOR_TEXT)
        painter.drawText(QtCore.QRectF(-70, 38, 140, 20), QtCore.Qt.AlignCenter, self.data.name)

    def _paint_circuit(self, painter: QtGui.QPainter, color: QtGui.QColor) -> None:
        painter.setPen(QtGui.QPen(color, 2))
        painter.setBrush(QtCore.Qt.NoBrush)
        symbol = self.data.symbol
        if len(self.data.ports) == 2:
            painter.drawLine(-62, 0, -35, 0)
            painter.drawLine(35, 0, 62, 0)
        if symbol == "resistor":
            points = [QtCore.QPointF(-35, 0)]
            for index in range(1, 8):
                points.append(QtCore.QPointF(-35 + index * 10, -10 if index % 2 else 10))
            points.append(QtCore.QPointF(35, 0))
            painter.drawPolyline(QtGui.QPolygonF(points))
        elif symbol == "capacitor":
            painter.drawLine(-8, -24, -8, 24)
            painter.drawLine(8, -24, 8, 24)
            painter.drawLine(-35, 0, -8, 0)
            painter.drawLine(8, 0, 35, 0)
        elif symbol == "inductor":
            for index in range(4):
                painter.drawArc(QtCore.QRectF(-35 + index * 17.5, -12, 17.5, 24), 0, 180 * 16)
        elif symbol in {"voltage_source", "current_source", "dependent_voltage", "dependent_current"}:
            if symbol.startswith("dependent"):
                if len(self.data.ports) == 4:
                    painter.drawLine(-62, -20, -22, -20)
                    painter.drawLine(22, -20, 62, -20)
                    painter.drawLine(-62, 20, -22, 20)
                    painter.drawLine(22, 20, 62, 20)
                painter.drawPolygon(
                    QtGui.QPolygonF(
                        [QtCore.QPointF(0, -28), QtCore.QPointF(28, 0), QtCore.QPointF(0, 28), QtCore.QPointF(-28, 0)]
                    )
                )
            else:
                painter.drawEllipse(QtCore.QPointF(0, 0), 27, 27)
            painter.drawText(QtCore.QRectF(-15, -20, 30, 18), QtCore.Qt.AlignCenter, "+" if "voltage" in symbol else "↑")
            if "voltage" in symbol:
                painter.drawText(QtCore.QRectF(-15, 3, 30, 18), QtCore.Qt.AlignCenter, "−")
        elif symbol == "diode":
            painter.drawPolygon(
                QtGui.QPolygonF([QtCore.QPointF(-23, -20), QtCore.QPointF(18, 0), QtCore.QPointF(-23, 20)])
            )
            painter.drawLine(20, -22, 20, 22)
        elif symbol == "ground":
            painter.drawLine(0, -36, 0, -5)
            painter.drawLine(-24, -5, 24, -5)
            painter.drawLine(-16, 3, 16, 3)
            painter.drawLine(-8, 11, 8, 11)
        elif symbol == "opamp":
            painter.drawLine(-62, -18, -45, -18)
            painter.drawLine(-62, 18, -45, 18)
            painter.drawLine(45, 0, 62, 0)
            painter.drawPolygon(
                QtGui.QPolygonF([QtCore.QPointF(-45, -32), QtCore.QPointF(45, 0), QtCore.QPointF(-45, 32)])
            )
            painter.drawText(QtCore.QRectF(-42, -24, 20, 18), QtCore.Qt.AlignCenter, "+")
            painter.drawText(QtCore.QRectF(-42, 8, 20, 18), QtCore.Qt.AlignCenter, "−")
        elif symbol == "mosfet":
            # Enhancement-mode N-MOSFET: insulated gate at the left, broken
            # channel at the right, drain above and source below.  Gate drive
            # and bulk are internal to this Studio composite and are therefore
            # deliberately not exposed as connection ports.
            painter.drawLine(18, -45, 18, -28)
            painter.drawLine(18, 28, 18, 45)
            painter.drawLine(18, -28, 18, -13)
            painter.drawLine(18, -8, 18, 8)
            painter.drawLine(18, 13, 18, 28)
            painter.drawLine(-8, -27, -8, 27)
            painter.drawLine(-28, 0, -8, 0)
            painter.drawLine(18, 18, 34, 18)
            painter.drawLine(34, 18, 34, -18)
            painter.drawLine(34, -18, 18, -18)
            painter.drawLine(28, 4, 40, 4)
            painter.drawPolygon(
                QtGui.QPolygonF(
                    [
                        QtCore.QPointF(28, 4),
                        QtCore.QPointF(34, -5),
                        QtCore.QPointF(40, 4),
                    ]
                )
            )
            painter.drawText(
                QtCore.QRectF(-62, -12, 32, 24),
                QtCore.Qt.AlignCenter,
                self.scene_ref.pwm_label(self.data.node_id),
            )
        elif symbol == "switch":
            if len(self.data.ports) == 4:
                painter.drawLine(-62, -20, -24, -20)
                painter.drawLine(24, -20, 62, -20)
                painter.drawLine(-62, 20, -8, 20)
                painter.drawLine(8, 20, 62, 20)
            painter.drawEllipse(QtCore.QPointF(-24, 0), 3, 3)
            painter.drawEllipse(QtCore.QPointF(24, 0), 3, 3)
            painter.drawLine(-21, -2, 17, -19)
            painter.drawText(
                QtCore.QRectF(-28, 16, 56, 18),
                QtCore.Qt.AlignCenter,
                self.scene_ref.pwm_label(self.data.node_id),
            )
        elif symbol == "ammeter":
            painter.drawEllipse(QtCore.QPointF(0, 0), 25, 25)
            painter.drawText(QtCore.QRectF(-20, -13, 40, 26), QtCore.Qt.AlignCenter, "A")
        else:
            painter.drawRect(QtCore.QRectF(-35, -25, 70, 50))
        painter.setPen(COLOR_TEXT)
        painter.drawText(QtCore.QRectF(-75, 38, 150, 19), QtCore.Qt.AlignCenter, self.data.name)

    def _paint_ports(self, painter: QtGui.QPainter) -> None:
        painter.setBrush(COLOR_PORT)
        painter.setPen(QtGui.QPen(QtGui.QColor("#065f46"), 1))
        for port in self.data.ports:
            point = self.port_local_position(port.port_id)
            painter.drawEllipse(point, 4, 4)
            painter.setPen(COLOR_MUTED)
            offset = 8 if point.x() <= 0 else -34
            painter.drawText(QtCore.QRectF(point.x() + offset, point.y() - 13, 28, 12), QtCore.Qt.AlignCenter, port.label)
            painter.setPen(QtGui.QPen(QtGui.QColor("#065f46"), 1))


class SchematicScene(QtWidgets.QGraphicsScene):
    def __init__(self, window: "StudioWindow", adapter: LayerAdapter) -> None:
        super().__init__(-4000, -3000, 8000, 6000)
        self.window = window
        self.adapter = adapter
        self.node_items: dict[str, NodeItem] = {}
        self.wire_items: dict[str, WireItem] = {}
        self.draft_source: tuple[str, str] | None = None
        self.draft_points: list[QtCore.QPointF] = []
        self.draft_cursor: QtCore.QPointF | None = None
        self.draft_horizontal_first = True
        self.draft_path = QtWidgets.QGraphicsPathItem()
        self.draft_path.setPen(QtGui.QPen(COLOR_SELECT, 2, QtCore.Qt.DashLine))
        self.draft_path.setZValue(10)
        self.addItem(self.draft_path)
        self.rebuild()

    def changed(self, message: str) -> None:
        self.window.document_changed(message)

    def open_child(self, node_id: str) -> None:
        self.window.enter_child(node_id)

    def rebuild(self) -> None:
        for item in [*self.wire_items.values(), *self.node_items.values()]:
            self.removeItem(item)
        self.node_items.clear()
        self.wire_items.clear()
        for node_id in self.adapter.node_ids():
            item = NodeItem(self, self.adapter.node(node_id))
            self.node_items[node_id] = item
            self.addItem(item)
        for wire in self.adapter.wires():
            item = WireItem(self, wire)
            self.wire_items[wire.wire_id] = item
            self.addItem(item)
        self.draft_path.setPath(QtGui.QPainterPath())
        self.selection_changed()

    def snap_point(self, point: QtCore.QPointF) -> QtCore.QPointF:
        grid = DEFAULT_GRID_SIZE
        return QtCore.QPointF(round(point.x() / grid) * grid, round(point.y() / grid) * grid)

    def port_scene_position(self, endpoint: tuple[str, str]) -> QtCore.QPointF:
        return self.node_items[endpoint[0]].port_scene_position(endpoint[1])

    def pwm_label(self, node_id: str) -> str:
        driven = [
            str(candidate["id"])
            for candidate in self.adapter.layer.get("nodes", [])
            if MNA_COMPONENTS.get(str(candidate.get("type")))
            and MNA_COMPONENTS[str(candidate["type"])].integrated_pwm_driver
        ]
        return f"PWM{driven.index(node_id) + 1}" if node_id in driven else "PWM"

    def port_at(self, point: QtCore.QPointF) -> tuple[str, str] | None:
        best: tuple[str, str] | None = None
        best_distance = 11.0
        for node_id, item in self.node_items.items():
            for port in item.data.ports:
                position = item.port_scene_position(port.port_id)
                distance = math.hypot(position.x() - point.x(), position.y() - point.y())
                if distance < best_distance:
                    best_distance = distance
                    best = (node_id, port.port_id)
        return best

    def refresh_wires(self) -> None:
        for wire in self.wire_items.values():
            wire.refresh()

    def selection_changed(self) -> None:
        selected_nodes = [
            item.data.node_id
            for item in self.selectedItems()
            if isinstance(item, NodeItem)
        ]
        self.window.selection_changed(selected_nodes)

    def transform_selected(self, rotation: int = 0, mirror: bool = False) -> None:
        node_ids = [
            item.data.node_id for item in self.selectedItems() if isinstance(item, NodeItem)
        ]
        if not node_ids:
            return
        self.adapter.transform_nodes(node_ids, rotation, mirror)
        self.changed("Component transform updated")

    def delete_selected(self) -> None:
        node_ids = [
            item.data.node_id for item in self.selectedItems() if isinstance(item, NodeItem)
        ]
        wire_ids = [
            item.data.wire_id for item in self.selectedItems() if isinstance(item, WireItem)
        ]
        try:
            if node_ids:
                self.adapter.delete_nodes(node_ids)
            for wire_id in wire_ids:
                self.adapter.delete_wire(wire_id)
            if node_ids or wire_ids:
                self.changed("Deleted selected item(s)")
        except backend.StudioError as exc:
            QtWidgets.QMessageBox.critical(self.window, APP_NAME, str(exc))

    def duplicate_selected(self) -> None:
        node_ids = [
            item.data.node_id for item in self.selectedItems() if isinstance(item, NodeItem)
        ]
        if node_ids:
            self.adapter.duplicate_nodes(node_ids)
            self.changed("Duplicated selected component(s)")

    def arrange_selected(self, mode: str) -> None:
        node_ids = [
            item.data.node_id for item in self.selectedItems() if isinstance(item, NodeItem)
        ]
        if len(node_ids) >= 2:
            self.adapter.arrange_nodes(node_ids, mode)
            self.changed("Component layout updated")

    def add_component(self, type_id: str, position: QtCore.QPointF) -> None:
        try:
            node_id = self.adapter.add_node(type_id, self.snap_point(position))
            self.changed(f"Added {node_id}")
        except backend.StudioError as exc:
            QtWidgets.QMessageBox.critical(self.window, APP_NAME, str(exc))

    def _orthogonal_append(
        self, points: list[QtCore.QPointF], target: QtCore.QPointF
    ) -> list[QtCore.QPointF]:
        result = list(points)
        last = result[-1]
        if not math.isclose(last.x(), target.x()) and not math.isclose(last.y(), target.y()):
            result.append(
                QtCore.QPointF(target.x(), last.y())
                if self.draft_horizontal_first
                else QtCore.QPointF(last.x(), target.y())
            )
        if result[-1] != target:
            result.append(target)
        return result

    def _draft_full_path(self, target: QtCore.QPointF) -> list[QtCore.QPointF]:
        if self.draft_source is None:
            return []
        source = self.port_scene_position(self.draft_source)
        return self._orthogonal_append([source, *self.draft_points], target)

    def _draw_draft(self, target: QtCore.QPointF) -> None:
        self.draft_cursor = QtCore.QPointF(target)
        points = self._draft_full_path(target)
        if not points:
            return
        path = QtGui.QPainterPath(points[0])
        for point in points[1:]:
            path.lineTo(point)
        self.draft_path.setPath(path)

    def cancel_wire(self) -> None:
        self.draft_source = None
        self.draft_points.clear()
        self.draft_cursor = None
        self.draft_horizontal_first = True
        self.draft_path.setPath(QtGui.QPainterPath())

    def mousePressEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:
        if event.button() == QtCore.Qt.RightButton and self.draft_source:
            self.cancel_wire()
            event.accept()
            return
        if event.button() == QtCore.Qt.LeftButton:
            endpoint = self.port_at(event.scenePos())
            if self.draft_source is None and endpoint:
                self.draft_source = endpoint
                self.draft_points.clear()
                self._draw_draft(event.scenePos())
                event.accept()
                return
            if self.draft_source is not None:
                if endpoint and endpoint != self.draft_source:
                    target = self.port_scene_position(endpoint)
                    full = self._draft_full_path(target)
                    try:
                        self.adapter.connect(self.draft_source, endpoint, full[1:-1])
                        self.cancel_wire()
                        self.changed("Wire created")
                    except backend.StudioError as exc:
                        QtWidgets.QMessageBox.critical(self.window, APP_NAME, str(exc))
                    event.accept()
                    return
                snapped = self.snap_point(event.scenePos())
                start = self.port_scene_position(self.draft_source)
                all_points = self._orthogonal_append([start, *self.draft_points], snapped)
                self.draft_points = all_points[1:]
                self._draw_draft(snapped)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:
        if self.draft_source:
            self._draw_draft(self.snap_point(event.scenePos()))
        super().mouseMoveEvent(event)

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.key() == QtCore.Qt.Key_Space and not event.isAutoRepeat():
            if self.draft_source is not None:
                self.draft_horizontal_first = not self.draft_horizontal_first
                if self.draft_cursor is not None:
                    self._draw_draft(self.draft_cursor)
            else:
                self.transform_selected(90, False)
            event.accept()
        elif event.key() == QtCore.Qt.Key_Escape:
            self.cancel_wire()
            event.accept()
        elif event.key() in {QtCore.Qt.Key_Delete, QtCore.Qt.Key_Backspace}:
            self.delete_selected()
            event.accept()
        elif event.key() == QtCore.Qt.Key_D and event.modifiers() & QtCore.Qt.ControlModifier:
            self.duplicate_selected()
            event.accept()
        elif event.key() == QtCore.Qt.Key_R:
            self.transform_selected(-90 if event.modifiers() & QtCore.Qt.ShiftModifier else 90)
            event.accept()
        elif event.key() == QtCore.Qt.Key_M:
            self.transform_selected(0, True)
            event.accept()
        else:
            super().keyPressEvent(event)


class SchematicView(QtWidgets.QGraphicsView):
    def __init__(self, scene: SchematicScene) -> None:
        super().__init__(scene)
        self.setRenderHints(
            QtGui.QPainter.Antialiasing | QtGui.QPainter.TextAntialiasing
        )
        self.setViewportUpdateMode(self.FullViewportUpdate)
        self.setTransformationAnchor(self.AnchorUnderMouse)
        self.setResizeAnchor(self.AnchorViewCenter)
        self.setDragMode(self.RubberBandDrag)
        self.setAcceptDrops(True)
        self.setBackgroundBrush(COLOR_BG)
        self._panning = False
        self._pan_start = QtCore.QPoint()

    @property
    def schematic_scene(self) -> SchematicScene:
        return self.scene()  # type: ignore[return-value]

    def drawBackground(self, painter: QtGui.QPainter, rect: QtCore.QRectF) -> None:
        painter.fillRect(rect, COLOR_BG)
        grid = DEFAULT_GRID_SIZE
        left = math.floor(rect.left() / grid) * grid
        top = math.floor(rect.top() / grid) * grid
        minor, major = [], []
        x = left
        while x < rect.right():
            target = major if int(round(x / grid)) % 5 == 0 else minor
            target.append(QtCore.QLineF(x, rect.top(), x, rect.bottom()))
            x += grid
        y = top
        while y < rect.bottom():
            target = major if int(round(y / grid)) % 5 == 0 else minor
            target.append(QtCore.QLineF(rect.left(), y, rect.right(), y))
            y += grid
        painter.setPen(QtGui.QPen(COLOR_GRID, 0))
        painter.drawLines(minor)
        painter.setPen(QtGui.QPen(COLOR_GRID_MAJOR, 0))
        painter.drawLines(major)

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        current = self.transform().m11()
        if 0.2 <= current * factor <= 4.0:
            self.scale(factor, factor)

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.key() == QtCore.Qt.Key_Space:
            self.schematic_scene.keyPressEvent(event)
            return
        super().keyPressEvent(event)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.MiddleButton:
            self._panning = True
            self._pan_start = event.pos()
            self.setCursor(QtCore.Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        if self._panning:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.MiddleButton:
            self._panning = False
            self.setCursor(QtCore.Qt.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def dragEnterEvent(self, event: QtGui.QDragEnterEvent) -> None:
        if event.mimeData().hasFormat(MIME_COMPONENT):
            event.acceptProposedAction()
        else:
            super().dragEnterEvent(event)

    def dragMoveEvent(self, event: QtGui.QDragMoveEvent) -> None:
        if event.mimeData().hasFormat(MIME_COMPONENT):
            event.acceptProposedAction()
        else:
            super().dragMoveEvent(event)

    def dropEvent(self, event: QtGui.QDropEvent) -> None:
        if event.mimeData().hasFormat(MIME_COMPONENT):
            type_id = bytes(event.mimeData().data(MIME_COMPONENT)).decode("utf-8")
            self.schematic_scene.add_component(type_id, self.mapToScene(event.pos()))
            event.acceptProposedAction()
        else:
            super().dropEvent(event)


class PropertyInspector(QtWidgets.QWidget):
    apply_requested = QtCore.pyqtSignal(str, str, dict, int)
    transform_requested = QtCore.pyqtSignal(int, bool)

    def __init__(self) -> None:
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("Properties")
        title.setStyleSheet("font-size: 16px; font-weight: 600;")
        layout.addWidget(title)
        self.summary = QtWidgets.QLabel("No selection")
        self.summary.setStyleSheet("color: #64748b;")
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        form = QtWidgets.QFormLayout()
        self.name_edit = QtWidgets.QLineEdit()
        form.addRow("Name", self.name_edit)
        self.order_spin = QtWidgets.QSpinBox()
        self.order_spin.setRange(0, 1_000_000)
        self.order_label = QtWidgets.QLabel("Execution order")
        form.addRow(self.order_label, self.order_spin)
        layout.addLayout(form)
        parameter_label = QtWidgets.QLabel("Component parameters")
        parameter_label.setStyleSheet("font-weight: 600; margin-top: 8px;")
        layout.addWidget(parameter_label)
        self.parameters = QtWidgets.QTableWidget(0, 2)
        self.parameters.setHorizontalHeaderLabels(["Parameter", "Value"])
        self.parameters.horizontalHeader().setStretchLastSection(True)
        self.parameters.verticalHeader().setVisible(False)
        self.parameters.setAlternatingRowColors(True)
        layout.addWidget(self.parameters, 1)
        transform_row = QtWidgets.QHBoxLayout()
        self.rotate_left = QtWidgets.QPushButton("↺ Rotate")
        self.rotate_right = QtWidgets.QPushButton("Rotate ↻")
        self.mirror = QtWidgets.QPushButton("⇋ Mirror")
        transform_row.addWidget(self.rotate_left)
        transform_row.addWidget(self.rotate_right)
        transform_row.addWidget(self.mirror)
        layout.addLayout(transform_row)
        self.ports = QtWidgets.QLabel()
        self.ports.setWordWrap(True)
        self.ports.setStyleSheet("color: #475569;")
        layout.addWidget(self.ports)
        self.apply_button = QtWidgets.QPushButton("Apply parameters")
        self.apply_button.setDefault(True)
        layout.addWidget(self.apply_button)
        self.apply_button.clicked.connect(self._emit_apply)
        self.rotate_left.clicked.connect(lambda: self.transform_requested.emit(-90, False))
        self.rotate_right.clicked.connect(lambda: self.transform_requested.emit(90, False))
        self.mirror.clicked.connect(lambda: self.transform_requested.emit(0, True))
        self.node_id: str | None = None
        self.specs: tuple[ParameterSpec, ...] = ()

    def clear(self, text: str = "No selection") -> None:
        self.node_id = None
        self.summary.setText(text)
        self.name_edit.clear()
        self.parameters.setRowCount(0)
        self.ports.clear()
        self.apply_button.setEnabled(False)

    def set_node(self, adapter: LayerAdapter, node: NodeData) -> None:
        self.node_id = node.node_id
        self.summary.setText(f"{node.display_name}\n{node.type_id}")
        self.name_edit.setText(node.name)
        system = adapter.kind == "system"
        self.order_label.setVisible(system)
        self.order_spin.setVisible(system)
        self.order_spin.setValue(node.execution_order)
        transform_visible = adapter.kind in {"circuit", "digital"}
        for button in (self.rotate_left, self.rotate_right, self.mirror):
            button.setVisible(transform_visible)
        self.specs = adapter.parameter_specs(node.node_id)
        self.parameters.setRowCount(len(self.specs))
        for row, spec in enumerate(self.specs):
            label = QtWidgets.QTableWidgetItem(spec.label)
            label.setData(QtCore.Qt.UserRole, spec.parameter_id)
            label.setToolTip(spec.description)
            label.setFlags(label.flags() & ~QtCore.Qt.ItemIsEditable)
            value = QtWidgets.QTableWidgetItem(str(node.parameters.get(spec.parameter_id, spec.default)))
            self.parameters.setItem(row, 0, label)
            self.parameters.setItem(row, 1, value)
        self.parameters.resizeRowsToContents()
        self.ports.setText(
            "Ports\n"
            + "\n".join(
                f"  {port.label}: {adapter.port_value((node.node_id, port.port_id))}"
                for port in node.ports
            )
        )
        self.apply_button.setEnabled(True)

    def _emit_apply(self) -> None:
        if self.node_id is None:
            return
        values: dict[str, Any] = {}
        for row, spec in enumerate(self.specs):
            item = self.parameters.item(row, 1)
            values[spec.parameter_id] = item.text() if item else str(spec.default)
        self.apply_requested.emit(
            self.node_id,
            self.name_edit.text().strip(),
            values,
            self.order_spin.value(),
        )


class StudioWindow(QtWidgets.QMainWindow):
    def __init__(self, project_path: Path | None = None) -> None:
        super().__init__()
        self.resize(1500, 920)
        self.setMinimumSize(1100, 700)
        self.components = backend.load_components([backend.BUILTIN_LIBRARY])
        self.document = EditorDocument(None, self.components)
        self.hierarchy = HierarchyDocument(self.document)
        self.layer_stack = [self.hierarchy.root_layer_id]
        self.adapter = LayerAdapter(self.document, self.hierarchy, self.layer_stack[-1])
        self.scene: SchematicScene | None = None
        self.view: SchematicView | None = None
        self._build_actions()
        self._build_ui()
        self.load_layer(self.layer_stack[-1])
        if project_path:
            self.open_path(project_path)

    def _build_actions(self) -> None:
        self.new_action = QtWidgets.QAction("New", self, shortcut=QtGui.QKeySequence.New)
        self.open_action = QtWidgets.QAction("Open", self, shortcut=QtGui.QKeySequence.Open)
        self.save_action = QtWidgets.QAction("Save", self, shortcut=QtGui.QKeySequence.Save)
        self.save_as_action = QtWidgets.QAction("Save As", self, shortcut=QtGui.QKeySequence.SaveAs)
        self.undo_action = QtWidgets.QAction("Undo", self, shortcut=QtGui.QKeySequence.Undo)
        self.redo_action = QtWidgets.QAction("Redo", self, shortcut=QtGui.QKeySequence.Redo)
        self.back_action = QtWidgets.QAction("Back", self, shortcut="Alt+Left")
        self.delete_action = QtWidgets.QAction("Delete", self, shortcut=QtGui.QKeySequence.Delete)
        self.duplicate_action = QtWidgets.QAction("Duplicate", self, shortcut="Ctrl+D")
        self.rotate_action = QtWidgets.QAction("Rotate 90° (Space)", self, shortcut="R")
        self.mirror_action = QtWidgets.QAction("Mirror", self, shortcut="M")
        self.fit_action = QtWidgets.QAction("Fit", self, shortcut="F6")
        self.export_action = QtWidgets.QAction("Export MNA netlist", self)
        self.new_action.triggered.connect(self.new_project)
        self.open_action.triggered.connect(self.open_project)
        self.save_action.triggered.connect(self.save_project)
        self.save_as_action.triggered.connect(self.save_project_as)
        self.undo_action.triggered.connect(self.undo)
        self.redo_action.triggered.connect(self.redo)
        self.back_action.triggered.connect(self.back)
        self.delete_action.triggered.connect(lambda: self.scene and self.scene.delete_selected())
        self.duplicate_action.triggered.connect(lambda: self.scene and self.scene.duplicate_selected())
        self.rotate_action.triggered.connect(lambda: self.scene and self.scene.transform_selected(90, False))
        self.mirror_action.triggered.connect(lambda: self.scene and self.scene.transform_selected(0, True))
        self.fit_action.triggered.connect(self.fit_scene)
        self.export_action.triggered.connect(self.export_mna)

    def _build_ui(self) -> None:
        file_menu = self.menuBar().addMenu("File")
        file_menu.addActions([self.new_action, self.open_action, self.save_action, self.save_as_action])
        file_menu.addSeparator()
        file_menu.addAction(self.export_action)
        edit_menu = self.menuBar().addMenu("Edit")
        edit_menu.addActions([self.undo_action, self.redo_action, self.duplicate_action, self.delete_action, self.rotate_action, self.mirror_action])
        layout_menu = self.menuBar().addMenu("Layout")
        for label, mode in (
            ("Align left", "left"),
            ("Align horizontal centers", "hcenter"),
            ("Align right", "right"),
            ("Align top", "top"),
            ("Align vertical centers", "vcenter"),
            ("Align bottom", "bottom"),
            ("Distribute horizontally", "distribute_h"),
            ("Distribute vertically", "distribute_v"),
        ):
            action = layout_menu.addAction(label)
            action.triggered.connect(
                lambda _checked=False, value=mode: self.scene
                and self.scene.arrange_selected(value)
            )
        toolbar = self.addToolBar("Main")
        toolbar.setMovable(False)
        toolbar.addActions([self.back_action, self.new_action, self.open_action, self.save_action])
        toolbar.addSeparator()
        toolbar.addActions([self.undo_action, self.redo_action, self.duplicate_action, self.delete_action, self.rotate_action, self.mirror_action, self.fit_action])

        self.palette = ComponentList()
        self.palette.setDragEnabled(True)
        self.palette.itemDoubleClicked.connect(self._palette_double_clicked)
        self.palette_filter = QtWidgets.QLineEdit()
        self.palette_filter.setPlaceholderText("Filter components")
        self.palette_filter.textChanged.connect(self.refresh_palette)
        palette_widget = QtWidgets.QWidget()
        palette_layout = QtWidgets.QVBoxLayout(palette_widget)
        palette_layout.setContentsMargins(6, 6, 6, 6)
        palette_layout.addWidget(QtWidgets.QLabel("Components"))
        palette_layout.addWidget(self.palette_filter)
        palette_layout.addWidget(self.palette, 2)
        palette_layout.addWidget(QtWidgets.QLabel("Design"))
        self.navigator = QtWidgets.QTreeWidget()
        self.navigator.setHeaderHidden(True)
        self.navigator.itemDoubleClicked.connect(self._navigator_double_clicked)
        palette_layout.addWidget(self.navigator, 1)
        left_dock = QtWidgets.QDockWidget("Library", self)
        left_dock.setWidget(palette_widget)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, left_dock)

        self.inspector = PropertyInspector()
        self.inspector.apply_requested.connect(self.apply_properties)
        self.inspector.transform_requested.connect(self.transform_selected)
        right_dock = QtWidgets.QDockWidget("Inspector", self)
        right_dock.setWidget(self.inspector)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, right_dock)

        central = QtWidgets.QWidget()
        central_layout = QtWidgets.QVBoxLayout(central)
        central_layout.setContentsMargins(0, 0, 0, 0)
        self.breadcrumb = QtWidgets.QWidget()
        self.breadcrumb_layout = QtWidgets.QHBoxLayout(self.breadcrumb)
        self.breadcrumb_layout.setContentsMargins(6, 3, 6, 3)
        self.breadcrumb_layout.addStretch(1)
        central_layout.addWidget(self.breadcrumb)
        self.view_container = QtWidgets.QVBoxLayout()
        central_layout.addLayout(self.view_container, 1)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Ready")

    def load_layer(self, layer_id: str, preserve_view: bool = False) -> None:
        old_transform = QtGui.QTransform(self.view.transform()) if preserve_view and self.view else None
        old_center = (
            self.view.mapToScene(self.view.viewport().rect().center())
            if preserve_view and self.view
            else None
        )
        self.hierarchy = HierarchyDocument(self.document)
        self.adapter = LayerAdapter(self.document, self.hierarchy, layer_id)
        if self.view:
            self.view_container.removeWidget(self.view)
            self.view.deleteLater()
        self.scene = SchematicScene(self, self.adapter)
        self.view = SchematicView(self.scene)
        self.view_container.addWidget(self.view)
        if old_transform is not None and old_center is not None:
            self.view.setTransform(old_transform)
            self.view.centerOn(old_center)
        self.refresh_palette()
        self.refresh_navigator()
        self.refresh_breadcrumb()
        self.inspector.clear(f"{self.adapter.name}\n{self.adapter.kind.title()} editor")
        self.export_action.setEnabled(
            self.adapter.kind == "circuit" and not self.adapter.legacy_circuit
        )
        if self.adapter.kind == "circuit":
            self.statusBar().showMessage(
                "Click a port to route; click for corners; Space changes corner direction or rotates a selected component."
            )
        self.update_title()

    def document_changed(self, message: str) -> None:
        current_layer = self.layer_stack[-1]
        self.load_layer(current_layer, preserve_view=True)
        self.statusBar().showMessage(message, 5000)

    def selection_changed(self, node_ids: Sequence[str]) -> None:
        if len(node_ids) == 1:
            self.inspector.set_node(self.adapter, self.adapter.node(node_ids[0]))
        elif node_ids:
            self.inspector.clear(f"{len(node_ids)} components selected")
        else:
            self.inspector.clear(f"{self.adapter.name}\n{self.adapter.kind.title()} editor")

    def refresh_palette(self) -> None:
        query = self.palette_filter.text().strip().lower() if hasattr(self, "palette_filter") else ""
        self.palette.clear()
        for type_id, label in sorted(self.adapter.palette().items(), key=lambda pair: pair[1]):
            if query and query not in f"{type_id} {label}".lower():
                continue
            item = QtWidgets.QListWidgetItem(label)
            item.setData(QtCore.Qt.UserRole, type_id)
            item.setToolTip(type_id)
            self.palette.addItem(item)

    def refresh_navigator(self) -> None:
        self.navigator.clear()
        for node_id in self.adapter.node_ids():
            node = self.adapter.node(node_id)
            item = QtWidgets.QTreeWidgetItem([f"{node.name} · {node.display_name}"])
            item.setData(0, QtCore.Qt.UserRole, node_id)
            self.navigator.addTopLevelItem(item)

    def refresh_breadcrumb(self) -> None:
        while self.breadcrumb_layout.count() > 1:
            item = self.breadcrumb_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        for index, layer_id in enumerate(self.layer_stack):
            if index:
                self.breadcrumb_layout.insertWidget(index * 2 - 1, QtWidgets.QLabel("›"))
            layer = self.hierarchy.layer(layer_id)
            button = QtWidgets.QPushButton(f"{layer['name']}  [{layer['kind']}]")
            button.clicked.connect(lambda _checked=False, value=index: self.jump_to(value))
            self.breadcrumb_layout.insertWidget(index * 2, button)

    def _palette_double_clicked(self, item: QtWidgets.QListWidgetItem) -> None:
        if self.view:
            self.scene.add_component(
                str(item.data(QtCore.Qt.UserRole)), self.view.mapToScene(self.view.viewport().rect().center())
            )

    def _navigator_double_clicked(self, item: QtWidgets.QTreeWidgetItem) -> None:
        self.enter_child(str(item.data(0, QtCore.Qt.UserRole)))

    def enter_child(self, node_id: str) -> None:
        child = self.adapter.node(node_id).child_layer
        if child:
            self.layer_stack.append(child)
            self.load_layer(child)

    def back(self) -> None:
        if len(self.layer_stack) > 1:
            self.layer_stack.pop()
            self.load_layer(self.layer_stack[-1])

    def jump_to(self, index: int) -> None:
        if 0 <= index < len(self.layer_stack):
            self.layer_stack = self.layer_stack[: index + 1]
            self.load_layer(self.layer_stack[-1])

    def apply_properties(
        self, node_id: str, name: str, parameters: dict[str, Any], order: int
    ) -> None:
        try:
            self.adapter.update_node(node_id, name, parameters, order)
            self.document_changed(f"Updated {name}")
        except backend.StudioError as exc:
            QtWidgets.QMessageBox.critical(self, APP_NAME, str(exc))

    def transform_selected(self, rotation: int, mirror: bool) -> None:
        if self.scene:
            self.scene.transform_selected(rotation, mirror)

    def fit_scene(self) -> None:
        if self.view and self.scene:
            bounds = self.scene.itemsBoundingRect()
            if not bounds.isEmpty():
                self.view.fitInView(bounds.adjusted(-60, -60, 60, 60), QtCore.Qt.KeepAspectRatio)

    def undo(self) -> None:
        if self.document.undo():
            self._recover_layer()
            self.document_changed("Undo")

    def redo(self) -> None:
        if self.document.redo():
            self._recover_layer()
            self.document_changed("Redo")

    def _recover_layer(self) -> None:
        self.hierarchy = HierarchyDocument(self.document)
        valid = self.hierarchy.data["layers"]
        self.layer_stack = [layer for layer in self.layer_stack if layer in valid]
        if not self.layer_stack:
            self.layer_stack = [self.hierarchy.root_layer_id]

    def maybe_save(self) -> bool:
        if not self.document.dirty:
            return True
        answer = QtWidgets.QMessageBox.question(
            self,
            APP_NAME,
            "Save the current project?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No | QtWidgets.QMessageBox.Cancel,
        )
        if answer == QtWidgets.QMessageBox.Cancel:
            return False
        return self.save_project() if answer == QtWidgets.QMessageBox.Yes else True

    def new_project(self) -> None:
        if not self.maybe_save():
            return
        self.components = backend.load_components([backend.BUILTIN_LIBRARY])
        self.document = EditorDocument(None, self.components)
        self.hierarchy = HierarchyDocument(self.document)
        self.layer_stack = [self.hierarchy.root_layer_id]
        self.load_layer(self.layer_stack[-1])

    def open_project(self) -> None:
        if not self.maybe_save():
            return
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open CCTL Studio project", "", "CCTL Project (*.json);;All Files (*)"
        )
        if filename:
            self.open_path(Path(filename))

    def open_path(self, path: Path) -> None:
        try:
            raw = backend._read_json(path)
            self.components = backend.load_components(
                backend.project_library_paths(path.resolve(), raw)
            )
            self.document = EditorDocument(raw, self.components, path)
            self.hierarchy = HierarchyDocument(self.document)
            self.layer_stack = [self.hierarchy.root_layer_id]
            self.load_layer(self.layer_stack[-1])
            self.statusBar().showMessage(f"Opened {path}", 5000)
        except backend.StudioError as exc:
            QtWidgets.QMessageBox.critical(self, APP_NAME, str(exc))

    def save_project(self) -> bool:
        if self.document.path is None:
            return self.save_project_as()
        try:
            self.document.save()
            self.update_title()
            self.statusBar().showMessage(f"Saved {self.document.path}", 5000)
            return True
        except (backend.StudioError, OSError) as exc:
            QtWidgets.QMessageBox.critical(self, APP_NAME, str(exc))
            return False

    def save_project_as(self) -> bool:
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save CCTL Studio project", "", "CCTL Project (*.json)"
        )
        if not filename:
            return False
        if not filename.lower().endswith(".json"):
            filename += ".json"
        try:
            self.document.save(Path(filename))
            self.update_title()
            return True
        except (backend.StudioError, OSError) as exc:
            QtWidgets.QMessageBox.critical(self, APP_NAME, str(exc))
            return False

    def export_mna(self) -> None:
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export MNA Solver netlist", "", "MNA/SPICE Netlist (*.cir)"
        )
        if not filename:
            return
        if not filename.lower().endswith(".cir"):
            filename += ".cir"
        try:
            text = export_mna_netlist(self.hierarchy, self.adapter.layer_id, self.adapter.name)
            Path(filename).write_text(text, encoding="utf-8", newline="\n")
            self.statusBar().showMessage(f"Exported MNA netlist: {filename}", 8000)
        except (backend.StudioError, OSError) as exc:
            QtWidgets.QMessageBox.critical(self, APP_NAME, str(exc))

    def update_title(self) -> None:
        name = self.document.path.name if self.document.path else "Untitled"
        self.setWindowTitle(f"{name}{' *' if self.document.dirty else ''} — {APP_NAME}")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        event.accept() if self.maybe_save() else event.ignore()


def launch(project: Path | None = None) -> int:
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    application.setApplicationName(APP_NAME)
    application.setStyle("Fusion")
    window = StudioWindow(project)
    window.show()
    return int(application.exec_())


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Launch the Qt GMP CCTL Studio editor.")
    parser.add_argument("project", nargs="?", type=Path)
    args = parser.parse_args(argv)
    return launch(args.project)


if __name__ == "__main__":
    raise SystemExit(main())
