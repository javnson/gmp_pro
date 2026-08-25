import json
import os
import sys
import tempfile
import unittest
from pathlib import Path


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
CORE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CORE_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from PyQt5 import QtCore, QtGui, QtTest, QtWidgets  # noqa: E402

import cctl_studio  # noqa: E402
from editor_model import EditorDocument  # noqa: E402
from hierarchy_model import HierarchyDocument  # noqa: E402
from qt_studio import (  # noqa: E402
    MOSFET_BODY_DIODE_CATHODE_Y,
    MOSFET_BODY_DIODE_TIP,
    LayerAdapter,
    PropertyInspector,
    StudioWindow,
    _drag_orthogonal_segment,
)
from topology_bundle import binding_key  # noqa: E402
from topology_fixture import write_topology_bundle  # noqa: E402


class QtUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        components = cctl_studio.load_components([cctl_studio.BUILTIN_LIBRARY])
        self.document = EditorDocument(None, components)
        self.hierarchy = HierarchyDocument(self.document)
        self.adapter = LayerAdapter(self.document, self.hierarchy, "circuit_main")

    def test_circuit_palette_and_parameter_first_inspector(self):
        self.assertIn("circuit.mosfet", self.adapter.palette())
        self.assertIn("GMD", self.adapter.palette()["circuit.ground"])
        resistor = self.hierarchy.add_node(
            "circuit_main", "circuit.resistor", 100, 100
        )
        inspector = PropertyInspector()
        inspector.set_node(self.adapter, self.adapter.node(resistor))

        self.assertTrue(inspector.order_spin.isHidden())
        self.assertFalse(inspector.rotate_right.isHidden())
        self.assertEqual(inspector.parameters.rowCount(), 1)
        self.assertEqual(inspector.parameters.item(0, 0).text(), "Resistance")
        self.assertEqual(inspector.parameters.item(0, 1).text(), "1k")

    def test_system_inspector_retains_execution_order(self):
        adapter = LayerAdapter(self.document, self.hierarchy, "system_root")
        inspector = PropertyInspector()
        inspector.set_node(adapter, adapter.node("TOP1"))
        self.assertFalse(inspector.order_spin.isHidden())
        self.assertTrue(inspector.rotate_right.isHidden())

    def test_compiled_topology_import_has_pin_controls_and_adaptive_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest_path = write_topology_bundle(root)
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            for index in range(5):
                manifest["interface"]["inputs"].append(
                    {
                        "name": f"AIN{index}",
                        "field": f"AIN{index}",
                        "data_type": "double",
                        "role": "analog_input",
                        "default": 0.0,
                    }
                )
            manifest_path.write_text(
                json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
            )

            window = StudioWindow()
            topology = window.adapter.import_compiled_topology(
                manifest_path, QtCore.QPointF(320, 180)
            )
            window.document_changed("Imported test topology")
            item = window.scene.node_items[topology]
            item.setSelected(True)
            window.scene.selection_changed()
            self.application.processEvents()

            self.assertIn("Class: BuckCircuit", window.inspector.summary.text())
            mode_rows = {
                spec.parameter_id: row
                for row, spec in enumerate(window.inspector.specs)
                if spec.choices
            }
            input_mode = binding_key("input", "VS1", "mode")
            output_mode = binding_key("output", "VF1", "mode")
            self.assertIsInstance(
                window.inspector.parameters.cellWidget(mode_rows[input_mode], 1),
                QtWidgets.QComboBox,
            )

            for port in item.data.ports:
                position = item.port_local_position(port.port_id)
                self.assertTrue(item.boundingRect().contains(position))
                self.assertAlmostEqual(position.y() % 20, 0)

            raw = window.hierarchy.node("system_root", topology)
            parameters = dict(raw["parameters"])
            parameters[input_mode] = "constant"
            parameters[binding_key("input", "VS1", "value")] = "24"
            parameters[output_mode] = "hidden"
            window.adapter.update_node(
                topology, raw["name"], parameters, raw["execution_order"]
            )
            window.document_changed("Configured test topology")
            configured = window.adapter.node(topology)
            self.assertNotIn("in_VS1", {port.port_id for port in configured.ports})
            self.assertNotIn("out_VF1", {port.port_id for port in configured.ports})
            self.assertEqual(
                configured.parameters[binding_key("input", "VS1", "value")],
                24.0,
            )
            window.hide()
            window.deleteLater()

    def test_wire_vertices_remain_editable_and_render_orthogonally(self):
        window = StudioWindow()
        first = window.hierarchy.add_node(
            "circuit_main", "circuit.resistor", 0, 0
        )
        second = window.hierarchy.add_node(
            "circuit_main", "circuit.capacitor", 240, 120
        )
        wire_id = window.hierarchy.connect(
            "circuit_main",
            (first, "n"),
            (second, "p"),
            [(120, 0), (120, 80)],
        )
        window.layer_stack = ["system_root", "circuit_main"]
        window.load_layer("circuit_main")
        wire = window.scene.wire_items[wire_id]
        wire.setSelected(True)
        self.application.processEvents()

        self.assertEqual(len(wire.handles), 2)
        for start, end in zip(wire.full_points(), wire.full_points()[1:]):
            self.assertTrue(start.x() == end.x() or start.y() == end.y())
        window.hide()
        window.deleteLater()

    def test_space_rotates_components_and_toggles_wire_corner_direction(self):
        window = StudioWindow()
        resistor = window.hierarchy.add_node(
            "circuit_main", "circuit.resistor", 0, 0
        )
        window.layer_stack = ["system_root", "circuit_main"]
        window.load_layer("circuit_main")
        window.show()
        window.scene.node_items[resistor].setSelected(True)
        window.view.setFocus(QtCore.Qt.OtherFocusReason)
        self.application.processEvents()
        QtTest.QTest.keyClick(
            self.application.focusWidget(), QtCore.Qt.Key_Space
        )
        self.assertEqual(
            window.hierarchy.node("circuit_main", resistor)["rotation"], 90
        )
        self.application.processEvents()
        self.assertTrue(window.scene.node_items[resistor].isSelected())
        self.assertIn(
            self.application.focusWidget(),
            {window.view, window.view.viewport()},
        )

        QtTest.QTest.keyClick(
            self.application.focusWidget(), QtCore.Qt.Key_Space
        )
        self.assertEqual(
            window.hierarchy.node("circuit_main", resistor)["rotation"], 180
        )
        self.assertTrue(window.scene.node_items[resistor].isSelected())

        scene = window.scene
        scene.draft_source = (resistor, "n")
        source = scene.port_scene_position(scene.draft_source)
        cursor = source + QtCore.QPointF(100, 60)
        scene._draw_draft(cursor)
        first_path = scene.draft_path.path()
        first_corner = first_path.elementAt(1)
        window.view.keyPressEvent(
            QtGui.QKeyEvent(QtCore.QEvent.KeyPress, QtCore.Qt.Key_Space, QtCore.Qt.NoModifier)
        )
        second_path = scene.draft_path.path()
        second_corner = second_path.elementAt(1)

        self.assertFalse(scene.draft_horizontal_first)
        self.assertNotEqual(
            (first_corner.x, first_corner.y),
            (second_corner.x, second_corner.y),
        )
        window.hide()
        window.deleteLater()

    def test_all_circuit_ports_stay_on_grid_after_rotation(self):
        window = StudioWindow()
        created = []
        circuit_palette = LayerAdapter(
            window.document, window.hierarchy, "circuit_main"
        ).palette()
        for index, type_id in enumerate(circuit_palette):
            created.append(
                window.hierarchy.add_node(
                    "circuit_main", type_id, index * 200, index * 120
                )
            )
        window.layer_stack = ["system_root", "circuit_main"]
        window.load_layer("circuit_main")
        for node_id in created:
            item = window.scene.node_items[node_id]
            for port in item.data.ports:
                point = item.port_scene_position(port.port_id)
                self.assertAlmostEqual(point.x() % 20, 0)
                self.assertAlmostEqual(point.y() % 20, 0)
            item.setRotation(90)
            for port in item.data.ports:
                point = item.port_scene_position(port.port_id)
                self.assertAlmostEqual(point.x() % 20, 0)
                self.assertAlmostEqual(point.y() % 20, 0)
        window.hide()
        window.deleteLater()

    def test_component_drag_stretches_connected_wire_orthogonally(self):
        window = StudioWindow()
        first = window.hierarchy.add_node("circuit_main", "circuit.resistor", 0, 0)
        second = window.hierarchy.add_node("circuit_main", "circuit.resistor", 240, 0)
        wire_id = window.hierarchy.connect(
            "circuit_main", (first, "n"), (second, "p")
        )
        window.layer_stack = ["system_root", "circuit_main"]
        window.load_layer("circuit_main")
        item = window.scene.node_items[first]
        item.setSelected(True)
        window.scene.begin_node_drag(item)
        item.setPos(0, 80)
        window.scene.finish_node_drag(item.data.name)

        wire = window.scene.wire_items[wire_id]
        self.assertEqual(wire.full_points()[0], window.scene.port_scene_position((first, "n")))
        for start, end in zip(wire.full_points(), wire.full_points()[1:]):
            self.assertTrue(start.x() == end.x() or start.y() == end.y())
        self.assertTrue(window.scene.node_items[first].isSelected())
        window.hide()
        window.deleteLater()

    def test_wire_segment_drag_retains_endpoints_and_orthogonality(self):
        route = [QtCore.QPointF(0, 0), QtCore.QPointF(120, 0)]
        moved = _drag_orthogonal_segment(route, 0, QtCore.QPointF(60, 40))
        self.assertEqual(moved[0], route[0])
        self.assertEqual(moved[-1], route[-1])
        self.assertGreaterEqual(len(moved), 4)
        for start, end in zip(moved, moved[1:]):
            self.assertTrue(start.x() == end.x() or start.y() == end.y())

    def test_double_click_target_can_become_electrical_junction(self):
        window = StudioWindow()
        first = window.hierarchy.add_node("circuit_main", "circuit.resistor", 0, 0)
        second = window.hierarchy.add_node("circuit_main", "circuit.resistor", 240, 0)
        wire_id = window.hierarchy.connect(
            "circuit_main", (first, "n"), (second, "p")
        )
        window.layer_stack = ["system_root", "circuit_main"]
        window.load_layer("circuit_main")
        junction = window.scene.create_junction_on_wire(
            wire_id, QtCore.QPointF(120, 0)
        )

        raw = window.hierarchy.node("circuit_main", junction)
        self.assertEqual(raw["type"], "circuit.junction")
        self.assertEqual(raw["position"], {"x": 120.0, "y": 0.0})
        touching = [
            wire
            for wire in window.adapter.wires()
            if (junction, "node") in {wire.source, wire.target}
        ]
        self.assertEqual(len(touching), 2)
        self.assertEqual(window.scene.node_items[junction].data.symbol, "junction")
        window.hide()
        window.deleteLater()

    def test_right_button_drag_pans_empty_canvas(self):
        window = StudioWindow()
        window.resize(900, 600)
        window.show()
        self.application.processEvents()
        start = QtCore.QPoint(20, 20)
        end = QtCore.QPoint(70, 55)
        before = (
            window.view.horizontalScrollBar().value(),
            window.view.verticalScrollBar().value(),
        )
        window.view.mousePressEvent(
            QtGui.QMouseEvent(
                QtCore.QEvent.MouseButtonPress,
                start,
                QtCore.Qt.RightButton,
                QtCore.Qt.RightButton,
                QtCore.Qt.NoModifier,
            )
        )
        window.view.mouseMoveEvent(
            QtGui.QMouseEvent(
                QtCore.QEvent.MouseMove,
                end,
                QtCore.Qt.NoButton,
                QtCore.Qt.RightButton,
                QtCore.Qt.NoModifier,
            )
        )
        window.view.mouseReleaseEvent(
            QtGui.QMouseEvent(
                QtCore.QEvent.MouseButtonRelease,
                end,
                QtCore.Qt.RightButton,
                QtCore.Qt.NoButton,
                QtCore.Qt.NoModifier,
            )
        )
        after = (
            window.view.horizontalScrollBar().value(),
            window.view.verticalScrollBar().value(),
        )
        self.assertNotEqual(before, after)
        window.hide()
        window.deleteLater()

    def test_mosfet_body_diode_cathode_touches_tip_on_drain_side(self):
        self.assertEqual(MOSFET_BODY_DIODE_TIP.y(), MOSFET_BODY_DIODE_CATHODE_Y)
        self.assertLess(MOSFET_BODY_DIODE_CATHODE_Y, 0)


if __name__ == "__main__":
    unittest.main()
