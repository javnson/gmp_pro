import os
import sys
import unittest
from pathlib import Path


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
CORE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CORE_ROOT))

from PyQt5 import QtCore, QtGui, QtWidgets  # noqa: E402

import cctl_studio  # noqa: E402
from editor_model import EditorDocument  # noqa: E402
from hierarchy_model import HierarchyDocument  # noqa: E402
from qt_studio import LayerAdapter, PropertyInspector, StudioWindow  # noqa: E402


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
        window.scene.node_items[resistor].setSelected(True)
        window.view.keyPressEvent(
            QtGui.QKeyEvent(QtCore.QEvent.KeyPress, QtCore.Qt.Key_Space, QtCore.Qt.NoModifier)
        )
        self.assertEqual(
            window.hierarchy.node("circuit_main", resistor)["rotation"], 90
        )

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


if __name__ == "__main__":
    unittest.main()
