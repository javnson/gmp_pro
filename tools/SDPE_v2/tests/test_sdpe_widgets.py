from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    from PyQt6.QtCore import Qt
    from PyQt6.QtWidgets import QApplication, QStyleOptionViewItem, QTableWidgetItem, QTreeWidgetItem
except ImportError:  # pragma: no cover - depends on the installed Qt binding.
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QApplication, QStyleOptionViewItem, QTableWidgetItem, QTreeWidgetItem

from gui_pyqt.sdpe_widgets import SDPEComboBox, SDPETableWidget, SDPETreeWidget
from gui_pyqt.sdpe_gui import MainWindow, ProjectPage


class SDPEWidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_table_clipboard_preserves_cell_widgets(self) -> None:
        table = SDPETableWidget()
        table.configure(["Name", "Mode", "Description"])
        table.insertRow(0)
        table.setItem(0, 0, QTableWidgetItem("sensor"))
        mode = SDPEComboBox()
        mode.setEditable(True)
        mode.addItems(["entity", "inline"])
        mode.setCurrentText("inline")
        table.setCellWidget(0, 1, mode)
        table.setItem(0, 2, QTableWidgetItem("Current sensor"))
        table.selectRow(0)

        table.copy()
        table.paste()

        self.assertEqual(table.rowCount(), 2)
        self.assertEqual(table.item(1, 0).text(), "sensor")
        self.assertEqual(table.cellWidget(1, 1).currentText(), "inline")

    def test_tree_clipboard_preserves_nested_groups(self) -> None:
        tree = SDPETreeWidget()
        tree.configure(["Name", "Value"], draggable=True)
        outer = QTreeWidgetItem(["Outer", ""])
        outer.setData(0, Qt.ItemDataRole.UserRole, "group")
        inner = QTreeWidgetItem(["Inner", ""])
        inner.setData(0, Qt.ItemDataRole.UserRole, "group")
        leaf = QTreeWidgetItem(["GAIN", "1"])
        leaf.setData(0, Qt.ItemDataRole.UserRole, "macro")
        tree.addTopLevelItem(outer)
        outer.addChild(inner)
        inner.addChild(leaf)

        self.assertEqual(tree.group_path(leaf), "Outer / Inner")
        tree.setCurrentItem(outer)
        outer.setSelected(True)
        tree.copy()
        tree.paste()

        copied_outer = outer.child(1)
        self.assertEqual(copied_outer.text(0), "Outer")
        self.assertEqual(copied_outer.child(0).child(0).text(0), "GAIN")

    def test_action_handler_can_keep_page_specific_serialization(self) -> None:
        tree = SDPETreeWidget()
        calls = []
        tree.set_action_handler("copy", lambda: calls.append("copy"))

        tree.copy()

        self.assertEqual(calls, ["copy"])

    def test_structural_action_emits_one_undo_transaction(self) -> None:
        table = SDPETableWidget()
        table.configure(["Name"])
        events = []
        table.mutationStarted.connect(lambda: events.append("before"))
        table.mutationFinished.connect(lambda: events.append("after"))

        def add_row() -> None:
            table.insertRow(table.rowCount())
            table.setItem(table.rowCount() - 1, 0, QTableWidgetItem("new row"))

        table.set_insert_handlers(add_item=add_row, item_label="Add row")
        table.add_item()

        self.assertEqual(events, ["before", "after"])
        self.assertEqual(table.rowCount(), 1)
        self.assertEqual(table.action("add_item").text(), "Add row")

    def test_editor_is_opaque_to_prevent_text_ghosting(self) -> None:
        tree = SDPETreeWidget()
        tree.configure(["Name"])
        item = QTreeWidgetItem(["Editable value"])
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable)
        tree.addTopLevelItem(item)
        index = tree.indexFromItem(item, 0)

        editor = tree.itemDelegate().createEditor(tree.viewport(), QStyleOptionViewItem(), index)

        self.assertIsNotNone(editor)
        self.assertTrue(editor.autoFillBackground())
        self.assertTrue(editor.testAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent))
        editor.deleteLater()

    def test_status_description_is_available_without_hover(self) -> None:
        table = SDPETableWidget()
        table.configure(["Name", "Description"])
        table.insertRow(0)
        table.setItem(0, 0, QTableWidgetItem("sensor"))
        table.setItem(0, 1, QTableWidgetItem("Current feedback sensor"))
        table.setCurrentCell(0, 0)
        status = []
        table.statusTextChanged.connect(status.append)

        table.publish_current_status()

        self.assertEqual(status, ["Current feedback sensor"])

    def test_search_and_display_sort_preserve_source_row_order(self) -> None:
        table = SDPETableWidget()
        table.configure(["Name", "Macro"])
        for row, values in enumerate((("Zulu", "CTRL_Z"), ("Alpha", "CTRL_A"))):
            table.insertRow(row)
            for col, value in enumerate(values):
                table.setItem(row, col, QTableWidgetItem(value))

        table.apply_display_sort(0, ascending=True)
        self.assertEqual(table.item(0, 0).text(), "Alpha")
        self.assertEqual(
            [table.item(row, 0).text() for row in table.source_row_numbers()],
            ["Zulu", "Alpha"],
        )
        self.assertTrue(table.apply_text_filter(r"CTRL_[AZ]", regex=True))
        self.assertTrue(table.find_text("ctrl_a", case_sensitive=False))
        self.assertFalse(table.apply_text_filter("[", regex=True))

    def test_undo_back_to_clean_snapshot_clears_dirty_state(self) -> None:
        examples = ROOT / "examples"
        with tempfile.TemporaryDirectory() as output:
            window = MainWindow(examples, mode="project", default_output_dir=Path(output))
            page = next(item for item in window.pages if isinstance(item, ProjectPage))
            window.tabs.setCurrentWidget(page)
            page.list_widget.setCurrentRow(0)
            self.app.processEvents()
            item = page.iter_requirement_items()[0]
            original = item.text(6)
            item.setText(6, original + " changed")
            page.capture_undo_snapshot()
            self.assertIn(page.current_id, page.dirty_ids)

            page.undo_current_change()

            self.assertNotIn(page.current_id, page.dirty_ids)
            self.assertFalse(page.list_widget.currentItem().text().endswith(" *"))
            window.deleteLater()
            self.app.processEvents()

    def test_main_window_tracks_one_active_view_and_undoes_group_creation(self) -> None:
        examples = ROOT / "examples"
        with tempfile.TemporaryDirectory() as output:
            window = MainWindow(examples, mode="project", default_output_dir=Path(output))
            page = next(item for item in window.pages if isinstance(item, ProjectPage))
            window.tabs.setCurrentWidget(page)
            page.list_widget.setCurrentRow(0)
            self.app.processEvents()

            requirement = page.iter_requirement_items()[0]
            page.requirements.setCurrentItem(requirement)
            requirement.setSelected(True)
            macro = page.iter_macro_items(page.feature_macros)[0]
            page.feature_macros.setCurrentItem(macro)
            macro.setSelected(True)
            self.app.processEvents()

            self.assertFalse(page.requirements.selectedItems())
            self.assertIs(window.active_data_view, page.feature_macros)
            self.assertEqual(window.action_add_item.text(), "Add selection macro")

            before = sum(
                item.data(0, Qt.ItemDataRole.UserRole) == "group"
                for item in page.feature_macros.iter_items()
            )
            macro_name = macro.text(0)
            original_group = page.feature_macros.group_path(macro)
            page.feature_macros.add_group()
            new_group = page.feature_macros.currentItem()
            after = sum(
                item.data(0, Qt.ItemDataRole.UserRole) == "group"
                for item in page.feature_macros.iter_items()
            )
            self.assertEqual(after, before + 1)

            page.feature_macros.mutationStarted.emit()
            parent = macro.parent()
            parent.takeChild(parent.indexOfChild(macro))
            new_group.addChild(macro)
            page.feature_macros.structureChanged.emit()
            page.feature_macros.contentChanged.emit()
            page.feature_macros.mutationFinished.emit()
            moved = next(item for item in page.collect_feature_macro_tree() if item["macro"] == macro_name)
            self.assertNotEqual(moved["group"], original_group)

            page.undo_current_change()
            restored_macro = next(item for item in page.collect_feature_macro_tree() if item["macro"] == macro_name)
            self.assertEqual(restored_macro["group"], original_group)
            self.assertEqual(
                sum(
                    item.data(0, Qt.ItemDataRole.UserRole) == "group"
                    for item in page.feature_macros.iter_items()
                ),
                after,
            )

            page.undo_current_change()
            restored = sum(
                item.data(0, Qt.ItemDataRole.UserRole) == "group"
                for item in page.feature_macros.iter_items()
            )
            self.assertEqual(restored, before)
            self.assertTrue(window.status_context.text())
            window.apply_theme("dark")
            self.assertEqual(window.theme, "dark")
            window.apply_theme("system")
            window.deleteLater()
            self.app.processEvents()


if __name__ == "__main__":
    unittest.main()
