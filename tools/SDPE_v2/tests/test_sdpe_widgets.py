from __future__ import annotations

import os
import json
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
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication, QAbstractItemView, QCheckBox, QStyleOptionViewItem, QTableWidgetItem, QTreeWidgetItem
except ImportError:  # pragma: no cover - depends on the installed Qt binding.
    from PySide6.QtCore import Qt
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication, QAbstractItemView, QCheckBox, QStyleOptionViewItem, QTableWidgetItem, QTreeWidgetItem

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

    def test_delete_key_removes_selected_row_outside_cell_editor(self) -> None:
        table = SDPETableWidget()
        table.configure(["Name"])
        for row, name in enumerate(("First", "Second")):
            table.insertRow(row)
            table.setItem(row, 0, QTableWidgetItem(name))
        table.setCurrentCell(0, 0)
        table.setFocus()

        QTest.keyClick(table, Qt.Key.Key_Delete)

        self.assertEqual(table.rowCount(), 1)
        self.assertEqual(table.item(0, 0).text(), "Second")

    def test_delete_key_inside_editor_does_not_remove_row(self) -> None:
        table = SDPETableWidget()
        table.configure(["Name"])
        table.insertRow(0)
        item = QTableWidgetItem("Editable")
        table.setItem(0, 0, item)
        table.show()
        table.setCurrentItem(item)
        table.setFocus()
        table.editItem(item)
        self.app.processEvents()
        editor = QApplication.focusWidget()
        self.assertIsNotNone(editor)

        QTest.keyClick(editor, Qt.Key.Key_Delete)

        self.assertEqual(table.rowCount(), 1)

    def test_delete_key_removes_selected_tree_row(self) -> None:
        tree = SDPETreeWidget()
        tree.configure(["Name"])
        first = QTreeWidgetItem(["First"])
        second = QTreeWidgetItem(["Second"])
        tree.addTopLevelItems([first, second])
        tree.setCurrentItem(first)
        tree.setFocus()

        QTest.keyClick(tree, Qt.Key.Key_Delete)

        self.assertEqual(tree.topLevelItemCount(), 1)
        self.assertEqual(tree.topLevelItem(0).text(0), "Second")

    def test_delete_key_from_permanent_cell_widget_removes_tree_row(self) -> None:
        tree = SDPETreeWidget()
        tree.configure(["Name", "En"])
        item = QTreeWidgetItem(["First", ""])
        tree.addTopLevelItem(item)
        checkbox = QCheckBox()
        tree.setItemWidget(item, 1, checkbox)
        tree.setCurrentItem(item, 1)
        checkbox.setFocus()

        QTest.keyClick(checkbox, Qt.Key.Key_Delete)

        self.assertEqual(tree.topLevelItemCount(), 0)

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

    def test_column_sort_cycles_and_cut_preserves_the_selected_source_row(self) -> None:
        table = SDPETableWidget()
        table.configure(["Name", "Macro"])
        for row, values in enumerate((("Beta", "CTRL_B"), ("Alpha", "CTRL_A"), ("Gamma", "CTRL_G"))):
            table.insertRow(row)
            for col, value in enumerate(values):
                table.setItem(row, col, QTableWidgetItem(value))

        table.cycle_display_sort(0)
        self.assertEqual([table.item(row, 0).text() for row in range(table.rowCount())], ["Alpha", "Beta", "Gamma"])
        table.cycle_display_sort(0)
        self.assertEqual([table.item(row, 0).text() for row in range(table.rowCount())], ["Gamma", "Beta", "Alpha"])
        table.cycle_display_sort(0)
        self.assertEqual([table.item(row, 0).text() for row in range(table.rowCount())], ["Beta", "Alpha", "Gamma"])

        table.cycle_display_sort(0)
        table.setCurrentCell(0, 0)
        table.selectRow(0)
        table.cut()
        self.assertEqual(table.rowCount(), 2)
        self.assertEqual(QApplication.clipboard().text().split("\t", 1)[0], "Alpha")
        self.assertEqual([table.item(row, 0).text() for row in range(table.rowCount())], ["Beta", "Gamma"])

    def test_single_click_selects_while_double_click_is_the_edit_trigger(self) -> None:
        table = SDPETableWidget()
        table.configure(["Name"])
        tree = SDPETreeWidget()
        tree.configure(["Name"])
        self.assertFalse(bool(table.editTriggers() & QAbstractItemView.EditTrigger.SelectedClicked))
        self.assertFalse(bool(tree.editTriggers() & QAbstractItemView.EditTrigger.SelectedClicked))
        self.assertTrue(bool(table.editTriggers() & QAbstractItemView.EditTrigger.DoubleClicked))
        self.assertTrue(bool(tree.editTriggers() & QAbstractItemView.EditTrigger.DoubleClicked))

    def test_project_requirement_ctrl_x_cuts_the_sorted_selected_requirement(self) -> None:
        examples = ROOT / "examples"
        with tempfile.TemporaryDirectory() as output:
            window = MainWindow(examples, mode="project", default_output_dir=Path(output))
            window.show()
            page = next(item for item in window.pages if isinstance(item, ProjectPage))
            window.tabs.setCurrentWidget(page)
            page.list_widget.setCurrentRow(0)
            self.app.processEvents()

            target = next(
                item
                for item in page.iter_requirement_items()
                if page.project_item_source(item) == "project private"
            )
            macro = target.text(1)
            page.requirements.apply_display_sort(1, ascending=True)
            page.requirements.setCurrentItem(target, 1)
            target.setSelected(True)
            page.requirements.setFocus()
            QTest.keyClick(page.requirements, Qt.Key.Key_X, Qt.KeyboardModifier.ControlModifier)
            self.app.processEvents()

            self.assertNotIn(macro, [item.text(1) for item in page.iter_requirement_items()])
            self.assertIn(macro, QApplication.clipboard().text())
            window.deleteLater()
            self.app.processEvents()

    def test_search_toolbar_has_default_columns_and_enter_finds_active_requirement(self) -> None:
        examples = ROOT / "examples"
        with tempfile.TemporaryDirectory() as output:
            window = MainWindow(examples, mode="project", default_output_dir=Path(output))
            window.show()
            page = next(item for item in window.pages if isinstance(item, ProjectPage))
            window.tabs.setCurrentWidget(page)
            page.list_widget.setCurrentRow(0)
            tabs = page.form_layout.itemAt(0).widget()
            tabs.setCurrentIndex(2)
            page.requirements.setFocus()
            self.app.processEvents()

            self.assertGreater(window.data_sort_column.count(), 0)
            window.data_search.setText("CTRL_INDUCTOR_CURRENT_SENSITIVITY")
            QTest.keyClick(window.data_search, Qt.Key.Key_Return)
            self.app.processEvents()
            self.assertEqual(page.requirements.currentItem().text(1), "CTRL_INDUCTOR_CURRENT_SENSITIVITY")
            window.deleteLater()
            self.app.processEvents()

    def test_project_code_page_edits_single_bound_common_code_sections(self) -> None:
        examples = ROOT / "examples"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            common_path = root / "common.json"
            project_path = root / "private.json"
            common_path.write_text(
                json.dumps(
                    {
                        "id": "common",
                        "output_header": "common.h",
                        "code_sections": {
                            "after_extern_open": "#include <common_prefix.h>",
                            "before_footer": "#define COMMON_TAIL 1",
                        },
                    }
                ),
                encoding="utf-8",
            )
            project_path.write_text(
                json.dumps(
                    {
                        "id": "private",
                        "output_header": "private.h",
                        "common_requirements": ["common.json"],
                        "code_sections": {
                            "after_extern_open": "#include <project_prefix.h>",
                            "before_footer": "#define PROJECT_TAIL 1",
                        },
                    }
                ),
                encoding="utf-8",
            )
            window = MainWindow(examples, mode="project", project_dirs=[project_path], default_output_dir=root / "out")
            page = next(item for item in window.pages if isinstance(item, ProjectPage))
            page.current_id = "private"
            page.refresh_list()
            page.load_current()

            self.assertFalse(page.common_prefix_code.isReadOnly())
            self.assertEqual(page.common_prefix_code.toPlainText(), "#include <common_prefix.h>")
            self.assertEqual(page.common_tail_code.toPlainText(), "#define COMMON_TAIL 1")
            self.assertEqual(page.prefix_code.toPlainText(), "#include <project_prefix.h>")
            self.assertEqual(page.tail_code.toPlainText(), "#define PROJECT_TAIL 1")
            page.common_tail_code.setPlainText("#define COMMON_TAIL 2")
            snapshot = page.collect_current_data()
            self.assertEqual(
                snapshot["__sdpe_common_documents"][str(common_path.resolve())]["code_sections"]["before_footer"],
                "#define COMMON_TAIL 2",
            )
            window.deleteLater()
            self.app.processEvents()

    def test_undo_back_to_clean_snapshot_clears_dirty_state(self) -> None:
        examples = ROOT / "examples"
        with tempfile.TemporaryDirectory() as output:
            window = MainWindow(examples, mode="project", default_output_dir=Path(output))
            page = next(item for item in window.pages if isinstance(item, ProjectPage))
            window.tabs.setCurrentWidget(page)
            page.list_widget.setCurrentRow(0)
            self.app.processEvents()
            item = page.iter_requirement_items()[0]
            original = item.text(7)
            item.setText(7, original + " changed")
            page.capture_undo_snapshot()
            self.assertIn(page.current_id, page.dirty_ids)

            page.undo_current_change()

            self.assertNotIn(page.current_id, page.dirty_ids)
            self.assertFalse(page.list_widget.currentItem().text().endswith(" *"))
            window.deleteLater()
            self.app.processEvents()

    def test_common_create_cover_delete_and_ctrl_s_write_the_common_file(self) -> None:
        examples = ROOT / "examples"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            common_path = root / "common.json"
            project_path = root / "private.json"
            common_path.write_text(
                json.dumps({"id": "base", "output_header": "base.h", "requirements": []}),
                encoding="utf-8",
            )
            project_path.write_text(
                json.dumps(
                    {
                        "id": "private",
                        "output_header": "private.h",
                        "common_requirements": ["common.json"],
                        "requirements": [],
                    }
                ),
                encoding="utf-8",
            )
            window = MainWindow(
                examples,
                mode="project",
                project_dirs=[project_path],
                default_output_dir=root / "out",
            )
            window.show()
            page = next(item for item in window.pages if isinstance(item, ProjectPage))
            page.current_id = "private"
            page.refresh_list()
            page.load_current()
            tabs = page.form_layout.itemAt(0).widget()
            tabs.setCurrentIndex(2)
            self.app.processEvents()

            page.add_common_requirement()
            common_item = page.requirements.currentItem()
            self.assertTrue(page.project_item_source(common_item).startswith("common:"))
            editor = QApplication.focusWidget()
            editor.selectAll()
            QTest.keyClicks(editor, "Saved Common Requirement")
            QTest.keyClick(editor, Qt.Key.Key_S, Qt.KeyboardModifier.ControlModifier)
            self.app.processEvents()

            saved_common = json.loads(common_path.read_text(encoding="utf-8"))
            saved_private = json.loads(project_path.read_text(encoding="utf-8"))
            self.assertEqual(saved_common["requirements"][0]["role"], "Saved Common Requirement")
            self.assertEqual(saved_private["requirements"], [])

            common_item = next(
                item
                for item in page.iter_requirement_items()
                if page.project_item_source(item).startswith("common:")
            )
            page.override_common_item("requirements", page.requirements, common_item)
            page.save_current()
            saved_common = json.loads(common_path.read_text(encoding="utf-8"))
            saved_private = json.loads(project_path.read_text(encoding="utf-8"))
            self.assertTrue(saved_common["requirements"][0]["weak"])
            self.assertEqual(saved_private["requirements"][0]["macro"], "NEW_COMMON_REQUIREMENT")

            common_item = next(
                item
                for item in page.iter_requirement_items()
                if page.project_item_source(item).startswith("common:")
            )
            page.requirements.setCurrentItem(common_item, 0)
            page.requirements.editItem(common_item, 0)
            self.app.processEvents()
            editor = QApplication.focusWidget()
            editor.selectAll()
            QTest.keyClicks(editor, "Covered Common Updated")
            QTest.keyClick(editor, Qt.Key.Key_S, Qt.KeyboardModifier.ControlModifier)
            self.app.processEvents()
            self.assertEqual(
                json.loads(common_path.read_text(encoding="utf-8"))["requirements"][0]["role"],
                "Covered Common Updated",
            )
            self.assertEqual(
                json.loads(project_path.read_text(encoding="utf-8"))["requirements"][0]["role"],
                "Saved Common Requirement",
            )

            common_item = next(
                item
                for item in page.iter_requirement_items()
                if page.project_item_source(item).startswith("common:")
            )
            page.requirements.setCurrentItem(common_item, 2)
            checkbox = page.requirements.itemWidget(common_item, 2)._sdpe_checkbox
            checkbox.setFocus()
            QTest.keyClick(checkbox, Qt.Key.Key_Delete)
            page.save_current()
            self.assertEqual(json.loads(common_path.read_text(encoding="utf-8"))["requirements"], [])
            self.assertEqual(len(json.loads(project_path.read_text(encoding="utf-8"))["requirements"]), 1)

            page.add_common_requirement()
            new_common = page.requirements.currentItem()
            owning_group = page.item_group_ancestor(new_common)
            page.requirements.clearSelection()
            page.requirements.setCurrentItem(owning_group, 0)
            owning_group.setSelected(True)
            page.remove_requirement_item()
            self.assertEqual(page.iter_requirement_items(), [])
            page.save_current()
            self.assertEqual(json.loads(common_path.read_text(encoding="utf-8"))["requirements"], [])
            self.assertEqual(json.loads(project_path.read_text(encoding="utf-8"))["requirements"], [])
            window.deleteLater()
            self.app.processEvents()

    def test_common_requirement_is_editable_saved_and_nested_under_private_override(self) -> None:
        examples = ROOT / "examples"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            common_path = root / "common.json"
            project_path = root / "private.json"
            common_path.write_text(
                json.dumps(
                    {
                        "id": "base",
                        "output_header": "base.h",
                        "requirements": [
                            {
                                "role": "Common Gain",
                                "macro": "CTRL_GAIN",
                                "enabled": True,
                                "binding": {"number": "1"},
                            },
                            {
                                "role": "Common Limit",
                                "macro": "CTRL_LIMIT",
                                "binding": {"number": "3"},
                            },
                        ],
                        "requirement_groups": [
                            {"name": "Control", "requirements": ["Common Gain", "Common Limit"]}
                        ],
                        "feature_macros": [
                            {"macro": "CTRL_MODE", "value": "1", "group": "Control", "enabled": True}
                        ],
                    }
                ),
                encoding="utf-8",
            )
            project_path.write_text(
                json.dumps(
                    {
                        "id": "private",
                        "output_header": "private.h",
                        "common_requirements": ["common.json"],
                        "requirements": [
                            {"role": "Private Gain", "macro": "CTRL_GAIN", "binding": {"number": "2"}}
                        ],
                        "requirement_groups": [{"name": "Control", "requirements": ["Private Gain"]}],
                        "feature_macros": [
                            {"macro": "CTRL_MODE", "value": "2", "group": "Control", "enabled": True}
                        ],
                    }
                ),
                encoding="utf-8",
            )
            window = MainWindow(
                examples,
                mode="project",
                project_dirs=[project_path],
                default_output_dir=root / "out",
            )
            page = next(item for item in window.pages if isinstance(item, ProjectPage))
            page.current_id = "private"
            page.refresh_list()
            page.load_current()
            private_item = next(
                item for item in page.iter_requirement_items() if page.project_item_source(item) == "project private"
            )
            common_item = next(
                item for item in page.iter_requirement_items() if page.project_item_source(item).startswith("common:")
            )

            self.assertIs(common_item.parent(), private_item)
            self.assertTrue(common_item.flags() & Qt.ItemFlag.ItemIsEditable)
            self.assertTrue(common_item.data(3, Qt.ItemDataRole.UserRole))
            old_enable_widget = page.requirements.itemWidget(common_item, 2)
            owning_group = page.item_group_ancestor(common_item)
            page.reparent_project_item(common_item, owning_group)
            self.assertIsNot(page.requirements.itemWidget(common_item, 2), old_enable_widget)
            page.enforce_override_hierarchy()
            self.assertIs(common_item.parent(), private_item)
            common_item.setText(0, "Edited Common Gain")
            common_checkbox = page.requirements.itemWidget(common_item, 2)._sdpe_checkbox
            common_checkbox.setChecked(False)
            private_macro = next(
                item
                for item in page.iter_macro_items(page.feature_macros)
                if page.project_item_source(item) == "project private" and item.text(0) == "CTRL_MODE"
            )
            common_macro = next(
                item
                for item in page.iter_macro_items(page.feature_macros)
                if page.project_item_source(item).startswith("common:") and item.text(0) == "CTRL_MODE"
            )
            self.assertIs(common_macro.parent(), private_macro)
            self.assertTrue(common_macro.flags() & Qt.ItemFlag.ItemIsEditable)
            page.feature_macros.itemWidget(common_macro, 1)._sdpe_checkbox.setChecked(False)
            common_limit = next(
                item
                for item in page.iter_requirement_items()
                if page.project_item_source(item).startswith("common:") and item.text(1) == "CTRL_LIMIT"
            )
            page.override_common_item("requirements", page.requirements, common_limit)
            private_limit = next(
                item
                for item in page.iter_requirement_items()
                if page.project_item_source(item) == "project private" and item.text(1) == "CTRL_LIMIT"
            )
            self.assertIs(common_limit.parent(), private_limit)
            page.save_current()

            saved_common = json.loads(common_path.read_text(encoding="utf-8"))
            saved_private = json.loads(project_path.read_text(encoding="utf-8"))
            self.assertEqual(saved_common["requirements"][0]["role"], "Edited Common Gain")
            self.assertFalse(saved_common["requirements"][0]["enabled"])
            self.assertTrue(saved_common["requirements"][0]["weak"])
            self.assertTrue(saved_common["requirements"][1]["weak"])
            self.assertFalse(saved_common["feature_macros"][0]["enabled"])
            self.assertTrue(saved_common["feature_macros"][0]["weak"])
            self.assertIn("CTRL_LIMIT", {row["macro"] for row in saved_private["requirements"]})
            self.assertNotIn("__sdpe_common_documents", saved_private)

            private_limit = next(
                item
                for item in page.iter_requirement_items()
                if page.project_item_source(item) == "project private" and item.text(1) == "CTRL_LIMIT"
            )
            common_limit = next(
                item
                for item in page.iter_requirement_items()
                if page.project_item_source(item).startswith("common:") and item.text(1) == "CTRL_LIMIT"
            )
            page.remove_requirement_items([private_limit])
            self.assertIn(common_limit, page.iter_requirement_items())
            self.assertEqual(common_limit.parent().data(0, Qt.ItemDataRole.UserRole), "group")
            self.assertTrue(page.requirements.itemWidget(common_limit, 3)._sdpe_checkbox.isEnabled())

            nested_group = page.requirements.ensure_group_path(
                "Control / Limits",
                page.create_requirement_group_item,
            )
            nested_item = page.add_requirement_item(
                nested_group,
                {"role": "Nested Limit", "macro": "CTRL_NESTED_LIMIT", "binding": {"number": "4"}},
            )
            groups = page._requirement_groups()
            memberships = [
                group["name"]
                for group in groups
                if nested_item.text(0) in group["requirements"]
            ]
            self.assertEqual(memberships, ["Control / Limits"])
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
