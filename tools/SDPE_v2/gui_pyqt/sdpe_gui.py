"""PyQt manager for SDPE v2 hardware templates, entities, and bindings."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

try:
    from PyQt6.QtCore import Qt
    from PyQt6.QtGui import QKeySequence, QShortcut
    from PyQt6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDialog,
        QDialogButtonBox,
        QFileDialog,
        QFormLayout,
        QHBoxLayout,
        QHeaderView,
        QInputDialog,
        QLabel,
        QLineEdit,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QMenu,
        QMessageBox,
        QPushButton,
        QSplitter,
        QTableWidget,
        QTableWidgetItem,
        QTabWidget,
        QTextEdit,
        QToolBar,
        QTreeWidget,
        QTreeWidgetItem,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover - depends on local desktop environment.
    try:
        from PySide6.QtCore import Qt
        from PySide6.QtGui import QKeySequence, QShortcut
        from PySide6.QtWidgets import (
            QApplication,
            QCheckBox,
            QComboBox,
            QDialog,
            QDialogButtonBox,
            QFileDialog,
            QFormLayout,
            QHBoxLayout,
            QHeaderView,
            QInputDialog,
            QLabel,
            QLineEdit,
            QListWidget,
            QListWidgetItem,
            QMainWindow,
            QMenu,
            QMessageBox,
            QPushButton,
            QSplitter,
            QTableWidget,
            QTableWidgetItem,
            QTabWidget,
            QTextEdit,
            QToolBar,
            QTreeWidget,
            QTreeWidgetItem,
            QVBoxLayout,
            QWidget,
        )
    except ImportError:
        print("SDPE PyQt GUI requires PyQt6 or PySide6.", file=sys.stderr)
        raise

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sdpe_v2.generator import HeaderGenerator
from sdpe_v2.library import SDPELibrary
from sdpe_v2.model import HardwareEntity, HardwareSchema, SDPEError
from sdpe_v2.util import read_json


def pretty_json(data: Any) -> str:
    """Format JSON for the preview/editor pane."""

    return json.dumps(data, indent=2, ensure_ascii=False)


def parse_json_text(text: str, fallback: Any) -> Any:
    """Parse optional JSON text."""

    value = text.strip()
    if not value:
        return fallback
    try:
        return json.loads(value)
    except json.JSONDecodeError:
        if fallback == value:
            return fallback
        raise


FORMAT_TO_LABEL = {
    "{}": "Custom",
    "raw": "Raw C",
    "({}f)": "Float",
    "{}f": "Float",
    "({})": "Integer",
    "({}U)": "Unsigned Integer",
    '"{}"': "String",
}
LABEL_TO_FORMAT = {
    "String": '"{}"',
    "Float": "({}f)",
    "Integer": "({})",
    "Unsigned Integer": "({}U)",
    "Raw C": "raw",
    "Custom": "{}",
}


def format_label(value_format: str) -> str:
    return FORMAT_TO_LABEL.get(value_format, value_format or "Custom")


def format_value(label: str) -> str:
    return LABEL_TO_FORMAT.get(label, label or "{}")


def validate_value_for_format(name: str, value: Any, value_format: str) -> None:
    if isinstance(value, dict):
        return
    if value == "":
        return
    label = format_label(value_format)
    if label in {"Raw C", "Custom"}:
        return
    if isinstance(value, str) and looks_like_c_expression(value):
        return
    if label == "String":
        if not isinstance(value, str):
            raise ValueError(f"Parameter '{name}' expects a string value.")
    elif label == "Float":
        if not isinstance(value, (int, float)):
            raise ValueError(f"Parameter '{name}' expects a numeric float value.")
    elif label in {"Integer", "Unsigned Integer"}:
        if not isinstance(value, int) or isinstance(value, bool):
            raise ValueError(f"Parameter '{name}' expects an integer value.")
        if label == "Unsigned Integer" and value < 0:
            raise ValueError(f"Parameter '{name}' expects an unsigned integer value.")


def looks_like_c_expression(value: str) -> bool:
    text = value.strip()
    return bool(text) and (
        text.startswith("(")
        or text.isidentifier()
        or any(token in text for token in ("+", "-", "*", "/", "_"))
    )


def parameter_value_by_name(parameters: dict[str, Any], name: str, c_name: str) -> Any:
    if name in parameters:
        return parameters[name]
    c_key = c_name.lower()
    if c_key in parameters:
        return parameters[c_key]
    for key, value in parameters.items():
        if key.lower() == c_key:
            return value
    return ""


def set_table_headers(
    table: QTableWidget, headers: list[str], mode: QHeaderView.ResizeMode = QHeaderView.ResizeMode.Stretch
) -> None:
    table.setColumnCount(len(headers))
    table.setHorizontalHeaderLabels(headers)
    table.horizontalHeader().setSectionResizeMode(mode)
    table.horizontalHeader().setStretchLastSection(True)
    table.verticalHeader().setVisible(False)
    table.setAlternatingRowColors(True)
    table.resizeColumnsToContents()


def item_text(table: QTableWidget, row: int, col: int) -> str:
    widget = table.cellWidget(row, col)
    if isinstance(widget, QComboBox):
        return widget.currentText().strip()
    item = table.item(row, col)
    return "" if item is None else item.text().strip()


def set_item(table: QTableWidget, row: int, col: int, value: Any) -> None:
    table.setItem(row, col, QTableWidgetItem("" if value is None else str(value)))


def set_combo(table: QTableWidget, row: int, col: int, values: list[str], current: Any = "") -> None:
    combo = QComboBox()
    combo.addItems(values)
    combo.setEditable(True)
    combo.setCurrentText("" if current is None else str(current))
    combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
    width = max([combo.fontMetrics().horizontalAdvance(value) for value in values + [combo.currentText()]] or [0]) + 48
    combo.setMinimumWidth(width)
    table.setCellWidget(row, col, combo)
    table.setColumnWidth(col, max(table.columnWidth(col), width + 12))


def edit_multiline(parent: QWidget, title: str, text: str) -> str | None:
    dialog = QDialog(parent)
    dialog.setWindowTitle(title)
    dialog.resize(620, 360)
    edit = QTextEdit()
    edit.setPlainText(text)
    buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    layout = QVBoxLayout(dialog)
    layout.addWidget(edit)
    layout.addWidget(buttons)
    if dialog.exec() == QDialog.DialogCode.Accepted:
        return edit.toPlainText()
    return None


def choose_item(parent: QWidget, title: str, items: list[str]) -> str | None:
    dialog = QDialog(parent)
    dialog.setWindowTitle(title)
    dialog.resize(560, 420)
    search = QLineEdit()
    search.setPlaceholderText("Search")
    list_widget = QListWidget()

    def populate() -> None:
        query = search.text().strip().lower()
        list_widget.clear()
        for item in items:
            if not query or all(part in item.lower() for part in query.split()):
                list_widget.addItem(item)

    search.textChanged.connect(populate)
    populate()
    buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    list_widget.itemDoubleClicked.connect(lambda _item: dialog.accept())
    layout = QVBoxLayout(dialog)
    layout.addWidget(search)
    layout.addWidget(list_widget)
    layout.addWidget(buttons)
    if dialog.exec() == QDialog.DialogCode.Accepted and list_widget.currentItem():
        return list_widget.currentItem().text()
    return None


def prompt_identifier(parent: QWidget, title: str, label: str) -> str:
    text, ok = QInputDialog.getText(parent, title, label)
    if not ok:
        return ""
    return text.strip()


def confirm_delete(parent: QWidget, title: str, text: str) -> bool:
    result = QMessageBox.question(
        parent,
        title,
        text,
        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        QMessageBox.StandardButton.No,
    )
    return result == QMessageBox.StandardButton.Yes


class SDPEPage(QWidget):
    """Common split view page: searchable object list, form editor, JSON/code preview."""

    def __init__(self, window: "MainWindow", title: str, has_code: bool = True):
        super().__init__()
        self.window = window
        self.title = title
        self.current_id = ""
        self.has_code = has_code
        self.loading = False
        self.drafts: dict[str, dict[str, Any]] = {}
        self.dirty_ids: set[str] = set()

        self.search = QLineEdit()
        self.search.setPlaceholderText("Search id, name, tag")
        self.search.textChanged.connect(self.refresh_list)
        self.list_widget = self.create_items_widget()

        self.form_panel = QWidget()
        self.form_layout = QVBoxLayout(self.form_panel)
        self.professional_panel = QTextEdit()
        self.professional_panel.setReadOnly(True)
        self.professional_panel.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        self.code_panel = QTextEdit()
        self.code_panel.setReadOnly(True)
        self.code_panel.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)

        self.items_panel = QWidget()
        left_layout = QVBoxLayout(self.items_panel)
        left_layout.addWidget(self.search)
        left_layout.addWidget(self.list_widget)

        self.splitter = QSplitter()
        self.splitter.addWidget(self.items_panel)
        self.splitter.addWidget(self.form_panel)
        self.splitter.addWidget(self.professional_panel)
        self.splitter.addWidget(self.code_panel)
        self.splitter.setSizes([240, 760, 460, 520])

        self.show_items = QCheckBox("Items")
        self.show_items.setChecked(True)
        self.show_items.toggled.connect(self.items_panel.setVisible)
        self.show_items.toggled.connect(lambda _checked: self.update_panel_sizes())
        self.show_basic = QCheckBox("Basic")
        self.show_basic.setChecked(True)
        self.show_basic.toggled.connect(self.form_panel.setVisible)
        self.show_basic.toggled.connect(lambda _checked: self.update_panel_sizes())
        self.show_professional = QCheckBox("Professional")
        self.show_professional.setChecked(False)
        self.show_professional.toggled.connect(self.professional_panel.setVisible)
        self.show_professional.toggled.connect(lambda _checked: self.update_panel_sizes())
        self.show_code = QCheckBox("Code")
        self.show_code.setChecked(False)
        self.show_code.toggled.connect(self.code_panel.setVisible)
        self.show_code.toggled.connect(lambda _checked: self.update_panel_sizes())
        self.professional_panel.setVisible(False)
        self.code_panel.setVisible(False)
        if not self.has_code:
            self.show_code.setEnabled(False)

        toolbar = QToolBar(title)
        toolbar.addWidget(QLabel(title))
        toolbar.addSeparator()
        toolbar.addWidget(self.show_items)
        toolbar.addWidget(self.show_basic)
        toolbar.addWidget(self.show_professional)
        toolbar.addWidget(self.show_code)

        layout = QVBoxLayout(self)
        layout.addWidget(toolbar)
        layout.addWidget(self.splitter)
        self.save_shortcut = QShortcut(QKeySequence.StandardKey.Save, self)
        self.save_shortcut.activated.connect(self.save_current)
        self.save_all_shortcut = QShortcut(QKeySequence("Ctrl+Shift+S"), self)
        self.save_all_shortcut.activated.connect(self.save_all)
        self.undo_shortcut = QShortcut(QKeySequence.StandardKey.Undo, self)
        self.undo_shortcut.activated.connect(self.undo_focused_widget)

    def create_items_widget(self):
        widget = QListWidget()
        widget.currentItemChanged.connect(self.on_current_changed)
        return widget

    def refresh_list(self) -> None:
        raise NotImplementedError

    def on_current_changed(self, current) -> None:
        if current is None:
            return
        item_id = current.data(0, Qt.ItemDataRole.UserRole) if isinstance(current, QTreeWidgetItem) else current.data(Qt.ItemDataRole.UserRole)
        if not item_id:
            return
        if self.current_id and item_id != self.current_id:
            self.store_current_draft()
        self.current_id = item_id
        self.load_current()

    def load_current(self) -> None:
        raise NotImplementedError

    def save_current(self) -> None:
        return None

    def collect_current_data(self) -> dict[str, Any] | None:
        return None

    def path_for_id(self, item_id: str) -> Path:
        raise NotImplementedError

    def mark_current_dirty(self) -> None:
        if self.loading or not self.current_id:
            return
        self.dirty_ids.add(self.current_id)
        current = self.list_widget.currentItem() if isinstance(self.list_widget, QTreeWidget) else self.list_widget.currentItem()
        if current is not None:
            if isinstance(current, QTreeWidgetItem):
                text = current.text(0)
                if not text.endswith(" *"):
                    current.setText(0, f"{text} *")
            else:
                text = current.text()
                if not text.endswith(" *"):
                    current.setText(f"{text} *")

    def store_current_draft(self) -> None:
        if self.loading or not self.current_id:
            return
        data = self.collect_current_data()
        if data is None:
            return
        self.drafts[self.current_id] = data
        path = self.path_for_id(self.current_id)
        old = read_json(path) if path.exists() else {}
        if pretty_json(data) != pretty_json(old):
            self.dirty_ids.add(self.current_id)

    def data_for_id(self, item_id: str, path: Path) -> dict[str, Any]:
        if item_id in self.drafts:
            return self.drafts[item_id]
        return read_json(path)

    def save_all(self) -> None:
        self.store_current_draft()
        saved = 0
        for item_id in list(self.dirty_ids):
            data = self.drafts.get(item_id)
            if data is None:
                continue
            self.window.write_json(self.path_for_id(item_id), data)
            saved += 1
        self.dirty_ids.clear()
        self.drafts.clear()
        if saved:
            self.window.reload()
            self.refresh_list()
            self.message("Saved", f"Saved {saved} file(s).")

    def set_professional_text(self, text: str) -> None:
        self.professional_panel.setPlainText(text)

    def set_code_text(self, text: str, reveal: bool = True) -> None:
        self.code_panel.setPlainText(text)
        if self.has_code and reveal:
            self.show_code.setChecked(True)
            self.update_panel_sizes()

    def update_panel_sizes(self) -> None:
        sizes = [
            240 if self.show_items.isChecked() else 0,
            760 if self.show_basic.isChecked() else 0,
            460 if self.show_professional.isChecked() else 0,
            560 if self.show_code.isChecked() and self.has_code else 0,
        ]
        if sum(sizes) > 0:
            self.splitter.setSizes(sizes)

    def undo_focused_widget(self) -> None:
        focused = QApplication.focusWidget()
        if focused is not None and hasattr(focused, "undo"):
            focused.undo()

    def select_first(self) -> None:
        if isinstance(self.list_widget, QTreeWidget):
            if self.list_widget.currentItem() is None and self.list_widget.topLevelItemCount():
                root = self.list_widget.topLevelItem(0)
                leaf = self.first_tree_leaf(root)
                if leaf is not None:
                    self.list_widget.setCurrentItem(leaf)
            return
        if self.list_widget.count() and self.list_widget.currentRow() < 0:
            self.list_widget.setCurrentRow(0)

    def first_tree_leaf(self, item: QTreeWidgetItem | None) -> QTreeWidgetItem | None:
        if item is None:
            return None
        if item.data(0, Qt.ItemDataRole.UserRole):
            return item
        for index in range(item.childCount()):
            leaf = self.first_tree_leaf(item.child(index))
            if leaf is not None:
                return leaf
        return None

    def filter_match(self, haystack: list[str]) -> bool:
        query = self.search.text().strip().lower()
        if not query:
            return True
        blob = " ".join(haystack).lower()
        return all(part in blob for part in query.split())

    def message(self, title: str, text: str) -> None:
        QMessageBox.information(self, title, text)

    def error(self, text: str) -> None:
        QMessageBox.critical(self, self.title, text)


class TemplatePage(SDPEPage):
    """Template/schema object editor."""

    def create_items_widget(self):
        widget = QTreeWidget()
        widget.setHeaderHidden(True)
        widget.currentItemChanged.connect(lambda current, _previous: self.on_current_changed(current))
        widget.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        widget.customContextMenuRequested.connect(self.show_template_context_menu)
        return widget

    def __init__(self, window: "MainWindow"):
        super().__init__(window, "Template Definition", has_code=False)
        self.id_edit = QLineEdit()
        self.name_edit = QLineEdit()
        self.category_edit = QLineEdit()
        self.tags_edit = QLineEdit()
        self.description_edit = QTextEdit()
        self.output_edit = QLineEdit()
        self.header_prefix_edit = QLineEdit()
        self.params = QTableWidget()
        set_table_headers(
            self.params,
            ["Name", "Macro Name", "Default", "Unit", "Required", "Format", "Description"],
            QHeaderView.ResizeMode.Interactive,
        )
        self.params.cellDoubleClicked.connect(self.on_template_param_double_clicked)
        self.slots = QTableWidget()
        set_table_headers(
            self.slots,
            ["Slot", "Mode", "Entity or Inline JSON", "Overrides JSON"],
            QHeaderView.ResizeMode.Interactive,
        )
        self.slots.cellDoubleClicked.connect(self.on_component_cell_double_clicked)
        for widget in [
            self.id_edit,
            self.name_edit,
            self.category_edit,
            self.tags_edit,
            self.output_edit,
            self.header_prefix_edit,
        ]:
            widget.textChanged.connect(self.mark_current_dirty)
        self.description_edit.textChanged.connect(self.mark_current_dirty)
        self.params.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.slots.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())

        form = QFormLayout()
        form.addRow("Template ID", self.id_edit)
        form.addRow("Name", self.name_edit)
        form.addRow("Category", self.category_edit)
        form.addRow("Tags", self.tags_edit)
        form.addRow("Output Folder", self.output_edit)
        form.addRow("Default Macro Prefix", self.header_prefix_edit)
        form.addRow("Description", self.description_edit)

        add_param = QPushButton("Add parameter")
        add_param.clicked.connect(self.add_template_parameter)
        del_param = QPushButton("Remove parameter")
        del_param.clicked.connect(lambda: self.params.removeRow(max(0, self.params.currentRow())))
        add_slot = QPushButton("Add sub module")
        add_slot.clicked.connect(self.add_template_component)
        del_slot = QPushButton("Remove slot")
        del_slot.clicked.connect(lambda: self.slots.removeRow(max(0, self.slots.currentRow())))
        new_template = QPushButton("New")
        new_template.clicked.connect(self.new_template)
        copy_template = QPushButton("Copy")
        copy_template.clicked.connect(self.copy_template)
        delete_template = QPushButton("Delete")
        delete_template.clicked.connect(self.delete_template)
        save = QPushButton("Save")
        save.clicked.connect(self.save_current)
        save_all = QPushButton("Save All")
        save_all.clicked.connect(self.save_all)

        tabs = QTabWidget()
        basic_tab = QWidget()
        basic_layout = QVBoxLayout(basic_tab)
        basic_layout.addLayout(form)
        basic_layout.addLayout(row_buttons([new_template, copy_template, delete_template, save, save_all]))
        basic_layout.addStretch(1)
        param_tab = QWidget()
        param_layout = QVBoxLayout(param_tab)
        param_layout.addWidget(self.params)
        param_layout.addLayout(row_buttons([add_param, del_param, save]))
        comp_tab = QWidget()
        comp_layout = QVBoxLayout(comp_tab)
        comp_layout.addWidget(self.slots)
        comp_layout.addLayout(row_buttons([add_slot, del_slot, save]))
        tabs.addTab(basic_tab, "Basic")
        tabs.addTab(param_tab, "Parameters")
        tabs.addTab(comp_tab, "Sub Components")
        self.form_layout.addWidget(tabs)

    def refresh_list(self) -> None:
        current = self.current_id
        self.list_widget.clear()
        category_nodes: dict[str, QTreeWidgetItem] = {}
        for schema in sorted(self.window.library.schemas.values(), key=lambda item: (item.category, item.display_name, item.id)):
            if not self.filter_match([schema.id, schema.display_name, schema.category, *schema.tags]):
                continue
            category = schema.category or "uncategorized"
            if category not in category_nodes:
                category_item = QTreeWidgetItem([category])
                category_item.setData(0, Qt.ItemDataRole.UserRole, "")
                self.list_widget.addTopLevelItem(category_item)
                category_nodes[category] = category_item
            suffix = " *" if schema.id in self.dirty_ids else ""
            item = QTreeWidgetItem([f"{schema.display_name} ({schema.id}){suffix}"])
            item.setToolTip(0, f"{schema.id}\nCategory: {schema.category}\nTags: {', '.join(schema.tags)}")
            item.setData(0, Qt.ItemDataRole.UserRole, schema.id)
            category_nodes[category].addChild(item)
            if schema.id == current:
                self.list_widget.setCurrentItem(item)
        self.list_widget.expandAll()
        self.select_first()

    def load_current(self) -> None:
        if not self.current_id:
            return
        path = self.window.schema_path(self.current_id)
        data = self.data_for_id(self.current_id, path)
        self.loading = True
        self.id_edit.setText(data.get("id", self.current_id))
        self.name_edit.setText(data.get("display_name", self.current_id))
        self.category_edit.setText(data.get("category", ""))
        self.tags_edit.setText(", ".join(data.get("tags", [])))
        self.output_edit.setText(data.get("output_subdir", data.get("id", self.current_id)))
        self.header_prefix_edit.setText(data.get("header_prefix", ""))
        self.description_edit.setPlainText(data.get("description", ""))
        self.params.setRowCount(0)
        for param in data.get("parameters", []):
            row = self.params.rowCount()
            self.params.insertRow(row)
            set_item(self.params, row, 0, param.get("name", ""))
            set_item(self.params, row, 1, param.get("c_name", param.get("name", "").upper()))
            set_item(self.params, row, 2, pretty_json(param["default"]) if "default" in param else "")
            set_item(self.params, row, 3, param.get("unit", ""))
            set_combo(self.params, row, 4, ["false", "true"], str(bool(param.get("required", False))).lower())
            set_combo(
                self.params,
                row,
                5,
                ["String", "Float", "Integer", "Unsigned Integer", "Raw C", "Custom"],
                format_label(param.get("value_format", "{}")),
            )
            set_item(self.params, row, 6, param.get("description", ""))
        self.slots.setRowCount(0)
        for name, slot in data.get("components", data.get("default_components", {})).items():
            row = self.slots.rowCount()
            self.slots.insertRow(row)
            set_item(self.slots, row, 0, name)
            if "inline" in slot:
                set_combo(self.slots, row, 1, ["entity", "inline"], "inline")
                set_item(self.slots, row, 2, pretty_json(slot.get("inline", {})))
            else:
                set_combo(self.slots, row, 1, ["entity", "inline"], "entity")
                set_item(self.slots, row, 2, slot.get("entity", ""))
            set_item(self.slots, row, 3, pretty_json(slot.get("overrides", {})))
        self.set_professional_text(pretty_json(data))
        self.loading = False

    def save_current(self) -> None:
        try:
            data = self.collect_current_data()
            if data is None:
                return
            path = self.path_for_id(self.current_id)
            self.window.write_json(path, data)
            self.dirty_ids.discard(self.current_id)
            self.drafts.pop(self.current_id, None)
            self.window.reload()
            self.refresh_list()
            self.message("Saved", f"Template saved: {path}")
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))

    def collect_current_data(self) -> dict[str, Any] | None:
        if not self.current_id:
            return None
        path = self.path_for_id(self.current_id)
        data = read_json(path) if path.exists() else {}
        data.update(
            {
                "id": self.id_edit.text().strip(),
                "display_name": self.name_edit.text().strip(),
                "description": self.description_edit.toPlainText().strip(),
                "category": self.category_edit.text().strip(),
                "tags": split_tags(self.tags_edit.text()),
                "output_subdir": self.output_edit.text().strip(),
                "header_prefix": self.header_prefix_edit.text().strip(),
            }
        )
        data["parameters"] = self._table_parameters()
        data["components"] = self._table_slots()
        data.pop("component_slots", None)
        data.pop("required_components", None)
        data.pop("default_components", None)
        return data

    def path_for_id(self, item_id: str) -> Path:
        return self.window.schema_path(item_id)

    def selected_category(self) -> str:
        item = self.list_widget.currentItem()
        if item is None:
            return "uncategorized"
        if item.data(0, Qt.ItemDataRole.UserRole):
            parent = item.parent()
            return parent.text(0).replace(" *", "") if parent else "uncategorized"
        return item.text(0).replace(" *", "")

    def new_template(self, category: str | None = None) -> None:
        schema_id = prompt_identifier(self, "New Template", "Template ID:")
        if not schema_id:
            return
        path = self.window.schema_path(schema_id)
        if path.exists():
            self.error(f"Template already exists: {schema_id}")
            return
        data = {
            "id": schema_id,
            "display_name": schema_id,
            "description": "",
            "category": category or self.selected_category(),
            "tags": [],
            "output_subdir": schema_id,
            "parameters": [],
            "components": {},
            "exports": {},
        }
        self.window.write_json(path, data)
        self.window.reload()
        self.current_id = schema_id
        self.refresh_list()
        self.load_current()

    def copy_template(self) -> None:
        if not self.current_id:
            return
        new_id = prompt_identifier(self, "Copy Template", "New Template ID:")
        if not new_id:
            return
        path = self.window.schema_path(new_id)
        if path.exists():
            self.error(f"Template already exists: {new_id}")
            return
        data = read_json(self.window.schema_path(self.current_id))
        data["id"] = new_id
        data["display_name"] = f"{data.get('display_name', self.current_id)} Copy"
        data["output_subdir"] = data.get("output_subdir") or new_id
        self.window.write_json(path, data)
        self.window.reload()
        self.current_id = new_id
        self.refresh_list()
        self.load_current()

    def delete_template(self) -> None:
        if not self.current_id:
            return
        path = self.window.schema_path(self.current_id)
        if confirm_delete(self, "Delete Template", f"Delete template '{self.current_id}'?\n\n{path}"):
            path.unlink(missing_ok=True)
            self.current_id = ""
            self.window.reload()
            self.refresh_list()

    def show_template_context_menu(self, pos) -> None:
        item = self.list_widget.itemAt(pos)
        if item is not None:
            self.list_widget.setCurrentItem(item)
        menu = QMenu(self)
        category = self.selected_category()
        menu.addAction(f"New Template in {category}", lambda: self.new_template(category))
        if self.current_id:
            menu.addAction("Copy Template", self.copy_template)
            menu.addAction("Delete Template", self.delete_template)
        menu.exec(self.list_widget.viewport().mapToGlobal(pos))

    def _table_parameters(self) -> list[dict[str, Any]]:
        rows = []
        for row in range(self.params.rowCount()):
            name = item_text(self.params, row, 0)
            if not name:
                continue
            item = {
                "name": name,
                "c_name": item_text(self.params, row, 1) or name.upper(),
                "unit": item_text(self.params, row, 3),
                "required": item_text(self.params, row, 4).lower() in {"1", "true", "yes"},
                "value_format": format_value(item_text(self.params, row, 5)),
                "description": item_text(self.params, row, 6),
            }
            default_text = item_text(self.params, row, 2)
            if default_text:
                item["default"] = parse_json_text(default_text, default_text)
            rows.append(item)
        return rows

    def edit_template_description(self, row: int) -> None:
        text = edit_multiline(self, "Parameter Description", item_text(self.params, row, 6))
        if text is not None:
            set_item(self.params, row, 6, text)

    def on_template_param_double_clicked(self, row: int, col: int) -> None:
        if col == 6:
            self.edit_template_description(row)

    def add_template_parameter(self) -> None:
        row = self.params.rowCount()
        self.params.insertRow(row)
        set_combo(self.params, row, 4, ["false", "true"], "false")
        set_combo(self.params, row, 5, ["String", "Float", "Integer", "Unsigned Integer", "Raw C", "Custom"], "Custom")

    def _table_slots(self) -> dict[str, dict[str, Any]]:
        components = {}
        for row in range(self.slots.rowCount()):
            slot = item_text(self.slots, row, 0)
            if not slot:
                continue
            mode = item_text(self.slots, row, 1) or "entity"
            value = item_text(self.slots, row, 2)
            comp: dict[str, Any]
            if mode == "inline":
                comp = {"inline": parse_json_text(value, {})}
            else:
                comp = {"entity": value}
            overrides = parse_json_text(item_text(self.slots, row, 3), {})
            if overrides:
                comp["overrides"] = overrides
            components[slot] = comp
        return components

    def add_template_component(self) -> None:
        row = self.slots.rowCount()
        self.slots.insertRow(row)
        set_combo(self.slots, row, 1, ["entity", "inline"], "entity")

    def on_component_cell_double_clicked(self, row: int, col: int) -> None:
        if col in {2, 3}:
            text = edit_multiline(self, "Sub Component", item_text(self.slots, row, col))
            if text is not None:
                set_item(self.slots, row, col, text)


class EntityPage(SDPEPage):
    """Hardware entity editor and per-entity header generation."""

    def create_items_widget(self):
        widget = QTreeWidget()
        widget.setHeaderHidden(True)
        widget.currentItemChanged.connect(lambda current, _previous: self.on_current_changed(current))
        widget.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        widget.customContextMenuRequested.connect(self.show_entity_context_menu)
        return widget

    def __init__(self, window: "MainWindow"):
        super().__init__(window, "Entity Instance", has_code=True)
        self.id_edit = QLineEdit()
        self.schema_combo = QComboBox()
        self.name_edit = QLineEdit()
        self.vendor_edit = QLineEdit()
        self.datasheet_edit = QLineEdit()
        self.document_edit = QLineEdit()
        self.macro_edit = QLineEdit()
        self.tags_edit = QLineEdit()
        self.description_edit = QTextEdit()
        self.tail_code_edit = QTextEdit()
        self.tail_code_edit.setFixedHeight(96)
        self.output_edit = QLineEdit()
        self.params = QTableWidget()
        set_table_headers(
            self.params,
            ["Parameter", "Value", "Unit", "Description"],
            QHeaderView.ResizeMode.Interactive,
        )
        self.params.cellDoubleClicked.connect(self.on_entity_param_double_clicked)
        self.components = QTableWidget()
        set_table_headers(
            self.components,
            ["Slot", "Mode", "Entity or Inline JSON", "Overrides JSON"],
            QHeaderView.ResizeMode.Interactive,
        )
        self.components.cellDoubleClicked.connect(self.on_entity_component_cell_double_clicked)
        for widget in [
            self.id_edit,
            self.name_edit,
            self.vendor_edit,
            self.datasheet_edit,
            self.document_edit,
            self.macro_edit,
            self.tags_edit,
            self.output_edit,
        ]:
            widget.textChanged.connect(self.mark_current_dirty)
        self.schema_combo.currentTextChanged.connect(lambda _text: self.mark_current_dirty())
        self.description_edit.textChanged.connect(self.mark_current_dirty)
        self.tail_code_edit.textChanged.connect(self.mark_current_dirty)
        self.params.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.components.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.out_dir = QLineEdit(str(window.default_output_dir))

        form = QFormLayout()
        form.addRow("Entity ID", self.id_edit)
        form.addRow("Template", self.schema_combo)
        form.addRow("Name", self.name_edit)
        form.addRow("Vendor", self.vendor_edit)
        form.addRow("Datasheet URL", self.datasheet_edit)
        form.addRow("Document URL", self.document_edit)
        form.addRow("Macro Prefix", self.macro_edit)
        form.addRow("Tags", self.tags_edit)
        form.addRow("Output Folder", self.output_edit)
        form.addRow("Description", self.description_edit)
        form.addRow("Tail Code", self.tail_code_edit)
        form.addRow("Header Output", self.out_dir)

        new_entity = QPushButton("New")
        new_entity.clicked.connect(self.new_entity)
        copy_entity = QPushButton("Copy")
        copy_entity.clicked.connect(self.copy_entity)
        delete_entity = QPushButton("Delete")
        delete_entity.clicked.connect(self.delete_entity)
        browse = QPushButton("Browse")
        browse.clicked.connect(self.browse_out)
        save = QPushButton("Save")
        save.clicked.connect(self.save_current)
        save_all = QPushButton("Save All")
        save_all.clicked.connect(self.save_all)
        add_comp = QPushButton("Add component")
        add_comp.clicked.connect(self.add_entity_component)
        del_comp = QPushButton("Remove component")
        del_comp.clicked.connect(lambda: self.components.removeRow(max(0, self.components.currentRow())))
        generate = QPushButton("Generate entity header")
        generate.clicked.connect(self.generate_header)
        preview_header = QPushButton("Preview header")
        preview_header.clicked.connect(self.preview_header)

        tabs = QTabWidget()
        basic_tab = QWidget()
        basic_layout = QVBoxLayout(basic_tab)
        basic_layout.addLayout(form)
        basic_layout.addLayout(row_buttons([new_entity, copy_entity, delete_entity, browse, save, save_all]))
        basic_layout.addStretch(1)
        param_tab = QWidget()
        param_layout = QVBoxLayout(param_tab)
        param_layout.addWidget(self.params)
        param_layout.addLayout(row_buttons([preview_header, generate, save]))
        comp_tab = QWidget()
        comp_layout = QVBoxLayout(comp_tab)
        comp_layout.addWidget(self.components)
        comp_layout.addLayout(row_buttons([add_comp, del_comp, save]))
        tabs.addTab(basic_tab, "Basic")
        tabs.addTab(param_tab, "Parameters")
        tabs.addTab(comp_tab, "Sub Components")
        self.form_layout.addWidget(tabs)

    def refresh_list(self) -> None:
        current = self.current_id
        self.schema_combo.clear()
        self.schema_combo.addItems(sorted(self.window.library.schemas))
        self.list_widget.clear()
        category_nodes: dict[str, QTreeWidgetItem] = {}
        schema_nodes: dict[str, QTreeWidgetItem] = {}
        for entity_id in sorted(self.window.library.entity_files):
            entity = self.window.library.entity(entity_id)
            schema = self.window.library.schema(entity.schema_id)
            if not self.filter_match(
                [
                    entity.id,
                    entity.display_name,
                    entity.schema_id,
                    schema.display_name,
                    schema.category,
                    *entity.tags,
                    *schema.tags,
                ]
            ):
                continue
            category = schema.category or "uncategorized"
            if category not in category_nodes:
                category_item = QTreeWidgetItem([category])
                category_item.setData(0, Qt.ItemDataRole.UserRole, "")
                self.list_widget.addTopLevelItem(category_item)
                category_nodes[category] = category_item
            schema_key = f"{category}/{schema.id}"
            if schema_key not in schema_nodes:
                schema_item = QTreeWidgetItem([f"{schema.display_name} ({schema.id})"])
                schema_item.setData(0, Qt.ItemDataRole.UserRole, "")
                schema_item.setData(0, Qt.ItemDataRole.UserRole + 1, schema.id)
                category_nodes[category].addChild(schema_item)
                schema_nodes[schema_key] = schema_item
            suffix = " *" if entity.id in self.dirty_ids else ""
            item = QTreeWidgetItem([f"{entity.display_name or entity.id} ({entity.id}){suffix}"])
            item.setToolTip(0, f"{entity.id}\nTemplate: {schema.display_name}\nCategory: {category}")
            item.setData(0, Qt.ItemDataRole.UserRole, entity.id)
            item.setData(0, Qt.ItemDataRole.UserRole + 1, schema.id)
            schema_nodes[schema_key].addChild(item)
            if entity.id == current:
                self.list_widget.setCurrentItem(item)
        self.list_widget.expandAll()
        self.select_first()

    def load_current(self) -> None:
        if not self.current_id:
            return
        path = self.path_for_id(self.current_id)
        data = self.data_for_id(self.current_id, path)
        entity = self.window.library.entity(self.current_id) if self.current_id in self.window.library.entity_files else HardwareEntity.from_json(data, path)
        self.loading = True
        self.id_edit.setText(data.get("id", entity.id))
        self.schema_combo.setCurrentText(data.get("schema", entity.schema_id))
        self.name_edit.setText(data.get("display_name", entity.display_name))
        self.vendor_edit.setText(data.get("vendor", entity.vendor))
        self.datasheet_edit.setText(data.get("datasheet_url", entity.datasheet_url))
        self.document_edit.setText(data.get("document_url", entity.document_url))
        self.macro_edit.setText(data.get("macro_prefix", entity.macro_prefix))
        self.tags_edit.setText(", ".join(data.get("tags", entity.tags)))
        self.output_edit.setText(data.get("output_subdir", entity.output_subdir))
        self.description_edit.setPlainText(data.get("description", entity.description))
        self.tail_code_edit.setPlainText(data.get("code_sections", {}).get("before_footer", ""))
        self.load_param_rows(entity, data)
        self.components.setRowCount(0)
        components_data = self.entity_components_with_defaults(entity, data)
        for slot, comp in components_data.items():
            row = self.components.rowCount()
            self.components.insertRow(row)
            set_item(self.components, row, 0, slot)
            if "entity" in comp:
                set_combo(self.components, row, 1, ["entity", "inline"], "entity")
                set_item(self.components, row, 2, comp["entity"])
            else:
                set_combo(self.components, row, 1, ["entity", "inline"], "inline")
                set_item(self.components, row, 2, pretty_json(comp.get("inline", {})))
            set_item(self.components, row, 3, pretty_json(comp.get("overrides", {})))
        self.set_professional_text(pretty_json(data))
        try:
            self.set_code_text(self.generator().render_entity_header(entity), reveal=False)
        except Exception as exc:
            self.code_panel.setPlainText(f"// Failed to render entity header preview:\n// {exc}")
        self.loading = False

    def load_param_rows(self, entity: HardwareEntity, data: dict[str, Any]) -> None:
        schema = self.window.library.schema(entity.schema_id)
        self.params.setRowCount(0)
        known = set()
        for name, pspec in schema.parameters.items():
            row = self.params.rowCount()
            self.params.insertRow(row)
            set_item(self.params, row, 0, name)
            set_item(self.params, row, 1, pretty_json(parameter_value_by_name(data.get("parameters", {}), name, pspec.c_name)))
            set_item(self.params, row, 2, pspec.unit)
            set_item(self.params, row, 3, pspec.description)
            self.params.item(row, 0).setToolTip(f"{pspec.description}\nUnit: {pspec.unit}")
            self.params.item(row, 1).setToolTip(f"{pspec.description}\nUnit: {pspec.unit}")
            known.add(name)
        for name, value in data.get("parameters", {}).items():
            if name in known:
                continue
            row = self.params.rowCount()
            self.params.insertRow(row)
            set_item(self.params, row, 0, name)
            set_item(self.params, row, 1, pretty_json(value))

    def save_current(self) -> None:
        try:
            data = self.collect_current_data()
            if data is None:
                return
            path = self.path_for_id(self.current_id)
            self.window.write_json(path, data)
            self.dirty_ids.discard(self.current_id)
            self.drafts.pop(self.current_id, None)
            self.window.reload()
            self.refresh_list()
            self.message("Saved", f"Entity saved: {path}")
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))

    def collect_current_data(self) -> dict[str, Any] | None:
        if not self.current_id:
            return None
        path = self.path_for_id(self.current_id)
        data = read_json(path) if path.exists() else {}
        data.update(
            {
                "id": self.id_edit.text().strip(),
                "schema": self.schema_combo.currentText().strip(),
                "display_name": self.name_edit.text().strip(),
                "description": self.description_edit.toPlainText().strip(),
                "vendor": self.vendor_edit.text().strip(),
                "datasheet_url": self.datasheet_edit.text().strip(),
                "document_url": self.document_edit.text().strip(),
                "macro_prefix": self.macro_edit.text().strip(),
                "tags": split_tags(self.tags_edit.text()),
                "output_subdir": self.output_edit.text().strip(),
                "parameters": self._table_parameters(),
                "components": self._table_components(),
            }
        )
        code_sections = dict(data.get("code_sections", {}))
        tail_code = self.tail_code_edit.toPlainText().strip()
        if tail_code:
            code_sections["before_footer"] = tail_code
        else:
            code_sections.pop("before_footer", None)
        if code_sections:
            data["code_sections"] = code_sections
        else:
            data.pop("code_sections", None)
        return data

    def path_for_id(self, item_id: str) -> Path:
        return self.window.entity_path(item_id)

    def selected_schema_id(self) -> str:
        item = self.list_widget.currentItem()
        if item is not None:
            schema_id = item.data(0, Qt.ItemDataRole.UserRole + 1)
            if schema_id:
                return schema_id
            category = item.text(0).replace(" *", "")
            for schema in self.window.library.schemas.values():
                if schema.category == category:
                    return schema.id
        return self.schema_combo.currentText().strip() or next(iter(self.window.library.schemas))

    def new_entity(self, schema_id: str | None = None) -> None:
        entity_id = prompt_identifier(self, "New Entity", "Entity ID:")
        if not entity_id:
            return
        path = self.window.entity_path(entity_id)
        if path.exists():
            self.error(f"Entity already exists: {entity_id}")
            return
        schema_id = schema_id or self.selected_schema_id()
        data = {
            "id": entity_id,
            "schema": schema_id,
            "display_name": entity_id,
            "macro_prefix": entity_id.upper(),
            "parameters": {},
            "components": {},
        }
        self.window.write_json(path, data)
        self.window.reload()
        self.current_id = entity_id
        self.refresh_list()
        self.load_current()

    def copy_entity(self) -> None:
        if not self.current_id:
            return
        new_id = prompt_identifier(self, "Copy Entity", "New Entity ID:")
        if not new_id:
            return
        path = self.window.entity_path(new_id)
        if path.exists():
            self.error(f"Entity already exists: {new_id}")
            return
        data = read_json(self.window.entity_path(self.current_id))
        data["id"] = new_id
        data["display_name"] = f"{data.get('display_name', self.current_id)} Copy"
        data["macro_prefix"] = new_id.upper()
        self.window.write_json(path, data)
        self.window.reload()
        self.current_id = new_id
        self.refresh_list()
        self.load_current()

    def delete_entity(self) -> None:
        if not self.current_id:
            return
        path = self.window.entity_path(self.current_id)
        if confirm_delete(self, "Delete Entity", f"Delete entity '{self.current_id}'?\n\n{path}"):
            path.unlink(missing_ok=True)
            self.current_id = ""
            self.window.reload()
            self.refresh_list()

    def show_entity_context_menu(self, pos) -> None:
        item = self.list_widget.itemAt(pos)
        if item is not None:
            self.list_widget.setCurrentItem(item)
        menu = QMenu(self)
        schema_id = self.selected_schema_id()
        menu.addAction(f"New Entity in {schema_id}", lambda: self.new_entity(schema_id))
        if self.current_id:
            menu.addAction("Copy Entity", self.copy_entity)
            menu.addAction("Delete Entity", self.delete_entity)
        menu.exec(self.list_widget.viewport().mapToGlobal(pos))

    def _table_parameters(self) -> dict[str, Any]:
        params = {}
        schema = self.window.library.schema(self.schema_combo.currentText().strip())
        for row in range(self.params.rowCount()):
            name = item_text(self.params, row, 0)
            value = item_text(self.params, row, 1)
            if not name or not value.strip():
                continue
            parsed = parse_json_text(value, value)
            if parsed == "":
                continue
            if name in schema.parameters:
                validate_value_for_format(name, parsed, schema.parameters[name].value_format)
            params[name] = parsed
        return params

    def select_parameter_symbol(self, row: int) -> None:
        symbols = self.window.symbols_for_subcomponents(self.current_id)
        selected = choose_item(self, "Select Sub Component Symbol", symbols)
        if selected:
            set_item(self.params, row, 1, pretty_json({"ref": selected}))

    def on_entity_param_double_clicked(self, row: int, col: int) -> None:
        if col == 1:
            self.select_parameter_symbol(row)
        elif col == 3:
            text = edit_multiline(self, "Parameter Description", item_text(self.params, row, 3))
            if text is not None:
                set_item(self.params, row, 3, text)

    def add_entity_component(self) -> None:
        row = self.components.rowCount()
        self.components.insertRow(row)
        set_combo(self.components, row, 1, ["entity", "inline"], "entity")

    def entity_components_with_defaults(self, entity: HardwareEntity, data: dict[str, Any]) -> dict[str, Any]:
        schema_data = read_json(self.window.schema_path(entity.schema_id))
        components = dict(schema_data.get("components", schema_data.get("default_components", {})))
        for slot, comp in data.get("components", {}).items():
            components[slot] = comp
        return components

    def on_entity_component_cell_double_clicked(self, row: int, col: int) -> None:
        if col in {2, 3}:
            text = edit_multiline(self, "Sub Component", item_text(self.components, row, col))
            if text is not None:
                set_item(self.components, row, col, text)

    def _table_components(self) -> dict[str, Any]:
        components = {}
        for row in range(self.components.rowCount()):
            slot = item_text(self.components, row, 0)
            if not slot:
                continue
            mode = item_text(self.components, row, 1) or "entity"
            value = item_text(self.components, row, 2)
            comp: dict[str, Any]
            if mode == "inline":
                comp = {"inline": parse_json_text(value, {})}
            else:
                comp = {"entity": value}
            overrides = parse_json_text(item_text(self.components, row, 3), {})
            if overrides:
                comp["overrides"] = overrides
            components[slot] = comp
        return components

    def browse_out(self) -> None:
        selected = QFileDialog.getExistingDirectory(self, "Select output directory", self.out_dir.text())
        if selected:
            self.out_dir.setText(selected)

    def generator(self) -> HeaderGenerator:
        return HeaderGenerator(self.window.library, Path(self.out_dir.text()))

    def preview_header(self) -> None:
        try:
            data = self.collect_current_data()
            if data is None:
                return
            entity = HardwareEntity.from_json(data, self.path_for_id(self.current_id))
            self.window.library._resolve_components(
                entity, self.window.library._merge_default_components(entity, data.get("components", {}))
            )
            self.set_code_text(self.generator().render_entity_header(entity))
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))

    def generate_header(self) -> None:
        try:
            files = self.generator().generate_entity_tree(self.current_id)
            self.message("Generated", "\n".join(str(item.path) for item in files))
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))


class ProjectPage(SDPEPage):
    """Project requirement editor."""

    def __init__(self, window: "MainWindow"):
        super().__init__(window, "Project Requirement", has_code=True)
        self.id_edit = QLineEdit()
        self.name_edit = QLineEdit()
        self.suite_edit = QLineEdit()
        self.header_edit = QLineEdit()
        self.hardware = QTableWidget()
        set_table_headers(self.hardware, ["Role", "Entity"], QHeaderView.ResizeMode.Interactive)
        self.requirements = QTableWidget()
        set_table_headers(
            self.requirements,
            ["Role", "Macro", "Binding Type", "Binding Value", "Description"],
            QHeaderView.ResizeMode.Interactive,
        )
        self.peripherals = QTextEdit()
        self.global_macros = QTextEdit()

        form = QFormLayout()
        form.addRow("Project ID", self.id_edit)
        form.addRow("Name", self.name_edit)
        form.addRow("Suite", self.suite_edit)
        form.addRow("Output Header", self.header_edit)
        save = QPushButton("Save project")
        save.clicked.connect(self.save_current)
        add_hw = QPushButton("Add hardware")
        add_hw.clicked.connect(lambda: self.hardware.insertRow(self.hardware.rowCount()))
        del_hw = QPushButton("Remove hardware")
        del_hw.clicked.connect(lambda: self.hardware.removeRow(max(0, self.hardware.currentRow())))
        add_req = QPushButton("Add requirement")
        add_req.clicked.connect(lambda: self.requirements.insertRow(self.requirements.rowCount()))
        del_req = QPushButton("Remove requirement")
        del_req.clicked.connect(lambda: self.requirements.removeRow(max(0, self.requirements.currentRow())))
        preview = QPushButton("Preview header")
        preview.clicked.connect(self.preview_project_header)

        self.form_layout.addLayout(form)
        self.form_layout.addWidget(QLabel("Hardware includes"))
        self.form_layout.addWidget(self.hardware)
        self.form_layout.addLayout(row_buttons([add_hw, del_hw]))
        self.form_layout.addWidget(QLabel("Requirements"))
        self.form_layout.addWidget(self.requirements)
        self.form_layout.addLayout(row_buttons([add_req, del_req, preview, save]))
        self.form_layout.addWidget(QLabel("Peripheral bindings JSON"))
        self.form_layout.addWidget(self.peripherals)
        self.form_layout.addWidget(QLabel("Global macros JSON"))
        self.form_layout.addWidget(self.global_macros)

    def refresh_list(self) -> None:
        current = self.current_id
        self.list_widget.clear()
        for path in self.window.project_paths():
            data = read_json(path)
            tags = [data.get("id", ""), data.get("display_name", ""), data.get("suite", "")]
            if not self.filter_match(tags):
                continue
            item = QListWidgetItem(f"{data.get('id', path.stem)}  ({data.get('suite', '')})")
            item.setData(Qt.ItemDataRole.UserRole, data.get("id", path.stem))
            self.list_widget.addItem(item)
            if data.get("id") == current:
                self.list_widget.setCurrentItem(item)
        self.select_first()

    def load_current(self) -> None:
        if not self.current_id:
            return
        data = read_json(self.window.project_path(self.current_id))
        self.id_edit.setText(data.get("id", ""))
        self.name_edit.setText(data.get("display_name", ""))
        self.suite_edit.setText(data.get("suite", ""))
        self.header_edit.setText(data.get("output_header", "sdpe_project_bindings.h"))
        self.hardware.setRowCount(0)
        for hw in data.get("hardware", []):
            row = self.hardware.rowCount()
            self.hardware.insertRow(row)
            set_item(self.hardware, row, 0, hw.get("role", ""))
            set_item(self.hardware, row, 1, hw.get("entity", ""))
        self.requirements.setRowCount(0)
        for req in data.get("requirements", []):
            row = self.requirements.rowCount()
            self.requirements.insertRow(row)
            btype, bvalue = binding_to_cells(req.get("binding", {}))
            for col, value in enumerate(
                [req.get("role", ""), req.get("macro", ""), btype, bvalue, req.get("description", "")]
            ):
                set_item(self.requirements, row, col, value)
        self.peripherals.setPlainText(pretty_json(data.get("peripheral_bindings", {})))
        self.global_macros.setPlainText(pretty_json(data.get("global_macros", {})))
        self.set_professional_text(pretty_json(data))
        try:
            self.set_code_text(
                HeaderGenerator(self.window.library, self.window.default_output_dir).render_project_header(data),
                reveal=False,
            )
        except Exception:
            self.code_panel.clear()

    def save_current(self) -> None:
        try:
            path = self.window.project_path(self.current_id)
            data = {
                "id": self.id_edit.text().strip(),
                "display_name": self.name_edit.text().strip(),
                "suite": self.suite_edit.text().strip(),
                "output_header": self.header_edit.text().strip(),
                "hardware": self._table_hardware(),
                "requirements": self._table_requirements(),
                "peripheral_bindings": parse_json_text(self.peripherals.toPlainText(), {}),
                "global_macros": parse_json_text(self.global_macros.toPlainText(), {}),
            }
            self.window.write_json(path, data)
            self.window.reload()
            self.refresh_list()
            self.message("Saved", f"Project saved: {path}")
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))

    def _table_hardware(self) -> list[dict[str, str]]:
        rows = []
        for row in range(self.hardware.rowCount()):
            role = item_text(self.hardware, row, 0)
            entity = item_text(self.hardware, row, 1)
            if role and entity:
                rows.append({"role": role, "entity": entity})
        return rows

    def _table_requirements(self) -> list[dict[str, Any]]:
        rows = []
        for row in range(self.requirements.rowCount()):
            macro = item_text(self.requirements, row, 1)
            if not macro:
                continue
            rows.append(
                {
                    "role": item_text(self.requirements, row, 0),
                    "macro": macro,
                    "binding": cells_to_binding(item_text(self.requirements, row, 2), item_text(self.requirements, row, 3)),
                    "description": item_text(self.requirements, row, 4),
                }
            )
        return rows

    def preview_project_header(self) -> None:
        try:
            data = read_json(self.window.project_path(self.current_id))
            self.set_code_text(HeaderGenerator(self.window.library, self.window.default_output_dir).render_project_header(data))
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))


class BindingPage(SDPEPage):
    """Focused requirement binding page with suggestions and generation."""

    def __init__(self, window: "MainWindow"):
        super().__init__(window, "Requirement Binding", has_code=True)
        self.out_dir = QLineEdit(str(window.default_output_dir))
        self.binding_table = QTableWidget()
        set_table_headers(
            self.binding_table,
            ["Macro", "Type", "Value", "Description"],
            QHeaderView.ResizeMode.Interactive,
        )
        self.suggestions = QListWidget()
        self.suggestions.itemDoubleClicked.connect(self.use_suggestion)
        browse = QPushButton("Browse")
        browse.clicked.connect(self.browse_out)
        save = QPushButton("Save bindings")
        save.clicked.connect(self.save_current)
        generate = QPushButton("Generate project header")
        generate.clicked.connect(self.generate_project)
        preview = QPushButton("Preview project header")
        preview.clicked.connect(self.preview_project)
        form = QFormLayout()
        form.addRow("Header Output", self.out_dir)
        self.form_layout.addLayout(form)
        self.form_layout.addLayout(row_buttons([browse, save, preview, generate]))
        self.form_layout.addWidget(QLabel("Requirement bindings"))
        self.form_layout.addWidget(self.binding_table)
        self.form_layout.addWidget(QLabel("Available scoped symbols"))
        self.form_layout.addWidget(self.suggestions)

    def refresh_list(self) -> None:
        current = self.current_id
        self.list_widget.clear()
        for path in self.window.project_paths():
            data = read_json(path)
            if not self.filter_match([data.get("id", ""), data.get("display_name", ""), data.get("suite", "")]):
                continue
            item = QListWidgetItem(f"{data.get('id', path.stem)}  ({data.get('suite', '')})")
            item.setData(Qt.ItemDataRole.UserRole, data.get("id", path.stem))
            self.list_widget.addItem(item)
            if data.get("id") == current:
                self.list_widget.setCurrentItem(item)
        self.select_first()

    def load_current(self) -> None:
        if not self.current_id:
            return
        data = read_json(self.window.project_path(self.current_id))
        self.binding_table.setRowCount(0)
        for req in data.get("requirements", []):
            row = self.binding_table.rowCount()
            self.binding_table.insertRow(row)
            btype, bvalue = binding_to_cells(req.get("binding", {}))
            for col, value in enumerate([req.get("macro", ""), btype, bvalue, req.get("description", "")]):
                set_item(self.binding_table, row, col, value)
        self.load_suggestions(data)
        self.set_professional_text(pretty_json(data))
        try:
            self.set_code_text(self.generator().render_project_header(data), reveal=False)
        except Exception as exc:
            self.code_panel.setPlainText(f"// Failed to render project header preview:\n// {exc}")

    def load_suggestions(self, data: dict[str, Any]) -> None:
        self.suggestions.clear()
        roots = [hw.get("entity") for hw in data.get("hardware", []) if hw.get("entity")]
        for entity_id in roots:
            try:
                for symbol in self.window.symbols_for_entity(entity_id):
                    self.suggestions.addItem(symbol)
            except SDPEError:
                continue

    def use_suggestion(self, item: QListWidgetItem) -> None:
        row = self.binding_table.currentRow()
        if row < 0:
            return
        set_item(self.binding_table, row, 1, "export")
        set_item(self.binding_table, row, 2, item.text())

    def save_current(self) -> None:
        try:
            path = self.window.project_path(self.current_id)
            data = read_json(path)
            requirements = data.get("requirements", [])
            for row in range(min(len(requirements), self.binding_table.rowCount())):
                requirements[row]["macro"] = item_text(self.binding_table, row, 0)
                requirements[row]["binding"] = cells_to_binding(
                    item_text(self.binding_table, row, 1), item_text(self.binding_table, row, 2)
                )
                requirements[row]["description"] = item_text(self.binding_table, row, 3)
            data["requirements"] = requirements
            self.window.write_json(path, data)
            self.window.reload()
            self.refresh_list()
            self.message("Saved", f"Bindings saved: {path}")
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))

    def browse_out(self) -> None:
        selected = QFileDialog.getExistingDirectory(self, "Select output directory", self.out_dir.text())
        if selected:
            self.out_dir.setText(selected)

    def generator(self) -> HeaderGenerator:
        return HeaderGenerator(self.window.library, Path(self.out_dir.text()))

    def preview_project(self) -> None:
        try:
            data = read_json(self.window.project_path(self.current_id))
            self.set_code_text(self.generator().render_project_header(data))
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))

    def generate_project(self) -> None:
        try:
            files = self.generator().generate_project(self.window.project_path(self.current_id))
            self.message("Generated", "\n".join(str(item.path) for item in files))
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))


class SettingsPage(QWidget):
    """Path settings for library packages and project-specific requirement files."""

    title = "Settings"

    def __init__(self, window: "MainWindow"):
        super().__init__()
        self.window = window
        self.schema_dirs = QTextEdit()
        self.entity_dirs = QTextEdit()
        self.project_dirs = QTextEdit()
        self.out_dir = QLineEdit(str(window.default_output_dir))

        self.schema_dirs.setPlainText("\n".join(str(path) for path in window.schema_dirs))
        self.entity_dirs.setPlainText("\n".join(str(path) for path in window.entity_dirs))
        self.project_dirs.setPlainText("\n".join(str(path) for path in window.project_dirs))

        form = QFormLayout()
        form.addRow("Template Paths", self.schema_dirs)
        form.addRow("Entity Paths", self.entity_dirs)
        form.addRow("Project Paths", self.project_dirs)
        form.addRow("Header Output", self.out_dir)

        apply_btn = QPushButton("Apply Settings")
        apply_btn.clicked.connect(self.apply_settings)
        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addLayout(row_buttons([apply_btn]))

    def apply_settings(self) -> None:
        self.window.schema_dirs = parse_path_lines(self.schema_dirs.toPlainText()) or self.window.schema_dirs
        self.window.entity_dirs = parse_path_lines(self.entity_dirs.toPlainText()) or self.window.entity_dirs
        self.window.project_dirs = parse_path_lines(self.project_dirs.toPlainText()) or self.window.project_dirs
        self.window.default_output_dir = Path(self.out_dir.text()).resolve()
        self.window.reload()
        self.window.refresh_pages()
        QMessageBox.information(self, "Settings", "Settings applied.")


class MainWindow(QMainWindow):
    """Main SDPE PyQt manager."""

    def __init__(
        self,
        library_root: Path,
        mode: str = "all",
        schema_dirs: list[Path] | None = None,
        entity_dirs: list[Path] | None = None,
        project_dirs: list[Path] | None = None,
        default_output_dir: Path | None = None,
    ):
        super().__init__()
        self.library_root = library_root
        self.mode = mode
        self.schema_dirs = schema_dirs or [library_root / "schemas"]
        self.entity_dirs = entity_dirs or [library_root / "entities"]
        self.project_dirs = project_dirs or [library_root / "projects"]
        self.default_output_dir = default_output_dir or (ROOT / "build_gui_pyqt")
        self.library = self.load_library()
        self.setWindowTitle(f"SDPE v2 Manager - {mode} - {library_root}")
        self.resize(1360, 820)
        self.tabs = QTabWidget()
        self.pages: list[SDPEPage] = []
        if mode in {"all", "library"}:
            self.pages.extend([TemplatePage(self), EntityPage(self)])
        if mode in {"all", "project"}:
            self.pages.extend([ProjectPage(self), BindingPage(self)])
        self.settings_page = SettingsPage(self)
        for page in self.pages:
            self.tabs.addTab(page, page.title)
        self.tabs.addTab(self.settings_page, self.settings_page.title)
        self.setCentralWidget(self.tabs)
        self.refresh_pages()

    def load_library(self) -> SDPELibrary:
        return SDPELibrary(self.library_root, self.schema_dirs, self.entity_dirs).load()

    def refresh_pages(self) -> None:
        for page in self.pages:
            page.refresh_list()

    def reload(self) -> None:
        self.library = self.load_library()

    def schema_path(self, schema_id: str) -> Path:
        for schema_dir in self.schema_dirs:
            for path in sorted(schema_dir.rglob("*.json")):
                if read_json(path).get("id") == schema_id:
                    return path
        return self.schema_dirs[0] / f"{schema_id}.json"

    def entity_path(self, entity_id: str) -> Path:
        return self.library.entity_files.get(entity_id, self.entity_dirs[0] / f"{entity_id}.json")

    def project_paths(self) -> list[Path]:
        paths: list[Path] = []
        for project_dir in self.project_dirs:
            if project_dir.is_file():
                paths.append(project_dir)
            elif project_dir.exists():
                paths.extend(project_dir.rglob("*.json"))
        return sorted(dict.fromkeys(paths))

    def project_path(self, project_id: str) -> Path:
        for path in self.project_paths():
            if read_json(path).get("id") == project_id:
                return path
        root = self.project_dirs[0]
        if root.is_file():
            root = root.parent
        return root / f"{project_id}.json"

    def write_json(self, path: Path, data: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(pretty_json(data) + "\n", encoding="utf-8")

    def symbols_for_entity(self, entity_id: str) -> list[str]:
        entity = self.library.entity(entity_id)
        symbols: list[str] = []
        self._collect_symbols(entity, entity.id, symbols)
        return symbols

    def symbols_for_subcomponents(self, entity_id: str) -> list[str]:
        entity = self.library.entity(entity_id)
        symbols: list[str] = []
        for slot, comp in entity.components.items():
            self._collect_symbols(comp.entity, slot, symbols)
        return symbols

    def _collect_symbols(self, entity: HardwareEntity, path: str, symbols: list[str]) -> None:
        schema = self.library.schema(entity.schema_id)
        for name in schema.exports:
            symbols.append(f"{path}.{name}")
        for name in schema.parameters:
            symbols.append(f"{path}.{name}")
        for slot, comp in entity.components.items():
            self._collect_symbols(comp.entity, f"{path}.{slot}", symbols)


def row_buttons(buttons: list[QPushButton]) -> QHBoxLayout:
    layout = QHBoxLayout()
    layout.addStretch(1)
    for button in buttons:
        layout.addWidget(button)
    return layout


def split_tags(text: str) -> list[str]:
    return [item.strip() for item in text.replace(";", ",").split(",") if item.strip()]


def parse_path_lines(text: str) -> list[Path]:
    paths = []
    for item in text.replace(";", "\n").splitlines():
        value = item.strip()
        if value:
            paths.append(Path(value).resolve())
    return paths


def binding_to_cells(binding: Any) -> tuple[str, str]:
    if isinstance(binding, dict):
        for key in ("export", "macro", "literal"):
            if key in binding:
                return key, str(binding[key])
    if isinstance(binding, str):
        return ("export" if "." in binding else "macro"), binding
    return "literal", ""


def cells_to_binding(kind: str, value: str) -> dict[str, str]:
    key = kind.strip() or "literal"
    if key not in {"export", "macro", "literal"}:
        key = "literal"
    return {key: value.strip()}


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Start SDPE v2 PyQt manager.")
    parser.add_argument("--library", type=Path, default=ROOT / "examples", help="SDPE library root.")
    parser.add_argument("--mode", choices=["all", "library", "project"], default="all", help="GUI work mode.")
    parser.add_argument("--schemas", nargs="*", type=Path, help="Template/schema directories.")
    parser.add_argument("--entities", nargs="*", type=Path, help="Entity instance directories.")
    parser.add_argument("--projects", nargs="*", type=Path, help="Project requirement files or directories.")
    parser.add_argument("--out", type=Path, default=ROOT / "build_gui_pyqt", help="Default header output directory.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    app = QApplication(sys.argv)
    library_root = args.library.resolve()
    window = MainWindow(
        library_root,
        mode=args.mode,
        schema_dirs=[path.resolve() for path in args.schemas] if args.schemas else None,
        entity_dirs=[path.resolve() for path in args.entities] if args.entities else None,
        project_dirs=[path.resolve() for path in args.projects] if args.projects else None,
        default_output_dir=args.out.resolve(),
    )
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
