"""PyQt manager for SDPE v2 hardware templates, entities, and bindings."""

from __future__ import annotations

import argparse
import json
import re
import sys
from datetime import date
from pathlib import Path
from typing import Any

try:
    from PyQt6.QtCore import QTimer, Qt
    from PyQt6.QtGui import QColor, QKeySequence, QShortcut
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
        QAbstractItemView,
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
        from PySide6.QtCore import QTimer, Qt
        from PySide6.QtGui import QColor, QKeySequence, QShortcut
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
            QAbstractItemView,
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
from gui_pyqt.dialogs import choose_item, choose_tree_item, confirm_delete, edit_multiline, prompt_identifier


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


def display_parameter_value(value: Any) -> str:
    if isinstance(value, dict) and set(value) == {"ref"}:
        return f"${{{value['ref']}}}"
    if isinstance(value, str):
        text = value.strip()
        match = re.fullmatch(r"\(\((.+)\)\)", text)
        if match:
            return match.group(1).strip()
    return pretty_json(value)


def parse_parameter_value(text: str) -> Any:
    value = text.strip()
    match = re.fullmatch(r"\$\{([A-Za-z0-9_][A-Za-z0-9_.]*)\}", value)
    if match:
        return {"ref": match.group(1)}
    return parse_json_text(value, value)


def set_table_headers(
    table: QTableWidget, headers: list[str], mode: QHeaderView.ResizeMode = QHeaderView.ResizeMode.Stretch
) -> None:
    table.setColumnCount(len(headers))
    table.setHorizontalHeaderLabels(headers)
    table.horizontalHeader().setSectionResizeMode(mode)
    table.horizontalHeader().setStretchLastSection(True)
    table.verticalHeader().setVisible(False)
    table.setAlternatingRowColors(True)
    table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
    table.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
    table.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
    table.setDragDropOverwriteMode(False)
    table.setEditTriggers(
        QAbstractItemView.EditTrigger.SelectedClicked
        | QAbstractItemView.EditTrigger.DoubleClicked
        | QAbstractItemView.EditTrigger.EditKeyPressed
    )
    install_table_clipboard_shortcuts(table)
    install_table_change_hooks(table)
    table.resizeColumnsToContents()


def page_for_widget(widget: QWidget) -> QWidget | None:
    current = widget
    while current is not None:
        if hasattr(current, "capture_undo_snapshot"):
            return current
        current = current.parentWidget()
    return None


def notify_table_changed(table: QTableWidget) -> None:
    page = page_for_widget(table)
    if page is not None:
        page.mark_current_dirty()


def install_table_change_hooks(table: QTableWidget) -> None:
    model = table.model()
    model.rowsInserted.connect(lambda *_args, t=table: notify_table_changed(t))
    model.rowsRemoved.connect(lambda *_args, t=table: notify_table_changed(t))
    model.rowsMoved.connect(lambda *_args, t=table: notify_table_changed(t))
    model.layoutChanged.connect(lambda *_args, t=table: notify_table_changed(t))


def fit_table_columns(table: QTableWidget, max_width: int = 360) -> None:
    table.resizeColumnsToContents()
    for col in range(table.columnCount()):
        header = table.horizontalHeaderItem(col)
        if header and "description" in header.text().lower():
            continue
        table.setColumnWidth(col, min(max(table.columnWidth(col) + 18, 96), max_width))


def fit_tree_key_columns(tree: QTreeWidget, description_col: int | None = None) -> None:
    header = tree.header()
    header.setStretchLastSection(description_col is not None)
    if description_col is None:
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        tree.setColumnWidth(0, max(tree.columnWidth(0), 260))
        return
    for col in range(tree.columnCount()):
        mode = QHeaderView.ResizeMode.Stretch if col == description_col else QHeaderView.ResizeMode.ResizeToContents
        header.setSectionResizeMode(col, mode)
    tree.setColumnWidth(0, max(tree.columnWidth(0), 360))


def tree_cell_text(tree: QTreeWidget, item: QTreeWidgetItem, col: int) -> str:
    widget = tree.itemWidget(item, col)
    if isinstance(widget, QComboBox):
        return widget.currentText().strip()
    return item.text(col).strip()


def set_tree_combo(tree: QTreeWidget, item: QTreeWidgetItem, col: int, values: list[str], current: Any = "") -> None:
    combo = QComboBox()
    combo.addItems(values)
    combo.setEditable(True)
    current_text = "" if current is None else str(current)
    item.setText(col, current_text)
    combo.setCurrentText(current_text)
    combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
    combo.currentTextChanged.connect(lambda text, i=item, c=col, t=tree: (i.setText(c, text), notify_tree_changed(t)))
    tree.setItemWidget(item, col, combo)


def notify_tree_changed(tree: QTreeWidget) -> None:
    page = page_for_widget(tree)
    if page is not None:
        page.mark_current_dirty()


def item_text(table: QTableWidget, row: int, col: int) -> str:
    widget = table.cellWidget(row, col)
    if isinstance(widget, QComboBox):
        return widget.currentText().strip()
    item = table.item(row, col)
    return "" if item is None else item.text().strip()


def selected_rows(table: QTableWidget) -> list[int]:
    rows = sorted({index.row() for index in table.selectedIndexes()})
    if not rows and table.currentRow() >= 0:
        rows = [table.currentRow()]
    return rows


def table_rows_to_text(table: QTableWidget, rows: list[int]) -> str:
    return "\n".join("\t".join(item_text(table, row, col) for col in range(table.columnCount())) for row in rows)


def paste_text_to_table(table: QTableWidget, text: str) -> None:
    lines = [line for line in text.splitlines() if line.strip()]
    if not lines:
        return
    row = table.currentRow()
    insert_at = table.rowCount() if row < 0 else row + 1
    for offset, line in enumerate(lines):
        table.insertRow(insert_at + offset)
        values = line.split("\t")
        for col, value in enumerate(values[: table.columnCount()]):
            set_item(table, insert_at + offset, col, value)


def install_table_clipboard_shortcuts(table: QTableWidget) -> None:
    def copy_rows() -> None:
        rows = selected_rows(table)
        if rows:
            QApplication.clipboard().setText(table_rows_to_text(table, rows))

    def cut_rows() -> None:
        notify_table_changed(table)
        rows = selected_rows(table)
        if not rows:
            return
        QApplication.clipboard().setText(table_rows_to_text(table, rows))
        for row in reversed(rows):
            table.removeRow(row)

    def paste_rows() -> None:
        notify_table_changed(table)
        paste_text_to_table(table, QApplication.clipboard().text())

    table._sdpe_copy_shortcut = QShortcut(QKeySequence.StandardKey.Copy, table)
    table._sdpe_copy_shortcut.activated.connect(copy_rows)
    table._sdpe_cut_shortcut = QShortcut(QKeySequence.StandardKey.Cut, table)
    table._sdpe_cut_shortcut.activated.connect(cut_rows)
    table._sdpe_paste_shortcut = QShortcut(QKeySequence.StandardKey.Paste, table)
    table._sdpe_paste_shortcut.activated.connect(paste_rows)


def set_item(table: QTableWidget, row: int, col: int, value: Any) -> None:
    table.setItem(row, col, QTableWidgetItem("" if value is None else str(value)))


def edit_table_cell_multiline(table: QTableWidget, row: int, col: int, parent: QWidget, title: str) -> None:
    text = edit_multiline(parent, title, item_text(table, row, col))
    if text is not None:
        set_item(table, row, col, text)


def setup_feature_macro_table(table: QTableWidget) -> None:
    set_table_headers(table, ["Macro", "Enabled", "Value", "Description"], QHeaderView.ResizeMode.Interactive)


def setup_option_macro_table(table: QTableWidget) -> None:
    set_table_headers(
        table,
        ["Macro", "Enabled", "Value", "Options Preset", "Options CSV", "Description"],
        QHeaderView.ResizeMode.Interactive,
    )


def setup_option_set_table(table: QTableWidget) -> None:
    set_table_headers(table, ["Name", "Options CSV", "Description"], QHeaderView.ResizeMode.Interactive)


def load_feature_macro_table(table: QTableWidget, items: list[dict[str, Any]]) -> None:
    table.setRowCount(0)
    for item in items:
        row = table.rowCount()
        table.insertRow(row)
        set_item(table, row, 0, item.get("macro", ""))
        set_combo(table, row, 1, ["Enable", "Disable"], "Enable" if item.get("enabled", True) else "Disable")
        set_item(table, row, 2, item.get("value", ""))
        set_item(table, row, 3, item.get("description", ""))
    fit_table_columns(table)


def load_option_macro_table(table: QTableWidget, items: list[dict[str, Any]]) -> None:
    table.setRowCount(0)
    for item in items:
        row = table.rowCount()
        table.insertRow(row)
        options = [str(v) for v in item.get("options", [])]
        preset = item.get("options_preset", item.get("preset", ""))
        if not preset and len(options) == 1:
            match = re.fullmatch(r"\$\{([^}]+)\}", options[0].strip())
            if match:
                preset = match.group(1)
                options = []
        set_item(table, row, 0, item.get("macro", ""))
        set_combo(table, row, 1, ["Enable", "Disable"], "Enable" if item.get("enabled", True) else "Disable")
        set_combo(table, row, 2, options or [str(item.get("value", ""))], item.get("value", ""))
        set_item(table, row, 3, preset)
        set_item(table, row, 4, ", ".join(options))
        set_item(table, row, 5, item.get("description", ""))
    fit_table_columns(table)


def load_option_set_table(table: QTableWidget, option_sets: dict[str, Any]) -> None:
    table.setRowCount(0)
    for name, value in option_sets.items():
        row = table.rowCount()
        table.insertRow(row)
        options = value.get("options", value) if isinstance(value, dict) else value
        description = value.get("description", "") if isinstance(value, dict) else ""
        if not isinstance(options, list):
            options = [options]
        set_item(table, row, 0, name)
        set_item(table, row, 1, ", ".join(str(item) for item in options))
        set_item(table, row, 2, description)
    fit_table_columns(table)


def collect_feature_macro_table(table: QTableWidget) -> list[dict[str, Any]]:
    rows = []
    for row in range(table.rowCount()):
        macro = item_text(table, row, 0)
        if macro:
            rows.append(
                {
                    "macro": macro,
                    "enabled": item_text(table, row, 1).lower() != "disable",
                    "value": item_text(table, row, 2),
                    "description": item_text(table, row, 3),
                }
            )
    return rows


def collect_option_macro_table(table: QTableWidget) -> list[dict[str, Any]]:
    rows = []
    for row in range(table.rowCount()):
        macro = item_text(table, row, 0)
        if macro:
            rows.append(
                {
                    "macro": macro,
                    "enabled": item_text(table, row, 1).lower() != "disable",
                    "value": item_text(table, row, 2),
                    "options_preset": item_text(table, row, 3),
                    "options": [item.strip() for item in item_text(table, row, 4).split(",") if item.strip()],
                    "description": item_text(table, row, 5),
                }
            )
    return rows


def collect_option_set_table(table: QTableWidget) -> dict[str, Any]:
    rows: dict[str, Any] = {}
    for row in range(table.rowCount()):
        name = item_text(table, row, 0)
        if not name:
            continue
        options = [item.strip() for item in item_text(table, row, 1).split(",") if item.strip()]
        description = item_text(table, row, 2)
        rows[name] = {"options": options, "description": description} if description else options
    return rows


def set_combo(table: QTableWidget, row: int, col: int, values: list[str], current: Any = "") -> None:
    combo = QComboBox()
    combo.addItems(values)
    combo.setEditable(True)
    combo.setCurrentText("" if current is None else str(current))
    combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
    width = max([combo.fontMetrics().horizontalAdvance(value) for value in values + [combo.currentText()]] or [0]) + 48
    combo.setMinimumWidth(width)
    combo.currentTextChanged.connect(lambda _text, t=table: notify_table_changed(t))
    table.setCellWidget(row, col, combo)
    table.setColumnWidth(col, max(table.columnWidth(col), width + 12))


class SDPEPage(QWidget):
    """Common split view page: searchable object list, form editor, JSON/code preview."""

    def __init__(self, window: "MainWindow", title: str, has_code: bool = True):
        super().__init__()
        self.window = window
        self.title = title
        self.current_id = ""
        self.has_code = has_code
        self.loading = False
        self.restoring_undo = False
        self.drafts: dict[str, dict[str, Any]] = {}
        self.dirty_ids: set[str] = set()
        self.undo_limit = 20
        self.undo_stack: list[dict[str, Any]] = []
        self.undo_timer = QTimer(self)
        self.undo_timer.setSingleShot(True)
        self.undo_timer.timeout.connect(self.capture_undo_snapshot)

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
        self.undo_shortcut.activated.connect(self.undo_current_change)

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
        if self.loading or self.restoring_undo or not self.current_id:
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
        self.schedule_undo_snapshot()

    def schedule_undo_snapshot(self) -> None:
        if self.loading or self.restoring_undo or not self.current_id:
            return
        self.undo_timer.start(350)

    def clone_data(self, data: dict[str, Any]) -> dict[str, Any]:
        return json.loads(pretty_json(data))

    def reset_undo_history(self, data: dict[str, Any] | None = None) -> None:
        if data is None:
            data = self.collect_current_data()
        self.undo_stack = [self.clone_data(data)] if data is not None else []

    def capture_undo_snapshot(self) -> None:
        if self.loading or self.restoring_undo or not self.current_id:
            return
        data = self.collect_current_data()
        if data is None:
            return
        snapshot = self.clone_data(data)
        if self.undo_stack and pretty_json(self.undo_stack[-1]) == pretty_json(snapshot):
            return
        self.undo_stack.append(snapshot)
        if len(self.undo_stack) > self.undo_limit:
            self.undo_stack = self.undo_stack[-self.undo_limit :]
        self.drafts[self.current_id] = snapshot

    def undo_current_change(self) -> None:
        if self.loading or self.restoring_undo or not self.current_id:
            return
        self.undo_timer.stop()
        self.capture_undo_snapshot()
        if len(self.undo_stack) <= 1:
            self.message("Undo", "No earlier change.")
            return
        self.undo_stack.pop()
        previous = self.clone_data(self.undo_stack[-1])
        self.restoring_undo = True
        try:
            self.drafts[self.current_id] = previous
            self.load_current()
            path = self.path_for_id(self.current_id)
            old = read_json(path) if path.exists() else {}
            if pretty_json(previous) != pretty_json(old):
                self.dirty_ids.add(self.current_id)
            else:
                self.dirty_ids.discard(self.current_id)
            self.message("Undo", "Restored previous change.")
        finally:
            self.restoring_undo = False

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

    def dirty_items(self) -> list[tuple[str, Path]]:
        if self.current_id in self.dirty_ids:
            self.store_current_draft()
        return [(item_id, self.path_for_id(item_id)) for item_id in sorted(self.dirty_ids)]

    def save_dirty(self) -> int:
        if self.current_id in self.dirty_ids:
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
        return saved

    def data_for_id(self, item_id: str, path: Path) -> dict[str, Any]:
        if item_id in self.drafts:
            return self.drafts[item_id]
        return read_json(path)

    def save_all(self) -> None:
        saved = self.save_dirty()
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
        self.window.statusBar().showMessage(f"{title}: {text}", 2600)

    def error(self, text: str) -> None:
        QMessageBox.critical(self, self.title, text)


class TemplatePage(SDPEPage):
    """Template/schema object editor."""

    def create_items_widget(self):
        widget = QTreeWidget()
        widget.setHeaderHidden(True)
        fit_tree_key_columns(widget)
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
        self.params = QTreeWidget()
        self.params.setHeaderLabels(["Name", "Macro Name", "Default", "Unit", "Required", "Format", "Description"])
        self.params.setAlternatingRowColors(True)
        self.params.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
        self.params.setDefaultDropAction(Qt.DropAction.MoveAction)
        self.params.itemChanged.connect(lambda _item, _col: self.mark_current_dirty())
        self.params.model().rowsMoved.connect(lambda *_args: self.after_template_parameter_tree_changed())
        self.params.model().rowsInserted.connect(lambda *_args: self.after_template_parameter_tree_changed())
        self.params.model().rowsRemoved.connect(lambda *_args: self.after_template_parameter_tree_changed())
        self.params.itemDoubleClicked.connect(self.on_template_param_double_clicked)
        self.slots = QTableWidget()
        set_table_headers(
            self.slots,
            ["Slot", "Mode", "Entity or Inline JSON", "Overrides JSON"],
            QHeaderView.ResizeMode.Interactive,
        )
        self.slots.cellDoubleClicked.connect(self.on_component_cell_double_clicked)
        self.feature_macros = QTableWidget()
        setup_feature_macro_table(self.feature_macros)
        self.feature_macros.cellDoubleClicked.connect(self.on_template_macro_double_clicked)
        self.option_macros = QTableWidget()
        setup_option_macro_table(self.option_macros)
        self.option_macros.cellDoubleClicked.connect(self.on_template_macro_double_clicked)
        self.option_sets = QTableWidget()
        setup_option_set_table(self.option_sets)
        self.option_sets.cellDoubleClicked.connect(self.on_option_set_double_clicked)
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
        self.slots.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.feature_macros.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.option_macros.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.option_sets.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())

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
        add_group = QPushButton("Add group")
        add_group.clicked.connect(self.add_parameter_group)
        del_param = QPushButton("Remove parameter")
        del_param.clicked.connect(self.remove_template_parameter_item)
        add_slot = QPushButton("Add sub module")
        add_slot.clicked.connect(self.add_template_component)
        del_slot = QPushButton("Remove slot")
        del_slot.clicked.connect(lambda: self.slots.removeRow(max(0, self.slots.currentRow())))
        add_feature = QPushButton("Add selection macro")
        add_feature.clicked.connect(lambda: self.add_macro_row(self.feature_macros, "feature"))
        del_feature = QPushButton("Remove selection macro")
        del_feature.clicked.connect(lambda: self.feature_macros.removeRow(max(0, self.feature_macros.currentRow())))
        add_option = QPushButton("Add option macro")
        add_option.clicked.connect(lambda: self.add_macro_row(self.option_macros, "option"))
        del_option = QPushButton("Remove option macro")
        del_option.clicked.connect(lambda: self.option_macros.removeRow(max(0, self.option_macros.currentRow())))
        add_option_set = QPushButton("Add option set")
        add_option_set.clicked.connect(self.add_option_set)
        del_option_set = QPushButton("Remove option set")
        del_option_set.clicked.connect(lambda: self.option_sets.removeRow(max(0, self.option_sets.currentRow())))
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
        param_layout.addLayout(row_buttons([add_group, add_param, del_param, save]))
        comp_tab = QWidget()
        comp_layout = QVBoxLayout(comp_tab)
        comp_layout.addWidget(self.slots)
        comp_layout.addLayout(row_buttons([add_slot, del_slot, save]))
        macro_tab = QWidget()
        macro_layout = QVBoxLayout(macro_tab)
        macro_layout.addWidget(QLabel("Selection macros"))
        macro_layout.addWidget(self.feature_macros)
        macro_layout.addLayout(row_buttons([add_feature, del_feature]))
        macro_layout.addWidget(QLabel("Option macros"))
        macro_layout.addWidget(self.option_macros)
        macro_layout.addLayout(row_buttons([add_option, del_option]))
        macro_layout.addWidget(QLabel("Option presets"))
        macro_layout.addWidget(self.option_sets)
        macro_layout.addLayout(row_buttons([add_option_set, del_option_set, save]))
        tabs.addTab(basic_tab, "Basic")
        tabs.addTab(param_tab, "Parameters")
        tabs.addTab(comp_tab, "Sub Components")
        tabs.addTab(macro_tab, "Macros")
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
        fit_tree_key_columns(self.list_widget)
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
        self.load_template_parameter_tree(data)
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
        fit_tree_key_columns(self.params, description_col=6)
        fit_table_columns(self.slots)
        load_feature_macro_table(self.feature_macros, data.get("feature_macros", []))
        load_option_macro_table(self.option_macros, data.get("option_macros", []))
        load_option_set_table(self.option_sets, data.get("option_sets", {}))
        self.set_professional_text(pretty_json(data))
        self.loading = False
        if not self.restoring_undo:
            self.reset_undo_history(self.collect_current_data() or data)

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
        data["parameter_groups"] = self._parameter_groups()
        data["components"] = self._table_slots()
        data["feature_macros"] = collect_feature_macro_table(self.feature_macros)
        data["option_macros"] = collect_option_macro_table(self.option_macros)
        option_sets = collect_option_set_table(self.option_sets)
        if option_sets:
            data["option_sets"] = option_sets
        else:
            data.pop("option_sets", None)
        if not data["feature_macros"]:
            data.pop("feature_macros", None)
        if not data["option_macros"]:
            data.pop("option_macros", None)
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
        for group_index in range(self.params.topLevelItemCount()):
            group = self.params.topLevelItem(group_index)
            for child_index in range(group.childCount()):
                child = group.child(child_index)
                name = tree_cell_text(self.params, child, 0)
                if not name:
                    continue
                item = {
                    "name": name,
                    "c_name": tree_cell_text(self.params, child, 1) or name.upper(),
                    "unit": tree_cell_text(self.params, child, 3),
                    "required": tree_cell_text(self.params, child, 4).lower() in {"1", "true", "yes"},
                    "value_format": format_value(tree_cell_text(self.params, child, 5)),
                    "description": tree_cell_text(self.params, child, 6),
                }
                default_text = tree_cell_text(self.params, child, 2)
                if default_text:
                    item["default"] = parse_json_text(default_text, default_text)
                rows.append(item)
        return rows

    def _parameter_groups(self) -> list[dict[str, Any]]:
        groups = []
        for group_index in range(self.params.topLevelItemCount()):
            group = self.params.topLevelItem(group_index)
            names = [
                tree_cell_text(self.params, group.child(child_index), 0)
                for child_index in range(group.childCount())
                if tree_cell_text(self.params, group.child(child_index), 0)
            ]
            groups.append({"name": group.text(0).strip() or "Parameters", "parameters": names})
        return groups

    def load_template_parameter_tree(self, data: dict[str, Any]) -> None:
        self.params.clear()
        parameters = list(data.get("parameters", []))
        by_name = {param.get("name", ""): param for param in parameters if param.get("name")}
        used: set[str] = set()
        groups = data.get("parameter_groups", [])
        if not groups:
            grouped: dict[str, list[str]] = {}
            for param in parameters:
                grouped.setdefault(param.get("group", "Parameters") or "Parameters", []).append(param.get("name", ""))
            groups = [{"name": name, "parameters": names} for name, names in grouped.items()] or [{"name": "Parameters", "parameters": []}]
        for group in groups:
            group_item = self.create_parameter_group_item(group.get("name", "Parameters"))
            self.params.addTopLevelItem(group_item)
            for name in group.get("parameters", []):
                if name in by_name:
                    self.add_parameter_item(group_item, by_name[name])
                    used.add(name)
        missing = [param for param in parameters if param.get("name") not in used]
        if missing:
            group_item = self.create_parameter_group_item("Ungrouped")
            self.params.addTopLevelItem(group_item)
            for param in missing:
                self.add_parameter_item(group_item, param)
        self.params.expandAll()
        fit_tree_key_columns(self.params, description_col=6)

    def create_parameter_group_item(self, name: str) -> QTreeWidgetItem:
        item = QTreeWidgetItem([name])
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable | Qt.ItemFlag.ItemIsDropEnabled)
        item.setData(0, Qt.ItemDataRole.UserRole, "group")
        for col in range(1, self.params.columnCount()):
            item.setText(col, "")
        return item

    def add_parameter_item(self, group: QTreeWidgetItem, param: dict[str, Any] | None = None) -> QTreeWidgetItem:
        param = param or {}
        name = param.get("name", "")
        item = QTreeWidgetItem(
            [
                name,
                param.get("c_name", name.upper()),
                pretty_json(param["default"]) if "default" in param else "",
                param.get("unit", ""),
                "",
                "",
                param.get("description", ""),
            ]
        )
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable | Qt.ItemFlag.ItemIsDragEnabled)
        item.setData(0, Qt.ItemDataRole.UserRole, "parameter")
        group.addChild(item)
        set_tree_combo(self.params, item, 4, ["false", "true"], str(bool(param.get("required", False))).lower())
        set_tree_combo(
            self.params,
            item,
            5,
            ["String", "Float", "Integer", "Unsigned Integer", "Raw C", "Custom"],
            format_label(param.get("value_format", "{}")),
        )
        return item

    def iter_parameter_items(self) -> list[QTreeWidgetItem]:
        items: list[QTreeWidgetItem] = []
        for group_index in range(self.params.topLevelItemCount()):
            group = self.params.topLevelItem(group_index)
            if group.data(0, Qt.ItemDataRole.UserRole) == "parameter":
                items.append(group)
                continue
            for child_index in range(group.childCount()):
                child = group.child(child_index)
                if child.data(0, Qt.ItemDataRole.UserRole) == "parameter":
                    items.append(child)
        return items

    def after_template_parameter_tree_changed(self) -> None:
        QTimer.singleShot(0, self.restore_template_parameter_widgets)
        self.mark_current_dirty()

    def restore_template_parameter_widgets(self) -> None:
        if self.loading:
            return
        self.loading = True
        try:
            self.normalize_template_parameter_groups()
            for item in self.iter_parameter_items():
                required = tree_cell_text(self.params, item, 4) or "false"
                value_format = tree_cell_text(self.params, item, 5) or "Custom"
                if not isinstance(self.params.itemWidget(item, 4), QComboBox):
                    set_tree_combo(self.params, item, 4, ["false", "true"], required)
                if not isinstance(self.params.itemWidget(item, 5), QComboBox):
                    set_tree_combo(
                        self.params,
                        item,
                        5,
                        ["String", "Float", "Integer", "Unsigned Integer", "Raw C", "Custom"],
                        value_format,
                    )
            fit_tree_key_columns(self.params, description_col=6)
        finally:
            self.loading = False

    def normalize_template_parameter_groups(self) -> None:
        if self.params.topLevelItemCount() == 0:
            self.params.addTopLevelItem(self.create_parameter_group_item("Parameters"))
            return
        ungrouped: QTreeWidgetItem | None = None
        top_level_parameters: list[QTreeWidgetItem] = []
        for index in range(self.params.topLevelItemCount()):
            item = self.params.topLevelItem(index)
            if item.data(0, Qt.ItemDataRole.UserRole) == "parameter":
                top_level_parameters.append(item)
        if top_level_parameters:
            for index in range(self.params.topLevelItemCount()):
                item = self.params.topLevelItem(index)
                if item.data(0, Qt.ItemDataRole.UserRole) == "group" and item.text(0) == "Ungrouped":
                    ungrouped = item
                    break
            if ungrouped is None:
                ungrouped = self.create_parameter_group_item("Ungrouped")
                self.params.addTopLevelItem(ungrouped)
            for item in top_level_parameters:
                index = self.params.indexOfTopLevelItem(item)
                moved = self.params.takeTopLevelItem(index)
                ungrouped.addChild(moved)
            ungrouped.setExpanded(True)

    def current_parameter_group(self) -> QTreeWidgetItem:
        item = self.params.currentItem()
        if item is not None:
            if item.data(0, Qt.ItemDataRole.UserRole) == "group":
                return item
            if item.parent() is not None:
                return item.parent()
        if self.params.topLevelItemCount() == 0:
            self.add_parameter_group()
        return self.params.topLevelItem(0)

    def edit_template_description(self, item: QTreeWidgetItem) -> None:
        text = edit_multiline(self, "Parameter Description", tree_cell_text(self.params, item, 6))
        if text is not None:
            item.setText(6, text)

    def on_template_param_double_clicked(self, item: QTreeWidgetItem, col: int) -> None:
        if item.data(0, Qt.ItemDataRole.UserRole) == "parameter" and col == 6:
            self.edit_template_description(item)

    def add_template_parameter(self) -> None:
        group = self.current_parameter_group()
        item = self.add_parameter_item(group, {"name": "new_parameter", "c_name": "NEW_PARAMETER", "value_format": "{}"})
        group.setExpanded(True)
        self.params.setCurrentItem(item)
        self.mark_current_dirty()

    def add_parameter_group(self) -> None:
        item = self.create_parameter_group_item("New Group")
        self.params.addTopLevelItem(item)
        self.params.setCurrentItem(item)
        self.params.editItem(item, 0)
        self.mark_current_dirty()

    def remove_template_parameter_item(self) -> None:
        item = self.params.currentItem()
        if item is None:
            return
        parent = item.parent()
        if parent is None:
            index = self.params.indexOfTopLevelItem(item)
            self.params.takeTopLevelItem(index)
        else:
            parent.removeChild(item)
        self.mark_current_dirty()

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

    def add_macro_row(self, table: QTableWidget, macro_type: str) -> None:
        row = table.rowCount()
        table.insertRow(row)
        set_combo(table, row, 1, ["Enable", "Disable"], "Enable")
        if macro_type == "option":
            set_combo(table, row, 2, ["1", "2", "3", "4", "5"], "1")
            set_item(table, row, 4, "1, 2, 3, 4, 5")

    def add_option_set(self) -> None:
        row = self.option_sets.rowCount()
        self.option_sets.insertRow(row)

    def on_template_macro_double_clicked(self, row: int, col: int) -> None:
        table = self.sender()
        if isinstance(table, QTableWidget) and (col == table.columnCount() - 1 or col == 4):
            edit_table_cell_multiline(table, row, col, self, "Macro Field")

    def on_option_set_double_clicked(self, row: int, col: int) -> None:
        if col in {1, 2}:
            edit_table_cell_multiline(self.option_sets, row, col, self, "Option Preset")

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
        fit_tree_key_columns(widget)
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
        self.prefix_code_edit = QTextEdit()
        self.tail_code_edit = QTextEdit()
        self.output_edit = QLineEdit()
        self.params = QTreeWidget()
        self.params.setHeaderLabels(["Parameter", "Value", "Unit", "Description"])
        self.params.setAlternatingRowColors(True)
        self.params.itemChanged.connect(lambda _item, _col: self.mark_current_dirty())
        self.params.itemDoubleClicked.connect(self.on_entity_param_double_clicked)
        self.components = QTableWidget()
        set_table_headers(
            self.components,
            ["Slot", "Mode", "Entity or Inline JSON", "Overrides JSON"],
            QHeaderView.ResizeMode.Interactive,
        )
        self.components.cellDoubleClicked.connect(self.on_entity_component_cell_double_clicked)
        self.feature_macros = QTableWidget()
        setup_feature_macro_table(self.feature_macros)
        self.feature_macros.cellDoubleClicked.connect(self.on_entity_macro_double_clicked)
        self.option_macros = QTableWidget()
        setup_option_macro_table(self.option_macros)
        self.option_macros.cellDoubleClicked.connect(self.on_entity_macro_double_clicked)
        self.option_sets = QTableWidget()
        setup_option_set_table(self.option_sets)
        self.option_sets.cellDoubleClicked.connect(self.on_entity_option_set_double_clicked)
        self.conditional_macros = QTableWidget()
        set_table_headers(self.conditional_macros, ["Condition", "Macros JSON", "Description"], QHeaderView.ResizeMode.Interactive)
        self.conditional_macros.cellDoubleClicked.connect(self.on_entity_conditional_double_clicked)
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
        self.prefix_code_edit.textChanged.connect(self.mark_current_dirty)
        self.tail_code_edit.textChanged.connect(self.mark_current_dirty)
        self.components.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.feature_macros.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.option_macros.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.option_sets.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.conditional_macros.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
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
        add_feature = QPushButton("Add selection macro")
        add_feature.clicked.connect(lambda: self.add_macro_row(self.feature_macros, "feature"))
        del_feature = QPushButton("Remove selection macro")
        del_feature.clicked.connect(lambda: self.feature_macros.removeRow(max(0, self.feature_macros.currentRow())))
        add_option = QPushButton("Add option macro")
        add_option.clicked.connect(lambda: self.add_macro_row(self.option_macros, "option"))
        del_option = QPushButton("Remove option macro")
        del_option.clicked.connect(lambda: self.option_macros.removeRow(max(0, self.option_macros.currentRow())))
        add_option_set = QPushButton("Add option preset")
        add_option_set.clicked.connect(self.add_option_set)
        del_option_set = QPushButton("Remove option preset")
        del_option_set.clicked.connect(lambda: self.option_sets.removeRow(max(0, self.option_sets.currentRow())))
        add_conditional = QPushButton("Add conditional macro")
        add_conditional.clicked.connect(self.add_conditional_macro)
        del_conditional = QPushButton("Remove conditional macro")
        del_conditional.clicked.connect(lambda: self.conditional_macros.removeRow(max(0, self.conditional_macros.currentRow())))
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
        macro_tab = QWidget()
        macro_layout = QVBoxLayout(macro_tab)
        macro_layout.addWidget(QLabel("Selection macros"))
        macro_layout.addWidget(self.feature_macros)
        macro_layout.addLayout(row_buttons([add_feature, del_feature]))
        macro_layout.addWidget(QLabel("Option macros"))
        macro_layout.addWidget(self.option_macros)
        macro_layout.addLayout(row_buttons([add_option, del_option]))
        macro_layout.addWidget(QLabel("Option presets"))
        macro_layout.addWidget(self.option_sets)
        macro_layout.addLayout(row_buttons([add_option_set, del_option_set]))
        macro_layout.addWidget(QLabel("Conditional macros"))
        macro_layout.addWidget(self.conditional_macros)
        macro_layout.addLayout(row_buttons([add_conditional, del_conditional, save]))
        code_tab = QWidget()
        code_layout = QVBoxLayout(code_tab)
        code_splitter = QSplitter(Qt.Orientation.Vertical)
        head_panel = QWidget()
        head_layout = QVBoxLayout(head_panel)
        head_layout.addWidget(QLabel("Head code"))
        head_layout.addWidget(self.prefix_code_edit)
        tail_panel = QWidget()
        tail_layout = QVBoxLayout(tail_panel)
        tail_layout.addWidget(QLabel("Tail code"))
        tail_layout.addWidget(self.tail_code_edit)
        code_splitter.addWidget(head_panel)
        code_splitter.addWidget(tail_panel)
        code_splitter.setSizes([360, 360])
        code_layout.addWidget(code_splitter)
        tabs.addTab(basic_tab, "Basic")
        tabs.addTab(param_tab, "Parameters")
        tabs.addTab(comp_tab, "Sub Components")
        tabs.addTab(macro_tab, "Macros")
        tabs.addTab(code_tab, "Code")
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
        fit_tree_key_columns(self.list_widget)
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
        code_sections = data.get("code_sections", {})
        self.prefix_code_edit.setPlainText(code_sections.get("after_includes", ""))
        self.tail_code_edit.setPlainText(code_sections.get("before_footer", ""))
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
        fit_tree_key_columns(self.params, description_col=3)
        fit_table_columns(self.components)
        load_feature_macro_table(self.feature_macros, data.get("feature_macros", []))
        load_option_macro_table(self.option_macros, data.get("option_macros", []))
        load_option_set_table(self.option_sets, data.get("option_sets", {}))
        self.load_conditional_macros(data.get("conditional_macros", []))
        self.set_professional_text(pretty_json(data))
        try:
            self.set_code_text(self.generator().render_entity_header(entity), reveal=False)
        except Exception as exc:
            self.code_panel.setPlainText(f"// Failed to render entity header preview:\n// {exc}")
        self.loading = False
        if not self.restoring_undo:
            self.reset_undo_history(self.collect_current_data() or data)

    def load_param_rows(self, entity: HardwareEntity, data: dict[str, Any]) -> None:
        schema = self.window.library.schema(entity.schema_id)
        schema_data = read_json(self.window.schema_path(entity.schema_id))
        self.params.clear()
        known = set()
        groups = schema_data.get("parameter_groups", [])
        if not groups:
            groups = [{"name": "Parameters", "parameters": list(schema.parameters)}]
        for group in groups:
            group_item = self.create_entity_parameter_group_item(group.get("name", "Parameters"))
            self.params.addTopLevelItem(group_item)
            for name in group.get("parameters", []):
                if name not in schema.parameters:
                    continue
                self.add_entity_parameter_item(group_item, name, schema.parameters[name], data)
                known.add(name)
        missing_schema_params = [name for name in schema.parameters if name not in known]
        if missing_schema_params:
            group_item = self.create_entity_parameter_group_item("Ungrouped")
            self.params.addTopLevelItem(group_item)
            for name in missing_schema_params:
                self.add_entity_parameter_item(group_item, name, schema.parameters[name], data)
                known.add(name)
        for name, value in data.get("parameters", {}).items():
            if name in known:
                continue
            group_item = self.ensure_entity_parameter_group("Ungrouped")
            item = QTreeWidgetItem([name, display_parameter_value(value), "", ""])
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable)
            item.setData(0, Qt.ItemDataRole.UserRole, "parameter")
            group_item.addChild(item)
        self.params.expandAll()
        fit_tree_key_columns(self.params, description_col=3)

    def create_entity_parameter_group_item(self, name: str) -> QTreeWidgetItem:
        item = QTreeWidgetItem([name])
        item.setFlags(item.flags())
        item.setData(0, Qt.ItemDataRole.UserRole, "group")
        return item

    def ensure_entity_parameter_group(self, name: str) -> QTreeWidgetItem:
        for index in range(self.params.topLevelItemCount()):
            item = self.params.topLevelItem(index)
            if item.text(0) == name:
                return item
        item = self.create_entity_parameter_group_item(name)
        self.params.addTopLevelItem(item)
        return item

    def add_entity_parameter_item(
        self, group: QTreeWidgetItem, name: str, pspec: Any, data: dict[str, Any]
    ) -> QTreeWidgetItem:
        value = display_parameter_value(parameter_value_by_name(data.get("parameters", {}), name, pspec.c_name))
        item = QTreeWidgetItem([name, value, pspec.unit, pspec.description])
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable)
        item.setData(0, Qt.ItemDataRole.UserRole, "parameter")
        item.setToolTip(0, f"{pspec.description}\nUnit: {pspec.unit}")
        item.setToolTip(1, f"{pspec.description}\nUnit: {pspec.unit}")
        group.addChild(item)
        return item

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
                "option_sets": collect_option_set_table(self.option_sets),
                "feature_macros": collect_feature_macro_table(self.feature_macros),
                "option_macros": collect_option_macro_table(self.option_macros),
                "conditional_macros": self._table_conditional_macros(),
            }
        )
        code_sections = dict(data.get("code_sections", {}))
        prefix_code = self.prefix_code_edit.toPlainText().strip()
        if prefix_code:
            code_sections["after_includes"] = prefix_code
        else:
            code_sections.pop("after_includes", None)
        tail_code = self.tail_code_edit.toPlainText().strip()
        if tail_code:
            code_sections["before_footer"] = tail_code
        else:
            code_sections.pop("before_footer", None)
        if code_sections:
            data["code_sections"] = code_sections
        else:
            data.pop("code_sections", None)
        if not data["option_sets"]:
            data.pop("option_sets", None)
        if not data["feature_macros"]:
            data.pop("feature_macros", None)
        if not data["option_macros"]:
            data.pop("option_macros", None)
        if not data["conditional_macros"]:
            data.pop("conditional_macros", None)
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
        for group_index in range(self.params.topLevelItemCount()):
            group = self.params.topLevelItem(group_index)
            for child_index in range(group.childCount()):
                item = group.child(child_index)
                if item.data(0, Qt.ItemDataRole.UserRole) != "parameter":
                    continue
                name = tree_cell_text(self.params, item, 0)
                value = tree_cell_text(self.params, item, 1)
                if not name or not value.strip():
                    continue
                parsed = parse_parameter_value(value)
                if parsed == "":
                    continue
                if name in schema.parameters:
                    validate_value_for_format(name, parsed, schema.parameters[name].value_format)
                params[name] = parsed
        return params

    def select_parameter_symbol(self, item: QTreeWidgetItem) -> None:
        symbols = self.window.symbols_for_subcomponents(self.current_id)
        selected = choose_item(self, "Select Sub Component Symbol", symbols)
        if selected:
            item.setText(1, f"${{{selected}}}")

    def on_entity_param_double_clicked(self, item: QTreeWidgetItem, col: int) -> None:
        if item.data(0, Qt.ItemDataRole.UserRole) != "parameter":
            return
        if col == 1:
            self.select_parameter_symbol(item)
        elif col == 3:
            text = edit_multiline(self, "Parameter Description", tree_cell_text(self.params, item, 3))
            if text is not None:
                item.setText(3, text)

    def add_entity_component(self) -> None:
        row = self.components.rowCount()
        self.components.insertRow(row)
        set_combo(self.components, row, 1, ["entity", "inline"], "entity")

    def add_macro_row(self, table: QTableWidget, macro_type: str) -> None:
        row = table.rowCount()
        table.insertRow(row)
        set_combo(table, row, 1, ["Enable", "Disable"], "Enable")
        if macro_type == "option":
            set_combo(table, row, 2, ["1", "2", "3", "4", "5"], "1")
            set_item(table, row, 4, "1, 2, 3, 4, 5")

    def add_option_set(self) -> None:
        row = self.option_sets.rowCount()
        self.option_sets.insertRow(row)

    def add_conditional_macro(self) -> None:
        row = self.conditional_macros.rowCount()
        self.conditional_macros.insertRow(row)
        set_item(self.conditional_macros, row, 1, "{}")

    def load_conditional_macros(self, items: list[dict[str, Any]]) -> None:
        self.conditional_macros.setRowCount(0)
        for item in items:
            row = self.conditional_macros.rowCount()
            self.conditional_macros.insertRow(row)
            set_item(self.conditional_macros, row, 0, item.get("condition") or item.get("condition_macro") or item.get("selector", ""))
            set_item(self.conditional_macros, row, 1, pretty_json(item.get("macros", {})))
            set_item(self.conditional_macros, row, 2, item.get("description", ""))
        fit_table_columns(self.conditional_macros)

    def _table_conditional_macros(self) -> list[dict[str, Any]]:
        rows = []
        for row in range(self.conditional_macros.rowCount()):
            condition = item_text(self.conditional_macros, row, 0)
            if condition:
                rows.append(
                    {
                        "condition": condition,
                        "macros": parse_json_text(item_text(self.conditional_macros, row, 1), {}),
                        "description": item_text(self.conditional_macros, row, 2),
                    }
                )
        return rows

    def on_entity_macro_double_clicked(self, row: int, col: int) -> None:
        table = self.sender()
        if isinstance(table, QTableWidget) and (col == table.columnCount() - 1 or col == 4):
            edit_table_cell_multiline(table, row, col, self, "Macro Field")

    def on_entity_option_set_double_clicked(self, row: int, col: int) -> None:
        if col in {1, 2}:
            edit_table_cell_multiline(self.option_sets, row, col, self, "Option Preset")

    def on_entity_conditional_double_clicked(self, row: int, col: int) -> None:
        if col in {1, 2}:
            edit_table_cell_multiline(self.conditional_macros, row, col, self, "Conditional Macro")

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

    def create_items_widget(self):
        widget = QListWidget()
        widget.currentItemChanged.connect(self.on_current_changed)
        widget.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        widget.customContextMenuRequested.connect(self.show_project_context_menu)
        return widget

    def __init__(self, window: "MainWindow"):
        super().__init__(window, "Project Requirement", has_code=True)
        self.id_edit = QLineEdit()
        self.path_edit = QLineEdit()
        self.path_edit.setReadOnly(True)
        self.name_edit = QLineEdit()
        self.suite_edit = QLineEdit()
        self.version_edit = QLineEdit()
        self.updated_edit = QLineEdit()
        self.updated_edit.setReadOnly(True)
        self.header_edit = QLineEdit()
        self.description_edit = QTextEdit()
        self.description_edit.setFixedHeight(92)
        self.prefix_code = QTextEdit()
        self.tail_code = QTextEdit()

        self.hardware_view = QComboBox()
        self.hardware_view.addItems(["Tree", "Table"])
        self.hardware_tree = QTreeWidget()
        self.hardware_tree.setHeaderLabels(["Hardware", "Description"])
        fit_tree_key_columns(self.hardware_tree, description_col=1)
        self.hardware_tree.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.hardware_tree.customContextMenuRequested.connect(self.show_hardware_context_menu)
        self.hardware_tree.itemDoubleClicked.connect(self.on_hardware_tree_double_clicked)
        self.hardware = QTableWidget()
        set_table_headers(self.hardware, ["Entity", "Name", "Template", "Category", "Description"], QHeaderView.ResizeMode.Interactive)
        self.hardware.cellDoubleClicked.connect(self.on_hardware_cell_double_clicked)

        self.requirements = QTableWidget()
        set_table_headers(
            self.requirements,
            ["Name", "Macro", "Binding Type", "Binding Value", "Description"],
            QHeaderView.ResizeMode.Interactive,
        )
        self.requirements.cellDoubleClicked.connect(self.on_requirement_cell_double_clicked)
        self.feature_macros = QTableWidget()
        setup_feature_macro_table(self.feature_macros)
        self.feature_macros.cellDoubleClicked.connect(self.on_macro_cell_double_clicked)
        self.enum_macros = QTableWidget()
        setup_option_macro_table(self.enum_macros)
        self.enum_macros.cellDoubleClicked.connect(self.on_macro_cell_double_clicked)

        for widget in [self.id_edit, self.name_edit, self.suite_edit, self.version_edit, self.header_edit]:
            widget.textChanged.connect(self.mark_current_dirty)
        for widget in [self.description_edit, self.prefix_code, self.tail_code]:
            widget.textChanged.connect(self.mark_current_dirty)
        for table in [self.hardware, self.requirements, self.feature_macros, self.enum_macros]:
            table.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.enum_macros.cellChanged.connect(self.on_enum_macro_cell_changed)
        self.hardware.cellChanged.connect(lambda _row, _col: self.refresh_hardware_status())
        self.hardware_view.currentTextChanged.connect(self.update_hardware_view)

        tabs = QTabWidget()
        basic_tab = QWidget()
        basic_form = QFormLayout(basic_tab)
        basic_form.addRow("File Path", self.path_edit)
        basic_form.addRow("Project ID", self.id_edit)
        basic_form.addRow("Name", self.name_edit)
        basic_form.addRow("Suite", self.suite_edit)
        basic_form.addRow("Version", self.version_edit)
        basic_form.addRow("Last Updated", self.updated_edit)
        basic_form.addRow("Output Header", self.header_edit)
        basic_form.addRow("Description / @note", self.description_edit)

        hw_tab = QWidget()
        hw_layout = QVBoxLayout(hw_tab)
        add_hw = QPushButton("Add hardware")
        add_hw.clicked.connect(self.add_hardware)
        replace_hw = QPushButton("Replace hardware")
        replace_hw.clicked.connect(self.replace_selected_hardware)
        del_hw = QPushButton("Remove hardware")
        del_hw.clicked.connect(self.remove_hardware)
        hw_layout.addLayout(row_buttons([QLabel("View"), self.hardware_view, add_hw, replace_hw, del_hw]))
        hw_layout.addWidget(self.hardware_tree)
        hw_layout.addWidget(self.hardware)

        req_tab = QWidget()
        req_layout = QVBoxLayout(req_tab)
        add_req = QPushButton("Add requirement")
        add_req.clicked.connect(self.add_requirement)
        del_req = QPushButton("Remove requirement")
        del_req.clicked.connect(lambda: self.requirements.removeRow(max(0, self.requirements.currentRow())))
        req_layout.addWidget(self.requirements)
        req_layout.addLayout(row_buttons([add_req, del_req]))

        macro_tab = QWidget()
        macro_layout = QVBoxLayout(macro_tab)
        add_feature = QPushButton("Add selection macro")
        add_feature.clicked.connect(self.add_feature_macro)
        del_feature = QPushButton("Remove selection macro")
        del_feature.clicked.connect(lambda: self.feature_macros.removeRow(max(0, self.feature_macros.currentRow())))
        add_enum = QPushButton("Add option macro")
        add_enum.clicked.connect(self.add_enum_macro)
        del_enum = QPushButton("Remove option macro")
        del_enum.clicked.connect(lambda: self.enum_macros.removeRow(max(0, self.enum_macros.currentRow())))
        macro_layout.addWidget(QLabel("Selection macros"))
        macro_layout.addWidget(self.feature_macros)
        macro_layout.addLayout(row_buttons([add_feature, del_feature]))
        macro_layout.addWidget(QLabel("Option macros"))
        macro_layout.addWidget(self.enum_macros)
        macro_layout.addLayout(row_buttons([add_enum, del_enum]))

        code_tab = QWidget()
        code_layout = QVBoxLayout(code_tab)
        code_splitter = QSplitter(Qt.Orientation.Vertical)
        prefix_panel = QWidget()
        prefix_layout = QVBoxLayout(prefix_panel)
        prefix_layout.addWidget(QLabel("Prefix code"))
        prefix_layout.addWidget(self.prefix_code)
        tail_panel = QWidget()
        tail_layout = QVBoxLayout(tail_panel)
        tail_layout.addWidget(QLabel("Tail code"))
        tail_layout.addWidget(self.tail_code)
        code_splitter.addWidget(prefix_panel)
        code_splitter.addWidget(tail_panel)
        code_splitter.setSizes([360, 360])
        code_layout.addWidget(code_splitter)

        tabs.addTab(basic_tab, "Basic")
        tabs.addTab(hw_tab, "Hardware Includes")
        tabs.addTab(req_tab, "Requirements")
        tabs.addTab(macro_tab, "Macros")
        tabs.addTab(code_tab, "Code")
        preview = QPushButton("Preview header")
        preview.clicked.connect(self.preview_project_header)
        save = QPushButton("Save project")
        save.clicked.connect(self.save_current)
        self.form_layout.addWidget(tabs)
        self.form_layout.addLayout(row_buttons([preview, save]))
        self.update_hardware_view()

    def refresh_list(self) -> None:
        current = self.current_id
        self.list_widget.clear()
        for path in self.window.project_paths():
            data = read_json(path)
            tags = [data.get("id", ""), data.get("display_name", ""), data.get("suite", ""), data.get("version", "")]
            if not self.filter_match(tags):
                continue
            item = QListWidgetItem(f"{data.get('id', path.stem)}  ({data.get('suite', '')})")
            item.setData(Qt.ItemDataRole.UserRole, data.get("id", path.stem))
            self.list_widget.addItem(item)
            if data.get("id") == current:
                self.list_widget.setCurrentItem(item)
        self.select_first()

    def show_project_context_menu(self, pos) -> None:
        item = self.list_widget.itemAt(pos)
        if item is not None:
            self.list_widget.setCurrentItem(item)
        menu = QMenu(self)
        menu.addAction("Open project file", self.open_project_file)
        if self.current_id:
            menu.addAction("Save", self.save_current)
            menu.addAction("Copy project file", self.copy_project_file)
        menu.exec(self.list_widget.viewport().mapToGlobal(pos))

    def open_project_file(self) -> None:
        selected, _filter = QFileDialog.getOpenFileName(self, "Open project requirement", str(self.window.library_root), "JSON (*.json)")
        if not selected:
            return
        path = Path(selected).resolve()
        if path not in self.window.project_dirs:
            self.window.project_dirs.append(path)
            self.window.save_recent_project_files()
        data = read_json(path)
        self.window.reload()
        self.current_id = data.get("id", path.stem)
        self.refresh_list()
        self.load_current()
        self.message("Opened", str(path))

    def copy_project_file(self) -> None:
        if not self.current_id:
            return
        source = self.window.project_path(self.current_id)
        target, _filter = QFileDialog.getSaveFileName(self, "Copy project requirement", str(source.with_name(f"{source.stem}_copy.json")), "JSON (*.json)")
        if not target:
            return
        data = self.collect_current_data() or read_json(source)
        data["id"] = Path(target).stem
        data["display_name"] = f"{data.get('display_name', self.current_id)} Copy"
        self.window.write_json(Path(target), data)
        target_path = Path(target).resolve()
        if target_path not in self.window.project_dirs:
            self.window.project_dirs.append(target_path)
            self.window.save_recent_project_files()
        self.window.reload()
        self.current_id = data["id"]
        self.refresh_list()
        self.load_current()
        self.message("Copied", str(target_path))

    def load_current(self) -> None:
        if not self.current_id:
            return
        data = self.data_for_id(self.current_id, self.window.project_path(self.current_id))
        self.loading = True
        self.path_edit.setText(str(self.window.project_path(self.current_id)))
        self.id_edit.setText(data.get("id", ""))
        self.name_edit.setText(data.get("display_name", ""))
        self.suite_edit.setText(data.get("suite", ""))
        self.version_edit.setText(data.get("version", "0.1.0"))
        self.updated_edit.setText(data.get("updated_at", ""))
        self.header_edit.setText(data.get("output_header", "sdpe_project_bindings.h"))
        self.description_edit.setPlainText(data.get("description", ""))
        sections = data.get("code_sections", {})
        self.prefix_code.setPlainText(sections.get("after_extern_open", ""))
        self.tail_code.setPlainText(sections.get("before_footer", ""))
        self.load_hardware(data)
        self.load_requirements(data)
        self.load_macro_tables(data)
        self.set_professional_text(pretty_json(data))
        try:
            self.set_code_text(HeaderGenerator(self.window.library, self.window.default_output_dir).render_project_header(data), reveal=False)
        except Exception:
            self.code_panel.clear()
        self.loading = False
        if not self.restoring_undo:
            self.reset_undo_history(self.collect_current_data() or data)

    def load_hardware(self, data: dict[str, Any]) -> None:
        self.hardware.setRowCount(0)
        for hw in data.get("hardware", []):
            row = self.hardware.rowCount()
            self.hardware.insertRow(row)
            set_item(self.hardware, row, 0, hw.get("entity", ""))
            self.populate_hardware_info(row, hw.get("entity", ""))
        self.refresh_hardware_status()
        fit_table_columns(self.hardware)

    def load_requirements(self, data: dict[str, Any]) -> None:
        self.requirements.setRowCount(0)
        for req in data.get("requirements", []):
            row = self.requirements.rowCount()
            self.requirements.insertRow(row)
            btype, bvalue = binding_to_cells(req.get("binding", {}))
            for col, value in enumerate([req.get("role", ""), req.get("macro", ""), btype, bvalue, req.get("description", "")]):
                if col == 2:
                    set_combo(self.requirements, row, col, binding_type_options(), value)
                else:
                    set_item(self.requirements, row, col, value)
        fit_table_columns(self.requirements)

    def load_macro_tables(self, data: dict[str, Any]) -> None:
        load_feature_macro_table(self.feature_macros, data.get("feature_macros", []))
        load_option_macro_table(self.enum_macros, data.get("option_macros", []))
        for row in range(self.enum_macros.rowCount()):
            self.sync_option_macro_preset(row, prefer_preset=bool(item_text(self.enum_macros, row, 3)))

    def collect_current_data(self) -> dict[str, Any] | None:
        if not self.current_id:
            return None
        path = self.window.project_path(self.current_id)
        data = read_json(path) if path.exists() else {}
        sections = dict(data.get("code_sections", {}))
        self._set_optional_section(sections, "after_extern_open", self.prefix_code.toPlainText())
        self._set_optional_section(sections, "before_footer", self.tail_code.toPlainText())
        data.update(
            {
                "id": self.id_edit.text().strip(),
                "display_name": self.name_edit.text().strip(),
                "description": self.description_edit.toPlainText().strip(),
                "suite": self.suite_edit.text().strip(),
                "version": self.version_edit.text().strip(),
                "updated_at": self.updated_edit.text().strip() or date.today().isoformat(),
                "output_header": self.header_edit.text().strip(),
                "hardware": self._table_hardware(),
                "requirements": self._table_requirements(),
                "feature_macros": self._table_feature_macros(),
                "option_macros": self._table_option_macros(),
            }
        )
        if sections:
            data["code_sections"] = sections
        else:
            data.pop("code_sections", None)
        return data

    def save_current(self) -> None:
        try:
            path = self.window.project_path(self.current_id)
            data = self.collect_current_data()
            if data is None:
                return
            data["updated_at"] = date.today().isoformat()
            self.updated_edit.setText(data["updated_at"])
            self.window.write_json(path, data)
            self.dirty_ids.discard(self.current_id)
            self.drafts.pop(self.current_id, None)
            self.window.reload()
            self.refresh_list()
            self.message("Saved", f"Project saved: {path}")
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))

    def path_for_id(self, item_id: str) -> Path:
        return self.window.project_path(item_id)

    def _set_optional_section(self, sections: dict[str, Any], name: str, text: str) -> None:
        value = text.strip()
        if value:
            sections[name] = value
        else:
            sections.pop(name, None)

    def _table_hardware(self) -> list[dict[str, str]]:
        rows = []
        for row in range(self.hardware.rowCount()):
            entity = item_text(self.hardware, row, 0)
            if entity:
                rows.append({"entity": entity})
        return rows

    def _table_requirements(self) -> list[dict[str, Any]]:
        rows = []
        for row in range(self.requirements.rowCount()):
            macro = item_text(self.requirements, row, 1)
            if macro:
                rows.append(
                    {
                        "role": item_text(self.requirements, row, 0),
                        "macro": macro,
                        "binding": cells_to_binding(item_text(self.requirements, row, 2), item_text(self.requirements, row, 3)),
                        "description": item_text(self.requirements, row, 4),
                    }
                )
        return rows

    def _table_feature_macros(self) -> list[dict[str, Any]]:
        return collect_feature_macro_table(self.feature_macros)

    def _table_option_macros(self) -> list[dict[str, Any]]:
        return collect_option_macro_table(self.enum_macros)

    def add_feature_macro(self) -> None:
        row = self.feature_macros.rowCount()
        self.feature_macros.insertRow(row)
        set_combo(self.feature_macros, row, 1, ["Enable", "Disable"], "Enable")

    def add_requirement(self) -> None:
        row = self.requirements.rowCount()
        self.requirements.insertRow(row)
        set_combo(self.requirements, row, 2, binding_type_options(), "number")

    def add_enum_macro(self) -> None:
        row = self.enum_macros.rowCount()
        self.enum_macros.insertRow(row)
        set_combo(self.enum_macros, row, 1, ["Enable", "Disable"], "Enable")
        set_combo(self.enum_macros, row, 2, ["1", "2", "3", "4", "5"], "1")
        set_item(self.enum_macros, row, 4, "1, 2, 3, 4, 5")

    def add_hardware(self, category_hint: str = "") -> None:
        selected = self.choose_entity(category_hint)
        if selected:
            row = self.hardware.rowCount()
            self.hardware.insertRow(row)
            set_item(self.hardware, row, 0, selected)
            self.populate_hardware_info(row, selected)
            self.refresh_hardware_status()

    def remove_hardware(self) -> None:
        row = self.hardware.currentRow()
        if row >= 0:
            self.hardware.removeRow(row)
            self.refresh_hardware_status()

    def choose_entity(self, category_hint: str = "") -> str | None:
        hinted: list[str] = []
        rest: list[str] = []
        for entity_id in sorted(self.window.library.entity_files):
            try:
                entity = self.window.library.entity(entity_id)
                schema = self.window.library.schema(entity.schema_id)
                label = f"{entity_id} | {entity.display_name} | {schema.display_name} | {schema.category}"
                (hinted if category_hint and schema.category == category_hint else rest).append(label)
            except SDPEError:
                continue
        selected = choose_item(self, "Select Hardware Entity", hinted + rest)
        return selected.split("|", 1)[0].strip() if selected else None

    def on_hardware_cell_double_clicked(self, row: int, col: int) -> None:
        entity_id = item_text(self.hardware, row, 0)
        if col == 0 and entity_id:
            self.open_entity_page(entity_id)
        elif col == 4:
            edit_table_cell_multiline(self.hardware, row, col, self, "Hardware Description")

    def replace_selected_hardware(self) -> None:
        row = self.hardware.currentRow()
        if row < 0:
            return
        old_entity = item_text(self.hardware, row, 0)
        selected = self.choose_entity(self.entity_category(old_entity))
        if selected:
            set_item(self.hardware, row, 0, selected)
            self.populate_hardware_info(row, selected)
            self.replace_requirement_entity(old_entity, selected)
            self.refresh_hardware_status()

    def on_hardware_tree_double_clicked(self, item: QTreeWidgetItem, _column: int) -> None:
        entity_id = item.data(0, Qt.ItemDataRole.UserRole)
        if entity_id:
            self.open_entity_page(entity_id)

    def open_entity_page(self, entity_id: str) -> None:
        for index, page in enumerate(self.window.pages):
            if isinstance(page, EntityPage):
                page.current_id = entity_id
                page.refresh_list()
                page.load_current()
                self.window.tabs.setCurrentWidget(page)
                return

    def on_requirement_cell_double_clicked(self, row: int, col: int) -> None:
        if col == 4:
            edit_table_cell_multiline(self.requirements, row, col, self, "Requirement Description")
            return
        if col != 3:
            return
        symbols: list[str] = []
        for hw in self._table_hardware():
            try:
                symbols.extend(self.window.symbols_for_entity(hw["entity"]))
            except SDPEError:
                continue
        selected = choose_tree_item(self, "Select Requirement Binding", sorted(dict.fromkeys(symbols)))
        if selected:
            if item_text(self.requirements, row, 2) != "expr":
                set_combo(self.requirements, row, 2, binding_type_options(), "export")
            set_item(self.requirements, row, 3, f"${{{selected}}}")

    def on_macro_cell_double_clicked(self, row: int, col: int) -> None:
        table = self.sender()
        if table is self.feature_macros and col == 3:
            edit_table_cell_multiline(self.feature_macros, row, col, self, "Selection Macro Description")
        elif table is self.enum_macros and col == 3:
            presets = self.option_preset_labels()
            selected = choose_item(self, "Select Option Preset", presets)
            if selected:
                set_item(self.enum_macros, row, col, selected.split("|", 1)[0].strip())
                self.sync_option_macro_preset(row, prefer_preset=True)
        elif table is self.enum_macros and col == 4:
            edit_table_cell_multiline(self.enum_macros, row, col, self, "Option Macro Options")
            self.sync_option_macro_value_combo(row)
        elif table is self.enum_macros and col == 5:
            edit_table_cell_multiline(self.enum_macros, row, col, self, "Option Macro Description")

    def on_enum_macro_cell_changed(self, row: int, col: int) -> None:
        if self.loading:
            return
        if col == 3:
            self.sync_option_macro_preset(row, prefer_preset=True)
        elif col == 4:
            self.sync_option_macro_value_combo(row)

    def sync_option_macro_preset(self, row: int, prefer_preset: bool = True) -> None:
        preset = item_text(self.enum_macros, row, 3)
        options = self.option_preset_options(preset) if preset else []
        if not options and not prefer_preset:
            return
        old_state = self.enum_macros.blockSignals(True)
        try:
            if options:
                set_item(self.enum_macros, row, 4, ", ".join(str(item) for item in options))
            self.sync_option_macro_value_combo(row)
        finally:
            self.enum_macros.blockSignals(old_state)

    def sync_option_macro_value_combo(self, row: int) -> None:
        options = [item.strip() for item in item_text(self.enum_macros, row, 4).split(",") if item.strip()]
        if not options:
            return
        current = item_text(self.enum_macros, row, 2)
        if current not in options:
            current = options[0]
        set_combo(self.enum_macros, row, 2, options, current)

    def replace_requirement_entity(self, old_entity: str, new_entity: str) -> None:
        if not old_entity or old_entity == new_entity:
            return
        for row in range(self.requirements.rowCount()):
            value = item_text(self.requirements, row, 3)
            if value == old_entity or value.startswith(f"{old_entity}."):
                set_item(self.requirements, row, 3, new_entity + value[len(old_entity):])

    def populate_hardware_info(self, row: int, entity_id: str) -> None:
        try:
            entity = self.window.library.entity(entity_id)
            schema = self.window.library.schema(entity.schema_id)
            set_item(self.hardware, row, 1, entity.display_name)
            set_item(self.hardware, row, 2, schema.display_name)
            set_item(self.hardware, row, 3, schema.category)
            set_item(self.hardware, row, 4, entity.description or schema.description)
        except SDPEError:
            for col in range(1, 5):
                set_item(self.hardware, row, col, "")

    def entity_category(self, entity_id: str) -> str:
        try:
            entity = self.window.library.entity(entity_id)
            return self.window.library.schema(entity.schema_id).category
        except SDPEError:
            return ""

    def refresh_hardware_status(self) -> None:
        old_state = self.hardware.blockSignals(True)
        try:
            for row in range(self.hardware.rowCount()):
                entity_id = item_text(self.hardware, row, 0)
                valid = entity_id in self.window.library.entity_files
                if valid:
                    self.populate_hardware_info(row, entity_id)
                for col in range(self.hardware.columnCount()):
                    item = self.hardware.item(row, col)
                    if item is None:
                        set_item(self.hardware, row, col, "")
                        item = self.hardware.item(row, col)
                    if entity_id and not valid:
                        item.setBackground(QColor(255, 235, 235))
                    else:
                        item.setData(Qt.ItemDataRole.BackgroundRole, None)
        finally:
            self.hardware.blockSignals(old_state)
        self.refresh_hardware_tree()

    def refresh_hardware_tree(self) -> None:
        self.hardware_tree.clear()
        categories: dict[str, QTreeWidgetItem] = {}
        for hw in self._table_hardware():
            entity_id = hw["entity"]
            category = self.entity_category(entity_id) or "Missing"
            if category not in categories:
                node = QTreeWidgetItem([category])
                self.hardware_tree.addTopLevelItem(node)
                categories[category] = node
            entity_node = QTreeWidgetItem([entity_id, self.entity_description(entity_id)])
            entity_node.setData(0, Qt.ItemDataRole.UserRole, entity_id)
            categories[category].addChild(entity_node)
            self.add_component_nodes(entity_node, entity_id, set())
        self.hardware_tree.expandAll()
        fit_tree_key_columns(self.hardware_tree, description_col=1)

    def add_component_nodes(self, parent: QTreeWidgetItem, entity_id: str, seen: set[str]) -> None:
        if entity_id in seen:
            return
        seen.add(entity_id)
        try:
            entity = self.window.library.entity(entity_id)
        except SDPEError:
            return
        for slot, comp in entity.components.items():
            child = QTreeWidgetItem([f"{slot}: {comp.entity.id}", comp.entity.description])
            child.setData(0, Qt.ItemDataRole.UserRole, comp.entity.id)
            parent.addChild(child)
            self.add_component_nodes(child, comp.entity.id, seen)

    def entity_description(self, entity_id: str) -> str:
        try:
            entity = self.window.library.entity(entity_id)
            schema = self.window.library.schema(entity.schema_id)
            return entity.description or schema.description
        except SDPEError:
            return ""

    def option_preset_labels(self) -> list[str]:
        labels: list[str] = []
        for schema in self.window.library.schemas.values():
            schema_data = read_json(self.window.schema_path(schema.id))
            for name, values in schema_data.get("option_sets", {}).items():
                options = values.get("options", values) if isinstance(values, dict) else values
                if not isinstance(options, list):
                    options = [options]
                labels.append(f"{name} | template:{schema.id} | {', '.join(str(v) for v in options)}")
        for hw in self._table_hardware():
            entity_id = hw["entity"]
            try:
                data = read_json(self.window.entity_path(entity_id))
            except Exception:
                continue
            for name, values in data.get("option_sets", {}).items():
                if isinstance(values, list):
                    labels.append(f"{entity_id}.{name} | entity:{entity_id} | {', '.join(str(v) for v in values)}")
                elif isinstance(values, dict):
                    options = values.get("options", [])
                    if not isinstance(options, list):
                        options = [options]
                    labels.append(f"{entity_id}.{name} | entity:{entity_id} | {', '.join(str(v) for v in options)}")
        return labels

    def option_preset_options(self, preset: str) -> list[str]:
        key = preset.strip()
        if not key:
            return []

        def values_from_store(store: dict[str, Any], name: str) -> list[str]:
            if name not in store:
                return []
            value = store[name]
            options = value.get("options", value) if isinstance(value, dict) else value
            if not isinstance(options, list):
                options = [options]
            return [str(item) for item in options]

        if "." in key:
            owner, name = key.split(".", 1)
            if owner in self.window.library.entity_files:
                entity_data = read_json(self.window.entity_path(owner))
                options = values_from_store(entity_data.get("option_sets", {}), name)
                if options:
                    return options
                schema = self.window.library.schema(self.window.library.entity(owner).schema_id)
                return values_from_store(read_json(self.window.schema_path(schema.id)).get("option_sets", {}), name)
            if owner in self.window.library.schemas:
                return values_from_store(read_json(self.window.schema_path(owner)).get("option_sets", {}), name)

        for hw in self._table_hardware():
            entity_id = hw["entity"]
            if entity_id not in self.window.library.entity_files:
                continue
            entity_data = read_json(self.window.entity_path(entity_id))
            options = values_from_store(entity_data.get("option_sets", {}), key)
            if options:
                return options
            schema = self.window.library.schema(self.window.library.entity(entity_id).schema_id)
            options = values_from_store(read_json(self.window.schema_path(schema.id)).get("option_sets", {}), key)
            if options:
                return options

        for schema in self.window.library.schemas.values():
            options = values_from_store(read_json(self.window.schema_path(schema.id)).get("option_sets", {}), key)
            if options:
                return options
        return []

    def update_hardware_view(self) -> None:
        show_tree = self.hardware_view.currentText() == "Tree"
        self.hardware_tree.setVisible(show_tree)
        self.hardware.setVisible(not show_tree)
        self.refresh_hardware_status()

    def show_hardware_context_menu(self, pos) -> None:
        item = self.hardware_tree.itemAt(pos)
        category = item.text(0) if item and not item.parent() else ""
        menu = QMenu(self)
        menu.addAction("Insert hardware", lambda: self.add_hardware(category))
        menu.addAction("Delete selected root hardware", self.remove_tree_root_hardware)
        menu.exec(self.hardware_tree.viewport().mapToGlobal(pos))

    def remove_tree_root_hardware(self) -> None:
        item = self.hardware_tree.currentItem()
        while item and item.parent() and item.parent().parent():
            item = item.parent()
        if item and item.parent():
            entity_id = item.data(0, Qt.ItemDataRole.UserRole)
            for row in range(self.hardware.rowCount()):
                if item_text(self.hardware, row, 0) == entity_id:
                    self.hardware.removeRow(row)
                    break
            self.refresh_hardware_status()

    def preview_project_header(self) -> None:
        try:
            data = self.collect_current_data()
            if data is not None:
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
        self.loading = True
        data = self.data_for_id(self.current_id, self.window.project_path(self.current_id))
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
        self.loading = False
        if not self.restoring_undo:
            self.reset_undo_history(self.collect_current_data() or data)

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

    def path_for_id(self, item_id: str) -> Path:
        return self.window.project_path(item_id)

    def collect_current_data(self) -> dict[str, Any] | None:
        if not self.current_id:
            return None
        data = read_json(self.window.project_path(self.current_id))
        requirements = data.get("requirements", [])
        for row in range(min(len(requirements), self.binding_table.rowCount())):
            requirements[row]["macro"] = item_text(self.binding_table, row, 0)
            requirements[row]["binding"] = cells_to_binding(
                item_text(self.binding_table, row, 1), item_text(self.binding_table, row, 2)
            )
            requirements[row]["description"] = item_text(self.binding_table, row, 3)
        data["requirements"] = requirements
        return data

    def save_current(self) -> None:
        try:
            path = self.window.project_path(self.current_id)
            data = self.collect_current_data()
            if data is None:
                return
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
        self.project_dirs.extend(self.load_recent_project_files())
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
        if mode == "project":
            self.pages.append(EntityPage(self))
        self.settings_page = SettingsPage(self)
        for page in self.pages:
            self.tabs.addTab(page, page.title)
        self.tabs.addTab(self.settings_page, self.settings_page.title)
        self.setCentralWidget(self.tabs)
        self.refresh_pages()

    def load_library(self) -> SDPELibrary:
        return SDPELibrary(self.library_root, self.schema_dirs, self.entity_dirs).load()

    def recent_project_file(self) -> Path:
        return self.library_root / ".sdpe_project_files.json"

    def load_recent_project_files(self) -> list[Path]:
        path = self.recent_project_file()
        if not path.exists():
            return []
        try:
            data = read_json(path)
            return [Path(item).resolve() for item in data.get("projects", [])]
        except Exception:
            return []

    def save_recent_project_files(self) -> None:
        files = [str(path.resolve()) for path in self.project_dirs if path.is_file()]
        if not files:
            return
        self.recent_project_file().write_text(pretty_json({"projects": sorted(dict.fromkeys(files))}) + "\n", encoding="utf-8")

    def refresh_pages(self) -> None:
        for page in self.pages:
            page.refresh_list()

    def reload(self) -> None:
        self.library = self.load_library()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override
        dirty: list[tuple[SDPEPage, str, Path]] = []
        for page in self.pages:
            for item_id, path in page.dirty_items():
                dirty.append((page, item_id, path))
        if not dirty:
            event.accept()
            return

        preview = "\n".join(f"- {page.title}: {item_id}\n  {path}" for page, item_id, path in dirty[:8])
        if len(dirty) > 8:
            preview += f"\n... and {len(dirty) - 8} more"
        message = QMessageBox(self)
        message.setIcon(QMessageBox.Icon.Warning)
        message.setWindowTitle("Unsaved Changes")
        message.setText("Some SDPE files have unsaved changes.")
        message.setInformativeText(f"{preview}\n\nSave changes before closing?")
        save_button = message.addButton("Save", QMessageBox.ButtonRole.AcceptRole)
        discard_button = message.addButton("Discard", QMessageBox.ButtonRole.DestructiveRole)
        cancel_button = message.addButton("Cancel", QMessageBox.ButtonRole.RejectRole)
        message.setDefaultButton(save_button)
        message.exec()
        clicked = message.clickedButton()
        if clicked is cancel_button:
            event.ignore()
            return
        if clicked is discard_button:
            event.accept()
            return
        if clicked is save_button:
            try:
                saved = 0
                for page in self.pages:
                    saved += page.save_dirty()
                if saved:
                    self.reload()
                event.accept()
            except Exception as exc:  # pragma: no cover - close guard.
                QMessageBox.critical(self, "Save Failed", str(exc))
                event.ignore()
            return
        event.ignore()

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
        for key in ("export", "macro", "string", "float", "number", "expr", "literal"):
            if key in binding:
                return key, str(binding[key])
    if isinstance(binding, str):
        return ("export" if "." in binding else "macro"), binding
    return "literal", ""


def binding_type_options() -> list[str]:
    return ["export", "expr", "macro", "string", "float", "number", "literal"]


def cells_to_binding(kind: str, value: str) -> dict[str, str]:
    key = kind.strip() or "literal"
    if key not in set(binding_type_options()):
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
