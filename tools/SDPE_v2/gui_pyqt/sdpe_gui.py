"""PyQt manager for SDPE v2 hardware templates, entities, and bindings."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from datetime import date
from pathlib import Path
from typing import Any

try:
    from PyQt6.QtCore import QTimer, Qt
    from PyQt6.QtGui import QAction, QColor, QKeySequence, QPen, QShortcut, QTextCursor, QTextDocument
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
        QStyledItemDelegate,
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
        from PySide6.QtGui import QAction, QColor, QKeySequence, QPen, QShortcut, QTextCursor, QTextDocument
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
            QStyledItemDelegate,
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

from sdpe_v2.generator import HeaderGenerator, macro_name
from sdpe_v2.library import SDPELibrary
from sdpe_v2.model import HardwareEntity, HardwareSchema, SDPEError
from sdpe_v2.util import read_json
from gui_pyqt.dialogs import choose_item, choose_tree_item, confirm_delete, edit_multiline, prompt_identifier


VALIDATION_BORDER_ROLE = Qt.ItemDataRole.UserRole.value + 101


class SDPEComboBox(QComboBox):
    """Combo box that does not steal mouse-wheel scrolling unless focused."""

    def __init__(self):
        super().__init__()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def wheelEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        if QApplication.focusWidget() is self:
            super().wheelEvent(event)
        else:
            event.ignore()


class ValidationBorderDelegate(QStyledItemDelegate):
    """Draw a validation outline without changing themed text/background colors."""

    def paint(self, painter, option, index) -> None:  # noqa: ANN001, N802 - Qt override signature.
        super().paint(painter, option, index)
        if not index.data(VALIDATION_BORDER_ROLE):
            return
        painter.save()
        pen = QPen(QColor(220, 40, 40))
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawRect(option.rect.adjusted(1, 1, -2, -2))
        painter.restore()


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


def project_hardware_items(data: dict[str, Any]) -> list[Any]:
    for key in ("hardware", "hardware_includes", "hardwareIncludes", "hardware_entities", "hardware_presets"):
        value = data.get(key)
        if isinstance(value, list):
            return value
    return []


def project_hardware_entity_id(item: Any) -> str:
    if isinstance(item, str):
        return item.strip()
    if isinstance(item, dict):
        for key in ("entity", "id", "name", "hardware"):
            value = item.get(key)
            if isinstance(value, str) and value.strip():
                return value.strip()
    return ""


def is_inherited_hardware_row(table: QTableWidget, row: int) -> bool:
    item = table.item(row, 0)
    return item is not None and item.data(Qt.ItemDataRole.UserRole + 10) == "inherited"


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
    for col, header in enumerate(headers):
        header_item = table.horizontalHeaderItem(col)
        if header_item is None:
            continue
        if header == "En":
            header_item.setToolTip("Enable: checked macros are emitted normally; unchecked macros are commented out.")
        elif header == "Wk":
            header_item.setToolTip("Weak macro: checked macros are wrapped by #ifndef / #define / #endif.")
    table.horizontalHeader().setSectionResizeMode(mode)
    table.horizontalHeader().setStretchLastSection(mode != QHeaderView.ResizeMode.Interactive)
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
    install_table_status_descriptions(table)
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
    table.horizontalHeader().setStretchLastSection(False)
    table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
    table.resizeColumnsToContents()
    for col in range(table.columnCount()):
        header = table.horizontalHeaderItem(col)
        if header and "description" in header.text().lower():
            continue
        table.setColumnWidth(col, min(max(table.columnWidth(col) + 18, 96), max_width))


def fit_tree_key_columns(tree: QTreeWidget, description_col: int | None = None, interactive: bool = False) -> None:
    header = tree.header()
    header.setStretchLastSection(description_col is not None and not interactive)
    if description_col is None:
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Interactive if interactive else QHeaderView.ResizeMode.ResizeToContents)
        tree.resizeColumnToContents(0)
        tree.setColumnWidth(0, max(tree.columnWidth(0), 260))
        return
    for col in range(tree.columnCount()):
        mode = QHeaderView.ResizeMode.Interactive if interactive else (
            QHeaderView.ResizeMode.Stretch if col == description_col else QHeaderView.ResizeMode.ResizeToContents
        )
        header.setSectionResizeMode(col, mode)
        if interactive:
            tree.resizeColumnToContents(col)
    tree.setColumnWidth(0, max(tree.columnWidth(0), 360))
    if interactive:
        for col in range(tree.columnCount()):
            if col == description_col:
                tree.setColumnWidth(col, max(tree.columnWidth(col), 220))
            else:
                tree.setColumnWidth(col, max(tree.columnWidth(col) + 18, 120))


def tree_cell_text(tree: QTreeWidget, item: QTreeWidgetItem, col: int) -> str:
    widget = tree.itemWidget(item, col)
    if isinstance(widget, QComboBox):
        return widget.currentText().strip()
    checked = checkbox_widget_checked(widget)
    if checked is not None:
        return "1" if checked else "0"
    value = item.data(col, Qt.ItemDataRole.UserRole)
    if col != 0 and value is not None:
        return str(value).strip()
    return item.text(col).strip()


def set_tree_combo(tree: QTreeWidget, item: QTreeWidgetItem, col: int, values: list[str], current: Any = "") -> None:
    combo = SDPEComboBox()
    combo.addItems(values)
    combo.setEditable(True)
    current_text = "" if current is None else str(current)
    item.setData(col, Qt.ItemDataRole.UserRole, current_text)
    item.setText(col, "")
    combo.setCurrentText(current_text)
    combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
    combo.currentTextChanged.connect(lambda text, i=item, c=col, t=tree: (i.setData(c, Qt.ItemDataRole.UserRole, text), notify_tree_changed(t)))
    tree.setItemWidget(item, col, combo)


def set_tree_check(item: QTreeWidgetItem, col: int, checked: bool) -> None:
    item.setText(col, "")
    item.setData(col, Qt.ItemDataRole.UserRole, bool(checked))
    tree = item.treeWidget()
    if tree is not None:
        def changed(i=item, c=col, t=tree) -> None:
            current = checkbox_widget_checked(t.itemWidget(i, c))
            if current is not None:
                i.setData(c, Qt.ItemDataRole.UserRole, current)
            notify_tree_changed(t)

        tree.setItemWidget(item, col, centered_checkbox(checked, changed))


def tree_checked(item: QTreeWidgetItem, col: int, default: bool = False) -> bool:
    tree = item.treeWidget()
    checked = checkbox_widget_checked(tree.itemWidget(item, col) if tree is not None else None)
    if checked is not None:
        return checked
    value = item.data(col, Qt.ItemDataRole.UserRole)
    if isinstance(value, bool):
        return value
    state = item.checkState(col)
    if state == Qt.CheckState.Checked:
        return True
    if state == Qt.CheckState.Unchecked:
        return False
    return default


def show_status_description(widget: QWidget, text: str) -> None:
    text = text.strip()
    if not text:
        return
    window = widget.window()
    if hasattr(window, "statusBar"):
        window.statusBar().showMessage(text)


def description_column(headers: list[str]) -> int | None:
    for index, header in enumerate(headers):
        if "description" in header.lower():
            return index
    return None


def install_table_status_descriptions(table: QTableWidget) -> None:
    headers = [table.horizontalHeaderItem(col).text() if table.horizontalHeaderItem(col) else "" for col in range(table.columnCount())]
    desc_col = description_column(headers)
    if desc_col is None:
        return
    table.setMouseTracking(True)

    def show_row(row: int) -> None:
        if row < 0 or row >= table.rowCount():
            return
        show_status_description(table, item_text(table, row, desc_col))

    table.cellEntered.connect(lambda row, _col: show_row(row))
    table.currentCellChanged.connect(lambda row, _col, _previous_row, _previous_col: show_row(row))


def install_tree_status_descriptions(tree: QTreeWidget, description_col: int | None = None) -> None:
    if description_col is None:
        headers = [tree.headerItem().text(col) for col in range(tree.columnCount())]
        description_col = description_column(headers)
    if description_col is None:
        return
    tree.setMouseTracking(True)

    def show_item(item: QTreeWidgetItem | None) -> None:
        if item is None:
            return
        show_status_description(tree, tree_cell_text(tree, item, description_col))

    tree.itemEntered.connect(lambda item, _col: show_item(item))
    tree.currentItemChanged.connect(lambda current, _previous: show_item(current))


def notify_tree_changed(tree: QTreeWidget) -> None:
    page = page_for_widget(tree)
    if page is not None:
        page.mark_current_dirty()


def item_text(table: QTableWidget, row: int, col: int) -> str:
    widget = table.cellWidget(row, col)
    if isinstance(widget, QComboBox):
        return widget.currentText().strip()
    checked = checkbox_widget_checked(widget)
    if checked is not None:
        return "1" if checked else "0"
    item = table.item(row, col)
    if item is None:
        return ""
    if item.data(Qt.ItemDataRole.CheckStateRole) is not None:
        return "1" if item.checkState() == Qt.CheckState.Checked else "0"
    return item.text().strip()


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


def centered_checkbox(checked: bool, changed_callback=None) -> QWidget:
    box = QCheckBox()
    box.setChecked(checked)
    box.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
    panel = QWidget()
    layout = QHBoxLayout(panel)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
    layout.addWidget(box)
    panel._sdpe_checkbox = box
    if changed_callback is not None:
        box.stateChanged.connect(lambda _state: changed_callback())
    return panel


def checkbox_widget_checked(widget: QWidget | None) -> bool | None:
    if isinstance(widget, QCheckBox):
        return widget.isChecked()
    box = getattr(widget, "_sdpe_checkbox", None) if widget is not None else None
    if isinstance(box, QCheckBox):
        return box.isChecked()
    return None


def set_bool_item(table: QTableWidget, row: int, col: int, checked: bool) -> None:
    item = QTableWidgetItem("")
    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
    table.setItem(row, col, item)
    table.setCellWidget(row, col, centered_checkbox(checked, lambda t=table: notify_table_changed(t)))


def table_checked(table: QTableWidget, row: int, col: int, default: bool = False) -> bool:
    checked = checkbox_widget_checked(table.cellWidget(row, col))
    if checked is not None:
        return checked
    item = table.item(row, col)
    if item is None:
        return default
    return item.checkState() == Qt.CheckState.Checked


def set_table_text(table: QTableWidget, row: int, col: int, value: Any) -> None:
    widget = table.cellWidget(row, col)
    text = "" if value is None else str(value)
    if isinstance(widget, QComboBox):
        widget.setCurrentText(text)
    else:
        set_item(table, row, col, text)


def edit_table_cell_multiline(table: QTableWidget, row: int, col: int, parent: QWidget, title: str) -> None:
    text = edit_multiline(parent, title, item_text(table, row, col))
    if text is not None:
        set_item(table, row, col, text)


def setup_feature_macro_table(table: QTableWidget) -> None:
    set_table_headers(table, ["Macro", "En", "Wk", "Value", "Description"], QHeaderView.ResizeMode.Interactive)
    table.setColumnWidth(1, 46)
    table.setColumnWidth(2, 46)


def setup_option_macro_table(table: QTableWidget) -> None:
    set_table_headers(
        table,
        ["Macro", "En", "Wk", "Value", "Options Preset", "Options CSV", "Description"],
        QHeaderView.ResizeMode.Interactive,
    )
    table.setColumnWidth(1, 46)
    table.setColumnWidth(2, 46)


def setup_option_set_table(table: QTableWidget) -> None:
    set_table_headers(table, ["Name", "Options CSV", "Description"], QHeaderView.ResizeMode.Interactive)


def load_feature_macro_table(table: QTableWidget, items: list[dict[str, Any]]) -> None:
    table.setRowCount(0)
    for item in items:
        row = table.rowCount()
        table.insertRow(row)
        set_item(table, row, 0, item.get("macro", ""))
        set_bool_item(table, row, 1, bool(item.get("enabled", True)))
        set_bool_item(table, row, 2, bool(item.get("weak", False)))
        set_item(table, row, 3, item.get("value", ""))
        set_item(table, row, 4, item.get("description", ""))
    fit_table_columns(table)
    table.setColumnWidth(1, 46)
    table.setColumnWidth(2, 46)


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
        set_bool_item(table, row, 1, bool(item.get("enabled", True)))
        set_bool_item(table, row, 2, bool(item.get("weak", False)))
        set_combo(table, row, 3, options or [str(item.get("value", ""))], item.get("value", ""))
        set_item(table, row, 4, preset)
        set_item(table, row, 5, ", ".join(options))
        set_item(table, row, 6, item.get("description", ""))
    fit_table_columns(table)
    table.setColumnWidth(1, 46)
    table.setColumnWidth(2, 46)


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
                    "enabled": table_checked(table, row, 1, True),
                    "weak": table_checked(table, row, 2, False),
                    "value": item_text(table, row, 3),
                    "description": item_text(table, row, 4),
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
                    "enabled": table_checked(table, row, 1, True),
                    "weak": table_checked(table, row, 2, False),
                    "value": item_text(table, row, 3),
                    "options_preset": item_text(table, row, 4),
                    "options": [item.strip() for item in item_text(table, row, 5).split(",") if item.strip()],
                    "description": item_text(table, row, 6),
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
    combo = SDPEComboBox()
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
        self.code_search = QLineEdit()
        self.code_search.setPlaceholderText("Find macro or text in Code")
        self.code_search.returnPressed.connect(lambda: self.find_in_preview(self.code_panel, self.code_search, forward=True))
        self.matlab_panel = QTextEdit()
        self.matlab_panel.setReadOnly(True)
        self.matlab_panel.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        self.matlab_search = QLineEdit()
        self.matlab_search.setPlaceholderText("Find macro or text in MATLAB Init")
        self.matlab_search.returnPressed.connect(lambda: self.find_in_preview(self.matlab_panel, self.matlab_search, forward=True))

        self.items_panel = QWidget()
        left_layout = QVBoxLayout(self.items_panel)
        left_layout.addWidget(self.search)
        left_layout.addWidget(self.list_widget)
        self.code_preview_panel = self.create_preview_panel(
            "Code Search",
            self.code_search,
            self.code_panel,
            lambda: self.find_in_preview(self.code_panel, self.code_search, forward=False),
            lambda: self.find_in_preview(self.code_panel, self.code_search, forward=True),
        )
        self.matlab_preview_panel = self.create_preview_panel(
            "MATLAB Search",
            self.matlab_search,
            self.matlab_panel,
            lambda: self.find_in_preview(self.matlab_panel, self.matlab_search, forward=False),
            lambda: self.find_in_preview(self.matlab_panel, self.matlab_search, forward=True),
        )

        self.splitter = QSplitter()
        self.splitter.addWidget(self.items_panel)
        self.splitter.addWidget(self.form_panel)
        self.splitter.addWidget(self.professional_panel)
        self.splitter.addWidget(self.code_preview_panel)
        self.splitter.addWidget(self.matlab_preview_panel)
        self.splitter.setSizes([240, 760, 420, 500, 500])

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
        self.show_code.toggled.connect(self.code_preview_panel.setVisible)
        self.show_code.toggled.connect(lambda _checked: self.update_panel_sizes())
        self.show_matlab = QCheckBox("MATLAB Init")
        self.show_matlab.setChecked(False)
        self.show_matlab.toggled.connect(self.matlab_preview_panel.setVisible)
        self.show_matlab.toggled.connect(lambda _checked: self.update_panel_sizes())
        self.professional_panel.setVisible(False)
        self.code_preview_panel.setVisible(False)
        self.matlab_preview_panel.setVisible(False)
        if not self.has_code:
            self.show_code.setEnabled(False)
            self.show_matlab.setEnabled(False)

        toolbar = QToolBar(title)
        toolbar.addWidget(QLabel(title))
        toolbar.addSeparator()
        toolbar.addWidget(self.show_items)
        toolbar.addWidget(self.show_basic)
        toolbar.addWidget(self.show_professional)
        toolbar.addWidget(self.show_code)
        toolbar.addWidget(self.show_matlab)

        layout = QVBoxLayout(self)
        layout.addWidget(toolbar)
        layout.addWidget(self.splitter)
        self.save_shortcut = QShortcut(QKeySequence.StandardKey.Save, self)
        self.save_shortcut.activated.connect(self.save_current)
        self.save_all_shortcut = QShortcut(QKeySequence("Ctrl+Shift+S"), self)
        self.save_all_shortcut.activated.connect(self.save_all)
        self.undo_shortcut = QShortcut(QKeySequence.StandardKey.Undo, self)
        self.undo_shortcut.activated.connect(self.undo_current_change)

    def create_preview_panel(self, title: str, search: QLineEdit, editor: QTextEdit, previous, next_) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        row = QHBoxLayout()
        row.addWidget(QLabel(title))
        row.addWidget(search)
        prev_button = QPushButton("Prev")
        prev_button.clicked.connect(previous)
        next_button = QPushButton("Next")
        next_button.clicked.connect(next_)
        row.addWidget(prev_button)
        row.addWidget(next_button)
        layout.addLayout(row)
        layout.addWidget(editor)
        return panel

    def find_in_preview(self, editor: QTextEdit, search: QLineEdit, forward: bool = True) -> bool:
        text = search.text().strip()
        if not text:
            return False
        flags = QTextDocument.FindFlag(0)
        if not forward:
            flags |= QTextDocument.FindFlag.FindBackward
        if editor.find(text, flags):
            self.message("Find", text)
            return True
        cursor = editor.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.Start if forward else QTextCursor.MoveOperation.End)
        editor.setTextCursor(cursor)
        if editor.find(text, flags):
            self.message("Find", f"Wrapped: {text}")
            return True
        self.message("Find", f"Not found: {text}")
        return False

    def jump_preview_to_text(self, text: str, reveal: bool = False) -> None:
        text = text.strip()
        if not text:
            return
        if reveal:
            if self.has_code:
                self.show_code.setChecked(True)
                self.show_matlab.setChecked(True)
        self.code_search.setText(text)
        self.matlab_search.setText(text)
        jumped = False
        if self.show_code.isChecked():
            cursor = self.code_panel.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.Start)
            self.code_panel.setTextCursor(cursor)
            jumped = self.find_in_preview(self.code_panel, self.code_search, forward=True) or jumped
        if self.show_matlab.isChecked():
            cursor = self.matlab_panel.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.Start)
            self.matlab_panel.setTextCursor(cursor)
            jumped = self.find_in_preview(self.matlab_panel, self.matlab_search, forward=True) or jumped
        if not jumped and (self.show_code.isChecked() or self.show_matlab.isChecked()):
            self.message("Jump", f"Not found: {text}")

    def set_parameter_header_tooltips(self, tree: QTreeWidget) -> None:
        for col in range(tree.columnCount()):
            header = tree.headerItem().text(col)
            if header == "En":
                tree.headerItem().setToolTip(col, "Enable: checked macros are emitted normally; unchecked macros are commented out.")
            elif header == "Wk":
                tree.headerItem().setToolTip(col, "Weak macro: checked macros are wrapped by #ifndef / #define / #endif.")

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

    def set_matlab_text(self, text: str, reveal: bool = True) -> None:
        self.matlab_panel.setPlainText(text)
        if self.has_code and reveal:
            self.show_matlab.setChecked(True)
            self.update_panel_sizes()

    def update_panel_sizes(self) -> None:
        sizes = [
            240 if self.show_items.isChecked() else 0,
            760 if self.show_basic.isChecked() else 0,
            460 if self.show_professional.isChecked() else 0,
            560 if self.show_code.isChecked() and self.has_code else 0,
            560 if self.show_matlab.isChecked() and self.has_code else 0,
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
        set_bool_item(table, row, 1, True)
        set_bool_item(table, row, 2, False)
        if macro_type == "option":
            set_combo(table, row, 3, ["1", "2", "3", "4", "5"], "1")
            set_item(table, row, 5, "1, 2, 3, 4, 5")

    def add_option_set(self) -> None:
        row = self.option_sets.rowCount()
        self.option_sets.insertRow(row)

    def on_template_macro_double_clicked(self, row: int, col: int) -> None:
        table = self.sender()
        if isinstance(table, QTableWidget) and (col == table.columnCount() - 1 or col == 5):
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
        self.params.setHeaderLabels(["Parameter", "En", "Wk", "Value", "Unit", "Description"])
        self.set_parameter_header_tooltips(self.params)
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
        fit_tree_key_columns(self.params, description_col=5, interactive=True)
        self.params.setColumnWidth(1, 46)
        self.params.setColumnWidth(2, 46)
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
            modifiers = self.entity_parameter_modifier(data, name, name)
            item = QTreeWidgetItem([name, "", "", display_parameter_value(value), "", ""])
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable)
            item.setData(0, Qt.ItemDataRole.UserRole, "parameter")
            group_item.addChild(item)
            set_tree_check(item, 1, modifiers.get("enabled", True))
            set_tree_check(item, 2, modifiers.get("weak", False))
        self.params.expandAll()
        fit_tree_key_columns(self.params, description_col=5, interactive=True)
        self.params.setColumnWidth(1, 46)
        self.params.setColumnWidth(2, 46)

    def set_parameter_header_tooltips(self, tree: QTreeWidget) -> None:
        for col in range(tree.columnCount()):
            header = tree.headerItem().text(col)
            if header == "En":
                tree.headerItem().setToolTip(col, "Enable: checked parameter macros are emitted normally; unchecked macros are commented out.")
            elif header == "Wk":
                tree.headerItem().setToolTip(col, "Weak macro: checked parameter macros are wrapped by #ifndef / #define / #endif.")

    def entity_parameter_modifier(self, data: dict[str, Any], name: str, c_name: str) -> dict[str, bool]:
        modifiers = data.get("parameter_macros", {})
        item = modifiers.get(name, modifiers.get(c_name, {})) if isinstance(modifiers, dict) else {}
        return {
            "enabled": bool(item.get("enabled", True)) if isinstance(item, dict) else True,
            "weak": bool(item.get("weak", False)) if isinstance(item, dict) else False,
        }

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
        modifiers = self.entity_parameter_modifier(data, name, pspec.c_name)
        item = QTreeWidgetItem([name, "", "", value, pspec.unit, pspec.description])
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable)
        item.setData(0, Qt.ItemDataRole.UserRole, "parameter")
        item.setToolTip(0, f"{pspec.description}\nUnit: {pspec.unit}")
        item.setToolTip(3, f"{pspec.description}\nUnit: {pspec.unit}")
        group.addChild(item)
        set_tree_check(item, 1, modifiers["enabled"])
        set_tree_check(item, 2, modifiers["weak"])
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
                "parameter_macros": self._table_parameter_macros(),
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
                value = tree_cell_text(self.params, item, 3)
                if not name or not value.strip():
                    continue
                parsed = parse_parameter_value(value)
                if parsed == "":
                    continue
                if name in schema.parameters:
                    validate_value_for_format(name, parsed, schema.parameters[name].value_format)
                params[name] = parsed
        return params

    def _table_parameter_macros(self) -> dict[str, dict[str, bool]]:
        modifiers: dict[str, dict[str, bool]] = {}
        for group_index in range(self.params.topLevelItemCount()):
            group = self.params.topLevelItem(group_index)
            for child_index in range(group.childCount()):
                item = group.child(child_index)
                if item.data(0, Qt.ItemDataRole.UserRole) != "parameter":
                    continue
                name = tree_cell_text(self.params, item, 0)
                if not name:
                    continue
                modifiers[name] = {
                    "enabled": tree_checked(item, 1, True),
                    "weak": tree_checked(item, 2, False),
                }
        return modifiers

    def select_parameter_symbol(self, item: QTreeWidgetItem) -> None:
        symbols = self.window.symbols_for_subcomponents(self.current_id)
        selected = choose_item(self, "Select Sub Component Symbol", symbols)
        if selected:
            item.setText(3, f"${{{selected}}}")

    def on_entity_param_double_clicked(self, item: QTreeWidgetItem, col: int) -> None:
        if item.data(0, Qt.ItemDataRole.UserRole) != "parameter":
            return
        if col == 3:
            self.select_parameter_symbol(item)
        elif col == 5:
            text = edit_multiline(self, "Parameter Description", tree_cell_text(self.params, item, 5))
            if text is not None:
                item.setText(5, text)

    def add_entity_component(self) -> None:
        row = self.components.rowCount()
        self.components.insertRow(row)
        set_combo(self.components, row, 1, ["entity", "inline"], "entity")

    def add_macro_row(self, table: QTableWidget, macro_type: str) -> None:
        row = table.rowCount()
        table.insertRow(row)
        set_bool_item(table, row, 1, True)
        set_bool_item(table, row, 2, False)
        if macro_type == "option":
            set_combo(table, row, 3, ["1", "2", "3", "4", "5"], "1")
            set_item(table, row, 5, "1, 2, 3, 4, 5")

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
        if isinstance(table, QTableWidget) and (col == table.columnCount() - 1 or col == 5):
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
        return self.window.generator(Path(self.out_dir.text()))

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
        self.macro_prefix_edit = QLineEdit()
        self.path_edit = QLineEdit()
        self.path_edit.setReadOnly(True)
        self.name_edit = QLineEdit()
        self.suite_edit = QLineEdit()
        self.version_edit = QLineEdit()
        self.updated_edit = QLineEdit()
        self.updated_edit.setReadOnly(True)
        self.header_edit = QLineEdit()
        self.matlab_file_edit = QLineEdit()
        self.matlab_file_edit.setReadOnly(True)
        self.description_edit = QTextEdit()
        self.description_edit.setFixedHeight(92)
        self.prefix_code = QTextEdit()
        self.tail_code = QTextEdit()

        self.hardware_view = QComboBox()
        self.hardware_view.addItems(["Tree", "Table"])
        self.hardware_tree = QTreeWidget()
        self.hardware_tree.setHeaderLabels(["Hardware", "Description"])
        fit_tree_key_columns(self.hardware_tree, description_col=1)
        install_tree_status_descriptions(self.hardware_tree, description_col=1)
        self.hardware_tree.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.hardware_tree.customContextMenuRequested.connect(self.show_hardware_context_menu)
        self.hardware_tree.itemDoubleClicked.connect(self.on_hardware_tree_double_clicked)
        self.hardware = QTableWidget()
        set_table_headers(self.hardware, ["Entity", "Name", "Template", "Category", "Description"], QHeaderView.ResizeMode.Interactive)
        self.hardware.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.hardware.customContextMenuRequested.connect(self.show_hardware_table_context_menu)
        self.hardware.cellDoubleClicked.connect(self.on_hardware_cell_double_clicked)

        self.requirements = QTreeWidget()
        self.requirements.setHeaderLabels(["Name", "Macro", "En", "Wk", "Binding Type", "Binding Value", "Description"])
        self.set_parameter_header_tooltips(self.requirements)
        self.requirements.setAlternatingRowColors(True)
        self.requirements.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.requirements.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.requirements.setItemDelegate(ValidationBorderDelegate(self.requirements))
        fit_tree_key_columns(self.requirements, description_col=6, interactive=True)
        self.requirements.setColumnWidth(2, 46)
        self.requirements.setColumnWidth(3, 46)
        install_tree_status_descriptions(self.requirements, description_col=6)
        self.requirements.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.requirements.customContextMenuRequested.connect(self.show_requirement_context_menu)
        self.requirements.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
        self.requirements.setDefaultDropAction(Qt.DropAction.MoveAction)
        self.requirements.itemChanged.connect(lambda _item, _col: self.mark_current_dirty())
        self.requirements.itemDoubleClicked.connect(self.on_requirement_cell_double_clicked)
        self.requirements.currentItemChanged.connect(lambda current, _previous: self.jump_from_requirement_item(current))
        self.requirements.model().rowsMoved.connect(lambda *_args: self.after_requirement_tree_changed())
        self.requirements.model().rowsInserted.connect(lambda *_args: self.after_requirement_tree_changed())
        self.requirements.model().rowsRemoved.connect(lambda *_args: self.after_requirement_tree_changed())
        self.requirements_copy_shortcut = QShortcut(QKeySequence.StandardKey.Copy, self.requirements)
        self.requirements_copy_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        self.requirements_copy_shortcut.activated.connect(self.copy_selected_requirements)
        self.requirements_cut_shortcut = QShortcut(QKeySequence.StandardKey.Cut, self.requirements)
        self.requirements_cut_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        self.requirements_cut_shortcut.activated.connect(self.cut_selected_requirements)
        self.requirements_paste_shortcut = QShortcut(QKeySequence.StandardKey.Paste, self.requirements)
        self.requirements_paste_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        self.requirements_paste_shortcut.activated.connect(self.paste_requirements)
        self.requirements_delete_shortcut = QShortcut(QKeySequence("Del"), self.requirements)
        self.requirements_delete_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        self.requirements_delete_shortcut.activated.connect(self.remove_requirement_item)
        self.feature_macros = QTreeWidget()
        self.feature_macros.setHeaderLabels(["Macro", "En", "Wk", "Value", "Description"])
        self.set_macro_header_tooltips(self.feature_macros)
        self.feature_macros.setItemDelegate(ValidationBorderDelegate(self.feature_macros))
        self.setup_macro_tree(self.feature_macros, description_col=4)
        self.feature_macros.itemDoubleClicked.connect(self.on_macro_tree_double_clicked)
        self.feature_macros.currentItemChanged.connect(lambda current, _previous: self.jump_from_macro_item(self.feature_macros, current))
        self.feature_macros.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.feature_macros.customContextMenuRequested.connect(lambda pos: self.show_macro_tree_context_menu(self.feature_macros, "feature", pos))
        self.install_macro_tree_shortcuts(self.feature_macros, "feature")
        self.enum_macros = QTreeWidget()
        self.enum_macros.setHeaderLabels(["Macro", "En", "Wk", "Value", "Options Preset", "Options CSV", "Description"])
        self.set_macro_header_tooltips(self.enum_macros)
        self.enum_macros.setItemDelegate(ValidationBorderDelegate(self.enum_macros))
        self.setup_macro_tree(self.enum_macros, description_col=6)
        self.enum_macros.itemDoubleClicked.connect(self.on_macro_tree_double_clicked)
        self.enum_macros.currentItemChanged.connect(lambda current, _previous: self.jump_from_macro_item(self.enum_macros, current))
        self.enum_macros.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.enum_macros.customContextMenuRequested.connect(lambda pos: self.show_macro_tree_context_menu(self.enum_macros, "option", pos))
        self.install_macro_tree_shortcuts(self.enum_macros, "option")

        for widget in [self.id_edit, self.macro_prefix_edit, self.name_edit, self.suite_edit, self.version_edit, self.header_edit]:
            widget.textChanged.connect(self.mark_current_dirty)
        self.header_edit.textChanged.connect(lambda _text: self.update_matlab_script_name())
        for widget in [self.description_edit, self.prefix_code, self.tail_code]:
            widget.textChanged.connect(self.mark_current_dirty)
        self.hardware.cellChanged.connect(lambda _row, _col: self.mark_current_dirty())
        self.feature_macros.itemChanged.connect(lambda _item, _col: self.mark_current_dirty())
        self.enum_macros.itemChanged.connect(lambda item, col: (self.mark_current_dirty(), self.on_enum_macro_item_changed(item, col)))
        self.hardware.cellChanged.connect(lambda _row, _col: self.refresh_hardware_status())
        self.hardware_view.currentTextChanged.connect(self.update_hardware_view)

        tabs = QTabWidget()
        basic_tab = QWidget()
        basic_form = QFormLayout(basic_tab)
        basic_form.addRow("File Path", self.path_edit)
        basic_form.addRow("Project ID", self.id_edit)
        basic_form.addRow("Macro Prefix", self.macro_prefix_edit)
        basic_form.addRow("Name", self.name_edit)
        basic_form.addRow("Suite", self.suite_edit)
        basic_form.addRow("Version", self.version_edit)
        basic_form.addRow("Last Updated", self.updated_edit)
        basic_form.addRow("C Header File", self.header_edit)
        basic_form.addRow("MATLAB Init Script", self.matlab_file_edit)
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
        add_req_group = QPushButton("Add group")
        add_req_group.clicked.connect(self.add_requirement_group)
        add_req = QPushButton("Add requirement")
        add_req.clicked.connect(self.add_requirement)
        del_req = QPushButton("Remove requirement")
        del_req.clicked.connect(self.remove_requirement_item)
        req_layout.addWidget(self.requirements)
        req_layout.addLayout(row_buttons([add_req_group, add_req, del_req]))

        macro_tab = QWidget()
        macro_layout = QVBoxLayout(macro_tab)
        add_feature_group = QPushButton("Add group")
        add_feature_group.clicked.connect(lambda: self.add_macro_group(self.feature_macros, "feature"))
        add_feature = QPushButton("Add selection macro")
        add_feature.clicked.connect(self.add_feature_macro)
        del_feature = QPushButton("Remove selection macro")
        del_feature.clicked.connect(lambda: self.remove_macro_items(self.feature_macros))
        add_enum_group = QPushButton("Add group")
        add_enum_group.clicked.connect(lambda: self.add_macro_group(self.enum_macros, "option"))
        add_enum = QPushButton("Add option macro")
        add_enum.clicked.connect(self.add_enum_macro)
        del_enum = QPushButton("Remove option macro")
        del_enum.clicked.connect(lambda: self.remove_macro_items(self.enum_macros))
        macro_layout.addWidget(QLabel("Selection macros"))
        macro_layout.addWidget(self.feature_macros)
        macro_layout.addLayout(row_buttons([add_feature_group, add_feature, del_feature]))
        macro_layout.addWidget(QLabel("Option macros"))
        macro_layout.addWidget(self.enum_macros)
        macro_layout.addLayout(row_buttons([add_enum_group, add_enum, del_enum]))

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
        generate_header = QPushButton("Generate project header")
        generate_header.clicked.connect(self.generate_project_header)
        validate_macros = QPushButton("Validate Macros")
        validate_macros.clicked.connect(self.validate_project_macros)
        matlab = QPushButton("Generate MATLAB Init Script")
        matlab.clicked.connect(self.generate_matlab_init_script)
        save = QPushButton("Save project")
        save.clicked.connect(self.save_current)
        self.form_layout.addWidget(tabs)
        self.form_layout.addLayout(row_buttons([preview, generate_header, validate_macros, matlab, save]))
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
        self.macro_prefix_edit.setText(data.get("macro_prefix", ""))
        self.name_edit.setText(data.get("display_name", ""))
        self.suite_edit.setText(data.get("suite", ""))
        self.version_edit.setText(data.get("version", "0.1.0"))
        self.updated_edit.setText(data.get("updated_at", ""))
        self.header_edit.setText(data.get("output_header", "sdpe_project_bindings.h"))
        self.update_matlab_script_name()
        self.description_edit.setPlainText(data.get("description", ""))
        sections = data.get("code_sections", {})
        self.prefix_code.setPlainText(sections.get("after_extern_open", ""))
        self.tail_code.setPlainText(sections.get("before_footer", ""))
        self.load_hardware(data)
        self.load_requirements(data)
        self.load_macro_tables(data)
        self.set_professional_text(pretty_json(data))
        try:
            self.set_code_text(self.window.generator().render_project_header(data), reveal=False)
        except Exception:
            self.code_panel.clear()
        try:
            self.set_matlab_text(self.window.generator().render_project_matlab_script(data), reveal=False)
        except Exception:
            self.matlab_panel.clear()
        self.loading = False
        if not self.restoring_undo:
            self.reset_undo_history(self.collect_current_data() or data)

    def update_matlab_script_name(self) -> None:
        header_name = Path(self.header_edit.text().strip() or "sdpe_project_bindings.h")
        self.matlab_file_edit.setText(f"{header_name.stem}_matlab_init.m")

    def load_hardware(self, data: dict[str, Any]) -> None:
        self.hardware.setRowCount(0)
        for hw in project_hardware_items(data):
            entity_id = project_hardware_entity_id(hw)
            if not entity_id:
                continue
            self.add_hardware_row(entity_id, inherited=False)
        for entity_id in self.inherited_hardware_entity_ids(data):
            if entity_id and not self.hardware_entity_in_table(entity_id):
                self.add_hardware_row(entity_id, inherited=True)
        self.refresh_hardware_status()
        fit_table_columns(self.hardware)

    def add_hardware_row(self, entity_id: str, inherited: bool = False) -> None:
        row = self.hardware.rowCount()
        self.hardware.insertRow(row)
        set_item(self.hardware, row, 0, entity_id)
        item = self.hardware.item(row, 0)
        if item is not None and inherited:
            item.setData(Qt.ItemDataRole.UserRole + 10, "inherited")
            item.setToolTip("Inherited from a related SDPE requirement. It will not be saved into this project file.")
        self.populate_hardware_info(row, entity_id)
        if inherited:
            for col in range(self.hardware.columnCount()):
                cell = self.hardware.item(row, col)
                if cell is not None:
                    cell.setFlags(cell.flags() & ~Qt.ItemFlag.ItemIsEditable)

    def hardware_entity_in_table(self, entity_id: str) -> bool:
        for row in range(self.hardware.rowCount()):
            if item_text(self.hardware, row, 0) == entity_id:
                return True
        return False

    def inherited_hardware_entity_ids(self, data: dict[str, Any]) -> list[str]:
        if not self.current_id:
            return []
        current_path = self.window.project_path(self.current_id).resolve()
        if "sdpe_general" in {part.lower() for part in current_path.parts}:
            return []
        suite = data.get("suite", "")
        ids: list[str] = []
        for path in self.window.project_paths():
            path = path.resolve()
            if path == current_path or "sdpe_general" not in {part.lower() for part in path.parts}:
                continue
            try:
                related = read_json(path)
            except Exception:
                continue
            if suite and related.get("suite") != suite:
                continue
            ids.extend(project_hardware_entity_id(hw) for hw in project_hardware_items(related))
        return [item for item in dict.fromkeys(ids) if item]

    def load_requirements(self, data: dict[str, Any]) -> None:
        self.requirements.clear()
        requirements = list(data.get("requirements", []))
        by_name = {req.get("role", req.get("macro", "")): req for req in requirements if req.get("role") or req.get("macro")}
        used: set[str] = set()
        groups = data.get("requirement_groups", [])
        if not groups:
            groups = [{"name": "Requirements", "requirements": [req.get("role", req.get("macro", "")) for req in requirements]}]
        for group in groups:
            group_item = self.create_requirement_group_item(group.get("name", "Requirements"))
            self.requirements.addTopLevelItem(group_item)
            for name in group.get("requirements", []):
                if name in by_name:
                    self.add_requirement_item(group_item, by_name[name])
                    used.add(name)
        missing = [req for req in requirements if req.get("role", req.get("macro", "")) not in used]
        if missing:
            group_item = self.create_requirement_group_item("Ungrouped")
            self.requirements.addTopLevelItem(group_item)
            for req in missing:
                self.add_requirement_item(group_item, req)
        self.requirements.expandAll()
        fit_tree_key_columns(self.requirements, description_col=6, interactive=True)
        self.requirements.setColumnWidth(2, 46)
        self.requirements.setColumnWidth(3, 46)

    def load_macro_tables(self, data: dict[str, Any]) -> None:
        self.load_feature_macro_tree(data.get("feature_macros", []))
        self.load_option_macro_tree(data.get("option_macros", []))

    def macro_group_map(self, tree: QTreeWidget, default_group: str) -> dict[str, QTreeWidgetItem]:
        groups: dict[str, QTreeWidgetItem] = {}
        for index in range(tree.topLevelItemCount()):
            group = tree.topLevelItem(index)
            groups[group.text(0).strip() or default_group] = group
        return groups

    def load_feature_macro_tree(self, items: list[dict[str, Any]]) -> None:
        self.feature_macros.clear()
        groups: dict[str, QTreeWidgetItem] = {}
        for item in items:
            group_name = item.get("group") or "Selection Macros"
            group = groups.get(group_name)
            if group is None:
                group = self.create_macro_group_item(group_name)
                self.feature_macros.addTopLevelItem(group)
                groups[group_name] = group
            self.add_macro_item(self.feature_macros, "feature", group, item)
        if not groups:
            self.feature_macros.addTopLevelItem(self.create_macro_group_item("Selection Macros"))
        self.feature_macros.expandAll()
        self.feature_macros.clearSelection()
        fit_tree_key_columns(self.feature_macros, description_col=4, interactive=True)
        self.feature_macros.setColumnWidth(1, 46)
        self.feature_macros.setColumnWidth(2, 46)

    def load_option_macro_tree(self, items: list[dict[str, Any]]) -> None:
        self.enum_macros.clear()
        groups: dict[str, QTreeWidgetItem] = {}
        for item in items:
            group_name = item.get("group") or "Option Macros"
            group = groups.get(group_name)
            if group is None:
                group = self.create_macro_group_item(group_name)
                self.enum_macros.addTopLevelItem(group)
                groups[group_name] = group
            self.add_macro_item(self.enum_macros, "option", group, item)
        if not groups:
            self.enum_macros.addTopLevelItem(self.create_macro_group_item("Option Macros"))
        self.enum_macros.expandAll()
        self.enum_macros.clearSelection()
        fit_tree_key_columns(self.enum_macros, description_col=6, interactive=True)
        self.enum_macros.setColumnWidth(1, 46)
        self.enum_macros.setColumnWidth(2, 46)

    def collect_feature_macro_tree(self) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for group_index in range(self.feature_macros.topLevelItemCount()):
            group = self.feature_macros.topLevelItem(group_index)
            for child_index in range(group.childCount()):
                item = group.child(child_index)
                data = self.macro_item_to_data(self.feature_macros, item, "feature")
                if data.get("macro"):
                    rows.append(data)
        return rows

    def collect_option_macro_tree(self) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for group_index in range(self.enum_macros.topLevelItemCount()):
            group = self.enum_macros.topLevelItem(group_index)
            for child_index in range(group.childCount()):
                item = group.child(child_index)
                data = self.macro_item_to_data(self.enum_macros, item, "option")
                if data.get("macro"):
                    rows.append(data)
        return rows

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
                "macro_prefix": self.macro_prefix_edit.text().strip(),
                "display_name": self.name_edit.text().strip(),
                "description": self.description_edit.toPlainText().strip(),
                "suite": self.suite_edit.text().strip(),
                "version": self.version_edit.text().strip(),
                "updated_at": self.updated_edit.text().strip() or date.today().isoformat(),
                "output_header": self.header_edit.text().strip(),
                "hardware": self._table_hardware(),
                "requirements": self._table_requirements(),
                "requirement_groups": self._requirement_groups(),
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
            if is_inherited_hardware_row(self.hardware, row):
                continue
            entity = item_text(self.hardware, row, 0)
            if entity:
                rows.append({"entity": entity})
        return rows

    def _display_hardware(self) -> list[dict[str, str]]:
        rows = []
        for row in range(self.hardware.rowCount()):
            entity = item_text(self.hardware, row, 0)
            if entity:
                rows.append({"entity": entity, "inherited": "1" if is_inherited_hardware_row(self.hardware, row) else ""})
        return rows

    def _table_requirements(self) -> list[dict[str, Any]]:
        rows = []
        for group_index in range(self.requirements.topLevelItemCount()):
            group = self.requirements.topLevelItem(group_index)
            for child_index in range(group.childCount()):
                item = group.child(child_index)
                if item.data(0, Qt.ItemDataRole.UserRole) != "requirement":
                    continue
                macro = tree_cell_text(self.requirements, item, 1)
                if macro:
                    rows.append(self.requirement_item_to_data(item))
        return rows

    def _requirement_groups(self) -> list[dict[str, Any]]:
        groups = []
        for group_index in range(self.requirements.topLevelItemCount()):
            group = self.requirements.topLevelItem(group_index)
            names = [
                tree_cell_text(self.requirements, group.child(child_index), 0)
                for child_index in range(group.childCount())
                if tree_cell_text(self.requirements, group.child(child_index), 0)
            ]
            groups.append({"name": group.text(0).strip() or "Requirements", "requirements": names})
        return groups

    def _table_feature_macros(self) -> list[dict[str, Any]]:
        return self.collect_feature_macro_tree()

    def _table_option_macros(self) -> list[dict[str, Any]]:
        return self.collect_option_macro_tree()

    def add_feature_macro(self) -> None:
        self.add_macro_item(self.feature_macros, "feature")

    def set_macro_header_tooltips(self, tree: QTreeWidget) -> None:
        for col in range(tree.columnCount()):
            header = tree.headerItem().text(col)
            if header == "En":
                tree.headerItem().setToolTip(col, "Enable: checked macros are emitted normally; unchecked macros are commented out.")
            elif header == "Wk":
                tree.headerItem().setToolTip(col, "Weak macro: checked macros are wrapped by #ifndef / #define / #endif.")

    def setup_macro_tree(self, tree: QTreeWidget, description_col: int) -> None:
        tree.setAlternatingRowColors(True)
        tree.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        tree.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        tree.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
        tree.setDefaultDropAction(Qt.DropAction.MoveAction)
        tree.model().rowsMoved.connect(lambda *_args, t=tree: self.after_macro_tree_changed(t))
        tree.model().rowsInserted.connect(lambda *_args, t=tree: self.after_macro_tree_changed(t))
        tree.model().rowsRemoved.connect(lambda *_args, t=tree: self.after_macro_tree_changed(t))
        fit_tree_key_columns(tree, description_col=description_col, interactive=True)
        install_tree_status_descriptions(tree, description_col=description_col)

    def create_macro_group_item(self, name: str) -> QTreeWidgetItem:
        item = QTreeWidgetItem([name])
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable | Qt.ItemFlag.ItemIsDropEnabled)
        item.setData(0, Qt.ItemDataRole.UserRole, "group")
        return item

    def current_macro_group(self, tree: QTreeWidget, default_group: str) -> QTreeWidgetItem:
        item = tree.currentItem()
        if item is not None:
            if item.data(0, Qt.ItemDataRole.UserRole) == "group":
                return item
            if item.parent() is not None:
                return item.parent()
        if tree.topLevelItemCount() == 0:
            tree.addTopLevelItem(self.create_macro_group_item(default_group))
        return tree.topLevelItem(0)

    def add_macro_item(self, tree: QTreeWidget, macro_type: str, group: QTreeWidgetItem | None = None, data: dict[str, Any] | None = None) -> QTreeWidgetItem:
        data = data or {}
        default_group = "Option Macros" if macro_type == "option" else "Selection Macros"
        parent = group or self.current_macro_group(tree, default_group)
        parent.setExpanded(True)
        if macro_type == "option":
            item = QTreeWidgetItem([
                data.get("macro", ""),
                "",
                "",
                str(data.get("value", "")),
                data.get("options_preset", data.get("preset", "")),
                ", ".join(str(v) for v in data.get("options", [])),
                data.get("description", ""),
            ])
            item.setData(0, Qt.ItemDataRole.UserRole, "option_macro")
            parent.addChild(item)
            set_tree_check(item, 1, bool(data.get("enabled", True)))
            set_tree_check(item, 2, bool(data.get("weak", False)))
            options = [item.strip() for item in tree_cell_text(tree, item, 5).split(",") if item.strip()]
            set_tree_combo(tree, item, 3, options or [tree_cell_text(tree, item, 3) or "1"], tree_cell_text(tree, item, 3) or (options[0] if options else "1"))
        else:
            item = QTreeWidgetItem([data.get("macro", ""), "", "", str(data.get("value", "")), data.get("description", "")])
            item.setData(0, Qt.ItemDataRole.UserRole, "feature_macro")
            parent.addChild(item)
            set_tree_check(item, 1, bool(data.get("enabled", True)))
            set_tree_check(item, 2, bool(data.get("weak", False)))
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable | Qt.ItemFlag.ItemIsDragEnabled)
        if not self.loading:
            tree.setCurrentItem(item, 0)
            self.mark_current_dirty()
        return item

    def add_macro_group(self, tree: QTreeWidget, macro_type: str) -> None:
        group_item = self.create_macro_group_item("New Group")
        tree.addTopLevelItem(group_item)
        tree.setCurrentItem(group_item, 0)
        tree.editItem(group_item, 0)
        self.mark_current_dirty()

    def selected_macro_items(self, tree: QTreeWidget) -> list[QTreeWidgetItem]:
        items = [item for item in tree.selectedItems() if item.data(0, Qt.ItemDataRole.UserRole) in {"feature_macro", "option_macro"}]
        current = tree.currentItem()
        if not items and current is not None and current.data(0, Qt.ItemDataRole.UserRole) in {"feature_macro", "option_macro"}:
            items = [current]
        return items

    def iter_macro_items(self, tree: QTreeWidget) -> list[QTreeWidgetItem]:
        items: list[QTreeWidgetItem] = []
        for group_index in range(tree.topLevelItemCount()):
            group = tree.topLevelItem(group_index)
            for child_index in range(group.childCount()):
                item = group.child(child_index)
                if item.data(0, Qt.ItemDataRole.UserRole) in {"feature_macro", "option_macro"}:
                    items.append(item)
        return items

    def remove_macro_items(self, tree: QTreeWidget) -> None:
        items = tree.selectedItems() or ([tree.currentItem()] if tree.currentItem() is not None else [])
        for item in reversed(items):
            parent = item.parent()
            if parent is None:
                index = tree.indexOfTopLevelItem(item)
                tree.takeTopLevelItem(index)
            else:
                parent.removeChild(item)
        if items:
            self.mark_current_dirty()

    def macro_item_to_data(self, tree: QTreeWidget, item: QTreeWidgetItem, macro_type: str) -> dict[str, Any]:
        group = item.parent().text(0) if item.parent() is not None else ("Option Macros" if macro_type == "option" else "Selection Macros")
        if macro_type == "option":
            return {
                "group": group,
                "macro": tree_cell_text(tree, item, 0),
                "enabled": tree_checked(item, 1, True),
                "weak": tree_checked(item, 2, False),
                "value": tree_cell_text(tree, item, 3),
                "options_preset": tree_cell_text(tree, item, 4),
                "options": [part.strip() for part in tree_cell_text(tree, item, 5).split(",") if part.strip()],
                "description": tree_cell_text(tree, item, 6),
            }
        return {
            "group": group,
            "macro": tree_cell_text(tree, item, 0),
            "enabled": tree_checked(item, 1, True),
            "weak": tree_checked(item, 2, False),
            "value": tree_cell_text(tree, item, 3),
            "description": tree_cell_text(tree, item, 4),
        }

    def copy_macro_items(self, tree: QTreeWidget, macro_type: str) -> None:
        rows = [self.macro_item_to_data(tree, item, macro_type) for item in self.selected_macro_items(tree)]
        if rows:
            QApplication.clipboard().setText(pretty_json({"sdpe_clipboard": f"project_{macro_type}_macros_v1", "rows": rows}))

    def cut_macro_items(self, tree: QTreeWidget, macro_type: str) -> None:
        self.copy_macro_items(tree, macro_type)
        self.remove_macro_items(tree)

    def paste_macro_items(self, tree: QTreeWidget, macro_type: str) -> None:
        text = QApplication.clipboard().text().strip()
        if not text:
            return
        try:
            data = json.loads(text)
            rows = data.get("rows", []) if isinstance(data, dict) else []
        except json.JSONDecodeError:
            rows = []
        if not rows:
            return
        groups = self.macro_group_map(tree, "Option Macros" if macro_type == "option" else "Selection Macros")
        for row in rows:
            group_name = row.get("group") or ("Option Macros" if macro_type == "option" else "Selection Macros")
            group = groups.get(group_name)
            if group is None:
                group = self.create_macro_group_item(group_name)
                tree.addTopLevelItem(group)
                groups[group_name] = group
            self.add_macro_item(tree, macro_type, group, row)
        self.mark_current_dirty()

    def show_macro_tree_context_menu(self, tree: QTreeWidget, macro_type: str, pos) -> None:
        item = tree.itemAt(pos)
        if item is not None and not item.isSelected():
            tree.setCurrentItem(item, max(0, tree.indexAt(pos).column()))
        menu = QMenu(self)
        copy_action = menu.addAction("Copy selected rows", lambda: self.copy_macro_items(tree, macro_type))
        copy_action.setEnabled(bool(self.selected_macro_items(tree)))
        cut_action = menu.addAction("Cut selected rows", lambda: self.cut_macro_items(tree, macro_type))
        cut_action.setEnabled(bool(self.selected_macro_items(tree)))
        paste_action = menu.addAction("Paste rows", lambda: self.paste_macro_items(tree, macro_type))
        paste_action.setEnabled(bool(QApplication.clipboard().text().strip()))
        menu.addSeparator()
        menu.addAction("Add group", lambda: self.add_macro_group(tree, macro_type))
        menu.addAction("Add macro", lambda: self.add_macro_item(tree, macro_type))
        remove_action = menu.addAction("Remove selected", lambda: self.remove_macro_items(tree))
        remove_action.setEnabled(item is not None)
        menu.exec(tree.viewport().mapToGlobal(pos))

    def install_macro_tree_shortcuts(self, tree: QTreeWidget, macro_type: str) -> None:
        tree._sdpe_copy_shortcut = QShortcut(QKeySequence.StandardKey.Copy, tree)
        tree._sdpe_copy_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        tree._sdpe_copy_shortcut.activated.connect(lambda t=tree, mt=macro_type: self.copy_macro_items(t, mt))
        tree._sdpe_cut_shortcut = QShortcut(QKeySequence.StandardKey.Cut, tree)
        tree._sdpe_cut_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        tree._sdpe_cut_shortcut.activated.connect(lambda t=tree, mt=macro_type: self.cut_macro_items(t, mt))
        tree._sdpe_paste_shortcut = QShortcut(QKeySequence.StandardKey.Paste, tree)
        tree._sdpe_paste_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        tree._sdpe_paste_shortcut.activated.connect(lambda t=tree, mt=macro_type: self.paste_macro_items(t, mt))
        tree._sdpe_delete_shortcut = QShortcut(QKeySequence("Del"), tree)
        tree._sdpe_delete_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        tree._sdpe_delete_shortcut.activated.connect(lambda t=tree: self.remove_macro_items(t))

    def add_requirement(self) -> None:
        group = self.current_requirement_group()
        item = self.add_requirement_item(group, {"role": "new_requirement", "macro": "NEW_REQUIREMENT", "binding": {"number": "0"}})
        group.setExpanded(True)
        self.requirements.setCurrentItem(item)
        self.mark_current_dirty()

    def add_requirement_group(self) -> None:
        item = self.create_requirement_group_item("New Group")
        self.requirements.addTopLevelItem(item)
        self.requirements.setCurrentItem(item)
        self.requirements.editItem(item, 0)
        self.mark_current_dirty()

    def create_requirement_group_item(self, name: str) -> QTreeWidgetItem:
        item = QTreeWidgetItem([name])
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable | Qt.ItemFlag.ItemIsDropEnabled)
        item.setData(0, Qt.ItemDataRole.UserRole, "group")
        return item

    def add_requirement_item(self, group: QTreeWidgetItem, req: dict[str, Any]) -> QTreeWidgetItem:
        return self.insert_requirement_item(group, group.childCount(), req)

    def insert_requirement_item(self, group: QTreeWidgetItem, index: int, req: dict[str, Any]) -> QTreeWidgetItem:
        btype, bvalue = binding_to_cells(req.get("binding", {}))
        item = QTreeWidgetItem([req.get("role", ""), req.get("macro", ""), "", "", "", bvalue, req.get("description", "")])
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable | Qt.ItemFlag.ItemIsDragEnabled)
        item.setData(0, Qt.ItemDataRole.UserRole, "requirement")
        group.insertChild(max(0, min(index, group.childCount())), item)
        set_tree_check(item, 2, bool(req.get("enabled", True)))
        set_tree_check(item, 3, bool(req.get("weak", False)))
        set_tree_combo(self.requirements, item, 4, binding_type_options(), btype)
        return item

    def current_requirement_group(self) -> QTreeWidgetItem:
        item = self.requirements.currentItem()
        if item is not None:
            if item.data(0, Qt.ItemDataRole.UserRole) == "group":
                return item
            if item.parent() is not None:
                return item.parent()
        if self.requirements.topLevelItemCount() == 0:
            self.add_requirement_group()
        return self.requirements.topLevelItem(0)

    def remove_requirement_item(self) -> None:
        selected = self.selected_requirement_items()
        if selected:
            self.remove_requirement_items(selected)
            return
        item = self.requirements.currentItem()
        if item is None:
            return
        parent = item.parent()
        if parent is None:
            index = self.requirements.indexOfTopLevelItem(item)
            self.requirements.takeTopLevelItem(index)
        else:
            parent.removeChild(item)
        self.mark_current_dirty()

    def selected_requirement_items(self) -> list[QTreeWidgetItem]:
        items = [
            item
            for item in self.requirements.selectedItems()
            if item.data(0, Qt.ItemDataRole.UserRole) == "requirement"
        ]
        current = self.requirements.currentItem()
        if not items and current is not None and current.data(0, Qt.ItemDataRole.UserRole) == "requirement":
            items = [current]
        order = {id(item): index for index, item in enumerate(self.iter_requirement_items())}
        return sorted(dict.fromkeys(items), key=lambda item: order.get(id(item), 10**9))

    def requirement_item_to_data(self, item: QTreeWidgetItem) -> dict[str, Any]:
        return {
            "role": tree_cell_text(self.requirements, item, 0),
            "macro": tree_cell_text(self.requirements, item, 1),
            "enabled": tree_checked(item, 2, True),
            "weak": tree_checked(item, 3, False),
            "binding": cells_to_binding(
                tree_cell_text(self.requirements, item, 4),
                tree_cell_text(self.requirements, item, 5),
            ),
            "description": tree_cell_text(self.requirements, item, 6),
        }

    def copy_selected_requirements(self) -> None:
        rows = [self.requirement_item_to_data(item) for item in self.selected_requirement_items()]
        if not rows:
            return
        QApplication.clipboard().setText(
            pretty_json(
                {
                    "sdpe_clipboard": "project_requirements_v1",
                    "rows": rows,
                }
            )
        )

    def cut_selected_requirements(self) -> None:
        items = self.selected_requirement_items()
        if not items:
            return
        self.copy_selected_requirements()
        self.remove_requirement_items(items)

    def remove_requirement_items(self, items: list[QTreeWidgetItem]) -> None:
        for item in reversed(items):
            parent = item.parent()
            if parent is not None:
                parent.removeChild(item)
        self.mark_current_dirty()

    def parse_requirement_clipboard(self, text: str) -> list[dict[str, Any]]:
        value = text.strip()
        if not value:
            return []
        try:
            data = json.loads(value)
            if isinstance(data, dict) and data.get("sdpe_clipboard") == "project_requirements_v1":
                rows = data.get("rows", [])
                return [row for row in rows if isinstance(row, dict)]
        except json.JSONDecodeError:
            pass
        rows = []
        for line in value.splitlines():
            if not line.strip():
                continue
            cells = line.split("\t")
            if len(cells) >= 7:
                rows.append(
                    {
                        "role": cells[0].strip(),
                        "macro": cells[1].strip(),
                        "enabled": cells[2].strip().lower() not in {"0", "false", "disable", "disabled", "no"},
                        "weak": cells[3].strip().lower() in {"1", "true", "enable", "enabled", "yes"},
                        "binding": cells_to_binding(cells[4], cells[5]),
                        "description": cells[6].strip(),
                    }
                )
                continue
            while len(cells) < 5:
                cells.append("")
            rows.append(
                {
                    "role": cells[0].strip(),
                    "macro": cells[1].strip(),
                    "enabled": True,
                    "weak": False,
                    "binding": cells_to_binding(cells[2], cells[3]),
                    "description": cells[4].strip(),
                }
            )
        return rows

    def requirement_paste_target(self) -> tuple[QTreeWidgetItem, int]:
        item = self.requirements.currentItem()
        if item is not None and item.data(0, Qt.ItemDataRole.UserRole) == "requirement" and item.parent() is not None:
            group = item.parent()
            return group, group.indexOfChild(item) + 1
        group = self.current_requirement_group()
        return group, group.childCount()

    def paste_requirements(self) -> None:
        rows = self.parse_requirement_clipboard(QApplication.clipboard().text())
        if not rows:
            return
        group, index = self.requirement_paste_target()
        first_item: QTreeWidgetItem | None = None
        for offset, row in enumerate(rows):
            item = self.insert_requirement_item(group, index + offset, row)
            if first_item is None:
                first_item = item
        group.setExpanded(True)
        if first_item is not None:
            self.requirements.setCurrentItem(first_item)
        self.mark_current_dirty()

    def after_requirement_tree_changed(self) -> None:
        QTimer.singleShot(0, self.restore_requirement_widgets)
        self.mark_current_dirty()

    def restore_requirement_widgets(self) -> None:
        if self.loading:
            return
        self.loading = True
        try:
            self.normalize_requirement_groups()
            for item in self.iter_requirement_items():
                if checkbox_widget_checked(self.requirements.itemWidget(item, 2)) is None:
                    set_tree_check(item, 2, tree_checked(item, 2, True))
                if checkbox_widget_checked(self.requirements.itemWidget(item, 3)) is None:
                    set_tree_check(item, 3, tree_checked(item, 3, False))
                btype = tree_cell_text(self.requirements, item, 4) or "number"
                if not isinstance(self.requirements.itemWidget(item, 4), QComboBox):
                    set_tree_combo(self.requirements, item, 4, binding_type_options(), btype)
            header = self.requirements.header()
            header.setStretchLastSection(False)
            for col in range(self.requirements.columnCount()):
                header.setSectionResizeMode(col, QHeaderView.ResizeMode.Interactive)
            self.requirements.setColumnWidth(2, 46)
            self.requirements.setColumnWidth(3, 46)
        finally:
            self.loading = False

    def after_macro_tree_changed(self, tree: QTreeWidget) -> None:
        if self.loading:
            return
        QTimer.singleShot(0, lambda t=tree: self.restore_macro_widgets(t))
        self.mark_current_dirty()

    def restore_macro_widgets(self, tree: QTreeWidget) -> None:
        if self.loading:
            return
        self.loading = True
        try:
            self.normalize_macro_groups(tree)
            macro_type = "option" if tree is self.enum_macros else "feature"
            for item in self.iter_macro_items(tree):
                if not (item.flags() & Qt.ItemFlag.ItemIsUserCheckable):
                    set_tree_check(item, 1, tree_checked(item, 1, True))
                    set_tree_check(item, 2, tree_checked(item, 2, False))
                if macro_type == "option":
                    options = [part.strip() for part in tree_cell_text(tree, item, 5).split(",") if part.strip()]
                    value = tree_cell_text(tree, item, 3) or (options[0] if options else "1")
                    if not isinstance(tree.itemWidget(item, 3), QComboBox):
                        set_tree_combo(tree, item, 3, options or [value], value)
            description_col = 6 if tree is self.enum_macros else 4
            fit_tree_key_columns(tree, description_col=description_col, interactive=True)
            tree.setColumnWidth(1, 46)
            tree.setColumnWidth(2, 46)
        finally:
            self.loading = False

    def normalize_macro_groups(self, tree: QTreeWidget) -> None:
        default_group = "Option Macros" if tree is self.enum_macros else "Selection Macros"
        if tree.topLevelItemCount() == 0:
            tree.addTopLevelItem(self.create_macro_group_item(default_group))
            return
        macro_roles = {"feature_macro", "option_macro"}
        top_level_macros: list[QTreeWidgetItem] = []
        for index in range(tree.topLevelItemCount()):
            item = tree.topLevelItem(index)
            if item.data(0, Qt.ItemDataRole.UserRole) in macro_roles:
                top_level_macros.append(item)
        if not top_level_macros:
            return
        ungrouped: QTreeWidgetItem | None = None
        for index in range(tree.topLevelItemCount()):
            item = tree.topLevelItem(index)
            if item.data(0, Qt.ItemDataRole.UserRole) == "group" and item.text(0) == "Ungrouped":
                ungrouped = item
                break
        if ungrouped is None:
            ungrouped = self.create_macro_group_item("Ungrouped")
            tree.addTopLevelItem(ungrouped)
        for item in top_level_macros:
            index = tree.indexOfTopLevelItem(item)
            moved = tree.takeTopLevelItem(index)
            ungrouped.addChild(moved)
        ungrouped.setExpanded(True)

    def iter_requirement_items(self) -> list[QTreeWidgetItem]:
        items: list[QTreeWidgetItem] = []
        for group_index in range(self.requirements.topLevelItemCount()):
            group = self.requirements.topLevelItem(group_index)
            if group.data(0, Qt.ItemDataRole.UserRole) == "requirement":
                items.append(group)
                continue
            for child_index in range(group.childCount()):
                child = group.child(child_index)
                if child.data(0, Qt.ItemDataRole.UserRole) == "requirement":
                    items.append(child)
        return items

    def normalize_requirement_groups(self) -> None:
        if self.requirements.topLevelItemCount() == 0:
            self.requirements.addTopLevelItem(self.create_requirement_group_item("Requirements"))
            return
        top_level_requirements: list[QTreeWidgetItem] = []
        for index in range(self.requirements.topLevelItemCount()):
            item = self.requirements.topLevelItem(index)
            if item.data(0, Qt.ItemDataRole.UserRole) == "requirement":
                top_level_requirements.append(item)
        if not top_level_requirements:
            return
        ungrouped: QTreeWidgetItem | None = None
        for index in range(self.requirements.topLevelItemCount()):
            item = self.requirements.topLevelItem(index)
            if item.data(0, Qt.ItemDataRole.UserRole) == "group" and item.text(0) == "Ungrouped":
                ungrouped = item
                break
        if ungrouped is None:
            ungrouped = self.create_requirement_group_item("Ungrouped")
            self.requirements.addTopLevelItem(ungrouped)
        for item in top_level_requirements:
            index = self.requirements.indexOfTopLevelItem(item)
            moved = self.requirements.takeTopLevelItem(index)
            ungrouped.addChild(moved)
        ungrouped.setExpanded(True)

    def add_enum_macro(self) -> None:
        self.add_macro_item(self.enum_macros, "option")

    def set_tree_row_validation_border(self, tree: QTreeWidget, item: QTreeWidgetItem, enabled: bool) -> None:
        for col in range(tree.columnCount()):
            item.setData(col, VALIDATION_BORDER_ROLE, enabled)

    def clear_macro_validation_highlights(self) -> None:
        for item in self.iter_requirement_items():
            self.set_tree_row_validation_border(self.requirements, item, False)
        for tree in (self.feature_macros, self.enum_macros):
            for item in self.iter_macro_items(tree):
                self.set_tree_row_validation_border(tree, item, False)

    def collect_project_macro_occurrences(self) -> dict[str, list[tuple[QTreeWidget, QTreeWidgetItem, str]]]:
        occurrences: dict[str, list[tuple[QTreeWidget, QTreeWidgetItem, str]]] = {}

        def add(name: str, tree: QTreeWidget, item: QTreeWidgetItem, label: str) -> None:
            macro = macro_name(name)
            if not macro:
                return
            occurrences.setdefault(macro, []).append((tree, item, label))

        for item in self.iter_requirement_items():
            add(tree_cell_text(self.requirements, item, 1), self.requirements, item, "requirement")

        for item in self.iter_macro_items(self.feature_macros):
            add(tree_cell_text(self.feature_macros, item, 0), self.feature_macros, item, "selection macro")

        for item in self.iter_macro_items(self.enum_macros):
            macro = tree_cell_text(self.enum_macros, item, 0)
            add(macro, self.enum_macros, item, "option macro")
            value = tree_cell_text(self.enum_macros, item, 3)
            if macro and re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", value):
                add(f"{macro}_{macro_name(value)}", self.enum_macros, item, "option marker")
        return occurrences

    def validate_project_macros(self) -> None:
        self.clear_macro_validation_highlights()
        occurrences = self.collect_project_macro_occurrences()
        metadata = self.window.generator().project_metadata_macro
        prefix_data = {"macro_prefix": self.macro_prefix_edit.text()}
        reserved = {metadata(prefix_data, "SDPE_PROJECT_ID")}
        if self.suite_edit.text().strip():
            reserved.add(metadata(prefix_data, "SDPE_PROJECT_SUITE"))
        if self.version_edit.text().strip():
            reserved.add(metadata(prefix_data, "SDPE_PROJECT_VERSION"))
        if self.updated_edit.text().strip():
            reserved.add(metadata(prefix_data, "SDPE_PROJECT_UPDATED_AT"))
        duplicates = {
            macro: rows
            for macro, rows in occurrences.items()
            if len(rows) > 1 or macro in reserved
        }
        if not duplicates:
            self.message("Validate Macros", "No duplicate project macros found.")
            return

        for rows in duplicates.values():
            for tree, item, _label in rows:
                self.set_tree_row_validation_border(tree, item, True)
        names = ", ".join(sorted(duplicates)[:8])
        extra = "" if len(duplicates) <= 8 else f" (+{len(duplicates) - 8} more)"
        self.message("Validate Macros", f"Duplicate macro(s): {names}{extra}")

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

    def choose_entity_with_same_template(self, old_entity: str) -> str | None:
        try:
            old = self.window.library.entity(old_entity)
        except SDPEError:
            return self.choose_entity()
        labels: list[str] = []
        for entity_id in sorted(self.window.library.entity_files):
            if entity_id == old_entity:
                continue
            try:
                entity = self.window.library.entity(entity_id)
                if entity.schema_id != old.schema_id:
                    continue
                schema = self.window.library.schema(entity.schema_id)
                labels.append(f"{entity_id} | {entity.display_name} | {schema.display_name} | {schema.category}")
            except SDPEError:
                continue
        selected = choose_item(self, "Replace With Same Template Entity", labels)
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
        self.replace_hardware_row(row)

    def replace_hardware_row(self, row: int) -> None:
        old_entity = item_text(self.hardware, row, 0)
        selected = self.choose_entity_with_same_template(old_entity)
        if selected:
            set_item(self.hardware, row, 0, selected)
            self.populate_hardware_info(row, selected)
            self.replace_project_entity_references(old_entity, selected)
            self.refresh_hardware_status()
            self.mark_current_dirty()

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

    def on_requirement_cell_double_clicked(self, item: QTreeWidgetItem, col: int) -> None:
        if item.data(0, Qt.ItemDataRole.UserRole) != "requirement":
            return
        if col == 6:
            self.edit_requirement_description(item)
            return
        if col != 5:
            return
        btype = tree_cell_text(self.requirements, item, 4)
        if btype == "export":
            self.select_requirement_hardware_parameter(item)
        elif btype in {"float", "number", "string", "macro"}:
            self.edit_requirement_value_in_cell(item)
        else:
            self.edit_requirement_value_dialog(item)

    def edit_requirement_value_in_cell(self, item: QTreeWidgetItem) -> None:
        self.requirements.setCurrentItem(item, 5)
        self.requirements.editItem(item, 5)

    def edit_requirement_value_dialog(self, item: QTreeWidgetItem) -> None:
        text = edit_multiline(self, "Requirement Binding Value", tree_cell_text(self.requirements, item, 5))
        if text is not None:
            item.setText(5, text)
            self.mark_current_dirty()

    def select_requirement_hardware_parameter(self, item: QTreeWidgetItem) -> None:
        symbols: list[str] = []
        for hw in self._display_hardware():
            try:
                symbols.extend(self.window.symbols_for_entity(hw["entity"]))
            except SDPEError:
                continue
        selected = choose_tree_item(self, "Select Requirement Binding", sorted(dict.fromkeys(symbols)))
        if selected:
            if tree_cell_text(self.requirements, item, 4) != "expr":
                set_tree_combo(self.requirements, item, 4, binding_type_options(), "export")
            item.setText(5, f"${{{selected}}}")
            self.mark_current_dirty()

    def on_macro_tree_double_clicked(self, item: QTreeWidgetItem, col: int) -> None:
        tree = self.sender()
        if item.data(0, Qt.ItemDataRole.UserRole) == "group":
            if col == 0:
                tree.editItem(item, 0)
            return
        if tree is self.feature_macros and col == 4:
            text = edit_multiline(self, "Selection Macro Description", tree_cell_text(self.feature_macros, item, col))
            if text is not None:
                item.setText(col, text)
                self.mark_current_dirty()
        elif tree is self.enum_macros and col == 4:
            presets = self.option_preset_labels()
            selected = choose_item(self, "Select Option Preset", presets)
            if selected:
                item.setText(col, selected.split("|", 1)[0].strip())
                self.sync_option_macro_preset_item(item, prefer_preset=True)
        elif tree is self.enum_macros and col == 5:
            text = edit_multiline(self, "Option Macro Options", tree_cell_text(self.enum_macros, item, col))
            if text is not None:
                item.setText(col, text)
                self.sync_option_macro_value_combo_item(item)
                self.mark_current_dirty()
        elif tree is self.enum_macros and col == 6:
            text = edit_multiline(self, "Option Macro Description", tree_cell_text(self.enum_macros, item, col))
            if text is not None:
                item.setText(col, text)
                self.mark_current_dirty()

    def on_enum_macro_item_changed(self, item: QTreeWidgetItem, col: int) -> None:
        if self.loading:
            return
        if item.data(0, Qt.ItemDataRole.UserRole) != "option_macro":
            return
        if col == 4:
            self.sync_option_macro_preset_item(item, prefer_preset=True)
        elif col == 5:
            self.sync_option_macro_value_combo_item(item)

    def sync_option_macro_preset_item(self, item: QTreeWidgetItem, prefer_preset: bool = True) -> None:
        preset = tree_cell_text(self.enum_macros, item, 4)
        options = self.option_preset_options(preset) if preset else []
        if not options and not prefer_preset:
            return
        old_state = self.enum_macros.blockSignals(True)
        try:
            if options:
                item.setText(5, ", ".join(str(option) for option in options))
            self.sync_option_macro_value_combo_item(item)
        finally:
            self.enum_macros.blockSignals(old_state)

    def sync_option_macro_value_combo_item(self, item: QTreeWidgetItem) -> None:
        options = [part.strip() for part in tree_cell_text(self.enum_macros, item, 5).split(",") if part.strip()]
        if not options:
            return
        current = tree_cell_text(self.enum_macros, item, 3)
        if current not in options:
            current = options[0]
        set_tree_combo(self.enum_macros, item, 3, options, current)

    def replace_requirement_entity(self, old_entity: str, new_entity: str) -> None:
        if not old_entity or old_entity == new_entity:
            return
        for item in self.iter_requirement_items():
            value = tree_cell_text(self.requirements, item, 5)
            if value == old_entity or value.startswith(f"{old_entity}."):
                item.setText(5, new_entity + value[len(old_entity):])

    def replace_project_entity_references(self, old_entity: str, new_entity: str, aliases: list[str] | None = None) -> None:
        if not old_entity or old_entity == new_entity:
            return
        names = [old_entity, *(aliases or [])]
        names = [name for name in dict.fromkeys(names) if name]

        def replace_text(text: str) -> str:
            result = text
            for name in names:
                pattern = re.compile(rf"(?<![A-Za-z0-9_]){re.escape(name)}(?=\.|\b)")
                result = pattern.sub(new_entity, result)
            return result

        for item in self.iter_requirement_items():
            for col in (0, 1, 5, 6):
                text = tree_cell_text(self.requirements, item, col)
                new_text = replace_text(text)
                if new_text != text:
                    item.setText(col, new_text)

        for tree in (self.feature_macros, self.enum_macros):
            for item in self.iter_macro_items(tree):
                for col in range(tree.columnCount()):
                    text = tree_cell_text(tree, item, col)
                    new_text = replace_text(text)
                    if new_text != text:
                        item.setText(col, new_text)

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
        for hw in self._display_hardware():
            entity_id = hw["entity"]
            category = self.entity_category(entity_id) or "Missing"
            if category not in categories:
                node = QTreeWidgetItem([category])
                self.hardware_tree.addTopLevelItem(node)
                categories[category] = node
            label = f"{entity_id} (inherited)" if hw.get("inherited") else entity_id
            entity_node = QTreeWidgetItem([label, self.entity_description(entity_id)])
            entity_node.setData(0, Qt.ItemDataRole.UserRole, entity_id)
            entity_node.setData(0, Qt.ItemDataRole.UserRole + 1, "root")
            entity_node.setData(0, Qt.ItemDataRole.UserRole + 2, entity_id)
            categories[category].addChild(entity_node)
            self.add_component_nodes(entity_node, entity_id, set(), entity_id, entity_id)
        self.hardware_tree.expandAll()
        fit_tree_key_columns(self.hardware_tree, description_col=1)

    def add_component_nodes(
        self,
        parent: QTreeWidgetItem,
        entity_id: str,
        seen: set[str],
        root_entity: str,
        path: str,
    ) -> None:
        if entity_id in seen:
            return
        seen.add(entity_id)
        try:
            entity = self.window.library.entity(entity_id)
        except SDPEError:
            return
        for slot, comp in entity.components.items():
            child_path = f"{path}.{slot}"
            child = QTreeWidgetItem([f"{slot}: {comp.entity.id}", comp.entity.description])
            child.setData(0, Qt.ItemDataRole.UserRole, comp.entity.id)
            child.setData(0, Qt.ItemDataRole.UserRole + 1, "component")
            child.setData(0, Qt.ItemDataRole.UserRole + 2, root_entity)
            child.setData(0, Qt.ItemDataRole.UserRole + 3, slot)
            child.setData(0, Qt.ItemDataRole.UserRole + 4, child_path)
            parent.addChild(child)
            self.add_component_nodes(child, comp.entity.id, seen, root_entity, child_path)

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
        for hw in self._display_hardware():
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

        for hw in self._display_hardware():
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
        root_item = self.hardware_root_item(item)
        is_component = item is not None and item.data(0, Qt.ItemDataRole.UserRole + 1) == "component"
        menu = QMenu(self)
        menu.addAction("Insert hardware", lambda: self.add_hardware(category))
        replace_action = menu.addAction("Replace with same template", lambda: self.replace_tree_root_hardware(root_item))
        replace_action.setEnabled(root_item is not None)
        replace_component_action = menu.addAction("Replace selected submodule", lambda: self.replace_tree_submodule(item))
        replace_component_action.setEnabled(is_component)
        delete_action = menu.addAction("Delete selected root hardware", self.remove_tree_root_hardware)
        delete_action.setEnabled(root_item is not None)
        menu.exec(self.hardware_tree.viewport().mapToGlobal(pos))

    def show_hardware_table_context_menu(self, pos) -> None:
        row = self.hardware.rowAt(pos.y())
        menu = QMenu(self)
        if row >= 0:
            self.hardware.setCurrentCell(row, max(0, self.hardware.currentColumn()))
        replace_action = menu.addAction("Replace with same template", lambda: self.replace_hardware_row(row))
        replace_action.setEnabled(row >= 0)
        remove_action = menu.addAction("Remove hardware", self.remove_hardware)
        remove_action.setEnabled(row >= 0)
        menu.exec(self.hardware.viewport().mapToGlobal(pos))

    def hardware_root_item(self, item: QTreeWidgetItem | None) -> QTreeWidgetItem | None:
        if item is None:
            return None
        if item.parent() is None:
            return None
        root = item
        while root.parent() is not None and root.parent().parent() is not None:
            root = root.parent()
        return root

    def remove_tree_root_hardware(self) -> None:
        item = self.hardware_tree.currentItem()
        item = self.hardware_root_item(item)
        if item and item.parent():
            entity_id = item.data(0, Qt.ItemDataRole.UserRole)
            for row in range(self.hardware.rowCount()):
                if item_text(self.hardware, row, 0) == entity_id:
                    self.hardware.removeRow(row)
                    break
            self.refresh_hardware_status()

    def replace_tree_root_hardware(self, item: QTreeWidgetItem | None) -> None:
        if item is None:
            return
        entity_id = item.data(0, Qt.ItemDataRole.UserRole)
        for row in range(self.hardware.rowCount()):
            if item_text(self.hardware, row, 0) == entity_id:
                self.replace_hardware_row(row)
                return

    def replace_tree_submodule(self, item: QTreeWidgetItem | None) -> None:
        if item is None or item.data(0, Qt.ItemDataRole.UserRole + 1) != "component":
            return
        old_entity = item.data(0, Qt.ItemDataRole.UserRole)
        if not old_entity:
            return
        component_path = item.data(0, Qt.ItemDataRole.UserRole + 4)
        selected = self.choose_entity_with_same_template(old_entity)
        if not selected:
            return
        if not self.hardware_entity_in_table(selected):
            self.add_hardware_row(selected, inherited=False)
        aliases = [component_path] if component_path else []
        self.replace_project_entity_references(old_entity, selected, aliases=aliases)
        self.refresh_hardware_status()
        self.mark_current_dirty()
        self.message("Submodule replaced", f"{old_entity} -> {selected}")

    def show_requirement_context_menu(self, pos) -> None:
        item = self.requirements.itemAt(pos)
        col = self.requirements.indexAt(pos).column()
        if item is not None and not item.isSelected():
            self.requirements.setCurrentItem(item, max(0, col))
        menu = QMenu(self)
        is_requirement = item is not None and item.data(0, Qt.ItemDataRole.UserRole) == "requirement"
        copy_action = menu.addAction("Copy selected rows", self.copy_selected_requirements)
        copy_action.setEnabled(bool(self.selected_requirement_items()))
        cut_action = menu.addAction("Cut selected rows", self.cut_selected_requirements)
        cut_action.setEnabled(bool(self.selected_requirement_items()))
        paste_action = menu.addAction("Paste rows", self.paste_requirements)
        paste_action.setEnabled(bool(QApplication.clipboard().text().strip()))
        menu.addSeparator()
        if is_requirement and col == 5:
            menu.addAction("Edit in cell", lambda: self.edit_requirement_value_in_cell(item))
            menu.addAction("Edit in dialog", lambda: self.edit_requirement_value_dialog(item))
            menu.addAction("Select hardware parameter", lambda: self.select_requirement_hardware_parameter(item))
        elif is_requirement and col == 6:
            menu.addAction("Edit description", lambda: self.edit_requirement_description(item))
        else:
            menu.addAction("Add group", self.add_requirement_group)
            menu.addAction("Add requirement", self.add_requirement)
            remove_action = menu.addAction("Remove selected", self.remove_requirement_item)
            remove_action.setEnabled(is_requirement or item is not None)
        menu.exec(self.requirements.viewport().mapToGlobal(pos))

    def edit_requirement_description(self, item: QTreeWidgetItem) -> None:
        text = edit_multiline(self, "Requirement Description", tree_cell_text(self.requirements, item, 6))
        if text is not None:
            item.setText(6, text)
            self.mark_current_dirty()

    def jump_from_requirement_item(self, item: QTreeWidgetItem | None) -> None:
        if self.loading or item is None or item.data(0, Qt.ItemDataRole.UserRole) != "requirement":
            return
        symbol = tree_cell_text(self.requirements, item, 1) or tree_cell_text(self.requirements, item, 0)
        self.jump_preview_to_text(symbol)

    def jump_from_macro_item(self, tree: QTreeWidget, item: QTreeWidgetItem | None) -> None:
        if self.loading or item is None or item.data(0, Qt.ItemDataRole.UserRole) not in {"feature_macro", "option_macro"}:
            return
        self.jump_preview_to_text(tree_cell_text(tree, item, 0))

    def preview_project_header(self) -> None:
        try:
            data = self.collect_current_data()
            if data is not None:
                self.set_code_text(self.window.generator().render_project_header(data))
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))

    def generate_project_header(self) -> None:
        try:
            self.save_current()
            if not self.current_id:
                return
            files = self.window.generator().generate_project(self.window.project_path(self.current_id))
            for item in files:
                if item.path.suffix.lower() == ".h" and item.path.exists():
                    self.set_code_text(item.path.read_text(encoding="utf-8"))
                    break
            self.message("Generated", "\n".join(str(item.path) for item in files))
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))

    def generate_matlab_init_script(self) -> None:
        try:
            self.save_current()
            if not self.current_id:
                return
            item = self.window.generator().generate_project_matlab_script(self.window.project_path(self.current_id))
            self.set_matlab_text(item.path.read_text(encoding="utf-8"))
            self.message("Generated", str(item.path))
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))


class BindingPage(SDPEPage):
    """Project-level SDPE overview and generation page."""

    def __init__(self, window: "MainWindow"):
        super().__init__(window, "SDPE Project Overview", has_code=True)
        self.overview = QTreeWidget()
        self.overview.setHeaderLabels(["Item", "Macro", "Value", "Source", "Description"])
        self.overview.setAlternatingRowColors(True)
        self.overview.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        fit_tree_key_columns(self.overview, description_col=4, interactive=True)
        install_tree_status_descriptions(self.overview, description_col=4)

        preview = QPushButton("Preview project header")
        preview.clicked.connect(self.preview_project)
        generate = QPushButton("Generate project header")
        generate.clicked.connect(self.generate_project)
        generate_matlab = QPushButton("Generate MATLAB Init Script")
        generate_matlab.clicked.connect(self.generate_matlab_init_script)

        self.form_layout.addLayout(row_buttons([preview, generate, generate_matlab]))
        self.form_layout.addWidget(QLabel("Supported project macros"))
        self.form_layout.addWidget(self.overview)

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
        self.populate_overview(data)
        self.set_professional_text(pretty_json(data))
        try:
            self.set_code_text(self.generator().render_project_header(data), reveal=False)
        except Exception as exc:
            self.code_panel.setPlainText(f"// Failed to render project header preview:\n// {exc}")
        try:
            self.set_matlab_text(self.generator().render_project_matlab_script(data), reveal=False)
        except Exception as exc:
            self.matlab_panel.setPlainText(f"% Failed to render MATLAB init preview:\n% {exc}")
        self.loading = False
        if not self.restoring_undo:
            self.reset_undo_history(self.collect_current_data() or data)

    def populate_overview(self, data: dict[str, Any]) -> None:
        self.overview.clear()
        generator = self.generator()
        self.add_metadata_group(data)
        self.add_hardware_group(data, generator)
        self.add_feature_macro_group(data)
        self.add_option_macro_group(data, generator)
        self.add_requirement_group(data, generator)
        self.overview.expandAll()
        fit_tree_key_columns(self.overview, description_col=4, interactive=True)

    def add_group(self, name: str, description: str = "") -> QTreeWidgetItem:
        item = QTreeWidgetItem([name, "", "", "", description])
        item.setFirstColumnSpanned(False)
        item.setData(0, Qt.ItemDataRole.UserRole, "group")
        self.overview.addTopLevelItem(item)
        return item

    def add_overview_row(
        self,
        parent: QTreeWidgetItem,
        item: str,
        macro: str = "",
        value: str = "",
        source: str = "",
        description: str = "",
    ) -> QTreeWidgetItem:
        child = QTreeWidgetItem([item, macro, value, source, description])
        child.setToolTip(4, description)
        parent.addChild(child)
        return child

    def add_metadata_group(self, data: dict[str, Any]) -> None:
        group = self.add_group("Project Metadata", "Macros generated for SDPE project identity.")
        project_id = data.get("id", self.current_id or "")
        metadata = self.generator().project_metadata_macro
        self.add_overview_row(group, "Project ID", metadata(data, "SDPE_PROJECT_ID"), f"\"{project_id}\"", "project", "SDPE project identifier.")
        if data.get("suite"):
            self.add_overview_row(group, "Suite", metadata(data, "SDPE_PROJECT_SUITE"), f"\"{data['suite']}\"", "project", "Suite identifier.")
        if data.get("version"):
            self.add_overview_row(group, "Version", metadata(data, "SDPE_PROJECT_VERSION"), f"\"{data['version']}\"", "project", "Requirement version.")
        if data.get("updated_at"):
            self.add_overview_row(group, "Updated At", metadata(data, "SDPE_PROJECT_UPDATED_AT"), f"\"{data['updated_at']}\"", "project", "Last update date.")

    def add_hardware_group(self, data: dict[str, Any], generator: HeaderGenerator) -> None:
        group = self.add_group("Hardware Includes", "Hardware headers included by the generated project header.")
        for hw in project_hardware_items(data):
            entity_id = project_hardware_entity_id(hw)
            if not entity_id:
                continue
            try:
                entity = self.window.library.entity(entity_id)
                schema = self.window.library.schema(entity.schema_id)
                include_path = generator.entity_include_path(entity)
                desc = entity.description or schema.description
                parent = self.add_overview_row(group, entity.display_name or entity.id, "#include", f"<{include_path}>", entity.id, desc)
                for pspec in schema.parameters:
                    macro = f"{generator.prefix(entity, schema)}_{pspec.c_name}"
                    self.add_overview_row(parent, pspec.name, macro, "", schema.id, pspec.description)
            except Exception as exc:
                self.add_overview_row(group, entity_id, "#include", "", entity_id, f"Unable to resolve hardware entity: {exc}")

    def add_feature_macro_group(self, data: dict[str, Any]) -> None:
        group = self.add_group("Selection Macros", "Enable or disable project feature macros.")
        for group_name, items in self.group_project_macros(data.get("feature_macros", []), "Selection Macros"):
            subgroup = QTreeWidgetItem([group_name, "", "", "", "Selection macro group."])
            group.addChild(subgroup)
            for item in items:
                macro = item.get("macro", "")
                enabled = item.get("enabled", True)
                weak = item.get("weak", False)
                value = str(item.get("value", ""))
                display_value = value if enabled else f"disabled ({value})" if value else "disabled"
                if weak:
                    display_value = f"{display_value}; weak"
                self.add_overview_row(subgroup, macro, macro, display_value, "selection macro", item.get("description", ""))

    def add_option_macro_group(self, data: dict[str, Any], generator: HeaderGenerator) -> None:
        group = self.add_group("Option Macros", "Project macros with selectable values.")
        for group_name, items in self.group_project_macros(data.get("option_macros", []), "Option Macros"):
            subgroup = QTreeWidgetItem([group_name, "", "", "", "Option macro group."])
            group.addChild(subgroup)
            for item in items:
                macro = item.get("macro", "")
                options = generator._option_macro_values(item, lambda preset: generator._resolve_project_option_preset(data, preset))
                value = str(item.get("value", ""))
                enabled = item.get("enabled", True)
                weak = item.get("weak", False)
                display_value = value if enabled else f"disabled ({value})"
                if weak:
                    display_value = f"{display_value}; weak"
                desc = item.get("description", "")
                parent = self.add_overview_row(subgroup, macro, macro, display_value, "option macro", desc)
                if options:
                    self.add_overview_row(parent, "Options", "", ", ".join(str(option) for option in options), item.get("options_preset", ""), "Selectable values.")
                if enabled and re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", value):
                    marker_macro = f"{macro}_{macro_name(value)}"
                    self.add_overview_row(parent, "Selected marker", marker_macro, "1", "generated marker", "Generated marker macro for the selected symbolic option.")

    def group_project_macros(self, items: list[dict[str, Any]], default_group: str) -> list[tuple[str, list[dict[str, Any]]]]:
        groups: dict[str, list[dict[str, Any]]] = {}
        for item in items:
            if not item.get("macro"):
                continue
            group = str(item.get("group") or default_group).strip() or default_group
            groups.setdefault(group, []).append(item)
        return list(groups.items())

    def add_requirement_group(self, data: dict[str, Any], generator: HeaderGenerator) -> None:
        group = self.add_group("Requirement Bindings", "Project requirement macros resolved from manual values or hardware parameters.")
        requirements = data.get("requirements", [])
        by_role = {req.get("role", req.get("macro", "")): req for req in requirements}
        grouped_roles: set[str] = set()
        for req_group in data.get("requirement_groups", []):
            subgroup = QTreeWidgetItem([req_group.get("name", "Requirements"), "", "", "", "Requirement group."])
            group.addChild(subgroup)
            for role in req_group.get("requirements", []):
                req = by_role.get(role)
                if req is not None:
                    self.add_requirement_row(subgroup, req, generator)
                    grouped_roles.add(role)
        for req in requirements:
            role = req.get("role", req.get("macro", ""))
            if role not in grouped_roles:
                self.add_requirement_row(group, req, generator)

    def add_requirement_row(self, parent: QTreeWidgetItem, req: dict[str, Any], generator: HeaderGenerator) -> None:
        macro = req.get("macro", "")
        role = req.get("role", macro)
        try:
            value = generator._resolve_binding_value(req.get("binding", {}))
        except Exception as exc:
            value = f"<unresolved: {exc}>"
        btype, bvalue = binding_to_cells(req.get("binding", {}))
        source = f"{btype}: {bvalue}" if bvalue else btype
        self.add_overview_row(parent, role, macro, value, source, req.get("description", ""))

    def path_for_id(self, item_id: str) -> Path:
        return self.window.project_path(item_id)

    def collect_current_data(self) -> dict[str, Any] | None:
        if not self.current_id:
            return None
        return read_json(self.window.project_path(self.current_id))

    def save_current(self) -> None:
        self.message("Overview", "Project Overview is read-only. Edit requirements in Project Requirement.")

    def generator(self) -> HeaderGenerator:
        return self.window.generator()

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

    def generate_matlab_init_script(self) -> None:
        try:
            item = self.generator().generate_project_matlab_script(self.window.project_path(self.current_id))
            self.set_matlab_text(item.path.read_text(encoding="utf-8"))
            self.message("Generated", str(item.path))
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
        include_prefix: str = "ctl/component",
        include_mode: str = "prefixed",
        project_subdir: str = "project",
        system_entity_dirs: list[Path] | None = None,
        system_out_dir: Path | None = None,
        system_include_prefix: str = "ctl",
    ):
        super().__init__()
        self.library_root = library_root
        self.mode = mode
        self.schema_dirs = schema_dirs or [library_root / "schemas"]
        self.entity_dirs = entity_dirs or [library_root / "entities"]
        self.project_dirs = project_dirs or [library_root / "projects"]
        self.project_dirs.extend(self.load_recent_project_files())
        self.default_output_dir = default_output_dir or (ROOT / "build_gui_pyqt")
        self.include_prefix = include_prefix
        self.include_mode = include_mode
        self.project_subdir = project_subdir
        self.system_entity_dirs = system_entity_dirs or []
        self.system_out_dir = system_out_dir
        self.system_include_prefix = system_include_prefix
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
        self.create_menus()
        self.tabs.currentChanged.connect(lambda _index: self.update_menu_actions())
        self.update_menu_actions()
        self.refresh_pages()

    def load_library(self) -> SDPELibrary:
        return SDPELibrary(self.library_root, self.schema_dirs, self.entity_dirs).load()

    def generator(self, out_dir: Path | None = None) -> HeaderGenerator:
        return HeaderGenerator(
            self.library,
            out_dir or self.default_output_dir,
            self.include_prefix,
            self.include_mode,
            self.project_subdir,
            self.system_entity_dirs,
            self.system_out_dir,
            self.system_include_prefix,
        )

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

    def create_menus(self) -> None:
        file_menu = self.menuBar().addMenu("&File")
        self.action_save = self.add_menu_action(file_menu, "Save current", self.save_current_page)
        self.action_save_all = self.add_menu_action(file_menu, "Save all", self.save_all_pages)
        file_menu.addSeparator()
        self.action_open_project = self.add_menu_action(file_menu, "Open project requirement...", lambda: self.call_page_action("open_project_file"))

        edit_menu = self.menuBar().addMenu("&Edit")
        self.action_undo = self.add_menu_action(edit_menu, "Undo", self.undo_current_page)
        edit_menu.addSeparator()
        self.add_menu_action(edit_menu, "Copy", lambda: self.call_focused_edit("copy"))
        self.add_menu_action(edit_menu, "Cut", lambda: self.call_focused_edit("cut"))
        self.add_menu_action(edit_menu, "Paste", lambda: self.call_focused_edit("paste"))
        self.add_menu_action(edit_menu, "Select all", lambda: self.call_focused_edit("selectAll"))

        generate_menu = self.menuBar().addMenu("&Generate")
        self.action_preview_header = self.add_menu_action(generate_menu, "Preview header", self.preview_current_header)
        self.action_generate_header = self.add_menu_action(generate_menu, "Generate header", self.generate_current_header)
        self.action_generate_matlab = self.add_menu_action(generate_menu, "Generate MATLAB init script", self.generate_current_matlab_init)
        generate_menu.addSeparator()
        self.action_validate_macros = self.add_menu_action(generate_menu, "Validate macros", self.validate_current_macros)

    def add_menu_action(self, menu: QMenu, text: str, callback, shortcut: QKeySequence | QKeySequence.StandardKey | None = None) -> QAction:
        action = QAction(text, self)
        if shortcut is not None:
            action.setShortcut(shortcut)
        action.triggered.connect(callback)
        menu.addAction(action)
        return action

    def current_sdpe_page(self) -> SDPEPage | None:
        widget = self.tabs.currentWidget()
        return widget if isinstance(widget, SDPEPage) else None

    def call_page_action(self, action_name: str) -> bool:
        page = self.current_sdpe_page()
        callback = getattr(page, action_name, None) if page is not None else None
        if callable(callback):
            callback()
            return True
        self.statusBar().showMessage("This action is not available on the current page.", 2600)
        return False

    def call_first_page_action(self, action_names: list[str]) -> bool:
        page = self.current_sdpe_page()
        if page is not None:
            for action_name in action_names:
                callback = getattr(page, action_name, None)
                if callable(callback):
                    callback()
                    return True
        self.statusBar().showMessage("This action is not available on the current page.", 2600)
        return False

    def save_current_page(self) -> None:
        self.call_page_action("save_current")

    def save_all_pages(self) -> None:
        saved = 0
        for page in self.pages:
            saved += page.save_dirty()
        if saved:
            self.reload()
            self.refresh_pages()
        self.statusBar().showMessage(f"Saved {saved} file(s).", 2600)

    def undo_current_page(self) -> None:
        self.call_page_action("undo_current_change")

    def preview_current_header(self) -> None:
        self.call_first_page_action(["preview_project_header", "preview_project", "preview_header"])

    def generate_current_header(self) -> None:
        self.call_first_page_action(["generate_project_header", "generate_project", "generate_header"])

    def generate_current_matlab_init(self) -> None:
        self.call_page_action("generate_matlab_init_script")

    def validate_current_macros(self) -> None:
        self.call_page_action("validate_project_macros")

    def call_focused_edit(self, method_name: str) -> None:
        widget = QApplication.focusWidget()
        method = getattr(widget, method_name, None) if widget is not None else None
        if callable(method):
            method()
            return
        self.statusBar().showMessage("The focused control does not support this edit action.", 2600)

    def update_menu_actions(self) -> None:
        page = self.current_sdpe_page()
        has_page = page is not None
        self.action_save.setEnabled(has_page and callable(getattr(page, "save_current", None)))
        self.action_save_all.setEnabled(has_page)
        self.action_undo.setEnabled(has_page and callable(getattr(page, "undo_current_change", None)))
        self.action_open_project.setEnabled(callable(getattr(page, "open_project_file", None)) if page is not None else False)
        self.action_preview_header.setEnabled(
            has_page
            and (
                callable(getattr(page, "preview_project_header", None))
                or callable(getattr(page, "preview_project", None))
                or callable(getattr(page, "preview_header", None))
            )
        )
        self.action_generate_header.setEnabled(
            has_page
            and (
                callable(getattr(page, "generate_project_header", None))
                or callable(getattr(page, "generate_project", None))
                or callable(getattr(page, "generate_header", None))
            )
        )
        self.action_generate_matlab.setEnabled(has_page and callable(getattr(page, "generate_matlab_init_script", None)))
        self.action_validate_macros.setEnabled(has_page and callable(getattr(page, "validate_project_macros", None)))

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
    parser.add_argument("--settings", type=Path, default=ROOT / "sdpe_settings.json", help="SDPE settings JSON file.")
    parser.add_argument("--library", type=Path, default=None, help="SDPE library root.")
    parser.add_argument("--mode", choices=["all", "library", "project"], default="all", help="GUI work mode.")
    parser.add_argument("--schemas", nargs="*", type=Path, help="Template/schema directories.")
    parser.add_argument("--entities", nargs="*", type=Path, help="Entity instance directories.")
    parser.add_argument("--projects", nargs="*", type=Path, help="Project requirement files or directories.")
    parser.add_argument("--out", type=Path, default=None, help="Default header output directory.")
    return parser.parse_args(argv)


def expand_settings_path(value: str | Path, base: Path = ROOT) -> Path:
    path = Path(os.path.expandvars(str(value)))
    return path if path.is_absolute() else (base / path).resolve()


def load_gui_settings(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return read_json(path)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    app = QApplication(sys.argv)
    settings = load_gui_settings(args.settings)
    explicit_library = args.library is not None
    library_root = args.library.resolve() if explicit_library else expand_settings_path(settings.get("library_root", ROOT / "examples"))
    schema_dirs = [path.resolve() for path in args.schemas] if args.schemas else (
        [] if explicit_library else [expand_settings_path(path) for path in settings.get("schema_dirs", [])]
    ) or None
    entity_dirs = [path.resolve() for path in args.entities] if args.entities else (
        [] if explicit_library else [expand_settings_path(path) for path in settings.get("entity_dirs", [])]
    ) or None
    generation_section = "local_generation" if args.mode == "project" else "global_generation"
    generation_cfg = settings.get(generation_section, {})
    system_cfg = settings.get("system_hardware", {})
    window = MainWindow(
        library_root,
        mode=args.mode,
        schema_dirs=schema_dirs,
        entity_dirs=entity_dirs,
        project_dirs=[path.resolve() for path in args.projects] if args.projects else None,
        default_output_dir=args.out.resolve() if args.out else expand_settings_path(settings.get("gui", {}).get("default_out", ROOT / "build_gui_pyqt")),
        include_prefix=str(generation_cfg.get("include_prefix", "ctl/component")),
        include_mode=str(generation_cfg.get("include_mode", "prefixed")),
        project_subdir=str(generation_cfg.get("project_subdir", "project")),
        system_entity_dirs=[expand_settings_path(path) for path in system_cfg.get("entity_dirs", [])],
        system_out_dir=expand_settings_path(system_cfg["out"]) if system_cfg.get("out") else None,
        system_include_prefix=str(system_cfg.get("include_prefix", "ctl")),
    )
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
