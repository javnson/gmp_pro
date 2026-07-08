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
        QFileDialog,
        QFormLayout,
        QHBoxLayout,
        QHeaderView,
        QLabel,
        QLineEdit,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSplitter,
        QTableWidget,
        QTableWidgetItem,
        QTabWidget,
        QTextEdit,
        QToolBar,
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
            QFileDialog,
            QFormLayout,
            QHBoxLayout,
            QHeaderView,
            QLabel,
            QLineEdit,
            QListWidget,
            QListWidgetItem,
            QMainWindow,
            QMessageBox,
            QPushButton,
            QSplitter,
            QTableWidget,
            QTableWidgetItem,
            QTabWidget,
            QTextEdit,
            QToolBar,
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
    return json.loads(value)


def set_table_headers(
    table: QTableWidget, headers: list[str], mode: QHeaderView.ResizeMode = QHeaderView.ResizeMode.Stretch
) -> None:
    table.setColumnCount(len(headers))
    table.setHorizontalHeaderLabels(headers)
    table.horizontalHeader().setSectionResizeMode(mode)


def item_text(table: QTableWidget, row: int, col: int) -> str:
    item = table.item(row, col)
    return "" if item is None else item.text().strip()


def set_item(table: QTableWidget, row: int, col: int, value: Any) -> None:
    table.setItem(row, col, QTableWidgetItem("" if value is None else str(value)))


class SDPEPage(QWidget):
    """Common split view page: searchable object list, form editor, JSON/code preview."""

    def __init__(self, window: "MainWindow", title: str):
        super().__init__()
        self.window = window
        self.title = title
        self.current_id = ""

        self.search = QLineEdit()
        self.search.setPlaceholderText("Search id, name, tag")
        self.search.textChanged.connect(self.refresh_list)
        self.list_widget = QListWidget()
        self.list_widget.currentItemChanged.connect(self.on_current_changed)

        self.form_panel = QWidget()
        self.form_layout = QVBoxLayout(self.form_panel)
        self.preview = QTextEdit()
        self.preview.setReadOnly(True)
        self.preview.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.addWidget(self.search)
        left_layout.addWidget(self.list_widget)

        self.splitter = QSplitter()
        self.splitter.addWidget(left)
        self.splitter.addWidget(self.form_panel)
        self.splitter.addWidget(self.preview)
        self.splitter.setSizes([220, 520, 520])

        self.show_form = QCheckBox("Form")
        self.show_form.setChecked(True)
        self.show_form.toggled.connect(self.form_panel.setVisible)
        self.show_preview = QCheckBox("Code")
        self.show_preview.setChecked(True)
        self.show_preview.toggled.connect(self.preview.setVisible)

        toolbar = QToolBar(title)
        toolbar.addWidget(QLabel(title))
        toolbar.addSeparator()
        toolbar.addWidget(self.show_form)
        toolbar.addWidget(self.show_preview)

        layout = QVBoxLayout(self)
        layout.addWidget(toolbar)
        layout.addWidget(self.splitter)
        self.save_shortcut = QShortcut(QKeySequence.StandardKey.Save, self)
        self.save_shortcut.activated.connect(self.save_current)
        self.undo_shortcut = QShortcut(QKeySequence.StandardKey.Undo, self)
        self.undo_shortcut.activated.connect(self.undo_focused_widget)

    def refresh_list(self) -> None:
        raise NotImplementedError

    def on_current_changed(self, current: QListWidgetItem | None) -> None:
        self.current_id = "" if current is None else current.data(Qt.ItemDataRole.UserRole)
        self.load_current()

    def load_current(self) -> None:
        raise NotImplementedError

    def save_current(self) -> None:
        return None

    def undo_focused_widget(self) -> None:
        focused = QApplication.focusWidget()
        if focused is not None and hasattr(focused, "undo"):
            focused.undo()

    def select_first(self) -> None:
        if self.list_widget.count() and self.list_widget.currentRow() < 0:
            self.list_widget.setCurrentRow(0)

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

    def __init__(self, window: "MainWindow"):
        super().__init__(window, "Template Definition")
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
            ["Name", "Macro Name", "Unit", "Required", "Default", "Format", "Description"],
            QHeaderView.ResizeMode.Interactive,
        )
        self.slots = QTableWidget()
        set_table_headers(
            self.slots,
            ["Slot", "Accepted Schemas", "Accepted Categories", "Required", "Description"],
            QHeaderView.ResizeMode.Interactive,
        )

        form = QFormLayout()
        form.addRow("Template ID", self.id_edit)
        form.addRow("Name", self.name_edit)
        form.addRow("Category", self.category_edit)
        form.addRow("Tags", self.tags_edit)
        form.addRow("Output Folder", self.output_edit)
        form.addRow("Default Macro Prefix", self.header_prefix_edit)
        form.addRow("Description", self.description_edit)

        add_param = QPushButton("Add parameter")
        add_param.clicked.connect(lambda: self.params.insertRow(self.params.rowCount()))
        del_param = QPushButton("Remove parameter")
        del_param.clicked.connect(lambda: self.params.removeRow(max(0, self.params.currentRow())))
        add_slot = QPushButton("Add slot")
        add_slot.clicked.connect(lambda: self.slots.insertRow(self.slots.rowCount()))
        del_slot = QPushButton("Remove slot")
        del_slot.clicked.connect(lambda: self.slots.removeRow(max(0, self.slots.currentRow())))
        save = QPushButton("Save template")
        save.clicked.connect(self.save_current)

        self.form_layout.addLayout(form)
        self.form_layout.addWidget(QLabel("Parameters"))
        self.form_layout.addWidget(self.params)
        self.form_layout.addLayout(row_buttons([add_param, del_param]))
        self.form_layout.addWidget(QLabel("Component slots"))
        self.form_layout.addWidget(self.slots)
        self.form_layout.addLayout(row_buttons([add_slot, del_slot, save]))

    def refresh_list(self) -> None:
        current = self.current_id
        self.list_widget.clear()
        for schema in sorted(self.window.library.schemas.values(), key=lambda item: item.id):
            if not self.filter_match([schema.id, schema.display_name, schema.category, *schema.tags]):
                continue
            item = QListWidgetItem(f"{schema.id}  [{', '.join(schema.tags)}]")
            item.setData(Qt.ItemDataRole.UserRole, schema.id)
            self.list_widget.addItem(item)
            if schema.id == current:
                self.list_widget.setCurrentItem(item)
        self.select_first()

    def load_current(self) -> None:
        if not self.current_id:
            return
        schema = self.window.library.schema(self.current_id)
        data = read_json(self.window.schema_path(schema.id))
        self.id_edit.setText(schema.id)
        self.name_edit.setText(schema.display_name)
        self.category_edit.setText(schema.category)
        self.tags_edit.setText(", ".join(schema.tags))
        self.output_edit.setText(schema.output_subdir)
        self.header_prefix_edit.setText(schema.header_prefix)
        self.description_edit.setPlainText(schema.description)
        self.params.setRowCount(0)
        for param in data.get("parameters", []):
            row = self.params.rowCount()
            self.params.insertRow(row)
            for col, key in enumerate(["name", "c_name", "unit", "required", "default", "value_format", "description"]):
                set_item(self.params, row, col, pretty_json(param[key]) if key == "default" and key in param else param.get(key, ""))
        self.slots.setRowCount(0)
        for name, slot in data.get("component_slots", {}).items():
            row = self.slots.rowCount()
            self.slots.insertRow(row)
            set_item(self.slots, row, 0, name)
            set_item(self.slots, row, 1, ", ".join(slot.get("accepted_schemas", [])))
            set_item(self.slots, row, 2, ", ".join(slot.get("accepted_categories", [])))
            set_item(self.slots, row, 3, slot.get("required", False))
            set_item(self.slots, row, 4, slot.get("description", ""))
        self.preview.setPlainText(pretty_json(data))

    def save_current(self) -> None:
        try:
            path = self.window.schema_path(self.current_id)
            data = read_json(path)
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
            data["component_slots"] = self._table_slots()
            self.window.write_json(path, data)
            self.window.reload()
            self.refresh_list()
            self.message("Saved", f"Template saved: {path}")
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))

    def _table_parameters(self) -> list[dict[str, Any]]:
        rows = []
        for row in range(self.params.rowCount()):
            name = item_text(self.params, row, 0)
            if not name:
                continue
            item = {
                "name": name,
                "c_name": item_text(self.params, row, 1) or name.upper(),
                "unit": item_text(self.params, row, 2),
                "required": item_text(self.params, row, 3).lower() in {"1", "true", "yes"},
                "value_format": item_text(self.params, row, 5) or "{}",
                "description": item_text(self.params, row, 6),
            }
            default_text = item_text(self.params, row, 4)
            if default_text:
                item["default"] = parse_json_text(default_text, default_text)
            rows.append(item)
        return rows

    def _table_slots(self) -> dict[str, dict[str, Any]]:
        slots = {}
        for row in range(self.slots.rowCount()):
            name = item_text(self.slots, row, 0)
            if not name:
                continue
            slots[name] = {
                "accepted_schemas": split_tags(item_text(self.slots, row, 1)),
                "accepted_categories": split_tags(item_text(self.slots, row, 2)),
                "required": item_text(self.slots, row, 3).lower() in {"1", "true", "yes"},
                "description": item_text(self.slots, row, 4),
            }
        return slots


class EntityPage(SDPEPage):
    """Hardware entity editor and per-entity header generation."""

    def __init__(self, window: "MainWindow"):
        super().__init__(window, "Entity Instance")
        self.id_edit = QLineEdit()
        self.schema_combo = QComboBox()
        self.name_edit = QLineEdit()
        self.vendor_edit = QLineEdit()
        self.datasheet_edit = QLineEdit()
        self.document_edit = QLineEdit()
        self.macro_edit = QLineEdit()
        self.tags_edit = QLineEdit()
        self.description_edit = QTextEdit()
        self.output_edit = QLineEdit()
        self.params = QTableWidget()
        set_table_headers(self.params, ["Parameter", "Value"], QHeaderView.ResizeMode.Interactive)
        self.components = QTableWidget()
        set_table_headers(
            self.components,
            ["Slot", "Mode", "Entity or Inline JSON", "Overrides JSON"],
            QHeaderView.ResizeMode.Interactive,
        )
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

        browse = QPushButton("Browse")
        browse.clicked.connect(self.browse_out)
        save = QPushButton("Save entity")
        save.clicked.connect(self.save_current)
        add_comp = QPushButton("Add component")
        add_comp.clicked.connect(lambda: self.components.insertRow(self.components.rowCount()))
        del_comp = QPushButton("Remove component")
        del_comp.clicked.connect(lambda: self.components.removeRow(max(0, self.components.currentRow())))
        generate = QPushButton("Generate entity header")
        generate.clicked.connect(self.generate_header)
        preview_header = QPushButton("Preview header")
        preview_header.clicked.connect(self.preview_header)

        self.form_layout.addLayout(form)
        self.form_layout.addLayout(row_buttons([browse, save]))
        self.form_layout.addWidget(QLabel("Parameters"))
        self.form_layout.addWidget(self.params)
        self.form_layout.addWidget(QLabel("Components"))
        self.form_layout.addWidget(self.components)
        self.form_layout.addLayout(row_buttons([add_comp, del_comp, preview_header, generate]))

    def refresh_list(self) -> None:
        current = self.current_id
        self.schema_combo.clear()
        self.schema_combo.addItems(sorted(self.window.library.schemas))
        self.list_widget.clear()
        for entity_id in sorted(self.window.library.entity_files):
            entity = self.window.library.entity(entity_id)
            schema = self.window.library.schema(entity.schema_id)
            if not self.filter_match([entity.id, entity.display_name, entity.schema_id, *entity.tags, *schema.tags]):
                continue
            item = QListWidgetItem(f"{entity.id}  ({entity.schema_id}) [{', '.join(entity.tags)}]")
            item.setData(Qt.ItemDataRole.UserRole, entity.id)
            self.list_widget.addItem(item)
            if entity.id == current:
                self.list_widget.setCurrentItem(item)
        self.select_first()

    def load_current(self) -> None:
        if not self.current_id:
            return
        entity = self.window.library.entity(self.current_id)
        data = read_json(self.window.entity_path(entity.id))
        self.id_edit.setText(entity.id)
        self.schema_combo.setCurrentText(entity.schema_id)
        self.name_edit.setText(entity.display_name)
        self.vendor_edit.setText(entity.vendor)
        self.datasheet_edit.setText(entity.datasheet_url)
        self.document_edit.setText(entity.document_url)
        self.macro_edit.setText(entity.macro_prefix)
        self.tags_edit.setText(", ".join(entity.tags))
        self.output_edit.setText(entity.output_subdir)
        self.description_edit.setPlainText(entity.description)
        self.load_param_rows(entity, data)
        self.components.setRowCount(0)
        for slot, comp in data.get("components", {}).items():
            row = self.components.rowCount()
            self.components.insertRow(row)
            set_item(self.components, row, 0, slot)
            if "entity" in comp:
                set_item(self.components, row, 1, "entity")
                set_item(self.components, row, 2, comp["entity"])
            else:
                set_item(self.components, row, 1, "inline")
                set_item(self.components, row, 2, pretty_json(comp.get("inline", {})))
            set_item(self.components, row, 3, pretty_json(comp.get("overrides", {})))
        self.preview.setPlainText(pretty_json(data))

    def load_param_rows(self, entity: HardwareEntity, data: dict[str, Any]) -> None:
        schema = self.window.library.schema(entity.schema_id)
        self.params.setRowCount(0)
        known = set()
        for name in schema.parameters:
            row = self.params.rowCount()
            self.params.insertRow(row)
            set_item(self.params, row, 0, name)
            set_item(self.params, row, 1, pretty_json(data.get("parameters", {}).get(name, "")))
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
            path = self.window.entity_path(self.current_id)
            data = read_json(path)
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
            self.window.write_json(path, data)
            self.window.reload()
            self.refresh_list()
            self.message("Saved", f"Entity saved: {path}")
        except Exception as exc:  # pragma: no cover - GUI guard.
            self.error(str(exc))

    def _table_parameters(self) -> dict[str, Any]:
        params = {}
        for row in range(self.params.rowCount()):
            name = item_text(self.params, row, 0)
            value = item_text(self.params, row, 1)
            if name and value:
                params[name] = parse_json_text(value, value)
        return params

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
            entity = self.window.library.entity(self.current_id)
            self.preview.setPlainText(self.generator().render_entity_header(entity))
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
        super().__init__(window, "Project Requirement")
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

        self.form_layout.addLayout(form)
        self.form_layout.addWidget(QLabel("Hardware includes"))
        self.form_layout.addWidget(self.hardware)
        self.form_layout.addLayout(row_buttons([add_hw, del_hw]))
        self.form_layout.addWidget(QLabel("Requirements"))
        self.form_layout.addWidget(self.requirements)
        self.form_layout.addLayout(row_buttons([add_req, del_req, save]))
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
        self.preview.setPlainText(pretty_json(data))

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


class BindingPage(SDPEPage):
    """Focused requirement binding page with suggestions and generation."""

    def __init__(self, window: "MainWindow"):
        super().__init__(window, "Requirement Binding")
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
        self.preview.setPlainText(pretty_json(data))

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
            self.preview.setPlainText(self.generator().render_project_header(data))
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
