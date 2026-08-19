#!/usr/bin/env python3
"""Qt GUI for editing CANopen EDS dictionary projects and generating sources."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Any

try:
    from PySide6.QtCore import Qt, QEvent
    from PySide6.QtGui import QAction, QFont, QKeySequence
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDockWidget,
        QDialog,
        QDialogButtonBox,
        QFormLayout,
        QGroupBox,
        QHBoxLayout,
        QHeaderView,
        QInputDialog,
        QLabel,
        QListWidget,
        QListWidgetItem,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QAbstractItemView,
        QSpinBox,
        QStatusBar,
        QSplitter,
        QStyledItemDelegate,
        QTableWidget,
        QTableWidgetItem,
        QTreeWidget,
        QTreeWidgetItem,
        QVBoxLayout,
        QFileDialog,
        QWidget,
    )
except Exception:  # pragma: no cover - headless/legacy environments.
    Qt = None
    QEvent = None

import canopen_eds_cc as backend


DEFAULT_PROJECT_NAME = "canopen_dictionary"
TOOL_CONFIG_FILE = "canopen_eds_cc_gui_config.json"
DEFAULT_CANOPEN_PROFILE_FILES = [
    "core/protocol/canopen/cia301/cia301_project.json",
    "core/protocol/canopen/cia401/cia401_project.json",
    "core/protocol/canopen/cia402/cia402_project.json",
]


def _default_gui_config() -> dict[str, Any]:
    return {
        "quick_open_projects": DEFAULT_CANOPEN_PROFILE_FILES,
        "quick_open": {
            "title": "Current Project Files",
            "visible": True,
            "width": 280,
        },
    }

_DATA_TYPES: list[str] = []
for name, code in backend.TYPE_NAME_TO_CODE.items():
    if not isinstance(code, int):
        continue
    if not isinstance(name, str):
        continue
    if name.startswith("GMP_CANOPEN_OD_"):
        continue
    _DATA_TYPES.append((int(code), name))
_DATA_TYPES.sort(key=lambda item: (item[0], item[1]))
DATA_TYPE_OPTIONS = [item[1] for item in _DATA_TYPES]

CANONICAL_DATA_TYPE_NAME = {}
for code, name in _DATA_TYPES:
    CANONICAL_DATA_TYPE_NAME.setdefault(code, name)

ROLE_KIND = Qt.UserRole if Qt else 0x0100
ROLE_ENTRY_ID = Qt.UserRole + 1 if Qt else 0x0101
ROLE_ENTRY_DATA = Qt.UserRole + 2 if Qt else 0x0102
ROLE_INDEX_KEY = Qt.UserRole + 3 if Qt else 0x0103

ENTITY_COLUMN_OBJECT = 0
ENTITY_COLUMN_INDEX = 1
ENTITY_COLUMN_SUBINDEX = 2
ENTITY_COLUMN_DATA_TYPE = 3
ENTITY_COLUMN_ACCESS = 4
ENTITY_COLUMN_STORAGE = 5
ENTITY_COLUMN_PDO = 6
ENTITY_COLUMN_SIZE = 7
ENTITY_COLUMN_VARIABLE = 8
ENTITY_COLUMN_COMMENT = 9


def _resolve_gmp_root() -> Path:
    env_root = os.getenv("GMP_PRO_LOCATION", "").strip()
    if env_root:
        candidate = Path(env_root).resolve()
        if (candidate / "tools" / "gmp_installer").exists():
            return candidate
    candidate = Path(__file__).resolve().parents[3]
    if (candidate / "tools" / "gmp_installer").exists():
        return candidate
    return Path(__file__).resolve().parents[0]


def _load_gui_config(gmp_root: Path) -> dict[str, Any]:
    config_path = gmp_root / "tools" / "protocol_mgr" / "canopen_eds_cc" / TOOL_CONFIG_FILE
    if not config_path.exists():
        return _default_gui_config()
    try:
        with config_path.open("r", encoding="utf-8") as fp:
            raw = json.load(fp)
    except Exception:
        return _default_gui_config()
    if not isinstance(raw, dict):
        return _default_gui_config()
    config = _default_gui_config()
    quick_open = raw.get("quick_open_projects")
    if isinstance(quick_open, list):
        cleaned = [str(item).strip() for item in quick_open if str(item).strip()]
        if cleaned:
            config["quick_open_projects"] = cleaned
    quick_open_cfg = raw.get("quick_open")
    if isinstance(quick_open_cfg, dict):
        width = quick_open_cfg.get("width", config["quick_open"]["width"])
        try:
            width = int(width)
        except Exception:
            width = int(config["quick_open"]["width"])
        config["quick_open"] = {
            "title": str(quick_open_cfg.get("title", config["quick_open"]["title"])).strip() or config["quick_open"]["title"],
            "visible": bool(quick_open_cfg.get("visible", config["quick_open"]["visible"])),
            "width": width,
        }
    return config


def _project_paths_from_config(config: dict[str, Any], *, gmp_root: Path) -> list[Path]:
    items = config.get("quick_open_projects")
    if not isinstance(items, list):
        return [ (gmp_root / Path(item)).resolve() for item in DEFAULT_CANOPEN_PROFILE_FILES]
    paths = []
    for raw in items:
        text = str(raw).strip()
        if not text:
            continue
        path = Path(text)
        if not path.is_absolute():
            path = gmp_root / path
        paths.append(path.resolve())
    if not paths:
        return [ (gmp_root / Path(item)).resolve() for item in DEFAULT_CANOPEN_PROFILE_FILES]
    return paths


def _to_int_expression(value: str) -> int:
    return backend.parse_int_expression(value or "0")


def _format_index(value: int) -> str:
    return f"0x{value:04X}"


def _format_subindex(value: int) -> str:
    return f"0x{value:02X}"


def _parse_group_path(text: str) -> list[str]:
    return [part.strip() for part in str(text).split("/") if part.strip()]


def _compose_group_path(parts: list[str]) -> str:
    return "/".join(parts)


def _format_row_size(data_type: int) -> int | None:
    info = backend.TYPE_INFO.get(data_type)
    if info is None:
        return None
    fixed_size = info[3]
    return fixed_size if fixed_size > 0 else None


def _default_variable_name(index_text: str, subindex_text: str) -> str:
    try:
        index = _to_int_expression(index_text)
        subindex = _to_int_expression(subindex_text)
    except Exception:
        return ""
    return f"gmp_canopen_od_{index:04X}_{subindex:02X}"


def _normalize_access_value(raw: str) -> str:
    value = str(raw).strip().lower()
    compact = value.replace(" ", "").replace("-", "").replace("_", "").replace("|", "").replace("/", "")
    if compact in {"const", "c"}:
        return "const"
    if compact in {"r", "ro", "readonly", "read", "readable"}:
        return "ro"
    if compact in {"w", "wo", "write", "writeonly"}:
        return "wo"
    if compact in {"rw", "rwr", "rww", "readwrite"}:
        return "rw"
    if value in {"ro", "wo", "rw", "const"}:
        return value
    return "ro"


def _normalize_access_display(raw: str) -> str:
    return _normalize_access_value(raw)


def _access_short_label(raw: str) -> str:
    value = _normalize_access_value(raw)
    if value == "const":
        return "C"
    if value == "rw":
        return "R/W"
    if value == "wo":
        return "W"
    return "R"


def _access_short_label_fixed(raw: str, width: int = 3) -> str:
    return _access_short_label(raw).ljust(width)


class AccessEditorDelegate(QStyledItemDelegate):
    """Use a combo + const check box for ACCESS field editing."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

    def createEditor(self, parent: QWidget, option, index) -> QWidget:  # type: ignore[override]
        editor = QWidget(parent)
        layout = QHBoxLayout(editor)
        layout.setContentsMargins(0, 0, 0, 0)
        combo = QComboBox(editor)
        combo.addItems(["R", "W", "R/W"])
        const_check = QCheckBox("Const", editor)
        layout.addWidget(combo, 1)
        layout.addWidget(const_check)
        editor.combo = combo  # type: ignore[attr-defined]
        editor.const_check = const_check  # type: ignore[attr-defined]
        return editor

    def setEditorData(self, editor: QWidget, index) -> None:  # type: ignore[override]
        text = str(index.model().data(index, Qt.ItemDataRole.EditRole) or "").strip().lower()
        normalized = _normalize_access_display(text)
        is_const = normalized == "const"
        if hasattr(editor, "combo"):
            combo = editor.combo  # type: ignore[attr-defined]
            combo.setCurrentText("R/W" if normalized == "rw" else ("W" if normalized == "wo" else "R"))
        if hasattr(editor, "const_check"):
            editor.const_check.setChecked(is_const)  # type: ignore[attr-defined]
            if is_const and hasattr(editor, "combo"):
                editor.combo.setDisabled(True)  # type: ignore[attr-defined]
            elif hasattr(editor, "combo"):
                editor.combo.setDisabled(False)  # type: ignore[attr-defined]

    def setModelData(self, editor: QWidget, model, index) -> None:  # type: ignore[override]
        is_const = False
        combo_value = "ro"
        if hasattr(editor, "const_check"):
            is_const = bool(editor.const_check.isChecked())  # type: ignore[attr-defined]
        if hasattr(editor, "combo"):
            combo_value = str(editor.combo.currentText())  # type: ignore[attr-defined]
        if is_const:
            value = "const"
        else:
            value = _normalize_access_value(combo_value)
        model.setData(index, backend.parse_access(value), Qt.ItemDataRole.EditRole)

    def updateEditorGeometry(self, editor: QWidget, option, index) -> None:  # type: ignore[override]
        editor.setGeometry(option.rect)
def _normalize_import_item(
    item: dict[str, Any],
    *,
    default_storage: str = "pointer",
    default_node: int = 1,
    default_id: str = "",
) -> dict[str, Any]:
    path = str(item.get("path", "")).strip()
    alias = str(item.get("alias", "")).strip() or (Path(path).stem if path else "")
    storage = str(item.get("storage", default_storage)).strip() or default_storage
    try:
        node_id = int(item.get("node_id", item.get("node-id", default_node)))
    except (TypeError, ValueError):
        node_id = default_node
    return {
        "path": path,
        "alias": alias,
        "storage": storage,
        "node_id": max(1, node_id),
        "import_id": str(item.get("import_id", default_id) or default_id),
    }


def _entry_to_row(entry: backend.DictionaryEntry) -> dict[str, Any]:
    return {
        "index_text": _format_index(entry.index),
        "subindex_text": _format_subindex(entry.subindex),
        "parameter_name": entry.parameter_name,
        "data_type": CANONICAL_DATA_TYPE_NAME.get(entry.data_type, f"0x{entry.data_type:04X}"),
        "access": entry.access,
        "pdo_mapping": bool(entry.pdo_mapping),
        "default_value": entry.default_value,
        "size": int(entry.size),
        "storage": entry.storage,
        "variable_name": entry.variable_name or "",
        "group_path": _compose_group_path(list(entry.group_path)),
        "comment": entry.comment or "",
    }


def _row_to_entry_payload(row: dict[str, Any]) -> dict[str, Any]:
    index = backend.parse_int_expression(str(row["index_text"]))
    subindex = backend.parse_int_expression(str(row["subindex_text"]), 1)
    data_type = backend.parse_data_type(row["data_type"])
    fixed_size = _format_row_size(data_type)
    size = int(str(row.get("size", 0)))
    if fixed_size is not None:
        size = fixed_size
    payload = {
        "index": _format_index(index),
        "subindex": _format_subindex(subindex),
        "parameter_name": str(row.get("parameter_name", "")).strip(),
        "data_type": _format_index(data_type),
        "access": backend.parse_access(str(row.get("access", "ro"))),
        "pdo_mapping": bool(row.get("pdo_mapping", False)),
        "default_value": str(row.get("default_value", "0")).strip() or "0",
        "size": size,
        "storage": str(row.get("storage", "pointer")).strip() or "pointer",
        "group_path": _parse_group_path(str(row.get("group_path", ""))),
    }
    variable_name = str(row.get("variable_name", "")).strip()
    if payload["storage"] in {"pointer", "value", "variable"}:
        if payload["storage"] == "variable":
            payload["variable_name"] = variable_name or _default_variable_name(payload["index"], payload["subindex"])
        elif variable_name:
            payload["variable_name"] = variable_name
    if row.get("comment", "") not in {None, ""}:
        payload["comment"] = str(row["comment"])
    return payload


def _normalize_entries(entries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    normalized: list[dict[str, Any]] = []
    for row in entries:
        if isinstance(row, dict):
            normalized.append({
                "index_text": str(row.get("index_text", "0x2000")).strip(),
                "subindex_text": str(row.get("subindex_text", "0x00")).strip(),
                "parameter_name": str(row.get("parameter_name", "")).strip(),
                "data_type": str(row.get("data_type", DATA_TYPE_OPTIONS[0] if DATA_TYPE_OPTIONS else "UNSIGNED8")).strip(),
                "access": str(row.get("access", "ro")).strip(),
                "pdo_mapping": bool(row.get("pdo_mapping", False)),
                "default_value": str(row.get("default_value", "0")).strip() or "0",
                "size": int(row.get("size", 0)),
                "storage": str(row.get("storage", "pointer")).strip() or "pointer",
                "variable_name": str(row.get("variable_name", "")).strip(),
                "group_path": str(row.get("group_path", "")) if not isinstance(row.get("group_path", ""), list) else _compose_group_path(row["group_path"]),
                "comment": str(row.get("comment", "")),
            })
    return normalized


def _new_default_entry() -> dict[str, Any]:
    return {
        "index_text": "0x2000",
        "subindex_text": "0x00",
        "parameter_name": "",
        "data_type": DATA_TYPE_OPTIONS[0] if DATA_TYPE_OPTIONS else "UNSIGNED8",
        "access": "ro",
        "pdo_mapping": False,
        "default_value": "0",
        "size": 4,
        "storage": "pointer",
        "variable_name": "",
        "group_path": "",
        "comment": "",
    }


class _EntryEditor(QDialog):
    def __init__(self, parent: QWidget, row: dict[str, Any] | None = None):
        super().__init__(parent)
        self.setWindowTitle("Edit Entry" if row else "New Entry")
        self.result: dict[str, Any] | None = None
        self._row = dict(_new_default_entry())
        if row is not None:
            self._row.update(row)
        self._is_variable_name_auto = self._row.get("storage") == "variable"
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        form = QFormLayout()

        self.index_edit = QLineEdit(self._row["index_text"])
        self.subindex_edit = QLineEdit(self._row["subindex_text"])
        self.parameter_edit = QLineEdit(self._row["parameter_name"])
        self.type_combo = QComboBox()
        self.type_combo.addItems(DATA_TYPE_OPTIONS)
        if self._row["data_type"] in DATA_TYPE_OPTIONS:
            self.type_combo.setCurrentText(self._row["data_type"])
        else:
            self.type_combo.setCurrentText(DATA_TYPE_OPTIONS[0] if DATA_TYPE_OPTIONS else "UNSIGNED8")

        self.read_checkbox = QCheckBox("Read")
        self.write_checkbox = QCheckBox("Write")
        self.const_checkbox = QCheckBox("Const")
        self._apply_access_state(self._row["access"])
        access_container = QWidget()
        access_layout = QHBoxLayout(access_container)
        access_layout.setContentsMargins(0, 0, 0, 0)
        access_layout.addWidget(self.read_checkbox)
        access_layout.addWidget(self.write_checkbox)
        access_layout.addWidget(self.const_checkbox)

        self.storage_combo = QComboBox()
        self.storage_combo.addItems(["pointer", "value", "variable"])
        self.storage_combo.setCurrentText(self._row["storage"])

        self.default_edit = QLineEdit(self._row["default_value"])
        self.size_spin = QSpinBox()
        self.size_spin.setRange(0, 65535)
        self.size_spin.setValue(int(self._row["size"] or 0))
        self.size_spin.setSingleStep(1)

        self.pdo_checkbox = QCheckBox("PDO Mapping")
        self.pdo_checkbox.setChecked(bool(self._row.get("pdo_mapping", False)))
        self.variable_edit = QLineEdit(self._row.get("variable_name", ""))
        self.group_edit = QLineEdit(self._row.get("group_path", ""))
        self.comment_edit = QLineEdit(self._row.get("comment", ""))

        form.addRow("Index", self.index_edit)
        form.addRow("Subindex", self.subindex_edit)
        form.addRow("Parameter Name", self.parameter_edit)
        form.addRow("Data Type", self.type_combo)
        form.addRow("Access", access_container)
        form.addRow("Storage", self.storage_combo)
        form.addRow("Default Value", self.default_edit)
        form.addRow("Size", self.size_spin)
        form.addRow("Variable Name", self.variable_edit)
        form.addRow("", self.pdo_checkbox)
        form.addRow("Group Path", self.group_edit)
        form.addRow("Comment", self.comment_edit)

        self.size_spin.setToolTip("Size in bytes. Fixed data types are auto-updated.")
        self.variable_edit.setPlaceholderText("gmp_canopen_od_XXXX_YY")
        self.variable_edit.editingFinished.connect(self._on_variable_edited)

        for widget in (
            self.index_edit,
            self.subindex_edit,
            self.default_edit,
            self.group_edit,
            self.comment_edit,
            self.variable_edit,
        ):
            widget.textChanged.connect(self._auto_update_fields)

        self.type_combo.currentIndexChanged.connect(self._on_data_type_changed)
        self.const_checkbox.stateChanged.connect(self._on_const_toggled)
        self.storage_combo.currentTextChanged.connect(self._on_storage_changed)
        self._on_data_type_changed()
        self._auto_update_fields()

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=self)
        buttons.accepted.connect(self._on_accept)
        buttons.rejected.connect(self.reject)

        layout.addLayout(form)
        layout.addWidget(buttons)

    def _apply_access_state(self, access: str) -> None:
        value = str(access).strip().lower()
        if value == "const":
            self.read_checkbox.setChecked(True)
            self.write_checkbox.setChecked(False)
            self.const_checkbox.setChecked(True)
            return
        self.const_checkbox.setChecked(False)
        if value == "rw" or value == "rwr" or value == "rww":
            self.read_checkbox.setChecked(True)
            self.write_checkbox.setChecked(True)
        elif value == "wo":
            self.read_checkbox.setChecked(False)
            self.write_checkbox.setChecked(True)
        else:
            self.read_checkbox.setChecked(True)
            self.write_checkbox.setChecked(False)

    def _access_value(self) -> str:
        if self.const_checkbox.isChecked():
            return "const"
        if self.read_checkbox.isChecked() and self.write_checkbox.isChecked():
            return "rw"
        if self.write_checkbox.isChecked() and not self.read_checkbox.isChecked():
            return "wo"
        if self.read_checkbox.isChecked():
            return "ro"
        return "ro"

    def _on_const_toggled(self) -> None:
        if self.const_checkbox.isChecked():
            self.read_checkbox.setChecked(True)
            self.read_checkbox.setEnabled(False)
            self.write_checkbox.setChecked(False)
            self.write_checkbox.setEnabled(False)
        else:
            self.read_checkbox.setEnabled(True)
            self.write_checkbox.setEnabled(True)

    def _on_data_type_changed(self) -> None:
        try:
            code = backend.parse_data_type(self.type_combo.currentText())
            fixed_size = _format_row_size(code)
        except Exception:
            fixed_size = None
        if fixed_size is not None:
            self.size_spin.setValue(fixed_size)
            self.size_spin.setEnabled(False)
        else:
            self.size_spin.setEnabled(True)

    def _is_default_variable_name(self, value: str, index_text: str, subindex_text: str) -> bool:
        if not value:
            return True
        return value.strip() == _default_variable_name(index_text, subindex_text)

    def _auto_update_fields(self) -> None:
        if self.storage_combo.currentText().strip() != "variable":
            return
        index_text = self.index_edit.text().strip()
        subindex_text = self.subindex_edit.text().strip()
        expected = _default_variable_name(index_text, subindex_text)
        if not expected:
            return
        current = self.variable_edit.text().strip()
        if self._is_variable_name_auto and self._is_default_variable_name(current, index_text, subindex_text):
            self.variable_edit.setText(expected)
        elif not current:
            self.variable_edit.setText(expected)
            self._is_variable_name_auto = True

    def _on_storage_changed(self) -> None:
        is_variable = self.storage_combo.currentText() == "variable"
        if is_variable:
            self.variable_edit.setEnabled(True)
            self._is_variable_name_auto = (
                not self.variable_edit.text().strip()
                or self._is_default_variable_name(
                    self.variable_edit.text().strip(),
                    self.index_edit.text().strip(),
                    self.subindex_edit.text().strip(),
                )
            )
            if self._is_variable_name_auto:
                self.variable_edit.setText(_default_variable_name(self.index_edit.text().strip(), self.subindex_edit.text().strip()))
        else:
            self.variable_edit.setEnabled(False)
            self._is_variable_name_auto = False

    def _on_variable_edited(self) -> None:
        self._is_variable_name_auto = self._is_default_variable_name(
            self.variable_edit.text().strip(),
            self.index_edit.text().strip(),
            self.subindex_edit.text().strip(),
        )

    def _on_accept(self) -> None:
        try:
            index = _to_int_expression(self.index_edit.text())
            subindex = backend.parse_int_expression(self.subindex_edit.text(), 1)
            if not (0 <= index <= 0xFFFF):
                raise ValueError("Index must be in [0x0000, 0xFFFF]")
            if not (0 <= subindex <= 0xFF):
                raise ValueError("Subindex must be in [0x00, 0xFF]")

            data_type = self.type_combo.currentText().strip() or (DATA_TYPE_OPTIONS[0] if DATA_TYPE_OPTIONS else "UNSIGNED8")
            code = backend.parse_data_type(data_type)
            backend.parse_access(self._access_value())
            size = int(self.size_spin.value())
            fixed_size = _format_row_size(code)
            if fixed_size is not None:
                size = fixed_size
            elif size <= 0:
                raise ValueError("Size must be > 0")

            storage = self.storage_combo.currentText().strip() or "pointer"
            backend.parse_storage(storage, self.parent()._default_storage if hasattr(self.parent(), "_default_storage") else "pointer")
            variable_name = self.variable_edit.text().strip()
            if storage == "variable" and not variable_name:
                variable_name = _default_variable_name(self.index_edit.text().strip(), self.subindex_edit.text().strip())

            if storage == "variable" and variable_name:
                backend.ID_RE.match(variable_name)

            self.result = {
                "index_text": _format_index(index),
                "subindex_text": _format_subindex(subindex),
                "parameter_name": self.parameter_edit.text().strip(),
                "data_type": data_type,
                "access": self._access_value(),
                "pdo_mapping": bool(self.pdo_checkbox.isChecked()),
                "default_value": self.default_edit.text().strip() or "0",
                "size": size,
                "storage": storage,
                "variable_name": variable_name,
                "group_path": self.group_edit.text().strip(),
                "comment": self.comment_edit.text().strip(),
            }
            self.accept()
        except Exception as error:
            QMessageBox.warning(self, "Invalid Entry", str(error))


class _ImportEditor(QDialog):
    def __init__(self, parent: QWidget, item: dict[str, Any], node_id: int, default_storage: str):
        super().__init__(parent)
        self.setWindowTitle("Edit Import")
        self.result: dict[str, Any] | None = None
        self._build_ui(item, node_id, default_storage)

    def _build_ui(self, item: dict[str, Any], node_id: int, default_storage: str) -> None:
        layout = QVBoxLayout(self)
        form = QFormLayout()
        self.alias_edit = QLineEdit(item.get("alias", ""))
        self.storage_combo = QComboBox()
        self.storage_combo.addItems(["pointer", "value", "variable"])
        self.storage_combo.setCurrentText(str(item.get("storage", default_storage)))
        self.node_spin = QSpinBox()
        self.node_spin.setRange(1, 127)
        self.node_spin.setValue(int(item.get("node_id", node_id)))
        form.addRow("Alias", self.alias_edit)
        form.addRow("Storage", self.storage_combo)
        form.addRow("Node ID", self.node_spin)
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=self)
        button_box.accepted.connect(self._on_accept)
        button_box.rejected.connect(self.reject)
        layout.addLayout(form)
        layout.addWidget(button_box)

    def _on_accept(self) -> None:
        self.result = {
            "alias": self.alias_edit.text().strip(),
            "storage": self.storage_combo.currentText().strip(),
            "node_id": int(self.node_spin.value()),
        }
        self.accept()


class CanopenEdsGui(QMainWindow):
    def __init__(self, project_path: Path | None = None) -> None:
        super().__init__()
        self.gmp_root = _resolve_gmp_root()
        self.gui_config = _load_gui_config(self.gmp_root)
        self.quick_open_paths = _project_paths_from_config(self.gui_config, gmp_root=self.gmp_root)
        self.project_path: Path | None = None
        self.current_preview_path: Path | None = None
        self.preset_dir = self.gmp_root / "tools" / "protocol_mgr" / "canopen_eds_cc" / "presets"
        self.preset_dir.mkdir(parents=True, exist_ok=True)

        self._dirty = False
        self._entry_id_counter = 0
        self._block_dirty = False
        self._rebuilding_entries = False
        self._default_storage = "pointer"

        self.project: dict[str, Any] = {
            "version": 1,
            "name": DEFAULT_PROJECT_NAME,
            "node_id": 1,
            "default_storage": "pointer",
            "conflict_policy": "last",
            "output_dir": None,
            "imports": [],
            "entries": [],
            "code": {
                "header_prefix": "",
                "header_suffix": "",
                "source_prefix": "",
                "source_suffix": "",
            },
        }
        self.entries: list[dict[str, Any]] = []
        self.imports: list[dict[str, Any]] = []

        self._build_ui()
        if project_path is not None:
            self.load_project(project_path)
        else:
            self._apply_project_state()

    # -------------------- UI --------------------
    def _build_ui(self) -> None:
        self.setWindowTitle("canopen_eds_cc GUI")
        self.resize(1280, 840)

        self._build_actions()
        self._build_menus()
        self._build_toolbar()

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        top = QWidget()
        top_layout = QFormLayout(top)
        self.project_path_label = QLabel("(unsaved)")
        self.name_edit = QLineEdit(self.project["name"])
        self.node_spin = QSpinBox()
        self.node_spin.setRange(1, 127)
        self.node_spin.setValue(int(self.project["node_id"]))
        self.default_storage_combo = QComboBox()
        self.default_storage_combo.addItems(["pointer", "value", "variable"])
        self.default_storage_combo.setCurrentText(self.project["default_storage"])
        self.conflict_combo = QComboBox()
        self.conflict_combo.addItems(["error", "first", "last"])
        self.conflict_combo.setCurrentText(self.project["conflict_policy"])
        self.output_edit = QLineEdit(self.project.get("output_dir") or "")
        self.output_browse = QPushButton("Browse")
        self.output_browse.clicked.connect(self.browse_output_dir)

        output_row = QWidget()
        output_layout = QHBoxLayout(output_row)
        output_layout.setContentsMargins(0, 0, 0, 0)
        output_layout.addWidget(self.output_edit)
        output_layout.addWidget(self.output_browse)

        top_layout.addRow("Project", self.project_path_label)
        top_layout.addRow("Name", self.name_edit)
        top_layout.addRow("Node ID", self.node_spin)
        top_layout.addRow("Default storage", self.default_storage_combo)
        top_layout.addRow("Conflict policy", self.conflict_combo)
        top_layout.addRow("Output directory", output_row)

        preset_panel = QWidget()
        preset_layout = QHBoxLayout(preset_panel)
        preset_layout.setContentsMargins(0, 0, 0, 0)
        preset_layout.addWidget(QLabel("Preset"))
        self.preset_combo = QComboBox()
        self.preset_combo.setEditable(False)
        self.preset_combo.addItem("")
        preset_layout.addWidget(self.preset_combo, 1)
        self.preset_load_btn = QPushButton("Load Preset")
        self.preset_save_btn = QPushButton("Save Preset")
        self.preset_del_btn = QPushButton("Delete Preset")
        self.preset_load_btn.clicked.connect(self.load_preset)
        self.preset_save_btn.clicked.connect(self.save_preset)
        self.preset_del_btn.clicked.connect(self.delete_preset)
        preset_layout.addWidget(self.preset_load_btn)
        preset_layout.addWidget(self.preset_save_btn)
        preset_layout.addWidget(self.preset_del_btn)

        root.addWidget(top)
        root.addWidget(preset_panel)

        self._build_quick_open_panel()

        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.setChildrenCollapsible(False)
        root.addWidget(splitter, 1)

        # Entities (tree-table merged page, appears before Imports)
        entities_box = QGroupBox("Entities")
        entities_layout = QVBoxLayout(entities_box)
        entity_buttons = QWidget()
        entity_button_layout = QHBoxLayout(entity_buttons)
        entity_button_layout.setContentsMargins(0, 0, 0, 0)
        self.add_entry_btn = QPushButton("Add Entry")
        self.edit_entry_btn = QPushButton("Edit")
        self.delete_entry_btn = QPushButton("Delete")
        self.import_entry_btn = QPushButton("Append from EDS")
        self.replace_entry_btn = QPushButton("Replace from EDS")
        self.add_entry_btn.clicked.connect(self.add_entry)
        self.edit_entry_btn.clicked.connect(self.edit_entry)
        self.delete_entry_btn.clicked.connect(self.delete_entry)
        self.import_entry_btn.clicked.connect(self.append_raw_eds_entries)
        self.replace_entry_btn.clicked.connect(self.replace_project_from_eds)
        entity_button_layout.addWidget(self.add_entry_btn)
        entity_button_layout.addWidget(self.edit_entry_btn)
        entity_button_layout.addWidget(self.delete_entry_btn)
        entity_button_layout.addWidget(self.import_entry_btn)
        entity_button_layout.addWidget(self.replace_entry_btn)
        entity_button_layout.addStretch()
        entities_layout.addWidget(entity_buttons)

        self.entity_tree = QTreeWidget()
        self.entity_tree.setColumnCount(10)
        self.entity_tree.setHeaderLabels(
            ["Object", "Index", "Sub", "Type", "Access", "Storage", "PDO", "Size", "Variable", "Comment"]
        )
        self.access_font = QFont(self.entity_tree.font())
        self.access_font.setFamily("Consolas")
        self.entity_tree.setItemDelegateForColumn(ENTITY_COLUMN_ACCESS, AccessEditorDelegate(self.entity_tree))
        self.entity_tree.setEditTriggers(
            QAbstractItemView.EditTrigger.DoubleClicked | QAbstractItemView.EditTrigger.EditKeyPressed
        )
        self.entity_tree.header().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.entity_tree.header().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.entity_tree.header().setSectionResizeMode(ENTITY_COLUMN_ACCESS, QHeaderView.ResizeMode.Fixed)
        self.entity_tree.header().resizeSection(ENTITY_COLUMN_ACCESS, 64)
        self.entity_tree.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.entity_tree.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.entity_tree.customContextMenuRequested.connect(self._show_entity_context_menu)
        self.entity_tree.itemChanged.connect(self._on_entry_item_changed)
        self.entity_tree.itemSelectionChanged.connect(self._on_selection_changed)
        entities_layout.addWidget(self.entity_tree)
        splitter.addWidget(entities_box)

        # Imports
        imports_box = QGroupBox("Imports")
        imports_layout = QVBoxLayout(imports_box)
        import_buttons = QWidget()
        import_button_layout = QHBoxLayout(import_buttons)
        import_button_layout.setContentsMargins(0, 0, 0, 0)
        self.add_import_btn = QPushButton("Add EDS...")
        self.edit_import_btn = QPushButton("Edit")
        self.remove_import_btn = QPushButton("Remove")
        self.up_import_btn = QPushButton("Move Up")
        self.down_import_btn = QPushButton("Move Down")
        self.refresh_import_btn = QPushButton("Refresh")
        self.add_import_btn.clicked.connect(self.import_from_eds)
        self.edit_import_btn.clicked.connect(self.edit_selected_import)
        self.remove_import_btn.clicked.connect(self.remove_import)
        self.up_import_btn.clicked.connect(lambda: self.move_selected_import(-1))
        self.down_import_btn.clicked.connect(lambda: self.move_selected_import(1))
        self.refresh_import_btn.clicked.connect(self.refresh_entry_table)
        import_button_layout.addWidget(self.add_import_btn)
        import_button_layout.addWidget(self.edit_import_btn)
        import_button_layout.addWidget(self.remove_import_btn)
        import_button_layout.addWidget(self.up_import_btn)
        import_button_layout.addWidget(self.down_import_btn)
        import_button_layout.addWidget(self.refresh_import_btn)
        import_button_layout.addStretch()
        imports_layout.addWidget(import_buttons)

        self.import_table = QTableWidget(0, 4)
        self.import_table.setHorizontalHeaderLabels(["EDS Path", "Alias", "Storage", "Node ID"])
        self.import_table.horizontalHeader().setStretchLastSection(True)
        self.import_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.import_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.import_table.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.import_table.customContextMenuRequested.connect(self._show_import_context_menu)
        self.import_table.cellDoubleClicked.connect(lambda *_args: self.edit_selected_import())
        imports_layout.addWidget(self.import_table)
        splitter.addWidget(imports_box)

        action_row = QWidget()
        action_layout = QHBoxLayout(action_row)
        action_layout.setContentsMargins(0, 0, 0, 0)
        self.generate_btn = QPushButton("Generate .h/.c")
        self.generate_btn.clicked.connect(self.generate)
        action_layout.addWidget(self.generate_btn)
        action_layout.addStretch()
        root.addWidget(action_row)

        status = QStatusBar()
        self.setStatusBar(status)
        status.showMessage("Ready")

        self._wire_project_signals()
        self._refresh_preset_list()
        self._refresh_quick_open_projects()
        quick_open = self.gui_config.get("quick_open", {})
        if quick_open.get("visible", True):
            self.quick_open_dock.show()
        else:
            self.quick_open_dock.hide()

    def _build_actions(self) -> None:
        self.new_action = QAction("New", self)
        self.new_action.setShortcut(QKeySequence("Ctrl+N"))
        self.new_action.triggered.connect(self.new_project)

        self.open_action = QAction("Open...", self)
        self.open_action.setShortcut(QKeySequence("Ctrl+O"))
        self.open_action.triggered.connect(self.open_project)

        self.save_action = QAction("Save", self)
        self.save_action.setShortcut(QKeySequence("Ctrl+S"))
        self.save_action.triggered.connect(self.save_project)

        self.save_as_action = QAction("Save As...", self)
        self.save_as_action.setShortcut(QKeySequence("Ctrl+Shift+S"))
        self.save_as_action.triggered.connect(self.save_project_as)

        self.exit_action = QAction("Exit", self)
        self.exit_action.setShortcut(QKeySequence("Ctrl+Q"))
        self.exit_action.triggered.connect(self.close)

        self.add_entry_action = QAction("Add Entry", self)
        self.add_entry_action.setShortcut(QKeySequence("Ins"))
        self.add_entry_action.triggered.connect(self.add_entry)

        self.edit_entry_action = QAction("Edit Entry", self)
        self.edit_entry_action.setShortcut(QKeySequence("Return"))
        self.edit_entry_action.triggered.connect(self.edit_entry)

        self.delete_entry_action = QAction("Delete Entry", self)
        self.delete_entry_action.setShortcut(QKeySequence("Del"))
        self.delete_entry_action.triggered.connect(self.delete_entry)

        self.append_eds_action = QAction("Append from EDS", self)
        self.append_eds_action.setShortcut(QKeySequence("Ctrl+I"))
        self.append_eds_action.triggered.connect(self.append_raw_eds_entries)

        self.generate_action = QAction("Generate", self)
        self.generate_action.setShortcut(QKeySequence("Ctrl+G"))
        self.generate_action.triggered.connect(self.generate)

        self.refresh_action = QAction("Refresh View", self)
        self.refresh_action.setShortcut(QKeySequence("F5"))
        self.refresh_action.triggered.connect(self.refresh_entry_table)

    def _build_quick_open_panel(self) -> None:
        quick_title = str(self.gui_config.get("quick_open", {}).get("title", "Project files")).strip() or "Project files"
        self.quick_open_dock = QDockWidget(quick_title, self)
        self.quick_open_dock.setObjectName("quick_open_projects")
        self.quick_open_dock.setAllowedAreas(Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea)
        self.quick_open_dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetClosable
            | QDockWidget.DockWidgetFeature.DockWidgetMovable
            | QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )

        quick_widget = QWidget()
        quick_layout = QVBoxLayout(quick_widget)
        quick_layout.setContentsMargins(0, 0, 0, 0)
        quick_layout.setSpacing(4)

        hint = QLabel("Double click to open a project.")
        hint.setWordWrap(True)
        quick_layout.addWidget(hint)

        self.quick_open_list = QListWidget()
        self.quick_open_list.itemDoubleClicked.connect(self._open_project_from_quick_list)
        self.quick_open_list.itemSelectionChanged.connect(self._sync_quick_open_selection)
        self.quick_open_list.setAlternatingRowColors(True)
        self.quick_open_list.setMinimumWidth(int(self.gui_config.get("quick_open", {}).get("width", 280)))
        quick_layout.addWidget(self.quick_open_list, 1)

        quick_button_row = QWidget()
        quick_button_layout = QHBoxLayout(quick_button_row)
        quick_button_layout.setContentsMargins(0, 0, 0, 0)
        self.refresh_quick_open_btn = QPushButton("Refresh")
        self.refresh_quick_open_btn.clicked.connect(self._refresh_quick_open_projects)
        self.open_quick_open_btn = QPushButton("Open")
        self.open_quick_open_btn.clicked.connect(self._open_selected_quick_project)
        quick_button_layout.addWidget(self.refresh_quick_open_btn)
        quick_button_layout.addWidget(self.open_quick_open_btn)
        quick_button_layout.addStretch()
        quick_layout.addWidget(quick_button_row)

        self.quick_open_dock.setWidget(quick_widget)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.quick_open_dock)

    def _quick_open_default_label(self, path: Path) -> str:
        try:
            return str(path.relative_to(self.gmp_root).as_posix())
        except ValueError:
            return str(path)

    def _refresh_quick_open_projects(self) -> None:
        self.quick_open_paths = _project_paths_from_config(self.gui_config, gmp_root=self.gmp_root)
        self.quick_open_list.clear()
        for path in self.quick_open_paths:
            display_name = self._quick_open_default_label(path)
            item = QListWidgetItem(display_name, self.quick_open_list)
            item.setToolTip(str(path))
            item.setData(Qt.ItemDataRole.UserRole, str(path))
            if not path.exists():
                item.setText(f"{display_name} (missing)")
                item.setForeground(self.palette().color(self.quick_open_list.foregroundRole()).lighter(170))
            self.quick_open_list.addItem(item)
        width = int(self.gui_config.get("quick_open", {}).get("width", 280))
        if width > 0:
            self.quick_open_dock.setMinimumWidth(width)
            self.quick_open_dock.resize(width, self.quick_open_dock.height())
        self._sync_quick_open_selection()

    def _sync_quick_open_selection(self) -> None:
        if self.project_path is None:
            self.quick_open_list.clearSelection()
            return
        needle = str(self.project_path.resolve())
        for index in range(self.quick_open_list.count()):
            item = self.quick_open_list.item(index)
            if not item:
                continue
            candidate = str(item.data(Qt.ItemDataRole.UserRole))
            if Path(candidate).resolve() == self.project_path.resolve():
                item.setSelected(True)
                self.quick_open_list.scrollToItem(item)
                return
        self.quick_open_list.clearSelection()

    def _open_project_from_quick_list(self, item: QListWidgetItem) -> None:
        if not self._confirm_unsaved():
            return
        candidate = item.data(Qt.ItemDataRole.UserRole)
        path = Path(str(candidate))
        if not path.exists():
            QMessageBox.warning(self, "Quick Open", f"File not found:\n{path}")
            return
        self.load_project(path)

    def _open_selected_quick_project(self) -> None:
        item = self.quick_open_list.currentItem()
        if item is None:
            return
        self._open_project_from_quick_list(item)

    def _build_menus(self) -> None:
        menu = self.menuBar()
        file_menu = menu.addMenu("File")
        file_menu.addAction(self.new_action)
        file_menu.addAction(self.open_action)
        file_menu.addAction(self.save_action)
        file_menu.addAction(self.save_as_action)
        file_menu.addSeparator()
        file_menu.addAction(self.exit_action)

        edit_menu = menu.addMenu("Edit")
        edit_menu.addAction(self.add_entry_action)
        edit_menu.addAction(self.edit_entry_action)
        edit_menu.addAction(self.delete_entry_action)
        edit_menu.addSeparator()
        edit_menu.addAction(self.append_eds_action)
        edit_menu.addAction(self.refresh_action)

        tool_menu = menu.addMenu("Tools")
        tool_menu.addAction(self.generate_action)

        view_menu = menu.addMenu("View")
        view_menu.addAction(self.quick_open_dock.toggleViewAction())

    def _build_toolbar(self) -> None:
        toolbar = self.addToolBar("Main")
        toolbar.setMovable(False)
        for action in (
            self.new_action,
            self.open_action,
            self.save_action,
            self.save_as_action,
            self.add_entry_action,
            self.edit_entry_action,
            self.delete_entry_action,
            self.append_eds_action,
            self.generate_action,
        ):
            toolbar.addAction(action)

    def _wire_project_signals(self) -> None:
        self.name_edit.textChanged.connect(lambda *_: self._mark_dirty())
        self.node_spin.valueChanged.connect(self._on_node_changed)
        self.default_storage_combo.currentTextChanged.connect(self._on_default_storage_changed)
        self.conflict_combo.currentTextChanged.connect(lambda *_: self._mark_dirty())
        self.output_edit.textChanged.connect(lambda *_: self._mark_dirty())

    # -------------------- Utilities --------------------
    def _on_node_changed(self) -> None:
        self._mark_dirty()
        self._default_storage = self.default_storage_combo.currentText().strip() or "pointer"

    def _on_default_storage_changed(self) -> None:
        self._default_storage = self.default_storage_combo.currentText().strip() or "pointer"
        self._mark_dirty()

    def _mark_dirty(self) -> None:
        if self._block_dirty:
            return
        self._dirty = True
        self._sync_title()

    def _set_dirty(self, dirty: bool) -> None:
        if self._block_dirty:
            return
        self._dirty = dirty
        self._sync_title()

    def _sync_title(self) -> None:
        suffix = " *" if self._dirty else ""
        if self.project_path is None:
            self.setWindowTitle(f"canopen_eds_cc GUI - (unsaved){suffix}")
            self.project_path_label.setText("(unsaved)")
        else:
            self.setWindowTitle(f"canopen_eds_cc GUI - {self.project_path.name}{suffix}")
            self.project_path_label.setText(str(self.project_path))
        self.statusBar().showMessage(f"Current project: {self.project_path or '(unsaved)'}")

    @contextmanager
    def _suspend_dirty(self) -> None:
        self._block_dirty = True
        try:
            yield
        finally:
            self._block_dirty = False

    def _next_entry_id(self) -> str:
        self._entry_id_counter += 1
        return f"entry-{self._entry_id_counter:06d}"

    def _ensure_entry_ids(self) -> None:
        for item in self.entries:
            if not item.get("entry_id"):
                item["entry_id"] = self._next_entry_id()
            else:
                try:
                    value = int(str(item["entry_id"]).replace("entry-", ""))
                    self._entry_id_counter = max(self._entry_id_counter, value)
                except ValueError:
                    pass

    def _payload(self) -> dict[str, Any]:
        sorted_entries = sorted(
            self.entries,
            key=lambda row: (
                backend.parse_int_expression(row["index_text"]),
                backend.parse_int_expression(row["subindex_text"]),
            ),
        )
        return {
            "version": self.project["version"],
            "name": self.name_edit.text().strip() or DEFAULT_PROJECT_NAME,
            "node_id": int(self.node_spin.value()),
            "default_storage": self.default_storage_combo.currentText().strip() or "pointer",
            "conflict_policy": self.conflict_combo.currentText().strip() or "last",
            "output_dir": self.output_edit.text().strip() or None,
            "imports": list(self.imports),
            "code": {
                "header_prefix": "",
                "header_suffix": "",
                "source_prefix": "",
                "source_suffix": "",
            },
            "entries": [_row_to_entry_payload(item) for item in sorted_entries],
        }

    def _project_title(self, path: Path | None = None) -> str:
        if path is None:
            return "(unsaved)"
        return str(path)

    # -------------------- Project IO --------------------
    def _apply_project_state(self) -> None:
        with self._suspend_dirty():
            self.name_edit.setText(self.project["name"])
            self.node_spin.setValue(int(self.project["node_id"]))
            default_storage = self.project.get("default_storage", "pointer")
            self.default_storage_combo.setCurrentText(default_storage)
            self.conflict_combo.setCurrentText(self.project.get("conflict_policy", "last"))
            self.output_edit.setText(self.project.get("output_dir") or "")

            self._default_storage = default_storage
            default_node = int(self.project.get("node_id", 1))
            self.imports = [
                _normalize_import_item(
                    dict(item),
                    default_storage=default_storage,
                    default_node=default_node,
                    default_id=f"import-{idx:04d}",
                )
                for idx, item in enumerate(self.project.get("imports", []))
            ]
            for item in self.imports:
                item.setdefault("import_id", f"import-{self.imports.index(item):04d}")
            self.entries = _normalize_entries(self.project.get("entries", []))
            self._entry_id_counter = 0
            self._ensure_entry_ids()

            self.refresh_entry_table()
            self.refresh_import_table()
            self._set_dirty(False)
            self.refresh_preview()
            self._update_preset_combo()
            self._sync_quick_open_selection()

    def _apply_project_payload(self, payload: dict[str, Any], *, source_path: Path | None = None) -> None:
        payload_copy = dict(payload)
        source_dir = source_path.parent if source_path is not None else self.preset_dir
        normalized_imports = []
        for idx, item in enumerate(payload_copy.get("imports", []) or []):
            if not isinstance(item, dict):
                continue
            normalized = _normalize_import_item(
                dict(item),
                default_storage=payload_copy.get("default_storage", "pointer"),
                default_node=payload_copy.get("node_id", 1),
                default_id=f"import-{idx:04d}",
            )
            if normalized["path"] and not Path(normalized["path"]).is_absolute():
                normalized["path"] = str((source_dir / normalized["path"]).resolve())
            normalized_imports.append(normalized)
        payload_copy["imports"] = normalized_imports

        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False, encoding="utf-8") as stream:
            stream.write(json.dumps(payload_copy, indent=2))
            temp_path = Path(stream.name)
        try:
            loaded = backend.load_project_json(temp_path)
        finally:
            temp_path.unlink(missing_ok=True)

        self.project = {
            "version": loaded.version,
            "name": loaded.name,
            "node_id": loaded.node_id,
            "default_storage": loaded.default_storage,
            "conflict_policy": loaded.conflict_policy,
            "output_dir": loaded.output_dir or "",
            "imports": [item.__dict__ for item in loaded.imports],
            "entries": [_entry_to_row(item) for item in loaded.entries],
            "code": {
                "header_prefix": loaded.code_template.header_prefix,
                "header_suffix": loaded.code_template.header_suffix,
                "source_prefix": loaded.code_template.source_prefix,
                "source_suffix": loaded.code_template.source_suffix,
            },
        }
        self._apply_project_state()

    def new_project(self) -> None:
        if not self._confirm_unsaved():
            return
        self.project = {
            "version": 1,
            "name": DEFAULT_PROJECT_NAME,
            "node_id": 1,
            "default_storage": "pointer",
            "conflict_policy": "last",
            "output_dir": "",
            "imports": [],
            "entries": [],
            "code": {"header_prefix": "", "header_suffix": "", "source_prefix": "", "source_suffix": ""},
        }
        self.project_path = None
        self._apply_project_state()

    def open_project(self) -> None:
        if not self._confirm_unsaved():
            return
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Open CANopen project json",
            str(self.gmp_root),
            "CANopen JSON (*.json);;All Files (*.*)",
        )
        if not path:
            return
        self.load_project(Path(path))

    def load_project(self, path: Path) -> None:
        if not path.exists():
            QMessageBox.critical(self, "Open failed", f"File not found: {path}")
            return
        try:
            loaded = backend.load_project_json(path)
            self.project_path = path
            self.project = {
                "version": loaded.version,
                "name": loaded.name,
                "node_id": loaded.node_id,
                "default_storage": loaded.default_storage,
                "conflict_policy": loaded.conflict_policy,
                "output_dir": loaded.output_dir or "",
                "imports": [item.__dict__ for item in loaded.imports],
                "entries": [_entry_to_row(item) for item in loaded.entries],
                "code": {
                    "header_prefix": loaded.code_template.header_prefix,
                    "header_suffix": loaded.code_template.header_suffix,
                    "source_prefix": loaded.code_template.source_prefix,
                    "source_suffix": loaded.code_template.source_suffix,
                },
            }
            self._apply_project_state()
        except Exception as error:
            QMessageBox.critical(self, "Open failed", str(error))

    def save_project(self) -> bool:
        if self.project_path is None:
            return self.save_project_as()
        payload = self._payload()
        self.project_path.parent.mkdir(parents=True, exist_ok=True)
        self.project_path.write_text(json.dumps(payload, indent=2), encoding="utf-8", newline="\n")
        self.current_preview_path = self.project_path
        self._set_dirty(False)
        return True

    def save_project_as(self) -> bool:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save CANopen project json",
            str(self.gmp_root),
            "CANopen JSON (*.json);;All Files (*.*)",
        )
        if not path:
            return False
        self.project_path = Path(path)
        return self.save_project()

    def _confirm_unsaved(self) -> bool:
        if not self._dirty:
            return True
        reply = QMessageBox.question(
            self,
            "Unsaved changes",
            "Current project has unsaved changes. Save now?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Yes,
        )
        if reply == QMessageBox.StandardButton.Cancel:
            return False
        if reply == QMessageBox.StandardButton.Yes:
            return self.save_project()
        return True

    def browse_output_dir(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "Select output directory", str(self.gmp_root))
        if path:
            self.output_edit.setText(path)

    # -------------------- Presets --------------------
    def _refresh_preset_list(self) -> None:
        names = sorted(item.stem for item in self.preset_dir.glob("*.json"))
        self.preset_combo.blockSignals(True)
        self.preset_combo.clear()
        self.preset_combo.addItem("")
        for item in names:
            self.preset_combo.addItem(item)
        self.preset_combo.blockSignals(False)
        self._set_dirty(self._dirty)

    def _update_preset_combo(self) -> None:
        self._refresh_preset_list()

    def _preset_path(self, name: str) -> Path:
        safe = "".join(ch for ch in name.strip() if ch not in '<>:"/\\|?*')
        return self.preset_dir / f"{safe}.json"

    def save_preset(self) -> None:
        default_name = self.preset_combo.currentText().strip() or self.name_edit.text().strip() or DEFAULT_PROJECT_NAME
        preset_name, ok = QInputDialog.getText(self, "Save Preset", "Preset name:", text=default_name)
        if not ok or not preset_name.strip():
            return
        payload = self._payload()
        path = self._preset_path(preset_name)
        payload = {"kind": "canopen_eds_cc_preset_v1", "name": preset_name.strip(), "project": payload}
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8", newline="\n")
        self.preset_combo.setCurrentText(preset_name.strip())
        self._refresh_preset_list()
        QMessageBox.information(self, "Save Preset", f"Preset saved: {path}")

    def load_preset(self) -> None:
        preset_name = self.preset_combo.currentText().strip()
        if not preset_name:
            return
        path = self._preset_path(preset_name)
        if not path.exists():
            QMessageBox.warning(self, "Load Preset", f"Preset not found: {path}")
            return
        if not self._confirm_unsaved():
            return
        data = json.loads(path.read_text(encoding="utf-8"))
        project = data.get("project", data)
        if not isinstance(project, dict):
            QMessageBox.warning(self, "Load Preset", "Invalid preset format.")
            return
        self.project_path = None
        self._apply_project_payload(project, source_path=path)
        self._set_dirty(False)

    def delete_preset(self) -> None:
        preset_name = self.preset_combo.currentText().strip()
        if not preset_name:
            return
        path = self._preset_path(preset_name)
        if not path.exists():
            QMessageBox.warning(self, "Delete Preset", "Preset not found.")
            return
        reply = QMessageBox.question(
            self,
            "Delete Preset",
            f"Delete preset '{preset_name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        path.unlink()
        self.preset_combo.setCurrentText("")
        self._refresh_preset_list()

    # -------------------- Entries --------------------
    def _on_selection_changed(self) -> None:
        # reserved for future live panel; keep UI responsive in case we add detail panel later.
        return

    def _selected_entry_item(self) -> QTreeWidgetItem | None:
        items = self.entity_tree.selectedItems()
        if not items:
            return None
        item = items[0]
        kind = item.data(0, ROLE_KIND)
        if kind != "entry":
            return None
        return item

    def _selected_entry(self) -> dict[str, Any] | None:
        item = self._selected_entry_item()
        if item is None:
            return None
        entry_id = item.data(0, ROLE_ENTRY_ID)
        return self._find_entry(entry_id)

    def _find_entry(self, entry_id: Any) -> dict[str, Any] | None:
        for row in self.entries:
            if row.get("entry_id") == entry_id:
                return row
        return None

    def _find_entry_index(self, entry_id: Any) -> int:
        for idx, row in enumerate(self.entries):
            if row.get("entry_id") == entry_id:
                return idx
        return -1

    def _select_entry_item_by_id(self, entry_id: Any) -> None:
        if entry_id is None:
            return
        self._rebuilding_entries = True
        try:
            def _walk(node: QTreeWidgetItem | QTreeWidget) -> bool:
                for i in range(node.childCount()):
                    child = node.child(i)
                    if child.data(0, ROLE_KIND) == "entry" and child.data(0, ROLE_ENTRY_ID) == entry_id:
                        self.entity_tree.setCurrentItem(child)
                        self.entity_tree.scrollToItem(child)
                        return True
                    if _walk(child):
                        return True
                return False

            _walk(self.entity_tree.invisibleRootItem())
        finally:
            self._rebuilding_entries = False

    def _set_item_text(self, item: QTreeWidgetItem, column: int, text: str) -> None:
        self._rebuilding_entries = True
        try:
            item.setText(column, text)
        finally:
            self._rebuilding_entries = False

    def _on_entry_item_changed(self, item: QTreeWidgetItem, column: int) -> None:
        if self._rebuilding_entries:
            return
        if item.data(0, ROLE_KIND) != "entry":
            return

        entry_id = item.data(0, ROLE_ENTRY_ID)
        row = self._find_entry(entry_id)
        if row is None:
            return

        old_index = str(row.get("index_text", ""))
        old_subindex = str(row.get("subindex_text", ""))
        old_key = (old_index, old_subindex)
        updated = dict(row)

        try:
            if column == ENTITY_COLUMN_OBJECT:
                updated["parameter_name"] = item.text(ENTITY_COLUMN_OBJECT).strip()
            elif column == ENTITY_COLUMN_INDEX:
                updated["index_text"] = _format_index(backend.parse_int_expression(item.text(ENTITY_COLUMN_INDEX)))
            elif column == ENTITY_COLUMN_SUBINDEX:
                updated["subindex_text"] = _format_subindex(
                    backend.parse_int_expression(item.text(ENTITY_COLUMN_SUBINDEX), 1)
                )
            elif column == ENTITY_COLUMN_DATA_TYPE:
                data_type = backend.parse_data_type(item.text(ENTITY_COLUMN_DATA_TYPE))
                updated["data_type"] = CANONICAL_DATA_TYPE_NAME.get(data_type, f"0x{data_type:04X}")
                fixed_size = _format_row_size(data_type)
                if fixed_size is not None:
                    updated["size"] = fixed_size
            elif column == ENTITY_COLUMN_ACCESS:
                updated["access"] = backend.parse_access(_normalize_access_value(item.text(ENTITY_COLUMN_ACCESS)))
            elif column == ENTITY_COLUMN_STORAGE:
                updated["storage"] = backend.parse_storage(
                    item.text(ENTITY_COLUMN_STORAGE),
                    self.default_storage_combo.currentText().strip() or "pointer",
                )
            elif column == ENTITY_COLUMN_PDO:
                updated["pdo_mapping"] = item.checkState(ENTITY_COLUMN_PDO) == Qt.CheckState.Checked
            elif column == ENTITY_COLUMN_SIZE:
                updated["size"] = int(item.text(ENTITY_COLUMN_SIZE))
            elif column == ENTITY_COLUMN_VARIABLE:
                updated["variable_name"] = item.text(ENTITY_COLUMN_VARIABLE).strip()
            elif column == ENTITY_COLUMN_COMMENT:
                updated["comment"] = item.text(ENTITY_COLUMN_COMMENT).strip()
            else:
                return

            new_index = str(updated.get("index_text", "")).strip()
            new_subindex = str(updated.get("subindex_text", "")).strip()
            if column in {ENTITY_COLUMN_INDEX, ENTITY_COLUMN_SUBINDEX}:
                if not new_index:
                    raise ValueError("Index cannot be empty")
                if not new_subindex:
                    raise ValueError("Subindex cannot be empty")

            if column in {ENTITY_COLUMN_SIZE, ENTITY_COLUMN_INDEX, ENTITY_COLUMN_SUBINDEX, ENTITY_COLUMN_DATA_TYPE}:
                if int(updated.get("size", 0)) <= 0:
                    raise ValueError("Size must be > 0")

            if (new_index, new_subindex) != old_key:
                for item_row in self.entries:
                    if item_row.get("entry_id") == entry_id:
                        continue
                    if str(item_row.get("index_text", "")) == new_index and str(item_row.get("subindex_text", "")) == new_subindex:
                        policy = self.conflict_combo.currentText().strip() or "last"
                        if policy == "error":
                            raise ValueError(f"Duplicate entry at {new_index}:{new_subindex}")
                        if policy == "first":
                            raise ValueError(f"Duplicate entry at {new_index}:{new_subindex}")
                        self.entries = [item for item in self.entries if item.get("entry_id") != item_row.get("entry_id")]
                        break

                updated["index_text"] = new_index
                updated["subindex_text"] = new_subindex
                idx = self._find_entry_index(entry_id)
                if idx >= 0:
                    self.entries[idx] = updated
                self.refresh_entry_table()
                self._select_entry_item_by_id(entry_id)
            else:
                idx = self._find_entry_index(entry_id)
                if idx >= 0:
                    self.entries[idx] = updated
                if column == ENTITY_COLUMN_DATA_TYPE:
                    fixed_size = _format_row_size(backend.parse_data_type(updated["data_type"]))
                    if fixed_size is not None:
                        row["size"] = fixed_size
                        self._set_item_text(item, ENTITY_COLUMN_SIZE, str(fixed_size))
                if column in {ENTITY_COLUMN_INDEX, ENTITY_COLUMN_SUBINDEX}:
                    self._set_item_text(item, ENTITY_COLUMN_INDEX, str(updated["index_text"]))
                    self._set_item_text(item, ENTITY_COLUMN_SUBINDEX, str(updated["subindex_text"]))
                if column == ENTITY_COLUMN_DATA_TYPE:
                    self._set_item_text(item, ENTITY_COLUMN_DATA_TYPE, str(updated["data_type"]))
            self._mark_dirty()
        except Exception as error:
            if str(error):
                QMessageBox.warning(self, "Invalid entry edit", str(error))
            self.refresh_entry_table()
            self._select_entry_item_by_id(entry_id)
            return

    def refresh_entry_table(self) -> None:
        self._rebuilding_entries = True
        self.entity_tree.clear()
        index_node_stats: dict[int, dict[str, Any]] = {}
        default_storage = self.default_storage_combo.currentText().strip() or "pointer"
        for row in sorted(
            self.entries,
            key=lambda row: (
                backend.parse_int_expression(row["index_text"]),
                backend.parse_int_expression(row["subindex_text"]),
            ),
        ):
            row["entry_id"] = row.get("entry_id") or self._next_entry_id()
            if not row.get("storage"):
                row["storage"] = default_storage
            row["index_text"] = _format_index(backend.parse_int_expression(str(row.get("index_text", "0x0000"))))
            row["subindex_text"] = _format_subindex(backend.parse_int_expression(str(row.get("subindex_text", "0x00")), 1))
            data_type = backend.parse_data_type(
                row.get("data_type", DATA_TYPE_OPTIONS[0] if DATA_TYPE_OPTIONS else "UNSIGNED8")
            )
            row["data_type"] = CANONICAL_DATA_TYPE_NAME.get(data_type, f"0x{data_type:04X}")
            fixed_size = _format_row_size(data_type)
            if fixed_size is not None:
                row["size"] = fixed_size

            group_path = _parse_group_path(row.get("group_path", ""))
            parent = self._ensure_group_nodes(group_path)
            index_node = self._ensure_index_node(parent, row["index_text"])
            index_stats = index_node_stats.setdefault(
                id(index_node),
                {"node": index_node, "count": 0, "labels": [], "subindex_min": None, "subindex_max": None},
            )
            index_stats["count"] += 1
            label = str(row.get("parameter_name", "")).strip()
            if label:
                index_stats["labels"].append(label)
            try:
                subindex_value = backend.parse_int_expression(row["subindex_text"], 1)
            except Exception:
                subindex_value = None
            else:
                current_min = index_stats.get("subindex_min")
                current_max = index_stats.get("subindex_max")
                if current_min is None or subindex_value < current_min:
                    index_stats["subindex_min"] = subindex_value
                if current_max is None or subindex_value > current_max:
                    index_stats["subindex_max"] = subindex_value

            item = QTreeWidgetItem(index_node)
            item.setText(0, str(row.get("parameter_name", "")))
            item.setText(1, str(row["index_text"]))
            item.setText(2, str(row.get("subindex_text", "")))
            item.setText(3, str(row.get("data_type", DATA_TYPE_OPTIONS[0] if DATA_TYPE_OPTIONS else "UNSIGNED8")))
            item.setText(4, _access_short_label_fixed(row.get("access", "ro")))
            item.setText(5, str(row.get("storage", "pointer")))
            item.setText(6, "")
            item.setText(7, str(row.get("size", 0)))
            item.setText(8, str(row.get("variable_name", "")))
            item.setText(9, str(row.get("comment", "")))
            item.setCheckState(ENTITY_COLUMN_PDO, Qt.CheckState.Checked if row.get("pdo_mapping", False) else Qt.CheckState.Unchecked)
            item.setTextAlignment(ENTITY_COLUMN_ACCESS, Qt.AlignmentFlag.AlignCenter)
            item.setFont(ENTITY_COLUMN_ACCESS, self.access_font)
            item.setData(0, ROLE_KIND, "entry")
            item.setData(0, ROLE_ENTRY_ID, row["entry_id"])
            item.setData(0, ROLE_ENTRY_DATA, row["entry_id"])
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable | Qt.ItemFlag.ItemIsUserCheckable)
        for stats in index_node_stats.values():
            node = stats["node"]
            index_key = str(node.data(0, ROLE_INDEX_KEY) or node.text(1) or node.text(0))
            count = int(stats["count"])
            labels = list(stats.get("labels", []))
            subindex_min = stats.get("subindex_min")
            subindex_max = stats.get("subindex_max")
            range_text = ""
            if isinstance(subindex_min, int) and isinstance(subindex_max, int):
                if subindex_min == subindex_max:
                    range_text = f" [{_format_subindex(subindex_min)}]"
                else:
                    range_text = f" [{_format_subindex(subindex_min)}..{_format_subindex(subindex_max)}]"
            suffix = f"{count} entries"
            if labels:
                preview = labels[0]
                if len(labels) > 1:
                    preview = f"{preview} (+{len(labels) - 1})"
                suffix = f"{count} entries: {preview}"
            node.setText(0, f"{index_key}{range_text} ({suffix})")
            if range_text:
                node.setToolTip(
                    0,
                    f"Index {index_key} contains subindices {_format_subindex(int(subindex_min))}..{_format_subindex(int(subindex_max))}, {count} entries.",
                )
            else:
                node.setToolTip(0, f"Index {index_key} contains {count} subindex entries.")
        self.entity_tree.expandAll()
        self._rebuilding_entries = False
        self.entity_tree.resizeColumnToContents(0)
        self.refresh_preview()

    def _ensure_index_node(
        self,
        parent: QTreeWidgetItem | QTreeWidget,
        index_text: str,
    ) -> QTreeWidgetItem:
        key = str(index_text).strip()
        for i in range(parent.childCount()):
            child = parent.child(i)
            if child.data(0, ROLE_KIND) == "index" and child.data(0, ROLE_INDEX_KEY) == key:
                return child
        index_node = QTreeWidgetItem(parent)
        index_node.setText(0, key)
        index_node.setText(1, key)
        index_node.setData(0, ROLE_KIND, "index")
        index_node.setData(0, ROLE_INDEX_KEY, key)
        index_node.setFlags(index_node.flags() & ~(Qt.ItemFlag.ItemIsEditable | Qt.ItemFlag.ItemIsUserCheckable))
        index_node.setExpanded(True)
        for col in range(2, index_node.columnCount()):
            index_node.setText(col, "")
        return index_node

    def _ensure_group_nodes(self, group_path: list[str]) -> QTreeWidgetItem | QTreeWidget:
        if not group_path:
            return self.entity_tree.invisibleRootItem()
        key_path: list[str] = []
        cursor: QTreeWidgetItem | QTreeWidget = self.entity_tree.invisibleRootItem()
        for part in group_path:
            if not part:
                continue
            key_path.append(part)
            node_text = "/".join(key_path)
            existing = None
            for i in range(cursor.childCount()):
                child = cursor.child(i)
                if child.text(0) == part and child.data(0, ROLE_KIND) == "group":
                    existing = child
                    break
            if existing is None:
                existing = QTreeWidgetItem(cursor)
                existing.setText(0, part)
                for col in range(1, existing.columnCount()):
                    existing.setText(col, "")
                existing.setData(0, ROLE_KIND, "group")
                existing.setExpanded(True)
            cursor = existing
        return cursor

    def refresh_import_table(self) -> None:
        self.import_table.setRowCount(0)
        for index, item in enumerate(self.imports):
            if not item.get("import_id"):
                item["import_id"] = f"import-{index:04d}"
            row = self.import_table.rowCount()
            self.import_table.insertRow(row)
            self.import_table.setItem(row, 0, QTableWidgetItem(str(item.get("path", ""))))
            self.import_table.setItem(row, 1, QTableWidgetItem(str(item.get("alias", ""))))
            self.import_table.setItem(row, 2, QTableWidgetItem(str(item.get("storage", "pointer"))))
            self.import_table.setItem(row, 3, QTableWidgetItem(str(item.get("node_id", self.node_spin.value()))))
            self.import_table.item(row, 0).setData(Qt.ItemDataRole.UserRole, item["import_id"])

    def _selected_import_index(self) -> int:
        current_row = self.import_table.currentRow()
        if current_row < 0:
            return -1
        item = self.import_table.item(current_row, 0)
        if item is None:
            return -1
        target_id = item.data(Qt.ItemDataRole.UserRole)
        for idx, item_data in enumerate(self.imports):
            if item_data.get("import_id") == target_id:
                return idx
        return current_row

    def _upsert_entry(self, row: dict[str, Any]) -> None:
        key = (row["index_text"], row["subindex_text"])
        for index, item in enumerate(self.entries):
            if (item["index_text"], item["subindex_text"]) == key:
                self.entries[index] = dict(item | row)
                self.refresh_entry_table()
                return
        if not row.get("entry_id"):
            row["entry_id"] = self._next_entry_id()
        self.entries.append(row)
        self.refresh_entry_table()

    def add_entry(self) -> None:
        editor = _EntryEditor(self, _new_default_entry())
        if editor.exec() != QDialog.DialogCode.Accepted or not editor.result:
            return
        self._upsert_entry(editor.result)
        self._mark_dirty()

    def edit_entry(self) -> None:
        selected = self._selected_entry()
        if selected is None:
            return
        editor = _EntryEditor(self, selected)
        if editor.exec() != QDialog.DialogCode.Accepted or not editor.result:
            return
        for index, item in enumerate(self.entries):
            if item.get("entry_id") == selected.get("entry_id"):
                self.entries[index] = {**item, **editor.result}
                break
        self.refresh_entry_table()
        self._mark_dirty()

    def delete_entry(self) -> None:
        selected = self._selected_entry()
        if selected is None:
            return
        self.entries = [item for item in self.entries if item.get("entry_id") != selected.get("entry_id")]
        self.refresh_entry_table()
        self._mark_dirty()

    def import_from_eds(self) -> None:
        files, _ = QFileDialog.getOpenFileNames(
            self,
            "Add EDS as import",
            str(self.gmp_root),
            "EDS (*.eds);;All Files (*.*)",
        )
        if not files:
            return

        default_node = int(self.node_spin.value())
        storage = self.default_storage_combo.currentText().strip() or "pointer"
        added = 0
        for raw_path in files:
            path = Path(raw_path)
            alias = path.stem
            try:
                for entry in backend.load_eds(path, storage, default_node, group_path=(alias,)):
                    self._upsert_entry(_entry_to_row(entry))
                self.imports.append(
                    {
                        "path": str(path.resolve()),
                        "alias": alias,
                        "storage": storage,
                        "node_id": default_node,
                    }
                )
                added += 1
            except Exception as error:
                QMessageBox.critical(self, "Import failed", f"{path}: {error}")
                return
        if added:
            self.refresh_import_table()
            self.refresh_entry_table()
            self._mark_dirty()

    def append_raw_eds_entries(self) -> None:
        files, _ = QFileDialog.getOpenFileNames(
            self,
            "Append raw EDS entries",
            str(self.gmp_root),
            "EDS (*.eds);;All Files (*.*)",
        )
        if not files:
            return
        storage = self.default_storage_combo.currentText().strip() or "pointer"
        node = int(self.node_spin.value())
        for raw_path in files:
            try:
                for entry in backend.load_eds(Path(raw_path), storage, node, group_path=(Path(raw_path).stem,)):
                    self._upsert_entry(_entry_to_row(entry))
            except Exception as error:
                QMessageBox.critical(self, "Import failed", str(error))
                return
        self.refresh_entry_table()
        self._mark_dirty()

    def replace_project_from_eds(self) -> None:
        files, _ = QFileDialog.getOpenFileNames(
            self,
            "Replace project from EDS",
            str(self.gmp_root),
            "EDS (*.eds);;All Files (*.*)",
        )
        if not files:
            return
        storage = self.default_storage_combo.currentText().strip() or "pointer"
        node = int(self.node_spin.value())
        merged: list[backend.DictionaryEntry] = []
        imports = []
        for raw_path in files:
            path = Path(raw_path)
            alias = path.stem
            imported = backend.load_eds(path, storage, node, group_path=(alias,))
            merged.extend(imported)
            imports.append({"path": str(path.resolve()), "alias": alias, "storage": storage, "node_id": node})

        self.entries = [_entry_to_row(item) for item in merged]
        self.imports = imports
        if files:
            self.name_edit.setText(Path(files[0]).stem)
        self.refresh_import_table()
        self.refresh_entry_table()
        self._mark_dirty()

    # -------------------- Import table actions --------------------
    def edit_selected_import(self) -> None:
        index = self._selected_import_index()
        if index < 0:
            return
        current = self.imports[index]
        dialog = _ImportEditor(
            self,
            current,
            node_id=int(self.node_spin.value()),
            default_storage=self.default_storage_combo.currentText().strip() or "pointer",
        )
        if dialog.exec() != QDialog.DialogCode.Accepted or not dialog.result:
            return
        current.update(dialog.result)
        self.refresh_import_table()
        self._mark_dirty()

    def remove_import(self) -> None:
        index = self._selected_import_index()
        if index < 0:
            return
        del self.imports[index]
        self.refresh_import_table()
        self._mark_dirty()

    def move_selected_import(self, delta: int) -> None:
        index = self._selected_import_index()
        if index < 0:
            return
        target = index + delta
        if target < 0 or target >= len(self.imports):
            return
        self.imports[index], self.imports[target] = self.imports[target], self.imports[index]
        self.refresh_import_table()
        self.import_table.selectRow(target)
        self._mark_dirty()

    # -------------------- Preview / generate --------------------
    def refresh_preview(self) -> None:
        payload = self._payload()
        self.statusBar().showMessage(payload["name"] + ".json")

    def generate(self) -> None:
        output_dir = self.output_edit.text().strip()
        if not output_dir:
            output_dir = str(self.gmp_root / "core" / "protocol" / "canopen" / "generated")
            self.output_edit.setText(output_dir)
        output_path = Path(output_dir)
        if self.project_path is not None and not output_path.is_absolute():
            output_path = (self.project_path.parent / output_path).resolve()
        output_path.mkdir(parents=True, exist_ok=True)

        emit_eds = QMessageBox.question(
            self,
            "Generate",
            "Also emit merged EDS file?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        ) == QMessageBox.StandardButton.Yes
        emit_path = output_path / f"{self.name_edit.text().strip() or DEFAULT_PROJECT_NAME}.eds" if emit_eds else None
        payload = self._payload()
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False, encoding="utf-8") as temp_file:
            temp_file.write(json.dumps(payload, indent=2))
            temp_path = Path(temp_file.name)
        try:
            header, source = backend.compile_project(
                temp_path,
                output_path,
                payload["node_id"],
                payload["conflict_policy"],
                emit_path if emit_eds else None,
            )
            msg = f"Generated:\n{header}\n{source}"
            if emit_eds:
                msg += f"\n{emit_path}"
            QMessageBox.information(self, "Generate", msg)
        except Exception as error:
            QMessageBox.critical(self, "Generate failed", str(error))
        finally:
            temp_path.unlink(missing_ok=True)

    # -------------------- Context menus --------------------
    def _show_entity_context_menu(self, point) -> None:
        menu = QMenu(self)
        menu.addAction(self.add_entry_action)
        menu.addAction(self.edit_entry_action)
        menu.addAction(self.delete_entry_action)
        menu.addSeparator()
        menu.addAction(self.append_eds_action)
        menu.exec(self.entity_tree.viewport().mapToGlobal(point))

    def _show_import_context_menu(self, point) -> None:
        menu = QMenu(self)
        add_action = QAction("Add EDS...", self)
        add_action.triggered.connect(self.import_from_eds)
        edit_action = QAction("Edit...", self)
        edit_action.triggered.connect(self.edit_selected_import)
        remove_action = QAction("Remove", self)
        remove_action.triggered.connect(self.remove_import)
        up_action = QAction("Move Up", self)
        up_action.triggered.connect(lambda: self.move_selected_import(-1))
        down_action = QAction("Move Down", self)
        down_action.triggered.connect(lambda: self.move_selected_import(1))
        menu.addAction(add_action)
        menu.addAction(edit_action)
        menu.addAction(remove_action)
        menu.addSeparator()
        menu.addAction(up_action)
        menu.addAction(down_action)
        menu.exec(self.import_table.viewport().mapToGlobal(point))

    # -------------------- Qt window override --------------------
    def closeEvent(self, event) -> None:  # noqa: N802
        if not self._confirm_unsaved():
            event.ignore()
            return
        event.accept()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CANopen EDS editor and generator (Qt GUI).")
    parser.add_argument("--project", type=Path, default=None, help="Open a project JSON directly.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if Qt is None:
        print("canopen_eds_cc_gui requires PySide6 or Tkinter fallback was not configured in this environment.")
        return 1
    app = QApplication(sys.argv)
    gui = CanopenEdsGui(args.project)
    gui.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
