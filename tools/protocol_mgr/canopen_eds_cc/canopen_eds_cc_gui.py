#!/usr/bin/env python3
"""Graphical helper for CANopen EDS project authoring and code generation."""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

from typing import Any

try:
    from . import canopen_eds_cc as backend  # type: ignore
except Exception:  # pragma: no cover - allow standalone run
    MODULE_ROOT = Path(__file__).resolve().parent
    if str(MODULE_ROOT) not in sys.path:
        sys.path.insert(0, str(MODULE_ROOT))
    import canopen_eds_cc  # type: ignore
    backend = canopen_eds_cc


DEFAULT_PROJECT_NAME = "canopen_dictionary"
DATA_TYPE_OPTIONS = sorted(
    (
        (f"0x{code:04X}", code, name)
        for code, name in backend.TYPE_NAME_TO_CODE.items()
        if isinstance(code, int)
    ),
    key=lambda item: item[1],
)
DATA_TYPE_TEXT = [item[0] for item in DATA_TYPE_OPTIONS]
DATA_TYPE_TEXT.extend([f"{code:04X}" for _, code in DATA_TYPE_OPTIONS])


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


def _to_int(value: str) -> int:
    return int(value.strip().lower().replace("0x", ""), 16) if value.strip().lower().startswith("0x") else int(value, 10)


def _format_index(value: int) -> str:
    return f"0x{value:04X}"


def _format_subindex(value: int) -> str:
    return f"0x{value:02X}"


def _format_group_path(parts: tuple[str, ...] | list[str]) -> str:
    return "/".join(part for part in parts if part)


def _normalize_import_item(
    item: dict[str, Any],
    *,
    default_storage: str = "pointer",
    default_node: int = 1,
    default_id: str = "",
) -> dict[str, Any]:
    raw_alias = str(item.get("alias", "")).strip()
    path = str(item.get("path", "")).strip()
    storage = str(item.get("storage", default_storage)).strip() or default_storage
    node_id = item.get("node_id", item.get("node-id", default_node))
    try:
        node_id_int = int(node_id)
    except (TypeError, ValueError):
        node_id_int = default_node
    return {
        "path": path,
        "alias": raw_alias or Path(path).stem,
        "storage": storage,
        "node_id": max(1, node_id_int),
        "import_id": str(item.get("import_id", default_id) or default_id),
    }


def _entry_to_row(entry: backend.DictionaryEntry) -> dict[str, Any]:
    return {
        "index": entry.index,
        "subindex": entry.subindex,
        "index_text": _format_index(entry.index),
        "subindex_text": _format_subindex(entry.subindex),
        "parameter_name": entry.parameter_name,
        "data_type": entry.data_type,
        "access": entry.access,
        "pdo_mapping": bool(entry.pdo_mapping),
        "default_value": entry.default_value,
        "size": entry.size,
        "storage": entry.storage,
        "variable_name": entry.variable_name or "",
        "group_path": _format_group_path(entry.group_path),
        "comment": entry.comment or "",
    }


def _row_to_entry_payload(row: dict[str, Any]) -> dict[str, Any]:
    index = backend.parse_int_expression(row["index_text"])  # keep compatibility
    subindex = backend.parse_int_expression(row["subindex_text"], 1)
    data_type_text = row["data_type"].strip()
    if data_type_text.startswith("0x") or data_type_text.startswith("0X"):
        data_type = backend.parse_int_expression(data_type_text)
    else:
        data_type = backend.parse_data_type(data_type_text)
    payload = {
        "index": _format_index(index),
        "subindex": _format_subindex(subindex),
        "parameter_name": row["parameter_name"].strip(),
        "data_type": _format_index(data_type),
        "access": row["access"].strip().lower() or "ro",
        "pdo_mapping": bool(row["pdo_mapping"]),
        "default_value": str(row["default_value"]).strip(),
        "size": int(row["size"]),
        "storage": row["storage"],
    }
    if payload["storage"] in {"pointer", "value"} and row["variable_name"].strip():
        payload["storage"] = "variable"
    if row["variable_name"].strip():
        payload["variable_name"] = row["variable_name"].strip()
    if row["group_path"].strip():
        payload["group_path"] = [part.strip() for part in row["group_path"].split("/") if part.strip()]
    if row["comment"].strip():
        payload["comment"] = row["comment"].strip()
    return payload


class _EntryEditor(tk.Toplevel):
    def __init__(self, parent: tk.Widget, row: dict[str, Any] | None = None):
        super().__init__(parent)
        self.title("Edit Entry" if row else "New Entry")
        self.result: dict[str, Any] | None = None
        self._row = row or {
            "index_text": "0x2000",
            "subindex_text": "0x00",
            "parameter_name": "",
            "data_type": DATA_TYPE_OPTIONS[0][0],
            "access": "ro",
            "pdo_mapping": False,
            "default_value": "0",
            "size": 4,
            "storage": "pointer",
            "variable_name": "",
            "group_path": "",
            "comment": "",
            "entry_id": "",
        }
        self._build()
        self.grab_set()
        self.transient(parent)
        self.wait_window(self)

    def _build(self) -> None:
        frame = ttk.Frame(self, padding=10)
        frame.grid(row=0, column=0, sticky=tk.NSEW)
        for col in range(4):
            frame.columnconfigure(col, weight=1)

        self.fields: dict[str, tk.Variable] = {}

        labels = [
            ("Index", "index_text", 0, 0),
            ("Subindex", "subindex_text", 0, 2),
            ("Parameter name", "parameter_name", 1, 0),
            ("Data type", "data_type", 1, 2),
            ("Access", "access", 2, 0),
            ("Storage", "storage", 2, 2),
            ("Default", "default_value", 3, 0),
            ("Size", "size", 3, 2),
            ("Variable", "variable_name", 4, 0),
            ("Group path", "group_path", 4, 2),
            ("Comment", "comment", 5, 0),
        ]

        for text, key, row, col in labels:
            label = ttk.Label(frame, text=f"{text}:")
            label.grid(row=row * 2, column=col, sticky=tk.W, padx=3, pady=(6, 2))
            if key == "access":
                widget = ttk.Combobox(frame, values=["ro", "wo", "rw", "rwr", "rww", "const"], state="readonly")
                widget.set(self._row[key])
                self.fields[key] = widget
                widget.grid(row=row * 2 + 1, column=col, sticky=tk.EW, padx=3, pady=(0, 6))
            elif key == "storage":
                widget = ttk.Combobox(frame, values=["pointer", "value", "variable"], state="readonly")
                widget.set(self._row[key])
                self.fields[key] = widget
                widget.grid(row=row * 2 + 1, column=col, sticky=tk.EW, padx=3, pady=(0, 6))
            elif key == "data_type":
                widget = ttk.Combobox(frame, values=DATA_TYPE_TEXT, state="readonly")
                widget.set(self._row[key])
                self.fields[key] = widget
                widget.grid(row=row * 2 + 1, column=col, sticky=tk.EW, padx=3, pady=(0, 6))
            elif key == "comment":
                widget = ttk.Entry(frame)
                widget.insert(0, self._row[key])
                self.fields[key] = widget
                widget.grid(row=row * 2 + 1, column=col, columnspan=3, sticky=tk.EW, padx=3, pady=(0, 6))
            else:
                widget = ttk.Entry(frame)
                widget.insert(0, str(self._row[key]))
                self.fields[key] = widget
                widget.grid(row=row * 2 + 1, column=col, sticky=tk.EW, padx=3, pady=(0, 6))

        self.pdo_var = tk.BooleanVar(value=bool(self._row["pdo_mapping"]))
        self.pdo_check = ttk.Checkbutton(frame, text="PDO Mapping", variable=self.pdo_var)
        self.pdo_check.grid(row=11, column=0, sticky=tk.W, padx=3, pady=(6, 6))

        button_box = ttk.Frame(frame)
        button_box.grid(row=12, column=0, columnspan=4, sticky=tk.E, pady=(10, 0))
        ttk.Button(button_box, text="Cancel", command=self.destroy).pack(side=tk.RIGHT, padx=5)
        ttk.Button(button_box, text="OK", command=self._on_ok).pack(side=tk.RIGHT, padx=5)

    def _on_ok(self) -> None:
        try:
            index = backend.parse_int_expression(self.fields["index_text"].get(), 1)
            subindex = _to_int(self.fields["subindex_text"].get())
            if not (0 <= index <= 0xFFFF):
                raise ValueError("index out of range")
            if not (0 <= subindex <= 0xFF):
                raise ValueError("subindex out of range")
            size = int(self.fields["size"].get() or 0)
            if size <= 0:
                raise ValueError("size must be > 0")
            _ = backend.parse_data_type(self.fields["data_type"].get())
            _ = backend.parse_access(self.fields["access"].get())
        except Exception as error:
            messagebox.showerror("Invalid Entry", str(error), parent=self)
            return
        self.result = {
            "index_text": _format_index(index),
            "subindex_text": _format_subindex(subindex),
            "parameter_name": self.fields["parameter_name"].get().strip(),
            "data_type": self.fields["data_type"].get().strip(),
            "access": self.fields["access"].get().strip(),
            "pdo_mapping": bool(self.pdo_var.get()),
            "default_value": self.fields["default_value"].get().strip() or "0",
            "size": size,
            "storage": self.fields["storage"].get().strip() or "pointer",
            "variable_name": self.fields["variable_name"].get().strip(),
            "group_path": self.fields["group_path"].get().strip(),
            "comment": self.fields["comment"].get().strip(),
        }
        self.destroy()


class CanopenEdsGui:
    def __init__(self, project_path: Path | None = None) -> None:
        self.root = tk.Tk()
        self.root.title("canopen_eds_cc GUI")
        self.root.geometry("1360x780")
        self.gmp_root = _resolve_gmp_root()
        self.project_path: Path | None = None
        self.current_preview_path: Path | None = None
        self.preset_dir = self.gmp_root / "tools" / "protocol_mgr" / "canopen_eds_cc" / "presets"
        self.preset_dir.mkdir(parents=True, exist_ok=True)

        self._suppress_dirty = False
        self._dirty = False
        self._entry_id_counter = 0

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
        self._bind_shortcuts()
        self._refresh_preset_list()
        if project_path is not None:
            self.load_project(project_path)
        else:
            self._apply_project_state()

    def _build_ui(self) -> None:
        self.root.rowconfigure(4, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._build_menus()

        top_bar = ttk.Frame(self.root)
        top_bar.grid(row=0, column=0, sticky=tk.EW, padx=10, pady=10)
        top_bar.columnconfigure(1, weight=1)
        self.project_path_label = ttk.Label(top_bar, text="(unsaved)")
        self.project_path_label.grid(row=0, column=0, sticky=tk.W)
        ttk.Button(top_bar, text="New", command=self.new_project).grid(row=0, column=1, sticky=tk.E, padx=4)
        ttk.Button(top_bar, text="Open...", command=self.open_project).grid(row=0, column=2, sticky=tk.E, padx=4)
        ttk.Button(top_bar, text="Save", command=self.save_project).grid(row=0, column=3, sticky=tk.E, padx=4)
        ttk.Button(top_bar, text="Save As...", command=self.save_project_as).grid(row=0, column=4, sticky=tk.E, padx=4)

        preset_bar = ttk.LabelFrame(self.root, text="Preset Templates")
        preset_bar.grid(row=1, column=0, sticky=tk.EW, padx=10, pady=(0, 6))
        preset_bar.columnconfigure(1, weight=1)
        ttk.Label(preset_bar, text="Preset").grid(row=0, column=0, padx=6, pady=4, sticky=tk.W)
        self.preset_name_var = tk.StringVar(value="")
        self.preset_combo = ttk.Combobox(preset_bar, textvariable=self.preset_name_var, state="readonly")
        self.preset_combo.grid(row=0, column=1, padx=6, pady=4, sticky=tk.EW)
        ttk.Button(preset_bar, text="Load", command=self.load_preset).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(preset_bar, text="Save as Preset", command=self.save_preset).grid(row=0, column=3, padx=4, pady=4)
        ttk.Button(preset_bar, text="Delete", command=self.delete_preset).grid(row=0, column=4, padx=4, pady=4)

        prop = ttk.LabelFrame(self.root, text="Project")
        prop.grid(row=2, column=0, sticky=tk.NSEW, padx=10, pady=(0, 10))
        for i in range(6):
            prop.columnconfigure(i, weight=1)

        ttk.Label(prop, text="Name").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.name_var = tk.StringVar(value=self.project["name"])
        ttk.Entry(prop, textvariable=self.name_var, width=30).grid(row=0, column=1, sticky=tk.EW, padx=5, pady=2)
        ttk.Label(prop, text="Node ID").grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        self.node_id_var = tk.IntVar(value=self.project["node_id"])
        ttk.Spinbox(prop, from_=1, to=127, textvariable=self.node_id_var, width=8).grid(row=0, column=3, padx=5, pady=2)
        ttk.Label(prop, text="Default storage").grid(row=0, column=4, sticky=tk.W, padx=5, pady=2)
        self.storage_var = tk.StringVar(value=self.project["default_storage"])
        ttk.Combobox(prop, values=["pointer", "value", "variable"], textvariable=self.storage_var, state="readonly").grid(
            row=0, column=5, sticky=tk.EW, padx=5, pady=2
        )

        ttk.Label(prop, text="Conflict policy").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.conflict_var = tk.StringVar(value=self.project["conflict_policy"])
        ttk.Combobox(prop, values=["error", "first", "last"], textvariable=self.conflict_var, state="readonly").grid(
            row=1, column=1, sticky=tk.EW, padx=5, pady=2
        )
        ttk.Label(prop, text="Output directory").grid(row=1, column=2, sticky=tk.W, padx=5, pady=2)
        self.output_dir_var = tk.StringVar(value="")
        out_entry = ttk.Entry(prop, textvariable=self.output_dir_var)
        out_entry.grid(row=1, column=3, columnspan=2, sticky=tk.EW, padx=5, pady=2)
        ttk.Button(prop, text="Browse", command=self.browse_output_dir).grid(row=1, column=5, sticky=tk.EW, padx=5, pady=2)

        self.name_var.trace_add("write", lambda *_: self._mark_dirty())
        self.storage_var.trace_add("write", lambda *_: self._mark_dirty())
        self.node_id_var.trace_add("write", lambda *_: self._mark_dirty())
        self.conflict_var.trace_add("write", lambda *_: self._mark_dirty())
        self.output_dir_var.trace_add("write", lambda *_: self._mark_dirty())

        notebook = ttk.Notebook(self.root)
        notebook.grid(row=3, column=0, sticky=tk.NSEW, padx=10, pady=(0, 10))
        notebook.rowconfigure(0, weight=1)
        notebook.columnconfigure(0, weight=1)

        imports_page = ttk.Frame(notebook)
        imports_page.rowconfigure(1, weight=1)
        imports_page.columnconfigure(0, weight=1)
        notebook.add(imports_page, text="Imports")

        entries_page = ttk.Frame(notebook)
        entries_page.rowconfigure(1, weight=1)
        entries_page.columnconfigure(0, weight=1)
        notebook.add(entries_page, text="Entries")

        import_buttons = ttk.Frame(imports_page)
        import_buttons.grid(row=0, column=0, sticky=tk.EW, padx=6, pady=6)
        ttk.Button(import_buttons, text="Add EDS import...", command=self.import_from_eds).pack(side=tk.LEFT)
        ttk.Button(import_buttons, text="Edit import...", command=self.edit_selected_import).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(import_buttons, text="Remove", command=self.remove_import).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(import_buttons, text="Move Up", command=lambda: self.move_selected_import(-1)).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(import_buttons, text="Move Down", command=lambda: self.move_selected_import(1)).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(import_buttons, text="Append from EDS", command=self.append_raw_eds_entries).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(import_buttons, text="Replace from EDS", command=self.replace_project_from_eds).pack(side=tk.LEFT, padx=(6, 0))

        self.import_tree = ttk.Treeview(
            imports_page,
            columns=("path", "alias", "storage", "node"),
            show="headings",
            selectmode="browse",
        )
        self.import_tree.heading("path", text="EDS Path")
        self.import_tree.heading("alias", text="Alias")
        self.import_tree.heading("storage", text="Storage")
        self.import_tree.heading("node", text="Node ID")
        self.import_tree.column("path", width=520, stretch=True)
        self.import_tree.grid(row=1, column=0, sticky=tk.NSEW, padx=6, pady=(0, 6))
        self.import_tree.bind("<Button-3>", self.show_import_context_menu)
        self.import_tree.bind("<Double-1>", lambda _: self.edit_selected_import())
        self.import_tree.bind("<Return>", lambda _: self.edit_selected_import())

        entry_view = ttk.Notebook(entries_page)
        entry_view.grid(row=1, column=0, sticky=tk.NSEW, padx=6, pady=6)
        entry_view.rowconfigure(0, weight=1)
        entry_view.columnconfigure(0, weight=1)

        entry_list_page = ttk.Frame(entry_view)
        entry_tree_page = ttk.Frame(entry_view)
        entry_view.add(entry_list_page, text="Entries Table")
        entry_view.add(entry_tree_page, text="Parameter Tree")

        entry_buttons = ttk.Frame(entry_list_page)
        entry_buttons.grid(row=0, column=0, sticky=tk.EW, padx=6, pady=6)
        ttk.Button(entry_buttons, text="Add", command=self.add_entry).pack(side=tk.LEFT)
        ttk.Button(entry_buttons, text="Edit", command=self.edit_entry).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(entry_buttons, text="Delete", command=self.delete_entry).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(entry_buttons, text="Import entries from EDS...", command=self.append_raw_eds_entries).pack(side=tk.LEFT, padx=(6, 0))

        columns = (
            "index",
            "subindex",
            "name",
            "type",
            "access",
            "storage",
            "pdo",
            "size",
            "default",
            "group",
            "variable",
            "comment",
        )
        self.entry_tree = ttk.Treeview(entry_list_page, columns=columns, show="headings", selectmode="browse")
        headings = {
            "index": "Index",
            "subindex": "Sub",
            "name": "Parameter Name",
            "type": "DataType",
            "access": "Access",
            "storage": "Storage",
            "pdo": "PDO",
            "size": "Size",
            "default": "Default",
            "group": "Group Path",
            "variable": "Variable",
            "comment": "Comment",
        }
        widths = {
            "index": 80,
            "subindex": 80,
            "name": 220,
            "type": 90,
            "access": 70,
            "storage": 90,
            "pdo": 60,
            "size": 50,
            "default": 120,
            "group": 200,
            "variable": 120,
            "comment": 220,
        }
        for column in columns:
            self.entry_tree.heading(column, text=headings[column])
            self.entry_tree.column(column, width=widths.get(column, 80), stretch=False)
        self.entry_tree.grid(row=1, column=0, sticky=tk.NSEW, padx=6, pady=(0, 6))
        self.entry_tree.bind("<Double-1>", lambda _: self.edit_entry())
        self.entry_tree.bind("<Return>", lambda _: self.edit_entry())
        self.entry_tree.bind("<Delete>", lambda _: self.delete_entry())
        self.entry_tree.bind("<Button-3>", self.show_entry_context_menu)
        entry_list_page.rowconfigure(1, weight=1)
        entry_list_page.columnconfigure(0, weight=1)

        tree_columns = (
            "index",
            "subindex",
            "type",
            "access",
            "storage",
            "pdo",
            "size",
            "default",
            "variable",
            "comment",
        )
        self.param_tree = ttk.Treeview(entry_tree_page, columns=tree_columns, show="tree headings", selectmode="browse")
        self.param_tree.heading("#0", text="Parameter / Group")
        tree_widths = {
            "index": 80,
            "subindex": 80,
            "type": 90,
            "access": 70,
            "storage": 90,
            "pdo": 60,
            "size": 50,
            "default": 110,
            "variable": 120,
            "comment": 220,
        }
        for col in tree_columns:
            self.param_tree.heading(col, text=headings[col if col in headings else "comment"])
            self.param_tree.column(col, width=tree_widths.get(col, 100), stretch=False)
        self.param_tree.grid(row=0, column=0, sticky=tk.NSEW, padx=6, pady=(0, 6))
        self.param_tree.bind("<Double-1>", lambda _: self.edit_entry())
        self.param_tree.bind("<Return>", lambda _: self.edit_entry())
        self.param_tree.bind("<Delete>", lambda _: self.delete_entry())
        self.param_tree.bind("<Button-3>", self.show_entry_context_menu)
        entry_tree_page.rowconfigure(0, weight=1)
        entry_tree_page.columnconfigure(0, weight=1)

        action_bar = ttk.Frame(self.root)
        action_bar.grid(row=4, column=0, sticky=tk.EW, padx=10, pady=(0, 10))
        ttk.Button(action_bar, text="Generate .h/.c...", command=self.generate).pack(side=tk.LEFT)
        ttk.Label(action_bar, text="JSON preview:").pack(side=tk.LEFT, padx=(30, 6))
        self.preview_var = tk.StringVar(value="")
        ttk.Entry(action_bar, textvariable=self.preview_var, width=48, state="readonly").pack(side=tk.RIGHT)
        ttk.Button(action_bar, text="Refresh Preview", command=self.refresh_preview).pack(side=tk.RIGHT, padx=(0, 6))

    def _build_menus(self) -> None:
        menu = tk.Menu(self.root)
        file_menu = tk.Menu(menu, tearoff=0)
        file_menu.add_command(label="New", command=self.new_project, accelerator="Ctrl+N")
        file_menu.add_command(label="Open...", command=self.open_project, accelerator="Ctrl+O")
        file_menu.add_command(label="Save", command=self.save_project, accelerator="Ctrl+S")
        file_menu.add_command(label="Save As...", command=self.save_project_as, accelerator="Ctrl+Shift+S")
        file_menu.add_separator()
        file_menu.add_command(label="Save Preset", command=self.save_preset)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self._on_close)
        menu.add_cascade(label="File", menu=file_menu)

        edit_menu = tk.Menu(menu, tearoff=0)
        edit_menu.add_command(label="Add Entry", command=self.add_entry, accelerator="Ins")
        edit_menu.add_command(label="Edit Entry", command=self.edit_entry, accelerator="Enter")
        edit_menu.add_command(label="Delete Entry", command=self.delete_entry, accelerator="Del")
        edit_menu.add_separator()
        edit_menu.add_command(label="Append from EDS", command=self.append_raw_eds_entries, accelerator="Ctrl+I")
        menu.add_cascade(label="Edit", menu=edit_menu)

        tools_menu = tk.Menu(menu, tearoff=0)
        tools_menu.add_command(label="Generate", command=self.generate, accelerator="Ctrl+G")
        tools_menu.add_command(label="Refresh Preview", command=self.refresh_preview, accelerator="F5")
        menu.add_cascade(label="Tools", menu=tools_menu)

        self.root.config(menu=menu)

    def _bind_shortcuts(self) -> None:
        self.root.bind_all("<Control-n>", lambda _: self.new_project())
        self.root.bind_all("<Control-o>", lambda _: self.open_project())
        self.root.bind_all("<Control-s>", lambda _: self.save_project())
        self.root.bind_all("<Control-Shift-S>", lambda _: self.save_project_as())
        self.root.bind_all("<Control-g>", lambda _: self.generate())
        self.root.bind_all("<F5>", lambda _: self.refresh_preview())
        self.root.bind_all("<Control-i>", lambda _: self.append_raw_eds_entries())
        self.root.bind_all("<Insert>", lambda _: self.add_entry())
        self.root.bind_all("<Return>", lambda _: self.edit_entry())
        self.root.bind_all("<Delete>", lambda _: self.delete_entry())
        self.root.bind_all("<Control-q>", lambda _: self._on_close())

    def _new_entry_id(self) -> str:
        self._entry_id_counter += 1
        return f"entry-{self._entry_id_counter:06d}"

    def _ensure_entry_ids(self) -> None:
        for item in self.entries:
            if not item.get("entry_id"):
                item["entry_id"] = self._new_entry_id()
            else:
                self._entry_id_counter = max(
                    self._entry_id_counter,
                    int(item["entry_id"].replace("entry-", ""), 10) if str(item["entry_id"]).startswith("entry-") and str(item["entry_id"]).replace("entry-", "").isdigit() else self._entry_id_counter,
                )

    def _payload(self) -> dict[str, Any]:
        sorted_entries = sorted(
            self.entries,
            key=lambda row: (
                backend.parse_int_expression(row["index_text"]),
                backend.parse_int_expression(row["subindex_text"]),
            ),
        )
        entries = [_row_to_entry_payload(item) for item in sorted_entries]
        for item in entries:
            if item["size"] is None:
                item["size"] = 0
        return {
            "version": self.project["version"],
            "name": self.name_var.get().strip() or DEFAULT_PROJECT_NAME,
            "node_id": self.node_id_var.get(),
            "default_storage": self.storage_var.get().strip() or "pointer",
            "conflict_policy": self.conflict_var.get().strip() or "last",
            "output_dir": self.output_dir_var.get().strip() or None,
            "imports": list(self.imports),
            "code": {
                "header_prefix": "",
                "header_suffix": "",
                "source_prefix": "",
                "source_suffix": "",
            },
            "entries": entries,
        }

    def _mark_dirty(self) -> None:
        if self._suppress_dirty:
            return
        self._dirty = True
        self._sync_title()

    def _set_dirty(self, dirty: bool) -> None:
        if self._suppress_dirty:
            return
        self._dirty = dirty
        self._sync_title()

    def _sync_title(self) -> None:
        suffix = " *" if self._dirty else ""
        if self.project_path is None:
            self.root.title(f"canopen_eds_cc GUI - (unsaved){suffix}")
            self.project_path_label.config(text="(unsaved)")
        else:
            self.root.title(f"canopen_eds_cc GUI - {self.project_path.name}{suffix}")
            self.project_path_label.config(text=str(self.project_path))

    def refresh_preview(self) -> None:
        payload = self._payload()
        self.preview_var.set(payload["name"] + ".json")

    def _apply_project_state(self) -> None:
        self._suppress_dirty = True
        self.name_var.set(self.project["name"])
        self.node_id_var.set(int(self.project["node_id"]))
        self.storage_var.set(self.project["default_storage"])
        self.conflict_var.set(self.project["conflict_policy"])
        self.output_dir_var.set(self.project.get("output_dir") or "")
        default_storage = self.project.get("default_storage", "pointer")
        default_node = int(self.project.get("node_id", 1))
        self.imports = [
            _normalize_import_item(dict(item), default_storage=default_storage, default_node=default_node, default_id=f"import-{idx:04d}")
            for idx, item in enumerate(self.project.get("imports", []))
        ]
        self.entries = list(self.project.get("entries", []))
        self._entry_id_counter = 0
        self._ensure_entry_ids()
        self.refresh_import_table()
        self.refresh_entry_table()
        self.refresh_preview()
        self._suppress_dirty = False
        self._set_dirty(False)
        self._update_current_preset_combo()

    def _update_current_preset_combo(self) -> None:
        self._refresh_preset_list()

    def refresh_import_table(self) -> None:
        self.import_tree.delete(*self.import_tree.get_children())
        for index, item in enumerate(self.imports):
            if not item.get("import_id"):
                item["import_id"] = f"import-{index:04d}"
            self.import_tree.insert(
                "",
                "end",
                iid=item["import_id"],
                values=(
                    str(item.get("path", "")),
                    str(item.get("alias", "")),
                    str(item.get("storage", "pointer")),
                    str(item.get("node_id", self.node_id_var.get())),
                ),
            )

    def refresh_entry_table(self) -> None:
        self.entry_tree.delete(*self.entry_tree.get_children())
        self.param_tree.delete(*self.param_tree.get_children())
        self.refresh_entry_list()
        self.refresh_entry_tree()

    def refresh_entry_list(self) -> None:
        for item in sorted(
            self.entries,
            key=lambda row: (
                backend.parse_int_expression(row["index_text"]),
                backend.parse_int_expression(row["subindex_text"]),
            ),
        ):
            item["entry_id"] = item.get("entry_id") or self._new_entry_id()
            item_id = item["entry_id"]
            self.entry_tree.insert(
                "",
                "end",
                iid=item_id,
                values=(
                    item["index_text"],
                    item["subindex_text"],
                    item["parameter_name"],
                    item["data_type"],
                    item["access"],
                    item["storage"],
                    "1" if item["pdo_mapping"] else "0",
                    item["size"],
                    item["default_value"],
                    item["group_path"],
                    item["variable_name"],
                    item["comment"],
                ),
            )

    def refresh_entry_tree(self) -> None:
        groups: dict[str, str] = {}
        for item in sorted(
            self.entries,
            key=lambda row: (
                backend.parse_int_expression(row["index_text"]),
                backend.parse_int_expression(row["subindex_text"]),
            ),
        ):
            item["entry_id"] = item.get("entry_id") or self._new_entry_id()
            entry_id = item["entry_id"]
            group_path = [part.strip() for part in str(item.get("group_path", "")).split("/") if part.strip()]
            parent = ""
            path_parts: list[str] = []
            for part in group_path:
                path_parts.append(part)
                key = "/".join(path_parts)
                node_id = f"group:{key}"
                if key not in groups:
                    groups[key] = node_id
                    self.param_tree.insert(
                        parent,
                        "end",
                        iid=node_id,
                        text=part,
                        values=("", "", "", "", "", "", "", "", "", ""),
                        open=True,
                    )
                parent = node_id
            self.param_tree.insert(
                parent,
                "end",
                iid=entry_id,
                text=item["parameter_name"],
                values=(
                    item["index_text"],
                    item["subindex_text"],
                    item["data_type"],
                    item["access"],
                    item["storage"],
                    "1" if item["pdo_mapping"] else "0",
                    item["size"],
                    item["default_value"],
                    item["variable_name"],
                    item["comment"],
                ),
            )

    def _selected_import(self) -> str | None:
        selected = self.import_tree.selection()
        return selected[0] if selected else None

    def _find_import_index(self, import_id: str | None) -> int:
        if not import_id:
            return -1
        for index, item in enumerate(self.imports):
            if item.get("import_id") == import_id:
                return index
        return -1

    def _selected_entry(self) -> dict[str, Any] | None:
        selected = self.entry_tree.selection()
        if not selected:
            selected = self.param_tree.selection()
        if not selected:
            return None
        entry_id = selected[0]
        if str(entry_id).startswith("group:"):
            return None
        for item in self.entries:
            if item.get("entry_id") == entry_id:
                return item
        return None

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

    def load_project(self, path: Path) -> None:
        if not path.exists():
            messagebox.showerror("Open failed", f"File not found: {path}")
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
            messagebox.showerror("Open failed", str(error))

    def _apply_project_payload(self, payload: dict[str, Any], *, source_path: Path | None = None) -> None:
        payload_copy = dict(payload)
        source_dir = source_path.parent if source_path is not None else self.preset_dir
        normalized_imports = []
        for index, item in enumerate(payload_copy.get("imports", []) or []):
            if not isinstance(item, dict):
                continue
            normalized_imports.append(
                _normalize_import_item(
                    dict(item),
                    default_storage=payload_copy.get("default_storage", "pointer"),
                    default_node=payload_copy.get("node_id", 1),
                    default_id=f"import-{index:04d}",
                )
            )
            if normalized_imports[-1]["path"] and not Path(normalized_imports[-1]["path"]).is_absolute():
                normalized_imports[-1]["path"] = str((source_dir / normalized_imports[-1]["path"]).resolve())
        payload_copy["imports"] = normalized_imports
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False, encoding="utf-8") as temp_file:
            temp_file.write(json.dumps(payload_copy, indent=2))
            temp_path = Path(temp_file.name)
        try:
            loaded = backend.load_project_json(temp_path)
            self.project_path = None
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
        finally:
            temp_path.unlink(missing_ok=True)

    def open_project(self) -> None:
        if not self._confirm_unsaved():
            return
        path = filedialog.askopenfilename(
            title="Open CANopen project json",
            filetypes=[("CANopen JSON", "*.json"), ("All Files", "*.*")],
            initialdir=str(self.gmp_root),
        )
        if path:
            self.load_project(Path(path))

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
        path = filedialog.asksaveasfilename(
            title="Save CANopen project json",
            defaultextension=".json",
            filetypes=[("CANopen JSON", "*.json"), ("All Files", "*.*")],
            initialdir=str(self.gmp_root),
        )
        if not path:
            return False
        self.project_path = Path(path)
        return self.save_project()

    def browse_output_dir(self) -> None:
        path = filedialog.askdirectory(title="Select output directory", initialdir=str(self.gmp_root))
        if path:
            self.output_dir_var.set(path)

    def _confirm_unsaved(self) -> bool:
        if not self._dirty:
            return True
        result = messagebox.askyesnocancel("Unsaved changes", "Current project has unsaved changes. Save now?")
        if result is None:
            return False
        if result:
            return self.save_project()
        return True

    def _on_close(self) -> None:
        if not self._confirm_unsaved():
            return
        self.root.destroy()

    def _refresh_preset_list(self) -> None:
        names = sorted(item.stem for item in self.preset_dir.glob("*.json"))
        self.preset_combo["values"] = names
        current = self.preset_name_var.get()
        if current and current not in names:
            self.preset_name_var.set("")

    def _preset_path(self, name: str) -> Path:
        safe = "".join(ch for ch in name.strip() if ch not in '<>:"/\\|?*')
        return self.preset_dir / f"{safe}.json"

    def save_preset(self) -> None:
        default_name = self.preset_name_var.get() or self.name_var.get() or DEFAULT_PROJECT_NAME
        preset_name = simpledialog.askstring(
            "Save Preset",
            "Preset name:",
            parent=self.root,
            initialvalue=default_name,
        )
        if not preset_name:
            return
        preset_name = preset_name.strip()
        if not preset_name:
            return
        payload = self._payload()
        payload = {"kind": "canopen_eds_cc_preset_v1", "name": preset_name, "project": payload}
        path = self._preset_path(preset_name)
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8", newline="\n")
        self.preset_name_var.set(preset_name)
        self._refresh_preset_list()
        messagebox.showinfo("Save Preset", f"Saved preset: {path}")

    def load_preset(self) -> None:
        preset_name = self.preset_name_var.get().strip()
        if not preset_name:
            messagebox.showerror("Load Preset", "Please choose a preset first.")
            return
        path = self._preset_path(preset_name)
        if not path.exists():
            messagebox.showerror("Load Preset", f"Preset not found: {path}")
            return
        if not self._confirm_unsaved():
            return
        with path.open("r", encoding="utf-8") as stream:
            data = json.load(stream)
        project = data.get("project", data)
        if not isinstance(project, dict):
            messagebox.showerror("Load Preset", "Invalid preset format.")
            return
        self._apply_project_payload(project, source_path=path)
        self.project_path = None
        self._set_dirty(False)

    def delete_preset(self) -> None:
        preset_name = self.preset_name_var.get().strip()
        if not preset_name:
            messagebox.showerror("Delete Preset", "Please choose a preset first.")
            return
        path = self._preset_path(preset_name)
        if not path.exists():
            messagebox.showerror("Delete Preset", "Preset not found.")
            return
        if not messagebox.askyesno("Delete Preset", f"Delete preset '{preset_name}' ?"):
            return
        path.unlink()
        self.preset_name_var.set("")
        self._refresh_preset_list()

    def add_entry(self) -> None:
        editor = _EntryEditor(self.root)
        if editor.result is None:
            return
        editor.result["entry_id"] = self._new_entry_id()
        self._upsert_entry(editor.result)
        self.refresh_entry_table()
        self._set_dirty(True)

    def edit_entry(self) -> None:
        selected = self._selected_entry()
        if selected is None:
            return
        editor = _EntryEditor(self.root, selected)
        if editor.result is None:
            return
        current_id = selected.get("entry_id")
        selected.clear()
        selected.update(editor.result)
        if current_id is not None:
            selected["entry_id"] = current_id
        self.refresh_entry_table()
        self._set_dirty(True)

    def delete_entry(self) -> None:
        selected_id = None
        selected = self.entry_tree.selection()
        if selected:
            selected_id = selected[0]
        else:
            selected = self.param_tree.selection()
            if selected:
                selected_id = selected[0]
        if not selected_id or str(selected_id).startswith("group:"):
            return
        self.entries = [item for item in self.entries if item.get("entry_id") != selected_id]
        self.refresh_entry_table()
        self._set_dirty(True)

    def import_from_eds(self) -> None:
        files = filedialog.askopenfilenames(
            title="Add EDS as import",
            filetypes=[("EDS", "*.eds"), ("All Files", "*.*")],
            initialdir=str(self.gmp_root),
        )
        if not files:
            return
        default_node = self.node_id_var.get()
        storage = self.storage_var.get().strip() or "pointer"
        added = 0
        for raw_path in files:
            path = Path(raw_path)
            alias = path.stem
            try:
                imported = backend.load_eds(path, storage, default_node, group_path=(alias,))
                for entry in imported:
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
                messagebox.showerror("Import failed", f"{path}: {error}")
                return
        if added:
            self.refresh_import_table()
            self.refresh_entry_table()
            self._set_dirty(True)

    def edit_selected_import(self) -> None:
        selected = self._selected_import()
        if not selected:
            return
        index = self._find_import_index(selected)
        if index < 0:
            return
        current = self.imports[index]
        alias = simpledialog.askstring("Import Alias", "Alias:", parent=self.root, initialvalue=current.get("alias", ""))
        if alias is None:
            return
        storage = simpledialog.askstring("Import Storage", "Storage:", parent=self.root, initialvalue=current.get("storage", self.storage_var.get()))
        if storage is None:
            return
        node = simpledialog.askinteger(
            "Import Node",
            "Node ID:",
            parent=self.root,
            initialvalue=int(current.get("node_id", self.node_id_var.get())),
            minvalue=1,
            maxvalue=127,
        )
        if node is None:
            return
        current.update({"alias": alias.strip() or current.get("alias", Path(current.get("path", "")).stem), "storage": storage.strip() or "pointer", "node_id": node})
        self.refresh_import_table()
        self._set_dirty(True)

    def remove_import(self) -> None:
        selected = self._selected_import()
        if not selected:
            return
        index = self._find_import_index(selected)
        if index < 0:
            return
        del self.imports[index]
        self.refresh_import_table()
        self._set_dirty(True)

    def move_selected_import(self, delta: int) -> None:
        selected = self._selected_import()
        if not selected:
            return
        index = self._find_import_index(selected)
        if index < 0:
            return
        target = index + delta
        if target < 0 or target >= len(self.imports):
            return
        self.imports[index], self.imports[target] = self.imports[target], self.imports[index]
        self.refresh_import_table()
        self._set_dirty(True)
        moved = self.imports[target].get("import_id")
        if moved:
            self.import_tree.selection_set(moved)

    def show_import_context_menu(self, event) -> None:
        menu = tk.Menu(self.root, tearoff=0)
        menu.add_command(label="Add EDS import...", command=self.import_from_eds)
        menu.add_command(label="Edit...", command=self.edit_selected_import)
        menu.add_command(label="Remove", command=self.remove_import)
        menu.add_separator()
        menu.add_command(label="Move Up", command=lambda: self.move_selected_import(-1))
        menu.add_command(label="Move Down", command=lambda: self.move_selected_import(1))
        menu.tk_popup(event.x_root, event.y_root)

    def show_entry_context_menu(self, event) -> None:
        menu = tk.Menu(self.root, tearoff=0)
        menu.add_command(label="Add", command=self.add_entry)
        menu.add_command(label="Edit", command=self.edit_entry)
        menu.add_command(label="Delete", command=self.delete_entry)
        menu.add_separator()
        menu.add_command(label="Import entries from EDS...", command=self.append_raw_eds_entries)
        menu.tk_popup(event.x_root, event.y_root)

    def append_raw_eds_entries(self) -> None:
        files = filedialog.askopenfilenames(
            title="Append raw EDS entries",
            filetypes=[("EDS", "*.eds"), ("All Files", "*.*")],
            initialdir=str(self.gmp_root),
        )
        if not files:
            return
        storage = self.storage_var.get().strip() or "pointer"
        node = self.node_id_var.get()
        for raw_path in files:
            try:
                for entry in backend.load_eds(Path(raw_path), storage, node, group_path=(Path(raw_path).stem,)):
                    self._upsert_entry(_entry_to_row(entry))
            except Exception as error:
                messagebox.showerror("Import failed", str(error))
                return
        self.refresh_entry_table()
        self._set_dirty(True)

    def replace_project_from_eds(self) -> None:
        files = filedialog.askopenfilenames(
            title="Replace with imported EDS",
            filetypes=[("EDS", "*.eds"), ("All Files", "*.*")],
            initialdir=str(self.gmp_root),
        )
        if not files:
            return
        node = self.node_id_var.get()
        storage = self.storage_var.get().strip() or "pointer"
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
            self.name_var.set(Path(files[0]).stem)
        self.refresh_import_table()
        self.refresh_entry_table()
        self._set_dirty(True)

    def _upsert_entry(self, row: dict[str, Any]) -> None:
        key = (row["index_text"], row["subindex_text"])
        for idx, item in enumerate(self.entries):
            if (item["index_text"], item["subindex_text"]) == key:
                self.entries[idx] = row
                return
        if not row.get("entry_id"):
            row["entry_id"] = self._new_entry_id()
        self.entries.append(row)

    def generate(self) -> None:
        output_dir = self.output_dir_var.get().strip()
        if not output_dir:
            output_dir = str(self.gmp_root / "core" / "protocol" / "canopen" / "generated")
            self.output_dir_var.set(output_dir)
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        emit_eds = messagebox.askyesno("Generate", "Also emit merged EDS file?")
        emit_path = output_path / f"{self.name_var.get().strip() or DEFAULT_PROJECT_NAME}.eds" if emit_eds else None
        payload = self._payload()
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False, encoding="utf-8") as temp_file:
            temp_file.write(json.dumps(payload, indent=2))
            temp_project = Path(temp_file.name)
        try:
            header, source = backend.compile_project(
                temp_project,
                output_path,
                payload["node_id"],
                payload["conflict_policy"],
                emit_path if emit_eds else None,
            )
            msg = f"Generated:\n- {header}\n- {source}"
            if emit_eds:
                msg += f"\n- {emit_path}"
            messagebox.showinfo("Generate", msg)
        except Exception as error:
            messagebox.showerror("Generate failed", str(error))
        finally:
            temp_project.unlink(missing_ok=True)

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CANopen EDS editor and generator.")
    parser.add_argument("--project", type=Path, default=None, help="Open a project JSON directly.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    gui = CanopenEdsGui(args.project)
    gui.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
