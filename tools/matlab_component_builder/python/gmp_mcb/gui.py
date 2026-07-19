"""Small PyQt editor for component definitions."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .generator import ComponentGenerator
from .model import ComponentDefinition


class BuilderWindow(QMainWindow):
    def __init__(self, config_path: Path):
        super().__init__()
        self.tool_root = Path(__file__).resolve().parents[2]
        self.config_path = config_path.resolve()
        self.data: dict = {}
        self.setWindowTitle("GMP MATLAB Component Builder")
        self.resize(1050, 720)
        self._build_ui()
        self.refresh_tree()
        self.load(self.config_path)

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        toolbar = QHBoxLayout()
        for text, slot in (
            ("Open", self.open_file),
            ("Save", self.save),
            ("Validate", self.validate),
            ("Preview Code", self.preview_code),
            ("Generate", self.generate),
            ("Install in MATLAB", self.install_matlab),
        ):
            button = QPushButton(text)
            button.clicked.connect(slot)
            toolbar.addWidget(button)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        splitter = QSplitter()
        tree_panel = QWidget()
        tree_layout = QVBoxLayout(tree_panel)
        tree_layout.addWidget(QLabel("Component Library"))
        self.component_tree = QTreeWidget()
        self.component_tree.setHeaderLabels(["Components"])
        self.component_tree.itemDoubleClicked.connect(self._tree_item_opened)
        tree_layout.addWidget(self.component_tree, 1)
        tree_buttons = QHBoxLayout()
        duplicate = QPushButton("Duplicate")
        duplicate.clicked.connect(self.duplicate_component)
        delete = QPushButton("Delete")
        delete.clicked.connect(self.delete_component)
        reload_tree = QPushButton("Reload")
        reload_tree.clicked.connect(self.refresh_tree)
        tree_buttons.addWidget(duplicate)
        tree_buttons.addWidget(delete)
        tree_buttons.addWidget(reload_tree)
        tree_layout.addLayout(tree_buttons)
        splitter.addWidget(tree_panel)

        self.tabs = QTabWidget()
        splitter.addWidget(self.tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([280, 800])
        layout.addWidget(splitter, 1)

        general = QWidget()
        form = QFormLayout(general)
        self.id_edit = QLineEdit()
        self.name_edit = QLineEdit()
        self.description_edit = QTextEdit()
        self.description_edit.setMaximumHeight(90)
        self.system_type_edit = QLineEdit()
        self.template_edit = QLineEdit()
        self.variant_edit = QLineEdit()
        self.instance_edit = QLineEdit()
        self.headers_edit = QTextEdit()
        self.headers_edit.setMaximumHeight(80)
        self.sources_edit = QTextEdit()
        self.sources_edit.setMaximumHeight(80)
        form.addRow("Component ID", self.id_edit)
        form.addRow("Display name", self.name_edit)
        form.addRow("Description", self.description_edit)
        form.addRow("System type (siso/mimo)", self.system_type_edit)
        form.addRow("Generator template", self.template_edit)
        form.addRow("Template variant", self.variant_edit)
        form.addRow("C instance type", self.instance_edit)
        form.addRow("Headers (one per line)", self.headers_edit)
        form.addRow("Sources (one per line)", self.sources_edit)
        self.tabs.addTab(general, "Component")

        ports = QWidget()
        ports_layout = QVBoxLayout(ports)
        ports_layout.addWidget(QLabel("Scalar Simulink ports, in displayed order."))
        ports_layout.addWidget(QLabel("Inputs"))
        self.input_table = QTableWidget(0, 4)
        self.input_table.setHorizontalHeaderLabels(["ID", "Label", "Width", "Type"])
        self.input_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        ports_layout.addWidget(self.input_table)
        input_buttons = QHBoxLayout()
        add_input = QPushButton("Add input")
        add_input.clicked.connect(lambda: self.input_table.insertRow(self.input_table.rowCount()))
        remove_input = QPushButton("Remove selected input")
        remove_input.clicked.connect(lambda: self._remove_selected(self.input_table))
        input_buttons.addWidget(add_input)
        input_buttons.addWidget(remove_input)
        input_buttons.addStretch(1)
        ports_layout.addLayout(input_buttons)
        ports_layout.addWidget(QLabel("Outputs"))
        self.output_table = QTableWidget(0, 4)
        self.output_table.setHorizontalHeaderLabels(["ID", "Label", "Width", "Type"])
        self.output_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        ports_layout.addWidget(self.output_table)
        output_buttons = QHBoxLayout()
        add_output = QPushButton("Add output")
        add_output.clicked.connect(lambda: self.output_table.insertRow(self.output_table.rowCount()))
        remove_output = QPushButton("Remove selected output")
        remove_output.clicked.connect(lambda: self._remove_selected(self.output_table))
        output_buttons.addWidget(add_output)
        output_buttons.addWidget(remove_output)
        output_buttons.addStretch(1)
        ports_layout.addLayout(output_buttons)
        self.tabs.addTab(ports, "Ports")

        parameters = QWidget()
        param_layout = QVBoxLayout(parameters)
        param_layout.addWidget(QLabel("Mask parameters. Edit values directly in the table."))
        self.parameter_table = QTableWidget(0, 8)
        self.parameter_table.setHorizontalHeaderLabels(
            ["ID", "Label", "Group", "Type", "Default", "Unit", "External input", "Description"]
        )
        self.parameter_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        param_layout.addWidget(self.parameter_table)
        param_buttons = QHBoxLayout()
        add_parameter = QPushButton("Add parameter")
        add_parameter.clicked.connect(lambda: self.parameter_table.insertRow(self.parameter_table.rowCount()))
        remove_parameter = QPushButton("Remove selected")
        remove_parameter.clicked.connect(lambda: self._remove_selected(self.parameter_table))
        param_buttons.addWidget(add_parameter)
        param_buttons.addWidget(remove_parameter)
        param_buttons.addStretch(1)
        param_layout.addLayout(param_buttons)
        self.tabs.addTab(parameters, "Parameters")

        initializers = QWidget()
        init_layout = QVBoxLayout(initializers)
        init_layout.addWidget(QLabel("Initializer arguments use parameter IDs; $instance identifies the controller object."))
        self.initializer_table = QTableWidget(0, 6)
        self.initializer_table.setHorizontalHeaderLabels(
            ["ID", "Label", "Init function", "Step function", "Arguments (comma separated)", "Reference model"]
        )
        self.initializer_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        init_layout.addWidget(self.initializer_table)
        init_buttons = QHBoxLayout()
        add_initializer = QPushButton("Add initializer")
        add_initializer.clicked.connect(lambda: self.initializer_table.insertRow(self.initializer_table.rowCount()))
        remove_initializer = QPushButton("Remove selected")
        remove_initializer.clicked.connect(lambda: self._remove_selected(self.initializer_table))
        init_buttons.addWidget(add_initializer)
        init_buttons.addWidget(remove_initializer)
        init_buttons.addStretch(1)
        init_layout.addLayout(init_buttons)
        self.tabs.addTab(initializers, "Initializers")

        analysis = QWidget()
        analysis_form = QFormLayout(analysis)
        self.freq_min_edit = QLineEdit()
        self.freq_max_edit = QLineEdit()
        self.freq_points_edit = QLineEdit()
        self.amplitude_edit = QLineEdit()
        self.bias_edit = QLineEdit()
        self.settling_edit = QLineEdit()
        self.measurement_edit = QLineEdit()
        analysis_form.addRow("Minimum frequency [Hz]", self.freq_min_edit)
        analysis_form.addRow("Maximum frequency [Hz]", self.freq_max_edit)
        analysis_form.addRow("Frequency points", self.freq_points_edit)
        analysis_form.addRow("Excitation amplitude", self.amplitude_edit)
        analysis_form.addRow("Excitation bias / operating point", self.bias_edit)
        analysis_form.addRow("Settling periods", self.settling_edit)
        analysis_form.addRow("Measurement periods", self.measurement_edit)
        self.tabs.addTab(analysis, "Analysis")

        raw = QWidget()
        raw_layout = QVBoxLayout(raw)
        self.raw_preview = QTextEdit()
        self.raw_preview.setReadOnly(True)
        raw_layout.addWidget(self.raw_preview)
        self.tabs.addTab(raw, "JSON Preview")

        code = QWidget()
        code_layout = QVBoxLayout(code)
        code_layout.addWidget(QLabel("Generated C MEX S-Function preview for the selected component."))
        self.code_preview = QTextEdit()
        self.code_preview.setReadOnly(True)
        self.code_preview.setLineWrapMode(QTextEdit.NoWrap)
        code_layout.addWidget(self.code_preview)
        self.tabs.addTab(code, "Generated Code")

        self.status = QLabel()
        self.status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.status)
        self.setCentralWidget(root)

    def load(self, path: Path) -> None:
        self.config_path = path.resolve()
        self.data = json.loads(self.config_path.read_text(encoding="utf-8"))
        impl = self.data.get("implementation", {})
        self.id_edit.setText(self.data.get("id", ""))
        self.name_edit.setText(self.data.get("display_name", ""))
        self.description_edit.setPlainText(self.data.get("description", ""))
        self.system_type_edit.setText(self.data.get("system_type", "siso"))
        self.template_edit.setText(self.data.get("template", "generic_stateful_v1"))
        self.variant_edit.setText(self.data.get("variant", ""))
        self.instance_edit.setText(impl.get("instance_type", ""))
        self._fill_table(self.input_table, self.data.get("inputs", []), ["id", "label", "width", "type"])
        self._fill_table(self.output_table, self.data.get("outputs", []), ["id", "label", "width", "type"])
        self.headers_edit.setPlainText("\n".join(impl.get("headers", [])))
        self.sources_edit.setPlainText("\n".join(impl.get("sources", [])))
        parameter_rows = []
        for item in self.data.get("parameters", []):
            parameter_rows.append({**item, "externalizable": "yes" if item.get("externalizable", False) else "no"})
        self._fill_table(
            self.parameter_table,
            parameter_rows,
            ["id", "label", "group", "type", "default", "unit", "externalizable", "description"],
        )
        for row, item in enumerate(self.data.get("parameters", [])):
            checkbox = QTableWidgetItem()
            checkbox.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsUserCheckable)
            checkbox.setCheckState(Qt.Checked if item.get("externalizable", False) else Qt.Unchecked)
            checkbox.setToolTip("Allow the Mask value to be replaced by a Simulink input port")
            self.parameter_table.setItem(row, 6, checkbox)
        init_rows = []
        for item in self.data.get("initializers", []):
            init_rows.append({**item, "arguments": ", ".join(item.get("arguments", []))})
        self._fill_table(
            self.initializer_table,
            init_rows,
            ["id", "label", "function", "step_function", "arguments", "reference_model"],
        )
        analysis = self.data.get("analysis", {})
        frequency = analysis.get("default_frequency_hz", [1.0, 1000.0])
        self.freq_min_edit.setText(str(frequency[0]))
        self.freq_max_edit.setText(str(frequency[1]))
        self.freq_points_edit.setText(str(analysis.get("default_points", 40)))
        self.amplitude_edit.setText(str(analysis.get("excitation_amplitude", 0.001)))
        self.bias_edit.setText(str(analysis.get("excitation_bias", 0.0)))
        self.settling_edit.setText(str(analysis.get("settling_periods", 8)))
        self.measurement_edit.setText(str(analysis.get("measurement_periods", 8)))
        self._refresh_preview()
        self._refresh_code_preview()
        self.status.setText(str(self.config_path))

    def refresh_tree(self) -> None:
        self.component_tree.clear()
        nodes: dict[tuple[str, ...], QTreeWidgetItem] = {}
        for path in sorted((self.tool_root / "components").glob("*.json")):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            parent = self.component_tree.invisibleRootItem()
            category_path: tuple[str, ...] = ()
            for category in data.get("category", ["Uncategorized"]):
                category_path += (str(category),)
                if category_path not in nodes:
                    node = QTreeWidgetItem([str(category)])
                    parent.addChild(node)
                    nodes[category_path] = node
                parent = nodes[category_path]
            leaf = QTreeWidgetItem([data.get("display_name", path.stem)])
            leaf.setData(0, Qt.UserRole, str(path.resolve()))
            leaf.setToolTip(0, data.get("id", path.stem))
            parent.addChild(leaf)
        self.component_tree.expandAll()

    def _tree_item_opened(self, item: QTreeWidgetItem) -> None:
        value = item.data(0, Qt.UserRole)
        if value:
            self.load(Path(value))

    @staticmethod
    def _fill_table(table: QTableWidget, rows: list[dict], keys: list[str]) -> None:
        table.setRowCount(len(rows))
        for row, item in enumerate(rows):
            for column, key in enumerate(keys):
                table.setItem(row, column, QTableWidgetItem(str(item.get(key, ""))))

    @staticmethod
    def _cell(table: QTableWidget, row: int, column: int) -> str:
        item = table.item(row, column)
        return item.text().strip() if item else ""

    @staticmethod
    def _remove_selected(table: QTableWidget) -> None:
        rows = sorted({index.row() for index in table.selectedIndexes()}, reverse=True)
        for row in rows:
            table.removeRow(row)

    def collect(self) -> dict:
        data = json.loads(json.dumps(self.data))
        data["id"] = self.id_edit.text().strip()
        data["display_name"] = self.name_edit.text().strip()
        data["description"] = self.description_edit.toPlainText().strip()
        data["system_type"] = self.system_type_edit.text().strip()
        data["template"] = self.template_edit.text().strip()
        variant = self.variant_edit.text().strip()
        if variant:
            data["variant"] = variant
        else:
            data.pop("variant", None)
        data.setdefault("implementation", {})["instance_type"] = self.instance_edit.text().strip()
        data["implementation"]["headers"] = [x.strip() for x in self.headers_edit.toPlainText().splitlines() if x.strip()]
        data["implementation"]["sources"] = [x.strip() for x in self.sources_edit.toPlainText().splitlines() if x.strip()]
        data["inputs"] = [
            {"id": self._cell(self.input_table, row, 0), "label": self._cell(self.input_table, row, 1),
             "width": int(self._cell(self.input_table, row, 2) or "1"),
             "type": self._cell(self.input_table, row, 3) or "double"}
            for row in range(self.input_table.rowCount())
        ]
        data["outputs"] = [
            {"id": self._cell(self.output_table, row, 0), "label": self._cell(self.output_table, row, 1),
             "width": int(self._cell(self.output_table, row, 2) or "1"),
             "type": self._cell(self.output_table, row, 3) or "double"}
            for row in range(self.output_table.rowCount())
        ]
        params = []
        for row in range(self.parameter_table.rowCount()):
            default_text = self._cell(self.parameter_table, row, 4)
            try:
                default: object = float(default_text)
                if default_text.isdigit():
                    default = int(default_text)
            except ValueError:
                default = default_text
            params.append(
                {
                    "id": self._cell(self.parameter_table, row, 0),
                    "label": self._cell(self.parameter_table, row, 1),
                    "group": self._cell(self.parameter_table, row, 2) or "General",
                    "type": self._cell(self.parameter_table, row, 3),
                    "default": default,
                    "unit": self._cell(self.parameter_table, row, 5),
                    "externalizable": bool(
                        self.parameter_table.item(row, 6)
                        and self.parameter_table.item(row, 6).checkState() == Qt.Checked
                    ),
                    "description": self._cell(self.parameter_table, row, 7),
                }
            )
        data["parameters"] = params
        initializers = []
        for row in range(self.initializer_table.rowCount()):
            initializers.append(
                {
                    "id": self._cell(self.initializer_table, row, 0),
                    "label": self._cell(self.initializer_table, row, 1),
                    "function": self._cell(self.initializer_table, row, 2),
                    "step_function": self._cell(self.initializer_table, row, 3),
                    "arguments": [x.strip() for x in self._cell(self.initializer_table, row, 4).split(",") if x.strip()],
                    "reference_model": self._cell(self.initializer_table, row, 5),
                }
            )
        data["initializers"] = initializers
        data["analysis"] = {
            "default_frequency_hz": [float(self.freq_min_edit.text()), float(self.freq_max_edit.text())],
            "default_points": int(self.freq_points_edit.text()),
            "excitation_amplitude": float(self.amplitude_edit.text()),
            "excitation_bias": float(self.bias_edit.text()),
            "settling_periods": int(self.settling_edit.text()),
            "measurement_periods": int(self.measurement_edit.text()),
        }
        return data

    def _refresh_preview(self) -> None:
        self.raw_preview.setPlainText(json.dumps(self.collect(), ensure_ascii=False, indent=2))

    def _refresh_code_preview(self) -> None:
        try:
            component = ComponentDefinition.load(self.config_path)
            self.code_preview.setPlainText(ComponentGenerator(self.tool_root).preview(component))
        except Exception as exc:
            self.code_preview.setPlainText(f"Code preview unavailable:\n{exc}")

    def open_file(self) -> None:
        value, _ = QFileDialog.getOpenFileName(self, "Open component", str(self.config_path.parent), "JSON (*.json)")
        if value:
            self.load(Path(value))

    def _save_or_raise(self) -> None:
        data = self.collect()
        ComponentDefinition(data=data, path=self.config_path).validate()
        self.config_path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.data = data
        self._refresh_preview()
        self._refresh_code_preview()
        self.refresh_tree()
        self.status.setText(f"Saved {self.config_path}")

    def save(self) -> None:
        try:
            self._save_or_raise()
        except Exception as exc:
            QMessageBox.critical(self, "Save failed", str(exc))

    def validate(self) -> None:
        try:
            self._save_or_raise()
            component = ComponentDefinition.load(self.config_path)
            self.status.setText(f"Valid: {component.component_id}")
            QMessageBox.information(self, "Validation", "Component definition is valid.")
        except Exception as exc:
            QMessageBox.critical(self, "Validation failed", str(exc))

    def generate(self) -> None:
        try:
            self._save_or_raise()
            configs = sorted((self.tool_root / "components").glob("*.json"))
            outputs = ComponentGenerator(self.tool_root).generate(configs, self.tool_root / "build")
            self.status.setText(f"Generated {len(outputs)} files under {self.tool_root / 'build'}")
        except Exception as exc:
            QMessageBox.critical(self, "Generation failed", str(exc))

    def preview_code(self) -> None:
        try:
            self._save_or_raise()
            self._refresh_code_preview()
            self.tabs.setCurrentWidget(self.code_preview.parentWidget())
        except Exception as exc:
            QMessageBox.critical(self, "Code preview failed", str(exc))

    def duplicate_component(self) -> None:
        target, _ = QFileDialog.getSaveFileName(
            self, "Duplicate component", str(self.tool_root / "components" / f"{self.config_path.stem}_copy.json"), "JSON (*.json)"
        )
        if not target:
            return
        path = Path(target).resolve()
        if path.parent != (self.tool_root / "components").resolve():
            QMessageBox.critical(self, "Invalid location", "Components must be stored in the components directory.")
            return
        data = self.collect()
        data["id"] = path.stem.replace("_", ".")
        data["display_name"] = f"{data.get('display_name', path.stem)} Copy"
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.refresh_tree()
        self.load(path)

    def delete_component(self) -> None:
        components_dir = (self.tool_root / "components").resolve()
        if self.config_path.parent != components_dir:
            return
        answer = QMessageBox.question(self, "Delete component", f"Delete {self.config_path.name}?")
        if answer != QMessageBox.Yes:
            return
        candidates = [path for path in sorted(components_dir.glob("*.json")) if path.resolve() != self.config_path]
        if not candidates:
            QMessageBox.critical(self, "Delete component", "At least one component definition must remain.")
            return
        self.config_path.unlink()
        self.refresh_tree()
        self.load(candidates[0])

    def install_matlab(self) -> None:
        try:
            self.generate()
            script = self.tool_root / "matlab" / "install_gmp_matlab_components.m"
            subprocess.Popen(["matlab", "-batch", f"run('{script.as_posix()}')"])
            self.status.setText("MATLAB installation started in a separate process.")
        except Exception as exc:
            QMessageBox.critical(self, "MATLAB launch failed", str(exc))


def run_gui(config_path: Path) -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    window = BuilderWindow(config_path)
    window.show()
    return app.exec()
