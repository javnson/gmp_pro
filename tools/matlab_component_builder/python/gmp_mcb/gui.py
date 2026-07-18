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
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
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
        self.load(self.config_path)

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        toolbar = QHBoxLayout()
        for text, slot in (
            ("Open", self.open_file),
            ("Save", self.save),
            ("Validate", self.validate),
            ("Generate", self.generate),
            ("Install in MATLAB", self.install_matlab),
        ):
            button = QPushButton(text)
            button.clicked.connect(slot)
            toolbar.addWidget(button)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 1)

        general = QWidget()
        form = QFormLayout(general)
        self.id_edit = QLineEdit()
        self.name_edit = QLineEdit()
        self.description_edit = QTextEdit()
        self.description_edit.setMaximumHeight(90)
        self.instance_edit = QLineEdit()
        self.input_id_edit = QLineEdit()
        self.input_label_edit = QLineEdit()
        self.output_id_edit = QLineEdit()
        self.output_label_edit = QLineEdit()
        self.headers_edit = QTextEdit()
        self.headers_edit.setMaximumHeight(80)
        self.sources_edit = QTextEdit()
        self.sources_edit.setMaximumHeight(80)
        form.addRow("Component ID", self.id_edit)
        form.addRow("Display name", self.name_edit)
        form.addRow("Description", self.description_edit)
        form.addRow("C instance type", self.instance_edit)
        form.addRow("Input ID", self.input_id_edit)
        form.addRow("Input label", self.input_label_edit)
        form.addRow("Output ID", self.output_id_edit)
        form.addRow("Output label", self.output_label_edit)
        form.addRow("Headers (one per line)", self.headers_edit)
        form.addRow("Sources (one per line)", self.sources_edit)
        self.tabs.addTab(general, "Component")

        parameters = QWidget()
        param_layout = QVBoxLayout(parameters)
        param_layout.addWidget(QLabel("Mask parameters. Edit values directly in the table."))
        self.parameter_table = QTableWidget(0, 6)
        self.parameter_table.setHorizontalHeaderLabels(["ID", "Label", "Type", "Default", "Unit", "Description"])
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
        self.settling_edit = QLineEdit()
        self.measurement_edit = QLineEdit()
        analysis_form.addRow("Minimum frequency [Hz]", self.freq_min_edit)
        analysis_form.addRow("Maximum frequency [Hz]", self.freq_max_edit)
        analysis_form.addRow("Frequency points", self.freq_points_edit)
        analysis_form.addRow("Excitation amplitude", self.amplitude_edit)
        analysis_form.addRow("Settling periods", self.settling_edit)
        analysis_form.addRow("Measurement periods", self.measurement_edit)
        self.tabs.addTab(analysis, "Analysis")

        raw = QWidget()
        raw_layout = QVBoxLayout(raw)
        self.raw_preview = QTextEdit()
        self.raw_preview.setReadOnly(True)
        raw_layout.addWidget(self.raw_preview)
        self.tabs.addTab(raw, "JSON Preview")

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
        self.instance_edit.setText(impl.get("instance_type", ""))
        inputs = self.data.get("inputs", [{}])
        outputs = self.data.get("outputs", [{}])
        self.input_id_edit.setText(inputs[0].get("id", "") if inputs else "")
        self.input_label_edit.setText(inputs[0].get("label", "") if inputs else "")
        self.output_id_edit.setText(outputs[0].get("id", "") if outputs else "")
        self.output_label_edit.setText(outputs[0].get("label", "") if outputs else "")
        self.headers_edit.setPlainText("\n".join(impl.get("headers", [])))
        self.sources_edit.setPlainText("\n".join(impl.get("sources", [])))
        self._fill_table(self.parameter_table, self.data.get("parameters", []), ["id", "label", "type", "default", "unit", "description"])
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
        self.settling_edit.setText(str(analysis.get("settling_periods", 8)))
        self.measurement_edit.setText(str(analysis.get("measurement_periods", 8)))
        self._refresh_preview()
        self.status.setText(str(self.config_path))

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
        data.setdefault("implementation", {})["instance_type"] = self.instance_edit.text().strip()
        data["implementation"]["headers"] = [x.strip() for x in self.headers_edit.toPlainText().splitlines() if x.strip()]
        data["implementation"]["sources"] = [x.strip() for x in self.sources_edit.toPlainText().splitlines() if x.strip()]
        data.setdefault("inputs", [{}])[0].update(
            {"id": self.input_id_edit.text().strip(), "label": self.input_label_edit.text().strip()}
        )
        data.setdefault("outputs", [{}])[0].update(
            {"id": self.output_id_edit.text().strip(), "label": self.output_label_edit.text().strip()}
        )
        params = []
        for row in range(self.parameter_table.rowCount()):
            default_text = self._cell(self.parameter_table, row, 3)
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
                    "type": self._cell(self.parameter_table, row, 2),
                    "default": default,
                    "unit": self._cell(self.parameter_table, row, 4),
                    "description": self._cell(self.parameter_table, row, 5),
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
            "settling_periods": int(self.settling_edit.text()),
            "measurement_periods": int(self.measurement_edit.text()),
        }
        return data

    def _refresh_preview(self) -> None:
        self.raw_preview.setPlainText(json.dumps(self.collect(), ensure_ascii=False, indent=2))

    def open_file(self) -> None:
        value, _ = QFileDialog.getOpenFileName(self, "Open component", str(self.config_path.parent), "JSON (*.json)")
        if value:
            self.load(Path(value))

    def save(self) -> None:
        try:
            self.data = self.collect()
            self.config_path.write_text(json.dumps(self.data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            self._refresh_preview()
            self.status.setText(f"Saved {self.config_path}")
        except Exception as exc:
            QMessageBox.critical(self, "Save failed", str(exc))

    def validate(self) -> None:
        try:
            self.save()
            component = ComponentDefinition.load(self.config_path)
            self.status.setText(f"Valid: {component.component_id}")
            QMessageBox.information(self, "Validation", "Component definition is valid.")
        except Exception as exc:
            QMessageBox.critical(self, "Validation failed", str(exc))

    def generate(self) -> None:
        try:
            self.save()
            outputs = ComponentGenerator(self.tool_root).generate([self.config_path], self.tool_root / "build")
            self.status.setText(f"Generated {len(outputs)} files under {self.tool_root / 'build'}")
        except Exception as exc:
            QMessageBox.critical(self, "Generation failed", str(exc))

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
