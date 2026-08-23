"""Interactive multi-plot viewer for large CCTL simulation CSV files."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

# pyqtgraph otherwise auto-selects the first installed Qt binding.  GMP's
# installer currently provides both PySide6 and PyQt5, while this application
# uses PyQt5 widgets explicitly; mixing those runtimes produces a misleading
# "Must construct a QApplication" failure at the first PlotWidget.
os.environ["PYQTGRAPH_QT_LIB"] = "PyQt5"
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from result_data import (
    IncrementalResultReader,
    ResultChunk,
    ResultFile,
    inspect_result_file,
    load_numeric_columns,
    minmax_decimate,
)
from simulation_manager import SimulationProcessManager


LIVE_REFRESH_INTERVAL_MS = 50


class WorkerSignals(QtCore.QObject):
    finished = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)


class ColumnWorker(QtCore.QRunnable):
    def __init__(self, result: ResultFile, columns: tuple[str, ...]):
        super().__init__()
        self.result = result
        self.columns = columns
        self.signals = WorkerSignals()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            self.signals.finished.emit(load_numeric_columns(self.result, self.columns))
        except Exception as error:  # GUI boundary: present a useful error instead of terminating.
            self.signals.failed.emit(str(error))


class MultiColumnWorker(QtCore.QRunnable):
    """Load decorated columns from several independently sampled files."""

    def __init__(self, requests: tuple[tuple[ResultFile, str, str], ...]):
        super().__init__()
        self.requests = requests
        self.signals = WorkerSignals()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            loaded: dict[str, np.ndarray] = {}
            for result in dict.fromkeys(request[0] for request in self.requests):
                selected = [request for request in self.requests if request[0] == result]
                values = load_numeric_columns(result, [item[1] for item in selected])
                for _source, raw_name, display_name in selected:
                    loaded[display_name] = values[raw_name]
            self.signals.finished.emit(loaded)
        except Exception as error:
            self.signals.failed.emit(str(error))


class LiveInitializeWorker(QtCore.QRunnable):
    def __init__(self, result: ResultFile, columns: tuple[str, ...]):
        super().__init__()
        self.result = result
        self.columns = columns
        self.signals = WorkerSignals()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            reader = IncrementalResultReader(self.result, self.columns)
            self.signals.finished.emit((reader, reader.read_available()))
        except Exception as error:  # GUI boundary.
            self.signals.failed.emit(str(error))


class LiveTailWorker(QtCore.QRunnable):
    def __init__(self, reader: IncrementalResultReader):
        super().__init__()
        self.reader = reader
        self.signals = WorkerSignals()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            self.signals.finished.emit((self.reader, self.reader.read_available()))
        except Exception as error:  # GUI boundary.
            self.signals.failed.emit(str(error))


class MultiLiveInitializeWorker(QtCore.QRunnable):
    def __init__(self, requests: tuple[tuple[ResultFile, str, str], ...]):
        super().__init__()
        self.requests = requests
        self.signals = WorkerSignals()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            payload = []
            for result in dict.fromkeys(item[0] for item in self.requests):
                selected = [item for item in self.requests if item[0] == result]
                mapping = {raw: display for _result, raw, display in selected}
                reader = IncrementalResultReader(result, mapping)
                payload.append((reader, reader.read_available(), mapping))
            self.signals.finished.emit(payload)
        except Exception as error:
            self.signals.failed.emit(str(error))


class MultiLiveTailWorker(QtCore.QRunnable):
    def __init__(self, readers: list[tuple[IncrementalResultReader, dict[str, str]]]):
        super().__init__()
        self.readers = readers
        self.signals = WorkerSignals()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            self.signals.finished.emit([
                (reader, reader.read_available(), mapping)
                for reader, mapping in self.readers
            ])
        except Exception as error:
            self.signals.failed.emit(str(error))


class EditableTitleLabel(pg.LabelItem):
    double_clicked = QtCore.pyqtSignal()

    def mouseDoubleClickEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:
        self.double_clicked.emit()
        event.accept()


class PlotPanel(QtWidgets.QFrame):
    activated = QtCore.pyqtSignal(object)
    title_changed = QtCore.pyqtSignal()

    def __init__(self, number: int):
        super().__init__()
        self.setObjectName("PlotPanel")
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setMinimumHeight(140)
        self.curves: dict[str, pg.PlotDataItem] = {}
        self.x_name: str | None = None
        self.title_text = f"Plot {number}"
        self.plot = pg.PlotWidget()
        self.plot.installEventFilter(self)
        self.plot.scene().sigMouseClicked.connect(
            lambda _event: self.activated.emit(self)
        )
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.addLegend()
        self.plot.setDownsampling(auto=True, mode="peak")
        self.plot.setClipToView(True)
        plot_item = self.plot.getPlotItem()
        old_title = plot_item.titleLabel
        plot_item.layout.removeItem(old_title)
        old_title.setParentItem(None)
        self.title_label = EditableTitleLabel(justify="center")
        self.title_label.double_clicked.connect(self.edit_title)
        plot_item.layout.addItem(self.title_label, 0, 1)
        plot_item.titleLabel = self.title_label
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(3, 3, 3, 3)
        layout.addWidget(self.plot)
        self.plot.setTitle(self.title_text)

    def edit_title(self) -> None:
        self.activated.emit(self)
        title, accepted = QtWidgets.QInputDialog.getText(
            self,
            "Edit plot title",
            "Title",
            QtWidgets.QLineEdit.Normal,
            self.title_text,
        )
        if accepted and title.strip():
            self.title_text = title.strip()
            self.plot.setTitle(self.title_text)
            self.title_changed.emit()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        self.activated.emit(self)
        super().mousePressEvent(event)

    def eventFilter(self, watched: QtCore.QObject, event: QtCore.QEvent) -> bool:
        if watched is self.plot and event.type() == QtCore.QEvent.MouseButtonPress:
            self.activated.emit(self)
        return super().eventFilter(watched, event)

    def set_active(self, active: bool) -> None:
        self.setStyleSheet(
            "QFrame#PlotPanel { border: 2px solid #3584e4; }" if active else ""
        )

    def set_interaction_mode(self, mode: str) -> None:
        view = self.plot.getViewBox()
        if mode == "pan":
            view.setMouseEnabled(x=True, y=True)
            view.setMouseMode(view.PanMode)
        elif mode == "x_zoom":
            view.setMouseEnabled(x=True, y=False)
            view.setMouseMode(view.RectMode)
        elif mode == "y_zoom":
            view.setMouseEnabled(x=False, y=True)
            view.setMouseMode(view.RectMode)
        elif mode == "box_zoom":
            view.setMouseEnabled(x=True, y=True)
            view.setMouseMode(view.RectMode)
        else:
            raise ValueError(f"unknown plot interaction mode: {mode}")


class ResultViewer(QtWidgets.QMainWindow):
    COLORS = (
        "#3584e4", "#e01b24", "#33d17a", "#f6d32d", "#9141ac",
        "#ff7800", "#1c71d8", "#c061cb", "#26a269", "#a51d2d",
    )

    def __init__(self):
        super().__init__()
        self.setWindowTitle("GMP CCTL Simulation Viewer Manager")
        self.resize(1450, 900)
        self.result: ResultFile | None = None
        self.results: list[ResultFile] = []
        self.bindings: dict[str, tuple[ResultFile, str]] = {}
        self.time_keys: dict[Path, str] = {}
        self.cache: dict[str, np.ndarray] = {}
        self.panels: list[PlotPanel] = []
        self.active_panel: PlotPanel | None = None
        self.pending: tuple[PlotPanel, tuple[str, ...], str] | None = None
        self.live_reader: IncrementalResultReader | None = None
        self.live_readers: list[tuple[IncrementalResultReader, dict[str, str]]] = []
        self.live_initializing = False
        self.live_poll_pending = False
        self.live_skipped_rows = 0
        self.live_generation = 0
        self.file_generation = 0
        self.interaction_mode = "pan"
        self.next_plot_number = 1
        self.pool = QtCore.QThreadPool.globalInstance()
        self.simulation = SimulationProcessManager(self)
        self.close_after_simulation = False
        self._build_ui()
        self._build_toolbar()
        self.simulation.message_received.connect(self._simulation_message)
        self.simulation.log_received.connect(self._simulation_log)
        self.simulation.state_changed.connect(self._simulation_state_changed)
        self.simulation.outputs_ready.connect(self._simulation_outputs_ready)
        self.simulation.process_finished.connect(self._simulation_finished)
        self.live_timer = QtCore.QTimer(self)
        self.live_timer.setInterval(LIVE_REFRESH_INTERVAL_MS)
        self.live_timer.timeout.connect(self.poll_live_file)
        self.add_plot()

    def _build_ui(self) -> None:
        root = QtWidgets.QSplitter()
        controls = QtWidgets.QWidget()
        form = QtWidgets.QVBoxLayout(controls)
        open_button = QtWidgets.QPushButton("Open CSV / TSV…")
        open_button.clicked.connect(self.open_dialog)
        self.file_label = QtWidgets.QLabel("No result file loaded")
        self.file_label.setWordWrap(True)
        self.x_column = QtWidgets.QComboBox()
        self.columns = QtWidgets.QListWidget()
        self.columns.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.columns.itemDoubleClicked.connect(self.add_double_clicked_curve)
        add_plot = QtWidgets.QPushButton("Add plot")
        add_plot.clicked.connect(self.add_plot)
        remove_plot = QtWidgets.QPushButton("Remove active plot")
        remove_plot.clicked.connect(self.remove_active_plot)
        add_curves = QtWidgets.QPushButton("Add selected curves")
        add_curves.clicked.connect(self.add_selected_curves)
        self.active_curve_label = QtWidgets.QLabel("Curves in active plot")
        self.active_curves = QtWidgets.QListWidget()
        self.active_curves.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection
        )
        remove_curves = QtWidgets.QPushButton("Remove curves from active plot")
        remove_curves.clicked.connect(self.remove_selected_curves)
        QtWidgets.QShortcut(
            QtGui.QKeySequence.Delete,
            self.active_curves,
            activated=self.remove_selected_curves,
        )
        clear_curves = QtWidgets.QPushButton("Clear active plot")
        clear_curves.clicked.connect(self.clear_active_plot)
        self.link_x = QtWidgets.QCheckBox("Link X zoom across plots")
        self.link_x.setChecked(True)
        self.link_y = QtWidgets.QCheckBox("Link Y zoom across plots")
        self.link_x.toggled.connect(self.apply_links)
        self.link_y.toggled.connect(self.apply_links)
        self.maximum_points = QtWidgets.QSpinBox()
        self.maximum_points.setRange(1_000, 2_000_000)
        self.maximum_points.setSingleStep(10_000)
        self.maximum_points.setValue(100_000)
        self.maximum_points.setToolTip("Per-curve display point limit; extrema are preserved")
        self.dynamic_refresh = QtWidgets.QCheckBox("Dynamic refresh (20 Hz)")
        self.dynamic_refresh.setToolTip(
            "Incrementally read rows appended by a running simulation every 50 ms"
        )
        self.dynamic_refresh.toggled.connect(self.set_dynamic_refresh)
        self.rolling_x = QtWidgets.QCheckBox("Rolling X window")
        self.rolling_x.setToolTip(
            "Keep a fixed time span ending at the newest complete sample"
        )
        self.rolling_x.toggled.connect(self.set_rolling_window)
        self.rolling_window_seconds = QtWidgets.QDoubleSpinBox()
        self.rolling_window_seconds.setDecimals(9)
        self.rolling_window_seconds.setRange(0.000001, 1_000_000.0)
        self.rolling_window_seconds.setValue(0.1)
        self.rolling_window_seconds.setSuffix(" s")
        self.rolling_window_seconds.setToolTip("Visible horizontal time span")
        self.rolling_window_seconds.valueChanged.connect(
            lambda _value: self.apply_rolling_window()
        )
        self.status_label = QtWidgets.QLabel()
        self.status_label.setWordWrap(True)
        form.addWidget(open_button)
        form.addWidget(self.file_label)
        form.addWidget(QtWidgets.QLabel("X axis"))
        form.addWidget(self.x_column)
        form.addWidget(QtWidgets.QLabel("Y columns (multi-select)"))
        form.addWidget(self.columns, 1)
        form.addWidget(add_curves)
        form.addWidget(self.active_curve_label)
        form.addWidget(self.active_curves)
        form.addWidget(remove_curves)
        form.addWidget(clear_curves)
        form.addWidget(add_plot)
        form.addWidget(remove_plot)
        form.addWidget(self.link_x)
        form.addWidget(self.link_y)
        form.addWidget(self.dynamic_refresh)
        form.addWidget(self.rolling_x)
        form.addWidget(self.rolling_window_seconds)
        form.addWidget(QtWidgets.QLabel("Maximum display points / curve"))
        form.addWidget(self.maximum_points)
        form.addWidget(self.status_label)

        self.plot_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.plot_splitter.setChildrenCollapsible(False)
        self.plot_splitter.setHandleWidth(8)
        root.addWidget(controls)
        root.addWidget(self.plot_splitter)
        root.setSizes([300, 1150])
        self.setCentralWidget(root)
        self._build_simulation_dock()

    def _build_simulation_dock(self) -> None:
        dock = QtWidgets.QDockWidget("Simulation", self)
        dock.setObjectName("cctlSimulationDock")
        content = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(content)
        self.simulator_path = QtWidgets.QLineEdit()
        browse_simulator = QtWidgets.QPushButton("Browse…")
        browse_simulator.clicked.connect(self.browse_simulator)
        self.simulation_output_path = QtWidgets.QLineEdit(
            str(Path.cwd() / "cctl_simulation.csv")
        )
        browse_output = QtWidgets.QPushButton("Output…")
        browse_output.clicked.connect(self.browse_simulation_output)
        self.simulation_duration = QtWidgets.QDoubleSpinBox()
        self.simulation_duration.setDecimals(6)
        self.simulation_duration.setRange(0.0, 1_000_000.0)
        self.simulation_duration.setValue(4.0)
        self.simulation_duration.setSuffix(" s")
        self.simulation_duration.setSpecialValueText("Unlimited")
        self.simulation_duration.editingFinished.connect(
            self.update_simulation_duration
        )
        self.simulation_start = QtWidgets.QPushButton("Start")
        self.simulation_start.clicked.connect(self.start_managed_simulation)
        self.simulation_pause = QtWidgets.QPushButton("Pause")
        self.simulation_pause.clicked.connect(self.simulation.pause)
        self.simulation_resume = QtWidgets.QPushButton("Resume")
        self.simulation_resume.clicked.connect(self.simulation.resume)
        self.simulation_stop = QtWidgets.QPushButton("Stop")
        self.simulation_stop.clicked.connect(self.simulation.stop)
        self.simulation_progress = QtWidgets.QProgressBar()
        self.simulation_progress.setRange(0, 1000)
        self.simulation_progress.setValue(0)
        self.simulation_progress.setFormat("Idle")
        self.simulation_metrics = QtWidgets.QLabel("state=idle")
        self.simulation_log = QtWidgets.QPlainTextEdit()
        self.simulation_log.setReadOnly(True)
        self.simulation_log.setMaximumBlockCount(5000)
        self.simulation_log.setMinimumHeight(100)
        layout.addWidget(QtWidgets.QLabel("Simulator"), 0, 0)
        layout.addWidget(self.simulator_path, 0, 1, 1, 4)
        layout.addWidget(browse_simulator, 0, 5)
        layout.addWidget(QtWidgets.QLabel("Output base"), 1, 0)
        layout.addWidget(self.simulation_output_path, 1, 1, 1, 4)
        layout.addWidget(browse_output, 1, 5)
        layout.addWidget(QtWidgets.QLabel("Target"), 2, 0)
        layout.addWidget(self.simulation_duration, 2, 1)
        layout.addWidget(self.simulation_start, 2, 2)
        layout.addWidget(self.simulation_pause, 2, 3)
        layout.addWidget(self.simulation_resume, 2, 4)
        layout.addWidget(self.simulation_stop, 2, 5)
        layout.addWidget(self.simulation_progress, 3, 0, 1, 6)
        layout.addWidget(self.simulation_metrics, 4, 0, 1, 6)
        layout.addWidget(self.simulation_log, 5, 0, 1, 6)
        dock.setWidget(content)
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock)
        self.simulation_dock = dock
        self._simulation_state_changed("idle")

    def _build_toolbar(self) -> None:
        toolbar = self.addToolBar("Plot interaction")
        toolbar.setMovable(False)
        group = QtWidgets.QActionGroup(self)
        group.setExclusive(True)
        actions = (
            ("Pan", "pan", "Pan in both axes"),
            ("Horizontal zoom", "x_zoom", "Drag a rectangle to zoom only X"),
            ("Vertical zoom", "y_zoom", "Drag a rectangle to zoom only Y"),
            ("Magnifier", "box_zoom", "Drag a rectangle to zoom X and Y"),
        )
        self.interaction_actions: dict[str, QtWidgets.QAction] = {}
        for text, mode, tip in actions:
            action = toolbar.addAction(text)
            action.setCheckable(True)
            action.setToolTip(tip)
            action.triggered.connect(
                lambda _checked, selected=mode: self.set_interaction_mode(selected)
            )
            group.addAction(action)
            self.interaction_actions[mode] = action
        self.interaction_actions["pan"].setChecked(True)
        toolbar.addSeparator()
        fit_active = toolbar.addAction("Fit active")
        fit_active.triggered.connect(self.fit_active_plot)
        fit_all = toolbar.addAction("Fit all")
        fit_all.triggered.connect(
            lambda: [panel.plot.autoRange() for panel in self.panels]
        )

    def browse_simulator(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select CCTL simulator", "", "Executables (*.exe);;All files (*)"
        )
        if path:
            self.simulator_path.setText(path)
            if not self.simulation_output_path.isModified():
                self.simulation_output_path.setText(
                    str(Path(path).resolve().parent / "cctl_simulation.csv")
                )

    def browse_simulation_output(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Select output base", self.simulation_output_path.text(),
            "CSV files (*.csv);;All files (*)",
        )
        if path:
            self.simulation_output_path.setText(path)

    def start_managed_simulation(self) -> None:
        if self.simulation.is_active():
            if self.simulation.state == "ready":
                self.simulation.start_simulation()
            elif self.simulation.state == "paused":
                self.simulation.resume()
            return
        try:
            self.simulation_log.clear()
            self.simulation.launch(
                Path(self.simulator_path.text()),
                Path(self.simulation_output_path.text()),
                self.simulation_duration.value(),
                auto_start=True,
            )
        except Exception as error:
            QtWidgets.QMessageBox.critical(
                self, "Cannot start simulation", str(error)
            )

    def update_simulation_duration(self) -> None:
        if self.simulation.is_active():
            self.simulation.set_duration(self.simulation_duration.value())

    @QtCore.pyqtSlot(dict)
    def _simulation_message(self, message: dict) -> None:
        if message.get("type") != "status":
            if message.get("type") == "command" and not message.get("accepted", False):
                self._simulation_log(message.get("message", "Command rejected"))
            return
        completed = int(message.get("completed_steps", 0))
        target = int(message.get("target_steps", 0))
        simulated = float(message.get("simulated_time_s", 0.0))
        target_time = float(message.get("target_time_s", 0.0))
        elapsed = float(message.get("elapsed_s", 0.0))
        eta = float(message.get("eta_s", 0.0))
        rate = float(message.get("rate_steps_s", 0.0)) / 1.0e6
        queued = int(message.get("queued", 0))
        capacity = int(message.get("capacity", 0))
        dropped = int(message.get("dropped", 0))
        if target:
            self.simulation_progress.setRange(0, 1000)
            self.simulation_progress.setValue(
                min(1000, int(completed * 1000 / target))
            )
            self.simulation_progress.setFormat(
                f"{simulated:.3f}/{target_time:.3f} s  %p%"
            )
        else:
            self.simulation_progress.setRange(0, 0)
            self.simulation_progress.setFormat(f"{simulated:.3f} s")
        self.simulation_metrics.setText(
            f"state={message.get('state', '?')}  elapsed={elapsed:.1f}s  "
            f"ETA={eta:.1f}s  rate={rate:.2f}Mstep/s  "
            f"queue={queued}/{capacity}  dropped={dropped}"
        )

    @QtCore.pyqtSlot(str)
    def _simulation_log(self, text: str) -> None:
        if text:
            self.simulation_log.appendPlainText(text)

    @QtCore.pyqtSlot(str)
    def _simulation_state_changed(self, state: str) -> None:
        active = self.simulation.is_active()
        self.simulation_start.setEnabled(not active or state in {"ready", "paused"})
        self.simulation_pause.setEnabled(active and state == "running")
        self.simulation_resume.setEnabled(active and state == "paused")
        self.simulation_stop.setEnabled(active and state not in {"stopping", "completed"})
        if state in {"idle", "starting", "ready", "paused", "stopping"}:
            self.simulation_progress.setFormat(state.capitalize())
        self.simulation_metrics.setText(f"state={state}")

    @QtCore.pyqtSlot(list)
    def _simulation_outputs_ready(self, paths: list[str]) -> None:
        self._attach_simulation_outputs(tuple(Path(path) for path in paths), 0)

    def _attach_simulation_outputs(
        self, paths: tuple[Path, ...], attempt: int
    ) -> None:
        if all(path.exists() and path.stat().st_size > 0 for path in paths):
            self.open_files(list(paths))
            self.dynamic_refresh.setChecked(True)
            return
        if attempt < 100 and self.simulation.is_active():
            QtCore.QTimer.singleShot(
                50,
                lambda selected=paths, retry=attempt + 1:
                    self._attach_simulation_outputs(selected, retry),
            )
        else:
            self._simulation_log("Output files were not ready for live viewing")

    @QtCore.pyqtSlot(int, int)
    def _simulation_finished(self, exit_code: int, _exit_status: int) -> None:
        self._simulation_log(f"Simulator exited with code {exit_code}")
        if self.close_after_simulation:
            self.close_after_simulation = False
            self.close()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self.simulation.is_active():
            answer = QtWidgets.QMessageBox.question(
                self,
                "Simulation is running",
                "Stop the simulation and close the Viewer Manager?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                event.ignore()
                return
            self.close_after_simulation = True
            self.simulation.stop()
            event.ignore()
            return
        event.accept()

    def open_dialog(self) -> None:
        paths, _ = QtWidgets.QFileDialog.getOpenFileNames(
            self, "Open simulation results", "", "Delimited data (*.csv *.tsv *.txt);;All files (*)"
        )
        if paths:
            self.open_files([Path(path) for path in paths])

    def open_file(self, path: Path) -> None:
        self.open_files([path])

    def open_files(self, paths: list[Path]) -> None:
        try:
            self.results = [inspect_result_file(path) for path in paths]
            if not self.results:
                raise ValueError("no result files were selected")
            self.result = self.results[0]
        except Exception as error:
            QtWidgets.QMessageBox.critical(self, "Cannot open result", str(error))
            return
        self.live_timer.stop()
        self.live_generation += 1
        self.file_generation += 1
        self.live_reader = None
        self.live_readers = []
        self.live_initializing = False
        self.live_poll_pending = False
        self.live_skipped_rows = 0
        self.pending = None
        self.cache.clear()
        self.file_label.setText("\n".join(str(result.path) for result in self.results))
        self.bindings.clear()
        self.time_keys.clear()
        multiple = len(self.results) > 1
        for result in self.results:
            prefix = f"{result.path.name} :: " if multiple else ""
            for raw_name in result.columns:
                self.bindings[prefix + raw_name] = (result, raw_name)
            raw_time = next(
                (name for name in result.columns if name.lower() in {"time", "time_s", "t"}),
                result.columns[0],
            )
            self.time_keys[result.path] = prefix + raw_time
        self.x_column.clear()
        self.x_column.addItems(self.time_keys.values())
        self.columns.clear()
        for name in self.bindings:
            if name not in self.time_keys.values():
                self.columns.addItem(name)
        for panel in self.panels:
            panel.plot.clear()
            panel.plot.addLegend()
            panel.curves.clear()
            panel.x_name = None
        self._sync_active_curve_list()
        self.status_label.setText(
            f"{len(self.results)} file(s), {len(self.bindings)} columns; "
            "curves use each file's own time column"
        )

    def add_plot(self) -> None:
        panel = PlotPanel(self.next_plot_number)
        self.next_plot_number += 1
        panel.activated.connect(self.set_active_panel)
        panel.title_changed.connect(self._sync_active_curve_list)
        panel.set_interaction_mode(self.interaction_mode)
        previous_sizes = self.plot_splitter.sizes()
        self.plot_splitter.addWidget(panel)
        self.panels.append(panel)
        if previous_sizes:
            target = max(180, sum(previous_sizes) // len(previous_sizes))
            self.plot_splitter.setSizes([*previous_sizes, target])
        self.set_active_panel(panel)
        self.apply_links()

    def remove_active_plot(self) -> None:
        if self.active_panel is not None:
            self.remove_plot(self.active_panel)

    def remove_plot(self, panel: PlotPanel) -> None:
        if len(self.panels) == 1:
            panel.plot.clear()
            panel.plot.addLegend()
            panel.curves.clear()
            panel.x_name = None
            self._sync_active_curve_list()
            return
        self.panels.remove(panel)
        panel.setParent(None)
        panel.deleteLater()
        self.set_active_panel(self.panels[-1])
        self.apply_links()

    def set_active_panel(self, selected: PlotPanel) -> None:
        self.active_panel = selected
        for panel in self.panels:
            panel.set_active(panel is selected)
        self._sync_active_curve_list()

    def _sync_active_curve_list(self) -> None:
        self.active_curves.clear()
        if self.active_panel is None:
            self.active_curve_label.setText("Curves in active plot")
            return
        self.active_curve_label.setText(
            f"Curves in active plot: {self.active_panel.title_text}"
        )
        self.active_curves.addItems(self.active_panel.curves)

    def clear_active_plot(self) -> None:
        if self.active_panel is not None:
            self.active_panel.plot.clear()
            self.active_panel.plot.addLegend()
            self.active_panel.curves.clear()
            self.active_panel.x_name = None
            self._sync_active_curve_list()

    def remove_selected_curves(self) -> None:
        if self.active_panel is None:
            return
        for item in self.active_curves.selectedItems():
            curve = self.active_panel.curves.pop(item.text(), None)
            if curve is not None:
                self.active_panel.plot.removeItem(curve)
        self._sync_active_curve_list()

    def add_double_clicked_curve(self, item: QtWidgets.QListWidgetItem) -> None:
        self._request_curves((item.text(),))

    def add_selected_curves(self) -> None:
        y_names = tuple(item.text() for item in self.columns.selectedItems())
        if not y_names:
            self.status_label.setText("Select one or more Y columns")
            return
        self._request_curves(y_names)

    def _request_curves(self, y_names: tuple[str, ...]) -> None:
        if self.result is None or self.active_panel is None:
            return
        if self.pending is not None:
            self.status_label.setText("A column load is already in progress…")
            return
        x_name = self.x_column.currentText()
        y_names = tuple(name for name in dict.fromkeys(y_names) if name in self.bindings)
        if not y_names:
            self.status_label.setText("The X column cannot also be a Y curve")
            return
        required = []
        for y_name in y_names:
            result, _raw = self.bindings[y_name]
            required.extend((self.time_keys[result.path], y_name))
        missing = tuple(name for name in dict.fromkeys(required) if name not in self.cache)
        if missing:
            self.pending = (self.active_panel, y_names, x_name)
            if self.dynamic_refresh.isChecked():
                names = tuple(dict.fromkeys((*self.cache, *required)))
                self._start_live_initialization(names)
            else:
                self.status_label.setText("Loading selected columns in the background…")
                generation = self.file_generation
                requests = tuple(
                    (self.bindings[name][0], self.bindings[name][1], name)
                    for name in missing
                )
                worker = MultiColumnWorker(requests)
                worker.signals.finished.connect(
                    lambda columns, token=generation: self._columns_loaded(token, columns)
                )
                worker.signals.failed.connect(
                    lambda message, token=generation: self._load_failed(token, message)
                )
                self.pool.start(worker)
        else:
            self._draw(self.active_panel, y_names, x_name)

    def _columns_loaded(self, generation: int, columns: object) -> None:
        if generation != self.file_generation:
            return
        self.cache.update(columns)
        pending, self.pending = self.pending, None
        if pending is not None:
            self._draw(*pending)
        if self.dynamic_refresh.isChecked():
            self._initialize_live_reader()

    def _load_failed(self, generation: int, message: str) -> None:
        if generation != self.file_generation:
            return
        self.pending = None
        self.status_label.setText("Load failed")
        QtWidgets.QMessageBox.critical(self, "Cannot load columns", message)

    def _draw(self, panel: PlotPanel, y_names: tuple[str, ...], x_name: str) -> None:
        panel.x_name = "time"
        limit = self.maximum_points.value()
        displayed_rows: int | None = None
        for y_name in y_names:
            result, _raw = self.bindings[y_name]
            curve_x_name = self.time_keys[result.path]
            x = self.cache[curve_x_name]
            y = self.cache[y_name]
            common_rows = min(len(x), len(y))
            displayed_rows = (common_rows if displayed_rows is None
                              else min(displayed_rows, common_rows))
            xd, yd = minmax_decimate(x[:common_rows], y[:common_rows], limit)
            if y_name in panel.curves:
                panel.curves[y_name].setData(xd, yd)
            else:
                color = self.COLORS[len(panel.curves) % len(self.COLORS)]
                panel.curves[y_name] = panel.plot.plot(xd, yd, pen=pg.mkPen(color, width=1.2), name=y_name)
        if panel is self.active_panel:
            self._sync_active_curve_list()
        panel.plot.setLabel("bottom", "time")
        panel.plot.autoRange()
        self.apply_rolling_window((panel,))
        self.status_label.setText(
            f"Loaded {displayed_rows or 0:,} common rows; displaying at most "
            f"{limit:,} points per curve"
        )

    def set_interaction_mode(self, mode: str) -> None:
        self.interaction_mode = mode
        for panel in self.panels:
            panel.set_interaction_mode(mode)
        action = self.interaction_actions.get(mode)
        if action is not None:
            action.setChecked(True)

    def fit_active_plot(self) -> None:
        if self.active_panel is not None:
            self.active_panel.plot.autoRange()

    @QtCore.pyqtSlot(bool)
    def set_rolling_window(self, enabled: bool) -> None:
        if enabled:
            self.apply_rolling_window()
        else:
            for panel in self.panels:
                panel.plot.autoRange()

    def apply_rolling_window(
        self, panels: tuple[PlotPanel, ...] | None = None
    ) -> None:
        """Pin X to a trailing time interval, like a streaming oscilloscope."""
        if not self.rolling_x.isChecked():
            return
        span = self.rolling_window_seconds.value()
        for panel in panels or tuple(self.panels):
            latest: float | None = None
            for y_name in panel.curves:
                binding = self.bindings.get(y_name)
                if binding is None:
                    continue
                x = self.cache.get(self.time_keys[binding[0].path])
                if x is None or not x.size:
                    continue
                finite = x[np.isfinite(x)]
                if finite.size:
                    candidate = float(finite[-1])
                    latest = candidate if latest is None else max(latest, candidate)
            if latest is not None:
                panel.plot.setXRange(latest - span, latest, padding=0.0)

    @QtCore.pyqtSlot(bool)
    def set_dynamic_refresh(self, enabled: bool) -> None:
        was_live_initializing = self.live_initializing
        self.live_timer.stop()
        self.live_generation += 1
        self.live_reader = None
        self.live_readers = []
        self.live_poll_pending = False
        if enabled:
            self._initialize_live_reader()
        else:
            self.live_initializing = False
            if was_live_initializing:
                self.pending = None
            self.status_label.setText("Dynamic refresh disabled")

    def _initialize_live_reader(self) -> None:
        if (
            not self.dynamic_refresh.isChecked()
            or self.result is None
            or self.pending is not None
            or self.live_initializing
        ):
            return
        names = tuple(self.cache)
        if not names:
            names = tuple(self.time_keys.values())
        self._start_live_initialization(names)

    def _start_live_initialization(self, names: tuple[str, ...]) -> None:
        if self.result is None:
            return
        self.live_timer.stop()
        self.live_generation += 1
        generation = self.live_generation
        self.live_reader = None
        self.live_poll_pending = False
        self.live_initializing = True
        self.live_skipped_rows = 0
        self.status_label.setText("Preparing the 20 Hz incremental reader…")
        requests = tuple(
            (self.bindings[name][0], self.bindings[name][1], name)
            for name in names if name in self.bindings
        )
        worker = MultiLiveInitializeWorker(requests)
        worker.signals.finished.connect(
            lambda payload, token=generation: self._live_initialized(token, payload)
        )
        worker.signals.failed.connect(
            lambda message, token=generation: self._live_failed(token, message)
        )
        self.pool.start(worker)

    def _live_initialized(self, generation: int, payload: object) -> None:
        if generation != self.live_generation:
            return
        self.live_initializing = False
        if not self.dynamic_refresh.isChecked() or self.result is None:
            return
        self.live_readers = []
        chunks = []
        for reader, chunk, mapping in payload:
            self.live_readers.append((reader, mapping))
            chunks.append(chunk)
            self.live_skipped_rows += int(chunk.skipped_trailing_row)
            for raw_name, values in chunk.columns.items():
                self.cache[mapping[raw_name]] = values
        self.live_reader = self.live_readers[0][0] if self.live_readers else None
        pending, self.pending = self.pending, None
        if pending is not None:
            self._draw(*pending)
        self._refresh_all_plots()
        self.live_timer.start(LIVE_REFRESH_INTERVAL_MS)
        self._set_live_status(chunks)

    @QtCore.pyqtSlot()
    def poll_live_file(self) -> None:
        if not self.live_readers or self.live_poll_pending:
            return
        self.live_poll_pending = True
        generation = self.live_generation
        worker = MultiLiveTailWorker(self.live_readers)
        worker.signals.finished.connect(
            lambda payload, token=generation: self._live_tail_loaded(token, payload)
        )
        worker.signals.failed.connect(
            lambda message, token=generation: self._live_failed(token, message)
        )
        self.pool.start(worker)

    def _live_tail_loaded(self, generation: int, payload: object) -> None:
        if generation != self.live_generation:
            return
        self.live_poll_pending = False
        changed = False
        chunks = []
        for _reader, chunk, mapping in payload:
            chunks.append(chunk)
            if chunk.reset:
                self.live_skipped_rows = 0
            self.live_skipped_rows += int(chunk.skipped_trailing_row)
            for raw_name, values in chunk.columns.items():
                name = mapping[raw_name]
                if chunk.reset:
                    self.cache[name] = values
                    changed = True
                elif values.size:
                    existing = self.cache.get(name)
                    self.cache[name] = (
                        values if existing is None or not existing.size
                        else np.concatenate((existing, values))
                    )
                    changed = True
        if changed:
            self._refresh_all_plots()
        self._set_live_status(chunks)

    def _live_failed(self, generation: int, message: str) -> None:
        if generation != self.live_generation:
            return
        self.live_poll_pending = False
        self.live_initializing = False
        self.live_timer.stop()
        self.live_reader = None
        self.live_readers = []
        self.pending = None
        self.status_label.setText(f"Dynamic refresh stopped: {message}")

    def _refresh_all_plots(self) -> None:
        limit = self.maximum_points.value()
        for panel in self.panels:
            if panel.x_name is None:
                continue
            for y_name, curve in panel.curves.items():
                if y_name not in self.cache:
                    continue
                result, _raw = self.bindings[y_name]
                x_name = self.time_keys[result.path]
                if x_name not in self.cache:
                    continue
                x = self.cache[x_name]
                y = self.cache[y_name]
                common_rows = min(len(x), len(y))
                xd, yd = minmax_decimate(
                    x[:common_rows], y[:common_rows], limit
                )
                curve.setData(xd, yd)
        self.apply_rolling_window()

    def _set_live_status(self, chunks: list[ResultChunk]) -> None:
        rows = max((len(values) for values in self.cache.values()), default=0)
        suffix = ""
        if any(chunk.incomplete_tail for chunk in chunks):
            suffix = "; incomplete final line deferred"
        if self.live_skipped_rows:
            suffix += f"; malformed trailing rows ignored={self.live_skipped_rows}"
        self.status_label.setText(
            f"Live 20 Hz: {rows:,} complete rows loaded{suffix}"
        )

    def apply_links(self) -> None:
        if not self.panels:
            return
        views = [panel.plot.getViewBox() for panel in self.panels]
        # Clear both dimensions first so a previous X+Y link cannot survive a
        # transition to one-axis-only linking.
        for view in views:
            view.setXLink(None)
            view.setYLink(None)
        leader = views[0]
        for view in views[1:]:
            if self.link_x.isChecked():
                view.setXLink(leader)
            if self.link_y.isChecked():
                view.setYLink(leader)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="GMP CCTL large-result viewer")
    parser.add_argument("files", nargs="*")
    parser.add_argument(
        "--live", action="store_true",
        help="enable 20 Hz incremental refresh after loading the files",
    )
    parser.add_argument(
        "--rolling-window", type=float, metavar="SECONDS",
        help="pin the horizontal axis to a trailing time window",
    )
    parser.add_argument(
        "--simulator", type=Path, metavar="EXECUTABLE",
        help="preselect a CCTL simulator for managed execution",
    )
    parser.add_argument(
        "--duration", type=float, metavar="SECONDS", default=4.0,
        help="initial managed simulation target; zero means unlimited",
    )
    parser.add_argument(
        "--output", type=Path, metavar="CSV",
        help="managed simulation output base path",
    )
    parser.add_argument(
        "--autostart", action="store_true",
        help="start the preselected simulator after the window is shown",
    )
    args = parser.parse_args(argv)
    if args.duration < 0.0:
        parser.error("--duration must be nonnegative")
    if args.autostart and args.simulator is None:
        parser.error("--autostart requires --simulator")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    pg.setConfigOptions(antialias=False, background="w", foreground="k")
    viewer = ResultViewer()
    viewer.simulation_duration.setValue(args.duration)
    if args.simulator is not None:
        viewer.simulator_path.setText(str(args.simulator))
        default_output = args.simulator.resolve().parent / "cctl_simulation.csv"
        viewer.simulation_output_path.setText(str(args.output or default_output))
    elif args.output is not None:
        viewer.simulation_output_path.setText(str(args.output))
    if args.files:
        viewer.open_files([Path(path) for path in args.files])
    if args.live:
        viewer.dynamic_refresh.setChecked(True)
    if args.rolling_window is not None:
        if args.rolling_window <= 0.0:
            parser.error("--rolling-window must be positive")
        viewer.rolling_window_seconds.setValue(args.rolling_window)
        viewer.rolling_x.setChecked(True)
    viewer.show()
    if args.autostart:
        QtCore.QTimer.singleShot(0, viewer.start_managed_simulation)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
