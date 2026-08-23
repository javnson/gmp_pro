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
    load_numeric_time_window,
    minmax_decimate,
)
from simulation_manager import SimulationProcessManager
from pil_server_panel import PilServerPanel


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

    def __init__(
        self,
        requests: tuple[tuple[ResultFile, str, str], ...],
        time_names: dict[Path, str],
        start_time: float,
        duration_s: float,
        follow_latest: bool,
    ):
        super().__init__()
        self.requests = requests
        self.time_names = time_names
        self.start_time = start_time
        self.duration_s = duration_s
        self.follow_latest = follow_latest
        self.signals = WorkerSignals()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            loaded: dict[str, np.ndarray] = {}
            for result in dict.fromkeys(request[0] for request in self.requests):
                selected = [request for request in self.requests if request[0] == result]
                values = load_numeric_time_window(
                    result,
                    [item[1] for item in selected],
                    self.time_names[result.path],
                    start_time=self.start_time,
                    duration_s=self.duration_s,
                    follow_latest=self.follow_latest,
                )
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
    def __init__(
        self,
        requests: tuple[tuple[ResultFile, str, str], ...],
        time_names: dict[Path, str],
        duration_s: float,
    ):
        super().__init__()
        self.requests = requests
        self.time_names = time_names
        self.duration_s = duration_s
        self.signals = WorkerSignals()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            payload = []
            for result in dict.fromkeys(item[0] for item in self.requests):
                selected = [item for item in self.requests if item[0] == result]
                mapping = {raw: display for _result, raw, display in selected}
                reader = IncrementalResultReader(result, mapping)
                payload.append((
                    reader,
                    reader.initialize_time_window(
                        self.time_names[result.path], self.duration_s
                    ),
                    mapping,
                ))
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
    visible_x_changed = QtCore.pyqtSignal(object)

    def __init__(self, number: int):
        super().__init__()
        self.setObjectName("PlotPanel")
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setMinimumSize(120, 80)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding
        )
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
        self.plot.setBackground("white")
        self.plot.getViewBox().sigXRangeChanged.connect(
            lambda _view, _range: self.visible_x_changed.emit(self)
        )
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
            "QFrame#PlotPanel { background: white; border: 2px solid #3584e4; "
            "border-radius: 4px; }"
            if active else
            "QFrame#PlotPanel { background: white; border: 1px solid #c7cbd1; "
            "border-radius: 4px; }"
        )

    def set_auto_fit(self, enabled: bool) -> None:
        """Reserve Y-range ownership for the viewer's explicit fitter."""
        plot_item = self.plot.getPlotItem()
        plot_item.setAutoVisible(y=False)
        plot_item.enableAutoRange(axis="y", enable=False)

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


class PlotPage(QtWidgets.QWidget):
    """One configurable grid of plot panels."""

    def __init__(self, rows: int = 2, columns: int = 1):
        super().__init__()
        self.setObjectName("PlotPage")
        self.setStyleSheet("QWidget#PlotPage { background: #eef0f3; }")
        self.rows = rows
        self.columns = columns
        self.panels: list[PlotPanel] = []
        self.grid = QtWidgets.QGridLayout(self)
        self.grid.setContentsMargins(8, 8, 8, 8)
        self.grid.setHorizontalSpacing(8)
        self.grid.setVerticalSpacing(8)

    @property
    def capacity(self) -> int:
        return self.rows * self.columns

    def set_dimensions(self, rows: int, columns: int) -> None:
        if not 1 <= rows <= 6 or not 1 <= columns <= 4:
            raise ValueError("plot layout must be within 6 rows by 4 columns")
        self.rows = rows
        self.columns = columns
        self.relayout()

    def relayout(self) -> None:
        while self.grid.count():
            self.grid.takeAt(0)
        for index, panel in enumerate(self.panels):
            self.grid.addWidget(panel, index // self.columns, index % self.columns)
        for row in range(6):
            self.grid.setRowStretch(row, 1 if row < self.rows else 0)
        for column in range(4):
            self.grid.setColumnStretch(column, 1 if column < self.columns else 0)


class LayoutPicker(QtWidgets.QWidget):
    """Compact 4-by-6 interactive grid for choosing a tiled plot layout."""

    layout_selected = QtCore.pyqtSignal(int, int)

    def __init__(self):
        super().__init__()
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)
        self.caption = QtWidgets.QLabel("1 × 1")
        self.caption.setAlignment(QtCore.Qt.AlignCenter)
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(3)
        self.buttons: dict[tuple[int, int], QtWidgets.QPushButton] = {}
        for row in range(1, 7):
            for column in range(1, 5):
                button = QtWidgets.QPushButton()
                button.setFixedSize(25, 25)
                button.setProperty("layout_row", row)
                button.setProperty("layout_column", column)
                button.installEventFilter(self)
                button.clicked.connect(
                    lambda _checked=False, r=row, c=column:
                        self.layout_selected.emit(r, c)
                )
                grid.addWidget(button, row - 1, column - 1)
                self.buttons[(row, column)] = button
        outer.addLayout(grid)
        outer.addWidget(self.caption)
        self.set_current(1, 1)

    def eventFilter(self, watched: QtCore.QObject, event: QtCore.QEvent) -> bool:
        if event.type() == QtCore.QEvent.Enter and isinstance(
            watched, QtWidgets.QPushButton
        ):
            self._highlight(
                int(watched.property("layout_row")),
                int(watched.property("layout_column")),
            )
        return super().eventFilter(watched, event)

    def set_current(self, rows: int, columns: int) -> None:
        self._highlight(rows, columns)

    def _highlight(self, rows: int, columns: int) -> None:
        self.caption.setText(f"{rows} × {columns}")
        for (row, column), button in self.buttons.items():
            selected = row <= rows and column <= columns
            button.setStyleSheet(
                "QPushButton { background: #3584e4; border: 1px solid #1c71d8; "
                "border-radius: 2px; }"
                if selected else
                "QPushButton { background: white; border: 1px solid #9da3aa; "
                "border-radius: 2px; }"
            )


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
        self.pages: list[PlotPage] = []
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
        self.live_timer = QtCore.QTimer(self)
        self.live_timer.setInterval(LIVE_REFRESH_INTERVAL_MS)
        self.live_timer.timeout.connect(self.poll_live_file)
        self._build_ui()
        self._build_toolbar()
        self.simulation.message_received.connect(self._simulation_message)
        self.simulation.log_received.connect(self._simulation_log)
        self.simulation.state_changed.connect(self._simulation_state_changed)
        self.simulation.outputs_ready.connect(self._simulation_outputs_ready)
        self.simulation.process_finished.connect(self._simulation_finished)
        self.add_view_page()
        self.add_plot()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(4, 4, 4, 4)
        self.view_tabs = QtWidgets.QTabWidget()
        self.view_tabs.currentChanged.connect(self._active_tab_changed)
        settings = QtWidgets.QWidget()
        settings_layout = QtWidgets.QVBoxLayout(settings)

        source_group = QtWidgets.QGroupBox("Data and curves")
        source_group.setMinimumWidth(260)
        source_group.setMaximumWidth(420)
        self.data_curve_panel = source_group
        form = QtWidgets.QVBoxLayout(source_group)
        open_button = QtWidgets.QPushButton("Open CSV / TSV…")
        open_button.clicked.connect(self.open_dialog)
        self.file_label = QtWidgets.QLabel("No result file loaded")
        self.file_label.setWordWrap(True)
        self.x_column = QtWidgets.QComboBox()
        self.columns = QtWidgets.QTreeWidget()
        self.columns.setHeaderLabel("Result signals")
        self.columns.setRootIsDecorated(True)
        self.columns.setUniformRowHeights(True)
        self.columns.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.columns.itemDoubleClicked.connect(self.add_double_clicked_curve)
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
        self.auto_fit = QtWidgets.QCheckBox("Auto-fit visible data")
        self.auto_fit.setToolTip(
            "Continuously fit each Y axis to samples inside its visible X window"
        )
        self.auto_fit.toggled.connect(self.set_auto_fit)
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
        self.memory_window_seconds = QtWidgets.QDoubleSpinBox()
        self.memory_window_seconds.setDecimals(6)
        self.memory_window_seconds.setRange(0.001, 1_000_000.0)
        self.memory_window_seconds.setValue(1.0)
        self.memory_window_seconds.setSuffix(" s")
        self.memory_window_seconds.setToolTip(
            "Maximum time interval retained in memory for each result file"
        )
        self.follow_latest = QtWidgets.QCheckBox("Load newest time segment")
        self.follow_latest.setChecked(True)
        self.segment_start = QtWidgets.QDoubleSpinBox()
        self.segment_start.setDecimals(9)
        self.segment_start.setRange(-1.0e12, 1.0e12)
        self.segment_start.setSuffix(" s")
        self.segment_start.setEnabled(False)
        self.follow_latest.toggled.connect(self.set_follow_latest)
        self.status_label = QtWidgets.QLabel()
        self.status_label.setWordWrap(True)
        form.addWidget(open_button)
        form.addWidget(self.file_label)
        form.addWidget(QtWidgets.QLabel("X axis"))
        form.addWidget(self.x_column)
        form.addWidget(QtWidgets.QLabel("Y columns by result file (multi-select)"))
        form.addWidget(self.columns, 1)
        form.addWidget(add_curves)
        form.addWidget(self.active_curve_label)
        form.addWidget(self.active_curves)
        form.addWidget(remove_curves)
        form.addWidget(clear_curves)
        form.addWidget(self.status_label)

        configuration = QtWidgets.QWidget()
        config_layout = QtWidgets.QVBoxLayout(configuration)
        simulation_group = QtWidgets.QGroupBox("Simulation")
        layout = QtWidgets.QGridLayout(simulation_group)
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
        layout.addWidget(QtWidgets.QLabel("Simulator"), 0, 0)
        layout.addWidget(self.simulator_path, 0, 1, 1, 4)
        layout.addWidget(browse_simulator, 0, 5)
        layout.addWidget(QtWidgets.QLabel("Output base"), 1, 0)
        layout.addWidget(self.simulation_output_path, 1, 1, 1, 4)
        layout.addWidget(browse_output, 1, 5)
        layout.addWidget(QtWidgets.QLabel("Target"), 2, 0)
        layout.addWidget(self.simulation_duration, 2, 1)
        config_layout.addWidget(simulation_group)

        display_group = QtWidgets.QGroupBox("Display and memory")
        display_form = QtWidgets.QFormLayout(display_group)
        display_form.addRow(self.dynamic_refresh)
        display_form.addRow(self.rolling_x, self.rolling_window_seconds)
        display_form.addRow("Memory time window", self.memory_window_seconds)
        display_form.addRow(self.follow_latest)
        display_form.addRow("Fixed segment start", self.segment_start)
        display_form.addRow("Maximum points / curve", self.maximum_points)
        display_form.addRow(self.auto_fit)
        reload_segment = QtWidgets.QPushButton("Reload selected time segment")
        reload_segment.clicked.connect(self.reload_time_segment)
        display_form.addRow(reload_segment)
        display_form.addRow(self.link_x)
        display_form.addRow(self.link_y)
        config_layout.addWidget(display_group)

        config_layout.addStretch(1)

        settings_layout.addWidget(configuration, 1)
        self.settings_page = settings
        self.view_tabs.addTab(settings, "Configuration")
        self.pil_server_panel = PilServerPanel(self.simulation, self)
        self.view_tabs.addTab(self.pil_server_panel, "PIL Server")
        self.workspace_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.workspace_splitter.setChildrenCollapsible(False)
        self.workspace_splitter.addWidget(source_group)
        self.workspace_splitter.addWidget(self.view_tabs)
        self.workspace_splitter.setSizes([300, 1200])
        source_group.hide()
        outer.addWidget(self.workspace_splitter, 1)

        control_bar = QtWidgets.QWidget()
        control_layout = QtWidgets.QHBoxLayout(control_bar)
        control_layout.setContentsMargins(2, 2, 2, 2)
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
        self.simulation_progress.setFormat("Stop")
        self.simulation_metrics = QtWidgets.QLabel("state=stopped")
        control_layout.addWidget(self.simulation_start)
        control_layout.addWidget(self.simulation_pause)
        control_layout.addWidget(self.simulation_resume)
        control_layout.addWidget(self.simulation_stop)
        control_layout.addWidget(self.simulation_progress, 1)
        control_layout.addWidget(self.simulation_metrics)
        outer.addWidget(control_bar)
        self.setCentralWidget(central)

        self.dynamic_refresh.setChecked(True)
        self.rolling_x.setChecked(True)
        self.auto_fit.setChecked(True)
        self._simulation_state_changed("stopped")

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
        self._build_waveform_menus()

    def _build_waveform_menus(self) -> None:
        """Create page/plot commands and every supported grid layout."""
        self.waveform_menu = self.menuBar().addMenu("Waveforms")
        self.waveform_menu.addAction("New page", self.add_view_page)
        self.waveform_menu.addAction("Remove current page", self.remove_active_page)
        self.waveform_menu.addSeparator()
        self.waveform_menu.addAction("Add plot", self.add_plot)
        self.waveform_menu.addAction("Remove active plot", self.remove_active_plot)

        self.layout_menu = self.menuBar().addMenu("Layout")
        self.layout_picker = LayoutPicker()
        self.layout_picker.layout_selected.connect(self._layout_picked)
        picker_action = QtWidgets.QWidgetAction(self.layout_menu)
        picker_action.setDefaultWidget(self.layout_picker)
        self.layout_menu.addAction(picker_action)
        self.layout_menu.aboutToShow.connect(self._prepare_layout_picker)

    def _prepare_layout_picker(self) -> None:
        page = self.current_plot_page()
        if page is not None:
            self.layout_picker.set_current(page.rows, page.columns)

    @QtCore.pyqtSlot(int, int)
    def _layout_picked(self, rows: int, columns: int) -> None:
        self.apply_page_layout(rows, columns)
        self.layout_menu.close()

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
        # Auto-start here means "start after the supervised child reports
        # ready".  This method itself is reached only from the user's Start
        # command, so one click performs the complete transition from Stop.
        self._launch_managed_simulation(auto_start=True)

    def prepare_managed_simulation(self, auto_start: bool = False) -> None:
        """Initialize the simulator and result headers without stepping it."""
        if self.simulation.is_active():
            return
        self._launch_managed_simulation(auto_start=auto_start)

    def _launch_managed_simulation(self, auto_start: bool) -> None:
        if self.simulation.is_active():
            if self.simulation.state == "ready":
                self.simulation.start_simulation()
            elif self.simulation.state == "paused":
                self.simulation.resume()
            return
        try:
            self.simulation.launch(
                Path(self.simulator_path.text()),
                Path(self.simulation_output_path.text()),
                self.simulation_duration.value(),
                auto_start=auto_start,
            )
        except Exception as error:
            QtWidgets.QMessageBox.critical(
                self, "Cannot start simulation", str(error)
            )

    def update_simulation_duration(self) -> None:
        if self.simulation.is_active():
            self.simulation.set_duration(self.simulation_duration.value())

    @QtCore.pyqtSlot(bool)
    def set_follow_latest(self, enabled: bool) -> None:
        self.segment_start.setEnabled(not enabled)
        if not enabled and self.dynamic_refresh.isChecked():
            self.dynamic_refresh.setChecked(False)
        # A cached time column and a newly loaded signal must always describe
        # the same interval.  Invalidate the previous selection when changing
        # between trailing and fixed-window modes.
        self.cache.clear()

    def reload_time_segment(self) -> None:
        """Reload every plotted signal using the configured time selection."""
        names = tuple(dict.fromkeys(
            name for panel in self.panels for name in panel.curves
        ))
        if not names:
            self.status_label.setText("Add at least one curve before reloading")
            return
        required = []
        for name in names:
            result, _raw = self.bindings[name]
            required.extend((self.time_keys[result.path], name))
        self.cache.clear()
        if self.dynamic_refresh.isChecked():
            self._start_live_initialization(tuple(dict.fromkeys(required)))
            return
        self.file_generation += 1
        generation = self.file_generation
        requests = tuple(
            (self.bindings[name][0], self.bindings[name][1], name)
            for name in dict.fromkeys(required)
        )
        worker = MultiColumnWorker(
            requests,
            {
                result.path: self.bindings[self.time_keys[result.path]][1]
                for result in self.results
            },
            self.segment_start.value(),
            self.memory_window_seconds.value(),
            self.follow_latest.isChecked(),
        )
        worker.signals.finished.connect(
            lambda columns, token=generation: self._window_reloaded(token, columns)
        )
        worker.signals.failed.connect(
            lambda message, token=generation: self._load_failed(token, message)
        )
        self.pool.start(worker)

    def _window_reloaded(self, generation: int, columns: object) -> None:
        if generation != self.file_generation:
            return
        self.cache.update(columns)
        self._refresh_all_plots()
        rows = max((len(values) for values in self.cache.values()), default=0)
        self.status_label.setText(f"Time segment loaded: {rows:,} rows retained")

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
        if message.get("state") == "ready":
            self.simulation_progress.setRange(0, 1000)
            self.simulation_progress.setValue(0)
            self.simulation_progress.setFormat("Stop")
            self.simulation_metrics.setText(
                f"state=stopped  elapsed={elapsed:.1f}s  "
                f"queue={queued}/{capacity}  dropped={dropped}"
            )
            return
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
            sys.stderr.write(text + ("" if text.endswith("\n") else "\n"))
            sys.stderr.flush()

    @QtCore.pyqtSlot(str)
    def _simulation_state_changed(self, state: str) -> None:
        active = self.simulation.is_active()
        self.simulation_start.setEnabled(not active or state in {"ready", "paused"})
        self.simulation_pause.setEnabled(active and state == "running")
        self.simulation_resume.setEnabled(active and state == "paused")
        self.simulation_stop.setEnabled(active and state not in {"stopping", "completed"})
        if state in {"idle", "stopped", "starting", "ready", "paused", "stopping"}:
            self.simulation_progress.setFormat(
                "Stop" if state in {"idle", "stopped", "ready"}
                else state.capitalize()
            )
        visible_state = "stopped" if state == "ready" else state
        self.simulation_metrics.setText(f"state={visible_state}")

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
        self.pil_server_panel.shutdown()
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
        for result in self.results:
            root = QtWidgets.QTreeWidgetItem([result.path.name])
            root.setToolTip(0, str(result.path))
            root.setFlags(root.flags() & ~QtCore.Qt.ItemIsSelectable)
            self.columns.addTopLevelItem(root)
            prefix = f"{result.path.name} :: " if multiple else ""
            time_key = self.time_keys[result.path]
            for raw_name in result.columns:
                binding_key = prefix + raw_name
                if binding_key == time_key:
                    continue
                child = QtWidgets.QTreeWidgetItem([raw_name])
                child.setData(0, QtCore.Qt.UserRole, binding_key)
                child.setToolTip(0, f"{result.path}\n{raw_name}")
                root.addChild(child)
            root.setExpanded(True)
        self.columns.resizeColumnToContents(0)
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

    def current_plot_page(self) -> PlotPage | None:
        widget = self.view_tabs.currentWidget()
        if isinstance(widget, PlotPage):
            return widget
        if self.active_panel is not None:
            selected = next(
                (page for page in self.pages if self.active_panel in page.panels),
                None,
            )
            if selected is not None:
                return selected
        return self.pages[-1] if self.pages else None

    def add_view_page(self) -> None:
        page = PlotPage(1, 1)
        self.pages.append(page)
        index = self.view_tabs.addTab(page, f"Waveforms {len(self.pages)}")
        self.view_tabs.setCurrentIndex(index)

    def remove_active_page(self) -> None:
        page = self.current_plot_page()
        if page is None:
            return
        if len(self.pages) == 1:
            self.status_label.setText("At least one waveform page is required")
            return
        for panel in tuple(page.panels):
            self.remove_plot(panel, allow_empty=True)
        self.pages.remove(page)
        self.view_tabs.removeTab(self.view_tabs.indexOf(page))
        page.deleteLater()
        target = self.pages[-1]
        self.view_tabs.setCurrentWidget(target)
        if target.panels:
            self.set_active_panel(target.panels[0])

    def apply_page_layout(
        self, rows: int | None = None, columns: int | None = None
    ) -> None:
        page = self.current_plot_page()
        if page is None:
            self.status_label.setText("Select a waveform page first")
            return
        selected_rows = rows or page.rows
        selected_columns = columns or page.columns
        target_count = selected_rows * selected_columns
        removed_curves = sum(
            len(panel.curves) for panel in page.panels[target_count:]
        )
        for panel in tuple(page.panels[target_count:]):
            self.remove_plot(panel, allow_empty=True)
        page.set_dimensions(selected_rows, selected_columns)
        while len(page.panels) < target_count:
            self._append_plot(page, make_active=False, relayout=False)
        page.relayout()
        if page.panels:
            self.set_active_panel(page.panels[0])
        self._sync_layout_action(page)
        self.apply_links()
        self.status_label.setText(
            f"Current page layout: {page.rows} × {page.columns}; "
            f"{target_count} equally sized plots"
            + (f" ({removed_curves} removed curves)" if removed_curves else "")
        )

    def _active_tab_changed(self, _index: int) -> None:
        page = self.view_tabs.currentWidget()
        is_waveform = isinstance(page, PlotPage)
        self.data_curve_panel.setVisible(is_waveform)
        if hasattr(self, "layout_menu"):
            self.layout_menu.setEnabled(is_waveform)
        if not isinstance(page, PlotPage):
            return
        if page.panels:
            self.set_active_panel(page.panels[0])
        self._sync_layout_action(page)
        self.apply_links()

    def _sync_layout_action(self, page: PlotPage) -> None:
        if hasattr(self, "layout_picker"):
            self.layout_picker.set_current(page.rows, page.columns)

    def add_plot(self) -> None:
        page = self.current_plot_page()
        if page is None:
            self.add_view_page()
            page = self.current_plot_page()
        assert page is not None
        if len(page.panels) >= page.capacity:
            self.status_label.setText(
                "The active page layout is full; enlarge it or create a page"
            )
            return
        panel = self._append_plot(page, make_active=True, relayout=True)
        self.apply_links()

    def _append_plot(
        self, page: PlotPage, *, make_active: bool, relayout: bool
    ) -> PlotPanel:
        """Append one consistently configured tiled plot to a page."""
        panel = PlotPanel(self.next_plot_number)
        self.next_plot_number += 1
        panel.activated.connect(self.set_active_panel)
        panel.title_changed.connect(self._sync_active_curve_list)
        panel.visible_x_changed.connect(self._visible_x_range_changed)
        panel.set_interaction_mode(self.interaction_mode)
        panel.set_auto_fit(self.auto_fit.isChecked())
        page.panels.append(panel)
        self.panels.append(panel)
        if relayout:
            page.relayout()
        if make_active:
            self.set_active_panel(panel)
        return panel

    def remove_active_plot(self) -> None:
        if self.active_panel is not None:
            self.remove_plot(self.active_panel)

    def remove_plot(self, panel: PlotPanel, allow_empty: bool = False) -> None:
        if len(self.panels) == 1 and not allow_empty:
            panel.plot.clear()
            panel.plot.addLegend()
            panel.curves.clear()
            panel.x_name = None
            self._sync_active_curve_list()
            return
        page = next((item for item in self.pages if panel in item.panels), None)
        if page is not None:
            page.panels.remove(panel)
        self.panels.remove(panel)
        panel.setParent(None)
        panel.deleteLater()
        if page is not None:
            page.relayout()
        if self.panels:
            self.set_active_panel(self.panels[-1])
        else:
            self.active_panel = None
            self._sync_active_curve_list()
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

    def add_double_clicked_curve(
        self, item: QtWidgets.QTreeWidgetItem, _column: int = 0
    ) -> None:
        name = item.data(0, QtCore.Qt.UserRole)
        if isinstance(name, str):
            self._request_curves((name,))

    def add_selected_curves(self) -> None:
        y_names = tuple(
            name for item in self.columns.selectedItems()
            if isinstance((name := item.data(0, QtCore.Qt.UserRole)), str)
        )
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
                worker = MultiColumnWorker(
                    requests,
                    {
                        result.path: self.bindings[self.time_keys[result.path]][1]
                        for result in self.results
                    },
                    self.segment_start.value(),
                    self.memory_window_seconds.value(),
                    self.follow_latest.isChecked(),
                )
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
        panel.set_auto_fit(self.auto_fit.isChecked())
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
            self.active_panel.set_auto_fit(self.auto_fit.isChecked())
            self.apply_rolling_window((self.active_panel,))

    @QtCore.pyqtSlot(bool)
    def set_auto_fit(self, enabled: bool) -> None:
        """Apply oscilloscope-style Y fitting to every tiled plot."""
        for panel in self.panels:
            panel.set_auto_fit(enabled)
        if enabled:
            self.apply_rolling_window()

    @QtCore.pyqtSlot(object)
    def _visible_x_range_changed(self, panel: PlotPanel) -> None:
        if self.auto_fit.isChecked():
            self._auto_fit_panels((panel,))

    @QtCore.pyqtSlot(bool)
    def set_rolling_window(self, enabled: bool) -> None:
        if enabled:
            self.apply_rolling_window()
        else:
            for panel in self.panels:
                panel.plot.autoRange()
                panel.set_auto_fit(self.auto_fit.isChecked())
            self._auto_fit_panels(tuple(self.panels))

    def apply_rolling_window(
        self, panels: tuple[PlotPanel, ...] | None = None
    ) -> None:
        """Pin X to a trailing time interval, like a streaming oscilloscope."""
        selected = panels or tuple(self.panels)
        if self.rolling_x.isChecked():
            span = self.rolling_window_seconds.value()
            for panel in selected:
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
                        latest = (
                            candidate if latest is None else max(latest, candidate)
                        )
                if latest is not None:
                    panel.plot.setXRange(latest - span, latest, padding=0.0)
        self._auto_fit_panels(selected)

    def _panel_visible_y_bounds(
        self, panel: PlotPanel
    ) -> tuple[float, float] | None:
        left, right = panel.plot.viewRange()[0]
        minimum: float | None = None
        maximum: float | None = None
        for y_name in panel.curves:
            binding = self.bindings.get(y_name)
            y = self.cache.get(y_name)
            if binding is None or y is None:
                continue
            x = self.cache.get(self.time_keys[binding[0].path])
            if x is None:
                continue
            common = min(len(x), len(y))
            if common == 0:
                continue
            xv = x[:common]
            yv = y[:common]
            if np.all(xv[:-1] <= xv[1:]):
                begin = int(np.searchsorted(xv, left, side="left"))
                end = int(np.searchsorted(xv, right, side="right"))
                visible = yv[begin:end]
            else:
                visible = yv[(xv >= left) & (xv <= right)]
            finite = visible[np.isfinite(visible)]
            if not finite.size:
                continue
            low = float(np.min(finite))
            high = float(np.max(finite))
            minimum = low if minimum is None else min(minimum, low)
            maximum = high if maximum is None else max(maximum, high)
        if minimum is None or maximum is None:
            return None
        return minimum, maximum

    @staticmethod
    def _padded_y_bounds(bounds: tuple[float, float]) -> tuple[float, float]:
        low, high = bounds
        span = high - low
        padding = span * 0.05
        if padding <= np.finfo(np.float64).eps:
            padding = max(abs(low), abs(high), 1.0) * 0.05
        return low - padding, high + padding

    def _auto_fit_panels(self, panels: tuple[PlotPanel, ...]) -> None:
        if not self.auto_fit.isChecked():
            return
        if self.link_y.isChecked():
            affected_pages = [
                page for page in self.pages
                if any(panel in page.panels for panel in panels)
            ]
            for page in affected_pages:
                bounds = [
                    value for panel in page.panels
                    if (value := self._panel_visible_y_bounds(panel)) is not None
                ]
                if not bounds or not page.panels:
                    continue
                padded = self._padded_y_bounds((
                    min(value[0] for value in bounds),
                    max(value[1] for value in bounds),
                ))
                page.panels[0].plot.setYRange(*padded, padding=0.0)
            return
        for panel in panels:
            bounds = self._panel_visible_y_bounds(panel)
            if bounds is not None:
                panel.plot.setYRange(
                    *self._padded_y_bounds(bounds), padding=0.0
                )

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
        worker = MultiLiveInitializeWorker(
            requests,
            {
                result.path: self.bindings[self.time_keys[result.path]][1]
                for result in self.results
            },
            self.memory_window_seconds.value(),
        )
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
            self._trim_cache_to_memory_window()
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

    def _trim_cache_to_memory_window(self) -> None:
        """Bound resident samples independently for every input file."""
        duration = self.memory_window_seconds.value()
        for result in self.results:
            time_key = self.time_keys.get(result.path)
            time_values = self.cache.get(time_key or "")
            if time_values is None or not time_values.size:
                continue
            cutoff = float(time_values[-1]) - duration
            begin = int(np.searchsorted(time_values, cutoff, side="left"))
            if begin <= 0:
                continue
            for display_name, (source, _raw) in self.bindings.items():
                if source.path == result.path and display_name in self.cache:
                    self.cache[display_name] = self.cache[display_name][begin:]

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
        for page in self.pages:
            views = [panel.plot.getViewBox() for panel in page.panels]
            # Links are page-local; X and Y remain independently selectable.
            for view in views:
                view.setXLink(None)
                view.setYLink(None)
            if not views:
                continue
            leader = views[0]
            for view in views[1:]:
                if self.link_x.isChecked():
                    view.setXLink(leader)
                if self.link_y.isChecked():
                    view.setYLink(leader)
        if hasattr(self, "auto_fit") and self.auto_fit.isChecked():
            self._auto_fit_panels(tuple(self.panels))


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
        help=(
            "prepare a CCTL simulator and load its CSV headers without "
            "starting numerical steps"
        ),
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
    if args.simulator is not None:
        QtCore.QTimer.singleShot(
            0,
            lambda start=args.autostart:
                viewer.prepare_managed_simulation(auto_start=start),
        )
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
