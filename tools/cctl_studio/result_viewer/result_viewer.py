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
        self.setWindowTitle("GMP CCTL Result Viewer")
        self.resize(1450, 900)
        self.result: ResultFile | None = None
        self.cache: dict[str, np.ndarray] = {}
        self.panels: list[PlotPanel] = []
        self.active_panel: PlotPanel | None = None
        self.pending: tuple[PlotPanel, tuple[str, ...], str] | None = None
        self.live_reader: IncrementalResultReader | None = None
        self.live_initializing = False
        self.live_poll_pending = False
        self.live_skipped_rows = 0
        self.live_generation = 0
        self.file_generation = 0
        self.interaction_mode = "pan"
        self.next_plot_number = 1
        self.pool = QtCore.QThreadPool.globalInstance()
        self._build_ui()
        self._build_toolbar()
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

    def open_dialog(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open simulation result", "", "Delimited data (*.csv *.tsv *.txt);;All files (*)"
        )
        if path:
            self.open_file(Path(path))

    def open_file(self, path: Path) -> None:
        try:
            self.result = inspect_result_file(path)
        except Exception as error:
            QtWidgets.QMessageBox.critical(self, "Cannot open result", str(error))
            return
        self.live_timer.stop()
        self.live_generation += 1
        self.file_generation += 1
        self.live_reader = None
        self.live_initializing = False
        self.live_poll_pending = False
        self.live_skipped_rows = 0
        self.pending = None
        self.cache.clear()
        self.file_label.setText(str(self.result.path))
        self.x_column.clear()
        self.x_column.addItems(self.result.columns)
        preferred = next(
            (index for index, name in enumerate(self.result.columns) if name.lower() in {"time", "time_s", "t"}),
            0,
        )
        self.x_column.setCurrentIndex(preferred)
        self.columns.clear()
        for name in self.result.columns:
            if name != self.x_column.currentText():
                self.columns.addItem(name)
        for panel in self.panels:
            panel.plot.clear()
            panel.plot.addLegend()
            panel.curves.clear()
            panel.x_name = None
        self._sync_active_curve_list()
        self.status_label.setText(f"{len(self.result.columns)} columns; select curves to load")

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
        y_names = tuple(name for name in dict.fromkeys(y_names) if name != x_name)
        if not y_names:
            self.status_label.setText("The X column cannot also be a Y curve")
            return
        missing = tuple(name for name in (x_name, *y_names) if name not in self.cache)
        if missing:
            self.pending = (self.active_panel, y_names, x_name)
            if self.dynamic_refresh.isChecked():
                names = tuple(dict.fromkeys((*self.cache, x_name, *y_names)))
                self._start_live_initialization(names)
            else:
                self.status_label.setText("Loading selected columns in the background…")
                generation = self.file_generation
                worker = ColumnWorker(self.result, missing)
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
        x = self.cache[x_name]
        panel.x_name = x_name
        limit = self.maximum_points.value()
        displayed_rows = len(x)
        for y_name in y_names:
            y = self.cache[y_name]
            common_rows = min(len(x), len(y))
            displayed_rows = min(displayed_rows, common_rows)
            xd, yd = minmax_decimate(x[:common_rows], y[:common_rows], limit)
            if y_name in panel.curves:
                panel.curves[y_name].setData(xd, yd)
            else:
                color = self.COLORS[len(panel.curves) % len(self.COLORS)]
                panel.curves[y_name] = panel.plot.plot(xd, yd, pen=pg.mkPen(color, width=1.2), name=y_name)
        if panel is self.active_panel:
            self._sync_active_curve_list()
        panel.plot.setLabel("bottom", x_name)
        panel.plot.autoRange()
        self.status_label.setText(
            f"Loaded {displayed_rows:,} common rows; displaying at most "
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
    def set_dynamic_refresh(self, enabled: bool) -> None:
        was_live_initializing = self.live_initializing
        self.live_timer.stop()
        self.live_generation += 1
        self.live_reader = None
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
            self.status_label.setText(
                "Dynamic refresh is enabled; select curves to begin reading"
            )
            return
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
        worker = LiveInitializeWorker(self.result, names)
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
        reader, chunk = payload
        if (
            not self.dynamic_refresh.isChecked()
            or self.result is None
            or reader.result.path != self.result.path
        ):
            return
        self.live_reader = reader
        self.live_skipped_rows += int(chunk.skipped_trailing_row)
        for name, values in chunk.columns.items():
            self.cache[name] = values
        pending, self.pending = self.pending, None
        if pending is not None:
            self._draw(*pending)
        self._refresh_all_plots()
        self.live_timer.start(LIVE_REFRESH_INTERVAL_MS)
        self._set_live_status(chunk)

    @QtCore.pyqtSlot()
    def poll_live_file(self) -> None:
        if self.live_reader is None or self.live_poll_pending:
            return
        self.live_poll_pending = True
        generation = self.live_generation
        worker = LiveTailWorker(self.live_reader)
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
        reader, chunk = payload
        if reader is not self.live_reader:
            return
        self.live_poll_pending = False
        if chunk.reset:
            self.live_skipped_rows = 0
        self.live_skipped_rows += int(chunk.skipped_trailing_row)
        for name, values in chunk.columns.items():
            if chunk.reset:
                self.cache[name] = values
            elif values.size:
                existing = self.cache.get(name)
                self.cache[name] = (
                    values if existing is None or not existing.size
                    else np.concatenate((existing, values))
                )
        if any(values.size for values in chunk.columns.values()) or chunk.reset:
            self._refresh_all_plots()
        self._set_live_status(chunk)

    def _live_failed(self, generation: int, message: str) -> None:
        if generation != self.live_generation:
            return
        self.live_poll_pending = False
        self.live_initializing = False
        self.live_timer.stop()
        self.live_reader = None
        self.pending = None
        self.status_label.setText(f"Dynamic refresh stopped: {message}")

    def _refresh_all_plots(self) -> None:
        limit = self.maximum_points.value()
        for panel in self.panels:
            if panel.x_name is None or panel.x_name not in self.cache:
                continue
            x = self.cache[panel.x_name]
            for y_name, curve in panel.curves.items():
                if y_name not in self.cache:
                    continue
                y = self.cache[y_name]
                common_rows = min(len(x), len(y))
                xd, yd = minmax_decimate(
                    x[:common_rows], y[:common_rows], limit
                )
                curve.setData(xd, yd)

    def _set_live_status(self, chunk: ResultChunk) -> None:
        rows = max((len(values) for values in self.cache.values()), default=0)
        suffix = ""
        if chunk.incomplete_tail:
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
    parser.add_argument("file", nargs="?")
    args = parser.parse_args(argv)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    pg.setConfigOptions(antialias=False, background="w", foreground="k")
    viewer = ResultViewer()
    if args.file:
        viewer.open_file(Path(args.file))
    viewer.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
