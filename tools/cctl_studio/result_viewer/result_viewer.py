"""Interactive multi-plot viewer for large CCTL simulation CSV files."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

from result_data import ResultFile, inspect_result_file, load_numeric_columns, minmax_decimate


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


class PlotPanel(QtWidgets.QFrame):
    activated = QtCore.pyqtSignal(object)
    remove_requested = QtCore.pyqtSignal(object)

    def __init__(self, number: int):
        super().__init__()
        self.setObjectName("PlotPanel")
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.curves: dict[str, pg.PlotDataItem] = {}
        bar = QtWidgets.QHBoxLayout()
        self.title = QtWidgets.QLineEdit(f"Plot {number}")
        remove = QtWidgets.QToolButton(text="×")
        remove.setToolTip("Remove this plot")
        remove.clicked.connect(lambda: self.remove_requested.emit(self))
        bar.addWidget(self.title)
        bar.addWidget(remove)
        self.plot = pg.PlotWidget()
        self.plot.installEventFilter(self)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.addLegend()
        self.plot.setDownsampling(auto=True, mode="peak")
        self.plot.setClipToView(True)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(bar)
        layout.addWidget(self.plot)
        self.title.textChanged.connect(self.plot.setTitle)
        self.plot.setTitle(self.title.text())

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
        self.pool = QtCore.QThreadPool.globalInstance()
        self._build_ui()
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
        add_plot = QtWidgets.QPushButton("Add plot")
        add_plot.clicked.connect(self.add_plot)
        add_curves = QtWidgets.QPushButton("Add selected curves")
        add_curves.clicked.connect(self.add_selected_curves)
        remove_curves = QtWidgets.QPushButton("Remove selected curves")
        remove_curves.clicked.connect(self.remove_selected_curves)
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
        auto_range = QtWidgets.QPushButton("Fit all plots")
        auto_range.clicked.connect(lambda: [panel.plot.autoRange() for panel in self.panels])
        self.status_label = QtWidgets.QLabel()
        self.status_label.setWordWrap(True)
        form.addWidget(open_button)
        form.addWidget(self.file_label)
        form.addWidget(QtWidgets.QLabel("X axis"))
        form.addWidget(self.x_column)
        form.addWidget(QtWidgets.QLabel("Y columns (multi-select)"))
        form.addWidget(self.columns, 1)
        form.addWidget(add_curves)
        form.addWidget(remove_curves)
        form.addWidget(clear_curves)
        form.addWidget(add_plot)
        form.addWidget(self.link_x)
        form.addWidget(self.link_y)
        form.addWidget(QtWidgets.QLabel("Maximum display points / curve"))
        form.addWidget(self.maximum_points)
        form.addWidget(auto_range)
        form.addWidget(self.status_label)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        self.plot_host = QtWidgets.QWidget()
        self.plot_layout = QtWidgets.QVBoxLayout(self.plot_host)
        self.plot_layout.addStretch(1)
        scroll.setWidget(self.plot_host)
        root.addWidget(controls)
        root.addWidget(scroll)
        root.setSizes([300, 1150])
        self.setCentralWidget(root)

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
        self.status_label.setText(f"{len(self.result.columns)} columns; select curves to load")

    def add_plot(self) -> None:
        panel = PlotPanel(len(self.panels) + 1)
        panel.activated.connect(self.set_active_panel)
        panel.remove_requested.connect(self.remove_plot)
        self.plot_layout.insertWidget(self.plot_layout.count() - 1, panel)
        self.panels.append(panel)
        self.set_active_panel(panel)
        self.apply_links()

    def remove_plot(self, panel: PlotPanel) -> None:
        if len(self.panels) == 1:
            panel.plot.clear()
            panel.plot.addLegend()
            panel.curves.clear()
            return
        self.panels.remove(panel)
        panel.deleteLater()
        self.set_active_panel(self.panels[-1])
        self.apply_links()

    def set_active_panel(self, selected: PlotPanel) -> None:
        self.active_panel = selected
        for panel in self.panels:
            panel.set_active(panel is selected)

    def clear_active_plot(self) -> None:
        if self.active_panel is not None:
            self.active_panel.plot.clear()
            self.active_panel.plot.addLegend()
            self.active_panel.curves.clear()

    def remove_selected_curves(self) -> None:
        if self.active_panel is None:
            return
        for item in self.columns.selectedItems():
            curve = self.active_panel.curves.pop(item.text(), None)
            if curve is not None:
                self.active_panel.plot.removeItem(curve)

    def add_selected_curves(self) -> None:
        if self.result is None or self.active_panel is None:
            return
        if self.pending is not None:
            self.status_label.setText("A column load is already in progress…")
            return
        y_names = tuple(item.text() for item in self.columns.selectedItems())
        if not y_names:
            self.status_label.setText("Select one or more Y columns")
            return
        x_name = self.x_column.currentText()
        missing = tuple(name for name in (x_name, *y_names) if name not in self.cache)
        if missing:
            self.pending = (self.active_panel, y_names, x_name)
            self.status_label.setText("Loading selected columns in the background…")
            worker = ColumnWorker(self.result, missing)
            worker.signals.finished.connect(self._columns_loaded)
            worker.signals.failed.connect(self._load_failed)
            self.pool.start(worker)
        else:
            self._draw(self.active_panel, y_names, x_name)

    @QtCore.pyqtSlot(object)
    def _columns_loaded(self, columns: object) -> None:
        self.cache.update(columns)
        pending, self.pending = self.pending, None
        if pending is not None:
            self._draw(*pending)

    @QtCore.pyqtSlot(str)
    def _load_failed(self, message: str) -> None:
        self.pending = None
        self.status_label.setText("Load failed")
        QtWidgets.QMessageBox.critical(self, "Cannot load columns", message)

    def _draw(self, panel: PlotPanel, y_names: tuple[str, ...], x_name: str) -> None:
        x = self.cache[x_name]
        limit = self.maximum_points.value()
        for y_name in y_names:
            xd, yd = minmax_decimate(x, self.cache[y_name], limit)
            if y_name in panel.curves:
                panel.curves[y_name].setData(xd, yd)
            else:
                color = self.COLORS[len(panel.curves) % len(self.COLORS)]
                panel.curves[y_name] = panel.plot.plot(xd, yd, pen=pg.mkPen(color, width=1.2), name=y_name)
        panel.plot.setLabel("bottom", x_name)
        panel.plot.autoRange()
        self.status_label.setText(
            f"Loaded {len(x):,} rows; displaying at most {limit:,} points per curve"
        )

    def apply_links(self) -> None:
        if not self.panels:
            return
        leader = self.panels[0].plot
        for panel in self.panels:
            panel.plot.setXLink(leader if self.link_x.isChecked() and panel is not self.panels[0] else None)
            panel.plot.setYLink(leader if self.link_y.isChecked() and panel is not self.panels[0] else None)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="GMP CCTL large-result viewer")
    parser.add_argument("file", nargs="?")
    args = parser.parse_args(argv)
    app = QtWidgets.QApplication(sys.argv[:1])
    pg.setConfigOptions(antialias=False, background="w", foreground="k")
    viewer = ResultViewer()
    if args.file:
        viewer.open_file(Path(args.file))
    viewer.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
