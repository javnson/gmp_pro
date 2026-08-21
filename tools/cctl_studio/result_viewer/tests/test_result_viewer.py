from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
VIEWER_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VIEWER_DIR))

from PyQt5 import QtCore, QtTest, QtWidgets  # noqa: E402
from result_viewer import LIVE_REFRESH_INTERVAL_MS, ResultViewer  # noqa: E402


class ResultViewerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def _wait_until(self, predicate, timeout_s: float = 3.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.app.processEvents()
            if predicate():
                return
            time.sleep(0.01)
        self.fail("timed out waiting for the result viewer background worker")

    def test_standalone_import_selects_the_same_pyqt5_runtime(self) -> None:
        command = (
            "import sys; "
            f"sys.path.insert(0, {str(VIEWER_DIR)!r}); "
            "import result_viewer; "
            "import pyqtgraph as pg; "
            "assert pg.Qt.QT_LIB == 'PyQt5', pg.Qt.QT_LIB; "
            "app = result_viewer.QtWidgets.QApplication.instance() or "
            "result_viewer.QtWidgets.QApplication([]); "
            "viewer = result_viewer.ResultViewer(); viewer.close()"
        )
        environment = os.environ.copy()
        environment["QT_QPA_PLATFORM"] = "offscreen"
        completed = subprocess.run(
            [sys.executable, "-c", command],
            capture_output=True,
            text=True,
            env=environment,
            timeout=10,
            check=False,
        )
        self.assertEqual(
            completed.returncode,
            0,
            completed.stdout + completed.stderr,
        )

    def test_dynamic_mode_refreshes_at_20_hz_and_defers_partial_row(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "running.csv"
            path.write_bytes(b"time_s,ia\n0,1\n1,2\n")
            viewer = ResultViewer()
            viewer.open_file(path)
            self.assertEqual(viewer.live_timer.interval(), LIVE_REFRESH_INTERVAL_MS)
            self.assertEqual(LIVE_REFRESH_INTERVAL_MS, 50)
            viewer.columns.item(0).setSelected(True)
            viewer.dynamic_refresh.setChecked(True)
            viewer.add_selected_curves()
            self._wait_until(
                lambda: viewer.live_reader is not None
                and len(viewer.cache.get("time_s", ())) == 2
            )

            with path.open("ab") as stream:
                stream.write(b"2,3")
            end = time.monotonic() + 0.15
            while time.monotonic() < end:
                self.app.processEvents()
                time.sleep(0.01)
            self.assertEqual(len(viewer.cache["time_s"]), 2)

            with path.open("ab") as stream:
                stream.write(b"\n")
            self._wait_until(lambda: len(viewer.cache["time_s"]) == 3)

            # This is the normal rerun workflow: the simulator truncates the
            # old result and starts a new CSV with the same header.
            path.write_bytes(b"time_s,ia\n10,11\n")
            self._wait_until(
                lambda: len(viewer.cache["time_s"]) == 1
                and viewer.cache["time_s"][0] == 10.0
            )
            with path.open("ab") as stream:
                stream.write(b"11,12\n")
            self._wait_until(lambda: len(viewer.cache["time_s"]) == 2)
            viewer.dynamic_refresh.setChecked(False)
            viewer.close()

    def test_axis_links_and_interaction_modes_are_independent(self) -> None:
        viewer = ResultViewer()
        viewer.add_plot()
        viewer.show()
        self.app.processEvents()
        QtTest.QTest.mouseClick(
            viewer.panels[0].plot.viewport(),
            QtCore.Qt.LeftButton,
            pos=viewer.panels[0].plot.viewport().rect().center(),
        )
        self.app.processEvents()
        self.assertIs(viewer.active_panel, viewer.panels[0])
        leader = viewer.panels[0].plot.getViewBox()
        follower = viewer.panels[1].plot.getViewBox()

        viewer.link_x.setChecked(True)
        viewer.link_y.setChecked(False)
        viewer.apply_links()
        self.assertIs(follower.linkedView(0), leader)
        self.assertIsNone(follower.linkedView(1))

        viewer.link_x.setChecked(False)
        viewer.link_y.setChecked(True)
        viewer.apply_links()
        self.assertIsNone(follower.linkedView(0))
        self.assertIs(follower.linkedView(1), leader)

        viewer.set_interaction_mode("x_zoom")
        self.assertEqual(follower.state["mouseEnabled"], [True, False])
        self.assertEqual(follower.state["mouseMode"], follower.RectMode)
        viewer.set_interaction_mode("y_zoom")
        self.assertEqual(follower.state["mouseEnabled"], [False, True])
        viewer.set_interaction_mode("box_zoom")
        self.assertEqual(follower.state["mouseEnabled"], [True, True])
        self.assertEqual(viewer.plot_splitter.count(), 2)
        self.assertIsNotNone(viewer.plot_splitter.handle(1))
        viewer.close()

    def test_title_edit_double_click_route_and_curve_add_remove(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "result.csv"
            path.write_bytes(b"time_s,ia,ib\n0,1,2\n1,3,4\n")
            viewer = ResultViewer()
            viewer.open_file(path)
            panel = viewer.active_panel
            viewer.show()
            self.app.processEvents()
            with mock.patch.object(
                QtWidgets.QInputDialog,
                "getText",
                return_value=("Phase currents", True),
            ):
                title_position = panel.plot.mapFromScene(
                    panel.title_label.sceneBoundingRect().center()
                )
                QtTest.QTest.mouseDClick(
                    panel.plot.viewport(),
                    QtCore.Qt.LeftButton,
                    pos=title_position,
                )
                self.app.processEvents()
            self.assertEqual(panel.title_text, "Phase currents")
            self.assertEqual(panel.title_label.text, "Phase currents")

            viewer.add_double_clicked_curve(viewer.columns.item(0))
            self._wait_until(lambda: "ia" in panel.curves)
            self.assertEqual(viewer.active_curves.count(), 1)
            viewer.active_curves.item(0).setSelected(True)
            viewer.remove_selected_curves()
            self.assertNotIn("ia", panel.curves)
            self.assertEqual(viewer.active_curves.count(), 0)
            viewer.close()


if __name__ == "__main__":
    unittest.main()
