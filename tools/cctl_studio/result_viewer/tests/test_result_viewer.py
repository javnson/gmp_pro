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
from pil_server_panel import HermesDatalinkQt  # noqa: E402
from simulation_manager import SimulationProcessManager  # noqa: E402


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

    @staticmethod
    def _signal_items(viewer: ResultViewer) -> list[QtWidgets.QTreeWidgetItem]:
        items = []
        for root_index in range(viewer.columns.topLevelItemCount()):
            root = viewer.columns.topLevelItem(root_index)
            items.extend(root.child(index) for index in range(root.childCount()))
        return items

    @classmethod
    def _signal_item(
        cls, viewer: ResultViewer, signal_name: str, file_name: str | None = None
    ) -> QtWidgets.QTreeWidgetItem:
        for root_index in range(viewer.columns.topLevelItemCount()):
            root = viewer.columns.topLevelItem(root_index)
            if file_name is not None and root.text(0) != file_name:
                continue
            for child_index in range(root.childCount()):
                child = root.child(child_index)
                if child.text(0) == signal_name:
                    return child
        raise AssertionError(f"signal not found: {file_name or '*'} / {signal_name}")

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

    def test_supervisor_commands_are_compact_json(self) -> None:
        payload = SimulationProcessManager.encode_command(
            {"command": "set_duration", "seconds": 2.5}
        )
        self.assertEqual(
            payload,
            b'{"command":"set_duration","seconds":2.5}\n',
        )

    def test_supervisor_ready_message_publishes_outputs(self) -> None:
        manager = SimulationProcessManager()
        states: list[str] = []
        outputs: list[list[str]] = []
        manager.state_changed.connect(states.append)
        manager.outputs_ready.connect(outputs.append)
        manager._dispatch_message({
            "type": "ready",
            "protocol": 1,
            "state": "ready",
            "outputs": [{"name": "circuit", "path": "drive_circuit.csv"}],
        })
        self.assertEqual(states, ["ready"])
        self.assertEqual(outputs, [["drive_circuit.csv"]])

    def test_supervisor_datalink_message_decodes_raw_bytes(self) -> None:
        manager = SimulationProcessManager()
        received: list[bytes] = []
        manager.datalink_received.connect(received.append)
        manager._dispatch_message({
            "type": "datalink",
            "encoding": "base64",
            "data": "eyU9fQ==",
        })
        self.assertEqual(received, [b"{%=}"])

    def test_debugger_managed_transport_preserves_standard_frames(self) -> None:
        writes: list[bytes] = []
        frames: list[tuple[int, int, bytes]] = []
        hermes = HermesDatalinkQt()
        hermes.sig_frame_received.connect(
            lambda target, command, payload: frames.append(
                (target, command, payload)
            )
        )
        self.assertTrue(hermes.connect_transport(writes.append, "test transport"))
        try:
            hermes.send_frame(7, 0x10, b"pil")
            self._wait_until(lambda: bool(writes))
            self.assertTrue(hermes.feed_transport(writes[0]))
            self._wait_until(lambda: bool(frames))
            self.assertEqual(frames, [(7, 0x10, b"pil")])
        finally:
            hermes.close()

    def test_dynamic_mode_refreshes_at_20_hz_and_defers_partial_row(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "running.csv"
            path.write_bytes(b"time_s,ia\n0,1\n1,2\n")
            viewer = ResultViewer()
            viewer.open_file(path)
            self.assertEqual(viewer.live_timer.interval(), LIVE_REFRESH_INTERVAL_MS)
            self.assertEqual(LIVE_REFRESH_INTERVAL_MS, 50)
            self._signal_item(viewer, "ia").setSelected(True)
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
            self._wait_until(
                lambda: len(viewer.cache["time_s"]) == 2
                and viewer.cache["time_s"][0] == 1.0
                and viewer.cache["time_s"][-1] == 2.0
            )

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
        viewer.apply_page_layout(1, 2)
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
        self.assertEqual(len(viewer.pages), 1)
        self.assertEqual(len(viewer.pages[0].panels), 2)
        viewer.close()

    def test_waveform_pages_enforce_configured_grid_capacity(self) -> None:
        viewer = ResultViewer()
        self.assertTrue(viewer.dynamic_refresh.isChecked())
        self.assertTrue(viewer.rolling_x.isChecked())
        self.assertAlmostEqual(viewer.rolling_window_seconds.value(), 0.1)
        self.assertAlmostEqual(viewer.memory_window_seconds.value(), 1.0)
        self.assertEqual(viewer.view_tabs.tabText(0), "Configuration")
        self.assertEqual(viewer.view_tabs.tabText(1), "PIL Server")
        self.assertTrue(viewer.data_curve_panel.isVisibleTo(viewer))
        self.assertEqual((viewer.pages[0].rows, viewer.pages[0].columns), (1, 1))
        first_panel = viewer.pages[0].panels[0]
        viewer.layout_picker.buttons[(1, 2)].click()
        self.assertEqual(len(viewer.pages[0].panels), 2)
        self.assertIs(viewer.pages[0].panels[0], first_panel)
        self.assertEqual(viewer.layout_picker.caption.text(), "1 × 2")
        viewer.show()
        self.app.processEvents()
        widths = [panel.width() for panel in viewer.pages[0].panels]
        heights = [panel.height() for panel in viewer.pages[0].panels]
        self.assertLessEqual(max(widths) - min(widths), 1)
        self.assertLessEqual(max(heights) - min(heights), 1)
        viewer.add_view_page()
        viewer.apply_page_layout(6, 4)
        self.assertEqual(viewer.pages[-1].capacity, 24)
        self.assertEqual(len(viewer.pages[-1].panels), 24)
        self.assertEqual(len(viewer.layout_picker.buttons), 24)
        viewer.apply_page_layout(1, 1)
        self.assertEqual(len(viewer.pages[-1].panels), 1)
        viewer.view_tabs.setCurrentWidget(viewer.settings_page)
        self.assertFalse(viewer.data_curve_panel.isVisible())
        viewer.close()

    def test_auto_fit_controls_visible_y_axis_ranging(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "autofit.csv"
            path.write_bytes(
                b"time_s,signal\n0,0\n0.5,100\n0.9,9\n1.0,10\n"
            )
            viewer = ResultViewer()
            viewer.open_file(path)
            self._signal_item(viewer, "signal").setSelected(True)
            viewer.add_selected_curves()
            self._wait_until(lambda: "signal" in viewer.active_panel.curves)
            low, high = viewer.active_panel.plot.viewRange()[1]
            self.assertLess(low, 9.0)
            self.assertGreater(high, 10.0)
            self.assertLess(high, 20.0)

            viewer.rolling_window_seconds.setValue(0.6)
            low, high = viewer.active_panel.plot.viewRange()[1]
            self.assertLess(low, 9.0)
            self.assertGreater(high, 100.0)

            viewer.auto_fit.setChecked(False)
            viewer.active_panel.plot.setYRange(-2.0, 2.0, padding=0.0)
            viewer._refresh_all_plots()
            low, high = viewer.active_panel.plot.viewRange()[1]
            self.assertAlmostEqual(low, -2.0, places=5)
            self.assertAlmostEqual(high, 2.0, places=5)
            viewer.close()

    def test_managed_simulation_waits_for_one_user_start_command(self) -> None:
        viewer = ResultViewer()
        self.assertEqual(viewer.simulation.state, "stopped")
        self.assertEqual(viewer.simulation_progress.format(), "Stop")
        with mock.patch.object(viewer.simulation, "launch") as launch:
            viewer.simulator_path.setText("simulator.exe")
            viewer.start_managed_simulation()
            self.assertTrue(launch.call_args.kwargs["auto_start"])
        viewer.close()

    def test_managed_preparation_loads_headers_before_user_start(self) -> None:
        viewer = ResultViewer()
        with mock.patch.object(viewer.simulation, "launch") as launch:
            viewer.simulator_path.setText("simulator.exe")
            viewer.prepare_managed_simulation()
            self.assertFalse(launch.call_args.kwargs["auto_start"])

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "header_only.csv"
            path.write_bytes(b"time_s,ia,ib,torque_nm\n")
            viewer._simulation_outputs_ready([str(path)])
            self.assertEqual(viewer.columns.topLevelItemCount(), 1)
            self.assertEqual(
                viewer.columns.topLevelItem(0).text(0), "header_only.csv"
            )
            names = [item.text(0) for item in self._signal_items(viewer)]
            self.assertEqual(names, ["ia", "ib", "torque_nm"])
        viewer._simulation_state_changed("ready")
        self.assertEqual(viewer.simulation_progress.format(), "Stop")
        self.assertEqual(viewer.simulation_metrics.text(), "state=stopped")
        viewer._simulation_message({
            "type": "status",
            "state": "ready",
            "completed_steps": 0,
            "target_steps": 100,
            "target_time_s": 1.0,
            "elapsed_s": 0.1,
            "queued": 0,
            "capacity": 16,
            "dropped": 0,
        })
        self.assertEqual(viewer.simulation_progress.format(), "Stop")
        self.assertIn("state=stopped", viewer.simulation_metrics.text())
        viewer.close()

    def test_fixed_time_segment_loads_only_requested_rows(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "segment.csv"
            path.write_bytes(
                b"time_s,signal\n0,10\n0.5,20\n1.0,30\n1.5,40\n2.0,50\n"
            )
            viewer = ResultViewer()
            viewer.open_file(path)
            viewer.follow_latest.setChecked(False)
            viewer.segment_start.setValue(0.5)
            viewer.memory_window_seconds.setValue(1.0)
            self._signal_item(viewer, "signal").setSelected(True)
            viewer.add_selected_curves()
            self._wait_until(lambda: "signal" in viewer.active_panel.curves)
            self.assertEqual(list(viewer.cache["time_s"]), [0.5, 1.0, 1.5])
            self.assertEqual(list(viewer.cache["signal"]), [20.0, 30.0, 40.0])
            viewer.close()

    def test_simulation_log_is_forwarded_to_console(self) -> None:
        viewer = ResultViewer()
        with mock.patch("sys.stderr") as stderr:
            viewer._simulation_log("user output")
            stderr.write.assert_called_once_with("user output\n")
            stderr.flush.assert_called_once_with()
        viewer.close()

    def test_multiple_sample_rates_use_each_files_time_column(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            circuit = Path(directory) / "circuit.csv"
            control = Path(directory) / "control.csv"
            circuit.write_bytes(b"time_s,ia\n0,1\n0.1,2\n0.2,3\n")
            control.write_bytes(b"time_s,scope_00\n0,10\n0.2,20\n")
            viewer = ResultViewer()
            viewer.open_files([circuit, control])
            self.assertEqual(viewer.columns.topLevelItemCount(), 2)
            for root_index in range(viewer.columns.topLevelItemCount()):
                root = viewer.columns.topLevelItem(root_index)
                self.assertFalse(root.flags() & QtCore.Qt.ItemIsSelectable)
            ia_item = self._signal_item(viewer, "ia", "circuit.csv")
            scope_item = self._signal_item(
                viewer, "scope_00", "control.csv"
            )
            self.assertNotIn("::", ia_item.text(0))
            self.assertEqual(
                ia_item.data(0, QtCore.Qt.UserRole), "circuit.csv :: ia"
            )
            ia_item.setSelected(True)
            scope_item.setSelected(True)
            viewer.add_selected_curves()
            self._wait_until(lambda: len(viewer.active_panel.curves) == 2)
            ia_key = next(name for name in viewer.active_panel.curves
                          if name.endswith(" :: ia"))
            scope_key = next(name for name in viewer.active_panel.curves
                             if name.endswith(" :: scope_00"))
            ia_x, _ = viewer.active_panel.curves[ia_key].getData()
            scope_x, _ = viewer.active_panel.curves[scope_key].getData()
            self.assertEqual(list(ia_x), [0.0, 0.1, 0.2])
            self.assertEqual(list(scope_x), [0.0, 0.2])
            viewer.close()

    def test_rolling_x_window_tracks_the_newest_sample(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scope.csv"
            path.write_bytes(b"time_s,signal\n0,1\n0.5,2\n1.0,3\n")
            viewer = ResultViewer()
            viewer.open_file(path)
            self._signal_item(viewer, "signal").setSelected(True)
            viewer.add_selected_curves()
            self._wait_until(lambda: "signal" in viewer.active_panel.curves)
            viewer.rolling_window_seconds.setValue(0.25)
            viewer.rolling_x.setChecked(True)
            left, right = viewer.active_panel.plot.viewRange()[0]
            self.assertAlmostEqual(left, 0.75, places=6)
            self.assertAlmostEqual(right, 1.0, places=6)
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

            viewer.add_double_clicked_curve(self._signal_item(viewer, "ia"))
            self._wait_until(lambda: "ia" in panel.curves)
            self.assertEqual(viewer.active_curves.count(), 1)
            viewer.active_curves.item(0).setSelected(True)
            viewer.remove_selected_curves()
            self.assertNotIn("ia", panel.curves)
            self.assertEqual(viewer.active_curves.count(), 0)
            viewer.close()


if __name__ == "__main__":
    unittest.main()
