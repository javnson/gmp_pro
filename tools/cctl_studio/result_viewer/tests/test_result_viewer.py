from __future__ import annotations

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
VIEWER_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VIEWER_DIR))

from PyQt5 import QtWidgets  # noqa: E402
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
            viewer.dynamic_refresh.setChecked(False)
            viewer.close()


if __name__ == "__main__":
    unittest.main()
