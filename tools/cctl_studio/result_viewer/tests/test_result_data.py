from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


VIEWER_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VIEWER_DIR))

from result_data import (  # noqa: E402
    IncrementalResultReader,
    inspect_result_file,
    load_numeric_columns,
    minmax_decimate,
)


class ResultDataTests(unittest.TestCase):
    def test_inspect_and_load_selected_columns(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "result.csv"
            path.write_text("time_s,ia,ib\n0,1,2\n1,3,4\n", encoding="utf-8")
            result = inspect_result_file(path)
            loaded = load_numeric_columns(result, ("time_s", "ib"))
        self.assertEqual(result.columns, ("time_s", "ia", "ib"))
        np.testing.assert_array_equal(loaded["time_s"], [0.0, 1.0])
        np.testing.assert_array_equal(loaded["ib"], [2.0, 4.0])

    def test_minmax_decimation_preserves_extrema_and_endpoints(self) -> None:
        x = np.arange(10_000, dtype=float)
        y = np.sin(x / 100.0)
        y[4321] = 100.0
        y[6789] = -90.0
        xd, yd = minmax_decimate(x, y, 400)
        self.assertLessEqual(len(xd), 400)
        self.assertEqual(xd[0], x[0])
        self.assertEqual(xd[-1], x[-1])
        self.assertEqual(float(np.max(yd)), 100.0)
        self.assertEqual(float(np.min(yd)), -90.0)
        self.assertTrue(np.all(np.diff(xd) >= 0.0))

    def test_static_load_ignores_incomplete_final_row(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "running.csv"
            path.write_bytes(b"time_s,ia,ib\n0,1,2\n1,3,4\n2,5")
            result = inspect_result_file(path)
            loaded = load_numeric_columns(result, ("time_s", "ib"))
        np.testing.assert_array_equal(loaded["time_s"], [0.0, 1.0])
        np.testing.assert_array_equal(loaded["ib"], [2.0, 4.0])

    def test_incremental_reader_waits_for_row_completion(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "running.csv"
            path.write_bytes(b"time_s,ia,ib\n0,1,2\n1,3")
            result = inspect_result_file(path)
            reader = IncrementalResultReader(result, ("time_s", "ib"))
            first = reader.read_available()
            self.assertTrue(first.incomplete_tail)
            np.testing.assert_array_equal(first.columns["time_s"], [0.0])

            with path.open("ab") as stream:
                stream.write(b",4\n2,5,6\n")
            second = reader.read_available()
        self.assertFalse(second.incomplete_tail)
        np.testing.assert_array_equal(second.columns["time_s"], [1.0, 2.0])
        np.testing.assert_array_equal(second.columns["ib"], [4.0, 6.0])

    def test_malformed_last_complete_row_is_ignored(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stopped.csv"
            path.write_text(
                "time_s,ia,ib\n0,1,2\n1,not-a-number,4\n",
                encoding="utf-8",
            )
            result = inspect_result_file(path)
            reader = IncrementalResultReader(result, ("time_s", "ia"))
            chunk = reader.read_available()
        self.assertTrue(chunk.skipped_trailing_row)
        np.testing.assert_array_equal(chunk.columns["time_s"], [0.0])

    def test_malformed_nonfinal_row_remains_an_error(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "corrupt.csv"
            path.write_text(
                "time_s,ia,ib\n0,bad,2\n1,3,4\n",
                encoding="utf-8",
            )
            result = inspect_result_file(path)
            reader = IncrementalResultReader(result, ("time_s", "ia"))
            with self.assertRaisesRegex(ValueError, "before the final row"):
                reader.read_available()


if __name__ == "__main__":
    unittest.main()
