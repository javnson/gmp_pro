from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


VIEWER_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VIEWER_DIR))

from result_data import inspect_result_file, load_numeric_columns, minmax_decimate  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()

