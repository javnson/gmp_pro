"""Numerically compare two CCTL simulation CSV outputs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("left", type=Path)
    parser.add_argument("right", type=Path)
    parser.add_argument("--tolerance", type=float, default=1.0e-8)
    args = parser.parse_args()

    maximum_difference = 0.0
    maximum_location = (0, 0)
    rows = 0
    with args.left.open(newline="", encoding="utf-8") as left_file, \
         args.right.open(newline="", encoding="utf-8") as right_file:
        left_reader = csv.reader(left_file)
        right_reader = csv.reader(right_file)
        if next(left_reader, None) != next(right_reader, None):
            raise RuntimeError("backend CSV headers do not match")

        for row_index, (left_row, right_row) in enumerate(
            zip(left_reader, right_reader, strict=True), start=1
        ):
            if len(left_row) != len(right_row):
                raise RuntimeError(f"column count differs at row {row_index}")
            for column_index, (left_text, right_text) in enumerate(
                zip(left_row, right_row, strict=True)
            ):
                difference = abs(float(left_text) - float(right_text))
                if not math.isfinite(difference):
                    raise RuntimeError(
                        f"non-finite difference at row {row_index}, column {column_index}"
                    )
                if difference > maximum_difference:
                    maximum_difference = difference
                    maximum_location = (row_index, column_index)
            rows = row_index

    print(
        f"compared {rows} rows; max abs difference={maximum_difference:.6e} "
        f"at row={maximum_location[0]}, column={maximum_location[1]}"
    )
    if maximum_difference > args.tolerance:
        print(f"[FAIL] Difference exceeds tolerance {args.tolerance:.6e}.")
        return 1
    print(f"[PASS] Backend outputs agree within {args.tolerance:.6e}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
