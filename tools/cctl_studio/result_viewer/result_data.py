"""Data access and display decimation for large CCTL CSV result files."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import numpy as np


@dataclass(frozen=True)
class ResultFile:
    path: Path
    columns: tuple[str, ...]
    delimiter: str


def inspect_result_file(path: str | Path) -> ResultFile:
    selected = Path(path)
    with selected.open("r", encoding="utf-8-sig", newline="") as stream:
        sample = stream.read(64 * 1024)
    if not sample.strip():
        raise ValueError("result file is empty")
    first_line = sample.splitlines()[0]
    try:
        delimiter = csv.Sniffer().sniff(sample, delimiters=",;\t").delimiter
    except csv.Error:
        delimiter = ","
    columns = tuple(value.strip() for value in next(csv.reader([first_line], delimiter=delimiter)))
    if not columns or any(not value for value in columns):
        raise ValueError("result file has an invalid or empty header")
    if len(set(columns)) != len(columns):
        raise ValueError("result file contains duplicate column names")
    return ResultFile(selected.resolve(), columns, delimiter)


def load_numeric_columns(
    result: ResultFile,
    names: Iterable[str],
    progress: Callable[[int], None] | None = None,
) -> dict[str, np.ndarray]:
    requested = tuple(dict.fromkeys(names))
    if not requested:
        return {}
    indices = []
    for name in requested:
        try:
            indices.append(result.columns.index(name))
        except ValueError as error:
            raise KeyError(f"unknown result column: {name}") from error
    if progress is not None:
        progress(5)
    values = np.loadtxt(
        result.path,
        delimiter=result.delimiter,
        skiprows=1,
        usecols=tuple(indices),
        ndmin=2,
        dtype=np.float64,
    )
    if progress is not None:
        progress(100)
    return {name: values[:, index] for index, name in enumerate(requested)}


def minmax_decimate(
    x: np.ndarray, y: np.ndarray, maximum_points: int
) -> tuple[np.ndarray, np.ndarray]:
    """Reduce a curve while retaining the first/last and each bucket's extrema."""

    x_values = np.asarray(x, dtype=np.float64).reshape(-1)
    y_values = np.asarray(y, dtype=np.float64).reshape(-1)
    if x_values.size != y_values.size:
        raise ValueError("x and y columns have different lengths")
    if maximum_points < 4:
        raise ValueError("maximum_points must be at least four")
    count = x_values.size
    if count <= maximum_points:
        return x_values, y_values

    bucket_count = max(1, (maximum_points - 2) // 2)
    edges = np.linspace(1, count - 1, bucket_count + 1, dtype=np.int64)
    selected: list[int] = [0]
    for begin, end in zip(edges[:-1], edges[1:]):
        if end <= begin:
            continue
        segment = y_values[begin:end]
        finite = np.isfinite(segment)
        if not finite.any():
            selected.append(int(begin))
            continue
        finite_indices = np.flatnonzero(finite)
        finite_values = segment[finite]
        low = int(begin + finite_indices[int(np.argmin(finite_values))])
        high = int(begin + finite_indices[int(np.argmax(finite_values))])
        selected.extend((low, high) if low <= high else (high, low))
    selected.append(count - 1)
    unique = np.asarray(list(dict.fromkeys(selected)), dtype=np.int64)
    return x_values[unique], y_values[unique]
