"""Data access and display decimation for large CCTL CSV result files."""

from __future__ import annotations

import csv
import io
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import numpy as np


@dataclass(frozen=True)
class ResultFile:
    path: Path
    columns: tuple[str, ...]
    delimiter: str


@dataclass(frozen=True)
class ResultChunk:
    columns: dict[str, np.ndarray]
    reset: bool = False
    skipped_trailing_row: bool = False
    incomplete_tail: bool = False


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
    if progress is not None:
        progress(5)
    reader = IncrementalResultReader(result, names)
    values = reader.read_available().columns
    if progress is not None:
        progress(100)
    return values


class IncrementalResultReader:
    """Read only newline-terminated numeric rows appended since the last poll.

    Writers commonly expose a partially written final row.  That byte suffix is
    deliberately left unread until a later poll supplies its newline.  A
    malformed newline-terminated row is ignored only when it is the final row
    at the current end of file; malformed rows in the middle remain errors.
    """

    def __init__(self, result: ResultFile, names: Iterable[str]):
        self.result = result
        self.names = tuple(dict.fromkeys(names))
        self.indices: tuple[int, ...] = self._resolve_indices(self.names)
        self._header_bytes = b""
        self._offset = 0
        self._reset_position()

    def _resolve_indices(self, names: tuple[str, ...]) -> tuple[int, ...]:
        indices = []
        for name in names:
            try:
                indices.append(self.result.columns.index(name))
            except ValueError as error:
                raise KeyError(f"unknown result column: {name}") from error
        return tuple(indices)

    def _reset_position(self) -> None:
        with self.result.path.open("rb") as stream:
            header = stream.readline()
        if not header:
            raise ValueError("result file is empty")
        self._header_bytes = header
        self._offset = len(header)

    def _empty_columns(self) -> dict[str, np.ndarray]:
        return {name: np.empty(0, dtype=np.float64) for name in self.names}

    def _load_block(self, block: bytes) -> np.ndarray:
        if not block.strip() or not self.names:
            return np.empty((0, len(self.names)), dtype=np.float64)
        return np.loadtxt(
            io.BytesIO(block),
            delimiter=self.result.delimiter,
            usecols=self.indices,
            ndmin=2,
            dtype=np.float64,
        )

    def read_available(self) -> ResultChunk:
        reset = False
        with self.result.path.open("rb") as stream:
            current_header = stream.readline()
            stream.seek(0, 2)
            file_size = stream.tell()
            if current_header != self._header_bytes or file_size < self._offset:
                self._header_bytes = current_header
                self._offset = len(current_header)
                reset = True
            stream.seek(self._offset)
            payload = stream.read()

        newline = payload.rfind(b"\n")
        if newline < 0:
            return ResultChunk(
                self._empty_columns(),
                reset=reset,
                incomplete_tail=bool(payload),
            )

        complete = payload[: newline + 1]
        self._offset += len(complete)
        incomplete_tail = newline + 1 < len(payload)
        lines = complete.splitlines(keepends=True)
        nonempty_positions = [
            index for index, line in enumerate(lines) if line.strip()
        ]
        final_nonempty = nonempty_positions[-1] if nonempty_positions else -1
        delimiter = self.result.delimiter.encode("ascii")
        expected_delimiters = len(self.result.columns) - 1
        malformed_widths = [
            index
            for index in nonempty_positions
            if lines[index].count(delimiter) != expected_delimiters
        ]
        skipped_trailing = False
        if malformed_widths:
            if malformed_widths == [final_nonempty] and not incomplete_tail:
                complete = b"".join(lines[:final_nonempty])
                skipped_trailing = True
            else:
                row_index = malformed_widths[0]
                raise ValueError(
                    f"invalid field count in appended row {row_index + 1} "
                    f"of {self.result.path}"
                )

        try:
            matrix = self._load_block(complete)
        except ValueError as original_error:
            # A width-correct last line can still contain a partially written
            # or otherwise nonnumeric field.  Retry without only that trailing
            # row; a failure in the valid prefix remains a hard error.
            if final_nonempty < 0 or incomplete_tail or skipped_trailing:
                raise ValueError(
                    f"invalid numeric data in {self.result.path}: {original_error}"
                ) from original_error
            prefix = b"".join(lines[:final_nonempty])
            try:
                matrix = self._load_block(prefix)
            except ValueError as prefix_error:
                raise ValueError(
                    f"invalid numeric data before the final row in "
                    f"{self.result.path}: {prefix_error}"
                ) from original_error
            skipped_trailing = True

        if not self.names:
            columns = {}
        elif matrix.size:
            columns = {
                name: matrix[:, index] for index, name in enumerate(self.names)
            }
        else:
            columns = self._empty_columns()
        return ResultChunk(
            columns,
            reset=reset,
            skipped_trailing_row=skipped_trailing,
            incomplete_tail=incomplete_tail,
        )


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
