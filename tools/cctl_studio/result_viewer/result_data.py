"""Data access and display decimation for large CCTL CSV result files."""

from __future__ import annotations

import csv
import io
from collections import deque
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


def load_numeric_time_window(
    result: ResultFile,
    names: Iterable[str],
    time_name: str,
    *,
    start_time: float | None = None,
    duration_s: float = 1.0,
    follow_latest: bool = True,
    maximum_offset: int | None = None,
) -> dict[str, np.ndarray]:
    """Load selected columns without retaining the complete result in memory.

    In ``follow_latest`` mode a time-bounded deque retains only the newest
    ``duration_s`` interval while the file is scanned.  Fixed selection mode
    retains only ``[start_time, start_time + duration_s]``.  The scan is
    intentionally streaming: file size affects I/O time, not resident array
    size.
    """
    selected_names = tuple(dict.fromkeys(names))
    if time_name not in selected_names:
        selected_names = (time_name, *selected_names)
    if duration_s <= 0.0 or not np.isfinite(duration_s):
        raise ValueError("time-window duration must be finite and positive")
    try:
        indices = tuple(result.columns.index(name) for name in selected_names)
        time_index = selected_names.index(time_name)
    except ValueError as error:
        raise KeyError(f"unknown result column: {error}") from error

    rows: deque[tuple[float, ...]] | list[tuple[float, ...]]
    rows = deque() if follow_latest else []
    lower = float(start_time or 0.0)
    upper = lower + duration_s
    pending_error: ValueError | None = None
    with result.path.open("rb") as stream:
        header = stream.readline()
        if not header:
            raise ValueError("result file is empty")
        while maximum_offset is None or stream.tell() < maximum_offset:
            line = stream.readline()
            if not line:
                break
            if maximum_offset is not None and stream.tell() > maximum_offset:
                break
            if not line.endswith((b"\n", b"\r")):
                break
            if not line.strip():
                continue
            try:
                fields = next(csv.reader(
                    [line.decode("utf-8")], delimiter=result.delimiter
                ))
                if len(fields) != len(result.columns):
                    raise ValueError("invalid field count")
                values = tuple(float(fields[index]) for index in indices)
            except (UnicodeDecodeError, ValueError) as error:
                pending_error = ValueError(
                    f"invalid numeric row in {result.path}: {error}"
                )
                continue
            if pending_error is not None:
                # Only an invalid final complete row is tolerated.
                raise pending_error
            timestamp = values[time_index]
            if not np.isfinite(timestamp):
                continue
            if follow_latest:
                rows.append(values)
                threshold = timestamp - duration_s
                while rows and rows[0][time_index] < threshold:
                    rows.popleft()
            elif lower <= timestamp <= upper:
                rows.append(values)

    materialized = list(rows)
    if not materialized:
        return {
            name: np.empty(0, dtype=np.float64) for name in selected_names
        }
    matrix = np.asarray(materialized, dtype=np.float64)
    return {
        name: matrix[:, index] for index, name in enumerate(selected_names)
    }


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

    def initialize_time_window(
        self, time_name: str, duration_s: float
    ) -> ResultChunk:
        """Initialize at EOF while retaining only the newest time interval."""
        with self.result.path.open("rb") as stream:
            stream.seek(0, 2)
            end = stream.tell()
            while end > len(self._header_bytes):
                stream.seek(end - 1)
                if stream.read(1) == b"\n":
                    break
                end -= 1
        values = load_numeric_time_window(
            self.result,
            self.names,
            time_name,
            duration_s=duration_s,
            follow_latest=True,
            maximum_offset=end,
        )
        self._offset = end
        return ResultChunk(values)

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
