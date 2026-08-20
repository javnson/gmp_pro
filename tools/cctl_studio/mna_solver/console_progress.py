"""Small dependency-free timed progress bar for MNA generation commands."""

from __future__ import annotations

import math
import sys
import time
from typing import TextIO


class TimedProgressBar:
    def __init__(
        self,
        label: str,
        total: int,
        *,
        width: int = 30,
        stream: TextIO | None = None,
        minimum_interval_s: float = 0.25,
    ) -> None:
        self.label = label
        self.total = max(int(total), 0)
        self.width = max(int(width), 10)
        self.stream = stream or sys.stdout
        self.interactive = bool(getattr(self.stream, "isatty", lambda: False)())
        self.minimum_interval_s = max(float(minimum_interval_s), 0.0)
        self.start_time = time.perf_counter()
        self.last_print_time = 0.0
        self.last_completed = -1
        self.finished = False
        self.last_percent_bucket = -1
        self.update(0, force=True)

    @staticmethod
    def _duration(seconds: float) -> str:
        if seconds < 0.0 or not math.isfinite(seconds):
            return "--:--"
        whole = int(seconds + 0.5)
        hours, remainder = divmod(whole, 3600)
        minutes, secs = divmod(remainder, 60)
        return f"{hours:d}:{minutes:02d}:{secs:02d}" if hours else f"{minutes:02d}:{secs:02d}"

    def update(self, completed: int, total: int | None = None, *, force: bool = False) -> None:
        if self.finished:
            return
        if total is not None:
            self.total = max(int(total), 0)
        completed = max(0, min(int(completed), self.total)) if self.total else 0
        now = time.perf_counter()
        percent_bucket = int((completed / self.total if self.total else 1.0) * 10.0)
        if (
            not force
            and completed != self.total
            and (
                now - self.last_print_time < self.minimum_interval_s
                or (not self.interactive and percent_bucket == self.last_percent_bucket)
            )
        ):
            return
        elapsed = now - self.start_time
        fraction = completed / self.total if self.total else 1.0
        filled = min(self.width, int(fraction * self.width + 0.5))
        rate = completed / elapsed if elapsed > 0.0 else 0.0
        eta = (self.total - completed) / rate if rate > 0.0 else float("inf")
        line = (
            f"{self.label:<24} [{'#' * filled}{'-' * (self.width - filled)}] "
            f"{completed:>{len(str(max(self.total, 1)))}}/{self.total} "
            f"{fraction * 100.0:6.2f}%  elapsed {self._duration(elapsed)}  ETA {self._duration(eta)}"
        )
        self.stream.write(("\r" if self.interactive else "") + line + ("" if self.interactive else "\n"))
        self.stream.flush()
        self.last_print_time = now
        self.last_completed = completed
        self.last_percent_bucket = percent_bucket

    def finish(self) -> None:
        if self.finished:
            return
        if self.last_completed != self.total:
            self.update(self.total, force=True)
        if self.interactive:
            self.stream.write("\n")
        self.stream.flush()
        self.finished = True
