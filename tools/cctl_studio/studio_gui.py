#!/usr/bin/env python3
"""Stable launcher for the two-level GMP CCTL Studio desktop editor."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from layered_ui import launch


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Launch the two-level GMP CCTL Studio graphical editor."
    )
    parser.add_argument("project", nargs="?", type=Path, help="optional project JSON")
    args = parser.parse_args(argv)
    return launch(args.project)


if __name__ == "__main__":
    raise SystemExit(main())
