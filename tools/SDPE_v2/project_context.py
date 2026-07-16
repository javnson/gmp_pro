#!/usr/bin/env python3
"""Resolve and open the related two-layer SDPE project requirements."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def requirement(manager: Path) -> Path | None:
    path = manager / "sdpe_requirement.json"
    return path.resolve() if path.is_file() else None


def find_suite_root(manager: Path) -> Path | None:
    for parent in (manager, *manager.parents):
        if (parent / "sdpe_general" / "sdpe_requirement.json").is_file():
            return parent
    return None


def related_requirements(manager: Path) -> list[Path]:
    manager = manager.resolve()
    current = requirement(manager)
    if current is None:
        raise FileNotFoundError(f"No sdpe_requirement.json in {manager}")
    suite_root = find_suite_root(manager)
    if suite_root is None:
        return [current]
    general = (suite_root / "sdpe_general" / "sdpe_requirement.json").resolve()
    if manager.name == "sdpe_general":
        project_root = suite_root / "project"
        projects = sorted(
            path.resolve()
            for path in project_root.glob("**/sdpe_mgr/sdpe_requirement.json")
            if path.is_file()
        )
        return list(dict.fromkeys([general, *projects]))
    return list(dict.fromkeys([general, current]))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manager", type=Path, required=True)
    parser.add_argument("--settings", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--print-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        projects = related_requirements(args.manager)
    except FileNotFoundError as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 2
    for path in projects:
        print(f"[SDPE] Project: {path}")
    if args.print_only:
        return 0
    command = [
        sys.executable,
        str(ROOT / "gui_pyqt" / "sdpe_gui.py"),
        "--settings", str(args.settings.resolve()),
        "--mode", "project",
        "--projects", *(str(path) for path in projects),
        "--out", str(args.out.resolve()),
    ]
    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
