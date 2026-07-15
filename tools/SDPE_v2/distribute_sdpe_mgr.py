#!/usr/bin/env python3
"""Distribute the canonical SDPE BAT toolchain to every project manager."""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DISCOVERY_MODULE_DIR = ROOT.parent / "facilities_generator" / "src_mgr"
if str(DISCOVERY_MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(DISCOVERY_MODULE_DIR))

from framework_project_discovery import exclude_git_ignored  # noqa: E402


TOOLCHAIN_FILES = (
    "sdpe_settings.bat",
    "sdpe_edit.bat",
    "sdpe_generate.bat",
    "sdpe_validate.bat",
)


def discover_managers(repository_root: Path) -> tuple[list[Path], list[Path]]:
    canonical = (ROOT / "sdpe_mgr").resolve()
    candidates: set[Path] = set()

    for dirpath, dirnames, _filenames in os.walk(repository_root):
        current = Path(dirpath).resolve()
        if current == canonical:
            dirnames[:] = []
            continue
        if current.name == "sdpe_mgr":
            dirnames[:] = []
            if (current / "sdpe_requirement.json").is_file():
                candidates.add(current)
            else:
                print(f"[SKIP] sdpe_mgr has no project requirement: {current}")
            continue

        # These roots cannot contain deployable project managers and can be
        # extremely large. Other ignored trees are filtered by Git below.
        dirnames[:] = [name for name in dirnames if name not in {".git", "bin", "__pycache__"}]

    visible, ignored = exclude_git_ignored(candidates, repository_root)
    return visible, ignored


def copy_atomic(source: Path, destination: Path) -> None:
    temporary = destination.with_name(destination.name + ".gmp-distribute.tmp")
    try:
        shutil.copy2(source, temporary)
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)


def distribute(*, dry_run: bool = False) -> bool:
    root_value = os.environ.get("GMP_PRO_LOCATION")
    if not root_value:
        print("[ERROR] GMP_PRO_LOCATION is not defined.", file=sys.stderr)
        return False
    repository_root = Path(root_value).resolve()
    if not (repository_root / ".gitignore").is_file():
        print(f"[ERROR] GMP_PRO_LOCATION is not a GMP repository: {repository_root}", file=sys.stderr)
        return False

    template = ROOT / "sdpe_mgr"
    missing_templates = [name for name in TOOLCHAIN_FILES if not (template / name).is_file()]
    if missing_templates:
        print(f"[ERROR] Canonical SDPE toolchain is incomplete: {', '.join(missing_templates)}")
        return False

    try:
        managers, ignored = discover_managers(repository_root)
    except RuntimeError as error:
        print(f"[ERROR] SDPE manager discovery failed: {error}", file=sys.stderr)
        return False

    for path in ignored:
        print(f"[IGNORE] Git-ignored SDPE manager: {path}")
    if not managers:
        print("[WARNING] No project-local sdpe_mgr directories were found.")
        return True

    failures = 0
    for manager in managers:
        relative = manager.relative_to(repository_root)
        print(f"[DEPLOY] {relative}")
        if dry_run:
            continue
        try:
            for name in TOOLCHAIN_FILES:
                copy_atomic(template / name, manager / name)
                print(f"  -> [OVERWRITE] {name}")
        except OSError as error:
            failures += 1
            print(f"  -> [ERROR] {error}")

    action = "would update" if dry_run else "updated"
    print(f"[SUMMARY] SDPE toolchain {action}: {len(managers) - failures} manager(s).")
    print(f"[SUMMARY] Failures: {failures} manager(s).")
    return failures == 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="discover and report without copying")
    args = parser.parse_args()
    return 0 if distribute(dry_run=args.dry_run) else 1


if __name__ == "__main__":
    raise SystemExit(main())
