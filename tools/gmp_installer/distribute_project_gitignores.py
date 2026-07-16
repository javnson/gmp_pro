#!/usr/bin/env python3
"""Distribute a portable root .gitignore subset to standalone GMP projects."""

from __future__ import annotations

from pathlib import Path


INSTALLER_DIR = Path(__file__).resolve().parent
GMP_ROOT = INSTALLER_DIR.parent.parent
TEMPLATE_PATH = INSTALLER_DIR / "project_gitignore.template"
BEGIN_MARKER = "# BEGIN GMP MANAGED PROJECT IGNORE RULES"
END_MARKER = "# END GMP MANAGED PROJECT IGNORE RULES"


def immediate_project_directories(parent: Path, excluded: set[str]) -> list[Path]:
    if not parent.is_dir():
        return []
    return [
        child
        for child in parent.iterdir()
        if child.is_dir() and child.name not in excluded and not child.name.startswith(".")
    ]


def discover_projects() -> list[Path]:
    projects = set(
        immediate_project_directories(
            GMP_ROOT / "csp" / "stm32",
            {"common", "src"},
        )
    )
    projects.update(
        immediate_project_directories(
            GMP_ROOT / "csp" / "c28x_syscfg",
            {"doc", "src"},
        )
    )

    suite_root = GMP_ROOT / "ctl" / "suite"
    if suite_root.is_dir():
        for suite in suite_root.iterdir():
            projects.update(immediate_project_directories(suite / "project", set()))

    return sorted(projects, key=lambda path: path.relative_to(GMP_ROOT).as_posix().lower())


def merge_managed_block(existing: str, managed_block: str) -> str:
    begin = existing.find(BEGIN_MARKER)
    end = existing.find(END_MARKER)
    if begin >= 0 or end >= 0:
        if begin < 0 or end < begin:
            raise ValueError("incomplete GMP managed ignore block")
        end += len(END_MARKER)
        prefix = existing[:begin].rstrip()
        suffix = existing[end:].strip()
        sections = [section for section in (prefix, managed_block, suffix) if section]
        return "\n\n".join(sections) + "\n"

    existing = existing.strip()
    if not existing:
        return managed_block + "\n"
    return managed_block + "\n\n# Project-specific ignore rules\n" + existing + "\n"


def main() -> int:
    managed_block = TEMPLATE_PATH.read_text(encoding="utf-8").strip()
    if not managed_block.startswith(BEGIN_MARKER) or not managed_block.endswith(END_MARKER):
        raise ValueError(f"managed markers are missing from {TEMPLATE_PATH}")

    projects = discover_projects()
    updated = 0
    for project in projects:
        destination = project / ".gitignore"
        existing = destination.read_text(encoding="utf-8") if destination.is_file() else ""
        merged = merge_managed_block(existing, managed_block)
        if existing.replace("\r\n", "\n") == merged:
            continue
        destination.write_text(merged, encoding="utf-8", newline="\n")
        updated += 1
        print(f"[UPDATE] {destination.relative_to(GMP_ROOT).as_posix()}")

    print(f"[OK] Project .gitignore distribution: {len(projects)} projects, {updated} updated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
