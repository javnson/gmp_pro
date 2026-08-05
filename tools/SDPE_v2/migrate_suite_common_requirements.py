#!/usr/bin/env python3
"""Migrate suite SDPE projects from manual common includes to explicit bindings."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

from sdpe_v2.project_requirements import COMMON_REQUIREMENTS_KEY, common_requirement_reference


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def remove_manual_common_include(text: str, header_name: str) -> str:
    pattern = re.compile(rf'^\s*#include\s*[<"]{re.escape(header_name)}[>"]\s*$')
    lines = [line for line in text.splitlines() if not pattern.match(line)]
    return "\n".join(lines).strip()


def migrate_project(project_path: Path, common_path: Path, common_header: str) -> bool:
    data = read_json(project_path)
    old = json.dumps(data, sort_keys=True, ensure_ascii=False)
    references = data.get(COMMON_REQUIREMENTS_KEY, [])
    if isinstance(references, str):
        references = [references]
    reference = common_requirement_reference(project_path, common_path)
    data[COMMON_REQUIREMENTS_KEY] = list(dict.fromkeys([reference, *references]))
    data.pop("common_requirement", None)

    sections = data.get("code_sections")
    if isinstance(sections, dict) and isinstance(sections.get("after_extern_open"), str):
        cleaned = remove_manual_common_include(sections["after_extern_open"], common_header)
        if cleaned:
            sections["after_extern_open"] = cleaned
        else:
            sections.pop("after_extern_open", None)
        if not sections:
            data.pop("code_sections", None)

    new = json.dumps(data, sort_keys=True, ensure_ascii=False)
    if new == old:
        return False
    project_path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return True


def migrate_suite_root(suite_root: Path, check_only: bool = False) -> tuple[int, int]:
    common_path = suite_root / "sdpe_general" / "sdpe_requirement.json"
    if not common_path.is_file():
        return 0, 0
    common = read_json(common_path)
    common_header = str(common.get("output_header", "")).strip()
    if not common_header:
        raise ValueError(f"Common requirement has no output_header: {common_path}")
    projects = sorted((suite_root / "project").rglob("sdpe_requirement.json"))
    changed = 0
    for project_path in projects:
        if check_only:
            data = read_json(project_path)
            reference = common_requirement_reference(project_path, common_path)
            references = data.get(COMMON_REQUIREMENTS_KEY, [])
            if isinstance(references, str):
                references = [references]
            if reference not in references:
                changed += 1
        elif migrate_project(project_path, common_path, common_header):
            changed += 1
    return changed, len(projects)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite-root", type=Path, required=True, help="Path containing ctl/suite subdirectories")
    parser.add_argument("--check", action="store_true", help="Report projects still requiring migration")
    args = parser.parse_args(argv)
    changed = 0
    total = 0
    for suite_root in sorted(path for path in args.suite_root.iterdir() if path.is_dir()):
        suite_changed, suite_total = migrate_suite_root(suite_root, args.check)
        if suite_total:
            print(f"[SDPE] {suite_root.name}: {suite_changed}/{suite_total} project(s) {'pending' if args.check else 'updated'}")
        changed += suite_changed
        total += suite_total
    print(f"[SDPE] Total: {changed}/{total} project(s) {'pending' if args.check else 'updated'}")
    return 1 if args.check and changed else 0


if __name__ == "__main__":
    raise SystemExit(main())
