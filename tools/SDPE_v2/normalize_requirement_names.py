#!/usr/bin/env python3
"""Normalize SDPE Requirement Name fields and their group references."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from sdpe_v2.project_requirements import title_case_name


def normalize_requirement_names(data: dict[str, Any]) -> bool:
    """Normalize one project document without changing macros or bindings."""

    changed = False
    renamed: dict[str, str] = {}
    for requirement in data.get("requirements", []):
        old = str(requirement.get("role", ""))
        new = title_case_name(old)
        if old and new != old:
            requirement["role"] = new
            renamed[old] = new
            changed = True
    for group in data.get("requirement_groups", []):
        old_names = [str(name) for name in group.get("requirements", [])]
        new_names = [renamed.get(name, title_case_name(name)) for name in old_names]
        if new_names != old_names:
            group["requirements"] = new_names
            changed = True
    return changed


def candidate_files(root: Path) -> list[Path]:
    if root.is_file():
        return [root]
    return sorted(path for path in root.rglob("*.json") if path.is_file())


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("roots", nargs="+", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    changed_paths: list[Path] = []
    for root in args.roots:
        for path in candidate_files(root):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(data, dict) or not isinstance(data.get("requirements"), list):
                continue
            if normalize_requirement_names(data):
                changed_paths.append(path)
                if not args.check:
                    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    for path in changed_paths:
        print(path)
    print(f"[SDPE] Requirement Name files {'pending' if args.check else 'updated'}: {len(changed_paths)}")
    return 1 if args.check and changed_paths else 0


if __name__ == "__main__":
    raise SystemExit(main())
