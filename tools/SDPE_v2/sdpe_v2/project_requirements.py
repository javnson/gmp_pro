"""Common/private project requirement composition for SDPE projects."""

from __future__ import annotations

import copy
import os
import re
from pathlib import Path
from typing import Any

from .util import macro_name, read_json


COMMON_REQUIREMENTS_KEY = "common_requirements"
LEGACY_COMMON_REQUIREMENT_KEY = "common_requirement"
SOURCE_KEY = "__sdpe_source"
GROUP_KEY = "__sdpe_group"


def title_case_name(value: Any) -> str:
    """Format a human-facing Name while preserving existing acronym casing."""

    words = re.split(r"\s+", str(value).replace("_", " ").strip())
    return " ".join(word[:1].upper() + word[1:] for word in words if word)


def _expanded_path(value: str) -> Path:
    expanded = re.sub(r"%([^%]+)%", lambda match: os.environ.get(match.group(1), match.group(0)), value)
    return Path(os.path.expandvars(os.path.expanduser(expanded)))


def legacy_common_requirement_path(project_path: Path) -> Path | None:
    """Locate the historical suite-level sdpe_general requirement."""

    project_path = project_path.resolve()
    for parent in project_path.parents:
        candidate = parent / "sdpe_general" / "sdpe_requirement.json"
        if candidate.is_file() and candidate.resolve() != project_path:
            return candidate.resolve()
    return None


def common_requirement_references(data: dict[str, Any]) -> list[str]:
    """Return configured references while accepting the short-lived singular form."""

    values = data.get(COMMON_REQUIREMENTS_KEY, [])
    if isinstance(values, str):
        values = [values]
    if not values and data.get(LEGACY_COMMON_REQUIREMENT_KEY):
        values = [data[LEGACY_COMMON_REQUIREMENT_KEY]]
    return [str(value).strip() for value in values if str(value).strip()]


def resolve_common_requirement_paths(project_path: Path, data: dict[str, Any] | None = None) -> list[Path]:
    """Resolve explicit common requirements, with legacy suite discovery fallback."""

    project_path = project_path.resolve()
    data = data if data is not None else read_json(project_path)
    references = common_requirement_references(data)
    resolved: list[Path] = []
    for reference in references:
        candidate = _expanded_path(reference)
        if not candidate.is_absolute():
            candidate = project_path.parent / candidate
        candidate = candidate.resolve()
        if candidate.is_file() and candidate != project_path:
            resolved.append(candidate)
    if resolved or references:
        return list(dict.fromkeys(resolved))
    legacy = legacy_common_requirement_path(project_path)
    return [legacy] if legacy is not None else []


def common_requirement_reference(project_path: Path, common_path: Path) -> str:
    """Return a portable path from a private requirement to its common file."""

    relative = os.path.relpath(common_path.resolve(), project_path.resolve().parent)
    return Path(relative).as_posix()


def load_project_requirements(project_path: Path) -> tuple[dict[str, Any], list[tuple[Path, dict[str, Any]]]]:
    """Load private and zero or more common requirement files in configured order."""

    project_path = project_path.resolve()
    private = read_json(project_path)
    commons = [
        (common_path, read_json(common_path))
        for common_path in resolve_common_requirement_paths(project_path, private)
    ]
    return private, commons


def requirement_rows(data: dict[str, Any], source: str) -> list[dict[str, Any]]:
    """Return requirements in configured group order with source metadata."""

    rows = [copy.deepcopy(item) for item in data.get("requirements", [])]
    pending: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        name = str(row.get("role", row.get("macro", "")))
        pending.setdefault(name, []).append(row)

    ordered: list[dict[str, Any]] = []
    for group in data.get("requirement_groups", []):
        group_name = str(group.get("name", "Requirements"))
        for name in group.get("requirements", []):
            candidates = pending.get(str(name), [])
            if candidates:
                row = candidates.pop(0)
                row[GROUP_KEY] = group_name
                ordered.append(row)
    ordered_ids = {id(row) for row in ordered}
    for row in rows:
        if id(row) in ordered_ids:
            continue
        row[GROUP_KEY] = str(row.get("group") or "Ungrouped")
        ordered.append(row)
    for row in ordered:
        row[SOURCE_KEY] = source
    return ordered


def macro_rows(data: dict[str, Any], key: str, source: str) -> list[dict[str, Any]]:
    rows = [copy.deepcopy(item) for item in data.get(key, [])]
    for row in rows:
        row[SOURCE_KEY] = source
    return rows


def merged_project_view(private: dict[str, Any], commons: list[dict[str, Any]]) -> dict[str, Any]:
    """Create GUI-only merged collections while retaining private project metadata."""

    merged = copy.deepcopy(private)
    sources = [("project private", private)]
    sources.extend((f"common:{data.get('id', index + 1)}", data) for index, data in enumerate(commons))
    merged["hardware"] = []
    for source, data in sources:
        for item in data.get("hardware", []):
            hardware = copy.deepcopy(item) if isinstance(item, dict) else {"entity": str(item)}
            hardware[SOURCE_KEY] = source
            merged["hardware"].append(hardware)
    merged["requirements"] = [row for source, data in sources for row in requirement_rows(data, source)]
    merged["feature_macros"] = [row for source, data in sources for row in macro_rows(data, "feature_macros", source)]
    merged["option_macros"] = [row for source, data in sources for row in macro_rows(data, "option_macros", source)]
    merged["feature_macro_groups"] = list(
        dict.fromkeys(
            group
            for _source, data in sources
            for group in data.get("feature_macro_groups", [])
        )
    )
    merged["option_macro_groups"] = list(
        dict.fromkeys(
            group
            for _source, data in sources
            for group in data.get("option_macro_groups", [])
        )
    )
    return merged


def project_requirement_paths(common_path: Path, search_roots: list[Path] | None = None) -> list[Path]:
    """Find private projects that actually bind the selected common file."""

    common_path = common_path.resolve()
    roots = [path.resolve() for path in (search_roots or [])]
    suite_project_root = common_path.parent.parent / "project"
    if suite_project_root.is_dir():
        roots.append(suite_project_root.resolve())
    gmp_root = os.environ.get("GMP_PRO_LOCATION", "").strip()
    if gmp_root:
        fleet_root = Path(gmp_root).expanduser().resolve() / "ctl" / "suite"
        if fleet_root.is_dir():
            roots.append(fleet_root)

    candidates: set[Path] = set()
    for root in dict.fromkeys(roots):
        paths = [root] if root.is_file() else root.rglob("sdpe_requirement.json")
        for path in paths:
            path = path.resolve()
            if not path.is_file() or path == common_path:
                continue
            try:
                data = read_json(path)
                if common_path in resolve_common_requirement_paths(path, data):
                    candidates.add(path)
            except (OSError, ValueError):
                continue
    return sorted(candidates)


def duplicate_macro_occurrences(documents: list[tuple[str, dict[str, Any]]]) -> dict[str, list[dict[str, Any]]]:
    """Return duplicate project macros with stable source/collection tokens."""

    occurrences: dict[str, list[dict[str, Any]]] = {}
    collections = (
        ("requirements", "Requirement"),
        ("feature_macros", "Selection Macro"),
        ("option_macros", "Option Macro"),
    )
    for source, data in documents:
        requirement_groups = {
            str(name): str(group.get("name", "Requirements"))
            for group in data.get("requirement_groups", [])
            for name in group.get("requirements", [])
        }
        for collection, kind in collections:
            for index, row in enumerate(data.get(collection, [])):
                macro = str(row.get("macro", "")).strip()
                key = macro_name(macro)
                if not key:
                    continue
                name = title_case_name(row.get("role", macro)) if collection == "requirements" else macro
                group = requirement_groups.get(name, "Ungrouped") if collection == "requirements" else str(row.get("group", "Macros"))
                occurrences.setdefault(key, []).append(
                    {
                        "token": f"{source}\x1f{collection}\x1f{index}",
                        "source": source,
                        "collection": collection,
                        "kind": kind,
                        "index": index,
                        "macro": macro,
                        "name": name,
                        "group": group,
                        "weak": bool(row.get("weak", False)),
                        "description": str(row.get("description", "")),
                    }
                )
    def valid_override(rows: list[dict[str, Any]]) -> bool:
        collections = {str(row["collection"]) for row in rows}
        private = [row for row in rows if row["source"] == "project private"]
        common = [row for row in rows if str(row["source"]).startswith("common:")]
        return (
            len(collections) == 1
            and len(private) == 1
            and len(common) == len(rows) - 1
            and all(bool(row.get("weak")) for row in common)
        )

    return {
        macro: rows
        for macro, rows in occurrences.items()
        if len(rows) > 1 and not valid_override(rows)
    }


def resolve_duplicate_macros(
    documents: list[tuple[str, dict[str, Any]]],
    keep_tokens: dict[str, str],
) -> list[tuple[str, dict[str, Any]]]:
    """Remove every duplicate occurrence except the selected token for each macro."""

    resolved = [(source, copy.deepcopy(data)) for source, data in documents]
    duplicate_keys = set(keep_tokens)
    for source, data in resolved:
        for collection in ("requirements", "feature_macros", "option_macros"):
            retained = []
            for index, row in enumerate(data.get(collection, [])):
                key = macro_name(str(row.get("macro", "")).strip())
                token = f"{source}\x1f{collection}\x1f{index}"
                if key not in duplicate_keys or keep_tokens.get(key) == token:
                    retained.append(row)
            data[collection] = retained

        valid_names = {
            str(row.get("role", row.get("macro", "")))
            for row in data.get("requirements", [])
        }
        grouped_names: set[str] = set()
        for group in data.get("requirement_groups", []):
            names = []
            for name in group.get("requirements", []):
                name = str(name)
                if name in valid_names and name not in grouped_names:
                    names.append(name)
                    grouped_names.add(name)
            group["requirements"] = names
    return resolved
