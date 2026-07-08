"""Utility helpers for SDPE v2."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any


def read_json(path: Path) -> dict[str, Any]:
    """Read a UTF-8 JSON file."""

    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def write_if_changed(path: Path, content: str) -> bool:
    """Write a text file only when content changes."""

    path.parent.mkdir(parents=True, exist_ok=True)
    old = None
    if path.exists():
        old = path.read_text(encoding="utf-8")
    if old == content:
        return False
    path.write_text(content, encoding="utf-8", newline="\n")
    return True


def macro_name(value: str) -> str:
    """Convert an identifier or path fragment to a C macro token."""

    text = re.sub(r"[^0-9A-Za-z]+", "_", value).strip("_").upper()
    if not text:
        return "UNNAMED"
    if text[0].isdigit():
        return f"_{text}"
    return text


def header_guard(relative_path: str) -> str:
    """Create a stable include guard from a relative path."""

    return f"_SDPE_V2_{macro_name(relative_path)}_"


def c_literal(value: Any, value_format: str = "{}") -> str:
    """Format a JSON value as a C literal."""

    if isinstance(value, bool):
        raw = "1" if value else "0"
    elif isinstance(value, (int, float)):
        raw = repr(value)
    else:
        raw = str(value)
    return value_format.format(raw)
