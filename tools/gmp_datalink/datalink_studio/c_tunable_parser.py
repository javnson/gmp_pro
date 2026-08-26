"""Parser for C ``gmp_param_item_t`` dictionary initializers."""

from __future__ import annotations

import ast
import re
from typing import Iterator


_TYPE_PATTERN = re.compile(r"^GMP_PARAM_TYPE_[A-Z0-9_]+$")
_PERMISSION_PATTERN = re.compile(r"^GMP_PARAM_PERM_[A-Z0-9_]+$")
_NULL_NAME_PATTERN = re.compile(
    r"^(?:NULL|nullptr|0|\(\s*(?:const\s+)?char\s*\*\s*\)\s*0)$"
)


def strip_c_comments(text: str) -> str:
    """Remove C comments without treating comment markers in strings as comments."""
    result: list[str] = []
    index = 0
    quote = ""
    while index < len(text):
        char = text[index]
        following = text[index + 1] if index + 1 < len(text) else ""
        if quote:
            result.append(char)
            if char == "\\" and following:
                result.append(following)
                index += 2
                continue
            if char == quote:
                quote = ""
            index += 1
            continue
        if char in {'"', "'"}:
            quote = char
            result.append(char)
            index += 1
            continue
        if char == "/" and following == "/":
            index += 2
            while index < len(text) and text[index] not in "\r\n":
                index += 1
            continue
        if char == "/" and following == "*":
            index += 2
            while index + 1 < len(text) and text[index:index + 2] != "*/":
                index += 1
            index = min(index + 2, len(text))
            continue
        result.append(char)
        index += 1
    return "".join(result)


def _initializer_bodies(text: str) -> Iterator[str]:
    """Yield innermost brace bodies, which are the flat dictionary entries."""
    stack: list[int] = []
    quote = ""
    index = 0
    while index < len(text):
        char = text[index]
        if quote:
            if char == "\\" and index + 1 < len(text):
                index += 2
                continue
            if char == quote:
                quote = ""
            index += 1
            continue
        if char in {'"', "'"}:
            quote = char
        elif char == "{":
            stack.append(index)
        elif char == "}" and stack:
            start = stack.pop()
            body = text[start + 1:index]
            if "{" not in body and "}" not in body:
                yield body
        index += 1


def _split_fields(body: str) -> list[str]:
    fields: list[str] = []
    start = 0
    depth = 0
    quote = ""
    index = 0
    while index < len(body):
        char = body[index]
        if quote:
            if char == "\\" and index + 1 < len(body):
                index += 2
                continue
            if char == quote:
                quote = ""
        elif char in {'"', "'"}:
            quote = char
        elif char in "([":
            depth += 1
        elif char in ")]":
            depth = max(depth - 1, 0)
        elif char == "," and depth == 0:
            fields.append(body[start:index].strip())
            start = index + 1
        index += 1
    fields.append(body[start:].strip())
    return fields


def _c_string(value: str) -> str | None:
    value = value.strip()
    if not value or _NULL_NAME_PATTERN.fullmatch(value):
        return None
    if not (value.startswith('"') and value.endswith('"')):
        return None
    try:
        parsed = ast.literal_eval(value)
    except (SyntaxError, ValueError):
        return value[1:-1]
    return parsed if isinstance(parsed, str) else None


def _address_name(address: str) -> str:
    """Convert ``(void *)&object.member`` to a useful Import C fallback name."""
    ampersand = address.find("&")
    name = address[ampersand + 1:] if ampersand >= 0 else address
    return name.strip().strip("() ")


def parse_c_tunable_dictionary(text: str) -> list[dict]:
    """Parse three-, four-, or legacy five-field tunable dictionary entries.

    The current fourth field is ``name``.  A missing, ``NULL``, or empty name
    falls back to the C address expression so hand-imported dictionaries remain
    readable.  A fifth string field is retained as a legacy UI-only unit.
    """
    entries: list[dict] = []
    for body in _initializer_bodies(strip_c_comments(text)):
        fields = _split_fields(body)
        if len(fields) < 3 or "&" not in fields[0]:
            continue
        variable_type = fields[1]
        permission = fields[2]
        if not _TYPE_PATTERN.fullmatch(variable_type) or not _PERMISSION_PATTERN.fullmatch(permission):
            continue
        explicit_name = _c_string(fields[3]) if len(fields) >= 4 else None
        unit = _c_string(fields[4]) if len(fields) >= 5 else None
        entries.append({
            "id": len(entries),
            "name": explicit_name or _address_name(fields[0]),
            "unit": unit or "",
            "type": variable_type,
            "perm": permission,
            "display_hex": False,
            "enum_map": None,
        })
    return entries
