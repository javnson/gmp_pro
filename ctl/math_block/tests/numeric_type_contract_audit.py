#!/usr/bin/env python3
"""Reject parameter_gt arithmetic inside GMP real-time function bodies."""

from __future__ import annotations

import re
import sys
import os
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SCAN_ROOTS = (
    REPO_ROOT / "ctl" / "component",
    REPO_ROOT / "ctl" / "framework",
    REPO_ROOT / "ctl" / "suite",
)
SKIP_PARTS = {
    "build",
    "debug",
    "release",
    "gmp_inc",
    "gmp_src",
    "slprj",
    "tb",
    "tests",
    "drivers",
    "vcpkg_installed",
    "out",
    ".git",
}
REALTIME_NAME = re.compile(
    r"^(?:ctl_step_.+|gmp_base_ctl_step|ctl_dispatch|ctl_input_callback|"
    r"ctl_output_callback|.+(?:_isr|_ISR)|.+periodic.+)$"
)
NAME_CALL = re.compile(r"\b([A-Za-z_]\w*)\s*\(")
PARAMETER_TOKEN = re.compile(r"\bparameter_gt\b")
CONCRETE_LIBM_CALL = re.compile(
    r"\b(?:fabsf?|sinf?|cosf?|tanf?|asinf?|acosf?|atanf?|atan2f?|"
    r"expf?|logf?|log10f?|powf?|sqrtf?|floorf?|ceilf?|fmodf?)\s*\("
)
LEGACY_CONVERSION = re.compile(r"\b(?:float2ctrl|ctrl2float)\s*\(")
EXPLICIT_REAL_STORAGE = re.compile(r"\breal_gt\b")
STANDARD_CTRL_LITERAL = re.compile(
    r"\breal2ctrl\(\s*[+-]?(?:0(?:\.0+)?|0\.5(?:0+)?|1(?:\.0+)?|"
    r"2(?:\.0+)?|3(?:\.0+)?|4(?:\.0+)?)[fFlL]?\s*\)"
)
PARAMETER_INIT_CONVERSION = re.compile(
    r"\bctl_init_(?:pid|qpr_controller|qpr_controller_prewarped|lead_form3)\s*\([^;]*?"
    r"\b(?:real2ctrl|param2ctrl)\s*\(",
    re.DOTALL,
)


def strip_comments_and_literals(text: str) -> str:
    pattern = re.compile(
        r"//[^\n]*|/\*.*?\*/|\"(?:\\.|[^\"\\])*\"|'(?:\\.|[^'\\])*'",
        re.DOTALL,
    )

    def replace(match: re.Match[str]) -> str:
        value = match.group(0)
        return "".join("\n" if char == "\n" else " " for char in value)

    return pattern.sub(replace, text)


def matching_delimiter(text: str, start: int, opening: str, closing: str) -> int | None:
    depth = 0
    for index in range(start, len(text)):
        if text[index] == opening:
            depth += 1
        elif text[index] == closing:
            depth -= 1
            if depth == 0:
                return index
    return None


def source_files() -> list[Path]:
    result: list[Path] = []
    for root in SCAN_ROOTS:
        for directory, names, files in os.walk(root):
            names[:] = [name for name in names if name.lower() not in SKIP_PARTS]
            base = Path(directory)
            for name in files:
                path = base / name
                if path.suffix.lower() in {".c", ".h", ".cpp", ".hpp"}:
                    result.append(path)
    return sorted(result)


def violations(path: Path) -> list[tuple[int, str]]:
    original = path.read_text(encoding="utf-8", errors="replace")
    text = strip_comments_and_literals(original)
    found: list[tuple[int, str]] = []

    for match in NAME_CALL.finditer(text):
        name = match.group(1)
        if not REALTIME_NAME.match(name):
            continue

        open_paren = text.find("(", match.start(1) + len(name))
        close_paren = matching_delimiter(text, open_paren, "(", ")")
        if close_paren is None:
            continue

        cursor = close_paren + 1
        while cursor < len(text) and text[cursor].isspace():
            cursor += 1
        if cursor >= len(text) or text[cursor] != "{":
            continue

        close_brace = matching_delimiter(text, cursor, "{", "}")
        if close_brace is None:
            continue

        function_text = text[open_paren : close_brace + 1]
        if PARAMETER_TOKEN.search(function_text):
            line = original.count("\n", 0, match.start(1)) + 1
            found.append((line, name))

    return found


def main() -> int:
    all_violations: list[tuple[Path, int, str]] = []
    libm_violations: list[tuple[Path, int, str]] = []
    boundary_violations: list[tuple[Path, int, str]] = []
    for path in source_files():
        for line, name in violations(path):
            all_violations.append((path, line, name))
        original = path.read_text(encoding="utf-8", errors="replace")
        text = strip_comments_and_literals(original)
        for match in CONCRETE_LIBM_CALL.finditer(text):
            line = original.count("\n", 0, match.start()) + 1
            libm_violations.append((path, line, match.group(0).rstrip("(")))
        for label, pattern in (
            ("LEGACY_CONVERSION", LEGACY_CONVERSION),
            ("EXPLICIT_REAL_STORAGE", EXPLICIT_REAL_STORAGE),
            ("STANDARD_LITERAL_CONVERSION", STANDARD_CTRL_LITERAL),
            ("PREMATURE_PARAMETER_INIT_CONVERSION", PARAMETER_INIT_CONVERSION),
        ):
            for match in pattern.finditer(text):
                line = original.count("\n", 0, match.start()) + 1
                boundary_violations.append((path, line, label))

    for path, line, name in all_violations:
        relative = path.relative_to(REPO_ROOT).as_posix()
        print(f"PARAMETER_IN_REALTIME {relative}:{line}: {name}")

    for path, line, name in libm_violations:
        relative = path.relative_to(REPO_ROOT).as_posix()
        print(f"CONCRETE_LIBM_CALL {relative}:{line}: {name}")

    for path, line, label in boundary_violations:
        relative = path.relative_to(REPO_ROOT).as_posix()
        print(f"{label} {relative}:{line}")

    print(
        "Numeric type contract audit: "
        f"{len(all_violations)} parameter_gt real-time violation(s), "
        f"{len(libm_violations)} concrete libm call(s), "
        f"{len(boundary_violations)} numeric-boundary violation(s)."
    )
    return 1 if all_violations or libm_violations or boundary_violations else 0


if __name__ == "__main__":
    sys.exit(main())
