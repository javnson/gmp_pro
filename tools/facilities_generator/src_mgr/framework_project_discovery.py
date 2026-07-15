"""Shared discovery filters for GMP source-manager fleet operations."""

from __future__ import annotations

import subprocess
from pathlib import Path


def exclude_git_ignored(
    paths: list[Path] | set[Path], repository_root: Path
) -> tuple[list[Path], list[Path]]:
    """Split paths using the repository's complete Git ignore semantics.

    ``git check-ignore`` is intentionally used instead of maintaining another
    approximation of .gitignore. This honors nested ignore files, negation,
    character classes, and future rules added by repository maintainers.
    """
    root = repository_root.resolve()
    inside: dict[str, Path] = {}
    visible_outside: list[Path] = []

    for candidate in sorted({Path(path).resolve() for path in paths}):
        try:
            relative = candidate.relative_to(root).as_posix()
        except ValueError:
            visible_outside.append(candidate)
            continue
        inside[relative] = candidate

    if not inside:
        return visible_outside, []

    input_paths = "\0".join(inside) + "\0"
    try:
        result = subprocess.run(
            [
                "git",
                "-C",
                str(root),
                "check-ignore",
                "--no-index",
                "--stdin",
                "-z",
            ],
            input=input_paths,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
    except OSError as error:
        raise RuntimeError("Git is required to apply GMP repository ignore rules") from error

    if result.returncode not in (0, 1):
        detail = result.stderr.strip()
        raise RuntimeError(
            "git check-ignore failed" + (f": {detail}" if detail else "")
        )

    ignored_names = {name for name in result.stdout.split("\0") if name}
    ignored = [inside[name] for name in inside if name in ignored_names]
    visible = visible_outside + [inside[name] for name in inside if name not in ignored_names]
    return sorted(visible), sorted(ignored)
