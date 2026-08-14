"""Audit Facility dependencies against repository-local #include directives.

The audit is intentionally read-only.  It verifies that every registered CTL
source/header can reach the Facility module that owns each direct repository
include.  Platform-alternative includes are handled by an explicit exemption.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
from pathlib import Path


CODE_SUFFIXES = {".h", ".c", ".cc", ".cpp", ".inl", ".inc"}
INCLUDE_RE = re.compile(rb'^\s*#\s*include\s*[<\"]([^>\"]+)[>\"]', re.MULTILINE)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo",
        type=Path,
        default=Path(os.environ.get("GMP_PRO_LOCATION", Path(__file__).resolve().parents[3])),
        help="GMP repository root (defaults to GMP_PRO_LOCATION or this script's repository)",
    )
    parser.add_argument(
        "--registry",
        type=Path,
        default=Path(__file__).with_name("gmp_framework_dic.json"),
        help="Facility registry JSON",
    )
    return parser.parse_args()


def load_modules(registry: Path) -> dict[str, dict]:
    data = json.loads(registry.read_text(encoding="utf-8"))
    return {
        f"{root}|{key}": value
        for root, tree in data["modules"].items()
        for key, value in tree.items()
        if isinstance(value, dict) and value.get("type") == "module"
    }


def registered_files(repo: Path, modules: dict[str, dict]):
    owners: dict[str, set[str]] = {}
    files: dict[str, set[str]] = {}
    actual_paths: dict[str, str] = {}
    for key, module in modules.items():
        selected: set[str] = set()
        for field in ("src_patterns", "inc_patterns"):
            for pattern in module.get(field, []):
                expanded = pattern.replace("${GMP_PRO_LOCATION}", str(repo)).replace("/", os.sep)
                for filename in glob.glob(expanded):
                    path = Path(filename)
                    if path.is_file() and path.suffix.lower() in CODE_SUFFIXES:
                        actual_relative = path.resolve().relative_to(repo).as_posix()
                        normalized_relative = actual_relative.lower()
                        selected.add(normalized_relative)
                        actual_paths.setdefault(normalized_relative, actual_relative)
                        owners.setdefault(normalized_relative, set()).add(key)
        files[key] = selected
    return owners, files, actual_paths


def dependency_closure(module: str, modules: dict[str, dict]) -> set[str]:
    found: set[str] = set()
    pending = list(modules[module].get("depends_on", [])) + list(
        modules[module].get("optional_depends_on", [])
    )
    while pending:
        item = pending.pop()
        if item in found:
            continue
        found.add(item)
        pending.extend(modules.get(item, {}).get("depends_on", []))
        pending.extend(modules.get(item, {}).get("optional_depends_on", []))
    return found


def canonical_owner(path: str, owners: dict[str, set[str]], files: dict[str, set[str]]):
    candidates = owners.get(path, set())
    if not candidates:
        return None
    return sorted(candidates, key=lambda key: (len(files[key]), -key.count("|"), key))[0]


def find_cycles(modules: dict[str, dict]) -> set[tuple[str, ...]]:
    cycles: set[tuple[str, ...]] = set()

    def visit(node: str, path: list[str]):
        if node in path:
            cycle = path[path.index(node) :] + [node]
            rotations = [tuple(cycle[i:-1] + cycle[:i] + [cycle[i]]) for i in range(len(cycle) - 1)]
            cycles.add(min(rotations))
            return
        for dependency in modules.get(node, {}).get("depends_on", []):
            if dependency in modules:
                visit(dependency, path + [node])

    for module in modules:
        visit(module, [])
    return cycles


def main() -> int:
    args = parse_args()
    repo = args.repo.resolve()
    modules = load_modules(args.registry.resolve())
    owners, files, actual_paths = registered_files(repo, modules)
    unresolved: set[tuple[str, str, str, str]] = set()

    for module, module_files in files.items():
        if not module.startswith("ctl|") or module == "ctl|portable|_internal":
            continue
        available = dependency_closure(module, modules) | {module}
        for relative in module_files:
            actual_relative = actual_paths[relative]
            for raw_include in INCLUDE_RE.findall((repo / actual_relative).read_bytes()):
                try:
                    include = raw_include.decode("ascii").replace("\\", "/").lower()
                except UnicodeDecodeError:
                    continue
                target = canonical_owner(include, owners, files)
                if target is None or target in available:
                    continue
                # GMP_CTL_PORTABLE selects an alternative type contract.  It is
                # not an unconditional dependency of the normal math package.
                if module == "ctl|math_block|_internal" and target == "ctl|portable|_internal":
                    continue
                unresolved.add((module, target, actual_relative, include))

    cycles = find_cycles(modules)
    for module, target, source, include in sorted(unresolved):
        print(f"MISSING {module} -> {target}: {source} includes {include}")
    for cycle in sorted(cycles):
        print("CYCLE " + " -> ".join(cycle))

    print(f"Facility dependency audit: {len(unresolved)} missing edge(s), {len(cycles)} cycle(s).")
    return 1 if unresolved or cycles else 0


if __name__ == "__main__":
    raise SystemExit(main())
