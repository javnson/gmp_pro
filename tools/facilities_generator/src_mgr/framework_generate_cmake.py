"""Generate a CMake integration file for a deployed GMP source workspace."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from framework_registry import RegistrySelectionError, resolve_selected_modules


SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".s", ".S"}
LOCAL_SOURCE_DIRS = ("user", "xplt")


def _cmake_quote(value: str) -> str:
    """Return a CMake-safe double-quoted string."""
    return '"' + value.replace("\\", "/").replace('"', '\\"') + '"'


def _find_workspace(path: Path) -> tuple[Path, Path]:
    """Return the source-manager directory and its project root."""
    workspace = path.resolve()
    if workspace.name == "gmp_src_mgr":
        manager_dir = workspace
        project_root = workspace.parent
    elif (workspace / "gmp_src_mgr").is_dir():
        manager_dir = workspace / "gmp_src_mgr"
        project_root = workspace
    else:
        raise ValueError(
            "The workspace must be a gmp_src_mgr directory or a project containing gmp_src_mgr."
        )
    return manager_dir, project_root


def _collect_sources(manager_dir: Path, project_root: Path) -> list[Path]:
    """Collect generated and conventional project-local source files."""
    source_dirs = [manager_dir / "gmp_src"]
    source_dirs.extend(project_root / name for name in LOCAL_SOURCE_DIRS)

    sources: set[Path] = set()
    for source_dir in source_dirs:
        if not source_dir.is_dir():
            continue
        for path in source_dir.rglob("*"):
            if path.is_file() and path.suffix in SOURCE_SUFFIXES:
                sources.add(path.resolve())
    return sorted(sources, key=lambda item: item.as_posix().lower())


def _parse_include_summary(summary_path: Path) -> list[str]:
    """Extract path entries from gmp_compiler_includes.txt."""
    if not summary_path.is_file():
        return []

    paths: list[str] = []
    for raw_line in summary_path.read_text(encoding="utf-8-sig").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("="):
            continue
        if line[0].isdigit() and "." in line[:3]:
            continue
        if line.replace("\\", "/").rstrip("/") in {"./gmp_inc", "gmp_inc"}:
            continue
        paths.append(line)
    return paths


def _collect_legacy_include_dirs(manager_dir: Path) -> list[str]:
    """Return legacy include-summary paths only for header-sync workspaces.

    A src_only workspace resolves its complete dependency closure from the
    registry.  Its summary is commonly an old machine-local artifact and may
    point at a different CSP whose standard header names would shadow the
    selected platform.
    """
    config_path = manager_dir / "gmp_framework_config.json"
    if config_path.is_file():
        config = json.loads(config_path.read_text(encoding="utf-8-sig"))
        if config.get("sync_mode", "all") == "src_only":
            return []
    return _parse_include_summary(manager_dir / "gmp_compiler_includes.txt")


def _collect_registered_include_dirs(
    manager_dir: Path,
    gmp_root: Path,
    dictionary_path: Path | None = None,
) -> list[str]:
    """Resolve every inc_dirs entry in the selected dependency closure.

    Source-only workspaces do not run the header mirror generator, so their
    gmp_compiler_includes.txt can be absent or stale. CMake generation must use
    the canonical registry and local selection directly instead of relying on
    that derived summary file.
    """
    config_path = manager_dir / "gmp_framework_config.json"
    registry_path = dictionary_path or Path(__file__).resolve().with_name(
        "gmp_framework_dic.json"
    )
    if not config_path.is_file():
        raise FileNotFoundError(f"Source-manager configuration not found: {config_path}")
    if not registry_path.is_file():
        raise FileNotFoundError(f"GMP facility registry not found: {registry_path}")

    config = json.loads(config_path.read_text(encoding="utf-8-sig"))
    registry = json.loads(registry_path.read_text(encoding="utf-8-sig"))
    try:
        selected_modules = resolve_selected_modules(registry, config)
    except RegistrySelectionError as error:
        raise ValueError(f"Cannot resolve Facility dependencies: {error}") from error

    macros = dict(registry.get("macros", {}))
    macros["GMP_PRO_LOCATION"] = gmp_root.as_posix()
    macro_items = sorted(macros.items(), key=lambda item: len(item[0]), reverse=True)

    include_dirs: set[str] = set()
    for root_name, module_name in selected_modules:
        module = registry["modules"][root_name][module_name]
        for raw_path in module.get("inc_dirs", []):
            expanded = str(raw_path)
            for macro_name, macro_value in macro_items:
                expanded = expanded.replace(f"${{{macro_name}}}", str(macro_value))
            candidate = Path(expanded)
            if not candidate.is_absolute():
                candidate = gmp_root / candidate
            if candidate.is_dir():
                include_dirs.add(candidate.resolve().as_posix())

    sync_mode = config.get("sync_mode", "all")
    if sync_mode in {"all", "inc_only"}:
        for raw_path in config.get("local_include_dirs", ["gmp_inc"]):
            candidate = manager_dir / str(raw_path)
            if candidate.is_dir():
                include_dirs.add(candidate.resolve().as_posix())
    return sorted(include_dirs)


def _render_include_path(
    raw_path: str,
    manager_dir: Path,
    project_root: Path,
    output_dir: Path,
    gmp_root: Path,
) -> str:
    """Convert an include path to a relocatable CMake expression."""
    normalized = raw_path.replace("\\", "/")
    candidate = Path(normalized)

    if normalized.startswith("./") or normalized.startswith("../"):
        absolute = (manager_dir / candidate).resolve()
    elif candidate.is_absolute():
        absolute = candidate.resolve()
    else:
        absolute = (manager_dir / candidate).resolve()

    try:
        relative = absolute.relative_to(gmp_root)
        suffix = relative.as_posix()
        return "${GMP_PRO_ROOT}" + (f"/{suffix}" if suffix != "." else "")
    except ValueError:
        pass

    try:
        relative = os.path.relpath(absolute, output_dir).replace("\\", "/")
        return "${CMAKE_CURRENT_LIST_DIR}" + (f"/{relative}" if relative != "." else "")
    except ValueError:
        return absolute.as_posix()


def _collect_cmake_libraries(manager_dir: Path) -> list[str]:
    """Collect CMake link libraries declared by the selected GMP modules."""
    config_path = manager_dir / "gmp_framework_config.json"
    dictionary_path = Path(__file__).resolve().with_name("gmp_framework_dic.json")
    if not config_path.is_file() or not dictionary_path.is_file():
        return []

    config = json.loads(config_path.read_text(encoding="utf-8-sig"))
    dictionary = json.loads(dictionary_path.read_text(encoding="utf-8-sig"))
    libraries: set[str] = set()
    try:
        selected_modules = resolve_selected_modules(dictionary, config)
    except RegistrySelectionError as error:
        raise ValueError(f"Cannot resolve Facility dependencies: {error}") from error
    for root_name, module_name in selected_modules:
        module = dictionary["modules"][root_name][module_name]
        libraries.update(module.get("cmake_libraries", []))
    return sorted(libraries)


def generate_cmake(workspace: Path, output: Path | None = None) -> Path:
    """Generate gmp_config.cmake and return its absolute path."""
    manager_dir, project_root = _find_workspace(workspace)
    gmp_location = os.environ.get("GMP_PRO_LOCATION")
    if not gmp_location:
        raise EnvironmentError("Environment variable GMP_PRO_LOCATION is not defined.")

    gmp_root = Path(gmp_location).resolve()
    sources = _collect_sources(manager_dir, project_root)
    if not sources:
        raise FileNotFoundError(
            f"No generated source files were found under {manager_dir / 'gmp_src'}. "
            "Run gmp_generate_src.bat first."
        )

    output_path = (output or (manager_dir / "gmp_config.cmake")).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    raw_include_paths = _collect_registered_include_dirs(manager_dir, gmp_root)
    # Header-sync workspaces may still carry explicitly emitted external paths.
    # src_only workspaces must use the registry exclusively so a stale summary
    # cannot shadow the selected CSP's standard csp.* headers.
    raw_include_paths.extend(_collect_legacy_include_dirs(manager_dir))
    include_paths = {
        _render_include_path(path, manager_dir, project_root, output_path.parent, gmp_root)
        for path in raw_include_paths
    }
    for directory_name in LOCAL_SOURCE_DIRS:
        if (project_root / directory_name).is_dir():
            relative = os.path.relpath(project_root / directory_name, output_path.parent)
            include_paths.add(f"${{CMAKE_CURRENT_LIST_DIR}}/{relative.replace(os.sep, '/')}")

    source_lines = []
    for source in sources:
        relative = os.path.relpath(source, output_path.parent).replace("\\", "/")
        source_lines.append(f"    {_cmake_quote('${CMAKE_CURRENT_LIST_DIR}/' + relative)}")

    include_lines = [f"    {_cmake_quote(path)}" for path in sorted(include_paths)]
    library_lines = [f"    {_cmake_quote(name)}" for name in _collect_cmake_libraries(manager_dir)]
    link_section = []
    if library_lines:
        link_section = [
            "target_link_libraries(${GMP_CMAKE_TARGET} PRIVATE",
            *library_lines,
            ")",
            "",
        ]
    content = "\n".join(
        [
            "# This file is generated by the GMP source manager. Do not edit it manually.",
            "",
            "if(NOT DEFINED ENV{GMP_PRO_LOCATION})",
            '    message(FATAL_ERROR "GMP_PRO_LOCATION is not defined. Install or activate GMP first.")',
            "endif()",
            'file(TO_CMAKE_PATH "$ENV{GMP_PRO_LOCATION}" GMP_PRO_ROOT)',
            "",
            "if(NOT DEFINED GMP_CMAKE_TARGET)",
            '    set(GMP_CMAKE_TARGET "${CMAKE_PROJECT_NAME}")',
            "endif()",
            "if(NOT TARGET ${GMP_CMAKE_TARGET})",
            '    message(FATAL_ERROR "GMP target \'${GMP_CMAKE_TARGET}\' does not exist. Include this file after add_executable/add_library.")',
            "endif()",
            "",
            "set(GMP_GENERATED_SOURCES",
            *source_lines,
            ")",
            "target_sources(${GMP_CMAKE_TARGET} PRIVATE ${GMP_GENERATED_SOURCES})",
            "",
            "target_include_directories(${GMP_CMAKE_TARGET} PRIVATE",
            *include_lines,
            ")",
            "",
            *link_section,
        ]
    )
    output_path.write_text(content, encoding="utf-8", newline="\n")
    return output_path


def main() -> int:
    """Run the command-line CMake generator."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path.cwd(),
        help="gmp_src_mgr directory or its parent project directory",
    )
    parser.add_argument("--output", type=Path, help="optional output .cmake path")
    args = parser.parse_args()

    try:
        output_path = generate_cmake(args.workspace, args.output)
    except Exception as exc:
        print(f"[ERROR] {exc}")
        return 1

    print(f"[SUCCESS] Generated CMake integration: {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
