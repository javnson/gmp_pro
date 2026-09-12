#!/usr/bin/env python3
"""Synchronize CubeMX IDE projects with a GMP CMake target.

CubeMX only knows about the sources it generated.  GMP projects also compile
generated GMP sources and family-level user/xplt sources.  This utility uses
CMake's compile_commands.json as the authoritative source, include and define
list, then updates an STM32CubeIDE .project/.cproject pair or an MDK uvprojx.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", required=True, type=Path)
    parser.add_argument("--compile-commands", required=True, type=Path)
    parser.add_argument("--cubeide", type=Path)
    parser.add_argument("--keil", type=Path)
    parser.add_argument("--keil-target", help="TargetName inside a multi-target uvprojx")
    parser.add_argument(
        "--target-object-fragment",
        help="Only use compile entries whose output path contains this fragment",
    )
    args = parser.parse_args()
    if not args.cubeide and not args.keil:
        parser.error("at least one of --cubeide or --keil is required")
    return args


def norm(path: Path) -> str:
    return os.path.normcase(str(path.resolve()))


def compile_entries(path: Path, fragment: str | None) -> list[dict[str, str]]:
    entries = json.loads(path.resolve().read_text(encoding="utf-8"))
    if fragment:
        needle = fragment.replace("\\", "/").lower()
        entries = [
            entry
            for entry in entries
            if needle in (entry.get("output", "") + " " + entry.get("command", ""))
            .replace("\\", "/")
            .lower()
        ]
    if not entries:
        raise RuntimeError("compile_commands.json contains no matching entries")
    return entries


def command_tokens(command: str, prefix: str) -> list[str]:
    # CMake emits -Ipath and -Dvalue here. Quoted values are retained without
    # relying on POSIX shell parsing, which mishandles Windows drive letters.
    pattern = re.compile(r"(?:^|\s)" + re.escape(prefix) + r'(?P<v>"[^"]+"|\S+)')
    result: list[str] = []
    for match in pattern.finditer(command):
        value = match.group("v")
        if value.startswith('"') and value.endswith('"'):
            value = value[1:-1]
        if value not in result:
            result.append(value)
    return result


def define_name(value: str) -> str:
    """Return the macro name from either NAME or NAME=VALUE."""
    return value.split("=", 1)[0]


def merge_defines(current: list[str], authoritative: list[str]) -> list[str]:
    """Replace CMake-owned definitions by macro name without duplicates."""
    authoritative_names = {define_name(value) for value in authoritative}
    merged = [
        value for value in current if define_name(value) not in authoritative_names
    ]
    for value in authoritative:
        if value not in merged:
            merged.append(value)
    return merged


def model(entries: list[dict[str, str]]) -> tuple[list[Path], list[Path], list[str]]:
    sources: list[Path] = []
    for entry in entries:
        source = Path(entry["file"]).resolve()
        if source.suffix.lower() in SOURCE_SUFFIXES and source not in sources:
            sources.append(source)

    main_entry = next(
        (entry for entry in entries if Path(entry["file"]).name.lower() == "main.c"),
        entries[0],
    )
    command = main_entry.get("command") or " ".join(main_entry.get("arguments", []))
    includes = [Path(value).resolve() for value in command_tokens(command, "-I")]
    defines = command_tokens(command, "-D")
    return sources, includes, defines


def indent_xml(root: ET.Element) -> None:
    ET.indent(root, space="\t")


def write_xml(
    path: Path,
    tree: ET.ElementTree,
    declaration: bool = True,
    processing_instruction: str | None = None,
) -> None:
    indent_xml(tree.getroot())
    payload = ET.tostring(tree.getroot(), encoding="UTF-8", short_empty_elements=True)
    prefix = b'<?xml version="1.0" encoding="UTF-8"?>\n' if declaration else b""
    if processing_instruction:
        prefix += f"<?{processing_instruction}?>".encode("UTF-8")
    path.write_bytes(prefix + payload)


def location_uri(project_dir: Path, source: Path) -> str:
    relative = Path(os.path.relpath(source, project_dir)).as_posix()
    parts = relative.split("/")
    parents = 0
    while parts and parts[0] == "..":
        parents += 1
        parts.pop(0)
    base = "PROJECT_LOC" if parents == 0 else f"PARENT-{parents}-PROJECT_LOC"
    return base + ("/" + "/".join(parts) if parts else "")


def resolve_location_uri(project_dir: Path, uri: str) -> Path | None:
    match = re.fullmatch(r"(?:(?:PARENT-(\d+)-)?PROJECT_LOC)(?:/(.*))?", uri)
    if not match:
        return None
    base = project_dir
    for _ in range(int(match.group(1) or 0)):
        base = base.parent
    return (base / (match.group(2) or "")).resolve()


def cube_link_name(board: Path, source: Path) -> str:
    try:
        rel = source.relative_to(board)
        return "GMP-CMake/Board/" + rel.as_posix()
    except ValueError:
        pass
    try:
        rel = source.relative_to(board.parent)
        return "GMP-CMake/Shared/" + rel.as_posix()
    except ValueError:
        return "GMP-CMake/External/" + source.name


def sync_cubeide(
    board: Path, project_dir: Path, sources: list[Path], includes: list[Path], defines: list[str]
) -> None:
    project_dir = project_dir.resolve()
    project_path = project_dir / ".project"
    cproject_path = project_dir / ".cproject"
    if not project_path.is_file() or not cproject_path.is_file():
        raise RuntimeError(f"CubeIDE metadata is missing under {project_dir}")

    expected = {norm(source) for source in sources}
    project_tree = ET.parse(project_path)
    project_root = project_tree.getroot()
    linked = project_root.find("linkedResources")
    if linked is None:
        linked = ET.SubElement(project_root, "linkedResources")

    existing: dict[str, ET.Element] = {}
    removed = 0
    for link in list(linked.findall("link")):
        uri_node = link.find("locationURI")
        if uri_node is None or not uri_node.text:
            continue
        target = resolve_location_uri(project_dir, uri_node.text)
        if target is None:
            continue
        if not target.exists() or (target.suffix.lower() in SOURCE_SUFFIXES and norm(target) not in expected):
            linked.remove(link)
            removed += 1
            continue
        existing[norm(target)] = link

    added = 0
    for source in sources:
        if norm(source) in existing:
            continue
        link = ET.SubElement(linked, "link")
        ET.SubElement(link, "name").text = cube_link_name(board, source)
        ET.SubElement(link, "type").text = "1"
        ET.SubElement(link, "locationURI").text = location_uri(project_dir, source)
        added += 1
    write_xml(project_path, project_tree)

    cproject_tree = ET.parse(cproject_path)
    cproject_root = cproject_tree.getroot()
    include_values = []
    # Compiler runs in STM32CubeIDE/{Debug,Release}; both have equal depth.
    reference_build_dir = project_dir / "Debug"
    for include in includes:
        value = Path(os.path.relpath(include, reference_build_dir)).as_posix()
        if value not in include_values:
            include_values.append(value)

    for option in cproject_root.iter("option"):
        superclass = option.get("superClass", "")
        if superclass.endswith("c.compiler.option.includepaths"):
            present = {child.get("value") for child in option.findall("listOptionValue")}
            for value in include_values:
                if value not in present:
                    ET.SubElement(option, "listOptionValue", builtIn="false", value=value)
        elif superclass.endswith("c.compiler.option.definedsymbols"):
            children = option.findall("listOptionValue")
            merged = merge_defines(
                [child.get("value") or "" for child in children], defines
            )
            for child in children:
                option.remove(child)
            for value in merged:
                ET.SubElement(option, "listOptionValue", builtIn="false", value=value)
    write_xml(cproject_path, cproject_tree, processing_instruction="fileVersion 4.0.0")
    print(f"CubeIDE synchronized: {added} source(s) added, {removed} stale source(s) removed")


def keil_path(source: Path) -> Path:
    value = str(source)
    marker = os.sep + "portable" + os.sep + "GCC" + os.sep
    if marker in value:
        candidate = Path(value.replace(marker, os.sep + "portable" + os.sep + "RVDS" + os.sep))
        if candidate.exists():
            return candidate.resolve()
    return source.resolve()


def resolve_keil_path(project_dir: Path, value: str) -> Path:
    return (project_dir / value.replace("\\", os.sep)).resolve()


def relative_keil_path(project_dir: Path, value: Path) -> str:
    return os.path.relpath(value, project_dir).replace("/", "\\")


def disabled_keil_group(groups: ET.Element, group_name: str) -> ET.Element:
    """Create a target-local placeholder for a group owned by another target.

    uVision treats the group layout of multi-target projects as shared state.  A
    group that exists only in one target can consequently leak into another
    target when the project is saved or built.  CubeMX avoids that for its CM4
    and CM7 groups by mirroring the group and disabling the non-owning copy.
    """
    template = next(
        (
            group
            for group in groups.findall("Group")
            if group.findtext(".//IncludeInBuild") == "0"
        ),
        None,
    )
    if template is not None:
        placeholder = copy.deepcopy(template)
        placeholder.find("GroupName").text = group_name
        files = placeholder.find("Files")
        if files is None:
            ET.SubElement(placeholder, "Files")
        else:
            for file_node in list(files):
                files.remove(file_node)
    else:
        placeholder = ET.Element("Group")
        ET.SubElement(placeholder, "GroupName").text = group_name
        group_option = ET.SubElement(placeholder, "GroupOption")
        common = ET.SubElement(group_option, "CommonProperty")
        ET.SubElement(common, "IncludeInBuild").text = "0"
        ET.SubElement(placeholder, "Files")
    groups.append(placeholder)
    return placeholder


def sync_keil(
    board: Path,
    uvprojx: Path,
    sources: list[Path],
    includes: list[Path],
    defines: list[str],
    target_name: str | None,
) -> None:
    del board  # Kept in the signature for parity and future board-specific rules.
    uvprojx = uvprojx.resolve()
    project_dir = uvprojx.parent
    expected_sources = [keil_path(source) for source in sources]
    expected = {norm(source) for source in expected_sources}
    tree = ET.parse(uvprojx)
    root = tree.getroot()
    scope = root
    if target_name:
        scope = next(
            (target for target in root.findall(".//Target") if target.findtext("TargetName") == target_name),
            None,
        )
        if scope is None:
            raise RuntimeError(f"Keil target not found: {target_name}")
        common_options = scope.find(".//TargetCommonOption")
        if common_options is not None:
            output_directory = common_options.find("OutputDirectory")
            listing_path = common_options.find("ListingPath")
            if output_directory is not None:
                output_directory.text = f"{target_name}_gmp\\"
            if listing_path is not None:
                listing_path.text = f"./{target_name}_gmp/"

    present: set[str] = set()
    removed = 0
    for group in scope.findall(".//Group"):
        files = group.find("Files")
        if files is None:
            continue
        for file_node in list(files.findall("File")):
            path_node = file_node.find("FilePath")
            if path_node is None or not path_node.text:
                continue
            target = resolve_keil_path(project_dir, path_node.text)
            if target.suffix.lower() in {".s", ".asm"} and not target.exists():
                local_startup = project_dir / (file_node.findtext("FileName") or "")
                if local_startup.is_file():
                    path_node.text = relative_keil_path(project_dir, local_startup)
                    target = local_startup.resolve()
            if target.suffix.lower() in SOURCE_SUFFIXES:
                if not target.exists() or norm(target) not in expected:
                    files.remove(file_node)
                    removed += 1
                    continue
                # CubeMX keeps middleware/peripheral files for both H755 cores
                # and marks the inactive copy at file level.  A source present
                # in the selected CMake target must be explicitly re-enabled.
                include_in_build = file_node.find(".//IncludeInBuild")
                if include_in_build is not None:
                    include_in_build.text = "1"
                present.add(norm(target))

    groups = scope.find(".//Groups")
    if groups is None:
        raise RuntimeError("Keil project has no Groups element")
    group_name = f"GMP-CMake-{target_name}" if target_name else "GMP-CMake"
    sync_group = next(
        (
            group
            for group in groups.findall("Group")
            if group.findtext("GroupName") in {group_name, "GMP-CMake"}
        ),
        None,
    )
    if sync_group is None:
        sync_group = ET.SubElement(groups, "Group")
        ET.SubElement(sync_group, "GroupName").text = group_name
        ET.SubElement(sync_group, "Files")
    else:
        sync_group.find("GroupName").text = group_name
    # This target owns the group.  If it was a mirrored placeholder from a
    # previous synchronization, remove the target-level build exclusion.
    group_option = sync_group.find("GroupOption")
    if group_option is not None:
        sync_group.remove(group_option)
    files = sync_group.find("Files")
    assert files is not None

    added = 0
    for source in expected_sources:
        if norm(source) in present:
            continue
        file_node = ET.SubElement(files, "File")
        ET.SubElement(file_node, "FileName").text = source.name
        ET.SubElement(file_node, "FileType").text = "1"
        ET.SubElement(file_node, "FilePath").text = relative_keil_path(project_dir, source)
        added += 1

    if target_name:
        # Mirror this group into every other target as an empty disabled group,
        # matching the structure CubeMX itself uses for dual-core projects.
        for other_target in root.findall(".//Target"):
            if other_target is scope:
                continue
            other_groups = other_target.find(".//Groups")
            if other_groups is None:
                continue
            placeholder = next(
                (
                    group
                    for group in other_groups.findall("Group")
                    if group.findtext("GroupName") == group_name
                ),
                None,
            )
            if placeholder is None:
                placeholder = disabled_keil_group(other_groups, group_name)
            placeholder_files = placeholder.find("Files")
            if placeholder_files is not None:
                for file_node in list(placeholder_files):
                    placeholder_files.remove(file_node)
            include_in_build = placeholder.find(".//IncludeInBuild")
            if include_in_build is None:
                group_option = placeholder.find("GroupOption")
                if group_option is None:
                    group_option = ET.SubElement(placeholder, "GroupOption")
                common = group_option.find("CommonProperty")
                if common is None:
                    common = ET.SubElement(group_option, "CommonProperty")
                include_in_build = ET.SubElement(common, "IncludeInBuild")
            include_in_build.text = "0"

        # uVision aligns the groups of a multi-target project by position when
        # loading it.  Keep all synchronized groups in the same deterministic
        # order in every target; otherwise two individually-correct XML target
        # sections can still be merged into one active source list at build
        # time.
        for project_target in root.findall(".//Target"):
            project_groups = project_target.find(".//Groups")
            if project_groups is None:
                continue
            synced_groups = [
                group
                for group in project_groups.findall("Group")
                if (group.findtext("GroupName") or "").startswith("GMP-CMake-")
            ]
            for group in synced_groups:
                project_groups.remove(group)
            for group in sorted(synced_groups, key=lambda item: item.findtext("GroupName") or ""):
                project_groups.append(group)

    keil_includes = [keil_path(include) for include in includes]
    for controls in scope.findall(".//Cads/VariousControls"):
        define_node = controls.find("Define")
        include_node = controls.find("IncludePath")
        if define_node is None or include_node is None:
            continue
        current_defines = merge_defines(
            [value for value in (define_node.text or "").split(",") if value],
            defines,
        )
        define_node.text = ",".join(current_defines)

        current_includes = [value for value in (include_node.text or "").split(";") if value]
        current_includes = [
            value
            for value in current_includes
            if "/portable/GCC/" not in value.replace("\\", "/")
        ]
        normalized = {norm(resolve_keil_path(project_dir, value)) for value in current_includes}
        for include in keil_includes:
            if norm(include) not in normalized:
                current_includes.append(relative_keil_path(project_dir, include))
        include_node.text = ";".join(current_includes)

    for scatter_node in scope.findall(".//LDads/ScatterFile"):
        if not scatter_node.text:
            continue
        scatter = resolve_keil_path(project_dir, scatter_node.text)
        if scatter.is_file():
            continue
        local_scatter = project_dir / Path(scatter_node.text.replace("\\", "/")).name
        if local_scatter.is_file():
            scatter_node.text = relative_keil_path(project_dir, local_scatter)

    write_xml(uvprojx, tree)
    print(f"Keil synchronized: {added} source(s) added, {removed} stale source(s) removed")


def main() -> int:
    args = parse_args()
    board = args.board.resolve()
    entries = compile_entries(args.compile_commands, args.target_object_fragment)
    sources, includes, defines = model(entries)
    missing = [str(source) for source in sources if not source.is_file()]
    if missing:
        raise RuntimeError("compile database contains missing sources:\n" + "\n".join(missing))
    if args.cubeide:
        sync_cubeide(board, args.cubeide, sources, includes, defines)
    if args.keil:
        sync_keil(board, args.keil, sources, includes, defines, args.keil_target)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # concise diagnostics for PowerShell/CI callers
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
