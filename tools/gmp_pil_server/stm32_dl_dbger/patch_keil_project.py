"""Restore project-local GMP groups after STM32CubeMX regeneration."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT / "MDK-ARM" / "stm32_dl_dbger.uvprojx"
LOCAL_INCLUDES = (
    "../user",
    "../xplt",
    "../gmp_src_mgr/gmp_inc",
    "../gmp_src_mgr/gmp_inc/csp/stm32",
)
OBSOLETE_INCLUDES = ("../App", "../Config")
MANAGED_GROUPS = ("Application/GMP User", "Application/GMP Platform", "GMP/Generated Sources")


def add_source(group: ET.Element, name: str, path: str) -> None:
    """Append one C source entry to a Keil group."""
    files = group.find("Files")
    if files is None:
        files = ET.SubElement(group, "Files")
    entry = ET.SubElement(files, "File")
    ET.SubElement(entry, "FileName").text = name
    ET.SubElement(entry, "FileType").text = "1"
    ET.SubElement(entry, "FilePath").text = path


def add_group(groups: ET.Element, name: str, sources: list[tuple[str, str]]) -> None:
    """Create one named source group from project-relative paths."""
    group = ET.SubElement(groups, "Group")
    ET.SubElement(group, "GroupName").text = name
    ET.SubElement(group, "Files")
    for file_name, file_path in sources:
        add_source(group, file_name, file_path)


def patch_project() -> None:
    """Add only project-local include paths and generated GMP sources."""
    tree = ET.parse(PROJECT)
    root = tree.getroot()

    include_path = root.find(".//TargetArmAds/Cads/VariousControls/IncludePath")
    if include_path is None:
        raise RuntimeError("CubeMX Keil include path was not found")
    paths = [path for path in (include_path.text or "").split(";") if path]
    paths = [path for path in paths if path not in OBSOLETE_INCLUDES]
    for path in LOCAL_INCLUDES:
        if path not in paths:
            paths.append(path)
    include_path.text = ";".join(paths)

    groups = root.find(".//Groups")
    if groups is None:
        raise RuntimeError("CubeMX Keil group list was not found")
    for group in list(groups):
        name = group.findtext("GroupName", default="")
        if name in MANAGED_GROUPS or name == "Application/GMP Data Link":
            groups.remove(group)

    add_group(
        groups,
        "Application/GMP User",
        [("user_main.c", "../user/user_main.c"), ("ctl_main.c", "../user/ctl_main.c")],
    )
    add_group(
        groups,
        "Application/GMP Platform",
        [("xplt.peripheral.c", "../xplt/xplt.peripheral.c")],
    )
    generated_sources = [
        (source.name, f"../gmp_src_mgr/gmp_src/{source.name}")
        for source in sorted((ROOT / "gmp_src_mgr" / "gmp_src").glob("*.c"))
    ]
    if not generated_sources:
        raise RuntimeError("Generate gmp_src_mgr/gmp_src before patching the Keil project")
    add_group(groups, "GMP/Generated Sources", generated_sources)

    ET.indent(tree, space="  ")
    tree.write(PROJECT, encoding="UTF-8", xml_declaration=True)


if __name__ == "__main__":
    patch_project()
    print(f"Patched {PROJECT}")
