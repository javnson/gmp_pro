#!/usr/bin/env python3
"""Generate a CMSIS/Keil software pack from the GMP source-manager registry."""

from __future__ import annotations

import argparse
import glob
import json
import re
import sys
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET


XSI = "http://www.w3.org/2001/XMLSchema-instance"
ET.register_namespace("xs", XSI)
ZIP_TIMESTAMP = (1980, 1, 1, 0, 0, 0)


class PackError(RuntimeError):
    """Raised for an invalid registry, metadata file, or pack input."""


@dataclass(frozen=True)
class Module:
    module_id: str
    key: str
    description: str
    sources: tuple[Path, ...]
    headers: tuple[Path, ...]
    dependencies: tuple[str, ...]

    @property
    def files(self) -> tuple[Path, ...]:
        return tuple(sorted(set(self.sources + self.headers), key=lambda p: p.as_posix()))


@dataclass(frozen=True)
class Metadata:
    vendor: str
    name: str
    version: str
    date: str
    description: str
    url: str
    release_note: str
    schema_version: str


def _text(element: ET.Element | None, default: str = "") -> str:
    if element is None or element.text is None:
        return default
    return element.text.strip()


def load_metadata(path: Path) -> Metadata:
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise PackError(f"cannot read metadata XML {path}: {exc}") from exc
    if root.tag != "package":
        raise PackError(f"metadata root must be <package>: {path}")

    release = root.find("./releases/release")
    if release is None:
        raise PackError("metadata XML needs at least one <release>")
    vendor = _text(root.find("vendor"))
    name = re.sub(r"\s+v?\d+(?:\.\d+)+$", "", _text(root.find("name"))).replace(" ", "")
    version = release.get("version", "").strip()
    if not vendor or not name or not version:
        raise PackError("metadata XML must define vendor, name, and release version")
    if re.search(r"\s", vendor + name):
        raise PackError("CMSIS-Pack vendor and name may not contain whitespace")

    return Metadata(
        vendor=vendor,
        name=name,
        version=version,
        date=release.get("date", "").strip(),
        description=_text(root.find("description"), "GMP Controller Template Library"),
        url=_text(root.find("url")),
        release_note=_text(release, "Generated from the GMP module registry."),
        schema_version=root.get("schemaVersion", "1.7.7"),
    )


def _expand_pattern(pattern: str, repo_root: Path, macros: dict[str, str]) -> list[Path]:
    resolved = pattern
    for name, value in sorted(macros.items(), key=lambda item: len(item[0]), reverse=True):
        resolved = resolved.replace(f"${{{name}}}", value)
    candidate = Path(resolved)
    if not candidate.is_absolute():
        candidate = repo_root / candidate
    return sorted(
        (Path(item).resolve() for item in glob.glob(str(candidate), recursive=True) if Path(item).is_file()),
        key=lambda p: p.as_posix(),
    )


def load_ctl_modules(registry_path: Path, repo_root: Path) -> list[Module]:
    try:
        registry = json.loads(registry_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PackError(f"cannot read registry {registry_path}: {exc}") from exc

    raw_modules = registry.get("modules", {}).get("ctl")
    if not isinstance(raw_modules, dict):
        raise PackError("registry does not contain modules.ctl")
    macros = {str(k): str(v) for k, v in registry.get("macros", {}).items()}
    macros["GMP_PRO_LOCATION"] = str(repo_root.resolve())

    modules: list[Module] = []
    for key in sorted(raw_modules):
        raw = raw_modules[key]
        source_paths: set[Path] = set()
        header_paths: set[Path] = set()
        for pattern in raw.get("src_patterns", []):
            source_paths.update(_expand_pattern(pattern, repo_root, macros))
        for pattern in raw.get("inc_patterns", []):
            header_paths.update(_expand_pattern(pattern, repo_root, macros))

        external = [p for p in source_paths | header_paths if repo_root.resolve() not in p.parents]
        if external:
            names = ", ".join(str(p) for p in sorted(external))
            raise PackError(f"CTL module ctl|{key} contains files outside the repository: {names}")

        dependencies = []
        for dependency in raw.get("depends_on", []):
            dependency = str(dependency)
            if not dependency.startswith(("ctl|", "core|", "csp|", "cctl|", "vctl|")):
                dependency = "ctl|" + dependency
            dependencies.append(dependency)

        modules.append(
            Module(
                module_id="ctl|" + key,
                key=key,
                description=str(raw.get("description", "")).strip(),
                sources=tuple(sorted(source_paths)),
                headers=tuple(sorted(header_paths)),
                dependencies=tuple(sorted(set(dependencies))),
            )
        )
    return modules


def _component_identity(key: str) -> tuple[str, str]:
    parts = key.split("|")
    # Cgroup/Csub are limited to 32 characters by PACK.xsd.  CTL keys usually
    # start with component|<domain>|..., so move the domain into Cgroup.
    group_parts = parts[:2] if parts[0] == "component" and len(parts) > 1 else parts[:1]
    subgroup_parts = parts[len(group_parts) :]
    group = " ".join(" ".join(word.capitalize() for word in part.split("_")) for part in group_parts)
    subgroup = "/".join(" ".join(word.capitalize() for word in part.split("_")) for part in subgroup_parts)
    if len(group) > 32 or len(subgroup) > 32:
        raise PackError(f"module key cannot fit CMSIS-Pack component fields: {key}")
    return group, subgroup or "Core"


def _condition_id(module_id: str) -> str:
    return "GMP." + re.sub(r"[^A-Za-z0-9_.-]", ".", module_id)


def _component_macro(key: str) -> str:
    return "RTE_GMP_CTL_" + re.sub(r"[^A-Za-z0-9]", "_", key).upper()


def _file_category(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix in {".h", ".hpp", ".hh", ".inl", ".inc"}:
        return "header"
    if suffix in {".c", ".cc", ".cpp", ".cxx"}:
        return "source"
    if suffix in {".md", ".txt", ".pdf", ".html", ".htm"}:
        return "doc"
    return "other"


def _add_component(
    parent: ET.Element,
    *,
    group: str,
    subgroup: str,
    version: str,
    description: str,
    files: Iterable[Path],
    repo_root: Path,
    macro: str,
    condition: str | None = None,
    variant: str | None = None,
    configurable: bool = False,
) -> None:
    attrs = {"Cclass": "GMP CTL", "Cgroup": group, "Csub": subgroup, "Cversion": version}
    if condition:
        attrs["condition"] = condition
    if variant:
        attrs["Cvariant"] = variant
    component = ET.SubElement(parent, "component", attrs)
    ET.SubElement(component, "description").text = description
    ET.SubElement(component, "RTE_Components_h").text = f"#define {macro}"
    file_list = ET.SubElement(component, "files")
    for path in sorted(set(files), key=lambda p: p.as_posix()):
        relative = path.resolve().relative_to(repo_root.resolve()).as_posix()
        file_attrs = {"category": _file_category(path), "name": relative}
        if configurable:
            file_attrs["attr"] = "config"
            file_attrs["version"] = version
        ET.SubElement(file_list, "file", file_attrs)


def build_pdsc(metadata: Metadata, modules: list[Module], repo_root: Path) -> ET.ElementTree:
    package = ET.Element(
        "package",
        {
            "schemaVersion": metadata.schema_version,
            f"{{{XSI}}}noNamespaceSchemaLocation": "PACK.xsd",
        },
    )
    ET.SubElement(package, "vendor").text = metadata.vendor
    ET.SubElement(package, "name").text = metadata.name
    ET.SubElement(package, "description").text = metadata.description
    if metadata.url:
        ET.SubElement(package, "url").text = metadata.url
    releases = ET.SubElement(package, "releases")
    release_attrs = {"version": metadata.version}
    if metadata.date:
        release_attrs["date"] = metadata.date
    ET.SubElement(releases, "release", release_attrs).text = metadata.release_note
    keywords = ET.SubElement(package, "keywords")
    for value in ("Controller", "Motor Control", "STM32", "C2000", "Portable"):
        ET.SubElement(keywords, "keyword").text = value

    files_by_id = {module.module_id: module for module in modules if module.files}
    module_by_key = {module.key: module for module in modules}
    portable_module = module_by_key.get("portable|_internal")
    if portable_module is None or not portable_module.files:
        raise PackError("registry must define ctl|portable|_internal with physical files")
    portable_common = list(portable_module.files)
    math_files = [path for module in modules if module.key.startswith("math_block|") for path in module.files]
    portable_common.extend(math_files)
    for path in portable_common:
        if not path.is_file():
            raise PackError(f"portable common file is missing: {path}")

    conditions = ET.SubElement(package, "conditions")
    config_condition = ET.SubElement(conditions, "condition", {"id": "GMP.CTL.Portable.Config"})
    ET.SubElement(config_condition, "description").text = "Select exactly one GMP CTL portable target configuration."
    ET.SubElement(config_condition, "accept", {"Cclass": "GMP CTL", "Cgroup": "Portable", "Csub": "Target", "Cvariant": "STM32"})
    ET.SubElement(config_condition, "accept", {"Cclass": "GMP CTL", "Cgroup": "Portable", "Csub": "Target", "Cvariant": "TI DSP"})

    common_condition = ET.SubElement(conditions, "condition", {"id": "GMP.CTL.Portable.Common"})
    ET.SubElement(common_condition, "description").text = "Require the GMP CTL portable common component."
    ET.SubElement(common_condition, "require", {"Cclass": "GMP CTL", "Cgroup": "Portable", "Csub": "Common"})

    for module in modules:
        if (
            not module.files
            or not module.dependencies
            or module.key.startswith("math_block|")
            or module.key.startswith("portable|")
        ):
            continue
        condition = ET.SubElement(conditions, "condition", {"id": _condition_id(module.module_id)})
        ET.SubElement(condition, "description").text = f"Dependencies for {module.module_id}."
        ET.SubElement(condition, "require", {"Cclass": "GMP CTL", "Cgroup": "Portable", "Csub": "Common"})
        for dependency in module.dependencies:
            dep = files_by_id.get(dependency)
            if dep is None:
                continue
            group, subgroup = _component_identity(dep.key)
            ET.SubElement(condition, "require", {"Cclass": "GMP CTL", "Cgroup": group, "Csub": subgroup})

    components = ET.SubElement(package, "components")
    _add_component(
        components,
        group="Portable",
        subgroup="Common",
        version=metadata.version,
        description="No-CSP CTL type, math, assertion, and time-hook contract.",
        files=portable_common,
        repo_root=repo_root,
        macro="RTE_GMP_CTL_PORTABLE_COMMON",
        condition="GMP.CTL.Portable.Config",
    )
    for target, label in (("stm32", "STM32"), ("ti_dsp", "TI DSP")):
        target_module = module_by_key.get(f"portable|{target}")
        if target_module is None or not target_module.files:
            raise PackError(f"registry must define ctl|portable|{target} with physical files")
        _add_component(
            components,
            group="Portable",
            subgroup="Target",
            version=metadata.version,
            description=f"Copyable no-CSP configuration template for {label}.",
            files=target_module.files,
            repo_root=repo_root,
            macro="RTE_GMP_CTL_PORTABLE_" + re.sub(r"\W", "_", label).upper(),
            variant=label,
            configurable=True,
        )

    for module in modules:
        if not module.files or module.key.startswith(("math_block|", "portable|")):
            continue
        group, subgroup = _component_identity(module.key)
        condition = _condition_id(module.module_id) if module.dependencies else "GMP.CTL.Portable.Common"
        _add_component(
            components,
            group=group,
            subgroup=subgroup,
            version=metadata.version,
            description=module.description or f"GMP CTL module {module.key}",
            files=module.files,
            repo_root=repo_root,
            macro=_component_macro(module.key),
            condition=condition,
        )
    return ET.ElementTree(package)


def _indent_and_write(tree: ET.ElementTree, path: Path) -> None:
    ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True, short_empty_elements=False)


def collect_referenced_files(pdsc_path: Path, repo_root: Path) -> list[Path]:
    tree = ET.parse(pdsc_path)
    files: set[Path] = set()
    for node in tree.findall("./components/component/files/file"):
        name = node.get("name", "")
        candidate = (repo_root / Path(name)).resolve()
        if repo_root.resolve() not in candidate.parents:
            raise PackError(f"PDSC file escapes repository: {name}")
        if not candidate.is_file():
            raise PackError(f"PDSC references a missing file: {name}")
        files.add(candidate)
    for extra in (repo_root / "LICENSE.txt", repo_root / "NOTICE", repo_root / "README.md", repo_root / "README_CN.md"):
        if extra.is_file():
            files.add(extra.resolve())
    return sorted(files, key=lambda p: p.relative_to(repo_root).as_posix())


def write_pack(pack_path: Path, pdsc_path: Path, files: list[Path], repo_root: Path) -> None:
    pack_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=pack_path.parent, suffix=".pack.tmp", delete=False) as handle:
        temporary = Path(handle.name)
    try:
        entries = [(pdsc_path, pdsc_path.name)]
        entries.extend((path, path.relative_to(repo_root).as_posix()) for path in files)
        with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
            for source, archive_name in sorted(entries, key=lambda item: item[1]):
                info = zipfile.ZipInfo(archive_name, ZIP_TIMESTAMP)
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = 0o100644 << 16
                archive.writestr(info, source.read_bytes())
        temporary.replace(pack_path)
    finally:
        temporary.unlink(missing_ok=True)


def generate(repo_root: Path, registry: Path, metadata_xml: Path, output_dir: Path, pdsc_only: bool) -> tuple[Path, Path | None, int, int]:
    repo_root = repo_root.resolve()
    metadata = load_metadata(metadata_xml)
    modules = load_ctl_modules(registry, repo_root)
    tree = build_pdsc(metadata, modules, repo_root)
    pdsc_path = output_dir / f"{metadata.vendor}.{metadata.name}.pdsc"
    _indent_and_write(tree, pdsc_path)
    files = collect_referenced_files(pdsc_path, repo_root)
    component_count = len(tree.findall("./components/component"))
    if pdsc_only:
        return pdsc_path, None, component_count, len(files)
    pack_path = output_dir / f"{metadata.vendor}.{metadata.name}.{metadata.version}.pack"
    write_pack(pack_path, pdsc_path, files, repo_root)
    return pdsc_path, pack_path, component_count, len(files)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_root = script_dir.parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=default_root)
    parser.add_argument("--registry", type=Path)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--pdsc-only", action="store_true")
    parser.add_argument("--check", action="store_true", help="generate in a temporary directory and validate references")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    repo_root = args.repo_root.resolve()
    registry = (args.registry or repo_root / "tools/facilities_generator/src_mgr/gmp_framework_dic.json").resolve()
    metadata_xml = (args.metadata or repo_root / "GMP.GeneralMotorPlatform.xml").resolve()
    try:
        if args.check:
            with tempfile.TemporaryDirectory(prefix="gmp-keil-pack-check-") as temp:
                pdsc, pack, components, files = generate(repo_root, registry, metadata_xml, Path(temp), args.pdsc_only)
                ET.parse(pdsc)
                if pack is not None:
                    with zipfile.ZipFile(pack) as archive:
                        bad = archive.testzip()
                        if bad:
                            raise PackError(f"corrupt archive entry: {bad}")
                print(f"CHECK OK: {components} components, {files} repository files")
            return 0

        output_dir = (args.output_dir or repo_root / "build/keil_pack").resolve()
        pdsc, pack, components, files = generate(repo_root, registry, metadata_xml, output_dir, args.pdsc_only)
        print(f"PDSC: {pdsc}")
        if pack is not None:
            print(f"PACK: {pack}")
        print(f"Generated {components} components from {files} repository files.")
        return 0
    except PackError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
