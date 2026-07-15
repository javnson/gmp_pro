#!/usr/bin/env python3
"""SDPE v2 command line interface."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from sdpe_v2.generator import HeaderGenerator
from sdpe_v2.library import SDPELibrary
from sdpe_v2.model import SDPEError
from sdpe_v2.util import macro_name


ROOT = Path(__file__).resolve().parent
DEFAULT_SETTINGS = ROOT / "sdpe_settings.json"


def read_settings(path: str | Path | None) -> dict:
    settings_path = Path(path) if path else DEFAULT_SETTINGS
    if not settings_path.exists():
        return {}
    return json.loads(settings_path.read_text(encoding="utf-8"))


def expand_path(value: str | Path, base: Path | None = None) -> Path:
    text = os.path.expandvars(str(value))
    path = Path(text)
    if path.is_absolute():
        return path
    return ((base or ROOT) / path).resolve()


def expand_path_list(values: list[str], base: Path | None = None) -> list[Path]:
    return [expand_path(value, base) for value in values]


def load_library(args) -> SDPELibrary:
    settings = read_settings(getattr(args, "settings", None))
    explicit_library = bool(getattr(args, "library", None))
    library_root = expand_path(args.library, Path.cwd()) if explicit_library else expand_path(settings.get("library_root", "."), ROOT)
    explicit_schema_dirs = getattr(args, "schema_dirs", None)
    explicit_entity_dirs = getattr(args, "entity_dirs", None)
    schema_values = explicit_schema_dirs if explicit_schema_dirs is not None else ([] if explicit_library else settings.get("schema_dirs", []))
    entity_values = explicit_entity_dirs if explicit_entity_dirs is not None else ([] if explicit_library else settings.get("entity_dirs", []))
    schema_base = Path.cwd() if explicit_schema_dirs is not None else ROOT
    entity_base = Path.cwd() if explicit_entity_dirs is not None else ROOT
    schema_dirs = expand_path_list(schema_values, schema_base) or None
    entity_dirs = expand_path_list(entity_values, entity_base) or None
    return SDPELibrary(library_root, schema_dirs=schema_dirs, entity_dirs=entity_dirs).load()


def generation_value(args, settings: dict, section: str, key: str, default: str) -> str:
    arg_value = getattr(args, key, None)
    if arg_value not in (None, ""):
        return arg_value
    if getattr(args, "library", ""):
        return default
    return str(settings.get(section, {}).get(key, default))


def build_generator(args, section: str = "global_generation") -> HeaderGenerator:
    settings = read_settings(getattr(args, "settings", None))
    lib = load_library(args)
    out_value = generation_value(args, settings, section, "out", "build")
    include_prefix = generation_value(args, settings, section, "include_prefix", "ctl/component")
    include_mode = generation_value(args, settings, section, "include_mode", "prefixed")
    project_subdir = generation_value(args, settings, section, "project_subdir", "project")
    system_cfg = settings.get("system_hardware", {})
    system_entity_dirs = expand_path_list(system_cfg.get("entity_dirs", []), ROOT)
    system_out_dir = expand_path(system_cfg["out"], ROOT) if system_cfg.get("out") else None
    system_include_prefix = str(system_cfg.get("include_prefix", "ctl"))
    return HeaderGenerator(
        lib,
        expand_path(out_value, Path.cwd()),
        include_prefix,
        include_mode,
        project_subdir,
        system_entity_dirs,
        system_out_dir,
        system_include_prefix,
    )


def print_generated(items) -> None:
    for item in items:
        flag = "updated" if item.changed else "unchanged"
        print(f"{flag}: {item.path}")


def cmd_validate(args) -> int:
    lib = load_library(args)
    warnings = lib.validate()
    print(f"schemas: {len(lib.schemas)}")
    print(f"entities: {len(lib.entity_files)}")
    for warning in warnings:
        print(f"warning: {warning}")
    print("validation passed")
    return 0


def cmd_generate_entity(args) -> int:
    gen = build_generator(args)
    print_generated(gen.generate_entity_tree(args.entity))
    return 0


def cmd_generate_all(args) -> int:
    gen = build_generator(args)
    print_generated(gen.generate_all_entities())
    return 0


def cmd_generate_project(args) -> int:
    gen = build_generator(args)
    print_generated(gen.generate_project(Path(args.project)))
    return 0


def cmd_generate_project_matlab(args) -> int:
    gen = build_generator(args)
    print_generated([gen.generate_project_matlab_script(Path(args.project))])
    return 0


def cmd_generate_global_hardware(args) -> int:
    gen = build_generator(args, "global_generation")
    print_generated(gen.generate_all_entities())
    return 0


def cmd_generate_project_local(args) -> int:
    gen, project_path = build_local_project_generator(args)
    print_generated(gen.generate_project(project_path))
    return 0


def cmd_generate_project_matlab_local(args) -> int:
    gen, project_path = build_local_project_generator(args)
    print_generated([gen.generate_project_matlab_script(project_path)])
    return 0


def build_local_project_generator(args) -> tuple[HeaderGenerator, Path]:
    settings = read_settings(getattr(args, "settings", None))
    project_path = Path(args.project).resolve()
    project_dir = Path(args.project_dir).resolve() if args.project_dir else project_path.parent
    local_cfg = settings.get("local_generation", {})
    out_value = args.out or local_cfg.get("out", ".")
    out_dir = expand_path(out_value, project_dir)
    include_prefix = args.include_prefix if args.include_prefix is not None else local_cfg.get("include_prefix", "")
    include_mode = args.include_mode or local_cfg.get("include_mode", "relative")
    project_subdir = args.project_subdir if args.project_subdir is not None else local_cfg.get("project_subdir", "")
    lib = load_library(args)
    system_cfg = settings.get("system_hardware", {})
    system_entity_dirs = expand_path_list(system_cfg.get("entity_dirs", []), ROOT)
    system_out_dir = expand_path(system_cfg["out"], ROOT) if system_cfg.get("out") else None
    system_include_prefix = str(system_cfg.get("include_prefix", "ctl"))
    gen = HeaderGenerator(
        lib,
        out_dir,
        include_prefix,
        include_mode,
        project_subdir,
        system_entity_dirs,
        system_out_dir,
        system_include_prefix,
    )
    return gen, project_path


def default_requirement(project_dir: Path, project_id: str, suite: str) -> dict:
    return {
        "id": project_id,
        "macro_prefix": macro_name(project_id),
        "display_name": project_id,
        "suite": suite,
        "version": "0.1.0",
        "output_header": f"{project_id}_sdpe_bindings.h",
        "hardware": [],
        "requirements": [],
        "feature_macros": [],
        "option_macros": [],
        "code_sections": {
            "after_extern_open": "/* Add project-specific includes here. */",
            "before_footer": "/* Add project-specific tail macros here. */",
        },
    }


def write_text_if_needed(path: Path, text: str, force: bool = False) -> bool:
    if path.exists() and not force:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")
    return True


def write_json_if_needed(path: Path, data: dict, force: bool = False) -> bool:
    return write_text_if_needed(path, json.dumps(data, indent=2, ensure_ascii=False) + "\n", force)


def project_mgr_files(project_id: str, suite: str) -> dict[str, str]:
    toolchain_names = (
        "sdpe_settings.bat",
        "sdpe_edit.bat",
        "sdpe_generate.bat",
        "sdpe_validate.bat",
    )
    files = {
        name: (ROOT / "sdpe_mgr" / name).read_text(encoding="utf-8-sig")
        for name in toolchain_names
    }
    files["README_SDPE_MGR.md"] = f"""# SDPE Project Manager

This folder is the project-local SDPE manager.

- `sdpe_requirement.json`: project requirement and hardware bindings.
- `sdpe_settings.bat`: project-local paths and output settings.
- `sdpe_edit.bat`: open the SDPE Project Requirement GUI.
- `sdpe_generate.bat`: generate SDPE headers for this project.
- `sdpe_validate.bat`: validate the central SDPE library and read this requirement file.

The scripts call `%GMP_PRO_LOCATION%\\tools\\SDPE_v2` and read `%GMP_PRO_LOCATION%\\tools\\SDPE_v2\\sdpe_settings.json`, so set `GMP_PRO_LOCATION` to the GMP repository root before using them.

Default generated output:

```text
sdpe_mgr\\
  <project_settings_header>.h
  hardware_preset\\
```

Project-local generated headers use relative include paths, so the generated project header can include the generated hardware headers without depending on the global `ctl\\hardware_preset` output.

Project id: `{project_id}`
Suite: `{suite}`
"""
    return files


def cmd_inspect_project(args) -> int:
    path = Path(args.project)
    data = json.loads(path.read_text(encoding="utf-8"))
    print(f"project: {data.get('id', path.stem)}")
    print(f"suite: {data.get('suite', '')}")
    print(f"hardware: {len(data.get('hardware', []))}")
    print(f"requirements: {len(data.get('requirements', []))}")
    print(f"output_header: {data.get('output_header', '')}")
    return 0


def cmd_deploy_project(args) -> int:
    project_dir = Path(args.project_dir).resolve()
    mgr_dir = project_dir / args.manager_name
    project_id = args.project_id or project_dir.name
    suite = args.suite or project_dir.parent.parent.name if project_dir.parent.parent else ""
    mgr_dir.mkdir(parents=True, exist_ok=True)

    created: list[Path] = []
    skipped: list[Path] = []
    requirement = default_requirement(project_dir, project_id, suite)
    requirement_path = mgr_dir / "sdpe_requirement.json"
    if write_json_if_needed(requirement_path, requirement, args.force):
        created.append(requirement_path)
    else:
        skipped.append(requirement_path)

    for name, content in project_mgr_files(project_id, suite).items():
        path = mgr_dir / name
        if write_text_if_needed(path, content, args.force):
            created.append(path)
        else:
            skipped.append(path)

    print(f"sdpe_mgr: {mgr_dir}")
    for path in created:
        print(f"created: {path}")
    for path in skipped:
        print(f"kept: {path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SDPE v2 hardware preset generator")
    parser.add_argument("--settings", default=str(DEFAULT_SETTINGS), help="SDPE settings JSON file")
    parser.add_argument("--library", default="", help="Library root containing schemas/ and entities/. Overrides settings JSON.")
    parser.add_argument("--schema-dir", dest="schema_dirs", action="append", help="Additional/override schema directory")
    parser.add_argument("--entity-dir", dest="entity_dirs", action="append", help="Additional/override entity directory")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("validate", help="Validate schemas and entities")
    p.set_defaults(func=cmd_validate)

    p = sub.add_parser("generate-entity", help="Generate one entity and its referenced children")
    p.add_argument("entity")
    p.add_argument("--out", default="")
    p.add_argument("--include-prefix", default=None)
    p.add_argument("--include-mode", default="")
    p.add_argument("--project-subdir", default=None)
    p.set_defaults(func=cmd_generate_entity)

    p = sub.add_parser("generate-all", help="Generate all entities")
    p.add_argument("--out", default="")
    p.add_argument("--include-prefix", default=None)
    p.add_argument("--include-mode", default="")
    p.add_argument("--project-subdir", default=None)
    p.set_defaults(func=cmd_generate_all)

    p = sub.add_parser("generate-project", help="Generate project binding header")
    p.add_argument("project")
    p.add_argument("--out", default="")
    p.add_argument("--include-prefix", default=None)
    p.add_argument("--include-mode", default="")
    p.add_argument("--project-subdir", default=None)
    p.set_defaults(func=cmd_generate_project)

    p = sub.add_parser("generate-project-matlab", help="Generate MATLAB initialization script for a project")
    p.add_argument("project")
    p.add_argument("--out", default="")
    p.add_argument("--include-prefix", default=None)
    p.add_argument("--include-mode", default="")
    p.add_argument("--project-subdir", default=None)
    p.set_defaults(func=cmd_generate_project_matlab)

    p = sub.add_parser("generate-global-hardware", help="Generate all global hardware preset headers from sdpe_src")
    p.add_argument("--out", default="")
    p.add_argument("--include-prefix", default=None)
    p.add_argument("--include-mode", default="")
    p.add_argument("--project-subdir", default=None)
    p.set_defaults(func=cmd_generate_global_hardware)

    p = sub.add_parser("generate-project-local", help="Generate a project header and all required hardware headers locally")
    p.add_argument("project")
    p.add_argument("--project-dir", default="")
    p.add_argument("--out", default="")
    p.add_argument("--include-prefix", default=None)
    p.add_argument("--include-mode", default="")
    p.add_argument("--project-subdir", default=None)
    p.set_defaults(func=cmd_generate_project_local)

    p = sub.add_parser("generate-project-matlab-local", help="Generate a project MATLAB initialization script locally")
    p.add_argument("project")
    p.add_argument("--project-dir", default="")
    p.add_argument("--out", default="")
    p.add_argument("--include-prefix", default=None)
    p.add_argument("--include-mode", default="")
    p.add_argument("--project-subdir", default=None)
    p.set_defaults(func=cmd_generate_project_matlab_local)

    p = sub.add_parser("inspect-project", help="Read and summarize one project requirement file")
    p.add_argument("project")
    p.set_defaults(func=cmd_inspect_project)

    p = sub.add_parser("deploy-project", help="Create a project-local sdpe_mgr folder")
    p.add_argument("project_dir", help="Project directory that should receive sdpe_mgr")
    p.add_argument("--manager-name", default="sdpe_mgr")
    p.add_argument("--project-id", default="")
    p.add_argument("--suite", default="")
    p.add_argument("--force", action="store_true", help="Overwrite existing sdpe_mgr files")
    p.set_defaults(func=cmd_deploy_project)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except SDPEError as exc:
        print(f"sdpe error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

