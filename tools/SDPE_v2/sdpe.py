#!/usr/bin/env python3
"""SDPE v2 command line interface."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from sdpe_v2.generator import HeaderGenerator
from sdpe_v2.library import SDPELibrary
from sdpe_v2.model import SDPEError


def load_library(path: str) -> SDPELibrary:
    return SDPELibrary(Path(path)).load()


def print_generated(items) -> None:
    for item in items:
        flag = "updated" if item.changed else "unchanged"
        print(f"{flag}: {item.path}")


def cmd_validate(args) -> int:
    lib = load_library(args.library)
    warnings = lib.validate()
    print(f"schemas: {len(lib.schemas)}")
    print(f"entities: {len(lib.entity_files)}")
    for warning in warnings:
        print(f"warning: {warning}")
    print("validation passed")
    return 0


def cmd_generate_entity(args) -> int:
    lib = load_library(args.library)
    gen = HeaderGenerator(lib, Path(args.out), args.include_prefix)
    print_generated(gen.generate_entity_tree(args.entity))
    return 0


def cmd_generate_all(args) -> int:
    lib = load_library(args.library)
    gen = HeaderGenerator(lib, Path(args.out), args.include_prefix)
    print_generated(gen.generate_all_entities())
    return 0


def cmd_generate_project(args) -> int:
    lib = load_library(args.library)
    gen = HeaderGenerator(lib, Path(args.out), args.include_prefix)
    print_generated(gen.generate_project(Path(args.project)))
    return 0


def default_requirement(project_dir: Path, project_id: str, suite: str) -> dict:
    return {
        "id": project_id,
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
    return {
        "sdpe_settings.bat": r'''@echo off
rem SDPE project manager settings.
rem This file is intentionally project-local. Adjust paths for your project branch.

if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the GMP repository root directory.
    exit /b 1
)

set "SDPE_LIBRARY=%GMP_PRO_LOCATION%\tools\SDPE_v2\examples"
set "SDPE_REQUIREMENT=%~dp0sdpe_requirement.json"
set "SDPE_OUT=%~dp0..\xplt\sdpe_generated"
set "SDPE_INCLUDE_PREFIX=ctl/component"
exit /b 0
''',
        "sdpe_edit.bat": r'''@echo off
setlocal EnableDelayedExpansion

title SDPE Project Requirement Editor
cd /d "%~dp0"
call "%~dp0sdpe_settings.bat"
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

echo =======================================================
echo [SDPE] Starting project requirement GUI...
echo =======================================================
echo [SDPE] Library    : %SDPE_LIBRARY%
echo [SDPE] Requirement: %SDPE_REQUIREMENT%
echo [SDPE] Output     : %SDPE_OUT%

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\gui_pyqt\sdpe_gui.py" --library "%SDPE_LIBRARY%" --mode project --projects "%SDPE_REQUIREMENT%" --out "%SDPE_OUT%"
if errorlevel 1 (
    echo.
    echo [ERROR] SDPE GUI exited abnormally. Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)
exit /b 0
''',
        "sdpe_generate.bat": r'''@echo off
setlocal EnableDelayedExpansion

title SDPE Project Header Generator
cd /d "%~dp0"
call "%~dp0sdpe_settings.bat"
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

echo =======================================================
echo [SDPE] Generating project SDPE headers...
echo =======================================================
echo [SDPE] Library    : %SDPE_LIBRARY%
echo [SDPE] Requirement: %SDPE_REQUIREMENT%
echo [SDPE] Output     : %SDPE_OUT%

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --library "%SDPE_LIBRARY%" validate
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --library "%SDPE_LIBRARY%" generate-project "%SDPE_REQUIREMENT%" --out "%SDPE_OUT%" --include-prefix "%SDPE_INCLUDE_PREFIX%"
if errorlevel 1 (
    echo.
    echo [ERROR] SDPE project generation failed. Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo =======================================================
echo [SUCCESS] SDPE project headers generated successfully.
echo =======================================================
exit /b 0
''',
        "sdpe_validate.bat": r'''@echo off
setlocal EnableDelayedExpansion

title SDPE Project Validator
cd /d "%~dp0"
call "%~dp0sdpe_settings.bat"
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

echo =======================================================
echo [SDPE] Validating SDPE library and project requirement...
echo =======================================================

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --library "%SDPE_LIBRARY%" validate
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --library "%SDPE_LIBRARY%" inspect-project "%SDPE_REQUIREMENT%"
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

echo.
echo =======================================================
echo [SUCCESS] SDPE project requirement is readable.
echo =======================================================
exit /b 0
''',
        "README_SDPE_MGR.md": f"""# SDPE Project Manager

This folder is the project-local SDPE manager.

- `sdpe_requirement.json`: project requirement and hardware bindings.
- `sdpe_settings.bat`: project-local paths and output settings.
- `sdpe_edit.bat`: open the SDPE Project Requirement GUI.
- `sdpe_generate.bat`: generate SDPE headers for this project.
- `sdpe_validate.bat`: validate the central SDPE library and read this requirement file.

The scripts call `%GMP_PRO_LOCATION%\\tools\\SDPE_v2`, so set `GMP_PRO_LOCATION` to the GMP repository root before using them.

Default generated output:

```text
..\\xplt\\sdpe_generated
```

Project id: `{project_id}`
Suite: `{suite}`
""",
    }


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
    parser.add_argument("--library", default="examples", help="Library root containing schemas/ and entities/")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("validate", help="Validate schemas and entities")
    p.set_defaults(func=cmd_validate)

    p = sub.add_parser("generate-entity", help="Generate one entity and its referenced children")
    p.add_argument("entity")
    p.add_argument("--out", default="build")
    p.add_argument("--include-prefix", default="ctl/component")
    p.set_defaults(func=cmd_generate_entity)

    p = sub.add_parser("generate-all", help="Generate all entities")
    p.add_argument("--out", default="build")
    p.add_argument("--include-prefix", default="ctl/component")
    p.set_defaults(func=cmd_generate_all)

    p = sub.add_parser("generate-project", help="Generate project binding header")
    p.add_argument("project")
    p.add_argument("--out", default="build")
    p.add_argument("--include-prefix", default="ctl/component")
    p.set_defaults(func=cmd_generate_project)

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

