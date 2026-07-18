#!/usr/bin/env python3
"""Entry point for the GMP MATLAB Component Builder."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PYTHON_ROOT = ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from gmp_mcb.generator import ComponentGenerator
from gmp_mcb.model import ComponentDefinition, ComponentError


def default_config() -> Path:
    return ROOT / "components" / "continuous_pid.json"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build GMP C components for MATLAB/Simulink.")
    sub = parser.add_subparsers(dest="command", required=True)

    gui = sub.add_parser("gui", help="Open the graphical component editor.")
    gui.add_argument("--config", type=Path, default=default_config())

    validate = sub.add_parser("validate", help="Validate one component definition.")
    validate.add_argument("config", nargs="?", type=Path, default=default_config())

    generate = sub.add_parser("generate", help="Generate S-Function sources and the MATLAB registry.")
    generate.add_argument("configs", nargs="*", type=Path, default=[])
    generate.add_argument("--out", type=Path, default=ROOT / "build")

    args = parser.parse_args(argv)
    try:
        if args.command == "gui":
            from gmp_mcb.gui import run_gui

            return run_gui(args.config)
        if args.command == "validate":
            component = ComponentDefinition.load(args.config)
            print(f"OK: {component.component_id} ({args.config})")
            return 0
        if args.command == "generate":
            configs = args.configs or sorted((ROOT / "components").glob("*.json"))
            generated = ComponentGenerator(ROOT).generate(configs, args.out)
            for path in generated:
                print(path)
            return 0
    except ComponentError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

