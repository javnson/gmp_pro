#!/usr/bin/env python3
"""SDPE v2 command line interface."""

from __future__ import annotations

import argparse
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
