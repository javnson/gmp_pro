"""Generate the TI CCS product metadata for all GMP device families.

The metadata is intentionally generated inside each CSP directory.  CCS then
selects a device-specific Product while both Products export GMP_PRO_ROOT for
repository-wide includes.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
from typing import Any


CONFIG_NAME = "GMP_CCS_Product_Info.json"
REQUIRED_ROOT_FILES = ("gmp_core.h", ".gitignore")
REQUIRED_ROOT_DIRS = ("tools/gmp_installer",)


class ProductConfigError(ValueError):
    """Raised when the product registry is incomplete or inconsistent."""


def _load_json(path: Path, *, require_object: bool = False) -> Any:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise ProductConfigError(f"cannot load {path}: {exc}") from exc
    if require_object and not isinstance(value, dict):
        raise ProductConfigError(f"{path} must contain a JSON object")
    return value


def _require(mapping: dict[str, Any], key: str, context: str) -> Any:
    value = mapping.get(key)
    if value is None or value == "" or value == []:
        raise ProductConfigError(f"missing {context}.{key}")
    return value


def validate_registry(registry: dict[str, Any]) -> None:
    if registry.get("schema_version") != 1:
        raise ProductConfigError("schema_version must be 1")

    common = _require(registry, "common", "registry")
    products = _require(registry, "products", "registry")
    if not isinstance(common, dict) or not isinstance(products, dict):
        raise ProductConfigError("common and products must be JSON objects")

    for key in (
        "library_version",
        "library_name",
        "description",
        "documentation_path",
        "image_path",
        "license_path",
        "metadata_version",
        "gmp_root_macro",
    ):
        _require(common, key, "common")

    seen_ids: set[str] = set()
    seen_paths: set[str] = set()
    seen_root_macros: set[str] = set()
    for family, product in products.items():
        if not isinstance(product, dict):
            raise ProductConfigError(f"products.{family} must be a JSON object")
        for key in (
            "product_id",
            "product_path",
            "display_name",
            "description",
            "root_macro_name",
            "csp_root_macro",
            "devices",
            "min_tool_version",
            "compiler_include_paths",
            "dependencies",
            "ccs_compatibility",
        ):
            _require(product, key, f"products.{family}")
        for value, seen, label in (
            (product["product_id"], seen_ids, "product_id"),
            (product["product_path"], seen_paths, "product_path"),
            (product["root_macro_name"], seen_root_macros, "root_macro_name"),
        ):
            if value in seen:
                raise ProductConfigError(f"duplicate {label}: {value}")
            seen.add(value)

        compatibility = product["ccs_compatibility"]
        if not isinstance(compatibility, dict) or not isinstance(
            compatibility.get("minimum_ccs_major"), int
        ):
            raise ProductConfigError(
                f"products.{family}.ccs_compatibility.minimum_ccs_major must be an integer"
            )


def resolve_gmp_root(explicit_root: str | None) -> Path:
    raw_root = explicit_root or os.environ.get("GMP_PRO_LOCATION", "").strip()
    if not raw_root:
        raise ProductConfigError(
            "GMP_PRO_LOCATION is not defined; run a GMP installer or pass --root"
        )
    root = Path(raw_root).expanduser().resolve()
    missing = [name for name in REQUIRED_ROOT_FILES if not (root / name).is_file()]
    missing.extend(name for name in REQUIRED_ROOT_DIRS if not (root / name).is_dir())
    if missing:
        raise ProductConfigError(
            f"{root} is not a GMP repository root; missing: {', '.join(missing)}"
        )
    return root


def _posix(path: Path) -> str:
    return path.resolve().as_posix()


def render_product_files(
    root: Path, common: dict[str, Any], product: dict[str, Any]
) -> dict[Path, Any]:
    product_root = (root / product["product_path"]).resolve()
    root_path = _posix(root)
    csp_path = _posix(product_root)

    product_json = {
        "name": product["product_id"],
        "displayName": product["display_name"],
        "version": common["library_version"],
        "documentationPath": _posix(root / common["documentation_path"]),
        "includePaths": [root_path, csp_path],
        "devices": product["devices"],
        "minToolVersion": product["min_tool_version"],
        "templates": [],
    }

    package_ccs = [
        {
            "rootMacroName": product["root_macro_name"],
            "compilerIncludePath": product["compiler_include_paths"],
            "compilerSymbols": product.get("compiler_symbols", []),
            "linkerSearchPath": product.get("linker_search_paths", []),
            "linkerLibraries": product.get("linker_libraries", []),
            "exports": [
                {
                    "macroName": common["gmp_root_macro"],
                    "location": "../..",
                },
                {
                    "macroName": product["csp_root_macro"],
                    "location": ".",
                },
            ],
        }
    ]

    package_tirex = [
        {
            "id": product["product_id"],
            "name": product["display_name"],
            "version": common["library_version"],
            "type": "software",
            "image": _posix(root / common["image_path"]),
            "license": _posix(root / common["license_path"]),
            "description": product["description"],
            "tags": ["GMP", product["product_id"].rsplit("-", 1)[-1]],
            "dependencies": product["dependencies"],
            "metadataVersion": common["metadata_version"],
        }
    ]

    metadata = product_root / ".metadata"
    return {
        metadata / "product.json": product_json,
        metadata / ".tirex" / "package.ccs.json": package_ccs,
        metadata / ".tirex" / "package.tirex.json": package_tirex,
    }


def _write_json_atomic(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", newline="\n", dir=path.parent, delete=False
    ) as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, path)


def install_products(
    root: Path,
    registry: dict[str, Any],
    families: list[str],
    check_only: bool = False,
) -> list[Path]:
    common = registry["common"]
    products = registry["products"]
    generated: list[Path] = []
    for family in families:
        if family not in products:
            raise ProductConfigError(
                f"unknown product family {family!r}; choose from {', '.join(products)}"
            )
        product_root = root / products[family]["product_path"]
        if not product_root.is_dir():
            raise ProductConfigError(
                f"product directory does not exist for {family}: {product_root}"
            )
        rendered = render_product_files(root, common, products[family])
        for path, value in rendered.items():
            if check_only:
                if not path.is_file():
                    raise ProductConfigError(f"generated metadata is missing: {path}")
                if _load_json(path) != value:
                    raise ProductConfigError(f"generated metadata is stale: {path}")
            else:
                _write_json_atomic(path, value)
            generated.append(path)
    return generated


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", help="GMP repository root (defaults to GMP_PRO_LOCATION)"
    )
    parser.add_argument(
        "--product",
        action="append",
        dest="products",
        help="product family to generate; repeat for several (default: all)",
    )
    parser.add_argument(
        "--check", action="store_true", help="verify generated files without writing"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        root = resolve_gmp_root(args.root)
        registry_path = Path(__file__).resolve().with_name(CONFIG_NAME)
        registry = _load_json(registry_path, require_object=True)
        validate_registry(registry)
        families = args.products or list(registry["products"])
        generated = install_products(root, registry, families, args.check)
    except ProductConfigError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    action = "verified" if args.check else "generated"
    print(f"[OK] {action} {len(generated)} CCS Product metadata files")
    for path in generated:
        print(f"  {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
