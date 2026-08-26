"""Compiled-topology manifest loading and instance pin configuration."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any, Mapping

from cctl_studio import StudioError


SCHEMA_NAME = "gmp.cctl.compiled_topology"
SCHEMA_VERSION = 1
TOPOLOGY_NODE_TYPE = "system.compiled_topology"
_FIELD_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\Z")


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise StudioError(f"compiled topology {label} must be an object")
    return value


def _artifact_path(root: Path, artifact: Mapping[str, Any], label: str) -> Path:
    filename = artifact.get("path")
    if not isinstance(filename, str) or not filename.strip():
        raise StudioError(f"compiled topology {label} path is missing")
    relative = Path(filename)
    if relative.is_absolute():
        raise StudioError(f"compiled topology {label} path must be relative")
    resolved = (root / relative).resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as exc:
        raise StudioError(f"compiled topology {label} escapes the bundle directory") from exc
    return resolved


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_topology_manifest(
    document: Mapping[str, Any],
    manifest_path: Path | None = None,
    verify_artifacts: bool = False,
) -> dict[str, Any]:
    """Validate a manifest and optionally verify its HPP/archive integrity."""
    schema = _require_mapping(document.get("schema"), "schema")
    if schema.get("name") != SCHEMA_NAME or schema.get("version") != SCHEMA_VERSION:
        raise StudioError(
            f"unsupported compiled topology schema {schema.get('name')!r} "
            f"version {schema.get('version')!r}"
        )
    topology = _require_mapping(document.get("topology"), "topology")
    if not isinstance(topology.get("name"), str) or not topology["name"].strip():
        raise StudioError("compiled topology name is missing")
    class_name = topology.get("class_name")
    if not isinstance(class_name, str) or not _FIELD_RE.fullmatch(class_name):
        raise StudioError("compiled topology class_name is invalid")
    interface = _require_mapping(document.get("interface"), "interface")
    seen_fields: set[str] = set()
    for direction in ("inputs", "outputs"):
        ports = interface.get(direction)
        if not isinstance(ports, list):
            raise StudioError(f"compiled topology interface.{direction} must be an array")
        for port in ports:
            value = _require_mapping(port, f"interface.{direction} port")
            field = value.get("field")
            if not isinstance(field, str) or not _FIELD_RE.fullmatch(field):
                raise StudioError(f"compiled topology port field {field!r} is invalid")
            key = f"{direction}:{field}".upper()
            if key in seen_fields:
                raise StudioError(f"duplicate compiled topology port field {field!r}")
            seen_fields.add(key)
            if value.get("data_type") not in {"double", "uint32_t"}:
                raise StudioError(
                    f"compiled topology port {field!r} has unsupported data_type"
                )
            if not isinstance(value.get("name"), str) or not value["name"]:
                raise StudioError(f"compiled topology port {field!r} has no name")

    artifacts = _require_mapping(document.get("artifacts"), "artifacts")
    header = _require_mapping(artifacts.get("header"), "header artifact")
    archive_value = artifacts.get("archive")
    if archive_value is not None:
        _require_mapping(archive_value, "archive artifact")
    cpp = _require_mapping(document.get("cpp"), "cpp interface")
    if cpp.get("class_name") != class_name:
        raise StudioError("compiled topology cpp class_name does not match topology")
    if cpp.get("include") != header.get("path"):
        raise StudioError("compiled topology C++ include does not match header artifact")
    constructor = _require_mapping(cpp.get("constructor"), "cpp constructor")
    archive_argument = constructor.get("archive_path_argument")
    if not isinstance(archive_argument, bool):
        raise StudioError("compiled topology archive_path_argument must be boolean")
    if archive_value is not None:
        archive = _require_mapping(archive_value, "archive artifact")
        if not archive_argument or constructor.get("default_archive") != archive.get("path"):
            raise StudioError("compiled topology constructor does not match archive artifact")
    elif archive_argument:
        raise StudioError("compiled topology without archive cannot require an archive path")
    methods = _require_mapping(cpp.get("methods"), "cpp methods")
    for method in ("normal_step", "short_step", "reset"):
        if not isinstance(methods.get(method), str) or not methods[method]:
            raise StudioError(f"compiled topology cpp method {method!r} is missing")
    solver = _require_mapping(document.get("solver"), "solver")
    backend = solver.get("matrix_backend")
    if backend not in {"eigen", "fixed", "fixed_point"}:
        raise StudioError(
            "compiled topology matrix_backend must be eigen, fixed, or fixed_point"
        )
    if backend == "eigen" and archive_value is None:
        raise StudioError("Eigen compiled topology requires an archive artifact")
    if backend in {"fixed", "fixed_point"} and archive_value is not None:
        raise StudioError(
            f"{backend} compiled topology must embed matrices in its header"
        )
    for label, artifact in (("header", header), ("archive", archive_value)):
        if artifact is None:
            continue
        artifact = _require_mapping(artifact, f"{label} artifact")
        if not isinstance(artifact.get("size_bytes"), int) or artifact["size_bytes"] < 0:
            raise StudioError(f"compiled topology {label} size is invalid")
        digest = artifact.get("sha256")
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", digest):
            raise StudioError(f"compiled topology {label} SHA-256 is invalid")
    if verify_artifacts:
        if manifest_path is None:
            raise StudioError("manifest path is required to verify topology artifacts")
        root = manifest_path.resolve().parent
        for label, artifact in (("header", header), ("archive", archive_value)):
            if artifact is None:
                continue
            artifact = _require_mapping(artifact, f"{label} artifact")
            path = _artifact_path(root, artifact, label)
            if not path.is_file():
                raise StudioError(f"compiled topology {label} does not exist: {path}")
            expected_size = artifact.get("size_bytes")
            if not isinstance(expected_size, int) or path.stat().st_size != expected_size:
                raise StudioError(f"compiled topology {label} size does not match manifest")
            expected_hash = artifact.get("sha256")
            if not isinstance(expected_hash, str) or _sha256(path) != expected_hash.lower():
                raise StudioError(f"compiled topology {label} SHA-256 does not match manifest")
    return copy.deepcopy(dict(document))


def load_topology_manifest(path: str | Path, verify_artifacts: bool = True) -> dict[str, Any]:
    manifest_path = Path(path).resolve()
    try:
        document = json.loads(manifest_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise StudioError(f"cannot read compiled topology {manifest_path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise StudioError(f"invalid compiled topology JSON {manifest_path}: {exc}") from exc
    if not isinstance(document, Mapping):
        raise StudioError("compiled topology document must be an object")
    return validate_topology_manifest(document, manifest_path, verify_artifacts)


def binding_key(direction: str, field: str, setting: str) -> str:
    return f"{direction}.{field}.{setting}"


def default_bindings(manifest: Mapping[str, Any]) -> dict[str, Any]:
    values: dict[str, Any] = {}
    interface = manifest["interface"]
    for port in interface["inputs"]:
        values[binding_key("input", port["field"], "mode")] = "port"
        values[binding_key("input", port["field"], "value")] = port.get("default", 0)
    for port in interface["outputs"]:
        values[binding_key("output", port["field"], "mode")] = "port"
    return values


def normalize_bindings(
    manifest: Mapping[str, Any], parameters: Mapping[str, Any]
) -> dict[str, Any]:
    expected = default_bindings(manifest)
    unknown = set(parameters) - set(expected)
    if unknown:
        raise StudioError(f"unknown compiled topology settings: {', '.join(sorted(unknown))}")
    values = dict(expected)
    values.update(parameters)
    for port in manifest["interface"]["inputs"]:
        field = port["field"]
        mode_key = binding_key("input", field, "mode")
        value_key = binding_key("input", field, "value")
        mode = str(values[mode_key]).strip().lower()
        if mode not in {"port", "constant"}:
            raise StudioError(f"{port['name']}: input mode must be port or constant")
        values[mode_key] = mode
        raw = values[value_key]
        try:
            if port["data_type"] == "uint32_t":
                number = int(str(raw), 0) if isinstance(raw, str) else int(raw)
                if number < 0 or number > 0xFFFFFFFF:
                    raise ValueError
                values[value_key] = number
            else:
                number = float(raw)
                if not math.isfinite(number):
                    raise ValueError
                values[value_key] = number
        except (TypeError, ValueError) as exc:
            raise StudioError(f"{port['name']}: invalid fixed value {raw!r}") from exc
    for port in manifest["interface"]["outputs"]:
        key = binding_key("output", port["field"], "mode")
        mode = str(values[key]).strip().lower()
        if mode not in {"port", "hidden"}:
            raise StudioError(f"{port['name']}: output mode must be port or hidden")
        values[key] = mode
    return values


def exposed_port_documents(
    manifest: Mapping[str, Any], parameters: Mapping[str, Any]
) -> list[dict[str, str]]:
    values = normalize_bindings(manifest, parameters)
    result: list[dict[str, str]] = []
    for port in manifest["interface"]["inputs"]:
        if values[binding_key("input", port["field"], "mode")] == "port":
            result.append(
                {
                    "port_id": f"in_{port['field']}",
                    "label": str(port["name"]),
                    "direction": "input",
                    "domain": "numeric",
                }
            )
    for port in manifest["interface"]["outputs"]:
        if values[binding_key("output", port["field"], "mode")] == "port":
            result.append(
                {
                    "port_id": f"out_{port['field']}",
                    "label": str(port["name"]),
                    "direction": "output",
                    "domain": "numeric",
                }
            )
    return result
