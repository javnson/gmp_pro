#!/usr/bin/env python3
"""Data-driven Xyce netlist generator used by the CCTL Studio prototype."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from string import Template
from typing import Any, Iterable, Mapping, Sequence


TOOL_ROOT = Path(__file__).resolve().parent
BUILTIN_LIBRARY = TOOL_ROOT / "components"
SUPPORTED_SCHEMA_VERSION = 1
IDENTIFIER_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_.$-]*$")
NODE_RE = re.compile(r"^[A-Za-z0-9_.$:+-]+$")
SPICE_SCALAR_RE = re.compile(
    r"^[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+|[A-Za-z]+)?$"
)


class StudioError(RuntimeError):
    """Expected configuration or execution error."""


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise StudioError(f"cannot read {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise StudioError(f"invalid JSON in {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise StudioError(f"top level of {path} must be a JSON object")
    return value


def _require_text(value: Any, description: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise StudioError(f"{description} must be a non-empty string")
    if "\n" in value or "\r" in value:
        raise StudioError(f"{description} must not contain a newline")
    return value.strip()


def _require_schema_version(data: Mapping[str, Any], description: str) -> None:
    if data.get("schema_version") != SUPPORTED_SCHEMA_VERSION:
        raise StudioError(
            f"{description} uses unsupported schema_version "
            f"{data.get('schema_version')!r}; expected {SUPPORTED_SCHEMA_VERSION}"
        )


@dataclass(frozen=True)
class Component:
    component_id: str
    display_name: str
    instance_prefix: str
    ports: tuple[str, ...]
    required_parameters: frozenset[str]
    parameter_types: Mapping[str, str]
    defaults: Mapping[str, Any]
    netlist_template: str
    source_path: Path

    @classmethod
    def load(cls, path: Path) -> "Component":
        data = _read_json(path)
        _require_schema_version(data, f"component {path}")
        component_id = _require_text(data.get("id"), f"component id in {path}")
        display_name = _require_text(
            data.get("display_name", component_id), f"display_name in {path}"
        )
        prefix = _require_text(
            data.get("instance_prefix"), f"instance_prefix in {path}"
        ).upper()
        if len(prefix) != 1 or not prefix.isalpha():
            raise StudioError(f"instance_prefix in {path} must be one letter")

        raw_ports = data.get("ports")
        if not isinstance(raw_ports, list) or not raw_ports:
            raise StudioError(f"ports in {path} must be a non-empty array")
        ports: list[str] = []
        for entry in raw_ports:
            if not isinstance(entry, dict):
                raise StudioError(f"each port in {path} must be an object")
            name = _require_text(entry.get("name"), f"port name in {path}")
            if not IDENTIFIER_RE.fullmatch(name):
                raise StudioError(f"invalid port name {name!r} in {path}")
            if name in ports:
                raise StudioError(f"duplicate port {name!r} in {path}")
            ports.append(name)

        raw_parameters = data.get("parameters", {})
        if not isinstance(raw_parameters, dict):
            raise StudioError(f"parameters in {path} must be an object")
        required: set[str] = set()
        parameter_types: dict[str, str] = {}
        defaults: dict[str, Any] = {}
        for name, spec in raw_parameters.items():
            if not IDENTIFIER_RE.fullmatch(name) or not isinstance(spec, dict):
                raise StudioError(f"invalid parameter definition {name!r} in {path}")
            value_type = spec.get("type", "string")
            if value_type not in {"spice_scalar", "string", "number", "integer", "boolean"}:
                raise StudioError(f"unsupported type {value_type!r} for {name} in {path}")
            parameter_types[name] = value_type
            if "default" in spec:
                defaults[name] = spec["default"]
            elif spec.get("required", False):
                required.add(name)

        xyce = data.get("xyce")
        if not isinstance(xyce, dict):
            raise StudioError(f"xyce in {path} must be an object")
        netlist_template = _require_text(
            xyce.get("netlist_template"), f"xyce.netlist_template in {path}"
        )
        allowed_fields = {"instance"}
        allowed_fields.update(f"port_{name}" for name in ports)
        allowed_fields.update(f"param_{name}" for name in parameter_types)
        referenced_fields = {
            match.group("named") or match.group("braced")
            for match in Template.pattern.finditer(netlist_template)
            if match.group("named") or match.group("braced")
        }
        unknown_fields = referenced_fields - allowed_fields
        if unknown_fields:
            raise StudioError(
                f"unknown template field(s) in {path}: {', '.join(sorted(unknown_fields))}"
            )
        return cls(
            component_id=component_id,
            display_name=display_name,
            instance_prefix=prefix,
            ports=tuple(ports),
            required_parameters=frozenset(required),
            parameter_types=parameter_types,
            defaults=defaults,
            netlist_template=netlist_template,
            source_path=path,
        )

    def render(self, instance: Mapping[str, Any]) -> str:
        name = _require_text(instance.get("name"), "instance name")
        if not IDENTIFIER_RE.fullmatch(name):
            raise StudioError(f"invalid instance name {name!r}")
        if not name.upper().startswith(self.instance_prefix):
            raise StudioError(
                f"instance {name!r} for {self.component_id} must start with "
                f"{self.instance_prefix!r} for SPICE device dispatch"
            )

        raw_ports = instance.get("ports")
        if not isinstance(raw_ports, dict):
            raise StudioError(f"ports of instance {name} must be an object")
        missing_ports = set(self.ports) - set(raw_ports)
        extra_ports = set(raw_ports) - set(self.ports)
        if missing_ports or extra_ports:
            raise StudioError(
                f"ports of instance {name} mismatch: missing={sorted(missing_ports)}, "
                f"unknown={sorted(extra_ports)}"
            )

        raw_parameters = instance.get("parameters", {})
        if not isinstance(raw_parameters, dict):
            raise StudioError(f"parameters of instance {name} must be an object")
        extra_parameters = set(raw_parameters) - set(self.parameter_types)
        if extra_parameters:
            raise StudioError(
                f"unknown parameter(s) for instance {name}: "
                f"{', '.join(sorted(extra_parameters))}"
            )
        parameters = dict(self.defaults)
        parameters.update(raw_parameters)
        missing_parameters = self.required_parameters - set(parameters)
        if missing_parameters:
            raise StudioError(
                f"missing parameter(s) for instance {name}: "
                f"{', '.join(sorted(missing_parameters))}"
            )

        fields: dict[str, str] = {"instance": name}
        for port in self.ports:
            node = _require_text(raw_ports[port], f"port {port} of instance {name}")
            if not NODE_RE.fullmatch(node):
                raise StudioError(f"invalid node {node!r} on instance {name}")
            fields[f"port_{port}"] = node
        for parameter, value_type in self.parameter_types.items():
            if parameter not in parameters:
                continue
            fields[f"param_{parameter}"] = _format_parameter(
                parameters[parameter], value_type, f"parameter {parameter} of instance {name}"
            )
        try:
            return Template(self.netlist_template).substitute(fields)
        except (KeyError, ValueError) as exc:
            raise StudioError(f"cannot render instance {name}: {exc}") from exc


def _format_parameter(value: Any, value_type: str, description: str) -> str:
    if value_type == "boolean":
        if not isinstance(value, bool):
            raise StudioError(f"{description} must be boolean")
        return "1" if value else "0"
    if value_type == "integer":
        if not isinstance(value, int) or isinstance(value, bool):
            raise StudioError(f"{description} must be an integer")
        return str(value)
    if value_type == "number":
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise StudioError(f"{description} must be numeric")
        return str(value)
    text = _require_text(value, description)
    if value_type == "spice_scalar" and not SPICE_SCALAR_RE.fullmatch(text):
        raise StudioError(f"{description} is not a conservative SPICE scalar: {text!r}")
    return text


def _library_json_files(path: Path) -> Iterable[Path]:
    if path.is_file():
        yield path
        return
    if not path.is_dir():
        raise StudioError(f"component library does not exist: {path}")
    yield from sorted(path.glob("*.json"))


def load_components(library_paths: Sequence[Path]) -> dict[str, Component]:
    components: dict[str, Component] = {}
    for library_path in library_paths:
        for component_file in _library_json_files(library_path):
            component = Component.load(component_file)
            if component.component_id in components:
                previous = components[component.component_id].source_path
                raise StudioError(
                    f"duplicate component id {component.component_id!r}: "
                    f"{previous} and {component_file}"
                )
            components[component.component_id] = component
    return components


def project_library_paths(project_path: Path, project: Mapping[str, Any]) -> list[Path]:
    raw_libraries = project.get("libraries", [])
    if not isinstance(raw_libraries, list):
        raise StudioError("project libraries must be an array")
    paths = [BUILTIN_LIBRARY]
    for raw_path in raw_libraries:
        relative_path = Path(_require_text(raw_path, "project library path"))
        paths.append(relative_path if relative_path.is_absolute() else project_path.parent / relative_path)
    return paths


def generate_netlist(
    project_path: Path, extra_libraries: Sequence[Path] = ()
) -> str:
    project_path = project_path.resolve()
    project = _read_json(project_path)
    _require_schema_version(project, f"project {project_path}")
    components = load_components(
        [*project_library_paths(project_path, project), *[path.resolve() for path in extra_libraries]]
    )

    title = _require_text(project.get("title", project_path.stem), "project title")
    raw_instances = project.get("instances")
    if not isinstance(raw_instances, list) or not raw_instances:
        raise StudioError("project instances must be a non-empty array")
    names: set[str] = set()
    device_lines: list[str] = []
    for raw_instance in raw_instances:
        if not isinstance(raw_instance, dict):
            raise StudioError("each project instance must be an object")
        name = _require_text(raw_instance.get("name"), "instance name")
        if name.upper() in names:
            raise StudioError(f"duplicate instance name {name!r}")
        names.add(name.upper())
        module_id = _require_text(raw_instance.get("module"), f"module of instance {name}")
        component = components.get(module_id)
        if component is None:
            raise StudioError(f"unknown component {module_id!r} used by instance {name}")
        device_lines.append(component.render(raw_instance))

    analysis = project.get("analysis")
    if not isinstance(analysis, dict) or analysis.get("type", "").lower() != "tran":
        raise StudioError("the prototype currently requires analysis.type = 'tran'")
    step = _format_parameter(analysis.get("step"), "spice_scalar", "analysis step")
    stop = _format_parameter(analysis.get("stop"), "spice_scalar", "analysis stop")
    tran_fields = [".TRAN", step, stop]
    if "start" in analysis:
        tran_fields.append(_format_parameter(analysis["start"], "spice_scalar", "analysis start"))

    raw_probes = project.get("probes")
    if not isinstance(raw_probes, list) or not raw_probes:
        raise StudioError("project probes must be a non-empty array")
    probes: list[str] = []
    for raw_probe in raw_probes:
        if not isinstance(raw_probe, dict):
            raise StudioError("each probe must be an object")
        expression = _require_text(raw_probe.get("expression"), "probe expression")
        if len(expression) > 256:
            raise StudioError("probe expression is too long")
        probes.append(expression)

    output = project.get("output", {})
    if not isinstance(output, dict):
        raise StudioError("project output must be an object")
    output_format = _require_text(output.get("format", "CSV"), "output format").upper()
    if output_format not in {"CSV", "STD", "TECPLOT"}:
        raise StudioError(f"unsupported Xyce output format {output_format!r}")
    output_file = _require_text(output.get("file", "waveforms.csv"), "output file")
    output_path = Path(output_file)
    if output_path.is_absolute() or ".." in output_path.parts:
        raise StudioError("output file must be relative and must not contain '..'")

    lines = [
        title,
        "* Generated by GMP CCTL Studio; edit the JSON project or component data.",
        *device_lines,
        " ".join(tran_fields),
        f".PRINT TRAN FORMAT={output_format} FILE={output_path.as_posix()} {' '.join(probes)}",
        ".END",
        "",
    ]
    return "\n".join(lines)


def find_xyce(explicit: str | None = None) -> Path:
    candidates = [explicit, os.environ.get("XYCE_EXECUTABLE")]
    for candidate in candidates:
        if candidate:
            path = Path(candidate).expanduser().resolve()
            if path.is_file():
                return path
            raise StudioError(f"Xyce executable does not exist: {path}")
    discovered = shutil.which("Xyce") or shutil.which("Xyce.exe")
    if discovered:
        return Path(discovered).resolve()
    raise StudioError(
        "Xyce executable was not found; pass --xyce, set XYCE_EXECUTABLE, or add Xyce to PATH"
    )


def run_xyce(
    project_path: Path,
    output_dir: Path,
    extra_libraries: Sequence[Path] = (),
    xyce_executable: str | None = None,
) -> subprocess.CompletedProcess[str]:
    netlist = generate_netlist(project_path, extra_libraries)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_dir = output_dir.resolve()
    netlist_path = output_dir / f"{project_path.stem}.cir"
    netlist_path.write_text(netlist, encoding="utf-8", newline="\n")
    xyce = find_xyce(xyce_executable)
    return subprocess.run(
        [str(xyce), str(netlist_path)],
        cwd=output_dir,
        check=True,
        text=True,
        capture_output=False,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate JSON circuit projects and generate Xyce-compatible .cir netlists."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_project_arguments(command: argparse.ArgumentParser) -> None:
        command.add_argument("project", type=Path)
        command.add_argument(
            "--library",
            action="append",
            default=[],
            type=Path,
            help="additional component JSON file or directory (repeatable)",
        )

    validate = subparsers.add_parser("validate", help="validate a project and its libraries")
    add_project_arguments(validate)

    generate = subparsers.add_parser("generate", help="generate a Xyce .cir netlist")
    add_project_arguments(generate)
    generate.add_argument("--output", "-o", required=True, type=Path)

    run = subparsers.add_parser("run", help="generate a netlist and invoke Xyce")
    add_project_arguments(run)
    run.add_argument("--output-dir", "-o", required=True, type=Path)
    run.add_argument("--xyce", help="path to the Xyce executable")

    list_components = subparsers.add_parser(
        "list-components", help="list components in the built-in and optional libraries"
    )
    list_components.add_argument("--library", action="append", default=[], type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "list-components":
            components = load_components([BUILTIN_LIBRARY, *args.library])
            for component_id in sorted(components):
                component = components[component_id]
                print(f"{component_id}\t{component.display_name}\t{component.source_path}")
            return 0

        if args.command == "validate":
            generate_netlist(args.project, args.library)
            print(f"valid: {args.project}")
            return 0
        if args.command == "generate":
            netlist = generate_netlist(args.project, args.library)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(netlist, encoding="utf-8", newline="\n")
            print(args.output.resolve())
            return 0
        if args.command == "run":
            run_xyce(args.project, args.output_dir, args.library, args.xyce)
            return 0
        raise StudioError(f"unsupported command {args.command!r}")
    except StudioError as exc:
        print(f"cctl_studio: error: {exc}", file=sys.stderr)
        return 2
    except subprocess.CalledProcessError as exc:
        print(f"cctl_studio: Xyce exited with status {exc.returncode}", file=sys.stderr)
        return exc.returncode or 1


if __name__ == "__main__":
    raise SystemExit(main())
