"""Component definition loading and validation."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any


class ComponentError(ValueError):
    """Raised when a component definition is incomplete or unsafe."""


_IDENTIFIER = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_COMPONENT_ID = re.compile(r"^[a-z][a-z0-9_.-]*$")


def repository_root() -> Path:
    value = os.environ.get("GMP_PRO_LOCATION", "").strip()
    if not value:
        raise ComponentError("GMP_PRO_LOCATION is not defined")
    root = Path(value).resolve()
    required = (root / "gmp_core.h", root / "tools" / "gmp_installer")
    if not all(path.exists() for path in required):
        raise ComponentError(f"GMP_PRO_LOCATION is not a GMP repository: {root}")
    return root


def _require(data: dict[str, Any], key: str, expected: type) -> Any:
    value = data.get(key)
    if not isinstance(value, expected):
        raise ComponentError(f"{key!r} must be {expected.__name__}")
    return value


def _safe_relative_path(root: Path, value: str, label: str) -> str:
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise ComponentError(f"{label} must be relative to GMP_PRO_LOCATION: {value}")
    resolved = (root / path).resolve()
    if root not in resolved.parents and resolved != root:
        raise ComponentError(f"{label} escapes GMP_PRO_LOCATION: {value}")
    if not resolved.is_file():
        raise ComponentError(f"{label} does not exist: {value}")
    return path.as_posix()


@dataclass(frozen=True)
class ComponentDefinition:
    """Validated component JSON plus its source path."""

    path: Path
    data: dict[str, Any]

    @property
    def component_id(self) -> str:
        return str(self.data["id"])

    @property
    def symbol(self) -> str:
        return self.component_id.replace(".", "_").replace("-", "_")

    @property
    def sfunction_name(self) -> str:
        return f"gmp_mcb_{self.symbol}"

    @classmethod
    def load(cls, path: Path) -> "ComponentDefinition":
        path = path.resolve()
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ComponentError(f"Cannot read {path}: {exc}") from exc
        item = cls(path=path, data=data)
        item.validate()
        return item

    def validate(self) -> None:
        root = repository_root()
        component_id = _require(self.data, "id", str)
        if not _COMPONENT_ID.fullmatch(component_id):
            raise ComponentError("id must use lower-case letters, digits, dots, dashes or underscores")
        system_type = self.data.get("system_type")
        if system_type not in {"siso", "mimo"}:
            raise ComponentError("system_type must be 'siso' or 'mimo'")

        implementation = _require(self.data, "implementation", dict)
        instance_type = _require(implementation, "instance_type", str)
        if not _IDENTIFIER.fullmatch(instance_type):
            raise ComponentError("implementation.instance_type must be a C identifier")
        headers = _require(implementation, "headers", list)
        sources = _require(implementation, "sources", list)
        if not headers:
            raise ComponentError("implementation requires at least one header")
        for index, value in enumerate(headers):
            if not isinstance(value, str):
                raise ComponentError("implementation.headers entries must be strings")
            _safe_relative_path(root, value, f"headers[{index}]")
        for index, value in enumerate(sources):
            if not isinstance(value, str):
                raise ComponentError("implementation.sources entries must be strings")
            _safe_relative_path(root, value, f"sources[{index}]")

        inputs = _require(self.data, "inputs", list)
        outputs = _require(self.data, "outputs", list)
        if not inputs or not outputs:
            raise ComponentError("a component requires at least one input and output")
        if system_type == "siso" and (len(inputs) != 1 or len(outputs) != 1):
            raise ComponentError("SISO definitions require exactly one input and one output")
        for collection_name, collection in (("inputs", inputs), ("outputs", outputs)):
            port_ids: set[str] = set()
            for port in collection:
                if not isinstance(port, dict):
                    raise ComponentError(f"{collection_name} entries must be objects")
                port_id = _require(port, "id", str)
                if not _IDENTIFIER.fullmatch(port_id) or port_id in port_ids:
                    raise ComponentError(f"invalid or duplicate {collection_name} id: {port_id}")
                port_ids.add(port_id)
                if port.get("width", 1) != 1 or port.get("type", "double") != "double":
                    raise ComponentError("the current generic MEX backend supports scalar double ports")

        parameters = _require(self.data, "parameters", list)
        parameter_ids: set[str] = set()
        for parameter in parameters:
            if not isinstance(parameter, dict):
                raise ComponentError("parameters entries must be objects")
            param_id = _require(parameter, "id", str)
            if not _IDENTIFIER.fullmatch(param_id) or param_id in parameter_ids:
                raise ComponentError(f"invalid or duplicate parameter id: {param_id}")
            parameter_ids.add(param_id)
            if parameter.get("type") not in {"number", "choice"}:
                raise ComponentError(f"unsupported parameter type for {param_id}")
            if not isinstance(parameter.get("group", "General"), str):
                raise ComponentError(f"parameter group must be a string for {param_id}")
            if not isinstance(parameter.get("externalizable", False), bool):
                raise ComponentError(f"externalizable must be true or false for {param_id}")

        initializers = _require(self.data, "initializers", list)
        if not initializers:
            raise ComponentError("at least one initializer is required")
        for initializer in initializers:
            function = _require(initializer, "function", str)
            step_function = _require(initializer, "step_function", str)
            if not _IDENTIFIER.fullmatch(function) or not _IDENTIFIER.fullmatch(step_function):
                raise ComponentError("initializer functions must be C identifiers")
            for argument in _require(initializer, "arguments", list):
                if argument != "$instance" and not str(argument).startswith("$memory:") and argument not in parameter_ids:
                    raise ComponentError(f"initializer references unknown parameter: {argument}")

        template = self.data.get("template")
        if template not in {"pid_siso_v2", "resonant_siso_v1", "generic_stateful_v1"}:
            raise ComponentError(f"unsupported component template: {template}")
        if template == "resonant_siso_v1" and self.data.get("variant") not in {"r", "pr", "qr", "qpr"}:
            raise ComponentError("resonant_siso_v1 requires variant r, pr, qr, or qpr")
        if template == "generic_stateful_v1":
            execution = _require(self.data, "execution", dict)
            step_code = _require(execution, "step_code", list)
            if not step_code or not all(isinstance(line, str) for line in step_code):
                raise ComponentError("generic_stateful_v1 execution.step_code must contain C++ statements")
            for field in ("initialize_code", "parameter_update_code", "terminate_code"):
                value = execution.get(field, [])
                if not isinstance(value, list) or not all(isinstance(line, str) for line in value):
                    raise ComponentError(f"execution.{field} must be a list of C++ statements")
            memory_ids: set[str] = set()
            for memory in self.data.get("memory", []):
                if not isinstance(memory, dict):
                    raise ComponentError("memory entries must be objects")
                memory_id = _require(memory, "id", str)
                if not _IDENTIFIER.fullmatch(memory_id) or memory_id in memory_ids:
                    raise ComponentError(f"invalid or duplicate memory id: {memory_id}")
                memory_ids.add(memory_id)
                if _require(memory, "length_parameter", str) not in parameter_ids:
                    raise ComponentError(f"memory {memory_id} references an unknown length parameter")
                pointer_field = _require(memory, "state_pointer_field", str)
                if not _IDENTIFIER.fullmatch(pointer_field):
                    raise ComponentError(f"memory {memory_id} state_pointer_field must be a C identifier")
