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
        if self.data.get("system_type") != "siso":
            raise ComponentError("the basic framework currently supports system_type='siso' only")

        implementation = _require(self.data, "implementation", dict)
        instance_type = _require(implementation, "instance_type", str)
        if not _IDENTIFIER.fullmatch(instance_type):
            raise ComponentError("implementation.instance_type must be a C identifier")
        headers = _require(implementation, "headers", list)
        sources = _require(implementation, "sources", list)
        if not headers or not sources:
            raise ComponentError("implementation requires at least one header and source")
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
        if len(inputs) != 1 or len(outputs) != 1:
            raise ComponentError("SISO definitions require exactly one input and one output")

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

        initializers = _require(self.data, "initializers", list)
        if not initializers:
            raise ComponentError("at least one initializer is required")
        for initializer in initializers:
            function = _require(initializer, "function", str)
            step_function = _require(initializer, "step_function", str)
            if not _IDENTIFIER.fullmatch(function) or not _IDENTIFIER.fullmatch(step_function):
                raise ComponentError("initializer functions must be C identifiers")
            for argument in _require(initializer, "arguments", list):
                if argument != "$instance" and argument not in parameter_ids:
                    raise ComponentError(f"initializer references unknown parameter: {argument}")

        if self.data.get("template") != "pid_siso_v1":
            raise ComponentError("the basic framework currently supports template='pid_siso_v1'")

