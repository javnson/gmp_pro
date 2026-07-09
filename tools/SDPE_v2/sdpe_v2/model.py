"""Core data model for SDPE v2."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


class SDPEError(RuntimeError):
    """Raised when SDPE data cannot be resolved or generated."""


@dataclass(frozen=True)
class ParameterSpec:
    """A parameter exposed by a hardware schema."""

    name: str
    c_name: str
    unit: str = ""
    description: str = ""
    required: bool = False
    default: Any = None
    value_format: str = "{}"


@dataclass(frozen=True)
class DerivedMacroSpec:
    """A macro derived from entity parameters."""

    name: str
    expr: str
    description: str = ""
    unit: str = ""


@dataclass(frozen=True)
class ComponentSlotSpec:
    """A named child hardware slot required or accepted by a schema."""

    name: str
    accepted_schemas: tuple[str, ...] = ()
    accepted_categories: tuple[str, ...] = ()
    required: bool = False
    description: str = ""


@dataclass(frozen=True)
class ExportSpec:
    """A logical export exposed by a schema."""

    name: str
    macro: str
    description: str = ""
    unit: str = ""


@dataclass
class HardwareSchema:
    """A hardware template definition."""

    id: str
    display_name: str
    description: str = ""
    category: str = ""
    tags: list[str] = field(default_factory=list)
    output_subdir: str = ""
    header_prefix: str = ""
    includes: tuple[str, ...] = ()
    code_sections: dict[str, Any] = field(default_factory=dict)
    option_sets: dict[str, Any] = field(default_factory=dict)
    feature_macros: list[dict[str, Any]] = field(default_factory=list)
    option_macros: list[dict[str, Any]] = field(default_factory=list)
    parameters: dict[str, ParameterSpec] = field(default_factory=dict)
    derived_macros: list[DerivedMacroSpec] = field(default_factory=list)
    default_components: dict[str, Any] = field(default_factory=dict)
    component_slots: dict[str, ComponentSlotSpec] = field(default_factory=dict)
    exports: dict[str, ExportSpec] = field(default_factory=dict)
    required_components: tuple[str, ...] = ()

    @classmethod
    def from_json(cls, data: dict[str, Any], source: Path) -> "HardwareSchema":
        schema_id = data.get("id")
        if not schema_id:
            raise SDPEError(f"Schema file {source} has no 'id'.")

        parameters = {}
        for item in data.get("parameters", []):
            name = item.get("name")
            if not name:
                raise SDPEError(f"Schema {schema_id} has a parameter without name.")
            parameters[name] = ParameterSpec(
                name=name,
                c_name=item.get("c_name", name.upper()),
                unit=item.get("unit", ""),
                description=item.get("description", ""),
                required=bool(item.get("required", False)),
                default=item.get("default"),
                value_format=item.get("value_format", "{}"),
            )

        derived_macros = [
            DerivedMacroSpec(
                name=item["name"],
                expr=item["expr"],
                description=item.get("description", ""),
                unit=item.get("unit", ""),
            )
            for item in data.get("derived_macros", [])
        ]

        component_slots = {}
        for name, item in data.get("component_slots", {}).items():
            component_slots[name] = ComponentSlotSpec(
                name=name,
                accepted_schemas=tuple(item.get("accepted_schemas", [])),
                accepted_categories=tuple(item.get("accepted_categories", [])),
                required=bool(item.get("required", False)),
                description=item.get("description", ""),
            )

        exports = {}
        for name, item in data.get("exports", {}).items():
            exports[name] = ExportSpec(
                name=name,
                macro=item["macro"],
                description=item.get("description", ""),
                unit=item.get("unit", ""),
            )

        return cls(
            id=schema_id,
            display_name=data.get("display_name", schema_id),
            description=data.get("description", ""),
            category=data.get("category", ""),
            tags=list(data.get("tags", [])),
            output_subdir=data.get("output_subdir", schema_id),
            header_prefix=data.get("header_prefix", ""),
            includes=tuple(data.get("includes", [])),
            code_sections=dict(data.get("code_sections", {})),
            option_sets=dict(data.get("option_sets", {})),
            feature_macros=list(data.get("feature_macros", [])),
            option_macros=list(data.get("option_macros", [])),
            parameters=parameters,
            derived_macros=derived_macros,
            default_components=dict(data.get("components", data.get("default_components", {}))),
            component_slots=component_slots,
            exports=exports,
            required_components=tuple(data.get("required_components", [])),
        )


@dataclass
class ComponentRef:
    """A resolved child component relation."""

    slot: str
    entity: "HardwareEntity"
    inline: bool = False
    overrides: dict[str, Any] = field(default_factory=dict)


@dataclass
class HardwareEntity:
    """A concrete hardware instance."""

    id: str
    schema_id: str
    display_name: str = ""
    description: str = ""
    vendor: str = ""
    datasheet_url: str = ""
    document_url: str = ""
    macro_prefix: str = ""
    output_subdir: str = ""
    includes: list[str] = field(default_factory=list)
    code_sections: dict[str, Any] = field(default_factory=dict)
    option_sets: dict[str, Any] = field(default_factory=dict)
    feature_macros: list[dict[str, Any]] = field(default_factory=list)
    option_macros: list[dict[str, Any]] = field(default_factory=list)
    conditional_macros: list[dict[str, Any]] = field(default_factory=list)
    parameters: dict[str, Any] = field(default_factory=dict)
    components: dict[str, ComponentRef] = field(default_factory=dict)
    tags: list[str] = field(default_factory=list)
    source: Path | None = None
    inline: bool = False

    @classmethod
    def from_json(cls, data: dict[str, Any], source: Path | None = None, inline: bool = False) -> "HardwareEntity":
        entity_id = data.get("id")
        schema_id = data.get("schema")
        if not entity_id:
            raise SDPEError(f"Entity in {source or '<inline>'} has no 'id'.")
        if not schema_id:
            raise SDPEError(f"Entity {entity_id} has no 'schema'.")
        return cls(
            id=entity_id,
            schema_id=schema_id,
            display_name=data.get("display_name", entity_id),
            description=data.get("description", ""),
            vendor=data.get("vendor", ""),
            datasheet_url=data.get("datasheet_url", ""),
            document_url=data.get("document_url", ""),
            macro_prefix=data.get("macro_prefix", ""),
            output_subdir=data.get("output_subdir", ""),
            includes=list(data.get("includes", [])),
            code_sections=dict(data.get("code_sections", {})),
            option_sets=dict(data.get("option_sets", {})),
            feature_macros=list(data.get("feature_macros", [])),
            option_macros=list(data.get("option_macros", [])),
            conditional_macros=list(data.get("conditional_macros", [])),
            parameters=dict(data.get("parameters", {})),
            tags=list(data.get("tags", [])),
            source=source,
            inline=inline,
        )
