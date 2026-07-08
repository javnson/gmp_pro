"""SDPE v2 library loader."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .model import ComponentRef, HardwareEntity, HardwareSchema, SDPEError
from .util import read_json


class SDPELibrary:
    """Load schemas and entities from a library directory."""

    def __init__(
        self,
        root: Path,
        schema_dirs: list[Path] | None = None,
        entity_dirs: list[Path] | None = None,
    ):
        self.root = root
        self.schemas_dir = root / "schemas"
        self.entities_dir = root / "entities"
        self.schema_dirs = schema_dirs or [self.schemas_dir]
        self.entity_dirs = entity_dirs or [self.entities_dir]
        self.schemas: dict[str, HardwareSchema] = {}
        self.entity_files: dict[str, Path] = {}
        self._entities: dict[str, HardwareEntity] = {}

    def load(self) -> "SDPELibrary":
        """Load all schema and entity indexes."""

        existing_schema_dirs = [path for path in self.schema_dirs if path.exists()]
        if not existing_schema_dirs:
            raise SDPEError(f"Schema directory not found: {self.schema_dirs[0]}")
        for schema_dir in existing_schema_dirs:
            for path in sorted(schema_dir.rglob("*.json")):
                schema = HardwareSchema.from_json(read_json(path), path)
                if schema.id in self.schemas:
                    raise SDPEError(f"Duplicate schema id: {schema.id}")
                self.schemas[schema.id] = schema

        for entity_dir in [path for path in self.entity_dirs if path.exists()]:
            for path in sorted(entity_dir.rglob("*.json")):
                data = read_json(path)
                entity_id = data.get("id")
                if not entity_id:
                    raise SDPEError(f"Entity file {path} has no 'id'.")
                if entity_id in self.entity_files:
                    raise SDPEError(f"Duplicate entity id: {entity_id}")
                self.entity_files[entity_id] = path
        return self

    def schema(self, schema_id: str) -> HardwareSchema:
        """Get a schema by id."""

        try:
            return self.schemas[schema_id]
        except KeyError as exc:
            raise SDPEError(f"Unknown schema: {schema_id}") from exc

    def entity(self, entity_id: str) -> HardwareEntity:
        """Resolve an entity and all child references."""

        if entity_id in self._entities:
            return self._entities[entity_id]
        try:
            path = self.entity_files[entity_id]
        except KeyError as exc:
            raise SDPEError(f"Unknown entity: {entity_id}") from exc
        entity = HardwareEntity.from_json(read_json(path), path)
        self._entities[entity.id] = entity
        self._resolve_components(entity, read_json(path).get("components", {}))
        return entity

    def inline_entity(self, data: dict[str, Any], parent_id: str, slot: str) -> HardwareEntity:
        """Create a resolved inline entity."""

        inline_data = dict(data)
        inline_data.setdefault("id", f"{parent_id}_{slot}")
        entity = HardwareEntity.from_json(inline_data, None, inline=True)
        self._resolve_components(entity, inline_data.get("components", {}))
        return entity

    def _resolve_components(self, entity: HardwareEntity, components_data: dict[str, Any]) -> None:
        schema = self.schema(entity.schema_id)

        for required in schema.required_components:
            if required not in components_data:
                raise SDPEError(f"Entity {entity.id} requires component slot '{required}'.")

        for slot, comp_data in components_data.items():
            if slot not in schema.component_slots:
                raise SDPEError(f"Entity {entity.id} uses unknown component slot '{slot}'.")
            slot_spec = schema.component_slots[slot]
            if "entity" in comp_data:
                child = self.entity(comp_data["entity"])
                inline = False
            elif "inline" in comp_data:
                child = self.inline_entity(comp_data["inline"], entity.id, slot)
                inline = True
            else:
                raise SDPEError(f"Component {entity.id}.{slot} must have 'entity' or 'inline'.")

            child_schema = self.schema(child.schema_id)
            has_schema_rule = bool(slot_spec.accepted_schemas)
            has_category_rule = bool(slot_spec.accepted_categories)
            schema_allowed = has_schema_rule and child.schema_id in slot_spec.accepted_schemas
            category_allowed = has_category_rule and child_schema.category in slot_spec.accepted_categories
            if (has_schema_rule or has_category_rule) and not (schema_allowed or category_allowed):
                allowed = ", ".join([*slot_spec.accepted_schemas, *slot_spec.accepted_categories])
                raise SDPEError(
                    f"Component {entity.id}.{slot} schema '{child.schema_id}' "
                    f"category '{child_schema.category}' is not allowed. "
                    f"Allowed: {allowed}"
                )
            overrides = dict(comp_data.get("overrides", {}))
            entity.components[slot] = ComponentRef(slot=slot, entity=child, inline=inline, overrides=overrides)

    def validate(self) -> list[str]:
        """Resolve every entity and return warning messages."""

        warnings: list[str] = []
        for entity_id in sorted(self.entity_files):
            entity = self.entity(entity_id)
            schema = self.schema(entity.schema_id)
            for pname, pspec in schema.parameters.items():
                if pspec.required and pname not in entity.parameters and pspec.default is None:
                    raise SDPEError(f"Entity {entity.id} misses required parameter '{pname}'.")
            for slot, slot_spec in schema.component_slots.items():
                if slot_spec.required and slot not in entity.components:
                    raise SDPEError(f"Entity {entity.id} misses required component '{slot}'.")
        return warnings
