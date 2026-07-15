"""C header generator for SDPE v2."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .library import SDPELibrary
from .model import ComponentRef, HardwareEntity, HardwareSchema, SDPEError
from .util import c_literal, header_guard, macro_name, read_json, write_if_changed


@dataclass(frozen=True)
class GeneratedFile:
    """Generated file metadata."""

    path: Path
    changed: bool


def folder_name(value: str) -> str:
    """Normalize generated folder names to lowercase snake_case."""

    text = re.sub(r"[^A-Za-z0-9]+", "_", str(value).strip())
    text = re.sub(r"_+", "_", text).strip("_").lower()
    return text or "misc"


class HeaderGenerator:
    """Generate hardware entity and project binding headers."""

    def __init__(
        self,
        library: SDPELibrary,
        out_dir: Path,
        include_prefix: str = "ctl/component",
        include_mode: str = "prefixed",
        project_subdir: str = "project",
        system_entity_dirs: list[Path] | None = None,
        system_out_dir: Path | None = None,
        system_include_prefix: str = "ctl",
    ):
        self.library = library
        self.out_dir = out_dir
        self.include_prefix = include_prefix.strip("/")
        self.include_mode = include_mode
        self.project_subdir = project_subdir.strip("/\\")
        self.system_entity_dirs = [path.resolve() for path in (system_entity_dirs or [])]
        self.system_out_dir = system_out_dir.resolve() if system_out_dir else None
        self.system_include_prefix = system_include_prefix.strip("/")

    @staticmethod
    def project_metadata_macro(data: dict[str, Any], name: str) -> str:
        """Return a project metadata macro with its optional namespace prefix."""

        prefix = str(data.get("macro_prefix", "")).strip()
        return f"{macro_name(prefix)}_{name}" if prefix else name

    def generate_entity_tree(self, entity_id: str, skip_system: bool = False) -> list[GeneratedFile]:
        """Generate headers for an entity and every referenced non-inline child."""

        entity = self.library.entity(entity_id)
        generated: list[GeneratedFile] = []
        visited: set[str] = set()

        def walk(node: HardwareEntity) -> None:
            if skip_system and self.is_system_entity(node):
                return
            for comp in node.components.values():
                if not comp.inline:
                    walk(comp.entity)
            if not node.inline and node.id not in visited:
                visited.add(node.id)
                path = self.entity_header_path(node)
                changed = write_if_changed(path, self.render_entity_header(node))
                generated.append(GeneratedFile(path, changed))

        walk(entity)
        return generated

    def generate_all_entities(self) -> list[GeneratedFile]:
        """Generate all entity headers in the library."""

        generated: list[GeneratedFile] = []
        seen: set[Path] = set()
        for entity_id in sorted(self.library.entity_files):
            for item in self.generate_entity_tree(entity_id):
                if item.path not in seen:
                    seen.add(item.path)
                    generated.append(item)
        return generated

    def entity_header_path(self, entity: HardwareEntity) -> Path:
        """Return the output header path for an entity."""

        if self.is_system_entity(entity) and self.system_out_dir is not None:
            return self.entity_header_path_in_root(entity, self.system_out_dir)
        return self.entity_header_path_in_root(entity, self.out_dir)

    def local_entity_header_path(self, entity: HardwareEntity) -> Path:
        """Return the local output header path for an entity."""

        return self.entity_header_path_in_root(entity, self.out_dir)

    def entity_header_path_in_root(self, entity: HardwareEntity, root: Path) -> Path:
        """Return the generated header path for an entity under a given root."""

        schema = self.library.schema(entity.schema_id)
        subdir = folder_name(entity.output_subdir or schema.output_subdir or schema.category or schema.id)
        return root / "hardware_preset" / subdir / f"{entity.id}.h"

    def is_system_entity(self, entity: HardwareEntity) -> bool:
        """Return whether the entity belongs to the configured global hardware library."""

        if not entity.source or not self.system_entity_dirs:
            return False
        source = entity.source.resolve()
        return any(source.is_relative_to(root) for root in self.system_entity_dirs)

    def entity_include_path(self, entity: HardwareEntity, from_path: Path | None = None) -> str:
        """Return generated include path relative to the output root."""

        if self.is_system_entity(entity) and self.system_out_dir is not None:
            path = self.entity_header_path_in_root(entity, self.system_out_dir)
            rel = path.relative_to(self.system_out_dir).as_posix()
            prefix = self.system_include_prefix
            return f"<{prefix}/{rel}>" if prefix else f"<{rel}>"

        path = self.entity_header_path(entity)
        if self.include_mode == "relative" and from_path is not None:
            return path.relative_to(from_path.parent, walk_up=True).as_posix()
        rel = path.relative_to(self.out_dir).as_posix()
        if self.include_prefix:
            return f"{self.include_prefix}/{rel}"
        return rel

    def include_directive(self, include_path: str) -> str:
        """Return an include directive with the configured delimiter style."""

        text = include_path.strip()
        if text.startswith("<") or text.startswith('"'):
            return f"#include {text}"
        if self.include_mode == "relative":
            return f'#include "{text}"'
        return f"#include <{text}>"

    def render_entity_header(self, entity: HardwareEntity) -> str:
        """Render a hardware entity header."""

        schema = self.library.schema(entity.schema_id)
        header_path = self.local_entity_header_path(entity)
        rel = header_path.relative_to(self.out_dir).as_posix()
        guard = header_guard(rel)
        prefix = self.prefix(entity, schema)
        includes = self._entity_includes(entity, schema, header_path)

        lines: list[str] = []
        lines.extend(
            [
                "/**",
                f" * @file {entity.id}.h",
                f" * @brief SDPE hardware preset for {entity.display_name or entity.id}.",
                " * @details Generated by tools/SDPE_v2. Do not edit generated macros directly.",
                " */",
                "",
                f"#ifndef {guard}",
                f"#define {guard}",
                "",
            ]
        )
        for inc in includes:
            lines.append(self.include_directive(inc))
        if includes:
            lines.append("")
        self._append_code_section(lines, schema, entity, "after_includes", "User code after includes", True)
        lines.extend(
            [
                "#ifdef __cplusplus",
                'extern "C"',
                "{",
                "#endif",
                "",
                "/**",
                f" * @defgroup sdpe_{entity.id} {entity.display_name or entity.id}",
                " * @{",
                " */",
                "",
                "// Identification",
                f"#define {prefix}_ID \"{entity.id}\"",
                f"#define {prefix}_SCHEMA \"{schema.id}\"",
                f"#define {prefix}_NAME \"{entity.display_name or entity.id}\"",
                "",
            ]
        )

        tags = sorted(dict.fromkeys([*schema.tags, *entity.tags]))
        if tags:
            lines.append(f"// Tags: {', '.join(tags)}")
            lines.append("")

        if schema.description or entity.description or entity.vendor or entity.datasheet_url or entity.document_url:
            lines.append("/*")
            if schema.description:
                self._append_block_comment_lines(lines, "Schema", schema.description)
            if entity.description:
                self._append_block_comment_lines(lines, "Entity", entity.description)
            if entity.vendor:
                lines.append(f" * Vendor: {entity.vendor}")
            if entity.datasheet_url:
                lines.append(f" * Datasheet: {entity.datasheet_url}")
            if entity.document_url:
                lines.append(f" * Document: {entity.document_url}")
            lines.append(" */")
            lines.append("")

        self._append_code_section(lines, schema, entity, "before_parameters", "User code before parameters")
        self._append_entity_config_macros(lines, schema, entity)
        self._append_parameter_macros(lines, entity, schema, prefix)
        self._append_derived_macros(lines, entity, schema, prefix)
        self._append_inline_components(lines, entity)
        self._append_component_overrides(lines, entity, schema)
        self._append_code_section(lines, schema, entity, "before_exports", "User code before exports")
        self._append_export_comments(lines, schema, prefix)
        self._append_code_section(lines, schema, entity, "before_footer", "User code before footer", True)

        lines.extend(
            [
                "/** @} */",
                "",
                "#ifdef __cplusplus",
                "}",
                "#endif",
                "",
                f"#endif // {guard}",
                "",
            ]
        )
        return "\n".join(lines)

    def _entity_includes(self, entity: HardwareEntity, schema: HardwareSchema, from_path: Path) -> list[str]:
        includes = [*schema.includes, *entity.includes]
        for comp in entity.components.values():
            if not comp.inline:
                includes.append(self.entity_include_path(comp.entity, from_path))
        return sorted(dict.fromkeys(includes))

    def _append_code_section(
        self,
        lines: list[str],
        schema: HardwareSchema,
        entity: HardwareEntity,
        name: str,
        title: str,
        placeholder_if_empty: bool = False,
    ) -> None:
        snippets: list[str] = []
        snippets.extend(self._code_section_values(schema.code_sections.get(name)))
        snippets.extend(self._code_section_values(entity.code_sections.get(name)))
        if not snippets:
            if placeholder_if_empty:
                lines.append(f"// {title}")
                lines.append(f"// SDPE extension point: add {name} code in the Entity Instance Code page if needed.")
                lines.append("")
            return
        lines.append(f"// {title}")
        for snippet in snippets:
            if snippet.strip():
                lines.extend(snippet.rstrip().splitlines())
        lines.append("")

    def _code_section_values(self, value: Any) -> list[str]:
        if value is None:
            return []
        if isinstance(value, str):
            return [value]
        if isinstance(value, list):
            return [str(item) for item in value if str(item).strip()]
        return [str(value)]

    def _append_doc_comment(self, lines: list[str], brief: str, unit: str = "") -> None:
        lines.append("/**")
        parts = str(brief).splitlines() or [""]
        for index, part in enumerate(parts):
            tag = "@brief " if index == 0 else "       "
            lines.append(f" * {tag}{part}")
        if unit:
            lines.append(f" * @unit {unit}")
        lines.append(" */")

    def _append_line_comment(self, lines: list[str], text: str) -> None:
        for part in str(text).splitlines():
            lines.append(f"// {part}")

    def _append_block_comment_lines(self, lines: list[str], label: str, text: str) -> None:
        for index, part in enumerate(str(text).splitlines() or [""]):
            prefix = f"{label}: " if index == 0 else " " * (len(label) + 2)
            lines.append(f" * {prefix}{part}")

    def _format_binding_text(self, value: str) -> str:
        text = str(value).strip()
        return re.sub(
            r"\$\{([A-Za-z0-9_][A-Za-z0-9_.]*)\}",
            lambda match: self._resolve_export(match.group(1)),
            text,
        )

    def _append_parameter_macros(
        self, lines: list[str], entity: HardwareEntity, schema: HardwareSchema, prefix: str
    ) -> None:
        if not schema.parameters:
            return
        lines.append("// Parameter macros")
        for pname, pspec in schema.parameters.items():
            found, value = self._entity_parameter_value(entity, pname, pspec.c_name)
            if found:
                pass
            elif pspec.default is not None:
                value = pspec.default
            elif pspec.required:
                raise SDPEError(f"Entity {entity.id} misses required parameter '{pname}'.")
            else:
                continue
            macro = f"{prefix}_{pspec.c_name}"
            desc = pspec.description or pname
            self._append_doc_comment(lines, desc, pspec.unit)
            lines.extend(
                [
                    f"#ifndef {macro}",
                    f"#define {macro} {self._format_parameter_value(value, pspec.value_format, entity, prefix)}",
                    f"#endif // {macro}",
                    "",
                ]
            )

    def _append_entity_config_macros(
        self, lines: list[str], schema: HardwareSchema, entity: HardwareEntity
    ) -> None:
        feature_macros = [*schema.feature_macros, *entity.feature_macros]
        option_macros = [*schema.option_macros, *entity.option_macros]

        if feature_macros:
            lines.append("// Entity selection macros")
            for item in feature_macros:
                macro = item.get("macro", "")
                if not macro:
                    continue
                if item.get("description"):
                    self._append_line_comment(lines, item["description"])
                value = f" {item['value']}" if item.get("value") else ""
                if item.get("enabled", True):
                    lines.append(f"#define {macro}{value}")
                else:
                    lines.append(f"// #define {macro}{value}")
            lines.append("")

        if option_macros:
            lines.append("// Entity option macros")
            for item in option_macros:
                macro = item.get("macro", "")
                if not macro:
                    continue
                options = self._option_macro_values(item, lambda preset: self._resolve_entity_option_preset(schema, entity, preset))
                if item.get("description") or options:
                    if item.get("description"):
                        self._append_line_comment(lines, item["description"])
                    if options:
                        self._append_line_comment(lines, f"Options: {', '.join(str(v) for v in options)}")
                value = str(item.get("value", ""))
                enabled = item.get("enabled", True)
                marker = "" if enabled else "// "
                lines.append(f"{marker}#define {macro} {value}")
                if enabled and re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", value):
                    lines.append(f"#define {macro}_{macro_name(value)} 1")
            lines.append("")

        if entity.conditional_macros:
            lines.append("// Entity conditional macros")
            for item in entity.conditional_macros:
                condition = item.get("condition") or item.get("condition_macro") or item.get("selector")
                if not condition:
                    continue
                lines.append(f"#if defined({condition})")
                for macro, value in item.get("macros", {}).items():
                    lines.append(f"#define {macro} {value}")
                lines.append(f"#endif // {condition}")
            lines.append("")

    def _append_derived_macros(
        self, lines: list[str], entity: HardwareEntity, schema: HardwareSchema, prefix: str
    ) -> None:
        if not schema.derived_macros:
            return
        lines.append("// Derived macros")
        for item in schema.derived_macros:
            macro = f"{prefix}_{item.name}"
            expr = self._format_expr(item.expr, prefix)
            self._append_doc_comment(lines, item.description or item.name, item.unit)
            lines.extend([f"#define {macro} {expr}", ""])

    def _append_inline_components(self, lines: list[str], entity: HardwareEntity) -> None:
        inline_components = [comp for comp in entity.components.values() if comp.inline]
        if not inline_components:
            return
        lines.append("// Inline component macros")
        for comp in inline_components:
            child_schema = self.library.schema(comp.entity.schema_id)
            lines.append(f"// Inline component: {comp.slot} ({child_schema.display_name})")
            self._append_parameter_macros(lines, comp.entity, child_schema, self.prefix(comp.entity, child_schema))
            self._append_derived_macros(lines, comp.entity, child_schema, self.prefix(comp.entity, child_schema))
            if comp.overrides:
                self._append_component_override_macros(lines, comp, entity, self.library.schema(entity.schema_id))

    def _append_component_overrides(
        self, lines: list[str], entity: HardwareEntity, schema: HardwareSchema
    ) -> None:
        if not entity.components:
            return
        override_components = [comp for comp in entity.components.values() if comp.overrides and not comp.inline]
        if not override_components:
            return
        lines.append("// Component local override macros")
        for comp in override_components:
            self._append_component_override_macros(lines, comp, entity, schema)

    def _append_component_override_macros(
        self, lines: list[str], comp: ComponentRef, parent: HardwareEntity, parent_schema: HardwareSchema
    ) -> None:
        child_schema = self.library.schema(comp.entity.schema_id)
        parent_prefix = self.prefix(parent, parent_schema)
        slot_prefix = f"{parent_prefix}_{macro_name(comp.slot)}"
        child_prefix = self.prefix(comp.entity, child_schema)
        lines.append(f"// Local overrides for {comp.slot}")
        for pname, pspec in child_schema.parameters.items():
            macro = f"{slot_prefix}_{pspec.c_name}"
            found_override, override_value = self._parameter_mapping_value(comp.overrides, pname, pspec.c_name)
            if found_override:
                value = self._format_parameter_value(override_value, pspec.value_format, parent, parent_prefix)
            else:
                target = f"{child_prefix}_{pspec.c_name}"
                value = target
            lines.append(f"#define {macro} {value}")
        for item in child_schema.derived_macros:
            macro = f"{slot_prefix}_{item.name}"
            expr = self._format_expr(item.expr, slot_prefix)
            lines.append(f"#define {macro} {expr}")
        lines.append("")

    def _append_export_comments(self, lines: list[str], schema: HardwareSchema, prefix: str) -> None:
        if not schema.exports:
            return
        lines.append("// Logical exports")
        for name, export in schema.exports.items():
            lines.append(f"// {name}: {self._format_expr(export.macro, prefix)}")
        lines.append("")

    def _format_expr(self, expr: str, prefix: str) -> str:
        return expr.replace("{PREFIX}", prefix)

    def _format_parameter_value(
        self, value: Any, value_format: str, entity: HardwareEntity, prefix: str
    ) -> str:
        if isinstance(value, dict):
            if "literal" in value:
                return str(value["literal"])
            if "macro" in value:
                return str(value["macro"])
            if "ref" in value:
                return self._resolve_entity_symbol(entity, prefix, str(value["ref"]))
            if "export" in value:
                return self._resolve_entity_symbol(entity, prefix, str(value["export"]))
            if "expr" in value:
                return self._format_local_expr(str(value["expr"]), entity, prefix)
            raise SDPEError(f"Unsupported parameter value object: {value!r}")
        if isinstance(value, str):
            match = re.fullmatch(r"\$\{([A-Za-z0-9_][A-Za-z0-9_.]*)\}", value.strip())
            if match:
                return self._resolve_entity_symbol(entity, prefix, match.group(1))
        return c_literal(value, value_format)

    def _format_local_expr(self, expr: str, entity: HardwareEntity, prefix: str) -> str:
        def replace(match: re.Match[str]) -> str:
            token = match.group(1)
            if token == "PREFIX":
                return prefix
            return self._resolve_entity_symbol(entity, prefix, token)

        return re.sub(r"\{([A-Za-z0-9_][A-Za-z0-9_.]*)\}", replace, expr)

    def prefix(self, entity: HardwareEntity, schema: HardwareSchema) -> str:
        """Resolve macro prefix for an entity."""

        if entity.macro_prefix:
            return entity.macro_prefix
        if schema.header_prefix:
            return f"{schema.header_prefix}_{macro_name(entity.id)}"
        return macro_name(entity.id)

    def _component_effective_prefix(
        self, parent: HardwareEntity, parent_schema: HardwareSchema, comp: ComponentRef
    ) -> str:
        child_schema = self.library.schema(comp.entity.schema_id)
        if not comp.overrides:
            return self.prefix(comp.entity, child_schema)
        return f"{self.prefix(parent, parent_schema)}_{macro_name(comp.slot)}"

    def generate_project(self, project_path: Path) -> list[GeneratedFile]:
        """Generate a project binding header."""

        data = read_json(project_path)
        generated: list[GeneratedFile] = []
        seen: set[Path] = set()
        included_entities = self._project_entity_ids(data)
        for entity_id in included_entities:
            for item in self.generate_entity_tree(entity_id, skip_system=True):
                if item.path not in seen:
                    seen.add(item.path)
                    generated.append(item)

        out_path = self.project_header_path(data)
        changed = write_if_changed(out_path, self.render_project_header(data))
        generated.append(GeneratedFile(out_path, changed))
        return generated

    def generate_project_matlab_script(self, project_path: Path) -> GeneratedFile:
        """Generate a MATLAB initialization script for project bindings."""

        data = read_json(project_path)
        out_path = self.project_matlab_script_path(data)
        changed = write_if_changed(out_path, self.render_project_matlab_script(data))
        return GeneratedFile(out_path, changed)

    def project_header_path(self, data: dict[str, Any]) -> Path:
        """Return the generated project header path."""

        header_name = data.get("output_header", "sdpe_project_bindings.h")
        if self.project_subdir:
            return self.out_dir / self.project_subdir / header_name
        return self.out_dir / header_name

    def project_matlab_script_path(self, data: dict[str, Any]) -> Path:
        """Return the generated MATLAB initialization script path."""

        header_name = Path(data.get("output_header", "sdpe_project_bindings.h"))
        script_name = f"{header_name.stem}_matlab_init.m"
        if self.project_subdir:
            return self.out_dir / self.project_subdir / script_name
        return self.out_dir / script_name

    def render_project_header(self, data: dict[str, Any]) -> str:
        """Render project requirement binding header."""

        project_id = data.get("id", "sdpe_project")
        guard = header_guard(f"project/{data.get('output_header', 'sdpe_project_bindings.h')}")
        hardware_ids = self._project_entity_ids(data)
        out_path = self.project_header_path(data)
        includes = [self.entity_include_path(self.library.entity(entity_id), out_path) for entity_id in hardware_ids]

        lines = [
            "/**",
            f" * @file {data.get('output_header', 'sdpe_project_bindings.h')}",
            f" * @brief SDPE project bindings for {data.get('display_name', project_id)}.",
        ]
        if data.get("description"):
            for index, part in enumerate(str(data["description"]).splitlines()):
                tag = "@note " if index == 0 else "      "
                lines.append(f" * {tag}{part}")
        lines.extend([" */", "", f"#ifndef {guard}", f"#define {guard}", ""])
        for inc in sorted(dict.fromkeys(includes)):
            lines.append(self.include_directive(inc))
        if includes:
            lines.append("")

        lines.extend(["#ifdef __cplusplus", 'extern "C"', "{", "#endif", ""])
        self._append_project_code_section(lines, data, "after_extern_open", "User project prefix code", True)

        self._append_project_section_header(lines, "Project metadata")
        lines.append(f"#define {self.project_metadata_macro(data, 'SDPE_PROJECT_ID')} \"{project_id}\"")
        if "suite" in data:
            lines.append(f"#define {self.project_metadata_macro(data, 'SDPE_PROJECT_SUITE')} \"{data['suite']}\"")
        if data.get("version"):
            lines.append(f"#define {self.project_metadata_macro(data, 'SDPE_PROJECT_VERSION')} \"{data['version']}\"")
        if data.get("updated_at"):
            lines.append(f"#define {self.project_metadata_macro(data, 'SDPE_PROJECT_UPDATED_AT')} \"{data['updated_at']}\"")
        lines.append("")

        for group, items in self._group_project_macros(data.get("feature_macros", []), "Selection macros"):
            self._append_project_section_header(lines, group)
            for item in items:
                macro = item.get("macro", "")
                if not macro:
                    continue
                desc = item.get("description", "")
                self._append_doc_comment(lines, desc or macro)
                value = f" {item['value']}" if item.get("value") else ""
                if item.get("enabled", True):
                    lines.append(f"#define {macro}{value}")
                else:
                    lines.append(f"// #define {macro}{value}")
                lines.append("")

        for group, items in self._group_project_macros(data.get("option_macros", []), "Option macros"):
            self._append_project_section_header(lines, group)
            for item in items:
                macro = item.get("macro", "")
                if not macro:
                    continue
                options = self._option_macro_values(item, lambda preset: self._resolve_project_option_preset(data, preset))
                desc = item.get("description", "")
                comment_lines = [desc or macro]
                if options:
                    comment_lines.append(f"Options: {', '.join(str(v) for v in options)}")
                self._append_doc_comment(lines, "\n".join(comment_lines))
                value = str(item.get("value", ""))
                enabled = item.get("enabled", True)
                marker = "" if enabled else "// "
                lines.append(f"{marker}#define {macro} {value}")
                lines.append("")

        self._append_project_section_header(lines, "Requirement bindings")
        for req in data.get("requirements", []):
            macro = req["macro"]
            value = self._resolve_binding_value(req["binding"])
            desc = req.get("description", req.get("role", macro))
            self._append_doc_comment(lines, desc)
            lines.extend([f"#define {macro} {value}", ""])

        if data.get("peripheral_bindings"):
            self._append_project_section_header(lines, "Board peripheral mapping")
            for macro, value in data["peripheral_bindings"].items():
                self._append_doc_comment(lines, macro)
                lines.append(f"#define {macro} {value}")
                lines.append("")

        if data.get("global_macros"):
            self._append_project_section_header(lines, "Global project macros")
            for macro, value in data["global_macros"].items():
                self._append_doc_comment(lines, macro)
                lines.append(f"#define {macro} {value}")
                lines.append("")

        self._append_project_code_section(lines, data, "before_footer", "User project tail code", True)
        lines.extend(["#ifdef __cplusplus", "}", "#endif", "", f"#endif // {guard}", ""])
        return "\n".join(lines)

    def render_project_matlab_script(self, data: dict[str, Any]) -> str:
        """Render a MATLAB script that mirrors project-visible SDPE macros."""

        project_id = data.get("id", "sdpe_project")
        lines = [
            "%% SDPE MATLAB initialization script",
            f"% Project: {data.get('display_name', project_id)}",
            "% Generated by tools/SDPE_v2. Do not edit generated variables directly.",
            "",
        ]
        if data.get("description"):
            lines.append("% Notes:")
            for part in str(data["description"]).splitlines():
                lines.append(f"%   {part}")
            lines.append("")

        emitted: set[str] = set()
        known_macros = self._project_matlab_macro_names(data)

        def emit(name: str, value: Any, description: str = "") -> None:
            macro = macro_name(name)
            if not macro or macro in emitted:
                return
            if description:
                for part in str(description).splitlines():
                    lines.append(f"% {part}")
            lines.append(f"{macro} = {self._matlab_value(value, known_macros)};")
            lines.append("")
            emitted.add(macro)

        lines.append("%% Project metadata")
        emit(self.project_metadata_macro(data, "SDPE_PROJECT_ID"), self._matlab_string(project_id))
        if "suite" in data:
            emit(self.project_metadata_macro(data, "SDPE_PROJECT_SUITE"), self._matlab_string(data["suite"]))
        if data.get("version"):
            emit(self.project_metadata_macro(data, "SDPE_PROJECT_VERSION"), self._matlab_string(data["version"]))
        if data.get("updated_at"):
            emit(self.project_metadata_macro(data, "SDPE_PROJECT_UPDATED_AT"), self._matlab_string(data["updated_at"]))

        hardware_ids = self._project_entity_ids(data)
        if hardware_ids:
            lines.append("%% Hardware macros")
            for entity_id in hardware_ids:
                self._append_entity_matlab_macros(lines, self.library.entity(entity_id), emitted, known_macros=known_macros)

        for group, items in self._group_project_macros(data.get("feature_macros", []), "Project selection macros"):
            lines.append(f"%% {group}")
            for item in items:
                macro = item.get("macro", "")
                if not macro:
                    continue
                value = item.get("value", "true") if item.get("value") else "true"
                if item.get("enabled", True):
                    emit(macro, value, item.get("description", ""))
                else:
                    lines.append(f"% {macro} is disabled in the SDPE project requirement.")
                    lines.append(f"% {macro} = {self._matlab_value(value, known_macros)};")
                    lines.append("")

        for group, items in self._group_project_macros(data.get("option_macros", []), "Project option macros"):
            lines.append(f"%% {group}")
            for item in items:
                macro = item.get("macro", "")
                if not macro:
                    continue
                value = str(item.get("value", ""))
                if item.get("enabled", True):
                    desc = item.get("description", "")
                    options = self._option_macro_values(item, lambda preset: self._resolve_project_option_preset(data, preset))
                    if options:
                        desc = f"{desc}\nOptions: {', '.join(str(v) for v in options)}".strip()
                    emit(macro, value, desc)
                else:
                    lines.append(f"% {macro} is disabled in the SDPE project requirement.")
                    lines.append(f"% {macro} = {self._matlab_value(value, known_macros)};")
                    lines.append("")

        lines.append("%% Requirement bindings")
        for req in data.get("requirements", []):
            macro = req.get("macro", "")
            if not macro:
                continue
            emit(macro, self._resolve_binding_value(req.get("binding", {})), req.get("description", req.get("role", macro)))

        lines.extend(
            [
                "%% Local helpers",
                "function value = sdpe_select(condition, true_value, false_value)",
                "if condition",
                "    value = true_value;",
                "else",
                "    value = false_value;",
                "end",
                "end",
                "",
            ]
        )
        return "\n".join(lines).rstrip() + "\n"

    def _append_entity_matlab_macros(
        self,
        lines: list[str],
        entity: HardwareEntity,
        emitted: set[str],
        seen: set[str] | None = None,
        known_macros: set[str] | None = None,
    ) -> None:
        """Append MATLAB variables for macros generated by an entity header."""

        if seen is None:
            seen = set()
        if entity.id in seen:
            return
        seen.add(entity.id)

        schema = self.library.schema(entity.schema_id)
        prefix = self.prefix(entity, schema)

        # A parent entity may export expressions that reference macros from one
        # of its component entities.  MATLAB evaluates a script sequentially,
        # unlike the C preprocessor, so emit component variables first.
        for comp in entity.components.values():
            self._append_entity_matlab_macros(lines, comp.entity, emitted, seen, known_macros)

        def emit(name: str, value: Any, description: str = "") -> None:
            macro = macro_name(name)
            if not macro or macro in emitted:
                return
            if description:
                for part in str(description).splitlines():
                    lines.append(f"% {part}")
            lines.append(f"{macro} = {self._matlab_value(value, known_macros)};")
            lines.append("")
            emitted.add(macro)

        emit(f"{prefix}_ID", self._matlab_string(entity.id))
        emit(f"{prefix}_SCHEMA", self._matlab_string(schema.id))
        emit(f"{prefix}_NAME", self._matlab_string(entity.display_name or entity.id))

        for item in [*schema.feature_macros, *entity.feature_macros]:
            macro = item.get("macro", "")
            if not macro:
                continue
            value = item.get("value", "true") if item.get("value") else "true"
            if item.get("enabled", True):
                emit(macro, value, item.get("description", ""))

        for item in [*schema.option_macros, *entity.option_macros]:
            macro = item.get("macro", "")
            if not macro or not item.get("enabled", True):
                continue
            value = str(item.get("value", ""))
            emit(macro, value, item.get("description", ""))
            if re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", value):
                emit(f"{macro}_{macro_name(value)}", "true")

        for pname, pspec in schema.parameters.items():
            found, value = self._entity_parameter_value(entity, pname, pspec.c_name)
            if not found:
                if pspec.default is None:
                    continue
                value = pspec.default
            formatted = self._format_parameter_value(value, pspec.value_format, entity, prefix)
            emit(f"{prefix}_{pspec.c_name}", formatted, pspec.description or pname)

        for item in schema.derived_macros:
            emit(f"{prefix}_{item.name}", self._format_expr(item.expr, prefix), item.description or item.name)

        for comp in entity.components.values():
            if comp.overrides:
                self._append_component_override_matlab_macros(lines, comp, entity, schema, emitted, known_macros)

    def _append_component_override_matlab_macros(
        self,
        lines: list[str],
        comp: ComponentRef,
        parent: HardwareEntity,
        parent_schema: HardwareSchema,
        emitted: set[str],
        known_macros: set[str] | None = None,
    ) -> None:
        child_schema = self.library.schema(comp.entity.schema_id)
        parent_prefix = self.prefix(parent, parent_schema)
        slot_prefix = f"{parent_prefix}_{macro_name(comp.slot)}"
        child_prefix = self.prefix(comp.entity, child_schema)

        def emit(name: str, value: Any, description: str = "") -> None:
            macro = macro_name(name)
            if not macro or macro in emitted:
                return
            if description:
                lines.append(f"% {description}")
            lines.append(f"{macro} = {self._matlab_value(value, known_macros)};")
            lines.append("")
            emitted.add(macro)

        for pname, pspec in child_schema.parameters.items():
            found_override, override_value = self._parameter_mapping_value(comp.overrides, pname, pspec.c_name)
            if found_override:
                value = self._format_parameter_value(override_value, pspec.value_format, parent, parent_prefix)
            else:
                value = f"{child_prefix}_{pspec.c_name}"
            emit(f"{slot_prefix}_{pspec.c_name}", value, pspec.description or pname)
        for item in child_schema.derived_macros:
                emit(f"{slot_prefix}_{item.name}", self._format_expr(item.expr, slot_prefix), item.description or item.name)

    def _project_matlab_macro_names(self, data: dict[str, Any]) -> set[str]:
        names = {self.project_metadata_macro(data, "SDPE_PROJECT_ID")}
        if "suite" in data:
            names.add(self.project_metadata_macro(data, "SDPE_PROJECT_SUITE"))
        if data.get("version"):
            names.add(self.project_metadata_macro(data, "SDPE_PROJECT_VERSION"))
        if data.get("updated_at"):
            names.add(self.project_metadata_macro(data, "SDPE_PROJECT_UPDATED_AT"))
        for entity_id in self._project_entity_ids(data):
            self._collect_entity_matlab_macro_names(self.library.entity(entity_id), names)
        for item in data.get("feature_macros", []):
            if item.get("macro") and item.get("enabled", True):
                names.add(macro_name(item["macro"]))
        for item in data.get("option_macros", []):
            if item.get("macro") and item.get("enabled", True):
                names.add(macro_name(item["macro"]))
        for req in data.get("requirements", []):
            if req.get("macro"):
                names.add(macro_name(req["macro"]))
        return names

    def _collect_entity_matlab_macro_names(
        self, entity: HardwareEntity, names: set[str], seen: set[str] | None = None
    ) -> None:
        if seen is None:
            seen = set()
        if entity.id in seen:
            return
        seen.add(entity.id)
        schema = self.library.schema(entity.schema_id)
        prefix = self.prefix(entity, schema)
        names.update(
            {
                f"{prefix}_ID",
                f"{prefix}_SCHEMA",
                f"{prefix}_NAME",
            }
        )
        for item in [*schema.feature_macros, *entity.feature_macros]:
            if item.get("macro") and item.get("enabled", True):
                names.add(macro_name(item["macro"]))
        for item in [*schema.option_macros, *entity.option_macros]:
            macro = item.get("macro", "")
            if not macro or not item.get("enabled", True):
                continue
            names.add(macro_name(macro))
            value = str(item.get("value", ""))
            if re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", value):
                names.add(f"{macro}_{macro_name(value)}")
        for pspec in schema.parameters.values():
            names.add(f"{prefix}_{pspec.c_name}")
        for item in schema.derived_macros:
            names.add(f"{prefix}_{item.name}")
        for comp in entity.components.values():
            if comp.overrides:
                child_schema = self.library.schema(comp.entity.schema_id)
                slot_prefix = f"{prefix}_{macro_name(comp.slot)}"
                for pspec in child_schema.parameters.values():
                    names.add(f"{slot_prefix}_{pspec.c_name}")
                for item in child_schema.derived_macros:
                    names.add(f"{slot_prefix}_{item.name}")
            self._collect_entity_matlab_macro_names(comp.entity, names, seen)

    def _matlab_string(self, value: Any) -> str:
        text = str(value).replace("'", "''")
        return f"'{text}'"

    def _matlab_value(self, value: Any, known_macros: set[str] | None = None) -> str:
        text = str(value).strip()
        if not text:
            return "[]"
        if text in {"true", "false"}:
            return text
        if text.startswith("'") and text.endswith("'"):
            return text
        if text.startswith('"') and text.endswith('"'):
            return self._matlab_string(text[1:-1])
        if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", text):
            lowered = text.lower()
            if lowered in {"true", "false", "nan", "inf"}:
                return lowered
            macro = macro_name(text)
            if known_macros is not None and macro not in known_macros:
                return self._matlab_string(text)
            return macro

        text = self._format_binding_text(text)
        text = re.sub(r"\b([0-9]+(?:\.[0-9]*)?|\.[0-9]+)([eE][+-]?[0-9]+)?[fFuUlL]*\b", r"\1\2", text)
        text = re.sub(r"\((?:float|double|int|uint16_t|uint32_t|ctrl_gt|parameter_gt)\)", "", text)
        text = text.replace("&&", " & ").replace("||", " | ")
        text = self._matlab_ternary(text)
        text = self._strip_balanced_outer_parentheses(text.strip())
        return text or "[]"

    def _matlab_ternary(self, text: str) -> str:
        if "?" not in text or ":" not in text:
            return text
        match = re.fullmatch(r"\s*(.+?)\s*\?\s*(.+?)\s*:\s*(.+)\s*", text)
        if not match:
            return text
        condition = self._normalize_ternary_part(match.group(1))
        true_value = self._normalize_ternary_part(match.group(2))
        false_value = self._normalize_ternary_part(match.group(3))
        return f"sdpe_select({condition}, {true_value}, {false_value})"

    def _normalize_ternary_part(self, text: str) -> str:
        value = text.strip()
        while value.startswith("(") and value.count("(") > value.count(")"):
            value = value[1:].strip()
        while value.endswith(")") and value.count(")") > value.count("("):
            value = value[:-1].strip()
        return self._strip_balanced_outer_parentheses(value)

    def _strip_balanced_outer_parentheses(self, text: str) -> str:
        while text.startswith("(") and text.endswith(")") and self._has_balanced_outer_parentheses(text):
            text = text[1:-1].strip()
        return text

    def _has_balanced_outer_parentheses(self, text: str) -> bool:
        depth = 0
        for index, char in enumerate(text):
            if char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0 and index != len(text) - 1:
                    return False
                if depth < 0:
                    return False
        return depth == 0

    def _append_project_section_header(self, lines: list[str], title: str) -> None:
        """Append a visually clear Doxygen section header."""

        if lines and lines[-1] != "":
            lines.append("")
        lines.extend(
            [
                "//=================================================================================================",
                "/**",
                f" * @brief {title}.",
                " */",
                "",
            ]
        )

    def _group_project_macros(self, items: list[dict[str, Any]], default_group: str) -> list[tuple[str, list[dict[str, Any]]]]:
        """Group project macros by the editable group label."""

        groups: dict[str, list[dict[str, Any]]] = {}
        for item in items:
            if not item.get("macro"):
                continue
            group = str(item.get("group") or default_group).strip() or default_group
            groups.setdefault(group, []).append(item)
        return list(groups.items())

    def _append_project_code_section(
        self, lines: list[str], data: dict[str, Any], name: str, title: str, placeholder_if_empty: bool = False
    ) -> None:
        snippets = self._code_section_values(data.get("code_sections", {}).get(name))
        if not snippets:
            if placeholder_if_empty:
                lines.append(f"// {title}")
                lines.append(f"// SDPE extension point: add {name} code in the Project Requirement Code page if needed.")
                lines.append("")
            return
        lines.append(f"// {title}")
        for snippet in snippets:
            if snippet.strip():
                lines.extend(snippet.rstrip().splitlines())
        lines.append("")

    def _resolve_entity_option_preset(
        self, schema: HardwareSchema, entity: HardwareEntity, preset: str
    ) -> list[Any]:
        key = str(preset).strip()
        if not key:
            return []
        candidates = [key]
        if "." in key:
            candidates.append(key.split(".", 1)[1])
        for store in (entity.option_sets, schema.option_sets):
            for candidate in candidates:
                if candidate in store:
                    return self._option_set_values(store[candidate])
        return []

    def _resolve_project_option_preset(self, data: dict[str, Any], preset: str) -> list[Any]:
        key = str(preset).strip()
        if not key:
            return []
        if "." in key:
            owner, name = key.split(".", 1)
            if owner in self.library.entity_files:
                entity = self.library.entity(owner)
                schema = self.library.schema(entity.schema_id)
                return self._resolve_entity_option_preset(schema, entity, name)
            if owner in self.library.schemas:
                schema = self.library.schema(owner)
                if name in schema.option_sets:
                    return self._option_set_values(schema.option_sets[name])
        for entity_id in self._project_entity_ids(data):
            entity = self.library.entity(entity_id)
            schema = self.library.schema(entity.schema_id)
            values = self._resolve_entity_option_preset(schema, entity, key)
            if values:
                return values
        for schema in self.library.schemas.values():
            if key in schema.option_sets:
                return self._option_set_values(schema.option_sets[key])
        return []

    def _option_set_values(self, value: Any) -> list[Any]:
        options = value.get("options", value) if isinstance(value, dict) else value
        return options if isinstance(options, list) else [options]

    def _option_macro_values(self, item: dict[str, Any], resolver) -> list[Any]:
        options = item.get("options", [])
        if len(options) == 1:
            match = re.fullmatch(r"\$\{([^}]+)\}", str(options[0]).strip())
            if match:
                return resolver(match.group(1))
        return options or resolver(item.get("options_preset", ""))

    def _project_entity_ids(self, data: dict[str, Any]) -> list[str]:
        ids = [item["entity"] for item in data.get("hardware", [])]
        for req in data.get("requirements", []):
            binding = req.get("binding")
            export_path = None
            if isinstance(binding, dict):
                export_path = binding.get("export")
                if binding.get("expr"):
                    for ref in re.findall(r"\$\{([A-Za-z0-9_][A-Za-z0-9_.]*)\}", str(binding["expr"])):
                        root_id = ref.split(".", 1)[0]
                        if root_id in self.library.entity_files:
                            ids.append(root_id)
            elif isinstance(binding, str) and "." in binding:
                export_path = binding
            if export_path:
                root_id = export_path.split(".", 1)[0]
                if root_id in self.library.entity_files:
                    ids.append(root_id)
        return list(dict.fromkeys(ids))

    def _resolve_binding_value(self, binding: Any) -> str:
        if isinstance(binding, dict):
            if "literal" in binding:
                return self._format_binding_text(str(binding["literal"]))
            if "macro" in binding:
                return self._format_binding_text(str(binding["macro"]))
            if "export" in binding:
                value = str(binding["export"])
                if value.strip().startswith("${"):
                    return self._format_binding_text(value)
                return self._resolve_export(value)
            if "expr" in binding:
                return self._format_binding_text(str(binding["expr"]))
            if "string" in binding:
                return '"' + str(binding["string"]).replace("\\", "\\\\").replace('"', '\\"') + '"'
            if "float" in binding:
                raw = str(binding["float"]).strip()
                return raw if raw.startswith("(") and raw.endswith(")") else f"({raw}f)"
            if "number" in binding:
                raw = str(binding["number"]).strip()
                return raw if raw.startswith("(") and raw.endswith(")") else f"({raw})"
        if isinstance(binding, str):
            binding = self._format_binding_text(binding)
            if "." in binding and not binding.startswith("${"):
                return self._resolve_export(binding)
            return binding
        raise SDPEError(f"Unsupported binding: {binding!r}")

    def _resolve_export(self, export_path: str) -> str:
        parts = export_path.split(".")
        if len(parts) < 2:
            raise SDPEError(f"Export binding must look like entity.export or entity.slot.export: {export_path}")
        entity = self.library.entity(parts[0])
        current = entity
        current_schema = self.library.schema(current.schema_id)
        current_prefix = self.prefix(current, current_schema)
        for slot in parts[1:-1]:
            if slot not in current.components:
                raise SDPEError(f"Entity {current.id} has no component slot '{slot}' in binding {export_path}")
            comp = current.components[slot]
            current_prefix = self._component_effective_prefix(current, self.library.schema(current.schema_id), comp)
            current = comp.entity
        export_name = parts[-1]
        schema = self.library.schema(current.schema_id)
        if export_name in schema.exports:
            export = schema.exports[export_name]
            return self._format_expr(export.macro, current_prefix)
        pspec = self._parameter_spec_by_symbol(schema, export_name)
        if pspec is not None:
            return f"{current_prefix}_{pspec.c_name}"
        for item in schema.derived_macros:
            if export_name == item.name or export_name.lower() == item.name.lower():
                return f"{current_prefix}_{item.name}"
        raise SDPEError(f"Schema {schema.id} has no export or parameter '{export_name}' in binding {export_path}")

    def _resolve_entity_symbol(self, entity: HardwareEntity, root_prefix: str, path: str) -> str:
        parts = path.split(".")
        if not parts:
            raise SDPEError(f"Empty entity symbol path for {entity.id}")
        current = entity
        current_prefix = root_prefix
        for slot in parts[:-1]:
            if slot not in current.components:
                raise SDPEError(f"Entity {current.id} has no component slot '{slot}' in symbol {path}")
            comp = current.components[slot]
            current_prefix = self._component_effective_prefix(current, self.library.schema(current.schema_id), comp)
            current = comp.entity
        name = parts[-1]
        schema = self.library.schema(current.schema_id)
        if name in schema.exports:
            return self._format_expr(schema.exports[name].macro, current_prefix)
        pspec = self._parameter_spec_by_symbol(schema, name)
        if pspec is not None:
            return f"{current_prefix}_{pspec.c_name}"
        for item in schema.derived_macros:
            if name == item.name or name.lower() == item.name.lower():
                return f"{current_prefix}_{item.name}"
        raise SDPEError(f"Schema {schema.id} has no export, parameter, or derived macro '{name}' in symbol {path}")

    def _entity_parameter_value(self, entity: HardwareEntity, pname: str, c_name: str) -> tuple[bool, Any]:
        return self._parameter_mapping_value(entity.parameters, pname, c_name)

    def _parameter_mapping_value(self, values: dict[str, Any], pname: str, c_name: str) -> tuple[bool, Any]:
        if pname in values:
            return True, values[pname]
        c_key = c_name.lower()
        if c_key in values:
            return True, values[c_key]
        normalized = c_key.replace("__", "_")
        for key, value in values.items():
            if key.lower() == normalized:
                return True, value
        return False, None

    def _parameter_spec_by_symbol(self, schema: HardwareSchema, name: str):
        if name in schema.parameters:
            return schema.parameters[name]
        key = name.lower()
        for pspec in schema.parameters.values():
            if pspec.c_name.lower() == key:
                return pspec
        return None
