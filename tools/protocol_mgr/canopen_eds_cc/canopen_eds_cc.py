#!/usr/bin/env python3
"""Compile CANopen dictionaries between EDS, JSON and GMP generated C sources."""

from __future__ import annotations

import argparse
import configparser
import dataclasses
import json
import math
import pathlib
import re
import sys
from typing import Iterable


SECTION_RE = re.compile(r"^([0-9A-Fa-f]{4})(?:sub([0-9A-Fa-f]{1,2}))?$")
ID_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")

JSON_VERSION = 1
CONFLICT_POLICIES = {"error", "first", "last"}

TYPE_INFO = {
    0x0001: ("GMP_CANOPEN_OD_BOOLEAN", "boolean", "fast_gt", 1, "1"),
    0x0002: ("GMP_CANOPEN_OD_INTEGER8", "i8", "int_least8_t", 1, "-128..127"),
    0x0003: ("GMP_CANOPEN_OD_INTEGER16", "i16", "int16_t", 2, "-32768..32767"),
    0x0004: ("GMP_CANOPEN_OD_INTEGER32", "i32", "int32_t", 4, "-2147483648..2147483647"),
    0x0005: ("GMP_CANOPEN_OD_UNSIGNED8", "u8", "byte_gt", 1, "0..255"),
    0x0006: ("GMP_CANOPEN_OD_UNSIGNED16", "u16", "uint16_t", 2, "0..65535"),
    0x0007: ("GMP_CANOPEN_OD_UNSIGNED32", "u32", "uint32_t", 4, "0..4294967295"),
    0x0008: ("GMP_CANOPEN_OD_REAL32", "real32", "float", 4, "float"),
    0x0009: ("GMP_CANOPEN_OD_VISIBLE_STRING", "octets", "byte_gt", 0, "variable"),
    0x000A: ("GMP_CANOPEN_OD_OCTET_STRING", "octets", "byte_gt", 0, "variable"),
    0x000F: ("GMP_CANOPEN_OD_DOMAIN", "octets", "byte_gt", 0, "variable"),
    0x0011: ("GMP_CANOPEN_OD_REAL64", "real64", "double", 8, "double"),
    0x0015: ("GMP_CANOPEN_OD_INTEGER64", "i64", "int64_t", 8, "-2^63..2^63-1"),
    0x001B: ("GMP_CANOPEN_OD_UNSIGNED64", "u64", "uint64_t", 8, "0..2^64-1"),
}

TYPE_NAME_TO_CODE = {
    "BOOLEAN": 0x0001,
    "INTEGER8": 0x0002,
    "INTEGER16": 0x0003,
    "INTEGER32": 0x0004,
    "UNSIGNED8": 0x0005,
    "UNSIGNED16": 0x0006,
    "UNSIGNED32": 0x0007,
    "REAL32": 0x0008,
    "VISIBLE_STRING": 0x0009,
    "OCTET_STRING": 0x000A,
    "DOMAIN": 0x000F,
    "REAL64": 0x0011,
    "INTEGER64": 0x0015,
    "UNSIGNED64": 0x001B,
}
for name in TYPE_INFO:
    TYPE_NAME_TO_CODE[TYPE_INFO[name][0]] = name

ACCESS_INFO = {
    "ro": "GMP_CANOPEN_OD_ACCESS_READ",
    "wo": "GMP_CANOPEN_OD_ACCESS_WRITE",
    "rw": "GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE",
    "rwr": "GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE",
    "rww": "GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE",
    "const": "GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST",
}

INTEGER_RANGES = {
    0x0001: (0, 1),
    0x0002: (-128, 127),
    0x0003: (-32768, 32767),
    0x0004: (-2147483648, 2147483647),
    0x0005: (0, 0xFF),
    0x0006: (0, 0xFFFF),
    0x0007: (0, 0xFFFFFFFF),
    0x0015: (-9223372036854775808, 9223372036854775807),
    0x001B: (0, 0xFFFFFFFFFFFFFFFF),
}

SCALAR_STORAGES = {"pointer", "value", "variable"}


def parse_access(raw: object) -> str:
    """Validate CANopen access mode."""
    access = str(raw).strip().lower()
    if access not in ACCESS_INFO:
        raise ValueError(f"unsupported access type {access!r}")
    return access


@dataclasses.dataclass(frozen=True)
class DictionaryEntry:
    index: int
    subindex: int
    parameter_name: str
    data_type: int
    access: str
    pdo_mapping: bool
    default_value: str
    size: int
    storage: str
    source_file: str | None = None
    variable_name: str | None = None
    group_path: tuple[str, ...] = dataclasses.field(default_factory=tuple)
    comment: str | None = None

    @property
    def symbol_suffix(self) -> str:
        return f"{self.index:04x}_{self.subindex:02x}"


@dataclasses.dataclass(frozen=True)
class ImportSpec:
    path: str
    alias: str
    storage: str | None = None
    node_id: int | None = None


@dataclasses.dataclass(frozen=True)
class DictionaryCodeTemplate:
    header_prefix: str = ""
    header_suffix: str = ""
    source_prefix: str = ""
    source_suffix: str = ""


@dataclasses.dataclass(frozen=True)
class DictionaryProject:
    version: int
    name: str
    node_id: int
    default_storage: str
    conflict_policy: str
    imports: list[ImportSpec]
    entries: list[DictionaryEntry]
    code_template: DictionaryCodeTemplate
    output_dir: str | None = None


def _strip_or_zero(value: str, default: str = "0") -> str:
    text = (value or default).strip()
    return text if text else default


def parse_int_expression(text: str, node_id: int = 1) -> int:
    value = text.strip().replace("$NODEID", str(node_id)).replace("$NodeID", str(node_id))
    value = value.replace("(", "").replace(")", "")
    if not re.fullmatch(r"[0-9A-Fa-fxX+\-\s]+", value):
        raise ValueError(f"unsupported integer expression: {text!r}")
    total = 0
    sign = 1
    for token in re.split(r"([+-])", value):
        token = token.strip()
        if not token:
            continue
        if token == "+":
            sign = 1
        elif token == "-":
            sign = -1
        else:
            total += sign * int(token, 16 if token.lower().startswith("0x") else 10)
    return total


def parse_int(text: str, node_id: int = 1) -> int:
    """Backward-compatible helper alias for `parse_int_expression`."""
    return parse_int_expression(text, node_id)


def parse_bool(raw: object) -> bool:
    if isinstance(raw, bool):
        return raw
    return str(raw).strip().lower() in {"1", "true", "yes", "on"}


def parse_storage(raw: object | None, default_storage: str) -> str:
    storage = str(raw).strip().lower() if raw is not None else default_storage
    if storage not in SCALAR_STORAGES:
        raise ValueError(f"unsupported storage mode {storage!r}")
    return storage


def parse_data_type(raw: object) -> int:
    if isinstance(raw, int):
        data_type = raw
    else:
        text = _strip_or_zero(str(raw), "")
        if text in TYPE_NAME_TO_CODE:
            data_type = TYPE_NAME_TO_CODE[text]
        else:
            data_type = parse_int_expression(text)
    if data_type not in TYPE_INFO:
        raise ValueError(f"unsupported DataType 0x{data_type:04X}")
    return data_type


def parse_index(raw: object, bits: int, label: str) -> int:
    if raw is None:
        raise ValueError(f"{label} is required")
    if isinstance(raw, int):
        value = raw
    else:
        text = str(raw).strip().lower()
        if text.startswith("0x"):
            value = parse_int_expression(text, 1)
        else:
            value = int(text, 10) if text else 0
    if value < 0 or value >= (1 << bits):
        raise ValueError(f"{label} out of range: {value}")
    return value


def c_identifier(text: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_]", "_", text)
    result = re.sub(r"_+", "_", result).strip("_")
    if not result or result[0].isdigit():
        result = "od_" + result
    return result.lower()


def c_string(text: str) -> str:
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def doxygen_text(text: str) -> str:
    return text.replace("*/", "* /").replace("\r", " ").replace("\n", " ").strip()


def unquote_text(text: str) -> str:
    raw = text.strip()
    if len(raw) >= 2 and raw[0] == raw[-1] and raw[0] in {'"', "'"}:
        return raw[1:-1]
    return raw


def raw_bytes(entry: DictionaryEntry) -> bytes:
    if entry.data_type == 0x0009:
        payload = unquote_text(entry.default_value).encode("utf-8")
    elif entry.data_type in {0x000A, 0x000F}:
        compact = re.sub(r"[\s,:_-]", "", unquote_text(entry.default_value))
        if compact:
            if (len(compact) % 2 != 0 or re.fullmatch(r"[0-9A-Fa-f]+", compact) is None):
                raise ValueError(f"0x{entry.index:04X}:{entry.subindex:02X} has invalid hex payload")
        payload = bytes.fromhex(compact) if compact else b""
    else:
        raise AssertionError("not a raw data type")
    if len(payload) > entry.size:
        raise ValueError(f"0x{entry.index:04X}:{entry.subindex:02X} default exceeds DataLength")
    return payload + bytes(entry.size - len(payload))


def numeric_literal(entry: DictionaryEntry, node_id: int) -> str:
    enum_name, _field, c_type, fixed_size, _ = TYPE_INFO[entry.data_type]
    if entry.data_type in {0x0008, 0x0011}:
        value = float(_strip_or_zero(entry.default_value))
        if not math.isfinite(value):
            raise ValueError(f"0x{entry.index:04X}:{entry.subindex:02X} default must be finite")
        suffix = "F" if entry.data_type == 0x0008 else ""
        return f"({c_type}){value:.17g}{suffix}"
    parsed = parse_int_expression(_strip_or_zero(entry.default_value), node_id)
    minimum, maximum = INTEGER_RANGES[entry.data_type]
    if parsed < minimum or parsed > maximum:
        raise ValueError(f"0x{entry.index:04X}:{entry.subindex:02X} default is outside range of data type")
    return f"({c_type}){parsed}"


def access_expression(entry: DictionaryEntry) -> str:
    expression = ACCESS_INFO[entry.access]
    if entry.pdo_mapping:
        expression += " | GMP_CANOPEN_OD_ACCESS_PDO"
    return expression


def _entry_storage_default(raw: object, default_storage: str) -> str:
    return parse_storage(raw, default_storage)


def load_eds(
    path: pathlib.Path,
    default_storage: str,
    node_id: int,
    *,
    source_file: str | None = None,
    group_path: tuple[str, ...] = tuple(),
) -> list[DictionaryEntry]:
    parser = configparser.ConfigParser(interpolation=None, strict=True)
    parser.optionxform = str
    with path.open("r", encoding="utf-8-sig") as stream:
        parser.read_file(stream)
    entries: list[DictionaryEntry] = []
    for section_name in parser.sections():
        match = SECTION_RE.fullmatch(section_name)
        if not match:
            continue
        section = {key.lower(): value for key, value in parser[section_name].items()}
        if "datatype" not in section:
            continue
        data_type = parse_data_type(section["datatype"])
        index = int(match.group(1), 16)
        subindex = int(match.group(2), 16) if match.group(2) else 0
        access = section.get("accesstype", "ro").strip().lower()
        access = parse_access(access)
        default_value = section.get("parametervalue", section.get("defaultvalue", "0"))
        pdo_mapping = parse_bool(section.get("pdomapping", "0"))
        fixed_size = TYPE_INFO[data_type][3]
        if fixed_size:
            size = fixed_size
        else:
            size_text = section.get("datalength", "")
            if size_text:
                size = parse_int_expression(size_text, node_id)
            else:
                size = max(1, len(unquote_text(default_value).encode("utf-8")))
        if size <= 0:
            raise ValueError(f"[{section_name}] object size must be positive")
        storage = parse_storage(section.get("gmpstorage"), default_storage)
        variable_name = _strip_or_zero(section.get("gmpstoragename", ""), "")
        if storage == "variable" and not variable_name:
            raise ValueError(f"[{section_name}] GMPStorage=variable requires GMPStorageVariable")
        entry = DictionaryEntry(
            index=index,
            subindex=subindex,
            parameter_name=section.get("parametername", f"0x{index:04X}:{subindex:02X}"),
            data_type=data_type,
            access=access,
            pdo_mapping=pdo_mapping,
            default_value=default_value,
            size=size,
            storage=storage,
            source_file=str(path),
            variable_name=variable_name or None,
            group_path=group_path,
        )
        if storage == "value" and fixed_size == 0 and size > 8:
            raise ValueError(f"[{section_name}] value storage exceeds 8 bytes")
        if fixed_size:
            numeric_literal(entry, node_id)
        else:
            raw_bytes(entry)
        entries.append(entry)
    entries.sort(key=lambda item: (item.index, item.subindex))
    keys = [(item.index, item.subindex) for item in entries]
    if len(keys) != len(set(keys)):
        raise ValueError(f"{path} contains duplicate index/subindex sections")
    if not entries:
        raise ValueError(f"{path} does not contain any supported object entry")
    return entries


def parse_json_entry(
    item: dict,
    default_storage: str,
    node_id: int,
    inherited_group: tuple[str, ...] = tuple(),
) -> DictionaryEntry:
    if not isinstance(item, dict):
        raise ValueError("JSON entry item must be an object")
    index = parse_index(item.get("index"), bits=16, label="index")
    subindex = parse_index(item.get("subindex", "0"), bits=8, label="subindex")
    data_type = parse_data_type(item.get("data_type", item.get("datatype")))
    access = str(item.get("access", "ro")).strip().lower()
    if access not in ACCESS_INFO:
        raise ValueError(f"0x{index:04X}:{subindex:02X} unsupported AccessType {access!r}")
    pdo_mapping = parse_bool(item.get("pdo_mapping", item.get("pdomapping", "0")))
    storage = parse_storage(item.get("storage"), default_storage)
    variable_name = _strip_or_zero(str(item.get("variable_name", item.get("variable", ""))), "")
    variable_name = variable_name or None
    fixed_size = TYPE_INFO[data_type][3]
    size = parse_int_expression(str(item.get("size", "0")), node_id) if item.get("size") is not None else 0
    if fixed_size:
        if size in (0, fixed_size):
            size = fixed_size
        else:
            raise ValueError(f"0x{index:04X}:{subindex:02X} fixed data type has invalid size override")
    else:
        if size <= 0:
            raise ValueError(f"0x{index:04X}:{subindex:02X} requires size for variable-length data")
    if storage == "variable" and not variable_name:
        raise ValueError(f"0x{index:04X}:{subindex:02X} storage=variable requires variable_name")
    default_value = str(item.get("default_value", item.get("default", "0")))
    entry = DictionaryEntry(
        index=index,
        subindex=subindex,
        parameter_name=str(item.get("parameter_name", item.get("name", f"0x{index:04X}:{subindex:02X}"))),
        data_type=data_type,
        access=access,
        pdo_mapping=pdo_mapping,
        default_value=default_value,
        size=size,
        storage=storage,
        variable_name=variable_name,
        group_path=tuple(inherited_group + tuple(item.get("group_path", []) or [])),
        comment=str(item.get("comment", "")) if item.get("comment") is not None else None,
    )
    if fixed_size:
        numeric_literal(entry, node_id)
    else:
        raw_bytes(entry)
    return entry


def parse_json_tree_nodes(
    nodes: list[dict],
    default_storage: str,
    node_id: int,
    inherited_group: tuple[str, ...] = tuple(),
) -> list[DictionaryEntry]:
    parsed: list[DictionaryEntry] = []
    for node in nodes:
        if not isinstance(node, dict):
            raise ValueError("JSON tree node must be object")
        if "children" in node and isinstance(node["children"], list):
            group_name = str(node.get("name", "")).strip()
            if not group_name:
                raise ValueError("JSON group requires non-empty name")
            parsed.extend(parse_json_tree_nodes(
                node["children"],
                default_storage,
                node_id,
                inherited_group=inherited_group + (group_name,),
            ))
            continue
        if node.get("type", "").lower() == "group":
            group_name = str(node.get("name", "")).strip()
            if not group_name:
                raise ValueError("JSON group requires non-empty name")
            parsed.extend(parse_json_tree_nodes(
                node.get("children", []),
                default_storage,
                node_id,
                inherited_group=inherited_group + (group_name,),
            ))
            continue
        parsed.append(parse_json_entry(node, default_storage, node_id, inherited_group))
    return parsed


def merge_entry(target: dict[tuple[int, int], DictionaryEntry], entry: DictionaryEntry, policy: str) -> None:
    key = (entry.index, entry.subindex)
    if key not in target:
        target[key] = entry
        return
    if policy == "error":
        existing = target[key]
        raise ValueError(
            f"duplicate index 0x{entry.index:04X}:{entry.subindex:02X}, "
            f"already from {existing.source_file or 'project'} and {entry.source_file or 'project'}")
    if policy == "first":
        return
    target[key] = entry


def load_project_json(path: pathlib.Path) -> DictionaryProject:
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, dict):
        raise ValueError(f"project file {path} must be a JSON object")

    version = int(payload.get("version", JSON_VERSION))
    if version > JSON_VERSION:
        raise ValueError(f"unsupported project schema version {version}")

    name = str(payload.get("name", path.stem)).strip()
    if not name:
        raise ValueError("project name cannot be empty")

    node_id = int(payload.get("node_id", payload.get("node-id", 1)))
    if not 1 <= node_id <= 127:
        raise ValueError("node_id must be in range 1..127")

    default_storage = parse_storage(payload.get("default_storage"), "pointer")
    conflict_policy = str(payload.get("conflict_policy", "last")).strip().lower()
    if conflict_policy not in CONFLICT_POLICIES:
        raise ValueError(f"conflict_policy must be one of {sorted(CONFLICT_POLICIES)}")

    code = payload.get("code", {}) or {}
    if not isinstance(code, dict):
        raise ValueError("project.code must be an object")

    import_items = payload.get("imports", []) or []
    imports: list[ImportSpec] = []
    for item in import_items:
        if not isinstance(item, dict):
            raise ValueError("project.imports item must be an object")
        import_path = str(item.get("path", "")).strip()
        if not import_path:
            raise ValueError("import.path cannot be empty")
        alias = str(item.get("alias", pathlib.Path(import_path).stem)).strip()
        if not alias:
            alias = pathlib.Path(import_path).stem
        imports.append(ImportSpec(
            path=import_path,
            alias=alias,
            storage=parse_storage(item.get("storage"), default_storage) if item.get("storage") is not None else None,
            node_id=int(item["node_id"]) if item.get("node_id") is not None else
            int(item["node-id"]) if item.get("node-id") is not None else None,
        ))

    manual_entries: list[DictionaryEntry] = []
    tree_nodes = payload.get("tree", [])
    if tree_nodes:
        manual_entries.extend(parse_json_tree_nodes(item_list_assert(tree_nodes), default_storage, node_id))
    for item in payload.get("entries", []) or []:
        manual_entries.append(parse_json_entry(item, default_storage, node_id, tuple(_coerce_group_path(item.get("group_path", [])))))

    merged: dict[tuple[int, int], DictionaryEntry] = {}
    root = path.parent
    for item in imports:
        import_path = pathlib.Path(item.path)
        if not import_path.is_absolute():
            import_path = root / import_path
        imported = load_eds(
            import_path,
            item.storage or default_storage,
            node_id=item.node_id or node_id,
            source_file=str(import_path),
            group_path=(item.alias,),
        )
        for entry in imported:
            merge_entry(merged, entry, conflict_policy)

    for entry in manual_entries:
        merge_entry(merged, entry, conflict_policy)

    return DictionaryProject(
        version=version,
        name=name,
        node_id=node_id,
        default_storage=default_storage,
        conflict_policy=conflict_policy,
        imports=imports,
        entries=sorted(merged.values(), key=lambda item: (item.index, item.subindex)),
        code_template=DictionaryCodeTemplate(
            header_prefix=str(code.get("header_prefix", "")),
            header_suffix=str(code.get("header_suffix", "")),
            source_prefix=str(code.get("source_prefix", "")),
            source_suffix=str(code.get("source_suffix", "")),
        ),
        output_dir=payload.get("output_dir"),
    )


def item_list_assert(raw: object) -> list[dict]:
    if not isinstance(raw, list):
        raise ValueError("tree node list must be array")
    nodes: list[dict] = []
    for item in raw:
        if not isinstance(item, dict):
            raise ValueError("tree node item must be object")
        nodes.append(item)
    return nodes


def _coerce_group_path(raw: object) -> list[str]:
    if raw is None:
        return []
    if isinstance(raw, str):
        return [segment for segment in (s.strip() for s in raw.split("/")) if segment]
    if isinstance(raw, list):
        values: list[str] = []
        for item in raw:
            text = str(item).strip()
            if text:
                values.append(text)
        return values
    raise ValueError("group_path must be string or string array")


def generate_header(entries: list[DictionaryEntry], api_name: str, header_name: str,
                    code_prefix: str, code_suffix: str) -> str:
    guard = f"_FILE_{api_name.upper()}_H_"
    storage_declarations = []
    for entry in entries:
        if entry.storage != "pointer" or entry.variable_name is not None:
            continue
        _, _, c_type, fixed_size, _ = TYPE_INFO[entry.data_type]
        symbol = f"{api_name}_storage_{entry.symbol_suffix}"
        comment = (
            f"/** @brief Storage for 0x{entry.index:04X}:{entry.subindex:02X} "
            f"({doxygen_text(entry.parameter_name)}). */"
        )
        if fixed_size:
            storage_declarations.append(f"{comment}\nextern {c_type} {symbol};")
        else:
            storage_declarations.append(f"{comment}\nextern byte_gt {symbol}[{entry.size}U];")
    storage_text = "\n".join(storage_declarations)
    body = f"""/** @file {header_name} @brief Generated from an EDS file; do not edit. */
#ifndef {guard}
#define {guard}

#include <core/protocol/canopen/od.h>

#ifdef __cplusplus
extern "C"
{{
#endif

/** @brief Generated OD entry count. */
#define {api_name.upper()}_ENTRY_COUNT {len(entries)}U

/** @brief Generated intrusive entry storage. */
extern gmp_canopen_od_entry_t {api_name}_entries[{api_name.upper()}_ENTRY_COUNT];
{storage_text}
/**
 * @brief Initialize the generated object dictionary exactly once.
 * @param dictionary Empty dictionary that receives all generated entries.
 * @return Non-zero when every entry is inserted and the RB tree validates.
 */
fast_gt {api_name}_init(gmp_canopen_od_t* dictionary);

#ifdef __cplusplus
}}
#endif

#endif /* {guard} */
"""
    if code_prefix:
        body = f"{code_prefix}\n\n{body}"
    if code_suffix:
        body = f"{body}\n\n{code_suffix}"
    return body


def variable_pointer(entry: DictionaryEntry) -> str:
    if entry.variable_name is None:
        return ""
    variable = entry.variable_name.strip()
    if not variable:
        return ""
    if ID_RE.fullmatch(variable):
        return f"&({variable})"
    return variable


def generate_source(entries: list[DictionaryEntry], api_name: str,
                    header_name: str, source_eds_name: str,
                    node_id: int,
                    code_prefix: str,
                    code_suffix: str) -> str:
    lines = [f"/** @file {api_name}.c @brief Generated from {source_eds_name}; do not edit. */"]
    if code_prefix:
        lines.append(code_prefix)
    lines.extend([f'#include "{header_name}"', ""])
    lines.append(f"gmp_canopen_od_entry_t {api_name}_entries[{api_name.upper()}_ENTRY_COUNT];")
    for entry in entries:
        if entry.storage == "value" or entry.variable_name is not None:
            continue
        _, _, c_type, fixed_size, _ = TYPE_INFO[entry.data_type]
        symbol = f"{api_name}_storage_{entry.symbol_suffix}"
        if fixed_size:
            lines.append(f"{c_type} {symbol} = {numeric_literal(entry, node_id)};")
        else:
            payload = raw_bytes(entry)
            values = ", ".join(f"0x{value:02X}U" for value in payload)
            lines.append(f"byte_gt {symbol}[{entry.size}U] = {{{values}}};")
    lines.append("")
    lines.append(f"fast_gt {api_name}_init(gmp_canopen_od_t* dictionary)")
    lines.append("{")
    lines.append("    size_gt entry_index;")
    lines.append("    if (dictionary == NULL)")
    lines.append("        return 0;")
    lines.append(f"    for (entry_index = 0U; entry_index < {api_name.upper()}_ENTRY_COUNT; ++entry_index)")
    lines.append(f"        if (!gmp_rb_node_is_detached(&{api_name}_entries[entry_index].rb_node))")
    lines.append("            return 0;")
    lines.append("    gmp_canopen_od_init(dictionary);")

    for ordinal, entry in enumerate(entries):
        enum_name, _field, c_type, fixed_size, _ = TYPE_INFO[entry.data_type]
        access = access_expression(entry)
        name = c_string(entry.parameter_name)
        if entry.storage != "value":
            pointer_target = variable_pointer(entry) if entry.variable_name else f"&{api_name}_storage_{entry.symbol_suffix}"
            lines.append(f"    gmp_canopen_od_entry_init_pointer(&{api_name}_entries[{ordinal}U],")
            lines.append(f"        0x{entry.index:04X}U, 0x{entry.subindex:02X}U, {enum_name},")
            lines.append(f"        {access}, {pointer_target}, {entry.size}U, {name});")
        else:
            lines.append(f"    gmp_canopen_od_entry_init_value(&{api_name}_entries[{ordinal}U],")
            lines.append(f"        0x{entry.index:04X}U, 0x{entry.subindex:02X}U, {enum_name},")
            lines.append(f"        {access}, {entry.size}U, {name});")
            if fixed_size:
                lines.append(f"    {api_name}_entries[{ordinal}U].storage.value.{_field} = {numeric_literal(entry, node_id)};")
            else:
                payload = raw_bytes(entry)
                for offset, byte_value in enumerate(payload):
                    lines.append(
                        f"    {api_name}_entries[{ordinal}U].storage.value.octets[{offset}U] = "
                        f"0x{byte_value:02X}U;")
        lines.append(f"    if (gmp_canopen_od_insert(dictionary, &{api_name}_entries[{ordinal}U]) != GMP_CANOPEN_OD_OK)")
        lines.append("        return 0;")
    lines.extend([
        f"    return gmp_canopen_od_validate(dictionary, {api_name.upper()}_ENTRY_COUNT);",
        "}",
    ])
    if code_suffix:
        lines.append("")
        lines.append(code_suffix)
    return "\n".join(lines) + "\n"


def generate_eds(entries: list[DictionaryEntry], node_id: int) -> str:
    lines = [
        "[FileInfo]",
        "FileName=generated.eds",
        "FileVersion=1",
        "FileRevision=0",
        "EDSVersion=4.0",
        "",
    ]
    for entry in entries:
        lines.append(f"[{entry.index:04X}]" + (f"sub{entry.subindex:02X}" if entry.subindex else ""))
        lines.append(f"ParameterName={entry.parameter_name}")
        lines.append("ObjectType=0x7")
        lines.append(f"DataType=0x{entry.data_type:04X}")
        lines.append(f"AccessType={entry.access}")
        if entry.data_type in {0x0008, 0x0011}:
            text = _strip_or_zero(entry.default_value)
            if not re.fullmatch(r"[+-]?\d+(\.\d+)?([eE][+-]?\d+)?", text):
                parsed = parse_int_expression(text, node_id)
                text = str(parsed)
            lines.append(f"DefaultValue={text}")
        elif TYPE_INFO[entry.data_type][3] == 0:
            if entry.data_type == 0x0009:
                lines.append(f"DefaultValue={unquote_text(entry.default_value)}")
            else:
                lines.append(f"DefaultValue={''.join(f'{value:02X}' for value in raw_bytes(entry))}")
        else:
            lines.append(f"DefaultValue={parse_int_expression(_strip_or_zero(entry.default_value), node_id)}")
        lines.append(f"PDOMapping={1 if entry.pdo_mapping else 0}")
        if TYPE_INFO[entry.data_type][3] == 0:
            lines.append(f"DataLength={entry.size}")
        if entry.storage != "pointer":
            lines.append(f"GMPStorage={entry.storage}")
            if entry.storage == "variable":
                lines.append(f"GMPStorageVariable={entry.variable_name}")
        lines.append("")
    return "\n".join(lines).strip() + "\n"


def compile_entries(
    entries: list[DictionaryEntry],
    output_dir: pathlib.Path,
    name: str,
    node_id: int,
    source_eds: pathlib.Path,
    header_prefix: str = "",
    header_suffix: str = "",
    source_prefix: str = "",
    source_suffix: str = "",
    emit_eds: pathlib.Path | None = None,
) -> tuple[pathlib.Path, pathlib.Path] | tuple[pathlib.Path, pathlib.Path, pathlib.Path]:
    entries = sorted(entries, key=lambda item: (item.index, item.subindex))
    api_name = c_identifier(name)
    output_dir.mkdir(parents=True, exist_ok=True)
    header_path = output_dir / f"{api_name}.h"
    source_path = output_dir / f"{api_name}.c"
    header_path.write_text(generate_header(entries, api_name, header_path.name, header_prefix, header_suffix),
                          encoding="utf-8", newline="\n")
    source_path.write_text(generate_source(entries, api_name, header_path.name,
                                         source_eds.name, node_id, source_prefix, source_suffix),
                          encoding="utf-8", newline="\n")
    if emit_eds is None:
        return header_path, source_path
    eds_path = emit_eds
    if emit_eds.is_dir():
        eds_path = emit_eds / f"{api_name}.eds"
    eds_path.write_text(generate_eds(entries, node_id), encoding="utf-8", newline="\n")
    return header_path, source_path, eds_path


def compile_eds(
    input_path: pathlib.Path,
    output_dir: pathlib.Path,
    name: str,
    storage: str,
    node_id: int,
    *,
    header_prefix: str = "",
    header_suffix: str = "",
    source_prefix: str = "",
    source_suffix: str = "",
    emit_eds: pathlib.Path | None = None,
) -> tuple[pathlib.Path, pathlib.Path] | tuple[pathlib.Path, pathlib.Path, pathlib.Path]:
    if storage not in {"pointer", "value"}:
        raise ValueError("legacy mode only supports storage pointer/value")
    entries = load_eds(input_path, storage, node_id)
    return compile_entries(
        entries=entries,
        output_dir=output_dir,
        name=name or input_path.stem,
        node_id=node_id,
        source_eds=input_path,
        header_prefix=header_prefix,
        header_suffix=header_suffix,
        source_prefix=source_prefix,
        source_suffix=source_suffix,
        emit_eds=emit_eds,
    )


def compile_project(
    project_path: pathlib.Path,
    output_dir: pathlib.Path | None,
    node_id: int | None,
    conflict_policy: str | None,
    emit_eds: pathlib.Path | None = None,
) -> tuple[pathlib.Path, pathlib.Path] | tuple[pathlib.Path, pathlib.Path, pathlib.Path]:
    project = load_project_json(project_path)
    if conflict_policy:
        project = dataclasses.replace(project, conflict_policy=conflict_policy)
    target_node_id = node_id if node_id is not None else project.node_id
    target_dir = output_dir
    if target_dir is None:
        if project.output_dir is None:
            raise ValueError("project output-dir must be supplied by --output-dir or project.output_dir")
        target_dir = pathlib.Path(project.output_dir)
    outputs = compile_entries(
        entries=project.entries,
        output_dir=target_dir,
        name=project.name,
        node_id=target_node_id,
        source_eds=project_path,
        header_prefix=project.code_template.header_prefix,
        header_suffix=project.code_template.header_suffix,
        source_prefix=project.code_template.source_prefix,
        source_suffix=project.code_template.source_suffix,
        emit_eds=None,
    )
    if emit_eds is not None:
        eds_path = emit_eds
        if eds_path.is_dir():
            eds_path = eds_path / f"{c_identifier(project.name)}.eds"
        eds_path.write_text(generate_eds(project.entries, target_node_id), encoding="utf-8", newline="\n")
    return outputs


def _json_from_eds_inputs(
    name: str,
    eds_inputs: list[pathlib.Path],
    node_id: int,
    default_storage: str,
    default_conflict: str,
    base_dir: pathlib.Path,
) -> str:
    project_entries: list[DictionaryEntry] = []
    imports: list[dict] = []
    for input_path in eds_inputs:
        source = input_path if input_path.is_absolute() else base_dir / input_path
        alias = source.stem
        entries = load_eds(source, default_storage, node_id, group_path=(alias,))
        for entry in entries:
            project_entries.append(entry)
        imports.append({
            "path": str(source),
            "alias": alias,
            "storage": default_storage,
            "node_id": node_id,
        })
    payload = {
        "version": JSON_VERSION,
        "name": name,
        "node_id": node_id,
        "default_storage": default_storage,
        "conflict_policy": default_conflict,
        "imports": imports,
        "code": {
            "header_prefix": "",
            "header_suffix": "",
            "source_prefix": "",
            "source_suffix": "",
        },
        "entries": [_entry_to_json(entry) for entry in sorted(project_entries, key=lambda item: (item.index, item.subindex))],
    }
    return json.dumps(payload, indent=2, sort_keys=True) + "\n"


def _entry_to_json(entry: DictionaryEntry) -> dict[str, object]:
    payload: dict[str, object] = {
        "index": f"0x{entry.index:04X}",
        "subindex": f"0x{entry.subindex:02X}",
        "parameter_name": entry.parameter_name,
        "data_type": f"0x{entry.data_type:04X}",
        "access": entry.access,
        "pdo_mapping": entry.pdo_mapping,
        "default_value": entry.default_value,
        "size": entry.size,
        "storage": entry.storage,
        "group_path": list(entry.group_path),
    }
    if entry.variable_name:
        payload["variable_name"] = entry.variable_name
    if entry.comment:
        payload["comment"] = entry.comment
    return payload


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=pathlib.Path, help="Legacy EDS input file")
    parser.add_argument("--project", type=pathlib.Path, help="Dictionary JSON project")
    parser.add_argument("--to-json", type=pathlib.Path, help="Convert EDS file(s) to JSON project")
    parser.add_argument("--import-eds", nargs="+", type=pathlib.Path, help="EDS file list for --to-json")
    parser.add_argument("--output-dir", type=pathlib.Path, help="Output directory")
    parser.add_argument("--name", default="", help="Generated output base name")
    parser.add_argument("--storage", choices=("pointer", "value", "variable"), default="pointer",
                        help="Default storage mode for EDS input")
    parser.add_argument("--node-id", type=int, default=1, help="Node ID for $NODEID substitution")
    parser.add_argument("--emit-eds", type=pathlib.Path, help="Emit generated EDS to this file or directory")
    parser.add_argument("--json-conflict-policy", choices=sorted(CONFLICT_POLICIES),
                        default="last", help="Conflict policy for project mode")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    raw_args = list(sys.argv[1:] if argv is None else argv)
    args = build_argument_parser().parse_args(argv)
    explicit_node_id = "--node-id" in raw_args
    if args.to_json is not None:
        sources = []
        if args.import_eds:
            sources.extend(args.import_eds)
        if args.input is not None:
            sources.append(args.input)
        if not sources:
            print("[ERROR] --to-json requires --input and/or --import-eds", file=sys.stderr)
            return 1
        name = args.name.strip() or sources[0].stem
        if not (1 <= args.node_id <= 127):
            print("[ERROR] --node-id must be in range 1..127", file=sys.stderr)
            return 1
        data = _json_from_eds_inputs(name, sources, args.node_id, args.storage if args.storage != "variable" else "pointer",
                                    "last", sources[0].parent)
        args.to_json.write_text(data, encoding="utf-8", newline="\n")
        print(f"[OK] {args.to_json}")
        return 0

    if args.project is not None:
        if args.output_dir is None and args.input is not None:
            # legacy compatibility: allow using positional input as output-dir for JSON mode, ignored in project mode.
            args.output_dir = None
        try:
            outputs = compile_project(
                args.project,
                args.output_dir,
                args.node_id if explicit_node_id else None,
                args.json_conflict_policy,
                args.emit_eds,
            )
        except (OSError, json.JSONDecodeError, ValueError, configparser.Error) as error:
            print(f"[ERROR] {error}", file=sys.stderr)
            return 1
        for output in outputs:
            print(f"[OK] {output}")
        return 0

    if args.input is None:
        print("[ERROR] missing input EDS path (legacy mode) or --project", file=sys.stderr)
        return 1
    if args.output_dir is None:
        print("[ERROR] --output-dir is required in legacy mode", file=sys.stderr)
        return 1
    if not 1 <= args.node_id <= 127:
        print("[ERROR] --node-id must be in range 1..127", file=sys.stderr)
        return 1
    try:
        outputs = compile_eds(
            args.input,
            args.output_dir,
            args.name or args.input.stem,
            args.storage,
            args.node_id,
            emit_eds=args.emit_eds,
        )
    except (OSError, configparser.Error, ValueError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    for output in outputs:
        print(f"[OK] {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
