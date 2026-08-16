#!/usr/bin/env python3
"""Compile a CiA 306-style EDS object list into GMP CANopen OD C sources."""

from __future__ import annotations

import argparse
import configparser
import dataclasses
import math
import pathlib
import re
import sys
from typing import Iterable


SECTION_RE = re.compile(r"^([0-9A-Fa-f]{4})(?:sub([0-9A-Fa-f]{1,2}))?$")

TYPE_INFO = {
    0x0001: ("GMP_CANOPEN_OD_BOOLEAN", "boolean", "fast_gt", 1),
    0x0002: ("GMP_CANOPEN_OD_INTEGER8", "i8", "int_least8_t", 1),
    0x0003: ("GMP_CANOPEN_OD_INTEGER16", "i16", "int16_t", 2),
    0x0004: ("GMP_CANOPEN_OD_INTEGER32", "i32", "int32_t", 4),
    0x0005: ("GMP_CANOPEN_OD_UNSIGNED8", "u8", "uint_least8_t", 1),
    0x0006: ("GMP_CANOPEN_OD_UNSIGNED16", "u16", "uint16_t", 2),
    0x0007: ("GMP_CANOPEN_OD_UNSIGNED32", "u32", "uint32_t", 4),
    0x0008: ("GMP_CANOPEN_OD_REAL32", "real32", "float", 4),
    0x0009: ("GMP_CANOPEN_OD_VISIBLE_STRING", "octets", "gmp_canopen_octet_t", 0),
    0x000A: ("GMP_CANOPEN_OD_OCTET_STRING", "octets", "gmp_canopen_octet_t", 0),
    0x000F: ("GMP_CANOPEN_OD_DOMAIN", "octets", "gmp_canopen_octet_t", 0),
    0x0011: ("GMP_CANOPEN_OD_REAL64", "real64", "double", 8),
    0x0015: ("GMP_CANOPEN_OD_INTEGER64", "i64", "int64_t", 8),
    0x001B: ("GMP_CANOPEN_OD_UNSIGNED64", "u64", "uint64_t", 8),
}

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


@dataclasses.dataclass(frozen=True)
class EdsEntry:
    index: int
    subindex: int
    parameter_name: str
    data_type: int
    access: str
    pdo_mapping: bool
    default_value: str
    size: int
    storage: str

    @property
    def symbol_suffix(self) -> str:
        return f"{self.index:04x}_{self.subindex:02x}"


def parse_int(text: str, node_id: int = 1) -> int:
    value = text.strip().replace("$NODEID", str(node_id)).replace("$NodeID", str(node_id))
    value = value.replace("(", "").replace(")", "")
    if not re.fullmatch(r"[0-9A-Fa-fxX()+\- ]+", value):
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


def parse_bool(text: str) -> bool:
    return text.strip().lower() in {"1", "true", "yes", "on"}


def unquote(text: str) -> str:
    text = text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in {'"', "'"}:
        return text[1:-1]
    return text


def load_eds(path: pathlib.Path, default_storage: str, node_id: int) -> list[EdsEntry]:
    parser = configparser.ConfigParser(interpolation=None, strict=True)
    parser.optionxform = str
    with path.open("r", encoding="utf-8-sig") as stream:
        parser.read_file(stream)
    entries: list[EdsEntry] = []
    for section_name in parser.sections():
        match = SECTION_RE.fullmatch(section_name)
        if not match:
            continue
        section = {key.lower(): value for key, value in parser[section_name].items()}
        if "datatype" not in section:
            continue
        data_type = parse_int(section["datatype"], node_id)
        if data_type not in TYPE_INFO:
            raise ValueError(f"[{section_name}] unsupported DataType 0x{data_type:04X}")
        index = int(match.group(1), 16)
        subindex = int(match.group(2), 16) if match.group(2) else 0
        access = section.get("accesstype", "ro").strip().lower()
        if access not in ACCESS_INFO:
            raise ValueError(f"[{section_name}] unsupported AccessType {access!r}")
        default_value = section.get("parametervalue", section.get("defaultvalue", "0"))
        fixed_size = TYPE_INFO[data_type][3]
        if fixed_size:
            size = fixed_size
        else:
            size_text = section.get("datalength", "")
            if size_text:
                size = parse_int(size_text, node_id)
            else:
                size = max(1, len(unquote(default_value).encode("utf-8")))
        storage = section.get("gmpstorage", default_storage).strip().lower()
        if storage not in {"pointer", "value"}:
            raise ValueError(f"[{section_name}] GMPStorage must be pointer or value")
        if storage == "value" and size > 8:
            raise ValueError(f"[{section_name}] value storage is limited to 8 octets")
        if size <= 0:
            raise ValueError(f"[{section_name}] object size must be positive")
        entry = EdsEntry(
            index=index,
            subindex=subindex,
            parameter_name=section.get("parametername", f"0x{index:04X}:{subindex:02X}"),
            data_type=data_type,
            access=access,
            pdo_mapping=parse_bool(section.get("pdomapping", "0")),
            default_value=default_value,
            size=size,
            storage=storage,
        )
        if fixed_size:
            numeric_literal(entry, node_id)
        else:
            raw_bytes(entry)
        entries.append(entry)
    entries.sort(key=lambda entry: (entry.index, entry.subindex))
    keys = [(entry.index, entry.subindex) for entry in entries]
    if len(keys) != len(set(keys)):
        raise ValueError("EDS contains duplicate index/subindex sections")
    if not entries:
        raise ValueError("EDS does not contain any supported object entries")
    return entries


def c_identifier(text: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_]", "_", text)
    result = re.sub(r"_+", "_", result).strip("_")
    if not result or result[0].isdigit():
        result = "od_" + result
    return result.lower()


def c_string(text: str) -> str:
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def numeric_literal(entry: EdsEntry, node_id: int) -> str:
    _, _, c_type, _ = TYPE_INFO[entry.data_type]
    if entry.data_type in {0x0008, 0x0011}:
        value = float(entry.default_value.strip() or "0")
        if not math.isfinite(value):
            raise ValueError(
                f"0x{entry.index:04X}:{entry.subindex:02X} default must be finite")
        suffix = "F" if entry.data_type == 0x0008 else ""
        return f"({c_type}){value:.17g}{suffix}"
    value = parse_int(entry.default_value or "0", node_id)
    minimum, maximum = INTEGER_RANGES[entry.data_type]
    if value < minimum or value > maximum:
        raise ValueError(
            f"0x{entry.index:04X}:{entry.subindex:02X} default is outside its data type")
    return f"({c_type}){value}"


def raw_bytes(entry: EdsEntry) -> bytes:
    text = unquote(entry.default_value)
    if entry.data_type == 0x0009:
        data = text.encode("utf-8")
    elif entry.data_type in {0x000A, 0x000F}:
        compact = re.sub(r"[\s,:_-]", "", text)
        if compact and (len(compact) % 2 != 0 or
                        re.fullmatch(r"[0-9A-Fa-f]+", compact) is None):
            raise ValueError(
                f"0x{entry.index:04X}:{entry.subindex:02X} has invalid raw hex data")
        data = bytes.fromhex(compact) if compact else b""
    else:
        raise AssertionError("not a raw EDS type")
    if len(data) > entry.size:
        raise ValueError(f"0x{entry.index:04X}:{entry.subindex:02X} default exceeds DataLength")
    return data + bytes(entry.size - len(data))


def access_expression(entry: EdsEntry) -> str:
    expression = ACCESS_INFO[entry.access]
    if entry.pdo_mapping:
        expression += " | GMP_CANOPEN_OD_ACCESS_PDO"
    return expression


def generate_header(entries: list[EdsEntry], api_name: str, header_name: str) -> str:
    guard = f"_FILE_{api_name.upper()}_H_"
    storage_declarations = []
    for entry in entries:
        if entry.storage != "pointer":
            continue
        _, _, c_type, fixed_size = TYPE_INFO[entry.data_type]
        symbol = f"{api_name}_storage_{entry.symbol_suffix}"
        if fixed_size:
            storage_declarations.append(f"extern {c_type} {symbol};")
        else:
            storage_declarations.append(f"extern gmp_canopen_octet_t {symbol}[{entry.size}U];")
    storage_text = "\n".join(storage_declarations)
    return f'''/** @file {header_name} @brief Generated from an EDS file; do not edit. */
#ifndef {guard}
#define {guard}

#include <core/protocol/canopen/od.h>

#ifdef __cplusplus
extern "C"
{{
#endif

#define {api_name.upper()}_ENTRY_COUNT {len(entries)}U

extern gmp_canopen_od_entry_t {api_name}_entries[{api_name.upper()}_ENTRY_COUNT];
{storage_text}
fast_gt {api_name}_init(gmp_canopen_od_t* dictionary);

#ifdef __cplusplus
}}
#endif

#endif /* {guard} */
'''


def generate_source(entries: list[EdsEntry], api_name: str,
                    header_name: str, source_eds: pathlib.Path,
                    node_id: int) -> str:
    lines = [
        f"/** @file {api_name}.c @brief Generated from {source_eds.name}; do not edit. */",
        f'#include "{header_name}"',
        "",
        f"gmp_canopen_od_entry_t {api_name}_entries[{api_name.upper()}_ENTRY_COUNT];",
    ]
    for entry in entries:
        if entry.storage != "pointer":
            continue
        _, _, c_type, fixed_size = TYPE_INFO[entry.data_type]
        symbol = f"{api_name}_storage_{entry.symbol_suffix}"
        if fixed_size:
            lines.append(f"{c_type} {symbol} = {numeric_literal(entry, node_id)};")
        else:
            payload = raw_bytes(entry)
            values = ", ".join(f"0x{value:02X}U" for value in payload)
            lines.append(f"gmp_canopen_octet_t {symbol}[{entry.size}U] = {{{values}}};")
    lines.extend(["", f"fast_gt {api_name}_init(gmp_canopen_od_t* dictionary)", "{",
                  "    size_gt entry_index;",
                  "    if (dictionary == NULL)", "        return 0;",
                  f"    for (entry_index = 0U; entry_index < {api_name.upper()}_ENTRY_COUNT; ++entry_index)",
                  f"        if (!gmp_rb_node_is_detached(&{api_name}_entries[entry_index].rb_node))",
                  "            return 0;",
                  "    gmp_canopen_od_init(dictionary);"])
    for ordinal, entry in enumerate(entries):
        enum_name, field, _, fixed_size = TYPE_INFO[entry.data_type]
        access = access_expression(entry)
        name = c_string(entry.parameter_name)
        if entry.storage == "pointer":
            symbol = f"{api_name}_storage_{entry.symbol_suffix}"
            pointer = f"&{symbol}" if fixed_size else symbol
            lines.extend([
                f"    gmp_canopen_od_entry_init_pointer(&{api_name}_entries[{ordinal}U],",
                f"        0x{entry.index:04X}U, 0x{entry.subindex:02X}U, {enum_name},",
                f"        {access}, {pointer}, {entry.size}U, {name});",
            ])
        else:
            lines.extend([
                f"    gmp_canopen_od_entry_init_value(&{api_name}_entries[{ordinal}U],",
                f"        0x{entry.index:04X}U, 0x{entry.subindex:02X}U, {enum_name},",
                f"        {access}, {entry.size}U, {name});",
            ])
            if fixed_size:
                lines.append(
                    f"    {api_name}_entries[{ordinal}U].storage.value.{field} = "
                    f"{numeric_literal(entry, node_id)};")
            else:
                for byte_index, value in enumerate(raw_bytes(entry)):
                    lines.append(
                        f"    {api_name}_entries[{ordinal}U].storage.value.octets[{byte_index}U] = "
                        f"0x{value:02X}U;")
        lines.extend([
            f"    if (gmp_canopen_od_insert(dictionary, &{api_name}_entries[{ordinal}U]) != GMP_CANOPEN_OD_OK)",
            "        return 0;",
        ])
    lines.extend([
        f"    return gmp_canopen_od_validate(dictionary, {api_name.upper()}_ENTRY_COUNT);",
        "}",
        "",
    ])
    return "\n".join(lines)


def compile_eds(input_path: pathlib.Path, output_dir: pathlib.Path,
                name: str, storage: str, node_id: int) -> tuple[pathlib.Path, pathlib.Path]:
    if not 1 <= node_id <= 127:
        raise ValueError("node ID must be in range 1..127")
    entries = load_eds(input_path, storage, node_id)
    api_name = c_identifier(name)
    output_dir.mkdir(parents=True, exist_ok=True)
    header_path = output_dir / f"{api_name}.h"
    source_path = output_dir / f"{api_name}.c"
    header_path.write_text(generate_header(entries, api_name, header_path.name), encoding="utf-8", newline="\n")
    source_path.write_text(generate_source(entries, api_name, header_path.name,
                                           input_path, node_id), encoding="utf-8", newline="\n")
    return header_path, source_path


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=pathlib.Path, help="CiA 306-style EDS input")
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    parser.add_argument("--name", required=True, help="C API/file base name")
    parser.add_argument("--storage", choices=("pointer", "value"), default="pointer")
    parser.add_argument("--node-id", type=int, default=1,
                        help="node ID used to resolve $NODEID expressions")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if not 1 <= args.node_id <= 127:
        raise SystemExit("--node-id must be in range 1..127")
    try:
        header, source = compile_eds(args.input, args.output_dir, args.name,
                                     args.storage, args.node_id)
    except (OSError, configparser.Error, ValueError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    print(f"[OK] {header}")
    print(f"[OK] {source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
