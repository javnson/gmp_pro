"""Normalize suite SDPE requirement layouts to the repository-wide taxonomy.

The script deliberately changes presentation metadata only: requirement rows,
bindings, macro values, hardware selections, and code sections remain intact.
Use ``--check`` in validation jobs and ``--write`` for the mechanical rewrite.
"""

from __future__ import annotations

import argparse
import json
from collections import OrderedDict
from datetime import date
from pathlib import Path
from typing import Any, Iterable


STANDARD_ROOT_GROUPS = (
    "Peripheral Parameters",
    "Per-Unit Base Parameters",
    "Main Element Parameters",
    "Protection Parameters",
    "Control Loop",
    "Runtime Parameters",
    "Voltage & Current Sensor",
    "Commissioning Defaults",
)


def _contains(text: str, *tokens: str) -> bool:
    return any(token in text for token in tokens)


def _sensor_group(text: str) -> str:
    text = (
        text.replace("voltage & current sensor", "")
        .replace("voltage current sensor", "")
        .replace("current voltage sensor", "")
    )
    if "position" in text or "encoder" in text:
        leaf = "Position Sensor"
    elif _contains(text, "dc bus voltage", "dcbus voltage"):
        leaf = "DC Bus Voltage Sensor"
    elif "input voltage" in text:
        leaf = "Input Voltage Sensor"
    elif "output voltage" in text:
        leaf = "Output Voltage Sensor"
    elif "grid voltage" in text:
        leaf = "Grid Voltage Sensor"
    elif _contains(text, "inverter phase voltage", "phase voltage"):
        leaf = "Inverter Phase Voltage Sensor"
    elif "ac voltage" in text:
        leaf = "AC Voltage Sensor"
    elif "voltage" in text:
        leaf = "Voltage Sensor"
    elif "input current" in text:
        leaf = "Input Current Sensor"
    elif "output current" in text:
        leaf = "Output Current Sensor"
    elif "inductor current" in text:
        leaf = "Inductor Current Sensor"
    elif "grid current" in text:
        leaf = "Grid Current Sensor"
    elif _contains(text, "inverter phase current", "phase current"):
        leaf = "Inverter Phase Current Sensor"
    elif "dc current" in text:
        leaf = "DC Current Sensor"
    elif "ac current" in text:
        leaf = "AC Current Sensor"
    else:
        leaf = "Current Sensor"
    return f"Voltage & Current Sensor / {leaf}"


def _control_group(text: str) -> str:
    if _contains(text, "pll", "spll", "sogi"):
        leaf = "PLL Controller"
    elif _contains(text, "fdrc", "repetitive"):
        leaf = "FDRC Controller"
    elif _contains(text, "virtual impedance"):
        leaf = "Virtual Impedance Controller"
    elif "vsm" in text:
        leaf = "VSM Controller"
    elif _contains(text, "synchron", "transition"):
        leaf = "Synchronization Controller"
    elif _contains(text, "power", "droop", "pq", "p/q"):
        leaf = "Power Controller"
    elif _contains(text, "position", "angle"):
        leaf = "Position Controller"
    elif _contains(text, "speed", "velocity"):
        leaf = "Speed Controller"
    elif "voltage" in text:
        leaf = "Voltage Controller"
    elif "current" in text:
        leaf = "Current Controller"
    elif _contains(text, "modulation", "modulator", "phase shift"):
        leaf = "Modulator"
    else:
        leaf = "General Controller"
    return f"Control Loop / {leaf}"


def canonical_requirement_group(old_group: str, row: dict[str, Any]) -> str:
    macro = str(row.get("macro", ""))
    role = str(row.get("role", ""))
    text = f"{old_group} {macro} {role}".lower().replace("_", " ")
    old = old_group.lower()

    if "MECH_DIV" in macro:
        return "Voltage & Current Sensor / Position Sensor"
    if "ADC_CALIBRATOR" in macro:
        return "Voltage & Current Sensor / Calibration"
    if "ZERO_QPR" in macro:
        return "Control Loop / Zero-Sequence Controller"
    if "DC_BUS_LOOP" in macro:
        return "Control Loop / DC Bus Voltage Controller"
    if "OUTER_LOOP" in macro:
        return "Control Loop / Outer-Loop Controller"
    if "OPEN_LOOP_" in macro:
        return "Commissioning Defaults"
    if "GRID_FREQUENCY" in macro:
        return "Per-Unit Base Parameters"
    if _contains(macro, "ADC_CALIB", "STARTUP", "CIA402") and "FDRC" not in macro:
        return "Runtime Parameters"

    if _contains(old, "sensor", "sensing") or _contains(
        macro,
        "_SENSITIVITY",
        "_BIAS",
        "_ENC_FS",
        "_ENC_BIAS",
    ):
        return _sensor_group(text)

    if _contains(old, "per-unit", "per unit", "per-unit base", "per-unit bases"):
        return "Per-Unit Base Parameters"

    if _contains(
        old,
        "main element",
        "resonant tank",
        "grid filter",
        "motor parameter",
        "mosfet parameter",
        "dc link parameter",
    ):
        if "resonant" in old:
            return "Main Element Parameters / Resonant Tank"
        if "grid filter" in old:
            return "Main Element Parameters / Grid Filter"
        if "motor" in old:
            return "Main Element Parameters / Motor"
        if "mosfet" in old:
            return "Main Element Parameters / Power Semiconductor"
        if "dc link" in old:
            return "Main Element Parameters / DC Link"
        return "Main Element Parameters"

    if "protection" in old:
        return "Protection Parameters"

    if _contains(old, "commissioning", "default parameter", "default parameters"):
        return "Commissioning Defaults"

    if _contains(
        old,
        "control loop",
        "control loops",
        "controller",
        "hybrid modulation",
        "voltage and grid-forming loops",
    ):
        return _control_group(text)

    if "runtime" in old:
        if _contains(macro, "FREQUENCY", "PWM_", "ADC_", "TIMER_", "CMP_MAX", "DEADBAND"):
            return "Peripheral Parameters"
        return "Runtime Parameters"

    if _contains(old, "peripheral", "adc platform", "board gpio", "clock"):
        if _contains(macro, "STARTUP", "TIMEOUT", "DELAY"):
            return "Runtime Parameters"
        return "Peripheral Parameters"

    if _contains(old, "system and four-wire"):
        return _control_group(text)

    # Fallbacks make newly added rows land in a standard root without changing
    # any values. Explicit group names above always take priority.
    if _contains(macro, "STARTUP", "TIMEOUT", "DELAY"):
        return "Runtime Parameters"
    if _contains(macro, "PROT_", "_LIMIT", "_MAX", "_MIN"):
        return "Protection Parameters"
    if _contains(macro, "_BASE", "RATED_"):
        return "Per-Unit Base Parameters"
    if _contains(text, "controller", "loop", "pll", "droop", "vsm"):
        return _control_group(text)
    return "Peripheral Parameters"


def _iter_group_rows(data: dict[str, Any]) -> Iterable[tuple[str, str]]:
    assigned: set[str] = set()
    role_to_row = {
        str(row.get("role", "")): row
        for row in data.get("requirements", [])
        if isinstance(row, dict)
    }
    for group in data.get("requirement_groups", []):
        if not isinstance(group, dict):
            continue
        old_name = str(group.get("name", ""))
        for role in group.get("requirements", []):
            role_text = str(role)
            row = role_to_row.get(role_text)
            if row is None or role_text in assigned:
                continue
            assigned.add(role_text)
            yield canonical_requirement_group(old_name, row), role_text

    for row in data.get("requirements", []):
        if not isinstance(row, dict):
            continue
        role = str(row.get("role", ""))
        if role and role not in assigned:
            yield canonical_requirement_group("", row), role


def _root_sort_key(name: str) -> tuple[int, str]:
    root = name.split(" / ", 1)[0]
    try:
        index = STANDARD_ROOT_GROUPS.index(root)
    except ValueError:
        index = len(STANDARD_ROOT_GROUPS)
    return index, name


def normalize_data(data: dict[str, Any]) -> dict[str, Any]:
    has_content = bool(
        data.get("requirements")
        or data.get("feature_macros")
        or data.get("option_macros")
        or data.get("hardware")
        or data.get("common_requirements")
    )
    if not has_content:
        # Empty manager files are deployment placeholders, not configured SDPE
        # projects. Avoid making a cosmetic rewrite look like a migration.
        data.pop("requirement_groups", None)
        data.pop("feature_macro_groups", None)
        data.pop("option_macro_groups", None)
        return data

    grouped: OrderedDict[str, list[str]] = OrderedDict()
    for group, role in _iter_group_rows(data):
        grouped.setdefault(group, []).append(role)
    data["requirement_groups"] = [
        {"name": name, "requirements": grouped[name]}
        for name in sorted(grouped, key=_root_sort_key)
    ]

    feature_groups = list(
        dict.fromkeys(
            str(row.get("group", "Selection Macros"))
            for row in data.get("feature_macros", [])
            if isinstance(row, dict)
        )
    )
    option_groups = list(
        dict.fromkeys(
            str(row.get("group", "Option Macros"))
            for row in data.get("option_macros", [])
            if isinstance(row, dict)
        )
    )
    data["feature_macro_groups"] = feature_groups
    data["option_macro_groups"] = option_groups
    if data.get("requirements") or data.get("feature_macros") or data.get("option_macros"):
        data["updated_at"] = date.today().isoformat()
    return data


def normalized_text(path: Path) -> str:
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    normalize_data(data)
    return json.dumps(data, ensure_ascii=False, indent=2) + "\n"


def requirement_files(suite_root: Path) -> list[Path]:
    return sorted(suite_root.rglob("sdpe_requirement.json"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true", help="report files that need normalization")
    mode.add_argument("--write", action="store_true", help="rewrite files in canonical layout")
    parser.add_argument(
        "--suite-root",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "ctl" / "suite",
    )
    args = parser.parse_args()

    changed: list[Path] = []
    for path in requirement_files(args.suite_root.resolve()):
        original = path.read_text(encoding="utf-8-sig")
        normalized = normalized_text(path)
        if original.replace("\r\n", "\n") == normalized:
            continue
        changed.append(path)
        if args.write:
            path.write_text(normalized, encoding="utf-8", newline="\n")

    for path in changed:
        print(path)
    if args.check and changed:
        print(f"{len(changed)} SDPE requirement file(s) need layout normalization.")
        return 1
    print(f"Normalized {len(changed)} SDPE requirement file(s)." if args.write else "SDPE layouts are normalized.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
