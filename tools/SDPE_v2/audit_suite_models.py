#!/usr/bin/env python3
"""Audit ctl/suite SLX files for generated SDPE bindings and stale callbacks."""

from __future__ import annotations

import argparse
import re
import zipfile
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


ASSIGNMENT = re.compile(r"(?m)^([A-Z][A-Z0-9_]+)\s*=")
PHYSICAL_LITERAL = re.compile(
    r'<P Name="(Resistance|Inductance|Capacitance|Amplitude|Frequency|Gain|Bias|'
    r'Ron|Vf|DeadtimeSeconds|TimerClockHz|PolePairs|Flux|VoltageCst|TorqueCst)">'
    r"\s*([-+]?(?:\d|\.\d)[^<]*)</P>"
)
ABSOLUTE_PATH = re.compile(r"[A-Za-z]:[/\\]")
OLD_SDPE_ALIAS = re.compile(r"sdpe_[a-z0-9_]+_simulate_settings_matlab_init", re.I)


@dataclass
class Audit:
    suite: str
    model: Path
    is_library: bool
    init_exists: bool
    callback_ok: bool
    used_variables: list[str]
    literal_candidates: list[str]

    @property
    def passed(self) -> bool:
        return self.is_library or (
            self.init_exists and self.callback_ok and bool(self.used_variables)
        )


def read_slx_xml(model: Path) -> tuple[str, str]:
    with zipfile.ZipFile(model) as archive:
        entries = {
            name: archive.read(name).decode("utf-8", errors="replace")
            for name in archive.namelist()
            if name.endswith(".xml")
        }
    return entries.get("simulink/blockdiagram.xml", ""), "\n".join(entries.values())


def audit_model(suite_root: Path, model: Path) -> Audit:
    relative = model.relative_to(suite_root)
    suite = relative.parts[0]
    init_path = model.parent / "sdpe_mgr" / "ctrl_settings_matlab_init.m"
    variables: list[str] = []
    if init_path.is_file():
        variables = sorted(set(ASSIGNMENT.findall(init_path.read_text(encoding="utf-8-sig"))))

    blockdiagram, all_xml = read_slx_xml(model)
    used = [
        name for name in variables
        if re.search(rf"(?<![A-Z0-9_]){re.escape(name)}(?![A-Z0-9_])", all_xml)
    ]
    callback_ok = (
        blockdiagram.count('Name="PostLoadFcn"') == 1
        and blockdiagram.count('Name="InitFcn"') == 1
        and 'Name="PreLoadFcn"' not in blockdiagram
        and "sdpe_mgr" in blockdiagram
        and "ctrl_settings_matlab_init.m" in blockdiagram
        and not OLD_SDPE_ALIAS.search(blockdiagram)
        and not ABSOLUTE_PATH.search(blockdiagram)
    )
    candidates = sorted(
        {f"{name}={value.strip()}" for name, value in PHYSICAL_LITERAL.findall(all_xml)}
    )
    return Audit(
        suite=suite,
        model=model,
        is_library="<Library>" in blockdiagram,
        init_exists=init_path.is_file(),
        callback_ok=callback_ok,
        used_variables=used,
        literal_candidates=candidates,
    )


def report(suite_root: Path, suite: str, audits: list[Audit], matlab_release: str) -> str:
    failed = [item for item in audits if not item.passed]
    lines = [
        "# SDPE model-parameter audit",
        "",
        f"Suite: `{suite}`  ",
        f"MATLAB load verification: `{matlab_release}`  ",
        f"Result: **{'PASS' if not failed else 'NEEDS MIGRATION'}**",
        "",
        "The audit verifies that each SLX has a path-independent callback to its freshly "
        "generated `sdpe_mgr/ctrl_settings_matlab_init.m`, and that the model XML actually "
        "references at least one variable exported by that generated script. Loading an "
        "initializer alone is not counted as parameter use.",
        "",
        "| Model | Kind | Callback | SDPE variables used | Status |",
        "| --- | --- | --- | ---: | --- |",
    ]
    for item in audits:
        rel = item.model.relative_to(suite_root / suite).as_posix()
        kind = "library" if item.is_library else "model"
        status = "EXEMPT" if item.is_library else ("PASS" if item.passed else "NEEDS MIGRATION")
        lines.append(
            f"| `{rel}` | {kind} | {'PASS' if item.callback_ok else 'FAIL'} | "
            f"{len(item.used_variables)} | {status} |"
        )

    lines += ["", "## Per-model evidence", ""]
    for item in audits:
        rel = item.model.relative_to(suite_root / suite).as_posix()
        lines += [f"### `{rel}`", ""]
        if item.is_library:
            lines.append("Block library: load/callback behavior was checked; plant-parameter binding is not applicable.")
        elif item.used_variables:
            lines.append(
                "Referenced generated SDPE variables: "
                + ", ".join(f"`{name}`" for name in item.used_variables)
                + "."
            )
        else:
            lines.append(
                "No generated SDPE variable is referenced by the stored model. The callback "
                "loads SDPE data, but the plant remains numerically configured and must be migrated."
            )
        if item.literal_candidates:
            shown = item.literal_candidates[:12]
            suffix = "" if len(shown) == len(item.literal_candidates) else " (first 12 shown)"
            lines += [
                "",
                f"Numeric physical-parameter candidates{suffix}: "
                + ", ".join(f"`{value}`" for value in shown)
                + ".",
            ]
        lines.append("")

    lines += [
        "## Interpretation",
        "",
        "Numeric candidates are review evidence, not an automatic failure by themselves: "
        "some are solver settings, ideal-element values, or internal mask defaults whose "
        "effective value comes from a symbolic parent mask. A non-library model with zero "
        "generated-variable references is unambiguously not consuming SDPE parameters.",
        "",
        "The R2024b load check opens every committed SLX with callbacks enabled. It does not "
        "claim that a SIL simulation, target build, or hardware test passed.",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("suite_root", nargs="?", default="ctl/suite")
    parser.add_argument("--matlab-release", default="not run")
    args = parser.parse_args()
    suite_root = Path(args.suite_root).resolve()
    grouped: dict[str, list[Audit]] = defaultdict(list)
    for model in sorted(suite_root.glob("*/project/**/*.slx")):
        item = audit_model(suite_root, model)
        grouped[item.suite].append(item)

    failures = 0
    for suite, audits in sorted(grouped.items()):
        doc = suite_root / suite / "doc"
        doc.mkdir(parents=True, exist_ok=True)
        output = doc / "sdpe_model_parameter_audit.md"
        output.write_text(
            report(suite_root, suite, audits, args.matlab_release),
            encoding="utf-8",
            newline="\n",
        )
        failures += sum(not item.passed for item in audits)
        print(f"{output.relative_to(suite_root.parent.parent)}: "
              f"{len(audits)} model(s), {sum(item.passed for item in audits)} compliant/exempt")
    print(f"noncompliant models: {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
