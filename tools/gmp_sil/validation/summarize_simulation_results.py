#!/usr/bin/env python3
"""Generate compact Markdown indexes for archived suite simulation results."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


PRIORITY_METRICS = {
    "dps_clllc": [
        "final_primary_voltage_v", "final_primary_current_a",
        "final_secondary_voltage_v", "final_resonant_current_a",
        "final_modulation_command", "steady_state_error_percent",
        "step_response.settling_time_s", "step_response.overshoot_percent",
    ],
    "dps_fsbb": [
        "vin_final_v", "vout_final_v", "vout_peak_v", "il_final_a",
        "iout_final_a", "steady_state_error_percent",
        "step_response.settling_time_s", "step_response.overshoot_percent",
    ],
    "mcs_acm_nt": [
        "step_response.reference_final", "step_response.response_final",
        "step_response.rise_time_10_90_s", "step_response.settling_time_s",
        "step_response.overshoot_percent", "steady_state_error_percent",
    ],
    "mcs_pmsm_nt": [
        "step_response.reference_final", "step_response.response_final",
        "step_response.rise_time_10_90_s", "step_response.settling_time_s",
        "step_response.overshoot_percent", "steady_state_error_percent",
    ],
    "pgs_inv_GFL_inverter": [
        "phase_current_rms_pu", "zero_current_rms_pu", "voltage_d_mean_pu",
        "voltage_q_rms_pu", "current_d_mean_pu", "current_q_mean_pu",
        "pll_error_rms_pu", "current_tracking_error_pu",
    ],
    "pgs_inv_GFM_inverter": [
        "phase_current_rms_pu", "phase_current_peak_pu", "voltage_d_mean_pu",
        "voltage_q_rms_pu", "current_d_mean_pu", "current_q_mean_pu",
        "frequency_hz", "voltage_tracking_error_pu",
    ],
    "pgs_sinv_rc": [
        "vac_rms_v", "iac_rms_a", "vbus_mean_v", "iref_rms_a",
        "current_error_rms_pu", "active_power_pu", "reactive_power_pu",
        "pll_frequency_hz", "iac_thd_percent", "fdrc_output_rms_pu",
    ],
}


def flatten(value: Any, prefix: str = "") -> dict[str, Any]:
    result: dict[str, Any] = {}
    if isinstance(value, dict):
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else key
            result.update(flatten(child, child_prefix))
    elif isinstance(value, list):
        if all(not isinstance(item, (dict, list)) for item in value):
            result[prefix] = value
    else:
        result[prefix] = value
    return result


def display(value: Any) -> str:
    if isinstance(value, bool):
        return "PASS" if value else "FAIL"
    if isinstance(value, float):
        return f"{value:.6g}"
    if isinstance(value, list):
        return ", ".join(display(item) for item in value)
    if value is None:
        return "—"
    return str(value).replace("|", "\\|")


def load_json(path: Path) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def relative_link(path: Path, result_dir: Path) -> str:
    return path.relative_to(result_dir).as_posix()


def csv_shape(path: Path) -> tuple[int, list[str]]:
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.reader(stream)
            header = next(reader, [])
            return sum(1 for _ in reader), header
    except (OSError, UnicodeDecodeError, csv.Error):
        return -1, []


def report_for(result_dir: Path) -> str:
    suite = result_dir.parents[1].name
    lines = [
        f"# {suite} simulation results",
        "",
        "This directory is the repository archive for simulation evidence. "
        "Validation scripts write here directly; generated controller/SDPE files remain outside this archive.",
        "",
    ]

    summary_path = result_dir / "sil_validation_summary.json"
    summary = load_json(summary_path)
    if summary and isinstance(summary.get("cases"), list):
        lines += [
            "## Validation matrix",
            "",
            f"Recorded overall result: **{'PASS' if summary.get('all_pass') else 'FAIL'}**.",
            "",
            "| BUILD_LEVEL | Build | Simulation | Dynamic | Steady state | Overall | Duration (s) |",
            "| ---: | --- | --- | --- | --- | --- | ---: |",
        ]
        for case in summary["cases"]:
            lines.append(
                "| {build_level} | {build_pass} | {simulation_pass} | {dynamic_pass} | "
                "{steady_state_pass} | {pass_} | {duration} |".format(
                    build_level=display(case.get("build_level")),
                    build_pass=display(case.get("build_pass")),
                    simulation_pass=display(case.get("simulation_pass")),
                    dynamic_pass=display(case.get("dynamic_pass")),
                    steady_state_pass=display(case.get("steady_state_pass")),
                    pass_=display(case.get("pass")),
                    duration=display(case.get("duration_s")),
                )
            )
        lines.append("")
    else:
        lines += [
            "## Validation matrix",
            "",
            "No complete machine-readable validation matrix is archived. "
            "The files below are retained as partial evidence only.",
            "",
        ]

    metric_files = sorted(result_dir.rglob("*_metrics.json"))
    lines += ["## Key recorded metrics", ""]
    if not metric_files:
        lines += ["No per-run metric JSON files are present.", ""]
    else:
        priorities = PRIORITY_METRICS.get(suite, [])
        for path in metric_files:
            data = load_json(path)
            if data is None:
                link = relative_link(path, result_dir)
                lines += [f"### [{link}]({link})", "", "Unreadable JSON.", ""]
                continue
            values = flatten(data)
            status = values.get("pass", values.get("simulation_pass"))
            model = values.get("model", "—")
            level = values.get("build_level", "—")
            stop = values.get("stop_time_s", "—")
            link = relative_link(path, result_dir)
            lines += [
                f"### [{link}]({link})",
                "",
                f"BUILD_LEVEL `{display(level)}`, model `{display(model)}`, "
                f"stop time `{display(stop)} s`, recorded status **{display(status)}**.",
                "",
                "| Metric | Value |",
                "| --- | ---: |",
            ]
            selected = [(key, values[key]) for key in priorities if key in values]
            if not selected:
                selected = [
                    (key, value) for key, value in values.items()
                    if key not in {"build_level", "model", "stop_time_s"}
                    and isinstance(value, (int, float, bool))
                ][:10]
            lines.extend(f"| `{key}` | {display(value)} |" for key, value in selected)
            lines.append("")

    csv_files = sorted(result_dir.rglob("*.csv"))
    lines += ["## CSV data inventory", ""]
    if csv_files:
        lines += [
            "| File | Data rows | Columns | Leading fields |",
            "| --- | ---: | ---: | --- |",
        ]
        for path in csv_files:
            rows, header = csv_shape(path)
            fields = ", ".join(f"`{name}`" for name in header[:6])
            if len(header) > 6:
                fields += ", …"
            link = relative_link(path, result_dir)
            lines.append(
                f"| [{link}]({link}) | {'unreadable' if rows < 0 else rows} "
                f"| {len(header)} | {fields or '—'} |"
            )
        lines.append("")
    else:
        lines += ["No CSV data files are archived.", ""]

    generic_json = sorted(
        path for path in result_dir.rglob("*.json")
        if path != summary_path and path not in metric_files
    )
    lines += ["## Other machine-readable records", ""]
    if generic_json:
        for path in generic_json:
            link = relative_link(path, result_dir)
            data = load_json(path)
            lines += [f"### [{link}]({link})", ""]
            if data is None:
                lines += ["Unreadable JSON.", ""]
                continue
            values = flatten(data)
            selected = [
                (key, value) for key, value in values.items()
                if isinstance(value, (int, float, bool, str))
                and not key.endswith(("path", "file"))
            ][:16]
            lines += ["| Field | Value |", "| --- | ---: |"]
            lines.extend(f"| `{key}` | {display(value)} |" for key, value in selected)
            lines.append("")
    else:
        lines += ["No additional JSON records are archived.", ""]

    documented = {summary_path, result_dir / "README.md", *metric_files, *csv_files, *generic_json}
    artifacts = sorted(
        path for path in result_dir.rglob("*")
        if path.is_file() and path not in documented
    )
    lines += ["## Supporting artifacts", ""]
    if artifacts:
        lines.extend(
            f"- [{relative_link(path, result_dir)}]({relative_link(path, result_dir)})"
            for path in artifacts
        )
    else:
        lines.append("No additional artifacts are archived.")
    lines += [
        "",
        "A recorded PASS applies only to the stored model, BUILD_LEVEL, parameters and toolchain represented by these files.",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("suite_root", nargs="?", default="ctl/suite")
    args = parser.parse_args()
    root = Path(args.suite_root).resolve()
    result_dirs = sorted(root.glob("*/doc/simulation_result"))
    for result_dir in result_dirs:
        output = result_dir / "README.md"
        output.write_text(report_for(result_dir), encoding="utf-8", newline="\n")
        print(output.relative_to(root.parent.parent))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
