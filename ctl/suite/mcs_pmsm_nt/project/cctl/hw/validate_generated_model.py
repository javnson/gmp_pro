"""Verify the generated ADC front-end gains against the project-private SDPE entity."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("circuit_data", type=Path)
    parser.add_argument("hardware", type=Path)
    args = parser.parse_args()
    gmp_root = Path(os.environ["GMP_PRO_LOCATION"])
    sys.path.insert(0, str(gmp_root / "tools" / "cctl_studio" / "mna_solver"))
    import circuit_data as circuit_data_module

    document = circuit_data_module.load_circuit_data(args.circuit_data)
    hardware = json.loads(args.hardware.read_text(encoding="utf-8"))["parameters"]
    simulator = circuit_data_module.CircuitDataSimulator(document)
    switches = document["switching"]["switches"]
    paths = [
        "channel" if item["pwm_port"] in {"PWM2", "PWM4", "PWM6"} else "off"
        for item in switches
    ]
    topology = next(item for item in document["topologies"] if item["mosfet_paths"] == paths)
    model = simulator.discrete_states[topology["calculation_state_index"]]["normal"]
    identity = np.eye(model["A"].shape[0])
    gain = model["C"] @ np.linalg.solve(identity - model["A"], model["B"]) + model["D"]
    inputs = [
        port["name"] for port in document["ports"]["inputs"]
        if port["data_type"] == "double"
    ]
    signals = document["signals"]["names"]

    expected_current = (
        float(hardware["phase_shunt_resistance_ohm"])
        * float(hardware["phase_current_gain_v_v"])
    )
    expected_voltage = float(hardware["bus_voltage_gain_v_v"])
    checks = tuple(
        (
            f"phase {phase} current",
            f"IPMSM1_{phase}",
            f"V(VADC_I{phase})",
            expected_current,
        )
        for phase in "ABC"
    ) + (("DC bus voltage", "VS1", "V(VADC_VDC)", expected_voltage),)
    for label, source, output, expected in checks:
        actual = float(gain[signals.index(output), inputs.index(source)])
        if not np.isclose(actual, expected, rtol=1e-4, atol=1e-9):
            raise RuntimeError(f"{label} gain mismatch: model={actual}, SDPE={expected}")
        print(f"{label:<18} model={actual:.12g}  SDPE={expected:.12g}")

    for measured_phase in "ABC":
        for source_phase in "ABC":
            if measured_phase == source_phase:
                continue
            actual = float(
                gain[
                    signals.index(f"V(VADC_I{measured_phase})"),
                    inputs.index(f"IPMSM1_{source_phase}"),
                ]
            )
            if not np.isclose(actual, 0.0, rtol=0.0, atol=1e-9):
                raise RuntimeError(
                    f"phase-current cross coupling I{source_phase}->"
                    f"VADC_I{measured_phase}: {actual}"
                )

    phase_gates = {"A": ("PWM1", "PWM2"), "B": ("PWM3", "PWM4"),
                   "C": ("PWM5", "PWM6")}
    for phase, (upper_pwm, lower_pwm) in phase_gates.items():
        paths = []
        for switch in switches:
            pwm = switch["pwm_port"]
            other_lower = pwm in {"PWM2", "PWM4", "PWM6"} and pwm != lower_pwm
            paths.append("channel" if pwm == upper_pwm or other_lower else "off")
        topology = next(
            item for item in document["topologies"]
            if item["mosfet_paths"] == paths
        )
        phase_model = simulator.discrete_states[
            topology["calculation_state_index"]
        ]["normal"]
        phase_gain = (
            phase_model["C"]
            @ np.linalg.solve(identity - phase_model["A"], phase_model["B"])
            + phase_model["D"]
        )
        responses = {
            output_phase: float(
                phase_gain[
                    signals.index(f"VPMSM1_{output_phase}"),
                    inputs.index("VS1"),
                ]
            )
            for output_phase in "ABC"
        }
        if responses[phase] < 0.9 or any(
            abs(value) > 1e-3
            for output_phase, value in responses.items()
            if output_phase != phase
        ):
            raise RuntimeError(
                f"{phase}-phase PWM bridge routing mismatch: {responses}"
            )
        print(f"phase {phase} PWM bridge routing: {responses}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
