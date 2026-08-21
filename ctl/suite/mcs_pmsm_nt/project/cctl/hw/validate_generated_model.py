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
    checks = (
        ("phase A current", "IPMSM1_A", "V(VADC_IC)", expected_current),
        ("phase B current", "IPMSM1_B", "V(VADC_IB)", expected_current),
        ("phase C current", "IPMSM1_C", "V(VADC_IA)", expected_current),
        ("DC bus voltage", "VS1", "V(VADC_VDC)", expected_voltage),
    )
    for label, source, output, expected in checks:
        actual = float(gain[signals.index(output), inputs.index(source)])
        if not np.isclose(actual, expected, rtol=1e-4, atol=1e-9):
            raise RuntimeError(f"{label} gain mismatch: model={actual}, SDPE={expected}")
        print(f"{label:<18} model={actual:.12g}  SDPE={expected:.12g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

