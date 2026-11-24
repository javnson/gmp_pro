"""Build the closed-loop Eigen plant from the existing FSBB TINA topology."""

import json
import os
import sys
from dataclasses import replace
from pathlib import Path

root = Path(os.environ["GMP_PRO_LOCATION"])
sys.path.insert(0, str(root / "tools/cctl_studio/mna_solver"))

from circuit_data import build_circuit_data, write_circuit_data
from cpp_codegen import generate_cpp_project
from mna_solver import Element, Observation, parse_netlist
from switched_solver import (
    build_piecewise_model,
    derive_mosfet_parameters,
    resolve_mosfet_half_bridge_pairs,
)


def bindings(path: Path) -> dict:
    return {
        item["macro"]: item["binding"]
        for item in json.loads(path.read_text(encoding="utf-8"))["requirements"]
    }


def generate(output: Path) -> None:
    hw = Path(__file__).resolve().parent
    source = hw / "FSBB.CIR"
    suite = hw.parents[2]
    common = bindings(suite / "sdpe_general/sdpe_requirement.json")
    sil = bindings(hw.parents[1] / "simulate/sdpe_mgr/sdpe_requirement.json")
    target = bindings(hw.parent / "sdpe_mgr/sdpe_requirement.json")

    circuit = parse_netlist(source)
    parameters = {
        "C1": common["FSBB_PARAM_COUT"]["parameter"],
        "C2": common["FSBB_PARAM_CIN"]["parameter"],
        "L1": common["FSBB_PARAM_L"]["parameter"],
        "R1": common["FSBB_PARAM_L_ESR"]["parameter"],
        "R2": common["FSBB_PARAM_RLOAD_MIN"]["parameter"],
    }
    for element in parameters:
        circuit.element(element)
    circuit.elements = [
        replace(element, value_text=parameters[element.name])
        if element.name in parameters else element
        for element in circuit.elements
    ]

    vin_node = circuit.element("C2").nodes[0]
    for name, resistance in (
        ("C1", common["FSBB_PARAM_COUT_ESR"]["parameter"]),
        ("C2", sil["FSBB_PARAM_CIN_ESR"]["parameter"]),
    ):
        capacitor = circuit.element(name)
        esr_node = name + "_internal"
        circuit.nodes.append(esr_node)
        circuit.elements[circuit.elements.index(capacitor)] = replace(
            capacitor, nodes=(esr_node, capacitor.nodes[1])
        )
        circuit.elements.append(Element("R_ESR_" + name, "R", (capacitor.nodes[0], esr_node), resistance))

    circuit.observations.extend((
        Observation("V", vin_node, label="VIN"),
        Observation("I", "R2", label="IOUT"),
    ))
    pairs = resolve_mosfet_half_bridge_pairs(circuit, [("PWM1", "PWM2"), ("PWM4", "PWM3")])
    ron = float(sil["FSBB_MODEL_MOSFET_RON"]["parameter"])
    mosfets = [derive_mosfet_parameters(circuit, part) for part in circuit.elements if part.kind == "M"]
    model = build_piecewise_model(
        circuit,
        {f"{part.public_name}.Ron": ron for part in mosfets},
        include_mosfet_body_diodes=True,
        mosfet_half_bridge_pairs=pairs,
        # Keep diode conduction during dead time. A new channel command takes precedence.
        mosfet_model="half_bridge_approx",
        retain_topology_details=False,
    )
    if any(part.on_resistance != ron for part in model.parameters):
        raise ValueError("The FSBB plant did not apply the Simulink MOSFET on resistance")
    document = build_circuit_data(
        model, source_path=source, normal_step_s=float(target["CCTL_SIM_PLANT_STEP_S"]["literal"]),
        short_step_s=1e-9, method="backward_euler",
    )
    output.mkdir(parents=True, exist_ok=True)
    data = output / "fsbb_closed_loop.json"
    write_circuit_data(data, document)
    generate_cpp_project(data, output, class_name="FsbbClosedLoopCircuit",
                         output_stem="fsbb_closed_loop", show_progress=False)
    print(f"Generated closed-loop FSBB model in {output}")


if __name__ == "__main__":
    generate(Path(sys.argv[1]))
