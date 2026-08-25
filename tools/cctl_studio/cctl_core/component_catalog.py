"""Component metadata aligned with the MNA Solver netlist parser."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


@dataclass(frozen=True)
class CatalogPort:
    port_id: str
    label: str


@dataclass(frozen=True)
class ParameterSpec:
    parameter_id: str
    label: str
    default: Any
    description: str = ""


@dataclass(frozen=True)
class MnaComponentSpec:
    type_id: str
    display_name: str
    designator: str
    symbol: str
    ports: tuple[CatalogPort, ...]
    parameters: tuple[ParameterSpec, ...] = ()
    netlist_kind: str | None = None
    default_model_line: str | None = None
    integrated_pwm_driver: bool = False

    @property
    def defaults(self) -> Mapping[str, Any]:
        return {parameter.parameter_id: parameter.default for parameter in self.parameters}


def _port(port_id: str, label: str) -> CatalogPort:
    return CatalogPort(port_id, label)


def _parameter(
    parameter_id: str,
    label: str,
    default: Any,
    description: str = "",
) -> ParameterSpec:
    return ParameterSpec(parameter_id, label, default, description)


# Keep this set synchronized with parse_netlist() in mna_solver/mna_solver.py.
# Ground is editor-only and is translated to node 0 rather than an element line.
MNA_COMPONENTS: dict[str, MnaComponentSpec] = {
    "circuit.ground": MnaComponentSpec(
        "circuit.ground",
        "GND / GMD (Ground)",
        "GND",
        "ground",
        (_port("node", "GND"),),
    ),
    "circuit.junction": MnaComponentSpec(
        "circuit.junction",
        "Junction",
        "J",
        "junction",
        (_port("node", ""),),
    ),
    "circuit.resistor": MnaComponentSpec(
        "circuit.resistor",
        "Resistor",
        "R",
        "resistor",
        (_port("p", "+"), _port("n", "−")),
        (_parameter("resistance", "Resistance", "1k", "SPICE resistance value"),),
        "R",
    ),
    "circuit.capacitor": MnaComponentSpec(
        "circuit.capacitor",
        "Capacitor",
        "C",
        "capacitor",
        (_port("p", "+"), _port("n", "−")),
        (_parameter("capacitance", "Capacitance", "1u"),),
        "C",
    ),
    "circuit.inductor": MnaComponentSpec(
        "circuit.inductor",
        "Inductor",
        "L",
        "inductor",
        (_port("p", "+"), _port("n", "−")),
        (_parameter("inductance", "Inductance", "1m"),),
        "L",
    ),
    "circuit.voltage_source": MnaComponentSpec(
        "circuit.voltage_source",
        "Voltage Source",
        "V",
        "voltage_source",
        (_port("p", "+"), _port("n", "−")),
        (_parameter("voltage", "Voltage", "24"),),
        "V",
    ),
    "circuit.current_source": MnaComponentSpec(
        "circuit.current_source",
        "Current Source",
        "I",
        "current_source",
        (_port("p", "+"), _port("n", "−")),
        (_parameter("current", "Current", "1"),),
        "I",
    ),
    "circuit.ideal_opamp": MnaComponentSpec(
        "circuit.ideal_opamp",
        "Ideal Op-Amp",
        "O",
        "opamp",
        (_port("plus", "+"), _port("minus", "−"), _port("out", "OUT")),
        netlist_kind="O",
    ),
    "circuit.idopamp": MnaComponentSpec(
        "circuit.idopamp",
        "IdOpamp Subcircuit",
        "X",
        "opamp",
        (_port("plus", "+"), _port("minus", "−"), _port("out", "OUT")),
        (_parameter("subcircuit", "Subcircuit", "IdOpamp"),),
        "X",
    ),
    "circuit.vcvs": MnaComponentSpec(
        "circuit.vcvs",
        "VCVS",
        "E",
        "dependent_voltage",
        (
            _port("p", "+"),
            _port("n", "−"),
            _port("control_p", "C+"),
            _port("control_n", "C−"),
        ),
        (_parameter("gain", "Voltage gain", "1"),),
        "E",
    ),
    "circuit.vccs": MnaComponentSpec(
        "circuit.vccs",
        "VCCS",
        "G",
        "dependent_current",
        (
            _port("p", "+"),
            _port("n", "−"),
            _port("control_p", "C+"),
            _port("control_n", "C−"),
        ),
        (_parameter("gain", "Transconductance", "1"),),
        "G",
    ),
    "circuit.cccs": MnaComponentSpec(
        "circuit.cccs",
        "CCCS",
        "F",
        "dependent_current",
        (_port("p", "+"), _port("n", "−")),
        (
            _parameter("control_source", "Control voltage source", "V1"),
            _parameter("gain", "Current gain", "1"),
        ),
        "F",
    ),
    "circuit.ccvs": MnaComponentSpec(
        "circuit.ccvs",
        "CCVS",
        "H",
        "dependent_voltage",
        (_port("p", "+"), _port("n", "−")),
        (
            _parameter("control_source", "Control voltage source", "V1"),
            _parameter("gain", "Transresistance", "1"),
        ),
        "H",
    ),
    "circuit.diode": MnaComponentSpec(
        "circuit.diode",
        "Diode",
        "D",
        "diode",
        (_port("anode", "A"), _port("cathode", "K")),
        (
            _parameter("model", "Model name", "D_DEFAULT"),
            _parameter("model_line", "Model definition", ".MODEL D_DEFAULT D(IS=1e-14 N=1)"),
        ),
        "D",
        ".MODEL D_DEFAULT D(IS=1e-14 N=1)",
    ),
    "circuit.mosfet": MnaComponentSpec(
        "circuit.mosfet",
        "PWM-driven N-MOSFET",
        "MT",
        "mosfet",
        (
            _port("drain", "D"),
            _port("source", "S"),
        ),
        (
            _parameter("model", "Model name", "M_DEFAULT"),
            _parameter(
                "model_line",
                "Model definition",
                ".MODEL M_DEFAULT NMOS(LEVEL=3 VTO=3.128 KP=21.14U L=2U W=1.1 RD=64.68M RS=120.7M RDS=600K PB=800M CBD=1.261N CGDO=310.6P)",
            ),
        ),
        "M",
        default_model_line=".MODEL M_DEFAULT NMOS(LEVEL=3 VTO=3.128 KP=21.14U L=2U W=1.1 RD=64.68M RS=120.7M RDS=600K PB=800M CBD=1.261N CGDO=310.6P)",
        integrated_pwm_driver=True,
    ),
    "circuit.voltage_switch": MnaComponentSpec(
        "circuit.voltage_switch",
        "PWM-driven Voltage Switch",
        "SW",
        "switch",
        (
            _port("p", "+"),
            _port("n", "−"),
        ),
        (
            _parameter("model", "Model name", "S_DEFAULT"),
            _parameter("model_line", "Model definition", ".MODEL S_DEFAULT VSWITCH(RON=0 ROFF=1G VON=1 VOFF=0)"),
        ),
        "S",
        default_model_line=".MODEL S_DEFAULT VSWITCH(RON=0 ROFF=1G VON=1 VOFF=0)",
        integrated_pwm_driver=True,
    ),
    "circuit.ammeter": MnaComponentSpec(
        "circuit.ammeter",
        "Current Arrow / Ammeter",
        "VAM",
        "ammeter",
        (_port("p", "+"), _port("n", "−")),
        netlist_kind="AMMETER",
    ),
}


MNA_PARSER_KINDS = frozenset("RLCVIOEGFHDMSX") | {"AMMETER"}
