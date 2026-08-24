import sys
import tempfile
import unittest
from pathlib import Path


CORE_ROOT = Path(__file__).resolve().parents[1]
MNA_ROOT = CORE_ROOT.parent / "mna_solver"
sys.path.insert(0, str(CORE_ROOT))
sys.path.insert(0, str(MNA_ROOT))

import cctl_studio  # noqa: E402
from component_catalog import MNA_COMPONENTS, MNA_PARSER_KINDS  # noqa: E402
from editor_model import EditorDocument  # noqa: E402
from hierarchy_model import HierarchyDocument  # noqa: E402
from mna_export import export_mna_netlist  # noqa: E402
from mna_solver import parse_netlist  # noqa: E402
from switched_solver import (  # noqa: E402
    build_multi_diode_switch_model,
    build_multi_mosfet_model,
)


class MnaExportTests(unittest.TestCase):
    def setUp(self):
        components = cctl_studio.load_components([cctl_studio.BUILTIN_LIBRARY])
        self.document = EditorDocument(None, components)
        self.hierarchy = HierarchyDocument(self.document)
        self.layer_id = "circuit_main"

    def parse_export(self, text: str):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "studio_export.cir"
            path.write_text(text, encoding="utf-8")
            return parse_netlist(path)

    def test_catalog_covers_every_mna_parser_element_kind(self):
        exported_kinds = {
            spec.netlist_kind
            for spec in MNA_COMPONENTS.values()
            if spec.netlist_kind is not None
        }
        self.assertEqual(exported_kinds, MNA_PARSER_KINDS)

    def test_rc_wires_transforms_and_export_round_trip(self):
        ground = self.hierarchy.add_node(self.layer_id, "circuit.ground", 0, 160)
        voltage = self.hierarchy.add_node(
            self.layer_id, "circuit.voltage_source", 0, 0
        )
        resistor = self.hierarchy.add_node(
            self.layer_id, "circuit.resistor", 180, 0
        )
        capacitor = self.hierarchy.add_node(
            self.layer_id, "circuit.capacitor", 360, 100
        )
        self.hierarchy.connect(self.layer_id, (voltage, "n"), (ground, "node"))
        self.hierarchy.connect(self.layer_id, (voltage, "p"), (resistor, "p"))
        wire_id = self.hierarchy.connect(
            self.layer_id,
            (resistor, "n"),
            (capacitor, "p"),
            [(260, 0), (260, 100)],
        )
        self.hierarchy.connect(self.layer_id, (capacitor, "n"), (ground, "node"))
        self.hierarchy.set_transform(
            self.layer_id, [resistor], rotation_delta=90, toggle_mirror=True
        )

        resistor_node = self.hierarchy.node(self.layer_id, resistor)
        self.assertEqual(resistor_node["rotation"], 90)
        self.assertTrue(resistor_node["mirror_x"])
        self.assertEqual(
            self.hierarchy.connection(self.layer_id, wire_id)["points"],
            [{"x": 260.0, "y": 0.0}, {"x": 260.0, "y": 100.0}],
        )

        netlist = export_mna_netlist(self.hierarchy, self.layer_id, "RC round trip")
        parsed = self.parse_export(netlist)
        self.assertEqual([element.kind for element in parsed.elements], ["V", "R", "C"])
        self.assertIn("0", netlist)

    def test_full_palette_exports_to_a_netlist_accepted_by_mna_solver(self):
        ground = self.hierarchy.add_node(self.layer_id, "circuit.ground", 0, 0)
        created = []
        for index, type_id in enumerate(MNA_COMPONENTS):
            if type_id == "circuit.ground":
                continue
            node_id = self.hierarchy.add_node(
                self.layer_id, type_id, 180 * (index % 5), 120 * (index // 5)
            )
            created.append(node_id)
            first_port = MNA_COMPONENTS[type_id].ports[0].port_id
            self.hierarchy.connect(
                self.layer_id, (node_id, first_port), (ground, "node")
            )

        netlist = export_mna_netlist(self.hierarchy, self.layer_id, "All MNA elements")
        parsed = self.parse_export(netlist)
        driven_count = sum(
            spec.integrated_pwm_driver for spec in MNA_COMPONENTS.values()
        )
        self.assertEqual(len(parsed.elements), len(created) + driven_count)
        self.assertEqual(
            {element.kind for element in parsed.elements},
            {"R", "L", "C", "V", "I", "O", "E", "G", "F", "H", "D", "M", "S", "AMMETER"},
        )
        self.assertEqual(
            [element.name for element in parsed.elements if element.name.startswith("VPWM")],
            ["VPWM1", "VPWM2"],
        )
        self.assertEqual(set(parsed.models), {"D_DEFAULT", "M_DEFAULT", "S_DEFAULT"})

    def test_pwm_driven_mosfet_matches_solver_gate_source_contract(self):
        ground = self.hierarchy.add_node(self.layer_id, "circuit.ground", 0, 200)
        supply = self.hierarchy.add_node(
            self.layer_id, "circuit.voltage_source", -240, 0
        )
        resistor = self.hierarchy.add_node(
            self.layer_id, "circuit.resistor", 0, 0
        )
        mosfet = self.hierarchy.add_node(
            self.layer_id, "circuit.mosfet", 240, 100
        )
        self.hierarchy.connect(self.layer_id, (supply, "n"), (ground, "node"))
        self.hierarchy.connect(self.layer_id, (supply, "p"), (resistor, "p"))
        self.hierarchy.connect(self.layer_id, (resistor, "n"), (mosfet, "drain"))
        self.hierarchy.connect(self.layer_id, (mosfet, "source"), (ground, "node"))

        netlist = export_mna_netlist(self.hierarchy, self.layer_id, "MOS driver")
        circuit = self.parse_export(netlist)
        device = next(element for element in circuit.elements if element.kind == "M")
        driver = next(element for element in circuit.elements if element.name == "VPWM1")

        self.assertEqual(mosfet, "MT1")
        self.assertEqual([port.port_id for port in MNA_COMPONENTS["circuit.mosfet"].ports], ["drain", "source"])
        self.assertEqual(device.nodes[1], driver.nodes[0])
        self.assertEqual(device.nodes[2], device.nodes[3])
        self.assertEqual(device.nodes[2], driver.nodes[1])
        model = build_multi_mosfet_model(circuit, include_body_diode_states=False)
        self.assertEqual(model.gate_sources[0].name, "VPWM1")

    def test_pwm_driven_voltage_switch_matches_solver_control_contract(self):
        ground = self.hierarchy.add_node(self.layer_id, "circuit.ground", 0, 200)
        supply = self.hierarchy.add_node(
            self.layer_id, "circuit.voltage_source", -240, 0
        )
        resistor = self.hierarchy.add_node(
            self.layer_id, "circuit.resistor", 0, 0
        )
        switch = self.hierarchy.add_node(
            self.layer_id, "circuit.voltage_switch", 240, 100
        )
        self.hierarchy.connect(self.layer_id, (supply, "n"), (ground, "node"))
        self.hierarchy.connect(self.layer_id, (supply, "p"), (resistor, "p"))
        self.hierarchy.connect(self.layer_id, (resistor, "n"), (switch, "p"))
        self.hierarchy.connect(self.layer_id, (switch, "n"), (ground, "node"))

        netlist = export_mna_netlist(self.hierarchy, self.layer_id, "Switch driver")
        circuit = self.parse_export(netlist)
        device = next(element for element in circuit.elements if element.kind == "S")
        driver = next(element for element in circuit.elements if element.name == "VPWM1")

        self.assertEqual(switch, "SW1")
        self.assertEqual([port.port_id for port in MNA_COMPONENTS["circuit.voltage_switch"].ports], ["p", "n"])
        self.assertEqual(device.nodes[2], driver.nodes[0])
        self.assertEqual(device.nodes[3], driver.nodes[1])
        model = build_multi_diode_switch_model(circuit)
        self.assertEqual(model.control_sources[0].name, "VPWM1")

    def test_nonlinear_model_name_must_match_its_definition(self):
        diode = self.hierarchy.add_node(
            self.layer_id, "circuit.diode", 100, 100
        )
        self.hierarchy.node(self.layer_id, diode)["parameters"]["model"] = "OTHER"
        with self.assertRaisesRegex(cctl_studio.StudioError, "must match"):
            export_mna_netlist(self.hierarchy, self.layer_id)


if __name__ == "__main__":
    unittest.main()
