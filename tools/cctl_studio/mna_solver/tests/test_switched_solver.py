from __future__ import annotations

from dataclasses import replace
import sys
import unittest
from pathlib import Path

import numpy as np


SOLVER_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOLVER_DIR))

import mna_solver as mna  # noqa: E402
import switched_solver as switched  # noqa: E402


TB_DIR = SOLVER_DIR / "tb"
BASIC_DIR = TB_DIR / "basic"
BUCK_DIR = TB_DIR / "buck"
BOOST_DIR = TB_DIR / "boost"
FSBB_DIR = TB_DIR / "fsbb"
SINV_DIR = TB_DIR / "sinv"
RECTIFIER_DIR = TB_DIR / "rectifier"
INV_DIR = TB_DIR / "inv"
BUCK_NPC_DIR = TB_DIR / "buck_npc"


class TinaExtendedParserTests(unittest.TestCase):
    def test_buck_models_and_continuations(self) -> None:
        circuit = mna.parse_netlist(BUCK_DIR / "buck.CIR")
        diode = circuit.element("D1")
        mosfet = circuit.element("MT1")
        self.assertEqual(diode.kind, "D")
        self.assertEqual(diode.nodes, ("0", "1"))
        self.assertEqual(mosfet.kind, "M")
        self.assertEqual(mosfet.nodes, ("5", "3", "1", "1"))
        self.assertAlmostEqual(circuit.models["D_1N1183_1"].numeric("CJO"), 460e-12)
        self.assertAlmostEqual(circuit.models["ME_2N6755_N_1"].numeric("RDS"), 600e3)

    def test_tina_idopamp_is_recognized_with_three_pins(self) -> None:
        circuit = mna.parse_netlist(BASIC_DIR / "1_OPAMP.CIR")
        opamp = circuit.element("XIOP1")
        self.assertEqual(opamp.kind, "O")
        self.assertEqual(opamp.nodes, ("4", "1", "VF1"))
        self.assertEqual(opamp.model_name, "IdOpamp")

        descriptor = mna.assemble_mna(circuit)
        state = mna.reduce_to_state_space(descriptor, mna.assemble_outputs(circuit, descriptor))
        self.assertEqual(state.A.shape, (1, 1))
        self.assertAlmostEqual(state.A[0, 0], -1000.0, places=8)
        dc = mna.frequency_response(state, [0.0]).response[0, 0, 0]
        self.assertAlmostEqual(dc.real, -1.0, places=12)
        self.assertAlmostEqual(dc.imag, 0.0, places=12)


class PiecewiseBuckTests(unittest.TestCase):
    def setUp(self) -> None:
        self.circuit = mna.parse_netlist(BUCK_DIR / "buck.CIR")
        self.model = switched.build_piecewise_model(self.circuit)

    def test_four_primary_and_two_body_diode_topologies(self) -> None:
        self.assertEqual(len(self.model.primary_topologies), 4)
        self.assertEqual(len(self.model.topologies), 6)
        self.assertEqual(self.model.gate_source_name, "VPWM1")
        for topology in self.model.topologies.values():
            self.assertNotIn("VPWM1", topology.state.input_names)
            self.assertEqual(topology.state.A.shape, (3, 3))
            self.assertIn("V(T1.D)", topology.state.output_names)
            self.assertIn("V(T1.S)", topology.state.output_names)
            self.assertIn("V(D1.A)", topology.state.output_names)
            self.assertIn("V(D1.K)", topology.state.output_names)

    def test_parameters_are_derived_from_tina_models(self) -> None:
        parameters = self.model.parameters
        self.assertAlmostEqual(parameters.diode_on_resistance, 0.002)
        self.assertAlmostEqual(parameters.diode_forward_voltage, 0.55)
        self.assertAlmostEqual(parameters.diode_junction_capacitance, 460e-12)
        self.assertAlmostEqual(parameters.mosfet_off_resistance, 600e3)
        self.assertAlmostEqual(parameters.mosfet_output_capacitance, 1.5716e-9)
        expected_ron = 0.06468 + 0.1207 + 1.0 / (21.14e-6 * (1.1 / 2e-6) * (10.0 - 3.128))
        self.assertAlmostEqual(parameters.mosfet_on_resistance, expected_ron, places=12)
        self.assertAlmostEqual(parameters.body_forward_voltage, 0.8)
        self.assertAlmostEqual(parameters.body_on_resistance, 0.18538)

    def test_junction_capacitance_uses_spice_forward_continuation(self) -> None:
        cjo, vj, grading, fc = 460e-12, 0.55, 0.44, 0.5
        self.assertAlmostEqual(
            switched.linearized_junction_capacitance(cjo, vj, grading, fc, 0.0), cjo
        )
        at_boundary = switched.linearized_junction_capacitance(cjo, vj, grading, fc, fc * vj)
        just_after = switched.linearized_junction_capacitance(cjo, vj, grading, fc, fc * vj + 1e-12)
        self.assertAlmostEqual(at_boundary, just_after, places=18)

    def test_10khz_pwm_uses_previous_voltage_and_ground_diode(self) -> None:
        result = switched.simulate_piecewise(
            self.model,
            duration=110e-6,
            dt=1e-9,
            pwm_frequency=10e3,
            pwm_duty=0.5,
            method="backward_euler",
        )
        self.assertTrue(np.all(np.isfinite(result.outputs)))
        self.assertTrue(np.any(result.gate))
        self.assertTrue(np.any(~result.gate))
        self.assertFalse(np.any(result.body_diode_on))
        self.assertIn("D_OFF__MOS_CHANNEL", result.topology)
        self.assertIn("D_ON__MOS_OFF", result.topology)
        self.assertTrue(np.any(result.diode_on))
        output = result.outputs[:, result.output_names.index("V(VF1)")]
        self.assertGreater(output[-1], 0.0)

    def test_50ms_model_parameter_run_reaches_steady_state(self) -> None:
        result = switched.simulate_piecewise(
            self.model,
            duration=50e-3,
            dt=100e-9,
            pwm_frequency=10e3,
            pwm_duty=0.5,
            transition_substep=0.5e-9,
            method="backward_euler",
        )
        output = result.outputs[:, result.output_names.index("V(VF1)")]
        tail = output[int(0.9 * len(output)) :]
        self.assertGreater(float(np.mean(tail)), 2.1)
        self.assertLess(float(np.mean(tail)), 2.25)
        self.assertLess(float(np.ptp(tail)), 0.2)

    def test_ideal_switch_override_still_reaches_half_supply_region(self) -> None:
        model = switched.build_piecewise_model(
            self.circuit, {"T1.Ron": 1e-3, "T1.BodyRon": 2e-3}
        )
        result = switched.simulate_piecewise(
            model,
            duration=50e-3,
            dt=100e-9,
            pwm_frequency=10e3,
            pwm_duty=0.5,
            transition_substep=0.5e-9,
            method="backward_euler",
        )
        output = result.outputs[:, result.output_names.index("V(VF1)")]
        tail = output[int(0.9 * len(output)) :]
        self.assertGreater(float(np.mean(tail)), 2.15)
        self.assertLess(float(np.mean(tail)), 2.3)


class PiecewiseBoostTests(unittest.TestCase):
    def setUp(self) -> None:
        self.circuit = mna.parse_netlist(BOOST_DIR / "BOOST.CIR")
        self.model = switched.build_piecewise_model(self.circuit)

    def test_boost_gate_and_topologies(self) -> None:
        self.assertEqual(self.model.gate_source_name, "VPWM1")
        self.assertEqual(self.model.mosfet.nodes, ("6", "4", "0", "0"))
        reference = next(iter(self.model.topologies.values())).state
        self.assertEqual(reference.A.shape, (4, 4))
        self.assertEqual(reference.output_names[:2], ["I(VAM1)", "V(VF1)"])

    def test_10khz_boost_uses_channel_and_output_diode(self) -> None:
        result = switched.simulate_piecewise(
            self.model,
            duration=5e-3,
            dt=100e-9,
            pwm_frequency=10e3,
            pwm_duty=0.5,
            transition_substep=1e-9,
            method="backward_euler",
        )
        voltage = result.outputs[:, result.output_names.index("V(VF1)")]
        self.assertTrue(np.all(np.isfinite(voltage)))
        self.assertGreater(float(np.mean(voltage[-1000:])), 8.0)
        self.assertLess(float(np.mean(voltage[-1000:])), 9.0)
        self.assertIn("D_OFF__MOS_CHANNEL", result.topology[-1000:])
        self.assertIn("D_ON__MOS_OFF", result.topology[-1000:])


class MultiMosfetFsbbTests(unittest.TestCase):
    def test_four_mosfets_expand_to_three_paths_each(self) -> None:
        circuit = mna.parse_netlist(FSBB_DIR / "FSBB.CIR")
        self.assertEqual(circuit.title, "FSBB (TINA Netlist Editor format)")
        self.assertNotIn("FSBB", [element.name for element in circuit.elements])

        model = switched.build_piecewise_model(circuit)
        self.assertIsInstance(model, switched.MultiMosfetLinearModel)
        self.assertEqual([item.name for item in model.mosfets], ["MT4", "MT3", "MT2", "MT1"])
        self.assertEqual([item.name for item in model.gate_sources], ["VPWM1", "VPWM2", "VPWM4", "VPWM3"])
        self.assertEqual(len(model.topologies), 3**4)
        for topology in model.topologies.values():
            self.assertEqual(topology.state.A.shape, (5, 5))
            for gate in model.gate_sources:
                self.assertNotIn(gate.name, topology.state.input_names)


class SinglePhaseInverterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.circuit = mna.parse_netlist(SINV_DIR / "SINV.CIR")

    def test_parallel_dc_link_capacitors_equal_one_summed_capacitor(self) -> None:
        capacitors = [
            element
            for element in self.circuit.elements
            if element.kind == "C" and set(element.nodes) == {"0", "5"}
        ]
        self.assertEqual([element.name for element in capacitors], ["C2", "C1"])
        self.assertAlmostEqual(
            sum(mna.parse_spice_value(element.value_text) or 0.0 for element in capacitors),
            200e-6,
        )

        equivalent_elements = [
            replace(element, value_text="200U") if element.name == "C1" else element
            for element in self.circuit.elements
            if element.name != "C2"
        ]
        equivalent = replace(self.circuit, elements=equivalent_elements)
        parallel_model = switched.build_piecewise_model(self.circuit)
        summed_model = switched.build_piecewise_model(equivalent)
        self.assertEqual(set(parallel_model.topologies), set(summed_model.topologies))
        for name, parallel in parallel_model.topologies.items():
            summed = summed_model.topologies[name]
            np.testing.assert_array_equal(parallel.descriptor.E, summed.descriptor.E)
            np.testing.assert_array_equal(parallel.descriptor.A, summed.descriptor.A)
            np.testing.assert_array_equal(parallel.descriptor.B, summed.descriptor.B)

    def test_differential_voltage_probe_is_stamped_between_both_nodes(self) -> None:
        self.assertEqual(
            [observation.name for observation in self.circuit.observations],
            ["I(VAM1)", "V(2,1)"],
        )
        model = switched.build_piecewise_model(self.circuit)
        self.assertIsInstance(model, switched.MultiMosfetLinearModel)
        self.assertEqual(len(model.topologies), 3**4)
        topology = next(iter(model.topologies.values()))
        outputs = mna.assemble_outputs(topology.circuit, topology.descriptor)
        row = outputs.names.index("V(2,1)")
        expected = np.zeros(topology.descriptor.A.shape[0])
        expected[topology.descriptor.node_indices["2"]] = 1.0
        expected[topology.descriptor.node_indices["1"]] = -1.0
        np.testing.assert_array_equal(outputs.L[row], expected)
        self.assertEqual(topology.state.output_names[:2], ["I(VAM1)", "V(2,1)"])


class ThreePhaseInverterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.circuit = mna.parse_netlist(INV_DIR / "INV.CIR")
        self.model = switched.build_piecewise_model(self.circuit)

    def test_six_mosfets_expand_to_729_compatible_topologies(self) -> None:
        self.assertIsInstance(self.model, switched.MultiMosfetLinearModel)
        self.assertEqual(
            [item.name for item in self.model.mosfets],
            ["MT6", "MT5", "MT4", "MT3", "MT2", "MT1"],
        )
        self.assertEqual(
            [item.name for item in self.model.gate_sources],
            ["VPWM3", "VPWM4", "VPWM1", "VPWM2", "VPWM5", "VPWM6"],
        )
        self.assertEqual(len(self.model.topologies), 3**6)
        for topology in self.model.topologies.values():
            self.assertEqual(topology.state.A.shape, (10, 10))
        self.assertEqual(self.circuit.element("R11").nodes, ("2", "0"))
        self.assertEqual(self.circuit.element("R12").nodes, ("1", "0"))
        self.assertEqual(self.circuit.element("R11").value_text, "1MEG")
        self.assertEqual(self.circuit.element("R12").value_text, "1MEG")

    def test_three_differential_phase_voltage_probes_are_preserved(self) -> None:
        self.assertEqual(
            [observation.name for observation in self.circuit.observations],
            ["I(VAM3)", "I(VAM2)", "I(VAM1)", "V(5,1)", "V(4,1)", "V(3,1)"],
        )
        reference = next(iter(self.model.topologies.values())).state
        self.assertEqual(
            reference.output_names[:6],
            ["I(VAM3)", "I(VAM2)", "I(VAM1)", "V(5,1)", "V(4,1)", "V(3,1)"],
        )


class NpcBuckTests(unittest.TestCase):
    def setUp(self) -> None:
        self.circuit = mna.parse_netlist(BUCK_NPC_DIR / "BUCK_NPC.CIR")
        self.model = switched.build_piecewise_model(self.circuit)

    def test_four_mosfets_and_two_clamp_diodes_expand_to_324_topologies(self) -> None:
        self.assertIsInstance(self.model, switched.MultiMosfetDiodeLinearModel)
        self.assertEqual(
            [item.name for item in self.model.mosfets], ["MT5", "MT2", "MT4", "MT3"]
        )
        self.assertEqual(
            [item.name for item in self.model.gate_sources],
            ["VPWM3", "VPWM4", "VPWM1", "VPWM2"],
        )
        self.assertEqual([item.name for item in self.model.diodes], ["D2", "D1"])
        self.assertEqual(len(self.model.topologies), 3**4 * 2**2)
        reference = next(iter(self.model.topologies.values())).state
        self.assertEqual(reference.A.shape, (7, 7))
        self.assertEqual(reference.output_names[:3], ["V(4)", "I(VAM1)", "V(VF1)"])
        for gate in self.model.gate_sources:
            self.assertNotIn(gate.name, reference.input_names)

    def test_updated_bus_has_explicit_source_series_resistors(self) -> None:
        self.assertEqual(self.circuit.element("R3").nodes, ("2", "3"))
        self.assertEqual(self.circuit.element("R4").nodes, ("1", "0"))
        self.assertAlmostEqual(self.circuit.element("R3").numeric_value({}), 50e-3)
        self.assertAlmostEqual(self.circuit.element("R4").numeric_value({}), 50e-3)


class ControlledPrechargeRectifierTests(unittest.TestCase):
    def setUp(self) -> None:
        self.circuit = mna.parse_netlist(RECTIFIER_DIR / "RECTIFIER.CIR")
        self.model = switched.build_piecewise_model(self.circuit)

    def test_vswitch_is_parsed_and_exposed_as_32_linear_topologies(self) -> None:
        switch = self.circuit.element("SW2")
        self.assertEqual(switch.kind, "S")
        self.assertEqual(switch.nodes, ("VF1", "5", "3", "0"))
        self.assertEqual(switch.model_name, "S_VSWITCH_1")
        self.assertIsInstance(self.model, switched.MultiDiodeSwitchLinearModel)
        self.assertEqual([item.name for item in self.model.diodes], ["D4", "D3", "D2", "D1"])
        self.assertEqual([item.name for item in self.model.switches], ["SW2"])
        self.assertEqual([item.name for item in self.model.control_sources], ["VSWGPIO1"])
        self.assertEqual(len(self.model.topologies), 2**5)
        self.assertAlmostEqual(self.model.switch_parameters[0].on_resistance, 1e-3)
        self.assertAlmostEqual(self.model.switch_parameters[0].off_resistance, 1e9)
        for topology in self.model.topologies.values():
            self.assertEqual(topology.state.A.shape, (1, 1))
            self.assertEqual(topology.state.input_names[0], "VS1")

    def test_diode_model_values_are_extracted_for_each_bridge_device(self) -> None:
        for item in self.model.diode_parameters:
            self.assertAlmostEqual(item.forward_voltage, 0.55)
            self.assertAlmostEqual(item.on_resistance, 2e-3)
            self.assertAlmostEqual(item.off_resistance, 1e9)
            self.assertAlmostEqual(item.junction_capacitance, 460e-12)


if __name__ == "__main__":
    unittest.main()
