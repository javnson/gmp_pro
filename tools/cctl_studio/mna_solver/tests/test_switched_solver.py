from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


SOLVER_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOLVER_DIR))

import mna_solver as mna  # noqa: E402
import switched_solver as switched  # noqa: E402


TB_DIR = SOLVER_DIR / "tb"


class TinaExtendedParserTests(unittest.TestCase):
    def test_buck_models_and_continuations(self) -> None:
        circuit = mna.parse_netlist(TB_DIR / "2_buck.CIR")
        diode = circuit.element("D1")
        mosfet = circuit.element("MT1")
        self.assertEqual(diode.kind, "D")
        self.assertEqual(diode.nodes, ("0", "4"))
        self.assertEqual(mosfet.kind, "M")
        self.assertEqual(mosfet.nodes, ("4", "1", "3", "3"))
        self.assertAlmostEqual(circuit.models["D_1N1183_1"].numeric("CJO"), 460e-12)
        self.assertAlmostEqual(circuit.models["ME_2N6755_N_1"].numeric("RDS"), 600e3)

    def test_tina_idopamp_is_recognized_with_three_pins(self) -> None:
        circuit = mna.parse_netlist(TB_DIR / "1_OPAMP.CIR")
        opamp = circuit.element("XIOP1")
        self.assertEqual(opamp.kind, "O")
        self.assertEqual(opamp.nodes, ("3", "1", "VF1"))
        self.assertEqual(opamp.model_name, "IdOpamp")

        descriptor = mna.assemble_mna(circuit)
        # This is a singular pencil, not a parser failure: with the conventional
        # TINA IdOpamp (+,-,out) order, node 3 is pulled to ground while the
        # ideal constraint equates it to the ideal source at node 1.
        for frequency in (0.0, 1.0, 1j, 1000j):
            self.assertLess(np.linalg.matrix_rank(frequency * descriptor.E - descriptor.A), descriptor.A.shape[0])
        with self.assertRaises(mna.NetlistError):
            mna.reduce_to_state_space(descriptor, mna.assemble_outputs(circuit, descriptor))


class PiecewiseBuckTests(unittest.TestCase):
    def setUp(self) -> None:
        self.circuit = mna.parse_netlist(TB_DIR / "2_buck.CIR")
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
        self.assertAlmostEqual(parameters.diode_junction_capacitance, 460e-12)
        self.assertAlmostEqual(parameters.mosfet_off_resistance, 600e3)
        self.assertAlmostEqual(parameters.mosfet_output_capacitance, 1.5716e-9)
        self.assertAlmostEqual(parameters.mosfet_on_resistance, 1e-3)
        self.assertAlmostEqual(parameters.body_on_resistance, 2e-3)

    def test_10khz_pwm_uses_previous_voltage_and_detects_reverse_conduction(self) -> None:
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
        self.assertTrue(np.any(result.body_diode_on))
        self.assertIn("D_OFF__MOS_CHANNEL", result.topology)
        self.assertIn("D_OFF__MOS_BODY_DIODE", result.topology)
        # In the supplied netlist S is tied to +5 V and D is the switch node;
        # the intrinsic S->D body diode therefore catches the falling node
        # before the ground-referenced D1 freewheel diode turns on.
        self.assertFalse(np.any(result.diode_on))
        output = result.outputs[:, result.output_names.index("V(VF1)")]
        self.assertGreater(output[-1], 0.0)

    def test_50ms_run_reaches_half_supply_region(self) -> None:
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
        # Ideal half-supply is 2.5 V.  Vf and finite path resistances lower the
        # PWL result, but the 50 ms run must settle in that neighborhood.
        self.assertGreater(float(np.mean(tail)), 2.15)
        self.assertLess(float(np.mean(tail)), 2.4)
        self.assertLess(float(np.ptp(tail)), 0.03)


if __name__ == "__main__":
    unittest.main()
