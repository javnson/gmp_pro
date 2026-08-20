from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


SOLVER_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOLVER_DIR))

import mna_solver as mna  # noqa: E402


TB_DIR = SOLVER_DIR / "tb"
BASIC_DIR = TB_DIR / "basic"


class ValueParserTests(unittest.TestCase):
    def test_spice_suffixes(self) -> None:
        self.assertEqual(mna.parse_spice_value("1K"), 1000.0)
        self.assertEqual(mna.parse_spice_value("1MEG"), 1e6)
        self.assertEqual(mna.parse_spice_value("2N"), 2e-9)
        self.assertEqual(mna.parse_spice_value("10E-6"), 10e-6)
        self.assertIsNone(mna.parse_spice_value("Symbolic"))


class NetlistTests(unittest.TestCase):
    def test_all_supplied_netlists_parse_and_build_symbolic_mna(self) -> None:
        paths = sorted(
            {path.resolve() for pattern in ("*.cir", "*.CIR") for path in BASIC_DIR.glob(pattern)}
        )
        for case_dir in (
            TB_DIR / "buck",
            TB_DIR / "boost",
            TB_DIR / "fsbb",
            TB_DIR / "sinv",
        ):
            paths.extend(
                sorted(
                    {path.resolve() for pattern in ("*.cir", "*.CIR") for path in case_dir.glob(pattern)}
                )
            )
        self.assertGreaterEqual(len(paths), 10)
        for path in paths:
            with self.subTest(path=path.name):
                circuit = mna.parse_netlist(path)
                if any(element.kind in {"D", "M"} for element in circuit.elements):
                    with self.assertRaises(mna.NetlistError):
                        mna.assemble_symbolic_mna(circuit)
                else:
                    symbolic = mna.assemble_symbolic_mna(circuit)
                    self.assertEqual(symbolic.E.nrows(), symbolic.A.nrows())
                    self.assertEqual(symbolic.A.nrows(), symbolic.A.ncols())

    def test_tina_directives_current_arrow_and_probe(self) -> None:
        circuit = mna.parse_netlist(BASIC_DIR / "0_divider.CIR")
        self.assertEqual([item.name for item in circuit.observations], ["V(VF1)", "I(VAM1)"])
        self.assertEqual(circuit.element("VAM1").kind, "AMMETER")
        self.assertTrue(any(item.upper().startswith(".LIB") for item in circuit.ignored_directives))
        descriptor = mna.assemble_mna(circuit)
        self.assertEqual(descriptor.input_names, ["VS1"])

    def test_symbolic_element_names_are_exact_matrix_symbols(self) -> None:
        circuit = mna.parse_netlist(BASIC_DIR / "example6.cir")
        symbolic = mna.assemble_symbolic_mna(circuit)
        matrix_text = f"{symbolic.E}\n{symbolic.A}"
        self.assertIn("C1", matrix_text)
        self.assertIn("L1", matrix_text)
        self.assertIn("R1", matrix_text)


class SolverTests(unittest.TestCase):
    def _temporary_netlist(self, contents: str) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "circuit.cir"
        path.write_text(contents, encoding="utf-8")
        return path

    def test_tina_divider_output_equation(self) -> None:
        circuit = mna.parse_netlist(BASIC_DIR / "0_divider.CIR")
        descriptor = mna.assemble_mna(circuit)
        state = mna.reduce_to_state_space(descriptor, mna.assemble_outputs(circuit, descriptor))
        self.assertEqual(state.A.shape, (0, 0))
        np.testing.assert_allclose(state.D[:, 0], [0.5, 0.0005], rtol=1e-12, atol=1e-12)

    def test_rc_state_reduction_euler_and_frequency_response(self) -> None:
        path = self._temporary_netlist(
            "RC low pass\n"
            "V1 in 0 1\n"
            "R1 in out 1K\n"
            "C1 out 0 1U\n"
            ".PROBE V(out) I(R1)\n"
            ".END\n"
        )
        circuit = mna.parse_netlist(path)
        descriptor = mna.assemble_mna(circuit)
        state = mna.reduce_to_state_space(descriptor, mna.assemble_outputs(circuit, descriptor))
        self.assertEqual(state.A.shape, (1, 1))
        np.testing.assert_allclose(state.A[0, 0], -1000.0, rtol=1e-12)

        discrete = mna.discretize(state, 1e-5)
        equations = mna.discrete_equations(discrete)
        self.assertTrue(any("x0[k+1]" in equation for equation in equations))
        self.assertTrue(any("V(out)[k]" in equation for equation in equations))
        result = mna.simulate(discrete, 1e-3)
        self.assertAlmostEqual(result.outputs[-1, 0], 1.0 - 0.99**100, places=12)

        response = mna.frequency_response(state, [0.0, 1000.0 / (2.0 * np.pi)])
        self.assertAlmostEqual(abs(response.response[0, 0, 0]), 1.0, places=12)
        self.assertAlmostEqual(response.magnitude_db[1, 0, 0], -3.0102999566, places=9)
        self.assertAlmostEqual(response.phase_deg[1, 0, 0], -45.0, places=9)

    def test_dependent_source_examples_reduce_with_parameter_values(self) -> None:
        for filename, parameter in (
            ("exampleE.cir", {"Ea": 2}),
            ("exampleF.cir", {"Fa": 3}),
            ("exampleG.cir", {"Ga": 0.001}),
            ("exampleH.cir", {"Ha": 1000}),
        ):
            with self.subTest(filename=filename):
                circuit = mna.parse_netlist(BASIC_DIR / filename)
                descriptor = mna.assemble_mna(circuit, parameter)
                state = mna.reduce_to_state_space(descriptor, mna.assemble_outputs(circuit, descriptor, parameter))
                self.assertEqual(state.A.shape, (0, 0))

    def test_unresolved_passive_parameter_is_reported(self) -> None:
        circuit = mna.parse_netlist(BASIC_DIR / "example5.cir")
        with self.assertRaises(mna.UnresolvedParameterError) as caught:
            mna.assemble_mna(circuit)
        self.assertEqual(caught.exception.names, ["R1", "R2"])


if __name__ == "__main__":
    unittest.main()
