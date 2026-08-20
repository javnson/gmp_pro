from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


SOLVER_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOLVER_DIR))

import circuit_data as data  # noqa: E402
import cpp_codegen as codegen  # noqa: E402
import mna_solver as mna  # noqa: E402
import switched_solver as switched  # noqa: E402


TB_DIR = SOLVER_DIR / "tb"
BUCK_DIR = TB_DIR / "buck"
BOOST_DIR = TB_DIR / "boost"
FSBB_DIR = TB_DIR / "fsbb"


class CircuitDataTests(unittest.TestCase):
    def setUp(self) -> None:
        source = BUCK_DIR / "buck.CIR"
        circuit = mna.parse_netlist(source)
        self.document = data.build_circuit_data(
            switched.build_piecewise_model(circuit),
            source_path=source,
            normal_step_s=100e-9,
            short_step_s=1e-9,
        )

    def test_document_contains_ports_models_and_six_dual_step_topologies(self) -> None:
        data.validate_circuit_data(self.document)
        self.assertEqual(self.document["schema"]["name"], data.SCHEMA_NAME)
        self.assertEqual([port["name"] for port in self.document["ports"]["inputs"]], ["PWM", "VS1"])
        self.assertEqual(
            [port["name"] for port in self.document["ports"]["outputs"]],
            ["I(VAM1)", "V(VF1)"],
        )
        self.assertEqual(len(self.document["topologies"]), 6)
        diode = self.document["devices"]["diode"]
        self.assertAlmostEqual(diode["model"]["parameters"]["VJ"]["value"], 0.55)
        self.assertEqual(diode["extraction"]["forward_voltage"], "VJ")
        mosfet = self.document["devices"]["mosfet"]
        self.assertEqual(mosfet["extraction"]["off_resistance"], "RDS")
        for topology in self.document["topologies"]:
            normal = np.asarray(topology["discrete"]["normal"]["A"])
            short = np.asarray(topology["discrete"]["short"]["A"])
            self.assertEqual(normal.shape, (3, 3))
            self.assertFalse(np.array_equal(normal, short))

    def test_json_round_trip_and_data_only_simulation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "buck.json"
            data.write_circuit_data(path, self.document)
            loaded = data.load_circuit_data(path)
            self.assertEqual(loaded["circuit"]["source"]["sha256"], self.document["circuit"]["source"]["sha256"])
            result = data.simulate_circuit_data(loaded, 1e-3, startup_short_steps=20)
        self.assertTrue(np.all(np.isfinite(result.outputs)))
        self.assertGreater(result.outputs[-1, 0], 0.0)
        self.assertIn("D_OFF__MOS_CHANNEL", result.topology)

    def test_cpp_generator_exposes_requested_class_interface(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "buck.json"
            data.write_circuit_data(data_path, self.document)
            files = codegen.generate_cpp_project(data_path, Path(directory) / "cpp", "BuckCircuit")
            header = files["header"].read_text(encoding="utf-8")
        self.assertEqual(set(files), {"header"})
        self.assertIn("class BuckCircuit", header)
        self.assertIn("step_short", header)
        self.assertIn("step_normal", header)
        self.assertIn("operator()", header)
        self.assertIn("Outputs output", header)
        self.assertIn("operator[](std::string_view", header)
        self.assertIn("std::uint32_t PWM", header)
        self.assertIn("double VS1", header)

    def test_default_cpp_class_uses_netlist_file_stem(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "buck.json"
            data.write_circuit_data(data_path, self.document)
            files = codegen.generate_cpp_project(data_path, Path(directory) / "cpp")
            header = files["header"].read_text(encoding="utf-8")
        self.assertIn("class BuckCircuit", header)
        self.assertEqual(set(files), {"header"})

    def test_fsbb_exports_four_pwm_ports_and_81_mosfet_topologies(self) -> None:
        source = FSBB_DIR / "FSBB.CIR"
        model = switched.build_piecewise_model(mna.parse_netlist(source))
        self.assertIsInstance(model, switched.MultiMosfetLinearModel)
        self.assertEqual(len(model.mosfets), 4)
        self.assertEqual(len(model.topologies), 81)

        document = data.build_circuit_data(
            model,
            source_path=source,
            normal_step_s=100e-9,
            short_step_s=1e-9,
        )
        data.validate_circuit_data(document)
        self.assertEqual(
            [port["name"] for port in document["ports"]["inputs"]],
            ["PWM1", "PWM2", "PWM3", "PWM4", "VS1"],
        )
        self.assertEqual(document["switching"]["kind"], "multi_mosfet")
        self.assertEqual(document["switching"]["topology_count"], 81)
        self.assertEqual(
            [(item["instance"], item["pwm_port"]) for item in document["switching"]["switches"]],
            [("MT4", "PWM1"), ("MT3", "PWM2"), ("MT2", "PWM4"), ("MT1", "PWM3")],
        )

        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "FSBB.json"
            data.write_circuit_data(data_path, document)
            files = codegen.generate_cpp_project(data_path, Path(directory) / "cpp")
            header = files["header"].read_text(encoding="utf-8")
        self.assertIn("class FsbbCircuit", header)
        self.assertIn("static constexpr std::size_t topology_count = 81", header)
        for pwm in ("PWM1", "PWM2", "PWM3", "PWM4"):
            self.assertIn(f"std::uint32_t {pwm}", header)
        self.assertIn("std::array<bool, 4> body_on_", header)

    def test_boost_data_reaches_periodic_boost_waveform(self) -> None:
        source = BOOST_DIR / "BOOST.CIR"
        document = data.build_circuit_data(
            switched.build_piecewise_model(mna.parse_netlist(source)),
            source_path=source,
            normal_step_s=100e-9,
            short_step_s=1e-9,
        )
        result = data.simulate_circuit_data(document, 5e-3, startup_short_steps=20)
        voltage = result.outputs[:, result.output_names.index("V(VF1)")]
        last_cycle = voltage[-1000:]
        self.assertGreater(float(np.mean(last_cycle)), 8.0)
        self.assertLess(float(np.mean(last_cycle)), 9.0)
        self.assertLess(float(np.ptp(last_cycle)), 0.6)
        self.assertIn("D_ON__MOS_OFF", result.topology[-1000:])


if __name__ == "__main__":
    unittest.main()
