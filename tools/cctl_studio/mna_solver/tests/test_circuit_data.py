from __future__ import annotations

import copy
import contextlib
import io
import json
import struct
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
SINV_DIR = TB_DIR / "sinv"
RECTIFIER_DIR = TB_DIR / "rectifier"
INV_DIR = TB_DIR / "inv"
BUCK_NPC_DIR = TB_DIR / "buck_npc"
PMSM_DIR = TB_DIR / "pmsm"


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
        self.assertEqual(
            self.document["solver"]["matrix_tolerance"], data.DEFAULT_MATRIX_TOLERANCE
        )
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

    def test_dimension_summary_matches_state_space_and_public_ports(self) -> None:
        dimensions = data.circuit_dimension_summary(self.document)
        self.assertEqual(dimensions["states"], 3)
        self.assertEqual(dimensions["analog_inputs"], 1)
        self.assertEqual(dimensions["command_inputs"], 1)
        self.assertEqual(dimensions["external_inputs"], 2)
        self.assertEqual(dimensions["signals"], len(self.document["signals"]["names"]))
        self.assertEqual(dimensions["public_outputs"], 2)

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            data.print_circuit_dimensions(self.document)
        self.assertIn("state space:         x=3, u=1, y=", output.getvalue())
        self.assertIn("matrix dimensions:   A=3x3, B=3x1", output.getvalue())
        self.assertIn("inputs=2 (analog=1, commands=1), outputs=2", output.getvalue())

    def test_cpp_generation_progress_reports_dimensions(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "buck.json"
            data.write_circuit_data(data_path, self.document)
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                codegen.generate_cpp_project(
                    data_path,
                    Path(directory) / "cpp",
                    show_progress=True,
                )
        self.assertIn("state space:         x=3, u=1, y=", output.getvalue())
        self.assertIn("external ports:      inputs=2", output.getvalue())

    def test_vadc_outputs_are_marked_as_adc_sample_voltages(self) -> None:
        names = ["V(VADC_VDC)", "V(VADC_VA)", "V(VADC_IA)", "V(OUT)"]
        fields = data._unique_output_fields(names)
        ports = [
            data._output_port_document(name, field, index)
            for index, (name, field) in enumerate(zip(names, fields))
        ]
        self.assertEqual(
            [(port["field"], port.get("role"), port.get("adc_channel")) for port in ports],
            [
                ("VADC_VDC", "adc_sample_voltage", "VDC"),
                ("VADC_VA", "adc_sample_voltage", "VA"),
                ("VADC_IA", "adc_sample_voltage", "IA"),
                ("OUT", None, None),
            ],
        )

    def test_json_round_trip_and_data_only_simulation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "buck.json"
            data.write_circuit_data(path, self.document)
            loaded = data.load_circuit_data(path)
            self.assertEqual(loaded["circuit"]["source"]["sha256"], self.document["circuit"]["source"]["sha256"])
            self.assertEqual(loaded["schema"]["version"], 2)
            self.assertIn("matrix_storage", loaded)
            self.assertNotIn("discrete", loaded["topologies"][0])
            self.assertEqual(
                len(loaded["matrix_storage"]["topology_to_calculation_state"]),
                len(loaded["topologies"]),
            )
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
            archive = files["archive"].read_bytes()
        self.assertEqual(set(files), {"header", "archive"})
        self.assertEqual(archive[:8], b"GMPMNA1\0")
        self.assertEqual(struct.unpack_from("<I", archive, 8)[0], 1)
        self.assertEqual(struct.unpack_from("<I", archive, 12)[0], 140)
        payload_size, payload_hash = struct.unpack_from("<QQ", archive, 124)
        self.assertEqual(payload_size, len(archive) - 140)
        self.assertEqual(payload_hash, codegen._fnv1a64(archive[140:]))
        self.assertIn("class BuckCircuit", header)
        self.assertIn("step_short", header)
        self.assertIn("step_normal", header)
        self.assertIn("operator()", header)
        self.assertIn("Outputs output", header)
        self.assertIn("operator[](std::string_view", header)
        self.assertIn("std::uint32_t PWM", header)
        self.assertIn("double VS1", header)
        self.assertIn("calculation_state_count", header)
        self.assertIn("topology_to_calculation_state", header)
        self.assertIn("state_matrices()", header)
        self.assertIn("load_archive", header)
        self.assertIn('matrix_storage = "archive"', header)
        self.assertIn('archive_filename = "buckcircuit.archive"', header)
        self.assertNotIn("value <<", header)
        self.assertIn("matrix_tolerance", header)
        self.assertIn('matrix_backend = "eigen"', header)

    def test_rk4_document_preserves_method_through_compact_json_and_archive(self) -> None:
        source = BUCK_DIR / "buck.CIR"
        document = data.build_circuit_data(
            switched.build_piecewise_model(mna.parse_netlist(source)),
            source_path=source,
            normal_step_s=100e-9,
            short_step_s=1e-9,
            method="rk4",
        )
        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "buck.json"
            data.write_circuit_data(data_path, document)
            loaded = data.load_circuit_data(data_path)
            files = codegen.generate_cpp_project(data_path, Path(directory) / "cpp")
            header = files["header"].read_text(encoding="utf-8")
        self.assertEqual(loaded["solver"]["method"], "rk4")
        self.assertIn('discretization_method = "rk4"', header)
        self.assertLessEqual(
            loaded["matrix_storage"]["statistics"]["unique_states"],
            loaded["matrix_storage"]["statistics"]["logical_states"],
        )

    def test_cpp_generator_supports_all_static_fixed_matrix_backend(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "buck.json"
            data.write_circuit_data(data_path, self.document)
            files = codegen.generate_cpp_project(
                data_path,
                Path(directory) / "cpp",
                "FixedBuckCircuit",
                backend="fixed",
            )
            header = files["header"].read_text(encoding="utf-8")

        self.assertEqual(set(files), {"header"})
        self.assertIn("#include <cctl/numerical_solver/fixed_matrix.hpp>", header)
        self.assertIn("#include <cctl/numerical_solver/fixed_vector.hpp>", header)
        self.assertNotIn("#include <Eigen/Dense>", header)
        self.assertIn('matrix_backend = "fixed"', header)
        self.assertIn("using StateMatrix = cctl::fixed_matrix", header)
        self.assertIn("using StateVector = cctl::fixed_vector", header)
        self.assertIn("const InputVector input_vector{", header)
        self.assertIn("cctl::affine_transform", header)
        self.assertIn("static constexpr std::array<StateMatrix", header)
        self.assertNotIn("value <<", header)

        with self.assertRaisesRegex(ValueError, "unsupported matrix backend"):
            codegen.render_header(self.document, "InvalidCircuit", backend="dynamic")

    def test_cpp_matrix_plan_deduplicates_states_and_interns_storage(self) -> None:
        synthetic = copy.deepcopy(self.document)
        synthetic["topologies"][1]["discrete"] = copy.deepcopy(
            synthetic["topologies"][0]["discrete"]
        )
        synthetic["topologies"][1]["discrete"]["normal"]["A"][0][0] += 0.5e-12
        plan = codegen.build_matrix_dedup_plan(synthetic, tolerance=1e-12)
        self.assertEqual(
            plan.topology_to_calculation_state[0],
            plan.topology_to_calculation_state[1],
        )
        self.assertGreaterEqual(plan.deduplicated_state_count, 1)
        self.assertGreater(plan.shared_matrix_copy_count, 0)
        self.assertLess(plan.coefficients_after, plan.coefficients_before)

        synthetic["topologies"][1]["discrete"]["normal"]["A"][0][0] += 1e-6
        distinct = codegen.build_matrix_dedup_plan(synthetic, tolerance=1e-12)
        self.assertNotEqual(
            distinct.topology_to_calculation_state[0],
            distinct.topology_to_calculation_state[1],
        )

    def test_default_cpp_class_uses_netlist_file_stem(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "buck.json"
            data.write_circuit_data(data_path, self.document)
            files = codegen.generate_cpp_project(data_path, Path(directory) / "cpp")
            header = files["header"].read_text(encoding="utf-8")
        self.assertIn("class BuckCircuit", header)
        self.assertEqual(set(files), {"header", "archive"})

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

    def test_binary_mosfet_mode_exports_two_states_per_switch(self) -> None:
        source = FSBB_DIR / "FSBB.CIR"
        circuit = mna.parse_netlist(source)
        self.assertEqual(
            switched.piecewise_topology_count(
                circuit, include_mosfet_body_diodes=False
            ),
            16,
        )
        model = switched.build_piecewise_model(
            circuit, include_mosfet_body_diodes=False
        )
        self.assertIsInstance(model, switched.MultiMosfetLinearModel)
        self.assertFalse(model.includes_body_diode_states)
        self.assertEqual(len(model.topologies), 16)

        document = data.build_circuit_data(
            model,
            source_path=source,
            normal_step_s=100e-9,
            short_step_s=1e-9,
        )
        data.validate_circuit_data(document)
        self.assertEqual(document["switching"]["kind"], "multi_mosfet_binary")
        self.assertFalse(
            document["switching"]["selection_uses_previous_terminal_voltage"]
        )

        simulator = data.CircuitDataSimulator(document)
        simulator.step_normal(
            {"PWM1": 1, "PWM2": 0, "PWM3": 1, "PWM4": 0}, {"VS1": 5.0}
        )
        self.assertIn("MOS1_CHANNEL", simulator.last_topology)

        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "FSBB.json"
            data.write_circuit_data(data_path, document)
            files = codegen.generate_cpp_project(data_path, Path(directory) / "cpp")
            header = files["header"].read_text(encoding="utf-8")
        self.assertIn("static constexpr std::size_t topology_count = 16", header)
        self.assertNotIn("body_on_", header)

    def test_single_phase_inverter_exports_differential_voltage_probe(self) -> None:
        source = SINV_DIR / "SINV.CIR"
        document = data.build_circuit_data(
            switched.build_piecewise_model(mna.parse_netlist(source)),
            source_path=source,
            normal_step_s=100e-9,
            short_step_s=1e-9,
        )
        data.validate_circuit_data(document)
        self.assertEqual(
            [port["name"] for port in document["ports"]["inputs"]],
            ["PWM1", "PWM2", "PWM3", "PWM4", "VS1"],
        )
        self.assertEqual(
            [port["name"] for port in document["ports"]["outputs"]],
            ["I(VAM1)", "V(2,1)"],
        )
        self.assertEqual(document["switching"]["topology_count"], 81)

        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "SINV.json"
            data.write_circuit_data(data_path, document)
            files = codegen.generate_cpp_project(data_path, Path(directory) / "cpp")
            header = files["header"].read_text(encoding="utf-8")
        self.assertIn("class SinvCircuit", header)
        self.assertIn('name == "V(2,1)"', header)

    def test_three_phase_inverter_exports_six_pwm_ports_and_phase_probes(self) -> None:
        source = INV_DIR / "INV.CIR"
        document = data.build_circuit_data(
            switched.build_piecewise_model(mna.parse_netlist(source)),
            source_path=source,
            normal_step_s=100e-9,
            short_step_s=1e-9,
        )
        data.validate_circuit_data(document)
        self.assertEqual(
            [port["name"] for port in document["ports"]["inputs"]],
            ["PWM1", "PWM2", "PWM3", "PWM4", "PWM5", "PWM6", "VS1"],
        )
        self.assertEqual(
            [port["name"] for port in document["ports"]["outputs"]],
            ["I(VAM3)", "I(VAM2)", "I(VAM1)", "V(5,1)", "V(4,1)", "V(3,1)"],
        )
        self.assertEqual(document["switching"]["topology_count"], 729)
        self.assertEqual(
            [(item["instance"], item["pwm_port"]) for item in document["switching"]["switches"]],
            [
                ("MT6", "PWM3"),
                ("MT5", "PWM4"),
                ("MT4", "PWM1"),
                ("MT3", "PWM2"),
                ("MT2", "PWM5"),
                ("MT1", "PWM6"),
            ],
        )

    def test_pmsm_inverter_exports_bidirectional_coupling_contract(self) -> None:
        source = PMSM_DIR / "PMSM.CIR"
        document = data.build_circuit_data(
            switched.build_piecewise_model(mna.parse_netlist(source)),
            source_path=source,
            normal_step_s=100e-9,
            short_step_s=1e-9,
        )
        data.validate_circuit_data(document)

        inputs = {port["name"]: port for port in document["ports"]["inputs"]}
        outputs = {port["name"]: port for port in document["ports"]["outputs"]}
        self.assertEqual(document["switching"]["topology_count"], 729)
        for phase in "ABC":
            current = inputs[f"IPMSM1_{phase}"]
            voltage = outputs[f"VPMSM1_{phase}"]
            self.assertEqual(current["role"], "pmsm_phase_current")
            self.assertEqual(current["motor"], "PMSM1")
            self.assertEqual(current["phase"], phase)
            self.assertEqual(current["default"], 0.0)
            self.assertEqual(voltage["role"], "pmsm_phase_voltage")
            self.assertEqual(voltage["motor"], "PMSM1")
            self.assertEqual(voltage["phase"], phase)

        motors = document["devices"]["pmsm_current_sources"]
        self.assertEqual(len(motors), 1)
        self.assertEqual(motors[0]["name"], "PMSM1")
        self.assertEqual(
            [(item["phase"], item["terminal_node"], item["neutral_node"])
             for item in motors[0]["phases"]],
            [("A", "4", "2"), ("B", "3", "2"), ("C", "1", "2")],
        )

        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "PMSM.json"
            data.write_circuit_data(data_path, document)
            header_path = codegen.generate_cpp_project(
                data_path, Path(directory) / "cpp", "PmsmCircuit"
            )["header"]
            header = header_path.read_text(encoding="utf-8")
        for phase in "ABC":
            self.assertIn(f"double IPMSM1_{phase}", header)
            self.assertIn(f"double VPMSM1_{phase}", header)

    def test_npc_buck_exports_mosfet_and_independent_diode_topology_axes(self) -> None:
        source = BUCK_NPC_DIR / "BUCK_NPC.CIR"
        document = data.build_circuit_data(
            switched.build_piecewise_model(mna.parse_netlist(source)),
            source_path=source,
            normal_step_s=100e-9,
            short_step_s=1e-9,
        )
        data.validate_circuit_data(document)
        self.assertEqual(
            [port["name"] for port in document["ports"]["inputs"]],
            ["PWM1", "PWM2", "PWM3", "PWM4", "VS2", "VS1"],
        )
        self.assertEqual(
            [port["name"] for port in document["ports"]["outputs"]],
            ["V(4)", "I(VAM1)", "V(VF1)"],
        )
        self.assertEqual(document["switching"]["kind"], "multi_mosfet_diode")
        self.assertEqual(document["switching"]["topology_count"], 324)
        self.assertEqual(
            [(item["instance"], item["pwm_port"]) for item in document["switching"]["switches"]],
            [("MT5", "PWM3"), ("MT2", "PWM4"), ("MT4", "PWM1"), ("MT3", "PWM2")],
        )
        self.assertEqual(
            [item["instance"] for item in document["switching"]["diodes"]], ["D2", "D1"]
        )
        for diode in document["devices"]["diodes"]:
            self.assertAlmostEqual(diode["reduced"]["extracted_junction_capacitance_F"], 460e-12)
            self.assertAlmostEqual(
                diode["reduced"]["implemented_junction_capacitance_F"], 460e-12
            )

        simulator = data.CircuitDataSimulator(document)
        for _ in range(20):
            outputs = simulator.step(
                {"PWM1": 0, "PWM2": 0, "PWM3": 1, "PWM4": 1},
                {"VS1": 30.0, "VS2": 30.0},
            )
        self.assertTrue(all(np.isfinite(value) for value in outputs.values()))

        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "BUCK_NPC.json"
            data.write_circuit_data(data_path, document)
            files = codegen.generate_cpp_project(data_path, Path(directory) / "cpp")
            header = files["header"].read_text(encoding="utf-8")
        self.assertIn("class BuckNpcCircuit", header)
        self.assertIn("static constexpr std::size_t topology_count = 324", header)
        self.assertIn("std::array<bool, 4> body_on_", header)
        self.assertIn("std::array<bool, 2> diode_on_", header)

    def test_rectifier_exports_boolean_switch_and_four_diode_selection(self) -> None:
        source = RECTIFIER_DIR / "RECTIFIER.CIR"
        document = data.build_circuit_data(
            switched.build_piecewise_model(mna.parse_netlist(source)),
            source_path=source,
            normal_step_s=1e-6,
            short_step_s=10e-9,
        )
        data.validate_circuit_data(document)
        self.assertEqual(
            [(port["name"], port["data_type"]) for port in document["ports"]["inputs"]],
            [("SWGPIO1", "uint32_t"), ("VS1", "double")],
        )
        self.assertEqual(
            [port["name"] for port in document["ports"]["outputs"]],
            ["V(1,2)", "V(VF1)"],
        )
        self.assertEqual(document["switching"]["kind"], "multi_diode_switch")
        self.assertEqual(document["switching"]["topology_count"], 32)
        self.assertEqual(
            [item["instance"] for item in document["switching"]["diodes"]],
            ["D4", "D3", "D2", "D1"],
        )
        self.assertEqual(document["switching"]["switches"][0]["command_port"], "SWGPIO1")
        for diode in document["devices"]["diodes"]:
            self.assertAlmostEqual(diode["reduced"]["extracted_junction_capacitance_F"], 460e-12)
            self.assertEqual(diode["reduced"]["implemented_junction_capacitance_F"], 0.0)
            self.assertTrue(
                diode["extraction"]["junction_capacitance_suppressed_for_ideal_source"]
            )

        with tempfile.TemporaryDirectory() as directory:
            data_path = Path(directory) / "RECTIFIER.json"
            data.write_circuit_data(data_path, document)
            files = codegen.generate_cpp_project(data_path, Path(directory) / "cpp")
            header = files["header"].read_text(encoding="utf-8")
        self.assertIn("class RectifierCircuit", header)
        self.assertIn("std::uint32_t SWGPIO1", header)
        self.assertIn("std::array<bool, 4> diode_on_", header)

    def test_rectifier_data_simulates_precharge_then_bypass(self) -> None:
        source = RECTIFIER_DIR / "RECTIFIER.CIR"
        document = data.build_circuit_data(
            switched.build_piecewise_model(mna.parse_netlist(source)),
            source_path=source,
            normal_step_s=1e-6,
            short_step_s=10e-9,
        )
        peak = 32.0 * np.sqrt(2.0)
        result = data.simulate_circuit_data(
            document,
            100e-3,
            inputs={"VS1": lambda time: peak * np.sin(2.0 * np.pi * 50.0 * time)},
            pwm_inputs={"SWGPIO1": lambda time: int(time >= 60e-3)},
        )
        voltage = result.outputs[:, result.output_names.index("V(VF1)")]
        precharge = voltage[40_000:60_000]
        bypassed = voltage[80_000:100_000]
        self.assertTrue(np.all(np.isfinite(voltage)))
        self.assertGreater(float(np.mean(precharge)), 12.0)
        self.assertLess(float(np.mean(precharge)), 17.0)
        self.assertGreater(float(np.mean(bypassed)), 29.0)
        self.assertLess(float(np.mean(bypassed)), 33.0)

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
