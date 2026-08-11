import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


TOOL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_ROOT))

import cctl_studio  # noqa: E402


EXAMPLE = TOOL_ROOT / "examples" / "rc_low_pass" / "project.json"


class CctlStudioTests(unittest.TestCase):
    def test_builtin_components_load(self):
        components = cctl_studio.load_components([cctl_studio.BUILTIN_LIBRARY])
        self.assertEqual(
            set(components),
            {
                "spice.capacitor",
                "spice.inductor",
                "spice.resistor",
                "spice.voltage_pulse",
            },
        )

    def test_rc_project_generates_xyce_netlist(self):
        netlist = cctl_studio.generate_netlist(EXAMPLE)
        self.assertTrue(netlist.startswith("CCTL Studio RC low-pass step response\n"))
        self.assertIn("VSTEP vin 0 PULSE(0 10 0 1us 1us 5ms 10ms)", netlist)
        self.assertIn("R1 vin out 1k", netlist)
        self.assertIn("C1 out 0 1u IC=0", netlist)
        self.assertIn(".TRAN 10us 5ms 0", netlist)
        self.assertIn(
            ".PRINT TRAN FORMAT=CSV FILE=waveforms.csv V(vin) V(out) I(VSTEP)",
            netlist,
        )
        self.assertTrue(netlist.endswith(".END\n"))

    def test_data_only_component_extension_needs_no_code_change(self):
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            component = {
                "schema_version": 1,
                "id": "local.current_dc",
                "display_name": "DC current source",
                "instance_prefix": "I",
                "ports": [{"name": "p"}, {"name": "n"}],
                "parameters": {"current": {"type": "spice_scalar", "required": True}},
                "xyce": {
                    "netlist_template": "$instance $port_p $port_n DC $param_current"
                },
            }
            (temp / "current_dc.json").write_text(json.dumps(component), encoding="utf-8")
            project = json.loads(EXAMPLE.read_text(encoding="utf-8"))
            project["libraries"] = ["current_dc.json"]
            project["instances"] = [
                {
                    "name": "I1",
                    "module": "local.current_dc",
                    "ports": {"p": "out", "n": "0"},
                    "parameters": {"current": "2A"},
                },
                {
                    "name": "R1",
                    "module": "spice.resistor",
                    "ports": {"p": "out", "n": "0"},
                    "parameters": {"resistance": "5"},
                },
            ]
            project_path = temp / "project.json"
            project_path.write_text(json.dumps(project), encoding="utf-8")
            netlist = cctl_studio.generate_netlist(project_path)
            self.assertIn("I1 out 0 DC 2A", netlist)

    def test_rejects_missing_port(self):
        with tempfile.TemporaryDirectory() as temp_name:
            project = json.loads(EXAMPLE.read_text(encoding="utf-8"))
            del project["instances"][1]["ports"]["n"]
            project_path = Path(temp_name) / "invalid.json"
            project_path.write_text(json.dumps(project), encoding="utf-8")
            with self.assertRaisesRegex(cctl_studio.StudioError, "ports of instance R1 mismatch"):
                cctl_studio.generate_netlist(project_path)

    def test_rejects_duplicate_instance_names_case_insensitively(self):
        with tempfile.TemporaryDirectory() as temp_name:
            project = json.loads(EXAMPLE.read_text(encoding="utf-8"))
            duplicate = dict(project["instances"][1])
            duplicate["name"] = "r1"
            project["instances"].append(duplicate)
            project_path = Path(temp_name) / "invalid.json"
            project_path.write_text(json.dumps(project), encoding="utf-8")
            with self.assertRaisesRegex(cctl_studio.StudioError, "duplicate instance name"):
                cctl_studio.generate_netlist(project_path)

    def test_xyce_lookup_error_is_actionable(self):
        old_value = os.environ.pop("XYCE_EXECUTABLE", None)
        old_which = cctl_studio.shutil.which
        cctl_studio.shutil.which = lambda _: None
        try:
            with self.assertRaisesRegex(cctl_studio.StudioError, "--xyce"):
                cctl_studio.find_xyce()
        finally:
            cctl_studio.shutil.which = old_which
            if old_value is not None:
                os.environ["XYCE_EXECUTABLE"] = old_value

    def test_runner_uses_absolute_netlist_path(self):
        with tempfile.TemporaryDirectory(dir=TOOL_ROOT) as temp_name:
            relative_output = Path(temp_name).resolve().relative_to(Path.cwd())
            with mock.patch.object(
                cctl_studio, "find_xyce", return_value=Path("C:/Xyce/Xyce.exe")
            ), mock.patch.object(cctl_studio.subprocess, "run") as run:
                cctl_studio.run_xyce(EXAMPLE, relative_output)
            command = run.call_args.args[0]
            self.assertTrue(Path(command[1]).is_absolute())
            self.assertEqual(run.call_args.kwargs["cwd"], relative_output.resolve())


if __name__ == "__main__":
    unittest.main()
