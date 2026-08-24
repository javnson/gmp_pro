import json
import sys
import tempfile
import unittest
from pathlib import Path


TOOL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_ROOT))

import cctl_studio  # noqa: E402
from editor_model import EditorDocument  # noqa: E402


EXAMPLE = TOOL_ROOT / "examples" / "rc_low_pass" / "project.json"


class EditorDocumentTests(unittest.TestCase):
    def setUp(self):
        self.components = cctl_studio.load_components([cctl_studio.BUILTIN_LIBRARY])

    def test_legacy_project_gets_non_semantic_layout_metadata(self):
        project = json.loads(EXAMPLE.read_text(encoding="utf-8"))
        project.pop("editor", None)
        document = EditorDocument(project, self.components, EXAMPLE)
        self.assertEqual(set(document.editor["positions"]), {"VSTEP", "R1", "C1"})
        self.assertEqual(document.editor["execution_order"], {"VSTEP": 1, "R1": 2, "C1": 3})
        self.assertFalse(document.dirty)

    def test_add_component_initialises_parameters_ports_and_execution_order(self):
        document = EditorDocument(None, self.components)
        name = document.add_instance("spice.capacitor", 120, 240)
        instance = document.instance(name)
        self.assertEqual(name, "C1")
        self.assertEqual(document.position(name), (120.0, 240.0))
        self.assertEqual(instance["parameters"]["capacitance"], "1")
        self.assertEqual(instance["parameters"]["initial_condition"], "0")
        self.assertTrue(instance["ports"]["p"].startswith("NC_C1_"))
        self.assertEqual(document.editor["execution_order"][name], 1)

    def test_connect_ports_creates_and_merges_named_networks(self):
        document = EditorDocument(None, self.components)
        resistor = document.add_instance("spice.resistor", 100, 100)
        capacitor = document.add_instance("spice.capacitor", 300, 100)
        source = document.add_instance("spice.voltage_pulse", 0, 100)
        first_net = document.connect_ports((resistor, "n"), (capacitor, "p"))
        document.connect_ports((source, "p"), (resistor, "p"))
        document.connect_ports((source, "p"), (capacitor, "p"))
        self.assertEqual(first_net, "net_1")
        merged = document.instance(resistor)["ports"]["p"]
        self.assertEqual(document.instance(resistor)["ports"]["n"], merged)
        self.assertEqual(document.instance(capacitor)["ports"]["p"], merged)
        self.assertEqual(len(document.connected_networks()[merged]), 4)

    def test_duplicate_is_disconnected_and_delete_cleans_editor_metadata(self):
        document = EditorDocument(None, self.components)
        original = document.add_instance("spice.resistor", 40, 60)
        clone = document.duplicate_instances([original])[0]
        self.assertEqual(clone, "R2")
        self.assertEqual(document.position(clone), (70.0, 90.0))
        self.assertTrue(all(node.startswith("NC_R2_") for node in document.instance(clone)["ports"].values()))
        document.delete_instances([original])
        self.assertNotIn(original, document.editor["positions"])
        self.assertNotIn(original, document.editor["z_order"])

    def test_rename_updates_all_editor_keys_and_is_undoable(self):
        document = EditorDocument(None, self.components)
        document.add_instance("spice.resistor", 40, 60)
        document.rename_instance("R1", "R_LOAD")
        self.assertEqual(document.instances[0]["name"], "R_LOAD")
        self.assertIn("R_LOAD", document.editor["positions"])
        self.assertNotIn("R1", document.editor["positions"])
        self.assertTrue(document.undo())
        self.assertEqual(document.instances[0]["name"], "R1")
        self.assertTrue(document.redo())
        self.assertEqual(document.instances[0]["name"], "R_LOAD")

    def test_move_preview_commits_as_one_undo_step(self):
        document = EditorDocument(None, self.components)
        name = document.add_instance("spice.inductor", 0, 0)
        before = document.snapshot()
        document.set_positions({name: (10, 20)}, record=False)
        document.set_positions({name: (30, 40)}, record=False)
        self.assertTrue(document.commit_preview(before))
        self.assertEqual(document.position(name), (30.0, 40.0))
        document.undo()
        self.assertEqual(document.position(name), (0.0, 0.0))

    def test_align_and_distribute_are_deterministic(self):
        document = EditorDocument(None, self.components)
        names = [
            document.add_instance("spice.resistor", x, y)
            for x, y in ((0, 20), (70, 40), (200, 60))
        ]
        document.arrange(names, "top")
        self.assertEqual([document.position(name)[1] for name in names], [20.0] * 3)
        document.arrange(names, "distribute_h")
        self.assertEqual([document.position(name)[0] for name in names], [0.0, 100.0, 200.0])

    def test_saved_editor_project_remains_compatible_with_netlist_generator(self):
        project = json.loads(EXAMPLE.read_text(encoding="utf-8"))
        document = EditorDocument(project, self.components)
        document.set_positions({"R1": (440, 180)})
        with tempfile.TemporaryDirectory() as temp_name:
            target = Path(temp_name) / "project.json"
            document.save(target)
            netlist = cctl_studio.generate_netlist(target)
        self.assertIn("R1 vin out 1k", netlist)
        self.assertIn("editor", document.project)
        self.assertFalse(document.dirty)


if __name__ == "__main__":
    unittest.main()
