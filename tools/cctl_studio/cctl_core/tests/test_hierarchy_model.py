import json
import sys
import tempfile
import unittest
from pathlib import Path


TOOL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cctl_studio  # noqa: E402
from editor_model import EditorDocument  # noqa: E402
from hierarchy_model import HierarchyDocument  # noqa: E402
from qt_studio import LayerAdapter  # noqa: E402
from topology_bundle import binding_key  # noqa: E402
from topology_fixture import write_topology_bundle  # noqa: E402


EXAMPLE = TOOL_ROOT / "examples" / "rc_low_pass" / "project.json"


class HierarchyDocumentTests(unittest.TestCase):
    def setUp(self):
        self.components = cctl_studio.load_components([cctl_studio.BUILTIN_LIBRARY])

    def document(self) -> EditorDocument:
        return EditorDocument(None, self.components)

    def test_default_root_owns_an_mna_ready_circuit_layer(self):
        document = self.document()
        hierarchy = HierarchyDocument(document)
        root = hierarchy.layer(hierarchy.root_layer_id)
        topology = hierarchy.node(root["id"], "TOP1")
        self.assertEqual(root["kind"], "system")
        self.assertEqual(topology["child_layer"], "circuit_main")
        circuit = hierarchy.layer("circuit_main")
        self.assertNotIn("source", circuit)
        self.assertEqual(circuit["nodes"], [])

    def test_system_ports_are_numeric_and_child_editor_kind_is_explicit(self):
        hierarchy = HierarchyDocument(self.document())
        topology_type = hierarchy.node_type(hierarchy.node("system_root", "TOP1"))
        self.assertEqual({port.domain for port in topology_type.ports}, {"numeric"})
        self.assertEqual(topology_type.child_kind, "circuit")
        digital_id = hierarchy.add_node("system_root", "system.digital_module", 400, 100)
        digital = hierarchy.node("system_root", digital_id)
        self.assertEqual(hierarchy.layer(digital["child_layer"])["kind"], "digital")

    def test_digital_connection_is_typed_directed_and_single_driver_per_input(self):
        document = self.document()
        hierarchy = HierarchyDocument(document)
        module_id = hierarchy.add_node("system_root", "system.digital_module", 400, 100)
        child = hierarchy.node("system_root", module_id)["child_layer"]
        source = hierarchy.add_node(child, "digital.input", 0, 0)
        gate = hierarchy.add_node(child, "digital.and", 200, 0)
        replacement = hierarchy.add_node(child, "digital.input", 0, 100)
        hierarchy.connect(child, (source, "out"), (gate, "a"))
        hierarchy.connect(child, (replacement, "out"), (gate, "a"))
        wires = hierarchy.layer(child)["connections"]
        self.assertEqual(len(wires), 1)
        self.assertEqual(wires[0]["domain"], "logic")
        self.assertEqual(wires[0]["source"]["node"], replacement)
        with self.assertRaisesRegex(cctl_studio.StudioError, "one input and one output"):
            hierarchy.connect(child, (gate, "a"), (gate, "b"))

    def test_new_electrical_topology_has_symbol_palette_and_passive_wires(self):
        document = self.document()
        hierarchy = HierarchyDocument(document)
        topology_id = hierarchy.add_node("system_root", "system.electrical_topology", 400, 100)
        child = hierarchy.node("system_root", topology_id)["child_layer"]
        palette = hierarchy.available_types(child)
        self.assertIn("circuit.resistor", palette)
        resistor = hierarchy.add_node(child, "circuit.resistor", 0, 0)
        capacitor = hierarchy.add_node(child, "circuit.capacitor", 200, 0)
        wire = hierarchy.connect(child, (resistor, "n"), (capacitor, "p"))
        self.assertEqual(wire, "wire_1")
        self.assertEqual(hierarchy.layer(child)["connections"][0]["domain"], "electrical")

    def test_deleting_composite_removes_its_child_layer_tree(self):
        document = self.document()
        hierarchy = HierarchyDocument(document)
        module_id = hierarchy.add_node("system_root", "system.digital_module", 400, 100)
        child = hierarchy.node("system_root", module_id)["child_layer"]
        hierarchy.add_node(child, "digital.and", 0, 0)
        hierarchy.delete_nodes("system_root", [module_id])
        self.assertNotIn(child, hierarchy.data["layers"])

    def test_schema_v1_main_topology_cannot_be_orphaned(self):
        project = json.loads(EXAMPLE.read_text(encoding="utf-8"))
        hierarchy = HierarchyDocument(EditorDocument(project, self.components))
        with self.assertRaisesRegex(cctl_studio.StudioError, "cannot be deleted"):
            hierarchy.delete_nodes("system_root", ["TOP1"])

    def test_layer_adapter_maps_existing_instances_to_circuit_symbols(self):
        project = json.loads(EXAMPLE.read_text(encoding="utf-8"))
        document = EditorDocument(project, self.components)
        hierarchy = HierarchyDocument(document)
        adapter = LayerAdapter(document, hierarchy, "circuit_main")
        self.assertEqual(adapter.kind, "circuit")
        self.assertEqual(adapter.node("R1").symbol, "resistor")
        self.assertEqual(adapter.node("C1").symbol, "capacitor")
        self.assertEqual(adapter.node("VSTEP").symbol, "voltage_source")

    def test_hierarchy_metadata_does_not_change_xyce_netlist(self):
        project = json.loads(EXAMPLE.read_text(encoding="utf-8"))
        document = EditorDocument(project, self.components)
        hierarchy = HierarchyDocument(document)
        hierarchy.add_node("system_root", "system.digital_module", 500, 200)
        generated = []
        for instance in document.instances:
            generated.append(self.components[instance["module"]].render(instance))
        self.assertIn("R1 vin out 1k", generated)
        self.assertEqual(len(document.instances), 3)

    def test_compiled_topology_import_configures_ports_and_survives_reload(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest_path = write_topology_bundle(root)
            document = self.document()
            hierarchy = HierarchyDocument(document)
            topology = hierarchy.import_compiled_topology(
                "system_root", str(manifest_path), 320, 180
            )
            self.assertEqual(topology, "CTOP1")
            self.assertEqual(
                [port.port_id for port in hierarchy.node_type(
                    hierarchy.node("system_root", topology)
                ).ports],
                ["in_PWM", "in_VS1", "out_VF1"],
            )

            adapter = hierarchy.add_node(
                "system_root", "system.signal_adapter", 600, 180
            )
            hierarchy.connect(
                "system_root", (topology, "out_VF1"), (adapter, "in")
            )
            hierarchy.connect(
                "system_root", (adapter, "out"), (topology, "in_VS1")
            )
            raw = hierarchy.node("system_root", topology)
            parameters = dict(raw["parameters"])
            parameters[binding_key("input", "VS1", "mode")] = "constant"
            parameters[binding_key("input", "VS1", "value")] = "12.5"
            parameters[binding_key("output", "VF1", "mode")] = "hidden"
            hierarchy.update_node(
                "system_root", topology, raw["name"], raw["execution_order"], parameters
            )

            updated = hierarchy.node("system_root", topology)
            self.assertEqual(
                updated["parameters"][binding_key("input", "VS1", "value")],
                12.5,
            )
            self.assertEqual(
                [port.port_id for port in hierarchy.node_type(updated).ports],
                ["in_PWM"],
            )
            self.assertEqual(hierarchy.layer("system_root")["connections"], [])

            project_path = root / "project.json"
            document.save(project_path)
            reloaded = EditorDocument.load(project_path, self.components)
            reloaded_hierarchy = HierarchyDocument(reloaded)
            reloaded_node = reloaded_hierarchy.node("system_root", topology)
            self.assertEqual(
                [port.port_id for port in reloaded_hierarchy.node_type(reloaded_node).ports],
                ["in_PWM"],
            )
            self.assertEqual(
                Path(reloaded_node["compiled_topology"]["manifest_path"]),
                manifest_path.resolve(),
            )


if __name__ == "__main__":
    unittest.main()
