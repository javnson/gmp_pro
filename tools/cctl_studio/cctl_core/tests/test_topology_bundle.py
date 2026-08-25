import sys
import tempfile
import unittest
from pathlib import Path


TOOL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cctl_studio  # noqa: E402
from topology_bundle import (  # noqa: E402
    binding_key,
    default_bindings,
    exposed_port_documents,
    load_topology_manifest,
    normalize_bindings,
)
from topology_fixture import write_topology_bundle  # noqa: E402


class TopologyBundleTests(unittest.TestCase):
    def test_manifest_verifies_artifacts_and_exposes_configurable_ports(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest_path = write_topology_bundle(Path(directory))
            manifest = load_topology_manifest(manifest_path)
            bindings = default_bindings(manifest)
            self.assertEqual(
                [port["port_id"] for port in exposed_port_documents(manifest, bindings)],
                ["in_PWM", "in_VS1", "out_VF1"],
            )
            bindings[binding_key("input", "VS1", "mode")] = "constant"
            bindings[binding_key("input", "VS1", "value")] = "12.5"
            bindings[binding_key("output", "VF1", "mode")] = "hidden"
            normalized = normalize_bindings(manifest, bindings)
            self.assertEqual(normalized[binding_key("input", "VS1", "value")], 12.5)
            self.assertEqual(
                [port["port_id"] for port in exposed_port_documents(manifest, normalized)],
                ["in_PWM"],
            )

    def test_manifest_rejects_tampered_archive(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest_path = write_topology_bundle(root)
            archive = root / "buckcircuit.archive"
            original = archive.read_bytes()
            archive.write_bytes(original[:-1] + bytes([original[-1] ^ 0xFF]))
            with self.assertRaisesRegex(cctl_studio.StudioError, "SHA-256 does not match"):
                load_topology_manifest(manifest_path)


if __name__ == "__main__":
    unittest.main()
