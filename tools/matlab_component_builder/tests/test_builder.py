from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

TOOL_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = TOOL_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from gmp_mcb.generator import ComponentGenerator
from gmp_mcb.model import ComponentDefinition, ComponentError


class BuilderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        os.environ.setdefault("GMP_PRO_LOCATION", str(TOOL_ROOT.parents[1]))

    def test_pid_definition_is_valid(self) -> None:
        component = ComponentDefinition.load(TOOL_ROOT / "components" / "continuous_pid.json")
        self.assertEqual(component.sfunction_name, "gmp_mcb_intrinsic_continuous_pid")

    def test_all_foundational_definitions_are_valid(self) -> None:
        components = [ComponentDefinition.load(path) for path in sorted((TOOL_ROOT / "components").glob("*.json"))]
        self.assertGreaterEqual(len(components), 14)
        self.assertEqual({item.data.get("variant") for item in components if item.data["template"] == "resonant_siso_v1"},
                         {"r", "pr", "qr", "qpr"})

    def test_generator_emits_source_and_registry(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            outputs = ComponentGenerator(TOOL_ROOT).generate(
                [TOOL_ROOT / "components" / "continuous_pid.json"], Path(temp)
            )
            registry = json.loads((Path(temp) / "registry.json").read_text(encoding="utf-8"))
            source = Path(registry["components"][0]["generated_source"])
            self.assertTrue(source.is_file())
            self.assertIn("ctl_step_pid_par", source.read_text(encoding="utf-8"))
            self.assertIn(Path(temp) / "registry.json", outputs)

    def test_generator_emits_all_foundational_components(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            configs = sorted((TOOL_ROOT / "components").glob("*.json"))
            ComponentGenerator(TOOL_ROOT).generate(configs, Path(temp))
            registry = json.loads((Path(temp) / "registry.json").read_text(encoding="utf-8"))
            self.assertEqual(len(registry["components"]), len(configs))
            for item in registry["components"]:
                self.assertTrue(Path(item["generated_source"]).is_file())

    def test_absolute_source_path_is_rejected(self) -> None:
        original = json.loads((TOOL_ROOT / "components" / "continuous_pid.json").read_text(encoding="utf-8"))
        original["implementation"]["sources"] = ["C:/outside.c"]
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "bad.json"
            path.write_text(json.dumps(original), encoding="utf-8")
            with self.assertRaises(ComponentError):
                ComponentDefinition.load(path)

    def test_windows_launcher_uses_gmp_environment_guard(self) -> None:
        launcher = (TOOL_ROOT / "run_python.bat").read_text(encoding="utf-8")
        self.assertIn("ensure_gmp_environment.bat", launcher)
        self.assertIn("import jinja2", launcher)
        self.assertNotIn("pip install", launcher.lower())


if __name__ == "__main__":
    unittest.main()
