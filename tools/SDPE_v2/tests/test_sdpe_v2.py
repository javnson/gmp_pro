from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from sdpe_v2.generator import HeaderGenerator
from sdpe_v2.library import SDPELibrary


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "examples"


class SDPEV2Tests(unittest.TestCase):
    def load_library(self) -> SDPELibrary:
        return SDPELibrary(EXAMPLES).load()

    def test_validate_examples(self) -> None:
        lib = self.load_library()
        warnings = lib.validate()
        self.assertEqual(warnings, [])
        self.assertIn("half_bridge", lib.schemas)
        self.assertIn("lvfb_half_bridge_phase_a", lib.entity_files)

    def test_generate_inline_component_without_standalone_header(self) -> None:
        lib = self.load_library()
        with tempfile.TemporaryDirectory() as tmp:
            gen = HeaderGenerator(lib, Path(tmp))
            files = gen.generate_entity_tree("fsbb_inline_shunt_half_bridge")
            paths = [item.path.as_posix() for item in files]
            self.assertTrue(any(path.endswith("fsbb_inline_shunt_half_bridge.h") for path in paths))
            self.assertFalse(any(path.endswith("fsbb_5m_shunt.h") for path in paths))

            header = (Path(tmp) / "hardware_preset" / "half_bridge" / "fsbb_inline_shunt_half_bridge.h").read_text(
                encoding="utf-8"
            )
            self.assertIn("#define FSBB_5M_SHUNT_SENSITIVITY_V_PER_A", header)
            self.assertIn("#define FSBB_HB_INLINE_CURRENT_SENSOR_SENSITIVITY FSBB_5M_SHUNT_SENSITIVITY_V_PER_A", header)
            self.assertIn("#include <ctl/component/hardware_preset/power_switch/bsc093n15ns5.h>", header)

    def test_project_bindings_resolve_nested_exports(self) -> None:
        lib = self.load_library()
        with tempfile.TemporaryDirectory() as tmp:
            gen = HeaderGenerator(lib, Path(tmp))
            files = gen.generate_project(EXAMPLES / "projects" / "dps_fsbb_iris_node.json")
            unique_paths = {item.path for item in files}
            self.assertEqual(len(files), len(unique_paths))

            header = (Path(tmp) / "project" / "sdpe_dps_fsbb_iris_bindings.h").read_text(encoding="utf-8")
            self.assertIn("#define CTRL_INDUCTOR_CURRENT_SENSITIVITY FSBB_5M_SHUNT_SENSITIVITY_V_PER_A", header)
            self.assertIn("#define PHASE_BUCK_BASE IRIS_F280039C_EPWM1_BASE", header)
            self.assertIn("#include <ctl/component/hardware_preset/half_bridge/fsbb_inline_shunt_half_bridge.h>", header)


if __name__ == "__main__":
    unittest.main()
