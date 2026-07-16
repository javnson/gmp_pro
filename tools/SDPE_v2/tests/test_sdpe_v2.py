from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from sdpe_v2.generator import HeaderGenerator
from sdpe_v2.library import SDPELibrary
from sdpe_v2.model import HardwareEntity
from sdpe_v2.util import read_json


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "examples"


def read_project(name: str) -> dict:
    return read_json(EXAMPLES / "projects" / f"{name}.json")


class SDPEV2Tests(unittest.TestCase):
    def load_library(self) -> SDPELibrary:
        return SDPELibrary(EXAMPLES).load()

    def test_validate_examples(self) -> None:
        lib = self.load_library()
        warnings = lib.validate()
        self.assertEqual(warnings, [])
        self.assertIn("half_bridge", lib.schemas)
        self.assertIn("current_sensor", lib.schemas["current_sensor"].tags)
        self.assertEqual(lib.schemas["current_sensor"].category, "Current Sensor")
        self.assertIn("lvfb_half_bridge_phase_a", lib.entity_files)
        self.assertIn("hall", lib.entity("tmcs1133_b2a").tags)
        self.assertEqual(lib.entity("tmcs1133_b2a").vendor, "Texas Instruments")
        self.assertIn("current_sensor", lib.schemas["half_bridge"].default_components)

    def test_real_hardware_presets_are_modeled(self) -> None:
        lib = self.load_library()
        for entity_id in [
            "tle4971a025",
            "tmcs1133",
            "gbm2804h_100t",
            "hbl48zl400330k",
            "pmsrm_4p_15kw520v",
            "sm060r20b30mnad",
            "tyi_5008_kv335",
            "tyi_5010_360kv",
            "acm_4p24v",
            "gmp_harmonia_3ph_lc_filter",
            "gmp_lvfb_150_2ph_v2",
        ]:
            self.assertIn(entity_id, lib.entity_files)

        self.assertEqual(lib.entity("gmp_lvfb_150_2ph_v2").components["current_sensor"].entity.id, "tmcs1133_b5a")
        self.assertEqual(
            lib.entity("gmp_harmonia_3ph_lc_filter").components["phase_current_sensor"].entity.id,
            "tle4971a025",
        )

    def test_schema_declared_includes_are_generated(self) -> None:
        lib = self.load_library()
        with tempfile.TemporaryDirectory() as tmp:
            gen = HeaderGenerator(lib, Path(tmp))
            gen.generate_entity_tree("gbm2804h_100t")
            header = (Path(tmp) / "hardware_preset" / "pmsm_motor" / "gbm2804h_100t.h").read_text(
                encoding="utf-8"
            )
            self.assertIn("#include <ctl/component/motor_control/consultant/unit_consultant.h>", header)

    def test_entity_code_sections_are_generated_before_footer(self) -> None:
        lib = self.load_library()
        entity = lib.entity("tmcs1133_b5a")
        entity.code_sections["before_footer"] = "#define TMCS1133_B5A_USER_CALIBRATION 1"
        with tempfile.TemporaryDirectory() as tmp:
            header = HeaderGenerator(lib, Path(tmp)).render_entity_header(entity)
            self.assertIn("// User code before footer", header)
            self.assertIn("#define TMCS1133_B5A_USER_CALIBRATION 1", header)
            self.assertLess(header.index("#define TMCS1133_B5A_USER_CALIBRATION 1"), header.rindex("#endif //"))

    def test_entity_option_sets_and_conditional_macros_are_available(self) -> None:
        lib = self.load_library()
        self.assertIn("epwm_base", lib.entity("iris_f280039c_node").option_sets)
        with tempfile.TemporaryDirectory() as tmp:
            header = HeaderGenerator(lib, Path(tmp)).render_entity_header(lib.entity("tmcs1133"))
            self.assertIn("#define TMCS1133_CURRENT_SUFFIX B5A", header)
            self.assertIn("#define TMCS1133_CURRENT_SUFFIX_B5A 1", header)
            self.assertIn("#if defined(TMCS1133_CURRENT_SUFFIX_B5A)", header)

    def test_schema_default_components_are_applied(self) -> None:
        lib = self.load_library()
        entity = lib.inline_entity(
            {
                "id": "default_half_bridge",
                "schema": "half_bridge",
                "parameters": {
                    "board_name": "Default half bridge"
                },
            },
            "test",
            "half_bridge",
        )
        self.assertIn("power_device", entity.components)
        self.assertIn("current_sensor", entity.components)
        self.assertEqual(entity.components["current_sensor"].entity.id, "tmcs1133_b5a")

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
            self.assertIn("#define FSBB_HB_INLINE_MEASURED_CURRENT_RANGE_A FSBB_5M_SHUNT_RANGE_A", header)
            self.assertNotIn("#define FSBB_HB_INLINE_CURRENT_SENSOR_SENSITIVITY", header)
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
            self.assertIn("#define CTRL_REFERENCE_CURRENT_RANGE_A TMCS1133_B2A_RANGE_A", header)
            self.assertIn("#define CTRL_VIN_ADC_OFFSET_MANUAL (2048U)", header)
            self.assertIn("#define PHASE_BUCK_BASE IRIS_F280039C_EPWM1_BASE", header)
            self.assertIn("#include <ctl/component/hardware_preset/half_bridge/fsbb_inline_shunt_half_bridge.h>", header)
            self.assertIn("#include <ctl/component/hardware_preset/current_sensor/tmcs1133_b2a.h>", header)

    def test_project_header_renders_metadata_and_macro_options(self) -> None:
        lib = self.load_library()
        data = read_project("mcs_pmsm_nt_sensor_binding")
        with tempfile.TemporaryDirectory() as tmp:
            header = HeaderGenerator(lib, Path(tmp)).render_project_header(data)
            self.assertIn("@note Typical PMSM_NT ctrl_settings-style requirement list", header)
            self.assertIn('#define SDPE_PROJECT_VERSION "0.2.0"', header)
            self.assertIn("// #define PMSM_CTRL_USING_DISCRETE_CTRL", header)
            self.assertIn("#define BUILD_LEVEL (4)", header)
            self.assertIn("#include <ctl/component/hardware_preset/inverter_3ph/GMP_3PH_2136SINV_DUAL_TMPL.h>", header)

    def test_project_metadata_can_use_a_namespace_prefix(self) -> None:
        lib = self.load_library()
        data = read_project("mcs_pmsm_nt_sensor_binding")
        data["macro_prefix"] = "mcs pmsm common"
        with tempfile.TemporaryDirectory() as tmp:
            generator = HeaderGenerator(lib, Path(tmp))
            header = generator.render_project_header(data)
            matlab = generator.render_project_matlab_script(data)
            self.assertIn('#define MCS_PMSM_COMMON_SDPE_PROJECT_ID "mcs_pmsm_nt_sensor_IRIS_node"', header)
            self.assertIn('#define MCS_PMSM_COMMON_SDPE_PROJECT_VERSION "0.2.0"', header)
            self.assertNotIn('#define SDPE_PROJECT_ID "', header)
            self.assertIn("MCS_PMSM_COMMON_SDPE_PROJECT_ID = 'mcs_pmsm_nt_sensor_IRIS_node';", matlab)

    def test_project_matlab_init_script_exports_macro_variables(self) -> None:
        lib = self.load_library()
        data = read_project("dps_fsbb_iris_node")
        with tempfile.TemporaryDirectory() as tmp:
            gen = HeaderGenerator(lib, Path(tmp))
            script = gen.render_project_matlab_script(data)
            self.assertIn("SDPE_PROJECT_ID = 'dps_fsbb_iris_node';", script)
            self.assertIn("BUILD_LEVEL = 1;", script)
            self.assertIn("CTRL_INDUCTOR_CURRENT_SENSITIVITY = FSBB_5M_SHUNT_SENSITIVITY_V_PER_A;", script)
            self.assertIn("TMCS1133_B2A_RANGE_A =", script)
            self.assertIn("sdpe_select(", script)

            generated = gen.generate_project_matlab_script(EXAMPLES / "projects" / "dps_fsbb_iris_node.json")
            self.assertTrue(generated.path.name.endswith("_matlab_init.m"))
            self.assertTrue(generated.path.exists())

    def test_matlab_components_are_emitted_before_parent_references(self) -> None:
        lib = self.load_library()
        data = read_project("pgs_sinv_rc_iris_node")
        with tempfile.TemporaryDirectory() as tmp:
            script = HeaderGenerator(lib, Path(tmp)).render_project_matlab_script(data)
            child = "TMCS1133_B5A_SENSITIVITY_V_PER_A ="
            parent = "GMP_LVFB_CURRENT_SENSITIVITY = TMCS1133_B5A_SENSITIVITY_V_PER_A;"
            self.assertIn(child, script)
            self.assertIn(parent, script)
            self.assertLess(script.index(child), script.index(parent))

    def test_project_macros_can_be_grouped_in_generated_outputs(self) -> None:
        lib = self.load_library()
        data = read_project("dps_fsbb_iris_node")
        data["feature_macros"][0]["group"] = "Debug Switches"
        data["option_macros"][0]["group"] = "Build Options"
        with tempfile.TemporaryDirectory() as tmp:
            gen = HeaderGenerator(lib, Path(tmp))
            header = gen.render_project_header(data)
            matlab = gen.render_project_matlab_script(data)
            self.assertIn("@brief Debug Switches.", header)
            self.assertIn("@brief Build Options.", header)
            self.assertIn("%% Debug Switches", matlab)
            self.assertIn("%% Build Options", matlab)

    def test_project_macros_can_be_weak(self) -> None:
        lib = self.load_library()
        data = read_project("dps_fsbb_iris_node")
        data["feature_macros"] = [{"macro": "SDPE_WEAK_FEATURE", "enabled": True, "weak": True, "value": "1"}]
        data["option_macros"] = [{"macro": "SDPE_WEAK_OPTION", "enabled": True, "weak": True, "value": "IRIS_EPWM1_BASE"}]
        with tempfile.TemporaryDirectory() as tmp:
            header = HeaderGenerator(lib, Path(tmp)).render_project_header(data)
            self.assertIn("#ifndef SDPE_WEAK_FEATURE", header)
            self.assertIn("#define SDPE_WEAK_FEATURE 1", header)
            self.assertIn("#endif // SDPE_WEAK_FEATURE", header)
            self.assertIn("#ifndef SDPE_WEAK_OPTION", header)
            self.assertIn("#define SDPE_WEAK_OPTION IRIS_EPWM1_BASE", header)

    def test_requirement_and_entity_parameter_macros_have_modifiers(self) -> None:
        lib = self.load_library()
        data = read_project("dps_fsbb_iris_node")
        data["requirements"][0]["enabled"] = False
        data["requirements"][0]["weak"] = True
        with tempfile.TemporaryDirectory() as tmp:
            header = HeaderGenerator(lib, Path(tmp)).render_project_header(data)
            self.assertIn(f"// #ifndef {data['requirements'][0]['macro']}", header)

        entity = read_json(EXAMPLES / "entities" / "tmcs1133.json")
        entity["parameter_macros"] = {"range_a": {"enabled": True, "weak": False}}
        with tempfile.TemporaryDirectory() as tmp:
            header = HeaderGenerator(lib, Path(tmp)).render_entity_header(HardwareEntity.from_json(entity, EXAMPLES / "entities" / "tmcs1133.json"))
            self.assertIn("#define TMCS1133_RANGE_A", header)
            self.assertNotIn("#ifndef TMCS1133_RANGE_A", header)

    def test_project_hardware_accepts_legacy_string_entries(self) -> None:
        lib = self.load_library()
        data = read_project("dps_fsbb_iris_node")
        data["hardware"] = [data["hardware"][0]["entity"]]
        with tempfile.TemporaryDirectory() as tmp:
            header = HeaderGenerator(lib, Path(tmp)).render_project_header(data)
            self.assertIn("#include", header)

    def test_component_overrides_create_slot_local_macros(self) -> None:
        lib = self.load_library()
        with tempfile.TemporaryDirectory() as tmp:
            gen = HeaderGenerator(lib, Path(tmp))
            gen.generate_entity_tree("lvfb_half_bridge_phase_b_tuned")
            header = (
                Path(tmp) / "hardware_preset" / "half_bridge" / "lvfb_half_bridge_phase_b_tuned.h"
            ).read_text(encoding="utf-8")
            self.assertIn("#define GMP_LVFB_HB_B_TUNED_CURRENT_SENSOR_RANGE_A (25.0f)", header)
            self.assertIn("#define GMP_LVFB_HB_B_TUNED_CURRENT_SENSOR_BIAS_V (1.64f)", header)
            self.assertIn(
                "#define GMP_LVFB_HB_B_TUNED_CURRENT_SENSOR_SENSITIVITY_MV_PER_A "
                "TMCS1133_B2A_SENSITIVITY_MV_PER_A",
                header,
            )
            self.assertNotIn("#define GMP_LVFB_HB_B_TUNED_CURRENT_SENSOR_SENSITIVITY ", header)


if __name__ == "__main__":
    unittest.main()
