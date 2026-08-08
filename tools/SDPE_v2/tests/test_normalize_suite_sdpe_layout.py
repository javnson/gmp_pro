from __future__ import annotations

import copy
import unittest

from normalize_suite_sdpe_layout import normalize_data


class NormalizeSuiteSdpeLayoutTests(unittest.TestCase):
    def test_standard_roots_and_controller_subgroups(self) -> None:
        data = {
            "requirements": [
                {"role": "PLL Kp", "macro": "CTRL_PLL_KP"},
                {"role": "Current Kp", "macro": "CTRL_CURRENT_LOOP_KP"},
                {"role": "Grid Current Bias", "macro": "CTRL_GRID_CURRENT_BIAS"},
            ],
            "requirement_groups": [
                {"name": "Control Loops", "requirements": ["PLL Kp", "Current Kp"]},
                {
                    "name": "Current Voltage Sensor / Grid Current Sensor",
                    "requirements": ["Grid Current Bias"],
                },
            ],
            "feature_macros": [],
            "option_macros": [],
        }

        normalized = normalize_data(data)
        groups = {group["name"]: group["requirements"] for group in normalized["requirement_groups"]}

        self.assertEqual(groups["Control Loop / PLL Controller"], ["PLL Kp"])
        self.assertEqual(groups["Control Loop / Current Controller"], ["Current Kp"])
        self.assertEqual(
            groups["Voltage & Current Sensor / Grid Current Sensor"],
            ["Grid Current Bias"],
        )

    def test_normalization_is_idempotent(self) -> None:
        data = {
            "requirements": [{"role": "Startup", "macro": "CTRL_STARTUP_DELAY"}],
            "requirement_groups": [{"name": "Clock and Runtime", "requirements": ["Startup"]}],
            "feature_macros": [{"group": "Runtime", "macro": "ENABLE_RUNTIME"}],
            "option_macros": [{"group": "PWM Channel", "macro": "PHASE_U_BASE"}],
        }

        once = normalize_data(copy.deepcopy(data))
        twice = normalize_data(copy.deepcopy(once))

        self.assertEqual(once, twice)
        self.assertEqual(once["requirement_groups"][0]["name"], "Runtime Parameters")
        self.assertEqual(once["feature_macro_groups"], ["Runtime"])
        self.assertEqual(once["option_macro_groups"], ["PWM Channel"])

    def test_empty_deployment_placeholder_is_not_presented_as_migrated(self) -> None:
        data = {
            "requirements": [],
            "feature_macros": [],
            "option_macros": [],
            "hardware": [],
            "requirement_groups": [],
            "feature_macro_groups": [],
            "option_macro_groups": [],
        }

        normalized = normalize_data(data)

        self.assertNotIn("requirement_groups", normalized)
        self.assertNotIn("feature_macro_groups", normalized)
        self.assertNotIn("option_macro_groups", normalized)


if __name__ == "__main__":
    unittest.main()
