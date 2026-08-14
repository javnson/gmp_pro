"""Tests for Facility dependency resolution used by source generation."""

from __future__ import annotations

import unittest

from framework_registry import RegistrySelectionError, resolve_selected_modules


class FrameworkRegistryTests(unittest.TestCase):
    """Verify transitive dependency closure and registry diagnostics."""

    def setUp(self):
        self.registry = {
            "modules": {
                "ctl": {
                    "scope": {"depends_on": ["ctl|math"]},
                    "math": {"depends_on": ["core|std"]},
                },
                "core": {"std": {"depends_on": []}},
            }
        }

    def test_transitive_dependencies_are_included_once(self):
        config = {"selected_modules": [{"root": "ctl", "module": "scope"}]}
        self.assertEqual(
            resolve_selected_modules(self.registry, config),
            [("ctl", "scope"), ("ctl", "math"), ("core", "std")],
        )

    def test_cycles_terminate_without_duplicates(self):
        self.registry["modules"]["core"]["std"]["depends_on"] = ["ctl|scope"]
        config = {"selected_modules": [{"root": "ctl", "module": "scope"}]}
        self.assertEqual(len(resolve_selected_modules(self.registry, config)), 3)

    def test_missing_selected_module_fails(self):
        config = {"selected_modules": [{"root": "ctl", "module": "missing"}]}
        with self.assertRaisesRegex(RegistrySelectionError, "not registered"):
            resolve_selected_modules(self.registry, config)

    def test_missing_dependency_fails(self):
        self.registry["modules"]["ctl"]["math"]["depends_on"] = ["core|missing"]
        config = {"selected_modules": [{"root": "ctl", "module": "scope"}]}
        with self.assertRaisesRegex(RegistrySelectionError, "core\\|missing"):
            resolve_selected_modules(self.registry, config)

    def test_optional_dependencies_are_not_automatically_selected(self):
        self.registry["modules"]["ctl"]["math"]["optional_depends_on"] = ["ctl|iqmath"]
        self.registry["modules"]["ctl"]["iqmath"] = {"depends_on": ["core|std"]}
        config = {"selected_modules": [{"root": "ctl", "module": "scope"}]}
        self.assertEqual(
            resolve_selected_modules(self.registry, config),
            [("ctl", "scope"), ("ctl", "math"), ("core", "std")],
        )

    def test_legacy_group_selection_expands_registered_descendants(self):
        self.registry["modules"]["ctl"]["filters|lowpass"] = {"type": "module", "depends_on": []}
        self.registry["modules"]["ctl"]["filters|highpass"] = {"type": "module", "depends_on": []}
        config = {"selected_modules": [{"root": "ctl", "module": "filters"}]}
        self.assertEqual(
            resolve_selected_modules(self.registry, config),
            [("ctl", "filters|lowpass"), ("ctl", "filters|highpass")],
        )


if __name__ == "__main__":
    unittest.main()
