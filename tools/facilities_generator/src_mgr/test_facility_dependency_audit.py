#!/usr/bin/env python3
"""Regression tests for the Facility dependency audit."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import facility_dependency_audit as audit


class FacilityDependencyAuditTests(unittest.TestCase):
    """Verify platform-independent registry path handling."""

    def test_registered_files_preserve_actual_path_case(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            header = root / "ctl/component/Preset/UPPER_CASE.h"
            header.parent.mkdir(parents=True)
            header.write_text("#pragma once\n", encoding="ascii")
            modules = {
                "ctl|preset": {
                    "src_patterns": [],
                    "inc_patterns": [str(root / "ctl/component/Preset/*.h")],
                }
            }

            owners, files, actual_paths = audit.registered_files(root, modules)

        normalized = "ctl/component/preset/upper_case.h"
        self.assertEqual(files["ctl|preset"], {normalized})
        self.assertEqual(owners[normalized], {"ctl|preset"})
        self.assertEqual(actual_paths[normalized], "ctl/component/Preset/UPPER_CASE.h")


if __name__ == "__main__":
    unittest.main()
