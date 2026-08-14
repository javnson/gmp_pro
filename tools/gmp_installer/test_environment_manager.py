#!/usr/bin/env python3
"""Regression tests for the GMP private environment manager."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import environment_manager as manager


class AggregateVcpkgManifestTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.root_patch = mock.patch.object(manager, "GMP_ROOT", self.root)
        self.root_patch.start()

    def tearDown(self) -> None:
        self.root_patch.stop()
        self.temporary.cleanup()

    def write_project(self, name: str, dependencies: list) -> Path:
        project = self.root / name
        project.mkdir(parents=True)
        (project / "vcpkg.json").write_text(
            json.dumps(
                {
                    "name": name.replace("_", "-"),
                    "version-string": "1.0.0",
                    "dependencies": dependencies,
                }
            ),
            encoding="utf-8",
        )
        return project

    def test_dependency_union_prevents_sequential_manifest_pruning(self) -> None:
        first = self.write_project("suite", ["asio", "fmt", "nlohmann-json"])
        second = self.write_project("simulink", ["asio", "nlohmann-json"])

        aggregate = manager.build_aggregate_vcpkg_manifest([first, second])

        self.assertEqual(aggregate["dependencies"], ["asio", "fmt", "nlohmann-json"])

    def test_conflicting_dependency_declarations_fail(self) -> None:
        first = self.write_project("suite", ["fmt"])
        second = self.write_project("simulink", [{"name": "fmt", "features": ["unicode"]}])

        with self.assertRaisesRegex(manager.EnvironmentError, "Conflicting vcpkg dependency"):
            manager.build_aggregate_vcpkg_manifest([first, second])


if __name__ == "__main__":
    unittest.main()
