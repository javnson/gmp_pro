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


class RepositoryConfigurationTests(unittest.TestCase):
    def test_configuration_audits_the_current_registry_without_the_retired_generator(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            python = root / "bin" / "python" / "python.exe"
            source_manager = root / "tools" / "facilities_generator" / "src_mgr"
            with (
                mock.patch.object(manager, "GMP_ROOT", root),
                mock.patch.object(manager, "BIN_DIR", root / "bin"),
                mock.patch.object(manager, "private_environment", return_value={}),
                mock.patch.object(manager, "run") as run,
            ):
                manager.configure_repository()

        commands = [list(call.args[0]) for call in run.call_args_list]
        self.assertIn(
            [python, source_manager / "facility_dependency_audit.py", "--repo", root],
            commands,
        )
        self.assertIn(
            [python, source_manager / "framework_distribute_tools_v3.py", "--deploy-only"],
            commands,
        )
        self.assertFalse(
            any("gmp_fac_generate_cfg_json.py" in str(argument) for command in commands for argument in command)
        )


if __name__ == "__main__":
    unittest.main()
