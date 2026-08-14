#!/usr/bin/env python3
"""Regression tests for the repository-private Linux environment manager."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import linux_environment_manager as manager


class LinuxEnvironmentManagerTests(unittest.TestCase):
    def test_native_triplet_mapping(self) -> None:
        self.assertEqual(manager.target_triplet("x86_64"), "x64-linux")
        self.assertEqual(manager.target_triplet("aarch64"), "arm64-linux")
        with self.assertRaisesRegex(manager.LinuxEnvironmentError, "Unsupported"):
            manager.target_triplet("mips64")

    def test_aggregate_manifest_keeps_dependency_union(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            projects = []
            for name, dependencies in (("first", ["asio", "fmt"]), ("second", ["nlohmann-json", "asio"])):
                project = root / name
                project.mkdir()
                (project / "vcpkg.json").write_text(
                    json.dumps({"name": name, "version-string": "1", "dependencies": dependencies}),
                    encoding="utf-8",
                )
                projects.append(project)
            with mock.patch.object(manager, "GMP_ROOT", root):
                aggregate = manager.build_aggregate_vcpkg_manifest(projects)
            self.assertEqual(aggregate["dependencies"], ["asio", "fmt", "nlohmann-json"])

    def test_activation_script_is_relocatable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            activation = Path(temporary) / "activate_gmp.sh"
            with mock.patch.object(manager, "ACTIVATION_PATH", activation):
                manager.write_activation_script()
            content = activation.read_text(encoding="utf-8")
            self.assertIn('dirname -- "${BASH_SOURCE[0]}"', content)
            self.assertNotIn(str(manager.GMP_ROOT), content)
            self.assertIn("source bin/linux/activate_gmp.sh", content)
            self.assertIn("VCPKG_FORCE_SYSTEM_BINARIES=1", content)

    def test_host_tool_validation_rejects_a_missing_tool(self) -> None:
        with mock.patch.object(manager.shutil, "which", return_value=None):
            with self.assertRaisesRegex(manager.LinuxEnvironmentError, "cmake"):
                manager.validate_host_tools()

    def test_host_tool_validation_checks_each_version(self) -> None:
        with (
            mock.patch.object(manager.shutil, "which", side_effect=lambda executable: f"/tools/{executable}"),
            mock.patch.object(manager, "run") as run,
        ):
            manager.validate_host_tools()
        self.assertEqual(run.call_count, 5)
        run.assert_any_call(["/tools/cmake", "--version"])
        run.assert_any_call(["/tools/g++", "--version"])

    def test_archive_configuration_uses_private_git_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cache = root / "bin/linux/cache"
            discovery = cache / "discovery-git"

            def simulate_run(command, **kwargs):
                if command[:3] == ["git", "init", "--bare"]:
                    discovery.mkdir(parents=True)
                    (discovery / "HEAD").write_text("ref: refs/heads/main\n", encoding="utf-8")

            with (
                mock.patch.object(manager, "GMP_ROOT", root),
                mock.patch.object(manager, "CACHE_ROOT", cache),
                mock.patch.object(manager, "VENV_DIR", root / "venv"),
                mock.patch.object(manager, "run", side_effect=simulate_run) as run,
            ):
                manager.configure_repository()

        configure_call = run.call_args_list[-1]
        environment = configure_call.kwargs["env"]
        self.assertEqual(environment["GIT_DIR"], str(discovery))
        self.assertEqual(environment["GIT_WORK_TREE"], str(root))


if __name__ == "__main__":
    unittest.main()
