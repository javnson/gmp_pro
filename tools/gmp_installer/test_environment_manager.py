#!/usr/bin/env python3
"""Regression tests for the GMP private environment manager."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest import mock

import environment_manager as manager


class ArchiveExtractionTests(unittest.TestCase):
    def test_temporary_directory_lock_is_retried_without_reextracting(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            archive = root / "application.zip"
            with zipfile.ZipFile(archive, "w") as bundle:
                bundle.writestr("package/bin/tool.exe", b"portable tool")
            destination = root / "application"
            replace = Path.replace
            attempts = []

            def temporarily_locked(source, target):
                attempts.append(source)
                if len(attempts) < 3:
                    raise PermissionError("directory is temporarily locked")
                return replace(source, target)

            with (
                mock.patch.object(Path, "replace", temporarily_locked),
                mock.patch.object(manager.time, "sleep") as sleep,
            ):
                manager.extract_zip(archive, destination, True)
            self.assertEqual((destination / "bin/tool.exe").read_bytes(), b"portable tool")
            self.assertEqual(len(attempts), 3)
            self.assertEqual(sleep.call_count, 2)
            self.assertEqual(list(root.glob(".application.staging-*")), [])

    def test_persistent_directory_lock_fails_after_bounded_retries(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            archive = root / "application.zip"
            with zipfile.ZipFile(archive, "w") as bundle:
                bundle.writestr("tool.exe", b"portable tool")
            with (
                mock.patch.object(Path, "replace", side_effect=PermissionError("still locked")) as replace,
                mock.patch.object(manager.time, "sleep") as sleep,
            ):
                with self.assertRaisesRegex(manager.EnvironmentError, "blocked after 10 attempts"):
                    manager.extract_zip(archive, root / "application", False)
            self.assertEqual(replace.call_count, 10)
            self.assertEqual(sleep.call_count, 9)
            self.assertFalse((root / "application").exists())
            self.assertEqual(list(root.glob(".application.staging-*")), [])


class PrivatePythonTests(unittest.TestCase):
    def test_foreign_python_and_pip_settings_cannot_redirect_private_packages(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with (
                mock.patch.object(manager, "GMP_ROOT", root),
                mock.patch.object(manager, "BIN_DIR", root / "bin"),
                mock.patch.object(manager, "apply_persisted_proxy"),
                mock.patch.dict(os.environ, {
                    "PYTHONHOME": "foreign-python",
                    "PYTHONPATH": "foreign-modules",
                    "PIP_CONFIG_FILE": "foreign-pip.ini",
                    "PIP_TARGET": "foreign-site-packages",
                    "PIP_PREFIX": "foreign-prefix",
                    "PIP_USER": "1",
                }),
            ):
                env = manager.private_environment()
            self.assertEqual(env["PYTHONHOME"], str(root / "bin/python"))
            self.assertEqual(env["PATH"].split(os.pathsep)[0], str(root / "bin/python"))
            self.assertEqual(env["PYTHONNOUSERSITE"], "1")
            self.assertEqual(env["PIP_USER"], "0")
            self.assertEqual(env["PIP_CONFIG_FILE"], os.devnull)
            for name in ("PYTHONPATH", "PIP_TARGET", "PIP_PREFIX"):
                self.assertEqual(env[name], "")

    @unittest.skipUnless(os.name == "nt", "Windows BAT environment guard")
    def test_guard_refuses_system_python_when_private_installation_is_incomplete(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            installer = root / "tools/gmp_installer"
            installer.mkdir(parents=True)
            for name in ("ensure_gmp_environment.bat", "activate_env.bat"):
                shutil.copy2(Path(manager.INSTALLER_DIR) / name, installer / name)
            python_home = root / "bin/python"
            python_home.mkdir(parents=True)
            env = os.environ.copy()
            env.update(GMP_PRO_LOCATION=str(root), GMP_ENV_ACTIVE="", PATH=str(Path(sys.executable).parent) + os.pathsep + env.get("PATH", ""))
            command = [os.environ["ComSpec"], "/d", "/c", "call", str(installer / "ensure_gmp_environment.bat")]
            for state in ("missing", "stale-active", "marker-only", "python-only"):
                with self.subTest(state=state):
                    env["GMP_ENV_ACTIVE"] = str(root) if state == "stale-active" else ""
                    marker = root / "bin/gmp_virtual_env_installed.flag"
                    marker.unlink(missing_ok=True)
                    if state == "marker-only":
                        marker.touch()
                    if state == "python-only":
                        (python_home / "python.exe").touch()
                    result = subprocess.run(command, env=env, capture_output=True, text=True)
                    self.assertNotEqual(result.returncode, 0, result.stdout)
                    self.assertIn("[ERROR]", result.stdout)

    @unittest.skipUnless(os.name == "nt", "Windows BAT activation")
    def test_repeated_activation_keeps_private_python_first_with_a_long_path(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            installer = root / "tools/gmp_installer"
            installer.mkdir(parents=True)
            for name in ("ensure_gmp_environment.bat", "activate_env.bat"):
                shutil.copy2(Path(manager.INSTALLER_DIR) / name, installer / name)
            shutil.copy2(Path(manager.INSTALLER_DIR) / "environment_manifest.json", installer)
            python_home = root / "bin/python"
            python_home.mkdir(parents=True)
            (python_home / "python.exe").touch()
            (root / "bin/gmp_virtual_env_installed.flag").touch()
            probe = root / "probe.bat"
            probe.write_text(
                '@echo off\n'
                + (f'call "{installer / "ensure_gmp_environment.bat"}"\n'
                   'if errorlevel 1 exit /b 1\n'
                   'if defined PYTHONPATH exit /b 2\n'
                   'if defined PIP_TARGET exit /b 3\n'
                   'set PATH\n') * 2,
                encoding="utf-8",
            )
            env = os.environ.copy()
            env.update(GMP_PRO_LOCATION=str(root), GMP_ENV_ACTIVE="", VSCMD_VER="test", PYTHONPATH="foreign", PIP_TARGET="foreign", PATH=";".join([r"C:\foreign-python"] * 280))
            result = subprocess.run([os.environ["ComSpec"], "/d", "/c", "call", str(probe)], env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            paths = [line for line in result.stdout.splitlines() if line.startswith("PATH=")]
            self.assertEqual(len(paths), 2)
            self.assertEqual(paths[0], paths[1])
            self.assertTrue(paths[0].startswith(f"PATH={python_home};"))


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
