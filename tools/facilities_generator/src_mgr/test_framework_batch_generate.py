"""Tests for the source-manager fleet generation contract."""

from __future__ import annotations

import subprocess
import tempfile
import unittest
from pathlib import Path

from framework_batch_generate_v3 import generate_project, parse_args


class _Result:
    returncode = 0
    stdout = ""


class FrameworkBatchGenerateTests(unittest.TestCase):
    """Verify ordering, validation, and command-line behavior."""

    def _project(self, root: Path) -> Path:
        project = root / "gmp_src_mgr"
        project.mkdir()
        (project / "gmp_framework_config.json").write_text("{}\n", encoding="utf-8")
        (project / "gmp_generate_inc.bat").write_text("@exit /b 0\n", encoding="ascii")
        (project / "gmp_generate_src.bat").write_text("@exit /b 0\n", encoding="ascii")
        return project

    def test_help_parser_has_no_generation_side_effect(self):
        with self.assertRaises(SystemExit) as context:
            parse_args(["--help"])
        self.assertEqual(context.exception.code, 0)

    def test_dry_run_requires_config_and_both_scripts(self):
        with tempfile.TemporaryDirectory() as directory:
            project = Path(directory) / "gmp_src_mgr"
            project.mkdir()
            self.assertFalse(generate_project(project, dry_run=True))

    def test_dry_run_rejects_unregistered_module(self):
        with tempfile.TemporaryDirectory() as directory:
            project = self._project(Path(directory))
            (project / "gmp_framework_config.json").write_text(
                '{"selected_modules":[{"root":"core","module":"missing"}]}\n',
                encoding="utf-8",
            )
            self.assertFalse(generate_project(project, dry_run=True))

    def test_generation_runs_header_before_source(self):
        calls = []

        def runner(command, **kwargs):
            calls.append(Path(command[0]).name)
            return _Result()

        with tempfile.TemporaryDirectory() as directory:
            project = self._project(Path(directory))
            self.assertTrue(generate_project(project, runner=runner))
        self.assertEqual(calls, ["gmp_generate_inc.bat", "gmp_generate_src.bat"])

    def test_generation_stops_after_header_failure(self):
        calls = []

        def runner(command, **kwargs):
            calls.append(Path(command[0]).name)
            return subprocess.CompletedProcess(command, 1, stdout="header failed")

        with tempfile.TemporaryDirectory() as directory:
            project = self._project(Path(directory))
            self.assertFalse(generate_project(project, runner=runner))
        self.assertEqual(calls, ["gmp_generate_inc.bat"])


if __name__ == "__main__":
    unittest.main()
