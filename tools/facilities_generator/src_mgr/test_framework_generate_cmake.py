"""Tests for registry-driven CMake include-path generation."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from framework_generate_cmake import _collect_registered_include_dirs


class FrameworkGenerateCMakeTests(unittest.TestCase):
    def test_src_only_resolves_selected_and_dependency_include_dirs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manager = root / "project" / "gmp_src_mgr"
            manager.mkdir(parents=True)
            csp_include = root / "csp" / "cctl"
            core_include = root / "core"
            csp_include.mkdir(parents=True)
            core_include.mkdir()

            (manager / "gmp_framework_config.json").write_text(
                json.dumps(
                    {
                        "sync_mode": "src_only",
                        "selected_modules": [{"root": "csp", "module": "cctl"}],
                    }
                ),
                encoding="utf-8",
            )
            registry = root / "registry.json"
            registry.write_text(
                json.dumps(
                    {
                        "modules": {
                            "csp": {
                                "cctl": {
                                    "inc_dirs": ["${GMP_PRO_LOCATION}/csp/cctl"],
                                    "depends_on": ["core|rt"],
                                }
                            },
                            "core": {
                                "rt": {
                                    "inc_dirs": ["${GMP_PRO_LOCATION}/core"],
                                    "depends_on": [],
                                }
                            },
                        }
                    }
                ),
                encoding="utf-8",
            )

            result = _collect_registered_include_dirs(manager, root, registry)
            self.assertEqual(
                result,
                sorted([csp_include.resolve().as_posix(), core_include.resolve().as_posix()]),
            )

    def test_src_only_does_not_require_include_summary(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manager = root / "gmp_src_mgr"
            manager.mkdir()
            include_dir = root / "include"
            include_dir.mkdir()
            (manager / "gmp_framework_config.json").write_text(
                json.dumps(
                    {
                        "sync_mode": "src_only",
                        "selected_modules": [{"root": "cctl", "module": "dsa"}],
                    }
                ),
                encoding="utf-8",
            )
            registry = root / "registry.json"
            registry.write_text(
                json.dumps(
                    {
                        "modules": {
                            "cctl": {
                                "dsa": {
                                    "inc_dirs": ["${GMP_PRO_LOCATION}/include"],
                                    "depends_on": [],
                                }
                            }
                        }
                    }
                ),
                encoding="utf-8",
            )

            self.assertEqual(
                _collect_registered_include_dirs(manager, root, registry),
                [include_dir.resolve().as_posix()],
            )


if __name__ == "__main__":
    unittest.main()
