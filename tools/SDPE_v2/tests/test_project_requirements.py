from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path

from sdpe_v2.generator import HeaderGenerator
from sdpe_v2.library import SDPELibrary
from sdpe_v2.project_requirements import (
    SOURCE_KEY,
    common_requirement_reference,
    duplicate_macro_occurrences,
    load_project_requirements,
    merged_project_view,
    project_requirement_paths,
    resolve_duplicate_macros,
    resolve_common_requirement_paths,
    title_case_name,
)


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "examples"


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


class ProjectRequirementCompositionTests(unittest.TestCase):
    def test_multiple_relative_absolute_and_environment_references(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project = root / "project" / "sdpe_requirement.json"
            relative = root / "common_a.json"
            absolute = root / "common_b.json"
            environment = root / "common_c.json"
            for index, path in enumerate((relative, absolute, environment)):
                write_json(path, {"id": f"common_{index}"})
            old = os.environ.get("SDPE_TEST_COMMON")
            os.environ["SDPE_TEST_COMMON"] = str(environment)
            try:
                write_json(
                    project,
                    {
                        "id": "private",
                        "common_requirements": [
                            common_requirement_reference(project, relative),
                            str(absolute.resolve()),
                            "%SDPE_TEST_COMMON%",
                        ],
                    },
                )
                self.assertEqual(
                    resolve_common_requirement_paths(project),
                    [relative.resolve(), absolute.resolve(), environment.resolve()],
                )
            finally:
                if old is None:
                    os.environ.pop("SDPE_TEST_COMMON", None)
                else:
                    os.environ["SDPE_TEST_COMMON"] = old

    def test_merged_view_marks_private_and_each_common_source(self) -> None:
        private = {"requirements": [{"role": "private", "macro": "PRIVATE"}]}
        commons = [
            {"id": "base_a", "requirements": [{"role": "a", "macro": "A"}]},
            {"id": "base_b", "requirements": [{"role": "b", "macro": "B"}]},
        ]
        merged = merged_project_view(private, commons)
        self.assertEqual(
            [item[SOURCE_KEY] for item in merged["requirements"]],
            ["project private", "common:base_a", "common:base_b"],
        )

    def test_merged_view_preserves_equal_rows_and_legacy_hardware_strings(self) -> None:
        duplicate = {"role": "same", "macro": "SAME"}
        private = {
            "hardware": ["legacy_entity"],
            "requirements": [duplicate.copy(), duplicate.copy()],
            "requirement_groups": [
                {"name": "First", "requirements": ["same"]},
                {"name": "Second", "requirements": ["same"]},
            ],
        }

        merged = merged_project_view(private, [])

        self.assertEqual(len(merged["requirements"]), 2)
        self.assertEqual(merged["hardware"][0]["entity"], "legacy_entity")
        self.assertEqual(merged["hardware"][0][SOURCE_KEY], "project private")

    def test_generator_emits_one_merged_header_for_private_and_common_inputs(self) -> None:
        library = SDPELibrary(EXAMPLES).load()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            common_a = root / "common_a.json"
            common_b = root / "common_b.json"
            project = root / "private.json"
            write_json(
                common_a,
                {
                    "id": "common_a",
                    "output_header": "common_a.h",
                    "requirements": [
                        {"role": "Gain", "macro": "CTRL_GAIN", "binding": {"number": "1"}},
                        {"role": "Common Only", "macro": "CTRL_COMMON_ONLY", "binding": {"number": "7"}},
                    ],
                },
            )
            write_json(common_b, {"id": "common_b", "output_header": "common_b.h", "requirements": []})
            write_json(
                project,
                {
                    "id": "private",
                    "output_header": "private.h",
                    "requirements": [
                        {"role": "Gain", "macro": "CTRL_GAIN", "binding": {"number": "2"}}
                    ],
                    "common_requirements": ["common_a.json", str(common_b.resolve())],
                },
            )
            private, commons = load_project_requirements(project)
            self.assertEqual(private["id"], "private")
            self.assertEqual([data["id"] for _path, data in commons], ["common_a", "common_b"])

            generator = HeaderGenerator(library, root / "out", project_subdir="")
            legacy_common_header = root / "out" / "common_a.h"
            legacy_common_header.parent.mkdir(parents=True, exist_ok=True)
            legacy_common_header.write_text(
                "/**\n"
                " * @file common_a.h\n"
                " * @brief SDPE project bindings for common_a.\n"
                " */\n",
                encoding="utf-8",
            )
            generated = generator.generate_project(project)
            headers = [item.path.name for item in generated if item.path.suffix == ".h"]
            self.assertEqual(headers, ["private.h"])
            private_header = (root / "out" / "private.h").read_text(encoding="utf-8")
            self.assertNotIn("#include <common_a.h>", private_header)
            self.assertNotIn("#include <common_b.h>", private_header)
            self.assertIn("#define CTRL_COMMON_ONLY (7)", private_header)
            self.assertIn("#ifndef CTRL_GAIN", private_header)
            self.assertLess(private_header.index("#define CTRL_GAIN (2)"), private_header.index("#ifndef CTRL_GAIN"))
            self.assertFalse(legacy_common_header.exists())
            self.assertFalse((root / "out" / "common_b.h").exists())

            scripts = generator.generate_project_matlab_scripts(project)
            self.assertEqual(
                [item.path.name for item in scripts],
                ["private_matlab_init.m", "common_a_matlab_init.m", "common_b_matlab_init.m"],
            )
            private_script = scripts[0].path.read_text(encoding="utf-8")
            self.assertIn("common_a_matlab_init.m", private_script)
            self.assertIn("common_b_matlab_init.m", private_script)
            self.assertIn("%% SDPE project summary", private_script)
            self.assertIn("disp(CTRL_GAIN)", private_script)
            self.assertIn("disp(CTRL_COMMON_ONLY)", private_script)

    def test_common_deployment_candidates_only_include_bound_private_projects(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            common_a = root / "commons" / "common_a.json"
            common_b = root / "commons" / "common_b.json"
            private_a = root / "projects" / "a" / "sdpe_requirement.json"
            private_b = root / "projects" / "b" / "sdpe_requirement.json"
            write_json(common_a, {"id": "common_a"})
            write_json(common_b, {"id": "common_b"})
            write_json(private_a, {"id": "a", "common_requirements": [str(common_a)]})
            write_json(private_b, {"id": "b", "common_requirements": [str(common_b)]})

            self.assertEqual(
                project_requirement_paths(common_a, [root / "projects"]),
                [private_a.resolve()],
            )

    def test_duplicate_resolution_keeps_selected_source_and_cleans_groups(self) -> None:
        private = {
            "requirements": [{"role": "private gain", "macro": "CTRL_GAIN"}],
            "requirement_groups": [{"name": "Private", "requirements": ["private gain"]}],
        }
        common = {
            "feature_macros": [{"macro": "CTRL_GAIN", "value": "2", "group": "Control"}],
        }
        documents = [("project private", private), ("common:base", common)]
        duplicates = duplicate_macro_occurrences(documents)
        keep = duplicates["CTRL_GAIN"][1]["token"]

        resolved = dict(resolve_duplicate_macros(documents, {"CTRL_GAIN": keep}))

        self.assertEqual(resolved["project private"]["requirements"], [])
        self.assertEqual(resolved["project private"]["requirement_groups"][0]["requirements"], [])
        self.assertEqual(resolved["common:base"]["feature_macros"][0]["macro"], "CTRL_GAIN")

    def test_private_plus_weak_common_is_an_intentional_override(self) -> None:
        documents = [
            ("project private", {"requirements": [{"role": "Gain", "macro": "CTRL_GAIN"}]}),
            ("common:base", {"requirements": [{"role": "Gain", "macro": "CTRL_GAIN", "weak": True}]}),
        ]

        self.assertEqual(duplicate_macro_occurrences(documents), {})

    def test_requirement_name_title_case_preserves_acronyms(self) -> None:
        self.assertEqual(title_case_name("input_PWM_frequency"), "Input PWM Frequency")


if __name__ == "__main__":
    unittest.main()
