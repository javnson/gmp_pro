from __future__ import annotations

import tempfile
import unittest
import json
from pathlib import Path

from project_context import related_requirements


class ProjectContextTests(unittest.TestCase):
    def requirement(self, folder: Path) -> Path:
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / "sdpe_requirement.json"
        path.write_text("{}\n", encoding="utf-8")
        return path.resolve()

    def test_general_opens_every_project_manager(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            suite = Path(temporary) / "suite"
            general = self.requirement(suite / "sdpe_general")
            first = self.requirement(suite / "project" / "simulate" / "sdpe_mgr")
            second = self.requirement(suite / "project" / "target" / "src" / "sdpe_mgr")
            self.assertEqual(
                related_requirements(general.parent),
                [general, first, second],
            )

    def test_project_opens_only_general_and_itself(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            suite = Path(temporary) / "suite"
            general = self.requirement(suite / "sdpe_general")
            current = self.requirement(suite / "project" / "simulate" / "sdpe_mgr")
            self.requirement(suite / "project" / "other" / "sdpe_mgr")
            self.assertEqual(related_requirements(current.parent), [general, current])

    def test_legacy_single_layer_still_opens(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            current = self.requirement(Path(temporary) / "project" / "sdpe_mgr")
            self.assertEqual(related_requirements(current.parent), [current])

    def test_project_opens_all_explicit_common_requirements(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            common_a = self.requirement(root / "library" / "common_a")
            common_b = self.requirement(root / "library" / "common_b")
            current = self.requirement(root / "project" / "sdpe_mgr")
            current.write_text(
                json.dumps(
                    {
                        "common_requirements": [
                            str(common_a),
                            str(common_b),
                        ]
                    }
                ),
                encoding="utf-8",
            )
            self.assertEqual(related_requirements(current.parent), [common_a, common_b, current])


if __name__ == "__main__":
    unittest.main()
