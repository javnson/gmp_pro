import tempfile
import unittest
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

import gmp_keil_pack


class KeilPackTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = Path(__file__).resolve().parents[3]

    def test_generate_real_registry(self):
        with tempfile.TemporaryDirectory() as temp:
            pdsc, pack, component_count, file_count = gmp_keil_pack.generate(
                self.repo_root,
                self.repo_root / "tools/facilities_generator/src_mgr/gmp_framework_dic.json",
                self.repo_root / "GMP.GeneralMotorPlatform.xml",
                Path(temp),
                False,
            )
            self.assertGreater(component_count, 10)
            self.assertGreater(file_count, 20)
            tree = ET.parse(pdsc)
            names = [node.get("name") for node in tree.findall("./components/component/files/file")]
            self.assertIn("gmp_core.h", names)
            self.assertIn("ctl/portable/gmp_ctl_portable.h", names)
            self.assertTrue(pack and pack.is_file())
            with zipfile.ZipFile(pack) as archive:
                self.assertIsNone(archive.testzip())
                self.assertEqual(1, len([name for name in archive.namelist() if name.endswith(".pdsc")]))

    def test_metadata_has_pack_safe_name(self):
        metadata = gmp_keil_pack.load_metadata(self.repo_root / "GMP.GeneralMotorPlatform.xml")
        self.assertNotIn(" ", metadata.name)
        self.assertRegex(metadata.version, r"^\d+\.\d+\.\d+$")


if __name__ == "__main__":
    unittest.main()
