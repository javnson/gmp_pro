import importlib.util
import pathlib
import subprocess
import sys
import tempfile
import unittest


TOOL_DIR = pathlib.Path(__file__).resolve().parents[1]
REPO_ROOT = TOOL_DIR.parents[2]
MODULE_PATH = TOOL_DIR / "canopen_eds_cc.py"
SPEC = importlib.util.spec_from_file_location("canopen_eds_cc", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class EdsCompilerTests(unittest.TestCase):
    def write_eds(self, directory: pathlib.Path, extra: str = "") -> pathlib.Path:
        path = directory / "sample.eds"
        path.write_text(
            "[FileInfo]\nFileName=sample.eds\n"
            "[2000]\nParameterName=Value Object\nObjectType=0x7\n"
            "DataType=0x0007\nAccessType=rw\nDefaultValue=0x12345678\nPDOMapping=1\n"
            "[2001]\nParameterName=Node expression\nObjectType=0x7\n"
            "DataType=0x0006\nAccessType=ro\nDefaultValue=$NODEID+0x600\nPDOMapping=0\n"
            + extra,
            encoding="utf-8",
        )
        return path

    def test_parser_orders_entries_and_resolves_node_id(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = self.write_eds(pathlib.Path(temporary))
            entries = MODULE.load_eds(path, "pointer", 5)
            self.assertEqual([(0x2000, 0), (0x2001, 0)],
                             [(item.index, item.subindex) for item in entries])
            self.assertEqual(0x605, MODULE.parse_int(entries[1].default_value, 5))
            self.assertEqual(12, MODULE.parse_int("00012", 5))

    def test_generates_pointer_and_value_variants(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            path = self.write_eds(root)
            pointer_h, pointer_c = MODULE.compile_eds(path, root / "pointer",
                                                       "sample_od", "pointer", 3)
            value_h, value_c = MODULE.compile_eds(path, root / "value",
                                                   "sample_value_od", "value", 3)
            self.assertIn("storage_2000_00", pointer_h.read_text(encoding="utf-8"))
            self.assertIn("entry_init_pointer", pointer_c.read_text(encoding="utf-8"))
            self.assertIn("entry_init_value", value_c.read_text(encoding="utf-8"))
            self.assertIn("storage.value.u32", value_c.read_text(encoding="utf-8"))

    def test_rejects_duplicate_and_oversized_value_storage(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            duplicate = self.write_eds(
                root,
                "[2000sub0]\nParameterName=Duplicate\n"
                "ObjectType=7\nDataType=7\nAccessType=ro\nDefaultValue=0\n",
            )
            with self.assertRaises(ValueError):
                MODULE.load_eds(duplicate, "pointer", 1)
            oversized = self.write_eds(
                root,
                "[2002]\nParameterName=Long\nObjectType=7\n"
                "DataType=0x000F\nAccessType=rw\nDataLength=9\n"
                "DefaultValue=000102030405060708\nGMPStorage=value\n",
            )
            with self.assertRaises(ValueError):
                MODULE.load_eds(oversized, "pointer", 1)

    def test_rejects_out_of_range_numeric_default(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = pathlib.Path(temporary) / "invalid.eds"
            path.write_text(
                "[2000]\nParameterName=Too large\nObjectType=7\n"
                "DataType=0x0005\nAccessType=rw\nDefaultValue=256\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "outside"):
                MODULE.load_eds(path, "pointer", 1)

    def test_cli_returns_nonzero_for_invalid_input(self):
        with tempfile.TemporaryDirectory() as temporary:
            result = subprocess.run(
                [sys.executable, str(MODULE_PATH), str(pathlib.Path(temporary) / "missing.eds"),
                 "--output-dir", temporary, "--name", "missing_od"],
                check=False, capture_output=True, text=True)
            self.assertNotEqual(0, result.returncode)
            self.assertIn("[ERROR]", result.stderr)

    def test_committed_profile_outputs_are_reproducible(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = pathlib.Path(temporary)
            for profile in ("cia301", "cia401", "cia402"):
                source_dir = REPO_ROOT / "core" / "protocol" / "canopen" / profile
                api_name = f"gmp_{profile}_od"
                header, implementation = MODULE.compile_eds(
                    source_dir / f"{profile}.eds", output / profile,
                    api_name, "pointer", 1,
                )
                self.assertEqual(
                    (source_dir / f"{api_name}.h").read_text(encoding="utf-8"),
                    header.read_text(encoding="utf-8"),
                )
                self.assertEqual(
                    (source_dir / f"{api_name}.c").read_text(encoding="utf-8"),
                    implementation.read_text(encoding="utf-8"),
                )


if __name__ == "__main__":
    unittest.main()
