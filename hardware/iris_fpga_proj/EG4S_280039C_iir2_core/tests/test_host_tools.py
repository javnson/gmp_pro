import json
import csv
import math
import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "host"))
import gmp_fpga_control as host


class HostToolsTest(unittest.TestCase):
    def test_quantize_and_saturation(self):
        self.assertEqual(host.quantize(0.5), 0x08000000)
        self.assertEqual(host.quantize(-0.5), -0x08000000)
        self.assertEqual(host.quantize(100.0), 0x7FFFFFFF)

    def test_sos_register_image(self):
        sos = [[0.5, 0.25, 0.0, 1.0, -0.5, 0.0]]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "registers.json"
            host.write_register_image(output, host.normalize_sos(sos))
            payload = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(payload["writes"][0]["address"], 0x40)
        self.assertEqual(payload["writes"][0]["value"], 0x08000000)
        self.assertEqual(payload["writes"][-1]["name"], "commit")

    def test_chirp_size_and_bound(self):
        values = host.make_log_chirp(1000.0, 1.0, 1.0, 100.0, 0.2)
        self.assertEqual(len(values), 1000)
        self.assertLessEqual(max(abs(v) for v in values), 0.2000001)

    def test_transfer_function_conversion(self):
        sos = host.transfer_function_to_sos([0.1, 0.1], [1.0, -0.8])
        self.assertEqual(len(sos), 1)
        self.assertEqual(len(sos[0]), 5)

    def test_identity_frequency_response(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "measurement.csv"
            result = Path(directory) / "response.csv"
            with source.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(("input", "output"))
                for index in range(128):
                    value = math.sin(2 * math.pi * 7 * index / 128)
                    writer.writerow((value, value))
            host.analyze_response(source, result, 128.0)
            with result.open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        self.assertGreater(len(rows), 0)
        self.assertTrue(all(abs(float(row["magnitude_db"])) < 1e-9 for row in rows))


if __name__ == "__main__":
    unittest.main()
