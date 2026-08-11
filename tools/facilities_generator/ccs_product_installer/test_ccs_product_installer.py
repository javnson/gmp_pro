import json
from pathlib import Path
import tempfile
import unittest

import ccs_product_installer as installer


class ProductInstallerTests(unittest.TestCase):
    def setUp(self):
        config_path = Path(installer.__file__).with_name(installer.CONFIG_NAME)
        self.registry = json.loads(config_path.read_text(encoding="utf-8"))

    def test_registry_is_valid(self):
        installer.validate_registry(self.registry)

    def test_family_specific_contract(self):
        c28x = self.registry["products"]["c28x"]
        c29x = self.registry["products"]["c29x"]

        self.assertEqual(18, c28x["ccs_compatibility"]["minimum_ccs_major"])
        self.assertEqual(21, c29x["ccs_compatibility"]["minimum_ccs_major"])
        self.assertEqual("GMP_C28X_CSP_ROOT", c28x["csp_root_macro"])
        self.assertEqual("GMP_C29X_CSP_ROOT", c29x["csp_root_macro"])
        self.assertEqual("C2000WARE", c28x["dependencies"][0]["packageId"])
        self.assertEqual("f29h85x-sdk", c29x["dependencies"][0]["packageId"])
        self.assertIn("F29H85x", c29x["devices"])

    def test_generates_both_products_and_checks_them(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "gmp_core.h").touch()
            (root / ".gitignore").touch()
            (root / "tools/gmp_installer").mkdir(parents=True)
            for product in self.registry["products"].values():
                (root / product["product_path"]).mkdir(parents=True)

            families = list(self.registry["products"])
            paths = installer.install_products(root, self.registry, families)
            self.assertEqual(6, len(paths))
            installer.install_products(
                root, self.registry, families, check_only=True
            )

            for family, product in self.registry["products"].items():
                product_json = json.loads(
                    (root / product["product_path"] / ".metadata/product.json").read_text(
                        encoding="utf-8"
                    )
                )
                self.assertEqual(product["product_id"], product_json["name"])
                self.assertEqual(root.resolve().as_posix(), product_json["includePaths"][0])

                ccs_json = json.loads(
                    (
                        root
                        / product["product_path"]
                        / ".metadata/.tirex/package.ccs.json"
                    ).read_text(encoding="utf-8")
                )[0]
                exports = {item["macroName"]: item["location"] for item in ccs_json["exports"]}
                self.assertEqual("../..", exports["GMP_PRO_ROOT"])
                self.assertEqual(".", exports[product["csp_root_macro"]])


if __name__ == "__main__":
    unittest.main()
