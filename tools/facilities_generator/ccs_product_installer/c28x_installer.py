"""Compatibility entry point for generating only the GMP C28x Product."""

from ccs_product_installer import main


if __name__ == "__main__":
    raise SystemExit(main(["--product", "c28x"]))
