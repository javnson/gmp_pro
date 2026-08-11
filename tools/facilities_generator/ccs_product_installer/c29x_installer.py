"""Compatibility entry point for generating only the GMP C29x Product."""

from ccs_product_installer import main


if __name__ == "__main__":
    raise SystemExit(main(["--product", "c29x"]))
