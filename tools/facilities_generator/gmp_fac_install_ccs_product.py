"""Backward-compatible entry point for the data-driven CCS Product installer."""

from pathlib import Path
import sys


INSTALLER_DIR = Path(__file__).resolve().parent / "ccs_product_installer"
sys.path.insert(0, str(INSTALLER_DIR))

from ccs_product_installer import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
