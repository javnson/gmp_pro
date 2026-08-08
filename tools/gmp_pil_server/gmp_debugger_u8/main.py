"""Launch the GMP debugger for byte-addressed targets.

The serial wire format is shared with the maintained u16 frontend. This entry
point selects the u8 target contract while reusing the common user interface,
which prevents the two protocol tools from drifting independently.
"""

from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path


def main() -> None:
    """Run the shared debugger frontend with the u8 target contract."""
    shared_frontend = Path(__file__).resolve().parent.parent / "gmp_debugger_u16"
    os.environ["GMP_DATALINK_TARGET_UNIT_BYTES"] = "1"
    sys.path.insert(0, str(shared_frontend))
    runpy.run_path(str(shared_frontend / "main.py"), run_name="__main__")


if __name__ == "__main__":
    main()
