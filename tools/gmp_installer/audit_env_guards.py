#!/usr/bin/env python3
"""Fail when a guarded GMP BAT scope contains an unmarked entry point."""

from __future__ import annotations

import os
import sys
from pathlib import Path


GUARD_MARKER = "[GMP_ENV_GUARD]"


def main() -> int:
    root_value = os.environ.get("GMP_PRO_LOCATION")
    if not root_value:
        print("[ERROR] GMP_PRO_LOCATION is not defined.", file=sys.stderr)
        return 1

    root = Path(root_value).resolve()
    targets = {root / "ctl" / "generate_doxygen.bat"}
    for directory in (
        root / "tools" / "SDPE_v2",
        root / "tools" / "facilities_generator" / "src_mgr",
        root / "tools" / "gmp_pil_server" / "gmp_debugger_v2",
    ):
        targets.update(directory.rglob("*.bat"))

    missing = []
    for path in sorted(targets):
        if not path.is_file() or GUARD_MARKER not in path.read_text(encoding="utf-8-sig", errors="replace"):
            missing.append(path)

    if missing:
        print("[ERROR] BAT files missing the GMP environment guard:")
        for path in missing:
            print(f"  - {path}")
        return 1

    print(f"[OK] GMP environment guard coverage: {len(targets)} BAT files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
