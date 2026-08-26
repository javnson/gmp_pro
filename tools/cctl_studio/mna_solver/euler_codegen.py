#!/usr/bin/env python3
"""Generate C++ for precomputed forward/backward-Euler MNA data."""

from __future__ import annotations

from cpp_codegen import main_for_methods


if __name__ == "__main__":
    raise SystemExit(main_for_methods({"forward_euler", "backward_euler"}))
