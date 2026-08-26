#!/usr/bin/env python3
"""Generate C++ for MNA data whose RK stages were precomputed into affine maps."""

from __future__ import annotations

from cpp_codegen import main_for_methods


if __name__ == "__main__":
    raise SystemExit(main_for_methods({"rk4"}))
