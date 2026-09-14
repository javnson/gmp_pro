#!/usr/bin/env python3
"""Idempotently connect CubeMX main.c to the shared GMP runtime."""

from __future__ import annotations
import argparse
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if text.count(old) != 1:
        raise RuntimeError(f"cannot locate unique {label} marker")
    return text.replace(old, new, 1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("main_c", nargs="?", default="Core/Src/main.c")
    path = Path(parser.parse_args().main_c)
    text = path.read_text(encoding="utf-8")
    text = replace_once(
        text,
        "/* USER CODE BEGIN Includes */\n\n/* USER CODE END Includes */",
        "/* USER CODE BEGIN Includes */\n#include <gmp_core.h>\n\n/* USER CODE END Includes */",
        "include",
    )
    text = replace_once(
        text,
        "  /* USER CODE BEGIN 2 */\n\n  /* USER CODE END 2 */",
        "  /* USER CODE BEGIN 2 */\n  gmp_base_entry();\n  Error_Handler();\n\n  /* USER CODE END 2 */",
        "runtime entry",
    )
    path.write_text(text, encoding="utf-8", newline="\n")
    print(f"patched: {path}")


if __name__ == "__main__":
    main()
