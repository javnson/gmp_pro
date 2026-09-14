#!/usr/bin/env python3
"""Idempotently connect CubeMX F103 sources to the shared GMP runtime."""

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
    parser.add_argument("interrupt_c", nargs="?", default="Core/Src/stm32f1xx_it.c")
    args = parser.parse_args()

    main_path = Path(args.main_c)
    text = main_path.read_text(encoding="utf-8")
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
    main_path.write_text(text, encoding="utf-8", newline="\n")

    interrupt_path = Path(args.interrupt_c)
    text = interrupt_path.read_text(encoding="utf-8")
    text = replace_once(
        text,
        "  /* USER CODE BEGIN USART2_IRQn 0 */\n\n  /* USER CODE END USART2_IRQn 0 */",
        "  /* USER CODE BEGIN USART2_IRQn 0 */\n  xplt_dl_uart_irq();\n\n  /* USER CODE END USART2_IRQn 0 */",
        "USART2 IDLE hook",
    )
    text = replace_once(
        text,
        "/* USER CODE BEGIN 0 */\n\n/* USER CODE END 0 */",
        "/* USER CODE BEGIN 0 */\nextern void xplt_dl_uart_irq(void);\n\n/* USER CODE END 0 */",
        "USART2 IDLE declaration",
    )
    interrupt_path.write_text(text, encoding="utf-8", newline="\n")
    print(f"patched: {main_path}, {interrupt_path}")


if __name__ == "__main__":
    main()
