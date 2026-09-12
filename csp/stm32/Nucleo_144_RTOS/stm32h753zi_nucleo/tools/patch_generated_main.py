#!/usr/bin/env python3
"""Complete CubeMX H753 output for the GMP FreeRTOS reference project.

CubeMX 6.17 RC recognizes the FreeRTOS middleware in the IOC but can omit the
middleware files and application startup. The IOC remains authoritative; this
script supplies the package-matched kernel and the small GMP-owned user block.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
from pathlib import Path


def locate_cube_firmware(project_root: Path, override: str | None) -> Path:
    ioc_text = (project_root / "stm32h753zi_nucleo.ioc").read_text(
        encoding="utf-8"
    )
    match = re.search(
        r"^ProjectManager\.FirmwarePackage=STM32Cube FW_H7 V([0-9.]+)$",
        ioc_text,
        re.MULTILINE,
    )
    if match is None:
        raise RuntimeError("cannot determine the STM32Cube H7 package from the IOC")

    package_name = f"STM32Cube_FW_H7_V{match.group(1)}"
    candidates: list[Path] = []
    if override:
        candidates.append(Path(override))
    if os.environ.get("STM32_CUBE_REPOSITORY"):
        candidates.append(Path(os.environ["STM32_CUBE_REPOSITORY"]) / package_name)
    candidates.append(Path.home() / "STM32Cube" / "Repository" / package_name)

    for candidate in candidates:
        if (candidate / "Middlewares/Third_Party/FreeRTOS/Source/tasks.c").is_file():
            return candidate.resolve()
    raise RuntimeError(
        f"cannot locate {package_name}; install it with STM32CubeMX or pass "
        "--cube-firmware-root"
    )


def sync_freertos(project_root: Path, firmware_root: Path) -> None:
    source = firmware_root / "Middlewares/Third_Party/FreeRTOS/Source"
    destination = project_root / "Middlewares/Third_Party/FreeRTOS/Source"
    shutil.copytree(source, destination, dirs_exist_ok=True)

    required = (
        "tasks.c",
        "list.c",
        "queue.c",
        "event_groups.c",
        "stream_buffer.c",
        "portable/GCC/ARM_CM7/r0p1/port.c",
        "portable/MemMang/heap_4.c",
    )
    missing = [relative for relative in required if not (destination / relative).is_file()]
    if missing:
        raise RuntimeError("FreeRTOS package is incomplete: " + ", ".join(missing))


def patch_main(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    include_block = (
        "/* USER CODE BEGIN Includes */\n"
        "#include <gmp_core.h>\n"
        "#include \"rtos_app.h\"\n"
        "/* USER CODE END Includes */"
    )
    text, include_count = re.subn(
        r"/\* USER CODE BEGIN Includes \*/.*?/\* USER CODE END Includes \*/",
        include_block,
        text,
        count=1,
        flags=re.DOTALL,
    )
    start_block = (
        "/* USER CODE BEGIN 2 */\n"
        "  gmp_rtos_app_start();\n"
        "  Error_Handler();\n"
        "  /* USER CODE END 2 */"
    )
    text, start_count = re.subn(
        r"/\* USER CODE BEGIN 2 \*/.*?/\* USER CODE END 2 \*/",
        start_block,
        text,
        count=1,
        flags=re.DOTALL,
    )
    if include_count != 1 or start_count != 1:
        raise RuntimeError("cannot locate CubeMX user blocks in main.c")
    if "static void MX_ADC1_Init(void)" not in text:
        raise RuntimeError("CubeMX output does not contain ADC1 initialization")
    if "static void MX_DMA_Init(void)" not in text:
        raise RuntimeError("CubeMX output does not contain DMA initialization")
    if "htim1.Init.Period = 4999;" not in text:
        raise RuntimeError("CubeMX output does not preserve the 20 kHz TIM1 contract")

    # DMA1 Stream0 is the 20 kHz control ISR. It deliberately runs above the
    # FreeRTOS API ceiling and therefore must never call an RTOS API.
    text = text.replace(
        "HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);",
        "HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 4, 0);",
        1,
    )
    path.write_text(text, encoding="utf-8", newline="\n")


def validate_interrupt_ownership(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    for handler in ("SVC_Handler", "PendSV_Handler", "SysTick_Handler"):
        if re.search(rf"\bvoid\s+{handler}\s*\(", text):
            raise RuntimeError(
                f"CubeMX generated {handler}; FreeRTOS must own the core handler"
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("main_c", nargs="?", default="Core/Src/main.c")
    parser.add_argument("--cube-firmware-root")
    args = parser.parse_args()

    main_path = Path(args.main_c).resolve()
    project_root = main_path.parent.parent.parent
    firmware_root = locate_cube_firmware(project_root, args.cube_firmware_root)
    sync_freertos(project_root, firmware_root)
    patch_main(main_path)
    validate_interrupt_ownership(main_path.parent / "stm32h7xx_it.c")
    print(f"patched H753ZI FreeRTOS project: {project_root}")


if __name__ == "__main__":
    main()
