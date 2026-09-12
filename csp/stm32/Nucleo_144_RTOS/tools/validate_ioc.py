#!/usr/bin/env python3
"""Validate the NUCLEO-H753ZI FreeRTOS reference IOC contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


REQUIRED_LINES = {
    "Mcu.Name=STM32H753ZITx": "STM32H753ZI MCU",
    "Mcu.Package=LQFP144": "Nucleo-144 package",
    "Mcu.IP6=FREERTOS": "FreeRTOS middleware",
    "VP_FREERTOS_VS_CMSIS_V1.Mode=CMSIS_V1": "CMSIS V1 FreeRTOS binding",
    "VP_SYS_VS_tim6.Mode=TIM6": "TIM6 HAL time base",
    "FREERTOS.configTICK_RATE_HZ=1000": "1 kHz RTOS tick",
    "FREERTOS.configMAX_PRIORITIES=7": "seven RTOS priorities",
    "FREERTOS.configTOTAL_HEAP_SIZE=32768": "32 KiB RTOS heap",
    "FREERTOS.configCHECK_FOR_STACK_OVERFLOW=2": "stack overflow checking",
    "FREERTOS.configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5": (
        "FreeRTOS ISR API priority ceiling"
    ),
    "FREERTOS.Tasks01=gmpService,2,1024,StartGmpService,Default,NULL,Dynamic,NULL,NULL": (
        "high-priority GMP service task intent"
    ),
    "ADC1.ConversionDataManagement=ADC_CONVERSIONDATA_DMA_CIRCULAR": (
        "circular control ADC DMA"
    ),
    "ADC1.ExternalTrigConv=ADC_EXTERNALTRIG_T1_TRGO2": "TIM1 ADC trigger",
    "TIM1.CounterMode=TIM_COUNTERMODE_CENTERALIGNED1": "center-aligned PWM",
    "TIM1.Period=4999": "20 kHz PWM period",
    "USART3.BaudRate=921600": "ST-Link UART Data Link",
    "NVIC.DMA1_Stream0_IRQn=true\\:4\\:0\\:false\\:false\\:true\\:false\\:true\\:true": (
        "control ISR above the RTOS syscall ceiling"
    ),
    "NVIC.DMA1_Stream1_IRQn=true\\:5\\:0\\:false\\:false\\:true\\:false\\:true\\:true": (
        "UART RX DMA RTOS-safe priority"
    ),
    "NVIC.DMA1_Stream2_IRQn=true\\:5\\:0\\:false\\:false\\:true\\:false\\:true\\:true": (
        "UART TX DMA RTOS-safe priority"
    ),
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("board_dir", type=Path)
    args = parser.parse_args()
    board_dir = args.board_dir.resolve()
    ioc_files = list(board_dir.glob("*.ioc"))
    if len(ioc_files) != 1:
        raise SystemExit(f"expected exactly one IOC in {board_dir}, found {len(ioc_files)}")

    text = ioc_files[0].read_text(encoding="utf-8")
    failures = [label for line, label in REQUIRED_LINES.items() if line not in text]
    active_ips = {
        line.split("=", 1)[1]
        for line in text.splitlines()
        if line.startswith("Mcu.IP") and not line.startswith("Mcu.IPNb=")
    }
    expected_ips = {
        "ADC1", "CORTEX_M7", "DMA", "ETH", "FREERTOS", "I2C1",
        "MEMORYMAP", "NVIC", "RCC", "SYS", "TIM1", "TIM3", "USART3",
        "NUCLEO-H753ZI",
    }
    missing_ips = sorted(expected_ips - active_ips)
    if missing_ips:
        failures.append("missing IPs: " + ", ".join(missing_ips))
    if "LWIP" in active_ips or "VP_LWIP" in text:
        failures.append("bare-metal NO_SYS LwIP must not be enabled in this project")

    required_files = (
        board_dir / "pin_assign.md",
        board_dir / "config/FreeRTOSConfig.h",
        board_dir / "sdpe_mgr/sdpe_requirement.json",
        board_dir.parent / "src/user/rtos_app.c",
    )
    for required in required_files:
        if not required.is_file():
            failures.append(f"missing contract file: {required}")

    requirement_path = board_dir / "sdpe_mgr/sdpe_requirement.json"
    if requirement_path.is_file():
        requirement = json.loads(requirement_path.read_text(encoding="utf-8"))
        if requirement.get("id") != "nucleo_144_rtos_stm32h753zi":
            failures.append("SDPE project ID is not the RTOS-specific contract")

    if failures:
        for failure in failures:
            print(f"ERROR: {failure}")
        raise SystemExit(1)
    print(f"validated RTOS IOC contract: {ioc_files[0]}")


if __name__ == "__main__":
    main()
