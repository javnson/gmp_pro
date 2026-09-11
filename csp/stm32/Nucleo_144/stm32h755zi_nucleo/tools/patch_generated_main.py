#!/usr/bin/env python3
"""Idempotently complete the H753ZI CubeMX output and enter GMP."""

from __future__ import annotations
import argparse
import os
import re
import shutil
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if text.count(old) != 1:
        raise RuntimeError(f"cannot locate unique {label} marker")
    return text.replace(old, new, 1)


def locate_cube_firmware(project_root: Path, override: str | None) -> Path:
    """Locate the exact H7 firmware package selected by the authoritative IOC."""
    ioc_text = (project_root / "stm32h753zi_nucleo.ioc").read_text(encoding="utf-8")
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
    repository = os.environ.get("STM32_CUBE_REPOSITORY")
    if repository:
        candidates.append(Path(repository) / package_name)
    candidates.append(Path.home() / "STM32Cube" / "Repository" / package_name)
    for candidate in candidates:
        if (candidate / "Drivers" / "STM32H7xx_HAL_Driver").is_dir():
            return candidate.resolve()
    raise RuntimeError(
        f"cannot locate {package_name}; install it with STM32CubeMX or pass "
        "--cube-firmware-root"
    )


def sync_adc_driver(project_root: Path, firmware_root: Path) -> None:
    """Supply ADC files that CubeMX 6.17 omits for this board-template IOC."""
    driver_root = firmware_root / "Drivers" / "STM32H7xx_HAL_Driver"
    project_driver = project_root / "Drivers" / "STM32H7xx_HAL_Driver"
    files = (
        Path("Inc/stm32h7xx_hal_adc.h"),
        Path("Inc/stm32h7xx_hal_adc_ex.h"),
        Path("Inc/stm32h7xx_ll_adc.h"),
        Path("Src/stm32h7xx_hal_adc.c"),
        Path("Src/stm32h7xx_hal_adc_ex.c"),
    )
    for relative in files:
        source = driver_root / relative
        destination = project_driver / relative
        if not source.is_file():
            raise RuntimeError(f"Cube firmware file is missing: {source}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not destination.is_file() or destination.read_bytes() != source.read_bytes():
            shutil.copy2(source, destination)


ADC_AND_DMA = r"""
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef channel = {0};
  RCC_PeriphCLKInitTypeDef peripheral_clock = {0};

  peripheral_clock.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  peripheral_clock.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
  if (HAL_RCCEx_PeriphCLKConfig(&peripheral_clock) != HAL_OK)
    Error_Handler();
  __HAL_RCC_ADC12_CLK_ENABLE();
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    Error_Handler();

  hdma_adc1.Instance = DMA1_Stream0;
  hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    Error_Handler();
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  channel.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  channel.SingleDiff = ADC_SINGLE_ENDED;
  channel.OffsetNumber = ADC_OFFSET_NONE;
  channel.Offset = 0;

  channel.Channel = ADC_CHANNEL_15;
  channel.Rank = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) Error_Handler();
  channel.Channel = ADC_CHANNEL_10;
  channel.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) Error_Handler();
  channel.Channel = ADC_CHANNEL_5;
  channel.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) Error_Handler();
  channel.Channel = ADC_CHANNEL_18;
  channel.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) Error_Handler();
  channel.Channel = ADC_CHANNEL_19;
  channel.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) Error_Handler();
  channel.Channel = ADC_CHANNEL_16;
  channel.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) Error_Handler();
}

"""


def patch_main(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    text = text.replace(
        "/* USER CODE BEGIN Includes */\n#include <gmp_core.h>\n\n/* USER CODE END Includes */",
        "/* USER CODE BEGIN Includes */\n\n/* USER CODE END Includes */",
        1,
    )
    text = replace_once(
        text,
        '#include "main.h"\n#include "lwip.h"',
        '#include "main.h"\n#include <gmp_core.h>\n#include "lwip.h"',
        "GMP include",
    )
    text = replace_once(
        text,
        "UART_HandleTypeDef huart3;\n",
        "UART_HandleTypeDef huart3;\n\n"
        "ADC_HandleTypeDef hadc1;\n"
        "DMA_HandleTypeDef hdma_adc1;\n"
        "DMA_HandleTypeDef hdma_usart3_rx;\n"
        "DMA_HandleTypeDef hdma_usart3_tx;\n",
        "ADC/DMA handles",
    )
    text = replace_once(
        text,
        "static void MX_USART3_UART_Init(void);\n",
        "static void MX_USART3_UART_Init(void);\n"
        "static void MX_DMA_Init(void);\n"
        "static void MX_ADC1_Init(void);\n",
        "ADC/DMA prototypes",
    )
    text = replace_once(
        text,
        "  MX_GPIO_Init();\n",
        "  MX_GPIO_Init();\n  MX_DMA_Init();\n  MX_ADC1_Init();\n",
        "ADC/DMA initialization",
    )
    text = replace_once(
        text,
        "  htim1.Init.Period = 65535;\n",
        "  htim1.Init.Period = 4999;\n",
        "TIM1 period",
    )
    text = replace_once(
        text,
        "/**\n  * @brief I2C1 Initialization Function",
        ADC_AND_DMA + "/**\n  * @brief I2C1 Initialization Function",
        "ADC/DMA functions",
    )
    text = replace_once(
        text,
        "  /* USER CODE BEGIN 2 */\n\n  /* USER CODE END 2 */",
        "  /* USER CODE BEGIN 2 */\n  gmp_base_entry();\n  Error_Handler();\n\n  /* USER CODE END 2 */",
        "runtime entry",
    )
    path.write_text(text, encoding="utf-8", newline="\n")


def patch_msp(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    text = replace_once(
        text,
        "/* USER CODE BEGIN ExternalFunctions */\n\n/* USER CODE END ExternalFunctions */",
        "/* USER CODE BEGIN ExternalFunctions */\n"
        "extern DMA_HandleTypeDef hdma_usart3_rx;\n"
        "extern DMA_HandleTypeDef hdma_usart3_tx;\n"
        "/* USER CODE END ExternalFunctions */",
        "DMA externs",
    )
    marker = "    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);\n\n    /* USART3 interrupt Init */"
    dma = """    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    hdma_usart3_rx.Instance = DMA1_Stream2;
    hdma_usart3_rx.Init.Request = DMA_REQUEST_USART3_RX;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);

    hdma_usart3_tx.Instance = DMA1_Stream1;
    hdma_usart3_tx.Init.Request = DMA_REQUEST_USART3_TX;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

    /* USART3 interrupt Init */"""
    text = replace_once(text, marker, dma, "USART3 DMA setup")
    path.write_text(text, encoding="utf-8", newline="\n")


def patch_interrupts(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    text = replace_once(
        text,
        "extern UART_HandleTypeDef huart3;\n",
        "extern UART_HandleTypeDef huart3;\n"
        "extern DMA_HandleTypeDef hdma_adc1;\n"
        "extern DMA_HandleTypeDef hdma_usart3_tx;\n"
        "extern DMA_HandleTypeDef hdma_usart3_rx;\n",
        "DMA IRQ externs",
    )
    handlers = """
void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

"""
    text = replace_once(
        text,
        "/**\n  * @brief This function handles USART3 global interrupt.",
        handlers + "/**\n  * @brief This function handles USART3 global interrupt.",
        "DMA and QEP IRQs",
    )
    path.write_text(text, encoding="utf-8", newline="\n")


def patch_hal_config(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    disabled = "  /* #define HAL_ADC_MODULE_ENABLED   */"
    enabled = "#define HAL_ADC_MODULE_ENABLED"
    if disabled in text:
        text = text.replace(disabled, enabled, 1)
    elif not any(line.strip() == enabled for line in text.splitlines()):
        raise RuntimeError("cannot locate HAL ADC module marker")
    path.write_text(text, encoding="utf-8", newline="\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("main_c", nargs="?", default="Core/Src/main.c")
    parser.add_argument("--cube-firmware-root")
    args = parser.parse_args()
    main_path = Path(args.main_c).resolve()
    core_src = main_path.parent
    project_root = core_src.parent.parent
    sync_adc_driver(
        project_root, locate_cube_firmware(project_root, args.cube_firmware_root)
    )
    patch_main(main_path)
    patch_msp(core_src / "stm32h7xx_hal_msp.c")
    patch_interrupts(core_src / "stm32h7xx_it.c")
    patch_hal_config(core_src.parent / "Inc" / "stm32h7xx_hal_conf.h")
    print(f"patched H753ZI CubeMX output: {main_path.parent.parent}")


if __name__ == "__main__":
    main()
