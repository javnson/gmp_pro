# NUCLEO-F411RE validation

## Safety boundary

- TIM1 CH1/CH2/CH3 and complementary outputs are configured, but CCER output enables and BDTR.MOE remain cleared during validation.
- The smoke test checks the exported PWM diagnostic and fails if any PWM output becomes enabled.
- No external power stage is required for this validation.

## Commands

```powershell
.\build.ps1 -Configuration Release
.\flash.ps1
python .\smoke_test.py --port COM8 --baudrate 921600
```

## Validation record

Hardware validation completed on 2026-09-11.

| Item | Result |
| --- | --- |
| Target | NUCLEO-F411RE / STM32F411xC/E, device ID `0x431`, revision A, 512 KiB Flash |
| Probe | ST-LINK serial `066DFF524957775187105120`, firmware V2J45M31, target voltage 3.25-3.26 V |
| Generator | STM32CubeMX 6.17.0, STM32Cube FW_F4 V1.28.3 |
| Build | GCC/CMake Release passed; Flash 30,324 B (5.78%), RAM 10,792 B (8.23%) |
| Program | SWD under-reset download, verify, and reset passed |
| Data Link | USART2/ST-Link VCP on COM8, u8 transport at 921600 baud passed |
| DL facilities | Memory discovery, PIL mask/STEP loopback, 3 Tunable parameters, and 400 x 2 float32 Scope passed |
| Control timing | ADC DMA completion ISR measured at 20,269.1 Hz |
| Runtime diagnostics | RX 221, TX 72, UART error baseline 0, LED toggles 31, PWM output state 0 |

The final hardware-trigger path is TIM1 OC4REF -> TIM1 TRGO -> TIM2 ITR0/reset -> TIM2 update TRGO -> ADC1 regular six-channel circular DMA. Hot-plug register reads confirmed `TIM2_CR2=0x20`, `TIM2_SMCR=0x04`, and `ADC1_CR2=0x16000301` while the firmware was running.

Safety reads confirmed `TIM1_CCER=0x00001000` (only internal CH4 enabled; CH1/CH1N through CH3/CH3N disabled) and `TIM1_BDTR=0x0000202A` (`MOE=0`). No external power stage was connected or driven.

Still pending external-instrument validation: PWM waveform/dead time after an explicitly authorized enable, calibrated ADC voltages and channel order, QEP ABZ electrical behavior, and I2C communication with a real slave. DAC and CAN/FDCAN are not available on STM32F411RE.
