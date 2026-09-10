# NUCLEO-G431RB validation record

## Declared configuration

- MCU: STM32G431RBTx
- CubeMX file format: 6.16.0
- STM32CubeG4 firmware package: 1.6.1
- System clock: 170 MHz
- Default control timer: TIM1, center-aligned, 20 kHz target
- Default QEP timer: TIM3
- Data Link: USART2 VCP, 115200 baud, circular RX DMA
- Optional routed peripherals: DAC1 channel 1 and FDCAN1 (no onboard transceiver)

## Evidence

| Check | State | Evidence |
| --- | --- | --- |
| IOC/SDPE static validation | Passed | `tools/validate_ioc.py`, 2026-09-10 |
| SDPE global validation | Passed | 30 schemas and 73 entities, 2026-09-10 |
| SDPE target generation | Passed | Generated and inspected temporary `ctrl_settings.h`, 2026-09-10 |
| GMP source-manager generation | Passed | 89 headers and 11 flattened sources generated from the shared configuration |
| Shared user/xplt syntax | Passed | `arm-none-eabi-gcc -fsyntax-only -Wall -Wextra` against regenerated G431 headers and shared GMP includes |
| CubeMX regeneration | Passed | STM32CubeMX 6.17.0-RC5 loaded the 6.16/DB.6.0.160 IOC and reported successful code generation |
| Generated HAL source syntax | Passed | All 11 generated `Src/*.c` files passed `arm-none-eabi-gcc -fsyntax-only -Wall -Wextra` |
| GCC/CMake build | Not run | Requires regenerated CubeMX project wrapper |
| LED / Tick / Data Link | Not tested | Physical board required |
| PWM waveform and safe enable | Not tested | Oscilloscope and power-safe fixture required |
| ADC six-channel order / trigger | Not tested | Calibrated voltage source required |
| QEP ABZ | Not tested | Encoder or pulse generator required |
| DAC | Not tested | Oscilloscope required |
| FDCAN loopback / external bus | Not tested | External transceiver and second node required |

Do not promote this target from `configured` to `compiled` or `hardware` without
updating the table with tool versions, board revision, test date, and evidence.
