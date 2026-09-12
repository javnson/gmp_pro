# NUCLEO-U083RC validation record

Status: hardware-validated base/control/DAC baseline; external electrical tests pending

Validation date: 2026-09-12

Tested hardware and tools:

- Board: NUCLEO-U083RC, STM32U083RCT6 revision A, 256 KiB Flash.
- ST-Link: serial `066EFF373857343143085141`, firmware V2J43M28, VCP COM75.
- STM32CubeMX 6.17.0 RC5 with STM32CubeU0 v1.3.0.
- STM32CubeProgrammer 2.19.0.
- GNU Arm Embedded 14.3.1, CMake, and Ninja.

## Implemented contract

- STM32CubeU0 v1.3.0 and CubeMX 6.17 configuration.
- TIM1 center-aligned three-pair complementary PWM, with CH4 OC reference driving ADC1 through TRGO2.
- Six-channel ADC1 regular scan with circular DMA.
- TIM3 AB encoder input and PC10 software index.
- USART2 ST-Link VCP with circular RX DMA and normal TX DMA.
- LD4 status LED, I2C1, 1 ms HAL tick, SWD debug, and optional DAC1 output.
- CAN/FDCAN is explicitly unavailable on STM32U083.

## Safety

Power-stage PWM output enable remains off after reset. The common runtime starts the timer and ADC sampling path, but only an explicit call to `xplt_pwm_enable()` asserts the channel enables and main output enable.

## Acceptance checklist

- [x] IOC survives a CubeMX load/save/generate cycle without losing DMA or pin assignments.
- [x] CubeMX-generated ADC MSP code contains PC0-PC3 and PA0-PA1 plus circular DMA.
- [x] SDPE generation and IOC validation pass.
- [x] GCC/CMake Release build passes: 43,908 bytes Flash and 10,848 bytes RAM.
- [x] GCC/CMake Debug build passes: 68,292 bytes Flash and 11,272 bytes RAM.
- [x] ST-Link flash and read-back verification pass.
- [x] LD4 heartbeat and 1 ms tick advance through platform diagnostics.
- [x] USART2 GMP Data Link u8 passes at 921600 baud.
- [x] Twenty-seven complete DL runs pass without reset or timeout; these include 243 successful
      256-byte Echo transfers, repeated Tunable/Memory/Scope operations, and bad-CRC recovery.
- [x] UART error count remains zero throughout the repeated tests.
- [x] TIM1/ADC control callback rate is approximately 19.97-20.01 kHz and PWM output state
      remains zero.
- [ ] External electrical tests for PWM, QEP, ADC, I2C, and DAC are completed when instruments are available.

The hardware validation above proves the on-board execution, SWD programming, VCP transport,
software-visible ADC acquisition, scheduler/heartbeat, and safe PWM state. It does not claim
connector-level electrical behavior for PWM, calibrated analog accuracy, encoder direction/index,
I2C transactions with a slave, or DAC voltage accuracy.

CubeMX 6.17 changes only `ProjectManager.LastFirmware` to `true` when saving against the currently
latest installed U0 package; the explicit package remains v1.3.0 and no peripheral, DMA, or pin
entry is removed. The canonical IOC keeps `LastFirmware=false` to preserve the repository's pinned
package policy.
