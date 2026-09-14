# NUCLEO-F302R8 validation record

Status: hardware-validated base/control/DAC/CAN baseline; external electrical tests pending

Validation date: 2026-09-14

Tested hardware and tools:

- Board: NUCLEO-F302R8, STM32F302R8T6 revision Z, 64 KiB Flash.
- ST-Link: serial `0672FF525750877267113836`, firmware V2J37M26, VCP COM77.
- STM32CubeMX 6.17.0 RC5 with STM32CubeF3 v1.11.6.
- STM32CubeProgrammer 2.19.0; target voltage 3.25 V during register readback.
- GNU Arm Embedded 14.3.1, CMake, and Ninja.

## Implemented contract

- TIM1 20 kHz center-aligned PWM with CH1/CH1N, CH2/CH2N, and CH3/CH3N,
  32 timer-clock-tick dead time, and CH4 OC reference exported through TRGO2.
- Six-channel ADC1 regular sequence with circular DMA and TIM1 TRGO2 trigger.
- TIM2 AB encoder input and PC10 software index. TIM2 is the documented platform
  exception because STM32F302R8 does not implement TIM3 or TIM4.
- USART2 ST-Link VCP with circular RX DMA and normal TX DMA at 921600 baud.
- PB13 LD2 status LED, I2C1, 1 ms HAL tick, DAC channel 1, classic CAN, and SWD.
- One shared Nucleo-64 `user`, `xplt`, and `gmp_src_mgr` implementation; no
  F302 board-name branch exists in the shared runtime.

## PWM startup contract

Reset and early peripheral setup keep the output stage disabled. The timer and
ADC sampling path start first, `ctl_init()` installs half-period compare values,
and normal `init()` then calls `xplt_pwm_enable()`. Explicit disable and fault
paths clear the advanced-timer output enables before slow work.

## Acceptance checklist

- [x] IOC static validation and SDPE schema/entity generation pass.
- [x] CubeMX saves the canonical IOC twice with an identical SHA-256 value
      `218C9C19212D366EDD8F9A4D56CFBBBBF395C038B65401D20605B1123DB48090`.
- [x] CubeMX-generated code contains ADC1, DAC, CAN, TIM1, TIM2, USART2, DMA,
      GPIO, I2C1, SWD, and their initialization functions.
- [x] GCC/CMake Release build passes: 32,180 bytes Flash and 10,768 bytes RAM.
- [x] GCC/CMake Debug build passes: 55,696 bytes Flash and 11,200 bytes RAM.
- [x] ST-Link programming, verification, and reset pass.
- [x] LD2 heartbeat and 1 ms tick advance through platform diagnostics.
- [x] USART2 GMP Data Link u8 passes at 921600 baud.
- [x] Twenty-one consecutive complete DL runs pass without reset or timeout;
      every run covers framing, invalid-CRC recovery, DMA Echo stress, PIL,
      Tunable, Memory Perspective, and a two-channel DSA Scope capture.
- [x] UART error count remains zero while RX/TX DMA callback counts continue to
      advance through the repeated tests.
- [x] TIM1/ADC callback rate remains approximately 20.009-20.043 kHz.
- [x] Platform diagnostics report PWM state `0x80000555` on every run.
- [x] Independent SWD hot-plug readback reports TIM1 `CCER=0x00001555` and
      `BDTR=0x0200A020`, confirming the six phase-channel enables, CH4 trigger
      channel, MOE, and dead-time field are active.
- [ ] Oscilloscope verification of pin-level complementary PWM frequency,
      polarity, dead time, and start/stop transients.
- [ ] Known-voltage ADC ordering/accuracy, QEP direction/index, I2C slave,
      DAC voltage, and external-transceiver CAN tests.

This validation proves generation stability, linking, programming, execution,
serial Data Link facilities, scheduler/heartbeat operation, software-visible ADC
acquisition, and register-level PWM enable state. It does not substitute register
state for connector-level electrical measurements or claim calibrated analog,
encoder, I2C, DAC, or CAN behavior.
