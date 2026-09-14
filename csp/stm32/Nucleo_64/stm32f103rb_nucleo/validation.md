# NUCLEO-F103RB validation record

Status: connected-hardware baseline passed; external electrical tests pending

Validation date: 2026-09-14

## Implemented contract

- STM32F103RBT6 at 72 MHz with SWD enabled and JTAG disabled.
- TIM1 20 kHz center-aligned three-phase complementary PWM with 36 timer-tick
  dead time and enabled physical outputs after the neutral compare values load.
- TIM1 OC4REF -> TIM3 ITR0/reset -> TIM3 update TRGO -> ADC1 hardware trigger.
- Six-channel ADC1 regular scan with DMA1 channel 1 in circular mode.
- TIM4 AB encoder plus PC10 software index.
- USART2 ST-Link VCP with DMA1 channels 6/7. The STM32F1 HAL has no
  `HAL_UARTEx_ReceiveToIdle_DMA`, so shared xplt uses circular DMA plus the UART
  IDLE interrupt while preserving the same GMP DL transport contract.
- PA5 LD2, remapped I2C1 on PB8/PB9, classic CAN on PA11/PA12, and 1 ms tick.
- No DAC capability is advertised because STM32F103RB has no on-chip DAC.

## Acceptance checklist

- [x] CubeMX generates ADC, CAN, I2C, TIM1/TIM3/TIM4, USART2, DMA, GPIO and SWD code.
- [x] Generated MSP code links ADC1/USART2 to DMA1 channels 1/6/7 and remaps I2C1.
- [x] IOC/SDPE static validation and repeat-save stability.
- [x] GCC/CMake Release and Debug builds.
- [x] ST-Link programming, verification and reset.
- [x] GMP Data Link u8 discovery, Echo/CRC recovery, PIL, Tunable, Memory and Scope.
- [x] 20 kHz ADC/control callback, LED heartbeat and enabled PWM register state.
- [ ] Oscilloscope and external-node tests for PWM, ADC, QEP, I2C and CAN.

## Reproducible evidence

- STM32CubeMX 6.17.0-RC5 with STM32Cube FW_F1 V1.8.0 completed a load,
  save and code-generation cycle. A second save preserved SHA-256
  `22F961CE5E1A9E651DCDB2D058A7C1041A69D8B19609CC88C91875E9FD19E605`.
  F1 I2C is explicitly described as fast mode, 400 kHz and duty cycle 2;
  unsupported digital/analog-filter keys are not stored in the IOC.
- The Nucleo-64 IOC validator passed all nine current targets. Generated MSP
  code contains ADC1 circular DMA plus USART2 RX/TX DMA and the I2C1 remap.
- GNU Arm Embedded 14.3.1 Release: 31,280 bytes text, 96 bytes data and
  10,600 bytes BSS. Debug: 49,156 bytes text, 188 bytes data and 10,940 bytes
  BSS. Both fit the 128 KiB Flash / 20 KiB RAM device.
- STM32CubeProgrammer 2.19.0 connected through ST-Link
  `0667FF303939554157193844`, firmware V2J45M31, at 3.24-3.25 V. Device ID
  `0x410` reported the 128 KiB medium-density Cortex-M3 target. Programming,
  verification and reset passed.
- One post-regeneration acceptance run and 25 consecutive full acceptance
  runs passed on COM78 at 921600 baud. Each run covered discovery, CRC reject
  and recovery, full 256-byte Echo, PIL mask/STEP, Tunable, Memory Perspective,
  Scope capture, LED heartbeat and platform diagnostics. UART error baseline
  stayed zero.
- The final post-generation run counted 19,982 callbacks over the measurement
  interval, corresponding to 20,006.9 Hz. Scope returned 400 x 2 float32 samples and
  validated the 25 Hz, 0.50x, 0.25 V signal.
- Independent hot-plug SWD reads returned TIM1 `CCER=0x00001555`, TIM1
  `BDTR=0x0000A024` and ADC1 `CR2=0x00180101`. These values confirm CH1/1N,
  CH2/2N, CH3/3N, the internal CH4 sample point, MOE, 36-tick dead time,
  ADC DMA and the external trigger path are active.

## Startup-race correction

The first hardware run exposed a shared Nucleo-64 initialization race: the
20 kHz PWM/ADC trigger started inside `setup_peripheral()`, before `ctl_init()`
and `user_dl_init()` had completed. An interrupt could therefore observe a
partially initialized variable-channel Scope workspace. The shared xplt now
arms and starts the control trigger from `gmp_csp_post_process()`, after GMP has
prepared the control and user layers. The corrected firmware passed the 25-run
stress sequence, and every existing Nucleo-64 Release target rebuilt.
