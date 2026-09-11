# NUCLEO-C092RC validation

## Safety boundary

- TIM1 CH1/CH2/CH3 and complementary outputs are configured, but CCER output enables and BDTR.MOE remain cleared during validation.
- The smoke test checks the exported PWM diagnostic and fails if any PWM output becomes enabled.
- No external power stage is required for this validation.

## Commands

```powershell
.\build.ps1 -Configuration Release
.\flash.ps1
python .\smoke_test.py --port COM59 --baudrate 921600
```

## Validation record

Validated on 2026-09-11 with the connected NUCLEO-C092RC:

- Board: NUCLEO-C092RC, STM32C09x device ID `0x44D`, revision A, 256 KiB flash.
- Probe: on-board ST-LINK/V2-1, serial `066CFF525648847187123216`, firmware
  `V2J45M31`, target voltage 3.24 V, VCP `COM59`.
- Tools: STM32CubeMX 6.17.0-RC5, STM32CubeC0 package 1.4.0,
  STM32CubeProgrammer 2.19.0, Arm GNU Toolchain 14.3.1.
- Headless CubeMX generation completed from the checked-in IOC.
- Release GCC/CMake build completed: 40,860 bytes flash (15.59% of 256 KiB) and
  10,912 bytes RAM (35.52% of 30 KiB).
- ST-LINK erase, program, verify, and reset completed successfully.
- Three GMP Data Link smoke runs, including one immediately after rebuilding
  and reflashing the target, passed at 921600 baud. Frame/CRC transport,
  scratch-memory discovery, PIL mask synchronization and STEP loopback, tunable
  discovery, and two-channel float32 DSA Scope acquisition all passed.
- The TIM1-triggered ADC DMA callback ran at 20,005.3 Hz, 20,004.8 Hz, and
  20,007.6 Hz. The UART error count remained zero and the user-LED heartbeat
  advanced.
- The exported PWM diagnostic remained zero. An independent hot-plug register
  read found `TIM1_CCER=0x00001000` (only internal CH4 trigger enabled) and
  `TIM1_BDTR=0x02002018`; CH1/CH1N through CH3/CH3N and BDTR.MOE were all clear.

This record validates generation, compilation, programming, the digital GMP DL
path, the 20 kHz control callback, heartbeat observation, and the safe-disabled
PWM state. It does not substitute for external electrical tests. Known-voltage
ADC channel/order tests, external QEP A/B/Z, complementary PWM/dead-time
waveforms, I2C communication, and two-node FDCAN communication remain pending.
