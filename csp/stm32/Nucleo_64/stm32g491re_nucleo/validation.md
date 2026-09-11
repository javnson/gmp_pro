# NUCLEO-G491RE validation record

## Validated configuration

- MCU: STM32G491RETx, LQFP64
- Board: NUCLEO-G491RE
- System clock: 170 MHz
- Control source: TIM1, center-aligned 20 kHz, internal CH4 ADC trigger
- Alternate resources retained in the IOC: TIM8 PWM and TIM4 QEP
- QEP: TIM3 AB with native ETR index
- ADC feedback: six injected channels across ADC1 and ADC2
- Data Link: USART2 on PA2/PA3 through ST-Link VCP, 921600 baud,
  circular RX DMA and normal split-frame TX DMA
- Optional routed peripherals: DAC1 channel 1 and FDCAN1
- Safety state: TIM1 CH1/1N, CH2/2N, CH3/3N and MOE remain disabled

The 921600-baud Data Link setting follows the validated
`tools/gmp_datalink/stm32_dl_dbger` STM32 reference. On this physical G491RE
board, repeated 115200-baud runs suffered intermittent VCP frame loss while the
target UART error counter remained unchanged. The same unmodified acceptance
test was stable at 921600 baud, so 921600 is the target default rather than a
host-side retry workaround.

## Hardware and tool evidence

Validation date: 2026-09-11.

- ST-Link serial number: 0023002A3331511734333834
- ST-Link firmware after manual recovery: V3J15M7
- Probe-reported board: NUCLEO-G491RE
- Target voltage: 3.31 V
- Device ID: 0x479, revision Z, 512 KB Flash, Cortex-M4
- ST-Link VCP: COM65
- STM32CubeMX: 6.17.0-RC5, IOC database DB.6.0.160
- STM32CubeG4: 1.6.1
- Arm GNU Toolchain: 14.3.1 (14.3.Rel1)
- CMake: 3.31.5; Ninja generator
- STM32CubeProgrammer: 2.19.0

## Results

| Check | State | Evidence |
| --- | --- | --- |
| IOC/SDPE validation | Passed | Nucleo-64 IOC checker and SDPE validation passed for 30 schemas and 75 entities |
| CubeMX IOC topology | Passed | G491RE accepted the ADC, TIM, USART, I2C, DAC and FDCAN topology and generated Core/Drivers/MDK output |
| Latest headless regeneration | Tool blocked | STM32CubeMX 6.17.0-RC5 stalled while loading DB.6.0.160 on two attempts; no source or board state was changed, and the authoritative IOC/SDPE build remained reproducible |
| GCC compile and link | Passed | Release image linked with ROM 40,984 B and RAM 11,208 B using the G491 112 KB SRAM / 16 KB CCM layout |
| SWD programming | Passed | STM32CubeProgrammer downloaded the 40.02 KB image, read it back, reported `Download verified successfully`, and reset the MCU |
| GMP DL complete acceptance | Passed | The unmodified target smoke test passed five consecutive runs at 921600 baud on COM65, plus a final run after the last rebuild/reflash |
| GMP DL full-MTU stress | Passed | An additional 100/100 Echo transactions passed with 256-byte payloads and no retry |
| Data Link facilities | Passed | INFO v3, frame escaping, CRC rejection/recovery, PIL mask/STEP, Tunable discovery/read/write, Memory Perspective discovery/read/write/permissions, and DSA Scope passed |
| Control runtime | Passed | TIM1-triggered ADC callback measured 19,998.1 to 20,008.5 Hz across the five acceptance runs |
| Status/safety diagnostics | Passed | LED heartbeat advanced, UART error count did not advance, and PWM output state remained zero in every run |

Final release HEX SHA-256:
`9930BB8CCFB0298D02901DA14B77CE0579F777AD8B414FD6CEF88D6F14E38D28`.

## Deliberately unclaimed electrical tests

- PWM pin waveforms and dead time were not measured because the acceptance
  firmware intentionally leaves all power-output channels disabled.
- The six ADC pins were sampled and returned valid 12-bit values, but channel
  order and scaling still need calibrated injected voltages.
- TIM3 ABZ needs an external encoder or pulse generator.
- DAC voltage needs an oscilloscope or meter.
- I2C needs an external slave.
- FDCAN needs a transceiver and second node.

## Reproduction

From this directory:

```powershell
.\build.ps1 -Configuration Release
.\flash.ps1
python .\smoke_test.py
```

The test auto-detects the single Nucleo ST-Link VCP. Use `--port` only when more
than one compatible board is attached.

CubeMX-derived Core, Drivers and MDK files plus SDPE/source-manager outputs are
ignored build artifacts. The IOC, SDPE requirement/entity, shared user/xplt
sources, CMake wrapper and test script are authoritative.
