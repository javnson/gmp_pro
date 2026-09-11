# NUCLEO-G474RE validation record

## Declared configuration

- MCU: STM32G474RETx, LQFP64
- Board: NUCLEO-G474RE
- System clock: 170 MHz
- Control source: TIM1, center-aligned 20 kHz, internal CH4 ADC trigger
- Alternate resources retained in the IOC: TIM8 PWM and TIM4 QEP
- QEP: TIM3 AB with native ETR index
- ADC feedback: six injected channels across ADC1 and ADC2
- Data Link: USART2 on PA2/PA3 through ST-Link VCP, 115200 baud,
  circular RX DMA and normal TX DMA
- Optional routed peripherals: DAC1 channel 1 and FDCAN1
- Safety state: TIM1 CH1/1N, CH2/2N, CH3/3N and MOE remain disabled

## Hardware and tool evidence

Validation date: 2026-09-11.

- Probe-reported board: NUCLEO-G474RE
- ST-Link firmware: V3J9M3
- Target voltage: 3.31 V
- Device ID: 0x469, revision X, 512 KB Flash, Cortex-M4
- STM32CubeMX: 6.17.0-RC5, IOC database DB.6.0.160
- STM32CubeG4: 1.6.1
- Arm GNU Toolchain: 14.3.1 (14.3.Rel1)
- CMake: 3.31.5; Ninja generator
- STM32CubeProgrammer: 2.19.0
- ST-Link VCP during validation: COM64

## Results

| Check | State | Evidence |
| --- | --- | --- |
| IOC/SDPE validation | Passed | Nucleo-64 IOC checker and SDPE validation passed for 30 schemas and 74 entities |
| CubeMX regeneration | Passed | G474RE accepted all migrated ADC, TIM, USART, I2C, DAC and FDCAN resources and generated Core/Drivers/MDK output |
| GCC compile and link | Passed | Release ELF/HEX/BIN linked; ROM 41,592 B, RAM 11,208 B |
| SWD programming | Passed | 40.62 KB programmed at 0x08000000 and read-back verification passed |
| GMP DL framing and discovery | Passed | Information response, escaping, sequence matching and facility discovery passed |
| GMP DL CRC recovery | Passed | Invalid payload CRC produced no response; the next valid request succeeded |
| GMP DL UART DMA | Passed | Initial 256-byte echo plus eight repeated 256-byte patterns passed; RX/TX callback counters advanced and the UART error counter remained zero during the final stress window |
| PIL facility | Passed | Mask synchronization and STEP loopback preserved digital, PWM and monitor fields |
| Tunable facility | Passed | Three float parameters discovered, changed, read back and restored |
| Memory perspective | Passed | Five regions discovered; scratch read/write/restore passed and the ADC read-only region rejected a write |
| PWM safety through DL | Passed | Compare values were changed/read/restored through the RW region while the hardware output state remained zero |
| TIM1 to ADC control chain | Passed | Injected ADC callback measured 20,005.9 Hz during the final acceptance window |
| Tick and LED service | Passed in software | Heartbeat/LED toggle counter advanced; visual light output was not machine-observed |
| DSA Scope | Passed | 400 x 2 float32 capture at 1 kHz validated a 25 Hz quadrature waveform at 0.5 gain and 0.25 offset |

## Deliberately unclaimed tests

- PWM pin waveforms and dead time were not measured because the acceptance
  firmware intentionally leaves all power-output channels disabled.
- The six ADC pins were sampled and values were in the 12-bit range, but channel
  order and scaling were not tested with calibrated injected voltages.
- TIM3 ABZ was initialized, but no external encoder or pulse generator was attached.
- DAC analog voltage was not measured with an oscilloscope.
- I2C was initialized, but no external slave was attached.
- FDCAN routing was generated, but no transceiver or second node was attached.

## Reproduction

From this directory:

```powershell
.\build.ps1 -Configuration Release
.\flash.ps1
python .\smoke_test.py --port COM64 --baudrate 115200
```

CubeMX-derived Core, Drivers and MDK files plus SDPE/source-manager outputs are
ignored build artifacts. The IOC, SDPE requirement/entity, shared user/xplt
sources, CMake wrapper and test script are authoritative.
