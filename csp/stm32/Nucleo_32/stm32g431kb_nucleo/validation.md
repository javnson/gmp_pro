# NUCLEO-G431KB validation record

## Declared configuration

- MCU: STM32G431KBTx, LQFP32, 128 KB Flash, 32 KB SRAM
- Board: NUCLEO-G431KB (MB1430)
- System clock: HSI16 through PLL, 170 MHz; HAL tick: 1 kHz
- Control source: TIM1, center-aligned 20 kHz, internal CH4 ADC trigger
- QEP: TIM3 AB with native ETR index
- ADC feedback: six injected channels across ADC1 and ADC2
- Data Link: USART2 on PA2/PA3 through ST-Link VCP, 921600 baud,
  circular RX DMA and normal TX DMA
- Optional routed peripherals: none in the control baseline
- Safety state: TIM1 CH1/1N, CH2/2N, CH3/3N and MOE remain disabled

## Hardware and tool evidence

Validation date: 2026-09-12.

- Probe-reported board: NUCLEO-G431KB
- ST-Link serial: `003800433133510437363734`
- ST-Link firmware: V3J9M3
- ST-Link VCP: COM73
- STM32CubeMX database: DB.6.0.160
- STM32CubeG4: 1.6.1
- Arm GNU Toolchain: 14.3.1 (14.3.Rel1)
- CMake: 3.31.5; Ninja: 1.13.2
- STM32CubeProgrammer: 2.19.0
- Target voltage: 3.30 V
- Device ID: 0x468, revision X, 128 KB Flash, Cortex-M4

## Results

| Check | State | Evidence |
| --- | --- | --- |
| IOC/SDPE validation | Passed | Nucleo-32 checker plus SDPE validation passed for 33 schemas and 82 entities |
| CubeMX regeneration | Passed | CubeMX 6.17.0-RC5 / DB.6.0.160 regenerated ADC1/2, TIM1/3, USART2 DMA, I2C1 and GPIO; the generation wrapper preserved the canonical IOC hash |
| GCC compile and link | Passed | Release ELF/HEX/BIN linked; ROM 37,724 B, RAM 10,840 B |
| SWD programming | Passed | 36.84 KB programmed at 0x08000000, read-back verification passed, and the MCU reset |
| GMP DL framing and discovery | Passed | Information response, escaping, sequence matching and facility discovery passed at 921600 baud |
| GMP DL CRC recovery | Passed | Invalid payload CRC produced no response; the following valid request succeeded |
| GMP DL UART DMA | Passed | Initial and eight repeated 256-byte payloads passed; RX/TX counters advanced and the UART error count did not advance during the acceptance window |
| PIL, tunable and memory facilities | Passed | PIL mask/STEP loopback, three float tunables, five memory regions, scratch read/write/restore and read-only rejection passed |
| DSA Scope | Passed | 400 x 2 float32 capture validated a 25 Hz quadrature waveform at 0.50 gain and 0.25 V offset |
| TIM1-to-ADC control chain | Passed | Injected ADC callback measured 20,054.2 Hz during the acceptance window |
| Tick and LED service | Passed in software | LED toggle counter advanced to 20; visual light output was not machine-observed |
| PWM safety | Passed | The exposed compare region was exercised while the hardware output diagnostic remained zero (CCER outputs and MOE disabled) |

One complete end-to-end acceptance run passed. A planned immediate five-run repeat
did not start because Windows reported the board's entire ST-Link USB composite
device as absent after the successful run; this was a USB disconnection, not a
Data Link assertion failure. Repeat-run evidence therefore remains unclaimed.

## Deliberately unclaimed electrical tests

- PWM pin waveforms and dead time require an oscilloscope or logic analyzer.
- ADC order and scaling require calibrated voltages on A0 through A5.
- TIM3 ABZ requires an encoder or pulse generator.
- I2C requires an external slave and pull-ups.
- DAC, FDCAN and PWM Break are not capabilities of this control baseline.

## Reproduction

From this directory:

```powershell
.\build.ps1 -Configuration Release
.\flash.ps1
python .\smoke_test.py --port COM73 --baudrate 921600
```

CubeMX Core/Drivers output, SDPE output, source-manager output, and build products
are derived and ignored. The IOC, SDPE requirement/entity, shared user/xplt
sources, CMake wrapper, and test scripts are authoritative.
