# NUCLEO-H533RE validation record

## Intended configuration

- MCU: STM32H533RETx, Arm Cortex-M33, LQFP64, TrustZone disabled
- Board: NUCLEO-H533RE (MB1814)
- System clock: 250 MHz from the on-board 24 MHz HSE
- Control source: TIM1, center-aligned 20 kHz, internal CH4 ADC trigger
- QEP: TIM3 AB with native ETR index
- ADC feedback: six injected channels across ADC1 and ADC2
- Data Link: USART2 on PA2/PA3 through STLINK-V3EC VCP, 921600 baud,
  circular RX GPDMA and normal split-frame TX GPDMA
- Base peripherals: PA5 status LED and I2C1 on PB6/PB7
- Optional routed peripheral: FDCAN1 on PA11/PA12
- DAC capability: disabled because the DAC pins are not independently routed
- Safety state: TIM1 CH1/1N, CH2/2N, CH3/3N and MOE remain disabled

## Hardware discovery

Validation date: 2026-09-11.

- ST-Link serial number: 002300243133511335333335
- ST-Link firmware: V3J13M4
- Probe-reported board: NUCLEO-H533RE
- Target voltage: 3.27 V
- Device ID: 0x478, STM32H533/523, 512 KB Flash, Cortex-M33
- Debug access: AP0 rejected as expected for the current H5 access map; the
  programmer selected AP1 and connected successfully under reset
- ST-Link VCP: COM69

## Results

| Check | State | Evidence |
| --- | --- | --- |
| Board discovery | Passed | STLINK-V3EC, VCP and target SWD access are visible |
| IOC/SDPE validation | Passed | All four Nucleo-64 IOCs and all 76 SDPE entities validate |
| CubeMX generation | Passed | STM32CubeMX 6.17 with STM32CubeH5 V1.6.0 generated ADC1/2, GPDMA, TIM1/3, USART2, I2C1 and FDCAN1 code |
| GCC compile/link | Passed | Cortex-M33/FPv5 release image: 50,748 B text, 88 B data, 10,184 B BSS; Flash 9.70%, RAM 3.69% |
| SWD programming | Passed | 49.65 KB HEX erased, downloaded, read back and verified; MCU reset completed |
| GMP DL acceptance | Passed | Five complete runs at 921600 baud: framing/CRC, Memory Perspective, PIL, Tunable and 400 x 2 float32 DSA Scope, including a final run after stress |
| GMP DL full-MTU stress | Passed | A separate 100/100 Echo run passed with 256-byte payloads and no retry |
| Control runtime/safety | Passed | TIM1-triggered callback measured 19,996.5-20,001.3 Hz; LED heartbeat advanced; PWM state stayed 0 in every run |

Release HEX SHA-256:
`BEF027F632DBA4CFC0C3C1CF991A12788BAF00545971A970C001D4DD4E8A09FE`.

The H5 implementation also verified that USART2 RX uses a circular GPDMA linked
list, TX uses a normal memory-to-peripheral GPDMA channel, and ADC1/ADC2 use
TIM1 TRGO rising-edge injected conversions. The one UART error visible at the
first diagnostic read remained the baseline value through the five acceptance
runs and the full-MTU stress test and did not increment.

STM32CubeMX 6.17.0-RC5 reports optional import diagnostics for the legacy
BOOTPATH field and an implicit RIF virtual pin inherited from the H5 database.
It reports no clock-readiness failure, exits successfully, and generates the
complete Core/Drivers output used for the verified build.

## Deliberately unclaimed electrical tests

- PWM waveforms, polarity and dead time require an oscilloscope.
- The six ADC channels require calibrated injected voltages.
- TIM3 ABZ requires an external encoder or pulse generator.
- I2C requires an external slave and pull-ups.
- FDCAN requires a transceiver and second node.

## Reproduction

From this directory:

```powershell
.\build.ps1 -Configuration Release
.\flash.ps1
python .\smoke_test.py
```

CubeMX-derived Core, Drivers and MDK files plus SDPE/source-manager outputs are
ignored build artifacts. The IOC, SDPE requirement/entity, shared user/xplt
sources, CMake wrapper and test script are authoritative.
