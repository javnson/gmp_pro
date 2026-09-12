# GMP STM32 Nucleo-32 common platform specification

**English** | [简体中文](README_CN.md)

Status: NUCLEO-G431KB hardware baseline v0.1

Date: 2026-09-12

This document defines the directory, hardware-resource, SDPE, shared-source,
generation, safety, and validation contracts for `csp/stm32/Nucleo_32`. It uses
the same architecture as `Nucleo_64`: applications migrate by selecting an SDPE
board entity, while shared platform code never hard-codes HAL handles or GPIOs.

MUST, SHOULD, and MAY are normative terms.

## 1. Scope and source boundaries

This platform covers 32-pin STM32 Nucleo boards with Arduino Nano V3
connectivity. Each supported board has one preferred IOC, one target SDPE input,
one pin table, and one evidence-based validation record.

- The IOC owns pins, clocks, DMA, interrupts, and peripheral initialization.
- `ctl/hardware_preset/sdpe_src/mcu_board` owns stable board-resource aliases.
- `sdpe_requirement.json` owns target options and application-specific values.
- `src/user`, `src/xplt`, and `src/gmp_src_mgr` are shared by boards here.
- CubeMX, SDPE, source-manager, and CMake outputs are derived artifacts.

Compilation, simulation, and hardware validation are distinct states. MCU
peripheral presence alone does not establish capability; the package and board
routing must satisfy the contract concurrently.

## 2. Independent capabilities

| Capability | Contract |
| --- | --- |
| `base` | ST-Link VCP, board status LED, 1 ms tick, and external I2C |
| `control` | Three complementary PWM pairs, six PWM-triggered ADC inputs, QEP ABZ |
| `dac` | An internal DAC output remains externally usable without consuming a baseline ADC |
| `can` | CAN/FDCAN TX/RX are externally routed; transceiver requirements are documented |
| `pwm_break` | An advanced-timer Break input is externally routed and configured |

Every formally supported board MUST provide `base`. Other capabilities are
independent 0/1 declarations; multiplexed functions cannot be claimed together.

## 3. Required resource contract

### 3.1 PWM and control sampling

A `control` board uses TIM1 or TIM8 CH1/1N, CH2/2N, and CH3/3N in center-aligned
mode with explicit dead time, polarity, idle state, and a hardware ADC trigger.
The ADC completion path invokes `gmp_base_ctl_step()`. CCER output bits and
BDTR.MOE remain clear during boot, initialization, and ordinary acceptance.

### 3.2 QEP and ADC

QEP normally uses TIM3 or TIM4 encoder mode. A/B are CH1/CH2 of one timer; Z is
either native ETR Index or a GPIO EXTI implementation with equivalent semantics.

ADC logical order is fixed as `GMP_ADC_FB0` through `GMP_ADC_FB5`. The IOC,
entity, generated settings, fast input callback, and application preserve that
order. Injected conversion is preferred; a timer-triggered regular circular-DMA
scan is allowed only when the entity declares it.

### 3.3 Base services

- Data Link uses the UART physically connected to ST-Link VCP, 8-N-1, circular
  RX DMA, and normal TX DMA or a non-blocking implementation outside control ISR.
- 115200 baud is the compatibility baseline; 921600 is validated per board.
- LED GPIO and active level are entity data.
- Non-standard Nano I2C routing is recorded in the pin table.
- HAL tick is 1 ms; the PWM/ADC path alone advances the control step.

DAC, CAN/FDCAN, and Break are declared only when they are externally usable in
the baseline without a pin conflict. A resource-tradeoff variant receives an
explicit suffix and cannot silently alter the preferred IOC.

## 4. SDPE contract

The schema is
`ctl/hardware_preset/sdpe_schemas/stm32_nucleo_32_board.json`; entities are
`ctl/hardware_preset/sdpe_src/mcu_board/<board_id>.json`.

Shared code consumes stable `GMP_NUCLEO_*` macros for identity, clock, PWM, QEP,
six ADC inputs, UART/DMA, LED, I2C, and optional peripherals. Every optional
`HAS_*` macro is always defined as 0 or 1.

## 5. Shared-source and safety contract

`src/user` provides a safe minimum GMP application, scheduler, heartbeat, and
Data Link acceptance facilities. `src/xplt` binds HAL resources and implements
fast input, output, and shutdown. Shared code cannot branch on a board ID or name
concrete resources such as `huart2` or `GPIOB`.

Safe compare values are loaded before output enable. Fast shutdown clears MOE
and channel enables before any logging or slow state work. The acceptance image
contains no remote command that enables power outputs.

## 6. Directory form

```text
csp/stm32/Nucleo_32/
├── README.md / README_CN.md
├── start_sdpe.bat
├── src/{user,xplt,gmp_src_mgr}/
├── tools/validate_ioc.py
└── stm32g431kb_nucleo/
    ├── stm32g431kb_nucleo.ioc
    ├── sdpe_mgr/sdpe_requirement.json
    ├── pin_assign.md / validation.md
    ├── generate.ps1 / build.ps1 / flash.ps1
    ├── CMakeLists.txt / cmake/
    └── smoke_test.py
```

## 7. Generation and build

The canonical order is IOC/SDPE validation, entity and target generation,
source-manager headers then sources, CubeMX regeneration, GMP entry injection,
and GCC/CMake linking. `GMP_PRO_LOCATION` is registered before GMP tools run.
The firmware package is pinned and `LastFirmware` is forbidden. Project paths
remain repository-relative.

## 8. Static validation

`tools/validate_ioc.py` checks device/package/schema, authoritative IOC path,
all six PWM outputs, QEP ABZ, ADC order and trigger, VCP/DMA, LED, I2C, clock
source, pinned package, and absolute-path contamination. Connector and solder-
bridge facts that an IOC cannot prove are owned by `pin_assign.md` and hardware
evidence.

## 9. NUCLEO-G431KB baseline

The first entity, `nucleo_g431kb`, declares `base=1` and `control=1`, with
`dac=0`, `can=0`, and `pwm_break=0` for this topology. It uses HSI16 through the
PLL at 170 MHz, TIM1 three-phase PWM at 20 kHz, two plus four injected ADC ranks
on Nano A0-A5, TIM3 ABZ, USART2 VCP, PB8 LD2, and I2C1 on PA15/PB7. Using HSI
releases PF0 for TIM1_CH3N; see `stm32g431kb_nucleo/pin_assign.md` for the HSE
solder-bridge constraint and the exact connector map.

## 10. Board acceptance

Each board records SDPE/IOC validation, CubeMX regeneration, full GCC link, SWD
program/verify, LED/tick, Data Link framing and CRC recovery, PIL/tunable/memory/
scope facilities, control ISR frequency, and proof that PWM remains disabled.
Electrical ADC, PWM, QEP, I2C, DAC, and CAN tests remain explicitly unclaimed
until the required external equipment is attached.

Reproduce G431KB from its directory:

```powershell
.\build.ps1 -Configuration Release
.\flash.ps1
python .\smoke_test.py --port COM73 --baudrate 921600
```
