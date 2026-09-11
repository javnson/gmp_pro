# GMP STM32 Nucleo-64 Common Platform Specification and Implementation Plan

**English** | [简体中文](README_CN.md)

Status: G431 configured baseline; G474RE, G491RE, H533RE, C092RC, and F411RE hardware baselines v0.7

Date: 2026-09-11

This document defines the directory layout, hardware-resource contract, SDPE
board components, shared `user`/`xplt` boundary, generation workflow, and
validation criteria for `csp/stm32/Nucleo_64`. The goal is to move one control
application between Nucleo-64 boards by selecting another SDPE board component,
without editing the shared platform sources.

The words MUST, SHOULD, and MAY are normative.

The first G431 baseline now includes the generic SDPE schema, the
`nucleo_g431rb` board entity, selectable TIM1/TIM8 and TIM3/TIM4 bindings, a
G431RB IOC, shared user/xplt sources, the shared source-manager configuration,
and an IOC consistency checker. Its support state is `configured`: SDPE
validation and generation, IOC checks, and shared-source syntax checks pass,
while CubeMX regeneration, a complete link, and hardware tests remain pending.
See `stm32g431rb_nucleo/validation.md` for the exact evidence.

The NUCLEO-G474RE target is now the first hardware-validated target. Its
reproducible flow covers CubeMX generation, SDPE/source-manager generation,
GCC/CMake linking, verified ST-Link programming, and GMP Data Link acceptance
for framing/CRC recovery, DMA transport, PIL, Tunable, Memory Perspective, and
DSA Scope. TIM1-triggered ADC callbacks ran at 20 kHz and the user LED
heartbeat advanced while all three complementary PWM pairs remained disabled.
External-instrument tests for PWM waveforms, calibrated ADC inputs, QEP, DAC,
I2C, and FDCAN remain explicitly pending; see
`stm32g474re_nucleo/validation.md`.

The NUCLEO-G491RE target is the second hardware-validated baseline. It uses the
same shared sources and passed generation, GCC/CMake linking, verified SWD
programming, five consecutive complete GMP Data Link acceptance runs at 921600
baud, and a separate 100/100 full-MTU Echo stress run. Its TIM1-triggered ADC
callback remained at 20 kHz, the UART error counter did not advance, and PWM
outputs remained disabled. The 921600-baud default follows the standalone STM32
Data Link validation project and was more stable than 115200 on the tested
board; see `stm32g491re_nucleo/validation.md` for the exact scope and remaining
electrical tests.

The NUCLEO-H533RE target is the first cross-family hardware baseline. It adds
Cortex-M33/STM32H5 build support, TrustZone-disabled startup, USART2 circular RX
and split TX GPDMA, six TIM1-triggered injected ADC channels, TIM3 ABZ, I2C1,
and optional FDCAN1 while retaining the same shared user/xplt sources. CubeMX
generation, Cortex-M33 GCC linking, verified SWD programming, five complete GMP
Data Link runs and a 100/100 full-MTU Echo stress run passed at 921600 baud. The
control ISR held 20 kHz and PWM outputs remained disabled; see
`stm32h533re_nucleo/validation.md`.

The NUCLEO-C092RC target is the STM32C0 low-resource hardware baseline. Because
STM32C092 has one ADC and no injected group, this target uses a six-channel
fixed regular sequence with circular DMA, triggered by TIM1 TRGO2; the DMA
completion callback advances the 20 kHz control step. QEP index uses a GPIO EXTI
software reset. The target also configures three complementary TIM1 PWM pairs,
TIM3 AB, USART2 VCP, the user LED, I2C1, and FDCAN1 plus the on-board transceiver
standby control. CubeMX generation, GCC/CMake linking, verified SWD programming,
and three complete GMP Data Link smoke runs passed at 921600 baud. Independent
register reads confirmed that the three phase outputs and MOE remained disabled;
see `stm32c092rc_nucleo/validation.md`.

The NUCLEO-F411RE target is the STM32F4 hardware baseline. Its ADC1 regular
group cannot directly select TIM1 TRGO/CC4, so the IOC implements a pin-free
hardware bridge: TIM1 OC4REF -> TIM2 ITR0/reset -> TIM2 update TRGO -> ADC1
six-channel circular DMA. CubeMX generation, SDPE/IOC validation, GCC/CMake
linking, verified SWD programming, and complete GMP Data Link acceptance at
921600 baud passed. The control callback measured 20.269 kHz, and independent
register reads confirmed that all three complementary PWM pairs and MOE stayed
disabled. STM32F411RE has neither on-chip DAC nor CAN/FDCAN; see
`stm32f411re_nucleo/validation.md`.

## 1. Scope and principles

This catalog targets Nucleo-64 boards whose STM32 devices use 64-pin packages
and expose Arduino Uno V3 and/or ST Morpho connectors. Presence in this catalog
does not imply full control capability; capability declarations and validation
records determine support status.

1. Each board has one subdirectory and one preferred `.ioc` file.
2. All boards share `src/user`, `src/xplt`, and `src/gmp_src_mgr`.
3. The IOC is authoritative for pins, clocks, DMA, interrupts, and peripheral
   configuration.
4. An SDPE board component resolves board-specific handles, instances, and
   capabilities into stable aliases.
5. Shared code only consumes stable aliases; it does not directly name
   `huart2`, `htim1`, `GPIOA`, or another board-specific object.
6. Control frequency, dead time, feedback scaling, and physical direction are
   target/application parameters, not properties of the Nucleo board.
7. CubeMX, SDPE, and GMP source-manager outputs are generated artifacts, not
   canonical sources in the main repository.
8. Compilation, simulation, and real-hardware validation are distinct states.

## 2. Capability model

Capabilities are independent flags rather than a single linear level.

| Capability | Meaning | Required resources |
| --- | --- | --- |
| `base` | Basic GMP runtime | DL UART, user LED, I2C, 1 ms system tick |
| `control` | Control-ready platform | advanced PWM, QEP ABZ, six ADC inputs, PWM-triggered ADC |
| `dac` | Analog debug output | at least one on-chip DAC channel on a connector |
| `can` | External communication | connector-accessible CAN/FDCAN TX and RX |
| `pwm_break` | Hardware shutdown | connector-accessible advanced-timer break input |

Every supported board MUST implement `base`. A board may remain in the catalog
without `control`, but it MUST NOT be called control-ready. DAC, CAN, and PWM
break are optional.

## 3. Required hardware contract

### 3.1 Advanced control PWM

The board MUST select TIM1 or TIM8 and provide:

- CH1/CH1N, CH2/CH2N, and CH3/CH3N complementary pairs;
- center-aligned counting;
- coherent three-phase compare updates;
- dead-time, polarity, and idle-state configuration;
- a hardware trigger for control-path ADC acquisition;
- disabled power outputs during reset and initialization;
- a declared `pwm_break` capability when a break signal is available externally.

The SDPE project selects TIM1 or TIM8. Shared `xplt` code MUST NOT branch on a
board ID or assume TIM1. Recommended IOC labels are:

```text
GMP_PWM_U_H    GMP_PWM_U_L
GMP_PWM_V_H    GMP_PWM_V_L
GMP_PWM_W_H    GMP_PWM_W_L
GMP_PWM_BREAK
```

The target SDPE configuration owns PWM frequency, dead time, polarity, and
sample position. The IOC contains a complete, generatable timer topology;
runtime code loads parameters and safely starts it but does not silently replace
the IOC topology.

### 3.2 QEP ABZ

TIM3 or TIM4 MUST operate in encoder mode:

- A and B use CH1 and CH2;
- Z uses an EXTI-capable GPIO;
- the Z event resets or latches the counter through a common platform contract;
- counter width, filtering, direction, and connector positions are documented;
- all three signals are externally accessible.

A device-native index feature MAY replace GPIO+EXTI if it preserves the same
public behavior. Recommended labels are `GMP_QEP_A`, `GMP_QEP_B`, and
`GMP_QEP_Z`.

### 3.3 ADC feedback

A `control` board MUST expose at least six stable logical feedback channels:

```text
GMP_ADC_FB0 ... GMP_ADC_FB5
```

- Arduino A0 through A5 SHOULD be preserved where possible.
- Morpho alternatives MUST be documented in `pin_assign.md`.
- Logical order MUST match the IOC, SDPE output, `ctl_input_callback()`, and the
  application input contract.
- The selected TIM1/TIM8 MUST trigger conversion in hardware.
- Injected conversion is preferred for the fast control path.
- Multiple ADCs may use 3+3, 4+2, or another device-appropriate distribution.
- Additional channels start at `GMP_ADC_AUX0` without reordering FB0 through FB5.
- Resolution, reference voltage, sample time, and synchronization are recorded.

The ADC result-ready path calls `gmp_base_ctl_step()`; a timer update alone MUST
NOT cause incomplete ADC values to be consumed.

### 3.4 GMP Data Link UART

One USART, UART, or LPUART MUST connect to the on-board ST-Link VCP:

- 8-N-1 format;
- 115200 baud as the required compatibility rate;
- 921600 baud as an optional high-speed validation point;
- circular DMA reception with half-complete, complete, and UART-idle handling;
- DMA transmission is allowed; blocking transmission is never called by the
  control ISR;
- the concrete peripheral name is hidden by SDPE aliases.

Recommended labels are `GMP_DL_TX` and `GMP_DL_RX`.

### 3.5 Status LED

One on-board user LED is required and SHOULD be labeled `GMP_STATUS_LED`. SDPE
provides its port, pin, asserted state, and deasserted state. Shared code never
assumes active-high behavior.

### 3.6 I2C

One I2C peripheral is required:

- Arduino D14/SDA and D15/SCL are preferred;
- the instance, pins, alternate functions, and connector positions are recorded;
- 100 kHz and 400 kHz are supported by the baseline test;
- required external pull-ups are documented;
- interrupt and DMA operation are not required in the first release.

Recommended labels are `GMP_I2C_SDA` and `GMP_I2C_SCL`.

### 3.7 System tick

The system tick has 1 ms semantics. STM32 normally uses the HAL tick, and
`gmp_base_get_system_tick()` uses the same timebase as `HAL_GetTick()`. The
PWM/ADC control rate is separate. HAL and the control ISR MUST NOT advance the
same tick twice. A non-SysTick HAL timebase is documented per board.

### 3.8 Optional DAC

An on-chip DAC channel MAY be exposed for fixed, ramp, and selected debug-signal
output. A filtered PWM output is not reported as an on-chip DAC.

### 3.9 Optional CAN/FDCAN

CAN/FDCAN MAY be enabled when TX and RX do not conflict with required resources.
The component records classic CAN versus FDCAN, handle, instance, pins, and
connector positions. It also states whether an external transceiver is required.
Internal loopback precedes external two-node validation. A secondary IOC is
created only for a real pin-multiplexing conflict.

## 4. SDPE board-component contract

A new `stm32_nucleo_64_board` schema SHOULD be added instead of overloading the
existing motor-specific schema:

```text
ctl/hardware_preset/sdpe_schemas/stm32_nucleo_64_board.json
ctl/hardware_preset/sdpe_src/mcu_board/<board_id>.json
```

Each entity references its IOC and describes capabilities and semantic resource
bindings. Application parameters such as feedback gains and motor constants
remain outside the board component.

The generated target settings resolve at least these aliases:

| Area | Required aliases |
| --- | --- |
| Identity | `GMP_NUCLEO_BOARD_ID`, `GMP_NUCLEO_MCU_ID` |
| Clock | `GMP_NUCLEO_SYSTEM_CLOCK_HZ`, `GMP_NUCLEO_SYSTEM_TICK_HZ` |
| PWM | `GMP_NUCLEO_PWM_TIMER_HANDLE`, `GMP_NUCLEO_PWM_TIMER_INSTANCE`, `GMP_NUCLEO_PWM_ADC_TRIGGER` |
| QEP | `GMP_NUCLEO_QEP_TIMER_HANDLE`, `GMP_NUCLEO_QEP_TIMER_INSTANCE`, `GMP_NUCLEO_QEP_Z_PORT`, `GMP_NUCLEO_QEP_Z_PIN`, `GMP_NUCLEO_QEP_SOFTWARE_INDEX` |
| ADC | `GMP_NUCLEO_ADC_REGULAR_DMA`, `GMP_NUCLEO_ADC_FB_COUNT`, `GMP_NUCLEO_ADC_FB<n>_HANDLE`, `GMP_NUCLEO_ADC_FB<n>_RANK` |
| Data Link | `GMP_NUCLEO_DL_UART_HANDLE`, `GMP_NUCLEO_DL_UART_INSTANCE`, `GMP_NUCLEO_DL_RX_DMA_HANDLE`, `GMP_NUCLEO_DL_BAUD_RATE` |
| LED | `GMP_NUCLEO_STATUS_LED_PORT`, `GMP_NUCLEO_STATUS_LED_PIN`, `GMP_NUCLEO_STATUS_LED_ON`, `GMP_NUCLEO_STATUS_LED_OFF` |
| I2C | `GMP_NUCLEO_I2C_HANDLE`, `GMP_NUCLEO_I2C_INSTANCE` |
| Optional | `GMP_NUCLEO_HAS_DAC`, `GMP_NUCLEO_HAS_CAN`, optional CAN-transceiver standby aliases, `GMP_NUCLEO_HAS_PWM_BREAK` |

`<n>` covers at least 0 through 5. Every `HAS_*` macro is always defined as 0
or 1 so unused functionality is removed at compile time.

Each board directory contains `sdpe_mgr/sdpe_requirement.json`. It selects one
board entity, chooses a legal TIM1/TIM8 resource, and emits the stable aliases.
Only the requirement and preset JSON are canonical in the main repository;
generated headers and MATLAB initialization files may be ignored and are kept
when a project is exported as a standalone repository.

## 5. Shared source contract

### `src/user`

The shared application provides the scheduler, LED heartbeat, Data Link service,
optional CAN service, and a safe test controller. PWM remains disabled until an
explicit test state requests it. User code does not name HAL handles or board
GPIOs directly.

### `src/xplt`

The initial shared platform files are:

```text
xplt.config.h
xplt.peripheral.h
xplt.peripheral.c
xplt.ctl_interface.h
```

They bind SDPE aliases, initialize and start ADC/UART/QEP/I2C, implement LED and
optional services, acquire six ADC channels, update three PWM compares, and
provide fast enable/disable functions. The ADC completion path, either injected
conversion interrupt or regular-sequence DMA, calls `gmp_base_ctl_step()`. Fast
shutdown is non-blocking and does not publish
stale PWM values. Optional services use `#if GMP_NUCLEO_HAS_*`.

### `src/gmp_src_mgr`

One project-local `gmp_framework_config.json` selects the modules used by every
board. Canonical distributed scripts are not independently edited here. Header
generation precedes source generation. `gmp_inc`, `gmp_src`, and machine-local
include lists are outputs, and flattened C/C++ source names must remain unique.

## 6. Target directory layout

```text
csp/stm32/Nucleo_64/
├── README.md
├── README_CN.md
├── start_sdpe.bat
├── src/
│   ├── user/
│   ├── xplt/
│   └── gmp_src_mgr/
├── tools/
│   ├── validate_ioc.py
│   ├── generate_board.ps1
│   └── build_all.ps1
├── stm32g431rb_nucleo/
│   ├── stm32g431rb_nucleo.ioc
│   ├── pin_assign.md
│   ├── validation.md
│   ├── .gitignore
│   └── sdpe_mgr/
│       └── sdpe_requirement.json
├── stm32g474re_nucleo/
├── stm32g491re_nucleo/
├── stm32h533re_nucleo/
├── stm32c092rc_nucleo/
└── stm32f411re_nucleo/
```

Each board maintains one preferred IOC. A suffixed IOC such as `*_tim8.ioc` or
`*_can.ioc` is allowed only for a real pin conflict that aliases and feature
macros cannot resolve. DAC, CAN, and compiler choices do not create a Cartesian
product of IOC variants.

## 7. Generation workflow

```text
select board
  -> generate board SDPE settings
  -> generate shared GMP sources
  -> run headless CubeMX into a temporary project
  -> add shared user/xplt and relative include paths
  -> build with CMake/GCC
  -> optionally generate a Keil project
  -> inspect expected outputs
```

Run or double-click `start_sdpe.bat` in this directory. It resolves the repository
root, invokes the canonical `tools/SDPE_v2/gmp_sdpe_project_gui.bat`, and uses the
whole `Nucleo_64` directory as the board-project discovery root, avoiding a
separate launcher for every board.

`GMP_PRO_LOCATION` is registered before GMP tools run. Supported CubeMX and
per-family firmware-package versions are pinned; `LastFirmware` is not a
reproducibility contract. CMake/GCC is the automated baseline. Compiler variants
do not duplicate IOC files. Scripts verify generated startup, HAL initialization,
linker, and ELF outputs rather than trusting exit status alone. Paths remain
relative to the project.

## 8. Static validation

`tools/validate_ioc.py` checks at least:

1. board, MCU, and package consistency between IOC and SDPE;
2. presence of PWM, QEP, ADC, DL UART/DMA, I2C, GPIO, and required clocks;
3. three complete complementary PWM pairs without pin conflicts;
4. unique physical pins, ADC instances, and ranks for FB0 through FB5;
5. an ADC trigger from the selected TIM1/TIM8;
6. QEP A/B on one TIM3/TIM4 CH1/CH2 pair and an EXTI-capable Z input;
7. DL routing to the actual on-board ST-Link VCP;
8. circular UART RX DMA;
9. consistency of LED, I2C, DAC, CAN, and break capability declarations;
10. connector mappings in `pin_assign.md`;
11. supported CubeMX and firmware-package versions;
12. absence of checkout-specific absolute paths.

Board wiring that an IOC cannot prove is reviewed in the SDPE preset and
`pin_assign.md`, then confirmed on hardware.

## 9. Safety and hardware validation

The shared smoke firmware starts with PWM disabled, then enables LED heartbeat,
Data Link, QEP, and ADC observation. PWM only starts after an explicit test
command and uses a bounded duty cycle. Timeout, fault, or communication loss
shuts it down. Safe compare values are written before every enable, and fast
disable removes advanced-timer outputs before slow logging or state work.

A board is accepted only when applicable checks pass:

- SDPE, GMP source-manager, and headless CubeMX outputs are inspected;
- CMake/GCC Debug builds;
- LED heartbeat and 1 ms tick behave correctly;
- Data Link passes at 115200 baud and, when claimed, 921600 baud;
- I2C communicates at 100 kHz and 400 kHz;
- an oscilloscope confirms complementary PWM, center alignment, polarity, and
  dead time, including safe start/stop behavior;
- six known ADC voltages confirm order, range, and trigger timing;
- QEP direction/count/index behavior passes;
- hardware break passes when claimed;
- optional DAC and CAN tests pass at their declared validation level.

Each `validation.md` records board revision, CubeMX, firmware package, compiler,
date, instruments, and exact scope. Hardware status is set only after a real
board test.

## 10. Implementation stages

### A. Freeze the contract

Approve this resource, alias, directory, and generated-file contract; create the
shared source skeleton and board documentation templates; pin first-release
tool versions and ignore rules.

### B. Add the SDPE model

Add the generic schema, stable aliases, G431RB entity, target requirement, and
IOC/SDPE consistency validation. Switching the entity must change all board
handles without a shared-source diff.

### C. Build the G431RB golden board

Create the IOC from the existing control-project evidence, configure every
required resource and applicable optional resource, implement the shared smoke
firmware and generation/build scripts, and complete documentation and hardware
validation.

### D. Add the first same-family boards

Add NUCLEO-G474RE and NUCLEO-G491RE. All three G4 projects use exactly the same
`src/user`, `src/xplt`, and `src/gmp_src_mgr`. A board-ID branch in shared code is
a signal that the SDPE alias contract must be improved.

### E. Expand across families

NUCLEO-H533RE is the validated STM32H5 golden board, NUCLEO-C092RC is the
validated STM32C0 low-resource golden board, and NUCLEO-F411RE is the validated
STM32F4 golden board. Continue with NUCLEO-U083RC, completing one golden board
for each new STM32 family before adding further members.

### F. Fleet validation and release

Run static checks and temporary generated builds for all boards, publish separate
configured/compiled/hardware status, and deprecate or redirect superseded IOC
examples only after their replacements are validated.

## 11. First release deliverables

- approved bilingual specification;
- `stm32_nucleo_64_board` SDPE schema;
- G431RB, G474RE, G491RE, H533RE, C092RC, and F411RE board entities;
- six preferred IOC files and six target SDPE projects;
- one shared `user`, `xplt`, and `gmp_src_mgr` implementation;
- IOC/SDPE static validator;
- headless CubeMX and batch CMake/GCC scripts;
- six pin maps and validation records;
- complete G431RB hardware validation, with accurate status for other boards.

## 12. Non-goals

The first release does not maintain an IOC per compiler, place motor/inverter
physical parameters in the board component, claim optional resources on every
Nucleo-64, substitute compilation for electrical tests, add board-name branches
to shared `xplt`, or hand-maintain generated CubeMX/SDPE/source-manager outputs.

## 13. Existing references

- [G431RB Nucleo example IOC](../Nucleo_Example/stm32g431rb_nucleo/stm32g431rb_nucleo.ioc):
  existing pin-planning evidence, not a complete implementation of this contract;
- [G431 control-suite IOC](../../../ctl/suite/mcs_pmsm_nt/project/stm32g431/stm32g431.ioc):
  TIM1/TIM8, dual injected ADC, UART DMA, and FDCAN reference;
- [STM32 Data Link validation project](../../../tools/gmp_datalink/stm32_dl_dbger):
  CubeMX, CMake, Keil, and Data Link hardware-validation workflow reference;
- [existing STM32 motor-control-board schema](../../../ctl/hardware_preset/sdpe_schemas/stm32_motor_control_board.json):
  a design reference rather than the generic Nucleo-64 schema.
