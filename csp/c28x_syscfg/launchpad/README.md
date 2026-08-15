# GMP C2000 LaunchPad evaluation framework

This directory is the switchable C2000 LaunchPad target for GMP. Application
and GMP-owned source files live under `src/`; every `C2000Lib_*` directory owns
the TI device package, board-specific SysConfig input, debugger target and
hardware references, plus SDPE-generated board binding headers.

## Target layout

```text
csp/c28x_syscfg/
  sdpe_component/                  reusable C2000 board schemas and entities
  launchpad/
    .project / .cproject           CCS project and all build configurations
    C2000Lib_<BOARD>/
      device_support/              selected C2000Ware device sources/assembly
      driverlib/                   C2000Ware DriverLib C sources (compiled here)
      device_cmd/                  local linker command files
      targetConfigs/               debugger and device selection
      hw/                          official schematic and TI board metadata
      LAUNCHXL_<BOARD>.syscfg      board-bound SysConfig source of truth
      build_support/               selected code-start and enlarged Flash linker file
      hardware_preset/mcu_board/   generated board entity header
      launchpad_board.h            generated project binding used by src/
    src/
      gmp_src_mgr/                 GMP source-manager input and scripts
      sdpe_mgr/                    common settings plus one requirement per board
        requirements/<BOARD>/     ADC, PWM and DAC channel selections
      user/                        scheduler, control and Data Link application code
      xplt/                        C2000 platform binding
      pil/                         optional PIL binding
    <BOARD>_Debug/                 ignored CCS/SysConfig build output
    <BOARD>_Release/               ignored CCS/SysConfig build output
    tools/                         reproducible build/flash/probe steps
```

The shared `src/sdpe_mgr/sdpe_requirement.json` is a Common requirement that
owns application timing and protocol settings. Each
`requirements/<BOARD>/sdpe_requirement.json` is a Private requirement that
binds that Common file through `common_requirements` and owns the application
channel choices for one board. Reusable board schemas and
entities live once under `../sdpe_component` and are registered in the global
`tools/SDPE_v2/sdpe_settings.json`; they are not copied into `C2000Lib_*`.
Each board requirement includes its board entity and exposes three selections:
`LAUNCHPAD_CONTROL_ADC_SELECT`, `LAUNCHPAD_CONTROL_PWM_BASE` and
`LAUNCHPAD_CONTROL_DAC_BASE`. The ADC selection generates a coupled SOC/result
mapping, while a no-DAC device selects `0U` and compiles out the DAC write.
`tools/generate_board_sdpe.bat` generates the selected board preset, one merged
`launchpad_board.h`, and a co-located Common-then-Private MATLAB script pair.
Source files consume only `launchpad_board.h`; there is no parallel generated
`ctrl_settings.h` in the compile path. Do not hand-edit those generated files
or retain stale `.h`/`.m` files. The complete rationale and data flow are in
`doc/SDPE_ARCHITECTURE.md`. GMP source-manager output
under `src/gmp_src_mgr/gmp_inc` and `gmp_src` is likewise regenerated from
`gmp_framework_config.json`.

Each CCS configuration exposes two independent build variables under
**Project Properties > Build > Variables**:

- `GMP_PREBUILD_SDPE=1` composes the shared settings into the active board's
  `launchpad_board.h` and regenerates its MATLAB scripts before compilation.
- `GMP_PREBUILD_SRC_MGR=1` regenerates `gmp_inc` and `gmp_src` before
  compilation.

Both switches default to `0`; `1`, `true`, `yes` or `on` enables one.  The
hook locates GMP through the registered `GMP-Core-C28x` Product and validates
the expected outputs instead of trusting the generator exit code alone.  If a
source-manager edit adds or removes `.c` files while building in the CCS GUI,
use **Rebuild Project** so managed-build refreshes its source list.  The
headless helper runs generation before project import and therefore handles a
source-set change in one invocation:

```powershell
.\tools\build.ps1 -Board F280049C -Mode All -GenerateSdpe -GenerateGmpSources
```

Every CCS configuration declares `C2000WARE:5.4.0.00` and
`GMP-Core-C28x:2.10.00.00` as products.  GMP includes and symbols enter through
`${COM_TI_COM_GMP_CORE_C28X_SDK_*}`; C2000 device and DriverLib headers enter
through `${COM_TI_C2000WARE_INSTALL_DIR}`.  Board-local DriverLib `.c` files are
compiled directly so application debugging can step into TI implementation
code; no prebuilt DriverLib archive or copied C2000Ware header is retained.
After moving the repository, rerun the
GMP CCS product installer so the product metadata is registered at the new
location; the project itself contains no `${PROJECT_LOC}/../..`,
`GMP_PRO_ROOT` or `GMP_C28X_CSP_ROOT` checkout guesses.

Portable application buffers use the logical linker section `mass_data`, not
a device-specific `ramgs*` name.  Each board linker file owns the physical
mapping: F2800137C uses `RAMLS1`, F280025C uses the combined `RAMLS4567`
region, and the other six targets use `RAMGS0`.  F28379D also maps the
DriverLib IPC sections to the corresponding CPU1/CPU2 message RAM.  Application
code can therefore keep one section name without assuming that a device has a
particular GS bank.

## Board matrix

| Configuration prefix | Device family | CPU target | Board debugger | Board support state |
|---|---|---:|---|---|
| `F2800137C` | F280013x | 120 MHz | XDS110 | portable baseline; Debug/Release built |
| `F280025C` | F28002x | 100 MHz | XDS110 | extended mapping; Debug/Release built |
| `F280039C` | F28003x | 120 MHz | XDS110 | extended mapping; Debug/Release built |
| `F280049C` | F28004x | 100 MHz | XDS110 | extended mapping, builds and hardware baseline validated |
| `F28377S` | F2837xS | 200 MHz | XDS100v2 | portable baseline; Debug/Release built |
| `F28379D` | F2837xD CPU1 | 200 MHz | XDS100v2 | portable baseline; Debug/Release built |
| `F28P55X` | F28P55x CPU1 | 150 MHz | XDS110 | portable MCAN baseline; Debug/Release built |
| `F28P65X` | F28P65x CPU1 | 200 MHz | XDS110 | portable baseline; Debug/Release built |

`F28377S` and `F28379D` deliberately use XDS100v2.  Those two older LaunchPads
do not contain XDS110 hardware, so changing only the CCXML connection name
would make the project description disagree with the board.

## Peripheral coverage

All eight configurations now implement the extended control-oriented mapping.
Every usable BOOSTXL analog input has an ADC SOC, every device DAC is enabled,
and six physical BOOSTXL PWM pairs are present.  When a DAC and ADC share a
pin, the DAC owns it.  The shared application forwards its selected ADC sample
to PWM and, when available, DAC.

| Board | ADC SOCs | Device DACs | BOOSTXL PWM pairs | ADC control interrupt |
|---|---:|---:|---:|---|
| F2800137C | 16 | 0 | 6 | ADCA INT1 |
| F280025C | 16 | 0 | 6 | ADCA INT1 |
| F280039C | 18 | 2 | 6 | ADCA INT1 |
| F280049C | 19 | 2 | 6 | ADCA INT1 |
| F28377S | 11 | 3 | 6 | ADCA INT1 |
| F28379D | 14 | 3 | 6 | ADCA INT1 |
| F28P55X | 19 | 1 | 6 | ADCA INT1 |
| F28P65X | 18 | 2 | 6 | ADCA INT1 |

The ADC counts are the BOOSTXL inputs left after DAC priority; they are spread
as evenly as the device pin map permits across the available ADC modules.
F28379D's DACC is configured even though it is not routed to a BOOSTXL analog
position, because the board contract exposes every DAC implemented by the
device.

- User LEDs are GPIO outputs and the XDS virtual COM port is 115200 bit/s.
- Classic CAN targets are configured for 1 Mbit/s. F28P55X instantiates the
  MCAN peripheral and nominal timing, but its shared foreground MCAN data
  service is still a stub; do not describe it as hardware-validated traffic.
- Every dedicated eQEP header is enabled with a position maximum of 10000.
- Site 1 exposes the DRV8301-compatible four-wire SPI mapping: BoosterPack
  pins 7 (clock), 19 (chip select), 15 (controller output) and 14 (controller
  input).
- The six PWM pairs are the three standard pairs on each BoosterPack site and
  retain stable physical aliases: `BOOSTXL_EPWM4039`, `BOOSTXL_EPWM3837`,
  `BOOSTXL_EPWM3635`, `BOOSTXL_EPWM8079`, `BOOSTXL_EPWM7877` and
  `BOOSTXL_EPWM7675`.
- ADC SOCs are triggered by the board's synchronized master ePWM SOCA.  This is
  ePWM1 on seven targets.  F28377S uses ePWM7 because the six PWM peripherals
  actually routed to its BOOSTXL connectors do not include ePWM1.  ADCA
  interrupt 1 calls `MainISR` on every target and is the GMP control tick.
- `GMP_LAUNCHPAD_PWM_FREQUENCY_HZ` in the shared SDPE requirement controls all
  six center-aligned PWM pairs.  `setup_peripheral()` stops TBCLKSYNC, applies
  one period and phase zero to every pair, resets all counters, and releases
  them together.  Frequencies below 1 kHz are rejected at compile time.
- Site-1 BOOSTXL pin 13 is `ENABLE_PORT` (output), pin 34 is
  `OVER_TEMPERATURE` (input), and pin 33 is `RELAY_PORT` (output) wherever the
  MCU and LaunchPad routing permit it.  Gate enable and relay are driven low
  during initialization before application tasks run.
- The system clock uses the board crystal and the maximum supported CPU rate.
- `device_support.useStandardCodeStartBranch` is false.  The selected
  `C2000Lib_*` supplies exactly one local code-start source, preventing the
  stale `./src/user/*codestartbranch.obj` linker dependency that affected the
  older projects.

Peripheral names encode physical connector positions, for example
`BOOSTXL_EPWM3940` and `BOOSTXL_ADC23`.  Portable code must consume the aliases
generated by SDPE instead of device GPIO numbers.

`src/user/user_main.c` now contains only scheduler composition and the
startup, heartbeat and independent CAN tasks. `src/user/user_dl.c` owns the
serial Data Link protocol, tunable parameters, memory service, scope/trigger
state and optional PIL service. The scheduler, task table, task counters,
Data Link context, scope state and CAN counters are non-static so they can be
added directly to the CCS Expressions view.

## Hardware multiplexing exceptions

SysConfig pin-mux validation and LaunchPad switch routing are both part of the
acceptance test.  A valid MCU pinmux is not sufficient when the PCB routes the
same GPIO through an exclusive switch.

On LAUNCHXL-F280049C, I2CA on the standard site-1 I2C position and eQEP1 on J12
both use GPIO35/GPIO37 through S3/S4.  The other I2CA choices conflict with one
of the required PWM pairs, CAN, or the J13 eQEP2 index.  Consequently the
single conflict-free baseline keeps both encoder headers, CAN, six PWM pairs
and the DRV8301 SPI, and does not falsely claim a simultaneously connected
hardware I2C port.  An I2C-oriented configuration must explicitly trade one of
those resources and state the required switch positions.

Control peripherals take priority over optional SPI/I2C routing.  A requested
control GPIO is omitted only where the physical board cannot provide it:

| Board | ENABLE pin 13 | Relay pin 33 | Over-temperature pin 34 | Exception |
|---|---|---|---|---|
| F2800137C | GPIO37 | omitted | GPIO33 | BOOSTXL pin 33 is not connected to a usable GPIO |
| F280025C | GPIO23 | omitted | GPIO42 | pin 33 is GPIO19/X1 and is reserved by the external crystal |
| F280039C | GPIO37 | GPIO48 | GPIO33 | none |
| F280049C | GPIO39 | GPIO30 | GPIO58 | none |
| F28377S | GPIO72 | GPIO21 | GPIO20 | none |
| F28379D | GPIO124 | omitted | GPIO24 | pin 33 is dedicated GPO16/OUTPUTXBAR, not a GPIO |
| F28P55X | GPIO21 | GPIO23 | GPIO22 | none |
| F28P65X | GPIO12 | GPIO15 | GPIO14 | none |

F280039C uses the green LED instead of the red LED because the red-LED GPIO is
also a required BOOSTXL ADC input.  F28P65X likewise uses its green LED because
BOOSTXL pin 13 takes priority as gate enable.  These choices keep the 500 ms
heartbeat without consuming a control pin.

## Verification gates

Each `<board>_Debug` and `<board>_Release` configuration is complete only when:

1. SysConfig CLI 1.21 accepts its `.syscfg` with C2000Ware 5.04.00.00.
2. The generated pin report contains no resource conflict.
3. The build uses only its selected `C2000Lib_*` directory and links one
   code-start object, one flash command file and the board-local DriverLib
   source objects.
4. Debug and Release both link without unresolved local object paths.
5. The F280049C hardware test confirms the one-shot task disables itself, the
   LED task toggles every 500 ms, the Data Link and CAN tasks run independently,
   and the tunable, memory, scope and optional PIL commands respond.

## Previously verified F280049C baseline

The pre-refactor F280049C baseline was built, flashed through the on-board XDS110 and
run on a physical LAUNCHXL-F280049C.  A 30-second debugger observation produced
one startup execution, 60 heartbeat executions, 15000 Data Link task
executions, 600019 ADC/control ISR executions and a 30000 ms GMP system tick.
The live ADC result was non-zero and changed between observations.

The serial smoke test passed at 115200 bit/s on the XDS110 application UART,
including payload echo and 256-byte stress transfer, three tunable parameters,
the 128-byte scratch memory service and the 400-sample two-channel scope.  The
SCI adapter uses its FIFO interrupt plus a foreground FIFO pump; protocol
dispatch remains in the required 2 ms scheduler task.  This prevents a long
continuous frame from overflowing the 16-word hardware FIFO.

Classic CAN is configured for 1 Mbit/s. In the current source, receive object
1 (0x101) is polled by its own 2 ms task rather than the Data Link task; after
the first valid input frame, transmit object 4 publishes
ADC/PWM telemetry on 0x201 every 100 ms.  Retry is disabled so a standalone
board without a CAN peer continues running its control and Data Link paths.
Physical two-node CAN traffic and a repeat of the full acceptance test after
the SDPE/task refactor remain separate acceptance tests.

The repository-root CCS project contains formal Debug and Release
configurations for all eight boards. They perform clean headless builds from
exactly one selected `C2000Lib_*` directory and the shared `src` tree.
CCS invokes SysConfig directly on the board-bound `.syscfg` and writes all
generated sources below the ignored configuration output directory.  No
generated SysConfig source is checked into the repository.  `build_support`
contains the one selected code-start source and enlarged Flash linker file.
The linker keeps the C runtime default `_c_int00` entry point; the Flash boot
vector still reaches the `codestart` section, whose `code_start` trampoline
branches to `_c_int00`.  This avoids warning #10063 without changing hardware
startup behavior.

From this directory, the repeatable build workflow is:

```powershell
.\tools\build.ps1 -Board F280049C -Mode All
.\tools\build.ps1 -Board F280039C -Mode All
.\tools\build.ps1 -Board F28P65X -Mode All
.\tools\flash_f280049c.ps1 -Mode Debug
.\tools\smoke_f280049c.ps1 -Port COM5
C:\ti\ccs1281\ccs\ccs_base\scripting\bin\dss.bat .\tools\hardware_probe_f280049c.js
```

## Hardware references

The PDFs under each `hw/` directory are the configuration authority:

- [LAUNCHXL-F2800137 schematic](https://www.ti.com/lit/df/sprr447a/sprr447a.pdf)
- [LAUNCHXL-F280025C schematic](https://www.ti.com/lit/df/sprr425a/sprr425a.pdf)
- [LAUNCHXL-F280039C schematic](https://www.ti.com/lit/df/sprr444a/sprr444a.pdf)
- [LAUNCHXL-F280049C schematic](https://www.ti.com/lit/df/sprr423/sprr423.pdf)
- [LAUNCHXL-F28377S user guide and schematic](https://www.ti.com/lit/pdf/sprui25)
- [LAUNCHXL-F28379D user guide and schematic](https://www.ti.com/lit/pdf/sprui77)
- [LAUNCHXL-F28P55X schematic](https://www.ti.com/lit/df/sprr499a/sprr499a.pdf)
- [LAUNCHXL-F28P65X schematic](https://www.ti.com/lit/df/sprr480/sprr480.pdf)
