# GMP C2000 LaunchPad evaluation framework

This directory is the switchable C2000 LaunchPad target for GMP.  Application
and GMP-owned files live under `src/`; every `C2000Lib_*` directory contains
only the TI device package, the board-specific SysConfig input, the debugger
target and hardware references required by that LaunchPad.

## Target layout

```text
launchpad/
  .project / .cproject             CCS project and all build configurations
  C2000Lib_<BOARD>/
    device_support/                selected C2000Ware device sources/assembly
    driverlib/                     C2000Ware DriverLib C sources (compiled here)
    device_cmd/                    local linker command files
    targetConfigs/                 debugger and device selection
    hw/                            official schematic and TI board metadata
    LAUNCHXL_<BOARD>.syscfg        board-bound SysConfig source of truth
    build_support/                 selected code-start and enlarged Flash linker file
    sdpe/                          board schema, entity and requirement source
    hardware_preset/mcu_board/     generated board entity header
    launchpad_board.h              generated project binding used by src/
  src/
    gmp_src_mgr/                   GMP source-manager input and scripts
    sdpe_mgr/                      SDPE requirement source of truth
    user/                          portable application tasks
    xplt/                          C2000 platform binding
    pil/                           optional PIL binding
  <BOARD>_Debug/                   ignored CCS/SysConfig build output
  <BOARD>_Release/                 ignored CCS/SysConfig build output
  tools/                           reproducible build/flash/probe steps
```

The shared `src/sdpe_mgr` project owns application timing and protocol
settings only.  Each `C2000Lib_*` owns a dedicated board schema, board entity
and project requirement under `sdpe/`; that requirement generates both the
board preset and `launchpad_board.h`.  Every board entity contains the common
repository-level `boostxl_dual_site` component, so portable application code
uses the same physical-position aliases while board differences remain local.
Regenerate a board with its `sdpe/sdpe_generate.bat`; do not hand-edit its
generated headers or retain stale `.h`/`.m` files.  GMP source-manager output
under `src/gmp_src_mgr/gmp_inc` and `gmp_src` is likewise regenerated from
`gmp_framework_config.json`.

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

All eight configurations implement the executable portability baseline: an
ePWM-triggered ADC control interrupt, one BOOSTXL PWM output, a user LED,
115200 bit/s Data Link UART, board-appropriate CAN/MCAN configuration, and a
DAC output where the device and board expose one.  The ADC sample is forwarded
to PWM and, when present, DAC by the shared application.

F280025C, F280039C and F280049C additionally carry the extended LaunchPad
mapping described below.  The other five boards intentionally remain at the
portable baseline; six-PWM, all-BOOSTXL-ADC, eQEP and optional SPI expansion on
those boards is follow-up work and is not claimed by the current SysConfig.

- User LEDs are GPIO outputs and the XDS virtual COM port is 115200 bit/s.
- Classic CAN targets are configured for 1 Mbit/s. F28P55X instantiates the
  MCAN peripheral and nominal timing, but its shared foreground MCAN data
  service is still a stub; do not describe it as hardware-validated traffic.
- Every dedicated eQEP header is enabled with a position maximum of 10000.
- Site 1 exposes the DRV8301-compatible four-wire SPI mapping: BoosterPack
  pins 7 (clock), 19 (chip select), 15 (controller output) and 14 (controller
  input).
- The six PWM pairs are the three standard pairs on each BoosterPack site.
- Every analog pin in the two BoosterPack analog banks is assigned an ADC SOC.
  SOCs are spread across ADC modules, triggered by ePWM1 SOCA.  ADC interrupt 1
  calls `MainISR` and is the GMP control tick.
- The system clock uses the board crystal and the maximum supported CPU rate.
- `device_support.useStandardCodeStartBranch` is false.  The selected
  `C2000Lib_*` supplies exactly one local code-start source, preventing the
  stale `./src/user/*codestartbranch.obj` linker dependency that affected the
  older projects.

Peripheral names encode physical connector positions, for example
`BOOSTXL_EPWM3940` and `BOOSTXL_ADC23`.  Portable code must consume the aliases
generated by SDPE instead of device GPIO numbers.

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

## Verification gates

Each `<board>_Debug` and `<board>_Release` configuration is complete only when:

1. SysConfig CLI 1.21 accepts its `.syscfg` with C2000Ware 5.04.00.00.
2. The generated pin report contains no resource conflict.
3. The build uses only its selected `C2000Lib_*` directory and links one
   code-start object, one flash command file and the board-local DriverLib
   source objects.
4. Debug and Release both link without unresolved local object paths.
5. The F280049C hardware test confirms the one-shot task disables itself, the
   LED task toggles every 500 ms, the Data Link task runs every 2 ms, and the
   tunable, memory, scope and optional PIL commands respond.

## Verified F280049C baseline

The F280049C baseline has been built, flashed through the on-board XDS110 and
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

Classic CAN is configured for 1 Mbit/s.  Receive object 1 (0x101) is polled by
the 2 ms service; after the first valid input frame, transmit object 4 publishes
ADC/PWM telemetry on 0x201 every 100 ms.  Retry is disabled so a standalone
board without a CAN peer continues running its control and Data Link paths.
Physical two-node CAN traffic remains a separate acceptance test.

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
