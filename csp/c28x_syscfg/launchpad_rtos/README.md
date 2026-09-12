# GMP C2000 LaunchPad FreeRTOS Reference

This directory contains a multi-target GMP + TI FreeRTOS reference project for
the C2000 LaunchPad families already supported by the sibling bare-metal
project. It is independent from `csp/c28x_syscfg/launchpad` and does not change
the bare-metal execution model.

## Supported targets

The single CCS project exposes Debug and Release configurations for all eight
targets:

| Board argument | CCS configurations | FreeRTOS CPU clock | Validation |
| --- | --- | ---: | --- |
| `F2800137C` | `F2800137C_Debug`, `F2800137C_Release` | 120 MHz | build passed |
| `F280025C` | `F280025C_Debug`, `F280025C_Release` | 50 MHz | build passed |
| `F280039C` | `F280039C_Debug`, `F280039C_Release` | 60 MHz | build passed |
| `F280049C` | `F280049C_Debug`, `F280049C_Release` | 100 MHz | build and hardware passed |
| `F28377S` | `F28377S_Debug`, `F28377S_Release` | 200 MHz | build passed |
| `F28379D` | `F28379D_Debug`, `F28379D_Release` | 200 MHz | build passed |
| `F28P55X` | `F28P55X_Debug`, `F28P55X_Release` | 150 MHz | build passed |
| `F28P65X` | `F28P65X_Debug`, `F28P65X_Release` | 200 MHz | build passed |

Build validation covers SysConfig generation, compilation, and linking. Only
the currently connected LAUNCHXL-F280049C has completed on-target validation;
the other boards still require individual clock, interrupt, control-loop, and
Data Link acceptance tests before release.

## Execution model

FreeRTOS owns scheduling and CPU Timer2/INT14. GMP's cooperative, non-blocking
dispatcher runs inside the priority-6 `gmpService` task every 1 ms. The
priority-1 `userTask` demonstrates independent application work. ADC control
loops remain hardware ISRs and call no RTOS API. SCI interrupts only transfer
bytes while GMP Data Link parsing runs in the service task.

Scope acquisition is called from `ctl_dispatch()` on every control interrupt,
so the reference configurations publish the real 20 kHz control rate. Optional
decimation is owned by the Scope protocol's runtime `sample_divider`; neither
the CSP nor the FreeRTOS service task performs hidden fixed-rate sampling.

Each root `LAUNCHXL_*.syscfg` file is the authoritative peripheral, FreeRTOS,
and task configuration for its board. A CCS configuration includes only its
matching SysConfig input, `C2000Lib_<board>`, and SDPE requirements. Generated
build and SysConfig output must not be committed. Static tasks are used,
dynamic allocation is disabled, and every board linker command file explicitly
places the FreeRTOS static-stack and heap sections.

## Select and build

The validated tool set is CCS 12.8.1, C2000Ware 5.04.00.00, SysConfig 1.21.0,
and TI C2000 Compiler 22.6.1.LTS. Import this directory as one existing CCS
project, then choose the desired target under `Build Configurations > Set
Active`; no project copy or manual SysConfig replacement is needed.

The command-line helper uses the same board names. For example:

```powershell
& .\csp\c28x_syscfg\launchpad_rtos\tools\build.ps1 `
    -Board F28379D -Mode All -GenerateGmpSources
```

`-Mode` accepts `Debug`, `Release`, or `All`. The default board remains
`F280049C` for compatibility with existing invocations.

## Flash and validate F280049C

The repository's automated flashing, serial smoke test, and hardware probe are
currently specific to the connected LAUNCHXL-F280049C:

```powershell
& .\csp\c28x_syscfg\launchpad_rtos\tools\flash_f280049c.ps1 -Mode Debug
& .\csp\c28x_syscfg\launchpad_rtos\tools\smoke_f280049c.ps1 `
    -Port COM5 -BaudRate 115200
```

Hardware validation on 2026-09-12 covered RTOS scheduling, the GMP service
task, the independent user task, the 20 kHz control ISR, and Data Link info,
echo, tunable, memory, and 400 x 2 float32 scope transfers. Protocol CRC, FIFO,
timeout, and SCI overrun counters were all zero. PIL is intentionally absent in
the normal physical-control configuration and is tested only when advertised.

After correcting the real-time Scope path, a five-second target probe measured
`control_isr_runs=99898` and `dl_scope_control_steps=99898`, with a published
sample rate of 20000 Hz and a runtime divider of zero. Scope acquisition is
therefore driven once per `ctl_dispatch()` call, not by the RTOS polling task or
a fixed CSP divider.

See [README_CN.md](README_CN.md) for the architecture, extension rules,
hardware-probe command, and SCI error-recovery constraint.
