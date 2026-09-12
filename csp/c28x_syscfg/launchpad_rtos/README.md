# GMP C2000 LaunchPad FreeRTOS Reference

This directory contains the `LAUNCHXL-F280049C` GMP + TI FreeRTOS reference
project. It is independent from the bare-metal project in
`csp/c28x_syscfg/launchpad`.

FreeRTOS owns scheduling and CPU Timer2/INT14. GMP's cooperative, non-blocking
dispatcher runs inside the priority-6 `gmpService` task every 1 ms. The
priority-1 `userTask` demonstrates that applications can add independent RTOS
work. The 20 kHz ADC control loop remains a hardware ISR and calls no RTOS API;
SCI interrupts only transfer bytes while GMP Data Link parsing runs in the
service task.

The authoritative configuration is `LAUNCHXL_F280049C.syscfg`. Generated
Debug/Release and SysConfig output must not be committed. The current reference
uses static tasks, disables dynamic allocation, and assigns the FreeRTOS stack
and heap sections explicitly in the F280049C linker command file.

## Build

The validated tool set is CCS 12.8.1, C2000Ware 5.04.00.00, SysConfig 1.21.0,
and TI C2000 Compiler 22.6.1.LTS. Import this directory as an existing CCS
project and select `F280049C_Debug` or `F280049C_Release`, or run from the
repository root:

```powershell
& .\csp\c28x_syscfg\launchpad_rtos\tools\build.ps1 `
    -Mode All -GenerateGmpSources
```

## Flash and validate

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

See [README_CN.md](README_CN.md) for the detailed architecture, extension
rules, hardware probe command, and SCI error-recovery constraint.
