# STM32 Nucleo-144 FreeRTOS reference

[中文说明](README_CN.md)

This directory is an opt-in FreeRTOS reference for the NUCLEO-H753ZI. It does
not modify or replace the existing bare-metal project in `../Nucleo_144`.

The ownership model is deliberately small:

- FreeRTOS owns task creation, priorities, delays, the system tick, and kernel
  startup.
- One high-priority FreeRTOS task runs GMP initialization and one bounded
  `gmp_base_loop()` iteration every millisecond.
- `core/pm` remains a cooperative, non-blocking dispatcher inside that task.
- ADC DMA owns the 20 kHz control ISR and never calls a FreeRTOS API.
- A separate user task demonstrates that applications can add ordinary RTOS
  work without handing scheduling ownership back to GMP.

The current reference enables UART GMP Data Link. Ethernet hardware remains in
the inherited pin contract, but the bare-metal `NO_SYS=1` LwIP polling stack is
not linked. A future RTOS Ethernet variant must use the LwIP TCP/IP thread
model and explicit task-to-control data ownership.

## Build and flash

From `stm32h753zi_nucleo`:

```powershell
.\generate.ps1
.\build.ps1 -Configuration Release
.\flash.ps1
```

`generate.ps1` keeps `stm32h753zi_nucleo.ioc` authoritative. With the installed
CubeMX 6.17 RC, FreeRTOS is recognized but its source files are not emitted;
the project patcher therefore copies the exact FreeRTOS version from the H7
firmware package selected by the IOC.

See [validation.md](stm32h753zi_nucleo/validation.md) for the board result and
[the architecture report](../../../manual/multithreading_rtos_architecture_analysis_cn.md)
for repository-wide limitations and follow-up work.
