# GMP PIL and Data Link tools

**English** | [简体中文](README_CN.md)

Choose the debugger from the target's smallest addressable data unit:

- `gmp_debugger_u8`: default tool for byte-addressed STM32, x86, and modern CPUs
  (`GMP_PORT_DATA_SIZE_PER_BYTES == 1`).
- `gmp_debugger_u16`: legacy tool for 16-bit-addressed DSP targets, primarily TI
  C28x (`GMP_PORT_DATA_SIZE_PER_BYTES == 2`).

Both tools use the same DL wire frame. They differ in the target-side storage
and native-address contract. New byte-addressed projects should start with
`gmp_debugger_u8/run.bat`.

The `stm32_dl_dbger` directory contains the NUCLEO-C092RC validation firmware
for the u8 backend.
