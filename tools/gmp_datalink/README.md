# GMP Data Link tools

**English** | [简体中文](README_CN.md)

Use the shared `datalink_studio` frontend and choose the launcher matching the
target's smallest addressable data unit:

- `datalink_studio/run_u8.bat`: byte-addressed STM32, x86, and modern CPUs.
- `datalink_studio/run_u16.bat`: 16-bit-addressed DSP targets, primarily TI C28x.

Both launchers use one maintained application because the DL wire frame is
identical. Target-side storage and native addressing remain selected by the C
headers through `GMP_PORT_DATA_SIZE_PER_BYTES`.

The `stm32_dl_dbger` directory contains the NUCLEO-C092RC validation firmware
for the u8 backend.
