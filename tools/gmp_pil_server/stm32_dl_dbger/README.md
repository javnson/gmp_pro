# NUCLEO-C092RC GMP Data Link u8 validation

**English** | [简体中文](README_CN.md)

This standalone STM32CubeMX project validates the byte-addressed GMP Data Link
backend on an STM32C092RCT6. USART2 uses the ST-Link virtual COM port on PA2 and
PA3 at `921600 8-N-1`. RX is circular DMA with half-transfer,
transfer-complete, and UART-idle events. TX uses DMA for both frame phases.

The application follows the standard GMP layout:

- `user/` owns the application, cooperative scheduler, and CTL lifecycle hooks;
- `xplt/` owns the STM32 CSP binding, UART DMA transport, timer, and LED;
- `gmp_src_mgr/gmp_inc` and `gmp_src_mgr/gmp_src` contain generated project-local
  GMP headers and sources;
- `Core/Src/main.c` enters GMP through `gmp_base_entry()`;
- the scheduler registers only a Data Link service task and an LED heartbeat
  task.

Keil and CMake use only files inside this project. After `gmp_src_mgr` has been
generated, the entire `stm32_dl_dbger` directory can be copied away from the GMP
repository and built without `GMP_PRO_LOCATION` or any absolute GMP path.

## Data Link and DSA services

The firmware exposes target information (`0x02`), ECHO (`0x00`), four read/write
Tunable values (`0x30`/`0x31`), and Memory Perspective (`0x50`/`0x51`).

TIM3 samples at 1 kHz and generates a 50 Hz sine/cosine pair. The GMP DSA
Trigger arms a coherent acquisition and DSA Scope stores 400 samples per
channel, covering 20 signal periods. The 800-float, 3200-byte structure-of-arrays
buffer is exposed as a read-only Memory Perspective region. Use the debugger's
`DSA Trigger/Scope` tab to arm, fetch, and plot it.

## Regenerate project-local GMP files

Regeneration requires a GMP development environment and is the only step that
uses the repository:

```bat
gmp_src_mgr\gmp_generate_all.bat
```

The checked-in generated output is already ready to build. The generated
`gmp_compiler_includes.txt` lists only project-relative include directories.

## Keil MDK-ARM

Open `MDK-ARM/stm32_dl_dbger.uvprojx` in Keil uVision and build with Arm
Compiler 6. `generate_keil.bat` can regenerate CubeMX code and then restores
the project-local `user`, `xplt`, and generated GMP groups. It does not
regenerate GMP sources.

## CMake and hardware validation

```powershell
cmake --preset Debug
cmake --build --preset Debug
python smoke_test.py
```

Flash either the CMake ELF or Keil HEX, reset the board, and run the smoke test.
It auto-detects the NUCLEO ST-Link VCP and validates frame escaping, CRC,
full-MTU DMA ECHO, Tunable and Memory Perspective access, DSA metadata, the
400-point sine/cosine snapshot, and read-only protection.
