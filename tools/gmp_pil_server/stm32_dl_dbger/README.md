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

The firmware exposes target information (`0x02`), ECHO (`0x00`), three named
read/write Tunable signal parameters (`0x30`/`0x31`), one named Memory Perspective region
(`0x50`/`0x51`), and an independent Data Link Scope service (`0x60`). Memory
and Tunable descriptor queries reuse `base + 1` with a one-byte indexed payload.

The Tunable workbench reports `Signal Frequency (Hz)`, `Signal Gain (x)`, and
`Signal DC Offset (V)`. Changes feed the waveform generator
directly, so their frequency, peak amplitude, and center line are immediately
visible in Data Link Scope. Values are bounded to safe demonstration ranges.

TIM3 samples at 1 kHz and generates a 50 Hz sine/cosine pair by default. The GMP DSA
Trigger arms a coherent acquisition and DSA Scope stores 400 samples per
channel, covering 20 signal periods. The 800-float, 3200-byte structure-of-arrays
buffer is registered only with Data Link Scope. Use the debugger's `Data Link
Scope` tab to discover it, select trigger mode/channel/level/position, arm it,
and plot the snapshot. Continuous display automatically re-arms and refreshes
snapshots for immediate or edge-triggered acquisition. Optional waveform
persistence retains age-faded history frames for jitter and stability analysis.
Continuous Display starts immediately when selected and stops future re-arming
when cleared. The current frame and all retained persistence frames can be
exported as CSV. The Scope page does not require or expose a physical address.

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
full-MTU DMA ECHO, target-reported Tunable and Memory descriptors, read/write
access, physical Tunable-to-Scope behavior, Scope discovery and configuration,
a 25-percent pre-trigger position, and the 400-point sine/cosine snapshot.
