# GMP Debugger u8

**English** | [简体中文](README_CN.md)

GMP Debugger u8 is the default host tool for byte-addressed CPUs where
`GMP_PORT_DATA_SIZE_PER_BYTES == 1`, including STM32, x86, and most modern
microcontrollers and processors.

Run `run.bat` after installing GMP. The entry point selects the u8 target
contract and reuses the maintained debugger frontend from `../gmp_debugger_u16`.
The DL serial wire format remains identical; the target contract differs in
buffer storage and native memory addressing.

Memory Perspective addresses are ordinary byte addresses. For the included
NUCLEO-C092RC validation firmware, select the ST-Link virtual COM port and use
`921600 8-N-1`.

The `Data Link Scope` tab discovers the C092 scope resource and can configure
trigger mode, source channel, level, position, and auto timeout before plotting
its 400-point sine/cosine snapshot. Continuous display automatically re-arms
and refreshes completed acquisitions. Configurable waveform persistence overlays
age-faded history frames as an oscilloscope-style afterglow. It uses the independent Scope service and
does not require a physical memory address.
