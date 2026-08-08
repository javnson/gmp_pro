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

The `DSA Trigger/Scope` tab can arm the C092 target, read its 400-point
sine/cosine snapshot through Memory Perspective, and plot both channels.
