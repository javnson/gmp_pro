# GMP Data Link Debugger

**English** | [简体中文](README_CN.md)

This directory contains the single maintained Python/PyQt frontend for GMP
Data Link and processor-in-the-loop workflows. The u8 and u16 targets use the
same serial wire format, so they share all protocol, resource-discovery, Scope,
and user-interface code.

Choose one guarded launcher after installing GMP:

- `run_u8.bat` selects the byte-addressed target profile used by STM32, x86,
  and most modern CPUs.
- `run_u16.bat` selects the 16-bit-addressed DSP profile used primarily by TI
  C28x devices.

The profile identifies the target contract in the window title; it does not
fork the host codec. Target-side C selection remains automatic through
`GMP_PORT_DATA_SIZE_PER_BYTES`.

Memory Perspective and Tunable pages can import named descriptors reported by
the target. Data Link Scope is an independent service with editable trigger
mode, source, level, and pre-trigger position. Continuous Display repeatedly
configures and starts the target as soon as it is enabled, then re-arms after
complete snapshots. Disabling it stops future re-arming without replacing the
currently displayed frame. Waveform Persistence retains faded history frames
in a separate display group. Waveform Export can save either the current frame
or every retained persistence frame plus the current frame as CSV.

System Log assigns a stable accent color to every page. Its Log Sources
dropdown contains checkable entries that can hide or restore each page's
retained messages.

Memory Perspective always uses byte addresses on the wire. Convert a C28x
native word address from a linker map to a byte address by multiplying it by
two before entering it manually.
