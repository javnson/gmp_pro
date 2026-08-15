# LaunchPad build matrix

Validated with CCS 12.8.1, C2000 compiler 22.6.1.LTS, SysConfig 1.21.0,
C2000Ware 5.04.00.00 and GMP-Core-C28x 2.10.00.00 on 2026-08-15.

| Board | Debug | Release | SysConfig source | Hardware run |
|---|---:|---:|---|---:|
| F2800137C | pass | pass | board-local | not run |
| F280025C | pass | pass | board-local | not run |
| F280039C | pass | pass | board-local | not run in this matrix |
| F280049C | pass | pass | board-local | pass |
| F28377S | pass | pass | board-local | not run |
| F28379D CPU1 | pass | pass | board-local | not run |
| F28P55X | pass | pass | board-local | not run |
| F28P65X CPU1 | pass | pass | board-local | not run |

All 16 configurations invoke SysConfig from CCS and generate files only below
their ignored configuration output directory. All configurations compile their
board-local DriverLib C sources and resolve DriverLib/device headers from the
registered C2000Ware installation; no local DriverLib archive or copied
C2000Ware header is used. The linker uses the default `_c_int00` ELF entry
point, while the Flash `codestart` section still branches through `code_start`.
TI warning #10063 is therefore absent. Remaining warnings are confined to TI
DriverLib unused-local diagnostics and, on smaller-device command files,
unassigned `ramgs*` output-section diagnostics.

The build matrix proves compilation and linking, not electrical pin routing on
unconnected hardware. Only F280049C currently has debugger, scheduler, live
ADC and Data Link evidence; see `F280049C_HARDWARE_ACCEPTANCE.md`.
